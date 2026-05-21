#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include "communication/UartLink.hpp"
#include "utils/CrcCalculator.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
	(void)signum;
	g_running = false;
}

struct __attribute__((packed)) DownlinkPacket {
	uint8_t  sync;           // 0xAA
	uint8_t  sync2;          // 0x55
	uint8_t  msg_id;         // 0x00
	uint8_t  length;         // payload 크기 (byte)
	uint16_t seq;            
	float    target_x;       
	float    target_y;       
	float    torpedo_x;      
	float    torpedo_y;      
	int16_t  steer;          
	uint8_t  flags;          
	uint16_t crc16;          
};

/*
speed_t get_baud_constant(int baudrate) {
	switch (baudrate) {
		case 9600:   return B9600;
		case 19200:  return B19200;
		case 38400:  return B38400;
		case 57600:  return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 921600: return B921600;
		default:     return B115200;
	}
}
*/

int main(int argc, char** argv) {
	signal(SIGINT, signalHandler);

	std::string port = "/dev/ttyS2";
	int baud_val = 115200; 

	if (argc > 1) port = argv[1];
	if (argc > 2) baud_val = std::stoi(argv[2]);

	std::cout << "Control Station Test (Using UartLink with Verified Config)" << std::endl;
	std::cout << "Port: " << port << ", Baud: " << baud_val << std::endl;

	// --- [Mode 1] Using UartLink (Active) ---
	UartLink uart(port, baud_val);
	if (!uart.initialize()) {
		std::cerr << "Failed to initialize UartLink on " << port << std::endl;
		return 1;
	}

	/*
	// --- [Mode 2] Direct System Call (Commented fallback) ---
	int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		std::cerr << "Failed to open " << port << std::endl;
		return 1;
	}

	struct termios tio;
	memset(&tio, 0, sizeof(tio));
	speed_t baud = get_baud_constant(baud_val);
	cfsetispeed(&tio, baud);
	cfsetospeed(&tio, baud);
	
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_iflag = IGNPAR;
	tio.c_cc[VMIN]  = 0;
	tio.c_cc[VTIME] = 0;
	
	tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &tio) < 0) {
		std::cerr << "Failed to set termios attributes" << std::endl;
		close(fd);
		return 1;
	}
	*/

	std::cout << "Waiting for data... (Every byte will be printed in HEX)" << std::endl;
	std::cout << "----------------------------------------------------------------" << std::endl;

	int state = 0; // 0: Sync1, 1: Sync2, 2: Payload + CRC
	DownlinkPacket pkt;
	uint8_t* pkt_ptr = (uint8_t*)&pkt;
	size_t rx_idx = 0;

	while (g_running) {
		uint8_t byte;
		
		// [Mode 1] Using UartLink
		ssize_t n = uart.receive(&byte, 1);
		
		// [Mode 2] Direct System Call
		// ssize_t n = read(fd, &byte, 1);
		
		if (n > 0) {
			// Unconditionally print every byte in hex
			std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " " << std::flush;

			// Parsing State Machine
			if (state == 0) {
				if (byte == 0xAA) {
					pkt_ptr[0] = byte;
					state = 1;
				}
			} else if (state == 1) {
				if (byte == 0x55) {
					pkt_ptr[1] = byte;
					state = 2;
					rx_idx = 2;
				} else if (byte == 0xAA) {
					state = 1;
				} else {
					state = 0;
				}
			} else if (state == 2) {
				pkt_ptr[rx_idx++] = byte;
				if (rx_idx >= sizeof(DownlinkPacket)) {
					// ALWAYS reset state and index when packet is complete
					uint16_t calc_crc = CrcCalculator::CalculateCrc16Ccitt((uint8_t*)&pkt, sizeof(DownlinkPacket) - 2);
					
					std::cout << std::dec << "\n  └─ [Parsed] ";
					if (calc_crc == pkt.crc16) {
						std::cout << "[CRC OK] Seq: " << pkt.seq 
							<< " | Tgt: (" << std::fixed << std::setprecision(2) << pkt.target_x << ", " << pkt.target_y << ")"
							<< " | Torp: (" << pkt.torpedo_x << ", " << pkt.torpedo_y << ")"
							<< " | Steer: " << pkt.steer
							<< " | Flags: 0x" << std::hex << (int)pkt.flags << std::dec << std::endl;
					} else {
						std::cout << "[CRC ERR] Recv: 0x" << std::hex << pkt.crc16 
							<< " | Calc: 0x" << calc_crc << std::dec << std::endl;
					}
					std::cout << "----------------------------------------------------------------" << std::endl;
					
					state = 0;
					rx_idx = 0;
				}
			}
		} else {
			usleep(100);
		}
	}

	// [Mode 1]
	uart.close();

	// [Mode 2]
	// close(fd);

	return 0;
}
