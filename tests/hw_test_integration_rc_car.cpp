#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <cstring>
#include <unistd.h>

#include "communication/UartLink.hpp"
#include "protocol/GenericPacket.hpp"
#include "protocol/Payloads.hpp"
#include "utils/CrcCalculator.hpp"

std::atomic<bool> g_running(true);

void signalHandler(int signum) {
	(void)signum;
	g_running = false;
}

// GCS에서 내려오는 패킷 타입 정의
using GcsControlPacket = GenericPacket<ControlStationPayload, uint16_t>;

int main(int argc, char** argv) {
	signal(SIGINT, signalHandler);

	std::string port = "/dev/ttyS2";
	int baud_val = 115200; 

	if (argc > 1) port = argv[1];
	if (argc > 2) baud_val = std::stoi(argv[2]);

	std::cout << "GCS Reception & Parsing Test" << std::endl;
	std::cout << "Port: " << port << ", Baud: " << baud_val << std::endl;
	std::cout << "Packet Size: " << sizeof(GcsControlPacket) << " bytes" << std::endl;

	UartLink uart(port, baud_val);
	if (!uart.initialize()) {
		std::cerr << "Failed to initialize UartLink on " << port << std::endl;
		return 1;
	}

	GcsControlPacket pkt;
	uint8_t* pkt_ptr = reinterpret_cast<uint8_t*>(&pkt);
	size_t rx_idx = 0;
	int state = 0; // 0: Sync1, 1: Sync2, 2: Rest

	std::cout << "Waiting for data..." << std::endl;
	std::cout << "----------------------------------------------------------------" << std::endl;

	while (g_running) {
		uint8_t byte;
		ssize_t n = uart.receive(&byte, 1);
		
		if (n > 0) {
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
				if (rx_idx >= sizeof(GcsControlPacket)) {
					// CRC16-CCITT 계산 (Policy에 따라 [ID, Length, Payload] 영역)
					uint16_t calc_crc = CrcCalculator::CalculateCrc16Ccitt(pkt_ptr, sizeof(GcsControlPacket) - 2);
					
					std::cout << "[Packet Received] ";
					if (calc_crc == pkt.crc) {
						std::cout << "[CRC OK]" << std::endl;
						std::cout << "  Seq: " << pkt.payload.seq << std::endl;
						std::cout << "  Target:  (" << std::fixed << std::setprecision(2) << pkt.payload.target_x << ", " << pkt.payload.target_y << ")" << std::endl;
						std::cout << "  Torpedo: (" << pkt.payload.torpedo_x << ", " << pkt.payload.torpedo_y << ")" << std::endl;
						std::cout << "  Steer: " << pkt.payload.steer << std::endl;
						std::cout << "  Flags: 0x" << std::hex << (int)pkt.payload.flags << std::dec << std::endl;
					} else {
						std::cout << "[CRC ERR] Recv: 0x" << std::hex << pkt.crc << " | Calc: 0x" << calc_crc << std::dec << std::endl;
						// 디버깅을 위해 수신 바이트 출력
						std::cout << "  Raw: ";
						for (size_t i = 0; i < sizeof(GcsControlPacket); ++i) {
							std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)pkt_ptr[i] << " ";
						}
						std::cout << std::dec << std::endl;
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

	uart.close();
	return 0;
}
