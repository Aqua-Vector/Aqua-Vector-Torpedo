#include <iostream>
#include <csignal>
#include <atomic>
#include <iomanip>
#include <cstring>
#include <unistd.h>
#include <thread>
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

int main(int argc, char** argv) {
	signal(SIGINT, signalHandler);

	std::string port = "/dev/ttyS2";
	int baud = 115200; 

	if (argc > 1) port = argv[1];
	if (argc > 2) baud = std::stoi(argv[2]);

	std::cout << "Control Station Test (User Implementation Mode)" << std::endl;
	std::cout << "Port: " << port << ", Baud: " << baud << std::endl;

	UartLink link(port, baud);
	if (!link.initialize()) {
		std::cerr << "Failed to initialize UART" << std::endl;
		return 1;
	}

	std::cout << "Waiting for packets... (AA 55)" << std::endl;

	while (g_running) {
		uint8_t head[2] = {0, 0};

		std::cout << "g_running\n";
		// 1. 헤더 0xAA, 0x55 찾기
		bool header_found = false;
		while (g_running) {
			if (link.receive(&head[0], 1) > 0) {
				std::cout << "before read haed 1 : 0x" << std::hex << (int)head[0] << std::dec << "\n";
				if (head[0] == 0xAA) {
					std::cout << "after read head 1\n";
					// 다음 바이트가 0x55인지 확인
					int retry = 50; 
					while (retry-- > 0 && g_running) {
						std::cout << "before read head 2\n";
						if (link.receive(&head[1], 1) > 0) {
							if (head[1] == 0x55) {
								header_found = true;
								break;
							} else if (head[1] == 0xAA) {
								// AA가 연속으로 올 경우 다시 55 대기
								retry = 50;
							} else {
								break;
							}
						}
						usleep(100);
					}
				}
			}
			if (header_found) break;
			usleep(100); // CPU 점유율 방지
		}

		if (!header_found) continue;

		// 2. DownlinkPacket 구조체 크기(27바이트)만큼 읽기
		// 헤더(2바이트)를 이미 읽었으므로 나머지 25바이트 읽기
		DownlinkPacket pkt;
		pkt.sync = 0xAA;
		pkt.sync2 = 0x55;

		uint8_t* p = (uint8_t*)&pkt + 2;
		int target = sizeof(DownlinkPacket) - 2; 
		int total_read = 0;

		while (total_read < target && g_running) {
			int n = link.receive(p + total_read, target - total_read);
			if (n > 0) {
				total_read += n;
			} else {
				usleep(100);
			}
		}

		if (total_read == target) {
			// 3. CRC 검증 (DownlinkPacket 구조체 크기 기준, Sync1~Flags 25바이트)
			uint16_t calc_crc = CrcCalculator::CalculateCrc16Ccitt((uint8_t*)&pkt, sizeof(DownlinkPacket) - 2);

			if (calc_crc == pkt.crc16) {
				std::cout << "[RX OK] Seq: " << pkt.seq 
					<< " | Tgt: (" << std::fixed << std::setprecision(2) << pkt.target_x << ", " << pkt.target_y << ")"
					<< " | Torp: (" << pkt.torpedo_x << ", " << pkt.torpedo_y << ")"
					<< " | Steer: " << pkt.steer
					<< " | Flags: 0x" << std::hex << (int)pkt.flags << std::dec << std::endl;
			} else {
				std::cout << "[CRC ERR] Recv: 0x" << std::hex << pkt.crc16 
					<< " | Calc: 0x" << calc_crc << std::dec << std::endl;
			}
		}
	}

	link.close();
	return 0;
}
