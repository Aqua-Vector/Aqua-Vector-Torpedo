#include <iostream>
#include <cstdint>
#include <iomanip>

uint8_t calculate_sum(const uint8_t* buf, size_t len) {
    uint8_t check = 0;
    while(len--) {
        check += *buf++;
    }
    return check;
}

int main() {
    uint8_t msg_id = 0x0A;
    uint8_t length = 0x0C;
    uint8_t payload[] = {
        0x00, 0x00, 0x00, 0x00, // 0.0f
        0x00, 0x00, 0x20, 0x41, // 10.0f
        0x00, 0x00, 0xA0, 0xC0  // -5.0f
    };

    uint8_t data[14];
    data[0] = msg_id;
    data[1] = length;
    for(int i=0; i<12; ++i) data[2+i] = payload[i];

    std::cout << "Sum (Excluding Sync): 0x" << std::hex << (int)calculate_sum(data, 14) << std::endl;

    uint8_t data_with_sync[16];
    data_with_sync[0] = 0xAA;
    data_with_sync[1] = 0x55;
    for(int i=0; i<14; ++i) data_with_sync[2+i] = data[i];

    std::cout << "Sum (Including Sync): 0x" << std::hex << (int)calculate_sum(data_with_sync, 16) << std::endl;

    return 0;
}
