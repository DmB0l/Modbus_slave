#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <cstdint>

namespace ModBus {
uint16_t crc16(uint8_t *_data, uint32_t _size);
}

#endif // MODBUS_CRC_H
