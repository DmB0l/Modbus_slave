#include "modbus_private.h"
#include "modbus_crc.h"

void ModBus::ModBusPrivate::append_crc(std::vector<uint8_t> &_data)
{
    uint16_t crc = crc16(_data.data(), _data.size());
    _data.push_back(crc & 0x00FF);
    _data.push_back(crc >> 8);
}

bool ModBus::ModBusPrivate::check_crc(std::vector<uint8_t> &_data)
{
    auto crc = crc16(_data.data(), _data.size()-2);
    return crc >> 8 == _data[_data.size() - 1] && (crc & 0xFF) == _data[_data.size() - 2];
}

std::vector<uint8_t> ModBus::ModBusPrivate::encode_exception_pkg(uint8_t _address, FunctionalCodes _fc, ModbusExceptions _ex)
{
    std::vector<uint8_t> data;

    data.push_back(_address);
    data.push_back(_fc | 1 << 7);
    data.push_back(_ex);

    return data;
}

std::string ModBus::ModBusPrivate::decode_exception_pkg(std::vector<uint8_t> &_data)
{
    return "";
}
