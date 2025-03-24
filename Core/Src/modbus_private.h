#ifndef MODBUS_PRIVATE_H
#define MODBUS_PRIVATE_H

#include "modbus_constants.h"
#include <cstdint>
#include <string>
#include <vector>



namespace ModBus{

    class ModBusPrivate{

    protected:
        void append_crc(std::vector<uint8_t> &_data);
        bool check_crc(std::vector<uint8_t> &_data);

        std::vector<uint8_t> encode_exception_pkg(uint8_t _address, FunctionalCodes _fc, ModbusExceptions _ex);
        std::string decode_exception_pkg(std::vector<uint8_t> &_data);
    };
}



#endif // MODBUS_PRIVATE_H
