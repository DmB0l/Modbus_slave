#ifndef MODBUS_H
#define MODBUS_H

#include <cstdint>
#include <vector>

#include "modbus_private.h"
#include "modbus_reg.h"
#include "modbus_settings.h"
#include "modbus_constants.h"

namespace ModBus{
    class ModBusReg;

    /*!
    \brief ModBusProperty
    */
    struct ModBusSlaveProperty{
        void (*write_to_uart)(uint8_t *_data, uint32_t _size);
        void (*analog_register_edit)(ModBusReg *const _reg, uint32_t _pos, uint32_t _size);
        void (*discrete_register_edit)(ModBusReg *const _reg, uint32_t _pos, uint32_t _size);

        uint32_t d_out_size, d_input_size, a_out_size, a_input_size;
        uint8_t address;
    };

    /*!
    \brief ModBusSlave
    */
    class ModBusSlave : public ModBusPrivate
    {
    public:
        ModBusSlave(const ModBusSlaveProperty &_prop);
        ModBusSlave(const ModBusSlaveProperty &&_prop);
        ~ModBusSlave();

        std::vector<uint8_t> execute();
        void write_command(uint8_t *_data, uint32_t _size);

        ModBusReg *const modify_register();
    private:
        const ModBusSlaveProperty m_property;
        ModBusReg *m_reg;

        std::vector<uint8_t> m_input_buf;
    };

    /*!
    \brief ModBusProperty
    */
    class ModBusMaster : public ModBusPrivate{
    public:
        ModBusMaster(uint8_t _address);

        std::vector<uint8_t> read_register(ModBusReg::Register _reg, uint16_t _pos, uint16_t _count);
        std::vector<uint8_t> write_single_register(ModBusReg::Register _reg, uint16_t _pos, uint16_t _value);
        std::vector<uint8_t> write_multiple_register(ModBusReg::Register _reg, uint16_t _pos, std::vector<uint8_t> &&_data);
    private:
        uint8_t m_address;
    };
}



#endif // MODBUS_H
