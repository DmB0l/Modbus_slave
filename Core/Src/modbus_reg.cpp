#include "modbus_reg.h"
#include "modbus_constants.h"

#include <cassert>
#include <cstring>
#include <stdexcept>
#include <string.h>
#include <vector>

ModBus::ModBusReg::ModBusReg(uint32_t _d_out_size, uint32_t _d_input_size, uint32_t _a_out_size, uint32_t _a_input_size):
    m_d_out_size(_d_out_size),
    m_d_input_size(_d_input_size),
    m_a_out_size(_a_out_size),
    m_a_input_size(_a_input_size),
    m_discrete_input(nullptr),
    m_discrete_output(nullptr),
    m_analog_input(nullptr),
    m_analog_output(nullptr)
{
    assert(MaxDiscreteOutput >= _d_out_size);
    assert(MaxDiscreteInput >= _d_input_size);
    assert(MaxAnalogOutput >= _a_out_size);
    assert(MaxAnalogInput >= _a_input_size);

    m_discrete_input = new uint8_t[m_d_input_size/Byte + 1];
    m_discrete_output = new uint8_t[m_d_out_size/Byte + 1];

    m_analog_input = new uint8_t[m_a_input_size * 2];
    m_analog_output = new uint8_t[m_a_out_size * 2];

    memset(m_discrete_input, 0, m_d_input_size/Byte + 1);
    memset(m_discrete_output, 0, m_d_out_size/Byte + 1);
    memset(m_analog_input, 0, m_a_input_size * 2);
    memset(m_analog_output, 0, m_a_out_size * 2);

    assert(m_discrete_input != nullptr ||
           m_discrete_output != nullptr ||
           m_analog_input != nullptr ||
           m_analog_output != nullptr);
}

ModBus::ModBusReg::~ModBusReg()
{
    delete[] m_discrete_input;
    delete[] m_discrete_output;
    delete[] m_analog_input;
    delete[] m_analog_output;
}

void ModBus::ModBusReg::write_float(Register _reg, uint32_t _index, float _val)
{
    auto reg = get_register(_reg)+_index;
    memcpy(reg, (uint8_t*)&_val, 4);
}

float ModBus::ModBusReg::read_float(Register _reg, uint32_t _index)
{
    auto reg = get_register(_reg)+_index;
    float f = 0;
    memcpy((uint8_t*)&f, reg, 4);

    return f;
}

void ModBus::ModBusReg::write_int(Register _reg, uint32_t _index, int _val)
{
    auto reg = get_register(_reg)+_index;
    memcpy(reg, (uint8_t*)&_val, 4);
}

int ModBus::ModBusReg::read_int(Register _reg, uint32_t _index)
{
    auto reg = get_register(_reg)+_index;
    int i = 0;
    memcpy((uint8_t*)&i, reg, 4);

    return i;
}

void ModBus::ModBusReg::write_int16(Register _reg, uint32_t _index, int16_t _val)
{
    auto reg = get_register(_reg)+_index;
    memcpy(reg, (uint8_t*)&_val, 2);
}

int16_t ModBus::ModBusReg::read_int16(Register _reg, uint32_t _index)
{
    auto reg = get_register(_reg)+_index;
    int16_t i = 0;
    memcpy((uint8_t*)&i, reg, 2);

    return i;
}

void ModBus::ModBusReg::write_int8(Register _reg, uint32_t _index, int8_t _val)
{
    uint8_t *reg = get_register(_reg);
    reg[_index] = _val;
}

int8_t ModBus::ModBusReg::read_int8(Register _reg, uint32_t _index)
{
    return get_register(_reg)[_index];
}

void ModBus::ModBusReg::write_bytes(Register _reg, uint8_t *_data, uint32_t _index, uint32_t _size)
{

}

ModBus::ModbusExceptions ModBus::ModBusReg::read_register(Register _reg, std::vector<uint8_t> &_data, uint32_t _index, uint32_t _size)
{
    const uint32_t max_reg_size = get_register_size(_reg);
    uint8_t *reg = get_register(_reg);

    if(_index > max_reg_size || _index + _size > max_reg_size)
        return ModbusExceptions::IlligalDataAddress;

    if (_reg == Register::AnalogInput || _reg == Register::AnalogOutput){

        for(int i = _index; i < _index + _size; i++){
            _data.push_back(reg[i*2+1]);
            _data.push_back(reg[i*2]);
        }

        // _data.insert(_data.begin() + _data.size(), reg + _index * 2, reg + _index * 2 + _size*2);
        return ModbusExceptions::Nope;
    }

    uint32_t pos = _index / Byte;
    uint32_t offset = _index - pos * Byte;
    uint32_t write_pos = 0;
    uint8_t write_offset = 0;
    uint8_t data = 0;

    for(uint32_t bit_count = 0; bit_count < _size; bit_count++){
        write_offset = (bit_count % Byte);

        uint8_t r = reg[pos];
        data |= int((r >> offset) & 1) << write_offset;

        offset++;

        if(offset == Byte){
            _data.push_back(data);
            data = 0;
        }

        pos = offset == Byte? pos+1 :pos;
        offset = offset>=Byte? 0 : offset;
    }

    return ModbusExceptions::Nope;
}

ModBus::ModbusExceptions ModBus::ModBusReg::write_single_register(Register _reg, uint32_t _index, uint16_t _value)
{
    const uint32_t max_reg_size = get_register_size(_reg);
    uint8_t *reg = get_register(_reg);

    if(_index >= max_reg_size)
        return ModbusExceptions::IlligalDataAddress;

    if (_reg == Register::AnalogInput || _reg == Register::AnalogOutput){
        memcpy(reg + _index * 2, (uint8_t*)&_value, sizeof(_value));
        return ModbusExceptions::Nope;
    }

    if (_value >> 8 != 0 && _value >> 8 != uint8_t(0xFF))
        return ModbusExceptions::IlligalDataValue;

    uint32_t pos = _index / Byte;
    uint32_t offset = _index - pos * Byte;

    if(_value >> 8 == 0xFF){
        reg[pos] |= 1 << offset;
    }else{
        reg[pos] &= ~(1 << offset);
    }

    return ModbusExceptions::Nope;
}

ModBus::ModbusExceptions ModBus::ModBusReg::write_register(Register _reg, std::vector<uint8_t> &_data, uint32_t _index, uint32_t _size)
{
    const uint32_t max_reg_size = get_register_size(_reg);
    uint8_t *reg = get_register(_reg);

    if(_index > max_reg_size || _index + _size/2 > max_reg_size)
        return ModbusExceptions::IlligalDataAddress;

    if (_reg == Register::AnalogInput || _reg == Register::AnalogOutput){
        memcpy(reg + _index * 2, _data.data(), _size);
        return ModbusExceptions::Nope;
    }

    if(_index > max_reg_size || _index + _size > max_reg_size)
        return ModbusExceptions::IlligalDataAddress;

    uint32_t offset_r = 0, offset_w  = 0;
    uint32_t pos = _index / Byte;
    uint32_t i, b;

    offset_w = _index - pos * Byte;

    for (i = 0; i < _size; i++) {
        b = (_data[int(i / 8)] & (1 << offset_r) ? 1 : 0);

        reg[pos] &= ~(1 << offset_w);
        reg[pos] |= b << offset_w;

        pos = offset_w+1 == Byte? pos+1 :pos;
        offset_w = (++offset_w) % Byte;
        offset_r = (++offset_r) % Byte;
    }
    return ModbusExceptions::Nope;
}

uint32_t ModBus::ModBusReg::get_register_size(Register _r)
{
    switch (_r) {
    case Register::AnalogInput:
        return m_a_input_size;
    case Register::AnalogOutput:
        return m_a_out_size;
    case Register::DiscreteInput:
        return m_d_input_size;
    case Register::DiscreteOutput:
        return m_d_out_size;
    default:
        throw std::runtime_error("Register incorrected");
    }
}

uint8_t *ModBus::ModBusReg::get_register(Register _r)
{
    switch (_r) {
    case Register::AnalogInput:
        return m_analog_input;
    case Register::AnalogOutput:
        return m_analog_output;
    case Register::DiscreteInput:
        return m_discrete_input;
    case Register::DiscreteOutput:
        return m_discrete_output;
    default:
        throw std::runtime_error("Register incorrected");
    }
}
