#include "modbus.h"
#include "modbus_reg.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <vector>

ModBus::ModBusSlave::ModBusSlave(const ModBusSlaveProperty &_prop):
    m_property(_prop),
    m_reg(new ModBus::ModBusReg(_prop.d_out_size,_prop.d_input_size,_prop.a_out_size,_prop.a_input_size))
{}

ModBus::ModBusSlave::ModBusSlave(const ModBusSlaveProperty &&_prop):
    m_property(_prop),
    m_reg(new ModBus::ModBusReg(_prop.d_out_size,_prop.d_input_size,_prop.a_out_size,_prop.a_input_size))
{}

ModBus::ModBusSlave::~ModBusSlave()
{
    delete m_reg;
}

std::vector<uint8_t> ModBus::ModBusSlave::execute()
{
    enum{
        Adress = 0,
        FunctionalCode = 1,
        OutputBitCount = 2,

        RegisterAddressHi = 2,
        RegisterAddressLo = 3,

        CountRegisterHi = 4,
        CountRegisterLo = 5,

        CountBytes = 6,
    };

    std::vector<uint8_t> answer;

    if(m_input_buf.size() > CountRegisterLo && m_input_buf.size() < ApplicationDataUnitSize){

        FunctionalCodes fc = FunctionalCodes(m_input_buf[FunctionalCode]);
        uint8_t address = m_input_buf[Adress];

        uint16_t mem_address = m_input_buf[RegisterAddressHi] << 8 | m_input_buf[RegisterAddressLo];
        uint16_t size = m_input_buf[CountRegisterHi] << 8 | m_input_buf[CountRegisterLo];

        answer.push_back(address);
        answer.push_back(fc);

        switch (fc) {
        case ReadCoils:
        case ReadDiscreteInputs:
        case ReadHoldingRegisters:
        case ReadInputRegisters:
            answer.push_back((fc == ReadCoils || fc == ReadDiscreteInputs)?
                                 ceill(size / 8.):
                                 size*2);
            {
                auto ex = m_reg->read_register(ModBusReg::Register(fc - 1), answer, mem_address, size);
                if(ex != ModbusExceptions::Nope)
                    answer = encode_exception_pkg(address, fc, ModbusExceptions::Acknowledge);
            }
            break;
        case WriteSingleCoil:
        case WriteSingleRegister:
            {
                uint16_t value = size;
                auto reg = fc == WriteSingleCoil? ModBusReg::Register::DiscreteOutput:ModBusReg::Register::AnalogOutput;
                ModbusExceptions ex = m_reg->write_single_register(reg, mem_address, value);

                if(ex != ModbusExceptions::Nope){
                    answer = encode_exception_pkg(address, fc, ex);
                    break;
                }

                if (reg == ModBusReg::Register::DiscreteOutput)
                    m_property.discrete_register_edit(m_reg, mem_address, 1);
                else
                    m_property.analog_register_edit(m_reg, mem_address, 1);
                answer.insert(answer.begin() + 2,
                              m_input_buf.begin() + 2,
                              m_input_buf.begin() + m_input_buf.size() - 2);
            }
            break;

        case WriteMultipleCoils:
        case WriteMultipleRegisters:
        {
            auto reg = fc == WriteMultipleCoils? ModBusReg::Register::DiscreteOutput:ModBusReg::Register::AnalogOutput;

            std::vector<uint8_t> data(m_input_buf.begin() + CountBytes + 1,
                                      m_input_buf.begin() + 1 + CountBytes + m_input_buf[CountBytes]);

            ModbusExceptions ex =  m_reg->write_register(reg, data, mem_address, m_input_buf[CountBytes]);

            if(ex != ModbusExceptions::Nope){
                answer = encode_exception_pkg(address, fc, ex);
                break;
            }

            if (reg == ModBusReg::Register::DiscreteOutput)
                m_property.discrete_register_edit(m_reg, mem_address, size);
            else
                m_property.analog_register_edit(m_reg, mem_address, m_input_buf[CountBytes]);

            answer.insert(answer.begin() + 2,
                          m_input_buf.begin() + 2,
                          m_input_buf.begin() + 6);
        }
            break;

        default:
            answer = encode_exception_pkg(address, fc, ModbusExceptions::IlligalFunction);
            break;
        }

        append_crc(answer);
        m_property.write_to_uart(answer.data(), answer.size());
    }
    return answer;
}

void ModBus::ModBusSlave::write_command(uint8_t *_data, uint32_t _size)
{
    m_input_buf = std::vector<uint8_t>(_data, _data + _size);
}

ModBus::ModBusReg *const ModBus::ModBusSlave::modify_register()
{
    return m_reg;
}

ModBus::ModBusMaster::ModBusMaster(uint8_t _address):m_address(_address){}

std::vector<uint8_t> ModBus::ModBusMaster::read_register(ModBusReg::Register _reg, uint16_t _pos, uint16_t _count)
{
    uint8_t fc = 0;
    switch (_reg) {
    case ModBus::ModBusReg::Register::DiscreteOutput:
        fc = ReadCoils;
        break;
    case ModBus::ModBusReg::Register::DiscreteInput:
        fc = ReadDiscreteInputs;
        break;
    case ModBus::ModBusReg::Register::AnalogOutput:
        fc = ReadHoldingRegisters;
        break;
    case ModBus::ModBusReg::Register::AnalogInput:
        fc = ReadInputRegisters;
        break;
    default:
        break;
    }

    std::vector<uint8_t> sig = {m_address, fc,
                                static_cast<unsigned char>(_pos>>8),
                                static_cast<unsigned char>(_pos&0xFF),
                                static_cast<unsigned char>(_count>>8),
                                static_cast<unsigned char>(_count&0xFF)};

    append_crc(sig);

    return sig;
}

std::vector<uint8_t> ModBus::ModBusMaster::write_single_register(ModBusReg::Register _reg, uint16_t _pos, uint16_t _value)
{
    assert(_reg != ModBusReg::Register::DiscreteInput && _reg != ModBusReg::Register::AnalogInput);

    uint8_t fc = _reg == ModBusReg::Register::DiscreteOutput? WriteSingleCoil:WriteSingleRegister;

    if(fc == WriteSingleCoil){
        _value = _value == 1 ? 0xFF << 8 : 0;
    }

    std::vector<uint8_t> sig = {m_address, fc,
                                static_cast<unsigned char>(_pos>>8),
                                static_cast<unsigned char>(_pos&0xFF),
                                static_cast<unsigned char>(_value>>8),
                                static_cast<unsigned char>(_value&0xFF)};

    append_crc(sig);

    return sig;
}

std::vector<uint8_t> ModBus::ModBusMaster::write_multiple_register(ModBusReg::Register _reg, uint16_t _pos, std::vector<uint8_t> &&_data)
{
    uint8_t fc = _reg == ModBusReg::Register::DiscreteOutput? WriteMultipleCoils : WriteMultipleRegisters;

    uint16_t reg_count = _reg == ModBusReg::Register::DiscreteOutput?_data.size()*8: _data.size() / 2;
    uint8_t d = _data.size();
    std::vector<uint8_t> sig = {m_address, fc,
                                static_cast<unsigned char>(_pos>>8),
                                static_cast<unsigned char>(_pos&0xFF),
                                static_cast<unsigned char>(reg_count>>8),
                                static_cast<unsigned char>(reg_count&0xFF),
                                d};

    for(int i = 0; i < _data.size(); i++){
        sig.push_back(_data[i]);
    }

    append_crc(sig);

    return sig;
}
