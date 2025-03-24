#ifndef MODBUS_REG_H
#define MODBUS_REG_H

#include <cstdint>
#include <vector>
#include "modbus_constants.h"

namespace ModBus{

class ModBusSlave;

/*!
\brief ModBusReg класс для взаимодействия с регистрами
*/
class ModBusReg
{
    enum{
        Byte = 8
    };
public:
    /*!
    \brief Регистры
    */
    enum class Register{
        DiscreteOutput = 0,
        DiscreteInput,
        AnalogOutput,
        AnalogInput,
    };

    enum class BytesSwapped{
        abcd,
        dcba,
        badc,
        cdab
    };

    /*!
    \brief Конструктор регисторов ModBus
    \param[in] _d_out_size Размер дискретного регистра для чтения
    \param[in] _d_input_size Размер дискретного регистра для записи
    \param[in] _a_out_size Размер аналогового регистра для чтения
    \param[in] _a_input_size Размер аналогового регистра для записи
    */
    ModBusReg(uint32_t _d_out_size, uint32_t _d_input_size, uint32_t _a_out_size, uint32_t _a_input_size);
    ~ModBusReg();

    /*!
    \brief Зпись float в регистр
    \param[in] _reg Регистр
    \param[in] _index Позиция в регистре
    \param[in] _val Значение
    */
    void write_float(Register _reg, uint32_t _index, float _val);

    /*!
    \brief Чтение float в регистр
    \param[in] _reg Регистр
    \param[in] _index Позиция в регистре
    \param[out] Значение
    */
    float read_float(Register _reg, uint32_t _index);

    /*!
    \brief Зпись int в регистр
    \param[in] _reg Регистр
    \param[in] _index Позиция в регистре
    \param[in] _val Значение
    */
    void write_int(Register _reg, uint32_t _index, int _val);

    /*!
    \brief Чтение int в регистр
    \param[in] _reg Регистр
    \param[in] _index Позиция в регистре
    \param[out] Значение
    */
    int read_int(Register _reg, uint32_t _index);

    void write_int16(Register _reg, uint32_t _index, int16_t _val);
    int16_t read_int16(Register _reg, uint32_t _index);

    void write_int8(Register _reg, uint32_t _index, int8_t _val);
    int8_t read_int8(Register _reg, uint32_t _index);

    /*!
    \brief Чтание float в регистр
    \param[in] _reg Регистр
    \param[in] _index Позиция в регистре
    \param[out] Значение
    */
    void write_bytes(Register _reg, uint8_t *_data, uint32_t _index, uint32_t _size);

protected:
    friend ModBusSlave;

    ModbusExceptions read_register(Register _reg, std::vector<uint8_t> &, uint32_t _index, uint32_t _size);
    ModbusExceptions write_single_register(Register _reg, uint32_t _index, uint16_t _value);
    ModbusExceptions write_register(Register _reg, std::vector<uint8_t> &, uint32_t _index, uint32_t _size);

    uint32_t get_register_size(Register _r);
    uint8_t *get_register(Register _r);

private:
    uint8_t *m_analog_input, *m_analog_output;
    uint8_t *m_discrete_input, *m_discrete_output;
    const uint32_t m_d_out_size, m_d_input_size, m_a_out_size, m_a_input_size;
    bool m_init_compleate;
};
}

#endif // MODBUS_REG_H
