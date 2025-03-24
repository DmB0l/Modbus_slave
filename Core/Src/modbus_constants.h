#ifndef MODBUS_CONSTANTS_H
#define MODBUS_CONSTANTS_H

namespace ModBus {
    /*!
    \brief Доступные коды ошибок ModBus
    */
    enum ModbusExceptions{
        Nope = 0,
        IlligalFunction = 0x01,
        IlligalDataAddress,
        IlligalDataValue,
        SlaveOrServerFailure,
        Acknowledge,
        SlaveOrServerBusy,
        NegativeAcknowledge,
        MemoryParity,
        NotDefined,
        GetewayPath,
        GetewayTarget,
    };

    /*!
    \brief Доступные функциональные коды ModBus
    */
    enum FunctionalCodes{
        ReadCoils = 0x01,
        ReadDiscreteInputs = 0x02,
        ReadHoldingRegisters = 0x03,
        ReadInputRegisters = 0x04,
        WriteSingleCoil = 0x05,
        WriteSingleRegister = 0x06,
        ReadExceptionStatus = 0x07,
        WriteMultipleCoils = 0x0F,
        WriteMultipleRegisters = 0x10,
        ReportSlaveId = 0x11,
        MaskWriteRegister = 0x16,
        WriteAndReadRegisters = 0x17,
    };
    /*!
    \brief Максимальный размер регистров ModBus
    */
    enum RegisterLimits{
        MaxDiscreteOutput = 9999,
        MaxDiscreteInput = 9999,
        MaxAnalogOutput = 9999,
        MaxAnalogInput = 9999,
    };
}

#endif // MODBUS_CONSTANTS_H
