/*
 * Copyright (c) 2022, ARM Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MODBUS_RTU_CLIENT_HPP
#define MODBUS_RTU_CLIENT_HPP

#include <mbed.h>
#include <chrono>
#include "ModbusRTU.hpp"
#include "Timer.h"
#include "mbed_trace.h"

namespace mbed
{
    class ModbusRTUClient : public ModbusRTU
    {
        
    public:
        explicit ModbusRTUClient(uint8_t slaveAddress);
        virtual ~ModbusRTUClient() = default;

        void SetReadHoldingRegisterCallback(Callback<bool(uint16_t)> callback);
        void SetWriteHoldingRegisterCallback(Callback<void(uint16_t address, const uint8_t*, uint8_t)> callback);
        void SetWriteCoilRegisterCallback(Callback<void(uint16_t address, bool value)> callback);

        bool ReadRegisters(uint16_t startRegisterNumber, uint16_t numOfRegisters, uint16_t* payload, uint8_t payloadSize);

        bool WriteSingleRegister(uint16_t registerNumber, const uint16_t payload);
        bool WriteRegisters(uint16_t registerNumber, uint16_t numOfRegisters, const uint8_t* payload, uint16_t payloadSize);

        void setBaudrate(uint32_t baud = -1);
        uint32_t calculateMinimumInterFrameTime(uint32_t baud, uint8_t char_bits = 11);
        void setInterFrameTime(uint32_t t_us);
        uint32_t charSendTime(uint32_t baud, uint8_t char_bits = 11);

        virtual bool ProcessBuffer(uint8_t*, uint32_t, uint8_t*, uint8_t&) override;

    protected:
        ModbusRTU::ExceptionCode ProcessPDU(uint8_t* framePtr, uint8_t frameSize);
        ModbusRTU::ExceptionCode CreateFC03Response(
            FunctionCode functionCode, uint16_t startRegisterNumber, uint16_t registerCount);

        ModbusRTU::ExceptionCode CreateFC16Response(
            FunctionCode functionCode, uint16_t startRegisterNumber, uint16_t registerCount, uint8_t* payload);

        ModbusRTU::ExceptionCode CreateFC06Response(
            FunctionCode functionCode, uint16_t registerNumber, uint8_t* payload);

        ModbusRTU::ExceptionCode CreateFC05Response(
            FunctionCode functionCode, uint16_t registerNumber, uint8_t* value);

    private:
        Callback<bool(uint16_t)> _onReadHoldingRegister;
        Callback<void(uint16_t address, const uint8_t*, uint8_t)> _onWriteHoldingRegister;
        Callback<void(uint16_t address, bool value)> _onWriteCoilRegister;
        std::chrono::microseconds t = 0us; // time sience last data byte arrived
        uint32_t _timestamp = 0;
    };

    using upModbusRTUClient = std::unique_ptr<ModbusRTUClient>;

} // namespace mbed

#endif // MODBUS_RTU_CLIENT_HPP






// #if defined(MODBUSRTU_FLUSH_DELAY)
//         uint32_t _t1; // char send time
// #endif

        //std::chrono::microseconds _iframePeriod; // inter-frame delay in uS

// int16_t _txPin = -1;

//         bool _direct = true;                     // Transmit control logic (true=direct, false=inverse)

// #if defined(MODBUSRTU_REDE)
//         int16_t _rxPin = -1;
// #endif  

 // uint16_t send(uint8_t slaveId, Address startreg, cbTransaction cb, uint8_t unit = MODBUSIP_UNIT, uint8_t *data = nullptr, bool waitResponse = true);
        // Prepare and send ModbusRTU frame. _frame buffer and _recvLength should be filled with Modbus data
        // slaveId - slave id
        // startreg - first local register to save returned data to (miningless for write to slave operations)
        // cb - transaction callback function
        // data - if not null use buffer to save returned data instead of local registers


 // template <class T>

        // bool begin(T *port, int16_t txPin = -1, bool direct = true);
// #if defined(MODBUSRTU_REDE)
//         template <class T>
//         // bool begin(T *port, int16_t txPin, int16_t rxPin, bool direct);
// #endif
        // bool begin(Stream *port, int16_t txPin = -1, bool direct = true);

   // uint8_t *_data = nullptr;
        // uint8_t *_sentFrame = nullptr;
        // Address _sentReg = COIL(0);
        // uint16_t maxRegs = 0x007D;
        // uint8_t address = 0;


 // inline void master() { client(); }
        // void server(uint8_t serverId) { _id = serverId; };
        // inline void slave(uint8_t slaveId) { server(slaveId); }
        // uint8_t server() { return _id; }
        // inline uint8_t slave() { return server(); }
        // uint32_t eventSource() override { return address; }

// template <class T>
// bool ::begin(T *port, int16_t txPin, bool direct)
// {
//     uint32_t baud = 0;
// #if defined(ESP32) || defined(ESP8266) // baudRate() only available with ESP32+ESP8266
//     baud = port->baudRate();
// #else
//     baud = 9600;
// #endif
//     setInterFrameTime(calculateMinimumInterFrameTime(baud));
// #if defined(MODBUSRTU_FLUSH_DELAY)
//     _t1 = charSendTime(baud);
// #endif
//     _serial = port;
//     if (txPin >= 0)
//     {
//         _txPin = txPin;
//         _direct = direct;
//         pinMode(_txPin, OUTPUT);
//         digitalWrite(_txPin, _direct ? LOW : HIGH);
//     }
//     return true;
// }
// #if defined(MODBUSRTU_REDE)
// template <class T>
// bool ::begin(T *port, int16_t txPin, int16_t rxPin, bool direct)
// {
//     begin(port, txPin, direct);
//     if (rxPin > 0)
//     {
//         _rxPin = rxPin;
//         pinMode(_rxPin, OUTPUT);
//         digitalWrite(_rxPin, _direct ? LOW : HIGH);
//     }
// }
// #endif

// class ModbusRTU : public ModbusAPI<>
// {
// };


