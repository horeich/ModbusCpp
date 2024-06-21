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

#include "ModbusRTUClient.hpp"

#define TRACE_GROUP "MODB"

using namespace mbed;
using namespace std::chrono_literals;

ModbusRTUClient::ModbusRTUClient(uint8_t address) : 
    ModbusRTU(address),
    _onReadHoldingRegister(nullptr),
    _onWriteHoldingRegister(nullptr),
    _onWriteCoilRegister(nullptr)
{
    log_debug("ModbusRTUClient::%s", __func__);
}

uint32_t ModbusRTUClient::charSendTime(uint32_t baud, uint8_t char_bits)
{
    return (uint32_t)char_bits * 1000000UL / baud;
}

uint32_t ModbusRTUClient::calculateMinimumInterFrameTime(uint32_t baud, uint8_t char_bits)
{
    // baud = baudrate of the serial port
    // char_bits = size of 1 modbus character (defined a 11 bits in modbus specificacion)
    // Returns: The minimum time between frames (defined as 3.5 characters time in modbus specification)

    // According to standard, the Modbus frame is always 11 bits long:
    // 1 start + 8 data + 1 parity + 1 stop
    // 1 start + 8 data + 2 stops
    // And the minimum time between frames is defined as 3.5 characters time in modbus specification.
    // This means the time between frames (in microseconds) should be calculated as follows:
    // _iframePeriod = 3.5 x 11 x 1000000 / baudrate = 38500000 / baudrate

    // Eg: For 9600 baudrate _iframePeriod = 38500000 / 9600 = 4010 us
    // For baudrates grater than 19200 the _iframePeriod should be fixed at 1750 us.

    // If the used modbus frame length is 10 bits (out of standard - 1 start + 8 data + 1 stop), then
    // it can be set using char_bits = 10.

    if (baud > 19200)
    {
        return 1750UL;
    }
    else
    {
        return 3.5 * charSendTime(baud, char_bits);
    }
}

// Kept for backward compatibility
void ModbusRTUClient::setBaudrate(uint32_t baud)
{
    setInterFrameTime(calculateMinimumInterFrameTime(baud));
}

void ModbusRTUClient::setInterFrameTime(uint32_t t_us)
{
    // This function sets the inter frame time. This time is the time that task() waits before considering that the frame being transmitted on the RS485 bus has finished.
    // If the interframe calculated by calculateMinimumInterFrameTime() is not enough, you can set the interframe time manually with this function.
    // The time must be set in micro seconds.
    // This is useful when you are receiving data as a slave and you notice that the slave is dividing a frame in two or more pieces (and obviously the CRC is failing on all pieces).
    // This is because it is detecting an interframe time inbetween bytes of the frame and thus it interprets one single frame as two or more frames.
    // In that case it is useful to be able to set a more "permissive" interframe time.
    
    //_iframePeriod = std::chrono::microseconds(t_us);
}

bool ModbusRTUClient::ReadRegisters(uint16_t startRegisterNumber, uint16_t numOfRegisters, uint16_t* payload, uint8_t payloadSize)
{
    log_debug("ModbusRTUClient::%s", __func__);

    if (payloadSize != 2 * numOfRegisters)
    {
        log_warn("Not enough buffer (%hu) for registers (%hu)", payloadSize, numOfRegisters);
        return false;
    }

    // _mutex.lock();
    for (ModbusAddress addr = startRegisterNumber; addr < (startRegisterNumber + numOfRegisters); ++addr)
    {
        auto it = _registers.find(addr);
        if (it != _registers.end())
        {
            if (it->second->isHreg()) // is this a writeable register?
            {
                payload[(addr-startRegisterNumber)*2]       = static_cast<uint8_t>(it->second->value >> 8);
                payload[(addr-startRegisterNumber)*2 + 1]   = static_cast<uint8_t>(it->second->value);
            }
            else
            {
                log_warn("Trying to read from non-HREG register [0x%04X]", (addr));
                // _mutex.unlock();
                return false;
            }
        }
        else
        {
            log_warn("Trying to read from non-existant register [0x%04X]", (addr));
            // _mutex.unlock();
            return false;
        }
    }
    // _mutex.unlock();
    return true;
}

// ModbusRTU::Register* ModbusRTUClient::GetRegisterReference(uint16_t registerNumber)
// {
//     auto it = _registers.find(registerNumber);
//     if (it != _registers.end())
//     {
//         return it->second.get();
//     }
//     return nullptr;
// }

bool ModbusRTUClient::WriteSingleRegister(uint16_t registerNumber, const uint16_t payload)
{
    log_debug("ModbusRTUClient::%s", __func__);
    // TODO: mutex
    // _mutex.lock();
    auto it = _registers.find(registerNumber);
    if (it != _registers.end())
    {
        it->second->value = payload;
    }
    else
    {
        log_warn("Trying to write to non-existant register [0x%04X]", (registerNumber));
        // _mutex.unlock();
        return false;
    }
    // _mutex.unlock();
    return true;
}

bool ModbusRTUClient::WriteRegisters(uint16_t startRegisterNumber, uint16_t numOfRegisters, const uint8_t* payload, uint16_t payloadSize)
{
    log_debug("ModbusRTUClient::%s", __func__);

    if (payloadSize != 2 * numOfRegisters)
    {
        log_warn("Payload (%hu) does not fit in registers (%hu)", payloadSize, numOfRegisters);
        return false;
    }
    
    // _mutex.lock();
    for (ModbusAddress addr = startRegisterNumber; addr < (startRegisterNumber + numOfRegisters); ++addr)
    {
        auto it = _registers.find(addr);
        if (it != _registers.end())
        {
            if (it->second->isHreg() || it->second->isCoil()) // is this a writeable register?
            {
                it->second->value = ((payload[(addr-startRegisterNumber)*2] << 8) | payload[(addr-startRegisterNumber)*2 + 1]);
            }
            else
            {
                log_warn("Trying to write to read-only register [0x%04X]", (addr));
                // _mutex.unlock();
                return false;
            }
        }
        else
        {
            log_warn("Trying to write to non-existant register [0x%04X]", (addr));
            // _mutex.unlock();
            return false;
        }
    }
    // _mutex.unlock();
    return true;
}

void ModbusRTUClient::SetReadHoldingRegisterCallback(Callback<bool(uint16_t)> callback)
{
    _onReadHoldingRegister = callback;
}

void ModbusRTUClient::SetWriteHoldingRegisterCallback(Callback<void(uint16_t address, const uint8_t*, uint8_t)> callback)
{
    _onWriteHoldingRegister = callback;
}

void ModbusRTUClient::SetWriteCoilRegisterCallback(Callback<void(uint16_t address, bool value)> callback)
{
    _onWriteCoilRegister = callback;
}

bool ModbusRTUClient::ProcessBuffer(uint8_t* recvBuffer, uint32_t recvBufferSize, uint8_t* respBuffer, uint8_t& respBufferSize)
{
    log_debug("ModbusRTUClient::%s", __func__);

    // Message structure:
    // |ADDRESS|FUNCTION CODE|252 x DATA|CRCCHECK|CRCCHECK|

    // Check maximum frame size
    if (recvBufferSize > MODBUS_RTU_MAX_MESSAGE_SIZE)
    {
        log_warn("Frame size to large (%u) - ignore message", recvBufferSize);
        return false; // leads to meassage timeout on calling machine
    }

    // Check minimum frame size
    // Frame size to small -> discard
    if (recvBufferSize < MODBUS_MINIMUM_FRAME_SIZE) // minimum size
    {
        log_warn("Frame size to small (%u) - ignore message", recvBufferSize);
        return false; // leads to meassage timeout on calling machine
    }

    uint8_t address = recvBuffer[0]; 
    log_info("Address [0x%02x]", address);

    // Check if the message is meant for this client
    // Frame is discarded otherwise (see specification)
    if (address != MODBUS_RTU_BROADCAST && address != _address)
    {
        log_warn("Invalid address 0x%02X - ignore message", address);
        return false;
    }

    // Check CRC
    // If invalid -> frame is discarded (see specification)
    uint16_t recvCrc = (recvBuffer[recvBufferSize - 1] << 8) | recvBuffer[recvBufferSize - 2];
    log_info("recvCrc [0x%04X]", recvCrc);
    uint16_t crc = CRC16(&recvBuffer[0], recvBufferSize - 2); // CRC includes address but not CRC itself
    log_info("crc [0x%04X]", crc);
    if (recvCrc != crc)
    {
        log_warn("Checksum failed - ignore message");
        return false; // leads to meassage timeout on calling machine
    }

    // Process the PDU (function code and )
    uint8_t* framePtr = &recvBuffer[1]; // includes function code
    uint8_t frameSize = recvBufferSize - 3; // - address - function code - crc x 2

    // all returns here will cause timeouts on the server/host
    _respBuffer = respBuffer;
    ProcessPDU(framePtr, frameSize);
    if (address != MODBUS_RTU_BROADCAST)
    {
        // Send();
        crc = CRC16(&_respBuffer[0], _respBufferSize);
        log_info("Response CRC [0x%04X]", crc);

        _respBuffer[_respBufferSize++] = crc & 0x00FF;
        _respBuffer[_respBufferSize++] = crc >> 8;

        respBufferSize = _respBufferSize;
        return true;
    }
    return false;
}

// Private

ModbusRTU::ExceptionCode ModbusRTUClient::ProcessPDU(uint8_t* framePtr, uint8_t frameSize)
{
    log_debug("ModbusRTUClient::%s", __func__);

    ExceptionCode rc { EX_NONE };

    // Check if function code is correct
    FunctionCode functionCode = static_cast<FunctionCode>(framePtr[0]); // TODO: test > 17 code
    log_info("FunctionCode [0x%02X]", (int)functionCode);
    if (!IsFunctionCodeValid(functionCode))
    {
        log_info("Invalid function code [0x%04X]", (int)functionCode);
        rc = EX_ILLEGAL_FUNCTION;
        CreateException(functionCode, rc);
        return rc;
    }

    uint16_t field1 = ((framePtr[1] << 8) | (framePtr[2] & 0x00FF)); // (start) data register number
    uint16_t field2 = ((framePtr[3] << 8) | (framePtr[4] & 0x00FF)); // register count

    // log_info("f1 [0x%04x]", field1);
    // log_info("f2 [0x%04x]", field2);

    switch (functionCode)
    {
    case FC_READ_HOLDING_REGS: // read holding registers FC03
        rc = CreateFC03Response(functionCode, field1, field2);
        break;
    case FC_WRITE_SINGLE_COIL:
        rc = CreateFC05Response(functionCode, field1, &framePtr[3]);
        break;
    case FC_WRITE_SINGLE_REG: // write single register FC06
        rc = CreateFC06Response(functionCode, field1, &framePtr[3]);
        break;
    case FC_WRITE_MULTIPLE_REGS: // write multiple registers FC16
        rc = CreateFC16Response(functionCode, field1, field2, &framePtr[5]);
        break;
    
        
    default:
        log_warn("Function code not handled");
    }

    if (rc != EX_NONE)
    {
        CreateException(functionCode, rc);
        return rc;
    }

    return rc;
}

ModbusRTU::ExceptionCode ModbusRTUClient::CreateFC05Response(FunctionCode functionCode, uint16_t registerNumber, uint8_t* value)
{
    // Write Single Coil (FC=05)
    // |  0xXX |     0x05    |     0xXXXX    |     0xXX00    |   0xXX  |   0xXX  |
    // |ADDRESS|FUNCTION CODE|REGISTER NUMBER|STATUS TO WRITE|CRC CHECK|CRC CHECK|

    // TODO: Check message size?
    log_info("Register number [0x%04X]", registerNumber);
    if (value[0] != 0xFF && value[0] != 0x00)
    {
        log_warn("Invalid coil value [0x%02X]", value[0]);
        return EX_ILLEGAL_VALUE;
    }

    log_info("Coil value [0x%02X]", value[0]);
    
    if (!WriteRegisters(registerNumber, 1, value, 2))
    {
        log_warn("Failed to write registers");
        return EX_ILLEGAL_DATA_ADDRESS;
    }

    if (_onWriteCoilRegister != nullptr)
    {
        _onWriteCoilRegister(registerNumber, static_cast<bool>(value[0]));
    }

    // Build response
    // |ADDRESS|FUNCTION CODE|REGISTER NUMBER|WRITTEN STATUS VALUE|CRCCHECK|CRCCHECK|
    _respBufferSize = 0; // reset buffer
    _respBuffer[_respBufferSize++] = _address;
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(functionCode);
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerNumber >> 8); 
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerNumber);
    _respBuffer[_respBufferSize++] = value[0]; // TODO: read from real register!
    _respBuffer[_respBufferSize++] = value[1];

    return EX_NONE;
}

ModbusRTU::ExceptionCode ModbusRTUClient::CreateFC06Response(FunctionCode functionCode, uint16_t registerNumber, uint8_t* payload)
{
    // Write Single Register (FC=06)
    // |   1   |      1      |     0xXXXX    |    0x0000    |  0xXX   |  0x00   |
    // |ADDRESS|FUNCTION CODE|REGISTER NUMBER|REGISTER VALUE|CRC CHECK|CRC CHECK|

    // Check min/max number of registers
    // TODO: Check message size?

    log_info("Register count [%hu]", 1);
    log_info("Register number [%hu]", registerNumber);

    // Payload size is fixed: 2
    if (!WriteRegisters(registerNumber, 1, payload, 2))
    {
        log_warn("Failed to write registers");
        return EX_ILLEGAL_DATA_ADDRESS;
    }

    // Do work if callback is registered
    if (_onWriteHoldingRegister != nullptr)
    {
        _onWriteHoldingRegister(registerNumber, payload, 2);
    }

    // Build response
    // |ADDRESS|FUNCTION CODE|REGISTER NUMBER|WRITTEN REGISTER VALUE|CRCCHECK|CRCCHECK|
    _respBufferSize = 0; // reset buffer
    _respBuffer[_respBufferSize++] = _address;
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(functionCode);
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerNumber >> 8); 
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerNumber);
    _respBuffer[_respBufferSize++] = payload[0]; // TODO: read from real register!
    _respBuffer[_respBufferSize++] = payload[1];

    return EX_NONE;
}

ModbusRTU::ExceptionCode ModbusRTUClient::CreateFC16Response(FunctionCode functionCode, uint16_t startRegisterNumber, uint16_t registerCount, uint8_t* payload)
{
    // Data format
    // |ADDRESS|FUNCTION CODE|START ADDRESS|NUMBER OF REGISTERS TO WRITE|NUM OF BYTES TO FOLLOW|PAYLOAD|CRCCHECK|CRCCHECK|

    // Check min/max number of registers
    if (registerCount < 0x01 || registerCount > MODBUS_MAX_WORDS) //  || 0xFFFF - startreg.address < numregs)
    {
        log_info("Invalid number of registers %u", registerCount); // (https://www.simplymodbus.ca/exceptions.htm)
        return EX_ILLEGAL_VALUE;
    }

    log_info("Register count [%hu]", registerCount);
    log_info("Register number [%hu]", startRegisterNumber);

    // First byte give the number 
    uint8_t payloadSize = payload[0];
    if (payloadSize != 2 * registerCount)
    {
        log_warn("Byte count does not find register count");
        return EX_ILLEGAL_VALUE;
    }
    payload++;
    if (!WriteRegisters(startRegisterNumber, registerCount, payload, payloadSize))
    {
        log_warn("Failed to write registers");
        return EX_ILLEGAL_DATA_ADDRESS;
    }

    if (_onWriteHoldingRegister != nullptr)
    {
        _onWriteHoldingRegister(startRegisterNumber, payload, payloadSize);
    }

    // Build response
    // |ADDRESS|FUNCTION CODE|START ADDRESS|NUMBER OF WRITTEN REGISTERS|CRCCHECK|CRCCHECK|
    _respBufferSize = 0; // reset buffer
    _respBuffer[_respBufferSize++] = _address;
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(functionCode);
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(startRegisterNumber >> 8); // we have assured earlier that all registers ...
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(startRegisterNumber);
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerCount >> 8); //... have been successfully written
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerCount);

    return EX_NONE;
}

ModbusRTU::ExceptionCode ModbusRTUClient::CreateFC03Response(FunctionCode functionCode, uint16_t startRegisterNumber, uint16_t registerCount)
{
    // Check min/max number of registers
    if (registerCount < 0x01 || registerCount > MODBUS_MAX_WORDS) //  || 0xFFFF - startreg.address < numregs)
    {
        log_info("Invalid number of registers %u", registerCount); // (https://www.simplymodbus.ca/exceptions.htm)
        return EX_ILLEGAL_VALUE;
    }

    log_info("Register count [%hu]", registerCount);
    log_info("Register number [%hu]", startRegisterNumber);

    // Callback to do any related work before sending reply
    if (_onReadHoldingRegister != nullptr)
    {
        if (!_onReadHoldingRegister(startRegisterNumber))
        {
            return EX_SLAVE_FAILURE;
        }
    }

    _respBufferSize = 0; // reset buffer

    // Add general frame content
    // |ADDRESS|FUNCTION CODE|NUMBER OF BYTES TO FOLLOW (max 155x2)|CONTENT|CRCCHECK|CRCCHECK|
    _respBuffer[_respBufferSize++] = _address;
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(functionCode);
    _respBuffer[_respBufferSize++] = static_cast<uint8_t>(registerCount * 2); // we can savely cast here, because we checked illegal number of registers above

    // Read the values from the requested registers
    // _mutex.lock();
    for (ModbusAddress addr = startRegisterNumber; addr < (startRegisterNumber + registerCount); ++addr)
    {
        auto it = _registers.find(addr);
        if (it != _registers.end())
        {
            _respBuffer[_respBufferSize++] = (it->second->value >> 8);
            _respBuffer[_respBufferSize++] = it->second->value;
        }
        else
        {
            log_warn("The requested register (%hu) does not exist", (addr));
            // _mutex.unlock();
            return EX_ILLEGAL_DATA_ADDRESS; // register does not exist (https://www.simplymodbus.ca/exceptions.htm)
        }
    }
    // _mutex.unlock();
    return EX_NONE;
}















//rc = REPLY_OFF; // no response for broadcasts

 // ExceptionCode rc = EX_PASSTHROUGH;
    // if (rc == EX_PASSTHROUGH || rc == EX_FORCE_PROCESS)
    // {

    //     for (uint8_t i = 0; i < _len; i++)
    //     {
    //         _frame[i] = _serial->read(); // read data + crc
    // #if defined(MODBUS_DEBUG)
    //         log_info("frame %s", _frame);
    // #endif
    //     }
    //_serial->readBytes(_frame, _len);


// bool ModbusRTUClient::cleanup()
// {
//     // Remove timeouted request and forced event
//     // if (_id && (micros() - _timestamp > ModbusRTUClient_TIMEOUT_US))
//     // {
//     //     if (_cb)
//     //     {
//     //         _cb(Modbus::EX_TIMEOUT, 0, nullptr);
//     //         _cb = nullptr;
//     //     }
//     //     free(_sentFrame);
//     //     _sentFrame = nullptr;
//     //     _data = nullptr;
//     //     _id = 0;
//     //     return true;
//     // }
//     return false;
// }

// if (_isMaster)
// {
//     if (micros() - t < _iframePeriod)
//     {
//         return;
//     }
// }
// else
// {

// For slave wait for whole message to come (unless ModbusRTUClient_MAX_READMS reached)
// LowPowerTimer timer;
// timer.start();
// std::chrono::microseconds taskStart = timer.elapsed_time();
// while (timer.elapsed_time() - t < _iframePeriod)
// { // Wait data whitespace
//     _serial->receive_nb(_recvBuffer, 1024, &_len);
//     if (_serial->available() > _len)
//     {
//         _len = _serial->available();
//         t = micros();
//     }
//     if (timer.elapsed_time() - taskStart > MODBUSRTU_MAX_READMS)
//     { // Prevent from task() executed too long
//         return;
//     }
// }
//}


// bool valid_frame = true;
    // address = _serial->read(); // first byte of frame = address
    // _len--;                  // Decrease by slaveId byte
    // if (_isMaster && _id == 0)
    // { // Check if slaveId is set
    //     valid_frame = false;
    // }

    // if (!valid_frame && !_cbRaw)
    // {
    //     for (uint8_t i = 0; i < _len; i++)
    //         _serial->read(); // Skip packet if SlaveId doesn't mach
    //     _len = 0;
    //     // if (_isMaster)
    //     //     cleanup();
    //     return;
    // }

    // free(_frame); // Just in case

    // _frame = (uint8_t *)malloc(_len);
    // if (!_frame)
    // { // Fail to allocate _recvBuffer
    //     for (uint8_t i = 0; i < _len; i++)
    //         _serial->read(); // Skip packet if can't allocate _recvBuffer
    //     _len = 0;
    //     if (_isMaster)
    //         cleanup();
    //     return;
    // }

    
// uint16_t ModbusRTUClient::send(uint8_t slaveId, TAddress startreg, cbTransaction cb, uint8_t unit, uint8_t *data, bool waitResponse)
// {
//     bool result = false;
//     if ((!_isMaster || !_id) && _len && _frame)
//     { // Check if waiting for previous request result and _frame filled
//         // if (_len && _frame) { // Check if waiting for previous request result and _frame filled
//         rawSend(slaveId, _frame, _len);
//         if (waitResponse && slaveId)
//         {
//             _id = slaveId;
//             _timestamp = micros();
//             _cb = cb;
//             _data = data;
//             _sentFrame = _frame;
//             _sentReg = startreg;
//             _frame = nullptr;
//         }
//         result = true;
//     }
//     free(_frame);
//     _frame = nullptr;
//     _len = 0;
//     return result;
// }


  // uint8_t _recvBuffer[100]; // max size of USB _recvBuffer on STM32L476RG
    // _serial->receive_nb(_recvBuffer, 1024, &_len);

    // if (_len == 0)
    // {
    //     return;
    // } // if (_cbRaw)
    // {
    //     frame_arg_t header_data = { address, 0x01 };
    //     _reply = _cbRaw(_frame, _len, (void *)&header_data);
    // }
    // if (!valid_frame && _reply != EX_FORCE_PROCESS)
    // {
    //     goto cleanup;
    // }


    // bool ModbusRTUClient::begin(Stream *port, int16_t txPin, bool direct)
// {
//     _serial = port;
//     _iframePeriod = 1750UL;
// #if defined(MODBUS_FLUSH_DELAY)
//     _t1 = charSendTime(0);
// #endif
//     if (txPin >= 0)
//     {
//         _txPin = txPin;
//         _direct = direct;
//         pinMode(_txPin, OUTPUT);
//         digitalWrite(_txPin, _direct ? LOW : HIGH);
//     }
//     return true;
// }


// bool ModbusRTUClient::rawSend(uint8_t slaveId, uint8_t *frame, uint8_t len)
// {
//    //  uint16_t newCrc = crc16(slaveId, frame, len);
//     // #if defined(MODBUS_DEBUG)
//     //     for (uint8_t i = 0; i < _len; i++)
//     //     {
//     //         Serial.print(_frame[i], HEX);
//     //         Serial.print(" ");
//     //     }
//     //     Serial.println();
//     // #endif
//     // #if defined(MODBUS_REDE)
//     //     if (_txPin >= 0 || _rxPin >= 0)
//     //     {
//     //         if (_txPin >= 0)
//     //             digitalWrite(_txPin, _direct ? HIGH : LOW);
//     //         if (_rxPin >= 0)
//     //             digitalWrite(_rxPin, _direct ? HIGH : LOW);
//     // #if !defined(ESP32)
//     //         delayMicroseconds(ModbusRTUClient_REDE_SWITCH_US);
//     // #endif
//     //     }
//     // #else
//     //     if (_txPin >= 0)
//     //     {
//     //         digitalWrite(_txPin, _direct ? HIGH : LOW);
//     // #if !defined(ESP32)
//     //         delayMicroseconds(ModbusRTUClient_REDE_SWITCH_US);
//     // #endif
//     //     }
//     // #endif
//     // #if defined(ESP32)
//     //     vTaskDelay(0);
//     // #endif

//     // char *_recvBuffer = (char *)malloc(len + 3);
//     // _recvBuffer[0] = slaveId; // Send slaveId
//     // for (int i = 0; i < len; ++i)
//     // {
//     //     _recvBuffer[i + 1] = frame[i];
//     // }
//     // _recvBuffer[len + 1] = newCrc >> 8;
//     // _recvBuffer[len + 2] = newCrc & 0xFF;
//     //_serial->send((uint8_t *)_recvBuffer, len + 3);
    
//     // _serial->write(slaveId);       // Send slaveId
//     // _serial->write(frame, len);    // Send PDU
//     // _serial->write(newCrc >> 8);   // Send CRC
//     // _serial->write(newCrc & 0xFF); // Send CRC
//     // _serial->flush();
//     // #if defined(MODBUS_REDE)
//     //     if (_txPin >= 0 || _rxPin >= 0)
//     //     {
//     // #if defined(MODBUS_FLUSH_DELAY)
//     //         delayMicroseconds(_t1 * MODBUS_FLUSH_DELAY);
//     // #endif
//     //         if (_txPin >= 0)
//     //             digitalWrite(_txPin, _direct ? LOW : HIGH);
//     //         if (_rxPin >= 0)
//     //             digitalWrite(_rxPin, _direct ? LOW : HIGH);
//     //     }
//     // #else
//     //     if (_txPin >= 0)
//     //     {
//     // #if defined(MODBUS_FLUSH_DELAY)
//     //         delayMicroseconds(_t1 * MODBUS_FLUSH_DELAY);
//     // #endif
//     //         digitalWrite(_txPin, _direct ? LOW : HIGH);
//     //     }
//     // #endif
//     return true;
// }