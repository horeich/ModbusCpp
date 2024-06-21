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

#include "ModbusRTU.hpp"

#define TRACE_GROUP "MODB"

using namespace mbed;
using namespace std::chrono_literals;

ModbusRTU::ModbusRTU(ModbusAddress address) :
    _address(address),
    _respBuffer(NULL),
    _respBufferSize(0),
    _registers()
    //_mutex(),
    // _serial(serial),
    // _thread(osPriorityHigh, 4096, nullptr, "modbus_thread"),
    // _state()
{
    log_debug("ModbusRTU::%s", __func__);

    // serial->attach(mbed::callback(this, &ModbusRTU::Sigio), USBStream::RxIrq);
    // osStatus status = _thread.start(mbed::callback(this, &ModbusRTU::Run));
    // if (status != osOK)
    // {
    //     tr_error("Failed to start the Modbus thread");
    //     // TODO: MBED_ERROR
    // }
}

ModbusRTU::~ModbusRTU()
{
    // _state.set(STATE_SUSPEND);
    // _thread.join();
}

void ModbusRTU::Send()
{
    // log_debug("ModbusRTU::%s", __func__);

    // uint16_t crc = CRC16(&_respBuffer[0], _respBufferSize);
    // log_info("Response CRC [0x%04X]", crc);

    // _respBuffer[_respBufferSize++] = crc & 0x00FF;
    // _respBuffer[_respBufferSize++] = crc >> 8;

    // for (int i = 0; i < _respBufferSize; ++i)
    // {
    //     log_info("[0x%02X]", _respBuffer[i]);
    // }

    // if (!_serial->send(_respBuffer, _respBufferSize)) // waits until buffer is sent
    // {
    //     log_warn("Failed to send message");
    // }

    // log_info("Successfully sent data");
}

bool ModbusRTU::IsFunctionCodeValid(FunctionCode code)
{
    log_debug("ModbusRTU::%s", __func__);

    return ((code <= FC_READWRITE_REGS && code >= FC_READ_COILS) ? true : false);
}

// Protected

// Private

// void ModbusRTU::Sigio()
// {
//     _state.set(STATE_RECEIVE_AND_SEND);
// }

// void ModbusRTU::Disconnected()
// {

// }#

// TODO: USB POWERED?

void ModbusRTU::Run()
{
    // log_debug("ModbusRTU::%s", __func__);
    // log_info("Connect USB");

    // _serial->connect(); // connects usb device

    // log_info("Wait USB ready");
    // // while(!_serial->ready())
    // // {
        
    // // }
    // _serial->wait_ready();

    // log_info("USB is ready");

    // while(1)
    // {
    //     // TODO: description why blocking
    //     int rc = _state.wait_any_for(STATE_RECEIVE_AND_SEND | STATE_SUSPEND, std::chrono::milliseconds::max(), true);
    //     if (rc > 0 && rc & STATE_SUSPEND)
    //     {
    //         break;
    //     }
    //     if (rc > 0 && rc & STATE_RECEIVE_AND_SEND)
    //     {
    //         log_info("Start receiving data");

    //         // According to modbus message frame definition,
    //         // the entire message frame must be transmitted as a continuous stream of characters.
    //         // ✓ This requires the message to be shorter than the USB buffer (STM32L476RG: 1024bytes, RX buffer: 64 bytes) 
    //         // ✓ Messages are only valid if received as whole (incomplete messages will just be discarded)
    //         uint32_t receivedBytes;
    //         _serial->receive(_recvBuffer, MODBUS_RTU_MAX_MESSAGE_SIZE, &receivedBytes); // receives data if avaiable -> assume full message
    //         if (receivedBytes > 0)
    //         {
    //             _recvBufferSize = (uint8_t)receivedBytes; // TODO: check size

    //             for (int i = 0; i < _recvBufferSize; ++i)
    //             {
    //                 log_info("[0x%02X]", _recvBuffer[i]);
    //             }
    //             ProcessData();
    //         }
    //         else
    //         {
    //             log_info("no incoming data");
    //         }
    //     }
    //     else // some error in flags
    //     {

    //     }
    // }
}

uint16_t ModbusRTU::CRC16(uint8_t* buffer, uint16_t buffer_length)
{
    log_debug("ModbusRTU::%s", __func__);

    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    // unsigned int i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        unsigned int i = crc_lo ^ *buffer++; /* calculate the CRC  */
        crc_lo = crc_hi ^ table_crc_hi[i];
        crc_hi = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}

bool ModbusRTU::AddRegister(Register::Type type, uint16_t startRegisterNumber, uint8_t numOfRegisters)
{
    log_debug("ModbusRTU::%s", __func__);

    // Check if registers already exist
    //_mutex.lock();
    for (ModbusAddress addr = startRegisterNumber; addr < (startRegisterNumber + numOfRegisters); ++addr)
    {
        auto it = _registers.find(addr);
        if (it != _registers.end()) // exists already
        {
            //_mutex.unlock();
            return false;
        }
    }

    for (ModbusAddress addr = startRegisterNumber; addr < (startRegisterNumber + numOfRegisters); ++addr)
    {
        _registers.insert({addr, std::make_unique<Register>(type, 0x00)});
    }
    //_mutex.unlock();
    return true;
}

void ModbusRTU::CreateException(FunctionCode functionCode, ExceptionCode ExceptionCode)
{
    log_debug("ModbusRTU::%s", __func__);

    // if (!_frame)
    // {
    //     _reply = REPLY_OFF;
    //     return;
    // }
    _respBufferSize = 0;
    _respBuffer[_respBufferSize++] = _address;
    uint8_t fc = static_cast<uint8_t>(functionCode);
    fc |= 0x80; // set highest bit
    _respBuffer[_respBufferSize++] = fc;
    _respBuffer[_respBufferSize++] = ExceptionCode;
}

























// std::vector<ModbusRTU::TCallback> ModbusRTU::_callbacks;
// std::function<ModbusRTU::ExceptionCode(ModbusRTU::FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)> ModbusRTU::_onFile;
// // #if defined(MODBUS_FILES)
// // cbModbusFileOp ModbusRTU::_onFile = nullptr;
// // #endif
// #endif



// void ModbusRTU::successResponce(ModbusRTU::Address startreg, uint16_t numoutputs, FunctionCode fn)
// {
//     free(_frame);
//     _len = 5;
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//     {
//         _reply = REPLY_OFF;
//         return;
//     }
//     _frame[0] = fn;
//     _frame[1] = startreg.address >> 8;
//     _frame[2] = startreg.address & 0x00FF;
//     _frame[3] = numoutputs >> 8;
//     _frame[4] = numoutputs & 0x00FF;
// }

// void ModbusRTU::exceptionResponse(FunctionCode fn, ExceptionCode excode)
// {
//     free(_frame);
//     _len = 2;
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//     {
//         _reply = REPLY_OFF;
//         return;
//     }
//     _frame[0] = fn + 0x80;
//     _frame[1] = excode;
//     _reply = REPLY_NORMAL;
// }

// void ModbusRTU::getMultipleBits(uint8_t *frame, ModbusRTU::Address startreg, uint16_t numregs)
// {
//     uint8_t bitn = 0;
//     uint16_t i = 0;
//     while (numregs--)
//     {
//         if (BIT_BOOL(Reg(startreg)))
//             bitSet(frame[i], bitn);
//         else
//             bitClear(frame[i], bitn);
//         bitn++; // increment the bit index
//         if (bitn == 8)
//         {
//             i++;
//             bitn = 0;
//         }
//         startreg++; // increment the register
//     }
// }

// void ModbusRTU::getMultipleWords(uint16_t *frame, ModbusRTU::Address startreg, uint16_t numregs)
// {
//     for (uint8_t i = 0; i < numregs; i++)
//     {
//         frame[i] = __swap_16(Reg(startreg + i));
//     }
// }

// ModbusRTU::ExceptionCode ModbusRTU::readBits(ModbusRTU::Address startreg, uint16_t numregs, FunctionCode fn)
// {
//     if (numregs < 0x0001 || numregs > MODBUS_MAX_BITS || (0xFFFF - startreg.address) < numregs)
//         return EX_ILLEGAL_VALUE;
//         // Check ModbusRTU::Address
//         // Check only startreg. Is this correct?
//         // When I check all registers in range I got errors in ScadaBR
//         // I think that ScadaBR request more than one in the single request
//         // when you have more then one datapoint configured from same type.
// #if defined(MODBUS_STRICT_REG)
//     for (k = 0; k < numregs; k++)
//     { // Check ModbusRTU::Address (startreg...startreg + numregs)
//         if (!FindRegister(startreg + k))
//             return EX_ILLEGAL_VALUE;
//     }
// #else
//     if (!FindRegister(startreg))
//         return EX_ILLEGAL_VALUE;
// #endif
//     free(_frame);
//     // Determine the message length = function type, byte count and
//     // for each group of 8 registers the message length increases by 1
//     _len = 2 + numregs / 8;
//     if (numregs % 8)
//         _len++; // Add 1 to the message length for the partial byte.
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//         return EX_SLAVE_FAILURE;
//     _frame[0] = fn;
//     _frame[1] = _len - 2; // byte count (_len - function code and byte count)
//     _frame[_len - 1] = 0; // Clean last probably partial byte
//     getMultipleBits(_frame + 2, startreg, numregs);
//     _reply = REPLY_NORMAL;
//     return EX_SUCCESS;
// }

// ModbusRTU::ExceptionCode ModbusRTU::readWords(ModbusRTU::Address startreg, uint16_t numregs, FunctionCode fn)
// {
//     // Check value (numregs)
//     if (numregs < 0x0001 || numregs > MODBUS_MAX_WORDS || 0xFFFF - startreg.address < numregs)
//         return EX_ILLEGAL_VALUE;
// #if defined(MODBUS_STRICT_REG)
//     for (k = 0; k < numregs; k++)
//     { // Check ModbusRTU::Address (startreg...startreg + numregs)
//         if (!FindRegister(startreg + k))
//             return EX_ILLEGAL_VALUE;
//     }
// #else
//     if (!FindRegister(startreg))
//         return EX_ILLEGAL_ADDRESS;
// #endif
//     free(_frame);
//     _len = 2 + numregs * 2; // calculate the query reply message length. 2 bytes per register + 2 bytes for header
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//         return EX_SLAVE_FAILURE;
//     _frame[0] = fn;
//     _frame[1] = _len - 2; // byte count
//     getMultipleWords((uint16_t *)(_frame + 2), startreg, numregs);
//     _reply = REPLY_NORMAL;
//     return EX_SUCCESS;
// }

// bool ModbusRTU::setMultipleBits(uint8_t *frame, Address startreg, uint16_t numoutputs)
// {
//     uint8_t bitn = 0;
//     uint16_t i = 0;
//     bool result = true;
//     while (numoutputs--)
//     {
//         Reg(startreg, BIT_VAL(bitRead(frame[i], bitn)));
//         if (Reg(startreg) != BIT_VAL(bitRead(frame[i], bitn)))
//             result = false;
//         bitn++; // increment the bit index
//         if (bitn == 8)
//         {
//             i++;
//             bitn = 0;
//         }
//         startreg++; // increment the register
//     }
//     return result;
// }

// bool ModbusRTU::setMultipleWords(uint16_t *frame, Address startreg, uint16_t numregs)
// {
//     bool result = true;
//     for (uint8_t i = 0; i < numregs; i++)
//     {
//         Reg(startreg + i, __swap_16(frame[i]));
//         if (Reg(startreg + i) != __swap_16(frame[i]))
//         {
//             result = false;
//         }
//     }
//     return result;
// }

// bool ModbusRTU::onGet(ModbusRTU::Address address, cbModbus cb, uint16_t numregs)
// {
//     Register *reg;
//     bool atLeastOne = false;
//     if (!cb)
//     {
//         return removeOnGet(address);
//     }
//     while (numregs > 0)
//     {
//         reg = FindRegister(address);
//         if (reg)
//         {
//             _callbacks.push_back({TCallback::ON_GET, address, cb});
//             atLeastOne = true;
//         }
//         address++;
//         numregs--;
//     }
//     return atLeastOne;
// }
// bool ModbusRTU::onSet(ModbusRTU::Address address, cbModbus cb, uint16_t numregs)
// {
//     ModbusRTU::Register *reg;
//     bool atLeastOne = false;
//     if (!cb)
//     {
//         return removeOnGet(address);
//     }
//     while (numregs > 0)
//     {
//         reg = FindRegister(address);
//         if (reg)
//         {
//             _callbacks.push_back({TCallback::ON_SET, address, cb});
//             atLeastOne = true;
//         }
//         address++;
//         numregs--;
//     }
//     return atLeastOne;
// }

// bool ModbusRTU::removeOn(TCallback::CallbackType t, ModbusRTU::Address address, cbModbus cb, uint16_t numregs)
// {
//     size_t s = _callbacks.size();
// #if defined(MODBUS_USE_STL)
// #define MODBUS_COMPARE_ON [t, address, cb](const TCallback entry) { return entry.type == t && entry.address == address && (!cb || std::addressof(cb) == std::addressof(entry.cb)); }
//     while (numregs--)
//     {
//         _callbacks.erase(remove_if(_callbacks.begin(), _callbacks.end(), MODBUS_COMPARE_ON), _callbacks.end());
//         address++;
//     }
// #else
// #define MODBUS_COMPARE_ON [t, address, cb](const TCallback entry) { return entry.type == t && entry.address == address && (!cb || entry.cb == cb); }
//     while (numregs--)
//     {
//         size_t r = 0;
//         do
//         {
//             r = _callbacks.find(MODBUS_COMPARE_ON);
//             _callbacks.remove(r);
//         } while (r < _callbacks.size());
//         address++;
//     }
// #endif
//     return s == _callbacks.size();
// }
// bool ModbusRTU::removeOnSet(ModbusRTU::Address address, cbModbus cb, uint16_t numregs)
// {
//     return removeOn(TCallback::ON_SET, address, cb, numregs);
// }

// bool ModbusRTU::removeOnGet(ModbusRTU::Address address, cbModbus cb, uint16_t numregs)
// {
//     return removeOn(TCallback::ON_GET, address, cb, numregs);
// }

// bool ModbusRTU::readSlave(uint16_t address, uint16_t numregs, FunctionCode fn)
// {
//     free(_frame);
//     _len = 5;
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//     {
//         _reply = REPLY_OFF;
//         return false;
//     }
//     _frame[0] = fn;
//     _frame[1] = address >> 8;
//     _frame[2] = address & 0x00FF;
//     _frame[3] = numregs >> 8;
//     _frame[4] = numregs & 0x00FF;
//     return true;
// }

// bool ModbusRTU::writeSlaveBits(ModbusRTU::Address startreg, uint16_t to, uint16_t numregs, FunctionCode fn, bool *data)
// {
//     free(_frame);
//     _len = 6 + numregs / 8;
//     if (numregs % 8)
//         _len++; // Add 1 to the message length for the partial byte.
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//     {
//         _reply = REPLY_OFF;
//         return false;
//     }
//     _frame[0] = fn;
//     _frame[1] = to >> 8;
//     _frame[2] = to & 0x00FF;
//     _frame[3] = numregs >> 8;
//     _frame[4] = numregs & 0x00FF;
//     _frame[5] = _len - 6;
//     _frame[_len - 1] = 0; // Clean last probably partial byte
//     if (data)
//     {
//         boolToBits(_frame + 6, data, numregs);
//     }
//     else
//     {
//         getMultipleBits(_frame + 6, startreg, numregs);
//     }
//     _reply = REPLY_NORMAL;
//     return true;
// }

// bool ModbusRTU::writeSlaveWords(ModbusRTU::Address startreg, uint16_t to, uint16_t numregs, FunctionCode fn, uint16_t *data)
// {
//     free(_frame);
//     _len = 6 + 2 * numregs;
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//     {
//         _reply = REPLY_OFF;
//         return false;
//     }
//     _frame[0] = fn;
//     _frame[1] = to >> 8;
//     _frame[2] = to & 0x00FF;
//     _frame[3] = numregs >> 8;
//     _frame[4] = numregs & 0x00FF;
//     _frame[5] = _len - 6;
//     if (data)
//     {
//         uint16_t *frame = (uint16_t *)(_frame + 6);
//         for (uint8_t i = 0; i < numregs; i++)
//         {
//             frame[i] = __swap_16(data[i]);
//         }
//     }
//     else
//     {
//         getMultipleWords((uint16_t *)(_frame + 6), startreg, numregs);
//     }
//     return true;
// }

// void ModbusRTU::bitSet(uint8_t &value, uint8_t bitn)
// {
//     uint8_t bit = (1 << bitn);
//     value |= bit;
// }

// void ModbusRTU::bitClear(uint8_t &value, uint8_t bitn)
// {
//     uint8_t bit = (1 << bitn);
//     value &= ~bit;
// }

// uint8_t ModbusRTU::bitRead(uint8_t &value, uint8_t bitn)
// {
//     uint8_t bit = (1 << bitn);
//     return (bit & value);
// }

// void ModbusRTU::boolToBits(uint8_t *dst, bool *src, uint16_t numregs)
// {
//     uint8_t bitn = 0;
//     uint16_t i = 0;
//     uint16_t j = 0;
//     while (numregs--)
//     {
//         if (src[j])
//             bitSet(dst[i], bitn);
//         else
//             bitClear(dst[i], bitn);
//         bitn++; // increment the bit index
//         if (bitn == 8)
//         {
//             i++;
//             bitn = 0;
//         }
//         j++; // increment the register
//     }
// }

// void ModbusRTU::bitsToBool(bool *dst, uint8_t *src, uint16_t numregs)
// {
//     uint8_t bitn = 0;
//     uint16_t i = 0;
//     uint16_t j = 0;
//     while (numregs--)
//     {
//         dst[j] = bitRead(src[i], bitn);
//         bitn++; // increment the bit index
//         if (bitn == 8)
//         {
//             i++;
//             bitn = 0;
//         }
//         j++; // increment the register
//     }
// }

// void ModbusRTU::masterPDU(uint8_t *frame, uint8_t *sourceFrame, ModbusRTU::Address startreg, uint8_t *output)
// {
//     uint8_t fcode = frame[0];
//     if ((fcode & 0x80) != 0)
//     { // Check if error responce
//         _reply = frame[1];
//         return;
//     }
//     if (fcode != sourceFrame[0])
//     { // Check if responce matches the request
//         _reply = EX_DATA_MISMACH;
//         return;
//     }
//     _reply = EX_SUCCESS;
//     uint16_t field2 = (uint16_t)sourceFrame[3] << 8 | (uint16_t)sourceFrame[4];
//     uint8_t bytecount_calc;
//     switch (fcode)
//     {
//     case FC_READ_HOLDING_REGS:
//     case FC_READ_INPUT_REGS:
//     case FC_READWRITE_REGS:
//         // field2 = numregs, frame[1] = data lenght, header len = 2
//         if (frame[1] != 2 * field2)
//         { // Check if data size matches
//             _reply = EX_DATA_MISMACH;
//             break;
//         }
//         if (output)
//         {
//             uint16_t *from = (uint16_t *)(frame + 2);
//             uint16_t *to = (uint16_t *)output;
//             while (field2--)
//             {
//                 *(to++) = __swap_16(*(from++));
//             }
//         }
//         else
//         {
//             setMultipleWords((uint16_t *)(frame + 2), startreg, field2);
//         }
//         break;
//     case FC_READ_COILS:
//     case FC_READ_INPUT_STAT:
//         // field2 = numregs, frame[1] = data length, header len = 2
//         bytecount_calc = field2 / 8;
//         if (field2 % 8)
//             bytecount_calc++;
//         if (frame[1] != bytecount_calc)
//         { // check if data size matches
//             _reply = EX_DATA_MISMACH;
//             break;
//         }
//         if (output)
//         {
//             bitsToBool((bool *)output, frame + 2, field2);
//         }
//         else
//         {
//             setMultipleBits(frame + 2, startreg, field2);
//         }
//         break;
// #if defined(MODBUS_FILES)
//     case FC_READ_FILE_REC:
//         // Should check if byte order swap needed
//         if (frame[1] < 0x07 || frame[1] > 0xF5)
//         { // Wrong request data size
//             _reply = EX_ILLEGAL_VALUE;
//             return;
//         }
//         {
//             uint8_t *data = frame + 2;
//             uint8_t *eoFrame = frame + frame[1];
//             while (data < eoFrame)
//             {
//                 // data[0] - sub-req length
//                 // data[1] = 0x06
//                 if (data[1] != 0x06 || data[0] < 0x07 || data[0] > 0xF5 || data + data[0] > eoFrame)
//                 { // Wrong request data size
//                     _reply = EX_ILLEGAL_VALUE;
//                     return;
//                 }
//                 memcpy(output, data + 2, data[0]);
//                 data += data[0] + 1;
//                 output += data[0] - 1;
//             }
//         }
//         break;
//     case FC_WRITE_FILE_REC:
// #endif
//     case FC_WRITE_REG:
//     case FC_WRITE_MULTIPLE_REGS:
//     case FC_WRITE_COIL:
//     case FC_WRITE_COILS:
//     case FC_MASKWRITE_REG:
//         break;

//     default:
//         _reply = EX_GENERAL_FAILURE;
//     }
// }

// bool ModbusRTU::cbEnable(const bool state)
// {
//     const bool old_state = state;
//     cbEnabled = state;
//     return old_state;
// }
// bool ModbusRTU::cbDisable()
// {
//     return cbEnable(false);
// }
// ModbusRTU::~Modbus()
// {
//     free(_frame);
// }

// #if defined(MODBUS_FILES)
// #if defined(MODBUS_USE_STL)
// bool ModbusRTU::onFile(std::function<ModbusRTU::ExceptionCode(ModbusRTU::FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)> cb)
// {
// #else
// bool ModbusRTU::onFile(ModbusRTU::ExceptionCode (*cb)(ModbusRTU::FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *))
// {
// #endif
//     _onFile = cb;
//     return true;
// }
// ModbusRTU::ExceptionCode ModbusRTU::fileOp(ModbusRTU::FunctionCode fc, uint16_t fileNum, uint16_t recNum, uint16_t recLen, uint8_t *frame)
// {
//     if (!_onFile)
//         return EX_ILLEGAL_ADDRESS;
//     return _onFile(fc, fileNum, recNum, recLen, frame);
// }

// bool ModbusRTU::readSlaveFile(uint16_t *fileNum, uint16_t *startRec, uint16_t *len, uint8_t count, FunctionCode fn)
// {
//     _len = count * 7 + 2;
//     if (_len > MODBUS_MAX_FRAME)
//         return false;
//     free(_frame);
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//         return false;
//     _frame[0] = fn;
//     _frame[1] = _len - 2;
//     uint8_t *subReq = _frame + 2;
//     for (uint8_t i = 0; i < count; i++)
//     {
//         subReq[0] = 0x06;
//         subReq[1] = fileNum[i] >> 8;
//         subReq[2] = fileNum[i] & 0x00FF;
//         subReq[3] = startRec[i] >> 8;
//         subReq[4] = startRec[i] & 0x00FF;
//         subReq[5] = len[i] >> 8;
//         subReq[6] = len[i] & 0x00FF;
//         subReq += 7;
//     }
//     return true;
// }
// bool ModbusRTU::writeSlaveFile(uint16_t *fileNum, uint16_t *startRec, uint16_t *len, uint8_t count, FunctionCode fn, uint8_t *data)
// {
//     _len = 2;
//     for (uint8_t i = 0; i < count; i++)
//     {
//         _len += len[i] * 2 + 7;
//     }
//     if (_len > MODBUS_MAX_FRAME)
//         return false;
//     free(_frame);
//     _frame = (uint8_t *)malloc(_len);
//     if (!_frame)
//         return false;
//     _frame[0] = fn;
//     _frame[1] = _len - 2;
//     uint8_t *subReq = _frame + 2;
//     for (uint8_t i = 0; i < count; i++)
//     {
//         subReq[0] = 0x06;
//         subReq[1] = fileNum[i] >> 8;
//         subReq[2] = fileNum[i] & 0x00FF;
//         subReq[3] = startRec[i] >> 8;
//         subReq[4] = startRec[i] & 0x00FF;
//         subReq[5] = len[i] >> 8;
//         subReq[6] = len[i] & 0x00FF;
//         uint8_t clen = len[i] * 2;
//         memcpy(subReq + 7, data, clen);
//         subReq += 7 + clen;
//         data += clen;
//     }
//     return true;
// }
// #endif

// bool ModbusRTU::onRaw(cbRaw cb)
// {
//     _cbRaw = cb;
//     return true;
// }
// ModbusRTU::ExceptionCode ModbusRTU::_onRequestDefault(ModbusRTU::FunctionCode fc, const RequestData data)
// {
//     return EX_SUCCESS;
// }
// bool ModbusRTU::onRequest(cbRequest cb)
// {
//     _onRequest = cb;
//     return true;
// }
// #if defined(MODBUSAPI_OPTIONAL)
// bool ModbusRTU::onRequestSuccess(cbRequest cb)
// {
//     _onRequestSuccess = cb;
//     return true;
// }
// #endif

// #if defined(ARDUINO_SAM_DUE_STL)
// namespace std
// {
//     void __throw_bad_function_call()
//     {
//         Serial.println(F("STL ERROR - __throw_bad_function_call"));
//         __builtin_unreachable();
//     }
// }
// #endif

// #endif //
// void ModbusRTU::DecodePDU(uint8_t *frame)
// {
//     // Retrieve function code and decide what to do
//     FunctionCode fcode = (FunctionCode)frame[0];
//     uint16_t field1 = (uint16_t)frame[1] << 8 | (uint16_t)frame[2]; // data register number
//     uint16_t field2 = (uint16_t)frame[3] << 8 | (uint16_t)frame[4]; // number of registeres
//     uint16_t field3 = 0;
//     uint16_t field4 = 0;
//     uint16_t bytecount_calc;
//     uint16_t k;
//     ExceptionCode ex;

//     log_info("%d", (int)fcode);
//     // rtos::ThisThread::sleep_for(1000s);

//     switch (fcode)
//     {
//     case FC_WRITE_REG:

//         // field1 = reg, field2 = value
//         ex = _onRequest(fcode, {HREG(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         if (!Reg(HREG(field1), field2))
//         { // Check ModbusRTU::Address and execute (reg exists?)
//             exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//             return;
//         }
//         if (Reg(HREG(field1)) != field2)
//         { // Check for failure
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         _reply = REPLY_ECHO;
//         _onRequestSuccess(fcode, {HREG(field1), field2});
//         break;

//     case FC_READ_HOLDING_REGS:
//         // field1 = startreg, field2 = numregs, header len = 3
//         ex = _onRequest(fcode, {HREG(field1), field2}); // Address(Address::HREG, field2));
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }

//         ex = readWords(HREG(field1), field2, fcode);
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         _onRequestSuccess(fcode, {HREG(field1), field2});
//         break;

//     case FC_WRITE_MULTIPLE_REGS:
//         // field1 = startreg, field2 = numregs, frame[5] = data lenght, header len = 6
//         ex = _onRequest(fcode, {HREG(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         if (field2 < 0x0001 || field2 > MODBUS_MAX_WORDS || 0xFFFF - field1 < field2 || frame[5] != 2 * field2)
//         { // Check constrains
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         for (k = 0; k < field2; k++)
//         { // Check ModbusRTU::Address (startreg...startreg + numregs)
//             if (!FindRegister(HREG(field1) + k))
//             {
//                 exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                 return;
//             }
//         }
//         if (!setMultipleWords((uint16_t *)(frame + 6), HREG(field1), field2))
//         {
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         successResponce(HREG(field1), field2, fcode);
//         _reply = REPLY_NORMAL;
//         _onRequestSuccess(fcode, {HREG(field1), field2});
//         break;

//     case FC_READ_COILS:
//         // field1 = startreg, field2 = numregs
//         ex = _onRequest(fcode, {COIL(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         ex = readBits(COIL(field1), field2, fcode);
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         _onRequestSuccess(fcode, {COIL(field1), field2});
//         break;

//     case FC_READ_INPUT_STAT:
//         // field1 = startreg, field2 = numregs
//         ex = _onRequest(fcode, {ISTS(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         ex = readBits(ISTS(field1), field2, fcode);
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         _onRequestSuccess(fcode, {ISTS(field1), field2});
//         break;

//     case FC_READ_INPUT_REGS:
//         // field1 = startreg, field2 = numregs
//         ex = _onRequest(fcode, {IREG(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         ex = readWords(IREG(field1), field2, fcode);
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         _onRequestSuccess(fcode, {IREG(field1), field2});
//         break;

//     case FC_WRITE_COIL:
//         // field1 = reg, field2 = status, header len = 3
//         ex = _onRequest(fcode, {COIL(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         if (field2 != 0xFF00 && field2 != 0x0000)
//         { // Check value (status)
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         if (!Reg(COIL(field1), field2))
//         { // Check ModbusRTU::Address and execute (reg exists?)
//             exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//             return;
//         }
//         if (Reg(COIL(field1)) != field2)
//         { // Check for failure
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         _reply = REPLY_ECHO;
//         _onRequestSuccess(fcode, {COIL(field1), field2});
//         break;

//     case FC_WRITE_COILS:
//         // field1 = startreg, field2 = numregs, frame[5] = bytecount, header len = 6
//         ex = _onRequest(fcode, {COIL(field1), field2});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         bytecount_calc = field2 / 8;
//         if (field2 % 8)
//             bytecount_calc++;
//         if (field2 < 0x0001 || field2 > MODBUS_MAX_BITS || 0xFFFF - field1 < field2 || frame[5] != bytecount_calc)
//         { // Check registers range and data size maches
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         for (k = 0; k < field2; k++)
//         { // Check ModbusRTU::Address (startreg...startreg + numregs)
//             if (!FindRegister(COIL(field1) + k))
//             {
//                 exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                 return;
//             }
//         }
//         if (!setMultipleBits(frame + 6, COIL(field1), field2))
//         {
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         successResponce(COIL(field1), field2, fcode);
//         _reply = REPLY_NORMAL;
//         _onRequestSuccess(fcode, {COIL(field1), field2});
//         break;
// #if defined(MODBUS_FILES)
//     case FC_READ_FILE_REC:
//         if (frame[1] < 0x07 || frame[1] > 0xF5)
//         { // Wrong request data size
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         {
//             uint8_t bufSize = 2;              // 2 bytes for frame header
//             uint8_t *recs = frame + 2;        // Begin of sub-recs blocks
//             uint8_t recsCount = frame[1] / 7; // Count of sub-rec blocks
//             for (uint8_t p = 0; p < recsCount; p++)
//             { // Calc output buffer size required
//                 // uint16_t fileNum = (uint16_t)recs[1] << 8 | (uint16_t)recs[2];
//                 uint16_t recNum = (uint16_t)recs[3] << 8 | (uint16_t)recs[4];
//                 uint16_t recLen = (uint16_t)recs[5] << 8 | (uint16_t)recs[6];
//                 // Serial.printf("%d, %d, %d\n", fileNum, recNum, recLen);
//                 if (recs[0] != 0x06 || recNum > 0x270F)
//                 { // Wrong ref type or count of records
//                     exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                     return;
//                 }
//                 bufSize += recLen * 2 + 2; // 4 bytes for header + data
//                 recs += 7;
//             }
//             if (bufSize > MODBUS_MAX_FRAME)
//             { // Frame to return too large
//                 exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                 return;
//             }
//             uint8_t *srcFrame = _frame;
//             _frame = (uint8_t *)malloc(bufSize);
//             if (!_frame)
//             {
//                 free(srcFrame);
//                 exceptionResponse(fcode, EX_SLAVE_FAILURE);
//                 return;
//             }
//             _len = bufSize;
//             recs = frame + 2; // Begin of sub-recs blocks
//             uint8_t *data = _frame + 2;
//             for (uint8_t p = 0; p < recsCount; p++)
//             {
//                 uint16_t fileNum = (uint16_t)recs[1] << 8 | (uint16_t)recs[2];
//                 uint16_t recNum = (uint16_t)recs[3] << 8 | (uint16_t)recs[4];
//                 uint16_t recLen = (uint16_t)recs[5] << 8 | (uint16_t)recs[6];
//                 ExceptionCode res = fileOp(fcode, fileNum, recNum, recLen, data + 2);
//                 if (res != EX_SUCCESS)
//                 { // File read failed
//                     free(srcFrame);
//                     exceptionResponse(fcode, res);
//                     return;
//                 }
//                 data[0] = recLen * 2 + 1;
//                 data[1] = 0x06;
//                 data += recLen * 2 + 2;
//                 recs += 7;
//             }
//             _frame[0] = fcode;
//             _frame[1] = bufSize;
//             _reply = REPLY_NORMAL;
//             free(srcFrame);
//         }
//         break;
//     case FC_WRITE_FILE_REC:
//     {
//         if (frame[1] < 0x09 || frame[1] > 0xFB)
//         { // Wrong request data size
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         uint8_t *recs = frame + 2; // Begin of sub-recs blocks
//         while (recs < frame + frame[1])
//         {
//             if (recs[0] != 0x06)
//             {
//                 exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                 return;
//             }
//             uint16_t fileNum = (uint16_t)recs[1] << 8 | (uint16_t)recs[2];
//             uint16_t recNum = (uint16_t)recs[3] << 8 | (uint16_t)recs[4];
//             uint16_t recLen = (uint16_t)recs[5] << 8 | (uint16_t)recs[6];
//             if (recs + recLen * 2 > frame + frame[1])
//             {
//                 exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//                 return;
//             }
//             ExceptionCode res = fileOp(fcode, fileNum, recNum, recLen, recs + 7);
//             if (res != EX_SUCCESS)
//             { // File write failed
//                 exceptionResponse(fcode, res);
//                 return;
//             }
//             recs += 7 + recLen * 2;
//         }
//     }
//         _reply = REPLY_ECHO;
//         break;
// #endif
//     case FC_MASKWRITE_REG:
//         // field1 = reg, field2 = AND mask, field3 = OR mask
//         //  Result = (Current Contents AND And_Mask) OR (Or_Mask AND (NOT And_Mask))
//         field3 = (uint16_t)frame[5] << 8 | (uint16_t)frame[6];
//         ex = _onRequest(fcode, {HREG(field1), field2, field3});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         field4 = Reg(HREG(field1));
//         field4 = (field4 & field2) | (field3 & ~field2);
//         if (!Reg(HREG(field1), field4))
//         { // Check ModbusRTU::Address and execute (reg exists?)
//             exceptionResponse(fcode, EX_ILLEGAL_ADDRESS);
//             return;
//         }
//         if (Reg(HREG(field1)) != field4)
//         { // Check for failure
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         _reply = REPLY_ECHO;
//         _onRequestSuccess(fcode, {HREG(field1), field2, field3});
//         break;
//     case FC_READWRITE_REGS:
//         // field1 = readreg, field2 = read count, frame[9] = data lenght, header len = 10
//         // field3 = wtitereg, field4 = write count
//         field3 = (uint16_t)frame[5] << 8 | (uint16_t)frame[6];
//         field4 = (uint16_t)frame[7] << 8 | (uint16_t)frame[8];
//         ex = _onRequest(fcode, {HREG(field1), field2, HREG(field3), field4});
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         if (field2 < 0x0001 || field2 > MODBUS_MAX_WORDS ||
//             field4 < 0x0001 || field4 > MODBUS_MAX_WORDS ||
//             0xFFFF - field1 < field2 || 0xFFFF - field1 < field2 ||
//             frame[9] != 2 * field4)
//         { // Check value
//             exceptionResponse(fcode, EX_ILLEGAL_VALUE);
//             return;
//         }
//         if (!setMultipleWords((uint16_t *)(frame + 10), HREG(field3), field4))
//         {
//             exceptionResponse(fcode, EX_SLAVE_FAILURE);
//             return;
//         }
//         ex = readWords(HREG(field1), field2, fcode);
//         if (ex != EX_SUCCESS)
//         {
//             exceptionResponse(fcode, ex);
//             return;
//         }
//         _onRequestSuccess(fcode, {HREG(field1), field2, HREG(field3), field4});
//         break;

//     default:
//         exceptionResponse(fcode, EX_ILLEGAL_FUNCTION);
//         return;
//     }
// }


// uint16_t ModbusRTU::callback(ModbusRTU::Register *reg, uint16_t val, TCallback::CallbackType t)
// {
// #define MODBUS_COMPARE_CB [reg, t](const TCallback &cb) { return cb.address == reg->address && cb.type == t; }
//     uint16_t newVal = val;
// #if defined(MODBUS_USE_STL)
//     std::vector<TCallback>::iterator it = _callbacks.begin();
//     do
//     {
//         it = std::find_if(it, _callbacks.end(), MODBUS_COMPARE_CB);
//         if (it != _callbacks.end())
//         {
//             //newVal = it->cb(reg, newVal);
//             it++;
//         }
//     } while (it != _callbacks.end());
//     return newVal;
// }

// ModbusRTU::Register *ModbusRTU::FindRegister(ModbusRTU::Address address)
// {
// #define MODBUS_COMPARE_REG [address](const ModbusRTU::Register &addr) { return (addr.address == address); }
//     std::vector<ModbusRTU::Register>::iterator it = std::find_if(_regs.begin(), _regs.end(), MODBUS_COMPARE_REG);
//     if (it != _regs.end())
//     {
//         return &*it;
//     }
//     return nullptr;
// }

// ModbusRTU::Register *ModbusRTU::FindRegister(uint16_t registerNumber)
// {
// #define MODBUS_COMPARE_REG [registerNumber](const ModbusRTU::Register &addr) { return (addr.registerNumber == registerNumber); }
//     std::list<std::unique_ptr<ModbusRTU::Register>>::iterator it = std::find_if(_registers.begin(), _registers.end(), MODBUS_COMPARE_REG);
//     if (it != _registers.end())
//     {
//         return (*it).get();
//     }
//     return nullptr;
// }

// ModbusRTU::Register *ModbusRTU::FindRegister(Register* reg)
// {
// #define MODBUS_COMPARE_REG [reg](const ModbusRTU::Register &addr) { return (addr.registerNumber == reg->registerNumber); }
//     std::list<std::unique_ptr<ModbusRTU::Register>>::iterator it = std::find_if(_registers.begin(), _registers.end(), MODBUS_COMPARE_REG);
//     if (it != _registers.end())
//     {
//         return (*it).get();
//     }
//     return nullptr;
// }

// bool ModbusRTU::AddRegister(Address address, uint16_t value, uint16_t numberOfRegisters)
// {
// #if defined(MODBUS_MAX_REGS)
//     if (_regs.size() + numberOfRegisters > MODBUS_MAX_REGS)
//         return false;
// #endif
//     if (0xFFFF - address.address < numberOfRegisters)
//     {
//         numberOfRegisters = 0xFFFF - address.address;
//     }
//     for (uint16_t i = 0; i < numberOfRegisters; i++)
//     {
//         // Does a register with given address already exist?
//         if (!FindRegister(address + i))
//         {
//             _regs.push_back({address + i, value});
//         }
//         return false;
//     }
//     // std::sort(_regs.begin(), _regs.end());
//     return true;
// }

// bool ModbusRTU::AddRegister(Address address, uint16_t value)
// {
//     ModbusRTU::Register *reg;
//     reg = FindRegister(address); // search for the register address
//     if (reg)
//     { // if found then assign the register value to the new value.
//         if (cbEnabled)
//         {
//             reg->value = callback(reg, value, TCallback::ON_SET);
//         }
//         else
//         {
//             reg->value = value;
//         }
//         return true;
//     }
//     else
//         return false;
// }

// bool ModbusRTU::AddRegister(ModbusRTU::Address address, uint16_t *value, uint16_t numberOfRegisters)
// {
//     if (0xFFFF - address.address < numberOfRegisters)
//     {
//         numberOfRegisters = 0xFFFF - address.address;
//     }
//     for (uint16_t k = 0; k < numberOfRegisters; k++)
//     {
//         // AddRegister(address + k, value[k]);
//     }
//     return true;
// }

// uint16_t ModbusRTU::Reg(ModbusRTU::Address address)
// {
//     ModbusRTU::Register *reg;
//     reg = FindRegister(address);
//     if (reg)
//         if (cbEnabled)
//         {
//             return callback(reg, reg->value, TCallback::ON_GET);
//         }
//         else
//         {
//             return reg->value;
//         }
//     else
//         return 0;
// }

// bool ModbusRTU::Reg(Address address, uint16_t value)
// {
//     ModbusRTU::Register* reg;
//     reg = FindRegister(address); //search for the register address
//     if (reg) { //if found then assign the register value to the new value.
//         if (cbEnabled) {
//             reg->value = callback(reg, value, TCallback::ON_SET);
//         } else {
//             reg->value = value;
//         }
//         return true;
//     } else
//         return false;
// }

// bool ModbusRTU::removeReg(ModbusRTU::Address address, uint16_t numregs)
// {
//     ModbusRTU::Register *reg;
//     bool atLeastOne = false;
//     if (0xFFFF - address.address < numregs)
//         numregs = 0xFFFF - address.address;
//     for (uint16_t i = 0; i < numregs; i++)
//     {
//         reg = FindRegister(address + i);
//         if (reg)
//         {
//             atLeastOne = true;
//             removeOnSet(address + i);
//             removeOnGet(address + i);
// #if defined(MODBUS_USE_STL)
//             _regs.erase(std::remove(_regs.begin(), _regs.end(), *reg), _regs.end());
// #else
//             _regs.remove(_regs.find(MODBUS_COMPARE_REG));
// #endif
//         }
//     }
//     return atLeastOne;
// }