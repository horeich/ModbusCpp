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

#ifndef MODBUS_HPP
#define MODBUS_HPP

// #define ENABLE_MODBUS_LOG
#ifdef ENABLE_MODBUS_LOG
#define log_info(...)       tr_info(__VA_ARGS__)  
#define log_warn(...)       tr_warn(__VA_ARGS__) 
#define log_debug(...)      tr_debug(__VA_ARGS__)
#else
#define log_info(...)       
#define log_warn(...)       
#define log_debug(...)      
#endif // ENABLE_MODBUS_LOG 

#include "ModbusSettings.h"
#include <vector>
#include <list>
#include <algorithm>
#include <functional>
#include <memory>
#include <chrono>
#include "mbed_trace.h"
#include <map>

static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
    0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
    0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
    0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
    0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
    0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
    0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
    0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
    0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
    0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
    0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
    0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
    0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
    0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
    0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
    0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
    0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

#define MODBUS_RTU_MAX_MESSAGE_SIZE     255  // see specification
#define MODBUS_MINIMUM_FRAME_SIZE       8    
#define MODBUS_MAX_WORDS                0x007D // 125

#define MODBUS_USE_STL

static inline uint16_t __swap_16(uint16_t num) { return (num >> 8) | (num << 8); }

using ModbusAddress = uint16_t;

namespace mbed
{   
    class ModbusRTU
    {
    public:

        enum STATE
        {
            STATE_RECEIVE_AND_SEND  = (1 << 0),
            STATE_SUSPEND           = (1 << 1),
        };

        struct Register
        {
            enum Type
            {
                COIL,
                ISTS,
                IREG,
                HREG
            };

            Register(Type t, uint16_t v) :
                type(t),
                value(v)
            {

            }

            Type type;
            uint16_t value; // 16 bit wide
            
            bool isCoil() const { return (type == COIL); }
            bool isIsts() const { return (type == ISTS); }
            bool isIreg() const { return (type == IREG); }
            bool isHreg() const { return (type == HREG); }
        };

        enum FunctionCode
        {
            FC_READ_COILS = 0x01,      // Read Coils (Output) Status
            FC_READ_INPUT_STAT = 0x02, // Read Input Status (Discrete Inputs)
            FC_READ_HOLDING_REGS = 0x03,       // Read Holding Registers
            FC_READ_INPUT_REGS = 0x04, // Read Input Registers
            FC_WRITE_SINGLE_COIL = 0x05,      // Write Single Coil (Output)
            FC_WRITE_SINGLE_REG = 0x06,       // Preset Single Register
            FC_DIAGNOSTICS = 0x08,     // Not implemented. Diagnostics (Serial Line only)
            FC_WRITE_COILS = 0x0F,     // Write Multiple Coils (Outputs)
            FC_WRITE_MULTIPLE_REGS = 0x10,      // Write block of contiguous registers
            FC_READ_FILE_REC = 0x14,   // Read File Record
            FC_WRITE_FILE_REC = 0x15,  // Write File Record
            FC_MASKWRITE_REG = 0x16,   // Mask Write Register
            FC_READWRITE_REGS = 0x17   // Read/Write Multiple registers

        };

        // Exception Codes
        // Custom result codes used internally and for callbacks but never used for Modbus responce
        enum ExceptionCode
        {
            EX_NONE = 0x00,                  // Custom. No error
            EX_ILLEGAL_FUNCTION = 0x01,         // Function Code not Supported
            EX_ILLEGAL_DATA_ADDRESS = 0x02,          // Output Address not exists
            EX_ILLEGAL_VALUE = 0x03,            // Output Value not in Range
            EX_SLAVE_FAILURE = 0x04,            // Slave or Master Device Fails to process request
            EX_ACKNOWLEDGE = 0x05,              // Not used
            EX_SLAVE_DEVICE_BUSY = 0x06,        // Not used
            EX_MEMORY_PARITY_ERROR = 0x08,      // Not used
            EX_PATH_UNAVAILABLE = 0x0A,         // Not used
            EX_DEVICE_FAILED_TO_RESPOND = 0x0B, // Not used
            EX_GENERAL_FAILURE = 0xE1,          // Custom. Unexpected master error
            EX_DATA_MISMACH = 0xE2,             // Custom. Inpud data size mismach
            EX_UNEXPECTED_RESPONSE = 0xE3,      // Custom. Returned result doesn't mach transaction
            EX_TIMEOUT = 0xE4,                  // Custom. Operation not finished within reasonable time
            EX_CONNECTION_LOST = 0xE5,          // Custom. Connection with device lost
            EX_CANCEL = 0xE6,                   // Custom. Transaction/request canceled
            EX_PASSTHROUGH = 0xE7,              // Custom. Raw callback. Indicate to normal processing on callback exit
            EX_FORCE_PROCESS = 0xE8             // Custom. Raw callback. Indicate to force processing on callback exit
        };

        // Reply Types
        // enum ReplyCode
        // {
        //     REPLY_OFF = 0x01,
        //     REPLY_ECHO = 0x02,
        //     REPLY_NORMAL = 0x03,
        //     REPLY_ERROR = 0x04,
        //     REPLY_UNEXPECTED = 0x05
        // };

    public:
        explicit ModbusRTU(ModbusAddress address);
        virtual ~ModbusRTU();
        bool AddRegister(Register::Type type, uint16_t startRegister, uint8_t numOfRegisters);

    protected:
        void CreateException(FunctionCode functionCode, ExceptionCode ExceptionCode);
        uint16_t CRC16(uint8_t* buffer, uint16_t buffer_length);
        void Send();
        bool IsFunctionCodeValid(FunctionCode code);
        ExceptionCode ProcessPDU(uint8_t* framePtr, uint8_t frameSize);
        virtual bool ProcessBuffer(uint8_t*, uint32_t, uint8_t*, uint8_t&) = 0; // makes this class abstract

    private:
        void Sigio();
        void Run();

    protected:
        uint8_t _address;
        uint8_t* _respBuffer;// [MODBUS_RTU_MAX_MESSAGE_SIZE] = {0};
        uint8_t _respBufferSize;
        std::map<ModbusAddress, std::unique_ptr<ModbusRTU::Register>> _registers;
        // rtos::Mutex _mutex;
    private:
        // USBStream *_serial;
        // rtos::Thread _thread;
        // rtos::EventFlags _state;
    };

} // namespace mbed

#endif // MODBUS_HPP


//         bool readSlaveFile(uint16_t *fileNum, uint16_t *startRec, uint16_t *len, uint8_t count, FunctionCode fn);
//         // fileNum - sequental array of files numbers to read
//         // startRec - array of strart records for each file
//         // len - array of counts of records to read in terms of register size (2 bytes) for each file
//         // count - count of records to be compose in the single request
//         // fn - Modbus function. Assumed to be 0x14
//         bool writeSlaveFile(uint16_t *fileNum, uint16_t *startRec, uint16_t *len, uint8_t count, FunctionCode fn, uint8_t *data);
//         // fileNum - sequental array of files numbers to read
//         // startRec - array of strart records for each file
//         // len - array of counts of records to read in terms of register size (2 bytes) for each file
//         // count - count of records to be compose in the single request
//         // fn - Modbus function. Assumed to be 0x15
//         // data - sequental set of data records


// #if defined(MODBUS_USE_STL)
// typedef std::function<bool(Modbus::ExceptionCode, uint16_t, void *)> cbTransaction; // Callback skeleton for requests
// #else
// typedef bool (*cbTransaction)(Modbus::ExceptionCode event, uint16_t transactionId, void *data); // Callback skeleton for requests
// #endif
// // typedef Modbus::ExceptionCode (*cbRequest)(Modbus::FunctionCode func, Register* reg, uint16_t regCount); // Callback function Type
// #if defined(MODBUS_FILES)
// // Callback skeleton for file read/write
// #if defined(MODBUS_USE_STL)
// typedef std::function<Modbus::ExceptionCode(Modbus::FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)> cbModbusFileOp;
// #else
// typedef Modbus::ExceptionCode (*cbModbusFileOp)(Modbus::FunctionCode func, uint16_t fileNum, uint16_t recNumber, uint16_t recLength, uint8_t *frame);
// #endif
// #endif

// #if defined(ARDUINO_SAM_DUE_STL)
// // Arduino Due STL workaround
// namespace std
// {
//     void __throw_bad_function_call();
// }
// #endif

   //  protected:
//         bool cbEnabled = true;
//         uint16_t callback(Register *reg, uint16_t val, TCallback::CallbackType t);

//         // Finds returns register if found, nullptr otherwise
//         virtual Register *FindRegister(Address addr);
//         virtual Register *FindRegister(uint16_t addr);
//         virtual Register *FindRegister(Register* reg);

//         void exceptionResponse(FunctionCode fn, ExceptionCode excode);                                        // Fills _frame with response
//         void successResponce(Address startreg, uint16_t numoutputs, FunctionCode fn);                      // Fills frame with response
//         void DecodePDU(uint8_t *frame);                                                                    // For Slave
//         void masterPDU(uint8_t *frame, uint8_t *sourceFrame, Address startreg, uint8_t *output = nullptr); // For Master
//         // frame - data received form slave
//         // sourceFrame - data have sent fo slave
//         // startreg - local register to start put data to
//         // output - if not null put data to the buffer insted local registers. output assumed to by array of uint16_t or boolean

//         bool readSlave(uint16_t address, uint16_t numregs, FunctionCode fn);
//         bool writeSlaveBits(Address startreg, uint16_t to, uint16_t numregs, FunctionCode fn, bool *data = nullptr);
//         bool writeSlaveWords(Address startreg, uint16_t to, uint16_t numregs, FunctionCode fn, uint16_t *data = nullptr);
//         // startreg - local register to get data from
//         // to - slave register to write data to
//         // numregs - number of registers
//         // fn - Modbus function
//         // data - if null use local registers. Otherwise use data from array to erite to slave
//         bool removeOn(TCallback::CallbackType t, Address address, cbModbus cb = nullptr, uint16_t numregs = 1);

//     public:
//         // Add single 16 bit value
//         bool AddRegister(Address address, uint16_t value = 0, uint16_t numberOfRegisters = 1);

//         // Add range of 16 bit values
//         bool AddRegister(Address address, uint16_t *value, uint16_t numberOfRegisters = 1);

//     private:
//         bool AddRegister(Address address, uint16_t value);

//     public:
//         bool Reg(Address address, uint16_t value);
//         uint16_t Reg(Address address);
//         bool removeReg(Address address, uint16_t numregs = 1);
//         bool Reg(Address address, uint16_t *value, uint16_t numregs = 1);

//         bool onGet(Address address, cbModbus cb = nullptr, uint16_t numregs = 1);
//         bool onSet(Address address, cbModbus cb = nullptr, uint16_t numregs = 1);
//         bool removeOnSet(Address address, cbModbus cb = nullptr, uint16_t numregs = 1);
//         bool removeOnGet(Address address, cbModbus cb = nullptr, uint16_t numregs = 1);

//         virtual uint32_t eventSource() { return 0; }
// #if defined(MODBUS_USE_STL)
//         typedef std::function<ExceptionCode(FunctionCode, const RequestData)> cbRequest; // Callback function Type
//         typedef std::function<ExceptionCode(uint8_t *, uint8_t, void *)> cbRaw;          // Callback function Type
// #else
//         typedef ExceptionCode (*cbRequest)(FunctionCode fc, const RequestData data); // Callback function Type
//         typedef ExceptionCode (*cbRaw)(uint8_t *, uint8_t, void *);                  // Callback function Type
// #endif

//     protected:
//         cbRaw _cbRaw = nullptr;
//         static ExceptionCode _onRequestDefault(FunctionCode fc, const RequestData data);
//         cbRequest _onRequest = _onRequestDefault;

//     public:
//         bool onRaw(cbRaw cb = nullptr);
//         bool onRequest(cbRequest cb = _onRequestDefault);
// #if defined(MODBUSAPI_OPTIONAL)
//     protected:
//         cbRequest _onRequestSuccess = _onRequestDefault;

//     public:
//         bool onRequestSuccess(cbRequest cb = _onRequestDefault);
// #endif

// #if defined(MODBUS_FILES)
//     public:
//         bool onFile(std::function<ExceptionCode(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)>);
// #else
//         bool onFile(ExceptionCode (*cb)(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *));
// #endif
//     private:
//         ExceptionCode fileOp(FunctionCode fc, uint16_t fileNum, uint16_t recNum, uint16_t recLen, uint8_t *frame);

//     protected:



        // ExceptionCode readBits(Address startreg, uint16_t numregs, FunctionCode fn);
        // ExceptionCode readWords(Address startreg, uint16_t numregs, FunctionCode fn);

        // bool setMultipleBits(uint8_t *frame, Address startreg, uint16_t numoutputs);
        // bool setMultipleWords(uint16_t *frame, Address startreg, uint16_t numoutputs);

        // void getMultipleBits(uint8_t *frame, Address startreg, uint16_t numregs);
        // void getMultipleWords(uint16_t *frame, Address startreg, uint16_t numregs);

        // void bitsToBool(bool *dst, uint8_t *src, uint16_t numregs);
        // void boolToBits(uint8_t *dst, bool *src, uint16_t numregs);

        // void bitSet(uint8_t &value, uint8_t bitn);
        // void bitClear(uint8_t &value, uint8_t bitn);
        // uint8_t bitRead(uint8_t &value, uint8_t bitn);

       // uint32_t _len = 0;
        // uint8_t *_frame = nullptr;
        // uint8_t _reply = 0;

// struct Register;
// #if defined(MODBUS_USE_STL)
// using cbModbus = std::function<uint16_t(Register *reg, uint16_t val)>; // Callback function Type
// #else
// typedef uint16_t (*cbModbus)(Register *reg, uint16_t val); // Callback function Type
// #endif

// #define COIL(n) 
//     (Address) { Address::COIL, n }
// #define ISTS(n) 
//     (Address) { Address::ISTS, n }
// #define IREG(n) 
//     (Address) { Address::IREG, n }
// #define HREG(n) 
//     (Address) { Address::HREG, n }
// #define BIT_VAL(v) (v ? 0xFF00 : 0x0000)
// #define BIT_BOOL(v) (v == 0xFF00)
// #define COIL_VAL(v) (v ? 0xFF00 : 0x0000)
// #define COIL_BOOL(v) (v == 0xFF00)
// #define ISTS_VAL(v) (v ? 0xFF00 : 0x0000)
// #define ISTS_BOOL(v) (v == 0xFF00)

// // For depricated (v1.xx) onSet/onGet format compatibility
// #define cbDefault nullptr

// #if defined(MODBUS_USE_STL)
// #if defined(MODBUS_GLOBAL_REGS)
//         static std::vector<Register> _regs;
//         static std::vector<TCallback> _callbacks;
// #if defined(MODBUS_FILES)
//         static std::function<ExceptionCode(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)> _onFile;
// #endif
// #else
//         std::vector<Register> _regs;
//         std::vector<TCallback> _callbacks;
// #if defined(MODBUS_FILES)
//         std::function<ExceptionCode(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *)> _onFile;
// #endif
// #endif
// #else
// #if defined(MODBUS_FILES)
//         static ExceptionCode (*_onFile)(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *);
// #endif
// #if defined(MODBUS_FILES)
//         ExceptionCode (*_onFile)(FunctionCode, uint16_t, uint16_t, uint16_t, uint8_t *) = nullptr;
// #endif
// #endif