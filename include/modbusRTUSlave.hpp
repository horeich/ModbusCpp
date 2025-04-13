// Copyright 2025 Andreas Reichle, Fawad Siddiqui (HOREICH GmbH)

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ModbusRTUSlave_h
#define ModbusRTUSlave_h

#include "rs485.hpp"

#define MODBUS_RTU_SLAVE_BUF_SIZE 256
#define NO_DE_PIN 255
#define NO_ID 0

class ModbusRTUSlave
{
public:
  ModbusRTUSlave(RS485 &serial, uint8_t id);
  void configureCoils(bool coils[], uint16_t numCoils);
  void configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs);
  void configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters);
  void configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters);
  void poll();

private:
  RS485 *_hardwareSerial = 0;
  uint8_t _buf[MODBUS_RTU_SLAVE_BUF_SIZE];
  bool *_coils = 0;
  bool *_discreteInputs = 0;
  uint16_t *_holdingRegisters = 0;
  uint16_t *_inputRegisters = 0;
  uint16_t _numCoils = 0;
  uint16_t _numDiscreteInputs = 0;
  uint16_t _numHoldingRegisters = 0;
  uint16_t _numInputRegisters = 0;
  uint8_t _id = NO_ID;
  unsigned long _charTimeout;
  unsigned long _frameTimeout;

  void _processReadCoils();
  void _processReadDiscreteInputs();
  void _processReadHoldingRegisters();
  void _processReadInputRegisters();
  void _processWriteSingleCoil();
  void _processWriteSingleHoldingRegister();
  void _processWriteMultipleCoils();
  void _processWriteMultipleHoldingRegisters();

  int _readRequest();
  void _writeResponse(uint8_t len);
  void _exceptionResponse(uint8_t code);
  uint16_t _crc(uint8_t len);
  uint16_t _div8RndUp(uint16_t value);
  uint16_t _bytesToWord(uint8_t high, uint8_t low);
  void bitSet(uint8_t &value, uint8_t bit);
  void bitClear(uint8_t &value, uint8_t bit);
};

#endif