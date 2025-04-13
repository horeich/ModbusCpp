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

#include "ModbusRTUSlave.hpp"

#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

ModbusRTUSlave::ModbusRTUSlave(RS485 &serial, uint8_t id)
{
  _hardwareSerial = &serial;
  if (id >= 1 && id <= 247)
    _id = id;
  else
    _id = NO_ID;
}


void ModbusRTUSlave::bitSet(uint8_t &value, uint8_t bit)
{
  value |= (1 << bit);
}

void ModbusRTUSlave::bitClear(uint8_t &value, uint8_t bit)
{
  value &= ~(1 << bit);
}


void ModbusRTUSlave::configureCoils(bool coils[], uint16_t numCoils)
{
  _coils = coils;
  _numCoils = numCoils;
}

void ModbusRTUSlave::configureDiscreteInputs(bool discreteInputs[], uint16_t numDiscreteInputs)
{
  _discreteInputs = discreteInputs;
  _numDiscreteInputs = numDiscreteInputs;
}

void ModbusRTUSlave::configureHoldingRegisters(uint16_t holdingRegisters[], uint16_t numHoldingRegisters)
{
  _holdingRegisters = holdingRegisters;
  _numHoldingRegisters = numHoldingRegisters;
}

void ModbusRTUSlave::configureInputRegisters(uint16_t inputRegisters[], uint16_t numInputRegisters)
{
  _inputRegisters = inputRegisters;
  _numInputRegisters = numInputRegisters;
}

uint8_t lowByte(uint16_t x)
{
  return (uint8_t)(x & 0xFF);
}

uint8_t highByte(uint16_t x)
{
  return (uint8_t)(x >> 8);
}

void ModbusRTUSlave::poll()
{
  int numOfBytes = 0;
  numOfBytes = _readRequest();
  if (numOfBytes > 1)
  {
    if (_buf[0] == _id)
    {
      printf("Received buffer is: ");
      for (int i = 0; i < numOfBytes; i++)
        printf("%d    ", _buf[i]);
      switch (_buf[1])
      {
      case 1:
        _processReadCoils();
        printf("\n FC 1 processed\n");
        break;
      case 2:
        _processReadDiscreteInputs();
        printf("\n FC 2 processed\n");
        break;
      case 3:
        _processReadHoldingRegisters();
        printf("\n FC 3 processed\n");
        break;
      case 4:
        _processReadInputRegisters();
        printf("\n FC 4 processed\n");
        break;
      case 5:
        _processWriteSingleCoil();
        printf("\n FC 5 processed\n");
        break;
      case 6:
        _processWriteSingleHoldingRegister();
        printf("\nFC 6 processed\n");
        break;
      case 15:
        _processWriteMultipleCoils();
        printf("\nFC 15 processed\n");
        break;
      case 16:
        _processWriteMultipleHoldingRegisters();
        printf("\n FC 16 processed\n");
        break;
      default:
        _exceptionResponse(1);
        break;
      }
    }
  }
}

void ModbusRTUSlave::_processReadCoils()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000)
    _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity))
    _exceptionResponse(2);
  else
  {
    _buf[2] = _div8RndUp(quantity);
    for (uint16_t i = 0; i < quantity; i++)
    {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _coils[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadDiscreteInputs()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_discreteInputs || _numDiscreteInputs == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 2000)
    _exceptionResponse(3);
  else if (quantity > _numDiscreteInputs || startAddress > (_numDiscreteInputs - quantity))
    _exceptionResponse(2);
  else
  {
    _buf[2] = _div8RndUp(quantity);
    for (uint16_t i = 0; i < quantity; i++)
    {
      bitWrite(_buf[3 + (i >> 3)], i & 7, _discreteInputs[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadHoldingRegisters()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125)
    _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity))
    _exceptionResponse(2);
  else
  {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++)
    {
      _buf[3 + (i * 2)] = highByte(_holdingRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_holdingRegisters[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processReadInputRegisters()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_inputRegisters || _numInputRegisters == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 125)
    _exceptionResponse(3);
  else if (quantity > _numInputRegisters || startAddress > (_numInputRegisters - quantity))
    _exceptionResponse(2);
  else
  {
    _buf[2] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++)
    {
      _buf[3 + (i * 2)] = highByte(_inputRegisters[startAddress + i]);
      _buf[4 + (i * 2)] = lowByte(_inputRegisters[startAddress + i]);
    }
    _writeResponse(3 + _buf[2]);
  }
}

void ModbusRTUSlave::_processWriteSingleCoil()
{
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0)
    _exceptionResponse(1);
  else if (value != 0 && value != 0xFF00)
    _exceptionResponse(3);
  else if (address >= _numCoils)
    _exceptionResponse(2);
  else
  {
    _coils[address] = value;
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteSingleHoldingRegister()
{
  uint16_t address = _bytesToWord(_buf[2], _buf[3]);
  uint16_t value = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0)
    _exceptionResponse(1);
  else if (address >= _numHoldingRegisters)
    _exceptionResponse(2);
  else
  {
    _holdingRegisters[address] = value;
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleCoils()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_coils || _numCoils == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 1968 || _buf[6] != _div8RndUp(quantity))
    _exceptionResponse(3);
  else if (quantity > _numCoils || startAddress > (_numCoils - quantity))
    _exceptionResponse(2);
  else
  {
    for (uint16_t i = 0; i < quantity; i++)
    {
      _coils[startAddress + i] = bitRead(_buf[7 + (i >> 3)], i & 7);
    }
    _writeResponse(6);
  }
}

void ModbusRTUSlave::_processWriteMultipleHoldingRegisters()
{
  uint16_t startAddress = _bytesToWord(_buf[2], _buf[3]);
  uint16_t quantity = _bytesToWord(_buf[4], _buf[5]);
  if (!_holdingRegisters || _numHoldingRegisters == 0)
    _exceptionResponse(1);
  else if (quantity == 0 || quantity > 123 || _buf[6] != (quantity * 2))
    _exceptionResponse(3);
  else if (quantity > _numHoldingRegisters || startAddress > (_numHoldingRegisters - quantity))
    _exceptionResponse(2);
  else
  {
    for (uint16_t i = 0; i < quantity; i++)
    {
      _holdingRegisters[startAddress + i] = _bytesToWord(_buf[i * 2 + 7], _buf[i * 2 + 8]);
    }
    _writeResponse(6);
  }
}

int ModbusRTUSlave::_readRequest()
{
  uint16_t numBytes = 0;
  numBytes = _hardwareSerial->ReceiveMessage(_buf, (std::chrono::milliseconds)(5000));
  return numBytes;
}

void ModbusRTUSlave::_writeResponse(uint8_t len)
{
  if (_buf[0] != 0)
  {
    uint16_t crc = _crc(len);
    _buf[len] = lowByte(crc);
    _buf[len + 1] = highByte(crc);
    _hardwareSerial->SendMessage(_buf, len + 2);
    _hardwareSerial->ReceiveMessage(_buf, (std::chrono::milliseconds)(2000));
  }
}

void ModbusRTUSlave::_exceptionResponse(uint8_t code)
{
  _buf[1] |= 0x80;
  _buf[2] = code;
  _writeResponse(3);
}

uint16_t ModbusRTUSlave::_crc(uint8_t len)
{
  uint16_t value = 0xFFFF;
  for (uint8_t i = 0; i < len; i++)
  {
    value ^= (uint16_t)_buf[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      bool lsb = value & 1;
      value >>= 1;
      if (lsb)
        value ^= 0xA001;
    }
  }
  return value;
}

uint16_t ModbusRTUSlave::_div8RndUp(uint16_t value)
{
  return (value + 7) >> 3;
}

uint16_t ModbusRTUSlave::_bytesToWord(uint8_t high, uint8_t low)
{
  return (high << 8) | low;
}