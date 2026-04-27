// Copyright 2025-2026 Andreas Reichle (HOREICH GmbH)

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ModbusRTUMaster.h"

#ifdef MBED_CONF_MODBUS_RTU
#define TRACE_GROUP "MODB"
#define dbg_info(...)      tr_info(__VA_ARGS__)
#define dbg_debug(...)     tr_debug(__VA_ARGS__)
#define dbg_warn(...)      tr_warn(__VA_ARGS__)
#define dbg_error(...)     tr_error(__VA_ARGS__)
#else
#define dbg_info(...)
#define dbg_debug(...)
#define dbg_warn(...)
#define dbg_error(...)
#endif

#define bitWrite(value, bit, bitvalue) ((bitvalue) ? set_bit(value, bit) : clear_bit(value, bit))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

using namespace mbed;
 
ModbusRTUMaster::ModbusRTUMaster(
	BufferedRS485 &rs485) :
	ModbusRTU(rs485),
	_response_timeout(),
	_last_exception(Exception::None)
{
	// Disable all serial inputs/ outputs by default
	_rs485.disable_tx();
	_rs485.disable_rx();
}

ModbusRTUMaster::~ModbusRTUMaster()
{
}

ssize_t ModbusRTUMaster::read_coils(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 1;
	uint8_t byte_count = div8RndUp(register_size);
	if (slave_id < 1 || slave_id > 247 || !registers || register_size == 0 || register_size > 2000)
	{
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Build request
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);

	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	// Wait for response
	ssize_t response_length = read_response(slave_id, function_code);
	if (response_length != (uint16_t)(3 + byte_count) || _buffer[2] != byte_count)
	{
		return -1;
	}
	for (uint16_t i = 0; i < register_size; i++)
	{
		registers[i] = bitRead(_buffer[3 + (i >> 3)], i & 7);
	}
	return rc;
}

ssize_t ModbusRTUMaster::read_discrete_inputs(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 2;
	uint8_t byte_count = div8RndUp(register_size);
	if (slave_id < 1 || slave_id > 247 || !registers || register_size == 0 || register_size > 2000)
	{
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Build request
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);

	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	// Wait for response
	ssize_t response_length = read_response(slave_id, function_code);
	if (response_length != (uint16_t)(3 + byte_count) || _buffer[2] != byte_count)
	{
		return -1;
	}
	for (uint16_t i = 0; i < register_size; i++)
	{
		registers[i] = bitRead(_buffer[3 + (i >> 3)], i & 7);
	}
	return rc;
}

ssize_t ModbusRTUMaster::read_holding_register(uint8_t slave_id, uint16_t start_address, uint16_t* registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 3;
	uint8_t byte_count = register_size * 2;
	if (slave_id < 1 || slave_id > 247 || !registers || register_size == 0 || register_size > 125)
	{
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Build request
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);
	
	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	// Await response
	ssize_t response_length = read_response(slave_id, function_code);
	if (response_length != (uint16_t)(3 + byte_count) || _buffer[2] != byte_count)
	{
		return -1;
	}	
	for (uint16_t i = 0; i < register_size; i++)
	{
		registers[i] = bytesToWord(_buffer[3 + (i * 2)], _buffer[4 + (i * 2)]);
	}
	return rc;
}

ssize_t ModbusRTUMaster::read_input_registers(uint8_t slave_id, uint16_t start_address, uint16_t* registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 4;
	uint8_t byte_count = register_size * 2;
	if (slave_id < 1 || slave_id > 247 || registers == nullptr || register_size == 0 || register_size > 125)
	{
		dbg_error("Invalid input");
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Fill frame
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);

	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	// Await response
	ssize_t response_length = read_response(slave_id, function_code);
	if (response_length < 0)
	{
		dbg_error("Error while reading response (%d)", response_length);
		return -1;
	}
	if (response_length != (uint16_t)(3 + byte_count) || _buffer[2] != byte_count)
	{
		return -1;
	}
	for (uint16_t i = 0; i < register_size; i++)
	{
		registers[start_address + i] = bytesToWord(_buffer[3 + (i * 2)], _buffer[4 + (i * 2)]);
	}

	return rc;
}

ssize_t ModbusRTUMaster::write_single_coil(uint8_t slave_id, uint16_t address, bool value)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 5;
	if (slave_id > 247)
	{
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Fill frame
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(address);
	_buffer[3] = lowByte(address);
	_buffer[4] = value * 255;
	_buffer[5] = 0;

	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	if (slave_id == 0) // broadcast
	{
		return rc;
	}

	// Await response
	uint16_t response_length = read_response(slave_id, function_code);
	if (response_length != 6 || bytesToWord(_buffer[2], _buffer[3]) != address || _buffer[4] != (value * 255) || _buffer[5] != 0)
	{
		return -1;
	}
	return rc;
}

ssize_t ModbusRTUMaster::write_single_holding_register(uint8_t slave_id, uint16_t address, uint16_t value)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	// TODO: check id range 0 - 247

	const uint8_t function_code = 6;

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Fill frame
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(address); // TODO:
	_buffer[3] = lowByte(address);
	_buffer[4] = highByte(value);
	_buffer[5] = lowByte(value);

	// Write to serial bus
	ssize_t rc = write_request(6);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	if (slave_id == 0) // broadcast
	{
		return rc;
	}

	// Await response
	uint16_t response_length = read_response(slave_id, function_code);
	if (response_length != 6 || bytesToWord(_buffer[2], _buffer[3]) != address || bytesToWord(_buffer[4], _buffer[5]) != value)
	{
		return -1;
	}	
	return rc;
}

ssize_t ModbusRTUMaster::write_multiple_coils(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 15;
	uint8_t byte_count = div8RndUp(register_size);
	if (!registers || register_size == 0 || register_size > 1968)
	{
		return -1;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Fill frame
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);
	_buffer[6] = byte_count;
	for (uint16_t i = 0; i < register_size; i++)
	{
		bitWrite(_buffer[7 + (i >> 3)], i & 7, registers[i]);
	}
	for (uint16_t i = register_size; i < (byte_count * 8); i++)
	{
		clear_bit(_buffer[7 + (i >> 3)], i & 7);
	}

	// Write to serial bus
	ssize_t rc = write_request(7 + byte_count);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}

	if (slave_id == 0) // broadcast
	{
		return rc;
	}

	// Await response
	uint16_t response_length = read_response(slave_id, function_code);
	if (response_length != 6 || bytesToWord(_buffer[2], _buffer[3]) != start_address || bytesToWord(_buffer[4], _buffer[5]) != register_size)
	{
		return -1;
	}
	return rc;
}

ssize_t ModbusRTUMaster::write_multiple_holding_registers(uint8_t slave_id, uint16_t start_address, uint16_t* registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	const uint8_t function_code = 16;
	uint8_t byte_count = register_size * 2;
	if (slave_id > 247 || !registers || register_size == 0 || register_size > 123)
	{
		return false;
	}

	ScopedLock<rtos::Mutex> lock(_dataMutex);

	// Fill frame
	_buffer[0] = slave_id;
	_buffer[1] = function_code;
	_buffer[2] = highByte(start_address);
	_buffer[3] = lowByte(start_address);
	_buffer[4] = highByte(register_size);
	_buffer[5] = lowByte(register_size);
	_buffer[6] = byte_count;
	for (uint16_t i = 0; i < register_size; i++)
	{
		_buffer[7 + (i * 2)] = highByte(registers[i]);
		_buffer[8 + (i * 2)] = lowByte(registers[i]);
	}

	// Write to serial bus
	ssize_t rc = write_request(7 + byte_count);
	if (rc < 0)
	{
		dbg_error("Error while writing to serial bus");
		return -1;
	}
	
	if (start_address == 0) // broadcast
	{
		dbg_debug("Master in broadcasting mode - do not expect response");
		return rc;
	}

	uint16_t response_length = read_response(slave_id, function_code);
	if (response_length != 6 || 
		bytesToWord(_buffer[2], _buffer[3]) != start_address || 
		bytesToWord(_buffer[4], _buffer[5]) != register_size)
	{
		return -1;
	}	
	return rc;
}

ssize_t ModbusRTUMaster::write_request(uint8_t len)
{
    dbg_debug("ModbusRTUMaster::%s", __func__);

	uint16_t crc = calculate_crc(len);
	_buffer[len] = lowByte(crc);
	_buffer[len + 1] = highByte(crc);

	dbg_debug("Write buffer contains %d bytes: ", len + 2);
    for (ssize_t i = 0; i < len + 2; ++i)
    {
        dbg_debug("0x%02X", _buffer[i]);
    }

	return ModbusRTU::write(_buffer, len + 2);
}

ssize_t ModbusRTUMaster::read_response(uint8_t slave_id, uint8_t function_code)
{
    // dbg_debug("ModbusRTUMaster::%s", __func__);

	_rs485.enable_rx();
	ssize_t rc = read(_buffer, sizeof(_buffer), false);
	_rs485.disable_rx();
	if (rc < 0)
    {
        dbg_error("Error while reading data from serial (%d)", rc);
        return rc;
    }
    else if (rc == 0)
    {
        dbg_info("No data read from serial");
        return rc;
    }

	// Print received data
	dbg_debug("Response buffer (%d bytes) contains: ", rc);
	for (ssize_t i = 0; i < rc; i++)
	{
		dbg_debug("0x%02X", _buffer[i]);
	}

	// byte 0: slave id
	// byte 1: FC
	// byte 2: num of registers
	// byte 3 - n-2: register values
	// byte n-1: CRC MSB
	// byte n: CRC LSB

	// Exception
	// byte 2: exception code

	// Check frame
	if (_buffer[0] != slave_id || (_buffer[1] != function_code && _buffer[1] != (function_code + 128)) || calculate_crc(rc - 2) != bytesToWord(_buffer[rc - 1], _buffer[rc - 2]))
	{
		dbg_error("Unexpected response format");
		return -1;
	}	
	else if (_buffer[1] == (function_code + 0x80))
	{
		_last_exception = static_cast<Exception>(_buffer[2]);
		dbg_error("Exception response: %d", (uint8_t)_last_exception);
		return -1;
	}

	return (rc - 2); // data without crc
}

// bool ModbusRTUMaster::getTimeoutFlag()
// {
// 	return _timeoutFlag;
// }

// void ModbusRTUMaster::clearTimeoutFlag()
// {
// 	_timeoutFlag = 0;
// }

// void ModbusRTUMaster::clearwrite_exception_response()
// {
// 	write_exception_response = 0;
// }