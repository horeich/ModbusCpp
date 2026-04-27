// Copyright 2025 Andreas Reichle (HOREICH GmbH)

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// TODO: make virtual = 0
// TODO: optional std::span support if c++ 20

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#define MBED_CONF_MODBUS_RTU

#include "BufferedRS485.h"
#include "Callback.h"
#ifdef MBED_CONF_MODBUS_RTU
#include "mbed_trace.h"
#endif
#include "Timer.h"
#include "events/Event.h"

// #define TRACE_GROUP "TEST" //remove later

// Coils: 				Generally 0-9999 in older systems, but can be up to 65,535.
// Discrete Inputs: 	Typically 10001-19999 in older systems, but can be up to 65,535.
// Input Registers: 	Usually 30001-39999 in older systems, up to 65,535.
// Holding Registers: 	Standard range 40001-49999, up to 65,535.

// Type				 |	Access	    |   Typical Range	|	Function Codes															|	Typical Use
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Coils			 |	Read/Write	|	00001–09999		|	01 (Read Coils), 05 (Write Single Coil), 15 (Write Multiple Coils)		|	Digital outputs (bits)
// Discrete Inputs	 |	Read Only	|	10001–19999		|	02 (Read Discrete Inputs)												|	Digital inputs (bits)
// Holding Registers |	Read/Write	|	40001–49999		|	03 (Read Holding Registers), 06 (Write Single), 16 (Write Multiple)		|	Analog outputs / internal variables
// Input Registers	 |	Read Only	|	30001–39999		|	04 (Read Input Registers)												|	Analog inputs (sensor values)

// Note that broadcast is only supported for write commands

namespace mbed
{
	class ModbusRTU
	{
	public:
		enum class Exception : uint8_t
		{
			None								= 0x00,
			IllegalFunction 					= 0x01, // function code not supported by the slave
			IllegalDataAddress 					= 0x02, // address is not valid for that function
			IllegalDataValue 					= 0x03, // value in request is invalid (e.g. out of range)
			SlaveDeviceFailure 					= 0x04, // unrecoverable error while attempting to perform the action.
			Acknowledge							= 0x05, // request accepted, processing will take long; master should retry later
			SlaveDeviceBusy						= 0x06, // slave is engaged in a long operation, retry later
			NegativeAcknowledge					= 0x07, // slave cannot perform the function
			MemoryParityError					= 0x08, // memory parity error detected
			GatewayPathUnavailable				= 0x0A, // gateway cannot route the request
			GatewayTargetFailedResponse			= 0x0B, // no response from target device behind a gateway
		};

		enum class FunctionCode : uint8_t
		{
			None								= 0x00,
			ReadCoils							= 0x01,
			ReadDiscreteInputs					= 0x02,
			ReadHoldingRegisters				= 0x03,
			ReadInputRegisters					= 0x04,
			WriteSingleCoil						= 0x05, 
			WriteSingleHoldingRegister			= 0x06, 
			WriteMultipleCoils					= 0x15, 
			WriteMultipleHoldingRegisters		= 0x16,
		};

		enum class TableType : uint8_t
		{
			Coils,
			Contacts,
			InputRegisters,
			HoldingRegisters,
		};
	
	public:
		explicit ModbusRTU(
			BufferedRS485 &serial,
			std::chrono::milliseconds response_timeout = std::chrono::milliseconds(MBED_CONF_MODBUS_RESPONSE_TIMEOUT_MS));

		virtual ~ModbusRTU();

		void lock();
		void unlock();

		void set_response_timeout(std::chrono::milliseconds response_timeout);

	private:
		std::chrono::microseconds char_time_us(uint32_t baudrate);
		std::chrono::microseconds t1_5(uint32_t baudrate);
		std::chrono::microseconds t3_5(uint32_t baudrate);
		
	public:

		/**
		 * These methods are helper to serialize and deserialize from/to modbus registers/ build in data types
		 */
		template<typename T>
		static ssize_t deserialize_register(T& value, const uint16_t* registers, uint16_t register_size, uint16_t offset, bool swap_words = false)
		{
			// Confirm that we only use trivially copyable data types
			MBED_ASSERT(std::is_trivially_copyable<T>::value);

			constexpr size_t byte_count = sizeof(T);
			constexpr size_t reg_count = (byte_count + 1) / 2;

			uint8_t bytes[byte_count] = {0};

			if (register_size < (offset + reg_count))
			{
				return -1;
			}

			// Optionally reverse register order (undo swap_words from serialization)
			if (swap_words && reg_count > 1) 
			{
				for (size_t i = 0; i < reg_count / 2; ++i)
				{
					// std::swap(registers[i + offset], registers[reg_count - 1 - i + offset]);
				}
			}

			// Reconstruct byte array from Modbus registers
			for (size_t i = 0; i < reg_count; ++i)
			{
				uint16_t reg_value = registers[reg_count - 1 - i + offset];
				// tr_warn("Reg value: %d", reg_value);

				// Split back into high/low bytes (big-endian per Modbus spec)
				if (i * 2 < byte_count)
				{
					bytes[i * 2 + 0] = static_cast<uint8_t>(reg_value & 0xFF); // low byte
					// tr_warn("Reg value:0x%02X", bytes[i * 2 + 0]);

				}	
				if (i * 2 + 1 < byte_count)
				{
					bytes[i * 2 + 1] = static_cast<uint8_t>((reg_value >> 8) & 0xFF); // high byte
					// tr_warn("Reg value:0x%02X", bytes[i * 2 + 1]);
				}
			}

			// Copy into value
			std::memcpy(&value, bytes, byte_count);
			// tr_warn("Reg value: %d", value);

			return reg_count;
		}

		template<typename T>
		static ssize_t serialize_register(
			const T& value, uint16_t* registers, uint16_t registers_size, uint16_t offset, bool swap_words = false)
		{
			// Confirm that we only use trivially copyable data types
			MBED_ASSERT(std::is_trivially_copyable<T>::value);

			constexpr size_t byte_count = sizeof(T);
			constexpr size_t reg_count = (byte_count + 1) / 2;

			uint8_t bytes[byte_count]; // only constant expression for static array sizes (thus constexpr above)
			
			if (registers_size < (offset + reg_count))
			{
				return -1;
			}

			std::memcpy(bytes, &value, byte_count); // little endian CPU: LSB first

			// Note that high byte and low byte basically switch positions when going from uint8_t to uint16_t
			// e.g. assume 0x11223344
			// memcpy creates the following order: bytes[0] = 0x44, bytes[1] = 0x33, bytes[2] = 0x22, bytes[3] = 0x11
			// goal: Split in two modbus registers: input_register[0] = 0x1122, input_register[1] = 0x3344
			// algorithm:	0x44 is low byte of register 1, high byte is 0x33 of register 1 > (0x33 << 8) | 0x44
			//				0x22 is low byte of register 0, high byte is 0x11 of register 0 > (0x11 << 8) | 0x33
			for (size_t i = 0; i < reg_count; ++i)
			{
				uint16_t low_byte  = (i * 2 + 0 < byte_count) ? bytes[i * 2 + 0] : 0x00;
				uint16_t high_byte = (i * 2 + 1 < byte_count) ? bytes[i * 2 + 1] : 0x00;
				registers[reg_count - 1 - i + offset] = (high_byte << 8) | low_byte;
			}

			if (swap_words && reg_count > 1) 
			{
				// Reverse register order (word swap) - untested
				for (size_t i = 0; i < reg_count / 2; ++i)
				{
					std::swap(registers[i + offset], registers[reg_count - 1 - i + offset]);
				}
			}

			return reg_count;
		}

	protected:
		ssize_t read(uint8_t *recv_buf, size_t recv_buf_len, bool async = false);
		ssize_t write(const uint8_t* write_buf, size_t write_buf_len);

		void set_bit(uint8_t &value, uint8_t bit);
		void clear_bit(uint8_t &value, uint8_t bit);
		uint16_t calculate_crc(uint8_t len);
		
		uint16_t div8RndUp(uint16_t value);
		uint16_t bytesToWord(uint8_t high, uint8_t low);

		uint8_t highByte(uint16_t x);
		uint8_t lowByte(uint16_t x);
		
#ifdef MBED_CONF_MODBUS_RTU
		static constexpr const char* GetExceptionString(Exception ex) noexcept
		{
			switch (ex)
			{
				case Exception::None : return "No exception";
				case Exception::IllegalFunction : return "IllegalFunction";
				case Exception::IllegalDataAddress : return "IllegalDataAddress";
				case Exception::IllegalDataValue : return "IllegalDataValue";
				case Exception::SlaveDeviceFailure : return "SlaveDeviceFailure";
				case Exception::Acknowledge : return "Acknowledge";
				case Exception::SlaveDeviceBusy : return "SlaveDeviceBusy";
				case Exception::NegativeAcknowledge : return "NegativeAcknowledge";
				case Exception::MemoryParityError : return "MemoryParityError";
				case Exception::GatewayPathUnavailable : return "GatewayPathUnavailable";
				case Exception::GatewayTargetFailedResponse : return "GatewayTargetFailedResponse";

				default : return "unknown exception";
			}
		}

		static constexpr const char* GetFCString(FunctionCode code) noexcept
		{
			switch (code)
			{
			    case FunctionCode::ReadCoils : return "ReadCoils";
			    case FunctionCode::ReadDiscreteInputs : return "ReadDiscreteInputs";
			    case FunctionCode::ReadHoldingRegisters : return "ReadHoldingRegisters";
			    case FunctionCode::ReadInputRegisters : return "ReadInputRegisters";
			    case FunctionCode::WriteSingleCoil : return "WriteSingleCoil";
			    case FunctionCode::WriteSingleHoldingRegister : return "WriteSingleHoldingRegister";
			    case FunctionCode::WriteMultipleCoils : return "WriteMultipleCoils";
			    case FunctionCode::WriteMultipleHoldingRegisters : return "WriteMultipleHoldingRegisters";
			    default : return "unknown";
			}
		}	
#endif // MBED_CONF_MODBUS_RTU

	protected:
		BufferedRS485 &_rs485;
		uint8_t _buffer[MBED_CONF_MODBUS_BUFFER_SIZE]; // The maximum size of Modbus messages won't exceed 255 bytes
		rtos::Mutex _dataMutex;

	private:
		std::chrono::microseconds _response_timeout;
	};

} // namespace mbed

#endif // MODBUS_RTU_H