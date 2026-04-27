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

#ifndef MODBUS_RTU_MASTER_H
#define MODBUS_RTU_MASTER_H

#include <stdint.h>
#include <algorithm>

#include "mbed.h"
#include "events/Event.h"

#include "UnbufferedRS485.h"
#include "ModbusRTU.h"

namespace mbed
{
	class ModbusRTUMaster : public ModbusRTU
	{
	public:
		explicit ModbusRTUMaster(
			BufferedRS485 &serial);
		virtual ~ModbusRTUMaster();

		ssize_t read_coils(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_count);
		ssize_t read_discrete_inputs(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_count);
		ssize_t read_holding_register(uint8_t slave_id, uint16_t start_address, uint16_t *registers, uint16_t register_count);
		ssize_t read_input_registers(uint8_t slave_id, uint16_t start_address, uint16_t *registers, uint16_t register_count);
		ssize_t write_single_coil(uint8_t slave_id, uint16_t address, bool value);
		ssize_t write_single_holding_register(uint8_t slave_id, uint16_t address, uint16_t value);
		ssize_t write_multiple_coils(uint8_t slave_id, uint16_t start_address, bool *registers, uint16_t register_count);
		ssize_t write_multiple_holding_registers(uint8_t slave_id, uint16_t start_address, uint16_t *registers, uint16_t register_count);


		Exception get_last_exception() const { return _last_exception; }

		// void setTimeout(unsigned long timeout);
		// bool getTimeoutFlag();
		// void clearTimeoutFlag();
		// void clearExceptionResponse();
	private:
		ssize_t write_request(uint8_t len);
		ssize_t read_response(uint8_t slave_id, uint8_t function_code);

	private:
		std::chrono::milliseconds _response_timeout;
		unsigned long _charTimeout;
		unsigned long _frameTimeout;
		bool _timeoutFlag = false;
		Exception _last_exception;

		// void _clearRxBuffer();
	};

} // namespace mbed

#endif // MODBUS_RTU_MASTER_H