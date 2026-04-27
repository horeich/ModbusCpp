// Copyright 2025-2026 Andreas Reichle, Fawad Siddiqui (HOREICH GmbH)

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#include <vector>
#include <array>
#include <bitset>

#include "BufferedRS485.h"
#include "Callback.h"
#include "mbed_trace.h"
#include "Timer.h"
#include "events/EventQueue.h"
#include "Thread.h"
#include "ModbusRTU.h"

namespace mbed
{
	class ModbusRTUSlave : public ModbusRTU
	{
	public:
		explicit ModbusRTUSlave(
			mbed::BufferedRS485 &serial, 
			uint8_t id = MBED_CONF_MODBUS_SLAVE_ID);
		~ModbusRTUSlave();

		void set_received_callback(mbed::Callback<void(FunctionCode, Exception)> on_request_received);
		void set_sigio_callback(mbed::Callback<void()> sigioCb);

		ssize_t process_request(FunctionCode& function_code);
		ssize_t respond_request(FunctionCode function_code);

		void set_coils(bool coils[], uint16_t coil_size);
		void set_discrete_inputs(bool discrete_inputs[], uint16_t discrete_input_size);
		void set_input_registers(uint16_t input_registers[], uint16_t input_register_size);
		void set_holding_registers(uint16_t holding_registers[], uint16_t holding_register_size);
		bool set_slave_id(uint8_t id);
		
		/**
		 * Enable the Modbus silent period (3.5 character times per specification).
		 * The slave will wait this duration before responding to a request.
		 * @param enable Whether to enable the silent period
		 */
		void enable_silent_period(bool enable);

	private:
		void on_sigio();
		void process_request_internal();
		ssize_t write_response(uint8_t len);
		void set_exception(Exception code);
		void wait_silent_period();

		ssize_t read_coils();
		ssize_t read_discrete_inputs();
		ssize_t read_holding_register(uint16_t* registers, uint16_t register_size);
		ssize_t read_input_registers(uint16_t* registers, uint16_t register_size);
		ssize_t write_single_coil();
		ssize_t write_single_holding_register(uint16_t* registers, uint16_t registers_size);
		ssize_t write_multiple_coils();
		ssize_t write_multiple_holding_registers(uint16_t* registers, uint16_t registers_size);

	private:
		uint8_t _id;
		rtos::Thread _modbusThread;
		events::EventQueue _queue;
		Callback<void()> _sigioCb;
		mbed::Callback<void(FunctionCode, Exception)> _on_request_processed;
		uint16_t* _input_registers;
		uint16_t _input_register_size;
		uint16_t* _holding_registers;
		uint16_t _holding_register_size;
		bool *_coils;
		uint16_t _coil_size;
		bool *_discrete_inputs;
		uint16_t _discrete_input_size ;

		unsigned long _charTimeout; // TODO:
		unsigned long _frameTimeout;
		mbed::Timer _request_timer;  // Timer to track when request was received
		bool _silent_period_enabled;  // Flag to enable/disable silent period
	};

} // namespace mbed

#endif // MODBUS_RTU_SLAVE_H