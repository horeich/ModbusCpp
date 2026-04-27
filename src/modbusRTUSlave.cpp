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

// Feature Requests
// TODO: suspend - test
// TODO: return more concise error codes

// function 0x03 + address 0x0000 → Holding Register 40001
// function 0x04 + address 0x0000 → Input Register 30001
// function 0x01 + address 0x0000 → Coil 00001
// function 0x02 + address 0x0000 → Discrete Input 10001

#include "ModbusRTUSlave.h"

#ifdef MBED_CONF_MODBUS_RTU
#define TRACE_GROUP "MODB"
#define dbg_info(...) tr_info(__VA_ARGS__)
#define dbg_debug(...) tr_debug(__VA_ARGS__)
#define dbg_warn(...) tr_warn(__VA_ARGS__)
#define dbg_error(...) tr_error(__VA_ARGS__)
#else
#define dbg_info(...)
#define dbg_debug(...)
#define dbg_warn(...)
#define dbg_error(...)
#endif

#define bitWrite(value, bit, bitvalue) ((bitvalue) ? set_bit(value, bit) : clear_bit(value, bit))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

using namespace mbed;

ModbusRTUSlave::ModbusRTUSlave(
    mbed::BufferedRS485 &rs485,
    uint8_t id) : 
        ModbusRTU(rs485),
        _modbusThread(osPriorityNormal, 2048, nullptr, "modbusSlaveThread"),
        _queue(64 * EVENTS_EVENT_SIZE),
        _sigioCb(nullptr),
        _on_request_processed(nullptr),
        _input_registers(nullptr),
        _input_register_size(0),
        _holding_registers(nullptr),
        _holding_register_size(0),
        _coils(nullptr),
        _coil_size(0),
        _discrete_inputs(nullptr),
        _discrete_input_size(0),
        _silent_period_enabled(true)
{
    if (!set_slave_id(id))
    {
        dbg_error("Invalid Modbus Id");
        MBED_ASSERT(false);
    }

    osStatus status = _modbusThread.start(callback(&_queue, &events::EventQueue::dispatch_forever));
    if (status != osOK)
    {
        dbg_error("Failed to start sensor detect thread");
    }

    // Reference signal in/out
    _rs485.sigio(mbed::callback(this, &ModbusRTUSlave::on_sigio));

    // The slave always listens on the RX line
    _rs485.enable_rx();
}

ModbusRTUSlave::~ModbusRTUSlave()
{
    // Stop listening on the RX line
    _rs485.disable_rx();
    _rs485.sigio(nullptr);

    _queue.break_dispatch();
    
    osStatus status = _modbusThread.join();
    if (status != osOK)
    {
        dbg_error("Failed to start sensor detect thread");
    }
}

void ModbusRTUSlave::on_sigio()
{
    // Note: This will be called if one/ multiple byte(s) arrive(s) at the UART RX buffer.
    // Especially at lower baud rates and large messages we expect more bytes to arrive at 
    // a later time. Set the _read_timeout variable accordingly.

    int eventId = _queue.call(mbed::callback(this, &ModbusRTUSlave::process_request_internal));
    if (eventId == 0)
    {
        // dbg_error("Queue full, increase queue size"); // ISR context no mutex!
    }

    if (_sigioCb != nullptr)
    {
        _sigioCb();
    }
}

void ModbusRTUSlave::set_received_callback(mbed::Callback<void(FunctionCode, Exception)> on_request_received)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);
    _on_request_processed = on_request_received;
}

bool ModbusRTUSlave::set_slave_id(uint8_t id)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    if (id >= 1 && id <= 247)
    {
        _id = id;
        return true;
    }
    else
    {
        dbg_error("Invalid Modbus Id");
        return false;
    }
}

void ModbusRTUSlave::set_sigio_callback(mbed::Callback<void()> sigioCb)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _sigioCb = sigioCb;
}

void ModbusRTUSlave::set_coils(bool coils[], uint16_t coil_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _coils = coils;
    _coil_size = coil_size;
}

void ModbusRTUSlave::set_discrete_inputs(bool discrete_inputs[], uint16_t discrete_input_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _discrete_inputs = discrete_inputs;
    _discrete_input_size = discrete_input_size;
}

void ModbusRTUSlave::set_holding_registers(uint16_t holding_registers[], uint16_t holding_register_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _holding_registers = holding_registers;
    _holding_register_size = holding_register_size;
}

void ModbusRTUSlave::set_input_registers(uint16_t input_registers[], uint16_t input_register_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _input_registers = input_registers;
    _input_register_size = input_register_size;
}

void ModbusRTUSlave::process_request_internal()
{
    // This is the method called from internal queue and also poses an
    // example on how to use it from an external application.
    // The purpose of splitting processing and request is to give the app
    // the chance to modify registers before they are sent as a response.

    dbg_debug("ModbusRTUSlave::%s", __func__);

    ScopedLock<rtos::Mutex> lock(_dataMutex);

    FunctionCode function_code = FunctionCode::None;
    ssize_t rc = process_request(function_code);
    if (rc <= 0)
    {
        dbg_warn("Invalid request or no data (%d)", rc);
        // TODO: MBED_ERROR
        return;
    }

    rc = respond_request(function_code);
    if (rc < 0)
    {
        dbg_warn("Error during response (%d)", rc);
        // TODO: MBED_ERROR
        return;
    }

    Exception ex_code = Exception::None;
    if ((_buffer[1] >> 4) & 0x0F)
    {
        ex_code = static_cast<Exception>(_buffer[2] &~ 0xF0);
    }

    // This callback allows the app to write to/ read from registers/ coils
    if (_on_request_processed != nullptr) 
    {
        _on_request_processed(function_code, ex_code);
    }

    dbg_info("Processed %s (exception %u: %s)", ModbusRTU::GetFCString(function_code), (uint8_t)ex_code, ModbusRTU::GetExceptionString(static_cast<Exception>(ex_code)));
}

ssize_t ModbusRTUSlave::process_request(FunctionCode& function_code)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    // Try to read data from serial
    ssize_t rc = read(_buffer, sizeof(_buffer), true);
    if (rc < 0)
    {
        dbg_warn("Error while reading serial data (%d)", rc);
        return rc;
    }
    else if (rc == 0)
    {
        dbg_info("No data read from serial");
        return rc;
    }

    // Message debug output
    dbg_debug("Receive buffer contains %d bytes: ", rc);
    for (ssize_t i = 0; i < rc; i++)
    {
        dbg_debug("0x%02X", _buffer[i]);
    }

    // Check minimum modbus message size
    if (rc < 8)
    {
        dbg_warn("Invalid message");
        return -1;
    }

    // Extract id and check with own id
    uint8_t id = _buffer[0];
    if (id != _id) // first byte contains slave id
    {
        dbg_warn("Ignore request (wrong ID: %d (actual: %u))", id, _id);
        return -1;
    }

    // CRC check - ignore request silently
    uint16_t crc = calculate_crc(rc - 2);
    if (crc != bytesToWord(_buffer[rc - 1], _buffer[rc - 2]))
	{
		dbg_warn("Invalid CRC (0x%04X vs 0x%04X) - ignore request", crc, bytesToWord(_buffer[rc - 1], _buffer[rc - 2]));
		return -1;
    }

    // Store function code
    function_code = static_cast<FunctionCode>(_buffer[1]);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]); // requested register count

    tr_info("Received request [id = %u, FC = %u, start = %u, count = %u]", id, _buffer[1], start_address, register_count);

    return rc;
}

ssize_t ModbusRTUSlave::respond_request(FunctionCode function_code)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    int rc = -1;

    switch (function_code) // second byte contains function code
    {
    case FunctionCode::ReadCoils:
        rc = read_coils();
        break;
    case FunctionCode::ReadDiscreteInputs:
        rc = read_discrete_inputs();
        break;
    case FunctionCode::ReadHoldingRegisters:
        rc = read_holding_register(_holding_registers, _holding_register_size);
        break;
    case FunctionCode::ReadInputRegisters:
        rc = read_input_registers(_input_registers, _input_register_size);
        break;
    case FunctionCode::WriteSingleCoil:
        rc = write_single_coil();
        break;
    case FunctionCode::WriteSingleHoldingRegister:
        rc = write_single_holding_register(_holding_registers, _holding_register_size);
        break;
    case FunctionCode::WriteMultipleCoils:
        rc = write_multiple_coils();
        break;
    case FunctionCode::WriteMultipleHoldingRegisters:
        rc = write_multiple_holding_registers(_holding_registers, _holding_register_size);
        break;
    default:
        set_exception(Exception::IllegalFunction);
        rc = write_response(3);
        break;
    }

    return rc;
}

ssize_t ModbusRTUSlave::read_coils()
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;

    if (!_coils || _coil_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (register_count == 0 || register_count > 2000)
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (register_count > _coil_size || start_address > (_coil_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        _buffer[2] = div8RndUp(register_count);
        for (uint16_t i = 0; i < register_count; i++)
        {
            bitWrite(_buffer[3 + (i >> 3)], i & 7, _coils[start_address + i]);
        }
        response_len += _buffer[2];
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::read_discrete_inputs()
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;

    if (!_discrete_inputs || _discrete_input_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (register_count == 0 || register_count > 2000)
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (register_count > _discrete_input_size || start_address > (_discrete_input_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        _buffer[2] = div8RndUp(register_count);
        for (uint16_t i = 0; i < register_count; i++)
        {
            bitWrite(_buffer[3 + (i >> 3)], i & 7, _discrete_inputs[start_address + i]);
        }
        response_len += _buffer[2];
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::read_holding_register(uint16_t* registers, uint16_t registers_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;

    if (registers == nullptr || registers_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (register_count == 0 || register_count > 125)
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (register_count > registers_size || start_address > (registers_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        _buffer[2] = register_count * 2;
        for (uint16_t i = 0; i < register_count; i++)
        {
            _buffer[3 + (i * 2)] = highByte(registers[start_address + i]);
            _buffer[4 + (i * 2)] = lowByte(registers[start_address + i]);
        }
        response_len += _buffer[2];
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::read_input_registers(uint16_t* registers, uint16_t register_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]); // requested register count

    uint16_t response_len = 3;

    if (register_size == 0) // || _input_register_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (register_count == 0 || register_count > 125)
    {
        set_exception(Exception::IllegalDataValue);
    }
    // else if (register_count > _input_register_size || start_address > (_input_register_size - register_count))
    else if (register_count > register_size || start_address > (register_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        _buffer[2] = register_count * 2;              // number of read registers
        for (uint16_t i = 0; i < register_count; i++) // add data to buffer
        {
            _buffer[3 + (i * 2)] = highByte(registers[start_address + i]);
            _buffer[4 + (i * 2)] = lowByte(registers[start_address + i]);
        }
        response_len += _buffer[2];
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::write_single_coil()
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t value = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;

    if (!_coils || _coil_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (value != 0 && value != 0xFF00)
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (address >= _coil_size)
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        _coils[address] = value;
        response_len += 3;
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::write_single_holding_register(uint16_t* registers, uint16_t registers_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t value = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;

    if (registers == nullptr || registers_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (address >= registers_size)
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        registers[address] = value;
        response_len += 3;
    }

    return write_response(response_len); 
}

ssize_t ModbusRTUSlave::write_multiple_coils()
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]);

    uint16_t response_len = 3;
   
    if (!_coils || _coil_size == 0)
    {
        set_exception(Exception::IllegalFunction); 
    }
    else if (register_count == 0 || register_count > 1968 || _buffer[6] != div8RndUp(register_count))
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (register_count > _coil_size || start_address > (_coil_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        for (uint16_t i = 0; i < register_count; i++)
        {
            _coils[start_address + i] = bitRead(_buffer[7 + (i >> 3)], i & 7);
        }
        response_len += 3;
    }

    return write_response(response_len); 
}

ssize_t ModbusRTUSlave::write_multiple_holding_registers(uint16_t* registers, uint16_t registers_size)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t start_address = bytesToWord(_buffer[2], _buffer[3]);
    uint16_t register_count = bytesToWord(_buffer[4], _buffer[5]);

    size_t response_len = 3;

    if (registers == nullptr || registers_size == 0)
    {
        set_exception(Exception::IllegalFunction);
    }
    else if (register_count == 0 || register_count > 123 || _buffer[6] != (register_count * 2))
    {
        set_exception(Exception::IllegalDataValue);
    }
    else if (register_count > registers_size || start_address > (registers_size - register_count))
    {
        set_exception(Exception::IllegalDataAddress);
    }
    else
    {
        for (uint16_t i = 0; i < register_count; i++)
        {
            registers[start_address + i] = bytesToWord(_buffer[i * 2 + 7], _buffer[i * 2 + 8]);
        }
        response_len += 3;
    }

    return write_response(response_len);
}

ssize_t ModbusRTUSlave::write_response(uint8_t len)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    uint16_t crc = calculate_crc(len);
    _buffer[len] = lowByte(crc);
    _buffer[len + 1] = highByte(crc);

    // Write to serial output (disable receive otherwise we'll receive an echo)
    _rs485.disable_rx();
    ssize_t rc = ModbusRTU::write(_buffer, len + 2);
    _rs485.enable_rx();

    dbg_debug("Wrote response (%d bytes): ", rc);
    for (int i = 0; i < rc; ++i)
    {
        dbg_debug("0x%02X", _buffer[i]);
    }

    return rc;
}

void ModbusRTUSlave::set_exception(Exception code)
{
    dbg_debug("ModbusRTUSlave::%s", __func__);

    _buffer[1] |= 0x80; // highes bit in function code is set
    _buffer[2] = static_cast<uint8_t>(code);
}