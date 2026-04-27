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

#include "ModbusRTU.h"

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
#endif // MBED_CONF_MODBUS_RTU

using namespace mbed;
using namespace std::chrono_literals;

ModbusRTU::ModbusRTU(
    BufferedRS485 &rs485,
    std::chrono::milliseconds response_timeout) :
    _rs485(rs485),
    _dataMutex(),
    _response_timeout(response_timeout)
{
}

ModbusRTU::~ModbusRTU()
{
    // no resources to be freed
}

void ModbusRTU::lock()
{
    _dataMutex.lock();
}

void ModbusRTU::unlock()
{
    _dataMutex.unlock();
}

uint16_t ModbusRTU::calculate_crc(uint8_t len)
{
    dbg_debug("ModbusRTU::%s", __func__);

	uint16_t value = 0xFFFF;
	for (uint8_t i = 0; i < len; i++)
	{
		value ^= (uint16_t)_buffer[i];
		for (uint8_t j = 0; j < 8; j++)
		{
			bool lsb = value & 1;
			value >>= 1;
			if (lsb == true)
            {
				value ^= 0xA001;
            }
		}
	}
	return value;
}

std::chrono::microseconds ModbusRTU::char_time_us(uint32_t baudrate)
{
	// 11 bits per RTU character (1 start + 8 data + parity + stop)
	return std::chrono::microseconds((11ULL * 1'000'000ULL) / baudrate);
}

std::chrono::microseconds ModbusRTU::t1_5(uint32_t baudrate)
{
	if (baudrate > 19200)
	{
		return std::chrono::microseconds(750);   // Modbus fixed value
	}
	return char_time_us(baudrate) * 3 / 2;
}

std::chrono::microseconds ModbusRTU::t3_5(uint32_t baudrate)
{
	if (baudrate > 19200)
	{
		return std::chrono::microseconds(1750);  // Modbus fixed value
	}
	return char_time_us(baudrate) * 7 / 2;
}

void ModbusRTU::set_response_timeout(std::chrono::milliseconds response_timeout)
{
    dbg_debug("ModbusRTU::%s", __func__);

	mbed::ScopedLock<rtos::Mutex> lock(_dataMutex);

    _response_timeout = response_timeout;
}

ssize_t ModbusRTU::read(uint8_t* recv_buf, size_t recv_buf_len, bool async)
{
    size_t recvd_len = 0;

    const uint32_t baud = _rs485.get_baudrate();
    const auto char_timeout  = t1_5(baud);
    const auto frame_timeout = t3_5(baud);

    mbed::Timer frame_timer;
    mbed::Timer char_timer;
    mbed::Timer idle_timer;

    idle_timer.start();

    if (!async || _rs485.readable())
    {
        while (idle_timer.elapsed_time() < _response_timeout)
        {
            if (_rs485.poll(0) & POLLIN)
            {
                // data available
                // dbg_debug("polled in %d", (int)recvd_len);
                ssize_t rc = _rs485.read(&recv_buf[recvd_len], recv_buf_len - recvd_len);
                if (rc > 0)
                {
                    if (recvd_len == 0)
                    {
                        // start timers on first byte received
                        frame_timer.reset();
                        frame_timer.start();
                    }
                    else // recvd_len > 0
                    {
                        // Inter-frame timing too long > discard frame
                        if (char_timer.elapsed_time() > char_timeout && frame_timer.elapsed_time() < frame_timeout)
                        {
                            dbg_debug("Modbus: char timeout → discard frame");
                            recvd_len = -1;
                            char_timer.stop();
                            frame_timer.stop();
                            return recvd_len; // invalid response > ignore
                        }
                    }

                    recvd_len += rc;

                    // reset both timers on byte reception
                    char_timer.reset();
                    char_timer.start();

                    frame_timer.reset();
                    frame_timer.start();
                }
                else if (rc < 0)
                {
                    // Error while reading
                    return rc;
                }
            }
            else
            {
                // During idle check for EoF
                if (recvd_len > 0 && frame_timer.elapsed_time() > frame_timeout)
                {
                    dbg_debug("Modbus: frame timeout → %d bytes", (int)recvd_len);
                    return recvd_len; // probably valid response
                }
            }
        }
    }
    
    dbg_debug("no data");

    idle_timer.stop();
    return recvd_len;
}

ssize_t ModbusRTU::write(const uint8_t* write_buf, size_t write_buf_len) // tODO:: uint16_t
{
    dbg_debug("ModbusRTU::%s", __func__);

    _rs485.enable_tx();
    ssize_t rc = _rs485.write(write_buf, write_buf_len);
    _rs485.sync(); // Make sure all data is transferred (add support for async API later)
    _rs485.disable_tx();

    if (rc != (ssize_t)write_buf_len)
    {
        dbg_warn("Not all written");
    }

    return rc;
}

void ModbusRTU::set_bit(uint8_t &value, uint8_t bit)
{
    dbg_debug("ModbusRTU::%s", __func__);

	value |= (1 << bit);
}

void ModbusRTU::clear_bit(uint8_t &value, uint8_t bit)
{
    dbg_debug("ModbusRTU::%s", __func__);

	value &= ~(1 << bit);
}

uint16_t ModbusRTU::div8RndUp(uint16_t value)
{
	return (value + 7) >> 3;
}

uint16_t ModbusRTU::bytesToWord(uint8_t high, uint8_t low)
{
	return (high << 8) | low;
}

uint8_t ModbusRTU::lowByte(uint16_t x)
{
	return (uint8_t)(x & 0xFF);
}

uint8_t ModbusRTU::highByte(uint16_t x)
{
	return (uint8_t)(x >> 8);
}