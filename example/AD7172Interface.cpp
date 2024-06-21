// #include "AD7172Interface.hpp"

// #define TRACE_GROUP "ADIF"

// #define ENABLE_AD7172_INTERFACE_LOG
// #ifdef ENABLE_AD7172_INTERFACE_LOG
// #define log_info(...)       tr_info(__VA_ARGS__)  
// #define log_warn(...)       tr_warn(__VA_ARGS__) 
// #define log_debug(...)      tr_debug(__VA_ARGS__)
// #define log_error(...)      tr_error(__VA_ARGS__)
// #endif // ENABLE_AD7172_INTERFACE_LOG 

// using namespace mbed;

// AD7172Interface::AD7172Interface(ModbusRTUClient* modbusClient, AD7172* ad7172) :
//     _modbusClient(modbusClient),
//     _ad7172(ad7172),
//     _state(),
//     _acquireThread(osPriorityRealtime1, 2048, nullptr, "acquireThread")
// {
//     log_debug("AD7172Interface::%s", __func__);

//     _ad7172->enable();
//     _ad7172->set_output_data_rate(AD7172::FILTER0, AD7172::RATE_10_SPS);
//     // ad7171->
//     // _ad7172->reset();
//     // _ad7172->set_adc_mode(AD7172::MODE::CONTINUOUS);

//     log_info("ID = %hu", ad7172->get_id());

//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::ADC_MODE_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::ID_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::INTERFACE_MODE_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL0_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL1_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL2_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL3_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL4_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL5_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL6_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::CHANNEL7_REGISTER, 1));

//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::SETUP_REGISTER0, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::FILTER_REGISTER0, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, RESET_REGISTER, 1));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, AD7172::DATA_REGISTER, 2));
//     MBED_ASSERT(_modbusClient->AddRegister(ModbusRTU::Register::HREG, SIM_DATA_REGISTER, 16));

//     _modbusClient->SetReadHoldingRegisterCallback(callback(this, &AD7172Interface::OnReadHoldingRegister));
//     _modbusClient->SetWriteHoldingRegisterCallback(callback(this, &AD7172Interface::OnWriteHoldingRegister));
//     _modbusClient->SetWriteCoilRegisterCallback(callback(this, &AD7172Interface::OnWriteCoilRegisterCallback));

//     _ad7172->set_ready_callback(callback(this, &AD7172Interface::DataReady));

//     // _dataRegister1 = _modbusClient->GetRegisterReference(AD7172::DATA_REGISTER);
//     // _dataRegister2 = _modbusClient->GetRegisterReference(AD7172::DATA_REGISTER + 1);
//     // osStatus status = _acquireThread.start(mbed::callback(this, &AD7172Interface::Receive));
//     // if (status != osOK)
//     // {
//     //     tr_error("Failed to start the Modbus thread");
//     //     // TODO: MBED_ERROR
//     // }
// }

// void AD7172Interface::Receive()
// {
//     // TODO: define timeout? continous mode condition, channel open condition
//     tr_info("Start acquire thread");

//     uint8_t binArray[32] = { 0x00 };
//     int counter = 0;

//     while(1)
//     {
//         //log_error("Wait for data");

//         int rc = _state.wait_any_for(STATE_DATA_READY, std::chrono::milliseconds::max(), true);
//         if (rc > 0 && rc & STATE_DATA_READY)
//         {
//             // TODO: can be done while waiting
//             uint16_t status = _ad7172->read_register(AD7172::INTERFACE_MODE_REGISTER, 2);
//             log_warn("read INTERFACE_MODE_REGISTER [0x%04X]", status);
//             int regValue {0x00};
//             if (status & (0x40))
//             {
//                 log_warn("STATUS ENABLED");
//                 regValue = _ad7172->read_register(AD7172::DATA_REGISTER, 4);
//                 log_warn("read DATA_REGISTER [0x%X]", regValue);

//                 int channel = (regValue & 0x7);
//                 log_warn("CHANNEL %d READING ON", channel);
//                 //memcpy(&binArray[channel*4], &regValue, 4);
//                 const uint8_t* binRegValue = reinterpret_cast<const uint8_t*>(&regValue);
//                 binArray[channel * 4] = binRegValue[0];
//                 binArray[channel * 4+1] = binRegValue[1];
//                 binArray[channel * 4+2] = binRegValue[2];
//                 binArray[channel * 4+3] = binRegValue[3];
//                 counter++;

//                 if (channel == 7)
//                 {
//                     tr_info("Counter = %d", counter);
//                     counter = 0;
//                     // tr_info("ALL CHANNELS READ");
//                     //if (!_modbusClient->WriteRegisters(SIM_DATA_REGISTER + 2 * channel, 2, binRegValue, 4))
//                     // if (!_modbusClient->WriteRegisters(SIM_DATA_REGISTER, 16, binArray, 32))
//                     // {
//                     //     tr_error("Error writing in register");
//                     // }
//                     log_warn("Written into register");
//                     _state.set(STATE_SIM_DATA_READY);
//                 }
//             }
//             else
//             {
//                 tr_error("NOT HANDLED");
//             }   
//         }
//     }
// }

// void AD7172Interface::DataReady()
// {
//     _state.set(STATE_DATA_READY);
// }

// void AD7172Interface::OnWriteCoilRegisterCallback(uint16_t address, bool value)
// {
//     log_debug("AD7172Interface::%s", __func__);

//     REGISTER addr = static_cast<REGISTER>(address);

//     switch (addr)
//     {
//     case RESET_REGISTER:
//     {
//         log_warn("Reset ADC");
//         _ad7172->reset();
//         break;
//     }
//     default:
//         break;
//     }
// }

// void AD7172Interface::OnWriteHoldingRegister(uint16_t address, const uint8_t* payload, uint8_t payloadSize)
// {
//     log_debug("AD7172Interface::%s", __func__);

//     // Note: the address of the SPI device corresponds with the Modbus Address of the MCU

//     switch (address)
//     {
//     case AD7172::ADC_MODE_REGISTER:
//     {
//         // Set continuous/single output mode
//         int regValue = (payload[0] << 8) | payload[1];
//         log_warn("write ADC_MODE_REGISTER [0x%04X]", regValue);
//         _ad7172->write_register(address, regValue, 2);
//         // log_error("Read register and set callback");
//         // _ad7172->set_ready_callback(callback(this, &AD7172Interface::DataReady));

//         // _ad7172->read_register(AD7172::DATA_REGISTER, 4);
//         break;
//     }
//     case AD7172::INTERFACE_MODE_REGISTER:
//     {
//         // Enable/disable status data output
//         int regValue = (payload[0] << 8) | payload[1];
//         log_warn("write INTERFACE_MODE_REGISTER [0x%04X]", regValue);
//         _ad7172->write_register(address, regValue, 2);
//         break;
//     }
//     case AD7172::SETUP_REGISTER0:
//     case AD7172::SETUP_REGISTER1:
//     case AD7172::SETUP_REGISTER2:
//     case AD7172::SETUP_REGISTER3:
//     case AD7172::SETUP_REGISTER4:
//     case AD7172::SETUP_REGISTER5:
//     case AD7172::SETUP_REGISTER6:
//     case AD7172::SETUP_REGISTER7:
//     {
//         int regValue = (payload[0] << 8) | payload[1];
//         log_warn("write SETUP_REGISTER%d [0x%04X]", address - (int)AD7172::SETUP_REGISTER0, regValue);
//         _ad7172->write_register(address, regValue, 2);
//         break;
//     }
//     case AD7172::CHANNEL0_REGISTER:
//     case AD7172::CHANNEL1_REGISTER:
//     case AD7172::CHANNEL2_REGISTER:
//     case AD7172::CHANNEL3_REGISTER:
//     case AD7172::CHANNEL4_REGISTER:
//     case AD7172::CHANNEL5_REGISTER:
//     case AD7172::CHANNEL6_REGISTER:
//     case AD7172::CHANNEL7_REGISTER:
//     {
//         int regValue = (payload[0] << 8) | payload[1];
//         log_warn("write CHANNEL%d_REGISTER [0x%04X]", address - (int)AD7172::CHANNEL0_REGISTER, regValue);
//         _ad7172->write_register(address, regValue, 2);
//         break;
//     }
//     case AD7172::FILTER_REGISTER0:
//     {
//         int regValue = (payload[0] << 8) | payload[1];
//         log_warn("write FILTER_REGISTER0 [0x%04X]", regValue);
//         _ad7172->write_register(address, regValue, 2);
//         break;
//     }
//     default:
//         break;
//     }
// }

// bool AD7172Interface::OnReadHoldingRegister(uint16_t address)
// {
//     log_debug("AD7172Interface::%s", __func__);
//     bool success{true};

//     // Note: the address of the SPI device corresponds with the Modbus Address of the MCU

//     switch (address)
//     {
//     case AD7172::ID_REGISTER:
//     {
//         uint16_t regValue = _ad7172->read_register(address, 2);
//         log_warn("read ID_REGISTER [0x%04X]", regValue);

//         if (!_modbusClient->WriteSingleRegister(address, regValue))
//         {
//             success = false;
//         }
//         break;
//     }
//     case AD7172::ADC_MODE_REGISTER:
//     {
//         uint16_t regValue = _ad7172->read_register(AD7172::ADC_MODE_REGISTER, 2);
//         log_warn("read ADC_MODE_REGISTER [0x%04X]", regValue);
//         if (!_modbusClient->WriteSingleRegister(AD7172::ADC_MODE_REGISTER, regValue))
//         {
//             success = true;
//         }
//         break;
//     }
//     case AD7172::INTERFACE_MODE_REGISTER:
//     {
//         uint16_t regValue = _ad7172->read_register(address, 2);
//         log_warn("read INTERFACE_MODE_REGISTER [0x%04X]", regValue);
//         if (!_modbusClient->WriteSingleRegister(AD7172::INTERFACE_MODE_REGISTER, regValue))
//         {
//             success = true;
//         }
//         break;
//     }
//     case AD7172::SETUP_REGISTER0:
//     {
//         uint16_t regValue = _ad7172->read_register(AD7172::SETUP_REGISTER0, 2);
//         // log_warn("read CHANNEL%d_REGISTER [0x%04X]", (int)(address - 0x20), regValue);
//         if (!_modbusClient->WriteSingleRegister(address, regValue))
//         {
//             success = true;
//         }
//         break;
//     }

//     case AD7172::CHANNEL0_REGISTER:
//     case AD7172::CHANNEL1_REGISTER:
//     case AD7172::CHANNEL2_REGISTER:
//     case AD7172::CHANNEL3_REGISTER:
//     case AD7172::CHANNEL4_REGISTER:
//     case AD7172::CHANNEL5_REGISTER:
//     case AD7172::CHANNEL6_REGISTER:
//     case AD7172::CHANNEL7_REGISTER:
//     {
//         uint16_t regValue = _ad7172->read_register(address, 2);
//         log_warn("read CHANNEL%d_REGISTER [0x%04X]", (int)(address - 0x10), regValue);
//         if (!_modbusClient->WriteSingleRegister(address, regValue))
//         {
//             success = true;
//         }
//         break;
//     }
//     case AD7172::FILTER_REGISTER0:
//     {
//         uint16_t regValue = _ad7172->read_register(address, 2);
//         log_warn("read FILTER_REGISTER0 [0x%04X]",regValue);
//         if (!_modbusClient->WriteSingleRegister(address, regValue))
//         {
//             success = true;
//         }
//         break;
//     }
//     case AD7172::DATA_REGISTER:
//     {
//         // TODO: define timeout? continous mode condition, channel open condition
//         log_debug("Wait for data");
//         int rc = _state.wait_any_for(STATE_DATA_READY, std::chrono::milliseconds::max(), true);
//         if (rc > 0 && rc & STATE_DATA_READY)
//         {
//             // TODO: can be done while waiting
//             uint16_t status = _ad7172->read_register(AD7172::INTERFACE_MODE_REGISTER, 2);
//             log_warn("read INTERFACE_MODE_REGISTER [0x%04X]", status);
//             int regValue {0x00};
//             if (status & (0x40))
//             {
//                 log_warn("STATUS ENABLED");
//                 regValue = _ad7172->read_register(AD7172::DATA_REGISTER, 4);
//                 log_warn("read DATA_REGISTER [0x%X]", regValue);
//             }
//             else
//             {
//                 regValue = _ad7172->read_register(AD7172::DATA_REGISTER, 3);
//                 log_warn("read DATA_REGISTER [0x%X]", regValue);
//             }
//             const uint8_t* binRegValue = reinterpret_cast<const uint8_t*>(&regValue);
//             // _dataRegister1->value = binRegValue[0] | (binRegValue[1] << 8);
//             // _dataRegister2->value = binRegValue[1] | (binRegValue[2] << 8);
//             if (!_modbusClient->WriteRegisters(AD7172::DATA_REGISTER, 2, binRegValue, 4))
//             {
//                 success = true;
//             }

//             // log_info("set ready callback again");
//             // _ad7172->set_ready_callback(callback(this, &AD7172Interface::DataReady));
            
//         }
//         break;
//     }
//     case SIM_DATA_REGISTER:
//     {
//         std::chrono::milliseconds timeout = 6000ms;
//         int rc = _state.wait_any_for(STATE_SIM_DATA_READY, timeout, true);
//         log_warn("Wait for sim data ready");
//         if (rc > 0 && rc & STATE_SIM_DATA_READY)
//         {
//             //if (!_modbusClient->WriteRegisters())     
//             //uint16_t status = _ad7172->read_register(SIM_DATA_REGISTER, 8 * 2);
//             log_warn("read STATE_SIM_DATA_READY [0x%X]", 1);    
//         }
//         else
//         {
//             success = false;
//         }
//         break;


//         // log_debug("Wait for data on all channels");

//         // int currentChannel = 0;

//         //     // TODO: define timeout? continous mode condition, channel open condition
//         //     int rc = _state.wait_any_for(STATE_SIM_DATA_READY, std::chrono::milliseconds::max(), true);
//         //     if (rc > 0 && rc & STATE_DATA_READY)
//         //     {
//         //         // TODO: can be done while waiting
//         //         uint16_t status = _ad7172->read_register(AD7172::INTERFACE_MODE_REGISTER, 2);
//         //         log_warn("read INTERFACE_MODE_REGISTER [0x%04X]", status);
//         //         int regValue {0x00};
//         //         if (status & (0x40))
//         //         {
//         //             log_warn("STATUS ENABLED");
//         //             regValue = _ad7172->read_register(AD7172::DATA_REGISTER, 4);
//         //             int channel = (regValue & 0x7);

//         //             log_warn("CHANNEL = %d", channel);
//         //             log_warn("read DATA_REGISTER [0x%X]", regValue);
//         //         }
//         //         else
//         //         {
//         //             tr_error("UNHANDLED ERROR");
//         //             success = false;
//         //         }
//         //         // TODO: error
//         //         // regValue = _ad7172->read_register(AD7172::DATA_REGISTER, 3);
//         //         // log_warn("read DATA_REGISTER [0x%X]", regValue);
//         //     }
//         //     const uint8_t* binRegValue = reinterpret_cast<const uint8_t*>(&regValue);
//         //     if (!_modbusClient->WriteRegisters(SIM_DATA_REGISTER, 2, binRegValue, 4))
//         //     {
//         //         success = false;
//         //     }
//     }
    
//     // case 0x6b:
//     // {
//     //     uint8_t payload[6] = { 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40 };
        
//     //     if (!_modbusClient->WriteRegisters(0x6b, 3, payload, 6))
//     //     {
//     //         success = false;
//     //     }
//     //     break;
//     // } 
   
//     default:
//         break;
//     }

//     return success;
// }