// #ifndef AD7172_INTERFACE_HPP
// #define AD7172_INTERFACE_HPP

// #include "modbus_rtu/src/ModbusRTUClient.hpp"
// #include "ad7172/src/ad7172.hpp"
// #include "Mail.h"

// namespace mbed
// {
//     class AD7172Interface
//     {
//     public:



//         enum STATE
//         {
//             STATE_DATA_READY = (1 << 0),
//             STATE_SIM_DATA_READY = (1 << 1),
//         };

//         enum REGISTER : uint16_t
//         {
//             // STATUS_REGISTER         = 0x00,
//             // ADC_MODE_REGISTER       = 0x01,
//             // INTERFACE_MODE_REGISTER = 0x02,
//             // DATA_REGISTER           = 0x04,
//             // ID_REGISTER             = 0x07,

//             // CHANNEL0_REGISTER       = 0x10,
//             // CHANNEL1_REGISTER       = 0x11,
//             // CHANNEL2_REGISTER       = 0x12,
//             // CHANNEL3_REGISTER       = 0x13,
//             // CHANNEL4_REGISTER       = 0x14,
//             // CHANNEL5_REGISTER       = 0x15,
//             // CHANNEL6_REGISTER       = 0x16,
//             // CHANNEL7_REGISTER       = 0x17,
            
//             // FILTER_REGISTER0        = 0x28,

//             SIM_DATA_REGISTER       = 0x77,

//             RESET_REGISTER          = 0x99,


            

//             // VALUE_REGISTER_0 = 0,
//             // VALUE_REGISTER_1,
//             // VALUE_REGISTER_2,
//             // VALUE_REGISTER_3,
//             // VALUE_REGISTER_4,
//             // VALUE_REGISTER_5,
//             // VALUE_REGISTER_6,
//             // VALUE_REGISTER_7,
//             // VALUE_REGISTER_8,
//             // VALUE_REGISTER_9,
//             // VALUE_REGISTER_10,
//             // VALUE_REGISTER_11,
//             // VALUE_REGISTER_12,
//             // STATUS_REGISTER,
//         };
//     public:

//         AD7172Interface(ModbusRTUClient* modbusClient, AD7172* ad7172);
//         ~AD7172Interface() = default;

//         void Run();
    
//     private:
//         void DataReady();
//         void Receive();

//         void OnWriteHoldingRegister(uint16_t address, const uint8_t* payload, uint8_t payloadSize);
//         bool OnReadHoldingRegister(uint16_t address);
//         void OnWriteCoilRegisterCallback(uint16_t address, bool value);
//     public:
//         // static constexpr uint16_t ADC_MODE_REGISTER = 0x01;
    
//     private:
        
//         ModbusRTUClient* _modbusClient;
//         AD7172* _ad7172;
//         EventFlags _state;
//         rtos::Thread _acquireThread;
//         ModbusRTU::Register* _dataRegister1;
//         ModbusRTU::Register* _dataRegister2;
//        // Mail<uint8[32], 1> _mail;
//     };
// }

// #endif // AD7172_INTERFACE_HPP