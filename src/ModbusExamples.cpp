#include "ModbusExamples.hpp"
#include "ModbusRTUMaster.h"
#include "ModbusRTUSlave.h"
#include "modbusTCPClient.hpp"
#include "W5500.hpp"
#include <mbed.h>
#include <stdint.h>

#define RTUMaster
//#define RTUSlave
//#define TCPClient

bool coils[2];
bool discreteInputs[2]={true,true};
uint16_t holdingRegisters[2]={5,10};
uint16_t inputRegisters[2]={20,30};
UnbufferedSerial serialInterface(PC_10,PC_11);
UnbufferedRS485 rs485(serialInterface,PA_14,PA_15,9600);

int ModbusExample()
{
  #ifdef RTUMaster
   /*******************Modbus RTU Master************************/
  // ModbusRTUMaster master(rs485);
  wait_us(5000*1000);

//  /*                        Write Coil Function                                */
//   bool bufCoils[2]={true,false};
//   master.writeMultipleCoils(11, 0,bufCoils, 2);
//  /*                         Write Holding Register Functions                  */
//   master.writeSingleHoldingRegister(11,0,9);
//   uint16_t buf[2]={5,6};
//   master.writeMultipleHoldingRegisters(11,0,buf,2);

  /*                         Read Coils Function                                 */
  bool readBuf[2]={0};
  // master.readCoils(11,0,readBuf,2);
  printf("\nCoil register value 1 is %d \n",readBuf[0]);
  printf("\nCoil register value 2 is %d \n",readBuf[1]);

//   /*                          Read Holding Register Function                   */
//   //Single Value Read
//   uint16_t readBuf1=0;
//   master.readHoldingRegisters(11,0,&readBuf1,1);
//   printf("\nholding register value is %d \n",readBuf1);
//   // Multiple Value Read
  // uint16_t readBuf2[2]={0};
//   master.readHoldingRegisters(11,0,readBuf2,2);
//   printf("\nholding register value 1 is %d \n",readBuf2[0]);
//   printf("\nholding register value 2 is %d \n",readBuf2[1]);

  // /*                             Read Discrete Inputs Function                  */
  // master.readDiscreteInputs(11,0,readBuf,2);
  // printf("\nDiscrete Input value 1 is %d \n",readBuf[0]);
  // printf("\nDiscrete Input value 2 is %d \n",readBuf[1]);

  // /*                              Read Input Registers Function                 */
  // master.readInputRegisters(11,0,readBuf2,2);
  // printf("\nInput register value 1 is %d \n",readBuf2[0]);
  // printf("\nInput register value 2 is %d \n",readBuf2[1]);
  #endif

  #ifdef RTUSlave
      /*******************Modbus RTU Slave************************/
    ModbusRTUSlave slave(rs485,11);
    slave.configureCoils(coils, 2);                       // bool array of coil values, number of coils
    slave.configureDiscreteInputs(discreteInputs, 2);     // bool array of discrete input values, number of discrete inputs
    slave.configureHoldingRegisters(holdingRegisters, 2); // unsigned 16 bit integer array of holding register values, number of holding registers
    slave.configureInputRegisters(inputRegisters, 2);     // unsigned 16 bit integer array of input register values, number of input registers

    while(1)
    {
      slave.poll();
    }
  #endif

  #ifdef TCPClient
          /*******************Modbus TCP Client************************/
    SPI spi(PB_15, PB_14, PB_13); // mosi, miso, sclk
    W5500 w5500(&spi, PB_12, PA_10);
    spi.format(8,0); // 8bit, mode 0
    spi.frequency(1000000); // 1MHz
    wait_us(1000*1000); // 1 second for stable state
    uint8_t MAC_Addrc[6] = {0x00,0x08,0xDC,0x12,0x34,0x56};
    w5500.socket0ConfigModbus("192.168.13.164","255.255.255.0","192.168.11.1","192.168.13.165",&MAC_Addrc[0]);
    ModbusTCPClient modbus(&w5500);
    modbus.connect();
    modbus.modbusSetSlaveId(11);

    // // read coil                        function 0x01
    // // bool read_coil;
    // modbus.modbusReadCoils(1, 1, &read_coil);

    // // read input bits(discrete input)  function 0x02
    // bool read_bits;
    // modbus.modbusReadInputBits(3, 1, &read_bits);

    // // read holding registers           function 0x03
    // uint16_t read_holding_regs[1];
    // modbus.modbusReadHoldingRegisters(0, 1, read_holding_regs);

    // // read input registers             function 0x04
    // uint16_t read_input_regs[1];
    // modbus.modbusReadInputRegisters(0, 1, read_input_regs);

    // // write single coil                function 0x05
    // modbus.modbusWriteCoil(0, true);

    // // write single reg                 function 0x06
    // modbus.modbusWriteRegister(0, 123);

    // // write multiple coils             function 0x0F
    bool write_cols[4] = {false, true, false, true};
    modbus.modbusWriteCoils(0, 4, write_cols);

    // // write multiple regs              function 0x10
    // uint16_t write_regs[4] = {21, 22, 23,24};
    // modbus.modbusWriteRegisters(4, 4, write_regs);

    // // close connection and free the memory
    // networkInterface.close(0);
  #endif
  return 0;
}