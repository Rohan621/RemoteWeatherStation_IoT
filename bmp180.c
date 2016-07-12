// BMP180 Driver
// Rohan Narula

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "bmp180.h"
#include "wait.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// This function reads the raw pressure data (16 to 19 bit) from the BMP180 pressure sensor. Depending
// on the value of Oversampling Ratio (oss), we can change the operating mode.
//*******************************************************************************************************
//          OSS                          Operating Mode             Samples         Conversion Time (ms)
//         -----                        ---------------            ---------       -----------------
//           0                           Ultra Low Power               1                  4.5
//           1                              Standard                   2                  7.5
//           2                           High Resolution               4                 13.5
//           3                          Ultra High Resolution          8                 25.5
//********************************************************************************************************
// By default, OSS = 0
long readPressure(uint8_t oss)
{
    long LSB, MSB, XLSB;
    I2C1_MSA_R = BMP180_ADDR_W;
    I2C1_MDR_R = 0xF4;
    I2C1_MCS_R = I2C1_MCS_START|I2C1_MCS_RUN;
    while(I2C1_MCS_R & I2C1_MCS_BUSY);
    if(oss == 0)
    {
        I2C1_MDR_R = 0x34;
    }
    else if(oss == 1)
    {
        I2C1_MDR_R = 0x74;
    }
    else if(oss == 2)
    {
        I2C1_MDR_R = 0xB4;
    }
    else
    {
        I2C1_MDR_R = 0xF4;
    }
    I2C1_MCS_R = I2C1_MCS_RUN|I2C1_MCS_STOP;
    while(I2C1_MCS_R & I2C1_MCS_BUSY);
    if(oss == 0)
    {
        waitMicrosecond(5000);
    }
    else if(oss == 1)
    {
        waitMicrosecond(8000);
    }
    else if(oss == 2)
    {
        waitMicrosecond(14000);
    }
    else
    {
        waitMicrosecond(26000);
    }

    writeByte(0xF6);
    MSB = readByte();
    writeByte(0xF7);
    LSB = readByte();
    writeByte(0xF8);
    XLSB = readByte();
    return ((MSB<<16|LSB<<8|XLSB)>>(8-oss));

}
// This function is used to read raw temperature data (16 bit) from BMP180 Sensor.
//*******************************************************************************************************
//      Operating Mode             Samples         Conversion Time (ms)
//     ----------------           ---------       -----------------
//        Standard                    2                  4.5
//********************************************************************************************************
long readTemperature()
{
    long temp;
    I2C1_MSA_R = BMP180_ADDR_W;
    I2C1_MDR_R = 0xF4;
    I2C1_MCS_R = I2C1_MCS_START|I2C1_MCS_RUN;
    while(I2C1_MCS_R & I2C1_MCS_BUSY);
    I2C1_MDR_R = 0x2E;
    I2C1_MCS_R = I2C1_MCS_RUN|I2C1_MCS_STOP;
    while(I2C1_MCS_R & I2C1_MCS_BUSY);
    waitMicrosecond(5000);
    writeByte(0xF6);
    temp = readWord();
    return temp;
}
// This function is used to write a data byte from TM4C to BMP180 sensor following I2C protocol.
void writeByte(uint8_t dataByte)
{
    I2C1_MSA_R = BMP180_ADDR_W;
    I2C1_MDR_R = dataByte;
    I2C1_MCS_R = I2C1_MCS_START|I2C1_MCS_RUN|I2C1_MCS_STOP;
    while(I2C1_MCS_R & I2C1_MCS_BUSY);
}
// This function is used to read a data byte from BMP180 to TM4C sensor following I2C protocol.
short readByte()
{
    I2C1_MSA_R = BMP180_ADDR_R;
    I2C1_MCS_R = I2C1_MCS_START|I2C1_MCS_RUN|I2C1_MCS_STOP;
    while(I2C1_MCS_R & 0x01);
    return (I2C1_MDR_R & 0xFF);
}
// This function is used to write a word from TM4C to BMP180 sensor following I2C protocol.
short readWord()
{
    uint8_t MSB, LSB;
    I2C1_MSA_R = BMP180_ADDR_R;
    I2C1_MCS_R = I2C1_MCS_START|I2C1_MCS_RUN|I2C1_MCS_ACK;
    while(I2C1_MCS_R & 0x01);
    MSB = (I2C1_MDR_R & 0xFF);
    I2C1_MCS_R = I2C1_MCS_STOP|I2C1_MCS_RUN;
    while(I2C1_MCS_R & 0x01);
    LSB =  (I2C1_MDR_R & 0xFF);
    return (LSB|(MSB<<8));
}
