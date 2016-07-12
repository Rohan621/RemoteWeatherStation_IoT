// BMP180 Driver
// Rohan Narula

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

#ifndef BMP180_H_
#define BMP180_H_

// ------------------------------------------------------------------------------
//  Defines
// ------------------------------------------------------------------------------

// I2C Master Control/Status Register (Read Only)
#define I2C1_MCS_BUSY   0x01
#define I2C1_MCS_ERR    0x02

// I2C Master Control/Status Register (Write Only)
#define I2C1_MCS_ACK    0x08
#define I2C1_MCS_NACK   0x00
#define I2C1_MCS_STOP   0x04
#define I2C1_MCS_START  0x02
#define I2C1_MCS_RUN    0x01

// Slave Address
#define BMP180_ADDR_W     0xEE
#define BMP180_ADDR_R     0xEF

// ------------------------------------------------------------------------------
//  Functions
// ------------------------------------------------------------------------------

void writeByte(uint8_t dataByte);
short readByte();
short readWord();
long readTemperature();
long readPressure(uint8_t oss);
#endif /* BMP180_H_ */
