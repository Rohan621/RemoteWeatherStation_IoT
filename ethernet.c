// Remote Weather Station Module
// Rohan Narula

// Information provided by this node
//  1. Temperature
//  2. Barometric Pressure (in Pascal and mm of Hg)
//  3. Relative Humidity
//  4. Altitude from Sea Level
//  5. Dew Point

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60, BMP180, & DHT11
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller 
//   MOSI (SSI2Tx) on PB7
//   MISO (SSI2Rx) on PB6
//   SCLK (SSI2Clk) on PB4
//   ~CS connected to PB1

// BMP180 Barometric & Temperature Pressure Sensor
//  SCL (I2C1SCL) on PA6
//  SDA (I2C1SDA) on PA7

// DHT11 Humidity & Temperature Sensor
// Data (GPIO) on PC6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "enc28j60.h"
#include "wait.h"
#include "dht11.h"
#include "bmp180.h"
#include "math.h"
#include "DataHeader.h"
#include "iotctrl.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))

//---------------------------------------------------------------------
// Variable Declarations
//-----------------------------------------------------------------------------

uint8_t T_Byte1, T_Byte2, RH_Byte1, RH_Byte2, checksum, index = 0, oss = 0;
short AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned short AC4, AC5, AC6;
long rawTemp, rawPressure, X1, X2, X3, B5, B6, B3, P;
unsigned long B4, B7;
double T;
float pInches, temporary, altitude, Td;
uint8_t* sensorData[29], sensorData_rev[28];
uint8_t* udpData;
uint8_t data[128];
uint8_t* name = "Weather";
uint8_t* information = "Sensor Data";
//---------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO ports for the following functions
    //*******************************************************
    //          PORT                Function
    //         -----               ----------
    //           A                   BMP180
    //           B                  ENC28J60
    //           C                   DHT11
    //           F                    LED
    //*******************************************************
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC;

    // Configure LED pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;  // enable LEDs

    // Configure ~CS for ENC28J60
    GPIO_PORTB_DIR_R = 0x02;  // make bit 1 an output
    GPIO_PORTB_DR2R_R = 0x02; // set drive strength to 2mA
    GPIO_PORTB_DEN_R = 0x02;  // enable bits 1 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
	GPIO_PORTB_AFSEL_R |= 0xD0;                      // select alternative functions for MOSI, MISO, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xD0;                        // enable digital operation on TX, RX, CLK pins

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;   // set SR=0, mode 0 (SPH=0, SPO=0), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2

    // Configure pins of Port C for DHT11 humidity sensor
    GPIO_PORTC_DEN_R  |= 0x40;
    GPIO_PORTC_DR2R_R |= 0x40;
    GPIO_PORTC_PUR_R  |= 0x40;

    // Configure the I2C module 1 for BMP180 sensor
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;          // turn-on I2C1 clocking
    GPIO_PORTA_AFSEL_R |= 0xC0;                     // select alternative functions for SDA, SCL pins
    GPIO_PORTA_DEN_R |= 0xC0;                       // enable digital operation on SDA, SCL pins
    GPIO_PORTA_ODR_R |= 0x80;                       // enable open-drain for SDA pin
    GPIO_PORTA_PUR_R |= 0x40;                       // enable pullup resistor for SCL pin
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA7_I2C1SDA | GPIO_PCTL_PA6_I2C1SCL; // map alt fns to I2C1
    I2C1_MCR_R |= 0x10;                             // Master Mode Enable
    I2C1_MTPR_R = 19;                               // Standard Mode Freq: 100kHz

    // Configure TIMER1 for reading sensor data
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x7270E00;                      // set load value to 120e6 for 0.33 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
}

//-----------------------------------------------------------------------------
// Main                
//-----------------------------------------------------------------------------

int main(void)
{
    // init controller
    initHw();

    // init ethernet interface
    etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
    etherSetIpAddress(192,168,137,30);

    // flash phy leds
    etherWritePhy(PHLCON, 0x0880);
    RED_LED = 1;
    waitMicrosecond(500000);
    etherWritePhy(PHLCON, 0x0990);
    RED_LED = 0;
    waitMicrosecond(500000);
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    // message loop
    while (1)
    {
        if (etherKbhit())
        {
            if (etherIsOverflow())
            {
                RED_LED = 1;
                waitMicrosecond(100000);
                RED_LED = 0;
            }
            // get packet
            etherGetPacket(data, 128);
            // handle arp request
            if (etherIsArp(data))
            {
                etherSendArpResp(data);
                RED_LED = 1;
                GREEN_LED = 1;
                waitMicrosecond(50000);
                RED_LED = 0;
                GREEN_LED = 0;
            }
            // handle ip datagram
            if (etherIsIp(data))
            {
            	if (etherIsIpUnicast(data))
            	{
            		// handle icmp ping request
					if (etherIsPingReq(data))
					{
					  etherSendPingResp(data);
					  RED_LED = 1;
					  BLUE_LED = 1;
					  waitMicrosecond(50000);
					  RED_LED = 0;
					  BLUE_LED = 0;
					}
					// handle udp datagram
					if (etherIsUdp(data))
					{
						udpData = etherGetUdpData(data);
						// Check what kind of UDP datagram the controller received
						// Check if the UDP datagram received is DISCOVERY CONTROL REQUEST
						if(checkUdpPacket(udpData) == 1)
						{
						    etherSendUdpData(data, udpData, 32);
						}
						// Check if the UDP datagram received is RULES CONTROL REQUEST
						else if(checkUdpPacket(udpData) == 2)
						{
						    etherSendUdpData(data, udpData, 60);
						}
						if (udpData[0] == '1')
							GREEN_LED = 1;
						if (udpData[0] == '0')
							GREEN_LED = 0;
						BLUE_LED = 1;
						waitMicrosecond(100000);
						BLUE_LED = 0;
					}
                }
            }
        }
    }
}

// This functions corrosponds to the action to be performed the timer count downs to zero.
// After every 3 seconda, it enters this function.
// This function reads the data from BMP180 and DHT11, creates an UDP Packet, and
// transmits it to the Weather Display node.
void Timer1Isr()
{
    // DHT11 Sensor Code
    waitMicrosecond(2000000);
    GPIO_PORTC_DIR_R  |= 0x40;
    DHT=0;
    waitMicrosecond(30000);
    DHT=1;
    waitMicrosecond(30);
    GPIO_PORTC_DIR_R  = 0x00;
    while (DHT==0);             //80us of sensor response
    while(DHT==1);              //80us of high voltage for sending data
    RH_Byte1 = Read();          //Relative humidity integer value
    RH_Byte2 = Read();          //Relative humidity decimal value
    T_Byte1  = Read();          //Tempature integer value
    T_Byte2  = Read();          //Tempature decimal value
    checksum = Read();          //checksum value

    // Dew point
    Td = (float)T_Byte1-((100-(float)RH_Byte1)/5);

    // BMP180 Sensor Code
    writeByte(0xAA);
    AC1 = readWord();
    writeByte(0xAC);
    AC2 = readWord();
    writeByte(0xAE);
    AC3 = readWord();
    writeByte(0xB0);
    AC4 = readWord();
    writeByte(0xB2);
    AC5 = readWord();
    writeByte(0xB4);
    AC6 = readWord();
    writeByte(0xB6);
    B1 = readWord();
    writeByte(0xB8);
    B2 = readWord();
    writeByte(0xBA);
    MB = readWord();
    writeByte(0xBC);
    MC = readWord();
    writeByte(0xBE);
    MD = readWord();

    // Read the raw Temperature & Raw Pressure Value
    rawTemp = readTemperature();
    rawPressure = readPressure(oss);

    // Calculate the True Temperature Value
    X1 = ((rawTemp - (long)AC6)*(long)AC5) >> 15;
    X2 = ((long)MC << 11)/(X1 + MD);
    B5 = X1+X2;
    T = ((double)((B5 + 8) >> 4))/10;

    // Calculate the True Pressure Value
    B6 = B5 - 4000;
    X1 = (B2*(B6*B6)>>12)>>11;
    X2 = (AC2*B6)>>11;
    X3 = X1+X2;
    B3 = (((((long)AC1)*4 + X3)<<oss) + 2)>>2;

    X1 = (AC3 * B6)>>13;
    X2 = (B1 * ((B6 * B6)>>12))>>16;
    X3 = ((X1 + X2) + 2)>>2;
    B4 = (AC4 * (unsigned long)(X3 + 32768))>>15;

    B7 = ((unsigned long)(rawPressure - B3) * (50000>>oss));
    if (B7 < 0x80000000)
    {
        P = (B7<<1)/B4;
    }
    else
    {
        P = (B7/B4)<<1;
    }
    X1 = (P>>8)*(P>>8);
    X1 = (X1 * 3038)>>16;
    X2 = (-7357 * P)>>16;
    P += (X1 + X2 + 3791)>>4;
    pInches = 0.295300*P*0.001;
    temporary = ((float)P/101325);
    altitude = 44320*(1-pow(temporary, 1/5.255));
    etherSendGraphicsLCD(data,T,P,RH_Byte1,Td, 19);   // Create UDP packet and send it to Weather Display Node
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;               // Clear Timer interrupt flag
}

// This function is used to check whether the packet received is a
// Discovery Control Packet or a Rule Control Packet
int checkUdpPacket(uint8_t* data)
{
    uint8_t* temp_ptr1;
    uint8_t* temp_ptr2;
    ctrlInfo = (void*)data;
    if(ctrlInfo->ctrlType == 0x01)
    {
        ctrlInfo->deviceType = 0x1e;
        ctrlInfo->nodeId = 0x1e;
        ctrlInfo->opCode = 0x02;
        ctrlInfo->noOfCtrlMessages = 2;
        ctrlData[0] = (void*)&ctrlInfo->data;
        temp_ptr1 = &ctrlData[0]->stringData;
        ctrlData[0]->localFunctionID = 0;
        ctrlData[0]->functionType = 0x01;
        ctrlData[0]->dataType = 0x01;
        ctrlData[0]->lengthOfFriendlyString = 0x07;
        for(index = 0; index < 7; index++)
        {
            *temp_ptr1 = name[index];
            temp_ptr1++;
        }
        ctrlData[1] = (void*)temp_ptr1;
        temp_ptr2 = &ctrlData[1]->stringData;
        ctrlData[1]->localFunctionID = 1;
        ctrlData[1]->functionType = 0x02;
        ctrlData[1]->dataType = 0x01;
        ctrlData[1]->lengthOfFriendlyString = 0x0B;
        for(index = 0; index < 11; index++)
        {
            *temp_ptr2 = information[index];
            temp_ptr2++;
        }
        return 1;
    }
    else if(ctrlInfo->ctrlType == 0x02)
    {
        ctrlInfo->opCode = 0x02;
        return 2;
    }
    return 0;
}
