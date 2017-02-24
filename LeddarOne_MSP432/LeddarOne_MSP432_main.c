/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

/*
 * This program communicates with a LeddarTech LeddarOne Sensing module using UART transmission and reception for control. The Launchpad used is the
 * MSP432 Launchpad and I used eUSCI Module A2 for the communication with the laser rangefinder while using eUSCI Module A0 for command line interfacing
 * for debugging purposes. This program combines the SDK provided by LeddarTech along with the driver library code provided by Texas Instruments. The
 * baud rate has been set for 115200 since this is the default rate for the LeddarOne.
 *
 * Author: Jordan Alexander
 */
///* DriverLib Includes */
#include "driverlib.h"
///* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
////#include <Leddar.h>
#include <string.h>
#include <Definitions.h>
#include <type.h>
#include <stdio.h>
#include <math.h>
#define ARRAYSIZE(v) (sizeof(v)/sizeof(v[0]))
#define SLAVE_ADDRESS 0x01


//CRC high value for calculating the CRC checksum
//TO-DO: Hoping to incorporate TI's CRC code that makes use of the CRC module.
static uint8_t CRC_HI[] =
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

//CRC low value
static uint8_t CRC_LO[] =
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

//GLOBAL VAR
uint8_t dataBuffer[25];
uint32_t recCount =0;

//code to do the 16 bit CRC16 check
bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck)
{
	uint8_t lCRCHi = 0xFF; // high uint8_t of CRC initialized
	uint8_t lCRCLo = 0xFF; // low uint8_t of CRC initialized
	int i;

	for (i = 0; i<aLength; ++i)
	{
		int lIndex = lCRCLo ^ aBuffer[i]; // calculate the CRC
		lCRCLo = lCRCHi ^ CRC_HI[lIndex];
		lCRCHi = CRC_LO[lIndex];
	}

	if (aCheck)
	{
		return ( aBuffer[aLength] == lCRCLo ) && ( aBuffer[aLength+1] == lCRCHi );
	}
	else
	{
		aBuffer[aLength] = lCRCLo;
		aBuffer[aLength+1] = lCRCHi;
		return true;
	}
};




/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        6,                                     // BRDIV = 78 for9600 6 for 115200
        8,                                       // UCxBRF = 2 8
        32,                                       // UCxBRS = 0 32
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};






int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    /* Setting DCO to 12MHz */
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);


    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);


    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableModule(EUSCI_A2_BASE);


    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);

    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    //recpetion count variable;
    recCount = 0;
    //sending for the first capture
    sendRequest();

    while(1)
    {
        //goes into low power mode if nothing is happening
    	MAP_PCM_gotoLPM0();
    }
}

/* EUSCI A0 UART ISR - I do not currently use the A0 Interrupt service but it was left for debugging purposes */
void EUSCIA0_IRQHandler(void)
{
//    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
//    //uint32_t status1 = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
//
//    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
//    //MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status1);
//
//    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
//    {
//        MAP_UART_transmitData(EUSCI_A0_BASE, MAP_UART_receiveData(EUSCI_A0_BASE));
//    }

}


//eUSCI A2 ISR
void EUSCIA2_IRQHandler(void)
{
    //uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    uint32_t status1 = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    //MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status1);

    //See if interrupts are enabled and if the flag has been marked
    if(status1 && EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //If the capture is over 25 signal for a bad response
    	if(recCount >= 25)return ERR_LEDDAR_BAD_RESPONSE;
    		//fill a buffer with data from the UART module
        	dataBuffer[recCount] = MAP_UART_receiveData(EUSCI_A2_BASE);
        	recCount++;
        	if (recCount == 25)
        	{
        		//Parse the data from the laser
        		parseResponse(recCount);

        		//delay to ensure that the MCU does not send a request before the laser is ready to capture again
        		_delay_cycles(10000);
        		//this toggles an led to show there was a capture
        		//MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        		clearDetections();
        		sendRequest();
        	}
    }
}


/*
 * sendRequest() calls for capture from the laser module
 * A buffer(dataBuffer) is filled with the values to tell the laser to capture and send data back
 */
void sendRequest(){


		unsigned int i = 0;

		// Send message on UART:
		dataBuffer[0] =  SLAVE_ADDRESS;
		dataBuffer[1] =  0x04;
		dataBuffer[2] =  0;
		dataBuffer[3] =  20;
		dataBuffer[4] =  0;
		dataBuffer[5] =  10;
		CRC16(dataBuffer, 6, false);

		for (i = 0; i<8; i++)
		{	//transmit data over A2 UART module
			MAP_UART_transmitData(EUSCI_A2_BASE,(dataBuffer[i]));
		}
}





char parseResponse(uint32_t recCount1)
{
	unsigned int crc = 0xFFFF;
	//uint8_t dataBuffer[25] = {0};
	unsigned int i = 0;
	unsigned int len = recCount1;

	unsigned int detcount = 0;

	if (len == 25 && dataBuffer[1] == 0x04 && dataBuffer[0] == SLAVE_ADDRESS)
	{
		// Check CRC
		if (!CRC16(dataBuffer, len-2, true))
		{
			return ERR_LEDDAR_BAD_CRC; //invalid CRC
		}

		NbDet = dataBuffer[10];

		if (NbDet > ARRAYSIZE(Detections))
		{
			return ERR_LEDDAR_NB_DETECTIONS;
		}

		//Timestamp
	    TimeStamp = ((unsigned long)dataBuffer[5]) << 24;
		TimeStamp += ((unsigned long)dataBuffer[6]) << 16;
		TimeStamp += ((unsigned long)dataBuffer[3]) <<8;
		TimeStamp += dataBuffer[4];



		// Internal Temperature
		Temperature = dataBuffer[7];
		Temperature += ((float)dataBuffer[8])/256;

		i = 11;
		for (detcount = 0; detcount < NbDet; detcount++)
		{
			// For each detection:
			// Bytes 0 and 1 = distance in cm
			// Bytes 2-3 are amplitude*256


			Detections[detcount].Distance = ((unsigned int)dataBuffer[i])*256 + dataBuffer[i+1];
			//MAP_UART_transmitData(EUSCI_A0_BASE, Detections[detcount].Distance);
			Detections[detcount].Amplitude = ((float)dataBuffer[i+2]) + ((float)dataBuffer[i+3])/256;

			i += 4;
		}

		//Debugging prints. This will not remain because data will be sent over TCP
		printf("Temperature: %f\r\n", Temperature);
		printf("TimeStamp: %d\r\n", TimeStamp);
		printf("Distance: %d\r\n", Detections[0].Distance);
		printf("Amplitude: %d\r\n", Detections[0].Amplitude);

		return NbDet;
	}
	else{
		return ERR_LEDDAR_BAD_RESPONSE; // Invalid response
	}
}

	/*
	 *unused method from the SDK. I did translate it for use with MSP432.
	 */

	char getDetections()
	{

		_delay_cycles(200000);


		clearDetections();
		sendRequest();
		_delay_cycles(200000);

		//return parseResponse();

	}





	/*
	 * clears the detections in preparation for the next capture
	 */
	void clearDetections()
	{
		memset(Detections, 0, sizeof Detections);
		recCount = 0;
	}

	/*This method converts a number into an array. This was used to try to avoid using compute intensive printf method.
	*However for debugging printf works fine
	*/
	char * convertNumberIntoArray(unsigned int number) {
	    unsigned int length = (int)(log10((float)number)) + 1;
	    char * arr = (char *) malloc(length * sizeof(char)), * curr = arr;
	    do {
	        *curr++ = number % 10;
	        number /= 10;
	    } while (number != 0);
	    return arr;
	}







