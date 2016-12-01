/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Simple IO routines to control the LEDs.
 * This file is called ParTest.c for historic reasons.  Originally it stood for
 * PARallel port TEST.
 *-----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "partest.h"

/* Xilinx includes. */
#include "xgpio.h"
#include "xspips.h"
#include "xparameters.h"


// GPIO

#define GPIO_CH	(1)
#define GPIO_GND_MSK (0b1000)
#define GPIO_VCC_MSK (0b0010)

#define GPIO_IRQ_MSK   (0b0001)
#define GPIO_CE_MSK    (0b0100)
#define GPIO_CE_BIT		3
#define GPIO_IRQ_BIT	0

// ensure that VCC, GND are always 1, 0 for writes
#define GPIO_WRITE(val)	{\
		last_write |= (val | GPIO_VCC_MSK) & ~GPIO_GND_MSK;\
		XGpio_DiscreteWrite(&xGpio, GPIO_CH, last_write);\
	}

#define SET_TX_MODE()	GPIO_WRITE(last_write & ~(GPIO_CE_MSK))
#define SET_RX_MODE()	GPIO_WRITE(1 << GPIO_CE_BIT)

#define DISABLE_INT()	GPIO_WRITE(GPIO_IRQ_MSK)
#define ENABLE_INT()	GPIO_WRITE(last_write & ~0)

static XGpio xGpio;
static u8 last_write = GPIO_VCC_MSK | GPIO_IRQ_MSK; // disable interrupts from beginning

//SPI
static XSpiPs xSPI;

#define SPI_TRANSFER(writeBuf, readBuf, len) {\
		XSpiPs_SetSlaveSelect(&xSPI, 0x01);\
		XSpiPs_PolledTransfer(&xSPI,(u8 *) (writeBuf),(u8 *) (readBuf), len);\
	}
#define NOP 0xFF


void initGPIO(void)
{
	XGpio_Config *pxConfigPtr;
	BaseType_t xStatus;

	//Initialize GPIO
	pxConfigPtr = XGpio_LookupConfig( XPAR_AXI_GPIO_0_DEVICE_ID );
    xStatus = XGpio_CfgInitialize( &xGpio, pxConfigPtr, pxConfigPtr->BaseAddress );
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus;

    // Set GPIO pins to output
    XGpio_SetDataDirection(&xGpio, GPIO_CH, 0x00);
	// Set VCC to 1 and rest of pins to 0.
	XGpio_DiscreteWrite(&xGpio, GPIO_CH, last_write);
	//SET_TX_MODE();
}

void initSPI(void)
{
	XSpiPs_Config *pxConfigPtrSPI;
	BaseType_t xStatusSPI;

	// Initialize SPI
	pxConfigPtrSPI = XSpiPs_LookupConfig(XPAR_PS7_SPI_1_DEVICE_ID);
	xStatusSPI = XSpiPs_CfgInitialize(&xSPI, pxConfigPtrSPI, pxConfigPtrSPI->BaseAddress);
	configASSERT( xStatusSPI == XST_SUCCESS );

	// Set SPI Options
	XSpiPs_SetOptions(&xSPI, XSPIPS_MASTER_OPTION);
	XSpiPs_SetClkPrescaler(&xSPI, XSPIPS_CLK_PRESCALE_16); // results in 10Mhz transfer
}

void vParTestInitialise( void )
{
	initGPIO();
	initSPI();
	SET_RX_MODE();
}

u16 SPI_SEND(u16 *val)
{
	u16 readBufferSPI1;

	XSpiPs_SetSlaveSelect(&xSPI, 0x01);
	XSpiPs_PolledTransfer(&xSPI,(u8 *) val,(u8 *) &readBufferSPI1, 6);

	return readBufferSPI1;
}

void readReg(u8 addr, u8 *readBuf)
{
	u8 writeBuf[2]; // = addr & 0x1F;

	writeBuf[0] = addr & 0x1F;
	//writeBuf[1] = NOP;

	SPI_TRANSFER(writeBuf, readBuf, 2);
}

u16 writeReg(u8 addr)
{
	u16 address = (addr & 0x1f) | 0x10;
	return SPI_SEND(&address);
}

u8 writeStatusRegister(u8 reg, u8 val)
{
	u8 readBuf[2];
	u8 writeBuf[2];

	writeBuf[0] = (reg & 0x3F) | 0x20;
	writeBuf[1] = val;

	SPI_TRANSFER(writeBuf, readBuf, 2);

	return *readBuf;
}

void readRX(u8 *readBuf)
{
	u8 writeBuf = 0b0110001;
	SPI_TRANSFER(&writeBuf, readBuf, 32);
}
