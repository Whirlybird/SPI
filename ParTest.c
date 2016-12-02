/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "partest.h"

/* Xilinx includes. */
#include "xgpio.h"
#include "xspips.h"
#include "xparameters.h"

// Ensure that VCC, GND are always 1, 0 for writes
#define GPIO_WRITE(val)	{\
		last_write |= (val | GPIO_VCC_MSK) & ~GPIO_GND_MSK;\
		XGpio_DiscreteWrite(&xGpio, GPIO_CH, last_write);\
	}

#define SET_TX_MODE()	GPIO_WRITE(last_write & ~(GPIO_CE_MSK))
#define SET_RX_MODE()	GPIO_WRITE(1 << GPIO_CE_BIT)

#define DISABLE_INT()	GPIO_WRITE(GPIO_IRQ_MSK)
#define ENABLE_INT()	GPIO_WRITE(last_write & ~GPI_IRQ_MSK)

static XGpio xGpio;
static u8 last_write = GPIO_VCC_MSK | GPIO_IRQ_MSK; // disable interrupts from beginning

//SPI
static XSpiPs xSPI;

#define SPI_TRANSFER(writeBuf, readBuf, len) {\
		XSpiPs_SetSlaveSelect(&xSPI, 0x01);\
		XSpiPs_PolledTransfer(&xSPI,(u8 *) (writeBuf),(u8 *) (readBuf), len);\
	}


void initGPIO(void)
{
	XGpio_Config *pxConfigPtr;
	BaseType_t xStatus;

	// Initialize GPIO.
	pxConfigPtr = XGpio_LookupConfig( XPAR_AXI_GPIO_0_DEVICE_ID );
    xStatus = XGpio_CfgInitialize( &xGpio, pxConfigPtr, pxConfigPtr->BaseAddress );
    configASSERT( xStatus == XST_SUCCESS );
    ( void ) xStatus;

    // Set GPIO pins to output.
    XGpio_SetDataDirection(&xGpio, GPIO_CH, 0x00);
	// Set VCC to 1 and rest of pins to 0.
	XGpio_DiscreteWrite(&xGpio, GPIO_CH, last_write);
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
	XSpiPs_SetOptions(&xSPI, XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION);
	XSpiPs_SetClkPrescaler(&xSPI, XSPIPS_CLK_PRESCALE_16); // results in 10Mhz transfer
}

void vParTestInitialise( void )
{
	initGPIO();
	initSPI();
}

void readReg(u8 addr, u8 *readBuf)
{
	u8 writeBuf[2];

	writeBuf[0] = addr & 0x1F;

	SPI_TRANSFER(writeBuf, readBuf, 2);
}

u8 writeReg(u8 reg, u8 val)
{
	u8 readBuf[2];
	u8 writeBuf[2];

	writeBuf[0] = (reg & 0x3F) | 0x20;
	writeBuf[1] = val;

	SPI_TRANSFER(writeBuf, readBuf, 2);

	return readBuf[1];
}
