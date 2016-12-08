/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo includes. */
#include "partest.h"

/* Xilinx includes. */
#include "xgpio.h"
#include "xspips.h"
#include "xparameters.h"
#include "sleep.h"
#include "assert.h"

#define MS_TO_US(time_in_ms)	((time_in_ms) * 1000)
#define DELAY_MS(time_ms)		usleep(MS_TO_US(time_ms))

// Ensure that VCC, GND are always 1, 0 for writes
#define GPIO_WRITE(val)	{\
		last_write = ((val) | GPIO_VCC_MSK) & ~GPIO_GND_MSK;\
		XGpio_DiscreteWrite(&xGpio, GPIO_CH, last_write);\
	}

#define RESET_CE()	GPIO_WRITE(last_write & ~_BV(GPIO_CE_BIT))
#define SET_CE()	GPIO_WRITE(_BV(GPIO_CE_BIT))

#define DISABLE_INT()	GPIO_WRITE(GPIO_IRQ_MSK)
#define ENABLE_INT()	GPIO_WRITE(last_write & ~GPI_IRQ_MSK)

static XGpio xGpio;
static u8 last_write = GPIO_VCC_MSK | GPIO_IRQ_MSK; // disable interrupts from beginning

// SPI
static XSpiPs xSPI;

#define SPI_TRANSFER(writeBuf, readBuf, len) {\
		XSpiPs_SetSlaveSelect(&xSPI, 0x01);\
		XSpiPs_PolledTransfer(&xSPI,(u8 *) (writeBuf),(u8 *) (readBuf), len);\
	}

// pipe addresses
u8 addresses[][6] = {"1Node","2Node"};
uint8_t pipe0_reading_address[5];
u8 payload_size = 32;
u8 radioNum = 0;

static const uint8_t child_pipe[] =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};

static const uint8_t child_payload_size[]  =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};

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
	XSpiPs_SetClkPrescaler(&xSPI, XSPIPS_CLK_PRESCALE_32); // results in 10Mhz transfer
}

u8 setup_reset(u8 delay, u8 count)
{
	return writeReg(SETUP_RETR, (delay & 0xF) << 4 | (count & 0xF));
}

// Only supports RF24_1MBPS
int setDataRate(rf24_datarate_e speed)
{
	u8 setup = getRegByte(RF_SETUP);

	if (speed == RF24_1MBPS)
	{
		setup &= ~(1 << RF_DR_LOW) | (1 << RF_DR_HIGH);
	}

	writeReg(RF_SETUP, setup);

	return setup == getRegByte(RF_SETUP);
}

// Mystery function. I don't know what this actually does but the arduino tutorial did it.
// From what I understand, it enables additional features on the RF24.
void toggle_features(void)
{
	u8 writeBuffer[2];

	writeBuffer[0] = ACTIVATE;
	writeBuffer[1] = 0x73;

	// Don't care about what gets sent back.
	SPI_TRANSFER(writeBuffer, writeBuffer, 2);
}

void setChannel(uint8_t channel)
{
	const uint8_t max_channel = 125;
	writeReg(RF_CH, min(channel, max_channel));
	getRegByte(RF_CH);
}

void flush_rx()
{
	u8 command = FLUSH_RX;

	SPI_TRANSFER(&command, &command, 1);
}

void flush_tx()
{
	u8 command = FLUSH_TX;

	SPI_TRANSFER(&command, &command, 1);
}

void powerUp()
{
	uint8_t cfg = getRegByte(NRF_CONFIG);

	// if not powered up then power up and wait for the radio to initialize
	if (!(cfg & _BV(PWR_UP))){
	  writeReg(NRF_CONFIG, cfg | _BV(PWR_UP));
	  cfg = getRegByte(NRF_CONFIG);

	  // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
	  DELAY_MS(5);
	  cfg = getRegByte(NRF_CONFIG);
	}

	// This will leave it in Standby-I mode.
	RESET_CE();
	cfg = getRegByte(NRF_CONFIG);
}

// Assumes powered up
void setTXMode()
{
	u8 config = getRegByte(NRF_CONFIG);

	assert(config & _BV(PWR_UP));

	config &= ~_BV(PRIM_RX);
	writeReg(NRF_CONFIG, config);

	SET_CE();
}

void setRXMode()
{
	u8 config = getRegByte(NRF_CONFIG);

	assert(config & _BV(PWR_UP));

	config |= _BV(PRIM_RX);
	writeReg(NRF_CONFIG, config);

	SET_CE();
}

void startListening()
{
	u8 config = getRegByte(NRF_CONFIG);

	assert(config & _BV(PWR_UP));

	writeReg(NRF_CONFIG, getRegByte(NRF_CONFIG) | _BV(PRIM_RX));
	writeReg(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	SET_CE();
	usleep(200); // time for rx to settle
	// Restore the pipe0 address, if exists

	if (pipe0_reading_address[0] > 0)
	{
		writeRegBuf(RX_ADDR_P0, pipe0_reading_address, ADDR_WIDTH);
	}
	else
	{
		closeReadingPipe(0);
	}


	// Flush buffers
	if(getRegByte(FEATURE) & _BV(EN_ACK_PAY)){
		flush_tx();
	}
}

void stopListening()
{
	RESET_CE();
	usleep(130); // in case of switching to TX

	if (getRegByte(FEATURE) & _BV(EN_ACK_PAY))
	{
		usleep(200); // tx delay
		flush_tx();
	}

	// Clear RX bit in Config reg to reset to Standby-I.
	writeReg(NRF_CONFIG, getRegByte(NRF_CONFIG) &  ~_BV(PRIM_RX));

	//writeReg(EN_RXADDR, getRegByte(EN_RXADDR) | )

}


u8 initRF24()
{
	pipe0_reading_address[0] = 0;
/*
	// Debug code for verifying that SPI works
	// TODO?: add assert for default config in SPI init
	u8 config = 0xFF;

	config = getRegByte(NRF_CONFIG);

	writeReg(NRF_CONFIG, config | 0x0C);
	config = getRegByte(NRF_CONFIG);
	(void)config;
*/
	// Must allow the radio time to settle else configuration bits will
	// not necessarily stick.
	// This is actually only required following power up
	// but some settling time also appears to be required after resets too.
	// For full coverage, we'll always assume the worst.
	// Enabling 16b CRC is by far the most obvious case if the wrong timing is used
	// - or skipped.
	// Technically we require 4.5ms + 14us as a worst case.
	// We'll just call it 5ms for good measure.
	// WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
	DELAY_MS(5);
	// Reset NRF_CONFIG and enable 16-bit CRC.
	writeReg(NRF_CONFIG, 0x0C);
	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	setup_reset(5, 15);
	setDataRate(RF24_1MBPS);
	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	toggle_features();
	writeReg(FEATURE, 0);
	writeReg(DYNPD, 0);

	// Reset current status
	// Notice reset and flush is the last thing we do
    writeReg(NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(108);

    // Flush buffers
    flush_rx();
    flush_tx();

    powerUp();
    DELAY_MS(2);

    // open up relevant pipes
    if (radioNum)
    {
        openWritingPipe(addresses[1]);
        openReadingPipe(1, addresses[0]);
    }
    else
    {
        openWritingPipe(addresses[0]);
        openReadingPipe(1, addresses[1]);
    }

    // Immediately start listening (or go into RX mode)
    startListening();

    return getRegByte(NRF_CONFIG);
}

void closeReadingPipe( uint8_t pipe )
{
	writeReg(EN_RXADDR, getRegByte(EN_RXADDR) & ~_BV(pgm_read_byte(&child_pipe_enable[pipe])));
}

void openReadingPipe(uint8_t child, uint8_t *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 1){
    memcpy(pipe0_reading_address, address, ADDR_WIDTH);
  }

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if (child < 2)
    {
      writeRegBuf(pgm_read_byte(&child_pipe[child]), address, ADDR_WIDTH);
    }
    else
    {
      writeRegBuf(pgm_read_byte(&child_pipe[child]), address, 1);
	}
    writeReg(pgm_read_byte(&child_payload_size[child]), payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    writeReg(EN_RXADDR, getRegByte(EN_RXADDR) | _BV(pgm_read_byte(&child_pipe_enable[child])));
  }
}


void openWritingPipe(uint8_t *address)
{
	// Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
	// expects it LSB first too, so we're good.
	u8 buffer[32];
	writeRegBuf(RX_ADDR_P0, address, ADDR_WIDTH);
	readRegBuf(RX_ADDR_P0, buffer, ADDR_WIDTH);
	writeRegBuf(TX_ADDR, address, ADDR_WIDTH);
	readRegBuf(TX_ADDR, buffer, ADDR_WIDTH);

	// Write payload_size of 32 (default)
	writeReg(RX_PW_P0, payload_size);
}

// Assumes len includes the extra byte for the status register that automatically
// comes in.
void read_payload(u8 *buffer, u32 len)
{
	*buffer = R_RX_PAYLOAD;
	SPI_TRANSFER(buffer, buffer, len + 1); // make room for the status reg by adding 1
}

void readRx(u8 *buf, u32 len)
{
	read_payload(buf, len);

	//Clear the two possible interrupt flags with one command
	writeReg(NRF_STATUS,_BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS) );
}

void write_payload(u8 *buf, u32 len)
{
	u8 write_buffer[32];
	u32 i;

	*write_buffer = W_TX_PAYLOAD;
	for (i = 1; i < len + 1; i++)
	{
		write_buffer[i] = buf[i - 1];
	}

	SPI_TRANSFER(write_buffer, write_buffer, len + 1);
}

void writeTx(u8 *buf, u32 len)
{
	write_payload(buf, len);
}


u8 dataAvailable()
{
	return (getRegByte(FIFO_STATUS) & _BV(RX_EMPTY)) == 0;
}

void vParTestInitialise( void )
{
	u8 val;
	initGPIO();
	initSPI();
	val = initRF24();
	(void)val;

}

void readReg(u8 addr, u8 *readBuf)
{
	u8 writeBuf[2];

	writeBuf[0] = addr & REGISTER_MASK;

	SPI_TRANSFER(writeBuf, readBuf, 2);
}


void readRegBuf(u8 reg, u8 *buf, u8 len)
{
	u8 readBuf[32];

	readBuf[0] = reg & REGISTER_MASK;
	SPI_TRANSFER(readBuf, buf, len + 1);
}


u8 getRegByte(u8 addr)
{
	u8 writeBuf[2], readBuf[2];

	writeBuf[0] = addr & 0x1F;

	SPI_TRANSFER(writeBuf, readBuf, 2);
	return readBuf[1];
}

#define WRITE_MASK(reg) (((reg) & REGISTER_MASK) | 0x20)

u8 writeReg(u8 reg, u8 val)
{
	u8 readBuf[2];
	u8 writeBuf[2];

	writeBuf[0] = WRITE_MASK(reg);
	writeBuf[1] = val;

	SPI_TRANSFER(writeBuf, readBuf, 2);

	return readBuf[1];
}

void writeRegBuf(u8 reg, u8 *buf, u8 len)
{
	u8 writeBuf[32];
	u8 i;
	writeBuf[0] = WRITE_MASK(reg);

	for (i = 1; i < len + 1; i++)
	{
		writeBuf[i] = buf[i - 1];
	}

	SPI_TRANSFER(writeBuf, NULL, len + 1);
}
