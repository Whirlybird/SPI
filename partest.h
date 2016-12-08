#ifndef PARTEST_H
#define PARTEST_H

#include "xgpio.h"
#include "xparameters.h"

/* Xilinx includes. */
#include "xgpio.h"
#include "xspips.h"
#include "xparameters.h"

#define _BV(bit) (1 << (bit))
// GPIO
#define GPIO_CH	(1)
#define GPIO_GND_MSK (0b1000)
#define GPIO_VCC_MSK (0b0010)

#define GPIO_IRQ_MSK   (0b0001)
#define GPIO_CE_MSK    (0b0100)
#define GPIO_CE_BIT		2
#define GPIO_IRQ_BIT	0

// RF24
#define ADDR_WIDTH 5
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;


/* Memory Map */
#define NRF_CONFIG  0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09
#define W_TX_PAYLOAD_NO_ACK  0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

// Misc

//#define write_register 	writeReg
#define min(a, b)		((a) < (b) ? (a) : (b))
#define pgm_read_byte(p) (*(p))


void vParTestInitialise( void );
void readReg(u8 addr, u8 *readBuf);
u8 getRegByte(u8 addr);
u8 writeReg(u8 reg, u8 val);
u8 dataAvailable();
void readRx(u8 *buf, u32 len);
void writeTx(u8 *buf, u32 len);
void stopListening();
void startListening();
extern u8 addresses[][6];
void writeRegBuf(u8 reg, u8 *buf, u8 len);
void closeReadingPipe(uint8_t pipe);
void openReadingPipe(uint8_t child, uint8_t *address);
void openWritingPipe(uint8_t *address);
void readRegBuf(u8 reg, u8 *buf, u8 len);


#endif
