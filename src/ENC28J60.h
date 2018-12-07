/*****************************************************************************
*
* Title        : Microchip ENC28J60 Ethernet Interface Driver
* Author        : Pascal Stang (c)2005
* Modified by Norbert Truchsess
* Copyright: GPL V2
*
*This driver provides initialization and transmit/receive
*functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
*This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
*chip, using an SPI interface to the host processor.
*
*
*****************************************************************************/

#ifndef ENC28J60_H
#define ENC28J60_H
#include <inttypes.h>
#include <arduino.h>
#include <SPI.h>
#include "util.h"

/*
 * ENC28J60 Control Registers
 * Control register definitions are a combination of address,
 * bank number, and Ethernet/MAC/PHY indicator bits.
 * - Register address	(bits 0-4)
 * - Bank number	(bits 5-6)
 * - MAC/MII indicator	(bit 7)
 */
#define ADDR_MASK	0x1F
#define BANK_MASK	0x60
#define SPRD_MASK	0x80
/* All-bank registers */
#define EIE		0x1B
#define EIR		0x1C
#define ESTAT		0x1D
#define ECON2		0x1E
#define ECON1		0x1F
/* Bank 0 registers */
#define ERDPTL		(0x00|0x00)
#define ERDPTH		(0x01|0x00)
#define EWRPTL		(0x02|0x00)
#define EWRPTH		(0x03|0x00)
#define ETXSTL		(0x04|0x00)
#define ETXSTH		(0x05|0x00)
#define ETXNDL		(0x06|0x00)
#define ETXNDH		(0x07|0x00)
#define ERXSTL		(0x08|0x00)
#define ERXSTH		(0x09|0x00)
#define ERXNDL		(0x0A|0x00)
#define ERXNDH		(0x0B|0x00)
#define ERXRDPTL	(0x0C|0x00)
#define ERXRDPTH	(0x0D|0x00)
#define ERXWRPTL	(0x0E|0x00)
#define ERXWRPTH	(0x0F|0x00)
#define EDMASTL		(0x10|0x00)
#define EDMASTH		(0x11|0x00)
#define EDMANDL		(0x12|0x00)
#define EDMANDH		(0x13|0x00)
#define EDMADSTL	(0x14|0x00)
#define EDMADSTH	(0x15|0x00)
#define EDMACSL		(0x16|0x00)
#define EDMACSH		(0x17|0x00)
/* Bank 1 registers */
#define EHT0		(0x00|0x20)
#define EHT1		(0x01|0x20)
#define EHT2		(0x02|0x20)
#define EHT3		(0x03|0x20)
#define EHT4		(0x04|0x20)
#define EHT5		(0x05|0x20)
#define EHT6		(0x06|0x20)
#define EHT7		(0x07|0x20)
#define EPMM0		(0x08|0x20)
#define EPMM1		(0x09|0x20)
#define EPMM2		(0x0A|0x20)
#define EPMM3		(0x0B|0x20)
#define EPMM4		(0x0C|0x20)
#define EPMM5		(0x0D|0x20)
#define EPMM6		(0x0E|0x20)
#define EPMM7		(0x0F|0x20)
#define EPMCSL		(0x10|0x20)
#define EPMCSH		(0x11|0x20)
#define EPMOL		(0x14|0x20)
#define EPMOH		(0x15|0x20)
#define EWOLIE		(0x16|0x20)
#define EWOLIR		(0x17|0x20)
#define ERXFCON		(0x18|0x20)
#define EPKTCNT		(0x19|0x20)
/* Bank 2 registers */
#define MACON1		(0x00|0x40|SPRD_MASK)
/* #define MACON2	(0x01|0x40|SPRD_MASK) */
#define MACON3		(0x02|0x40|SPRD_MASK)
#define MACON4		(0x03|0x40|SPRD_MASK)
#define MABBIPG		(0x04|0x40|SPRD_MASK)
#define MAIPGL		(0x06|0x40|SPRD_MASK)
#define MAIPGH		(0x07|0x40|SPRD_MASK)
#define MACLCON1	(0x08|0x40|SPRD_MASK)
#define MACLCON2	(0x09|0x40|SPRD_MASK)
#define MAMXFLL		(0x0A|0x40|SPRD_MASK)
#define MAMXFLH		(0x0B|0x40|SPRD_MASK)
#define MAPHSUP		(0x0D|0x40|SPRD_MASK)
#define MICON		(0x11|0x40|SPRD_MASK)
#define MICMD		(0x12|0x40|SPRD_MASK)
#define MIREGADR	(0x14|0x40|SPRD_MASK)
#define MIWRL		(0x16|0x40|SPRD_MASK)
#define MIWRH		(0x17|0x40|SPRD_MASK)
#define MIRDL		(0x18|0x40|SPRD_MASK)
#define MIRDH		(0x19|0x40|SPRD_MASK)
/* Bank 3 registers */
#define MAADR1		(0x00|0x60|SPRD_MASK)
#define MAADR0		(0x01|0x60|SPRD_MASK)
#define MAADR3		(0x02|0x60|SPRD_MASK)
#define MAADR2		(0x03|0x60|SPRD_MASK)
#define MAADR5		(0x04|0x60|SPRD_MASK)
#define MAADR4		(0x05|0x60|SPRD_MASK)
#define EBSTSD		(0x06|0x60)
#define EBSTCON		(0x07|0x60)
#define EBSTCSL		(0x08|0x60)
#define EBSTCSH		(0x09|0x60)
#define MISTAT		(0x0A|0x60|SPRD_MASK)
#define EREVID		(0x12|0x60)
#define ECOCON		(0x15|0x60)
#define EFLOCON		(0x17|0x60)
#define EPAUSL		(0x18|0x60)
#define EPAUSH		(0x19|0x60)
/* PHY registers */
#define PHCON1		0x00
#define PHSTAT1		0x01
#define PHHID1		0x02
#define PHHID2		0x03
#define PHCON2		0x10
#define PHSTAT2		0x11
#define PHIE		0x12
#define PHIR		0x13
#define PHLCON		0x14

/* ENC28J60 EIE Register Bit Definitions */
#define EIE_INTIE	0x80
#define EIE_PKTIE	0x40
#define EIE_DMAIE	0x20
#define EIE_LINKIE	0x10
#define EIE_TXIE	0x08
/* #define EIE_WOLIE	0x04 (reserved) */
#define EIE_TXERIE	0x02
#define EIE_RXERIE	0x01
/* ENC28J60 EIR Register Bit Definitions */
#define EIR_PKTIF	0x40
#define EIR_DMAIF	0x20
#define EIR_LINKIF	0x10
#define EIR_TXIF	0x08
/* #define EIR_WOLIF	0x04 (reserved) */
#define EIR_TXERIF	0x02
#define EIR_RXERIF	0x01
/* ENC28J60 ESTAT Register Bit Definitions */
#define ESTAT_INT	0x80
#define ESTAT_LATECOL	0x10
#define ESTAT_RXBUSY	0x04
#define ESTAT_TXABRT	0x02
#define ESTAT_CLKRDY	0x01
/* ENC28J60 ECON2 Register Bit Definitions */
#define ECON2_AUTOINC	0x80
#define ECON2_PKTDEC	0x40
#define ECON2_PWRSV	0x20
#define ECON2_VRPS	0x08
/* ENC28J60 ECON1 Register Bit Definitions */
#define ECON1_TXRST	0x80
#define ECON1_RXRST	0x40
#define ECON1_DMAST	0x20
#define ECON1_CSUMEN	0x10
#define ECON1_TXRTS	0x08
#define ECON1_RXEN	0x04
#define ECON1_BSEL1	0x02
#define ECON1_BSEL0	0x01
#define ECON1_BSEL_MASK (ECON1_BSEL1 | ECON1_BSEL0)
/* ENC28J60 MACON1 Register Bit Definitions */
#define MACON1_LOOPBK	0x10
#define MACON1_TXPAUS	0x08
#define MACON1_RXPAUS	0x04
#define MACON1_PASSALL	0x02
#define MACON1_MARXEN	0x01
/* ENC28J60 MACON2 Register Bit Definitions */
#define MACON2_MARST	0x80
#define MACON2_RNDRST	0x40
#define MACON2_MARXRST	0x08
#define MACON2_RFUNRST	0x04
#define MACON2_MATXRST	0x02
#define MACON2_TFUNRST	0x01
/* ENC28J60 MACON3 Register Bit Definitions */
#define MACON3_PADCFG2	0x80
#define MACON3_PADCFG1	0x40
#define MACON3_PADCFG0	0x20
#define MACON3_TXCRCEN	0x10
#define MACON3_PHDRLEN	0x08
#define MACON3_HFRMLEN	0x04
#define MACON3_FRMLNEN	0x02
#define MACON3_FULDPX	0x01
/* ENC28J60 MACON4 Register Bit Definitions */
#define MACON4_DEFER    0x40
/* ENC28J60 MICMD Register Bit Definitions */
#define MICMD_MIISCAN	0x02
#define MICMD_MIIRD	0x01
/* ENC28J60 MISTAT Register Bit Definitions */
#define MISTAT_NVALID	0x04
#define MISTAT_SCAN	0x02
#define MISTAT_BUSY	0x01
/* ENC28J60 ERXFCON Register Bit Definitions */
#define ERXFCON_UCEN	0x80
#define ERXFCON_ANDOR	0x40
#define ERXFCON_CRCEN	0x20
#define ERXFCON_PMEN	0x10
#define ERXFCON_MPEN	0x08
#define ERXFCON_HTEN	0x04
#define ERXFCON_MCEN	0x02
#define ERXFCON_BCEN	0x01

/* ENC28J60 PHY PHCON1 Register Bit Definitions */
#define PHCON1_PRST	0x8000
#define PHCON1_PLOOPBK	0x4000
#define PHCON1_PPWRSV	0x0800
#define PHCON1_PDPXMD	0x0100
/* ENC28J60 PHY PHSTAT1 Register Bit Definitions */
#define PHSTAT1_PFDPX	0x1000
#define PHSTAT1_PHDPX	0x0800
#define PHSTAT1_LLSTAT	0x0004
#define PHSTAT1_JBSTAT	0x0002
/* ENC28J60 PHY PHSTAT2 Register Bit Definitions */
#define PHSTAT2_TXSTAT	(1 << 13)
#define PHSTAT2_RXSTAT	(1 << 12)
#define PHSTAT2_COLSTAT	(1 << 11)
#define PHSTAT2_LSTAT	(1 << 10)
#define PHSTAT2_DPXSTAT	(1 << 9)
#define PHSTAT2_PLRITY	(1 << 5)
/* ENC28J60 PHY PHCON2 Register Bit Definitions */
#define PHCON2_FRCLINK	0x4000
#define PHCON2_TXDIS	0x2000
#define PHCON2_JABBER	0x0400
#define PHCON2_HDLDIS	0x0100
/* ENC28J60 PHY PHIE Register Bit Definitions */
#define PHIE_PLNKIE	(1 << 4)
#define PHIE_PGEIE	(1 << 1)
/* ENC28J60 PHY PHIR Register Bit Definitions */
#define PHIR_PLNKIF	(1 << 4)
#define PHIR_PGEIF	(1 << 1)

/* ENC28J60 Packet Control Byte Bit Definitions */
#define PKTCTRL_PHUGEEN		0x08
#define PKTCTRL_PPADEN		0x04
#define PKTCTRL_PCRCEN		0x02
#define PKTCTRL_POVERRIDE	0x01

/* ENC28J60 Transmit Status Vector */
#define TSV_TXBYTECNT		0
#define TSV_TXCOLLISIONCNT	16
#define TSV_TXCRCERROR		20
#define TSV_TXLENCHKERROR	21
#define TSV_TXLENOUTOFRANGE	22
#define TSV_TXDONE		23
#define TSV_TXMULTICAST		24
#define TSV_TXBROADCAST		25
#define TSV_TXPACKETDEFER	26
#define TSV_TXEXDEFER		27
#define TSV_TXEXCOLLISION	28
#define TSV_TXLATECOLLISION	29
#define TSV_TXGIANT		30
#define TSV_TXUNDERRUN		31
#define TSV_TOTBYTETXONWIRE	32
#define TSV_TXCONTROLFRAME	48
#define TSV_TXPAUSEFRAME	49
#define TSV_BACKPRESSUREAPP	50
#define TSV_TXVLANTAGFRAME	51

#define TSV_SIZE		7
#define TSV_BYTEOF(x)		((x) / 8)
#define TSV_BITMASK(x)		(1 << ((x) % 8))
#define TSV_GETBIT(x, y)	(((x)[TSV_BYTEOF(y)] & TSV_BITMASK(y)) ? 1 : 0)

/* ENC28J60 Receive Status Vector */
#define RSV_RXLONGEVDROPEV	16
#define RSV_CARRIEREV		18
#define RSV_CRCERROR		20
#define RSV_LENCHECKERR		21
#define RSV_LENOUTOFRANGE	22
#define RSV_RXOK		    23
#define RSV_RXMULTICAST		24
#define RSV_RXBROADCAST		25
#define RSV_DRIBBLENIBBLE	26
#define RSV_RXCONTROLFRAME	27
#define RSV_RXPAUSEFRAME	28
#define RSV_RXUNKNOWNOPCODE	29
#define RSV_RXTYPEVLAN		30
#define RSV_ZERO		    31

#define RSV_SIZE		6
#define RSV_BITMASK(x)		(1 << ((x) - 16))
#define RSV_GETBIT(x, y)	(((x) & RSV_BITMASK(y)) ? 1 : 0)

// SPI operation codes
#define ENC28J60_READ_CTRL_REG       0x00
#define ENC28J60_READ_BUF_MEM        0x3A
#define ENC28J60_WRITE_CTRL_REG      0x40
#define ENC28J60_WRITE_BUF_MEM       0x7A
#define ENC28J60_BIT_FIELD_SET       0x80
#define ENC28J60_BIT_FIELD_CLR       0xA0
#define ENC28J60_SOFT_RESET          0xFF


// The RXSTART_INIT should be zero. See Rev. B4 Silicon Errata
// buffer boundaries applied to internal 8K ram
// the entire available packet buffer space is allocated
//
// start with recbuf at 0/
#define RXSTART_INIT     0x0
// receive buffer end. make sure this is an odd value ( See Rev. B1,B4,B5,B7 Silicon Errata 'Memory (Ethernet Buffer)')
#define RXSTOP_INIT      (0x1FFF-0x1000) /* = 4095, 0x0fff */
// #define RXSTOP_INIT      (0x1FFF-0x1800) /* = 2047, 0x07ff */
// start TX buffer RXSTOP_INIT+1
#define TXSTART_INIT     (RXSTOP_INIT+1)
// stp TX buffer at end of mem
#define TXSTOP_INIT      0x1FFF
//
// max frame length which the conroller will accept:
#define MAX_FRAMELEN        1500  // (0x05dc, note: maximum ethernet frame length would be 1518)
#define RSV_LEN 4 // length of recieve status vector in bytes



/* Preferred half duplex: LEDA: Link status LEDB: Rx/Tx activity */
#define ENC28J60_LAMPS_MODE	0x3476

/**
 * Timing
 */

#define RESET_DELAY 2000

#define POLL_TIMEOUT 20

#define FULL_DUPLEX 1

/**
 * Constants
 */

#define MAC_BYTES 6
#define ETH_HEADER_BYTES (MAC_BYTES * 2 + 2)

#define DEBUG_ETH 0
#define DEBUG_ETH_BASIC 0

// Maximum sequence number
#define MAX_SEQUENCE 0xffff

#define DEBUG_PREFIX \
Serial.print(F(" | SPCR is 0x")); \
Serial.print(SPCR, HEX)



void enc_op_write(byte op, byte arg, byte data);
byte enc_op_read(uint8_t op, uint8_t arg);
void enc_soft_reset();
void enc_bank_sel(byte reg);
byte enc_read_reg(byte reg);
uint16_t enc_read_regw(byte reg);
void enc_write_reg(byte reg, byte value);
void enc_write_regw(byte reg, uint16_t value);
void enc_bit_set(byte reg, byte bits);
void enc_bit_clr(byte reg, byte bits);
void enc_write_buf(byte * data, int len);
void enc_read_buf(byte * data, int len);
byte enc_read_buf_b();
uint16_t enc_read_buf_w();
void enc_set_mac_addr(byte * mac_addr);
void enc_reg_print(byte reg);
void enc_regs_print(byte reg, int n_regs);
void enc_regs_debug();
int enc_hw_init();
void enc_hw_enable();
void enc_hw_disable();
void enc_peek_buf(int offset, int len);
void _enc_dump_pkt(int bcnt);
void _enc_refresh_rsv_globals();
void _enc_print_rxstat(uint16_t rxstat);
uint16_t _erxrdpt_workaround(uint16_t erxrdpt, uint16_t start, uint16_t end);
int _bounded_distance(int from, int to, int bst, int bnd);
uint16_t _buffer_sum(int start, int offset);
uint16_t _buffer_distance(int from, int to);
void free_packet();
void enc_peek_buf_slow(int offset, int len);

extern uint16_t g_enc_npp;
extern uint16_t g_enc_rxstat;
extern byte * g_enc_rsv;
extern uint16_t g_enc_rxbcnt;
extern byte * g_enc_eth_frame_buf;
extern uint16_t g_enc_sequence;
extern long g_enc_pkts_consumed;
extern bool g_enc_repeat_breakpoints;
extern bool g_enc_debug_io;

enum enc_err{
    ENC_NO_ERR,
    ENC_ERR_OP,
    ENC_ERR_ARG,
    ENC_ERR_BANK,
    ENC_ERR_RST,
};

extern enc_err g_enc_err;

#endif
