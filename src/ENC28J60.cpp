#include "ENC28J60.h"
#include "enc_spi.h"

/**
 * ENC specific stuff
 */

/**
 * Temporary Recieve Status Vector storage
 * (including Next Packet Pointer because that's how Linux does it)
 */
byte * g_enc_rsv = (byte *) malloc(RSV_SIZE * sizeof(byte));
byte * g_enc_eth_frame_buf = (byte *) malloc(MAX_FRAMELEN);
uint16_t g_enc_npp = RXSTART_INIT;
uint16_t g_enc_rxstat = 0;
uint16_t g_enc_rxbcnt = 0;
enc_err g_enc_err = ENC_NO_ERR;
byte g_enc_series = 0;

inline byte _first_byte(byte op, byte arg) {
    // assembe first byte in operation from opcode and argument
    return op | (arg & ADDR_MASK);
}

void enc_op_write(byte op, byte arg, byte data) {
    // Perform a SPI operation on the ENC28J60 which has a single byte data argument
    // and does not expect reply
    // Supported ops : WCR, WBM, BFS, BFC
    if(
        (ENC28J60_WRITE_CTRL_REG != op) &
        (ENC28J60_WRITE_BUF_MEM != op) &
        (ENC28J60_BIT_FIELD_SET != op) &
        (ENC28J60_BIT_FIELD_CLR != op)
    ) {
        Serial.print(F("ERR: op"));
        Serial.print(op);
        Serial.println(F(" should not expect reply, and require data argument"));
        g_enc_err = ENC_ERR_OP;
        return;
    }

    if(
        (arg & SPRD_MASK) & (
            (ENC28J60_BIT_FIELD_SET == op)
            || (ENC28J60_BIT_FIELD_CLR == op)
        )
    ) {
        Serial.print(F("ERR: op"));
        Serial.print(op);
        Serial.println(F(" not available for MAC or MII registers"));
        g_enc_err = ENC_ERR_OP;
        return;
    }

    do {

        _beginTransaction(spiSettings);
        _spiWrite(_first_byte(op, arg));
        _spiWrite(data);
        if(arg & SPRD_MASK) {
            // extra hold time for MAC/MII commands
            delayMicroseconds(CS_HOLD_M_US);
        } else {
            delayMicroseconds(CS_HOLD_US);
        }
        _endTransaction();

        #if REPEAT_BREAKPOINTS
        if (SOMETIMES_PRINT_COND)
        #else
        if (DEBUG_OP_RW)
        #endif
        {
            Serial.print(F("SPCR is 0x"));
            Serial.print(SPCR, HEX);
            if(ENC28J60_WRITE_CTRL_REG == op) {
                Serial.print(F(" | writing 0x"));
            }
            else if(ENC28J60_BIT_FIELD_SET == op) {
                Serial.print(F(" | setting mask 0x"));
            }
            else if(ENC28J60_BIT_FIELD_CLR == op) {
                Serial.print(F(" | clearing mask 0x"));
            }
            Serial.print(data, HEX);
            Serial.print(F(" in arg "));
            Serial.print(arg & ADDR_MASK, HEX);
            Serial.print(F(" with "));
            Serial.println(op, HEX);
            SOMETIMES_PRINT_END;
        }
    }
    #if REPEAT_BREAKPOINTS
        while (!Serial.available());
        while (Serial.available()){Serial.read();delay(1);}
    #else
        while (0);
    #endif

}

byte enc_op_read(uint8_t op, uint8_t arg) {
    // Perform a SPI operation on the ENC28J60 which does not have a data arg
    // and expects reply
    if(
        (ENC28J60_READ_CTRL_REG != op) &
        (ENC28J60_READ_BUF_MEM != op)
    ) {
        Serial.print(F("ERR: op"));
        Serial.print(op);
        Serial.println(F(" should expect reply and require data argument"));
        g_enc_err = ENC_ERR_OP;
        return 0;
    }

    byte result = 0x00;

    do {
        _beginTransaction(spiSettings);
        _spiWrite(_first_byte(op, arg));
        // handle dummy byte for certain RCRs
        if(arg & SPRD_MASK) _spiRead();
        // clock in response
        result = _spiRead();
        _endTransaction();

        #if REPEAT_BREAKPOINTS
        if (SOMETIMES_PRINT_COND)
        #else
        if (DEBUG_OP_RW)
        #endif
        {
            Serial.print(F("SPCR is 0x"));
            Serial.print(SPCR, HEX);
            Serial.print(F(" | reading 0x"));
            Serial.print(result, HEX);
            Serial.print(F(" from arg "));
            Serial.print(arg & ADDR_MASK, HEX);
            Serial.print(F(" with "));
            Serial.println(op, HEX);
            SOMETIMES_PRINT_END;
        }
    }
    #if REPEAT_BREAKPOINTS
        while (!Serial.available());
        while (Serial.available()){Serial.read();delay(1);}
    #else
        while (0);
    #endif

    return result;
}

int poll_ready(byte reg, byte mask, byte val) {
    long poll_start = millis();
    while((enc_read_reg(reg) & mask) != val) {
        if( (millis() - poll_start) > POLL_TIMEOUT ) {
            return 1;
        }
    }
    return 0;
}

void enc_soft_reset() {
    // Send a soft reset command and wait
    Serial.println(F("RESET"));

    // Errata 19.
    enc_op_write(ENC28J60_BIT_FIELD_CLR, ECON2, ECON2_PWRSV);
    delayMicroseconds(600);

    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_SOFT_RESET);
    delayMicroseconds(RESET_DELAY);
    _endTransaction();
    /* Fix for errata #2: CLKRDY set early */
    delayMicroseconds(RESET_DELAY);


    if (poll_ready(ESTAT, ESTAT_RXBUSY, 0)) {
        Serial.println(F("WARN: ESTAT_RXBUSY should be clear after reset"));
    } else if (poll_ready(ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY)) {
        Serial.println(F("WARN: ESTAT_CLKRDY should be set after reset"));
    } else {
        Serial.println(F("RESET succesful"));
    }
}

/**
 * Select the correct bank in order to access register, if not already selected
 *  - reg: The register definition, with bank encoded in BANK_MASK bits
 */
void enc_bank_sel(byte reg) {

    if((reg & ADDR_MASK) > 0x1A) {
        // Is an all-bank register, selected bank does not matter.
        return;
    }

    byte bsel = (reg & BANK_MASK) >> 5;
    byte econ1 = enc_op_read(ENC28J60_READ_CTRL_REG, ECON1);
    // Serial.print(F("ECON1: 0x"));
    // Serial.println(econ1, HEX);

    if ((ECON1_BSEL_MASK & (econ1 ^ bsel))) {
        if (DEBUG_OP_RW) {
            Serial.print(F("SPCR is 0x"));
            Serial.print(SPCR, HEX);
            Serial.print(F(" | changing bank from "));
            Serial.print(econ1 & ECON1_BSEL_MASK, HEX);
            Serial.print(F(" to "));
            Serial.println(bsel & ECON1_BSEL_MASK, HEX);
        }
    } else {
        return;
    }

    // bits which need to be set in ECON1.bsel
    byte set_bits = ECON1_BSEL_MASK & (~econ1 & bsel);
    // Serial.print(F("set bits: 0x"));
    // Serial.println(set_bits, HEX);

    if( set_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_SET, ECON1, set_bits);
    }


    // bits which need to be cleared in ECON1.bsel
    byte clear_bits = ECON1_BSEL_MASK & (econ1 & ~bsel);
    // Serial.print(F("clear bits: 0x"));
    // Serial.println(clear_bits, HEX);
    if( clear_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_CLR, ECON1, clear_bits);
    }

    econ1 = enc_op_read(ENC28J60_READ_CTRL_REG, ECON1);

    if (ECON1_BSEL_MASK & (econ1 ^ bsel)) {
        Serial.println(F("ERR: Bank select failed!"));
        g_enc_err = ENC_ERR_BANK;
        return;
    }

}

/**
 * Read a byte from a register on the ENC
 */
byte enc_read_reg(byte reg) {
    // TODO: cache registers?
    enc_bank_sel(reg);
    return enc_op_read(ENC28J60_READ_CTRL_REG, reg);
}

/**
 * Read a word from a register pair on the ENC
 */
uint16_t enc_read_regw(byte reg) {
    enc_bank_sel(reg);
    uint16_t result = enc_op_read(ENC28J60_READ_CTRL_REG, reg+1);
    result = result << 8;
    result |= enc_op_read(ENC28J60_READ_CTRL_REG, reg);
    return result;
}

/**
 * Write a byte to a register on the ENC
 */
void enc_write_reg(byte reg, byte value) {
    // TODO: update cache?
    enc_bank_sel(reg);
    enc_op_write(ENC28J60_WRITE_CTRL_REG, reg, value);
}

/**
 * write a word to a register pair on the ENC
 */
void enc_write_regw(byte reg, uint16_t value) {
    enc_bank_sel(reg);
    enc_op_write(ENC28J60_WRITE_CTRL_REG, reg, (byte)(value & 0xff));
    enc_op_write(ENC28J60_WRITE_CTRL_REG, reg+1, (byte)((value & 0xff00 ) >> 8));
}


/**
 *  Set a bit field
 */
void enc_bit_set(byte reg, byte bits) {
    enc_bank_sel(reg);
    enc_op_write(ENC28J60_BIT_FIELD_SET, reg, bits);
}

/**
 *  Set a bit field
 */
void enc_bit_clr(byte reg, byte bits) {
    enc_bank_sel(reg);
    enc_op_write(ENC28J60_BIT_FIELD_CLR, reg, bits);
}

/**
 * Write to the buffer
 */
void enc_write_buf(byte * data, uint8_t len) {
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_WRITE_BUF_MEM);
    for(uint8_t i=0; i<len; i++){
        _spiWrite(data[i]);
    }
    _endTransaction();
}

void enc_read_buf(byte * data, uint8_t len) {
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_READ_BUF_MEM);
    byte result;
    for(uint8_t i=0; i<len; i++){
        result = _spiRead();
        if(data) {data[i] = result;}
    }
    _endTransaction();
}

/**
 * Read single byte from buffer
 */
byte enc_read_buf_b() {
    byte result;
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_READ_BUF_MEM);
    result = _spiRead();
    _endTransaction();
    return result;
}

/**
 * Read single 16b word from buffer (LSB first)
 */
uint16_t enc_read_buf_w() {
    union {uint16_t val; struct {byte lsb, msb;};} word;
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_READ_BUF_MEM);
    word.lsb = _spiRead();
    word.msb = _spiRead();
    _endTransaction();
    return word.val;
}

void enc_set_mac_addr(byte * mac_addr) {
    /* NOTE: MAC address in ENC28J60 is byte-backward */
    enc_write_reg(MAADR5, mac_addr[0]);
    enc_write_reg(MAADR4, mac_addr[1]);
    enc_write_reg(MAADR3, mac_addr[2]);
    enc_write_reg(MAADR2, mac_addr[3]);
    enc_write_reg(MAADR1, mac_addr[4]);
    enc_write_reg(MAADR0, mac_addr[5]);
}

/*
 * Wait until the PHY operation is complete.
 */
static bool wait_phy_ready() {
    return poll_ready(MISTAT, MISTAT_BUSY, 0) ? 0 : 1;
}

/*
 * PHY register read
 * PHY registers are not accessed directly, but through the MII
 */
uint16_t enc_phy_read(byte addr){
    enc_write_reg(MIREGADR, addr);
    enc_write_reg(MICMD, MICMD_MIIRD);
    wait_phy_ready();
    enc_write_reg(MICMD, 0x00);
    return enc_read_regw(MIRDL);
}

bool enc_phy_write(byte addr, uint16_t val){
    enc_write_reg(MIREGADR, addr);
    enc_write_regw(MIWRL, val);
    return wait_phy_ready();
}

/*
 * ERXRDPT must only be set at odd addresses, refer to errata datasheet #14
 */
uint16_t _erxrdpt_workaround(uint16_t erxrdpt, uint16_t start, uint16_t end) {
    if( (erxrdpt - 1 < start) || (erxrdpt - 1 > end)) {
		return end;
	} else {
		return ((erxrdpt - 1) | 1);
	}
}

int _bounded_distance(int from, int to, int bst, int bnd) {
    if (bst > bnd) {
        Serial.println(F("ERR: bad _buffer_distance arg: bst > bnd"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(from < bst) {
        Serial.println(F("ERR: bad _buffer_distance arg: from < bst"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(from > bnd) {
        Serial.println(F("ERR: bad _buffer_distance arg: from > bnd"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(to < bst) {
        Serial.println(F("ERR: bad _buffer_distance arg: to < bst"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(to > bnd) {
        Serial.println(F("ERR: bad _buffer_distance arg: to > bnd"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(from >= to) {
        return (from - to);
    } else {
        return (bnd - bst) + (from - to);
    }
}

int _bounded_sum(int start, int offset, int bst, int bnd) {
    if (bst > bnd) {
        Serial.println(F("ERR: bad _bounded_sum arg: bst > bnd"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(start < bst) {
        Serial.println(F("ERR: bad _bounded_sum arg: start < bst"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }
    if(start > bnd) {
        Serial.println(F("ERR: bad _bounded_sum arg: start > bnd"));
        g_enc_err = ENC_ERR_ARG;
        return 0;
    }

    return bst + ( (start - bst + offset) % (bst - bnd) );
}

/**
 * Number of auto-incrementing buffer reads between `from` and `to` assuming
 *   - erdpt wraps from erxnd to erxst.
 *   - erxst and erxnd never deviate from RXSTART_INIT and RXSTOP_INIT
 */
uint16_t _buffer_distance(int from, int to) {
    return (uint16_t)_bounded_distance(from, to, RXSTART_INIT, RXSTOP_INIT);
}

uint16_t _buffer_sum(int start, int offset) {
    return (uint16_t)_bounded_sum(start, offset, RXSTART_INIT, RXSTOP_INIT);
}

/**
 * Hardware initialization, based off linux drivers,

 */
int enc_hw_init() {
    byte reg;

    /*
	 * Check the RevID.
	 * If it's 0x00 or 0xFF probably the enc28j60 is not mounted or
	 * damaged
	 */
	reg = enc_read_reg(EREVID);
	if (reg == 0x00 || reg == 0xff) {
		return 0;
	}

    /* reset the chip */
    enc_soft_reset();
    /* Clear ECON1 */
    enc_write_reg(ECON1, 0x00);
    /* enable address auto increment */
    reg = 0x00;
    reg |= ECON2_AUTOINC;
    // TODO: enable voltage regulator powersave?
    // reg |= ECON2_PWRSV;
    enc_write_reg(ECON2, reg);

    /* 6.1: set receive buffer start + end */
    enc_write_regw(ERXSTL, RXSTART_INIT);
    enc_write_regw(ERXNDL, RXSTOP_INIT);
    enc_write_regw(ERXRDPTL, _erxrdpt_workaround(RXSTART_INIT, RXSTART_INIT, RXSTOP_INIT));
    enc_write_regw(ERDPTL, 0);
    /* 6.2: set transmit buffer start + end */
    enc_write_regw(ETXSTL, TXSTART_INIT);
    enc_write_regw(ETXNDL, TXSTOP_INIT);

    /* 6.3: default eth RX filter mode: (unicast OR broadcast) AND crc valid = 0xA1*/
    reg = 0x00;
    reg |= ERXFCON_UCEN; // ( unicast
    // reg |= ERXFCON_BCEN; //   OR broadcast )
    reg |= ERXFCON_CRCEN; // AND crc valid
    enc_write_reg(ERXFCON, reg);

	/* enable MAC receive, TX/RX Pause = 0x0d */
    reg = 0x00;
    reg |= MACON1_MARXEN; // Enable MAC to recieve frames
    reg |= MACON1_TXPAUS; // Allow IEEE defined flow control to function
    reg |= MACON1_RXPAUS; // Allow IEEE defined flow control to function
	enc_write_reg(MACON1, reg);

    enc_reg_print("MACON1 should be 0x0d", MACON1);
    enc_reg_print("MACON3", MACON3);

	/* enable automatic padding and CRC operations */
	if (FULL_DUPLEX) {
        reg = 0x00;
        // All short frames will be zero padded to 60 bytes and a valid CRC
        // will then be appended
        reg |= MACON3_PADCFG0;
        // MAC will append a valid CRC to all frames transmitted regardless of
        // PADCFG bits. TXCRCEN must be set if the PADCFG bits specify that a
        // valid CRC will be appended
        reg |= MACON3_TXCRCEN;
        // The type/length field of transmitted and received frames will be
        // checked. If it represents a length, the frame size will be compared
        // and mismatches will be reported in the transmit/receive status vector.
        reg |= MACON3_FRMLNEN;
        //
        reg |= MACON3_FULDPX;
		enc_write_reg(MACON3, reg);
		/* set inter-frame gap (non-back-to-back) */
		enc_write_reg(MAIPGL, 0x12);
		/* set inter-frame gap (back-to-back) */
		enc_write_reg(MABBIPG, 0x15);
	} else {
		enc_write_reg(MACON3,
				    MACON3_PADCFG0 | MACON3_TXCRCEN |
				    MACON3_FRMLNEN);
		enc_write_reg(MACON4, MACON4_DEFER);	/* DEFER bit */
		/* set inter-frame gap (non-back-to-back) */
		enc_write_regw(MAIPGL, 0x0C12);
		/* set inter-frame gap (back-to-back) */
		enc_write_reg(MABBIPG, 0x12);
	}

    enc_reg_print("MACON1 should be 0x0d", MACON1);
    enc_reg_print("MACON3", MACON3);

	/*
	 * MACLCON1 (default)
	 * MACLCON2 (default)
	 * Set the maximum packet size which the controller will accept
	 */
	enc_write_regw(MAMXFLL, MAX_FRAMELEN);

	/* Configure LEDs */
	if (!enc_phy_write(PHLCON, ENC28J60_LAMPS_MODE))
		return 0;

	if (FULL_DUPLEX) {
		if (!enc_phy_write(PHCON1, PHCON1_PDPXMD))
			return 0;
		if (!enc_phy_write(PHCON2, 0x00))
			return 0;
	} else {
		if (!enc_phy_write(PHCON1, 0x00))
			return 0;
		if (!enc_phy_write(PHCON2, PHCON2_HDLDIS))
			return 0;
	}

    return 1;
}

void enc_hw_enable() {
    byte mask;

    mask = 0x00;
    mask |= EIE_PKTIE;
    mask |= EIE_INTIE;
    enc_bit_set(EIE, mask);
    enc_bit_set(ECON1, ECON1_RXEN);
}

void enc_hw_disable() {
    enc_write_reg(EIE, 0x00);
    enc_bit_clr(ECON1, ECON1_RXEN);
}

void enc_reg_print(String name, byte reg) {
    byte result;
    result = enc_read_reg(reg);
    Serial.print(name);
    Serial.print(F(": 0x"));
    if (result<0x10) {Serial.print(0);}
    Serial.println(result, HEX);
}

void enc_regs_print(String name, byte reg, int n_regs) {
    byte result;
    Serial.print(name);
    Serial.print(F(": 0x"));
    for(int i=n_regs-1; i>=0; i--) {
        result = enc_read_reg(reg + (byte)(i));
        if (result<0x10) {Serial.print(0);}
        Serial.print(result, HEX);
    }
    Serial.println();
}

void enc_peek_buf(int offset, int len) {
    if(len > MAX_FRAMELEN) {
        Serial.print(F("ERR: enc_peek_buf too long: "));
        Serial.println(len);
        g_enc_err = ENC_ERR_ARG;
        return;
    }

    uint16_t old_erdpt = enc_read_regw(ERDPTL);
    enc_write_regw(ERDPTL, _buffer_sum(old_erdpt, offset));
    // Serial.println(F("g_enc_eth_frame_buf: 0x"));
    // Serial.println((int)(g_enc_eth_frame_buf), HEX);
    enc_read_buf(g_enc_eth_frame_buf, len);
    if( DEBUG_ETH ) {
        for( int i=0; i<len; i++){
            if(g_enc_eth_frame_buf[i]<0x10) Serial.print(0);
            Serial.print(g_enc_eth_frame_buf[i], HEX);
        }
        Serial.println();
    }
    enc_write_regw(ERDPTL, old_erdpt);
}

void _enc_print_rxstat(uint16_t rxstat) {
    Serial.print(F("-> RSV_RXLONGEVDROPEV: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXLONGEVDROPEV), HEX);
    Serial.print(F("-> RSV_CARRIEREV: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_CARRIEREV), HEX);
    Serial.print(F("-> RSV_CRCERROR: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_CRCERROR), HEX);
    Serial.print(F("-> RSV_LENCHECKERR: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_LENCHECKERR), HEX);
    Serial.print(F("-> RSV_LENOUTOFRANGE: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_LENOUTOFRANGE), HEX);
    Serial.print(F("-> RSV_RXOK: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXOK), HEX);
    Serial.print(F("-> RSV_RXMULTICAST: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXMULTICAST), HEX);
    Serial.print(F("-> RSV_RXBROADCAST: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXBROADCAST), HEX);
    Serial.print(F("-> RSV_DRIBBLENIBBLE: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_DRIBBLENIBBLE), HEX);
    Serial.print(F("-> RSV_RXCONTROLFRAME: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXCONTROLFRAME), HEX);
    Serial.print(F("-> RSV_RXPAUSEFRAME: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXPAUSEFRAME), HEX);
    Serial.print(F("-> RSV_RXUNKNOWNOPCODE: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXUNKNOWNOPCODE), HEX);
    Serial.print(F("-> RSV_RXTYPEVLAN: 0x"));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXTYPEVLAN), HEX);
}

/**
 * Temporary MAC address storage
 */
byte * mac = (byte *)malloc(MAC_BYTES*sizeof(byte));


inline void _print_mac() {
    for(int i=0; i<MAC_BYTES; i++){
        if(i>0) Serial.print(F(":"));
        if(mac[i]<0x10) Serial.print(0);
        Serial.print(mac[i], HEX);
    }
    Serial.println();
}

void _enc_dump_pkt(int bcnt) {
    enc_read_buf(mac, MAC_BYTES);
    bcnt -= MAC_BYTES;
    uint16_t typ_len = enc_read_buf_w();
    if( DEBUG_ETH ) {
        Serial.print(F("-> DA: "));
        _print_mac();
    }
    enc_read_buf(mac, MAC_BYTES);
    bcnt -= MAC_BYTES;
    if( DEBUG_ETH ) {
        Serial.print(F("-> SA: "));
        _print_mac();
        Serial.print(F("-> TYP/LEN: "));
        Serial.println(typ_len, HEX);
        Serial.print(F("-> (bcnt remaining): "));
        Serial.println(bcnt);
    }
    g_enc_series = enc_read_buf_b();
    bcnt -= 1;
    enc_peek_buf(0, bcnt);
}

/**
 * Consumes a packet
 */
void consume_packet() {
    enc_write_regw(ERDPTL, g_enc_npp);
    enc_write_regw(ERXRDPTL, _erxrdpt_workaround(g_enc_npp, RXSTART_INIT, RXSTOP_INIT));
    enc_bit_set(ECON2, ECON2_PKTDEC);
}

/**
 *  Grab the Next Packet Pointer, RX Status, RX Byte Count from buffer
 *  Does not restore ERDPT
 */
void _enc_refresh_rsv_globals() {
    enc_read_buf(g_enc_rsv, RSV_SIZE);

    g_enc_npp = g_enc_rsv[1];
    g_enc_npp <<= 8;
    g_enc_npp |= g_enc_rsv[0];

    g_enc_rxbcnt = g_enc_rsv[3];
    g_enc_rxbcnt <<= 8;
    g_enc_rxbcnt |= g_enc_rsv[2];

    g_enc_rxstat = g_enc_rsv[5];
    g_enc_rxstat <<= 8;
    g_enc_rxstat |= g_enc_rsv[4];
}

/**
 * Dumps the NPP, Receive status vector and packet located at ERDPT, seeks back
 */
void enc_peek_npp_rsv_pkt() {
    uint16_t old_erdpt = enc_read_regw(ERDPTL);

    Serial.print(F("old erdpt: "));
    Serial.println(old_erdpt, HEX);

    _enc_refresh_rsv_globals();

    Serial.print(F("next packet: 0x"));
    Serial.println(g_enc_npp, HEX);

    Serial.print(F("g_enc_rxbcnt: "));
    Serial.println(g_enc_rxbcnt);

    Serial.println(F("rxstat: "));
    _enc_print_rxstat(g_enc_rxstat);

    Serial.println(F("packet: "));
    _enc_dump_pkt(g_enc_rxbcnt);

    enc_write_regw(ERDPTL, old_erdpt);
}

void enc_regs_debug() {
    Serial.println(F("\nHwRevID:\n------"));
    enc_reg_print("\nEREVID", EREVID);
    Serial.println(F("Cntrl:\n-----"));
    enc_reg_print("ECON1", ECON1);
    enc_reg_print("ECON2", ECON2);
    enc_reg_print("ESTAT", ESTAT);
    enc_reg_print("EIR", EIR);
    enc_reg_print("EIE", EIE);
    Serial.println(F("\nMAC:\n-----"));
    enc_reg_print("MACON1", MACON1);
    enc_reg_print("MACON3", MACON3);
    enc_reg_print("MACON4", MACON4);
    Serial.println(F("\nRX:\n-----"));
    enc_regs_print("ERXSTH:ERXSTL", ERXSTL, 2);
    enc_regs_print("ERXNDH:ERXNDL", ERXNDL, 2);
    enc_regs_print("ERXWRPTH:ERXWRPTL", ERXWRPTL, 2);
    enc_regs_print("ERXRDPTH:ERXRDPTL", ERXRDPTL, 2);
    enc_reg_print("ERXFCON", ERXFCON);
    enc_reg_print("EPKTCNT", EPKTCNT);
    enc_regs_print("MAMXFLH:MAMXFLL", MAMXFLL, 2);

    Serial.println(F("\nTX:\n-----"));
    enc_regs_print("ETXSTH:ETXSTL", ETXSTL, 2);
    enc_regs_print("ETXNDH:ETXNDL", ETXNDL, 2);
}
