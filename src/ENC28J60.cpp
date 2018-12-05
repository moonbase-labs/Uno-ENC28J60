#include "ENC28J60.h"

/**
 * SPI stuff
 * could have its own source file
 */

#define waitspi() while(!(SPSR&(1<<SPIF)))

inline void _beginTransaction(SPISettings spiSettings) {

    // digitalWrite(MOSI, LOW);
    // digitalWrite(SCK, LOW);
    digitalWrite(CS_PIN, LOW);

    delayMicroseconds(CS_DELAY);
    #ifdef USE_SPI_LIBRARY
        SPI.beginTransaction(spiSettings);
    #else
        noInterrupts();
    #endif
}

inline byte _spiTransfer(byte data) {
    byte response;
    #ifdef USE_SPI_LIBRARY
        response = SPI.transfer(data);
    #else
        SPDR = data;
        waitspi();
        response = SPDR;
    #endif
    delayMicroseconds(WORD_DELAY);
    return response;
}

inline void _endTransaction() {
    #if USE_SPI_LIBRARY
        SPI.endTransaction();
    #else
        interrupts();
    #endif
    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(CS_DELAY);
}

/**
 * ENC specific stuff
 */

inline byte _first_byte(byte op, byte arg) {
    // assembe first byte in operation from opcode and argument
    return op | (arg & ADDR_MASK);
}

long last_print = 0;
#define PRINT_DELAY 100

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
        Serial.print("ERR: op");
        Serial.print(op);
        Serial.println(" should not expect reply, and require data argument");
        return;
    }

    #ifdef REPEAT_BREAKPOINTS
        do {
    #endif

    _beginTransaction(spiSettings);
    _spiTransfer(_first_byte(op, arg));
    _spiTransfer(data);
    _endTransaction();

    #ifdef REPEAT_BREAKPOINTS
            if ((millis() - last_print) > PRINT_DELAY) {
                Serial.print("SPCR is 0x");
                Serial.print(SPCR, HEX);
                if(ENC28J60_WRITE_CTRL_REG == op) {
                    Serial.print(" | writing 0x");
                }
                else if(ENC28J60_BIT_FIELD_SET == op) {
                    Serial.print("setting mask 0x");
                }
                else if(ENC28J60_BIT_FIELD_CLR == op) {
                    Serial.print("clearing mask 0x");
                }
                Serial.print(data, HEX);
                Serial.print(" in reg with ");
                Serial.println(_first_byte(op, arg), HEX);
                last_print = millis();
            }
        } while (!Serial.available());
        while (Serial.available()){Serial.read();delay(1);}
    #endif

}

byte enc_op_read(uint8_t op, uint8_t arg) {
    // Perform a SPI operation on the ENC28J60 which does not have a data arg
    // and expects reply
    if(
        (ENC28J60_READ_CTRL_REG != op) &
        (ENC28J60_READ_BUF_MEM != op)
    ) {
        Serial.print("ERR: op");
        Serial.print(op);
        Serial.println(" should expect reply and require data argument");
        return 0;
    }

    byte result = 0x00;

    #ifdef REPEAT_BREAKPOINTS
        do {
    #endif

    _beginTransaction(spiSettings);
    _spiTransfer(_first_byte(op, arg));
    // handle dummy byte for certain RCRs
    if(arg & SPRD_MASK) _spiTransfer(0);
    // clock in response
    result = _spiTransfer(0);
    _endTransaction();

    #ifdef REPEAT_BREAKPOINTS
            if ((millis() - last_print) > PRINT_DELAY) {
                Serial.print("SPCR is 0x");
                Serial.print(SPCR, HEX);
                Serial.print(" | reading 0x");
                Serial.print(result, HEX);
                Serial.print(" from reg with ");
                Serial.println(_first_byte(op, arg), HEX);
                last_print = millis();
            }
        } while (!Serial.available());
        while (Serial.available()){Serial.read();delay(1);}
    #endif

    return result;
}

void enc_soft_reset() {
    // Send a soft reset command and wait
    Serial.println("RESET");
    _beginTransaction(spiSettings);
    _spiTransfer(ENC28J60_SOFT_RESET);
    delayMicroseconds(RESET_DELAY);
    _endTransaction();
    /* Fix for errata #2: CLKRDY set early */
    delayMicroseconds(RESET_DELAY);
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
    // Serial.print("ECON1: 0x");
    // Serial.println(econ1, HEX);

    if (ECON1_BSEL_MASK & (econ1 & bsel)) {
        Serial.print("changing bank from ");
        Serial.print(econ1 & ECON1_BSEL_MASK, HEX);
        Serial.print(" to ");
        Serial.println(bsel & ECON1_BSEL_MASK, HEX);
    }

    // bits which need to be set in ECON1.bsel
    byte set_bits = ECON1_BSEL_MASK & (~econ1 & bsel);
    // Serial.print("set bits: 0x");
    // Serial.println(set_bits, HEX);

    if( set_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_SET, ECON1, set_bits);
    }


    // bits which need to be cleared in ECON1.bsel
    byte clear_bits = ECON1_BSEL_MASK & (econ1 & ~bsel);
    // Serial.print("clear bits: 0x");
    // Serial.println(clear_bits, HEX);
    if( clear_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_CLR, ECON1, clear_bits);
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
    _spiTransfer(ENC28J60_WRITE_BUF_MEM);
    for(uint8_t i=0; i<len; i++){
        _spiTransfer(data[i]);
    }
    _endTransaction();
}

void enc_read_buf(byte * data, uint8_t len) {
    _beginTransaction(spiSettings);
    _spiTransfer(ENC28J60_READ_BUF_MEM);
    byte result;
    for(uint8_t i=0; i<len; i++){
        result = _spiTransfer(0x00);
        if(data) {data[i] = result;}
    }
    _endTransaction();
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

int poll_ready(byte reg, byte mask, byte val) {
    long poll_start = millis();
    while((enc_read_reg(reg) & mask) != val) {
        if( (millis() - poll_start) > POLL_TIMEOUT ) {
            return 1;
        }
        delay(1);
    }
    return 0;
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
    if ((erxrdpt - 1 < start) || (erxrdpt - 1 > end)) {
		return end;
	} else {
		return ((erxrdpt - 1) | 1);
	}
}

/**
 * Initialize SPI
 */
void spi_init() {

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

    /* set receive buffer start + end */
    enc_write_regw(ERXSTL, RXSTART_INIT);
    enc_write_regw(ERXNDL, RXSTOP_INIT);
    enc_write_regw(ERXRDPTL, _erxrdpt_workaround(RXSTART_INIT, ERXSTL, ERXNDL));
    /* set transmit buffer start + end */
    enc_write_regw(ETXSTL, TXSTART_INIT);
    enc_write_regw(ETXNDL, TXSTOP_INIT);

    /* default eth RX filter mode: (unicast OR broadcast) AND crc valid = 0xA1*/
    reg = 0x00;
    reg |= ERXFCON_UCEN;
    reg |= ERXFCON_CRCEN;
    reg |= ERXFCON_BCEN;
    enc_write_reg(ERXFCON, reg);

	/* enable MAC receive, TX/RX Pause = 0x0d */
    reg = 0x00;
    reg |= MACON1_MARXEN;
    reg |= MACON1_TXPAUS;
    reg |= MACON1_RXPAUS;
	enc_write_reg(MACON1, reg);

    enc_reg_print("MACON1", MACON1);

	/* enable automatic padding and CRC operations */
	if (FULL_DUPLEX) {
		enc_write_reg(MACON3,
				    MACON3_PADCFG0 | MACON3_TXCRCEN |
				    MACON3_FRMLNEN | MACON3_FULDPX);
		/* set inter-frame gap (non-back-to-back) */
		enc_write_reg(MAIPGL, 0x12);
		/* set inter-frame gap (back-to-back) */
		enc_write_reg(MABBIPG, 0x15);
	} else {
		enc_write_reg(MACON3,
				    MACON3_PADCFG0 | MACON3_TXCRCEN |
				    MACON3_FRMLNEN);
		enc_write_reg(MACON4, 1 << 6);	/* DEFER bit */
		/* set inter-frame gap (non-back-to-back) */
		enc_write_regw(MAIPGL, 0x0C12);
		/* set inter-frame gap (back-to-back) */
		enc_write_reg(MABBIPG, 0x12);
	}
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
    Serial.print(name);
    Serial.print(": 0x");
    result = enc_read_reg(reg);
    if (result<0x10) {Serial.print(0);}
    Serial.println(result, HEX);
}

void enc_regs_print(String name, byte reg, int n_regs) {
    byte result;
    Serial.print(name);
    Serial.print(": 0x");
    for(int i=n_regs-1; i>=0; i--) {
        result = enc_read_reg(reg + (byte)(i));
        if (result<0x10) {Serial.print(0);}
        Serial.print(result, HEX);
    }
    Serial.println();
}



void enc_regs_debug() {
    Serial.println("\nHwRevID:\n------");
    enc_reg_print("\nEREVID", EREVID);
    Serial.println("Cntrl:\n-----");
    enc_reg_print("ECON1", ECON1);
    enc_reg_print("ECON2", ECON2);
    enc_reg_print("ESTAT", ESTAT);
    enc_reg_print("EIR", EIR);
    enc_reg_print("EIE", EIE);
    Serial.println("\nMAC:\n-----");
    enc_reg_print("MACON1", MACON1);
    enc_reg_print("MACON3", MACON3);
    enc_reg_print("MACON4", MACON4);
    Serial.println("\nRX:\n-----");
    enc_regs_print("ERXSTH:ERXSTL", ERXSTL, 2);
    enc_regs_print("ERXNDH:ERXNDL", ERXNDL, 2);
    enc_regs_print("ERXWRPTH:ERXWRPTL", ERXWRPTL, 2);
    enc_regs_print("ERXRDPTH:ERXRDPTL", ERXRDPTL, 2);
    enc_reg_print("ERXFCON", ERXFCON);
    enc_reg_print("EPKTCNT", EPKTCNT);
    enc_regs_print("MAMXFLH:MAMXFLL", MAMXFLL, 2);

    Serial.println("\nTX:\n-----");
    enc_regs_print("ETXSTH:ETXSTL", ETXSTL, 2);
    enc_regs_print("ETXNDH:ETXNDL", ETXNDL, 2);
}
