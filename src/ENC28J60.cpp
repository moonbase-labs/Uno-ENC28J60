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
uint16_t g_enc_sequence = 0;

byte g_enc_reg_econ1 = 0;
byte g_enc_reg_econ2 = 0;
byte g_enc_reg_estat = 0;
byte g_enc_reg_eir = 0;
byte g_enc_reg_eie = 0;
byte g_enc_reg_macon1 = 0;
byte g_enc_reg_macon3 = 0;
byte g_enc_reg_macon4 = 0;
byte g_enc_reg_erxfcon = 0;
long g_enc_pkts_consumed = 0;
bool g_enc_repeat_breakpoints = false;
bool g_enc_debug_io = false;

inline byte _first_byte(byte op, byte arg) {
    // assembe first byte in operation from opcode and argument
    return op | (arg & ADDR_MASK);
}

byte * _get_cache(byte arg) {
    switch(arg) {
        case ECON1: return &g_enc_reg_econ1;
        case ECON2: return &g_enc_reg_econ2;
        case ESTAT: return &g_enc_reg_estat;
        case EIR: return &g_enc_reg_eir;
        case EIE: return &g_enc_reg_eie;
        case MACON1: return &g_enc_reg_macon1;
        case MACON3: return &g_enc_reg_macon3;
        case MACON4: return &g_enc_reg_macon4;
        case ERXFCON: return &g_enc_reg_erxfcon;
        default: return NULL;
    }
}

void _print_op(byte op) {
    switch(op) {
        case ENC28J60_READ_CTRL_REG: Serial.print(F("RCR")); break;
        case ENC28J60_READ_BUF_MEM: Serial.print(F("RBM")); break;
        case ENC28J60_WRITE_CTRL_REG: Serial.print(F("WCR")); break;
        case ENC28J60_WRITE_BUF_MEM: Serial.print(F("WBM")); break;
        case ENC28J60_BIT_FIELD_SET: Serial.print(F("BFS")); break;
        case ENC28J60_BIT_FIELD_CLR: Serial.print(F("BFC")); break;
        case ENC28J60_SOFT_RESET: Serial.print(F("SR")); break;
        default: Serial.print(F("???"));
    }
}

void _print_arg(byte arg) {
    switch(arg) {
        case EIE: Serial.print(F("EIE")); break;
        case EIR: Serial.print(F("EIR")); break;
        case ESTAT: Serial.print(F("ESTAT")); break;
        case ECON2: Serial.print(F("ECON2")); break;
        case ECON1: Serial.print(F("ECON1")); break;
        case ERDPTL: Serial.print(F("ERDPTL")); break;
        case ERDPTH: Serial.print(F("ERDPTH")); break;
        case EWRPTL: Serial.print(F("EWRPTL")); break;
        case EWRPTH: Serial.print(F("EWRPTH")); break;
        case ETXSTL: Serial.print(F("ETXSTL")); break;
        case ETXSTH: Serial.print(F("ETXSTH")); break;
        case ETXNDL: Serial.print(F("ETXNDL")); break;
        case ETXNDH: Serial.print(F("ETXNDH")); break;
        case ERXSTL: Serial.print(F("ERXSTL")); break;
        case ERXSTH: Serial.print(F("ERXSTH")); break;
        case ERXNDL: Serial.print(F("ERXNDL")); break;
        case ERXNDH: Serial.print(F("ERXNDH")); break;
        case ERXRDPTL: Serial.print(F("ERXRDPTL")); break;
        case ERXRDPTH: Serial.print(F("ERXRDPTH")); break;
        case ERXWRPTL: Serial.print(F("ERXWRPTL")); break;
        case ERXWRPTH: Serial.print(F("ERXWRPTH")); break;
        case EDMASTL: Serial.print(F("EDMASTL")); break;
        case EDMASTH: Serial.print(F("EDMASTH")); break;
        case EDMANDL: Serial.print(F("EDMANDL")); break;
        case EDMANDH: Serial.print(F("EDMANDH")); break;
        case EDMADSTL: Serial.print(F("EDMADSTL")); break;
        case EDMADSTH: Serial.print(F("EDMADSTH")); break;
        case EDMACSL: Serial.print(F("EDMACSL")); break;
        case EDMACSH: Serial.print(F("EDMACSH")); break;
        case EHT0: Serial.print(F("EHT0")); break;
        case EHT1: Serial.print(F("EHT1")); break;
        case EHT2: Serial.print(F("EHT2")); break;
        case EHT3: Serial.print(F("EHT3")); break;
        case EHT4: Serial.print(F("EHT4")); break;
        case EHT5: Serial.print(F("EHT5")); break;
        case EHT6: Serial.print(F("EHT6")); break;
        case EHT7: Serial.print(F("EHT7")); break;
        case EPMM0: Serial.print(F("EPMM0")); break;
        case EPMM1: Serial.print(F("EPMM1")); break;
        case EPMM2: Serial.print(F("EPMM2")); break;
        case EPMM3: Serial.print(F("EPMM3")); break;
        case EPMM4: Serial.print(F("EPMM4")); break;
        case EPMM5: Serial.print(F("EPMM5")); break;
        case EPMM6: Serial.print(F("EPMM6")); break;
        case EPMM7: Serial.print(F("EPMM7")); break;
        case EPMCSL: Serial.print(F("EPMCSL")); break;
        case EPMCSH: Serial.print(F("EPMCSH")); break;
        case EPMOL: Serial.print(F("EPMOL")); break;
        case EPMOH: Serial.print(F("EPMOH")); break;
        case EWOLIE: Serial.print(F("EWOLIE")); break;
        case EWOLIR: Serial.print(F("EWOLIR")); break;
        case ERXFCON: Serial.print(F("ERXFCON")); break;
        case EPKTCNT: Serial.print(F("EPKTCNT")); break;
        case MACON1: Serial.print(F("MACON1")); break;
        case MACON3: Serial.print(F("MACON3")); break;
        case MACON4: Serial.print(F("MACON4")); break;
        case MABBIPG: Serial.print(F("MABBIPG")); break;
        case MAIPGL: Serial.print(F("MAIPGL")); break;
        case MAIPGH: Serial.print(F("MAIPGH")); break;
        case MACLCON1: Serial.print(F("MACLCON1")); break;
        case MACLCON2: Serial.print(F("MACLCON2")); break;
        case MAMXFLL: Serial.print(F("MAMXFLL")); break;
        case MAMXFLH: Serial.print(F("MAMXFLH")); break;
        case MAPHSUP: Serial.print(F("MAPHSUP")); break;
        case MICON: Serial.print(F("MICON")); break;
        case MICMD: Serial.print(F("MICMD")); break;
        case MIREGADR: Serial.print(F("MIREGADR")); break;
        case MIWRL: Serial.print(F("MIWRL")); break;
        case MIWRH: Serial.print(F("MIWRH")); break;
        case MIRDL: Serial.print(F("MIRDL")); break;
        case MIRDH: Serial.print(F("MIRDH")); break;
        case MAADR1: Serial.print(F("MAADR1")); break;
        case MAADR0: Serial.print(F("MAADR0")); break;
        case MAADR3: Serial.print(F("MAADR3")); break;
        case MAADR2: Serial.print(F("MAADR2")); break;
        case MAADR5: Serial.print(F("MAADR5")); break;
        case MAADR4: Serial.print(F("MAADR4")); break;
        case EBSTSD: Serial.print(F("EBSTSD")); break;
        case EBSTCON: Serial.print(F("EBSTCON")); break;
        case EBSTCSL: Serial.print(F("EBSTCSL")); break;
        case EBSTCSH: Serial.print(F("EBSTCSH")); break;
        case MISTAT: Serial.print(F("MISTAT")); break;
        case EREVID: Serial.print(F("EREVID")); break;
        case ECOCON: Serial.print(F("ECOCON")); break;
        case EFLOCON: Serial.print(F("EFLOCON")); break;
        case EPAUSL: Serial.print(F("EPAUSL")); break;
        case EPAUSH: Serial.print(F("EPAUSH")); break;
        default: break;
    }
    Serial.print(":");
    arg &= ADDR_MASK;
    print_hex_byte(arg);
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
        _print_op(op);
        Serial.println(F(" should not expect reply, and require data argument"));
        g_enc_err = ENC_ERR_OP;
        return;
    }

    if(
        (arg & SPRD_MASK)
        && ((ENC28J60_BIT_FIELD_SET == op) || (ENC28J60_BIT_FIELD_CLR == op))
    ) {
        Serial.print(F("ERR: op"));
        _print_op(op);
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

        if (g_enc_debug_io && (SOMETIMES_PRINT_COND || !g_enc_repeat_breakpoints)) {
            DEBUG_PREFIX;
            if(ENC28J60_WRITE_CTRL_REG == op) {
                Serial.print(F(" | writing 0x"));
            }
            else if(ENC28J60_BIT_FIELD_SET == op) {
                Serial.print(F(" | setting mask 0x"));
            }
            else if(ENC28J60_BIT_FIELD_CLR == op) {
                Serial.print(F(" | clearing mask 0x"));
            }
            print_hex_byte(data);
            Serial.print(F(" in arg "));
            _print_arg(arg);
            Serial.print(F(" with "));
            _print_op(op);
            Serial.println();
            SOMETIMES_PRINT_END;
        }
    } while(g_enc_repeat_breakpoints && !Serial.available());
    if(g_enc_repeat_breakpoints) while (Serial.available()){Serial.read();delay(1);}

    byte * cache = _get_cache(arg);

    if(cache) {
        switch(op) {
            case ENC28J60_WRITE_CTRL_REG: *cache = data; break;
            case ENC28J60_BIT_FIELD_SET: *cache |= data; break;
            case ENC28J60_BIT_FIELD_CLR:  *cache ^= data; break;
        }
    }

    // do {} while (!Serial.available());
    // while (Serial.available()){Serial.read();delay(1);}

}

byte enc_op_read(uint8_t op, uint8_t arg) {
    // Perform a SPI operation on the ENC28J60 which does not have a data arg
    // and expects reply
    if(
        (ENC28J60_READ_CTRL_REG != op) &
        (ENC28J60_READ_BUF_MEM != op)
    ) {
        Serial.print(F("ERR: op"));
        _print_op(op);
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
        if (g_enc_debug_io && (SOMETIMES_PRINT_COND || !g_enc_repeat_breakpoints)) {
            DEBUG_PREFIX;
            Serial.print(F(" | reading 0x"));
            print_hex_byte(result);
            Serial.print(F(" from arg "));
            _print_arg(arg);
            Serial.print(F(" with "));
            _print_op(op);
            Serial.println();
            SOMETIMES_PRINT_END;
        }
    } while(g_enc_repeat_breakpoints && !Serial.available());
    if(g_enc_repeat_breakpoints) while (Serial.available()){Serial.read();delay(1);}

    byte * cache = _get_cache(arg);

    if(cache) {
        if (ENC28J60_READ_CTRL_REG == op) *cache = result;
    }

    // do {} while (!Serial.available());
    // while (Serial.available()){Serial.read();delay(1);}

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
        Serial.println(F("ERR: ESTAT_RXBUSY should be clear after reset"));
        g_enc_err = ENC_ERR_RST;
    } else if (poll_ready(ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY)) {
        Serial.println(F("ERR: ESTAT_CLKRDY should be set after reset"));
        g_enc_err = ENC_ERR_RST;
    } else {
        Serial.println(F("RESET succesful"));
        g_enc_err = ENC_NO_ERR;
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
    g_enc_reg_econ1 = enc_op_read(ENC28J60_READ_CTRL_REG, ECON1);
    // Serial.print(F("ECON1: 0x"));
    // println_hex_byte(econ1);

    if ((ECON1_BSEL_MASK & (g_enc_reg_econ1 ^ bsel))) {
        if (g_enc_debug_io) {
            DEBUG_PREFIX;
            Serial.print(F(" | changing bank from "));
            Serial.print(g_enc_reg_econ1 & ECON1_BSEL_MASK, HEX);
            Serial.print(F(" to "));
            Serial.println(bsel & ECON1_BSEL_MASK, HEX);
        }
    } else {
        return;
    }

    // bits which need to be set in ECON1.bsel
    byte set_bits = ECON1_BSEL_MASK & (~g_enc_reg_econ1 & bsel);
    // Serial.print(F("set bits: 0x"));
    // println_hex_byte(set_bits);

    if( set_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_SET, ECON1, set_bits);
    }


    // bits which need to be cleared in ECON1.bsel
    byte clear_bits = ECON1_BSEL_MASK & (g_enc_reg_econ1 & ~bsel);
    // Serial.print(F("clear bits: 0x"));
    // println_hex_byte(clear_bits);
    if( clear_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_CLR, ECON1, clear_bits);
    }

    g_enc_reg_econ1 = enc_op_read(ENC28J60_READ_CTRL_REG, ECON1);

    if (ECON1_BSEL_MASK & (g_enc_reg_econ1 ^ bsel)) {
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
void enc_write_buf(byte * data, int len) {
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_WRITE_BUF_MEM);
    for(uint8_t i=0; i<len; i++){
        _spiWrite(data[i]);
    }
    _endTransaction();
}

void enc_read_buf(byte * data, int len) {
    _beginTransaction(spiSettings);
    _spiWrite(ENC28J60_READ_BUF_MEM);
    byte result;
    for(int i=0; i<len; i++){
        result = _spiRead();
        if(data) {data[i] = result;}
    }
    _endTransaction();

    if(g_enc_debug_io){
        DEBUG_PREFIX;
        Serial.print(F(" | reading "));
        Serial.print(len);
        Serial.println(F(" bytes from buffer "));

        if(DEBUG_ETH && data) for(int i=0; i<len; i++){
            if (data[i] < 0x10) Serial.print(0);
            Serial.print(data[i]);
        }
    }
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

    if(g_enc_debug_io) {
        DEBUG_PREFIX;
        Serial.println(F(" | reading 1 byte from buffer"));
    }
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

    if(g_enc_debug_io) {
        DEBUG_PREFIX;
        Serial.println(F(" | reading 1 word from buffer"));
    }

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

void enc_set_pattern_match(byte * pattern) {
    /* NOTE: MAC address in ENC28J60 is byte-backward */
    enc_write_reg(EPMM7, pattern[0]);
    enc_write_reg(EPMM6, pattern[1]);
    enc_write_reg(EPMM5, pattern[2]);
    enc_write_reg(EPMM4, pattern[3]);
    enc_write_reg(EPMM3, pattern[4]);
    enc_write_reg(EPMM2, pattern[5]);
    enc_write_reg(EPMM1, pattern[6]);
    enc_write_reg(EPMM0, pattern[7]);
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
        return (bnd - bst - 1) + (to - from);
    } else {
        return (to - from);
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

    return bst + ( (start - bst + offset) % (bst - bnd - 1) );
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
	if (reg == 0x00 || reg == 0xff || g_enc_err > 0) {
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

    // Serial.print(F("ERXFCON: ")); enc_reg_print(ERXFCON); Serial.println();

	/* enable MAC receive, TX/RX Pause = 0x0d */
    reg = 0x00;
    reg |= MACON1_MARXEN; // Enable MAC to recieve frames
    reg |= MACON1_TXPAUS; // Allow IEEE defined flow control to function
    reg |= MACON1_RXPAUS; // Allow IEEE defined flow control to function
	enc_write_reg(MACON1, reg);

    // Serial.print(F("MACON1 should be 0x0d: ")); enc_reg_print(MACON1); Serial.println();
    // Serial.print(F("MACON3: ")); enc_reg_print(MACON3); Serial.println();
    // Serial.print(F("ERXFCON: ")); enc_reg_print(ERXFCON); Serial.println();

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

    // Serial.print(F("MACON1 should be 0x0d: ")); enc_reg_print(MACON1); Serial.println();
    // Serial.print(F("MACON3: ")); enc_reg_print(MACON3); Serial.println();

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

void enc_reg_print(byte reg) {
    byte result = enc_read_reg(reg);
    Serial.print(F("0x"));
    print_hex_byte(result);
}

void enc_regs_print(byte reg, int n_regs) {
    byte result;
    Serial.print(F("0x"));
    for(int i=n_regs-1; i>=0; i--) {
        result = enc_read_reg(reg + (byte)(i));
        print_hex_byte(result);
    }
}

// /**
//  * reads the buffer at offset into g_enc_eth_frame_buf (which sometimes won't malloc)
//  */
// void enc_peek_buf(int offset, int len) {
//     if(len > MAX_FRAMELEN) {
//         Serial.print(F("ERR: enc_peek_buf too long: "));
//         Serial.println(len);
//         g_enc_err = ENC_ERR_ARG;
//         return;
//     }
//
//     uint16_t old_erdpt = enc_read_regw(ERDPTL);
//     if(offset) {
//         enc_write_regw(ERDPTL, _buffer_sum(old_erdpt, offset));
//     }
//     enc_read_buf(g_enc_eth_frame_buf, len);
//     for( int i=0; i<len; i++){
//         print_hex_byte(g_enc_eth_frame_buf[i]);
//     }
//     Serial.println();
//     enc_write_regw(ERDPTL, old_erdpt);
// }

/**
 * Slower buf peek that doesn't rely on malloc
 */
void enc_peek_buf_slow(int offset, int len) {
    uint16_t old_erdpt = enc_read_regw(ERDPTL);
    if(offset) {
        enc_write_regw(ERDPTL, _buffer_sum(old_erdpt, offset));
    }
    byte buf_byte;
    for( int i=0; i<len; i++){
        buf_byte = enc_read_buf_b();
        print_hex_byte(buf_byte);
    }
    Serial.println();
    enc_write_regw(ERDPTL, old_erdpt);
}

void _enc_print_rxstat(uint16_t rxstat) {
    Serial.print(F("-> RSV_RXLONGEVDROPEV: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXLONGEVDROPEV));
    Serial.print(F("-> RSV_CARRIEREV: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_CARRIEREV));
    Serial.print(F("-> RSV_CRCERROR: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_CRCERROR));
    Serial.print(F("-> RSV_LENCHECKERR: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_LENCHECKERR));
    Serial.print(F("-> RSV_LENOUTOFRANGE: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_LENOUTOFRANGE));
    Serial.print(F("-> RSV_RXOK: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXOK));
    Serial.print(F("-> RSV_RXMULTICAST: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXMULTICAST));
    Serial.print(F("-> RSV_RXBROADCAST: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXBROADCAST));
    Serial.print(F("-> RSV_DRIBBLENIBBLE: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_DRIBBLENIBBLE));
    Serial.print(F("-> RSV_RXCONTROLFRAME: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXCONTROLFRAME));
    Serial.print(F("-> RSV_RXPAUSEFRAME: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXPAUSEFRAME));
    Serial.print(F("-> RSV_RXUNKNOWNOPCODE: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXUNKNOWNOPCODE));
    Serial.print(F("-> RSV_RXTYPEVLAN: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_RXTYPEVLAN));
    Serial.print(F("-> RSV_ZERO: "));
    Serial.println(RSV_GETBIT(rxstat, RSV_ZERO));
}

/**
 * Temporary MAC address storage
 */
byte * mac = (byte *)malloc(MAC_BYTES*sizeof(byte));


inline void _print_mac() {
    for(int i=0; i<MAC_BYTES; i++){
        if(i>0) Serial.print(F(":"));
        print_hex_byte(mac[i]);
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
        println_hex_byte(typ_len);
        Serial.print(F("-> (bcnt remaining): "));
        Serial.println(bcnt);
    }
    g_enc_sequence = enc_read_buf_w();
    bcnt -= 1;
    if( DEBUG_ETH ) {
        enc_peek_buf_slow(0, bcnt);
    }
}

/**
 * Consumes a packet
 */
void free_packet() {
    if(DEBUG_ETH_BASIC) {
        Serial.print(F("Freeing packet, advancing to 0x"));
        println_hex_word(g_enc_npp);
    }
    enc_write_regw(ERDPTL, g_enc_npp);
    enc_write_regw(ERXRDPTL, _erxrdpt_workaround(g_enc_npp, RXSTART_INIT, RXSTOP_INIT));
    enc_bit_set(ECON2, ECON2_PKTDEC);
    g_enc_pkts_consumed++;
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
    println_hex_byte(old_erdpt);

    _enc_refresh_rsv_globals();

    Serial.print(F("next packet: 0x"));
    println_hex_byte(g_enc_npp);

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
    Serial.print(F("\nEREVID: ")); enc_reg_print(EREVID); Serial.println();
    Serial.println(F("Cntrl:\n-----"));
    Serial.print(F("ECON1: ")); enc_reg_print(ECON1); Serial.println();
    if(g_enc_reg_econ1) {
        Serial.print(F(" -> "));
        if(g_enc_reg_econ1 & ECON1_TXRST) Serial.print(F("TXRST "));
        if(g_enc_reg_econ1 & ECON1_RXRST) Serial.print(F("RXRST "));
        if(g_enc_reg_econ1 & ECON1_DMAST) Serial.print(F("DMAST "));
        if(g_enc_reg_econ1 & ECON1_CSUMEN) Serial.print(F("CSUMEN "));
        if(g_enc_reg_econ1 & ECON1_TXRTS) Serial.print(F("TXRTS "));
        if(g_enc_reg_econ1 & ECON1_RXEN) Serial.print(F("RXEN "));
        if(g_enc_reg_econ1 & ECON1_BSEL1) Serial.print(F("BSEL1 "));
        if(g_enc_reg_econ1 & ECON1_BSEL0) Serial.print(F("BSEL0 "));
        Serial.println();
    }
    Serial.print(F("ECON2: ")); enc_reg_print(ECON2); Serial.println();
    if(g_enc_reg_econ2) {
        Serial.print(F(" -> "));
        if(g_enc_reg_econ2 & ECON2_AUTOINC) Serial.print(F("AUTOINC "));
        if(g_enc_reg_econ2 & ECON2_PKTDEC) Serial.print(F("PKTDEC "));
        if(g_enc_reg_econ2 & ECON2_PWRSV) Serial.print(F("PWRSV "));
        if(g_enc_reg_econ2 & ECON2_VRPS) Serial.print(F("VRPS "));
        Serial.println();
    }
    Serial.print(F("ESTAT: ")); enc_reg_print(ESTAT); Serial.println();
    if(g_enc_reg_estat) {
        Serial.print(F(" -> "));
        if(g_enc_reg_estat & ESTAT_INT) Serial.print(F("INT "));
        if(g_enc_reg_estat & ESTAT_LATECOL) Serial.print(F("LATECOL "));
        if(g_enc_reg_estat & ESTAT_RXBUSY) Serial.print(F("RXBUSY "));
        if(g_enc_reg_estat & ESTAT_TXABRT) Serial.print(F("TXABRT "));
        if(g_enc_reg_estat & ESTAT_CLKRDY) Serial.print(F("CLKRDY "));
        Serial.println();
    }
    Serial.print(F("EIR: ")); enc_reg_print(EIR); Serial.println();
    if(g_enc_reg_eir) {
        Serial.print(F(" -> "));
        if(g_enc_reg_eir & EIR_PKTIF) Serial.print(F("PKTIF "));
        if(g_enc_reg_eir & EIR_DMAIF) Serial.print(F("DMAIF "));
        if(g_enc_reg_eir & EIR_LINKIF) Serial.print(F("LINKIF "));
        if(g_enc_reg_eir & EIR_TXIF) Serial.print(F("TXIF "));
        if(g_enc_reg_eir & EIR_TXERIF) Serial.print(F("TXERIF "));
        if(g_enc_reg_eir & EIR_RXERIF) Serial.print(F("RXERIF "));
        Serial.println();
    }
    Serial.print(F("EIE: ")); enc_reg_print(EIE); Serial.println();
    if(g_enc_reg_eie) {
        Serial.print(F(" -> "));
        if(g_enc_reg_eie & EIE_INTIE) Serial.print(F("INTIE "));
        if(g_enc_reg_eie & EIE_PKTIE) Serial.print(F("PKTIE "));
        if(g_enc_reg_eie & EIE_DMAIE) Serial.print(F("DMAIE "));
        if(g_enc_reg_eie & EIE_LINKIE) Serial.print(F("LINKIE "));
        if(g_enc_reg_eie & EIE_TXIE) Serial.print(F("TXIE "));
        if(g_enc_reg_eie & EIE_TXERIE) Serial.print(F("TXERIE "));
        if(g_enc_reg_eie & EIE_RXERIE) Serial.print(F("RXERIE "));
        Serial.println();
    }
    Serial.print(F("ERDPT: ")); enc_regs_print(ERDPTL, 2); Serial.println();

    Serial.println(F("\nMAC:\n-----"));
    Serial.print(F("MACON1: ")); enc_reg_print(MACON1); Serial.println();
    if(g_enc_reg_macon1) {
        Serial.print(F(" -> "));
        if(g_enc_reg_macon1 & MACON1_LOOPBK) Serial.print(F("LOOPBK "));
        if(g_enc_reg_macon1 & MACON1_TXPAUS) Serial.print(F("TXPAUS "));
        if(g_enc_reg_macon1 & MACON1_RXPAUS) Serial.print(F("RXPAUS "));
        if(g_enc_reg_macon1 & MACON1_PASSALL) Serial.print(F("PASSALL "));
        if(g_enc_reg_macon1 & MACON1_MARXEN) Serial.print(F("MARXEN "));
        Serial.println();
    }
    Serial.print(F("MACON3: ")); enc_reg_print(MACON3); Serial.println();
    if(g_enc_reg_macon3) {
        Serial.print(F(" -> "));
        if(g_enc_reg_macon3 & MACON3_PADCFG2) Serial.print(F("PADCFG2 "));
        if(g_enc_reg_macon3 & MACON3_PADCFG1) Serial.print(F("PADCFG1 "));
        if(g_enc_reg_macon3 & MACON3_PADCFG0) Serial.print(F("PADCFG0 "));
        if(g_enc_reg_macon3 & MACON3_TXCRCEN) Serial.print(F("TXCRCEN "));
        if(g_enc_reg_macon3 & MACON3_PHDRLEN) Serial.print(F("PHDRLEN "));
        if(g_enc_reg_macon3 & MACON3_HFRMLEN) Serial.print(F("HFRMLEN "));
        if(g_enc_reg_macon3 & MACON3_FRMLNEN) Serial.print(F("FRMLNEN "));
        if(g_enc_reg_macon3 & MACON3_FULDPX) Serial.print(F("FULDPX "));
        Serial.println();
    }
    Serial.print(F("MACON4: ")); enc_reg_print(MACON4); Serial.println();
    Serial.println(F("\nRX:\n-----"));
    Serial.print(F("ERXST: ")); enc_regs_print(ERXSTL, 2); Serial.println();
    Serial.print(F("ERXND: ")); enc_regs_print(ERXNDL, 2); Serial.println();
    Serial.print(F("ERXWRPT: ")); enc_regs_print(ERXWRPTL, 2); Serial.println();
    Serial.print(F("ERXRDPT: ")); enc_regs_print(ERXRDPTL, 2); Serial.println();
    Serial.print(F("ERXFCON: ")); enc_reg_print(ERXFCON); Serial.println();
    if(g_enc_reg_erxfcon) {
        Serial.print(F(" -> "));
        if(g_enc_reg_erxfcon & ERXFCON_UCEN) Serial.print(F("UCEN "));
        if(g_enc_reg_erxfcon & ERXFCON_ANDOR) Serial.print(F("ANDOR "));
        if(g_enc_reg_erxfcon & ERXFCON_CRCEN) Serial.print(F("CRCEN "));
        if(g_enc_reg_erxfcon & ERXFCON_PMEN) Serial.print(F("PMEN "));
        if(g_enc_reg_erxfcon & ERXFCON_MPEN) Serial.print(F("MPEN "));
        if(g_enc_reg_erxfcon & ERXFCON_HTEN) Serial.print(F("HTEN "));
        if(g_enc_reg_erxfcon & ERXFCON_MCEN) Serial.print(F("MCEN "));
        if(g_enc_reg_erxfcon & ERXFCON_BCEN) Serial.print(F("BCEN "));
        Serial.println();
    }
    Serial.print(F("EPKTCNT: ")); enc_reg_print(EPKTCNT); Serial.println();
    Serial.print(F("MAMXFL: ")); enc_regs_print(MAMXFLL, 2); Serial.println();

    Serial.println(F("\nTX:\n-----"));
    Serial.print(F("ETXST: ")); enc_regs_print(ETXSTL, 2); Serial.println();
    Serial.print(F("ETXND: ")); enc_regs_print(ETXNDL, 2); Serial.println();
}
