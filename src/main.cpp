#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>
#include "ENC28J60.h"

#define CS_PIN 10
#define CS_DELAY 0
#define WORD_DELAY 0
#define SPI_MODE SPI_MODE3
#define SPI_SPEED 10000000
#define N_LEDS 60

#define ARRAY_LEN(x) sizeof(x)/sizeof(x[0])

SPISettings spiSettings;

// Remember the last value of econ_1
byte g_econ1 = 0x00;

inline byte _first_byte(byte op, byte arg) {
    // assembe first byte in operation from opcode and argument
    return op | (arg & ADDR_MASK);
}

inline void _beginTransaction(SPISettings spiSettings) {
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(CS_DELAY);
    SPI.beginTransaction(spiSettings);
}

inline byte _spiTransfer(byte data) {
    byte response = SPI.transfer(data);
    delayMicroseconds(WORD_DELAY);
    return response;
}

inline void _endTransaction() {
    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);
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
        Serial.print("ERR: op");
        Serial.print(op);
        Serial.println(" should not expect reply, and require data argument");
        return;
    }

    _beginTransaction(spiSettings);
    _spiTransfer(_first_byte(op, arg));
    _spiTransfer(data);
    _endTransaction();
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

    _beginTransaction(spiSettings);
    _spiTransfer(_first_byte(op, arg));
    // TODO: handle dummy byte for certain RCRs
    // clock in response
    byte result = _spiTransfer(0);
    _endTransaction();
    return result;
}

void enc_soft_reset() {
    // Send a soft reset command and wait
    Serial.println("RESET");
    _beginTransaction(spiSettings);
    _spiTransfer(ENC28J60_SOFT_RESET);
    _endTransaction();
    delay(10);
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

    g_econ1 = enc_op_read(ENC28J60_READ_CTRL_REG, ECON1);
    // Serial.print("ECON1: 0x");
    // Serial.println(g_econ1, HEX);

    // bits which need to be set in ECON1.bsel
    byte set_bits = ECON1_BSEL_MASK & (~g_econ1 & bsel);
    // Serial.print("set bits: 0x");
    // Serial.println(set_bits, HEX);

    if( set_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_SET, ECON1, set_bits);
    }


    // bits which need to be cleared in ECON1.bsel
    byte clear_bits = ECON1_BSEL_MASK & (g_econ1 & ~bsel);
    // Serial.print("clear bits: 0x");
    // Serial.println(clear_bits, HEX);
    if( clear_bits ) {
        enc_op_write(ENC28J60_BIT_FIELD_CLR, ECON1, clear_bits);
    }
}

/**
 * Read a register from the ENC
 */
byte enc_read_reg(byte reg) {
    // TODO: cache registers?
    enc_bank_sel(reg);
    return enc_op_read(ENC28J60_READ_CTRL_REG, reg);
}

/**
 * Write to a register on the ENC
 */
void enc_write_reg(byte reg, byte value) {
    // TODO: update cache?
    enc_bank_sel(reg);
    enc_op_write(ENC28J60_WRITE_CTRL_REG, reg, value);
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

long timer_start;

inline void start_timer() {
    timer_start = millis();
}

inline long stop_timer() {
    return millis() - timer_start;
}

void setup() {
    Serial.begin(9600);
    Serial.println("SETUP");

    // Set SPI Pin Mode
    pinMode(CS_PIN, OUTPUT);
    SPI.begin();
    spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE);

    // Reset chip
    enc_soft_reset();

    // timer
    timer_start = millis();
}

// void demo_read_reg() {
//     byte result;
//     byte reg_addr;
//
//     static byte allregs[] = {
//         ERDPTL, ERDPTH, EWRPTL, EWRPTH, ETXSTL, ETXSTH, ETXNDL, ETXNDH, ERXSTL,
//         ERXSTH, ERXNDL, ERXNDH, ERXRDPTL, ERXRDPTH, ERXWRPTL, ERXWRPTH, EDMASTL,
//         EDMASTH, EDMANDL, EDMANDH, EDMADSTL, EDMADSTH, EDMACSL, EDMACSH,
//
//         EHT0, EHT1, EHT2, EHT3, EHT4, EHT5, EHT6, EHT7, EPMM0, EPMM1, EPMM2,
//         EPMM3, EPMM4, EPMM5, EPMM6, EPMM7, EPMCSL, EPMCSH, EPMOL, EPMOH, EWOLIE,
//         EWOLIR, ERXFCON, EPKTCNT,
//
//         MACON1, MACON3, MACON4, MABBIPG, MAIPGL, MAIPGH, MACLCON1, MACLCON2,
//         MAMXFLL, MAMXFLH, MAPHSUP, MICON, MICMD, MIREGADR, MIWRL, MIWRH, MIRDL,
//         MIRDH,
//
//         MAADR1, MAADR0, MAADR3, MAADR2, MAADR5, MAADR4, EBSTSD, EBSTCON,
//         EBSTCSL, EBSTCSH, MISTAT, EREVID, ECOCON, EFLOCON, EPAUSL, EPAUSH,
//     };
//
//     for (uint8_t i=0; i<ARRAY_LEN(allregs); i++) {
//         reg_addr = allregs[i];
//         result = enc_read_reg(reg_addr);
//         Serial.print("read register 0x");
//         if(reg_addr < 0x10) {Serial.print(0);}
//         Serial.print(reg_addr, HEX);
//         Serial.print(", response: 0x");
//         if(result < 0x10) {Serial.print(0);}
//         Serial.println(result, HEX);
//     }
//
//     // Should look like
//     // FA
//     // 05
//     // 00
//     // 00
//     // 00
//     // 00
//     // 00
//     // 00
//     // FA
//     // 05
//     // FF
//     // 1F
//     // FA
//     // 05
//     // 00 ...
//
//     // Works in SPI_MODE1, SPI_MODE3
//     // Doesn't work in SPI_MODE0
// }

// void demo_write_reg() {
//     byte result;
//     Serial.print("View the ERXSTL:ERXSTH registers (should be initialized to 0xfa, 0x05): 0x");
//     result = enc_read_reg(ERXSTL);
//     Serial.print(result, HEX);
//     Serial.print(", 0x");
//     result = enc_read_reg(ERXSTH);
//     Serial.println(result, HEX);
//
//     Serial.println("set the ERXSTL:ERXSTH registers to (0xaa, 0x05):");
//     enc_write_reg(ERXSTL, 0xaa);
//     enc_write_reg(ERXSTH, 0x05);
//
//     Serial.print("View the ERXSTL:ERXSTH registers (should now be set to 0xaa, 0x05): 0x");
//     result = enc_read_reg(ERXSTL);
//     Serial.print(result, HEX);
//     Serial.print(", 0x");
//     result = enc_read_reg(ERXSTH);
//     Serial.println(result, HEX);
//
//     Serial.println("return the ERXSTL:ERXSTH registers to (0xfa, 0x05):");
//     enc_write_reg(ERXSTL, 0xfa);
//     enc_write_reg(ERXSTH, 0x05);
//
//     // Doesn't work in SPI_MODE1, SPI_MODE2,
//     // Works in SPI_MODE3
// }

// void demo_bitflip() {
//     byte result;
//
//     Serial.print("View the ECON1 register (should be initialized to 0x00): 0x");
//     result = enc_read_reg(ECON1);
//     Serial.println(result, HEX);
//     Serial.print("Set the ECON1_RXEN bit field (Enable Recieve)");
//     enc_bit_set(ECON1, ECON1_RXEN);
//     Serial.print("View the ECON1 register (should be set to 0x40): 0x");
//     result = enc_read_reg(ECON1);
//     Serial.println(result, HEX);
//     Serial.print("clear the ECON1_RXEN bit field (Enable Recieve)");
//     enc_bit_clr(ECON1, ECON1_RXEN);
//     Serial.print("View the ECON1 register (should be set to 0x00): 0x");
//     result = enc_read_reg(ECON1);
//     Serial.println(result, HEX);
// }

// void demo_buf_write() {
//     static byte testdata[] = {
//         0x00, 0x00, 0x00, 0x00, // START Frame
//         0xf0, 0xff, 0x00, 0x00, // Blue
//         0xf0, 0x00, 0xff, 0x00, // Green
//         0xf0, 0x00, 0x00, 0xff, // Red
//         0xff, 0xff, 0xff, 0xff, // END Frame
//     };
//     byte * results = (byte *) malloc(sizeof(testdata));
//     byte result;
//
//     Serial.println("Set buf write pointer (EWRPTL:EWRPTH) byte (0x02:0x03) to 0x0000");
//     enc_write_reg(EWRPTL, 0x00);
//     enc_write_reg(EWRPTH, 0x00);
//     Serial.println("Write bytes to buffer");
//
//     enc_write_buf(testdata, ARRAY_LEN(testdata));
//     Serial.println("Set buf read pointer (ERDPTL:ERDPTH) byte (0x00:0x01) to 0x0000");
//     enc_write_reg(ERDPTL, 0x00);
//     enc_write_reg(ERDPTH, 0x00);
//     Serial.print("read bytes from buffer: 0x");
//
//     enc_read_buf(results, ARRAY_LEN(testdata));
//
//     for(uint8_t i=0; i<ARRAY_LEN(testdata); i++){
//         result = results[i];
//         if(result < 0x10) {Serial.print(0);}
//         Serial.print(result, HEX);
//     }
//     Serial.println();
// }

int led_buffer_size = 4 * (N_LEDS + 2);
byte * led_data = (byte *) malloc(led_buffer_size * sizeof(byte));

// temporary storage of HSV colour space value for animations
byte hsv[3];
// temporary storage of RGB colour space value for animations
byte rgb[3];
uint8_t frame = 0;

void hsv2rgb(){
    // populate the global RGB storage from the global HSV storage
    // stolen from https://github.com/ratkins/RGBConverter/blob/master/RGBConverter.cpp
    double h = hsv[0] / 255.0;
    double s = hsv[1] / 255.0;
    double v = hsv[2] / 255.0;
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;

    int i = int(h * 6);
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch(i % 6){
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    rgb[0] = (byte)(r * 255);
    rgb[1] = (byte)(g * 255);
    rgb[2] = (byte)(b * 255);
}

void demo_rainbows() {
    // start_timer();

    frame = (frame + 2) % 255;
    // Create start frame:
    for( int i=0; i<4; i++) {
        led_data[i] = 0x00;
    }

    // LEDs
    for( int l=0; l<N_LEDS; l++) {
        led_data[4 + (l*4) + 0] = 0xf0;

        hsv[0] = l + frame;
        hsv[1] = 255;
        hsv[2] = 255;

        hsv2rgb();

        led_data[4 + (l*4) + 1] = rgb[2];
        led_data[4 + (l*4) + 2] = rgb[1];
        led_data[4 + (l*4) + 3] = rgb[0];
    }

    // end frame
    for( int i=0; i<4; i++) {
        led_data[N_LEDS*4 + i] = 0xff;
    }

    // Serial.print("LED buffer(");
    // Serial.print(led_buffer_size);
    // Serial.print("): 0x");
    // for( int i=0; i<led_buffer_size; i++){
    //     if( led_data[i] < 0x10){ Serial.print(0); }
    //     Serial.print(led_data[i], HEX);
    // }
    // Serial.println();

    // Serial.println("Set buf write pointer (EWRPTL:EWRPTH) byte (0x02:0x03) to 0x0000");
    enc_write_reg(EWRPTL, 0x00);
    enc_write_reg(EWRPTH, 0x00);
    // Serial.println("Write bytes to buffer");

    enc_write_buf(led_data, led_buffer_size);
    // Serial.println("Set buf read pointer (ERDPTL:ERDPTH) byte (0x00:0x01) to 0x0000");
    enc_write_reg(ERDPTL, 0x00);
    enc_write_reg(ERDPTH, 0x00);
    // Serial.println("read bytes from buffer to LED");

    enc_read_buf((byte *) 0, led_buffer_size);
}

void loop() {

    // Reset chip
    enc_soft_reset();

    // demo_read_reg();
    // demo_write_reg();
    // demo_bitflip();
    // demo_buf_write();
    demo_rainbows();
}
