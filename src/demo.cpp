/**
 Demo code used to test connectivity with ENC, not actually used
 */

#if 0

#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>
#include "ENC28J60.h"
#include "util.h"

void demo_read_reg() {
    byte result;
    byte reg_addr;

    static byte allregs[] = {
        ERDPTL, ERDPTH, EWRPTL, EWRPTH, ETXSTL, ETXSTH, ETXNDL, ETXNDH, ERXSTL,
        ERXSTH, ERXNDL, ERXNDH, ERXRDPTL, ERXRDPTH, ERXWRPTL, ERXWRPTH, EDMASTL,
        EDMASTH, EDMANDL, EDMANDH, EDMADSTL, EDMADSTH, EDMACSL, EDMACSH,

        EHT0, EHT1, EHT2, EHT3, EHT4, EHT5, EHT6, EHT7, EPMM0, EPMM1, EPMM2,
        EPMM3, EPMM4, EPMM5, EPMM6, EPMM7, EPMCSL, EPMCSH, EPMOL, EPMOH, EWOLIE,
        EWOLIR, ERXFCON, EPKTCNT,

        MACON1, MACON3, MACON4, MABBIPG, MAIPGL, MAIPGH, MACLCON1, MACLCON2,
        MAMXFLL, MAMXFLH, MAPHSUP, MICON, MICMD, MIREGADR, MIWRL, MIWRH, MIRDL,
        MIRDH,

        MAADR1, MAADR0, MAADR3, MAADR2, MAADR5, MAADR4, EBSTSD, EBSTCON,
        EBSTCSL, EBSTCSH, MISTAT, EREVID, ECOCON, EFLOCON, EPAUSL, EPAUSH,
    };

    for (uint8_t i=0; i<ARRAY_LEN(allregs); i++) {
        reg_addr = allregs[i];
        result = enc_read_reg(reg_addr);
        Serial.print("read register 0x");
        if(reg_addr < 0x10) {Serial.print(0);}
        Serial.print(reg_addr, HEX);
        Serial.print(", response: 0x");
        if(result < 0x10) {Serial.print(0);}
        Serial.println(result, HEX);
    }

    // Should look like
    // FA
    // 05
    // 00
    // 00
    // 00
    // 00
    // 00
    // 00
    // FA
    // 05
    // FF
    // 1F
    // FA
    // 05
    // 00 ...

    // Works in SPI_MODE1, SPI_MODE3
    // Doesn't work in SPI_MODE0
}

void demo_write_reg() {
    byte result;
    Serial.print("View the ERXSTL:ERXSTH registers (should be initialized to 0xfa, 0x05): 0x");
    result = enc_read_reg(ERXSTL);
    Serial.print(result, HEX);
    Serial.print(", 0x");
    result = enc_read_reg(ERXSTH);
    Serial.println(result, HEX);

    Serial.println("set the ERXSTL:ERXSTH registers to (0xaa, 0x05):");
    enc_write_reg(ERXSTL, 0xaa);
    enc_write_reg(ERXSTH, 0x05);

    Serial.print("View the ERXSTL:ERXSTH registers (should now be set to 0xaa, 0x05): 0x");
    result = enc_read_reg(ERXSTL);
    Serial.print(result, HEX);
    Serial.print(", 0x");
    result = enc_read_reg(ERXSTH);
    Serial.println(result, HEX);

    Serial.println("return the ERXSTL:ERXSTH registers to (0xfa, 0x05):");
    enc_write_reg(ERXSTL, 0xfa);
    enc_write_reg(ERXSTH, 0x05);

    // Doesn't work in SPI_MODE1, SPI_MODE2,
    // Works in SPI_MODE3
}

void demo_write_reg_2() {
    byte result;
    Serial.print("View the ERXNDL:ERXNDH registers (should be initialized to 0xff, 0x1f): 0x");
    result = enc_read_reg(ERXNDL);
    Serial.print(result, HEX);
    Serial.print(", 0x");
    result = enc_read_reg(ERXNDH);
    Serial.println(result, HEX);

    Serial.println("set the ERXNDL:ERXNDH registers to (0xff, 0x07):");
    enc_write_reg(ERXNDL, 0xff);
    enc_write_reg(ERXNDH, 0x07);

    Serial.print("View the ERXNDL:ERXNDH registers (should now be set to 0xff, 0x07): 0x");
    result = enc_read_reg(ERXNDL);
    Serial.print(result, HEX);
    Serial.print(", 0x");
    result = enc_read_reg(ERXNDH);
    Serial.println(result, HEX);

    Serial.println("return the ERXNDL:ERXNDH registers to (0xff, 0x1f):");
    enc_write_reg(ERXNDL, 0xff);
    enc_write_reg(ERXNDH, 0x1f);
}

void demo_buf_write() {
    static byte testdata[] = {
        0x00, 0x00, 0x00, 0x00, // START Frame
        0xf0, 0xff, 0x00, 0x00, // Blue
        0xf0, 0x00, 0xff, 0x00, // Green
        0xf0, 0x00, 0x00, 0xff, // Red
        0xff, 0xff, 0xff, 0xff, // END Frame
    };
    byte * results = (byte *) malloc(sizeof(testdata));
    byte result;

    Serial.println("Set buf write pointer (EWRPTL:EWRPTH) byte (0x02:0x03) to 0x0000");
    enc_write_reg(EWRPTL, 0x00);
    enc_write_reg(EWRPTH, 0x00);
    Serial.println("Write bytes to buffer");

    enc_write_buf(testdata, ARRAY_LEN(testdata));
    Serial.println("Set buf read pointer (ERDPTL:ERDPTH) byte (0x00:0x01) to 0x0000");
    enc_write_reg(ERDPTL, 0x00);
    enc_write_reg(ERDPTH, 0x00);
    Serial.print("read bytes from buffer: 0x");

    enc_read_buf(results, ARRAY_LEN(testdata));

    for(uint8_t i=0; i<ARRAY_LEN(testdata); i++){
        result = results[i];
        if(result < 0x10) {Serial.print(0);}
        Serial.print(result, HEX);
    }
    Serial.println();
}

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

#endif
