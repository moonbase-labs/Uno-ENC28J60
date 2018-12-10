/**
 Demo code used to test connectivity with ENC, not actually used
 */

#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>
#include "ENC28J60.h"
#include "util.h"
#include "demo.h"
#include "enc_spi.h"

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
//         Serial.print(F("read register 0x"));
//         if(reg_addr < 0x10) {Serial.print(0);}
//         print_hex_byte(reg_addr);
//         Serial.print(F(", response: 0x"));
//         if(result < 0x10) {Serial.print(0);}
//         println_hex_byte(result);
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
//
// void demo_bitflip() {
//     byte result;
//
//     Serial.print(F("View the ECON1 register (should be initialized to 0x00): 0x"));
//     result = enc_read_reg(ECON1);
//     println_hex_byte(result);
//     Serial.print(F("Set the ECON1_RXEN bit field (Enable Recieve)"));
//     enc_bit_set(ECON1, ECON1_RXEN);
//     Serial.print(F("View the ECON1 register (should be set to 0x04): 0x"));
//     result = enc_read_reg(ECON1);
//     println_hex_byte(result);
//     Serial.print(F("clear the ECON1_RXEN bit field (Enable Recieve)"));
//     enc_bit_clr(ECON1, ECON1_RXEN);
//     Serial.print(F("View the ECON1 register (should be set to 0x00): 0x"));
//     result = enc_read_reg(ECON1);
//     println_hex_byte(result);
// }
//
// void demo_write_reg() {
//     byte result;
//     Serial.print(F("View the ERXSTL:ERXSTH registers (should be initialized to 0xfa, 0x05): 0x"));
//     result = enc_read_reg(ERXSTL);
//     print_hex_byte(result);
//     Serial.print(F(", 0x"));
//     // result = enc_read_reg(ERXSTH);
//     println_hex_byte(result);
//
//     Serial.println(F("set the ERXSTL:ERXSTH registers to (0xaa, 0x05):"));
//     enc_write_reg(ERXSTL, 0xaa);
//     // enc_write_reg(ERXSTH, 0x05);
//
//     Serial.print(F("View the ERXSTL:ERXSTH registers (should now be set to 0xaa, 0x05): 0x"));
//     result = enc_read_reg(ERXSTL);
//     print_hex_byte(result);
//     Serial.print(F(", 0x"));
//     // result = enc_read_reg(ERXSTH);
//     println_hex_byte(result);
//
//     Serial.println(F("return the ERXSTL:ERXSTH registers to (0xfa, 0x05):"));
//     enc_write_reg(ERXSTL, 0xfa);
//     // enc_write_reg(ERXSTH, 0x05);
//
//     // Doesn't work in SPI_MODE1, SPI_MODE2,
//     // Works in SPI_MODE3
// }
//
// void demo_write_reg_2() {
//     byte result;
//     Serial.print(F("View the ERXNDL:ERXNDH registers (should be initialized to 0xff, 0x1f): 0x"));
//     result = enc_read_reg(ERXNDL);
//     print_hex_byte(result);
//     Serial.print(F(", 0x"));
//     result = enc_read_reg(ERXNDH);
//     println_hex_byte(result);
//
//     Serial.println(F("set the ERXNDL:ERXNDH registers to (0xff, 0x07):"));
//     enc_write_reg(ERXNDL, 0xff);
//     enc_write_reg(ERXNDH, 0x07);
//
//     Serial.print(F("View the ERXNDL:ERXNDH registers (should now be set to 0xff, 0x07): 0x"));
//     result = enc_read_reg(ERXNDL);
//     print_hex_byte(result);
//     Serial.print(F(", 0x"));
//     result = enc_read_reg(ERXNDH);
//     println_hex_byte(result);
//
//     Serial.println(F("return the ERXNDL:ERXNDH registers to (0xff, 0x1f):"));
//     enc_write_reg(ERXNDL, 0xff);
//     enc_write_reg(ERXNDH, 0x1f);
// }
//
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
//     Serial.println(F("Set buf write pointer (EWRPTL:EWRPTH) byte (0x02:0x03) to 0x0000"));
//     enc_write_reg(EWRPTL, 0x00);
//     enc_write_reg(EWRPTH, 0x00);
//     Serial.println(F("Write bytes to buffer"));
//
//     enc_write_buf(testdata, ARRAY_LEN(testdata));
//     Serial.println(F("Set buf read pointer (ERDPTL:ERDPTH) byte (0x00:0x01) to 0x0000"));
//     enc_write_reg(ERDPTL, 0x00);
//     enc_write_reg(ERDPTH, 0x00);
//     Serial.print(F("read bytes from buffer: 0x"));
//
//     enc_read_buf(results, ARRAY_LEN(testdata));
//
//     for(uint8_t i=0; i<ARRAY_LEN(testdata); i++){
//         result = results[i];
//         if(result < 0x10) {Serial.print(0);}
//         print_hex_byte(result);
//     }
//     Serial.println();
// }

int led_buffer_size = 4 * (N_LEDS + 2);

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
    byte * led_data;
    if(!g_enc_eth_frame_buf){
        led_data = (byte *) malloc(led_buffer_size);
    } else {
        led_data = &g_enc_eth_frame_buf[
            RSV_LEN + ETH_HEADER_BYTES + SEQ_BYTES
        ];
    }




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

    // Serial.print(F("LED buffer("));
    // Serial.print(led_buffer_size);
    // Serial.print(F("): 0x"));
    // for( int i=0; i<led_buffer_size; i++){
    //     if( led_data[i] < 0x10){ Serial.print(0); }
    //     Serial.print(led_data[i], HEX);
    // }
    // Serial.println();

    // Serial.println(F("Set buf write pointer (EWRPTL:EWRPTH) byte (0x02:0x03) to 0x0000"));
    enc_write_reg(EWRPTL, 0x00);
    enc_write_reg(EWRPTH, 0x00);
    // Serial.println(F("Write bytes to buffer"));

    enc_write_buf(led_data, led_buffer_size);
    // Serial.println(F("Set buf read pointer (ERDPTL:ERDPTH) byte (0x00:0x01) to 0x0000"));
    enc_write_reg(ERDPTL, 0x00);
    enc_write_reg(ERDPTH, 0x00);
    // Serial.println(F("read bytes from buffer to LED"));

    enc_read_buf((byte *) 0, led_buffer_size);

    if(!g_enc_eth_frame_buf){
        free(led_data);
    }
}

/**
 *
 * CURRENT PROBLEM:
 *
 * Write 0x0d (
 *     MACON1_MARXEN
 *     |MACON1_TXPAUS
 *     |MACON1_RXPAUS
 * ) to MACON1
 * Stays 0x0d until write 0x33 (
 *     MACON3_PADCFG0
 *     |MACON3_TXCRCEN
 *     |MACON3_FRMLNEN
 *     |MACON3_FULDPX
 * ) to MACON3
 * and then it changes to 0x13 (
 *     Reserved: maintan as 0
 *     |MACON1_PASSALL
 *     |MACON1_MARXEN
 * )
 * which doesn't make sense!
 *
 * Another version:
 *
 * Write 0x0d (
 *     MACON1_MARXEN
 *     |MACON1_TXPAUS
 *     |MACON1_RXPAUS
 * ) to MACON1
 * Stays 0x0d until write 0x01 (
 *     MACON3_FULDPX
 * ) to MACON3
 * and then it changes to 0x1
 *
 * Another version:
 *
 * Write 0x0d (
 *     MACON1_MARXEN
 *     |MACON1_TXPAUS
 *     |MACON1_RXPAUS
 * ) to MACON1
 * Stays 0x0d until write 0x02 (
 *     MACON3_FRMLNEN
 * ) to MACON3
 * and then it changes to 0x2
 *
 * Another version:
 *
 * Write 0x0d (
 *     MACON1_MARXEN
 *     |MACON1_TXPAUS
 *     |MACON1_RXPAUS
 * ) to MACON1
 * Stays 0x0d until write 0x10 (
 *     MACON3_TXCRCEN
 * ) to MACON3
 * and then it changes to 0x10


 */

// void demo_mac_interference() {
//     byte reg;
//
//     /* enable MAC receive, TX/RX Pause = 0x0d */
//     reg = 0x00;
//     reg |= MACON1_MARXEN; // Enable MAC to recieve frames
//     reg |= MACON1_TXPAUS; // Allow IEEE defined flow control to function
//     reg |= MACON1_RXPAUS; // Allow IEEE defined flow control to function
//     enc_write_reg(MACON1, reg);
//
//     Serial.print(F("MACON1 should be 0x0d: ")); enc_reg_print(MACON1); Serial.println();
//     Serial.print(F("MACON3: ")); enc_reg_print(MACON3); Serial.println();
//
//     reg = 0x00;
//     // All short frames will be zero padded to 60 bytes and a valid CRC
//     // will then be appended
//     // reg |= MACON3_PADCFG0;
//     // MAC will append a valid CRC to all frames transmitted regardless of
//     // PADCFG bits. TXCRCEN must be set if the PADCFG bits specify that a
//     // valid CRC will be appended
//     // reg |= MACON3_TXCRCEN;
//     // The type/length field of transmitted and received frames will be
//     // checked. If it represents a length, the frame size will be compared
//     // and mismatches will be reported in the transmit/receive status vector.
//     // reg |= MACON3_FRMLNEN;
//     //
//     reg |= MACON3_FULDPX;
//     enc_write_reg(MACON3, reg);
//
//     Serial.print(F("MACON1 should be 0x0d: ")); enc_reg_print(MACON1); Serial.println();
//     Serial.print(F("MACON3: ")); enc_reg_print(MACON3); Serial.println();
// }

/**
 * Receive ethernet packets from the ENC
 */
void demo_receive() {

    byte * led_data;
    if(!g_enc_eth_frame_buf){
        led_data = (byte *) malloc(led_buffer_size);
    } else {
        led_data = &g_enc_eth_frame_buf[
            NPP_SIZE + RSV_LEN + ETH_HEADER_BYTES + SEQ_BYTES
        ];
    }


    enc_hw_init();

    // do {} while (!Serial.available());
    // while (Serial.available()){Serial.read();delay(1);}
    // return;

    enc_regs_debug();

    // Enable interrupt on receive packet

    // Set MAC Addr
    byte mac_addr[] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
    enc_set_mac_addr(mac_addr);

    uint16_t old_erdpt;
    // Determines whether packet is dumped onto LEDs
    bool dump_packet;


    enc_hw_enable();

    // g_enc_debug_io = true;

    uint16_t prev_sequence = 0;


    do {
        // delay(100);
        // g_enc_repeat_breakpoints = false;
        // g_enc_debug_io = false;


        if(g_enc_err) {
            enc_regs_debug();
            Serial.print(F("ERROR, resetting "));
            Serial.println(g_enc_err);
            g_enc_err = ENC_NO_ERR;
            break;
        }

        // enc_hw_enable();

        if(digitalRead(INT_PIN) == HIGH){
            if (SOMETIMES_PRINT_COND & DEBUG_ETH) {
                Serial.println(F("int pint high as fuck"));
                SOMETIMES_PRINT_END;
            }
            continue;
        } else {
            if( DEBUG_ETH ) {
                Serial.println(F("int pin low"));
            }
        }

        byte epktcnt = enc_read_reg(EPKTCNT);

        if( DEBUG_ETH_BASIC & SOMETIMES_PRINT_COND) {
            Serial.print(F("epktcnt: "));
            println_hex_byte(epktcnt);
            SOMETIMES_PRINT_END;
        }

        if(!epktcnt) {
            if (SOMETIMES_PRINT_COND & DEBUG_ETH) {
                Serial.println(F("No packets yet"));
                SOMETIMES_PRINT_END;
            }
            continue;
        }

        old_erdpt = enc_read_regw(ERDPTL);

        if(DEBUG_ETH){
            Serial.print(F("buffer at 0x"));
            print_hex_byte(old_erdpt);
            Serial.println(F(": "));
            enc_peek_buf_slow(0, 100);
        }

        if(g_enc_err) {
            enc_regs_debug();
            Serial.print(F("ERROR, resetting "));
            Serial.println(g_enc_err);
            g_enc_err = ENC_NO_ERR;
            break;
        }

        if( DEBUG_ETH_BASIC & SOMETIMES_PRINT_COND) {
            Serial.print(F("old erdpt: "));
            println_hex_word(old_erdpt);
            SOMETIMES_PRINT_END;
        }

        _enc_refresh_rsv_globals();

        /* TODO: move this to separate validation function? */

        // if( DEBUG_ETH ) {
        //     Serial.print(F("next packet: 0x"));
        //     print_hex_word(*g_enc_npp);
        //     Serial.print(F(", rxbcnt: "));
        //     Serial.println(*g_enc_rxbcnt);
        // }

        if(RSV_GETBIT(*g_enc_rxstat, RSV_ZERO)) {
            Serial.println(F("RXSTAT corrupt, resetting"));
            _enc_print_rxstat(*g_enc_rxstat);
            enc_regs_debug();
            break;
        }

        /*
        Data sheet says:
            When a packet is accepted and completely
            written into the buffer, the EPKTCNT register will increment,
            the EIR.PKTIF bit will be set, an interrupt will be
            generated (if enabled) and the Hardware Write Pointer,
            ERXWRPT, will automatically advance.
        This means if EPKTCNT > 0, the first packet in the buffer must be
        fully written.
         */

        if(
            RSV_GETBIT(*g_enc_rxstat, RSV_RXCONTROLFRAME)
            || RSV_GETBIT(*g_enc_rxstat, RSV_CRCERROR)
            || RSV_GETBIT(*g_enc_rxstat, RSV_DRIBBLENIBBLE)
        ) {
            // don't care about control frames, CRC or Dribblenibbles yet
            free_packet();
            continue;
        }

        if(!RSV_GETBIT(*g_enc_rxstat, RSV_RXOK)) {
            Serial.println(F("RX not ok, "));
            Serial.println(F("rxstat:"));
            _enc_print_rxstat(*g_enc_rxstat);
            enc_regs_debug();
            free_packet();
            continue;
        }

        if((*g_enc_rxbcnt > MAX_FRAMELEN)) {
            Serial.print(F("Length not ok, "));
            Serial.print(*g_enc_rxbcnt);
            Serial.print(F(" > MAX_FRAMELEN: "));
            Serial.println(MAX_FRAMELEN);
            Serial.println(F("rxstat:"));
            _enc_print_rxstat(*g_enc_rxstat);
            Serial.println(F("header: "));
            _enc_dump_pkt(ETH_HEADER_BYTES);
            break;
        }

        if((*g_enc_rxbcnt < ETH_HEADER_BYTES)) {
            Serial.print(F("Length not ok, "));
            Serial.print(*g_enc_rxbcnt);
            Serial.print(F(" < ETH_HEADER_BYTES: "));
            Serial.println(ETH_HEADER_BYTES);
            Serial.println(F("rxstat:"));
            _enc_print_rxstat(*g_enc_rxstat);
            break;
        }

        if(((int)(*g_enc_npp) < RXSTART_INIT)) {
            Serial.print(F("Next Packet Pointer out of bounds: 0x"));
            print_hex_word(*g_enc_npp);
            Serial.print(F(" < RXSTART_INIT: 0x"));
            println_hex_word(RXSTART_INIT);
            break;
        }

        if(((int)(*g_enc_npp) > RXSTOP_INIT)) {
            Serial.print(F("Next Packet Pointer out of bounds: 0x"));
            print_hex_word(*g_enc_npp);
            Serial.print(F(" > RXSTOP_INIT: 0x"));
            println_hex_word(RXSTOP_INIT);
            break;
        }

        if( DEBUG_ETH ) {
            Serial.println(F("packet: "));
            _enc_dump_pkt(*g_enc_rxbcnt);
            _enc_print_rxstat(*g_enc_rxstat);
            // enc_peek_buf(ETH_HEADER_BYTES, *g_enc_rxbcnt-ETH_HEADER_BYTES);
        }

        /**
         *  Reset ERDPT to start of ethernet body
         *  TODO: This may not be necessary if buffer is managed correctly.
         */
        uint16_t data_start = _buffer_sum(old_erdpt, NPP_SIZE + RSV_LEN + ETH_HEADER_BYTES);
        uint16_t data_len = _buffer_distance(data_start, *g_enc_npp);
        enc_write_regw(ERDPTL, data_start);

        enc_read_buf((byte *)(g_enc_sequence), SEQ_BYTES);

        /**
         *  ERDPT Is now 2 bytes in to ethernet body (LED data starts here)
         */

        if(DEBUG_ETH_BASIC) {
            Serial.print(F("sequence: "));
            print_hex_word(*g_enc_sequence);
            Serial.print(F(", rxbcnt: "));
            Serial.println(*g_enc_rxbcnt);
        }

        dump_packet = true;

        if(
            (prev_sequence >= *g_enc_sequence)
            && (prev_sequence - *g_enc_sequence < (MAX_SEQUENCE / 2))
        ) {
            Serial.print(F("out of order! prev: "));
            print_hex_word(prev_sequence);
            Serial.print(", this: ");
            println_hex_word(*g_enc_sequence);

            dump_packet = false;
        }

        if(*g_enc_sequence - prev_sequence > 1) {
            Serial.print(F("skipped seq! prev: "));
            print_hex_word(prev_sequence);
            Serial.print(", this: ");
            println_hex_word(*g_enc_sequence);
        }

        if(dump_packet) {
            // if(DEBUG_ETH || 1) {
            enc_read_buf(led_data, data_len);
            if(DEBUG_LED) {
                for(uint16_t i=0; i<50; i++) print_hex_byte(led_data[i]);
                Serial.print(' ');
            }

            prev_sequence = *g_enc_sequence;

            // }
            // enc_read_buf(0, *g_enc_rxbcnt - 1);
        }

        free_packet();

        // print summary

        if( SOMETIMES_PRINT_COND) {
            float pps = (float)(1000.0 * g_enc_pkts_consumed) / (float)(probe_timer());
            Serial.print(F("SEQ:"));
            print_hex_word(*g_enc_sequence);
            Serial.print(F(" CNT:"));
            print_hex_byte(epktcnt);
            Serial.print(F(" PPS:"));
            Serial.print(pps);
            Serial.println();
            SOMETIMES_PRINT_END;
        }

    } while (1);

    enc_hw_disable();

    if(!g_enc_eth_frame_buf){
        free(led_data);
    }

}
}
