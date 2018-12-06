#include <inttypes.h>
#include "ENC28J60.h"
#include "demo.h"
#include "enc_spi.h"
#include <arduino.h>


void setup() {
    Serial.begin(115200);
    Serial.println(F("SETUP"));

    spi_init();

    Serial.print(F("SPI_MODE is 0x"));
    Serial.println(SPI_MODE, HEX);
    Serial.print(F("SPCR is 0x"));
    Serial.println(SPCR, HEX);
    Serial.print(F("SPSR is 0x"));
    Serial.println(SPSR, HEX);

    // Serial.print(F("g_enc_eth_frame_buf: 0x"));
    // Serial.println((int)(g_enc_eth_frame_buf), HEX);

    // Reset chip
    // enc_soft_reset();

    // timer
    timer_start = millis();
}

void loop() {

    // Reset chip
    enc_soft_reset();

    // demo_read_reg();
    // demo_write_reg();
    // demo_write_reg_2();
    // demo_bitflip();
    // demo_buf_write();
    // demo_rainbows();
    // demo_mac_interference();
    demo_receive();

    do {delay(10);} while (!Serial.available());
    while (Serial.available()){Serial.read();delay(1);}

}
