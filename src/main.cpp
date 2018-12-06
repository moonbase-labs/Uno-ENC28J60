#include <inttypes.h>
#include "ENC28J60.h"
#include "demo.h"
#include "enc_spi.h"
#include <arduino.h>


void setup() {
    Serial.begin(115200);
    Serial.println("SETUP");

    spi_init();

    Serial.print("SPI_MODE is 0x");
    Serial.println(SPI_MODE, HEX);
    Serial.print("SPCR is 0x");
    Serial.println(SPCR, HEX);
    Serial.print("SPSR is 0x");
    Serial.println(SPSR, HEX);

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
