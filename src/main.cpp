#include <SPI.h>
#include <inttypes.h>
#include "ENC28J60.h"
#include <arduino.h>

#define N_LEDS 60

SPISettings spiSettings;

void setup() {
    Serial.begin(9600);
    Serial.println("SETUP");

    // Set Pin Modes
    pinMode(CS_PIN, OUTPUT);
    pinMode(INT_PIN, INPUT);
    SPI.begin();
    spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE);

    // Reset chip
    enc_soft_reset();

    // timer
    timer_start = millis();
}

/**
 * Receive ethernet packets from the ENC
 */
void demo_receive() {
    enc_hw_init();

    // Enable interrupt on receive packet

    // Set MAC Addr
    byte mac_addr[] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
    enc_set_mac_addr(mac_addr);

    while(1) {
        enc_hw_enable();

        if(digitalRead(INT_PIN) == HIGH){
            Serial.println("int pint high as fuck");
        } else {
            Serial.println("int pin low");
        }

        enc_regs_debug();
        enc_hw_disable();
        delay(1000);
    }

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

    demo_receive();
    do {delay(10);} while (!Serial.available());
    while (Serial.available()){Serial.read();delay(1);}

}
