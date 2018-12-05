#include <SPI.h>
#include <inttypes.h>
#include "ENC28J60.h"
#include "demo.h"
#include <arduino.h>

SPISettings spiSettings;

void setup() {
    Serial.begin(9600);
    Serial.println("SETUP");

    // Set Pin Modes
    pinMode(CS_PIN, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(INT_PIN, INPUT);
    pinMode(MISO, INPUT);
    #ifdef USE_SPI_LIBRARY
        SPI.begin();
        spiSettings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE);
    #else
        SPCR =
            (1<<SPE) // SPI Enable
            |(1<<MSTR) // SPI Master
            |SPI_MODE
            // |(1<<SPR1)|(1<<SPR0); // SPI Clk divider = 128
        ;
    #endif

    digitalWrite(MOSI, LOW);
    digitalWrite(SCK, LOW);
    digitalWrite(CS_PIN, HIGH);

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
    demo_mac_interference();
    // demo_receive();

    do {delay(10);} while (!Serial.available());
    while (Serial.available()){Serial.read();delay(1);}

}
