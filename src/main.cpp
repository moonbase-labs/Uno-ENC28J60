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
            |(1<<SPR1)|(1<<SPR0); // SPI Clk divider = 128
    #endif

    digitalWrite(MOSI, LOW);
    digitalWrite(SCK, LOW);
    digitalWrite(CS_PIN, HIGH);

    Serial.print("SPCR is 0x");
    Serial.println(SPCR, HEX);
    Serial.print("SPSR is 0x");
    Serial.println(SPSR, HEX);

    // Reset chip
    enc_soft_reset();

    // timer
    timer_start = millis();
}

void demo_bitflip() {
    byte result;

    Serial.print("View the ECON1 register (should be initialized to 0x00): 0x");
    result = enc_read_reg(ECON1);
    Serial.println(result, HEX);
    Serial.print("Set the ECON1_RXEN bit field (Enable Recieve)");
    enc_bit_set(ECON1, ECON1_RXEN);
    Serial.print("View the ECON1 register (should be set to 0x40): 0x");
    result = enc_read_reg(ECON1);
    Serial.println(result, HEX);
    Serial.print("clear the ECON1_RXEN bit field (Enable Recieve)");
    enc_bit_clr(ECON1, ECON1_RXEN);
    Serial.print("View the ECON1 register (should be set to 0x00): 0x");
    result = enc_read_reg(ECON1);
    Serial.println(result, HEX);
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
        delay(100);
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
