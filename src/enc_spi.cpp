#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>
#include "enc_spi.h"

/**
 * SPI stuff
 * could have its own source file
 */
SPISettings spiSettings;


/**
 * Initialize SPI
 */
void spi_init() {
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
}
