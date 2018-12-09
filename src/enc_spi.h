#ifndef __ENC_SPI_H__
#define __ENC_SPI_H__

#include <Arduino.h>
#include <SPI.h>
#include <inttypes.h>

/**
 * SPI Settings
 */
#define SPI_MODE SPI_MODE0

// #define SPI_SPEED 50000
#define SPI_SPEED 9000000
#define INT_PIN 9
// #define USE_SPI_LIBRARY 1

/**
 * Timing
 */

#define NANOS_2_MILLIS(x) (x/1000)

#define CS_PIN 10
// CS Setup time, micros (Table 16-6)
#define CS_SETUP_US NANOS_2_MILLIS(50)
// CS Hold time, micros (Table 16-6)
#define CS_HOLD_US NANOS_2_MILLIS(10)
// CS Hold time, micros for MAC & MII  (Table 16-6)
#define CS_HOLD_M_US NANOS_2_MILLIS(210)
// CS Disable time, micros (Table 16-6)
#define CS_DISABLE_US NANOS_2_MILLIS(50)
#define MODE_CHANGE_US 10
#define WORD_US 10



#define waitspi() while(!(SPSR&(1<<SPIF)))

#ifdef USE_SPI_LIBRARY
    extern SPISettings spiSettings;
#endif

inline void _changeSPIMode(byte mode) {
    delayMicroseconds(MODE_CHANGE_US);
    SPCR = (SPCR & (~SPI_MODE3)) | mode;
    delayMicroseconds(MODE_CHANGE_US);
}

inline void _beginTransaction() {

    // digitalWrite(MOSI, LOW);
    // digitalWrite(SCK, LOW);
    digitalWrite(CS_PIN, LOW);

    delayMicroseconds(CS_SETUP_US);
    #ifdef USE_SPI_LIBRARY
        SPI.beginTransaction(spiSettings);
    #else
        noInterrupts();
        // _changeSPIMode(SPI_MODE);
    #endif
}

inline byte _spiRead() {
    byte response;
    #ifdef USE_SPI_LIBRARY
        response = SPI.transfer(0);
    #else
        // _changeSPIMode(SPI_MODE);
        SPDR = 0;
        waitspi();
        response = SPDR;
        // digitalWrite(MOSI, LOW);
    #endif
    delayMicroseconds(WORD_US);
    return response;
}

inline void _spiWrite(byte data) {
    #ifdef USE_SPI_LIBRARY
        SPI.transfer(data);
    #else
        // _changeSPIMode(SPI_MODE);
        SPDR = data;
        waitspi();
        // _changeSPIMode(SPI_MODE);
        // digitalWrite(MOSI, LOW);
    #endif
    delayMicroseconds(WORD_US);
}

inline void _endTransaction() {
    #if USE_SPI_LIBRARY
        SPI.endTransaction();
    #else
        interrupts();
    #endif
    digitalWrite(CS_PIN, HIGH);
    // digitalWrite(MOSI, LOW);
    delayMicroseconds(CS_DISABLE_US);
}

void spi_init();

#endif
