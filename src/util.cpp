#include <Arduino.h>
#include "util.h"

long timer_start;
long g_last_print = 0;

void start_timer() {
    timer_start = millis();
}

long probe_timer() {
    return millis() - timer_start;
}

void print_hex_byte(byte arg) {
    if(arg<0x10) Serial.print(0);
    Serial.print(arg, HEX);
}

void println_hex_byte(byte arg) {
    if(arg<0x10) Serial.print(0);
    Serial.println(arg, HEX);
}

void print_hex_word(uint16_t arg) {
    if(arg<0x10) Serial.print(0);
    if(arg<0x100) Serial.print(0);
    if(arg<0x1000) Serial.print(0);
    Serial.print(arg, HEX);
}

void println_hex_word(uint16_t arg) {
    if(arg<0x10) Serial.print(0);
    if(arg<0x100) Serial.print(0);
    if(arg<0x1000) Serial.print(0);
    Serial.println(arg, HEX);
}
