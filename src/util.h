#ifndef ENC_UTIL_H
#define ENC_UTIL_H

#include <Arduino.h>
/**
 * Debugging
 */
#define PRINT_DELAY 200

#define SOMETIMES_PRINT_COND (millis() - g_last_print > PRINT_DELAY)
#define SOMETIMES_PRINT_END (g_last_print = millis())
#define ARRAY_LEN(x) sizeof(x)/sizeof(x[0])


extern long timer_start;
extern long g_last_print;
long probe_timer();
void start_timer();

void print_hex_byte(byte arg);
void print_hex_word(uint16_t arg);
void println_hex_byte(byte arg);
void println_hex_word(uint16_t arg);

#endif
