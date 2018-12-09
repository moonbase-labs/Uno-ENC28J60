
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

extern unsigned int __heap_start;
extern void *__brkval;

/*
 * The free list structure as maintained by the
 * avr-libc memory allocation routines.
 */
struct __freelist {
  size_t sz;
  struct __freelist *nx;
};

/* The head of the free list structure */
extern struct __freelist *__flp;

#include "util.h"

/* Calculates the size of the free list */
int freeListSize() {
  struct __freelist* current;
  int total = 0;
  for (current = __flp; current; current = current->nx) {
    total += 2; /* Add two bytes for the memory block's header  */
    total += (int) current->sz;
  }
  return total;
}

int freeMemory() {
  int free_memory;
  if ((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__heap_start);
  } else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
    free_memory += freeListSize();
  }
  return free_memory;
}


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
