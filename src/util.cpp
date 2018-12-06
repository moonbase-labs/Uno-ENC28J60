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
