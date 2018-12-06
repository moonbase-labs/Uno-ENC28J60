#include <Arduino.h>
#include "util.h"

long timer_start;
long g_last_print = 0;

inline void start_timer() {
    timer_start = millis();
}

inline long stop_timer() {
    return millis() - timer_start;
}
