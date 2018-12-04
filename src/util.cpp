#include <Arduino.h>
#include "util.h"

long timer_start;

inline void start_timer() {
    timer_start = millis();
}

inline long stop_timer() {
    return millis() - timer_start;
}
