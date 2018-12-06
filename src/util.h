#ifndef ENC_UTIL_H
#define ENC_UTIL_H

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

#endif
