#ifndef ENC_UTIL_H
#define ENC_UTIL_H

#define ARRAY_LEN(x) sizeof(x)/sizeof(x[0])

extern long timer_start;
inline long stop_timer();
inline void start_timer();

#endif
