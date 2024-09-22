#include "EVNISRTimer.h"

repeating_timer_t EVNISRTimer0::timers[16];
alarm_pool_t* EVNISRTimer0::pool;
bool EVNISRTimer0::_started = false;


repeating_timer_t EVNISRTimer1::timers[16];
alarm_pool_t* EVNISRTimer1::pool;
bool EVNISRTimer1::_started = false;
