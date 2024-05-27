#ifndef EVNISRTimer_h
#define EVNISRTimer_h
#include <Arduino.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"

class EVNISRTimer0
{
public:
    static const uint8_t  CORE0_TIMER_NUM = 1;
    EVNISRTimer0() {};

    void begin()
    {
        if (!_started)
        {
            _started = true;
            pool = alarm_pool_create(CORE0_TIMER_NUM, 16);
        }
    };

    static repeating_timer& sharedISRTimer(uint8_t timer_no) { static EVNISRTimer0 shared; shared.begin(); return shared.timers[timer_no]; }
    static alarm_pool* sharedAlarmPool() { static EVNISRTimer0 shared; shared.begin(); return shared.pool; }

private:
    static struct alarm_pool* pool;
    static struct repeating_timer timers[16];
    static bool _started;
};

class EVNISRTimer1
{
public:
    static const uint8_t  CORE1_TIMER_NUM = 2;

    EVNISRTimer1() {};
    void begin()
    {
        if (!_started)
        {
            _started = true;
            pool = alarm_pool_create(CORE1_TIMER_NUM, 16);
        }
    };

    static repeating_timer& sharedISRTimer(uint8_t timer_no) { static EVNISRTimer1 shared; shared.begin(); return shared.timers[timer_no]; }
    static alarm_pool* sharedAlarmPool() { static EVNISRTimer1 shared; shared.begin(); return shared.pool; }

private:
    static struct alarm_pool* pool;
    static struct repeating_timer timers[16];
    static bool _started;
};

#endif