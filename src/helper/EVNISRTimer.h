#ifndef EVNISRTimer_h
#define EVNISRTimer_h
#include <Arduino.h>
#include "pico/stdlib.h"

#define CORE0_TIMER_NUM 1
#define CORE1_TIMER_NUM 2

class EVNISRTimer
{
public:
    EVNISRTimer(uint8_t timer_num)
    {
        _timer_num = timer_num;
    };

    void begin()
    {
        if (!_started)
        {
            _pool = alarm_pool_create(_timer_num, 16);

            //set timer interrupts to be lower priority than pin change interrupts
            switch (_timer_num)
            {
            case 0:
                irq_set_priority(TIMER_IRQ_0, 0xC0);
                break;
            case 1:
                irq_set_priority(TIMER_IRQ_1, 0xC0);
                break;
            case 2:
                irq_set_priority(TIMER_IRQ_2, 0xC0);
                break;
            case 3:
                irq_set_priority(TIMER_IRQ_3, 0xC0);
                break;
            }

            _started = true;
        }
    };

    repeating_timer_t& sharedISRTimer(uint8_t idx) { this->begin(); return _timers[idx]; }
    alarm_pool_t* sharedAlarmPool() { this->begin(); return _pool; }

private:
    alarm_pool_t* _pool = nullptr;
    repeating_timer_t _timers[16] = {};
    volatile bool _started = false;
    volatile uint8_t _timer_num;
};

extern EVNISRTimer EVNISRTimer0, EVNISRTimer1;

#endif