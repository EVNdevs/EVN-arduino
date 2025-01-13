#ifndef EVNCoreSync_h
#define EVNCoreSync_h
#include <Arduino.h>
#include "pico/stdlib.h"

#define CORE1_ISR_SPIN_TIMEOUT_US 500
#define CORE1_ISR_STALL_TIME_US 50

class EVNCoreSync
{
public:
    EVNCoreSync(uint16_t spin_timeout_us, uint16_t stall_time_us)
    {
        _spin_timeout_us = spin_timeout_us;
        _stall_time_us = stall_time_us;
    }

    void begin()
    {
        if (!_started)
        {
            mutex_init(&_mutex);
            _lock = spin_lock_init(spin_lock_claim_unused(true));
            _core = rp2040.cpuid();
            _started = true;
        }
    }

    void core0_ensure_core1_isr_can_execute()
    {
        if (!_started) return;

        // boolean is only used when we are checking if timer ISR has executed
        // on the same core the timer ISR runs on
        // doing this for the other core is not thread-safe, because writing booleans is NOT atomic
        if (rp2040.cpuid() == _core)
        {
            if (!_timer_isr_executed)
                stall();
            else
                _timer_isr_executed = false;
        }
        // for the non-timer ISR core, a spin lock is used as an atomic variable
        // if pin ISRs are using mutex, 2nd spin lock is used
        else if (!is_spin_locked(_lock))
            stall();
    }

    void core0_enter()
    {
        if (!_started) return;

        core0_ensure_core1_isr_can_execute();
        mutex_enter_blocking(&_mutex);
    }

    void core0_exit()
    {
        //WARNING: assumes that core0_enter has been called

        if (!_started) return;

        mutex_exit(&_mutex);
    }

    bool core1_timer_isr_enter()
    {
        if (!_started) return false;

        _timer_isr_acquired = mutex_try_enter_block_until(&_mutex, delayed_by_us(get_absolute_time(), _spin_timeout_us));

        if (_timer_isr_acquired)
            spin_lock_unsafe_blocking(_lock);

        return _timer_isr_acquired;
    }

    void core1_timer_isr_exit()
    {
        //WARNING: assumes that core1_timer_isr_enter() is called and returns true

        if (!_started) return;

        if (_timer_isr_acquired)
        {
            _timer_isr_acquired = false;
            _timer_isr_executed = true;
            spin_unlock_unsafe(_lock);
            mutex_exit(&_mutex);
        }
    }

    bool core1_pin_isr_enter()
    {
        //WARNING: assumes that timer ISR is lower priority, and no higher priority ISRs on same core are accessing the shared memory
        //WARNING: assumes that value written to owner is the core holding the mutex (SDK default behaviour, but can be overwritten)

        if (!_started) return false;

        uint32_t owner;
        _pin_isr_acquired = mutex_try_enter(&_mutex, &owner);
        if (!_pin_isr_acquired && owner != _core)
            _pin_isr_acquired = mutex_try_enter_block_until(&_mutex, delayed_by_us(get_absolute_time(), _spin_timeout_us));

        return (_pin_isr_acquired || owner == _core);
    }

    void core1_pin_isr_exit()
    {
        //WARNING: assumes that core1_pin_isr_enter has been called
        if (!_started) return;

        if (_pin_isr_acquired)
        {
            _pin_isr_acquired = false;
            mutex_exit(&_mutex);
        }
    }

private:

    void stall()
    {
        uint32_t start = micros();
        while (micros() - start < _stall_time_us);
    }

    mutex_t _mutex = {};
    spin_lock_t* _lock = nullptr;
    volatile uint8_t _core;
    volatile bool _started = false;

    volatile uint16_t _spin_timeout_us;
    volatile uint16_t _stall_time_us;

    volatile bool _pin_isr_acquired = false;
    volatile bool _timer_isr_acquired = false;
    volatile bool _timer_isr_executed = false;
};

extern EVNCoreSync EVNCoreSync0, EVNCoreSync1;

#endif