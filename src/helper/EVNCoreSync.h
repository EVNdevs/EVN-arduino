#ifndef EVNCoreSync_h
#define EVNCoreSync_h
#include <Arduino.h>
#include "pico/stdlib.h"

#define CORE1_ISR_WAIT_TIME_US 1000

class EVNCoreSync
{
public:
    EVNCoreSync(uint16_t wait_time_us)
    {
        _wait_time_us = wait_time_us;
    };

    void begin()
    {
        if (!_started)
        {
            mutex_init(&_mutex);
            _lock = spin_lock_init(spin_lock_claim_unused(true));
            _core = rp2040.cpuid();
            _started = true;
        }
    };

    void core0_ensure_core1_isr_executed()
    {
        if (!_started) return;

        if (rp2040.cpuid() == _core)
        {
            // boolean is only used when we are checking if timer ISR has executed
            // on the same core the timer ISR runs on
            // doing this for the other core is not thread-safe, because writing booleans is NOT atomic
            while (!_timer_isr_executed);
            _timer_isr_executed = false;
        }
        else
            // for the non-timer ISR core, a spin lock is used as an atomic variable
            // if pin ISRs are using mutex, 2nd spin lock is used
            while (!is_spin_locked(_lock));

    };

    void core0_enter()
    {
        if (!_started) return;

        core0_ensure_core1_isr_executed();
        mutex_enter_blocking(&_mutex);
    };

    void core0_exit()
    {
        //WARNING: assumes that core0_enter has been called

        if (!_started) return;

        mutex_exit(&_mutex);
    };

    bool core1_timer_isr_enter()
    {
        if (!_started) return false;

        _timer_isr_acquired = mutex_try_enter_block_until(&_mutex, delayed_by_us(get_absolute_time(), _wait_time_us));

        if (_timer_isr_acquired)
            spin_lock_unsafe_blocking(_lock);

        return _timer_isr_acquired;
    };

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
    };

    bool core1_pin_isr_enter()
    {
        //WARNING: assumes that timer ISR is lower priority, and no higher priority ISRs on same core are accessing the shared memory
        //WARNING: assumes that value written to owner is the core holding the mutex (SDK default behaviour, but can be overwritten)

        if (!_started) return false;

        uint32_t owner;
        _pin_isr_acquired = mutex_try_enter(&_mutex, &owner);
        if (!_pin_isr_acquired && owner != _core)
            _pin_isr_acquired = mutex_try_enter_block_until(&_mutex, delayed_by_us(get_absolute_time(), _wait_time_us));

        return (_pin_isr_acquired || owner == _core);
    };

    void core1_pin_isr_exit()
    {
        //WARNING: assumes that core1_pin_isr_enter has been called
        if (!_started) return;

        if (_pin_isr_acquired)
        {
            _pin_isr_acquired = false;
            mutex_exit(&_mutex);
        }
    };

private:
    mutex_t _mutex = {};
    spin_lock_t* _lock = nullptr;
    volatile uint8_t _core;
    volatile bool _started = false;

    volatile uint16_t _wait_time_us;

    volatile bool _pin_isr_acquired = false;
    volatile bool _timer_isr_acquired = false;
    volatile bool _timer_isr_executed = false;
};

extern EVNCoreSync EVNCoreSync0, EVNCoreSync1;

#endif