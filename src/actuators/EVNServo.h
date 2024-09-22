#ifndef EVNServo_h
#define EVNServo_h

#include <Arduino.h>
#include <Servo.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"
#include "../evn_pins_defs.h"

#define DIRECT	1
#define REVERSE	0

// TODO: Add end function for classes
// TODO: Wrap mutex & spinlock in helper class
// TODO: Have EVNServo and EVNContinuousServo inherit from same parent class. They share some functions

typedef struct
{
    Servo* servo;
    bool sweep;
    float pulse;
    bool servo_dir;
    uint8_t pin;
    uint8_t port;
    uint16_t range;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    float position;
    float end_position;
    float dps;
    float max_dps;
    uint32_t last_loop;
} servo_state_t;

class EVNServo
{
public:
    friend class EVNContinuousServo;
    static const uint16_t TIMER_INTERVAL_US = 10000;
    static const uint8_t MAX_SERVO_OBJECTS = 4;

    EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_position = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500);
    void begin() volatile;
    void write(float position, uint16_t wait_time_ms = 0, float dps = 0) volatile;
    void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms = 0) volatile;
    uint16_t getRange() volatile { return _servo.range; };
    float getMaxDPS() volatile { return _servo.max_dps; };

protected:
    volatile servo_state_t _servo = {};
    static volatile servo_state_t* servoArgs[MAX_SERVO_OBJECTS];
    static volatile bool fservos_enabled[MAX_SERVO_OBJECTS];
    static volatile bool cservos_enabled[MAX_SERVO_OBJECTS];
    static volatile bool timerisr_enabled;
    static volatile bool timerisr_executed;
    static volatile uint8_t core;
    static mutex_t mutex;
    static spin_lock_t* spin_lock;

    void ensure_isr_executed() volatile;

    static void attach_servo_interrupt(volatile servo_state_t* arg)
    {
        if (!cservos_enabled[arg->port - 1])
        {
            servoArgs[arg->port - 1] = arg;
            fservos_enabled[arg->port - 1] = true;

            if (!timerisr_enabled)
            {
                mutex_init(&mutex);
                spin_lock = spin_lock_init(spin_lock_claim_unused(true));
                core = rp2040.cpuid();
                if (core == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(0));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(0));

                timerisr_enabled = true;
            }
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        if (mutex_try_enter_block_until(&mutex, delayed_by_us(get_absolute_time(), 1000)))
        {
            spin_lock_unsafe_blocking(spin_lock);

            for (int i = 0; i < 4; i++)
            {
                if (fservos_enabled[i])
                    update(servoArgs[i]);
            }

            timerisr_executed = true;
            spin_unlock_unsafe(spin_lock);
            mutex_exit(&mutex);
        }
        return true;
    }

    static void update(volatile servo_state_t* arg)
    {
        uint32_t now = micros();
        float time_since_last_loop = ((float)(now - arg->last_loop)) / 1000000;
        arg->last_loop = now;

        if (time_since_last_loop < 0)
            return;

        if (EVNAlpha::motorsEnabled())
        {
            if (arg->sweep)
            {
                if (arg->position == arg->end_position)
                    arg->sweep = false;

                else
                {
                    float deg_per_loop = arg->dps * time_since_last_loop;
                    if (arg->position > arg->end_position)
                    {
                        arg->position -= deg_per_loop;
                        if (arg->position < arg->end_position) arg->position = arg->end_position;
                    }

                    if (arg->position < arg->end_position)
                    {
                        arg->position += deg_per_loop;
                        if (arg->position > arg->end_position) arg->position = arg->end_position;
                    }

                    float pulse = (float)(arg->position / arg->range) * (float)(arg->max_pulse_us - arg->min_pulse_us);
                    if (arg->servo_dir == DIRECT)
                        pulse = (float)arg->min_pulse_us + pulse;
                    else
                        pulse = (float)arg->max_pulse_us - pulse;

                    arg->servo->writeMicroseconds(pulse);
                }
            }
            else
            {
                arg->servo->writeMicroseconds(arg->pulse);
            }
        }
        else
            arg->sweep = false;
    }
};


class EVNContinuousServo
{
public:
    static const uint16_t TIMER_INTERVAL_US = 10000;

    EVNContinuousServo(uint8_t port, bool servo_dir = DIRECT, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400);
    void begin() volatile;
    void write(float duty_cycle_pct) volatile;
    void writeMicroseconds(uint16_t pulse_us) volatile;

protected:
    volatile servo_state_t _servo = {};
    static volatile bool timerisr_enabled;
    static volatile bool timerisr_executed;
    static volatile uint8_t core;

    void ensure_isr_executed() volatile;

    static void attach_servo_interrupt(volatile servo_state_t* arg)
    {
        if (!EVNServo::fservos_enabled[arg->port - 1])
        {
            EVNServo::servoArgs[arg->port - 1] = arg;
            EVNServo::cservos_enabled[arg->port - 1] = true;

            if (!timerisr_enabled)
            {
                if (EVNServo::timerisr_enabled)
                {
                    cancel_repeating_timer(&EVNISRTimer0::sharedISRTimer(0));
                    cancel_repeating_timer(&EVNISRTimer1::sharedISRTimer(0));
                }
                else
                {
                    mutex_init(&EVNServo::mutex);
                    EVNServo::spin_lock = spin_lock_init(spin_lock_claim_unused(true));
                }

                core = rp2040.cpuid();
                if (core == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(1));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(1));

                EVNServo::timerisr_enabled = true;
                timerisr_enabled = true;
            }
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        if (mutex_try_enter_block_until(&EVNServo::mutex, delayed_by_us(get_absolute_time(), 1000)))
        {
            spin_lock_unsafe_blocking(EVNServo::spin_lock);
            for (int i = 0; i < 4; i++)
            {
                if (EVNServo::fservos_enabled[i])
                    EVNServo::update(EVNServo::servoArgs[i]);

                if (EVNServo::cservos_enabled[i])
                    update(EVNServo::servoArgs[i]);
            }
            timerisr_executed = true;
            spin_unlock_unsafe(EVNServo::spin_lock);
            mutex_exit(&EVNServo::mutex);
        }
        return true;
    }

    static void update(volatile servo_state_t* arg)
    {
        if (EVNAlpha::motorsEnabled())
        {
            arg->servo->writeMicroseconds(arg->pulse);
        }
        else
        {
            arg->servo->writeMicroseconds((arg->max_pulse_us + arg->min_pulse_us) / 2);
        }
    }
};

#endif