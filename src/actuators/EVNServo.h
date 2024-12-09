#ifndef EVNServo_h
#define EVNServo_h

#include <Arduino.h>
#include <Servo.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"
#include "../helper/EVNCoreSync.h"
#include "../evn_pins_defs.h"

#define DIRECT	1
#define REVERSE	0

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

class EVNServoBase
{
public:
    static const uint16_t TIMER_INTERVAL_US = 10000;
    static const uint8_t MAX_SERVO_OBJECTS = 4;

    EVNServoBase(uint8_t port, bool servo_dir = DIRECT, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400);

protected:
    void begin() volatile;
    void end() volatile;

    volatile servo_state_t _servo = {};
    static volatile servo_state_t* servoArgs[MAX_SERVO_OBJECTS];
    static volatile bool fservos_enabled[MAX_SERVO_OBJECTS];
    static volatile bool cservos_enabled[MAX_SERVO_OBJECTS];
};

class EVNServo : public EVNServoBase
{
public:
    friend class EVNContinuousServo;

    EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_position = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500);
    void begin() volatile;
    void write(float position, uint16_t wait_time_ms = 0, float dps = 0) volatile;
    void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms = 0) volatile;
    uint16_t getRange() volatile { return _servo.range; };
    float getMaxDPS() volatile { return _servo.max_dps; };
    void setMode(bool enable) volatile;

protected:
    static volatile bool timerisr_enabled;

    static void attach_interrupts(volatile servo_state_t* arg)
    {
        if (!cservos_enabled[arg->port - 1])
        {
            servoArgs[arg->port - 1] = arg;
            fservos_enabled[arg->port - 1] = true;

            if (!timerisr_enabled)
            {
                EVNCoreSync1.begin();

                if (rp2040.cpuid() == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0.sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer0.sharedISRTimer(0));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1.sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer1.sharedISRTimer(0));

                timerisr_enabled = true;
            }
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        if (EVNCoreSync1.core1_timer_isr_enter())
        {
            for (int i = 0; i < 4; i++)
            {
                if (fservos_enabled[i])
                    update(servoArgs[i]);
            }

            EVNCoreSync1.core1_timer_isr_exit();
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
                        if (arg->position < arg->end_position)
                            arg->position = arg->end_position;
                    }

                    if (arg->position < arg->end_position)
                    {
                        arg->position += deg_per_loop;
                        if (arg->position > arg->end_position)
                            arg->position = arg->end_position;
                    }

                    arg->pulse = (float)(arg->position / arg->range) * (float)(arg->max_pulse_us - arg->min_pulse_us);
                    if (arg->servo_dir == DIRECT)
                        arg->pulse = (float)arg->min_pulse_us + arg->pulse;
                    else
                        arg->pulse = (float)arg->max_pulse_us - arg->pulse;

                    arg->servo->writeMicroseconds(arg->pulse);
                }
            }
            else
                arg->servo->writeMicroseconds(arg->pulse);
        }
        else
            arg->sweep = false;
    }
};


class EVNContinuousServo : public EVNServoBase
{
    friend class EVNServo;
    using EVNServoBase::EVNServoBase;

public:
    void begin() volatile;
    void write(float duty_cycle_pct) volatile;
    void writeMicroseconds(uint16_t pulse_us) volatile;
    void setMode(bool enable) volatile;

protected:
    static volatile bool timerisr_enabled;

    static void attach_interrupts(volatile servo_state_t* arg)
    {
        if (!fservos_enabled[arg->port - 1])
        {
            servoArgs[arg->port - 1] = arg;
            cservos_enabled[arg->port - 1] = true;

            if (!timerisr_enabled)
            {
                if (EVNServo::timerisr_enabled)
                {
                    cancel_repeating_timer(&EVNISRTimer0.sharedISRTimer(0));
                    cancel_repeating_timer(&EVNISRTimer1.sharedISRTimer(0));
                }

                EVNCoreSync1.begin();

                if (rp2040.cpuid() == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0.sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer0.sharedISRTimer(1));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1.sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer1.sharedISRTimer(1));

                EVNServo::timerisr_enabled = true;
                timerisr_enabled = true;
            }
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        if (EVNCoreSync1.core1_timer_isr_enter())
        {
            for (int i = 0; i < 4; i++)
            {
                if (fservos_enabled[i])
                    EVNServo::update(servoArgs[i]);

                else if (cservos_enabled[i])
                    update(servoArgs[i]);
            }

            EVNCoreSync1.core1_timer_isr_exit();
        }
        return true;
    }

    static void update(volatile servo_state_t* arg)
    {
        if (EVNAlpha::motorsEnabled())
            arg->servo->writeMicroseconds(arg->pulse);
        else
            arg->servo->writeMicroseconds((arg->max_pulse_us + arg->min_pulse_us) / 2);
    }
};

#endif