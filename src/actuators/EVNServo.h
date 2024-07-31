#ifndef EVNServo_h
#define EVNServo_h

#include <Arduino.h>
#include <Servo.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"

#define DIRECT	1
#define REVERSE	0

typedef struct
{
    Servo* servo;
    volatile bool sweep;
    volatile float pulse;
    volatile bool servo_dir;
    volatile uint8_t pin;
    volatile uint8_t port;
    volatile uint16_t range;
    volatile uint16_t min_pulse_us;
    volatile uint16_t max_pulse_us;
    volatile float position;
    volatile float end_position;
    volatile float dps;
    volatile float max_dps;
    volatile uint64_t last_loop;
} servo_state_t;

class EVNServo
{
public:
    friend class EVNContinuousServo;
    static const uint16_t TIMER_INTERVAL_US = 10000;

    EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_position = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500);
    void begin();
    void write(float position, uint16_t wait_time_ms = 0, float dps = 0);
    void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms = 0);
    uint16_t getRange() { return _servo.range; };
    float getMaxDPS() { return _servo.max_dps; };

protected:
    servo_state_t _servo;
    static servo_state_t* servoArgs[4];
    static bool servos_enabled[4];
    static bool cservos_enabled[4];

    static void attach_servo_interrupt(servo_state_t* arg)
    {
        if (!cservos_enabled[arg->port - 1])
        {
            if (!servos_enabled[0] && !servos_enabled[1] && !servos_enabled[2] && !servos_enabled[3])
            {
                if (rp2040.cpuid == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(0));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(0));
            }
            servoArgs[arg->port - 1] = arg;
            servos_enabled[arg->port - 1] = true;
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        for (int i = 0; i < 4; i++)
        {
            if (servos_enabled[i])
                update(servoArgs[i]);
        }
        return true;
    }

    static void update(servo_state_t* arg)
    {
        uint64_t now = micros();
        float time_since_last_loop = ((float)now - (float)arg->last_loop) / 1000000;
        arg->last_loop = now;

        if (motors_enabled())
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
        {
            arg->sweep = false;
        }
    }

    static bool motors_enabled()
    {
        return ((EVNAlpha::sharedButtonLED().read() && EVNAlpha::sharedButtonLED().sharedState()->link_movement)
            || !EVNAlpha::sharedButtonLED().sharedState()->link_movement);
    }
};


class EVNContinuousServo
{
public:
    friend class EVNServo;
    static const uint16_t TIMER_INTERVAL_US = 10000;

    EVNContinuousServo(uint8_t port, bool servo_dir = DIRECT, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400);
    void begin();
    void write(float duty_cycle_pct);
    void writeMicroseconds(uint16_t pulse_us);

protected:
    servo_state_t _servo;
    static servo_state_t* cservoArgs[4];

    static void attach_servo_interrupt(servo_state_t* arg)
    {
        if (!EVNServo::servos_enabled[arg->port - 1])
        {
            if (!EVNServo::cservos_enabled[0] && !EVNServo::cservos_enabled[1] && !EVNServo::cservos_enabled[2] && !EVNServo::cservos_enabled[3])
            {
                if (rp2040.cpuid == 0)
                    alarm_pool_add_repeating_timer_us(EVNISRTimer0::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer0::sharedISRTimer(1));
                else
                    alarm_pool_add_repeating_timer_us(EVNISRTimer1::sharedAlarmPool(), TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer1::sharedISRTimer(1));
            }
            cservoArgs[arg->port - 1] = arg;
            EVNServo::cservos_enabled[arg->port - 1] = true;
        }
    }

    static bool timerisr(struct repeating_timer* t)
    {
        for (int i = 0; i < 4; i++)
        {
            if (EVNServo::cservos_enabled[i])
                update(cservoArgs[i]);
        }
        return true;
    }

    static void update(servo_state_t* arg)
    {
        if (motors_enabled())
            arg->servo->writeMicroseconds(arg->pulse);
        else
            arg->servo->writeMicroseconds((arg->max_pulse_us + arg->min_pulse_us) / 2);
    }

    static bool motors_enabled()
    {
        return ((EVNAlpha::sharedButtonLED().read() && EVNAlpha::sharedButtonLED().sharedState()->link_movement)
            || !EVNAlpha::sharedButtonLED().sharedState()->link_movement);
    }
};

#endif