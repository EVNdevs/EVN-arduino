#include "EVNServo.h"

volatile servo_state_t* EVNServo::servoArgs[];
volatile bool EVNServo::fservos_enabled[] = { false, false, false, false };
volatile bool EVNServo::cservos_enabled[] = { false, false, false, false };
volatile bool EVNServo::timerisr_enabled = false;
volatile bool EVNServo::timerisr_executed = false;
volatile uint8_t EVNServo::core;
mutex_t EVNServo::mutex;
spin_lock_t* EVNServo::spin_lock;

volatile bool EVNContinuousServo::timerisr_enabled = false;
volatile bool EVNContinuousServo::timerisr_executed = false;
volatile uint8_t EVNContinuousServo::core;

EVNServo::EVNServo(uint8_t port, bool servo_dir, uint16_t range, float start_position, uint16_t min_pulse_us, uint16_t max_pulse_us, float max_dps)
{
    _servo.servo_dir = servo_dir;
    _servo.range = range;
    _servo.min_pulse_us = min_pulse_us;
    _servo.max_pulse_us = max_pulse_us;
    _servo.max_dps = max(1, max_dps);
    _servo.position = constrain(start_position, 0, range);

    uint8_t portc = constrain(port, 1, 4);
    _servo.port = portc;

    switch (portc)
    {
    case 1:
        _servo.pin = PIN_SERVO1;
        break;
    case 2:
        _servo.pin = PIN_SERVO2;
        break;
    case 3:
        _servo.pin = PIN_SERVO3;
        break;
    case 4:
        _servo.pin = PIN_SERVO4;
        break;
    }

    _servo.servo = new Servo;
}

void EVNServo::ensure_isr_executed() volatile
{
    if (rp2040.cpuid() == core)
    {
        while (!timerisr_executed);
        timerisr_executed = false;
    }
    else
    {
        while (!is_spin_locked(spin_lock));
    }
}

void EVNServo::begin() volatile
{
    _servo.servo->attach(_servo.pin, 200, 2800);

    attach_servo_interrupt(&_servo);

    float pulse = (float)(_servo.position / _servo.range) * (float)(_servo.max_pulse_us - _servo.min_pulse_us);
    if (_servo.servo_dir == DIRECT)
        pulse = (float)_servo.min_pulse_us + pulse;
    else
        pulse = (float)_servo.max_pulse_us - pulse;

    this->writeMicroseconds(pulse);
}

void EVNServo::write(float position, uint16_t wait_time_ms, float dps) volatile
{
    if (!timerisr_enabled) return;
    ensure_isr_executed();
    mutex_enter_blocking(&mutex);

    if (dps == 0)
    {
        _servo.position = constrain(position, 0, _servo.range);
        float pulse = (float)(_servo.position / _servo.range) * (float)(_servo.max_pulse_us - _servo.min_pulse_us);
        if (_servo.servo_dir == DIRECT)
            pulse = (float)_servo.min_pulse_us + pulse;
        else
            pulse = (float)_servo.max_pulse_us - pulse;
        _servo.pulse = constrain(pulse, 200, 2800);
        _servo.sweep = false;
    }
    else
    {
        _servo.end_position = constrain(position, 0, _servo.range);
        _servo.dps = min(fabs(dps), _servo.max_dps);
        _servo.sweep = true;
    }
    mutex_exit(&mutex);

    uint32_t start_time = millis();
    while ((millis() - start_time) < wait_time_ms)
    {
        if (!EVNAlpha::motorsEnabled())
            break;
    }
}

void EVNServo::writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms) volatile
{
    if (!timerisr_enabled) return;
    ensure_isr_executed();
    mutex_enter_blocking(&mutex);
    _servo.pulse = constrain(pulse_us, 200, 2800);
    _servo.sweep = false;
    mutex_exit(&mutex);

    uint32_t start_time = millis();
    while ((millis() - start_time) < wait_time_ms)
    {
        if (!EVNAlpha::motorsEnabled())
            break;
    }
}

EVNContinuousServo::EVNContinuousServo(uint8_t port, bool servo_dir, uint16_t min_pulse_us, uint16_t max_pulse_us)
{
    _servo.servo_dir = servo_dir;
    _servo.min_pulse_us = min_pulse_us;
    _servo.max_pulse_us = max_pulse_us;

    uint8_t portc = constrain(port, 1, 4);
    _servo.port = portc;

    switch (portc)
    {
    case 1:
        _servo.pin = PIN_SERVO1;
        break;
    case 2:
        _servo.pin = PIN_SERVO2;
        break;
    case 3:
        _servo.pin = PIN_SERVO3;
        break;
    case 4:
        _servo.pin = PIN_SERVO4;
        break;
    }

    _servo.servo = new Servo;
}

void EVNContinuousServo::begin() volatile
{
    _servo.servo->attach(_servo.pin, 200, 2800);
    attach_servo_interrupt(&_servo);
    float pulse = (float)(_servo.max_pulse_us - _servo.min_pulse_us) / 2 + _servo.min_pulse_us;
    this->writeMicroseconds(pulse);
}

void EVNContinuousServo::ensure_isr_executed() volatile
{
    if (rp2040.cpuid() == core)
    {
        while (!timerisr_executed);
        timerisr_executed = false;
    }
    else
    {
        while (!is_spin_locked(EVNServo::spin_lock));
    }
}

void EVNContinuousServo::write(float duty_cycle_pct) volatile
{
    float duty_cyclec = constrain(duty_cycle_pct, -100, 100) * 0.01;
    uint16_t pulse = (uint16_t)((1 - fabs(duty_cyclec)) * (float)(_servo.max_pulse_us - _servo.min_pulse_us) / 2);

    if ((_servo.servo_dir == DIRECT) == (duty_cyclec > 0))
        pulse = _servo.min_pulse_us + pulse;
    else
        pulse = _servo.max_pulse_us - pulse;

    this->writeMicroseconds(pulse);
}

void EVNContinuousServo::writeMicroseconds(uint16_t pulse_us) volatile
{
    if (!timerisr_enabled) return;
    ensure_isr_executed();
    mutex_enter_blocking(&EVNServo::mutex);

    _servo.pulse = constrain(pulse_us, 200, 2800);

    mutex_exit(&EVNServo::mutex);
}