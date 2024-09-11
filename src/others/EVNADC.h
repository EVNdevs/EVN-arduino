#ifndef EVNADC_h
#define EVNADC_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

class EVNADC : private EVNI2CDevice {
public:

    enum reg : uint8_t
    {
        CONVERSION = 0x00,
        CONFIG = 0x01,
    };

    enum range : uint8_t
    {
        MV_6144 = 0,
        MV_4096 = 1,
        MV_2048 = 2,
        MV_1024 = 3,
        MV_512 = 4,
        MV_256 = 5,
    };

    enum mode : uint8_t
    {
        CONTINUOUS = 0,
        SINGLE_SHOT = 1
    };

    //this enum is just for reference
    // enum mux : uint8_t
    // {
    //     AIN_0_AIN_1 = 0,
    //     AIN_0_AIN_3 = 1,
    //     AIN_1_AIN_3 = 2,
    //     AIN_2_AIN_3 = 3,
    //     AIN_0 = 4,
    //     AIN_1 = 5,
    //     AIN_2 = 6,
    //     AIN_3 = 7,
    // };

    enum data_rate : uint8_t
    {
        SPS_8 = 0,
        SPS_16 = 1,
        SPS_32 = 2,
        SPS_64 = 3,
        SPS_128 = 4,
        SPS_250 = 5,
        SPS_475 = 6,
        SPS_860 = 7
    };

    static const uint8_t I2C_ADDR = 0x48;
    static const uint16_t ID = 0x8583;

    EVNADC(uint8_t port) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        EVNAlpha::sharedPorts().setPort(_port);
        _wire->beginTransmission(0x00);
        _wire->write(0x06);
        _wire->endTransmission();

        delay(3);

        uint16_t id = read16((uint8_t)reg::CONFIG, false);

        if (id != ID)
        {
            return _sensor_started;
        }

        _sensor_started = true;
        setRange(range::MV_4096);
        setDataRate(data_rate::SPS_860);
        setMode(mode::SINGLE_SHOT);
        setPin(0);

        return _sensor_started;
    };

    void setRange(range range)
    {
        switch (range)
        {
        case MV_4096:
            _uv_per_lsb = 125;
            break;
        case MV_2048:
            _uv_per_lsb = 62.5;
            break;
        case MV_1024:
            _uv_per_lsb = 31.25;
            break;
        case MV_512:
            _uv_per_lsb = 15.625;
            break;
        case MV_256:
            _uv_per_lsb = 7.8125;
            break;
        }

        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        value &= 0b1111000111111111;
        value |= ((uint8_t)range << 9);
        uint8_t value1 = value >> 8;
        uint8_t value2 = value & 0xFF;
        write16((uint8_t)reg::CONFIG, value1, value2);
    };

    void setDataRate(data_rate data_rate)
    {
        switch (data_rate)
        {
        case data_rate::SPS_8:
            _conversion_time_us = 125000;
            break;

        case data_rate::SPS_16:
            _conversion_time_us = 62500;
            break;

        case data_rate::SPS_32:
            _conversion_time_us = 31250;
            break;

        case data_rate::SPS_64:
            _conversion_time_us = 15625;
            break;

        case data_rate::SPS_128:
            _conversion_time_us = 7813;
            break;

        case data_rate::SPS_250:
            _conversion_time_us = 4000;
            break;

        case data_rate::SPS_475:
            _conversion_time_us = 2106;
            break;

        case data_rate::SPS_860:
            _conversion_time_us = 1163;
            break;
        }

        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        value &= 0b1111111100011111;
        value |= ((uint8_t)data_rate << 5);
        uint8_t value1 = value >> 8;
        uint8_t value2 = value & 0xFF;
        write16((uint8_t)reg::CONFIG, value1, value2);
    };

    bool ready()
    {
        return micros() - _last_request_us > _conversion_time_us + _power_up_time_us
            && notPerformingConversion();
    };

    bool request(uint8_t pin)
    {
        if (_sensor_started)
        {
            if (!ready()) return false;

            if (_pin != pin) setPin(pin);
            if (_continuous_enabled) setMode(mode::SINGLE_SHOT);
            startConversion();
            _last_request_us = micros();
            return true;
        }
        return false;
    };

    float receive(bool blocking = true)
    {
        if (_sensor_started && !_continuous_enabled)
        {
            if (blocking)
                while (!ready());

            if (ready())
                update();
            else
                return 0;

            return _conversion;
        }
        return 0;
    };

    float read(uint8_t pin)
    {
        if (_sensor_started && !_continuous_enabled)
        {
            request(pin);
            return receive(true);
        }
        return 0;
    };

    void startContinuous(uint8_t pin)
    {
        if (_sensor_started)
        {
            if (_pin != pin) setPin(pin);
            if (!_continuous_enabled) setMode(mode::CONTINUOUS);
            _last_request_us = micros();
        }
    };

    bool readyContinuous()
    {
        return micros() - _last_request_us > _conversion_time_us + _power_up_time_us;
    }

    float readContinuous(bool blocking = true)
    {
        if (_sensor_started && _continuous_enabled)
        {
            if (blocking)
                while (!readyContinuous());

            if (readyContinuous())
            {
                update();
                _last_request_us = micros();
            }
            else
                return 0;

            return _conversion;
        }
        return 0;
    };

private:
    void setMode(mode mode)
    {
        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        value &= 0b1111111011111111;
        value |= ((uint8_t)mode << 8);
        uint8_t value1 = value >> 8;
        uint8_t value2 = value & 0xFF;
        write16((uint8_t)reg::CONFIG, value1, value2);

        if (mode == mode::SINGLE_SHOT)
        {
            _power_up_time_us = 25;
            _continuous_enabled = false;
        }
        else
        {
            _power_up_time_us = 0;
            _continuous_enabled = true;
        }
    };

    void setPin(uint8_t pin)
    {
        uint8_t pinc = constrain(pin, 0, 7);
        _pin = pinc;
        pinc += 4;
        pinc %= 8;
        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        value &= 0b1000111111111111;
        value |= (pinc << 12);
        uint8_t value1 = value >> 8;
        uint8_t value2 = value & 0xFF;
        write16((uint8_t)reg::CONFIG, value1, value2);
    };

    void startConversion()
    {
        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        value |= (1 << 15);
        uint8_t value1 = value >> 8;
        uint8_t value2 = value & 0xFF;
        write16((uint8_t)reg::CONFIG, value1, value2);
    };

    bool notPerformingConversion()
    {
        uint16_t value = read16((uint8_t)reg::CONFIG, false);
        return (value >> 15);
    };

    void update()
    {
        _conversion = (float)read16((uint8_t)reg::CONVERSION, false) * _uv_per_lsb;
    };

    bool _continuous_enabled = false;
    uint32_t _conversion_time_us;
    uint32_t _power_up_time_us;
    float _uv_per_lsb;
    uint32_t _last_request_us;
    uint8_t _pin = 0;
    float _conversion = 0;
};

#endif