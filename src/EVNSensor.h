#ifndef EVNSensor_h
#define EVNSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "EVNAlpha.h"

class EVNSensor {

public:
    EVNSensor(uint8_t port)
    {
        _port = constrain(port, 1, 16);
        if (_port <= 8)
            _wire = &Wire;
        else
            _wire = &Wire1;

        _sensor_started = false;
    };

    void write8(uint8_t reg, uint8_t value)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->write(value);
        _wire->endTransmission();
    };

    uint8_t read8(uint8_t reg, bool stop_message = true)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(stop_message);
        _wire->requestFrom(_addr, (uint8_t)1);
        uint8_t out = _wire->read();

        return out;
    };

    uint16_t read16(uint8_t reg, bool lsb_start = true, bool stop_message = true)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(stop_message);
        _wire->requestFrom(_addr, (uint8_t)2);

        uint16_t high, low;

        if (lsb_start)
        {
            low = _wire->read();
            high = _wire->read();
        }
        else
        {
            high = _wire->read();
            low = _wire->read();
        }
        high <<= 8;
        high |= low;

        return high;
    };

    void readBuffer(uint8_t reg, uint8_t size, uint8_t* buffer, bool stop_message = true)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(stop_message);
        _wire->requestFrom(_addr, size);

        for (int i = 0; i < size; i++)
        {
            buffer[i] = _wire->read();
        }
    };

protected:
    bool _sensor_started = false;
    uint8_t _port;
    uint8_t _addr;
    TwoWire* _wire;
};

#endif