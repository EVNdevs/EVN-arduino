#ifndef EVNColourSensor_h
#define EVNColourSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

#define COLOUR_GAIN_X1  (EVNColourSensor::gain::X1)
#define COLOUR_GAIN_X4  (EVNColourSensor::gain::X4)
#define COLOUR_GAIN_X16 (EVNColourSensor::gain::X16)
#define COLOUR_GAIN_X60 (EVNColourSensor::gain::X60)

struct evn_col
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;

    float red_pct;
    float green_pct;
    float blue_pct;
    float clear_pct;

    float red_norm;
    float green_norm;
    float blue_norm;
    float clear_norm;

    float hue;
    float saturation;
    float value;
};

class EVNColourSensor : private EVNI2CDevice {
public:
    static const uint8_t I2C_ADDR = 0x29;
    static const uint8_t ID_REG_PART_NUMBER = 0x44;
    static const uint8_t ALT_ID_REG_PART_NUMBER = 0x4D;
    static const uint8_t TCS34725_COMMAND_BIT = 0x80;

    enum class reg : uint8_t
    {
        ENABLE = 0x00,
        ATIME = 0x01,
        WTIME = 0x03,
        AILTL = 0x04,
        AILTH = 0x05,
        AIHTL = 0x06,
        AIHTH = 0x07,
        PERS = 0x0C,
        CONFIG = 0x0D,
        CONTROL = 0x0F,
        ID = 0x12,
        STATUS = 0x13,

        CDATAL = 0x14,
        CDATAH = 0x15,
        RDATAL = 0x16,
        RDATAH = 0x17,
        GDATAL = 0x18,
        GDATAH = 0x19,
        BDATAL = 0x1A,
        BDATAH = 0x1B
    };

    enum class mask : uint8_t
    {
        ENABLE_AIEN = 0x10,
        ENABLE_WEN = 0x08,
        ENABLE_AEN = 0x02,
        ENABLE_PON = 0x01,
        STATUS_AINT = 0x10,
        STATUS_AVALID = 0x01
    };

    enum class gain : uint8_t
    {
        X1 = 0x00,
        X4 = 0x01,
        X16 = 0x02,
        X64 = 0x03
    };

    EVNColourSensor(uint8_t port, uint8_t integration_cycles = 1, gain gain = gain::X16) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
        _gain = gain;
        _int_cycles = constrain(integration_cycles, 1, 255);
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        uint8_t id = read8(TCS34725_COMMAND_BIT | (uint8_t)reg::ID);

        if (id != ID_REG_PART_NUMBER && id != ALT_ID_REG_PART_NUMBER)
        {
            return _sensor_started;
        };

        _sensor_started = true;

        write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ENABLE, (uint8_t)mask::ENABLE_PON);
        delay(3);
        write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ENABLE, (uint8_t)mask::ENABLE_PON | (uint8_t)mask::ENABLE_AEN);

        this->setIntegrationCycles(_int_cycles);
        this->setGain(_gain);

        return _sensor_started;
    };

    void setGain(gain gain)
    {
        if (_sensor_started)
        {
            _gain = gain;
            write8(TCS34725_COMMAND_BIT | (uint8_t)reg::CONTROL, (uint8_t)_gain);
        }
    };

    void setIntegrationCycles(uint8_t integration_cycles)
    {
        if (_sensor_started)
        {
            _int_cycles = constrain(integration_cycles, 1, 255);
            _max_count = _int_cycles * 1024;
            write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ATIME, (uint8_t)(255 - (_int_cycles - 1)));
            _int_time_us = _int_cycles * 2400;
        }
    };

    void setRedRange(uint16_t low, uint16_t high)
    {
        _r_norm = true;
        _r_low = low;
        _r_high = max(low + 1, high);
    };

    void setGreenRange(uint16_t low, uint16_t high)
    {
        _g_norm = true;
        _g_low = low;
        _g_high = max(low + 1, high);
    };

    void setBlueRange(uint16_t low, uint16_t high)
    {
        _b_norm = true;
        _b_low = low;
        _b_high = max(low + 1, high);
    };

    void setClearRange(uint16_t low, uint16_t high)
    {
        _c_norm = true;
        _c_low = low;
        _c_high = max(low + 1, high);
    };

    evn_col readAll(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();

        evn_col reading;

        reading.red = _r;
        reading.green = _g;
        reading.blue = _b;
        reading.clear = _c;

        reading.red_pct = _r_pct;
        reading.green_pct = _g_pct;
        reading.blue_pct = _b_pct;
        reading.clear_pct = _c_pct;

        reading.red_norm = readRedNorm(false);
        reading.green_norm = readGreenNorm(false);
        reading.blue_norm = readBlueNorm(false);
        reading.clear_norm = readClearNorm(false);

        reading.hue = readHueHSV(false);
        reading.saturation = readSaturationHSV(false);
        reading.value = readValueHSV(false);

        return reading;
    };

    uint16_t read(bool blocking = true)
    {
        return this->readClear(blocking);
    };

    uint16_t readRed(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _r;
        }
        return 0;
    };

    uint16_t readGreen(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _g;
        }
        return 0;
    };
    uint16_t readBlue(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _b;
        }
        return 0;
    };

    uint16_t readClear(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _c;
        }
        return 0;
    };

    float readRedNorm(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (_r_norm)
                return normalise(this->readRed(blocking), _r_low, _r_high);
        }
        return 0;
    };

    float readGreenNorm(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (_g_norm)
                return normalise(this->readGreen(blocking), _g_low, _g_high);
        }
        return 0;
    };

    float readBlueNorm(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (_b_norm)
                return normalise(this->readBlue(blocking), _b_low, _b_high);
        }
        return 0;
    };

    float readClearNorm(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (_c_norm)
                return normalise(this->readClear(blocking), _c_low, _c_high);
        }
        return 0;
    };

    float readClearPCT(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();
        return _c_pct;
    };

    float readRedPCT(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();
        return _r_pct;
    };

    float readGreenPCT(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();
        return _g_pct;
    };

    float readBluePCT(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();
        return _b_pct;
    };

    float readHueHSV(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();

        float cmax = max(_r_pct, max(_g_pct, _b_pct));
        float cmin = min(_r_pct, min(_g_pct, _b_pct));
        float cdiff = cmax - cmin;

        float hue = 0;
        if (cdiff == 0)
            return 0;

        else if (cmax == _r_pct)
            hue = 60 * fmod((_g_pct - _b_pct) / cdiff, 6);

        else if (cmax == _g_pct)
            hue = 60 * ((_b_pct - _r_pct) / cdiff + 2);

        else if (cmax == _b_pct)
            hue = 60 * ((_r_pct - _g_pct) / cdiff + 4);

        return hue;
    };

    float readSaturationHSV(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();

        float cmax = max(_r_pct, max(_g_pct, _b_pct));
        float cmin = min(_r_pct, min(_g_pct, _b_pct));
        float cdiff = cmax - cmin;

        if (cmax == 0)
            return 0;
        else
            return cdiff / cmax;
    };

    float readValueHSV(bool blocking = true)
    {
        this->update(blocking);
        convertToPCT();

        return max(_r_pct, max(_g_pct, _b_pct));
    };

private:

    float normalise(uint16_t reading, uint16_t low, uint16_t high)
    {
        return constrain(((float)(reading - low)) / ((float)(high - low)), 0, 1);
    };

    void convertToPCT()
    {
        if (!_converted_to_pct)
        {
            _c_pct = (float)_c / (float)_max_count;
            _r_pct = (float)_r / (float)_max_count;
            _g_pct = (float)_g / (float)_max_count;
            _b_pct = (float)_b / (float)_max_count;
            _converted_to_pct = true;
        }
    };

    void update(bool blocking = false)
    {
        if (_sensor_started)
        {
            if (blocking)
                while ((micros() - _last_reading_us) < _int_time_us);

            if ((micros() - _last_reading_us) >= _int_time_us);
            {
                _last_reading_us = micros();

                readBuffer(TCS34725_COMMAND_BIT | (uint8_t)reg::CDATAL, 8, _buffer);

                _c = _buffer[0] | (_buffer[1] << 8);
                _r = _buffer[2] | (_buffer[3] << 8);
                _g = _buffer[4] | (_buffer[5] << 8);
                _b = _buffer[6] | (_buffer[7] << 8);

                _converted_to_pct = false;
            }
        }
    };

    uint8_t _buffer[8] = { 0 };

    uint32_t _last_reading_us;
    uint8_t _int_cycles;
    uint32_t _int_time_us;
    uint16_t _max_count;
    gain _gain;

    uint16_t _r = 0, _g = 0, _b = 0, _c = 0;
    bool _converted_to_pct = false;
    float _r_pct = 0, _g_pct = 0, _b_pct = 0, _c_pct = 0;
    bool _r_norm = false, _g_norm = false, _b_norm = false, _c_norm = false;
    uint16_t _r_low, _g_low, _b_low, _c_low;
    uint16_t _r_high, _g_high, _b_high, _c_high;
};

#endif