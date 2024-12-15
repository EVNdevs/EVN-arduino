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

#define CLEAR   0
#define RED     1
#define GREEN   2
#define BLUE    3

#define HUE     0
#define SAT     1
#define VAL     2

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

    EVNColourSensor(uint8_t port, uint8_t integration_cycles = 1, gain gain = COLOUR_GAIN_X16) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
        _gain = gain;
        _int_cycles = constrain(integration_cycles, 1, 255);
    }

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        uint8_t id = read8(TCS34725_COMMAND_BIT | (uint8_t)reg::ID);

        if (id != ID_REG_PART_NUMBER && id != ALT_ID_REG_PART_NUMBER)
            return _sensor_started;

        _sensor_started = true;

        write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ENABLE, (uint8_t)mask::ENABLE_PON);
        delay(3);
        write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ENABLE, (uint8_t)mask::ENABLE_PON | (uint8_t)mask::ENABLE_AEN);

        this->setIntegrationCycles(_int_cycles);
        this->setGain(_gain);

        return _sensor_started;
    }

    void setGain(gain gain)
    {
        if (_sensor_started)
        {
            _gain = gain;
            write8(TCS34725_COMMAND_BIT | (uint8_t)reg::CONTROL, (uint8_t)_gain);
        }
    }

    void setIntegrationCycles(uint8_t integration_cycles)
    {
        if (_sensor_started)
        {
            _int_cycles = constrain(integration_cycles, 1, 255);
            _max_count = _int_cycles * 1024;
            write8(TCS34725_COMMAND_BIT | (uint8_t)reg::ATIME, (uint8_t)(255 - (_int_cycles - 1)));
            _int_time_us = _int_cycles * 2400;
        }
    }

    void setRange(uint8_t component, uint16_t low, uint16_t high)
    {
        component = constrain(component, 0, 3);

        switch (component)
        {
        case CLEAR:
            _c_norm = true;
            _c_low = low;
            _c_high = max(low + 1, high);
            break;
        case RED:
            _r_norm = true;
            _r_low = low;
            _r_high = max(low + 1, high);
            break;
        case GREEN:
            _g_norm = true;
            _g_low = low;
            _g_high = max(low + 1, high);
            break;
        case BLUE:
            _b_norm = true;
            _b_low = low;
            _b_high = max(low + 1, high);
            break;
        }
    }

    void setClearRange(uint16_t low, uint16_t high) { setRange(CLEAR, low, high); }
    void setRedRange(uint16_t low, uint16_t high) { setRange(RED, low, high); }
    void setGreenRange(uint16_t low, uint16_t high) { setRange(GREEN, low, high); }
    void setBlueRange(uint16_t low, uint16_t high) { setRange(BLUE, low, high); }

    uint16_t read(uint8_t component, bool blocking = true)
    {
        component = constrain(component, 0, 3);

        if (_sensor_started)
        {
            this->update(blocking);

            switch (component)
            {
            case CLEAR: return _c;
            case RED: return _r;
            case GREEN: return _g;
            case BLUE: return _b;
            }
        }
        return 0;
    }

    uint16_t readRed(bool blocking = true) { return read(RED, blocking); }
    uint16_t readGreen(bool blocking = true) { return read(GREEN, blocking); }
    uint16_t readBlue(bool blocking = true) { return read(BLUE, blocking); }
    uint16_t readClear(bool blocking = true) { return read(CLEAR, blocking); }

    float readPct(uint8_t component, bool blocking = true)
    {
        component = constrain(component, 0, 3);

        if (_sensor_started)
        {
            this->update(blocking);
            convertToPct();

            switch (component)
            {
            case CLEAR: return _c_pct;
            case RED: return _r_pct;
            case GREEN: return _g_pct;
            case BLUE: return _b_pct;
            }
        }
        return 0;
    }

    float readClearPct(bool blocking = true) { return readPct(CLEAR, blocking); }
    float readRedPct(bool blocking = true) { return readPct(RED, blocking); }
    float readGreenPct(bool blocking = true) { return readPct(GREEN, blocking); }
    float readBluePct(bool blocking = true) { return readPct(BLUE, blocking); }

    float readNorm(uint8_t component, bool blocking = true)
    {
        component = constrain(component, 0, 3);

        switch (component)
        {
        case CLEAR:
            if (_c_norm)
                return normalise(this->read(CLEAR, blocking), _c_low, _c_high);
        case RED:
            if (_r_norm)
                return normalise(this->read(RED, blocking), _r_low, _r_high);
        case GREEN:
            if (_g_norm)
                return normalise(this->read(GREEN, blocking), _g_low, _g_high);
        case BLUE:
            if (_b_norm)
                return normalise(this->read(BLUE, blocking), _b_low, _b_high);
        }

        return 0;
    }

    float readClearNorm(bool blocking = true) { return readNorm(CLEAR, blocking); }
    float readRedNorm(bool blocking = true) { return readNorm(RED, blocking); }
    float readGreenNorm(bool blocking = true) { return readNorm(GREEN, blocking); }
    float readBlueNorm(bool blocking = true) { return readNorm(BLUE, blocking); }

    void useNormForHSV(bool enable) { _use_norm_for_hsv = enable; }

    float readHSV(uint8_t component, bool blocking = true)
    {
        component = constrain(component, 0, 2);

        this->update(blocking);
        convertToPct();

        float _r_float = _r_pct;
        float _g_float = _g_pct;
        float _b_float = _b_pct;

        if (_use_norm_for_hsv)
        {
            _r_float = normalise(this->read(RED, false), _r_low, _r_high);
            _g_float = normalise(this->read(GREEN, false), _g_low, _g_high);
            _b_float = normalise(this->read(BLUE, false), _b_low, _b_high);
        }

        float cmax = max(_r_float, max(_g_float, _b_float));
        float cmin = min(_r_float, min(_g_float, _b_float));
        float cdiff = cmax - cmin;

        switch (component)
        {
        case HUE:
            float hue;
            if (cdiff == 0)
                hue = 0;
            else if (cmax == _r_float)
                hue = 60 * fmod((_g_float - _b_float) / cdiff, 6);

            else if (cmax == _g_float)
                hue = 60 * ((_b_float - _r_float) / cdiff + 2);

            else if (cmax == _b_float)
                hue = 60 * ((_r_float - _g_float) / cdiff + 4);
            return hue;
        case SAT:
            return cmax ? cdiff / cmax : 0;
        case VAL:
            return cmax;
        }
    }

    float readHue(bool blocking = true) { return readHSV(HUE, blocking); }
    float readSaturation(bool blocking = true) { return readHSV(SAT, blocking); }
    float readValue(bool blocking = true) { return readHSV(VAL, blocking); }

private:

    float normalise(uint16_t reading, uint16_t low, uint16_t high)
    {
        return constrain(((float)(reading - low)) / ((float)(high - low)), 0, 1);
    }

    void convertToPct()
    {
        if (!_converted_to_pct)
        {
            _c_pct = (float)_c / (float)_max_count * 100;
            _r_pct = (float)_r / (float)_max_count * 100;
            _g_pct = (float)_g / (float)_max_count * 100;
            _b_pct = (float)_b / (float)_max_count * 100;
            _converted_to_pct = true;
        }
    }

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
    }

    uint8_t _buffer[8] = { 0 };

    uint32_t _last_reading_us;
    uint8_t _int_cycles;
    uint32_t _int_time_us;
    uint16_t _max_count;
    gain _gain;

    uint16_t _r = 0, _g = 0, _b = 0, _c = 0;
    bool _converted_to_pct = false, _use_norm_for_hsv = false;
    float _r_pct = 0, _g_pct = 0, _b_pct = 0, _c_pct = 0;
    bool _r_norm = false, _g_norm = false, _b_norm = false, _c_norm = false;
    uint16_t _r_low, _g_low, _b_low, _c_low;
    uint16_t _r_high, _g_high, _b_high, _c_high;
};

#endif