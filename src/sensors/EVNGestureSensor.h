#ifndef EVNGestureSensor_h
#define EVNGestureSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

#define GESTURE_NONE    0
#define GESTURE_UP      1
#define GESTURE_DOWN    2
#define GESTURE_LEFT    3
#define GESTURE_RIGHT   4

class EVNGestureSensor : private EVNI2CDevice {
public:
    static const uint8_t I2C_ADDR = 0x39;
    static const uint8_t ID = 0xAB;
    static const uint8_t ALT_ID = 0xA8;

    enum class reg : uint8_t
    {
        ENABLE = 0x80,
        ATIME = 0x81,
        WTIME = 0x83,
        AILTL = 0x84,
        AILTH = 0x85,
        AIHTL = 0x86,
        AIHTH = 0x87,
        PILT = 0x89,
        PIHT = 0x8B,
        PERS = 0x8C,
        CONFIG1 = 0x8D,
        PPULSE = 0x8E,
        CONTROL = 0x8F,
        CONFIG2 = 0x90,
        ID = 0x92,
        STATUS = 0x93,
        CDATAL = 0x94,
        CDATAH = 0x95,
        RDATAL = 0x96,
        RDATAH = 0x97,
        GDATAL = 0x98,
        GDATAH = 0x99,
        BDATAL = 0x9A,
        BDATAH = 0x9B,
        PDATA = 0x9C,
        POFFSET_UR = 0x9D,
        POFFSET_DL = 0x9E,
        CONFIG3 = 0x9F,
        GPENTH = 0xA0,
        GEXTH = 0xA1,
        GCONF1 = 0xA2,
        GCONF2 = 0xA3,
        GOFFSET_U = 0xA4,
        GOFFSET_D = 0xA5,
        GOFFSET_L = 0xA7,
        GOFFSET_R = 0xA9,
        GPULSE = 0xA6,
        GCONF3 = 0xAA,
        GCONF4 = 0xAB,
        GFLVL = 0xAE,
        GSTATUS = 0xAF,
        IFORCE = 0xE4,
        PICLEAR = 0xE5,
        CICLEAR = 0xE6,
        AICLEAR = 0xE7,
        GFIFO_U = 0xFC,
        GFIFO_D = 0xFD,
        GFIFO_L = 0xFE,
        GFIFO_R = 0xFF,
    };

    enum class gesture_fifo : uint8_t
    {
        DATASETS_1 = 0x00,
        DATASETS_4 = 0x01,
        DATASETS_8 = 0x02,
        DATASETS_16 = 0x03
    };

    enum class led_curr : uint8_t
    {
        MA_100 = 0,
        MA_50 = 1,
        MA_25 = 2,
        MA_12_5 = 3,
    };

    enum class led_boost : uint8_t
    {
        X1 = 0,
        X1_5 = 1,
        X2 = 2,
        X3 = 3,
    };

    enum class colour_gain : uint8_t
    {
        X1 = 0x00,
        X4 = 0x01,
        X16 = 0x02,
        X64 = 0x03
    };

    enum class gesture_gain : uint8_t
    {
        X1 = 0x00,
        X2 = 0x01,
        X4 = 0x02,
        X8 = 0x03
    };

    enum class gesture_dims : uint8_t
    {
        UP_DOWN = 0x01,
        LEFT_RIGHT = 0x02,
        ALL = 0x03,
    };

    enum class proximity_gain : uint8_t
    {
        X1 = 0x00,
        X2 = 0x01,
        X4 = 0x02,
        X8 = 0x03
    };

    enum class pulse_len : uint8_t
    {
        US_4 = 0x00,
        US_8 = 0x01,
        US_16 = 0x02,
        US_32 = 0x03
    };

    EVNGestureSensor(uint8_t port) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        uint8_t id = read8((uint8_t)reg::ID, false);
        if (id != ID && id != ALT_ID)
        {
            return _sensor_started;
        }

        _sensor_started = true;

        write8((uint8_t)reg::ENABLE, 0x00);
        delay(10);

        setColourGain(colour_gain::X4);
        setColourIntegrationCycles(1);

        setProximityLED(led_curr::MA_100);
        setGestureLED(led_curr::MA_100);
        setLEDBoost(led_boost::X1);

        setGestureDimensions(gesture_dims::ALL);
        setGestureFIFOThreshold(gesture_fifo::DATASETS_4);
        setGestureGain(gesture_gain::X4);
        setGestureEntryThreshold(30);
        setGestureExitThreshold(20);
        setGesturePulseLength(pulse_len::US_32);
        setGesturePulseCount(10);

        setProximityGain(proximity_gain::X4);
        setProximityPulseCount(8);
        setProximityPulseLength(pulse_len::US_16);

        setPower(true);
        setWait(false);
        setColourMode(true);
        setProximityMode(true);
        setGestureMode(true);

        return _sensor_started;
    };

    void setPower(bool enable)
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b11111110;
            value |= enable ? 1 : 0;
            write8((uint8_t)reg::ENABLE, value);
        }
    };

    void setWait(bool enable)
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b11110111;
            value |= (enable << 3);
            write8((uint8_t)reg::ENABLE, value);
        }
    };

    void setLEDBoost(led_boost boost)
    {
        if (_sensor_started)
        {
            bool mode1 = getProximityMode();
            bool mode2 = getGestureMode();
            setProximityMode(false);
            setGestureMode(false);
            uint8_t value = read8((uint8_t)reg::CONFIG2, false);
            value &= 0b11001111;
            value |= ((uint8_t)boost << 4);
            write8((uint8_t)reg::CONFIG2, value);
            setProximityMode(mode1);
            setGestureMode(mode2);
        }
    };

    void setColourMode(bool enable)
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b11111101;
            value |= (enable << 1);
            write8((uint8_t)reg::ENABLE, value);

            if (enable)
                _colour_measurement_time_us = 278 * getColourIntegrationCycles();
            else
                _colour_measurement_time_us = 0;
        }
    };

    bool getColourMode()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b00000010;
            return (value >> 1);
        }
        return false;
    };

    void setProximityMode(bool enable)
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b11111011;
            value |= (enable << 2);
            write8((uint8_t)reg::ENABLE, value);
            _skip_proximity = !enable;

            if (enable)
                _proximity_measurement_time_us = 697;
            else
                _proximity_measurement_time_us = 0;
        }
    };

    bool getProximityMode()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 000000100;
            return (value >> 2);
        }
        return false;
    };

    void setGestureMode(bool enable)
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b10111111;
            value |= (enable << 6);
            write8((uint8_t)reg::ENABLE, value);

            if (!enable)
            {
                value = read8((uint8_t)reg::GCONF4, false);
                value &= 0b11111110;
                value |= enable;
                write8((uint8_t)reg::GCONF4, value);
                _gesture_measurement_time_us = 0;
            }
            else
                _gesture_measurement_time_us = 697 * 2;
        }
    };

    bool getGestureMode()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ENABLE, false);
            value &= 0b01000000;
            return (value >> 6);
        }
        return false;
    };

    void setGestureLED(led_curr current)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            uint8_t value = read8((uint8_t)reg::GCONF2, false);
            value &= 0b11100111;
            value |= ((uint8_t)current << 3);
            write8((uint8_t)reg::GCONF2, value);
            setGestureMode(mode);
        }
    };

    void setGestureGain(gesture_gain gain)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            uint8_t value = read8((uint8_t)reg::GCONF2, false);
            value &= 0b10011111;
            value |= ((uint8_t)gain << 5);
            write8((uint8_t)reg::GCONF2, value);
            setGestureMode(mode);
        }
    };

    void setGesturePulseCount(uint8_t pulse_count)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            uint8_t pulse_countc = constrain(pulse_count, 1, 64);
            uint8_t value = read8((uint8_t)reg::GPULSE, false);
            value &= 0b11000000;
            value |= ((uint8_t)pulse_count - 1);
            write8((uint8_t)reg::GPULSE, value);
            setGestureMode(mode);
        }
    };

    void setGesturePulseLength(pulse_len pulse_length)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            uint8_t value = read8((uint8_t)reg::GPULSE, false);
            value &= 0b00111111;
            value |= ((uint8_t)pulse_length << 6);
            write8((uint8_t)reg::GPULSE, value);
            setGestureMode(mode);
        }
    };

    void setGestureFIFOThreshold(gesture_fifo threshold)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            uint8_t value = read8((uint8_t)reg::GCONF1, false);
            value &= 0b00111111;
            value |= ((uint8_t)threshold << 6);
            write8((uint8_t)reg::GCONF1, value);
            setGestureMode(mode);
        }
    };

    void setGestureEntryThreshold(uint8_t threshold)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            write8((uint8_t)reg::GPENTH, threshold);
            setGestureMode(mode);
        }
    };

    void setGestureExitThreshold(uint8_t threshold)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            write8((uint8_t)reg::GEXTH, threshold);
            setGestureMode(mode);
        }
    };

    void setGestureDimensions(gesture_dims dims)
    {
        if (_sensor_started)
        {
            bool mode = getGestureMode();
            setGestureMode(false);
            write8((uint8_t)reg::GCONF3, (uint8_t)dims);
            setGestureMode(mode);
        }
    };

    bool gestureDetected()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::GSTATUS, false);
            return (value & 0b1);
        }
        return false;
    };

    uint8_t readGesture(bool blocking = false, uint64_t timeout_ms = 5000)
    {
        if (_sensor_started)
        {
            if (blocking)
                blockUntilGesture(timeout_ms);

            if (!obtainGesture()) return GESTURE_NONE;
            return _gesture;
        }
        return GESTURE_NONE;
    };

    uint8_t readGestureUpDown(bool blocking = false, uint64_t timeout_ms = 5000)
    {
        if (_sensor_started)
        {
            if (blocking)
                blockUntilGesture(timeout_ms);

            if (!obtainGesture()) return GESTURE_NONE;
            return _gesture_ud;
        }
        return GESTURE_NONE;
    };

    uint8_t readGestureLeftRight(bool blocking = false, uint64_t timeout_ms = 5000)
    {
        if (_sensor_started)
        {
            if (blocking)
                blockUntilGesture(timeout_ms);

            if (!obtainGesture()) return GESTURE_NONE;
            return _gesture_lr;
        }
        return GESTURE_NONE;
    };

    void setProximityGain(proximity_gain gain)
    {
        if (_sensor_started)
        {
            bool mode = getProximityMode();
            setProximityMode(false);
            uint8_t value = read8((uint8_t)reg::CONTROL, false);
            value &= 0b11110011;
            value |= ((uint8_t)gain << 2);
            write8((uint8_t)reg::CONTROL, value);
            setProximityMode(mode);
        }
    };

    void setProximityLED(led_curr current)
    {
        if (_sensor_started)
        {
            bool mode = getProximityMode();
            setProximityMode(false);
            uint8_t value = read8((uint8_t)reg::CONTROL, false);
            value &= 0b00111111;
            value |= ((uint8_t)current << 6);
            write8((uint8_t)reg::CONTROL, value);
            setProximityMode(mode);
        }
    };

    void setProximityPulseCount(uint8_t pulse_count)
    {
        if (_sensor_started)
        {
            bool mode = getProximityMode();
            setProximityMode(false);
            uint8_t pulse_countc = constrain(pulse_count, 1, 64);
            uint8_t value = read8((uint8_t)reg::PPULSE, false);
            value &= 0b11000000;
            value |= ((uint8_t)pulse_countc - 1);
            write8((uint8_t)reg::PPULSE, value);
            setProximityMode(mode);
        }
    };

    void setProximityPulseLength(pulse_len pulse_length)
    {
        if (_sensor_started)
        {
            bool mode = getProximityMode();
            setProximityMode(false);
            uint8_t value = read8((uint8_t)reg::PPULSE, false);
            value &= 0b00111111;
            value |= ((uint8_t)pulse_length << 6);
            write8((uint8_t)reg::PPULSE, value);
            setProximityMode(mode);
        }
    };

    uint8_t readProximity(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->updateProximity(blocking);
            return _proximity;
        }
        return 0;
    };

    void setColourGain(colour_gain gain)
    {
        if (_sensor_started)
        {
            bool mode = getColourMode();
            setColourMode(false);
            uint8_t value = read8((uint8_t)reg::CONTROL, false);
            value &= 0b11111100;
            value |= (uint8_t)gain;
            write8((uint8_t)reg::CONTROL, value);
            setColourMode(mode);
        }
    };

    uint16_t getColourIntegrationCycles()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::ATIME, false);
            return (256 - value);
        }
        return 0;
    };

    void setColourIntegrationCycles(uint16_t int_cycles)
    {
        if (_sensor_started)
        {
            bool mode = getColourMode();
            setColourMode(false);
            uint8_t int_cyclesc = constrain(int_cycles, 1, 256);
            write8((uint8_t)reg::ATIME, (256 - int_cyclesc));
            _colour_measurement_time_us = 2780 * int_cycles;
            setColourMode(mode);
        }
    };

    uint16_t readRed(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->updateColourandProximity(blocking);
            return _r;
        }
        return 0;
    };

    uint16_t readGreen(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->updateColourandProximity(blocking);
            return _g;
        }
        return 0;
    };

    uint16_t readBlue(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->updateColourandProximity(blocking);
            return _b;
        }
        return 0;
    };

    uint16_t readClear(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->updateColourandProximity(blocking);
            return _c;
        }
        return 0;
    };

private:
    uint8_t getGestureFIFOLevel()
    {
        if (_sensor_started)
        {
            uint8_t value = read8((uint8_t)reg::GFLVL, false);
            return value;
        }
        return 0;
    };

    void updateColourandProximity(bool blocking = false)
    {
        uint64_t measurement_time_us =
            _proximity_measurement_time_us +
            _colour_measurement_time_us +
            _gesture_measurement_time_us;

        if (blocking)
            while (micros() - _last_reading1_us < measurement_time_us);

        if (micros() - _last_reading1_us >= measurement_time_us)
        {
            bool update_proximity = false;

            if (!_skip_proximity && micros() - _last_reading2_us >= measurement_time_us)
            {
                update_proximity = true;
                readBuffer((uint8_t)reg::CDATAL, 9, _buffer1, false);
            }
            else
                readBuffer((uint8_t)reg::CDATAL, 8, _buffer1, false);

            _last_reading1_us = micros();

            if (update_proximity)
            {
                _proximity = _buffer1[8];
                _last_reading2_us = _last_reading1_us;
            }

            _c = _buffer1[0] | _buffer1[1] << 8;
            _r = _buffer1[2] | _buffer1[3] << 8;
            _g = _buffer1[4] | _buffer1[5] << 8;
            _b = _buffer1[6] | _buffer1[7] << 8;
        }
    };

    void updateProximity(bool blocking = false)
    {
        uint64_t measurement_time_us =
            _proximity_measurement_time_us +
            _colour_measurement_time_us +
            _gesture_measurement_time_us;

        if (blocking)
            while (micros() - _last_reading2_us < measurement_time_us);

        if (micros() - _last_reading2_us >= measurement_time_us)
        {
            _last_reading2_us = micros();
            _proximity = read8((uint8_t)reg::PDATA, false);
        }
    };

    void blockUntilGesture(uint64_t timeout_ms)
    {
        uint64_t start = millis();
        uint64_t timeout_msc = timeout_ms;
        if (timeout_ms == 0) timeout_msc = INT_MAX;
        while (millis() - start <= timeout_msc && !gestureDetected());
    };

    void updateGesture(uint8_t fifo_lvl)
    {
        readBuffer((uint8_t)reg::GFIFO_U, 4 * fifo_lvl, _buffer2, false);
    };

    uint8_t obtainGesture()
    {
        if (_sensor_started)
        {
            _gesture_fifo_index = 0;
            _gesture = GESTURE_NONE;
            _gesture_ud = GESTURE_NONE;
            _gesture_lr = GESTURE_NONE;

            if (!gestureDetected())
                return GESTURE_NONE;   //NO GESTURE

            while (true)
            {
                if (gestureDetected())
                {
                    uint8_t fifo_lvl = getGestureFIFOLevel();

                    this->updateGesture(fifo_lvl);

                    if (_gesture_fifo_index + fifo_lvl > 64)
                    {
                        return GESTURE_NONE; //NO EXIT
                    }

                    for (int i = 0; i < fifo_lvl; i++)
                    {
                        _gesture_fifo_up[_gesture_fifo_index] = _buffer2[i * 4];
                        _gesture_fifo_down[_gesture_fifo_index] = _buffer2[i * 4 + 1];
                        _gesture_fifo_left[_gesture_fifo_index] = _buffer2[i * 4 + 2];
                        _gesture_fifo_right[_gesture_fifo_index] = _buffer2[i * 4 + 3];
                        _gesture_fifo_index++;
                    }
                }
                else
                    break; //keep reading until gesture has exited
            }

            if (_gesture_fifo_index < 4)
            {
                return GESTURE_NONE; //NO GESTURE
            }

            processGesture();
            return _gesture;
        }

        return 0;
    };

    void processGesture()
    {
        bool last_gesture_found = false;
        bool first_gesture_found = false;
        uint8_t first_gesture_index = 0;
        uint8_t last_gesture_index = 0;

        for (int i = _gesture_fifo_index; i >= 0; i--)
        {
            if (_gesture_fifo_up[i] > 10 &&
                _gesture_fifo_down[i] > 10 &&
                _gesture_fifo_left[i] > 10 &&
                _gesture_fifo_right[i] > 10)
            {
                if (!last_gesture_found)
                {
                    last_gesture_index = i;
                    last_gesture_found = true;
                }
                else
                {
                    first_gesture_index = i;
                    first_gesture_found = true;
                }
            }
        }

        if (!last_gesture_found || !first_gesture_found)
        {
            return;
        }

        uint8_t up_first = _gesture_fifo_up[first_gesture_index];
        uint8_t down_first = _gesture_fifo_down[first_gesture_index];
        uint8_t left_first = _gesture_fifo_left[first_gesture_index];
        uint8_t right_first = _gesture_fifo_right[first_gesture_index];

        uint8_t up_last = _gesture_fifo_up[last_gesture_index];
        uint8_t down_last = _gesture_fifo_down[last_gesture_index];
        uint8_t left_last = _gesture_fifo_left[last_gesture_index];
        uint8_t right_last = _gesture_fifo_right[last_gesture_index];

        int32_t ud_ratio_first = ((up_first - down_first) * 1000) / (up_first + down_first);
        int32_t lr_ratio_first = ((left_first - right_first) * 1000) / (left_first + right_first);
        int32_t ud_ratio_last = ((up_last - down_last) * 1000) / (up_last + down_last);
        int32_t lr_ratio_last = ((left_last - right_last) * 1000) / (left_last + right_last);

        int32_t ud_delta = ud_ratio_last - ud_ratio_first;
        int32_t lr_delta = lr_ratio_last - lr_ratio_first;

        _gesture_ud = (ud_delta < 0) ? GESTURE_UP : GESTURE_DOWN;
        _gesture_lr = (lr_delta < 0) ? GESTURE_LEFT : GESTURE_RIGHT;
        _gesture = (abs(lr_delta) < abs(ud_delta)) ? _gesture_ud : _gesture_lr;
    };

    bool _skip_proximity = false;
    uint8_t _buffer1[9];
    uint8_t _buffer2[128];

    uint8_t _proximity;
    uint8_t _c, _r, _g, _b;
    uint8_t _gesture;
    uint8_t _gesture_ud;
    uint8_t _gesture_lr;

    uint64_t _gesture_measurement_time_us;
    uint64_t _colour_measurement_time_us;
    uint64_t _proximity_measurement_time_us;
    uint64_t _last_reading1_us;
    uint64_t _last_reading2_us;

    uint8_t _gesture_fifo_up[64];
    uint8_t _gesture_fifo_down[64];
    uint8_t _gesture_fifo_left[64];
    uint8_t _gesture_fifo_right[64];
    uint8_t _gesture_fifo_index;
};

#endif