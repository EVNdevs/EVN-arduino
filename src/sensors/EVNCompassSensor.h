#ifndef EVNCompassSensor_h
#define EVNCompassSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

#define COMPASS_HMC_HZ_75           (EVNCompassSensor::hmc_data_rate::HZ_75)
#define COMPASS_HMC_HZ_30           (EVNCompassSensor::hmc_data_rate::HZ_30)
#define COMPASS_HMC_HZ_15           (EVNCompassSensor::hmc_data_rate::HZ_15)
#define COMPASS_HMC_HZ_7_5          (EVNCompassSensor::hmc_data_rate::HZ_7_5)
#define COMPASS_HMC_HZ_3            (EVNCompassSensor::hmc_data_rate::HZ_3)
#define COMPASS_HMC_HZ_1_5          (EVNCompassSensor::hmc_data_rate::HZ_1_5)
#define COMPASS_HMC_HZ_0_75         (EVNCompassSensor::hmc_data_rate::HZ_0_75)

#define COMPASS_HMC_GA_8_1          (EVNCompassSensor::hmc_range::GA_8_1)
#define COMPASS_HMC_GA_5_6          (EVNCompassSensor::hmc_range::GA_5_6)
#define COMPASS_HMC_GA_4_7          (EVNCompassSensor::hmc_range::GA_4_7)
#define COMPASS_HMC_GA_4            (EVNCompassSensor::hmc_range::GA_4)
#define COMPASS_HMC_GA_2_5          (EVNCompassSensor::hmc_range::GA_2_5)
#define COMPASS_HMC_GA_1_9          (EVNCompassSensor::hmc_range::GA_1_9)
#define COMPASS_HMC_GA_1_3          (EVNCompassSensor::hmc_range::GA_1_3)
#define COMPASS_HMC_GA_0_88         (EVNCompassSensor::hmc_range::GA_0_88)

#define COMPASS_HMC_SAMPLING_X1     (EVNCompassSensor::hmc_sampling::X1)
#define COMPASS_HMC_SAMPLING_X2     (EVNCompassSensor::hmc_sampling::X2)
#define COMPASS_HMC_SAMPLING_X4     (EVNCompassSensor::hmc_sampling::X4)
#define COMPASS_HMC_SAMPLING_X8     (EVNCompassSensor::hmc_sampling::X8)

#define COMPASS_QMC_HZ_10           (EVNCompassSensor::qmc_data_rate::HZ_10)
#define COMPASS_QMC_HZ_50           (EVNCompassSensor::qmc_data_rate::HZ_50)
#define COMPASS_QMC_HZ_100          (EVNCompassSensor::qmc_data_rate::HZ_100)
#define COMPASS_QMC_HZ_200          (EVNCompassSensor::qmc_data_rate::HZ_200)

#define COMPASS_QMC_GA_8            (EVNCompassSensor::qmc_range::GA_8)
#define COMPASS_QMC_GA_2            (EVNCompassSensor::qmc_range::GA_2)

#define COMPASS_QMC_SAMPLING_X64    (EVNCompassSensor::qmc_sampling::X64)
#define COMPASS_QMC_SAMPLING_X128   (EVNCompassSensor::qmc_sampling::X128)
#define COMPASS_QMC_SAMPLING_X256   (EVNCompassSensor::qmc_sampling::X256)
#define COMPASS_QMC_SAMPLING_X512   (EVNCompassSensor::qmc_sampling::X512)

class EVNCompassSensor : private EVNI2CDevice {
public:

    friend class EVNIMUSensor;

    static const uint8_t HMC_I2C_ADDR = 0x1E;
    static const uint8_t HMC_CHIP_ID = 0x48 ^ 0x34 ^ 0x33;

    static const uint8_t QMC_I2C_ADDR = 0x0D;
    static const uint8_t QMC_CHIP_ID = 0xFF;

    enum hmc_reg
    {
        CONFIG_A = 0x00,
        CONFIG_B = 0x01,
        MODE = 0x02,
        OUT_X_M = 0x03,
        OUT_X_L = 0x04,
        OUT_Z_M = 0x05,
        OUT_Z_L = 0x06,
        OUT_Y_M = 0x07,
        OUT_Y_L = 0x08,
        STATUS = 0x09,
        IDEN_A = 0x0A,
        IDEN_B = 0x0B,
        IDEN_C = 0x0C,
    };

    enum class hmc_mode : uint8_t
    {
        CONTINUOUS = 0x00,
        // MODE_SINGLE = 0x01,
        STANDBY = 0x02,
    };

    enum class hmc_data_rate : uint8_t
    {
        HZ_75 = 0x06,
        HZ_30 = 0x05,
        HZ_15 = 0x04,
        HZ_7_5 = 0x03,
        HZ_3 = 0x02,
        HZ_1_5 = 0x01,
        HZ_0_75 = 0x00,
    };

    enum class hmc_range : uint8_t
    {
        GA_8_1 = 0x07,
        GA_5_6 = 0x06,
        GA_4_7 = 0x05,
        GA_4 = 0x04,
        GA_2_5 = 0x03,
        GA_1_9 = 0x02,
        GA_1_3 = 0x01,
        GA_0_88 = 0x00,
    };

    enum class hmc_sampling : uint8_t
    {
        X1 = 0x00,
        X2 = 0x01,
        X4 = 0x02,
        X8 = 0x03,
    };

    enum qmc_reg
    {
        X_L = 0x00,
        X_M = 0x01,
        Y_L = 0x02,
        Y_M = 0x03,
        Z_L = 0x04,
        Z_M = 0x05,
        QMC_STATUS = 0x06,
        TEMP_M = 0x07,
        TEMP_L = 0x08,
        CONFIG = 0x09,
        CONFIG2 = 0x0A,
        RESET = 0x0B,
        RESERVED = 0x0C,
        CHIP_ID = 0x0D,
    };

    enum class qmc_mode : uint8_t
    {
        STANDBY = 0x00,
        CONTINUOUS = 0x01,
    };

    enum class qmc_data_rate : uint8_t
    {
        HZ_10 = 0x00,
        HZ_50 = 0x01,
        HZ_100 = 0x02,
        HZ_200 = 0x03,
    };

    enum class qmc_range : uint8_t
    {
        GA_8 = 0x01,
        GA_2 = 0x00,
    };

    enum class qmc_sampling : uint8_t {
        X64 = 0x03,
        X128 = 0x02,
        X256 = 0x01,
        X512 = 0x00,
    };

    EVNCompassSensor(uint8_t port,
        float hard_x = 0, float hard_y = 0, float hard_z = 0,
        float soft_x_0 = 1, float soft_x_1 = 0, float soft_x_2 = 0,
        float soft_y_0 = 0, float soft_y_1 = 1, float soft_y_2 = 0,
        float soft_z_0 = 0, float soft_z_1 = 0, float soft_z_2 = 1) : EVNI2CDevice(port)
    {
        _x_hard_cal = hard_x / 100;
        _y_hard_cal = hard_y / 100;
        _z_hard_cal = hard_z / 100;

        _x_soft_cal[0] = soft_x_0;
        _x_soft_cal[1] = soft_x_1;
        _x_soft_cal[2] = soft_x_2;
        _y_soft_cal[0] = soft_y_0;
        _y_soft_cal[1] = soft_y_1;
        _y_soft_cal[2] = soft_y_2;
        _z_soft_cal[0] = soft_z_0;
        _z_soft_cal[1] = soft_z_1;
        _z_soft_cal[2] = soft_z_2;

        if (hard_x != 0 || hard_y != 0 || hard_z != 0)
            _calibrated = true;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        _addr = HMC_I2C_ADDR;

        uint8_t id =
            read8((uint8_t)hmc_reg::IDEN_A) ^
            read8((uint8_t)hmc_reg::IDEN_B) ^
            read8((uint8_t)hmc_reg::IDEN_C);

        if (id != HMC_CHIP_ID)
        {
            _addr = QMC_I2C_ADDR;
            id = read8((uint8_t)qmc_reg::CHIP_ID);
            if (id == QMC_CHIP_ID)
                _is_qmc = true;
            else
                return _sensor_started;
        }

        _sensor_started = true;

        setMode(true);

        if (_is_qmc)
        {
            _addr = QMC_I2C_ADDR;
            write8((uint8_t)qmc_reg::RESET, 0x01);
            delay(10);
            setDataRateQMC(qmc_data_rate::HZ_200);
            setRangeQMC(qmc_range::GA_2);
            setSamplingRateQMC(qmc_sampling::X512);
        }
        else
        {
            _addr = HMC_I2C_ADDR;
            setDataRateHMC(hmc_data_rate::HZ_75);
            setRangeHMC(hmc_range::GA_4);
            setSamplingRateHMC(hmc_sampling::X1);
        }
        return _sensor_started;
    };

    bool isQMC()
    {
        return _is_qmc;
    };

    bool isHMC()
    {
        return !_is_qmc;
    };

    void setCalibration(float hard_x = 0, float hard_y = 0, float hard_z = 0,
        float soft_x_0 = 1, float soft_x_1 = 0, float soft_x_2 = 0,
        float soft_y_0 = 0, float soft_y_1 = 1, float soft_y_2 = 0,
        float soft_z_0 = 0, float soft_z_1 = 0, float soft_z_2 = 1)
    {
        _x_hard_cal = hard_x / 100;
        _y_hard_cal = hard_y / 100;
        _z_hard_cal = hard_z / 100;

        _x_soft_cal[0] = soft_x_0;
        _x_soft_cal[1] = soft_x_1;
        _x_soft_cal[2] = soft_x_2;
        _y_soft_cal[0] = soft_y_0;
        _y_soft_cal[1] = soft_y_1;
        _y_soft_cal[2] = soft_y_2;
        _z_soft_cal[0] = soft_z_0;
        _z_soft_cal[1] = soft_z_1;
        _z_soft_cal[2] = soft_z_2;

        if (hard_x != 0 || hard_y != 0 || hard_z != 0)
            _calibrated = true;
    };

    void setMode(bool enable)
    {
        if (_sensor_started)
        {
            if (!_is_qmc)
            {
                uint8_t value;
                value = read8(hmc_reg::MODE);
                value &= 0b11111100;
                if (enable)
                    value |= hmc_mode::CONTINUOUS;
                else
                    value |= hmc_mode::STANDBY;
                write8(hmc_reg::MODE, value);
            }
            else
            {
                uint8_t value;
                value = read8(qmc_reg::CONFIG);
                value &= 0b11111100;
                if (enable)
                    value |= qmc_mode::CONTINUOUS;
                else
                    value |= qmc_mode::STANDBY;
                write8(qmc_reg::CONFIG, value);
            }
        }
    };

    void setDataRateHMC(hmc_data_rate data_rate)
    {
        if (_sensor_started && !_is_qmc)
        {
            switch (data_rate)
            {
            case hmc_data_rate::HZ_0_75:
                _freq = 0.75;
                break;
            case hmc_data_rate::HZ_1_5:
                _freq = 1.5;
                break;
            case hmc_data_rate::HZ_3:
                _freq = 3;
                break;
            case hmc_data_rate::HZ_7_5:
                _freq = 7.5;
                break;
            case hmc_data_rate::HZ_15:
                _freq = 15;
                break;
            case hmc_data_rate::HZ_30:
                _freq = 30;
                break;
            case hmc_data_rate::HZ_75:
                _freq = 75;
                break;
            }

            uint8_t value;
            value = read8(hmc_reg::CONFIG_A);
            value &= 0b11100011;
            value |= ((uint8_t)data_rate << 2);
            write8(hmc_reg::CONFIG_A, value);
        }
    };

    void setDataRateQMC(qmc_data_rate data_rate)
    {
        if (_sensor_started && _is_qmc)
        {
            switch (data_rate)
            {
            case qmc_data_rate::HZ_10:
                _freq = 10;
                break;
            case qmc_data_rate::HZ_50:
                _freq = 50;
                break;
            case qmc_data_rate::HZ_100:
                _freq = 100;
                break;
            case qmc_data_rate::HZ_200:
                _freq = 200;
                break;
            }

            uint8_t value;
            value = read8(qmc_reg::CONFIG);
            value &= 0b11110011;
            value |= ((uint8_t)data_rate << 2);
            write8(qmc_reg::CONFIG, value);
        }
    };

    void setRangeHMC(hmc_range range)
    {
        if (_sensor_started && !_is_qmc)
        {
            switch (range)
            {
            case hmc_range::GA_0_88:
                _gain = 1370;
                break;
            case hmc_range::GA_1_3:
                _gain = 1090;
                break;
            case hmc_range::GA_1_9:
                _gain = 820;
                break;
            case hmc_range::GA_2_5:
                _gain = 660;
                break;
            case hmc_range::GA_4:
                _gain = 440;
                break;
            case hmc_range::GA_4_7:
                _gain = 390;
                break;
            case hmc_range::GA_5_6:
                _gain = 330;
                break;
            case hmc_range::GA_8_1:
                _gain = 230;
                break;
            }
            write8(hmc_reg::CONFIG_B, (uint8_t)range << 5);
        }
    };

    void setRangeQMC(qmc_range range)
    {
        if (_sensor_started && _is_qmc)
        {
            switch (range)
            {
            case qmc_range::GA_2:
                _gain = 12000;
                break;
            case qmc_range::GA_8:
                _gain = 3000;
                break;
            }
            uint8_t value;
            value = read8(qmc_reg::CONFIG);
            value &= 0b11001111;
            value |= ((uint8_t)range << 4);
            write8(qmc_reg::CONFIG, value);
        }
    };

    void setSamplingRateHMC(hmc_sampling samples)
    {
        if (_sensor_started && !_is_qmc)
        {
            uint8_t value;
            value = read8(hmc_reg::CONFIG_A);
            value &= 0b10011111;
            value |= ((uint8_t)samples << 5);
            write8(hmc_reg::CONFIG_A, value);
        }
    };

    void setSamplingRateQMC(qmc_sampling samples)
    {
        if (_sensor_started && _is_qmc)
        {
            uint8_t value;
            value = read8(qmc_reg::CONFIG);
            value &= 0b00111111;
            value |= ((uint8_t)samples << 6);
            write8(qmc_reg::CONFIG, value);
        }
    };

    bool isCalibrated()
    {
        return  _calibrated;
    };

    float readRawX(bool blocking = true)
    {
        this->update(blocking);
        return (float)_x / _gain;
    };

    float readRawY(bool blocking = true)
    {
        this->update(blocking);
        return (float)_y / _gain;
    };

    float readRawZ(bool blocking = true)
    {
        this->update(blocking);
        return (float)_z / _gain;
    };

    float readCalX(bool blocking = true)
    {
        this->update(blocking);
        this->calibrateReadings();
        return (float)_xcal;
    };

    float readCalY(bool blocking = true)
    {
        this->update(blocking);
        this->calibrateReadings();
        return (float)_ycal;
    };

    float readCalZ(bool blocking = true)
    {
        this->update(blocking);
        this->calibrateReadings();
        return (float)_zcal;
    };

    float read(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            this->calibrateReadings();

            _yaw = (atan2(_ycal, _xcal) / M_PI * 180) + 180;
            return fmod(_yaw - _yaw_offset + 360, 360);
        }
        return 0;
    };

    void setNorth()
    {
        this->read();
        _yaw_offset = _yaw;
    };

    void setHeading(uint8_t heading)
    {
        this->read();
        _yaw_offset = _yaw - heading;
    };

    void setTopAxis(uint8_t axis)
    {
        axis = constrain(axis, 0, 2);
        _top_axis = axis;

        if (_top_axis == _front_axis)
        {
            if ((_right_axis == AXIS_X && _top_axis == AXIS_Y) || (_top_axis == AXIS_X && _right_axis == AXIS_Y))
                _front_axis = AXIS_Z;
            else if ((_right_axis == AXIS_Y && _top_axis == AXIS_Z) || (_top_axis == AXIS_Y && _right_axis == AXIS_Z))
                _front_axis = AXIS_X;
            else
                _front_axis = AXIS_Y;
        }

        if ((_top_axis == AXIS_X && _front_axis == AXIS_Y) || (_front_axis == AXIS_X && _top_axis == AXIS_Y))
            _right_axis = AXIS_Z;
        else if ((_top_axis == AXIS_Y && _front_axis == AXIS_Z) || (_front_axis == AXIS_Y && _top_axis == AXIS_Z))
            _right_axis = AXIS_X;
        else
            _right_axis = AXIS_Y;
    };

    void setFrontAxis(uint8_t axis)
    {
        axis = constrain(axis, 0, 2);
        _front_axis = axis;

        if (_top_axis == _front_axis)
        {
            if ((_right_axis == AXIS_X && _front_axis == AXIS_Y) || (_front_axis == AXIS_X && _right_axis == AXIS_Y))
                _top_axis = AXIS_Z;
            else if ((_right_axis == AXIS_Y && _front_axis == AXIS_Z) || (_front_axis == AXIS_Y && _right_axis == AXIS_Z))
                _top_axis = AXIS_X;
            else
                _top_axis = AXIS_Y;
        }

        if ((_top_axis == AXIS_X && _front_axis == AXIS_Y) || (_front_axis == AXIS_X && _top_axis == AXIS_Y))
            _right_axis = AXIS_Z;
        else if ((_top_axis == AXIS_Y && _front_axis == AXIS_Z) || (_front_axis == AXIS_Y && _top_axis == AXIS_Z))
            _right_axis = AXIS_X;
        else
            _right_axis = AXIS_Y;
    };

protected:
    void update(bool blocking = true)
    {
        if (_sensor_started)
        {
            if (blocking)
                while ((micros() - _last_reading_us) < (1000000 / _freq));

            if ((micros() - _last_reading_us) >= (1000000 / _freq))
            {
                if (_is_qmc)
                {
                    readBuffer((uint8_t)qmc_reg::X_L, 6, _buffer);

                    switch (_top_axis)
                    {
                    case AXIS_X:
                        _z = _buffer[1] << 8 | _buffer[0];
                        break;
                    case AXIS_Y:
                        _z = _buffer[3] << 8 | _buffer[2];
                        break;
                    case AXIS_Z:
                        _z = _buffer[5] << 8 | _buffer[4];
                        break;
                    }

                    switch (_front_axis)
                    {
                    case AXIS_X:
                        _x = _buffer[1] << 8 | _buffer[0];
                        break;
                    case AXIS_Y:
                        _x = _buffer[3] << 8 | _buffer[2];
                        break;
                    case AXIS_Z:
                        _x = _buffer[5] << 8 | _buffer[4];
                        break;
                    }

                    switch (_right_axis)
                    {
                    case AXIS_X:
                        _y = _buffer[1] << 8 | _buffer[0];
                        break;
                    case AXIS_Y:
                        _y = _buffer[3] << 8 | _buffer[2];
                        break;
                    case AXIS_Z:
                        _y = _buffer[5] << 8 | _buffer[4];
                        break;
                    }

                    // _x = _buffer[1] << 8 | _buffer[0];
                    // _y = _buffer[3] << 8 | _buffer[2];
                    // _z = _buffer[5] << 8 | _buffer[4];
                }
                else
                {
                    readBuffer((uint8_t)hmc_reg::OUT_X_M, 6, _buffer);
                    // _x = _buffer[0] << 8 | _buffer[1];
                    // _z = _buffer[2] << 8 | _buffer[3];
                    // _y = _buffer[4] << 8 | _buffer[5];

                    switch (_top_axis)
                    {
                    case AXIS_X:
                        _z = _buffer[0] << 8 | _buffer[1];
                        break;
                    case AXIS_Z:
                        _z = _buffer[2] << 8 | _buffer[3];
                        break;
                    case AXIS_Y:
                        _z = _buffer[4] << 8 | _buffer[5];
                        break;
                    }

                    switch (_front_axis)
                    {
                    case AXIS_X:
                        _x = _buffer[0] << 8 | _buffer[1];
                        break;
                    case AXIS_Z:
                        _x = _buffer[2] << 8 | _buffer[3];
                        break;
                    case AXIS_Y:
                        _x = _buffer[4] << 8 | _buffer[5];
                        break;
                    }

                    switch (_right_axis)
                    {
                    case AXIS_X:
                        _y = _buffer[0] << 8 | _buffer[1];
                        break;
                    case AXIS_Z:
                        _y = _buffer[2] << 8 | _buffer[3];
                        break;
                    case AXIS_Y:
                        _y = _buffer[4] << 8 | _buffer[5];
                        break;
                    }
                }
                _last_reading_us = micros();
            }
        }
    };

    void calibrateReadings()
    {
        if (_sensor_started)
        {
            if (_calibrated)
            {
                float x0, y0, z0;
                x0 = (float)_x / _gain - _x_hard_cal;
                y0 = (float)_y / _gain - _y_hard_cal;
                z0 = (float)_z / _gain - _z_hard_cal;
                _xcal = x0 * _x_soft_cal[0] + y0 * _x_soft_cal[1] + z0 * _x_soft_cal[2];
                _ycal = x0 * _y_soft_cal[0] + y0 * _y_soft_cal[1] + z0 * _y_soft_cal[2];
                _zcal = x0 * _z_soft_cal[0] + y0 * _z_soft_cal[1] + z0 * _z_soft_cal[2];
            }
            else
            {
                _xcal = (float)_x / _gain;
                _ycal = (float)_y / _gain;
                _zcal = (float)_z / _gain;
            }
        }
    };

    int16_t _x = 0, _y = 0, _z = 0;
    float _xcal = 0, _ycal = 0, _zcal = 0;
    float _yaw_offset = 0;
    float _yaw;

    float _x_hard_cal, _x_soft_cal[3];
    float _y_hard_cal, _y_soft_cal[3];
    float _z_hard_cal, _z_soft_cal[3];

    uint8_t _top_axis = AXIS_Z, _front_axis = AXIS_X, _right_axis = AXIS_Y;

    float _freq, _gain;
    uint32_t _last_reading_us = 0;
    bool _calibrated = true, _is_qmc = false;

    uint8_t _buffer[6] = { 0 };
};

#endif