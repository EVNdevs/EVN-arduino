#ifndef EVNEnvSensor_h
#define EVNEnvSensor_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "../helper/EVNI2CDevice.h"

#define ENV_SAMPLING_OFF    (EVNEnvSensor::sampling::OFF)
#define ENV_SAMPLING_X1     (EVNEnvSensor::sampling::X1)
#define ENV_SAMPLING_X2     (EVNEnvSensor::sampling::X2)
#define ENV_SAMPLING_X4     (EVNEnvSensor::sampling::X4)
#define ENV_SAMPLING_X8     (EVNEnvSensor::sampling::X8)
#define ENV_SAMPLING_X16    (EVNEnvSensor::sampling::X16)

#define ENV_FILTER_OFF      (EVNEnvSensor::filter::OFF)
#define ENV_FILTER_X2       (EVNEnvSensor::filter::X2)
#define ENV_FILTER_X4       (EVNEnvSensor::filter::X4)
#define ENV_FILTER_X8       (EVNEnvSensor::filter::X8)
#define ENV_FILTER_X16      (EVNEnvSensor::filter::X16)

#define ENV_STANDBY_MS_0_5  (EVNEnvSensor::standby::MS_0_5)
#define ENV_STANDBY_MS_10   (EVNEnvSensor::standby::MS_10)
#define ENV_STANDBY_MS_20   (EVNEnvSensor::standby::MS_20)
#define ENV_STANDBY_MS_62_5 (EVNEnvSensor::standby::MS_62_5)
#define ENV_STANDBY_MS_125  (EVNEnvSensor::standby::MS_125)
#define ENV_STANDBY_MS_250  (EVNEnvSensor::standby::MS_250)
#define ENV_STANDBY_MS_500  (EVNEnvSensor::standby::MS_500)
#define ENV_STANDBY_MS_1000 (EVNEnvSensor::standby::MS_1000)

struct evn_env
{
    float pressure;
    float humidity;
    float temperature;
    float temperature_f;
    float altitude;
    float altitude_ft;
};

class EVNEnvSensor : private EVNI2CDevice {
public:
    static const uint8_t I2C_ADDR = 0x76;
    static const uint8_t ID = 0x60;

    struct bme280_calibration_data
    {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;

        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

        uint8_t dig_H1;
        int16_t dig_H2;
        uint8_t dig_H3;
        int16_t dig_H4;
        int16_t dig_H5;
        int8_t dig_H6;
    };

    enum class reg : uint8_t
    {
        DIG_T1 = 0x88,
        DIG_T2 = 0x8A,
        DIG_T3 = 0x8C,
        DIG_P1 = 0x8E,
        DIG_P2 = 0x90,
        DIG_P3 = 0x92,
        DIG_P4 = 0x94,
        DIG_P5 = 0x96,
        DIG_P6 = 0x98,
        DIG_P7 = 0x9A,
        DIG_P8 = 0x9C,
        DIG_P9 = 0x9E,
        DIG_H1 = 0xA1,
        DIG_H2 = 0xE1,
        DIG_H3 = 0xE3,
        DIG_H4 = 0xE4,
        DIG_H5 = 0xE5,
        DIG_H6 = 0xE7,
        ID_REG = 0xD0,
        RESET_REG = 0xE0,
        CTRL_HUM = 0xF2,
        STATUS = 0xF3,
        CTRL_MEAS = 0xF4,
        CONFIG = 0xF5,
        PRES_MSB = 0xF7,
        TEMP_MSB = 0xFA,
        HUM_MSB = 0xFD,
    };

    enum class mode : uint8_t
    {
        SLEEP = 0x00,
        // FORCED = 0x01,
        NORMAL = 0x03,
    };

    enum class filter : uint8_t
    {
        OFF = 0x00,
        X2 = 0x01,
        X4 = 0x02,
        X8 = 0x03,
        X16 = 0x04,
    };

    enum class sampling : uint8_t
    {
        OFF = 0x00,
        X1 = 0x01,
        X2 = 0x02,
        X4 = 0x03,
        X8 = 0x04,
        X16 = 0x05,
    };

    enum class standby : uint8_t
    {
        MS_0_5 = 0x00,
        MS_10 = 0x06,
        MS_20 = 0x07,
        MS_62_5 = 0x01,
        MS_125 = 0x02,
        MS_250 = 0x03,
        MS_500 = 0x04,
        MS_1000 = 0x05,
    };

    EVNEnvSensor(uint8_t port, sampling sampling_temp = ENV_SAMPLING_X1, sampling sampling_pres = ENV_SAMPLING_X1, sampling sampling_hum = ENV_SAMPLING_X1, filter filter = ENV_FILTER_OFF, standby standby = ENV_STANDBY_MS_0_5) : EVNI2CDevice(port)
    {
        _addr = I2C_ADDR;

        _sampling_temp = sampling_temp;
        _sampling_pres = sampling_pres;
        _sampling_hum = sampling_hum;
        _filter = filter;
        _standby = standby;
    };

    bool begin()
    {
        EVNAlpha::sharedPorts().begin();

        EVNAlpha::sharedPorts().setPort(_port);

        if (read8((uint8_t)reg::ID_REG) != ID)
            return _sensor_started;

        write8((uint8_t)reg::RESET_REG, 0xB6);

        delay(2);

        uint32_t start_time = millis();

        while (read8((uint8_t)reg::STATUS) & 0b1 != 0)
        {
            delay(10);
            if (millis() - start_time > 2000)
                return _sensor_started;
        };

        _calibration.dig_T1 = read16((uint8_t)reg::DIG_T1);
        _calibration.dig_T2 = read16((uint8_t)reg::DIG_T2);
        _calibration.dig_T3 = read16((uint8_t)reg::DIG_T3);
        _calibration.dig_P1 = read16((uint8_t)reg::DIG_P1);
        _calibration.dig_P2 = read16((uint8_t)reg::DIG_P2);
        _calibration.dig_P3 = read16((uint8_t)reg::DIG_P3);
        _calibration.dig_P4 = read16((uint8_t)reg::DIG_P4);
        _calibration.dig_P5 = read16((uint8_t)reg::DIG_P5);
        _calibration.dig_P6 = read16((uint8_t)reg::DIG_P6);
        _calibration.dig_P7 = read16((uint8_t)reg::DIG_P7);
        _calibration.dig_P8 = read16((uint8_t)reg::DIG_P8);
        _calibration.dig_P9 = read16((uint8_t)reg::DIG_P9);
        _calibration.dig_H1 = read8((uint8_t)reg::DIG_H1);
        _calibration.dig_H2 = read16((uint8_t)reg::DIG_H2);
        _calibration.dig_H3 = read8((uint8_t)reg::DIG_H3);
        _calibration.dig_H4 = ((int8_t)read8((uint8_t)reg::DIG_H4) << 4) |
            (read8((uint8_t)reg::DIG_H4 + 1) & 0xF);
        _calibration.dig_H5 = ((int8_t)read8((uint8_t)reg::DIG_H5 + 1) << 4) |
            (read8((uint8_t)reg::DIG_H5) >> 4);
        _calibration.dig_H6 = read8((uint8_t)reg::DIG_H6);

        _sensor_started = true;

        setSamplingRate(_sampling_temp, _sampling_hum, _sampling_pres);
        setFilterRate(_filter);
        setStandbyTime(_standby);
        setMode(true);

        _last_reading_us = micros();

        return _sensor_started;
    };

    void setMode(bool enable)
    {
        uint8_t value = read8((uint8_t)reg::CTRL_MEAS);
        value &= 0b11111100;
        if (enable)
            write8((uint8_t)reg::CTRL_MEAS, value | (uint8_t)mode::NORMAL);
        else
            write8((uint8_t)reg::CTRL_MEAS, value | (uint8_t)mode::SLEEP);
    };

    void setSamplingRate(sampling sampling_temp, sampling sampling_hum, sampling sampling_pres)
    {
        uint32_t measurement_time_us = 1250;

        sampling sampling_tempc = sampling_temp;

        if (sampling_tempc == sampling::OFF)
            sampling_tempc = sampling::X1;

        measurement_time_us += 2300 * pow(2, (uint8_t)sampling_tempc - 1);

        if ((uint8_t)sampling_pres != 0)
            measurement_time_us += 2300 * pow(2, (uint8_t)sampling_pres - 1) + 575;
        else
            _pressure_skipped = true;

        if ((uint8_t)sampling_hum != 0)
            measurement_time_us += 2300 * pow(2, (uint8_t)sampling_hum - 1) + 575;
        else
            _humidity_skipped = true;

        _measurement_time_us = measurement_time_us;

        uint8_t value = read8((uint8_t)reg::CTRL_MEAS);
        value &= 0b00000011;
        value |= ((uint8_t)sampling_tempc << 5) | ((uint8_t)sampling_pres << 2);

        //enable sleep mode
        write8((uint8_t)reg::CTRL_MEAS, value & 0b11111100);
        //write setting
        write8((uint8_t)reg::CTRL_HUM, (uint8_t)sampling_hum);
        //write original mode
        write8((uint8_t)reg::CTRL_MEAS, value);

        _sampling_temp = sampling_tempc;
        _sampling_hum = sampling_hum;
        _sampling_pres = sampling_pres;
    };

    void setFilterRate(filter filter)
    {
        //enable sleep mode
        uint8_t value1 = read8((uint8_t)reg::CTRL_MEAS);
        write8((uint8_t)reg::CTRL_MEAS, value1 & 0b11111100);

        //write setting
        uint8_t value2 = read8((uint8_t)reg::CONFIG);
        value2 &= 0b11100011;
        write8((uint8_t)reg::CONFIG, value2 | ((uint8_t)filter << 2));

        //write original mode
        write8((uint8_t)reg::CTRL_MEAS, value1);

        _filter = filter;
    };

    void setStandbyTime(standby standby)
    {
        switch (standby)
        {
        case standby::MS_0_5:
            _standby_time_us = 500;
            break;

        case standby::MS_10:
            _standby_time_us = 10000;
            break;

        case standby::MS_20:
            _standby_time_us = 20000;
            break;

        case standby::MS_62_5:
            _standby_time_us = 62500;
            break;

        case standby::MS_125:
            _standby_time_us = 125000;
            break;

        case standby::MS_250:
            _standby_time_us = 250000;
            break;

        case standby::MS_500:
            _standby_time_us = 500000;
            break;

        case standby::MS_1000:
            _standby_time_us = 1000000;
            break;
        }

        //enable sleep mode
        uint8_t value1 = read8((uint8_t)reg::CTRL_MEAS);
        write8((uint8_t)reg::CTRL_MEAS, value1 & 0b11111100);

        //write setting
        uint8_t value2 = read8((uint8_t)reg::CONFIG);
        value2 &= 0b00011111;
        write8((uint8_t)reg::CONFIG, value2 | ((uint8_t)standby << 5));

        //write original mode
        write8((uint8_t)reg::CTRL_MEAS, value1);

        _standby = standby;
    };

    evn_env readAll(bool blocking = true)
    {
        this->update(blocking);

        evn_env reading;

        reading.humidity = _humidity;
        reading.temperature = readTemperatureC(false);
        reading.temperature_f = readTemperatureF(false);
        reading.pressure = _pressure;
        reading.altitude = readAltitudeMetres(false);
        reading.altitude_ft = readAltitudeFeet(false);

        return reading;
    }

    float readPres(bool blocking = true)
    {
        return readPressure(blocking);
    };

    float readPressure(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _pressure;
        }
        return 0;
    };

    float readHum(bool blocking = true)
    {
        return readHumidity(blocking);
    }

    float readHumidity(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _humidity;
        }
        return 0;
    };

    float readTemp(bool blocking = true)
    {
        return readTemperatureC(blocking);
    };

    float readTemperature(bool blocking = true)
    {
        return readTemperatureC(blocking);
    };

    float readTemperatureC(bool blocking = true)
    {
        if (_sensor_started)
        {
            this->update(blocking);
            return _temperature;
        }
        return 0;
    };

    float readTemperatureF(bool blocking = true)
    {
        return (readTemperatureC(blocking) * 9) / 5 + 32;
    };

    void setTemperatureOffset(float offset)
    {
        _temperature_offset = offset;
    };

    void setSeaLevelPressure(float pressure)
    {
        _sea_level_pressure = max(pressure, 0.001);
    };

    float readAltitude(bool blocking = true)
    {
        return readAltitudeMetres(blocking);
    };

    float readAltitudeMetres(bool blocking = true)
    {
        this->readPressure(blocking);

        return ((float)-44330.77) * (pow(_pressure / _sea_level_pressure, 0.190263) - (float)1);
    };

    float readAltitudeFeet(bool blocking = true)
    {
        return readAltitudeMetres(blocking) * 3.28084;
    };

private:
    bool isMeasuring()
    {
        return read8((uint8_t)reg::STATUS) & (1 << 3);
    };

    bool update(bool blocking)
    {
        if (_sensor_started)
        {
            if (blocking)
                while (micros() - _last_reading_us < _measurement_time_us + _standby_time_us || this->isMeasuring());

            if (micros() - _last_reading_us >= _measurement_time_us + _standby_time_us && !this->isMeasuring())
            {
                _last_reading_us = micros();

                if (_humidity_skipped && _pressure_skipped)
                {
                    readBuffer((uint8_t)reg::TEMP_MSB, 3, _buffer);
                    _buffer[3] = _buffer[0];
                    _buffer[4] = _buffer[1];
                    _buffer[5] = _buffer[2];
                }
                else if (_humidity_skipped)
                    readBuffer((uint8_t)reg::PRES_MSB, 6, _buffer);
                else
                    readBuffer((uint8_t)reg::PRES_MSB, 8, _buffer);

                //TEMPERATURE
                {
                    int32_t adc_T = ((uint32_t)_buffer[3] << 12) | ((uint32_t)_buffer[4] << 4) | ((_buffer[5] >> 4) & 0x0F);
                    int64_t var1, var2;
                    var1 = ((((adc_T >> 3) - ((int32_t)_calibration.dig_T1 << 1))) * ((int32_t)_calibration.dig_T2)) >> 11;
                    var2 = (((((adc_T >> 4) - ((int32_t)_calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)_calibration.dig_T1))) >> 12) *
                        ((int32_t)_calibration.dig_T3)) >> 14;
                    t_fine = var1 + var2;
                    float output = (t_fine * 5 + 128) >> 8;
                    _temperature = output / 100 + _temperature_offset;
                }

                //HUMIDITY
                if (!_humidity_skipped)
                {
                    int32_t adc_H = ((uint32_t)_buffer[6] << 8) | ((uint32_t)_buffer[7]);
                    int32_t var1;
                    var1 = (t_fine - ((int32_t)76800));
                    var1 = (((((adc_H << 14) - (((int32_t)_calibration.dig_H4) << 20) - (((int32_t)_calibration.dig_H5) * var1)) +
                        ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)_calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)_calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                            ((int32_t)_calibration.dig_H2) + 8192) >> 14));
                    var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)_calibration.dig_H1)) >> 4));
                    var1 = (var1 < 0 ? 0 : var1);
                    var1 = (var1 > 419430400 ? 419430400 : var1);
                    _humidity = (float)(var1 >> 12) / 1024.0;
                }

                //PRESSURE
                if (!_pressure_skipped)
                {
                    int32_t adc_P = ((uint32_t)_buffer[0] << 12) | ((uint32_t)_buffer[1] << 4) | ((_buffer[2] >> 4) & 0x0F);
                    int64_t var1, var2, p_acc;
                    var1 = ((int64_t)t_fine) - 128000;
                    var2 = var1 * var1 * (int64_t)_calibration.dig_P6;
                    var2 = var2 + ((var1 * (int64_t)_calibration.dig_P5) << 17);
                    var2 = var2 + (((int64_t)_calibration.dig_P4) << 35);
                    var1 = ((var1 * var1 * (int64_t)_calibration.dig_P3) >> 8) + ((var1 * (int64_t)_calibration.dig_P2) << 12);
                    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_calibration.dig_P1) >> 33;
                    if (var1 == 0)
                        return 0; // avoid exception caused by division by zero
                    p_acc = 1048576 - adc_P;
                    p_acc = (((p_acc << 31) - var2) * 3125) / var1;
                    var1 = (((int64_t)_calibration.dig_P9) * (p_acc >> 13) * (p_acc >> 13)) >> 25;
                    var2 = (((int64_t)_calibration.dig_P8) * p_acc) >> 19;
                    p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)_calibration.dig_P7) << 4);
                    _pressure = (float)p_acc / 256.0;
                }

                return true;
            }
        }
        return false;
    }

    float _pressure = 0;
    float _humidity = 0;
    float _temperature = 0;

    float _temperature_offset;
    float _sea_level_pressure = 100800;

    bool _humidity_skipped = false;
    bool _pressure_skipped = false;

    int32_t t_fine = 0;
    int32_t t_fine_adjust = 0;
    bme280_calibration_data _calibration;

    uint8_t _buffer[8] = { 0 };
    uint32_t _measurement_time_us;
    uint32_t _standby_time_us;
    uint32_t _last_reading_us;

    sampling _sampling_temp;
    sampling _sampling_pres;
    sampling _sampling_hum;
    filter _filter;
    standby _standby;
};

#endif