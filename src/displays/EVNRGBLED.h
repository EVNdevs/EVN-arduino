#ifndef EVNRGBLED_H
#define EVNRGBLED_H

#include <Arduino.h>
#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <Servo.h>
#include "ws2812/ws2812.pio.h"

static PIOProgram _rgbLedPgm(&ws2812_program);

class EVNRGBLED
{
public:
    EVNRGBLED(uint8_t port, uint8_t led_count = 8, bool invert = false)
    {
        _led_count = led_count;

        uint8_t portc = constrain(port, 1, 4);

        switch (portc)
        {
        case 1:
            _pin = PIN_SERVO1;
            break;
        case 2:
            _pin = PIN_SERVO2;
            break;
        case 3:
            _pin = PIN_SERVO3;
            break;
        case 4:
            _pin = PIN_SERVO4;
            break;
        }

        _pio = nullptr;
        _sm = -1;
        _offset = -1;
        _attached = false;
        _invert = invert;
    };

    bool begin()
    {
        if (!_rgbLedPgm.prepare(&_pio, &_sm, &_offset))
            return _attached;

        pio_add_program(_pio, &ws2812_program);
        ws2812_program_init(_pio, _sm, _offset, _pin, 800000, false);
        this->clearAll();

        _attached = true;
        return _attached;
    };

    void setInvert(bool enable)
    {
        _invert = enable;
    };

    uint8_t getInvert()
    {
        return _invert;
    };

    void setLEDCount(uint8_t led_count)
    {
        _led_count = led_count;
    };

    uint8_t getLEDCount()
    {
        return _led_count;
    };

    void writeOne(uint8_t led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        if (led > _led_count - 1) return;

        uint8_t ledc = led;

        if (_invert)
            ledc = _led_count - 1 - ledc;

        if (_buffer[ledc][0] != r || _buffer[ledc][1] != g || _buffer[ledc][2] != b)
        {
            _buffer[ledc][0] = r;
            _buffer[ledc][1] = g;
            _buffer[ledc][2] = b;
            _buffer_changed = true;
        }

        if (show) this->update();
    };

    void clearOne(uint8_t led, bool show = true)
    {
        writeOne(led, 0, 0, 0, show);
    };

    void writeLine(uint8_t start_led, uint8_t end_led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        if (end_led > _led_count - 1) return;
        if (start_led > _led_count - 1) return;

        for (int i = start_led; i <= end_led; i++)
            writeOne(i, r, g, b, false);

        if (show) this->update();
    };

    void clearLine(uint8_t start_led, uint8_t end_led, bool show = true)
    {
        writeLine(start_led, end_led, 0, 0, 0, show);
    }

    void writeAll(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        for (int i = 0; i < _led_count; i++)
        {
            if (_buffer[i][0] != r || _buffer[i][1] != g || _buffer[i][2] != b)
            {
                _buffer[i][0] = r;
                _buffer[i][1] = g;
                _buffer[i][2] = b;
                _buffer_changed = true;
            }
        }

        if (show) this->update();
    };

    void clearAll(bool show = true)
    {
        this->writeAll(0, 0, 0, show);
    };

    void update()
    {
        if (_buffer_changed)
        {
            //TODO: Maybe let user toggle this? If there's a demand for it perhaps
            delay_for_new_frame();

            for (int i = 0; i < _led_count; i++)
                put_pixel(urgb_u32(_buffer[i][0], _buffer[i][1], _buffer[i][2]));

            _buffer_changed = false;
        }
    };

private:
    static uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
        return
            ((uint32_t)(r) << 8) |
            ((uint32_t)(g) << 16) |
            (uint32_t)(b);
    };

    void put_pixel(uint32_t pixel_grb) {
        pio_sm_put_blocking(_pio, _sm, pixel_grb << 8u);
    };

    void delay_for_new_frame()
    {
        //wait for 8-word fifo to empty itself (240us max)
        while (!pio_sm_is_tx_fifo_empty(_pio, _sm));
        //30us for last word to be transmitted over IO pin + 280us for frame reset
        delayMicroseconds(310);
    }

    PIO _pio;
    int _sm;
    int _offset;
    bool _attached;

    bool _buffer_changed = true;

    uint8_t _led_count;
    uint8_t _pin;
    uint8_t _buffer[256][3] = { 0 };

    bool _invert;
};

#endif