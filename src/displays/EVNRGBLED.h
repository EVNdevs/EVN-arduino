#ifndef EVNRGBLED_H
#define EVNRGBLED_H

#include <Arduino.h>
#include <pico/stdlib.h>
#include <Servo.h>
#include "ws2812/ws2812.pio.h"

static PIOProgram _rgbLedPgm(&ws2812_program);

class EVNRGBLED
{
public:
    const static uint16_t RESET_TIME_US = 310;
    const static uint8_t MAX_OBJECTS = 4;

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
        _started = false;
        _invert = invert;
    }

    bool begin()
    {
        if (!_started)
        {
            if (!_rgbLedPgm.prepare(&_pio, &_sm, &_offset))
                return _started;

            pio_add_program(_pio, &ws2812_program);
            ws2812_program_init(_pio, _sm, _offset, _pin, 800000, false);
            dma_init();
            this->clearAll();

            _started = true;
        }

        return _started;
    }

    void end()
    {
        if (_started)
        {
            _started = false;
            switch (_pin)
            {
            case PIN_SERVO1:
                ports_started[0] = false;
                break;
            case PIN_SERVO2:
                ports_started[1] = false;
                break;
            case PIN_SERVO3:
                ports_started[2] = false;
                break;
            case PIN_SERVO4:
                ports_started[3] = false;
                break;
            }

            mutex_enter_blocking(&_mutex);

            dma_channel_set_irq0_enabled(_dma, false);
            dma_channel_config channel_config = dma_channel_get_default_config(_dma);
            dma_channel_configure(_dma, &channel_config, NULL, NULL, 0, false);
            dma_channel_unclaim(_dma);

            if (!ports_started[0] && !ports_started[1] && !ports_started[2] && !ports_started[3])
            {
                irq_set_enabled(DMA_IRQ_0, false);
                irq_remove_handler(DMA_IRQ_0, dma_isr);
            }

            pio_sm_set_enabled(_pio, _sm, false);
            pio_sm_unclaim(_pio, _sm);
            _started = false;

            mutex_exit(&_mutex);
        }
    }

    void setInvert(bool enable)
    {
        _invert = enable;
    }

    uint8_t getInvert()
    {
        return _invert;
    }

    void setLEDCount(uint8_t led_count)
    {
        _led_count = led_count;

        if (_started)
        {
            mutex_enter_blocking(&_mutex);
            dma_channel_set_trans_count(_dma, _led_count, false);
            mutex_exit(&_mutex);
        }
    }

    uint8_t getLEDCount()
    {
        return _led_count;
    }

    void writeOne(uint8_t led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        if (_started)
        {
            if (led > _led_count - 1) return;

            uint8_t ledc = led;

            if (_invert)
                ledc = _led_count - 1 - ledc;

            uint32_t new_pixel = urgb_u32(r, g, b);
            if (_buffer[ledc] != new_pixel)
            {
                _buffer[ledc] = new_pixel;
                _buffer_changed = true;
            }
        }

        if (show) this->update();
    }

    void clearOne(uint8_t led, bool show = true)
    {
        writeOne(led, 0, 0, 0, show);
    }

    void writeLine(uint8_t start_led, uint8_t end_led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        if (end_led > _led_count - 1) return;
        if (start_led > _led_count - 1) return;

        for (int i = start_led; i <= end_led; i++)
            writeOne(i, r, g, b, false);

        if (show) this->update();
    }

    void clearLine(uint8_t start_led, uint8_t end_led, bool show = true)
    {
        writeLine(start_led, end_led, 0, 0, 0, show);
    }

    void writeAll(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)
    {
        if (_started)
        {
            for (int i = 0; i < _led_count; i++)
            {
                uint32_t new_pixel = urgb_u32(r, g, b);
                if (_buffer[i] != new_pixel)
                {
                    _buffer[i] = new_pixel;
                    _buffer_changed = true;
                }
            }
        }

        if (show) this->update();
    }

    void clearAll(bool show = true)
    {
        this->writeAll(0, 0, 0, show);
    }

    void update()
    {
        if (_started && _buffer_changed)
        {
            mutex_enter_blocking(&_mutex);
            dma_channel_set_read_addr(_dma, _buffer, true);
        }
    }

private:
    static uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
        return
            ((uint32_t)(r) << 16) |
            ((uint32_t)(g) << 24) |
            ((uint32_t)(b) << 8);
    }

    void dma_init()
    {
        _dma = dma_claim_unused_channel(true);

        dma_channel_config channel_config = dma_channel_get_default_config(_dma);
        channel_config_set_dreq(&channel_config, pio_get_dreq(_pio, _sm, true));
        channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
        channel_config_set_read_increment(&channel_config, true);
        dma_channel_configure(_dma, &channel_config, &_pio->txf[_sm], NULL, _led_count, false);

        if (!ports_started[0] && !ports_started[1] && !ports_started[2] && !ports_started[3])
        {
            mutex_init(&_mutex);
            irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
            dma_channel_set_irq0_enabled(_dma, true);
            irq_set_enabled(DMA_IRQ_0, true);
        }

        switch (_pin)
        {
        case PIN_SERVO1:
            ports_started[0] = true;
            dma_channels[0] = _dma;
            break;
        case PIN_SERVO2:
            ports_started[1] = true;
            dma_channels[1] = _dma;
            break;
        case PIN_SERVO3:
            ports_started[2] = true;
            dma_channels[2] = _dma;
            break;
        case PIN_SERVO4:
            ports_started[3] = true;
            dma_channels[3] = _dma;
            break;
        }
    }

    static void dma_isr()
    {
        for (int i = 0; i < MAX_OBJECTS; i++)
        {
            if (ports_started[i])
                if (dma_channel_get_irq0_status(dma_channels[i]))
                {
                    dma_channel_acknowledge_irq0(dma_channels[i]);

                    if (_alarm_id != 0)
                        cancel_alarm(_alarm_id);

                    _alarm_id = add_alarm_in_us(RESET_TIME_US, release_mutex, NULL, true);
                }
        }
    }

    static int64_t release_mutex(alarm_id_t id, void* user_data)
    {
        _alarm_id = 0;
        mutex_exit(&_mutex);
        return 0;
    }

    static mutex_t _mutex;
    static alarm_id_t _alarm_id;
    static int dma_channels[MAX_OBJECTS];
    static bool ports_started[MAX_OBJECTS];

    uint8_t _led_count;
    uint8_t _pin;

    int _dma;
    PIO _pio;
    int _sm;
    int _offset;

    bool _started;
    bool _invert;
    bool _buffer_changed = true;

    uint32_t _buffer[256] = { };
};

#endif