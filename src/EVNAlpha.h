#ifndef EVNAlpha_h
#define EVNAlpha_h

#include "helper/EVNButtonLED.h"
#include "helper/EVNPortSelector.h"
#include <Arduino.h>

class EVNAlpha {
private:
    enum bq25887 : uint8_t
    {
        I2C_ADDR = 0x6A,
        I2C_PORT = 16,
        ID = 0x05,
        REG_CHG_CONTROL1 = 0x05,
        REG_ADC_CONTROL = 0x15,
        REG_VBAT_ADC1 = 0x1D,
        REG_VCELLTOP_ADC1 = 0x1F,
        REG_PART_INFO = 0x25,
        REG_VCELLBOT_ADC1 = 0x26,
        CMD_ADC_CONTROL_ENABLE = 0b10110000,
        CMD_WATCHDOG_DISABLE = 0b10001101,
        MASK_PART_INFO = 0b01111000,
    };

public:
    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false, uint32_t i2c_freq = DEFAULT_I2C_FREQ);
    void begin();

    //Button & LED Functions
    bool buttonRead() { return button_led.read(); };
    void ledWrite(bool state) { if (!button_led.getFlash()) digitalWrite(PIN_LED, state); };

    void setMode(uint8_t mode) { button_led.setMode(mode); };
    void setLinkLED(bool enable) { button_led.setLinkLED(enable); };
    void setLinkMovement(bool enable) { button_led.setLinkMovement(enable); };
    void setButtonInvert(bool enable) { button_led.setButtonInvert(enable); };

    uint8_t getMode() { return button_led.getMode(); };
    bool getLinkLED() { return button_led.getLinkLED(); };
    bool getLinkMovement() { return button_led.getLinkMovement(); };
    bool getButtonInvert() { return button_led.getButtonInvert(); };

    // not exposed to the user, since flash is used for battery alerts
    // void setFlash(bool enable) { button_led.setFlash(enable); };
    // bool getFlash() { return button_led.getFlash(); };

    //I2C Multiplexer Functions
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    uint8_t getWirePort() { return ports.getWire0Port(); }
    uint8_t getWire1Port() { return ports.getWire1Port(); }

    int16_t getBatteryVoltage(bool flash_when_low = true, uint16_t low_threshold_mv = 6900);
    int16_t getCell1Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450);
    int16_t getCell2Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450);

    void printPorts();

    //Singletons for Port Selector and Button/LED
    static EVNPortSelector& sharedPorts() { static EVNAlpha shared; return shared.ports; }
    static EVNButtonLED& sharedButtonLED() { static EVNAlpha shared; return shared.button_led; }

private:
    bool beginADC();
    uint16_t readADC16(uint8_t reg);
    void updateBatteryVoltage();
    void updateCell1Voltage();
    void updateCell2Voltage();

    bool _battery_adc_started;

    int16_t _vbatt = 0, _vcell1 = 0, _vcell2 = 0;

    static EVNButtonLED button_led;
    static EVNPortSelector ports;
    static uint8_t _mode;
    static bool _link_led, _link_movement, _button_invert;
    static uint32_t _i2c_freq;
};

#endif