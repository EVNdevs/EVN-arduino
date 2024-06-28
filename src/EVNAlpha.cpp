#include "EVNAlpha.h"

uint8_t EVNAlpha::_mode;
bool EVNAlpha::_link_led;
bool EVNAlpha::_link_movement;
bool EVNAlpha::_button_invert;
EVNButtonLED EVNAlpha::button_led(_mode, _link_led, _link_movement, _button_invert);

uint32_t EVNAlpha::_i2c_freq;
EVNPortSelector EVNAlpha::ports(_i2c_freq);

EVNAlpha::EVNAlpha(uint8_t mode, bool link_led, bool link_movement, bool button_invert, uint32_t i2c_freq)
{
    _battery_adc_started = false;
    _mode = constrain(mode, 0, 2);
    _link_led = link_led;
    _link_movement = link_movement;
    _button_invert = button_invert;
    _i2c_freq = i2c_freq;
}

void EVNAlpha::begin()
{
    //set correct I2C and Serial pins
    //TODO: to be removed after getting our board in arduino-pico core
    Wire.setSDA(WIRE0_SDA);
    Wire.setSCL(WIRE0_SCL);
    Wire1.setSDA(WIRE1_SDA);
    Wire1.setSCL(WIRE1_SCL);

    Serial1.setRX(SERIAL1_RX);
    Serial1.setTX(SERIAL1_TX);
    Serial2.setRX(SERIAL2_RX);
    Serial2.setTX(SERIAL2_TX);

    //initialize helper objects
    ports.begin();
    button_led.begin();

    //initialize battery ADC if available
    if (this->beginADC()) _battery_adc_started = true;

    ports.setPort(1);
}

void EVNAlpha::printPorts()
{
    uint16_t vbatt = getBatteryVoltage(false);
    uint16_t vcell1 = getCell1Voltage(false);
    uint16_t vcell2 = vbatt - vcell1;
    Serial.println("EVN Alpha I2C Port Scanner");
    Serial.print("Battery: ");
    Serial.print((float)vbatt / 1000, 3);
    Serial.print("V | Cell 1: ");
    Serial.print((float)vcell1 / 1000, 3);
    Serial.print("V | Cell 2: ");
    Serial.println((float)vcell2 / 1000, 3);
    ports.printPorts();

}

bool EVNAlpha::beginADC()
{
    ports.setPort((uint8_t)bq25887::I2C_PORT);

    //check for BQ25887 ID
    //NOTE: BQ25887 not in use for our pre-V1.3 users, so this is important
    uint8_t id = 0;
    Wire1.beginTransmission((uint8_t)bq25887::I2C_ADDR);
    Wire1.write((uint8_t)bq25887::REG_PART_INFO);
    Wire1.endTransmission();
    Wire1.requestFrom((uint8_t)bq25887::I2C_ADDR, (uint8_t)1);
    id = Wire1.read();

    id = (id & (uint8_t)bq25887::MASK_PART_INFO) >> 3;

    if (id != (uint8_t)bq25887::ID)
        return false;

    //enable ADCs on BQ25887
    Wire1.beginTransmission((uint8_t)bq25887::I2C_ADDR);
    Wire1.write((uint8_t)bq25887::REG_ADC_CONTROL);
    Wire1.write((uint8_t)bq25887::CMD_ADC_CONTROL_ENABLE);
    Wire1.endTransmission();

    //disable watchdog, or ADC resets after 40s
    Wire1.beginTransmission((uint8_t)bq25887::I2C_ADDR);
    Wire1.write((uint8_t)bq25887::REG_CHG_CONTROL1);
    Wire1.write((uint8_t)bq25887::CMD_WATCHDOG_DISABLE);
    Wire1.endTransmission();

    return true;
}

int16_t EVNAlpha::getBatteryVoltage(bool flash_when_low, uint16_t low_threshold_mv)
{
    if (_battery_adc_started)
    {
        updateBatteryVoltage();

        if (flash_when_low)
            button_led.setFlash(_vbatt < low_threshold_mv);

        return _vbatt;
    }
    return 0;
}

int16_t EVNAlpha::getCell1Voltage(bool flash_when_low, uint16_t low_threshold_mv)
{
    if (_battery_adc_started)
    {
        updateCell1Voltage();

        if (flash_when_low)
            button_led.setFlash(_vcell1 < low_threshold_mv);

        return _vcell1;
    }
    return 0;
}

int16_t EVNAlpha::getCell2Voltage(bool flash_when_low, uint16_t low_threshold_mv)
{
    if (_battery_adc_started)
    {
        updateCell2Voltage();

        if (flash_when_low)
            button_led.setFlash(_vcell2 < low_threshold_mv);

        return _vcell2;
    }
    return 0;
}

void EVNAlpha::updateBatteryVoltage() { if (_battery_adc_started)_vbatt = readADC16((uint8_t)bq25887::REG_VBAT_ADC1); }
void EVNAlpha::updateCell1Voltage() { if (_battery_adc_started) _vcell1 = readADC16((uint8_t)bq25887::REG_VCELLTOP_ADC1); }
void EVNAlpha::updateCell2Voltage() { if (_battery_adc_started) _vcell2 = readADC16((uint8_t)bq25887::REG_VCELLBOT_ADC1); }

uint16_t EVNAlpha::readADC16(uint8_t reg)
{
    ports.setPort((uint8_t)bq25887::I2C_PORT);

    Wire1.beginTransmission((uint8_t)bq25887::I2C_ADDR);
    Wire1.write(reg);
    Wire1.endTransmission();
    Wire1.requestFrom((uint8_t)bq25887::I2C_ADDR, (uint8_t)2);

    return Wire1.read() << 8 | Wire1.read();
}