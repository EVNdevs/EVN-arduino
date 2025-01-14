#ifndef EVNPortSelector_h
#define EVNPortSelector_h

#include <Arduino.h>
#include <Wire.h>

#define DEFAULT_I2C_FREQ (uint32_t)400000

class EVNPortSelector
{
public:
	static const uint8_t I2C_ADDR = 0x70;
	EVNPortSelector(uint32_t i2c_freq = DEFAULT_I2C_FREQ);
	void begin();
	void setPort(uint8_t port);
	uint8_t getPort();
	uint8_t getWire0Port();
	uint8_t getWire1Port();
	void printPorts();

private:
	uint8_t _wire0_port, _wire1_port;
	uint32_t _wire0_time_ms, _wire1_time_ms;
	uint32_t _i2c_freq;
	bool _started = false;
};

#endif