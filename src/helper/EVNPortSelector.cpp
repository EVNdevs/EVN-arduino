#include "EVNPortSelector.h"

EVNPortSelector::EVNPortSelector(uint32_t i2c_freq)
{
	_i2c_freq = i2c_freq;
}

void EVNPortSelector::begin()
{
	if (!_started)
	{
		Wire.begin();
		Wire1.begin();

		Wire.setClock(_i2c_freq);
		Wire1.setClock(_i2c_freq);

		delay(50);

		Wire.beginTransmission(I2C_ADDR);
		Wire.write(0);
		Wire.endTransmission();
		
		Wire1.beginTransmission(I2C_ADDR);
		Wire1.write(0);
		Wire1.endTransmission();
		
		delay(50);

		Wire.beginTransmission(I2C_ADDR);
		Wire.write(1);
		Wire.endTransmission();
		_wire0_port = 1;
		_wire0_time_ms = millis();

		Wire1.beginTransmission(I2C_ADDR);
		Wire1.write(1);
		Wire1.endTransmission();
		_wire1_port = 9;
		_wire1_time_ms = millis();

		_started = true;
	}
}

void EVNPortSelector::printPorts()
{
	for (uint8_t t = 1; t < 9; t++)
	{
		this->setPort(t);
		bool line_started = false;

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == I2C_ADDR)
				continue;
			Wire.beginTransmission(addr);
			if (!Wire.endTransmission())
			{
				if (!line_started)
				{
					Serial.print("Port ");
					Serial.print(t);
					Serial.print(": ");
					line_started = true;
				}
				Serial.print("0x");
				Serial.print(addr, HEX);
				Serial.print(" ");
			}
		}
		if (line_started) Serial.println();
	}

	for (uint8_t t = 9; t < 17; t++)
	{
		this->setPort(t);
		bool line_started = false;

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == I2C_ADDR)
				continue;
			Wire1.beginTransmission(addr);
			if (!Wire1.endTransmission())
			{
				if (!line_started)
				{
					Serial.print("Port ");
					Serial.print(t);
					Serial.print(": ");
					line_started = true;
				}
				Serial.print("0x");
				Serial.print(addr, HEX);
				Serial.print(" ");
			}
		}
		if (line_started) Serial.println();
	}
}

void EVNPortSelector::setPort(uint8_t port)
{
	uint8_t portc = constrain(port, 1, 16);

	if (portc <= 8 && _wire0_port != portc)
	{
		Wire.beginTransmission(I2C_ADDR);
		Wire.write(1 << (portc - 1));
		Wire.endTransmission();
		_wire0_port = portc;
		_wire0_time_ms = millis();
	}
	else if (portc > 8 && _wire1_port != portc)
	{
		Wire1.beginTransmission(I2C_ADDR);
		Wire1.write(1 << (portc - 9));
		Wire1.endTransmission();
		_wire1_port = portc;
		_wire1_time_ms = millis();
	}
}

uint8_t EVNPortSelector::getPort()
{
	return (_wire0_time_ms >= _wire1_time_ms) ? _wire0_port : _wire1_port;
}

uint8_t EVNPortSelector::getWire0Port()
{
	return _wire0_port;
}

uint8_t EVNPortSelector::getWire1Port()
{
	return _wire1_port;
}