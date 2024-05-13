#include "EVNPortSelector.h"

EVNPortSelector::EVNPortSelector(uint32_t i2c_freq)
{
	_i2c_freq = i2c_freq;
}

void EVNPortSelector::begin()
{
	Wire.begin();
	Wire1.begin();
	Serial.begin();

	Wire.setClock(_i2c_freq);
	Wire1.setClock(_i2c_freq);

	Wire1.beginTransmission(I2C_ADDR);
	Wire1.write(1 << 0);
	Wire1.endTransmission();
	_wire1_port = 9;
	_wire1_time_ms = millis();

	Wire.beginTransmission(I2C_ADDR);
	Wire.write(1 << 0);
	Wire.endTransmission();
	_wire0_port = 1;
	_wire0_time_ms = millis();
}

void EVNPortSelector::printPorts()
{
	for (uint8_t t = 1; t < 9; t++)
	{
		this->setPort(t);
		Serial.print("---Port ");
		Serial.print(t);
		Serial.println("---");

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == I2C_ADDR)
				continue;
			Wire.beginTransmission(addr);
			if (!Wire.endTransmission())
			{
				Serial.print("Address 0x");
				Serial.print(addr, HEX);
				Serial.println(" found");
			}
		}
	}

	for (uint8_t t = 9; t < 17; t++)
	{
		this->setPort(t);
		Serial.print("---Port ");
		Serial.print(t);
		Serial.println("---");

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == I2C_ADDR)
				continue;
			Wire1.beginTransmission(addr);
			if (!Wire1.endTransmission())
			{
				Serial.print("Address 0x");
				Serial.print(addr, HEX);
				Serial.println(" found");
			}
		}
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