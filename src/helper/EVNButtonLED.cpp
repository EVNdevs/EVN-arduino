#include "EVNButtonLED.h"

volatile button_led_state_t EVNButtonLED::_button;
volatile uint8_t EVNButtonLED::_btn_lock_num = spin_lock_claim_unused(true),
				EVNButtonLED::_var_lock_num = spin_lock_claim_unused(true),
				EVNButtonLED::_core;
spin_lock_t* EVNButtonLED::_var_lock;

EVNButtonLED::EVNButtonLED(uint8_t mode, bool link_led, bool link_movement, bool button_invert)
{
	_button.state = false;

	_button.mode = constrain(mode, 0, 2);
	_button.link_led = link_led;
	_button.link_movement = link_movement;
	_button.button_invert = button_invert;

	if (_button.mode == BUTTON_TOGGLE && _button.button_invert)
		_button.state = !_button.state;

	_button.flash = false;
}

void EVNButtonLED::begin()
{
	if (!_button.started)
	{
		_button.lock = spin_lock_init(_btn_lock_num);
		_var_lock = spin_lock_init(_var_lock_num);

		pinMode(PIN_BUTTON, INPUT_PULLUP);
		pinMode(PIN_LED, OUTPUT_8MA);
		attachInterrupt(PIN_BUTTON, isr, CHANGE);

		if (rp2040.cpuid() == 0)
			alarm_pool_add_repeating_timer_ms(EVNISRTimer0.sharedAlarmPool(), UPDATE_INTERVAL_MS, update, nullptr, &EVNISRTimer0.sharedISRTimer(2));
		else
			alarm_pool_add_repeating_timer_ms(EVNISRTimer1.sharedAlarmPool(), UPDATE_INTERVAL_MS, update, nullptr, &EVNISRTimer1.sharedISRTimer(2));

		_button.started = true;
		_core = rp2040.cpuid();
	}
}

bool EVNButtonLED::read() 
{
	uint32_t out = spin_lock_blocking(_var_lock);
	bool output = true;
	if (_button.mode != BUTTON_DISABLE && _button.started)
		output = is_spin_locked(_button.lock);
	spin_unlock(_var_lock, out);
	return output;
}

bool EVNButtonLED::motorsEnabled() 
{
	if (rp2040.cpuid() != _core)
		spin_lock_unsafe_blocking(_var_lock);

	bool output = true;
	if (_button.mode != BUTTON_DISABLE && _button.started && _button.link_movement)
		output = is_spin_locked(_button.lock);
	
	if (rp2040.cpuid() != _core)
		spin_unlock_unsafe(_var_lock);

	return output;
}

void EVNButtonLED::setMode(uint8_t mode) { _button.mode = constrain(mode, 0, 2); }
uint8_t EVNButtonLED::getMode() { return _button.mode; }

void EVNButtonLED::setLinkLED(bool enable) { _button.link_led = enable; }
bool EVNButtonLED::getLinkLED() { return _button.link_led; }

void EVNButtonLED::setLinkMovement(bool enable) 
{
	uint32_t out = spin_lock_blocking(_var_lock);
	_button.link_movement = enable;
	spin_unlock(_var_lock, out);
}

bool EVNButtonLED::getLinkMovement()
{
	uint32_t out = spin_lock_blocking(_var_lock);
	bool output = _button.link_movement;
	spin_unlock(_var_lock, out);
	return output;
}

void EVNButtonLED::setButtonInvert(bool enable)
{
	if (_button.mode == BUTTON_TOGGLE && enable != _button.button_invert)
		_button.state = !_button.state;
	_button.button_invert = enable;
}

bool EVNButtonLED::getButtonInvert() { return _button.button_invert; }

void EVNButtonLED::setFlash(bool enable) { _button.flash = enable; }
bool EVNButtonLED::getFlash() { return _button.flash; }

