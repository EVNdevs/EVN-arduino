#include "EVNButtonLED.h"

volatile button_led_state_t EVNButtonLED::button;

EVNButtonLED::EVNButtonLED(uint8_t mode, bool link_led, bool link_movement, bool button_invert)
{
	button.state = false;

	button.mode = constrain(mode, 0, 2);
	button.link_led = link_led;
	button.link_movement = link_movement;
	button.button_invert = button_invert;

	if (button.mode == BUTTON_TOGGLE && button.button_invert)
		button.state = !button.state;

	button.flash = false;
}

void EVNButtonLED::begin()
{
	//use button.started to ensure that setup only occurs once
	if (!button.started)
	{
		//button pin change interrupt
		pinMode(PIN_BUTTON, INPUT_PULLUP);
		attachInterrupt(PIN_BUTTON, isr, CHANGE);

		//led timer interrupt
		pinMode(PIN_LED, OUTPUT_8MA);
		button.spin_lock = spin_lock_init(spin_lock_claim_unused(true));
		if (rp2040.cpuid() == 0)
			alarm_pool_add_repeating_timer_ms(EVNISRTimer0.sharedAlarmPool(), UPDATE_INTERVAL_MS, update, nullptr, &EVNISRTimer0.sharedISRTimer(2));
		else
			alarm_pool_add_repeating_timer_ms(EVNISRTimer1.sharedAlarmPool(), UPDATE_INTERVAL_MS, update, nullptr, &EVNISRTimer1.sharedISRTimer(2));

		button.started = true;
	}
}

bool EVNButtonLED::read() {

	if (button.mode == BUTTON_DISABLE || !button.started)
		return true;
	else
		return is_spin_locked(button.spin_lock);
}

void EVNButtonLED::setMode(uint8_t mode) { button.mode = constrain(mode, 0, 2); }
uint8_t EVNButtonLED::getMode() { return button.mode; }

void EVNButtonLED::setLinkLED(bool enable) { button.link_led = enable; }
bool EVNButtonLED::getLinkLED() { return button.link_led; }

void EVNButtonLED::setLinkMovement(bool enable) { button.link_movement = enable; }
bool EVNButtonLED::getLinkMovement() { return button.link_movement; }

void EVNButtonLED::setButtonInvert(bool enable)
{
	if (button.mode == BUTTON_TOGGLE && enable != button.button_invert)
		button.state = !button.state;
	button.button_invert = enable;
}

bool EVNButtonLED::getButtonInvert() { return button.button_invert; }

void EVNButtonLED::setFlash(bool enable) { button.flash = enable; }
bool EVNButtonLED::getFlash() { return button.flash; }

