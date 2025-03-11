#ifndef EVNButtonLED_h
#define EVNButtonLED_h

#include <Arduino.h>
#include "EVNISRTimer.h"
#include "../evn_pins_defs.h"

#define BUTTON_DISABLE		0
#define BUTTON_TOGGLE		1
#define BUTTON_PUSHBUTTON	2

typedef struct
{
	bool started;
	bool state;
	bool substate;
	uint32_t last_change;

	uint8_t mode;
	bool link_led;
	bool link_movement;
	bool button_invert;

	bool flash;
	spin_lock_t* lock;
} button_led_state_t;

class EVNButtonLED
{
private:
	static const uint16_t DEBOUNCE_TIMING_MS = 10;
	static const uint16_t UPDATE_INTERVAL_MS = 100;

public:
	EVNButtonLED(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false);
	void begin();
	bool read();
	bool motorsEnabled(); //used by ISRs

	void setMode(uint8_t mode);
	void setLinkLED(bool enable);
	void setLinkMovement(bool enable);
	void setButtonInvert(bool enable);
	void setFlash(bool enable);

	uint8_t getMode();
	bool getLinkLED();
	bool getLinkMovement();
	bool getButtonInvert();
	bool getFlash();

	//singleton for state struct
	volatile button_led_state_t* sharedState() volatile { return &_button; }

private:
	static spin_lock_t* _var_lock;
	static volatile button_led_state_t _button;
	static volatile uint8_t _btn_lock_num, _var_lock_num, _core;
	static void isr()
	{
		if (millis() - _button.last_change > DEBOUNCE_TIMING_MS)
		{
			uint8_t reading = digitalRead(PIN_BUTTON);

			if (_button.mode == BUTTON_PUSHBUTTON)
			{
				//flip reading for _button state, as pin is LOW when _button is pressed
				if (_button.button_invert)
					_button.state = reading;
				else
					_button.state = !reading;
			}
			else if (_button.mode == BUTTON_TOGGLE)
			{
				//for toggle mode, the raw _button reading is stored in substate instead of state
				//when _button was not pressed (substate false) and is pressed now (reading false), state is flipped
				if (!_button.substate && !reading)
					_button.state = !_button.state;

				_button.substate = !reading;
			}

			//spin lock is used here as an atomic boolean variable
			if (_button.state && !is_spin_locked(_button.lock))
				spin_lock_unsafe_blocking(_button.lock);
			else if (!_button.state && is_spin_locked(_button.lock))
				spin_unlock_unsafe(_button.lock);

			_button.last_change = millis();
		}

		//ensures that LED reflects output of read()
		if (_button.link_led && !_button.flash) digitalWrite(PIN_LED, _button.state);
	}

	static bool update(struct repeating_timer* t)
	{
		if (_button.flash)
			digitalWrite(PIN_LED, !digitalRead(PIN_LED));
		else
			isr();

		return true;
	}
};

#endif