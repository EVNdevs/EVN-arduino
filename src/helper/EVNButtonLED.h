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
	spin_lock_t* spin_lock;

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
	volatile button_led_state_t* sharedState() volatile { return &button; }

private:
	static volatile button_led_state_t button;
	static void isr()
	{
		if (millis() - button.last_change > DEBOUNCE_TIMING_MS)
		{
			uint8_t reading = digitalRead(PIN_BUTTON);

			if (button.mode == BUTTON_PUSHBUTTON)
			{
				//flip reading for button state, as pin is LOW when button is pressed
				if (button.button_invert)
					button.state = reading;
				else
					button.state = !reading;
			}
			else if (button.mode == BUTTON_TOGGLE)
			{
				//for toggle mode, the raw button reading is stored in substate instead of state
				//when button was not pressed (substate false) and is pressed now (reading false), state is flipped
				if (!button.substate && !reading)
					button.state = !button.state;

				button.substate = !reading;
			}

			//spin lock is used here as an atomic boolean variable
			if (button.state && !is_spin_locked(button.spin_lock))
				spin_lock_unsafe_blocking(button.spin_lock);
			else if (!button.state && is_spin_locked(button.spin_lock))
				spin_unlock_unsafe(button.spin_lock);

			button.last_change = millis();
		}

		//ensures that LED reflects output of read()
		if (button.link_led && !button.flash) digitalWrite(PIN_LED, button.state);
	}

	static bool update(struct repeating_timer* t)
	{
		if (button.flash)
			digitalWrite(PIN_LED, !digitalRead(PIN_LED));
		else
			isr();

		return true;
	}
};

#endif