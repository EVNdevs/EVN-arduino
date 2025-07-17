#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

#define DIRECT	1
#define REVERSE	0

class PIDController
{
public:
	PIDController(float kp,
		float ki,
		float kd,
		uint8_t dir,
		float deriv_filter_alpha = 1);

	void setKp(float kp) volatile;
	float getKp() volatile;
	void setKi(float ki) volatile;
	float getKi() volatile;
	void setKd(float kd) volatile;
	float getKd() volatile;
	void setDerivativeFilterAlpha(float deriv_filter_alpha) volatile;
	float getDerivativeFilterAlpha() volatile;

	float compute(float error,
		bool constrain_integral = false,
		bool constrain_input = false,
		bool constrain_output = false) volatile;

	void resetIntegral() volatile;
	float getIntegral() volatile;
	void reset() volatile;

private:
	volatile float _kp, _ki, _kd, _dir, _deriv_filter_alpha;
	volatile float _error = 0, _integral = 0, _preverror = 0, _output = 0, _last_filtered_derivative = 0;
	volatile uint32_t _last_update_time_us = 0;
};

#endif