#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

#define DIRECT	1
#define REVERSE	0

class PIDController
{
public:
	PIDController(float kp, float ki, float kd, uint8_t dir);
	void setKp(float kp) volatile;
	float getKp() volatile;
	void setKi(float ki) volatile;
	float getKi() volatile;
	void setKd(float kd) volatile;
	float getKd() volatile;
	float compute(float error, bool constrain_integral = false, bool constrain_input = false, bool constrain_output = false) volatile;
	void constrainIntegral(float low, float high) volatile;
	void resetIntegral() volatile;
	float getIntegral() volatile;
	void reset() volatile;

private:
	volatile float _kp, _ki, _kd, _dir;
	volatile float _error = 0, _summederror = 0, _preverror = 0, _output = 0;
};

#endif