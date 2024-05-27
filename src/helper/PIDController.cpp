#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, uint8_t dir)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_dir = dir;
}

void PIDController::setKp(float kp) { _kp = kp; }
float PIDController::getKp() { return _kp; }
void PIDController::setKi(float ki) { _ki = ki; }
float PIDController::getKi() { return _ki; }
void PIDController::setKd(float kd) { _kd = kd; }
float PIDController::getKd() { return _kd; }

float PIDController::compute(float error, bool constrain_integral, bool constrain_input, bool constrain_output)
{
	float errorc = error;
	if (constrain_input)
		errorc = constrain(errorc, -1, 1);

	//calculate output value
	_preverror = _error;
	_error = errorc;
	_summederror += errorc;

	if (constrain_integral)
		_summederror = constrain(_summederror, -1 / _ki, 1 / _ki);

	_output = _kp * _error + _kd * (_error - _preverror) + _ki * _summederror;

	if (constrain_output)
		_output = constrain(_output, -1, 1);

	//flip value if needed
	if (_dir == REVERSE)
		_output *= -1;

	return _output;
}

void PIDController::reset()
{
	_error = 0;
	_preverror = 0;
	_summederror = 0;
}

float PIDController::getIntegral()
{
	return _summederror;
}

void PIDController::constrainIntegral(float low, float high)
{
	_summederror = constrain(_summederror, low, high);
}

void PIDController::resetIntegral()
{
	_summederror = 0;
}