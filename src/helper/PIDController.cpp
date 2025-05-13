#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, uint8_t dir, float deriv_filter_alpha)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_dir = dir;
	_deriv_filter_alpha = deriv_filter_alpha;
}

void PIDController::setKp(float kp) volatile { _kp = kp; }
float PIDController::getKp() volatile { return _kp; }
void PIDController::setKi(float ki) volatile { _ki = ki; }
float PIDController::getKi() volatile { return _ki; }
void PIDController::setKd(float kd) volatile { _kd = kd; }
float PIDController::getKd() volatile { return _kd; }
void PIDController::setDerivativeFilterAlpha(float deriv_filter_alpha) volatile { _deriv_filter_alpha = constrain(deriv_filter_alpha, 0, 1); }
float PIDController::getDerivativeFilterAlpha() volatile { return _deriv_filter_alpha; }

float PIDController::compute(float error, bool constrain_integral, bool constrain_input, bool constrain_output) volatile
{
	uint32_t now = micros();
	uint32_t time = now - _last_update_time_us;
	_last_update_time_us = now;

	if (constrain_input)
		error = constrain(error, -1, 1);

	_error = error;
	_integral += _ki * error * time;

	float error_delta = (_error - _preverror);
	float derivative =  (_kd * error_delta * 1000000.0) / (float) time;
	float filtered_derivative = _deriv_filter_alpha * derivative + (1 - _deriv_filter_alpha) * _last_filtered_derivative;

	if (constrain_integral)
		_integral = constrain(_integral, -1, 1);

	_output = _kp * _error + filtered_derivative + _integral;
	_last_filtered_derivative = filtered_derivative;
	_preverror = _error;

	if (constrain_output)
		_output = constrain(_output, -1, 1);

	return _output * (_dir == REVERSE ? -1 : 1);
}

void PIDController::reset() volatile
{
	_error = 0;
	_preverror = 0;
	_integral = 0;
	_last_filtered_derivative = 0;
	_last_update_time_us = micros();
}

float PIDController::getIntegral() volatile { return _integral; }
void PIDController::resetIntegral() volatile { _integral = 0; }