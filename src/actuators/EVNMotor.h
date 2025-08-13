#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include "../EVNAlpha.h"
#include "../helper/EVNISRTimer.h"
#include "../helper/PIDController.h"
#include "../evn_motor_defs.h"
#include "../evn_pins_defs.h"
#include "../helper/EVNCoreSync.h"

//INPUT PARAMETER MACROS
#define DIRECT				1
#define REVERSE				0

#define EV3_LARGE			0
#define NXT_LARGE			1
#define EV3_MED				2
#define CUSTOM_MOTOR		3

#define MAX_STOP_VALUE		3
#define STOP_BRAKE			0
#define STOP_COAST			1
#define STOP_HOLD			2
#define STOP_NONE			3

#define DEBUG_OFF			0
#define DEBUG_SPEED			1
#define DEBUG_TURN_RATE		2

//DPS MEASUREMENT (TIME BETWEEN PULSES)
#define NO_OF_EDGES_STORED 3

class EVNMotor;

typedef struct
{
	//POSITION MEASUREMENT
	uint8_t enca;
	uint8_t encb;
	int32_t count;
	bool enca_state;
	bool encb_state;
	int8_t dir;
	uint8_t state;
	float ppr;
	bool pos_calculated;
	float position;
	float position_offset;

	//DPS (VELOCITY) MEASUREMENT (TIME BETWEEN PULSES)
	uint8_t last_edge_index;
	uint32_t edge_times[NO_OF_EDGES_STORED];
	bool dps_calculated;
	bool obtained_one_pulse;
	float avg_dps;
	float avg_pulse_width;

	//DERIVED VALUES
	float _90_div_ppr;
	float _360000000_div_ppr;
} encoder_state_t;

typedef struct
{
	//MOTOR CHARACTERISTICS
	float pwm_mag;
	float pwm_exp;
	uint8_t port;
	uint8_t motor_type;
	uint8_t motora;
	uint8_t motorb;
	float unloaded_max_rpm;
	float loaded_max_rpm;
	float max_rpm;
	bool max_rpm_calculated;
	float accel;
	float decel;

	//CONTROLLER
	PIDController* pos_pid;

	//USER-SET
	bool run_pwm;
	bool run_speed;
	bool run_dir;
	float target_dps;
	bool run_pos;
	float end_pos;
	bool run_time;
	uint32_t run_time_us;
	uint8_t stop_action;
	bool debug;

	//LOOP
	uint32_t last_update;
	float time_since_last_loop;
	float target_dps_end_decel;
	float target_dps_constrained;
	float target_pos;
	float error;
	float output;
	uint32_t start_time_us;
	bool stalled;
	bool hold_done;
	uint32_t hold_start_time_us;
	uint32_t last_stopped_time_us;
	bool hit_end;
	uint32_t hit_end_time_us;
	bool accel_decel_off;

	//DERIVED VALUES
	float max_error_before_decel; 	//only used by runPosition
	float time_to_decel_us;			//only used by runTime
	float decel_dps_per_us;			//only used by runTime
} pid_control_t;

typedef struct
{
	//DRIVEBASE CHARACTERISTICS
	uint8_t motor_type;
	float wheel_dia;
	float axle_track;
	EVNMotor* motor_left;
	EVNMotor* motor_right;
	uint8_t id;

	float speed_accel;
	float speed_decel;
	float turn_rate_accel;
	float turn_rate_decel;

	//CONTROLLERS
	PIDController* turn_rate_pid;
	PIDController* speed_pid;

	//USER-SET
	float target_speed;
	float target_turn_rate;
	float target_speed_constrained;
	float target_turn_rate_constrained;
	bool drive;
	bool drive_position;
	float end_angle;
	float end_distance;
	uint8_t stop_action;
	uint8_t debug;

	//LOOP
	uint32_t last_update;
	float time_since_last_loop;
	float position_x;
	float position_y;
	float current_angle;
	float current_distance;
	float target_angle;
	float target_distance;
	float prev_distance;
	float angle_error;
	float angle_output;
	float speed_error;
	float speed_output;
	float target_motor_left_duty_cycle;
	float target_motor_right_duty_cycle;
	float target_turn_rate_after_decel;
	float target_speed_after_decel;
	bool hold_done;
	uint32_t hold_start_time_us;
	uint32_t last_stopped_time_us;
	bool hit_end;
	uint32_t hit_end_time_us;
	bool accel_decel_off;

	//DERIVED VARIABLES
	float wheel_dia_mul_pi_div_720;
	float wheel_dia_mul_pi_div_360;
	float wheel_dia_mul_pi_div_360_div_axle_track;
	float _360_div_pi_div_wheel_dia;
	float axle_track_div_wheel_dia;
	float wheel_dia_div_axle_track;

	float max_rpm;
	float max_dps;
	float max_turn_rate;
	float max_speed;

	float time_to_decel_speed;			//only used by straight/curve
	float time_to_decel_turn_rate;		//only used by straight/curve
	float slowed_turn_rate_decel;		//only used by straight/curve
	float slowed_speed_decel;			//only used by straight/curve
	float target_speed_sq_div_2;		//only used by straight/curve
	float target_turn_rate_sq_div_2;	//only used by straight/curve
} drivebase_state_t;

class EVNMotor
{
public:
	static const uint32_t PWM_FREQ = 20000;
	static const uint32_t PWM_MAX_VAL = 255;
	static const uint16_t PID_TIMER_INTERVAL_US = 1000;
	static const uint32_t ENCODER_PULSE_TIMEOUT_US = 166667 * 2;
	static const uint8_t MAX_MOTOR_OBJECTS = 4;

	friend class EVNDrivebase;
	friend class EVNOmniDrivebaseBasic;

	EVNMotor(uint8_t port, uint8_t motor_type = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin() volatile;
	void setKp(float kp) volatile;
	void setKd(float kd) volatile;
	void setPWMMapping(float mag, float exp) volatile;
	void setAccel(float accel_dps_per_s) volatile;
	void setDecel(float decel_dps_per_s) volatile;
	void setMaxRPM(float unloaded_max_rpm, float loaded_max_rpm = -1) volatile;
	void setPPR(float ppr) volatile;
	void setDebug(bool enable) volatile;

	float getKp() volatile;
	float getKd() volatile;
	float getPWMMag() volatile;
	float getPWMExp() volatile;
	float getAccel() volatile;
	float getDecel() volatile;
	float getMaxRPM() volatile;
	float getMaxDPS() volatile;
	float getPPR() volatile;
	bool getDebug() volatile;

	float getError() volatile;
	float getPosition() volatile;
	float getHeading() volatile;
	void setPosition(float position) volatile;
	void resetPosition() volatile;
	float getSpeed() volatile;

	void runPWM(float duty_cycle_pct) volatile;
	void runSpeed(float dps, bool disable_accel_decel = false) volatile;
	void runPosition(float dps, float position, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runAngle(float dps, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runHeading(float dps, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void runTime(float dps, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;

	void stop() volatile;
	void coast() volatile;
	void hold() volatile;

	bool completed() volatile;
	bool stalled() volatile;

private:
	float get_target_position() volatile;
	float get_target_heading() volatile;

	void disable_connected_drivebase_unsafe() volatile;
	void runPWM_unsafe(float duty_cycle) volatile;
	void runSpeed_unsafe(float dps, bool disable_accel_decel) volatile;
	void stop_unsafe() volatile;
	void coast_unsafe() volatile;
	void hold_unsafe() volatile;
	bool stalled_unsafe() volatile;

	float clean_input_dps_unsafe(float dps) volatile;
	uint8_t clean_input_dir(float dps) volatile;
	uint8_t clean_input_stop_action(uint8_t stop_action) volatile;

	void compute_ppr_derived_values_unsafe() volatile;
	void compute_max_rpm_unsafe() volatile;

	volatile pid_control_t _pid_control = {};
	volatile encoder_state_t _encoder = {};

	static volatile encoder_state_t* encoderArgs[MAX_MOTOR_OBJECTS];
	static volatile pid_control_t* pidArgs[MAX_MOTOR_OBJECTS];
	static volatile bool ports_enabled[MAX_MOTOR_OBJECTS];
	static volatile bool timerisr_enabled;
	static volatile bool odom_enabled[MAX_MOTOR_OBJECTS];

	static bool loop_control_enabled(volatile pid_control_t* arg) { return (arg->run_speed || arg->run_time || arg->run_pos); }

	static void velocity_update(volatile encoder_state_t* arg, uint32_t now)
	{
		if (arg->enca_state)
		{
			arg->last_edge_index++;
			arg->last_edge_index %= NO_OF_EDGES_STORED;
			arg->edge_times[arg->last_edge_index] = now;

			if (!arg->obtained_one_pulse && arg->last_edge_index == 2)
				arg->obtained_one_pulse = true;

			arg->dps_calculated = false;
		}
	}

	static void pos_read(volatile encoder_state_t* arg)
	{
		arg->enca_state = digitalRead(arg->enca);
		arg->encb_state = digitalRead(arg->encb);
	}

	static void pos_update(volatile encoder_state_t* arg)
	{
		arg->pos_calculated = false;
		uint8_t state = arg->state & 3;

		if (arg->enca_state)
			state |= 4;
		if (arg->encb_state)
			state |= 8;
		arg->state = (state >> 2);

		switch (state)
		{
		case 1: case 7: case 8: case 14:
			arg->count++;
			arg->dir = 1;
			return;
		case 2: case 4: case 11: case 13:
			arg->count--;
			arg->dir = -1;
			return;
		case 3: case 12: case 6: case 9:
			// since timer interrupts are lower priority than pin change (configured in EVNISRTimer)
			// this case should rarely (if not NEVER) happen
			arg->count += 2 * arg->dir;
			return;
		}
	}

	static void runPWM_static(volatile pid_control_t* pidArg, float speed)
	{
		float speedc = constrain(speed, -1, 1);

		if (EVNAlpha::motorsEnabled()) {
			if (speedc > 0)
			{
				digitalWrite(pidArg->motorb, LOW);
				analogWrite(pidArg->motora, speedc * PWM_MAX_VAL);
			}
			else
			{
				digitalWrite(pidArg->motora, LOW);
				analogWrite(pidArg->motorb, -speedc * PWM_MAX_VAL);
			}
		}
	}

	static void stopAction_static(volatile pid_control_t* pidArg, volatile encoder_state_t* encoderArg, float pos, float dps)
	{
		bool no_loop_control_enabled = !loop_control_enabled(pidArg);

		//keep at most recent state
		pidArg->target_dps_constrained = dps;

		//reset PID controller, stop loop control
		pidArg->run_pwm = false;
		pidArg->run_speed = false;
		pidArg->run_pos = false;
		pidArg->run_time = false;

		//coast and brake stop functions based on DRV8833 datasheet
		//hold uses position PID control, setting the current position as an endpoint
		switch (pidArg->stop_action)
		{
		case STOP_BRAKE:
			digitalWrite(pidArg->motora, HIGH);
			digitalWrite(pidArg->motorb, HIGH);
			pidArg->target_pos = pos;
			pidArg->hold_done = true;
			break;
			
		case STOP_COAST:
			digitalWrite(pidArg->motora, LOW);
			digitalWrite(pidArg->motorb, LOW);
			pidArg->target_pos = pos;
			pidArg->hold_done = true;
			break;
			
		case STOP_HOLD:
			if (no_loop_control_enabled)
			{
				pidArg->end_pos = pos;
				pidArg->target_dps = pidArg->max_rpm * 6;
			}
			else
				pidArg->end_pos = pidArg->target_pos;

			pidArg->run_pos = true;
			pidArg->hold_done = false;
			break;

		case STOP_NONE:
			pidArg->target_dps = pidArg->target_dps_constrained;
			pidArg->run_speed = true;
			pidArg->hold_done = true;
			break;
		}
	}

	static float getPosition_static(volatile encoder_state_t* arg)
	{
		// resolution of encoder readout is in CPR (4 * PPR)
		if (!arg->pos_calculated)
		{
			arg->position = ((float)arg->count * arg->_90_div_ppr) - arg->position_offset;
			arg->pos_calculated = true;
		}

		return arg->position;
	}

	static float getDPS_static(volatile encoder_state_t* arg)
	{
		int64_t last_pulse_width = micros() - arg->edge_times[arg->last_edge_index];
		int64_t last_full_pulse_width = arg->edge_times[arg->last_edge_index] - arg->edge_times[(arg->last_edge_index - 1 + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED];

		if (!arg->obtained_one_pulse) return 0;

		if (!arg->dps_calculated)
		{
			float sum_dps = 0;
			uint8_t number_of_pulses = 0;

			for (uint8_t i = 0; i < NO_OF_EDGES_STORED - 1; i++)
			{
				uint8_t end_index = (arg->last_edge_index - i + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint8_t start_index = (end_index - 1 + NO_OF_EDGES_STORED) % NO_OF_EDGES_STORED;
				uint32_t end = arg->edge_times[end_index];
				uint32_t start = arg->edge_times[start_index];

				int64_t pulse_width = end - start;
				float pulse_dps = 0;
				if (pulse_width > 0)
				{
					pulse_dps = arg->_360000000_div_ppr / pulse_width;
					number_of_pulses++;
				}

				sum_dps += pulse_dps;
			}

			arg->avg_dps = sum_dps / number_of_pulses;
			arg->dps_calculated = true;
		}

		// if timeout, DPS is 0
		if (last_pulse_width > ENCODER_PULSE_TIMEOUT_US) return 0;

		// if latest pulse is longer than pulse width of last 2 edges, use latest pulse
		// this occurs when the motor is slowing down (pulses get longer and longer)
		else if (last_pulse_width > last_full_pulse_width)
		{
			float final_dps = arg->_360000000_div_ppr / last_pulse_width;
			return final_dps * (float)arg->dir;
		}
		else
			return arg->avg_dps * (float)arg->dir;
	}

	static float motorStopped_static(volatile encoder_state_t* arg)
	{
		return getDPS_static(arg) <= MOTOR_HOLD_THRESHOLD_DPS;
	}

	static void pid_update(volatile pid_control_t* pidArg, volatile encoder_state_t* encoderArg)
	{
		uint32_t now = micros();
		pidArg->time_since_last_loop = ((float)(now - pidArg->last_update)) * 0.000001;	//seconds
		pidArg->last_update = now;

		float pos = getPosition_static(encoderArg);
		float dps = getDPS_static(encoderArg);

		if (pidArg->time_since_last_loop < 0)
			return;

		if (!pidArg->hold_done)
		{
			if (pidArg->end_pos == pidArg->target_pos)
			{
				if (!motorStopped_static(encoderArg))
				{
					if (fabs(pidArg->error) <= MOTOR_HOLD_MIN_ERROR_MOTOR_DEG)
					{
						pidArg->last_stopped_time_us = pidArg->last_update;
					}
				}
				else
				{
					if (pidArg->last_update - pidArg->hold_start_time_us  >= MOTOR_HOLD_TIMEOUT_US
					|| pidArg->last_update - pidArg->last_stopped_time_us >= MOTOR_HOLD_MIN_TIME_US)
					{
						pidArg->target_dps_constrained = 0;
						pidArg->target_dps = 0;
						pidArg->run_speed = true;
						pidArg->run_pos = false;
						pidArg->hold_done = true;
					}
				}
			}
			else
			{
				pidArg->last_stopped_time_us = pidArg->last_update;
			}
		}
		else
		{
			pidArg->hold_start_time_us = pidArg->last_update;
			pidArg->last_stopped_time_us = pidArg->last_update;
		}

		if (EVNAlpha::motorsEnabled() && loop_control_enabled(pidArg))
		{
			float decel_dps = pidArg->target_dps;

			if (pidArg->run_pos)
			{
				pidArg->run_dir = (pidArg->end_pos - pidArg->target_pos) > 0 ? DIRECT : REVERSE;

				if (pidArg->stop_action != STOP_NONE)
				{
					float error = fabs(pidArg->end_pos - pidArg->target_pos);
					if (error < pidArg->max_error_before_decel)
						decel_dps = fabs(pidArg->target_dps) * sqrt(error / pidArg->max_error_before_decel);
				}
			}

			if (pidArg->run_time)
			{
				if (pidArg->last_update - pidArg->start_time_us >= pidArg->run_time_us)
				{
					stopAction_static(pidArg, encoderArg, pos, dps);
					return;
				}
				
				if (pidArg->stop_action != STOP_NONE)
				{
					uint32_t elapsed_run_time_us = pidArg->last_update - pidArg->start_time_us;
					if (pidArg->time_to_decel_us > pidArg->run_time_us - elapsed_run_time_us)
						decel_dps = pidArg->decel_dps_per_us * (float)(pidArg->run_time_us - elapsed_run_time_us);
				}
			}

			pidArg->target_dps_end_decel = decel_dps;
			float signed_target_dps_end_decel = pidArg->target_dps_end_decel * (pidArg->run_dir == DIRECT ? 1 : -1);

			if (fabs(pidArg->error * pidArg->pos_pid->getKp()) < 1) //anti-windup
			{
				pidArg->stalled = false;

				if (!pidArg->accel_decel_off)
				{
					if (pidArg->target_dps_constrained < signed_target_dps_end_decel)
					{
						if (pidArg->target_dps_constrained > 0)
							pidArg->target_dps_constrained += pidArg->time_since_last_loop * pidArg->accel;
						else
							pidArg->target_dps_constrained += pidArg->time_since_last_loop * pidArg->decel;

						if (pidArg->target_dps_constrained > signed_target_dps_end_decel)
							pidArg->target_dps_constrained = signed_target_dps_end_decel;
					}
					else
					{
						if (pidArg->target_dps_constrained > 0)
							pidArg->target_dps_constrained -= pidArg->time_since_last_loop * pidArg->decel;
						else
							pidArg->target_dps_constrained -= pidArg->time_since_last_loop * pidArg->accel;

						if (pidArg->target_dps_constrained < signed_target_dps_end_decel)
							pidArg->target_dps_constrained = signed_target_dps_end_decel;
					}
				}
				else
					pidArg->target_dps_constrained = signed_target_dps_end_decel;

				pidArg->target_pos += pidArg->time_since_last_loop * pidArg->target_dps_constrained;

				if (pidArg->run_pos)
				{
					if ((pidArg->target_dps_constrained > 0 && pidArg->target_pos > pidArg->end_pos)
					|| (pidArg->target_dps_constrained < 0 && pidArg->target_pos < pidArg->end_pos))
						pidArg->target_pos = pidArg->end_pos;

					if (pidArg->hold_done && pidArg->target_pos == pidArg->end_pos)
					{
						if (!pidArg->hit_end)
						{
							pidArg->hit_end = true;
							pidArg->hit_end_time_us = pidArg->last_update;
						}

						if (fabs(pidArg->error) <= MOTOR_POS_MIN_ERROR_MOTOR_DEG
						|| pidArg->last_update - pidArg->hit_end_time_us >= MOTOR_POS_TIMEOUT_US)
						{
							stopAction_static(pidArg, encoderArg, pos, dps);
							return;
						}
					}
					else
						pidArg->hit_end = false;
				}
			}
			else
				pidArg->stalled = true;

			pidArg->error = pidArg->target_pos - pos;
			pidArg->output = pidArg->pos_pid->compute(pidArg->error);

			if (pidArg->error != 0 && pidArg->pwm_exp > 0 && pidArg->pwm_mag > 0)
				pidArg->output = (pidArg->output > 0 ? 1 : -1) * pidArg->pwm_mag * exp(fabs(pidArg->output) * pidArg->pwm_exp);

			runPWM_static(pidArg, pidArg->output);

			if (pidArg->debug)
			{
				Serial.print("Err:");
				Serial.println(pidArg->error);
			}
		}
		else if (!(EVNAlpha::motorsEnabled() && pidArg->run_pwm))
		{
			if (!EVNAlpha::motorsEnabled())
				pidArg->stop_action = STOP_BRAKE;

			stopAction_static(pidArg, encoderArg, pos, dps);
		}
	}

	//different pin change interrupts attached depending on port (determined by pins)
	//also adds timer interrupt shared by all ports (if not already added)
	static void attach_interrupts(volatile encoder_state_t* encoderArg, volatile pid_control_t* pidArg)
	{
		switch (pidArg->port)
		{
		case 1:
			if (!ports_enabled[0])
			{
				encoderArgs[0] = encoderArg;
				pidArgs[0] = pidArg;
				ports_enabled[0] = true;
				if (!odom_enabled[0])
				{
					attachInterrupt(encoderArg->enca, isr0, CHANGE);
					attachInterrupt(encoderArg->encb, isr1, CHANGE);
					odom_enabled[0] = true;
				}
			}
			break;

		case 2:
			if (!ports_enabled[1])
			{
				encoderArgs[1] = encoderArg;
				pidArgs[1] = pidArg;
				ports_enabled[1] = true;
				if (!odom_enabled[1])
				{
					attachInterrupt(encoderArg->enca, isr2, CHANGE);
					attachInterrupt(encoderArg->encb, isr3, CHANGE);
					odom_enabled[1] = true;
				}
			}
			break;

		case 3:
			if (!ports_enabled[2])
			{
				encoderArgs[2] = encoderArg;
				pidArgs[2] = pidArg;
				ports_enabled[2] = true;
				if (!odom_enabled[2])
				{
					attachInterrupt(encoderArg->enca, isr4, CHANGE);
					attachInterrupt(encoderArg->encb, isr5, CHANGE);
					odom_enabled[2] = true;
				}
			}
			break;

		case 4:
			if (!ports_enabled[3])
			{
				encoderArgs[3] = encoderArg;
				pidArgs[3] = pidArg;
				ports_enabled[3] = true;
				if (!odom_enabled[3])
				{
					attachInterrupt(encoderArg->enca, isr6, CHANGE);
					attachInterrupt(encoderArg->encb, isr7, CHANGE);
					odom_enabled[3] = true;
				}
			}
			break;
		}

		if (!timerisr_enabled)
		{
			if (rp2040.cpuid() == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0.sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer0.sharedISRTimer(3));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1.sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer1.sharedISRTimer(3));

			timerisr_enabled = true;
		}
	}

	// pin change interrupt ISRs (calling generic functions)
	// RPM measurement only uses pulses on one encoder wheel (more accurate, from our testing)
	static void isr0()
	{
		pos_read(encoderArgs[0]);

		uint32_t now = micros();
		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[0]);
			velocity_update(encoderArgs[0], now);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr1()
	{
		pos_read(encoderArgs[0]);

		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[0]);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr2()
	{
		pos_read(encoderArgs[1]);

		uint32_t now = micros();
		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[1]);
			velocity_update(encoderArgs[1], now);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr3()
	{
		pos_read(encoderArgs[1]);

		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[1]);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr4()
	{
		pos_read(encoderArgs[2]);

		uint32_t now = micros();
		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[2]);
			velocity_update(encoderArgs[2], now);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr5()
	{
		pos_read(encoderArgs[2]);

		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[2]);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr6()
	{
		pos_read(encoderArgs[3]);

		uint32_t now = micros();
		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[3]);
			velocity_update(encoderArgs[3], now);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	static void isr7()
	{
		pos_read(encoderArgs[3]);

		if (EVNCoreSync0.core1_pin_isr_enter())
		{
			pos_update(encoderArgs[3]);
			EVNCoreSync0.core1_pin_isr_exit();
		}
	}

	// timer interrupt ISR
	static bool timerisr(struct repeating_timer* t)
	{
		if (EVNCoreSync0.core1_timer_isr_enter())
		{
			for (int i = 0; i < MAX_MOTOR_OBJECTS; i++)
			{
				if (ports_enabled[i])
					pid_update(pidArgs[i], encoderArgs[i]);
			}

			EVNCoreSync0.core1_timer_isr_exit();
		}

		return true;
	}
};

class EVNDrivebase
{
public:
	static const uint8_t MAX_DB_OBJECTS = 2;
	static const uint16_t PID_TIMER_INTERVAL_US = 1000;

	friend class EVNMotor;

	EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right);
	void begin() volatile;

	void setSpeedKp(float kp) volatile;
	void setSpeedKd(float kd) volatile;
	void setTurnRateKp(float kp) volatile;
	void setTurnRateKd(float kd) volatile;
	void setSpeedAccel(float accel_mm_per_s_sq) volatile;
	void setSpeedDecel(float decel_mm_per_s_sq) volatile;
	void setTurnRateAccel(float accel_deg_per_s_sq) volatile;
	void setTurnRateDecel(float decel_deg_per_s_sq) volatile;
	void setDebug(uint8_t debug_type) volatile;
	void setAxleTrack(float axle_track) volatile;
	void setWheelDia(float wheel_dia) volatile;

	float getSpeedKp() volatile;
	float getSpeedKd() volatile;
	float getTurnRateKp() volatile;
	float getTurnRateKd() volatile;
	float getSpeedAccel() volatile;
	float getSpeedDecel() volatile;
	float getTurnRateAccel() volatile;
	float getTurnRateDecel() volatile;
	uint8_t getDebug() volatile;
	float getAxleTrack() volatile;
	float getWheelDia() volatile;
	float getMaxSpeed() volatile;
	float getMaxTurnRate() volatile;

	float getDistance() volatile;
	float getAngle() volatile;
	float getHeading() volatile;
	float getX() volatile;
	float getY() volatile;
	void resetXY() volatile;
	float getDistanceToPoint(float x, float y) volatile;

	void drivePct(float speed_outer_pct, float turn_rate_pct, bool disable_accel_decel = true) volatile;
	void drive(float speed, float turn_rate, bool disable_accel_decel = false) volatile;
	void driveTurnRate(float speed, float turn_rate, bool disable_accel_decel = false) volatile;
	void driveRadius(float speed, float radius, bool disable_accel_decel = false) volatile;
	void straight(float speed, float distance, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curve(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curveRadius(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turn(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turnDegrees(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void turnHeading(float turn_rate, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true) volatile;
	void driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true) volatile;

	void stop() volatile;
	void coast() volatile;
	void hold() volatile;

	bool completed() volatile;

private:
	float get_target_angle() volatile;
	float get_target_heading() volatile;

	uint8_t clean_input_stop_action(uint8_t stop_action) volatile;
	float clean_input_turn_rate_unsafe(float turn_rate) volatile;
	float clean_input_speed_unsafe(float speed, float turn_rate) volatile;
	float scaling_factor_for_maintaining_radius_unsafe(float speed, float turn_rate) volatile;
	float radius_to_turn_rate_unsafe(float speed, float radius) volatile;

	volatile drivebase_state_t db = {};
	static volatile drivebase_state_t* dbArgs[MAX_DB_OBJECTS];
	static volatile bool dbs_enabled[MAX_DB_OBJECTS];
	static volatile bool odom_enabled[MAX_DB_OBJECTS];
	static volatile bool timerisr_enabled;

	static void compute_max_rpm_drivebase_derived_values_unsafe(volatile drivebase_state_t* arg)
	{
		arg->max_rpm = min(arg->motor_left->_pid_control.max_rpm, arg->motor_right->_pid_control.max_rpm);
		arg->max_dps = arg->max_rpm * 6;
		arg->max_speed = arg->max_dps * arg->wheel_dia_mul_pi_div_360;
		arg->max_turn_rate = arg->max_dps * arg->wheel_dia_div_axle_track;
	}

	static void compute_targets_decel_derived_values_unsafe(volatile drivebase_state_t* arg)
	{
		arg->time_to_decel_speed = fabs(arg->target_speed) / arg->speed_decel;
		arg->time_to_decel_turn_rate = fabs(arg->target_turn_rate) / arg->turn_rate_decel;
		arg->slowed_turn_rate_decel = arg->turn_rate_decel * arg->time_to_decel_turn_rate / arg->time_to_decel_speed;
		arg->slowed_speed_decel = arg->speed_decel * arg->time_to_decel_speed / arg->time_to_decel_turn_rate;
		arg->target_speed_sq_div_2 = arg->target_speed * arg->target_speed / 2;
		arg->target_turn_rate_sq_div_2 = arg->target_turn_rate * arg->target_turn_rate / 2;
	}

	static void compute_drivebase_derived_values_unsafe(volatile drivebase_state_t* arg)
	{
		arg->wheel_dia_mul_pi_div_720 = arg->wheel_dia * M_PI / 720;
		arg->wheel_dia_mul_pi_div_360 = arg->wheel_dia_mul_pi_div_720 * 2;
		arg->wheel_dia_mul_pi_div_360_div_axle_track = arg->wheel_dia_mul_pi_div_360 / arg->axle_track;
		arg->_360_div_pi_div_wheel_dia = 360 / (M_PI * arg->wheel_dia);
		arg->axle_track_div_wheel_dia = arg->axle_track / arg->wheel_dia;
		arg->wheel_dia_div_axle_track = arg->wheel_dia / arg->axle_track;
	}

	static void set_drivebase_mode_unsafe(uint8_t idx, bool enable)
	{
		if (odom_enabled[idx])
		{
			if (dbs_enabled[idx] != enable)
			{
				dbArgs[idx]->stop_action = STOP_BRAKE;
				stopAction_static(dbArgs[idx]);
				dbs_enabled[idx] = enable;
			}
		}
	}

	static void attach_interrupts(volatile drivebase_state_t* arg)
	{
		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (!dbs_enabled[i])
			{
				dbArgs[i] = arg;
				dbs_enabled[i] = true;
				odom_enabled[i] = true;
				arg->id = i;
				break;
			}
		}

		if (!timerisr_enabled)
		{
			if (EVNMotor::timerisr_enabled)
			{
				//remove timer interrupt added by EVNMotor (if added)
				if (rp2040.cpuid() == 0)
					cancel_repeating_timer(&EVNISRTimer0.sharedISRTimer(3));
				else
					cancel_repeating_timer(&EVNISRTimer1.sharedISRTimer(3));
			}

			if (rp2040.cpuid() == 0)
				alarm_pool_add_repeating_timer_us(EVNISRTimer0.sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer0.sharedISRTimer(4));
			else
				alarm_pool_add_repeating_timer_us(EVNISRTimer1.sharedAlarmPool(), PID_TIMER_INTERVAL_US, timerisr, nullptr, &EVNISRTimer1.sharedISRTimer(4));

			EVNMotor::timerisr_enabled = true;
			timerisr_enabled = true;
		}
	}

	static bool timerisr(struct repeating_timer* t)
	{
		if (EVNCoreSync0.core1_timer_isr_enter())
		{
			for (int i = 0; i < MAX_DB_OBJECTS; i++)
			{
				if (odom_enabled[i]
					&& EVNMotor::ports_enabled[dbArgs[i]->motor_left->_pid_control.port - 1]
					&& EVNMotor::ports_enabled[dbArgs[i]->motor_right->_pid_control.port - 1])
				{
					max_rpm_update(dbArgs[i]);
					pos_update(dbArgs[i]);
				}

				if (dbs_enabled[i]
					&& EVNMotor::ports_enabled[dbArgs[i]->motor_left->_pid_control.port - 1]
					&& EVNMotor::ports_enabled[dbArgs[i]->motor_right->_pid_control.port - 1])
					pid_update(dbArgs[i]);
			}

			for (int i = 0; i < EVNMotor::MAX_MOTOR_OBJECTS; i++)
				if (EVNMotor::ports_enabled[i])
					EVNMotor::pid_update(EVNMotor::pidArgs[i], EVNMotor::encoderArgs[i]);

			EVNCoreSync0.core1_timer_isr_exit();
		}

		return true;
	}

	static void stopAction_static(volatile drivebase_state_t* arg)
	{
		switch (arg->stop_action)
		{
		case STOP_BRAKE:
			arg->target_speed_constrained = 0;
			arg->target_turn_rate_constrained = 0;
			arg->motor_left->stop_unsafe();
			arg->motor_right->stop_unsafe();
			arg->target_angle = arg->current_angle;
			arg->target_distance = arg->current_distance;
			arg->drive = false;
			arg->drive_position = false;
			arg->hold_done = true;
			break;

		case STOP_COAST:
			arg->target_speed_constrained = 0;
			arg->target_turn_rate_constrained = 0;
			arg->motor_left->coast_unsafe();
			arg->motor_right->coast_unsafe();
			arg->target_angle = arg->current_angle;
			arg->target_distance = arg->current_distance;
			arg->drive = false;
			arg->drive_position = false;
			arg->hold_done = true;
			break;

		case STOP_HOLD:
			if (!arg->drive && !arg->drive_position)
			{
				arg->end_angle = arg->current_angle;
				arg->end_distance = arg->current_distance;
				arg->target_speed = arg->max_speed;
				arg->target_turn_rate = arg->max_turn_rate;
			}
			else
			{
				arg->end_angle = arg->target_angle;
				arg->end_distance = arg->target_distance;
			}

			compute_targets_decel_derived_values_unsafe(arg);
			arg->drive = true;
			arg->drive_position = true;
			arg->hold_done = false;
			break;

		case STOP_NONE:
			arg->target_speed = arg->target_speed_constrained;
			arg->target_turn_rate = arg->target_turn_rate_constrained;
			arg->drive = true;
			arg->drive_position = false;
			arg->hold_done = true;
			break;
		}
	}

	static float getDistance_static(volatile drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition_static(&arg->motor_right->_encoder) + arg->motor_left->getPosition_static(&arg->motor_left->_encoder)) * arg->wheel_dia_mul_pi_div_720;
	}

	static float getAngleRad_static(volatile drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition_static(&arg->motor_right->_encoder) - arg->motor_left->getPosition_static(&arg->motor_left->_encoder)) * arg->wheel_dia_mul_pi_div_360_div_axle_track;
	}

	static void max_rpm_update(volatile drivebase_state_t* arg)
	{
		if (!arg->motor_left->_pid_control.max_rpm_calculated || !arg->motor_right->_pid_control.max_rpm_calculated)
		{
			compute_max_rpm_drivebase_derived_values_unsafe(arg);
			arg->motor_left->_pid_control.max_rpm_calculated = true;
			arg->motor_right->_pid_control.max_rpm_calculated = true;
		}
	}

	static void pos_update(volatile drivebase_state_t* arg)
	{
		//update time between loops
		uint32_t now = micros();
		arg->time_since_last_loop = (float)(now - arg->last_update) * 0.000001;
		arg->last_update = now;

		if (arg->time_since_last_loop < 0)
			return;

		//update angle and linear distance travelled
		float current_angle_rad = getAngleRad_static(arg);
		arg->current_distance = getDistance_static(arg);
		float distance_travelled_in_last_loop = arg->current_distance - arg->prev_distance;
		arg->prev_distance = arg->current_distance;

		arg->position_x += distance_travelled_in_last_loop * cos(current_angle_rad);
		arg->position_y += distance_travelled_in_last_loop * sin(current_angle_rad);
		arg->current_angle = current_angle_rad * RAD_TO_DEG;
	}

	static float motorsStopped_static(volatile drivebase_state_t* arg)
	{
		return (arg->motor_right->getDPS_static(&arg->motor_right->_encoder) <= DRIVEBASE_HOLD_THRESHOLD_DPS
			&& arg->motor_left->getDPS_static(&arg->motor_left->_encoder) <= DRIVEBASE_HOLD_THRESHOLD_DPS);
	}

	static void update_target_speed_constrained(volatile drivebase_state_t* arg)
	{
		if (arg->target_speed_constrained < arg->target_speed_after_decel)
		{
			if (arg->target_speed_constrained > 0)
				arg->target_speed_constrained += arg->time_since_last_loop * arg->speed_accel;
			else
				arg->target_speed_constrained += arg->time_since_last_loop * arg->speed_decel;

			if (arg->target_speed_constrained > arg->target_speed_after_decel)
				arg->target_speed_constrained = arg->target_speed_after_decel;
		}
		else if (arg->target_speed_constrained > arg->target_speed_after_decel)
		{
			if (arg->target_speed_constrained > 0)
				arg->target_speed_constrained -= arg->time_since_last_loop * arg->speed_decel;
			else
				arg->target_speed_constrained -= arg->time_since_last_loop * arg->speed_accel;

			if (arg->target_speed_constrained < arg->target_speed_after_decel)
				arg->target_speed_constrained = arg->target_speed_after_decel;
		}
	}

	static void update_target_turn_rate_constrained(volatile drivebase_state_t* arg)
	{
		if (arg->target_turn_rate_constrained < arg->target_turn_rate_after_decel)
		{
			if (arg->target_turn_rate_constrained > 0)
				arg->target_turn_rate_constrained += arg->time_since_last_loop * arg->turn_rate_accel;
			else
				arg->target_turn_rate_constrained += arg->time_since_last_loop * arg->turn_rate_decel;

			if (arg->target_turn_rate_constrained > arg->target_turn_rate_after_decel)
				arg->target_turn_rate_constrained = arg->target_turn_rate_after_decel;
		}
		else if (arg->target_turn_rate_constrained > arg->target_turn_rate_after_decel)
		{
			if (arg->target_turn_rate_constrained > 0)
				arg->target_turn_rate_constrained -= arg->time_since_last_loop * arg->turn_rate_decel;
			else
				arg->target_turn_rate_constrained -= arg->time_since_last_loop * arg->turn_rate_accel;

			if (arg->target_turn_rate_constrained < arg->target_turn_rate_after_decel)
				arg->target_turn_rate_constrained = arg->target_turn_rate_after_decel;
		}
	}

	static void pid_update(volatile drivebase_state_t* arg)
	{
		if (arg->time_since_last_loop < 0)
			return;

		if (!arg->hold_done)
		{
			if (arg->end_angle == arg->target_angle && arg->end_distance == arg->target_distance)
			{
				if (!motorsStopped_static(arg))
				{
					if (fabs(arg->angle_error) <= DRIVEBASE_HOLD_MIN_ERROR_MOTOR_DEG 
					|| fabs(arg->speed_error) <= DRIVEBASE_HOLD_MIN_ERROR_MOTOR_DEG)
						arg->last_stopped_time_us = arg->last_update;
				}
				else
				{
					if (arg->last_update - arg->hold_start_time_us >= DRIVEBASE_HOLD_TIMEOUT_US
					|| arg->last_update - arg->last_stopped_time_us >= DRIVEBASE_HOLD_MIN_TIME_US)
					{
						arg->target_speed_constrained = 0;
						arg->target_turn_rate_constrained = 0;
						arg->target_speed = 0;
						arg->target_turn_rate = 0;
						arg->drive = true;
						arg->drive_position = false;
						arg->hold_done = true;
					}
				}
			}
			else
			{
				arg->last_stopped_time_us = arg->last_update;
			}
		}
		else
		{
			arg->hold_start_time_us = arg->last_update;
			arg->last_stopped_time_us = arg->last_update;
		}

		if (EVNAlpha::motorsEnabled() && arg->drive)
		{
			//calculate speed & turn rate when motor is coming to a pause
			if (arg->drive_position)
			{
				//set correct polarities to correct for error
				arg->target_speed = (arg->end_distance - arg->target_distance > 0 ? 1 : -1) * fabs(arg->target_speed);
				arg->target_turn_rate = (arg->end_angle - arg->target_angle > 0 ? 1 : -1) * fabs(arg->target_turn_rate);

				arg->target_speed_after_decel = arg->target_speed;
				arg->target_turn_rate_after_decel = arg->target_turn_rate;

				//only apply decel to stop if stop action is not STOP_NONE
				if (arg->stop_action != STOP_NONE)
				{
					//calculate speed and turn rate with decel taken into account
					//note that only one of these will be chosen later in the code
					float error_to_start_decel_speed = arg->target_speed_sq_div_2 / arg->speed_decel;
					float error_to_start_decel_turn_rate = arg->target_turn_rate_sq_div_2 / arg->turn_rate_decel;
					float distance_error = fabs(arg->end_distance - arg->target_distance);
					float angle_error = fabs(arg->end_angle - arg->target_angle);

					if (distance_error < error_to_start_decel_speed)
						arg->target_speed_after_decel *= sqrt(distance_error / error_to_start_decel_speed);
					if (angle_error < error_to_start_decel_turn_rate)
						arg->target_turn_rate_after_decel *= sqrt(angle_error / error_to_start_decel_turn_rate);
				}
			}
			else
			{
				arg->target_speed_after_decel = arg->target_speed;
				arg->target_turn_rate_after_decel = arg->target_turn_rate;
			}

			//if speed + turn rate output is saturated, stop incrementing (anti-windup)
			if (fabs(arg->speed_error * arg->speed_pid->getKp()) + fabs(arg->angle_error * arg->turn_rate_pid->getKp()) < 1)
			{
				if (!arg->accel_decel_off)
				{
					float time_to_hit_speed;
					float time_to_hit_turn_rate;
					float difference = arg->target_speed_after_decel - arg->target_speed_constrained;

					//calculate time to hit speed
					if (arg->target_speed_after_decel > 0 && arg->target_speed_constrained > 0)
					{
						if (difference > 0)
							time_to_hit_speed = fabs(difference) / arg->speed_accel;
						else
							time_to_hit_speed = fabs(difference) / arg->speed_decel;
					}
					else if (arg->target_speed_after_decel < 0 && arg->target_speed_constrained < 0)
					{
						float difference = arg->target_speed_after_decel - arg->target_speed_constrained;
						if (difference < 0)
							time_to_hit_speed = fabs(difference) / arg->speed_accel;
						else
							time_to_hit_speed = fabs(difference) / arg->speed_decel;
					}
					else
						time_to_hit_speed = fabs(arg->target_speed_constrained) / arg->speed_decel + fabs(arg->target_speed_after_decel) / arg->speed_accel;

					//calculate time to hit turn rate
					difference = arg->target_turn_rate_after_decel - arg->target_turn_rate_constrained;
					if (arg->target_turn_rate_after_decel > 0 && arg->target_turn_rate_constrained > 0)
					{
						if (difference > 0)
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_accel;
						else
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
					else if (arg->target_turn_rate_after_decel < 0 && arg->target_turn_rate_constrained < 0)
					{
						if (difference < 0)
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_accel;
						else
							time_to_hit_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
					else
						time_to_hit_turn_rate = fabs(arg->target_turn_rate_constrained) / arg->turn_rate_decel + fabs(arg->target_turn_rate_after_decel) / arg->turn_rate_accel;

					//if speed takes longer to hit, force turn rate to sync with speed to obey turning radius
					if (time_to_hit_turn_rate < time_to_hit_speed && arg->target_speed != 0)
					{
						update_target_speed_constrained(arg);
						arg->target_turn_rate_constrained = arg->target_turn_rate / arg->target_speed * arg->target_speed_constrained;
					}
					//if turn rate takes longer to hit, force speed to sync with turn rate to obey turning radius
					else if (time_to_hit_turn_rate > time_to_hit_speed && arg->target_turn_rate != 0)
					{
						update_target_turn_rate_constrained(arg);
						arg->target_speed_constrained = arg->target_speed / arg->target_turn_rate * arg->target_turn_rate_constrained;
					}
					else
					{
						update_target_speed_constrained(arg);
						update_target_turn_rate_constrained(arg);
					}
				}
				else
				{
					arg->target_speed_constrained = arg->target_speed_after_decel;
					arg->target_turn_rate_constrained = arg->target_turn_rate_after_decel;
				}

				//increment/decrement target angle and distance with updated target speed/turn rate values
				arg->target_angle += arg->time_since_last_loop * arg->target_turn_rate_constrained;
				arg->target_distance += arg->time_since_last_loop * arg->target_speed_constrained;
			}

			if (arg->drive_position)
			{
				if ((arg->target_speed_constrained > 0 && arg->target_distance > arg->end_distance)
				|| (arg->target_speed_constrained < 0 && arg->target_distance < arg->end_distance))
					arg->target_distance = arg->end_distance;

				if ((arg->target_turn_rate_constrained > 0 && arg->target_angle > arg->end_angle)
				|| (arg->target_turn_rate_constrained < 0 && arg->target_angle < arg->end_angle))
					arg->target_angle = arg->end_angle;

				//ideally, we should only stop when both errors are in acceptable range
				//however, our control scheme might only hit both targets SOME of the time
				//so we count it as complete when the motor is already targeting the angle and distance endpoints, and either error is acceptable
				if (arg->hold_done && arg->target_angle == arg->end_angle && arg->target_distance == arg->end_distance)
				{
					if (!arg->hit_end)
					{
						arg->hit_end = true;
						arg->hit_end_time_us = arg->last_update;
					}

					if (fabs(arg->angle_error) <= DRIVEBASE_POS_MIN_ERROR_MOTOR_DEG
					|| fabs(arg->speed_error) <= DRIVEBASE_POS_MIN_ERROR_MOTOR_DEG
					|| arg->last_update - arg->hit_end_time_us >= DRIVEBASE_POS_TIMEOUT_US)
					{
						stopAction_static(arg);
						return;
					}
				}
				else
					arg->hit_end = false;
			}

			//angle error -> difference between the robot's current angle and the angle it should travel at
			arg->angle_error = arg->target_angle - arg->current_angle;
			if (arg->angle_error > 180)
				arg->angle_error -= 360;
			if (arg->angle_error < -180)
				arg->angle_error += 360;

			arg->angle_error *= arg->axle_track_div_wheel_dia;						//in motor degrees
			arg->angle_output = arg->turn_rate_pid->compute(arg->angle_error);

			//speed error -> distance between the db position and its target
			arg->speed_error = arg->target_distance - arg->current_distance;
			arg->speed_error = arg->speed_error * arg->_360_div_pi_div_wheel_dia;	//in motor degrees
			arg->speed_output = arg->speed_pid->compute(arg->speed_error);

			//calculate motor speeds
			//speed PID output   -> average speed
			//angle PID output -> difference between speeds
			if (arg->debug == DEBUG_SPEED)
			{
				arg->target_motor_left_duty_cycle = arg->speed_output;
				arg->target_motor_right_duty_cycle = arg->speed_output;
			}
			else if (arg->debug == DEBUG_TURN_RATE)
			{
				arg->target_motor_left_duty_cycle = -arg->angle_output;
				arg->target_motor_right_duty_cycle = arg->angle_output;
			}
			else
			{
				arg->target_motor_left_duty_cycle = arg->speed_output - arg->angle_output;
				arg->target_motor_right_duty_cycle = arg->speed_output + arg->angle_output;
			}

			//maintain ratio between speeds when either exceeds motor limits
			if (arg->target_motor_left_duty_cycle != 0 && arg->target_motor_right_duty_cycle != 0)
			{
				if (fabs(arg->target_motor_left_duty_cycle) > 1)
				{
					float ratio = arg->target_motor_right_duty_cycle / arg->target_motor_left_duty_cycle;
					arg->target_motor_left_duty_cycle = arg->target_motor_left_duty_cycle > 0 ? 1 : -1;
					arg->target_motor_right_duty_cycle = arg->target_motor_left_duty_cycle * ratio;
				}

				if (fabs(arg->target_motor_right_duty_cycle) > 1)
				{
					float ratio = arg->target_motor_left_duty_cycle / arg->target_motor_right_duty_cycle;
					arg->target_motor_right_duty_cycle = arg->target_motor_right_duty_cycle > 0 ? 1 : -1;
					arg->target_motor_left_duty_cycle = arg->target_motor_right_duty_cycle * ratio;
				}
			}

			if (arg->motor_left->_pid_control.pwm_exp > 0 && arg->motor_left->_pid_control.pwm_mag > 0)
				arg->target_motor_left_duty_cycle = (arg->target_motor_left_duty_cycle > 0 ? 1 : -1)
					* arg->motor_left->_pid_control.pwm_mag
					* exp(fabs(arg->target_motor_left_duty_cycle) * arg->motor_left->_pid_control.pwm_exp);

			if (arg->motor_right->_pid_control.pwm_exp > 0 && arg->motor_right->_pid_control.pwm_mag > 0)
				arg->target_motor_right_duty_cycle = (arg->target_motor_right_duty_cycle > 0 ? 1 : -1)
					* arg->motor_right->_pid_control.pwm_mag
					* exp(fabs(arg->target_motor_right_duty_cycle) * arg->motor_right->_pid_control.pwm_exp);

			//write speeds to motors
			//non-threadsafe write used here, assumed safe because EVNDrivebase and EVNMotor should be on same core
			arg->motor_left->runPWM_unsafe(arg->target_motor_left_duty_cycle);
			arg->motor_right->runPWM_unsafe(arg->target_motor_right_duty_cycle);

			if (arg->debug == DEBUG_SPEED)
			{
				Serial.print("Spd_Err:");
				Serial.println(arg->speed_error);
			}
			else if (arg->debug == DEBUG_TURN_RATE)
			{
				Serial.print("Turn_Err:");
				Serial.println(arg->angle_error);
			}
		}
		else
		{
			if (!EVNAlpha::motorsEnabled())
				arg->stop_action = STOP_BRAKE;

			stopAction_static(arg);
		}
	}
};

class EVNOmniDrivebaseBasic
{
public:
	EVNOmniDrivebaseBasic(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* fl, EVNMotor* fr, EVNMotor* bl, EVNMotor* br)
	{
		_fl = fl;
		_fr = fr;
		_bl = bl;
		_br = br;
	};

	void begin()
	{
		_max_rpm = min(min(_fl->_pid_control.max_rpm, _fr->_pid_control.max_rpm), min(_bl->_pid_control.max_rpm, _br->_pid_control.max_rpm));
	};

	void steer(float speed, float angle, float turn_rate)
	{
		float speedc, anglec, rotatec;
		float anglec_rad, speedc_x, speedc_y;
		float speedc_x_left, speedc_y_left, speedc_x_right, speedc_y_right;

		speedc = constrain(speed, -_max_rpm * 6, _max_rpm * 6);
		anglec = constrain(angle, 0, 360);
		anglec = fmod(anglec + 45, 360);
		rotatec = constrain(turn_rate, -1, 1);

		anglec_rad = anglec / 180 * M_PI;
		speedc_x = sin(anglec_rad) * speedc;
		speedc_y = cos(anglec_rad) * speedc;

		if (anglec >= 90 && anglec <= 270)
		{
			if (rotatec >= 0)
			{
				speedc_y_left = speedc_y * (1 - 2 * rotatec);
				speedc_y_right = speedc_y;
			}
			else {
				speedc_y_right = speedc_y * (1 + 2 * rotatec);
				speedc_y_left = speedc_y;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_y_right = speedc_y * (1 - 2 * rotatec);
				speedc_y_left = speedc_y;
			}
			else {
				speedc_y_left = speedc_y * (1 + 2 * rotatec);
				speedc_y_right = speedc_y;
			}
		}

		if (anglec >= 0 && anglec <= 180)
		{
			if (rotatec >= 0)
			{
				speedc_x_right = speedc_x * (1 - 2 * rotatec);
				speedc_x_left = speedc_x;
			}
			else {
				speedc_x_left = speedc_x * (1 + 2 * rotatec);
				speedc_x_right = speedc_x;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_x_left = speedc_x * (1 - 2 * rotatec);
				speedc_x_right = speedc_x;
			}
			else {
				speedc_x_right = speedc_x * (1 + 2 * rotatec);
				speedc_x_left = speedc_x;
			}
		}

		_fl->runSpeed(speedc_x_left);
		_br->runSpeed(speedc_x_right);
		_fr->runSpeed(speedc_y_right);
		_bl->runSpeed(speedc_y_left);
	};

	void stop()
	{
		_fl->stop();
		_fr->stop();
		_bl->stop();
		_br->stop();
	};

	void coast()
	{
		_fl->coast();
		_fr->coast();
		_bl->coast();
		_br->coast();
	};

	void hold()
	{
		_fl->hold();
		_fr->hold();
		_bl->hold();
		_br->hold();
	};

private:
	EVNMotor* _fl;
	EVNMotor* _fr;
	EVNMotor* _bl;
	EVNMotor* _br;
	float _max_rpm;
};

#endif
