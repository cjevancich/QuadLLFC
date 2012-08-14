#include "control.h"
#include "setpoint.h"
#include "core/systick.h"
#include "state.h"
#include "esc.h"
#include "logging.h"
#include "gain.h"

#define CONTROL_UPDATE_INTERVAL 20

static uint32_t _control_last_update;
static state_t _control_setpoint_error_last;

static uint32_t _control_enabled;

static float _control_integral_slice_max[4];
static float _control_integral_max[4];
static float _control_motor_integrals[4];

void control_init(void) {
	// TODO
	// Set the gain values
	//
	_control_integral_slice_max[0] = gain_get_p_pitch(0) * .05;
	_control_integral_slice_max[1] = gain_get_p_roll(1) * .05;
	_control_integral_slice_max[2] = gain_get_p_pitch(2) * .05;
	_control_integral_slice_max[3] = gain_get_p_roll(3) * .05;

	_control_integral_max[0] = gain_get_p_pitch(0) * .5;
	_control_integral_max[1] = gain_get_p_roll(1) * .5;
	_control_integral_max[2] = gain_get_p_pitch(2) * .5;
	_control_integral_max[3] = gain_get_p_roll(3) * .5;
}

void control_reset(void) {
        /* Zero error integral */
	state_scale(&_control_setpoint_error_last, 0,
	            &_control_setpoint_error_last);

	int i = 0;
	for(i = 0;i < 4;i++) {
		_control_motor_integrals[i] = 0;
	}

        _control_last_update = 0;
}

void control_set_enabled(int value) {
	_control_enabled = value;
}

int control_enabled(void) {
	return _control_enabled;
}

void control_state_gains_multiply_to_motors(state_t *gains,
                                            state_t *error,
                                            float *motors) {
	int i;
	for(i = 0;i < ESC_CNT;i++) {
                motors[i] += gains->roll * error->roll;
                motors[i] += gains->pitch * error->pitch;
                motors[i] += gains->yaw * error->yaw;
                motors[i] += gains->z * error->z;
	}
}


void control_update(void) {
	int i;
        int j;
	float motor_accum[4] = { 0, 0, 0, 0 };
	state_t setpoint_error;
	state_t error_dt;
	state_t error_integral_slice;

	for(i = 0;i < 6;i++) {
		((float*)&setpoint_error)[i] = 0;
		((float*)&error_dt)[i] = 0;
		((float*)&error_integral_slice)[i] = 0;
	}

	// Check if control is enabled
	if(!_control_enabled)
		return;

	// Has enough time passed since last control update
	if((systickGetTicks() - _control_last_update) < 5)
		return;

	// Safety Third!
	state_t *inertial_state = state_inertial_get();
	if(inertial_state->roll >= 1 || inertial_state->roll <= -1 || inertial_state->pitch >= 1 || inertial_state->pitch <= -1) {
		logging_send_string(LOGGING_ERROR, "Shutting down due to extreme attenuation");
		esc_set_all_throttles(motor_accum);	
		_control_enabled = 0;
		return;
	}

	uint32_t d_msecs = systickGetTicks() - _control_last_update;
	float dt = d_msecs / 1000.0f;

	// Calculate p error
	state_subtract(setpoint_get(), state_inertial_get(), &setpoint_error);

	float motor_slice[4];
	if(_control_last_update != 0) {
		// Calculate error integral
		state_scale(&setpoint_error, dt, &error_integral_slice);

		// Calculate d error / dt
		state_subtract(&setpoint_error, &_control_setpoint_error_last, &error_dt);
		state_scale(&error_dt, dt, &error_dt);

		for(i = 0;i < 4;i++) {
			float *motor_i_gains = (float*)(&control_get_i_gains()[i]);
			motor_slice[i] = 0;
			for(j = 0;j < 6;j++) {
				motor_slice[i] += ((float*)&error_integral_slice)[j] * motor_i_gains[j];
			}
	/*
			if(motor_slice[i] > _control_integral_slice_max[i])
				motor_slice[i] = _control_integral_slice_max[i];
			else if (motor_slice[i] < -_control_integral_slice_max[i])
				motor_slice[i] = -_control_integral_slice_max[i];
	*/
			_control_motor_integrals[i] += motor_slice[i];
	/*

			// Check for max motor integral val
			if(_control_motor_integrals[i] > _control_integral_max[i])
				_control_motor_integrals[i] = _control_integral_max[i];
			else if(_control_motor_integrals[i] < -_control_integral_max[i])
				_control_motor_integrals[i] = -_control_integral_max[i];
	*/
		}
	}
	_control_last_update = systickGetTicks();

	// Update error_last
	state_copy(&setpoint_error, &_control_setpoint_error_last);

	if(_control_last_update == 0)
		return;

	// Accumulate gains * error
        //p
        for(j = 0;j < 4;j++) {
                motor_accum[j] += setpoint_error.roll*gain_get_p_roll(j) + setpoint_error.pitch*gain_get_p_pitch(j) +
                        setpoint_error.yaw*gain_get_p_yaw(j) + setpoint_error.z*gain_get_p_z(j);
        }

        //d
        for(j = 0;j < 4;j++) {
                motor_accum[j] += error_dt.roll*gain_get_d_roll(j) + error_dt.pitch*gain_get_d_pitch(j) +
                        error_dt.yaw*gain_get_d_yaw(j) + error_dt.z*gain_get_d_z(j);
        }

        //i
        for(j = 0;j < 4;j++)
		motor_accum[j] += _control_motor_integrals[j];

	// Apply throttles
	esc_set_all_throttles(motor_accum);
}

