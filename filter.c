#include "math.h"
#include "state.h"
#include "filter.h"
#include "sensors/itg3200.h"
#include "sensors/adxl345.h"
#include "sensors/hmc5883l.h"
#include "core/systick.h"
#include "commands.h"
#include "logging.h"

#define FILTER_ACCEL_UPDATE_INTERVAL 5
#define FILTER_MAG_UPDATE_INTERVAL 5

static uint32_t  _state_accel_last_update;
static uint32_t  _last_accel_update_ticks;
static AccelData _state_accel_last;

static uint32_t _state_mag_last_update;
static uint32_t _last_mag_update_ticks;
static MagData  _state_mag_last;

extern float rotation_b_to_i[3][3]; 

static float _corr_vector[3];
float _gyro_error[] = {0,0,0};

// pull accelerometer data
// make _state_accel_last.X, etc. available for use and update ticks

void filter_get_accel_data() 
{
	uint32_t ticks = systickGetTicks();
        if((ticks - _state_accel_last_update) >= FILTER_ACCEL_UPDATE_INTERVAL) 
	{

                if(adxl345GetData(&_state_accel_last) == i2c_ok) {
                	float accel_dt;
                	uint32_t accel_tick_diff = systickGetTicks() - _last_accel_update_ticks;

                        // Set dt in seconds
                        if(_last_accel_update_ticks == 0)
                                accel_dt = 0;
                        else
                                accel_dt = accel_tick_diff / 1000.0;
		}
         }

}

// pull mag data, which doesn't exist yet

/*void filter_get_mag_data()
{
	uint32_t ticks = systickGetTicks();
        if((ticks - _state_mag_last_update) >= FILTER_MAG_UPDATE_INTERVAL) 
	{
                if(hmc5883lGetData(&_state_mag_last) == i2c_ok) 
		{
                        uint32_t mag_tick_diff = systickGetTicks() - _last_mag_update_ticks;
                        float mag_dt;
                        // Set dt in seconds
                        if(_last_mag_update_ticks == 0)
                                mag_dt = 0;
                        else
                                mag_dt = mag_tick_diff / 1000.0;
		}
        }
}*/

// using existing r_matrix, accel data, magdata -- find total_correction

void filter_find_total_correction_vector() 
{
	filter_get_accel_data();
//	filter_get_mag_data();

        float g_ref[3];
        g_ref[0] = _state_accel_last.X;
        g_ref[1] = _state_accel_last.Y;
        g_ref[2] = _state_accel_last.Z;

        const float weight_rollpitch = 1;
        float rollpitch_corrplane[3];
        rollpitch_corrplane[0] = rotation_b_to_i[2][0];
  	rollpitch_corrplane[1] = rotation_b_to_i[2][1];
	rollpitch_corrplane[2] = rotation_b_to_i[2][2];

//      const float weight_yaw = 1;
//      float yaw_corr_heading;
//      yaw_corr_heading = atan2(_state_mag_last.Y,_state_mag_last.X);

        int temp = 0;
	int i;
        for (i=0; i<3; i++)
        {
            _corr_vector[i] = weight_rollpitch * rollpitch_corrplane[i];
//            _corr_vector[i] = temp + (weight_yaw * yaw_corr_heading);
        }
}

// using total_correction and available globals, find the error
// PI controller creates value to correct for the determined error 
// in the rotation correction matrix

void filter_get_gyro_correction_data(float *gyro_dt)
{
        filter_find_total_correction_vector();

        float wI_correction = 1; // what is the value of this? no idea.
        float kP = 1; // proportional gain constant
        float kI = 1; // integral gain constant

        state_t test;

//        float fix_yaw = 0;
        float fix_rollpitch = 0;
	int i;

        int dt = (int) gyro_dt;

        test.roll = _corr_vector[0];
        test.pitch = _corr_vector[1];
        test.yaw = _corr_vector[2];

        command_send(COMMAND_INERTIAL_STATE, (unsigned char*)&test, sizeof(state_t));

        for (i=0; i<3; i++)
        {
//                fix_yaw = kP * _corr_vector[i];
                fix_rollpitch = kI * dt;
		fix_rollpitch = fix_rollpitch * _corr_vector[i];
                fix_rollpitch = fix_rollpitch + wI_correction;
                _gyro_error[i] = fix_rollpitch;
        }
}


