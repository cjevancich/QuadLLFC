#include "gain.h"
#include "control.h"
#include "setpoint.h"
#include "core/systick.h"
#include "state.h"
#include "esc.h"
#include "logging.h"

/*Reference:
typedef struct state_t {
	float roll;
	float pitch;
	float yaw;
	float x;
	float y;
	float z;
} state_t;
*/

const float GAIN_Z_THRESHOLD = 6.0;
static float _gain_reference_z;

//Imported from control.c, all gains stuff transferred
static state_t _control_p_gains[4];
static state_t _control_i_gains[4];
static state_t _control_d_gains[4];

float gain_get_p_pitch(int index)
{
  return _control_p_gains[index].pitch;
}

float gain_get_p_roll(int index)
{
  return _control_p_gains[index].roll;
}

float gain_get_p_yaw(int index)
{
  return _control_p_gains[index].yaw;
}

float gain_get_p_x(int index)
{
  return _control_p_gains[index].x;
}

float gain_get_p_y(int index)
{
  return _control_p_gains[index].y;
}

float gain_get_p_z(int index)
{
  return _control_p_gains[index].z;
}

float gain_get_i_pitch(int index)
{
  return _control_i_gains[index].pitch;
}

float gain_get_i_roll(int index)
{
  return _control_i_gains[index].roll;
}

float gain_get_i_yaw(int index)
{
  return _control_i_gains[index].yaw;
}

float gain_get_i_x(int index)
{
  return _control_i_gains[index].x;
}

float gain_get_i_y(int index)
{
  return _control_i_gains[index].y;
}

float gain_get_i_z(int index)
{
  return _control_i_gains[index].z;
}

float gain_get_d_pitch(int index)
{
  return _control_d_gains[index].pitch;
}

float gain_get_d_roll(int index)
{
  return _control_d_gains[index].roll;
}

float gain_get_d_yaw(int index)
{
  return _control_d_gains[index].yaw;
}

float gain_get_d_x(int index)
{
  return _control_d_gains[index].x;
}

float gain_get_d_y(int index)
{
  return _control_d_gains[index].y;
}

float gain_get_d_z(int index)
{
  return _control_d_gains[index].z;
}

//These should be renamed and references updated
state_t *control_get_p_gains(void) {
	return _control_p_gains;
}

state_t *control_get_i_gains(void) {
	return _control_i_gains;
}

state_t *control_get_d_gains(void) {
	return _control_d_gains;
}

void gain_init()
{
  //Set to the initial z value from inertial_state in state.c
  _gain_reference_z = state_get_inertial_z();
  //Gains initialized
  _control_p_gains[1].roll = -.065;
  _control_p_gains[3].roll = .065;
  _control_d_gains[1].roll = -440.0;
  _control_d_gains[3].roll = 440.0;
  _control_i_gains[1].roll = -.03;
  _control_i_gains[3].roll = .03;

  _control_p_gains[0].pitch = .065;
  _control_p_gains[2].pitch = -.065;
  _control_d_gains[0].pitch = 440.0;
  _control_d_gains[2].pitch = -440.0;
  _control_i_gains[0].roll = -.03;
  _control_i_gains[2].roll = .03;

  _control_p_gains[0].yaw = 0.075;
  _control_p_gains[1].yaw = -0.075;
  _control_p_gains[2].yaw = 0.075;
  _control_p_gains[3].yaw = -0.075;
  _control_d_gains[0].yaw = 240.0;
  _control_d_gains[1].yaw = -240.0;
  _control_d_gains[2].yaw = 240.0;
  _control_d_gains[3].yaw = -240.0;

  _control_p_gains[0].z = 1;
  _control_p_gains[1].z = 1;
  _control_p_gains[2].z = 1;
  _control_p_gains[3].z = 1;
}

void gain_update()
{
  float getZDelta = (state_get_inertial_z() - _gain_reference_z);
  
  if (getZDelta > GAIN_Z_THRESHOLD)
  {
    //we have liftoff
    //so we use the gains values from control.c as the default flight values
    _control_i_gains[1].roll = -.03;
    _control_i_gains[3].roll = .03;
    
    _control_i_gains[0].roll = -.03;
    _control_i_gains[2].roll = .03;
  }
  else if (getZDelta <= GAIN_Z_THRESHOLD)
  {
    //we have landed
    //we set all gains to 0?
    _control_i_gains[1].roll = 0;
    _control_i_gains[3].roll = 0;

    _control_i_gains[0].roll = 0;
    _control_i_gains[2].roll = 0;
  }
}