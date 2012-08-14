#include <stdint.h> 
#include "state.h"

//In control.c I updated the functions which used these variables, like the init function, to reference these instead
float gain_get_p_pitch(int);
float gain_get_p_roll(int);
float gain_get_p_yaw(int);
float gain_get_p_x(int);
float gain_get_p_y(int);
float gain_get_p_z(int);

float gain_get_i_pitch(int);
float gain_get_i_roll(int);
float gain_get_i_yaw(int);
float gain_get_i_x(int);
float gain_get_i_y(int);
float gain_get_i_z(int);

float gain_get_d_pitch(int);
float gain_get_d_roll(int);
float gain_get_d_yaw(int);
float gain_get_d_x(int);
float gain_get_d_y(int);
float gain_get_d_z(int);

//Update names and references
state_t *control_get_p_gains(void);
state_t *control_get_i_gains(void);
state_t *control_get_d_gains(void);

void gain_init(void);
void gain_update(void);