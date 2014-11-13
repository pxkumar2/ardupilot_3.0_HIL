/* Header File for abc controller for use in simulink */

#ifndef _ABC_CONTROLLER_
#define _ABC_CONTROLLER_

#include <stdio.h>
#include <math.h>

#define TRUE 1
#define FALSE 0

#include <limits.h>
#include "abc_controller_type.h"

extern void ABC_Controller(double input[16], double output[4], double bona_in[5], double save_data[12]);
extern float Derivative(float Prev_value, float Current_value, float Delta_t);
extern float lag_filter(float X, float tau, float Y_last);


#endif /* _ABC_CONTROLLER_ */

