/* Header File for abc controller for use in simulink */

#ifndef _ABC_CONTROLLER_TYPE_
#define _ABC_CONTROLLER_TYPE_

typedef float        FLT;
typedef double       DBL;
typedef int          INT;
typedef unsigned int UINT;

typedef struct {
  float gamma_command;
  float kcas_command;
  float kcas;
  float pitch;
  float pitch_rate;
  float gamma_air;
  float flap_pos_constant;
  float altitude;
  float rpm;
  float pla;
  float gear_pos;
  float elevator;
  float weight;
  float mass;
} 
WSU_STATE;

typedef struct {
  float delta_t;
  float wing_mac;
  float wing_area;
  float elevator_neg_limit;
  float elevator_pos_limit;
} 
WSU_AIRPLANE;

#endif /* _ABC_CONTROLLER_TYPE_ */


