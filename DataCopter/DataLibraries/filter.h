
#ifndef FILTER_H
#define FILTER_H

#include "stdint.h"

typedef struct
{
	float in_est;    //Estimator
	float in_obs;    //Observation
	
	float fix_ki;
	float ei_limit;     //

	float e;
	float ei;

	float out;
}_inte_fix_filter_st;

typedef struct
{
	float in_est_d;   //Estimator
	float in_obs;    //Observation
	
	float fix_kp;
	float e_limit;

	float e;

	float out;
}_fix_inte_filter_st;

void inte_fix_filter(float dT,_inte_fix_filter_st *data);
void fix_inte_filter(float dT,_fix_inte_filter_st *data);

#endif //FILTER_H
