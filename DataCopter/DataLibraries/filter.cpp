
#include "filter.h"
#include "global.h"

void inte_fix_filter(float dT,_inte_fix_filter_st *data)
{
	float ei_lim_val;
	
	if(data->ei_limit>0)
	{		
		ei_lim_val = LIMIT(data->ei,-data->ei_limit,data->ei_limit);
	}
	else
	{
		ei_lim_val = data->ei;
	}	
	
	data->out = (data->in_est + ei_lim_val);
	
	data->e = data->fix_ki *(data->in_obs - data->out);

	data->ei += data->e *dT;
}

void fix_inte_filter(float dT,_fix_inte_filter_st *data)
{	
	data->out += (data->in_est_d + data->e ) *dT;
	
	data->e = data->fix_kp *(data->in_obs - data->out);

	if(data->e_limit>0)
	{		
		data->e = LIMIT(data->e,-data->e_limit,data->e_limit);
	}
}

