#include "bsp_fitler.h"

//
float bsp_lpf_fitler(float current, float index)
{
	float ret=0.0;
	
	static int flag;
	static float last;
	
	if(flag!=0)
		ret=index*current+(1.0-index)*last;
	else
	{
		ret=current;
		flag=1;
	}

	
	last=current;
	
	return ret;
	
}
