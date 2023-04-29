/*
 * regulator.c
 *
 *  Created on: Jan 14, 2023
 *      Author: krzys
 */

#include "regulator.h"

float regulator_signal(regulator_Handle_TypeDef* Reg, float y_ref, float pomiar)
{
	float e;
	float u;
	float u_sat;
	e = y_ref - pomiar;
	Reg->u_i += Reg->Ki * e;

	u = Reg->u_i;

	if(u > Reg->limitup){
		u_sat = Reg->limitup;
	}
	else if( u < Reg->limitdown){
		u_sat = Reg->limitdown;
	}
	else{
		u_sat = u;
	}
	if(u != u_sat)
	{
		Reg->u_i -= Reg->Ki * e;
	}
	return u_sat ;
}
