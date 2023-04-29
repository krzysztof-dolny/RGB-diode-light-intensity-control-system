/*
 * regulator.h
 *
 *  Created on: Jan 14, 2023
 *      Author: krzys
 */

#ifndef INC_REGULATOR_H_
#define INC_REGULATOR_H_

//#define reg_Type regulator_Handle_TypeDef*

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float u_p;
	float u_i;
	float u_d;
	float limitup;
	float limitdown;
}regulator_Handle_TypeDef;

float regulator_signal(regulator_Handle_TypeDef* reg, float y_ref, float pomiar);

#endif /* INC_REGULATOR_H_ */
