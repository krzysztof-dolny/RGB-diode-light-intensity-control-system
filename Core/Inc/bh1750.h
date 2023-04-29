/*
 * BH1750.h
 *
 *  Created on: Jan 14, 2023
 *      Author: krzys
 */

#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "stm32f7xx_hal.h"
#include "i2c.h"

#define BH1750_ADDRESS_L (0x23 << 1)
#define BH1750_ADDRESS_H (0x5C << 1)
#define BH1750_POWER_DOWN 0x00
#define BH1750_POWER_ON 0x01
#define BH1750_RESET 0x07
#define BH1750_CONTINOUS_H_RES_MODE 0X10
#define BH1750_CONTINOUS_H_RES_MODE2 0x11
#define BH1750_CONTINOUS_L_RES_MODE 0x13
#define BH1750_ONE_TIME_H_RES_MODE 0x20
#define BH1750_ONE_TIME_H_RES_MODE2 0x21
#define BH1750_ONE_TIME_L_RES_MODE 0x23
#define BH1750_GROUND_ADDR_WRITE (0x46 + 0)
#define BH1750_GROUND_ADDR_READ (0x46 + 1)

#define BH1750_I2CType I2C_HandleTypeDef*

typedef struct
{
	BH1750_I2CType I2C;
	uint8_t Address_r;
	uint8_t Address_w;
	uint32_t Timeout;
}BH1750_HandleTypeDef;

void BH1750_Init(BH1750_HandleTypeDef* bh1750, uint8_t command);
float BH1750_ReadLux(BH1750_HandleTypeDef* bh1750);

#endif /* INC_BH1750_H_ */
