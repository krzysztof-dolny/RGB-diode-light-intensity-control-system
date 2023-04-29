/*
 * BH1750.c
 *
 *  Created on: Jan 14, 2023
 *      Author: krzys
 */

#include <bh1750.h>

void BH1750_Init(BH1750_HandleTypeDef* bh1750, uint8_t command){
uint8_t start = BH1750_POWER_ON;
HAL_I2C_Master_Transmit(bh1750->I2C, bh1750->Address_w, &start, 1, bh1750->Timeout);
HAL_I2C_Master_Transmit(bh1750->I2C, bh1750->Address_w, &command, 1, bh1750->Timeout);
}

float BH1750_ReadLux(BH1750_HandleTypeDef* bh1750){
float light = 0;
uint8_t buff[2];

HAL_I2C_Master_Receive(bh1750->I2C, bh1750->Address_r, buff, 2, bh1750->Timeout);
light = ((buff[0] << 8) | buff[1]) / 1.2;

return light;
}
