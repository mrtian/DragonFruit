#ifndef __MPU_H
#define __MPU_H

#include "mm32_device.h"
#include <stdint.h>

// 根据 image_658c07.png，PD0 为 SDA，PD1 为 SCL
#define SCL_H GPIOD->BSRR = (1 << 1)
#define SCL_L GPIOD->BRR  = (1 << 1)
#define SDA_H GPIOD->BSRR = (1 << 0)
#define SDA_L GPIOD->BRR  = (1 << 0)
#define SDA_IN (GPIOD->IDR & (1 << 0))

// 器件地址
#define MPU_ADDR 0x69

// 函数声明
void MPU_Init(void);
void MPU_Get_Data_All(int16_t *ax, int16_t *ay, int16_t *az, int16_t *temp_cent,
                      int16_t *gx, int16_t *gy, int16_t *gz);

#endif