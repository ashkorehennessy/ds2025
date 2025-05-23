//
// Created by ashkore on 25-4-18.
//

#ifndef POOL_H
#define POOL_H
#include <stdint.h>
#include "mpu6050.h"
#include "tfluna_i2c.h"
extern uint8_t buf[128];
extern TF_Luna_Lidar TF_Luna_1;

extern MPU6050_t mpu6050;
extern int16_t  tfDist;
extern int16_t  tfFlux;
extern int16_t  tfTemp;
#endif //POOL_H
