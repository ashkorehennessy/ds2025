//
// Created by ashkore on 25-4-18.
//

#ifndef POOL_H
#define POOL_H
#include <stdint.h>
#include "wheel.h"
void start_task(int index);
extern float angle_adc;
extern float angle_show;
extern float angle_yaw;
extern float speed;
extern int speed_setpoint;
extern float angle_offset;
extern float alpha;
extern float adc_pidout;
extern float yaw_pidout;
extern float deadzone;
extern uint32_t adc_raw;
extern Whell motor;
extern float target_angle_yaw;
extern int task_running;
extern int task_index;
#endif //POOL_H
