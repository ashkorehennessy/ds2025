//
// Created by ashkore on 25-4-18.
//
#include "pool.h"

float angle_adc = 0;
float angle_show = 0;
float angle_yaw = 0;
float angle_offset = 12;
float alpha = 0.4f;
float speed = 0;
int speed_setpoint = 0;
float adc_pidout = 0;
uint32_t adc_raw = 0;
float yaw_pidout = 0;
float deadzone = 800;