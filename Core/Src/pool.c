//
// Created by ashkore on 25-4-18.
//
#include "pool.h"

float angle_adc = 0;
float angle_top = 0;
float angle_yaw = 0;
float angle_offset = 0;
float alpha = 0.4f;
float speed = 0;
int speed_setpoint = 0;
uint32_t adc_raw = 0;
float adc_use = 0;
float adc_pidout = 0;
float yaw_pidout = 0;
float nav_pidout = 0;
float deadzone = 1000;
Whell motor;
float target_angle_yaw = 0;
float angle_turn_limit = 90;
int task_running = 0;
int task_index = 1;
float samples[140] = {0};
float samples_adc[140] = {0};
float offset;
float adc_mid = 2060;
float dead_angle = 30;
float angle_show = 0;