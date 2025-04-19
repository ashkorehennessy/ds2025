/*
 * PID_Base.h
 *
 *  Created on: 2023��9��13��
 *      Author: ashkore
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "PID.h"
#include "stdint.h"
#include "tgmath.h"

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float last_error;
    float last_out;
    float integral;
    float outmax;
    float outmin;
    uint8_t use_lowpass_filter;
    float lowpass_filter_factor;
} PID_Base;

typedef struct{
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float last_last_error;
    float last_out;
    float out;
    float outmax;
    float outmin;
    uint8_t use_lowpass_filter;  // �������������̫����Կ�������һ����Ӧ�ٶ�
    float lowpass_filter_factor;
} PID_Incremental;

PID_Base PID_Base_Init(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_lowpass_filter, float lowpass_filter_factor);

float PID_Base_Calc(PID_Base *pid, float input_value, float setpoint);

PID_Incremental PID_Incremental_Init(float Kp, float Ki, float Kd, float outmax, float outmin, uint8_t use_lowpass_filter, float lowpass_filter_factor);

float PID_Incremental_Calc(PID_Incremental *pid, float input_value, float setpoint);

#endif /* CODE_PID_H_ */
