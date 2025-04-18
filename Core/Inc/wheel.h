//
// Created by ashkore on 23-7-25.
//

#ifndef BALANCE_CAR_WHEEL_H
#define BALANCE_CAR_WHEEL_H

#endif //BALANCE_CAR_WHEEL_H
#include "stm32f4xx_hal.h"


// Wheel struct
typedef struct {
    TIM_HandleTypeDef *encoder_tim;  // wheel encoder TIM
    TIM_HandleTypeDef *pwm_tim;  // wheel PWM TIM
    uint32_t pwm_channel_a;  // wheel PWM TIM channel a
    uint32_t pwm_channel_b;  // wheel PWM TIM channel b
    int speed;  // wheel speed
} Whell;

void whell_init(Whell *wheel, TIM_HandleTypeDef *encoder_tim, TIM_HandleTypeDef *pwm_tim, uint32_t pwm_channel_a, uint32_t pwm_channel_b);

int whell_get_speed(Whell *wheel);

void whell_set_speed(Whell *wheel, int speed);