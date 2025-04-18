//
// Created by ashkore on 23-7-25.
//

#include "wheel.h"

void whell_init(Whell *wheel, TIM_HandleTypeDef *encoder_tim, TIM_HandleTypeDef *pwm_tim, uint32_t pwm_channel_a, uint32_t pwm_channel_b) {
  wheel->encoder_tim = encoder_tim;
  wheel->pwm_tim = pwm_tim;
  wheel->pwm_channel_a = pwm_channel_a;
  wheel->pwm_channel_b = pwm_channel_b;
  wheel->speed = 0;
}

int whell_get_speed(Whell *wheel) {
  int speed = __HAL_TIM_GET_COUNTER(wheel->encoder_tim);
  if(speed > 32767) {
    speed -= 65536;  // If speed is negative, make it positive
  }
  wheel->speed = speed;
  __HAL_TIM_SET_COUNTER(wheel->encoder_tim, 0);
  return speed;
}

void whell_set_speed(Whell *wheel, int speed) {
  if (speed > 0) {
    __HAL_TIM_SET_COMPARE(wheel->pwm_tim, wheel->pwm_channel_a, speed);
    __HAL_TIM_SET_COMPARE(wheel->pwm_tim, wheel->pwm_channel_b, 0);
  } else {
    __HAL_TIM_SET_COMPARE(wheel->pwm_tim, wheel->pwm_channel_a, 0);
    __HAL_TIM_SET_COMPARE(wheel->pwm_tim, wheel->pwm_channel_b, -speed);
  }
}