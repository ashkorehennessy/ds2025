//
// Created by ashkore on 25-4-18.
//
#include "pool.h"


uint8_t buf[128];
MPU6050_t mpu6050;
int tfDist = 0;
int led_count = 0;
float angle_mix = 0;
float angle_sample[SAMPLE_SIZE];
float alpha_sample[SAMPLE_SIZE];
int task_index = 0;
int task1_count = 170;
float task2_result = -1;
int task2_fool = 0;
float task3_result = -1;
float task4_result = -1;
float task5_result = -1;
uint8_t uart1_rx_byte;                 // 当前接收的字节
char uart1_rx_buf[UART_RX_BUF_SIZE];   // 接收缓冲区
volatile uint16_t uart1_rx_index = 0;  // 当前缓冲区索引
volatile uint8_t uart1_rx_done = 0;    // 接收完成标志

void angle_sample_push(float angle){
  for (int i = SAMPLE_SIZE - 1; i > 0; i--) {
    angle_sample[i] = angle_sample[i - 1];
  }
  angle_sample[0] = angle;
}

void alpha_sample_push(float angle){
  for (int i = SAMPLE_SIZE - 1; i > 0; i--) {
    alpha_sample[i] = alpha_sample[i - 1];
  }
  alpha_sample[0] = angle;
}

float alpha_sample_avg(){
  float sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += alpha_sample[i];
  }
  return sum / SAMPLE_SIZE;
}

void process_uart1_buffer(void)
{
  for (uint16_t i = 0; i < uart1_rx_index; i++) {
    if (uart1_rx_buf[i] == 'r') {
      NVIC_SystemReset();
    }
    if (i+1 < uart1_rx_index && uart1_rx_buf[i] == 's') {
      switch (uart1_rx_buf[i+1]) {
        case '1':
          task_index = 1;
          mprintf("switch to task1\n");
          break;
        case '2':
          task_index = 2;
          mprintf("switch to task2\n");
          if(i+3 < uart1_rx_index && uart1_rx_buf[i+2] >= '0' && uart1_rx_buf[i+2] <= '9' && uart1_rx_buf[i+3] >= '0' && uart1_rx_buf[i+3] <= '9') {
            task2_fool = 1;
            task2_result = (uart1_rx_buf[i+2] - '0' * 10) + (uart1_rx_buf[i+3] - '0');
          } else {
            task2_fool = 0;
          }
          break;
        case '3':
          task_index = 3;
          mprintf("switch to task3\n");
          break;
        case '4':
          task_index = 4;
          mprintf("switch to task4\n");
          break;
        case '5':
          task_index = 5;
          mprintf("switch to task5\n");
          if(i+2 < uart1_rx_index && uart1_rx_buf[i+2] >= '0' && uart1_rx_buf[i+2] <= '9') {
            task5_result = uart1_rx_buf[i+2] - '0';
          }
          break;
        default:
          task_index = 0;
          mprintf("switch to task0\n");
          break;
      }
    }
  }
  uart1_rx_index = 0;
  uart1_rx_done = 0;
}
void detect_peaks_and_valleys() {
  float b0 = angle_sample[0];
  float b1 = angle_sample[2];
  float b2 = angle_sample[4];
  float b3 = angle_sample[6];
  float b4 = angle_sample[8];

  // 检测极小值（最低点）
  if (task_index == 2 && b2<5 && led_count <= 0 &&
    b0 > b1 && b1 > b2 && b2 < b3 && b3 < b4) {
    led_count = 100;
    task2_result = (float)(54 - tfDist) / 10.0f;
    return;
  }

  // 检测极大值（最高点）
  static uint32_t last_time = 0;
  if (task_index == 3 && b2>5 && led_count <= 0 &&
    b0 < b1 && b1 < b2 && b2 > b3 && b3 > b4) {
    led_count = 100;
    uint32_t current_time = HAL_GetTick();
    float period = (float)(current_time - last_time) / 1000.0f*2; // 转换为秒
    last_time = current_time;
    if (period < 2) {
      task3_result = period;
    }

    return;
  }
}

void detect_vertical() {
  static int dist_min = INT_MAX;
  if (tfDist < dist_min) {
    dist_min = tfDist;
  }
  if (task_index == 4 && tfDist <= dist_min+1) {
    alpha_sample_push(angle_sample[10]);
    task4_result = alpha_sample_avg();
    return;
  }
}