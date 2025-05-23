//
// Created by ashkore on 25-4-18.
//
#include "pool.h"


uint8_t buf[128];
MPU6050_t mpu6050;
TF_Luna_Lidar TF_Luna_1;
int16_t  tfDist = 0;
int16_t  tfFlux = 0;
int16_t  tfTemp = 0;
int led_count = 0;
float angle_mix = 0;
float angle_sample[SAMPLE_SIZE];
int task_index = 0;
int task1_count = 0;
float task2_result = 0;
int task2_fool = 0;
float task5_result = 0;
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