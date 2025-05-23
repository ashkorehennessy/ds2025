//
// Created by ashkore on 25-4-18.
//

#ifndef POOL_H
#define POOL_H
#include <stdint.h>
#include "mpu6050.h"
#include <stdlib.h>
#include "string.h"
#include "stdio.h"
#include "usart.h"
#include "tgmath.h"
#include "limits.h"
#include "VL53L0X.h"
#define mprintf(fmt, ...)                   \
    do {                                    \
        memset(buf,0,sizeof(buf));           \
        sprintf((char*)buf, fmt, ##__VA_ARGS__);              \
        HAL_UART_Transmit(&huart1, buf, strlen(buf), 1000); \
    } while(0)
#define UART_RX_BUF_SIZE 128
#define UART_RX_END_CHAR '\n'
#define SAMPLE_SIZE 20
extern uint8_t buf[128];
extern int led_count;
extern MPU6050_t mpu6050;
extern int tfDist;
extern float angle_mix;
extern float angle_sample[SAMPLE_SIZE];
extern float alpha_sample[SAMPLE_SIZE];
extern int task_index;
extern int task1_count;
extern float task2_result;
extern int task2_fool;
extern float task3_result;
extern float task4_result;
extern float task5_result;
extern uint8_t uart1_rx_byte;                 // 当前接收的字节
extern char uart1_rx_buf[UART_RX_BUF_SIZE];   // 接收缓冲区
extern volatile uint16_t uart1_rx_index;  // 当前缓冲区索引
extern volatile uint8_t uart1_rx_done;    // 接收完成标志
void angle_sample_push(float angle);
void tfdist_sample_push(int dist);
float alpha_sample_avg();
void process_uart1_buffer(void);
void detect_peaks_and_valleys();
void detect_vertical();
#endif //POOL_H
