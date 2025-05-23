//
// Created by ashkore on 25-4-18.
//

#ifndef POOL_H
#define POOL_H
#include <stdint.h>
#include "mpu6050.h"
#include "tfluna_i2c.h"

#define UART_RX_BUF_SIZE 128
#define UART_RX_END_CHAR '\n'
extern uint8_t buf[128];
extern TF_Luna_Lidar TF_Luna_1;

extern MPU6050_t mpu6050;
extern int16_t  tfDist;
extern int16_t  tfFlux;
extern int16_t  tfTemp;


extern uint8_t uart1_rx_byte;                 // 当前接收的字节
extern char uart1_rx_buf[UART_RX_BUF_SIZE];   // 接收缓冲区
extern volatile uint16_t uart1_rx_index;  // 当前缓冲区索引
extern volatile uint8_t uart1_rx_done;    // 接收完成标志
#endif //POOL_H
