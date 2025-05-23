//
// Created by ashkore on 25-4-18.
//
#include "pool.h"


uint8_t buf[128];
MPU6050_t mpu6050;
TF_Luna_Lidar TF_Luna_1;

int16_t  tfDist = 0 ;   // distance in centimeters
int16_t  tfFlux = 0 ;   // signal quality in arbitrary units
int16_t  tfTemp = 0 ;   // temperature in 0.01 degree Celsius

uint8_t uart1_rx_byte;                 // 当前接收的字节
char uart1_rx_buf[UART_RX_BUF_SIZE];   // 接收缓冲区
volatile uint16_t uart1_rx_index = 0;  // 当前缓冲区索引
volatile uint8_t uart1_rx_done = 0;    // 接收完成标志