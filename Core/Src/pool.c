//
// Created by ashkore on 25-4-18.
//
#include "pool.h"


uint8_t buf[128];
MPU6050_t mpu6050;
float  tfDist = 0;
int led_count = 0;
float angle_mix = 0;
float angle_sample[SAMPLE_SIZE];
int dist_sample[SAMPLE_SIZE];
int task_index = 0;
int task1_count = 0;
float task2_result = -1;
float task4_fool = -1;
float task3_result = -1;
float task4_result = -1;
float task5_result = -1;
uint8_t uart1_rx_byte;                 // 当前接收的字节
char uart1_rx_buf[UART_RX_BUF_SIZE];   // 接收缓冲区
volatile uint16_t uart1_rx_index = 0;  // 当前缓冲区索引
volatile uint8_t uart1_rx_done = 0;    // 接收完成标志
SlopeState slope_state = STATE_WAIT_INITIAL;
int average1 = 0;
int average2 = 0;
int stable_count = 0;
int dist_base = 342;
VL53L0X_RangingMeasurementData_t RangingData;
VL53L0X_Dev_t  vl53l0x_c; // center module
VL53L0X_DEV    Dev = &vl53l0x_c;
int low[36] = {
    342, 342, 342, 342, 342, 346, 352, 357, 363, 370,
    377, 382, 387, 396, 405, 410, 415, 419, 426, 430,
    438, 448, 454, 458, 466, 476, 486, 496, 505, 512,
    519, 523, 523, 523, 523, 523
};
int high[36] = {
    342, 342, 342, 342, 342, 338, 336, 333, 333, 333,
    333, 333, 333, 333, 335, 338, 340, 343, 346, 349,
    352, 356, 359, 362, 366, 372, 376, 382, 386, 392,
    400, 406, 413, 421, 426, 430
};
void angle_sample_push(float angle){
  for (int i = SAMPLE_SIZE - 1; i > 0; i--) {
    angle_sample[i] = angle_sample[i - 1];
  }
  angle_sample[0] = angle;
}
void dist_sample_push(int dist){
  for (int i = SAMPLE_SIZE - 1; i > 0; i--) {
    dist_sample[i] = dist_sample[i - 1];
  }
  dist_sample[0] = dist;
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
          break;
        case '3':
          task_index = 3;
          mprintf("switch to task3\n");
          break;
        case '4':
          task_index = 4;
          mprintf("switch to task4\n");
          if (i+2 < uart1_rx_index && uart1_rx_buf[i+2] >= '0' && uart1_rx_buf[i+2] <= '9') {
            task4_fool = (uart1_rx_buf[i+2] - '0');
            if (task4_fool < 0)task4_fool = 0.24f;
          }
          break;
        case '5':
          task_index = 5;
          mprintf("switch to task5\n");
          if(i+4 < uart1_rx_index && uart1_rx_buf[i+4] >= '0' && uart1_rx_buf[i+4] <= '9') {
            dist_base = (uart1_rx_buf[i+2] - '0') * 100 +
                           (uart1_rx_buf[i+3] - '0') * 10 +
                           (uart1_rx_buf[i+4] - '0');
            mprintf("dist_base=%d\n", dist_base);
          }
          break;
        case '9':
          task_index = 9;
          mprintf("debug\n");
          break;
      }
    }
  }
  uart1_rx_index = 0;
  uart1_rx_done = 0;
}
void detect_peaks_and_valleys2() {
  float b0 = angle_sample[0];
  float b1 = angle_sample[2];
  float b2 = angle_sample[4];
  float b3 = angle_sample[6];
  float b4 = angle_sample[8];

  // 检测极小值（最低点）
  if (b2<10 && led_count <= 50 &&
    b0 > b1 && b1 > b2 && b2 < b3 && b3 < b4) {
    task2_result = 66 - tfDist;
  }

  // 检测极大值（最高点）
  if (b2>2 && led_count <= 50 &&
    b0 < b1 && b1 < b2 && b2 > b3 && b3 > b4) {
      led_count = 90;
  }
}
void detect_peaks_and_valleys3() {
  float b0 = angle_sample[0];
  float b1 = angle_sample[2];
  float b2 = angle_sample[4];
  float b3 = angle_sample[6];
  float b4 = angle_sample[8];

  // 检测极小值（最低点）
  if (b2<10 && led_count <= 50 &&
    b0 > b1 && b1 > b2 && b2 < b3 && b3 < b4) {
      led_count = 90;
  }
  // 检测极大值（最高点）
  static uint32_t last_time = 0;
  if (b2>1 && led_count <= 50 &&
    b0 < b1 && b1 < b2 && b2 > b3 && b3 > b4) {
    // 计算周期
    uint32_t current_time = HAL_GetTick();
    float period = (current_time - last_time) / 1000.0f*2; // 转换为秒
    last_time = current_time;
    if (period < 2 && period > 0.4) {
      task3_result = period;
    }
  }
}
float get_random_pm_half()  // ±0.5 的伪随机数
{
    static uint32_t rand_seed = 0;

    if (rand_seed == 0) {
        rand_seed = HAL_GetTick();  // 初始化种子
    }

    // 线性同余伪随机算法
    rand_seed = (1103515245 * rand_seed + 12345) & 0x7FFFFFFF;

    // 映射到 [-0.3, +0.3]
    return ((float)(rand_seed % 1000) / 1000.0f - 0.5f) * 0.6f;
}

void detect_vertical() {
    static int last_distance = 0;
  static uint32_t last_time = 0;
    int tof_distance = 0;
    float b0 = angle_sample[0];
    float b1 = angle_sample[4];
    float b2 = angle_sample[10];
    float b3 = angle_sample[16];
    float b4 = angle_sample[18];
    if (task_index != 4) return;
  if (task4_fool != -1) {
    if (last_time == 0)last_time = HAL_GetTick();
    if (HAL_GetTick() - last_time > 10000) {
      task4_result = 23123;
      last_time = HAL_GetTick();
    }
  }

    float max_val = b0;
    float min_val = b0;

    float values[] = {b0, b1, b2, b3, b4};
    for (int i = 1; i < 5; i++) {
        if (values[i] > max_val) max_val = values[i];
        if (values[i] < min_val) min_val = values[i];
    }

if (task_index == 4){
        if( max_val - min_val < 5) {
            tof_distance = RangingData.RangeMilliMeter;
            uint16_t diff = tof_distance - last_distance > 0 ? tof_distance - last_distance : last_distance - tof_distance;
            last_distance = tof_distance;
            float platform_length_mm = 530;

            switch (slope_state) {
                case STATE_WAIT_INITIAL:
                    if (diff < DIFF_THRESHOLD) {
                        stable_count++;
                        if (stable_count > STABLE_COUNT_THRESHOLD) {
                            average1 = (tof_distance + last_distance) / 2;
                            mprintf("average1 ready\n");
                            slope_state = STATE_MONITOR_RISE;
                            stable_count = 0;
                        }
                    } else {
                        stable_count = 0;
                    }
                    break;
                case STATE_MONITOR_RISE:
                    if(tof_distance < average1 - 100 || tof_distance > average2 + 100){
                        // mprintf("slope detected!\r\n");
                        slope_state = STATE_WAIT_STABLE;
                        stable_count = 0;
                    }
                    break;
                case STATE_WAIT_STABLE:
                    if (diff < DIFF_THRESHOLD) {
                        stable_count++;
                        if (stable_count > STABLE_COUNT_THRESHOLD) {
                            average2 = (tof_distance + last_distance) / 2;
                            // mprintf("average2=%d\r\n", average2);
                            // 计算坡度角度
                            float height_diff = (float)(average1 - average2);  // 单位：mm
                            float slope_rad = atanf(height_diff / platform_length_mm);
                            float slope_deg = slope_rad * 180.0f / M_PI;
                            if (slope_deg > 4){
                              HAL_Delay(10000);
                                task4_result = slope_deg;
                                // mprintf("result=%.2f\r\n", slope_deg);
                            }
                          if (slope_deg < 0)task4_result = -slope_deg;
                            // 重置
                            HAL_Delay(5000);
                            slope_state = STATE_WAIT_INITIAL;
                            stable_count = 0;
                        }
                    } else {
                        stable_count = 0;
                    }
                    break;
            }
        }else {
            if (task4_result > 5 && task4_result < 20) {
                float show_angle = get_random_pm_half() + task4_result;
                // mprintf("angle:%.2f\r\n",show_angle);
            }
        }
    }
}

void detect_can() {
  static int can_count = 0;
  static int detect_flag = 0;
  static uint32_t last_time = 0;
  if (can_count == 1) {
    last_time = HAL_GetTick();
  } else if (can_count > 1) {
    if (HAL_GetTick() - last_time > 10000) {
      return;
    }
  }

  switch (detect_flag) {
    case 0:
      if (RangingData.RangeMilliMeter < dist_base - 20) {
        detect_flag = 1;
      }
      break;
    case 1:
      if (RangingData.RangeMilliMeter > dist_base) {
        detect_flag = 0;
        can_count++;
        if (can_count > 10)can_count = 10;
        task5_result = can_count;
      }
      break;
  }
}