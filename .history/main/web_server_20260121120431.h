#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "led_strip.h"

// 电机状态枚举（与 main.cpp 保持一致）
enum MotorState { M_STOP, M_FORWARD, M_BACKWARD };

// 外部引用的状态变量
extern MotorState motor1_state;
extern MotorState motor2_state;
extern led_strip_handle_t led_strip;

// 函数声明
void start_webserver();

#endif
