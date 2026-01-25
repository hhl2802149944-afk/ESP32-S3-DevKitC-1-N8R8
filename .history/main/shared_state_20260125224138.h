#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include "led_strip.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M_STOP = 0,
    M_FORWARD = 1,
    M_BACKWARD = 2
} MotorState;

extern MotorState motor1_state;
extern MotorState motor2_state;
extern int motor_step_delay;
extern led_strip_handle_t led_strip;

// 传感器数据
extern float current_lat;
extern float current_lon;
extern float pitch, roll, yaw;

#ifdef __cplusplus
}
#endif

#endif // SHARED_STATE_H
