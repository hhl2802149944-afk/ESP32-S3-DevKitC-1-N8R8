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
extern MotorState motor3_state;  // 摄像头电机
extern int motor_step_delay;
extern led_strip_handle_t led_strip;

#ifdef __cplusplus
}
#endif

#endif // SHARED_STATE_H
