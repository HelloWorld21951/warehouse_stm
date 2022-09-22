#ifndef _H_MOTOR_
#define _H_MOTOR_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define MOTOR_KIN_STACK_DEPTH    256

#define MK_LIN_KIN_MATRIX \
       -17.7777777777778f,   30.7920073802283f,    0.0f, \
       -17.7777777777778f,  -30.7920073802283f,    0.0f, \
        35.5555555555556f,   0.0f,                 0.0f

#define MK_ROT_KIN_MATRIX \
        0.0f,   0.0f,    4.59060375941986f,  \
        0.0f,   0.0f,    4.59060375941986f,  \
        0.0f,   0.0f,    4.59060375941986f


typedef struct {
  float curr_time;
  float prev_time;
  float p_p;     //П коэфициент
  float p_i;     //И коэфициент
  float p_d;     //Д коэфициент
  float max_sum_error; //Максимальная суммарная ошибка (что бы И регулятор не
                       //уходил до максимума если невозможно добиться требуемого
                       //значения)
  float max_output; //Максимальный выход, что бы поправка не выходила за рамки
  float min_output;
  float prev_error[3];
  float sum_error[3];
  float target_value[3];
} pid_struct;

#define PID_DELTA_TIME 0.04f

typedef struct {
    int status;
    float vel_x;
    float vel_y;
    float wz;
    TaskHandle_t mk_notify;
    SemaphoreHandle_t lock;
    float pwm_motors[3];
} motors_ctrl_t;

typedef struct {
        uint8_t channel;
        float pwm_value;
} __attribute__((packed)) cmd_set_pwm_t;

#define SET_PWM_ARGS_ERR(cmd_args) \
        ((cmd_args)->channel > 3) || ((cmd_args)->channel < 1) || \
        (fabsf((cmd_args)->pwm_value) < 0.1f) || \
        (fabsf((cmd_args)->pwm_value) > 1.0f)

/*
 * SET_SPEED command args structure
 */
typedef struct {
        float vx;
        float vy;
        float wz;
} __attribute__((packed)) cmd_set_speed_t;

typedef struct {
        float p;
        float i;
        float d;
} __attribute__((packed)) cmd_set_pid_coef_t;

TaskHandle_t pid_sem;

StackType_t motor_kinematics_ts[MOTOR_KIN_STACK_DEPTH];
StaticTask_t motor_kinematics_tb;

void motor_pwm(void *args);

#endif
