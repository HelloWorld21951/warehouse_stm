#ifndef _ODOMETRY_H_
#define _ODOMETRY_H_

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/*
 * Encoder ticks to radians coefficient calculation
 */
// TODO: measure the robot size!
#define COUNT_PER_REV              2248.86f
#define RADIUS                     0.0295f
#define REV_METERS                 2 * M_PI * RADIUS
#define TICKS_TO_METER             REV_METERS / COUNT_PER_REV

#define DELTA_TIME                 0.01f
#define TICKS_TO_RAD_S             TICKS_TO_METER / \
                                   (DELTA_TIME * RADIUS)
/*
 * Set coordinates command structure
 */
typedef struct {
    float x;
    float y;
    float alpha;
} __attribute__((packed)) cmd_set_coord_t;

typedef struct {
    float v1;
    float v2;
    float v3;
} __attribute__((packed)) cmd_set_wheel_speed_t;

typedef struct {
    float x;
    float y;
} __attribute__((packed)) cmd_set_xy_t;

typedef struct {
    float theta;
} __attribute__((packed)) cmd_set_theta_t;
/*
 * Odometry control structure
 */
#define ODOMETRY_STACK_DEPTH    1024
StackType_t odometry_ts[ODOMETRY_STACK_DEPTH];
StaticTask_t odometry_tb;

#define DIST_TO_CENTER      0.1368f
// TODO: should be same value
#define RAD_WHEEL_X         0.01866561f
#define RAD_WHEEL_Y         0.01866561f    
#define RAD_FOR_THETA       0.0298f
/*
 * Inverse kinematics matrix
*/
#define ODOM_INV_KINEMATICS \
        -0.5f * RAD_WHEEL_X,                      -0.5f * RAD_WHEEL_X,                      RAD_WHEEL_X, \
        0.8660254f * RAD_WHEEL_Y,                 -0.8660254f * RAD_WHEEL_Y,                0.0f, \
        RAD_FOR_THETA / (3.0f * DIST_TO_CENTER),    RAD_FOR_THETA / (3.0f * DIST_TO_CENTER),    RAD_FOR_THETA / (3.0f * DIST_TO_CENTER)

/*
 * Main freertos task
 */
typedef struct {
    float curr_time;
    float prev_time;
    float coordinate[3];
    float inst_global_speed[3];
    float inst_local_speed[3];
    float wheel_speed[3];
    uint16_t *p_enc_ticks[3];
    int16_t delta_enc_ticks[3];
    TaskHandle_t odom_notify;
} odometry_ctrl_t;

void odom(void *args);
float pid_current_speed();

#endif
