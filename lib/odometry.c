#include <string.h>

#include <stdio.h>

#include "odometry.h"
#include "periph.h"
#include "gpio.h"

#include "task.h"

#include "arm_math.h"


static odometry_ctrl_t *odom_ctrl = NULL;

static inline float normalize_angle(float angle)
{
    if (angle > 2 * M_PI){
        return angle - 2 * M_PI;
    }
    if (angle < -2 * M_PI){
        return angle + 2 * M_PI;
    }
    return angle;
}

/*
 * Hardware initialization
 */
static void odom_hw_config(void)
{
    /*
     * Enable gpio and timers clocking
     */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    /*
     * Initialization first encoder 
     */
    
    LL_GPIO_SetPinMode(ENCODER_1_A_PORT, ENCODER_1_A_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(ENCODER_1_A_PORT, ENCODER_1_A_PIN,
                         ENCODER_1_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_1_A_PORT, ENCODER_1_A_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_GPIO_SetPinMode(ENCODER_1_B_PORT, ENCODER_1_B_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(ENCODER_1_B_PORT, ENCODER_1_B_PIN,
                          ENCODER_1_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_1_B_PORT, ENCODER_1_B_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_TIM_SetEncoderMode(ENCODER_1_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
    
    LL_TIM_IC_SetActiveInput(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_POLARITY_RISING);
    
    LL_TIM_IC_SetActiveInput(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_1_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_POLARITY_RISING);
    
    *(ENCODER_1_CNT) = ENCODER_TIM_CNT_INITIAL_VALUE;
    
    LL_TIM_EnableCounter(ENCODER_1_TIM_MODULE);
    
    /*
     * Initialization second encoder 
     */

    LL_GPIO_SetPinMode(ENCODER_2_A_PORT, ENCODER_2_A_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(ENCODER_2_A_PORT, ENCODER_2_A_PIN,
                         ENCODER_2_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_2_A_PORT, ENCODER_2_A_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_GPIO_SetPinMode(ENCODER_2_B_PORT, ENCODER_2_B_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(ENCODER_2_B_PORT, ENCODER_2_B_PIN,
                          ENCODER_2_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_2_B_PORT, ENCODER_2_B_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_TIM_SetEncoderMode(ENCODER_2_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
    
    LL_TIM_IC_SetActiveInput(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_POLARITY_RISING);
    
    LL_TIM_IC_SetActiveInput(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_2_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_POLARITY_RISING);
    
    *(ENCODER_2_CNT) = ENCODER_TIM_CNT_INITIAL_VALUE;
    
    LL_TIM_EnableCounter(ENCODER_2_TIM_MODULE);
    
    /*
     * Initialization third encoder 
     */

    LL_GPIO_SetPinMode(ENCODER_3_A_PORT, ENCODER_3_A_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(ENCODER_3_A_PORT, ENCODER_3_A_PIN,
                         ENCODER_3_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_3_A_PORT, ENCODER_3_A_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_GPIO_SetPinMode(ENCODER_3_B_PORT, ENCODER_3_B_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(ENCODER_3_B_PORT, ENCODER_3_B_PIN,
                          ENCODER_3_PIN_AF);
    LL_GPIO_SetPinOutputType(ENCODER_3_B_PORT, ENCODER_3_B_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
     
    LL_TIM_SetEncoderMode(ENCODER_3_TIM_MODULE, LL_TIM_ENCODERMODE_X4_TI12);
    
    LL_TIM_IC_SetActiveInput(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH1, 
                            LL_TIM_IC_POLARITY_RISING);
    
    LL_TIM_IC_SetActiveInput(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(ENCODER_3_TIM_MODULE, LL_TIM_CHANNEL_CH2, 
                            LL_TIM_IC_POLARITY_RISING);
    
    *(ENCODER_3_CNT) = ENCODER_TIM_CNT_INITIAL_VALUE;
    
    LL_TIM_EnableCounter(ENCODER_3_TIM_MODULE);

    /*
     * Initialisation odometry timer
     */ 

    LL_TIM_SetAutoReload(ODOMETRY_TIM_MODULE, ODOMETRY_TIM_ARR);
    LL_TIM_SetPrescaler(ODOMETRY_TIM_MODULE, ODOMETRY_TIM_PSC);
    LL_TIM_SetCounterMode(ODOMETRY_TIM_MODULE, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(ODOMETRY_TIM_MODULE);
    NVIC_SetPriority(ODOMETRY_IRQN, ODOMETRY_IRQN_PRIORITY);
    NVIC_EnableIRQ(ODOMETRY_IRQN);

    LL_TIM_EnableCounter(ODOMETRY_TIM_MODULE);

    return;
}

float pid_current_speed(int i)
{
    return odom_ctrl->wheel_speed[i];
}

/*
 * Read data from encoders and calculate wheels speed
 */

static void odom_calc_wheels_speeds(odometry_ctrl_t *odom_ctrl)
{
    int i = 0;
    for (i = 0; i < 3; i++) {
        odom_ctrl->delta_enc_ticks[i] = (int16_t)
                                        *(odom_ctrl->p_enc_ticks[i]) -
                                        ENCODER_TIM_CNT_INITIAL_VALUE;
        *(odom_ctrl->p_enc_ticks[i]) = ENCODER_TIM_CNT_INITIAL_VALUE;
        odom_ctrl->wheel_speed[i] =
        ((float) odom_ctrl->delta_enc_ticks[i] * TICKS_TO_RAD_S);
    }
    
    // TODO: check this!
    odom_ctrl->wheel_speed[2] = -odom_ctrl->wheel_speed[2];
    
    return;
}

static void odom_calc_robot_speed(odometry_ctrl_t *odom_ctrl)
{
    /*
     * Inverse kinematic matrix for calculation robot speed using wheels
     * speeds
     */
    static arm_matrix_instance_f32 m_inv_kin;
    static float inv_kin[9] = {ODOM_INV_KINEMATICS};
    /*
     * Input wheels speeds in robot's coordinate system
     */
    static arm_matrix_instance_f32 m_wheel_sp;
    static float wheel_sp[3] = {0.0f};
    wheel_sp[0] = odom_ctrl->wheel_speed[0];
    wheel_sp[1] = odom_ctrl->wheel_speed[1];
    wheel_sp[2] = odom_ctrl->wheel_speed[2];
    /*
     * Calculated robot speed
     */
    static arm_matrix_instance_f32 m_inst_speed;

    /*
     * Init arm_math matrice data structures
     */
    arm_mat_init_f32(&m_inv_kin, 3, 3, inv_kin);
    arm_mat_init_f32(&m_wheel_sp, 3, 1, wheel_sp);
    arm_mat_init_f32(&m_inst_speed, 3, 1, odom_ctrl->inst_local_speed);
    /*
     * Calculate instant robot speed
     */
    arm_mat_mult_f32(&m_inv_kin, &m_wheel_sp, &m_inst_speed);
    
    return;
}

static void odom_calc_glob_params(odometry_ctrl_t *odom_ctrl)
{
    /*
     * Rotation angle since last calculations
     */
    float rot_angle = odom_ctrl->coordinate[2] +
                      (odom_ctrl->inst_local_speed[2]) * 0.01f;
    rot_angle = normalize_angle(rot_angle);
    /*
     * Rotation matrix for ransformation of speed in robot coordinate
     * system to global one
     */
    static arm_matrix_instance_f32 m_rot_matrix;
    float rot_matrix[4] = {
            cosf(rot_angle), -sinf(rot_angle),
            sinf(rot_angle),  cosf(rot_angle)
    };
    /*
     * Robot instant global speed
     */
    static arm_matrix_instance_f32 m_inst_global_speed;
    /*
     * Robot instant local speed
     */
    static arm_matrix_instance_f32 m_inst_local_speed;

    /*
     * Init arm_math matrice data structures
     */
    arm_mat_init_f32(&m_rot_matrix, 2, 2, rot_matrix);
    arm_mat_init_f32(&m_inst_global_speed, 2, 1,
                     odom_ctrl->inst_global_speed);
    arm_mat_init_f32(&m_inst_local_speed, 2, 1,
                     odom_ctrl->inst_local_speed);
    /*
     * Robot global speed calculation
     */
    arm_mat_mult_f32(&m_rot_matrix, &m_inst_local_speed,
                     &m_inst_global_speed);
    odom_ctrl->inst_global_speed[2] = odom_ctrl->inst_local_speed[2];
    /*
     * Robot global coordinates calculation
     */
    
    odom_ctrl->coordinate[0] += odom_ctrl->inst_global_speed[0] * 0.01;
    odom_ctrl->coordinate[1] += odom_ctrl->inst_global_speed[1] * 0.01;

    odom_ctrl->coordinate[2] = rot_angle;
    
    return;
}

void odom(void *args)
{
    (void) args;
    odometry_ctrl_t odom_ctrl_st = {
        .curr_time = 0.0f,
        .prev_time = 0.0f,
        .coordinate = {0.0f, 0.0f, 0.0f},
        .inst_global_speed = {0.0f, 0.0f, 0.0f},
        .inst_local_speed = {0.0f, 0.0f, 0.0f},
        .wheel_speed = {0.0f, 0.0f, 0.0f},
        .p_enc_ticks = {ENCODER_1_CNT, ENCODER_2_CNT, ENCODER_3_CNT},
        .delta_enc_ticks = {0.0f, 0.0f, 0.0f}
    };
    odom_ctrl_st.odom_notify = xTaskGetCurrentTaskHandle();
    odom_ctrl = &odom_ctrl_st;
    
    odom_hw_config();
    
    while (1)
    {   
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /*
         * Calculate wheel speeds
         */
        odom_calc_wheels_speeds(&odom_ctrl_st);
        /*
         * Calculate robot instant local speed
         */
        odom_calc_robot_speed(&odom_ctrl_st);
        /*
         * Calculate robot instant global speed and coordinate
         */
        odom_calc_glob_params(&odom_ctrl_st);
    }
    return;
}

int cmd_get_wheel_speed(void *args)
{
    /*
     * Check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "ER", 3);
        return 3;
    }
    /*
     * Update odometry control structure
     */
    memcpy(args, odom_ctrl->wheel_speed, 12);
    return 12;
}

int cmd_get_speed(void *args)
{
    /*
     * Check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "ER", 3);
        return 3;
    }
    /*
     * Give robot instant local speed values
     */
    memcpy(args, odom_ctrl->inst_local_speed, 12);
    return 12;
}

int cmd_set_coord(void *args)
{
    cmd_set_coord_t *cmd_args = (cmd_set_coord_t *)args;

    /*
     * Check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "ER", 3);
        return 3;
    }
    /*
     * Update odometry control structure
     */
    odom_ctrl->coordinate[0] = cmd_args->x;
    odom_ctrl->coordinate[1] = cmd_args->y;
    odom_ctrl->coordinate[2] = cmd_args->alpha;
    memcpy(args, "OK", 3);
    return 3;
}


int cmd_get_coord(void *args)
{
    /*
     * Check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "ERROR", 6);
        return 6;
    }
    /*
     * Update odometry control structure
     */
    memcpy(args, odom_ctrl->coordinate, 12);
    return 12;
}

int cmd_set_xy(void *args)
{
    cmd_set_xy_t *cmd_args = (cmd_set_xy_t *)args;

    /*
     * check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "er", 3);
        return 3;
    }
    /*
     * update odometry control structure
     */
    odom_ctrl->coordinate[0] = cmd_args->x;
    odom_ctrl->coordinate[1] = cmd_args->y;
    memcpy(args, "ok", 3);
    return 3;
}

int cmd_set_theta(void *args)
{
    cmd_set_theta_t *cmd_args = (cmd_set_theta_t *)args;

    /*
     * check whether odometry is ready or not
     */
    if (!odom_ctrl){
        memcpy(args, "er", 3);
        return 3;
    }
    /*
     * update odometry control structure
     */
    odom_ctrl->coordinate[2] = cmd_args->theta;
    memcpy(args, "ok", 3);
    return 3;
}

void TIM6_DAC_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_TIM_IsActiveFlag_UPDATE(ODOMETRY_TIM_MODULE)) {
        LL_TIM_ClearFlag_UPDATE(ODOMETRY_TIM_MODULE);
        /*
         * Increment time in milliseconds
         */
        /*
         * Notify task
         */
        vTaskNotifyGiveFromISR(odom_ctrl->odom_notify,
                               &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
