#include "gpio.h"
#include "periph.h"
#include "motor.h"
#include "odometry.h"

#include "math.h"
#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"

static motors_ctrl_t *mk_ctrl = NULL;

static pid_struct *pid_ctrl = NULL;

static void new_target(float *new_setpoint)
{
    for (int i = 0; i < 3; ++i)
    {
        pid_ctrl->target_value[i] = new_setpoint[i];
    }
}

static void mk_speed(motors_ctrl_t *mk_ctrl)
{
        static arm_matrix_instance_f32 m_fk_lin;
        static float fk_linear[9] = {MK_LIN_KIN_MATRIX};
        static arm_matrix_instance_f32 m_fk_rot;
        static float fk_rotational[9] = {MK_ROT_KIN_MATRIX};
        
        static arm_matrix_instance_f32 m_lin_sp;
        static float lin_sp[3] = {0.0f};
        static arm_matrix_instance_f32 m_rot_sp;
        static float rot_sp[3] = {0.0f};
        
        static arm_matrix_instance_f32 m_speed;
        static float speed[3] = {0.0f};
        
        static arm_matrix_instance_f32 m_input_sp;
        static float input_speed[3] = {0.0f};
        input_speed[0] = mk_ctrl->vel_x;
        input_speed[1] = mk_ctrl->vel_y;
        input_speed[2] = mk_ctrl->wz;

        arm_mat_init_f32(&m_fk_lin, 3, 3, fk_linear);
        arm_mat_init_f32(&m_fk_rot, 3, 3, fk_rotational);
        arm_mat_init_f32(&m_lin_sp, 3, 1, lin_sp);
        arm_mat_init_f32(&m_rot_sp, 3, 1, rot_sp);
        arm_mat_init_f32(&m_speed, 3, 1, speed);
        arm_mat_init_f32(&m_input_sp, 3, 1, input_speed);
        
        arm_mat_mult_f32(&m_fk_lin, &m_input_sp, &m_lin_sp);
        
        arm_mat_mult_f32(&m_fk_rot, &m_input_sp, &m_rot_sp);
        
        arm_mat_add_f32(&m_lin_sp, &m_rot_sp, &m_speed);
        
        new_target(speed);
        
        return;
}

static void pid_hw_config()
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
    LL_TIM_SetAutoReload(PID_TIM_MODULE, PID_TIM_ARR);
    LL_TIM_SetPrescaler(PID_TIM_MODULE, PID_TIM_PSC);
    LL_TIM_SetCounterMode(PID_TIM_MODULE, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_UPDATE(PID_TIM_MODULE);
    NVIC_SetPriority(PID_IRQN, PID_IRQN_PRIORITY);
    NVIC_EnableIRQ(PID_IRQN);
    
    LL_TIM_EnableCounter(PID_TIM_MODULE);
}

static void pwm_set (float *pwm_value)
{
    if (pwm_value[0] >= 0.0f)
    {
        LL_GPIO_SetOutputPin(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN);
        LL_GPIO_ResetOutputPin(MOTOR_1_DIR_CCW_PORT, MOTOR_1_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH1(MOTOR_1_TIM, pwm_value[0]);
    }

    else
    {
        LL_GPIO_ResetOutputPin(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN);
        LL_GPIO_SetOutputPin(MOTOR_1_DIR_CCW_PORT, MOTOR_1_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH1(MOTOR_1_TIM, -pwm_value[0]);
    }
    
    if (pwm_value[1] >= 0.0f)
    {
        LL_GPIO_SetOutputPin(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN);
        LL_GPIO_ResetOutputPin(MOTOR_2_DIR_CCW_PORT, MOTOR_2_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH2(MOTOR_2_TIM, pwm_value[1]);
    }
    
    else
    {
        LL_GPIO_ResetOutputPin(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN);
        LL_GPIO_SetOutputPin(MOTOR_2_DIR_CCW_PORT, MOTOR_2_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH2(MOTOR_2_TIM, -pwm_value[1]);
    }

    if (pwm_value[2] >= 0.0f)
    {
        LL_GPIO_SetOutputPin(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN);
        LL_GPIO_ResetOutputPin(MOTOR_3_DIR_CCW_PORT, MOTOR_3_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH3(MOTOR_3_TIM, pwm_value[2]);
        
    }

    else
    {
        LL_GPIO_ResetOutputPin(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN);
        LL_GPIO_SetOutputPin(MOTOR_3_DIR_CCW_PORT, MOTOR_3_DIR_CCW_PIN);
        LL_TIM_OC_SetCompareCH3(MOTOR_3_TIM, -pwm_value[2]);
    }
    return;
}

static void wheel_stop(uint8_t i)
{
    float wheels[3];
    wheels[i] = 0.0f;
    pwm_set(wheels);
}

static void pid_calc(float *target_new, pid_struct *pid_ctrl) 
{   
    float current_speed[3];
    for (int i = 0; i < 3; ++i)
    {
        current_speed[i] = pid_current_speed(i);
        if (target_new[i] == 0.0f)
        {
            mk_ctrl->pwm_motors[i] = 0.0f;
            wheel_stop(i);
            pid_ctrl->prev_error[i] = 0;
            pid_ctrl->sum_error[i] = 0;
        }
        else
        {
            float error, dif_error;
            error = target_new[i] - current_speed[i];
            dif_error = error - pid_ctrl->prev_error[i];
            pid_ctrl->prev_error[i] = error;
            pid_ctrl->sum_error[i] += error;

            if (pid_ctrl->sum_error[i] > pid_ctrl->max_sum_error)
                pid_ctrl->sum_error[i] = pid_ctrl->max_sum_error;
            if (pid_ctrl->sum_error[i] < -pid_ctrl->max_sum_error)
                pid_ctrl->sum_error[i] = -pid_ctrl->max_sum_error;
            float _integral = (pid_ctrl->p_i * pid_ctrl->sum_error[i]);
            mk_ctrl->pwm_motors[i]  = ((float) (pid_ctrl->p_p * error) + _integral +
                                   (pid_ctrl->p_d * dif_error));


            if (mk_ctrl->pwm_motors[i] > pid_ctrl->max_output)
                mk_ctrl->pwm_motors[i] = pid_ctrl->max_output;
            else if (mk_ctrl->pwm_motors[i] < -pid_ctrl->max_output)
                mk_ctrl->pwm_motors[i] = -pid_ctrl->max_output;

            if (mk_ctrl->pwm_motors[i] < pid_ctrl->min_output &&
                mk_ctrl->pwm_motors[i] > -pid_ctrl->min_output)
            {
                mk_ctrl->pwm_motors[i] = 0;
                pid_ctrl->prev_error[i] = 0;
                pid_ctrl->sum_error[i] = 0;

            }
        }
    }
    pwm_set(mk_ctrl->pwm_motors);
}

static void mk_hw_config()
{
    /* Init motor_kinematics pins */
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    /* Config direction pins */
    
    LL_GPIO_SetPinMode(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN,
                       LL_GPIO_PULL_NO);
    
    LL_GPIO_SetPinMode(MOTOR_1_DIR_CCW_PORT, MOTOR_1_DIR_CCW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_1_DIR_CCW_PORT, MOTOR_1_DIR_CCW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_1_DIR_CCW_PORT, MOTOR_1_DIR_CCW_PIN,
                       LL_GPIO_PULL_NO);
    
    LL_GPIO_SetPinMode(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN,
                       LL_GPIO_PULL_NO);
    
    LL_GPIO_SetPinMode(MOTOR_2_DIR_CCW_PORT, MOTOR_2_DIR_CCW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_2_DIR_CCW_PORT, MOTOR_2_DIR_CCW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_2_DIR_CCW_PORT, MOTOR_2_DIR_CCW_PIN,
                       LL_GPIO_PULL_NO);
    
    LL_GPIO_SetPinMode(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN,
                       LL_GPIO_PULL_NO);
    
    LL_GPIO_SetPinMode(MOTOR_3_DIR_CCW_PORT, MOTOR_3_DIR_CCW_PIN,
                       LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(MOTOR_3_DIR_CCW_PORT, MOTOR_3_DIR_CCW_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(MOTOR_3_DIR_CCW_PORT, MOTOR_3_DIR_CCW_PIN,
                       LL_GPIO_PULL_NO);

    /* Config PWM pins */
    
    LL_GPIO_SetPinMode(PWM_1_PORT, PWM_1_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(PWM_1_PORT, PWM_1_PIN,
                          PWM_1_PIN_AF); 
    LL_GPIO_SetPinOutputType(PWM_1_PORT, PWM_1_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_GPIO_SetPinMode(PWM_2_PORT, PWM_2_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(PWM_2_PORT, PWM_2_PIN,
                          PWM_2_PIN_AF); 
    LL_GPIO_SetPinOutputType(PWM_2_PORT, PWM_2_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    LL_GPIO_SetPinMode(PWM_3_PORT, PWM_3_PIN,
                       LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(PWM_3_PORT, PWM_3_PIN,
                          PWM_3_PIN_AF); 
    LL_GPIO_SetPinOutputType(PWM_3_PORT, PWM_3_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    
    /* Init timer in PWM mode */
    
    LL_TIM_EnableUpdateEvent(MOTOR_1_TIM);
    LL_TIM_EnableUpdateEvent(MOTOR_2_TIM);
    LL_TIM_EnableUpdateEvent(MOTOR_3_TIM);
    
    LL_TIM_SetClockDivision(MOTOR_1_TIM, LL_TIM_CLOCKDIVISION_DIV4); 
    LL_TIM_SetClockDivision(MOTOR_2_TIM, LL_TIM_CLOCKDIVISION_DIV4); 
    LL_TIM_SetClockDivision(MOTOR_3_TIM, LL_TIM_CLOCKDIVISION_DIV4); 
    
    LL_TIM_SetCounterMode(MOTOR_1_TIM, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetCounterMode(MOTOR_2_TIM, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetCounterMode(MOTOR_3_TIM, LL_TIM_COUNTERMODE_UP);
    
    LL_TIM_SetAutoReload(MOTOR_1_TIM, MOTOR_1_PWM_TIM_ARR);
    LL_TIM_SetAutoReload(MOTOR_2_TIM, MOTOR_2_PWM_TIM_ARR);
    LL_TIM_SetAutoReload(MOTOR_3_TIM, MOTOR_3_PWM_TIM_ARR);
    
    LL_TIM_SetUpdateSource(MOTOR_1_TIM, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_SetUpdateSource(MOTOR_2_TIM, LL_TIM_UPDATESOURCE_REGULAR);
    LL_TIM_SetUpdateSource(MOTOR_3_TIM, LL_TIM_UPDATESOURCE_REGULAR);

    /* Enable capture mode */
    
    LL_TIM_CC_EnableChannel(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL);
    LL_TIM_CC_EnableChannel(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
    LL_TIM_CC_EnableChannel(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
    
    /* Set PWM mode */
    
    LL_TIM_OC_SetMode(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_SetMode(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL, LL_TIM_OCMODE_PWM1);
    
    /* Enable fast mode */
    
    LL_TIM_OC_EnableFast(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL);
    LL_TIM_OC_EnableFast(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
    LL_TIM_OC_EnableFast(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
    
    /* Enable preload */
     
    LL_TIM_OC_EnablePreload(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL);
    LL_TIM_OC_EnablePreload(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
    LL_TIM_OC_EnablePreload(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
    
    LL_TIM_EnableARRPreload(MOTOR_1_TIM);
    LL_TIM_EnableARRPreload(MOTOR_2_TIM);
    LL_TIM_EnableARRPreload(MOTOR_3_TIM);
    
    /* Set initial value */
    
    LL_TIM_OC_SetCompareCH1(MOTOR_1_TIM, MOTOR_2_PWM_TIM_CCR_INIT);
    LL_TIM_OC_SetCompareCH2(MOTOR_2_TIM, MOTOR_2_PWM_TIM_CCR_INIT);
    LL_TIM_OC_SetCompareCH3(MOTOR_3_TIM, MOTOR_3_PWM_TIM_CCR_INIT);

    /* Enable timer */
    
    LL_TIM_GenerateEvent_UPDATE(MOTOR_1_TIM);
    LL_TIM_GenerateEvent_UPDATE(MOTOR_2_TIM);
    LL_TIM_GenerateEvent_UPDATE(MOTOR_3_TIM);
    
    LL_TIM_EnableCounter(MOTOR_1_TIM);
    LL_TIM_EnableCounter(MOTOR_2_TIM);
    LL_TIM_EnableCounter(MOTOR_3_TIM);

    /* Reset CW direction pins */
    
    LL_GPIO_ResetOutputPin(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CW_PIN);
    LL_GPIO_ResetOutputPin(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CW_PIN);
    LL_GPIO_ResetOutputPin(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CW_PIN);
    
    /* Reset CCW direction pins */
    
    LL_GPIO_ResetOutputPin(MOTOR_1_DIR_CW_PORT, MOTOR_1_DIR_CCW_PIN);
    LL_GPIO_ResetOutputPin(MOTOR_2_DIR_CW_PORT, MOTOR_2_DIR_CCW_PIN);
    LL_GPIO_ResetOutputPin(MOTOR_3_DIR_CW_PORT, MOTOR_3_DIR_CCW_PIN);
    
    return;
}

void motor_pwm (void *args)
{
    (void) args;
       
    motors_ctrl_t mk_ctrl_st = {
        .status = 0x00,
        .vel_x = 0.0f,
        .vel_y = 0.0f,
        .wz = 0.0f,
        .pwm_motors = {0.1f, 0.1f, 0.1f}
    };

    pid_struct pid_ctrl_st = {
        .p_p = 70.0f,
        .p_i = 20.0f,
        .p_d = 0.01f,
        .max_sum_error = 1000000000000.0,
        .max_output = 4200.0f,
        .min_output = -4200.0f
    };

    mk_ctrl->mk_notify = xTaskGetCurrentTaskHandle();
    mk_hw_config();
    mk_ctrl = &mk_ctrl_st;

    pid_sem = xTaskGetCurrentTaskHandle();
    pid_hw_config();
    pid_ctrl = &pid_ctrl_st;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        pid_calc(pid_ctrl->target_value, &pid_ctrl_st);
    }  
}

int cmd_set_pwm(void *args)
{
    cmd_set_pwm_t *cmd_args = (cmd_set_pwm_t *)args;

    if (!mk_ctrl || SET_PWM_ARGS_ERR(cmd_args)){
        memcpy(args, "ER", 3);
        return 3;
    }
    
    xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
    pwm_set(mk_ctrl->pwm_motors);
    mk_ctrl->pwm_motors[cmd_args->channel - 1] = cmd_args->pwm_value;
    xSemaphoreGive(mk_ctrl->lock);
    
    xTaskNotifyGive(mk_ctrl->mk_notify);
    memcpy(args, "OK", 3);
    return 3;
}

int cmd_set_pid_coef(void *args)
{
    cmd_set_pid_coef_t *cmd_args = (cmd_set_pid_coef_t *)args;
    if (!pid_ctrl){
        memcpy(args, "ER", 3);
        return 3;
    }
    xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
    memcpy(&(pid_ctrl->p_p), &(cmd_args->p), 12);
    xSemaphoreGive(mk_ctrl->lock);
    xTaskNotifyGive(mk_ctrl->mk_notify);
    memcpy(args, "OK", 3);
    return 3;
}

int cmd_get_pid_output(void *args)
{
    if (!mk_ctrl){
        memcpy(args, "ERROR", 6);
        return 6;
    }
    memcpy(args, mk_ctrl->pwm_motors , 12);
    return 12;
}

int cmd_get_pid_setpoint(void *args)
{
    if (!pid_ctrl){
        memcpy(args, "ERROR", 6);
        return 6;
    }
    memcpy(args, pid_ctrl->target_value, 12);
    return 12;
}

int cmd_set_speed(void *args)
{
    cmd_set_speed_t *cmd_args = (cmd_set_speed_t *)args;
    if (!mk_ctrl){
        memcpy(args, "ER", 3);
        return 3;
    }
    xSemaphoreTake(mk_ctrl->lock, portMAX_DELAY);
    memcpy(&(mk_ctrl->vel_x), &(cmd_args->vx), 12);
    xSemaphoreGive(mk_ctrl->lock);
    xTaskNotifyGive(mk_ctrl->mk_notify);
    mk_speed(mk_ctrl);
    memcpy(args, "OK", 3);
    return 3;
}

void TIM7_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_TIM_IsActiveFlag_UPDATE(PID_TIM_MODULE)) {
            LL_TIM_ClearFlag_UPDATE(PID_TIM_MODULE);
            
            vTaskNotifyGiveFromISR(pid_sem,
                                   &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

