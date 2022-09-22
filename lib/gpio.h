#ifndef _H_GPIO_
#define _H_GPIO_

#include "stm32f407xx.h"
#include "stm32f4xx_ll_gpio.h"

#define check                               LL_GPIO_TogglePin(GPIOD,LL_GPIO_PIN_15)

/*
 * Terminal UART pinout
 */
#define TERM_USART_TX_PORT                  GPIOB
#define TERM_USART_TX_PIN                   LL_GPIO_PIN_6
#define TERM_USART_RX_PORT                  GPIOB
#define TERM_USART_RX_PIN                   LL_GPIO_PIN_7
#define TERM_USART_PIN_AF                   LL_GPIO_AF_7
#define TERM_USART_OUTPUT_TYPE              LL_GPIO_OUTPUT_PUSHPULL

#define MOTOR_1_DIR_CW_PORT					GPIOC
#define MOTOR_1_DIR_CW_PIN                  LL_GPIO_PIN_0
#define MOTOR_1_DIR_CCW_PORT				GPIOC
#define MOTOR_1_DIR_CCW_PIN                 LL_GPIO_PIN_1

#define MOTOR_2_DIR_CW_PORT					GPIOC
#define MOTOR_2_DIR_CW_PIN                  LL_GPIO_PIN_2
#define MOTOR_2_DIR_CCW_PORT				GPIOC
#define MOTOR_2_DIR_CCW_PIN                 LL_GPIO_PIN_3

#define MOTOR_3_DIR_CW_PORT					GPIOC
#define MOTOR_3_DIR_CW_PIN                  LL_GPIO_PIN_4
#define MOTOR_3_DIR_CCW_PORT				GPIOC
#define MOTOR_3_DIR_CCW_PIN                 LL_GPIO_PIN_5

#define ENCODER_1_A_PORT					GPIOA
#define ENCODER_1_A_PIN                     LL_GPIO_PIN_15
#define ENCODER_1_B_PORT					GPIOB
#define ENCODER_1_B_PIN                     LL_GPIO_PIN_3
#define ENCODER_1_PIN_AF                    LL_GPIO_AF_1

#define ENCODER_2_A_PORT					GPIOD
#define ENCODER_2_A_PIN                     LL_GPIO_PIN_12
#define ENCODER_2_B_PORT					GPIOD
#define ENCODER_2_B_PIN                     LL_GPIO_PIN_13
#define ENCODER_2_PIN_AF                    LL_GPIO_AF_2

#define ENCODER_3_A_PORT					GPIOA
#define ENCODER_3_A_PIN                     LL_GPIO_PIN_0
#define ENCODER_3_B_PORT					GPIOA
#define ENCODER_3_B_PIN                     LL_GPIO_PIN_1
#define ENCODER_3_PIN_AF                    LL_GPIO_AF_2

#define PWM_1_PORT						    GPIOC
#define PWM_1_PIN                           LL_GPIO_PIN_6
#define PWM_1_PIN_AF                        LL_GPIO_AF_2

#define PWM_2_PORT						    GPIOC
#define PWM_2_PIN                           LL_GPIO_PIN_7
#define PWM_2_PIN_AF                        LL_GPIO_AF_2

#define PWM_3_PORT						    GPIOC
#define PWM_3_PIN                           LL_GPIO_PIN_8
#define PWM_3_PIN_AF                        LL_GPIO_AF_2

#endif
