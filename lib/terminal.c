#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f4xx_ll_usart.h"
#include "periph.h"
#include "gpio.h"
#include "terminal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "terminal_cmds.h"

/*
 * Private task notifier
 */
static terminal_task_t * term_ctrl;

static void terminal_hw_config()
{
         /* Init terminal pins */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

        LL_GPIO_SetAFPin_0_7(TERM_USART_TX_PORT, TERM_USART_TX_PIN,
                             TERM_USART_PIN_AF);
        LL_GPIO_SetPinMode(TERM_USART_TX_PORT, TERM_USART_TX_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinOutputType(TERM_USART_TX_PORT, TERM_USART_TX_PIN,
                                 TERM_USART_OUTPUT_TYPE);
        LL_GPIO_SetPinPull(TERM_USART_TX_PORT, TERM_USART_TX_PIN,
                           LL_GPIO_PULL_NO);
        LL_GPIO_SetPinSpeed(TERM_USART_TX_PORT, TERM_USART_TX_PIN,
                            LL_GPIO_SPEED_FREQ_HIGH);

        LL_GPIO_SetAFPin_0_7(TERM_USART_RX_PORT, TERM_USART_RX_PIN,
                             TERM_USART_PIN_AF);
        LL_GPIO_SetPinMode(TERM_USART_RX_PORT, TERM_USART_RX_PIN,
                           LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetPinOutputType(TERM_USART_RX_PORT, TERM_USART_RX_PIN,
                                 TERM_USART_OUTPUT_TYPE);
        LL_GPIO_SetPinPull(TERM_USART_RX_PORT, TERM_USART_RX_PIN,
                           LL_GPIO_PULL_NO);
        LL_GPIO_SetPinSpeed(TERM_USART_RX_PORT, TERM_USART_RX_PIN,
                            LL_GPIO_SPEED_FREQ_HIGH);

        /* Enable clocking on USART and DMA */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

        /* UART configuration */
        LL_USART_SetTransferDirection(TERM_USART,
                                      TERM_USART_TRANSFER_DIRECTION);
        LL_USART_SetParity(TERM_USART, TERM_USART_PARITY);
        LL_USART_SetDataWidth(TERM_USART, TERM_USART_DATA_WIDTH);
        LL_USART_SetStopBitsLength(TERM_USART, TERM_USART_STOPBITS);
        LL_USART_SetHWFlowCtrl(TERM_USART, TERM_USART_HARDWARE_FLOAT_CNTRL);
        LL_USART_SetBaudRate(TERM_USART,
                             SystemCoreClock/TERM_USART_PERIPH_PRESCALER,
                             TERM_USART_OVERSAMPL, TERM_USART_BAUDRATE);
        LL_USART_EnableDirectionRx(TERM_USART);
        LL_USART_EnableDirectionTx(TERM_USART);
        LL_USART_EnableDMAReq_RX(TERM_USART);
        LL_USART_EnableIT_IDLE(TERM_USART);
        LL_USART_Enable(TERM_USART);

        NVIC_SetPriority(TERM_USART_IRQN, TERM_USART_IRQN_PRIORITY);
        NVIC_EnableIRQ(TERM_USART_IRQN);

        /* DMA configuration */
        LL_DMA_SetChannelSelection(TERM_DMA, TERM_DMA_STREAM,
                                   TERM_DMA_CHANNEL);
        LL_DMA_ConfigAddresses(TERM_DMA, TERM_DMA_STREAM, TERM_DMA_SRC_ADDR,
                               (uint32_t)term_ctrl->buffer, TERM_DMA_DIRECTION);
        LL_DMA_SetDataLength(TERM_DMA, TERM_DMA_STREAM, TERM_CH_BUF_SIZE);
        LL_DMA_SetMemoryIncMode(TERM_DMA, TERM_DMA_STREAM,
                                TERM_DMA_MEM_INC_MODE);

        LL_DMA_EnableStream(TERM_DMA, TERM_DMA_STREAM);
        LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_STREAM);

        /* Enable global DMA stream interrupts */
        NVIC_SetPriority(TERM_DMA_STREAM_IRQN, TERM_DMA_STREAM_IRQN_PRIORITY);
        NVIC_EnableIRQ(TERM_DMA_STREAM_IRQN);

        return;
}

void merge(char lb, char rb, char * byte) {
    (*byte) = (lb & 0b00001111) | (rb & 0b11110000);
}

void decode(char * from, char * to, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        char r;
        merge(*(from), *(from + 1), &r);
        (*to) = r;
        ++to;
        from += 2;
    }
}


static void term_response(terminal_task_t *term_t, int resp_len)
{
        int i = 0;

        LL_USART_ClearFlag_TC(term_t->dev);
        while (resp_len--) {
                while (!LL_USART_IsActiveFlag_TXE(term_t->dev))
                        taskYIELD();
                LL_USART_TransmitData8(term_t->dev, term_t->com_args[i++]);
        }
        while (!LL_USART_IsActiveFlag_TC(term_t->dev))
                taskYIELD();
        return;
}

static int term_request(terminal_task_t *term_t)
{
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {  
                decode(term_t->buffer, term_t->buffer2, TERM_CH_BUF_SIZE / 2);
                term_t->com_args = &(term_t->buffer2[1]);
                return (int)(term_t->buffer2[0]);
        }
        return 0;
}
void terminal_manager(void *arg)
{
        (void) arg;

        int command_code = 0;
        int resp_len = 0;
        terminal_task_t term_t;

        term_t.dev = TERM_USART;
        term_t.buffer = malloc(TERM_CH_BUF_SIZE);
        term_t.buffer2 = malloc(TERM_CH_BUF_SIZE / 2);
        term_t.com_args = malloc(TERM_ARGS_BUF_SIZE);
        term_t.com_resp = term_t.com_args;
        term_t.xTaskToNotify = xTaskGetCurrentTaskHandle();
        term_ctrl = &term_t;
        terminal_hw_config();

        while (1) {
                command_code = term_request(&term_t);
                if (!IS_COMMAND_VALID(command_code) ||
                    !commands_handlers[command_code]) {
                        term_t.com_args[0] = 'a';
                        term_t.com_args[1] = 'b';
                        term_t.com_args[2] = 'c';
                        term_response(&term_t, 3);
                        continue;
                }
                resp_len = commands_handlers[command_code](term_t.com_args);
                term_response(&term_t, resp_len);
        }
        return;
}

/*
 * Hardware interrupts
 */
void USART1_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        LL_USART_ClearFlag_IDLE(TERM_USART);
        LL_DMA_DisableStream(TERM_DMA, TERM_DMA_STREAM);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA2_Stream2_IRQHandler(void)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (LL_DMA_IsActiveFlag_TC2(TERM_DMA)) {
                LL_DMA_ClearFlag_TC2(TERM_DMA);
                LL_DMA_ClearFlag_HT2(TERM_DMA);
                LL_DMA_EnableStream(TERM_DMA, TERM_DMA_STREAM);
                vTaskNotifyGiveFromISR(term_ctrl->xTaskToNotify,
                                       &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
