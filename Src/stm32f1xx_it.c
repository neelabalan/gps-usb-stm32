/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

extern PCD_HandleTypeDef hpcd_USB_FS;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t rxbuf[50];
uint8_t rx_data[4];
extern char gps_start[4];
extern uint8_t gstart;
extern uint8_t gps_i;
extern uint8_t i;
extern uint8_t start;
extern char gpsrx[50];
char gpstemp[10];
char gstart1;
char gstart2, x, y;
extern char new_data;

void NMI_Handler(void){}

void HardFault_Handler(void)
{
    while (1){}
}

void MemManage_Handler(void)
{
    while (1){}
}

void BusFault_Handler(void)
{
    while (1){}
}

void UsageFault_Handler(void)
{
    while (1) {}
}


void SVC_Handler(void){}

void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_FS);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
    HAL_UART_Receive_IT(&huart3, gps_start, 1);
    new_data = 1;
    if (gps_start[0] == '$')
    {
        gstart1 = 1;
    }
    if (gstart2 == 1)
    {
        gpsrx[y] = gps_start[0];
        y++;
        if (y > 49)
        {
            y = 0;
        }
    }
    if (gstart1)
    {
        gpstemp[x] = gps_start[0];
        x++;
        if (x > 9)
            x = 0;
        if (x == 6)
        {
            if ((gpstemp[1] == 'G') && (gpstemp[2] == 'P') && (gpstemp[3] == 'R') && (gpstemp[4] == 'M') && (gpstemp[5] == 'C'))
            {
                gstart2 = 1;
                gstart1 = 0;
                //y = 0;
            }
        }
    }

    if ((gps_start[0] == 'N') || (gps_start[0] == 'S'))
    {
        x = 0;
        y = 0;
        gstart1 = 0;
        gstart2 = 0;
        gpstemp[1] = 0;
        gpstemp[2] = 0;
        gpstemp[3] = 0;
        new_data = 1;
    }
}
