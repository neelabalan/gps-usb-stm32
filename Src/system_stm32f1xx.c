/**
  ******************************************************************************
  * @file    system_stm32f1xx.c
  * @author  MCD Application Team
  * @version V4.2.0
  * @date    31-March-2017
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  * 
  * 1.  This file provides two functions and one global variable to be called from 
  *     user application:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      factors, AHB/APBx prescalers and Flash settings). 
  *                      This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f1xx_xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (8 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f1xx_xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 4. The default value of HSE crystal is set to 8 MHz (or 25 MHz, depending on
  *    the product used), refer to "HSE_VALUE". 
  *    When HSE is used as system clock source, directly or through PLL, and you
  *    are using different crystal you have to adapt the HSE value to your own
  *    configuration.
  *        
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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

#include "stm32f1xx.h"

#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U 
                               
#endif                     

#if !defined(HSI_VALUE)
#define HSI_VALUE 8000000U 
                             
#endif                     

/*!< Uncomment the following line if you need to use external SRAM  */
#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
/* #define DATA_IN_ExtSRAM */
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */


#define VECT_TAB_OFFSET 0x00000000U 

#if defined(STM32F100xB) || defined(STM32F100xE)
uint32_t SystemCoreClock = 24000000U; 
#else                                 
uint32_t SystemCoreClock = 72000000U; 
#endif

const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] = {0, 0, 0, 0, 1, 2, 3, 4};

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
#ifdef DATA_IN_ExtSRAM
static void SystemInit_ExtMemCtl(void);
#endif 
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */


void SystemInit(void)
{
    RCC->CR |= 0x00000001U;

#if !defined(STM32F105xC) && !defined(STM32F107xC)
    RCC->CFGR &= 0xF8FF0000U;
#else
    RCC->CFGR &= 0xF0FF0000U;
#endif /* STM32F105xC */

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= 0xFEF6FFFFU;
    /* Reset HSEBYP bit */
    RCC->CR &= 0xFFFBFFFFU;
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= 0xFF80FFFFU;

#if defined(STM32F105xC) || defined(STM32F107xC)
    /* Reset PLL2ON and PLL3ON bits */
    RCC->CR &= 0xEBFFFFFFU;
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x00FF0000U;
    /* Reset CFGR2 register */
    RCC->CFGR2 = 0x00000000U;
#elif defined(STM32F100xB) || defined(STM32F100xE)
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x009F0000U;
    /* Reset CFGR2 register */
    RCC->CFGR2 = 0x00000000U;
#else
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x009F0000U;
#endif /* STM32F105xC */

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)
#ifdef DATA_IN_ExtSRAM
    SystemInit_ExtMemCtl();
#endif /* DATA_IN_ExtSRAM */
#endif

#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM. */
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH. */
#endif
}

void SystemCoreClockUpdate(void)
{
    uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;

#if defined(STM32F105xC) || defined(STM32F107xC)
    uint32_t prediv1source = 0U, prediv1factor = 0U, prediv2factor = 0U, pll2mull = 0U;
#endif /* STM32F105xC */

#if defined(STM32F100xB) || defined(STM32F100xE)
    uint32_t prediv1factor = 0U;
#endif /* STM32F100xB or STM32F100xE */

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
    case 0x00U: /* HSI used as system clock */
        SystemCoreClock = HSI_VALUE;
        break;
    case 0x04U: /* HSE used as system clock */
        SystemCoreClock = HSE_VALUE;
        break;
    case 0x08U: /* PLL used as system clock */

        /* Get PLL clock source and multiplication factor ----------------------*/
        pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
        pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

#if !defined(STM32F105xC) && !defined(STM32F107xC)
        pllmull = (pllmull >> 18U) + 2U;

        if (pllsource == 0x00U)
        {
            /* HSI oscillator clock divided by 2 selected as PLL clock entry */
            SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
        }
        else
        {
#if defined(STM32F100xB) || defined(STM32F100xE)
            prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1U;
            /* HSE oscillator clock selected as PREDIV1 clock entry */
            SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
#else
            /* HSE selected as PLL clock entry */
            if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
            { /* HSE oscillator clock divided by 2 */
                SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
            }
            else
            {
                SystemCoreClock = HSE_VALUE * pllmull;
            }
#endif
        }
#else
        pllmull = pllmull >> 18U;

        if (pllmull != 0x0DU)
        {
            pllmull += 2U;
        }
        else
        { /* PLL multiplication factor = PLL input clock * 6.5 */
            pllmull = 13U / 2U;
        }

        if (pllsource == 0x00U)
        {
            /* HSI oscillator clock divided by 2 selected as PLL clock entry */
            SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
        }
        else
        { /* PREDIV1 selected as PLL clock entry */

            /* Get PREDIV1 clock source and division factor */
            prediv1source = RCC->CFGR2 & RCC_CFGR2_PREDIV1SRC;
            prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1U;

            if (prediv1source == 0U)
            {
                /* HSE oscillator clock selected as PREDIV1 clock entry */
                SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
            }
            else
            { /* PLL2 clock selected as PREDIV1 clock entry */

                /* Get PREDIV2 division factor and PLL2 multiplication factor */
                prediv2factor = ((RCC->CFGR2 & RCC_CFGR2_PREDIV2) >> 4U) + 1U;
                pll2mull = ((RCC->CFGR2 & RCC_CFGR2_PLL2MUL) >> 8U) + 2U;
                SystemCoreClock = (((HSE_VALUE / prediv2factor) * pll2mull) / prediv1factor) * pllmull;
            }
        }
#endif /* STM32F105xC */
        break;

    default:
        SystemCoreClock = HSI_VALUE;
        break;
    }

    /* Compute HCLK clock frequency ----------------*/
    /* Get HCLK prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
    /* HCLK clock frequency */
    SystemCoreClock >>= tmp;
}

#if defined(STM32F100xE) || defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F103xE) || defined(STM32F103xG)

#ifdef DATA_IN_ExtSRAM

void SystemInit_ExtMemCtl(void)
{
    __IO uint32_t tmpreg;

    /* Enable FSMC clock */
    RCC->AHBENR = 0x00000114U;

    /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);

    /* Enable GPIOD, GPIOE, GPIOF and GPIOG clocks */
    RCC->APB2ENR = 0x000001E0U;

    /* Delay after an RCC peripheral clock enabling */
    tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);

    (void)(tmpreg);

    GPIOD->CRL = 0x44BB44BBU;
    GPIOD->CRH = 0xBBBBBBBBU;

    GPIOE->CRL = 0xB44444BBU;
    GPIOE->CRH = 0xBBBBBBBBU;

    GPIOF->CRL = 0x44BBBBBBU;
    GPIOF->CRH = 0xBBBB4444U;

    GPIOG->CRL = 0x44BBBBBBU;
    GPIOG->CRH = 0x444B4B44U;

    FSMC_Bank1->BTCR[4U] = 0x00001091U;
    FSMC_Bank1->BTCR[5U] = 0x00110212U;
}
#endif /* DATA_IN_ExtSRAM */
#endif /* STM32F100xE || STM32F101xE || STM32F101xG || STM32F103xE || STM32F103xG */

