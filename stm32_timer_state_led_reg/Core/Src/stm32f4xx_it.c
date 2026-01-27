/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1) { }
}

void HardFault_Handler(void)
{
  while (1) { }
}

void MemManage_Handler(void)
{
  while (1) { }
}

void BusFault_Handler(void)
{
  while (1) { }
}

void UsageFault_Handler(void)
{
  while (1) { }
}

void SVC_Handler(void) { }

void DebugMon_Handler(void) { }

void PendSV_Handler(void) { }

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief This function handles TIM2 global interrupt.
  *
  * 2주차 4일차 포인트:
  * - HAL_TIM_IRQHandler(&htim2) 호출 없이
  * - TIM2->SR의 UIF 플래그를 직접 확인/클리어해서
  * - "레지스터 기반"으로 타이머 인터럽트를 처리한다.
  *
  * 주의:
  * - CubeMX에서 NVIC의 TIM2 global interrupt Enable 필요
  * - main.c에서 TIM2 DIER(UIE) 설정 + NVIC Enable + CR1(CEN) enable이 필요
  */
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_UIF)
  {
    TIM2->SR &= ~TIM_SR_UIF;  // Update interrupt flag clear

    // 여기서 "10ms tick 처리"는 main.c에 있는 콜백(HAL_TIM_PeriodElapsedCallback)을 쓰지 않고,
    // 레지스터 방식으로 직접 처리하는 것이 Day4 의도.
    //
    // 가장 간단한 방식:
    // - main.c의 tick/상태 로직을 '함수'로 빼서 여기서 호출한다.
    //
    // 아래 외부 함수는 main.c에 반드시 구현되어 있어야 한다.
    extern void TIM2_Tick_10ms_Task(void);
    TIM2_Tick_10ms_Task();
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
