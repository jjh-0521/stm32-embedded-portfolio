/*******************************************************
 * File: Core/Src/main.c
 * Project: stm32_timer_state_led (Week2 Day3)
 *
 * 기능:
 *  - 버튼(EXTI)로 LED 상태 전환: OFF -> ON -> BLINK -> OFF
 *  - TIM2 주기 인터럽트(10ms tick)로 상태에 따라 LED 제어
 *  - BLINK는 500ms(=10ms * 50)마다 토글
 *  - EXTI에서 간단 소프트 디바운싱(50ms)
 *******************************************************/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  LED_STATE_OFF = 0,
  LED_STATE_ON,
  LED_STATE_BLINK
} LedState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TICK_MS                 (10U)   // TIM2 update period: 10ms
#define BLINK_TOGGLE_MS         (500U)  // BLINK 토글 주기: 500ms
#define BLINK_TOGGLE_TICKS      (BLINK_TOGGLE_MS / TICK_MS)  // 50 ticks
#define BTN_DEBOUNCE_MS         (50U)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static volatile LedState_t g_led_state = LED_STATE_OFF;
static volatile uint32_t g_tick10ms = 0;       // 10ms heartbeat 카운터
static volatile uint32_t g_btn_last_ms = 0;    // 버튼 디바운스 타임스탬프(ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
static void Led_ApplyState_Immediate(LedState_t st);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Led_ApplyState_Immediate(LedState_t st)
{
  // 상태가 바뀌는 즉시 LED를 “대표값”으로 맞춰주고 싶을 때 사용
  // (BLINK는 타이머에서 토글하므로 여기서는 건드리지 않아도 됨)
  if (st == LED_STATE_OFF) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  } else if (st == LED_STATE_ON) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  } else {
    // BLINK: 즉시값 지정은 선택 사항
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  // TIM2 인터럽트 기반 시작
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }

  // 시작 상태 반영
  Led_ApplyState_Immediate(g_led_state);
  /* USER CODE END 2 */

  while (1)
  {
    // 2주차 3일차 의도: 메인 루프는 비워두고(폴링 최소),
    // 버튼 이벤트(EXTI) + 주기 인터럽트(TIM2)로 시스템을 굴린다.
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  // HSI 16MHz -> PLL -> SYSCLK 84MHz 구성(네 코드 그대로)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // PCLK1 = 42MHz, TIM2 clk = 84MHz(타이머 x2)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  // TIM2 clock = 84MHz
  // Prescaler = 8399 -> 84MHz / 8400 = 10kHz (0.1ms tick)
  // Period    = 99   -> 10kHz / 100 = 100Hz => 10ms 업데이트 인터럽트
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  // NVIC (CubeMX에서 TIM2 global interrupt Enable 하면 자동 생성되는 부분이지만,
  // 혹시 누락됐을 때를 대비해 여기서도 설정)
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  // Button: B1 (보통 PC13) - Falling edge EXTI
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // LED: LD2 (보통 PA5)
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  // EXTI NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

// 버튼 인터럽트 콜백: 상태만 바꾸는 “이벤트 트리거” 역할
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != B1_Pin) return;

  uint32_t now = HAL_GetTick();
  if ((now - g_btn_last_ms) < BTN_DEBOUNCE_MS) {
    return; // 간단 디바운스
  }
  g_btn_last_ms = now;

  // OFF -> ON -> BLINK -> OFF
  if (g_led_state == LED_STATE_OFF) {
    g_led_state = LED_STATE_ON;
    Led_ApplyState_Immediate(g_led_state);
  } else if (g_led_state == LED_STATE_ON) {
    g_led_state = LED_STATE_BLINK;
    // BLINK로 들어갈 때 LED를 확정하고 싶으면 아래 주석 해제
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  } else {
    g_led_state = LED_STATE_OFF;
    Led_ApplyState_Immediate(g_led_state);
  }
}

// 타이머 주기 인터럽트: 시스템 “heartbeat”
// 10ms마다 호출되며, 상태에 따라 LED를 갱신
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM2) return;

  g_tick10ms++;

  switch (g_led_state)
  {
    case LED_STATE_OFF:
      // OFF는 계속 OFF 유지
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      break;

    case LED_STATE_ON:
      // ON은 계속 ON 유지
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      break;

    case LED_STATE_BLINK:
      // 500ms마다 토글
      if ((g_tick10ms % BLINK_TOGGLE_TICKS) == 0U) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
      break;

    default:
      g_led_state = LED_STATE_OFF;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      break;
  }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif


/*******************************************************
 * File: Core/Src/stm32f4xx_it.c  (필수)
 *
 * 주의:
 *  - CubeMX에서 "TIM2 global interrupt"를 Enable하면
 *    아래 TIM2_IRQHandler가 자동 생성될 수도 있음.
 *  - 만약 TIM2가 안 돈다면, 이 핸들러가 존재하는지 확인.
 *******************************************************/

#include "main.h"

extern TIM_HandleTypeDef htim2;


/* EXTI 핸들러는 CubeMX가 이미 만들어둔 것 사용하면 됨:
   void EXTI15_10_IRQHandler(void)
   {
     HAL_GPIO_EXTI_IRQHandler(B1_Pin);
   }
*/
