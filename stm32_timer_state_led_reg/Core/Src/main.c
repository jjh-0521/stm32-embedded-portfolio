/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Week2 Day4 - TIM2 register control + EXTI state change
  ******************************************************************************
  * 동작:
  *  - PC13 버튼(EXTI) 누를 때마다 상태 전환: OFF -> ON -> BLINK -> OFF
  *  - TIM2는 HAL init(클럭/핸들 생성)은 두되, "레지스터로" 10ms Update IRQ 구성
  *  - TIM2 IRQ에서 10ms tick 누적, BLINK는 500ms마다 토글
  *  - 간단 디바운스: HAL_GetTick() 50ms
  *
  * 주의:
  *  - TIM2_IRQHandler는 main.c에 만들지 말고 Core/Src/stm32f4xx_it.c에 있는 것 사용
  *    (중복 정의 방지)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
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
#define TICK_MS                 (10U)    // TIM2 Update period
#define BLINK_TOGGLE_MS         (500U)
#define BLINK_TOGGLE_TICKS      (BLINK_TOGGLE_MS / TICK_MS) // 50
#define BTN_DEBOUNCE_MS         (50U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static volatile LedState_t g_led_state = LED_STATE_OFF;
static volatile uint32_t g_tick10ms = 0;
static volatile uint32_t g_btn_last_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void TIM2_RegInit_10ms_IT(void);
static void Led_ApplyState_Immediate(LedState_t st);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_Tick_10ms_Task(void)
{
  g_tick10ms++;

  switch (g_led_state)
  {
    case LED_STATE_OFF:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;

    case LED_STATE_ON:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      break;

    case LED_STATE_BLINK:
      if ((g_tick10ms % BLINK_TOGGLE_TICKS) == 0U) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      }
      break;

    default:
      g_led_state = LED_STATE_OFF;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;
  }
}


static void Led_ApplyState_Immediate(LedState_t st)
{
  if (st == LED_STATE_OFF) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  } else if (st == LED_STATE_ON) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  } else {
    // BLINK: 타이머에서 토글하므로 여기서는 고정값을 강제하지 않음(선택사항)
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  }
}

/*
 * TIM2 레지스터 직접 설정(10ms Update Interrupt)
 * - TIM2 clock = 84MHz (APB1 prescaler=2 => TIMx clock x2)
 * - PSC=8399  => 84MHz/(8399+1)=10kHz (0.1ms)
 * - ARR=99    => 10kHz/(99+1)=100Hz => 10ms
 */
static void TIM2_RegInit_10ms_IT(void)
{
  // 1) TIM2 peripheral clock enable (CubeMX가 해주지만 "레지스터 실습"이라 명시적으로 호출)
  __HAL_RCC_TIM2_CLK_ENABLE();

  // 2) Reset/Stop
  TIM2->CR1 = 0;                // counter disable, upcount, etc.
  TIM2->DIER = 0;               // disable all interrupts
  TIM2->SR = 0;                 // clear flags

  // 3) Set prescaler & auto-reload
  TIM2->PSC = 8399;
  TIM2->ARR = 99;

  // 4) Update event to load PSC/ARR
  TIM2->EGR = TIM_EGR_UG;

  // 5) Enable update interrupt
  TIM2->DIER |= TIM_DIER_UIE;

  // 6) NVIC enable (CubeMX에서 TIM2 NVIC Enable 안 했어도 여기서 켜면 됨)
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // 7) Counter enable
  TIM2->CR1 |= TIM_CR1_CEN;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();  // Day4: "HAL init 함수를 유지"하되, 실제 타이머 동작은 레지스터로 재구성

  /* USER CODE BEGIN 2 */
  // Day4 핵심: HAL_TIM_Base_Start_IT()를 쓰지 않고, 레지스터로 TIM2를 구성한다.
  TIM2_RegInit_10ms_IT();

  // 시작 상태 반영
  Led_ApplyState_Immediate(g_led_state);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // 의도: 메인 루프는 비워두고, EXTI(이벤트) + TIM2(heartbeat)로 동작
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // PCLK1=42MHz, TIM2 clk=84MHz
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
  // Day4 목적:
  // - 이 함수는 CubeMX가 생성한 "HAL 기반 초기화" 형태를 유지(비교용)
  // - 하지만 실제 구동은 TIM2_RegInit_10ms_IT()에서 레지스터로 재구성
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295; // CubeMX 기본값 유지(실제 동작은 RegInit에서 ARR=99로 재설정)
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

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  // PC13: EXTI Falling
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // PA5: LED output
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // EXTI NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

// 버튼 이벤트(EXTI) 콜백: 상태만 전환(가볍게)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin != GPIO_PIN_13) return;

  uint32_t now = HAL_GetTick();
  if ((now - g_btn_last_ms) < BTN_DEBOUNCE_MS) return;
  g_btn_last_ms = now;

  if (g_led_state == LED_STATE_OFF) {
    g_led_state = LED_STATE_ON;
    Led_ApplyState_Immediate(g_led_state);

  } else if (g_led_state == LED_STATE_ON) {
    g_led_state = LED_STATE_BLINK;

    // BLINK 진입 시 사용자 체감 혼란 방지: 기준값을 한번 잡아준다
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    // (선택) 토글 타이밍을 즉시 일정하게 만들고 싶으면 tick 초기화
    g_tick10ms = 0;

  } else {
    g_led_state = LED_STATE_OFF;
    Led_ApplyState_Immediate(g_led_state);
  }
}


/*
 * Day4: TIM2 IRQ에서 HAL 콜백을 쓰지 않고, "레지스터"로 UIF 플래그를 확인/클리어
 * - TIM2_IRQHandler는 반드시 stm32f4xx_it.c에 있는 것을 사용
 * - 여기서는 ISR에서 호출되는 "처리 함수"만 제공(직접 호출되진 않음)
 *
 * 구현 방법은 2가지 중 택1:
 *  A) stm32f4xx_it.c의 TIM2_IRQHandler 안에서 아래 로직을 직접 작성
 *  B) 아래 함수를 만들고, TIM2_IRQHandler에서 호출
 *
 * 여기서는 B로 구현:
 */


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

/***************************************************************
 * 반드시 같이 수정할 파일: Core/Src/stm32f4xx_it.c
 *
 * TIM2_IRQHandler가 아래처럼 되어있다면 "HAL_TIM_IRQHandler"를 빼고,
 * TIM2_TickHandler_Reg()를 호출하도록 바꿔야 Day4 의도가 맞음.
 *
 * (main.c에 TIM2_IRQHandler를 만들면 중복 정의로 빌드 에러남)
 *
 * ---- 예시 수정본(핵심만) ----
 *
 * extern void TIM2_TickHandler_Reg(void);  // main.c에 static이면 extern 불가.
 * // 따라서 위 로직은 static inline 대신, non-static 함수로 바꾸거나
 * // it.c에 로직을 직접 작성하는 방식을 추천.
 *
 * 가장 추천(간단/확실):
 * void TIM2_IRQHandler(void)
 * {
 *   if (TIM2->SR & TIM_SR_UIF) {
 *     TIM2->SR &= ~TIM_SR_UIF;
 *     // main.c의 tick/상태 로직을 여기로 옮기거나,
 *     // main.c에 non-static 함수로 만들어 호출
 *   }
 * }
 *
 ***************************************************************/
