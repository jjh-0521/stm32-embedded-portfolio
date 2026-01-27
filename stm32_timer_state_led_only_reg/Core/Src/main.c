/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Week2 Day5 - Register-only GPIO/EXTI/TIM2 state LED
  *
  * 동작:
  *  - PC13 버튼(EXTI13) 누를 때마다 상태 전환: OFF -> ON -> BLINK -> OFF
  *  - PA5(LD2) LED 제어
  *  - TIM2 Update Interrupt를 10ms tick으로 구성(레지스터)
  *  - BLINK는 500ms마다 토글
  *  - 디바운스는 HAL_GetTick 기반 소프트 디바운스
  *
  * 원칙(Week2 Day5):
  *  - GPIO/EXTI/TIM2 설정은 레지스터로 직접 제어
  *  - HAL은 HAL_Init(), SystemClock_Config(), HAL_GetTick() 정도만 사용
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2; // CubeMX 생성물 유지(사용은 안 함)

/* USER CODE BEGIN PV */
volatile LedState_t g_led_state = LED_STATE_OFF;
volatile uint32_t   g_tick10ms = 0;
volatile uint32_t   g_btn_last_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
static void Reg_GPIO_Init_LED_PA5(void);
static void Reg_EXTI_Init_Button_PC13(void);
static void Reg_TIM2_Init_10ms_IT(void);
static void Reg_NVIC_Enable(void);

static inline void LED_On(void);
static inline void LED_Off(void);
static inline void LED_Toggle(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void LED_On(void)     { GPIOA->BSRR = (1U << 5); }
static inline void LED_Off(void)    { GPIOA->BSRR = (1U << (5 + 16U)); }
static inline void LED_Toggle(void) { GPIOA->ODR ^= (1U << 5); }

/*
 * PA5(LD2) GPIO Output 설정 (레지스터)
 */
static void Reg_GPIO_Init_LED_PA5(void)
{
  // GPIOA clock enable
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  (void)RCC->AHB1ENR;

  // PA5: output mode (MODER[11:10] = 01)
  GPIOA->MODER &= ~(3U << (5U * 2U));
  GPIOA->MODER |=  (1U << (5U * 2U));

  // push-pull
  GPIOA->OTYPER &= ~(1U << 5U);

  // no pull-up/down
  GPIOA->PUPDR &= ~(3U << (5U * 2U));

  // low speed ok
  GPIOA->OSPEEDR &= ~(3U << (5U * 2U));

  LED_Off();
}

/*
 * PC13 버튼 EXTI13 설정 (레지스터)
 * - falling edge
 * - pending clear
 */
static void Reg_EXTI_Init_Button_PC13(void)
{
  // GPIOC clock enable
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  (void)RCC->AHB1ENR;

  // PC13: input (MODER[27:26] = 00) -> CubeMX가 이미 했겠지만 명시
  GPIOC->MODER &= ~(3U << (13U * 2U));
  // pull은 Nucleo 보드 구조상 보통 nopull 유지
  GPIOC->PUPDR &= ~(3U << (13U * 2U));

  // SYSCFG clock enable (EXTI 라인 매핑)
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  (void)RCC->APB2ENR;

  // EXTI13 source = PC13
  // EXTICR4 (index 3), EXTI13은 EXTICR[3]의 bits[7:4]
  SYSCFG->EXTICR[3] &= ~(0xFU << 4);
  SYSCFG->EXTICR[3] |=  (0x2U << 4); // Port C = 0b0010

  // Interrupt mask enable for line 13
  EXTI->IMR |= (1U << 13U);

  // Falling trigger enable, Rising disable
  EXTI->FTSR |=  (1U << 13U);
  EXTI->RTSR &= ~(1U << 13U);

  // Clear pending
  EXTI->PR = (1U << 13U);
}

/*
 * TIM2 10ms Update Interrupt 설정 (레지스터)
 * - TIM2 clock = 84MHz (APB1 prescaler=2 -> TIM clock x2)
 * - PSC=8399 => 10kHz (0.1ms)
 * - ARR=99   => 100Hz (10ms)
 */
static void Reg_TIM2_Init_10ms_IT(void)
{
  // TIM2 clock enable
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  (void)RCC->APB1ENR;

  // Stop / reset-ish
  TIM2->CR1 = 0;
  TIM2->DIER = 0;
  TIM2->SR = 0;

  // Set PSC/ARR
  TIM2->PSC = 8399U;
  TIM2->ARR = 99U;

  // Update event to latch PSC/ARR
  TIM2->EGR = TIM_EGR_UG;

  // Enable update interrupt
  TIM2->DIER |= TIM_DIER_UIE;

  // Clear UIF
  TIM2->SR &= ~TIM_SR_UIF;

  // Start counter
  TIM2->CR1 |= TIM_CR1_CEN;
}

static void Reg_NVIC_Enable(void)
{
  // EXTI15_10 IRQ enable
  NVIC_SetPriority(EXTI15_10_IRQn, 0);
  NVIC_EnableIRQ(EXTI15_10_IRQn);

  // TIM2 IRQ enable
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_EnableIRQ(TIM2_IRQn);
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

  /* CubeMX 자동 init은 비교용으로 남겨두되, Day5는 레지스터로 직접 구성 */
  // MX_GPIO_Init();
  // MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  Reg_GPIO_Init_LED_PA5();
  Reg_EXTI_Init_Button_PC13();
  Reg_TIM2_Init_10ms_IT();
  Reg_NVIC_Enable();

  // 시작 상태 반영
  LED_Off();
  /* USER CODE END 2 */

  while (1)
  {
    // 메인 루프는 비워두고, EXTI + TIM2 인터럽트로만 동작
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* CubeMX 생성 함수들: Day5에서는 호출하지 않지만, 파일 구조상 남겨둠 */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/*
 * 아래 두 ISR은 "main.c에 만들면" CubeMX가 생성한 it.c와 중복 정의로 빌드 에러가 날 수 있다.
 * 따라서 Day5는 Core/Src/stm32f4xx_it.c에서 레지스터 기반 ISR로 구현하는 것을 권장한다.
 *
 * → 아래는 참고용 로직이며, 실제 구현은 it.c에 넣어야 한다.
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
