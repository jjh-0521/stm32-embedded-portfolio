/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Button debounce (EXTI + TIM2 10ms) + short/long + LED state
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_LED_OFF = 0,
  STATE_LED_ON,
  STATE_LED_BLINK
} system_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TICKS        5u     // 50ms (10ms * 5)
#define LONG_PRESS_TICKS      100u   // 1s  (10ms * 100)
#define BLINK_TOGGLE_TICKS    10u    // 100ms (10ms * 10)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
static volatile uint8_t  g_debounce_active = 0;
static volatile uint16_t g_debounce_cnt = 0;

static volatile uint8_t  g_btn_state = 0;          // debounced state: 1=pressed, 0=released
static volatile uint16_t g_press_ticks = 0;

static volatile uint8_t  g_evt_short_press = 0;
static volatile uint8_t  g_evt_long_press  = 0;

static volatile uint16_t g_blink_cnt = 0;
static volatile uint8_t  g_led_toggle_req = 0;

static volatile system_state_t g_state = STATE_LED_OFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
static inline uint8_t read_button_pressed_raw(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* NUCLEO-F401RE USER 버튼(PC13)은 보통 Active-Low:
   - 눌림  : GPIO_PIN_RESET
   - 떼짐  : GPIO_PIN_SET
*/
static inline uint8_t read_button_pressed_raw(void)
{
  return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) ? 1u : 0u;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    g_debounce_active = 1;
    g_debounce_cnt = 0;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM2)
  {
    /* 1) Debounce */
    if (g_debounce_active)
    {
      uint8_t raw_pressed = read_button_pressed_raw();

      if (raw_pressed == g_btn_state)
      {
        g_debounce_active = 0;
        g_debounce_cnt = 0;
      }
      else
      {
        g_debounce_cnt++;
        if (g_debounce_cnt >= DEBOUNCE_TICKS)
        {
          g_btn_state = raw_pressed;
          g_debounce_active = 0;
          g_debounce_cnt = 0;

          if (g_btn_state == 1u)
          {
            g_press_ticks = 0;
          }
          else
          {
            if (g_press_ticks > 0u)
            {
              if (g_press_ticks < LONG_PRESS_TICKS) g_evt_short_press = 1u;
              else                                  g_evt_long_press  = 1u;
            }
            g_press_ticks = 0;
          }
        }
      }
    }

    /* 2) Press time */
    if (g_btn_state == 1u)
    {
      if (g_press_ticks < 0xFFFFu) g_press_ticks++;
    }

    /* 3) Blink request */
    if (g_state == STATE_LED_BLINK)
    {
      g_blink_cnt++;
      if (g_blink_cnt >= BLINK_TOGGLE_TICKS)
      {
        g_blink_cnt = 0;
        g_led_toggle_req = 1u;
      }
    }
    else
    {
      g_blink_cnt = 0;
    }
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
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    if (g_evt_short_press)
    {
      g_evt_short_press = 0;

      if (g_state == STATE_LED_OFF)      g_state = STATE_LED_ON;
      else if (g_state == STATE_LED_ON)  g_state = STATE_LED_BLINK;
      else                               g_state = STATE_LED_OFF;

      if (g_state == STATE_LED_OFF)
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      else if (g_state == STATE_LED_ON)
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }

    if (g_evt_long_press)
    {
      g_evt_long_press = 0;
      g_state = STATE_LED_OFF;
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

    if (g_led_toggle_req)
    {
      g_led_toggle_req = 0;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    /* USER CODE END 3 */
  }
}

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;   // 10ms tick
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();

  /* NVIC (CubeMX에서 TIM2 IRQ 체크 안 했으면 여기서라도 켬) */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* PC13 EXTI (Active-Low 버튼이므로 Falling이 눌림) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* LD2 (PA5) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
