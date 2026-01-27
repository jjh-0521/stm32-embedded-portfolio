# TIM2 레지스터 기반 타이머 인터럽트 + 상태 머신 (stm32_timer_state_led_reg)

## 프로젝트 개요
이번 실습은 2주차 3일차에서 완성한 **이벤트(EXTI) + 상태 머신 + 타이머 heartbeat 구조**를 그대로 유지한 채,  
TIM2를 **HAL API가 아닌 레지스터 직접 제어 방식**으로 구현하여 HAL 내부 동작을 하드웨어 레벨에서 명확히 이해하는 것이 목표다.

- 버튼(PC13, EXTI): 상태 전환 트리거 (OFF → ON → BLINK → OFF)
- TIM2(10ms tick): 레지스터 기반 Update Interrupt
- BLINK: tick 분주를 통해 500ms 주기로 토글
- 디바운싱: HAL_GetTick 기반 소프트 디바운싱

---

## 개발 환경
- Board: Nucleo-F401RE (STM32F401RE)
- IDE: STM32CubeIDE
- Config Tool: STM32CubeMX (.ioc)
- Library: STM32 HAL Driver  
  (단, **TIM2 구동 및 인터럽트 처리는 레지스터 직접 제어**)
- Peripherals
  - GPIO: PA5(LD2 LED), PC13(User Button)
  - EXTI: EXTI15_10
  - TIM2: Base Timer + Update Interrupt

---

## 구현 내용
### 1) 기능 요구사항
1. 버튼(PC13)을 누를 때마다 LED 상태가 순환 전환된다.
   - OFF → ON → BLINK → OFF
2. 상태별 LED 동작
   - OFF: 항상 OFF
   - ON: 항상 ON
   - BLINK: 500ms 주기로 토글
3. 타이머는 10ms 주기 tick을 기준으로 동작
4. 버튼 바운싱 방지를 위한 소프트 디바운싱 적용

### 2) 핵심 설계
- **EXTI = 이벤트 트리거**
  - 버튼 인터럽트에서는 “상태만 전환”
- **TIM2 = heartbeat**
  - 주기적으로 상태를 확인하고 LED를 갱신
- 구조는 3일차와 동일하지만, **타이머 구동 방식만 HAL → 레지스터로 변경**

---

## 동작 방식
### 1) CubeMX 설정 요약
- GPIO
  - PA5: Output (LED)
  - PC13: EXTI Falling
- NVIC
  - EXTI15_10_IRQn Enable
  - TIM2_IRQn Enable
- TIM2
  - MX_TIM2_Init(): HAL 초기화 형태 유지(비교용)
  - 실제 동작: main.c에서 레지스터로 재구성

### 2) TIM2 10ms tick 계산
- SYSCLK = 84MHz
- APB1 Prescaler = /2 → PCLK1 = 42MHz
- TIM2 clock = 84MHz (APB prescaler ≠ 1 이므로 ×2)

레지스터 설정:
- PSC = 8399 → 84MHz / (8399 + 1) = 10kHz (0.1ms)
- ARR = 99   → 10kHz / (99 + 1) = 100Hz → 10ms

### 3) 인터럽트 흐름
1. 버튼 입력
   - EXTI15_10_IRQHandler → HAL_GPIO_EXTI_IRQHandler
   - HAL_GPIO_EXTI_Callback에서 상태 전환
2. 타이머 tick
   - TIM2_IRQHandler에서 UIF 플래그 직접 확인/클리어
   - `TIM2_Tick_10ms_Task()` 호출 → tick 누적 + 상태 기반 LED 제어

---

## 학습 포인트
### 1) 2주차 4일차의 본질
- 3일차까지는 “잘 동작하는 구조”를 만드는 데 집중했다면,
- 4일차는 **그 구조를 HAL 추상화 아래가 아닌 하드웨어 레벨에서 완전히 이해**하는 단계다.
- 즉, “기능 구현”이 아니라 “동작 원리 해부”가 핵심 성과다.

### 2) 2주차 1~4일차 비교(심화)
#### 2주차 1일차: EXTI 즉시 처리
- ISR에서 LED를 바로 토글
- 인터럽트 개념 학습에는 적합
- 확장성/안정성은 낮음

#### 2주차 2일차: 디바운싱
- 시간 개념 도입
- “이벤트”와 “시간”의 분리 필요성 인식

#### 2주차 3일차: 상태 머신 + 타이머
- 이벤트(버튼)와 주기 작업(타이머)을 분리
- 임베디드에서 가장 자주 쓰이는 구조 완성

#### 2주차 4일차: HAL ↔ Register 대응
- 구조는 그대로 두고 구현 수단만 변경
- HAL API가 내부적으로 무엇을 하는지 레지스터 단위로 확인
- 디버깅/튜닝/면접에서 설명 가능한 수준으로 이해도 상승

---

## HAL ↔ Register 1:1 대응표 (2주차 4일차 핵심)

| HAL 개념 / API                     | 실제 레지스터 동작              | 의미                          |
|-----------------------------------|---------------------------|------------------------------|
| `__HAL_RCC_TIM2_CLK_ENABLE()`     | RCC->APB1ENR |= TIM2EN     | TIM2 클럭 공급                 |
| `htim2.Init.Prescaler`            | TIM2->PSC                  | 타이머 분주비 설정                |
| `htim2.Init.Period`               | TIM2->ARR                  | 자동 재장전 값                  |
| `HAL_TIM_Base_Start_IT()`         | DIER(UIE) + CR1(CEN)       | Update IRQ 허용 + 카운터 시작    |
| `HAL_TIM_IRQHandler()`            | SR(UIF) 확인 / 클리어         | 인터럽트 원인 처리                |
| `HAL_TIM_PeriodElapsedCallback()` | 사용자 tick 처리 함수           | 주기 작업 수행                  |
| NVIC Enable                       | NVIC->ISER                 | TIM2 IRQ CPU 전달             |

이 대응표를 통해,
- HAL은 “레지스터 설정 + 플래그 처리 + 콜백 분배”를 대신해주는 계층임을 확인했다.
- HAL을 쓰지 않아도 동일한 동작을 만들 수 있으며,  
  문제 발생 시 레지스터 레벨에서 원인을 추적할 수 있는 기반을 확보했다.

---

## 실무/면접 활용 포인트
- “동일 기능을 HAL과 레지스터 방식으로 모두 구현해 비교했다.”
- “타이머 Update Interrupt의 핵심은 PSC/ARR/DIER/UIF/CR1 조합임을 이해하고 있다.”
- “이벤트(EXTI)와 주기 작업(TIM)을 분리한 구조로 확장 가능한 설계를 했다.”

---

## 파일 구조
- stm32_timer_state_led_reg/
  - Core/
    - Inc/
      - main.h
      - stm32f4xx_it.h
    - Src/
      - main.c
      - stm32f4xx_it.c
      - stm32f4xx_hal_msp.c
      - system_stm32f4xx.c
  - Drivers/
  - stm32_timer_state_led_reg.ioc
  - README.md
