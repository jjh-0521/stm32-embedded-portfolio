# Week 2 Day 5 – TIM2 Only Register Control + EXTI State LED

## 프로젝트 개요
2주차 5일차 실습에서는 **HAL API를 거의 사용하지 않고**,  
TIM2와 EXTI를 **레지스터 직접 제어 방식만으로** 구현하여  
버튼 입력에 따른 LED 상태 전환 시스템을 완성한다.

버튼(PC13)을 누를 때마다 LED(PA5)의 상태가  
OFF → ON → BLINK → OFF 로 순환하며,  
타이머 인터럽트를 이용해 BLINK 동작을 정확한 주기로 제어한다.

이번 실습은 **HAL 추상화 계층을 걷어내고, MCU 내부 동작을 직접 다루는 단계**라는 점에서  
2주차의 핵심 전환점에 해당한다.

---

## 개발 환경
- Board: NUCLEO-F401RE
- MCU: STM32F401RE
- IDE: STM32CubeIDE
- Language: C
- Clock:
  - SYSCLK = 84 MHz
  - APB1 = 42 MHz (TIM2 clock = 84 MHz)

---

## 구현 기능 요약
- PC13 버튼(EXTI) 입력 처리 (레지스터 기반)
- 버튼 디바운싱 (HAL_GetTick 기반)
- TIM2 Update Interrupt 10ms 주기 생성 (레지스터 직접 설정)
- LED 상태 머신 구현
  - OFF: LED 꺼짐
  - ON: LED 항상 켜짐
  - BLINK: 500ms 주기로 토글
- main loop는 비워두고 **완전한 인터럽트 기반 구조**로 동작

---

## 동작 방식

### 1. 버튼(EXTI) 처리 흐름
- PC13 Falling Edge 발생
- EXTI Pending Register(PR) 직접 확인 및 클리어
- HAL_GPIO_EXTI_IRQHandler 미사용
- 디바운스 조건:
  - 이전 버튼 입력 시점과 현재 시점 차이가 150ms 미만이면 무시
- 버튼 입력 시 LED 상태 전환:
  - OFF → ON
  - ON → BLINK
  - BLINK → OFF

---

### 2. TIM2 타이머 동작
- TIM2 클럭: 84 MHz
- Prescaler = 8399 → 10 kHz (0.1ms)
- Auto Reload Register(ARR) = 99 → 10ms
- Update Interrupt(UIF) 발생 시:
  - 10ms tick 누적
  - 현재 LED 상태에 따라 제어 수행

BLINK 상태에서는  
`10ms × 50 = 500ms` 주기로 LED 토글

---

### 3. 인터럽트 중심 구조
- main()의 while(1) 루프는 비어 있음
- 모든 로직은:
  - EXTI15_10_IRQHandler
  - TIM2_IRQHandler
  에서 처리
- Polling, Delay, HAL callback 사용하지 않음

---

## 1일차 · 2일차 · 3~4일차 · 5일차 비교

### 2주차 1일차 (EXTI 기본)
- HAL_GPIO_EXTI_Callback 사용
- 버튼 입력 = 즉시 LED 토글
- 상태 개념 없음
- 인터럽트는 “이벤트 발생” 정도의 의미

---

### 2주차 2일차 (Timer + HAL)
- HAL_TIM_Base_Start_IT 사용
- HAL_TIM_PeriodElapsedCallback 사용
- 타이머 주기 동작은 가능하지만
- 내부 동작은 HAL에 의해 숨겨져 있음

---

### 2주차 3~4일차 (HAL + Register 혼합)
- CubeMX HAL 초기화 유지
- 실제 타이머 동작은 레지스터 직접 설정
- HAL_TIM_IRQHandler 미사용
- 레지스터 기반 타이머 인터럽트 처리 경험

---

### 2주차 5일차 (Only Register Control)
- TIM2, EXTI 모두 레지스터 직접 제어
- HAL은:
  - 클럭 설정
  - SysTick (HAL_GetTick)
  정도만 사용
- 상태 머신 + 타이머 + 인터럽트 완전 결합
- MCU 내부 동작 흐름을 **직접 설계하는 단계**

---

## HAL vs 레지스터 관점 정리

| HAL 개념 / API                    | 실제 레지스터 동작              | 의미 |
|----------------------------------|----------------------------------|------|
| __HAL_RCC_TIM2_CLK_ENABLE()      | RCC->APB1ENR (TIM2EN)            | TIM2 클럭 공급 |
| htim2.Init.Prescaler             | TIM2->PSC                        | 타이머 분주비 |
| htim2.Init.Period                | TIM2->ARR                        | 자동 재장전 값 |
| HAL_TIM_Base_Start_IT()          | DIER(UIE) + CR1(CEN)             | IRQ 허용 + 카운터 시작 |
| HAL_TIM_IRQHandler()             | SR(UIF) 확인/클리어              | 인터럽트 원인 처리 |
| HAL_TIM_PeriodElapsedCallback()  | 사용자 tick 처리 코드            | 주기 작업 수행 |
| NVIC Enable                      | NVIC->ISER                       | CPU에 IRQ 전달 |

---

## 학습 포인트
- HAL은 “편의”, 레지스터는 “제어”
- 인터럽트 기반 설계의 구조적 장점 이해
- 상태 머신(State Machine)을 임베디드 로직에 적용하는 방법
- 타이머를 단순 지연이 아닌 **시스템 heartbeat**로 사용하는 개념
- 디바운스는 GPIO 문제가 아니라 **시간 관리 문제**임을 이해

---

## 파일 구조
stm32_timer_state_led_only_reg/
├─ Core/
│ ├─ Inc/
│ │ └─ main.h
│ └─ Src/
│ ├─ main.c
│ └─ stm32f4xx_it.c
├─ stm32_timer_state_led_only_reg.ioc
└─ README.md