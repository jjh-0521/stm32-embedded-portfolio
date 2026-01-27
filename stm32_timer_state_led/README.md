# 2주차 3일차 - Timer Interrupt 기반 LED 상태 머신 (stm32_timer_state_led)

## 프로젝트 개요
이번 실습은 버튼 입력(EXTI)을 “즉시 동작”으로 처리하지 않고, 버튼은 상태(State)를 바꾸는 이벤트 트리거로만 사용하며, 실제 LED 제어는 TIM2 주기 인터럽트(heartbeat)에서 수행하는 구조를 구현한다.  
즉, 메인 루프(while(1))에서 폴링/딜레이 중심으로 처리하던 방식을 벗어나, 인터럽트 기반의 이벤트/상태 머신 패턴을 적용하는 것이 목표다.

- 버튼(EXTI) 동작: OFF → ON → BLINK → OFF (상태 전환)
- 타이머(TIM2) 동작: 10ms 주기 tick으로 상태에 따라 LED를 갱신
- BLINK 동작: tick 누적을 이용해 500ms마다 LED 토글

## 개발 환경
- Board: Nucleo-F401RE (STM32F401RE)
- IDE: STM32CubeIDE
- Config Tool: STM32CubeMX (CubeIDE 내 .ioc)
- Library: STM32 HAL Driver
- 주요 주변장치
  - GPIO: LD2(LED), B1(User Button)
  - EXTI: 버튼 인터럽트 (Falling edge)
  - TIM2: Base Timer + Update Interrupt

## 구현 내용
### 1) 기능 요구사항
1. 버튼(B1)을 누를 때마다 LED 상태를 순환 전환한다.
   - LED_STATE_OFF → LED_STATE_ON → LED_STATE_BLINK → LED_STATE_OFF
2. LED 상태에 따른 실제 출력 제어는 TIM2 인터럽트에서만 수행한다.
3. BLINK는 10ms tick 기반 분주(divider)로 500ms 주기로 토글한다.
4. 버튼 바운싱으로 인한 연속 입력을 줄이기 위해 간단 소프트 디바운싱을 적용한다(HAL_GetTick 기반 50ms).

### 2) 핵심 설계 포인트
- 상태(State)를 전역 변수로 유지하고, EXTI는 상태만 변경한다.
- 타이머 인터럽트는 시스템의 “heartbeat” 역할을 하며, 상태에 따라 LED를 갱신한다.
- ISR(인터럽트 서비스 루틴)에서 무거운 로직을 최소화하고, 역할을 분리한다.
  - EXTI ISR: 이벤트 수신/상태 전환
  - TIM2 ISR: 주기적 갱신/상태 기반 동작 수행

## 동작 방식
### 1) CubeMX 설정 개요
- GPIO
  - LD2(PA5): Output
  - B1(PC13): EXTI Falling Edge
- NVIC
  - EXTI15_10_IRQn Enable
  - TIM2_IRQn Enable
- TIM2 (Time Base)
  - Prescaler/Period를 설정하여 10ms 주기 업데이트 인터럽트 발생
  - main()에서 HAL_TIM_Base_Start_IT(&htim2) 호출로 인터럽트 시작

### 2) 런타임 흐름
1. 초기화 단계
   - HAL_Init() / SystemClock_Config()
   - MX_GPIO_Init(), MX_TIM2_Init()
   - HAL_TIM_Base_Start_IT(&htim2)로 TIM2 인터럽트 구동 시작
2. 버튼 입력 발생
   - EXTI 콜백(HAL_GPIO_EXTI_Callback) 호출
   - 디바운싱 조건을 통과하면 상태를 다음 상태로 변경
3. 주기적 갱신
   - TIM2 업데이트 인터럽트(TIM2_IRQHandler → HAL_TIM_IRQHandler → HAL_TIM_PeriodElapsedCallback)
   - 현재 상태에 따라 LED를 OFF/ON/BLINK로 유지/토글

## 학습 포인트
### 1) 2주차 3일차의 핵심: “이벤트 + 상태 + 주기 갱신” 구조
이번 실습의 핵심은 버튼 입력을 “즉시 실행(직접 토글)”로 처리하던 방식에서 벗어나,
- 버튼(EXTI)은 이벤트로써 “상태만 바꾸고”
- 타이머(TIM)는 주기적으로 돌아가며 “상태에 따른 동작을 수행”하는 구조를 구현한 것이다.

이 방식은 기능이 늘어나도 구조가 무너지지 않고(확장성), ISR이 비대해지지 않으며, RTOS 없이도 RTOS스러운 설계 패턴(이벤트 기반, 상태 머신)을 구현할 수 있다.

### 2) 2주차 1일차/2일차/3일차 비교 (중요)
#### (1) 2주차 1일차: EXTI 기본(즉시 처리)
- 구조
  - 버튼 입력 → EXTI ISR → 즉시 LED 토글
- 장점
  - 구현이 단순하고 빠르게 동작 확인 가능
- 한계/문제
  - 바운싱 영향이 직접적으로 드러남(의도치 않은 다중 토글)
  - ISR에서 직접 제어가 늘어나면 코드가 커지고, 다른 기능과 충돌 가능
  - 시간이 걸리는 작업을 ISR에 넣기 시작하면 전체 시스템 응답성이 나빠질 수 있음

정리하면, 1일차는 “인터럽트로 즉시 반응”을 경험하는 단계이며, 실제 시스템 규모가 커지면 유지보수에 취약해질 수 있는 구조다.

#### (2) 2주차 2일차: 디바운싱(시간을 다루는 방법 학습)
- 구조(대표적인 접근)
  - 버튼 입력(EXTI) → 디바운스 타이머 시작/지연 → 안정된 시점에 한 번만 인정
- 핵심 변화
  - 버튼 ISR은 단순 트리거로 역할이 줄고,
  - “시간”을 이용한 안정화(디바운싱)를 도입함
- 의미
  - 임베디드에서 시간 처리는 delay가 아니라 “타이머/인터럽트/스케줄링”으로 해결하는 방향을 학습
  - ISR을 가볍게 유지하는 습관이 시작됨

정리하면, 2일차는 “노이즈/바운싱 같은 현실 문제를 시간 기반으로 해결”하는 단계다.

#### (3) 2주차 3일차: 상태 머신 + 타이머 heartbeat(구조적 설계로 확장)
- 구조
  - 버튼 입력(EXTI) → 상태 전환(OFF/ON/BLINK)
  - 타이머(TIM2) → 10ms마다 주기 인터럽트 발생 → 상태에 따른 LED 갱신
- 2일차 대비 결정적 차이
  - 2일차는 “버튼 입력을 안정화(디바운싱)”시키는 데 초점이 있었다면,
  - 3일차는 “시스템 전체 동작을 주기적으로 갱신하는 중심축(heartbeat)을 타이머로 만든다”는 점이 다르다.
- 효과
  - 메인 루프는 거의 비어도 시스템이 동작함(폴링 최소화)
  - 기능이 늘어나도 ‘상태 추가’와 ‘tick 기반 처리’로 확장 가능
  - ISR 역할 분리가 명확해지고, 코드 구조가 임베디드다운 형태로 발전함

정리하면, 3일차는 “시간 기반 안정화”를 넘어 “이벤트-상태-주기 갱신”이라는 아키텍처 수준의 설계를 배우는 단계다.

### 3) 실무/면접 관점 포인트
- ISR에서는 가능한 한 짧게 처리하고(상태 변경/플래그 설정), 실제 로직은 주기 인터럽트나 메인 컨텍스트에서 처리한다.
- 타이머 tick 기반 설계는 다양한 주기 작업(LED, 센서 샘플링, 통신 타임아웃, watchdog 등)으로 확장 가능하다.
- 상태 머신은 임베디드 시스템에서 동작 모드를 안정적으로 관리하는 핵심 패턴이다.

## 파일 구조
- stm32_timer_state_led/
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
  - stm32_timer_state_led.ioc
  - README.md

