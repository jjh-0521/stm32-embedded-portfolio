# stm32_timer_base_interrupt

## 1. 프로젝트 개요
본 프로젝트는 STM32F401RE(NUCLEO-F401RE) 보드를 사용하여  
**타이머(TIM2) 기반 주기 인터럽트 시스템**을 구현한 예제이다.

기존의 `while(1) + delay` 구조에서 벗어나,  
**하드웨어 타이머가 시간 흐름을 관리하고 CPU는 이벤트에 반응하는 구조**로 전환하는 것이 핵심 목표이다.

TIM2를 1ms 주기의 기준 시계(time base)로 설정하고,  
이 주기 인터럽트를 활용하여 500ms마다 LED(LD2)를 토글한다.

---

## 2. 개발 환경
- Board: NUCLEO-F401RE
- MCU: STM32F401RE
- IDE: STM32CubeIDE
- HAL Driver 사용
- Clock Source: HSI + PLL
  - SYSCLK = 84 MHz
  - APB1 = 42 MHz (TIM2CLK = 84 MHz)

---

## 3. 구현 내용

### 3.1 Timer 기본 구조
- TIM2를 **Up-counting Time Base 모드**로 설정
- Prescaler와 Auto Reload Register(ARR)를 이용해 **1ms 주기 인터럽트** 생성

타이머 주기 계산식:
Update Frequency = TIM2CLK / (PSC + 1) / (ARR + 1)

설정 값:
- TIM2CLK = 84 MHz
- Prescaler = 8399 → 10 kHz
- Period(ARR) = 9 → 1 kHz (1 ms)

---

### 3.2 인터럽트 기반 시간 관리
- TIM2 Update Event 발생 시 `HAL_TIM_PeriodElapsedCallback()` 호출
- 인터럽트 내부에서 소프트웨어 카운터(ms 단위) 증가
- 500ms 경과 시 LED 토글 수행

CPU는 메인 루프에서 시간을 기다리지 않으며,  
모든 시간 관련 로직은 **타이머 인터럽트에 의해 구동**된다.

---

### 3.3 GPIO 및 외부 인터럽트
- LD2 (PA5): 출력 GPIO, LED 토글 확인용
- USER Button (PC13):
  - EXTI 인터럽트로 동작
  - 버튼 입력 시 LED 토글 동작 ON / OFF 제어

---

## 4. 동작 방식

1. 시스템 클럭 및 GPIO 초기화
2. TIM2 초기화 후 인터럽트 모드로 시작
3. TIM2가 1ms마다 Update Interrupt 발생
4. 인터럽트 콜백에서 ms 카운터 증가
5. 500ms마다 LD2 LED 토글
6. 메인 while 루프는 비워두고 이벤트 대기

이 구조에서 **시간 흐름은 하드웨어가 담당**하며,  
CPU는 오직 인터럽트 발생 시에만 개입한다.

---

## 5. 이번 실습의 핵심 특징

### 5.1 Polling → Interrupt 기반 구조 전환
- delay 함수에 의존하지 않는 구조
- CPU 점유 없이 정확한 주기 동작 가능

### 5.2 Timer를 “기준 시계”로 사용
- Timer를 단순 보조 수단이 아닌 **시스템 시간의 기준**으로 활용
- 이후 PWM, 스케줄러, RTOS 개념으로 자연스럽게 확장 가능

### 5.3 확장 가능한 설계
- 1ms tick 기반 구조
- 여러 주기 작업(10ms, 100ms, 1s 등)을 소프트웨어적으로 쉽게 추가 가능
- 실무 임베디드 시스템에서 가장 흔히 쓰이는 패턴

---

## 6. 효능 및 학습 효과

### 6.1 임베디드 사고 방식의 전환
- “코드를 순서대로 실행한다” → “이벤트와 시간에 반응한다”
- MCU를 단순한 프로그램 실행기가 아닌 **실시간 시스템**으로 이해하게 됨

### 6.2 Timer 레지스터 구조 이해의 발판
- Prescaler, Counter, ARR의 역할을 명확히 체감
- 이후 레지스터 직접 제어 방식 학습 시 이해 속도 대폭 향상


