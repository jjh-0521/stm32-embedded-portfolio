# hal_timer_led

## 프로젝트 개요
STM32F401RE(Nucleo) 보드를 사용하여  
**HAL 라이브러리를 기반으로 TIM2 타이머 인터럽트를 구성하고,  
1초 주기로 LED를 토글하는 기능을 구현한 기초 실습 프로젝트**이다.

본 프로젝트에서는 `HAL_Delay()`와 같은 소프트웨어 지연 방식이 아닌  
**하드웨어 타이머(TIM)와 인터럽트 기반의 주기 제어 구조**를 구현하는 것을 목표로 한다.

---

## 개발 환경
- Board: STM32 NUCLEO-F401RE
- MCU: STM32F401RE (Cortex-M4)
- IDE: STM32CubeIDE
- Language: C
- Framework: STM32 HAL
- Clock Source: HSI + PLL (`SystemClock_Config` 기본 설정)
- OS: Windows

---

## 구현 내용

### 1. Timer Base (TIM2)
- TIM2를 Internal Clock 기반 타임베이스 타이머로 설정
- Prescaler 및 Auto-Reload 값을 이용해 1초 주기 생성
- APB1 Timer Clock 사용

### 2. GPIO Output (LED)
- PA5 핀에 연결된 LD2 LED 사용
- Push-Pull 출력 방식 설정

### 3. HAL 기반 Timer Interrupt
- `HAL_TIM_Base_Start_IT()`를 통해 타이머 인터럽트 시작
- `HAL_TIM_PeriodElapsedCallback()` 콜백 함수에서 LED 토글
- 인터럽트 플래그 및 ISR 연결은 HAL이 내부적으로 처리

---

## 동작 방식
1. 시스템 초기화 (`HAL_Init`, `SystemClock_Config`)
2. GPIO 초기화 (LD2 핀 Output)
3. TIM2 초기화 (HAL)
4. TIM2 타이머 인터럽트 시작
5. TIM2 업데이트 이벤트 발생
6. HAL 콜백 함수 자동 호출
7. 콜백 함수 내부에서 LED 토글 수행

메인 루프는 비워두고, 주기 동작은 인터럽트 기반으로 처리된다.

---

## 학습 포인트
- HAL 타이머 인터럽트의 기본 사용 흐름 이해
- HAL이 내부적으로 수행하는 역할(ISR 연결, 플래그 처리) 파악
- 주기 작업을 메인 루프가 아닌 인터럽트에서 처리하는 구조의 장점
- 빠른 개발과 가독성을 중시한 HAL 방식의 특징 이해

---

## 파일 구조
stm32_timer_compare/hal_timer_led/
├─ Core/
│  ├─ Inc/
│  └─ Src/
│     └─ main.c
├─ Drivers/
├─ hal_timer_led.ioc
├─ README.md
└─ .gitignore
# reg_timer_led

## 프로젝트 개요
STM32F401RE(Nucleo) 보드를 사용하여  
**TIM2 타이머를 레지스터 직접 제어 방식으로 설정하고,  
1초 주기의 인터럽트 기반 LED 토글 기능을 구현한 실습 프로젝트**이다.

HAL 기반 구현과 동일한 동작을 수행하지만,  
**RCC, TIM, NVIC 레지스터를 직접 설정함으로써  
타이머와 인터럽트의 내부 동작 원리를 이해하는 것**을 목표로 한다.

---

## 개발 환경
- Board: STM32 NUCLEO-F401RE
- MCU: STM32F401RE (Cortex-M4)
- IDE: STM32CubeIDE
- Language: C
- Programming Level: CMSIS / Register-level
- Clock Source: HSI + PLL
- OS: Windows

---

## 구현 내용

### 1. Timer Base (TIM2, Register-level)
- RCC 레지스터를 이용해 TIM2 클럭 직접 활성화
- Prescaler, Auto-Reload, Counter 레지스터 직접 설정
- APB1 Timer Clock 기반 1초 주기 생성

### 2. NVIC 설정
- NVIC 레지스터를 직접 설정하여 TIM2 인터럽트 활성화
- HAL API를 사용하지 않고 인터럽트 허용

### 3. Interrupt Service Routine (ISR)
- `TIM2_IRQHandler()` 직접 구현
- Update Interrupt Flag(UIF) 직접 클리어
- 인터럽트 발생 시 LED 토글

---

## 동작 방식
1. 시스템 초기화 (`HAL_Init`, `SystemClock_Config`)
2. GPIO 초기화 (LD2 핀 Output)
3. TIM2 레지스터 직접 설정
4. NVIC에서 TIM2 인터럽트 활성화
5. TIM2 카운터 동작 시작
6. Update Event 발생 시 `TIM2_IRQHandler()` 진입
7. 인터럽트 플래그(UIF) 클리어 후 LED 토글

HAL과 달리 인터럽트 플래그를 개발자가 직접 관리해야 한다는 점이 핵심이다.

---

## 학습 포인트
- APB1 Timer Clock 구조와 타이머 클럭 동작 이해
- HAL 없이도 타이머 및 인터럽트 구성 가능함을 확인
- 인터럽트 플래그를 클리어하지 않을 경우 발생하는 문제 이해
- HAL이 내부적으로 처리해주는 작업의 범위 명확화
- 저수준 제어가 필요한 상황에서 레지스터 접근의 중요성 인식

---

## 파일 구조
stm32_timer_compare/reg_timer_led/
├─ Core/
│  ├─ Inc/
│  └─ Src/
│     └─ main.c
├─ README.md
└─ .gitignore
