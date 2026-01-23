# stm32_timer_led

## 프로젝트 개요
STM32F401RE(Nucleo) 보드를 사용하여  
**하드웨어 타이머(TIM)를 이용한 주기적 LED 제어**를 학습하는 기초 실습 프로젝트이다.

본 프로젝트에서는 `HAL_Delay()`와 같은 소프트웨어 지연 방식이 아닌  
**Timer Peripheral 기반의 정확한 주기 제어 구조**를 구현하였다.

---

## 개발 환경
- Board: STM32 Nucleo-F401RE
- MCU: STM32F401RE (Cortex-M4)
- IDE: STM32CubeIDE
- Language: C
- HAL Driver 사용
- OS: Windows

---

## 구현 내용

### 1. Timer Base (TIM2)
- TIM2를 Base Timer 모드로 설정
- Prescaler와 Auto-Reload 값을 이용해 1초 주기 생성
- 내부 클럭(APB1 Timer Clock) 사용

### 2. GPIO Output (LED)
- PA5 핀에 연결된 LED 제어
- Push-Pull 출력 방식 사용

### 3. Timer 기반 LED 제어
- Timer 주기마다 LED 토글
- 소프트웨어 Delay 없이 일정한 주기 유지

---

## 동작 방식
1. 시스템 초기화 (`HAL_Init`, `SystemClock_Config`)
2. GPIO 및 TIM2 초기화
3. TIM2 Base Timer 시작
4. Timer 주기 도달 시 LED 상태 변경
5. 동일한 주기로 반복 동작

---

## 학습 포인트
- Timer Peripheral의 역할과 필요성 이해
- Delay 기반 제어의 한계 인식
- Prescaler / Auto-Reload 값 계산 흐름 이해
- 주기적 태스크 구현의 기본 구조 습득
- 이후 Timer Interrupt(ISR) 확장의 기초 마련

---

## 파일 구조
stm32_timer_led/
├─ Core/
│ ├─ Inc/
│ └─ Src/
├─ Drivers/
├─ stm32_timer_led.ioc
├─ README.md
└─ .gitignore
