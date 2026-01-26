# stm32_button_debounce

## 프로젝트 개요
본 프로젝트는 STM32F401RE(NUCLEO-F401RE) 보드에서  
버튼 입력을 안정적으로 처리하기 위해 EXTI와 Timer를 결합한  
소프트웨어 디바운싱 구조를 구현한 실습이다.

버튼 입력은 이벤트로만 감지하고,  
입력 판단과 상태 전이는 Timer 기반 로직으로 처리하여  
ISR 최소화와 상태 머신 구조를 적용하였다.

---

## 개발 환경
- Board: NUCLEO-F401RE
- MCU: STM32F401RE
- IDE: STM32CubeIDE
- HAL Driver 사용
- Language: C

---

## 핀 구성
| 기능 | 핀 |
|------|----|
| LED (LD2) | PA5 |
| USER Button | PC13 (Active-Low) |
| Timer | TIM2 |

---

## Timer 설정
- Prescaler: 8399
- Period: 99
- Timer Tick: 10 ms

84 MHz / (8399 + 1) = 10 kHz  
0.1 ms × 100 = 10 ms 주기

---

## 구현 내용

### 1. 버튼 입력 처리
버튼 입력은 EXTI 인터럽트로 감지하며,  
ISR에서는 상태 판단 없이 이벤트 발생만 기록한다.

    void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
    {
        if (GPIO_Pin == GPIO_PIN_13)
        {
            g_debounce_active = 1;
            g_debounce_cnt = 0;
        }
    }

---

### 2. 소프트웨어 디바운싱
TIM2 인터럽트(10ms 주기)에서 버튼 상태를 샘플링하고,  
50ms 동안 동일 상태가 유지되면 유효 입력으로 확정한다.

    #define DEBOUNCE_TICKS 5   // 50ms

물리적 버튼 튐(bounce)에 의한 오동작을 방지한다.

---

### 3. 짧게 누르기 / 길게 누르기 판정
버튼이 눌린 시간을 Timer tick으로 측정하고,  
버튼이 떼어지는 시점에서 입력 길이를 판정한다.

| 구분 | 기준 |
|------|------|
| Short Press | < 1초 |
| Long Press | ≥ 1초 |

    #define LONG_PRESS_TICKS 100  // 1s

---

### 4. 상태 머신(State Machine) 기반 LED 제어
LED 동작은 버튼 입력과 직접 연결하지 않고  
상태 전이(State Transition) 방식으로 제어한다.

    typedef enum {
        STATE_LED_OFF,
        STATE_LED_ON,
        STATE_LED_BLINK
    } system_state_t;

상태 전이 규칙:
- Short Press: OFF → ON → BLINK → OFF
- Long Press: 어떤 상태에서든 OFF로 복귀

---

### 5. Timer 기반 LED Blink
BLINK 상태일 때만 Timer에서 토글 요청을 생성하고,  
실제 LED 토글은 main loop에서 수행한다.

    if (g_state == STATE_LED_BLINK)
    {
        g_blink_cnt++;
        if (g_blink_cnt >= BLINK_TOGGLE_TICKS)
        {
            g_blink_cnt = 0;
            g_led_toggle_req = 1;
        }
    }

---

## 동작 방식 요약
1. 버튼 입력 발생 → EXTI 인터럽트
2. Timer에서 디바운싱 처리
3. 버튼 떼는 시점에 Short / Long Press 판정
4. 이벤트에 따른 상태 전이
5. 상태에 따른 LED ON / OFF / BLINK

---

## 학습 포인트
- ISR에서 최소한의 처리만 수행하는 구조
- Timer 기반 소프트웨어 디바운싱의 필요성
- 입력 이벤트와 동작 로직의 분리
- 상태 머신 기반 임베디드 설계 방식 이해

---

## 파일 구조
    stm32_button_debounce/
     ├─ Core/
     │   ├─ Src/
     │   │   ├─ main.c
     │   │   ├─ stm32f4xx_it.c
     │   └─ Inc/
     │       └─ main.h
     ├─ stm32_button_debounce.ioc
     └─ README.md

---

## 정리
본 프로젝트를 통해  
단순한 버튼 입력을 넘어,  
안정성과 확장성을 고려한 임베디드 입력 처리 구조를 구현하였다.
