# stm32_exti_debounce

## 1. 프로젝트 개요
본 프로젝트는 STM32 Nucleo-F401RE 보드를 사용하여  
**외부 인터럽트(EXTI)를 이용한 버튼 입력 처리와 소프트웨어 디바운싱**을 구현하는 것을 목표로 한다.

USER 버튼(PC13)을 누르면 LED(PA5)가 토글되며,  
버튼 채터링으로 인해 발생하는 중복 입력을 **HAL_GetTick() 기반 시간 비교 방식**으로 제거하였다.

본 프로젝트는 2주차 2일차 실습에 해당하며,  
인터럽트 기반 입력 처리의 기본 흐름과 실무에서 사용하는 디바운싱 패턴을 학습하는 데 목적이 있다.

---

## 2. 개발 환경
- Board: Nucleo-F401RE
- MCU: STM32F401RE
- IDE: STM32CubeIDE
- Configuration Tool: STM32CubeMX
- Language: C
- Library: STM32 HAL Driver

---

## 3. 구현 내용
### 3.1 GPIO 설정
- LED
  - Pin: PA5 (LD2)
  - Mode: GPIO Output (Push-Pull)

- Button
  - Pin: PC13 (B1)
  - Mode: GPIO External Interrupt
  - Trigger: Falling Edge
  - Pull-up: Board 기본 회로 사용 (No Pull)

### 3.2 인터럽트 설정
- EXTI Line: EXTI15_10
- NVIC에서 EXTI15_10 IRQ Enable

### 3.3 디바운싱 로직
- 버튼이 눌린 마지막 시각을 전역 변수로 저장
- 현재 Tick 값과 비교하여 일정 시간(debounce_delay_ms) 이내의 입력은 무시
- 인터럽트 내부에서 HAL_Delay()는 사용하지 않음

---

## 4. 동작 방식
1. USER 버튼(PC13)이 눌리면 Falling Edge 발생
2. EXTI13 인터럽트 요청
3. `EXTI15_10_IRQHandler()` 실행
4. `HAL_GPIO_EXTI_Callback()` 함수 호출
5. 이전 버튼 입력 시각과 현재 시각 비교
6. 디바운싱 조건을 만족할 경우 LED(PA5) 토글

메인 루프에서는 별도의 동작을 수행하지 않으며,  
모든 입력 처리는 인터럽트 기반으로 이루어진다.

---

## 5. 학습 포인트
- EXTI 인터럽트의 전체 호출 흐름 이해
- 인터럽트 핸들러와 HAL 콜백 함수의 역할 구분
- 버튼 채터링이 발생하는 원인과 디바운싱의 필요성
- 인터럽트 내부에서 블로킹 함수(HAL_Delay)를 사용하지 않는 이유
- HAL_GetTick()을 이용한 시간 기반 입력 필터링 기법

---

## 6. 파일 구조
stm32_exti_debounce/
├── Core
│ ├── Inc
│ │ └── main.h
│ └── Src
│ ├── main.c
│ ├── gpio.c
│ └── stm32f4xx_it.c
├── stm32_exti_debounce.ioc
└── README.md

---

## 7. 확장 아이디어
- 타이머(TIM)를 이용한 디바운싱 처리
- 버튼 길게 누르기(Long Press) 판별
- 더블 클릭(Double Click) 인식
- RTOS 환경에서의 이벤트 처리 방식 비교
