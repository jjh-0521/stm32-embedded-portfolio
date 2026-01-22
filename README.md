# stm32_led_test

##  프로젝트 개요
STM32F401RE(Nucleo) 보드를 사용하여  
GPIO 입력/출력과 EXTI 인터럽트 기반 버튼 제어를 학습하는 기초 실습 프로젝트이다.

본 프로젝트에서는 polling 방식이 아닌  
**외부 인터럽트(EXTI)를 이용한 이벤트 기반 구조**를 구현하였다.

---

##  개발 환경
- Board: STM32 Nucleo-F401RE
- MCU: STM32F401RE (Cortex-M4)
- IDE: STM32CubeIDE
- HAL Driver 사용
- OS: Windows

---

##  구현 내용

### 1. GPIO Output (LED)
- PA5 핀에 연결된 LED 제어
- Push-Pull 출력 방식 사용

### 2. GPIO Input (Button)
- PC13 핀 (User Button)
- Internal Pull-up 사용
- Falling Edge 트리거

### 3. EXTI 인터럽트
- 버튼 입력을 External Interrupt 방식으로 처리
- NVIC에서 EXTI line[15:10] 활성화
- `HAL_GPIO_EXTI_Callback()`에서 이벤트 처리

### 4. 소프트웨어 디바운싱
- `HAL_GetTick()` 기반 디바운싱 적용
- Delay 기반 디바운싱의 단점을 회피

---

##  동작 방식
1. 버튼(PC13)을 누르면 Falling Edge 발생
2. EXTI 인터럽트 트리거
3. `HAL_GPIO_EXTI_Callback()` 호출
4. 디바운싱 조건 통과 시 LED 토글

---

##  학습 포인트
- Polling 방식과 Interrupt 방식의 차이 이해
- 이벤트 기반 펌웨어 구조 경험
- EXTI 및 NVIC 설정 흐름 이해
- 하드웨어 입력의 디바운싱 필요성

---


---

##  참고
- 인터럽트 기반 구조에서는 `while(1)` 루프가 비어 있을 수 있음
- EXTI 콜백 함수에서 GPIO 핀을 구분하여 처리하는 것이 중요함

---

##  Next Step
- Timer 인터럽트 기반 주기 제어
- Delay 없는 LED 제어
- 상태 머신(FSM) 구조 적용

