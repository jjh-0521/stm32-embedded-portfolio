# stm32_timer_pwm_control

## 1. 프로젝트 개요
본 프로젝트는 STM32F401RE(NUCLEO-F401RE) 보드를 사용하여  
**타이머 PWM(Output Compare) 기능을 이용한 출력 제어**를 구현한 예제이다.

3주차 1일차에서 구현한 **Timer Base Interrupt(time base)** 구조를 바탕으로,  
이번 실습에서는 타이머의 **채널(Channel)과 Compare Register(CCR)** 를 활용하여  
**주기 신호(PWM)를 실제 출력으로 생성하고 제어**하는 데 초점을 둔다.

TIM2를 PWM 모드로 설정하고,  
CCR 값을 변경하여 **출력 신호의 듀티비(Duty Cycle)** 를 제어한다.

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

### 3.1 Timer PWM 구조
- TIM2를 **PWM Generation Mode (CH1)** 로 설정
- Prescaler(PSC)와 Auto Reload Register(ARR)로 PWM 주파수 결정
- Compare Register(CCR1)를 통해 듀티비 제어

PWM 주파수 계산식:
- f_pwm = TIM2CLK / (PSC + 1) / (ARR + 1)

듀티비 계산식:
- Duty(%) = CCR / (ARR + 1) × 100

---

### 3.2 PWM 출력 설정
- TIM2_CH1 핀을 **Alternate Function(AF)** 으로 설정
- GPIO 출력이 아닌, **타이머 하드웨어가 직접 파형을 생성**
- CPU 개입 없이 PWM 신호가 지속적으로 출력됨

---

### 3.3 듀티비 제어 방식
- CubeMX에서 `Pulse` 값으로 **CCR의 초기값** 설정
- 실행 중에는 `__HAL_TIM_SET_COMPARE()` 매크로를 사용해 CCR 값을 변경
- 버튼 입력(EXTI)을 통해 듀티비를 단계적으로 변경
  - 10% → 50% → 90% → 반복

---

## 4. 동작 방식

1. 시스템 클럭 및 GPIO 초기화
2. TIM2를 PWM 모드로 초기화
3. `HAL_TIM_PWM_Start()` 호출로 PWM 출력 시작
4. 기본 듀티비로 PWM 파형 출력
5. 사용자 버튼 입력 시 CCR 값 변경
6. 듀티비 변화에 따라 PWM 출력 특성 변화

메인 `while(1)` 루프는 비워두고,  
PWM 출력은 **하드웨어 타이머가 지속적으로 유지**한다.

---

## 5. 이번 실습의 핵심 특징

### 5.1 Time Base → Output Control 확장
- 1일차: 타이머를 “언제 이벤트가 발생하는지”에 사용
- 2일차: 타이머를 “어떤 파형을 어떻게 출력할지”에 사용

즉, **시간 관리 도구 → 출력 생성 장치**로 타이머의 역할이 확장된다.

---

### 5.2 Compare Register(CCR)의 역할 명확화
- ARR: 주기(Period)를 결정
- CCR: High 구간의 길이를 결정
- CCR 값을 바꾸는 것만으로 출력 특성을 실시간 변경 가능

이는 PWM, 모터 제어, LED 밝기 제어 등에서 핵심이 되는 개념이다.

---

### 5.3 CPU 비개입 하드웨어 출력
- PWM 파형은 CPU가 토글하지 않음
- 인터럽트나 while 루프와 무관하게 출력 유지
- 실시간성과 안정성이 매우 높음

---

## 6. 효능 및 학습 효과

### 6.1 Timer 내부 구조에 대한 실질적 이해
- Counter, ARR, CCR의 관계를 “이론”이 아닌 “출력 결과”로 체감
- 이후 레지스터 직접 제어 학습 시 이해 난이도 대폭 감소

---

### 6.2 실무형 PWM 사고 방식 습득
- “핀을 켜고 끈다”가 아니라  
  “주기 신호의 비율을 제어한다”는 관점 정립
- 모터 속도 제어, LED 밝기 제어, 전력 제어의 공통 원리 이해

---

### 6.3 1일차 대비 구조적 성장
- 단순 인터럽트 처리에서 벗어나
- **타이머 채널 + 하드웨어 출력**을 사용하는 단계로 진입
- 임베디드 시스템에서 매우 빈번히 사용되는 패턴 습득

---

