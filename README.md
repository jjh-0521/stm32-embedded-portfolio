# STM32 Embedded Portfolio (NUCLEO-F401RE)

STM32 NUCLEO-F401RE 기반 펌웨어 포트폴리오 프로젝트입니다.  
CubeMX로 프로젝트 뼈대를 생성하고, 기능을 **단계적으로 추가**하면서 Git 커밋으로 작업 과정을 기록합니다.

---

## 1. Target & Environment
- Board: NUCLEO-F401RE
- MCU: STM32F401RE
- IDE: STM32CubeIDE
- Language: C (HAL 기반, 이후 Register 버전도 추가 예정)

---

## 2. Project Goals
- GPIO / UART / Timer 기초 주변장치 구현
- Interrupt 기반 설계(EXTI, Timer ISR) 및 디바운싱
- HAL 구현과 Register 직접 제어 구현 비교
- 디버깅 이슈 및 해결 과정을 문서화(README + 커밋 로그)

---

## 3. Current Status
- [x] Project initialized with CubeMX (.ioc committed)
- [x] GPIO LED control
- [x] EXTI button interrupt + debounce
- [ ] UART debug logging
- [ ] Timer periodic task
- [ ] HAL vs Register comparison


---

## 4. Repository Structure (planned)
- `Core/` : application code (main, interrupt handlers, user code)
- `Drivers/` : CMSIS + HAL drivers
- `Docs/` (to be added) : diagrams, debugging notes, screenshots

---

## 5. Build & Run
1) Open the project in STM32CubeIDE  
2) Build (Debug configuration)  
3) Flash via ST-LINK (Nucleo onboard)

---

## 6. Notes / Troubleshooting
(추후 디버깅 이슈와 해결 과정을 여기에 누적)
- Example:
  - Issue: ...
  - Root cause: ...
  - Fix: ...

---

## 7. License
Personal portfolio project.
