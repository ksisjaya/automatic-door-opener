# Summary
This project is for the course *ECE 298: Instrumentation and Prototyping*. The objective is to prototype an automatic door opener system using the [STM32F401RE MCU](https://www.st.com/en/microcontrollers-microprocessors/stm32f401re.html). Additional sensors and actuators have also been included to operate the system which is simulated in the [Proteus Design Suite](https://www.labcenter.com/). Labs go through the prototyping design process, beginning with electrical schematics and ending with Gerber plots to fabricate a PCB to host the system.

## Lab 1: Design Requirements
- Define functional, non-functional and constraint requirements
- Choose components and corresponding parameters of operation
- Define connections with the MCU and any internal MCU resources required (e.g. ADC, timers, interrupts, GPIO functions)
- Verification and validation of proper component operation, along with simulations of this
- System level block diagram

## Lab 2: MCU Interfacing
- Determine appropriate hardware interfacing with MCU pins
- Step-down voltage interfaces
- Step-up voltage interfaces
- Analog to digital interfaces
- Motor controller IC interfaces
- Simple simulations and proof-of-concept for these interfaces

## Lab 3: Prototyping
- Connecting all components with the appropriate hardware interfaces to the MCU pins
- Writing embedded software to be run on the MCU (see [Operational Overview.pdf](https://github.com/ksisjaya/automatic-door-opener/blob/main/Lab3/Operational%20Overview.pdf) for an overview of the design)
- Simulate test cases to ensure edge cases have been dealt with

## Lab 4: PCB Design
- Prepare schematics from Lab 3 to be processed into Gerber plots
- Add terminal blocks and rectangular connectors for connections
- Add decoupling capacitors
- Add nets and test points
- Generate a two layer PCB
- Route traces on the PCB
- Add vias where needed
- Generate Gerber plots
- Generate pick and place files for manufacturing
- Generate a bill of materials