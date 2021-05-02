# TurbochargerController
This project implements an absolute pressure turbocharger controller, suited for aircraft use. 
---
## Hardware Design:
![Block Diagram](/images/blockdiagram.PNG)

### Microcontroller
The microcontroller selected for this project is the NucleoL412KB. This is an STM32 board based around the cortex M4 core, packaged inside an arduino nano form factor. This was preferable for our project, since two headers can be placed on the PCB instead of a full microcontroller implementation. Pins were selected such that the timer peripheral, analog to digital converter, USART, and GPIO could be utilized most effectively for each of the applications below. Power to the system is supplied through the external 5v input, 3.3V power is generated through the MCU's onboard voltage regulator.

![MCU Schematic](/images/mcuschematic.PNG)

### Pressure Sensor
The pressure sensor used in this design is the NPP301. This sensor is an absolute pressure sensor capable of sensing up to 100psi. The device acts as a wheatstone bridge, where the resistance varies based on pressure. The sensor is connected to a two-stage amplifier stage, with a potentiometer available for sensitivity adjustment. This output feeds into the ADC12_channel1 of the microcontroller. 

![Pressure Sensor Schematic](/images/pressureschematic.PNG)

### Stepper Motor Driver:
The stepper motor driving stage is created using an A4988 "Stepstick" shield. Three general purpose output pins are used to drive the enable, direction, and step pins. The output of the motor is routed to a four-place screw terminal block for easy wiring to motors. Microstepping settings are selectable using a semi-permanent solder bridge feature on the back of the board. Power for the stepper motor is supplied through a separate motor supply line through the DB9.

![Stepper Motor Schematic](/images/stepperschematic.PNG)

### Solenoid Driver
The bypass valve is actuated using a 12v 32Hz PWM signal. In order to drive this load, a BJT driving stage was designed for this specific load and microcontroller. A diode is included on the load circuit to prevent reverse current due to the inductance of the solenoid valve. The base of the BJT is driven through a resistor by the general purpose TIM2_CH1. Power for driving the solenoid is sourced from an external high current supply pin through the DB9.

![Solenoid Driver Schematic](/images/solenoidschematic.PNG)

### RS232 Communications
A MAX3232 RS232 driver was utilized to convert the UART signal from the microcontroller to RS232 levels. This converter utilizes four external 0.1uF capacitors with a charge pump to increase the 3.3V supply to the correct level for communication.

![RS232 Driver Schematic](/images/rs232schematic.PNG)
