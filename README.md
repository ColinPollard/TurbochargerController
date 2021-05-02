# TurbochargerController
This project implements an absolute pressure turbocharger controller, suited for aircraft use. 
---
## Hardware Design:
![Block Diagram](/images/blockdiagram.png)
### Pressure Sensor
The pressure sensor used in this design is the NPP301. This sensor is an absolute pressure sensor capable of sensing up to 100psi. The device acts as a wheatstone bridge, where the resistance varies based on pressure. The sensor is connected to a two-stage amplifier stage, with a potentiometer available for sensitivity adjustment. This output feeds into the ADC12_channel1 of the microcontroller. 
![Pressure Sensor Schematic](/images/pressureschematic.png)
### Stepper Motor Driver:
The stepper motor driving stage is created using an A4988 "Stepstick" shield. Three general purpose output pins are used to drive the enable, direction, and step pins. The output of the motor is routed to a four-place screw terminal block for easy wiring to motors. Microstepping settings are selectable using a semi-permanent solder bridge feature on the back of the board.
![Stepper Motor Schematic](/images/stepperschematic.png)
### Solenoid Driver
The bypass valve is actuated using a 12v 32Hz PWM signal. In order to drive this load, a BJT driving stage was designed for this specific load and microcontroller. A diode is included on the load circuit to prevent reverse current due to the inductance of the solenoid valve. The base of the BJT is driven through a resistor by the general purpose TIM2_CH1.
![Solenoid Driver Schematic](/images/solenoidschematic.png)
### RS232 Communications
A MAX3232 RS232 driver was utilized to convert the UART signal from the microcontroller to RS232 levels. This converter utilizes four external 0.1uF capacitors with a charge pump to increase the 3.3V supply to the correct level for communication.
![RS232 Driver Schematic](/images/rs232schematic.png)
* An absolute pressure sensor
* Stepper motor driver
* PWM solenoid driver
* RS232 driver

