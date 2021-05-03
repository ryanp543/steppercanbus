# CANBus Controlled Stepper Motor Driver 

## Overview

This repository contains the schematic and layout for the custom stepper motor driver developed for the robotic arm mounted on the MIT Bioinstrumentation Lab rover. This information is found in the `Stepper CANBus V2` folder.

The driver had to be able to 1) fit in a CANBus network, 2) require the fewest numbers of wires to operate (in this case, power, ground, and the two CANBus lines), and 3) run on 21 V. What resulted from these constraints is a stepper driver that can be used universally with 24 V bipolar stepper motors and can be easily modified via the interface of the mounted Teensy. Despite the fact that the PCB shape is tailored to this robotic arm, multiple of these drivers can be linked together in any desired daisy chained CANBus network, as long as the last driver in the chain has the switch turned on to activate the 120 Ohm terminator resistor.

The microcontroller chosen for this driver was a Teensy 3.2 based on its widely used interface, easy customization, and relatively small size for a breakout board microcontroller. A Texas Instruments DRV8825 microstepper driver carrier board was selected so it could also be easily mounted onto the custom PCB and take commands from the Teensy. For the step-down power converter, a buck switching regulator (specifically Texas Instruments chip LM2594MX-5.0/NOPBCT-ND) was picked to provide a fixed 500 mA at 5 V, stepped down from an input voltage of around 20 to 24 V. External components such as capacitors and diodes were placed in the external circuit according to the data sheet.

The firmware that is uploaded to the Teensy from the Arduino IDE can be found in the `Stepper_CANBus_Firmware` directory. 

## Contributors

#### Primary Contributors
Ryan Poon from the [MIT Bioinstrumentation Lab](https://bioinstrumentation.mit.edu/) is the primary contributor of this project.

#### Other Contributors
Professor Ian Hunter served as the main advisor.