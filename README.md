# FanController
Working with the TM4C microcontroller to control a DC fan. MSE 352 Fall 2018 Course project

## Schematic
![Schematic](/Schematic-FanController.png)


## Description
The goal of this project was to control a DC fan using a TM4C ARM microcontroller and a potentiometer. The speed of the fan was then also to be displayed on three 7-segment displays dropping the last digit, since the rated speed of the fan was 2400 rpm. In programming the system, I researched how to use the Tiva C Series API to control the peripherals from the controller.

First the ADC was used to read in an analog signal by configuring the chosen pin connected to the potentiometer. This signal was used to control the PWM by calculating the percentage error between the ADC and the speed of the fan based on duty cycle. The speed is then controlled by adjusting the PWM signal to follow the ADC input. The PWM output pin was connected to the input signal pin of the fan.

Finally, for the display, each digit of the speed value was converted into its 4-bit corresponding values and written to the GPIO pins which were inputs to a single 7447 IC. This chip decodes 4-bit BCD codes into the output for the 7-segments. To show the speed values on the displays, instead of using three 7447 chips on each display, only one chip was connected in parallel to all three displays and the power pins on the displays were used as selectors. The display loop wrote each digit of the speed (240) - BCD value to the same 4 GPIO pins and cycled through each selector pin to display the digit.


## Suggested improvements
 - Using less interrupts to allow the debugger read values while program runs.
 - Using prescalers to adjust processing overhead.
