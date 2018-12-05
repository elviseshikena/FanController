/**
 * FanController
 * MSE 352 Fall 2018 Course Project
 * TM4C123GH6PM
 * License in LICENSE
 * JK.EE
*/

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"

// Macros
#define TACH_FREQ                   10          // Hz Frequency of tachometer interrupt
#define ADC_FREQ                    20000       // Hz Frequency of ADC interrupt
#define CNTRL_FREQ                  2000        // Hz Frequency of control cycle
#define PWM_FREQUENCY               20000       // Hz Frequency of PWM signal
#define TACH_TIMER_LOAD             10000       // Count Edge-count reference
#define NUM_7SEG_MODULES            3           // Number of 7 segment modules
#define BCD_BITS                    4           // Number of bits in the BCD code
#define MAX_TACH_SAMPLES            5           // Samples averaged by the tachometer
#define INIT_ADJUST                 50          // Initialization of the adjust value
#define MIN_ADJUST                  50          // Minimum PWM adjust value
#define MAX_ADJUST                  950        // Maximum PWM adjust value

// Variables ------------------------------------------------------------------
uint32_t ui32PWMClock;
uint32_t ui32Load;
uint32_t ui32Adjust;
uint32_t ui32TachPulses;
uint32_t ui32FanRPM;
uint32_t ui32PotAdcMap;
uint8_t ui8SampleCount;
double pADC;
double pRPM;
double pErr;

uint32_t divisor;
uint32_t digit[NUM_7SEG_MODULES];
uint8_t bits[BCD_BITS];
uint8_t module;
uint8_t val;
uint8_t bitmask;

// Prototypes  ----------------------------------------------------------------
void InitADC(void);
void InitTach(void);
void InitPWM(void);
void InitControl(void);
void InitDisplay(void);
void ADCIntHandler(void);
void TachIntHandler(void);
void ControlIntHandler(void);
void DisplayHandler(void);

// MAIN -----------------------------------------------------------------------
void main(void)
{

    // Set clock to 40MHz
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Initialize peripherals
    InitADC();
    InitTach();
    InitPWM();
    InitDisplay();
    InitControl();

    // Enable all interrupts
    IntMasterEnable();

    // Loop
    while(1)
    {
        DisplayHandler();
    }
}

// FUNCTION DEFS --------------------------------------------------------------
// Initializations ----------------------------------------
// Initialize ADC on PD1
void InitADC(void)
{
    // Enable Peripherals for ADC0 Port E Pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // ADC Initializations, sequencers 3 and processors
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                            ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear interrupt flag before sampling begins
    ADCIntClear(ADC0_BASE, 3);

    // Timer0 Peripheral for 50µs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/ADC_FREQ);
    // Setup interrupt on Timeout
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER0_BASE, TIMER_A, *ADCIntHandler);

    // Enable Timer0A
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    return;
}

// Initialize Tachometer on PB0 with Timer2 and Timer1
void InitTach(void)
{
    // Timer 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet()/TACH_FREQ);
    // Setup the interrupt for the Timer1
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER1_BASE, TIMER_A, *TachIntHandler);

    // Timer 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Configure Input capture on PB0
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PB0_T2CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_0);
    // T2CCP0
    TimerDisable(TIMER2_BASE, TIMER_A);
    TimerConfigure(TIMER2_BASE,
        TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_COUNT);
    TimerControlEvent(TIMER2_BASE,TIMER_A,TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER2_BASE, TIMER_A, TACH_TIMER_LOAD);

    // Enable the timers.
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);

    return;
}

// Initialize PWM on PD0
void InitPWM(void)
{
    ui32Adjust = INIT_ADJUST;
    // Set PWM Clock for 625kHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Setup PWM1 and Port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // PD0 must be configured as PWM output pin for module 1
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    // The PWM clock is SYSCLK/64 (set in step 12 above). Divide the PWM clock by the de-
    // sired frequency (55Hz) to determine the count to be loaded into the Load register. Then
    // subtract 1 since the counter down-counts to zero. Configure module 1 PWM generator 0
    // as a down-counter and load the count value.
    ui32PWMClock = SysCtlClockGet() / 64;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

    // Set pulse width
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);

    // PWM module 1, generator 0 needs to be enabled as an output and enabled to run.
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);

    // Enable PWM1
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);

    return;
}

// Initialize PA1-4 & PC1-3 for digital 7-Segment display
void InitDisplay(void)
{
    // Setup GPIO for Port A and Port C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Configure PA4-7 for BCD
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,
        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTC_BASE,
        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_WAKE_LOW);

    // Configure PC4-6 for selectors
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, 
        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTC_BASE, 
        GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_WAKE_LOW);

    return;
}

// Initialize control event Timer4
void InitControl(void)
{
    // Setup timer for control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER4_BASE, TIMER_A, SysCtlClockGet()/CNTRL_FREQ);
    // Setup Interrupt
    IntEnable(INT_TIMER4A);
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER4_BASE, TIMER_A, *ControlIntHandler);

    // Enable Timer4A
    TimerEnable(TIMER4_BASE, TIMER_A);

    return;
}

// Interrupt Routines -------------------------------------
// Handle ADC timer interrupt Timer0
void ADCIntHandler(void)
{
    // Disable and clear timer interrupt
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Clear ADC interrupts and trigger ADC processors
    ADCIntClear(ADC0_BASE, 3);
    ADCProcessorTrigger(ADC0_BASE, 3);

    // Wait until ADC processing is complete
    while(!ADCIntStatus(ADC0_BASE, 3, false));

    // Obtain ADC maps
    ADCSequenceDataGet(ADC0_BASE, 3, &ui32PotAdcMap);

    // Re-enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    return;
}

// Read pulse counter every Timer1
void TachIntHandler(void)
{
    // Clear timer
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ui8SampleCount++;

    ui32TachPulses += (TACH_TIMER_LOAD - TimerValueGet(TIMER2_BASE,TIMER_A)) * 100 / MAX_TACH_SAMPLES;  // *100 converts kHz to Hz for easy displaying

    if (ui8SampleCount == MAX_TACH_SAMPLES)
    {
        ui32FanRPM = (ui32TachPulses * 30) / (TACH_FREQ) ;
        ui8SampleCount = 0;
        ui32TachPulses = 0;
    }

    // Reset edge counter
    TimerLoadSet(TIMER2_BASE, TIMER_A, TACH_TIMER_LOAD);

    return;
}

// Adjust the speed of the fan Timer4
void ControlIntHandler(void)
{
    // Disable and clear timer interrupt
    TimerIntDisable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    ui32Adjust = ((MAX_ADJUST-MIN_ADJUST) * ui32PotAdcMap)/4095.0 + MIN_ADJUST;

    // Calculate %ADC
    pADC = 100 - 100 * ui32PotAdcMap / 4095.0;
    // Calculate %RPM
    pRPM = 100 * ui32FanRPM / 2400.;
    // Calculate %err
    pErr = pADC - pRPM;

    // Check boundaries
    if (ui32Adjust < MIN_ADJUST)
    {
        ui32Adjust = MIN_ADJUST;
    }
    else if (ui32Adjust > MAX_ADJUST)
    {
        ui32Adjust = MAX_ADJUST;
    }

    // Modify pulse width
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32Adjust * ui32Load / 1000);

    // Re-enable timer interrupt
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    return;
}

// Helper Routines ----------------------------------------
// Display speed on the 7-Segment Timer3
void DisplayHandler(void)
{
    // Clear and disable timer
    int i;
    int j;

    divisor = 10000;

    // Split into digits
    for (i=0; i < NUM_7SEG_MODULES; i++)
    {
        // Convert digit
        digit[i] = (ui32FanRPM - ((ui32FanRPM/divisor) * divisor))/(divisor/10);
        divisor/=10;
    }

    module = GPIO_PIN_4;

    // Write on 7-Segments
    for (i=0; i < NUM_7SEG_MODULES; i++)
    {
        // reset mask
        bitmask = 0;

        // Obtain bits of the binary value
        val = (uint8_t)digit[i];

        for (j=0; j < BCD_BITS; j++)
        {
          bits[j] = val & 0x1;
          // If there was an on bit add it to the mask
          if (bits[j])
          {
              switch(j)
              {
                  case 0: bitmask |= GPIO_PIN_7; break;
                  case 1: bitmask |= GPIO_PIN_6; break;
                  case 2: bitmask |= GPIO_PIN_5; break;
                  case 3: bitmask |= GPIO_PIN_4; break;
                  default: break;
              }
          }
          // Shift to next value bit;
          val >>= 1;
        }


        // Write on display pins ABCD
        GPIOPinWrite(GPIO_PORTA_BASE,
                    GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, bitmask); // A  MSB
        // Select module and set pin values
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, module);

        module <<= 1; // Left-shift bits for next GPIO Pin (2<<4<<8);
        SysCtlDelay(40000);
    }

    return;
}
