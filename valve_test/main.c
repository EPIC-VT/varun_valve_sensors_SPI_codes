/*
 * valve_test
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
uint32_t ui32SysClock;
void delayMs(uint32_t ui32Ms);

int main(void)
{
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_5| GPIO_PIN_4| GPIO_PIN_3| GPIO_PIN_2| GPIO_PIN_1| GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_5| GPIO_PIN_4| GPIO_PIN_3| GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7| GPIO_PIN_6| GPIO_PIN_5| GPIO_PIN_4| GPIO_PIN_3| GPIO_PIN_2| GPIO_PIN_1| GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3| GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4| GPIO_PIN_3| GPIO_PIN_2| GPIO_PIN_1| GPIO_PIN_0);

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PWM high for valve 1 
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PWM high for valve 2 
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_3, GPIO_PIN_3);  // PWM high for valve 3 
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);  // PWM high for valve 4 
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PWM high for valve 5

    // Use STBY pins according to the ICs used
	// If 2 valves are operated with a single IC, they use only one STBY pin 
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);  // STBY high for valve 1 
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  // STBY high for valve 2
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);  // STBY high for valve 3
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3);  // STBY high for valve 4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);  // STBY high for valve 5

   // Pump in IC1
   int i=0;

   GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);  // Pump ON
   GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0x0);         // Pump ON

   while(i<3){
       delayMs(1000);
       i++;
   }
	
   GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0x0);         // Pump OFF
   GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0x0);         // Pump OFF

   DelayMS(2000);

    
	
	
	// Valve 1 in IC1
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);  // BIN1 high port B
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // BIN2 low

    DelayMS(30);

    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // BIN1 low turning the solenoid current off
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // BIN1 low

    DelayMS(30);

    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // BIN1 low switching to port A
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);  // BIN2 high

    DelayMS(30);

    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // BIN1 low
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // BIN1 low

    DelayMS(5000);

    
    
    
    // Valve 2 in IC5
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);  // AIN1 high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN2 low

    DelayMS(20);

	GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);         // AIN1 low
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

	DelayMS(20);
	
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);  		// AIN1 low
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);  // AIN2 high

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);         // AIN1 low
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

    DelayMS(2000);

    
    
    
    // Valve 3 in IC5
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);  // BIN1 high
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0x0);         // BIN2 low

    DelayMS(20);

	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0x0);         // BIN1 low
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0x0);         // BIN2 low

	DelayMS(20);
	
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0x0);  				// BIN1 high
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, GPIO_PIN_5);          // BIN2 low

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0x0);         // BIN1 low
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0x0);         // BIN2 low

    DelayMS(2000);

    
    
    
    // Valve 4 in IC4
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, GPIO_PIN_4);  // AIN1 high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0x0);         // AIN2 low
    //GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_5, GPIO_PIN_5);  // STBY high

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x0);  				// AIN1 high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);          // AIN2 low

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x0);         // AIN1 low
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0x0);         // AIN2 low

    DelayMS(2000);


	
	
	// Valve 5 in IC4
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);  // BIN1 high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x0);         // BIN2 low

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x0);         // BIN1 high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);  // BIN2 low

    DelayMS(20);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x0);         // BIN1 low
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x0);         // BIN2 low
    
    return 0;
}

void DelayMS(uint32_t ui32Ms) {

    // 1 clock cycle = 1 / SysCtlClockGet() second
    // 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
    // 1 second = SysCtlClockGet() / 3
    // 0.001 second = 1 ms = SysCtlClockGet() / 3000.0
    // ui32Ms is the variable which represents the number of milliseconds required
    // Number of milliseconds required = ui32Ms* SysCtlClockGet() / 3000.0

    float f;
    f = (float)ui32Ms * ((float)ui32SysClock / 3000.0);
    f = (uint32_t) f;
    SysCtlDelay(f);
}
