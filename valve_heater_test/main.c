/*
 * heater_test
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

    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3| GPIO_PIN_2| GPIO_PIN_1| GPIO_PIN_0);
//      GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
//      GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2);
//      GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1);
//      GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0);

//    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
//    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0x0);
//    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_3);
//    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);
    int i=0;

    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0x0);
    while(i<60){
        delayMs(1000);
        i++;
    }

    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0x0);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, 0x0);
    while(1) {}

    return 0;
}

void delayMs(uint32_t ui32Ms) {

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
