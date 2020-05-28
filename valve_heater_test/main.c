/**
 * main.c
 This code is written for EPIC project for VTMEMS
 @authors, Shaunak, Rohit, Varun, Tanvi, Mustahsin
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "AD7793.h"

//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2 (Serial Clock)
//! - SSI0Fss - PA3 (Chip Select)
//! - SSI0Rx  - PA4 (DIN of ADC)
//! - SSI0Tx  - PA5 (DOUT of ADC)
//!

uint32_t ui32SysClock;  // System Clock

volatile bool timeout = false;
int timer_count = 0;

// AD7793 Registers

// ID Register
uint32_t pui32_ad7793_id;

// Status Register
uint32_t pui32_ad7793_status;

// Mode Register
uint32_t pui32_mode_reg_msb;
uint32_t pui32_mode_reg_lsb;

// Configure Register
uint32_t pui32_conf_reg_msb;
uint32_t pui32_conf_reg_lsb;

// IO Register
uint32_t pui32_io_reg;

// Data Registers
uint32_t pui32_data_reg;

void initPeripherals()
{
    // Configure system clock

    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Enable SSI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Enable Port A as it supports SSI and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // SSI Pins Configurations
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);     //  SSI Clock
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);     //  SSI CS
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);   //  SSI MOSI
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);   //  SSI MISO

    // Initialize SSI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Set SSI Clock
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
        SSI_MODE_MASTER, 1000000, 8);   // Sets the SSI protocol, mode of operation, bit rate, and data width.

    // Start SSI
    SSIEnable(SSI0_BASE);
}

// Defining Delay functions which are used between READ and WRITE operations
void DelayUS(uint32_t us)              // Delay in microseconds
{
    float f = 1000000 / (float) us;    //  Frequency = 1 / Period

    f = (float) ui32SysClock / (3.0 * f);
    SysCtlDelay((uint32_t) f);
}

void DelayMS(uint32_t ui32Ms)           // Delay in milliseconds 
{

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

void DelayS(uint32_t ui32S)             // Delay in seconds 
{
    float f;
    f = (float)ui32S * ((float)ui32SysClock / 3.0);
    f = (uint32_t) f;
    SysCtlDelay(f);
}


void resetAD7793()
{
    int i = 0;
    uint32_t pui32_get_ssi_garbage;
    uint8_t data_to_send[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

    for (i = 0; i < 4; i++)
    {
        SSIDataPut(SSI0_BASE, data_to_send[i]); // SSIDataPut function is used to write to registers
        while (SSIBusy(SSI0_BASE)) {}
    }
    /*clear fifo*/
    for (i = 0; i < 4; i++)
    {
        SSIDataGet(SSI0_BASE, &pui32_get_ssi_garbage); // clearing FIFO after sending 4 bytes of reset instruction
        while (SSIBusy(SSI0_BASE)) {}
    }
}


void writeToCommRegister(uint32_t reg_value)
{
    SSIDataPut(SSI0_BASE, reg_value);
    while (SSIBusy(SSI0_BASE)) {}
}

void readID()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_id_value = 0x60; // Reads ID reg

    // First we write to the communication register
    writeToCommRegister(ui32_read_id_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);
    SSIDataPut(SSI0_BASE, 0);

    // Read the ID
    SSIDataGet(SSI0_BASE, &pui32_ad7793_id);
    while(SSIBusy(SSI0_BASE)) {}

    DelayUS(100);
}

void readStatus()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_status_value = 0x40; // Reads Status reg

    // First we write to the communication register
    writeToCommRegister(ui32_read_status_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);
    SSIDataPut(SSI0_BASE, 0);

    // Read the Status
    SSIDataGet(SSI0_BASE, &pui32_ad7793_status);
    while(SSIBusy(SSI0_BASE)) {}

    DelayUS(100);

    uint8_t ready_bit = (pui32_ad7793_status >> 7) & 0x01;

//    UARTprintf("Ready Bit is %d \t", ready_bit);
}

void writeToConfigReg()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_write_config_value = 0x10;    // Write to COMM Reg to indicate write to config reg
    uint32_t pui32_config_write_msb = 0x11;     // Gain: 2 and Unipolar
    uint32_t pui32_config_write_lsb = 0x10;     // Buffered Mode on

    writeToCommRegister(ui32_write_config_value);

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    // Write to CONFIGURATION register
    SSIDataPut(SSI0_BASE, pui32_config_write_msb);
    SSIDataPut(SSI0_BASE, pui32_config_write_lsb);

    while(SSIBusy(SSI0_BASE));

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    DelayUS(100);
}

void readConfig()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_config_value = 0x50; // Reads Status reg

    // First we write to the communication register
    writeToCommRegister(ui32_read_config_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    SSIDataPut(SSI0_BASE, 0);
    SSIDataPut(SSI0_BASE, 0);

    // Read the config register
    SSIDataGet(SSI0_BASE, &pui32_conf_reg_msb);
    SSIDataGet(SSI0_BASE, &pui32_conf_reg_lsb);
    while(SSIBusy(SSI0_BASE)) {}

    DelayUS(100);
}

void writeToModeReg()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_write_mode_value = 0x08;    // Write to COMM Reg to indicate write to mode reg
    uint32_t pui32_mode_write_msb = 0x20;     // Mode: Single Conversion
    uint32_t pui32_mode_write_lsb = 0x0A;     // Update rate: 16.7 and Internal clock

    writeToCommRegister(ui32_write_mode_value);

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    // Write to CONFIGURATION register
    SSIDataPut(SSI0_BASE, pui32_mode_write_msb);
    SSIDataPut(SSI0_BASE, pui32_mode_write_lsb);

    while(SSIBusy(SSI0_BASE));

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    DelayUS(100);
}

void readMode()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_mode_value = 0x48; // Reads Mode Reg

    // First we write to the communication register
    writeToCommRegister(ui32_read_mode_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    SSIDataPut(SSI0_BASE, 0);
    SSIDataPut(SSI0_BASE, 0);

    // Read the config register
    SSIDataGet(SSI0_BASE, &pui32_mode_reg_msb);
    SSIDataGet(SSI0_BASE, &pui32_mode_reg_lsb);
    while(SSIBusy(SSI0_BASE)) {}

    DelayUS(100);
}

void writeToIOReg()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_write_io_value = 0x28;    // Write to COMM Reg to indicate write to mode reg
    uint32_t pui32_io_write = 0x02;     // Mode: Single Conversion

    writeToCommRegister(ui32_write_io_value);

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    // Write to CONFIGURATION register
    SSIDataPut(SSI0_BASE, pui32_io_write);

    while(SSIBusy(SSI0_BASE));

    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    DelayUS(100);
}

void readIO()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_io_value = 0x68; // Reads IO Reg

    // First we write to the communication register
    writeToCommRegister(ui32_read_io_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    SSIDataPut(SSI0_BASE, 0);

    // Read the config register
    SSIDataGet(SSI0_BASE, &pui32_io_reg);
    while(SSIBusy(SSI0_BASE)) {}

    DelayUS(100);
}


uint32_t readData()
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_read_data_value = 0x58; // Reads Data Reg
    uint32_t pui32_data_msb;
    uint32_t pui32_data_csb;
    uint32_t pui32_data_lsb;

    // First we write to the communication register
    writeToCommRegister(ui32_read_data_value);

    // Clear the garbage data from the SSI FIFO
    SSIDataGet(SSI0_BASE, &pui32_ssi_garbage);

    SSIDataPut(SSI0_BASE, 0);
    SSIDataPut(SSI0_BASE, 0);
    SSIDataPut(SSI0_BASE, 0);

    // Read the config register
    SSIDataGet(SSI0_BASE, &pui32_data_msb);
    SSIDataGet(SSI0_BASE, &pui32_data_csb);
    SSIDataGet(SSI0_BASE, &pui32_data_lsb);
    while(SSIBusy(SSI0_BASE)) {}

    pui32_data_reg &= 0x0;

    pui32_data_reg |= (pui32_data_msb << 16) | (pui32_data_csb << 8) | pui32_data_lsb;

    DelayUS(100);

//    UARTprintf("ADC Data is %d\n", pui32_data_reg);

    return pui32_data_reg;
}

void configureAD7793()
{
    // Read ID register
    readID();

    // Read Status register
    readStatus();

    // Write to Config register
    writeToConfigReg();

    // Read Config register
    readConfig();

    // Write to IO register
    writeToIOReg();

    // Read IO Register
    readIO();
}

void initUART()
{
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //    UARTConfigSetExpClk(UART0_BASE, ui32SysClk, 115200,
    //                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTStdioConfig(0, 115200, ui32SysClock);
}

void initSolenoid()
{
    // Enabling required peripheral ports.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    // AIN1 and AIN2 are the direction pins of the motor driver break out board.
    // STBY is for enabling and disabling the motor driver board.
    // STBY - HIGH for enabling the board.
    // STBY - LOW for disabling the board.
    // PWM pin is used to supply the voltage pulse to the solenoid valve.
    // Defining the required pins as output pins.

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);                             // PWM pin of Motor Driver
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2);     // IN1, IN2, STBY pins of Motor Driver
}

int Valve(int valve_direction)
{
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);      // PWM tied to high
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, GPIO_PIN_2);      // STBY tied to high

    if(valve_direction == 1)                                    // Flow from port B of the solenoid valve
    {
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);  // IN2 high
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // IN1 low

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // IN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // IN1 low

        DelayMS(30);
    }

    else                                                        // Flow from port A of the solenoid valve
    {
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // IN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);  // IN1 high

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // IN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // IN1 low

        DelayMS(30);
    }

    return 0;
}

void initHeater()
{
    // Heater is connected to the Pin 3 of Port L
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    DelayUS(100);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);
    DelayUS(100);
}

void heaterON()
{
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
    DelayUS(100);
}

void heaterOFF()
{
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0);
    DelayUS(100);
}

const int resistance_upper_threshold = 2235000;

void maintainTemp(uint32_t current_adc_data)
{
    uint32_t adc_data = current_adc_data;
    bool local_timeout = false;

    UARTprintf("Trying to maintain the temperature at %d\n", adc_data);
    UARTprintf("-------------Starting 15 sec delay \n");

    TimerEnable(TIMER0_BASE, TIMER_A);

    while(!(local_timeout))
    {
        UARTprintf("%d\n", adc_data);

        if(timer_count == 1)        // Decides the number of times the timer will run
        {
            break;
        }

        configureAD7793();

        // Write to Mode register
        writeToModeReg();

        DelayUS(120000);

        // Read Data Register
        adc_data = readData();

        if(adc_data < resistance_upper_threshold)
        {
            UARTprintf("Starting ON the heater\n");
            heaterON();
        }

        if(adc_data > (resistance_upper_threshold))
        {
            UARTprintf("Shutting OFF the heater\n");
            heaterOFF();
        }
    }
    UARTprintf("-------------Done with 15 sec delay\n");
}

void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    UARTprintf("___________Inside Timer Interrupt___________\n");

    timer_count++;          // Increment timer count at each interrupt
}

void initInterrupt()
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = (ui32SysClock * 30);                  // Period = Clock * Timer duration (seconds)
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
}

void initSystem()
{
    // Initialize all the peripherals for the clock, SSI and UART
    initPeripherals();
    DelayUS(1000);

    // Initialize UART functions
    initUART();
    DelayUS(1000);

    // Reset AD7793 chip and wait for 800uS
    resetAD7793();
    DelayUS(800);

    initSolenoid();
    DelayUS(800);

    // Always initialize heater after initializing the solenoid valves.
    initHeater();
    DelayUS(800);

    // Initialize the timer interrupt
    initInterrupt();
}

int main()
{
    initSystem();

    // State 1: Adsorption Process
    Valve(2);                                                   // Flow from terminal A of the solenoid valve which is located at the bottom

    UARTprintf("Adsorption Process\n");
    DelayS(90);
    DelayS(90);                                                 // Adsorption for 3 minutes

    // State 2: Desorption Switching
    UARTprintf("Starting Desorption Process\n");
    Valve(1);                                                   // Switch to flow from terminal B of the solenoid valve

    uint32_t adc_data = 0;

    UARTprintf("Now starting the heater\n");
    heaterON();
    DelayMS(1000);

    int garbage_index;
    UARTprintf("-----Garbage reads----\n");                     // Throwing first 10 values in the garbage
    for(garbage_index = 0; garbage_index < 10; garbage_index++)
    {
        DelayMS(10);

        configureAD7793();

        // Write to Mode register
        writeToModeReg();

        DelayUS(120000);

        // Read Data Register
        adc_data = readData();
    }

    UARTprintf("-----Actual reads----\n");
    adc_data = 0;

    while(adc_data < resistance_upper_threshold)                // Letting the heater reach the threshold
    {
        DelayMS(10);

        configureAD7793();

        // Write to Mode register
        writeToModeReg();

        DelayUS(120000);

        // Read Data Register
        adc_data = readData();
        UARTprintf("%d\n", adc_data);
    }

    UARTprintf("Reached desired temperature\n");
    maintainTemp(adc_data);                                     // Maintaining the heater at the desired temperature

    UARTprintf("Switch the solenoid valve instantly\n");
    
    // State 3-4: Involves quick switching of the Solenoid valve from Position B to A to B; 2 = position A
    
    Valve(2);           // Switches the valve to port A       
    DelayMS(30);
    Valve(1);           // Switches the valve to port B
    DelayMS(30);

    heaterOFF();        // Final heater OFF

    while(adc_data > 0)
    {
        DelayMS(10);

        configureAD7793();

        // Write to Mode register
        writeToModeReg();

        DelayUS(120000);

        // Read Data Register
        adc_data = readData();
        UARTprintf("%d\n", adc_data);   // Print ADC data values after turning the heater off
    }

    return 0;
}
