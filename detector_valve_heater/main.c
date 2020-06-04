/**
 * main.c
 This code is written for EPIC project for VTMEMS
 @authors, Shaunak, Rohit, Varun, Tanvi, Mustahsin
 */


/*  Connections
 *
 *  Valve 1 - Pin 2: PL0,   Pin 1: PL1,     STBY: PL2,      PWM: PN2
 *  Valve 2 - Pin 2: PM0,   Pin 1: PM1,     STBY: PM2,      PWM: PN3
 *  Valve 3 - Pin 2: PK0,   Pin 1: PK1,     STBY: PK2,      PWM: PN4
 *  
 *  Passing value 1 to any valve function will switch it to Port B
 *  Passing any other value will switch the valve to Port A 
 *  
 *  Pump - PK4 (GND), PK5 (VCC)
 *  
 *  ADC Chip selects - PM4, PM5
 *
 *  Heater - PL3
 *
 *  SSI connections given ahead
 *
 *  NOTE
 *  Delay functions can't give a delay higher than 100 seconds in one function call
 *  If delay required is longer than 100 secs, give multiple delay calls
 *
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

    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Enable SSI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Enable Port A as it supports SSI and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // SSI Pins Configurations
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); //  SSI Clock
    GPIOPinConfigure(GPIO_PA3_SSI0FSS); //  SSI CS
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);   //  SSI MOSI
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);   // SSI MISO

    // Initialize SSI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

    // Set SSI Clock
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
        SSI_MODE_MASTER, 1000000, 8); // Sets the SSI protocol, mode of operation, bit rate, and data width.

    // Start SSI
    SSIEnable(SSI0_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4| GPIO_PIN_5); // Chip select pins

}

// Defining Delay function which is used between READ and WRITE operations
void DelayUS(uint32_t us)   //1000
{
    float f = 1000000 / (float) us;   //  Frequency = 1 / Period

    f = (float) ui32SysClock / (3.0 * f);
    SysCtlDelay((uint32_t) f);
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

void DelayS(uint32_t ui32S) {

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

uint32_t mode_msb;
uint32_t mode_lsb;
uint32_t config_msb;
uint32_t config_lsb;

void writeToConfigReg(uint32_t msb, uint32_t lsb)
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_write_config_value = 0x10;    // Write to COMM Reg to indicate write to config reg
    uint32_t pui32_config_write_msb = msb;     
    uint32_t pui32_config_write_lsb = lsb;     

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

void writeToModeReg(uint32_t msb, uint32_t lsb)
{
    uint32_t pui32_ssi_garbage;
    uint32_t ui32_write_mode_value = 0x08;    // Write to COMM Reg to indicate write to mode reg
    uint32_t pui32_mode_write_msb = msb;     
    uint32_t pui32_mode_write_lsb = lsb;     

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
    uint32_t pui32_io_write = 0x02;     // Excitation current: 210uA

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

    return pui32_data_reg;
}

void configureRTD()                                             // Configuring registers of the RTD 
{
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, GPIO_PIN_4);      // Chip select: Turn TCD off
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0x0);             // Turn RTD on

    mode_msb = 0x20;        // Mode: Single Conversion
    mode_lsb = 0x0A;        // Update rate: 16.7 Hz and Internal clock

    config_msb = 0x11;      // Gain: 2 and Unipolar mode
    config_lsb = 0x10;      // Buffered Mode on

    // Read ID register
    readID();

    // Read Status register
    readStatus();

    // Write to Config register
    writeToConfigReg(config_msb, config_lsb);

    // Read Config register
    readConfig();

    // Write to IO register
    writeToIOReg();

    // Read IO Register
    readIO();

    // Write to Mode register
    writeToModeReg(mode_msb, mode_lsb);

}

void configureTCD()                                         // Configuring registers of the TCD 
{
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_4, 0x0);         // Chip select: Turn TCD on
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);  // Turn RTD off

    mode_msb = 0x20;        // Mode: Single Conversion
    mode_lsb = 0x01;        // Update rate: 470Hz and Internal clock

//    config_msb = 0x01;      // Gain: 2 and Bipolar
    config_msb = 0x11;      // Gain: 2 and Unipolar
    config_lsb = 0x80;      // Internal reference, Buffered Mode off

    // Read ID register
    readID();

    // Read Status register
    readStatus();

    // Write to Config register
    writeToConfigReg(config_msb, config_lsb);

    // Read Config register
    readConfig();

    // Write to Mode register
    writeToModeReg(mode_msb, mode_lsb);

}


void initUART()
{
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, ui32SysClock);
}

void initSolenoid()
{
    // Enabling required peripheral ports.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    // AIN1 and AIN2 are the direction pins of the motor driver break out board.
    // STBY is for enabling and disabling the motor driver board.
    // STBY - HIGH for enabling the board.
    // STBY - LOW for disabling the board.
    // PWM pin is used to supply the voltage pulse to the solenoid valve.
    // Defining the required pins as output pins.

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4); // PWM pins of all valves
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2); // AIN2, AIN1, STBY pins of Valve 1
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2); // AIN2, AIN1, STBY pins of Valve 2
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2| GPIO_PIN_4| GPIO_PIN_5); // AIN2, AIN1, STBY pins of Valve 3, Inputs of Pump

}

int Valve1(int valve_direction)
{
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);  // PWM tied to high
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, GPIO_PIN_2);  // STBY high

    if(valve_direction == 1)    // Flow from port B of the solenoid valve
    {

        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);  // AIN2 high
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

    }

    else    // Flow from port A of the solenoid valve
    {
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1);  // AIN1 high

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

    }

    return 0;
}

int Valve2(int valve_direction)
{
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, GPIO_PIN_3);  // PWM tied to high
    GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_2, GPIO_PIN_2);  // STBY high

    if(valve_direction == 1)    // Flow from port B of the solenoid valve
    {

        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_PIN_0); // AIN2 high
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

    }

    else    // Flow from port A of the solenoid valve
    {

        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, GPIO_PIN_1);  // AIN1 high

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

    }

    return 0;
}

int Valve3(int valve_direction)
{
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_PIN_4);      // PWM tied to high
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);      // STBY high

    if(valve_direction == 1)    // Flow from port B of the solenoid valve
    {

        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, GPIO_PIN_0);  // AIN2 high
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

        DelayMS(30);

    }

    else    // Flow from port A of the solenoid valve
    {

        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);  // AIN1 high

        DelayMS(30);

        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0x0);         // AIN2 low
        GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x0);         // AIN1 low

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

void initPump()
{
    // Pump is connected to Pin 4 and 5 of Port K
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x0);
    DelayUS(100);
}

void pumpON()
{
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
    DelayUS(100);
}

void pumpOFF()
{
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_4, 0x0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0x0);
    DelayUS(100);
}


const int resistance_upper_threshold = 2235000;         // Threshold for temperature 200 degrees Celsius at 18V supply
//const int resistance_upper_threshold = 2240000;
//const int resistance_upper_threshold = 200000;


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

        if(timer_count == 1)
        {
            break;
        }

        configureRTD();

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

    timer_count++;
}

void initInterrupt()
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = (ui32SysClock*30);                     // Period = Clock * Timer duration (seconds)
                                                        // Give duration of maintaining heater temperature in seconds (30 by default)
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();
}

int main()
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

    initPump();
    DelayUS(800);

    // Initialize the timer interrupt
    initInterrupt();

    // Step 0-1: Initialization
    Valve1(1);                                                  // Flow from terminal B of the valve 1
    Valve2(2);                                                  // Flow from terminal A of the valve 2
    Valve3(1);                                                  // Flow from terminal B of the valve 3

    DelayMS(1000);                                              // Change delay between each step as required

    // Step 2: Adsorption Process
    Valve1(2);                                                  // Flow from terminal A of the valve 1
    Valve2(2);                                                  // Flow from terminal A of the valve 2
    Valve3(1);                                                  // Flow from terminal B of the valve 3

    // Turning pump ON
    pumpON();
    
    UARTprintf("Pump turned on before adsorption process\n");

    // DelayS(90);
    // DelayS(90);                                                 // Adsorption for 3 minutes

    uint32_t adc_data = 0;
    int i = 0;

    int ads_delay = 30;                                         // Give Adsorption delay in seconds

    int j = ads_delay * 1000 / 130;                             // j = total delay time (ms) / individual data read time (ms) 
                                                                // We read one data value from the ADC every 130 ms
    for (i = 0; i < j; i++)
    {
        DelayMS(10);

        configureTCD();                                         // Read detector data during adsorption

        DelayMS(120);

        // Read Data Register
        adc_data = readData();
        UARTprintf("Detector: %d\n", adc_data);
    }

    adc_data = 0;

    // Step 3: Desorption Switching
    UARTprintf("Starting Desorption Process\n");
    Valve1(1);                                                   // Flow from terminal B of the solenoid valve 1
    Valve2(1);                                                   // Flow from terminal B of the solenoid valve 2
    Valve3(2);                                                   // Flow from terminal A of the solenoid valve 3

    UARTprintf("Now turning off the pump and starting the heater\n");
    pumpOFF();
    heaterON();
    DelayMS(1000);

    UARTprintf("-----Data reads----\n");
    adc_data = 0;

    while(adc_data < resistance_upper_threshold)                // Letting the heater reach 200 degrees C
    {
        DelayMS(10);

        configureRTD();

        DelayMS(120);

        // Read Data Register
        adc_data = readData();
        UARTprintf("%d\n", adc_data);
    }

    UARTprintf("Reached desired temperature\n");
    maintainTemp(adc_data);                                     // Maintaining heater at 200 degrees 

    UARTprintf("Switch the solenoid valve 2 instantly\n");

    // Step 4-5: Involves quick switching of the Solenoid valve 2 from Position B to A to B; 2 = position A

    Valve1(2);           // Valve 2 switch to A
    DelayMS(30);
    Valve1(1);           // Valve 2 switch to B
    DelayMS(30);

    // Final heater OFF
    heaterOFF();

    while(adc_data > 1750000)           // Print heater data till it reaches room temp
    {
        DelayMS(10);

        configureRTD();

        DelayUS(120000);

        // Read Data Register
        adc_data = readData();
        UARTprintf("%d\n", adc_data);
    }

    adc_data = 0;

    while(1)                            // Print detector output
    {
        DelayMS(10);

        configureTCD();

        DelayMS(120);

        // Read Data Register
        adc_data = readData();
        UARTprintf("Detector: %d\n", adc_data);
    }

    return 0;
}
