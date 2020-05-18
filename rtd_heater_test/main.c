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

void readID();
void readStatus();
void writeToConfigReg();
void readConfig();
void writeToIOReg();
void readIO();

//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2 (Serial Clock)
//! - SSI0Fss - PA3 (Chip Select)
//! - SSI0Rx  - PA4 (DIN of ADC)
//! - SSI0Tx  - PA5 (DOUT of ADC)
//!

//const int resistance_upper_threshold = 10066329;
//const int resistance_upper_threshold = 13528329; // 2.0k
//const int resistance_upper_threshold = 15783950; // 2.2k
//const int resistance_upper_threshold = 14092861;
const int resistance_upper_threshold = 8053950; //2.4k with 2 gain
//const int resistance_offset = 8653950;

uint32_t ui32SysClock;  // System Clock

volatile bool timeout = false;

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

uint32_t int_count;

void initPeripherals()
{
    // Configure system clock
//    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//            SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
//           25000000);

    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Enable SSI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Enable Port A as it supports SSI and UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /* Enable Chip Select pins for the AD7793 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);    // Port N(Pins: 2, 3, 4, 5)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);    // Port P(Pins: 0, 1, 2, 3, 4, 5)

    // Chip Select for RTD Circuits
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2); // Chip Select 6
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5); // Chip Select 7
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4); // Chip Select 8
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_3); // Chip Select 9
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2); // Chip Select 10

    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0);           // Enabling Chip Select 10
//    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_3, GPIO_PIN_3);  // Disabling Chip Select 9
//    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, GPIO_PIN_4);  // Disabling Chip Select 8
//    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_5, GPIO_PIN_5);  // Disabling Chip Select 7
//    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);  // Disabling Chip Select 6


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

void initUART()
{
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //    UARTConfigSetExpClk(UART0_BASE, ui32SysClk, 115200,
    //                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTStdioConfig(0, 115200, ui32SysClock);
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

int main()
{
    // Initialize all the peripherals for the clock, SSI and UART
    initPeripherals();
    /* initialize ADC*/
    DelayUS(1000);

    // Initialize UART functions
    initUART();
    DelayUS(1000);

    // Reset AD7793 chip and wait for 800uS
    resetAD7793();
    DelayUS(800);

    initHeater();
    DelayUS(800);

    DelayMS(10);

    configureAD7793();

       // Write to Mode register
    writeToModeReg();

    DelayMS(120);
    /* initialize ADC*/

    uint32_t adc_data = 0;
    const int resistance_upper_threshold = 150000;

    UARTprintf("Now starting the heater\n");
    UARTprintf("-----Actual reads----\n");

    while(1) {
        DelayMS(10);

        configureAD7793();

        // Write to Mode register
        writeToModeReg();

        DelayMS(120);
        adc_data = readData(); // Read Data Register
        UARTprintf("ADC Data is %d \n", adc_data);

        if(adc_data < resistance_upper_threshold)
        {
            UARTprintf("Starting ON the heater\n");
            heaterON();
        }

        if(adc_data > (resistance_upper_threshold + (resistance_upper_threshold * 0.01)))
        {
            UARTprintf("Shutting OFF the heater\n");
            heaterOFF();
        }
    }

    return 0;
}








    /*
     * Algorithms for the GC heater circuit
     * 1. Initially valve will be at position A and heater at room temperature for time t1
     * 2. Then, valve will be switched to B and heater will be turned ON until its temperature reached 200 degree C.
     * 3. 200 degree C will be maintained for next 5 seconds by an ON-OFF controller with threshold of +-10
     * 4. The valve will be switched to A for a minimum possible time when temperature is around 200.
     * 5. Finally, the valve will be shut off by switching to B and heater will be turned OFF.
     */

    // State 1: Adsorption Process
//    UARTprintf("Starting Adsorption Process\n");
//    First_Solenoid(2); // Flow from terminal A of the solenoid valve which is located at the bottom
//    DelayMS(10000);  // change this delay to 180s afterwards

    // State 2: Desorption Switching
//    First_Solenoid(1); // Flow from terminal B of the solenoid valve
//    DelayMS(6000);

//    uint32_t adc_data = 0;
//
//    UARTprintf("Now starting the heater\n");
//    heaterON();
//    DelayMS(1000);

//    int garbage_index;
//    // Throwing first 10 values in the garbage
//    UARTprintf("-----Garbage reads----\n");
//    for(garbage_index = 0; garbage_index < 10; garbage_index++)
//    {
//        DelayMS(10);
//
//        configureAD7793();
//
//        // Write to Mode register
//        writeToModeReg();
//
//        DelayUS(120000);
//        // Read Mode register
//        // readMode();
//
//        // Read Data Register
//        adc_data = readData();
//    }

//    UARTprintf("-----Actual reads----\n");
//    adc_data = 0;

//    TimerEnable(TIMER0_BASE, TIMER_A); // Comment it out when deploy
//    while(adc_data < resistance_upper_threshold)
//    int curr_int_count = 1;

//    DelayMS(10);
//
//        configureAD7793();
//
//        // Write to Mode register
//        writeToModeReg();
//
//        DelayMS(120);
//        // Read Mode register
//        // readMode();
//
//        // Read Data Register
//        adc_data = readData();
//        UARTprintf("ADC Data is %d \n", adc_data);
//        heaterOFF();

//    while(1)
//    {
//        if(int_count != curr_int_count)
//        {
//            UARTprintf("--------Inside the if switching the GPIO--------\n");
//            if((int_count % 2) == 1)
//            {
//
//                GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);  // Disabling Chip Select 10
//                DelayUS(10000);
//                GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_3, 0);           // Enabling Chip Select 9
//                DelayUS(10000);
//            }
//            else
//            {
//                GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0);           // Enabling Chip Select 10
//                DelayUS(10000);
//                GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_3, GPIO_PIN_3);  // Disabling Chip Select 9
//                DelayUS(10000);
//            }
//            curr_int_count = int_count;
//        }

//    }

//    UARTprintf("Reached desired temperature\n");
//    maintainTemp(adc_data);

//    UARTprintf("Switch the solenoid valve instantly\n");
//    // State 3-4: Involves quick switching of the Solenoid valve from Position B to A to B
//    First_Solenoid(2);
//    DelayUS(15);
//    First_Solenoid(1);
//    DelayUS(15);

    // Final heater OFF
//    heaterOFF();

//    while(1)
//    {
//        // Read ID register
//        readID();
//
//        // Read Status register
//        readStatus();
//
//        // Write to Config register
//        writeToConfigReg();
//
//        // Read Config register
//        readConfig();
//
//        // Write to IO register
//        writeToIOReg();
//
//        // Read IO Register
//        readIO();
//
//        configureAD7793();
//
//        // Write to Mode register
//        writeToModeReg();
//
//        DelayUS(120000);
//        // Read Mode register
//        // readMode();
//
//        // Read Data Register
//        uint32_t adc_data = 0;
//        adc_data = readData();
//    }

