/**
 * main.c
 This code is written for EPIC project for VTMEMS
 @authors, Shaunak, Rohit, Varun, Tanvi, Mustahsin
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "AD7793.h"


//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2 (Serial Clock)
//! - SSI0Fss - PA3 (Chip Select)
//! - SSI0Rx  - PA4 (DIN of ADC)
//! - SSI0Tx  - PA5 (DOUT of ADC)
//!

uint32_t ui32SysClock; // Used for configuring the clock
uint8_t pui32DataTx; // Used to write to the communication (COMM) register
uint32_t pui32DataRx;
uint32_t ui32Index;

uint32_t pui32DataRxID; // To read the ID register

uint32_t pui32DataRxSTAT; // To read the STATUS register

uint32_t pui32DataRxMODE1; // To read 8 bits of the MODE register
uint32_t pui32DataRxMODE2; // To read the other 8 bits of the MODE register
uint32_t pui32DataTxMODE1 = 0x20; //To be written to the MODE register
uint32_t pui32DataTxMODE2 = 0x01; //To be written to the MODE register

uint32_t pui32DataRxCONF1, pui32DataRxCONF_1; // To read 8 bits of the CONF register
uint32_t pui32DataRxCONF2, pui32DataRxCONF_2; // To read the other 8 bits of the CONF register
uint32_t pui32DataTxCONF1 = 0x80; //To be written to the CONF register
uint32_t pui32DataTxCONF2 = 0x01; //To be written to the CONF register

uint32_t pui32DataRxDATA1; // To read the DATA register
uint32_t pui32DataRxDATA2; // To read the DATA register
uint32_t pui32DataRxDATA3; // To read the DATA register
uint32_t pui32_comp_data;

uint32_t pui32DataRxIO; //To read the IO register
uint32_t pui32DataTxIO = 0x01; //To write to IO register

uint8_t pui32DataTxSTAT;
uint8_t data_to_send[4] = { 0xFF, 0xFF, 0xFF, 0xFF };

const float slope = 0.0000000697;
float voltage_value;
float voltage;

/*function prototypes*/
static int CS;
void Epic_Init();
void SPI_init();

void AD77_Reset();
/*function prototypes*/
void AD77_Reset()
{
    int i = 0;
    for (i = 0; i < 4; i++)
    {
        SSIDataPut(SSI0_BASE, data_to_send[i]); // SSIDataPut function is used to write to registers
        while (SSIBusy(SSI0_BASE))
        {
        }
    }
    /*clear fifo*/
    for (i = 0; i < 4; i++)
    {
        SSIDataGet(SSI0_BASE, &pui32DataRx); // clearing FIFO after sending 4 bytes of reset instruction
        while (SSIBusy(SSI0_BASE))
        {
        }
    }
    /*clear fifo end*/
}

void Epic_Init()
{    //Configuring the clock for the circuit.
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
    SYSCTL_OSC_MAIN |
    SYSCTL_USE_OSC),
                                      25000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // Enables peripheral components of the SSI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enables peripheral components of the SSI

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2); // Configures GPIO pin for use as a GPIO output
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF); // Writes 0xFF to the GPIO pin
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0xFF);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); // Associates CLK with pin PA2
    GPIOPinConfigure(GPIO_PA3_SSI0FSS); // Associates chip select with pin PA3
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0); // Associates MOSI with pin PA4
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1); // Associates MISO with pin PA5

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
    GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_3,
    SSI_MODE_MASTER,
                       1000000, 8); // Sets the SSI protocol, mode of operation, bit rate, and data width.

    SSIEnable(SSI0_BASE); // Enables the SSI interface
}

//Defining Delay function which is used between READ and WRITE operations
void DelayUS(uint32_t us)   //1000
{
    float f = 1000000 / (float) us;   //  Frequency = 1 / Period

    f = (float) ui32SysClock / (3.0 * f);
    SysCtlDelay((uint32_t) f);
}
/*AD_initialization*/

void AD77_Init(int ChipSelect)
{
    if(ChipSelect == 1)
    {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
    }

    if(ChipSelect == 2)
    {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_3, 0x00);
    }
}

void WriteToCommRegister(uint32_t commval)
{
    pui32DataTx = commval;
    SSIDataPut(SSI0_BASE, pui32DataTx);
    while (SSIBusy(SSI0_BASE)) {}
}

void ReadID(uint32_t pui32DataTx)
{
    WriteToCommRegister(0x60); //Binary: 01100000

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response. This clears the FIFO of the TM4c129.
    SSIDataPut(SSI0_BASE, 0); // dummy write

    SSIDataGet(SSI0_BASE, &pui32DataRxID); // Reading the ID in the ID register

    while (SSIBusy(SSI0_BASE)) {} //Execution
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x60, 1);
//    AD7793_Reset();
//    AD7793_GetRegisterValue(AD7793_REG_ID, 1);


}

void ReadSTATUS(uint32_t pui32DataTx)
{
    WriteToCommRegister(0x40); //Binary: 01000000
    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response

    // READ the STATUS register pui32DataRxSTAT
    SSIDataPut(SSI0_BASE, 0); // dummy write
    while (SSIBusy(SSI0_BASE)) {}

    SSIDataGet(SSI0_BASE, &pui32DataRxSTAT);
    while (SSIBusy(SSI0_BASE)) {}
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x40, 1);
//    AD7793_Reset();
//    AD7793_GetRegisterValue(AD7793_REG_STAT, 1);

}


void WriteMODE(uint32_t pui32DataTx, uint32_t pui32DataTxMODE2, uint32_t pui32DataTxMODE1)
{
    WriteToCommRegister(0x08); //Binary: 00001000

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    //DelayUS(800); // Delay between WRITE and READ operations

    // WRITE to MODE register
    SSIDataPut(SSI0_BASE, pui32DataTxMODE2); //Writing LSB of 16-bit register
    SSIDataPut(SSI0_BASE, pui32DataTxMODE1); //Writing MSB of 16-bit register

    while (SSIBusy(SSI0_BASE)) {} //Execution

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x08, 1);
//    AD7793_Reset();
//    AD7793_SetRegisterValue(AD7793_REG_MODE, 0x2001, 2);
}

void ReadMODE(uint32_t pui32DataTx)
{
    WriteToCommRegister(0x48); //Binary: 01001000

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    //DelayUS(800); // Delay between WRITE and READ operations
            // READ from MODE register

    SSIDataPut(SSI0_BASE, 0); // dummy write
    SSIDataPut(SSI0_BASE, 0); // dummy write

    SSIDataGet(SSI0_BASE, &pui32DataRxMODE1); // Reading MSB of 16-bit register in 8-bit int
    SSIDataGet(SSI0_BASE, &pui32DataRxMODE2); // Reading LSB of 16-bit register in 8-bit int

    while (SSIBusy(SSI0_BASE)) {} // Execution
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x48, 1);
//    AD7793_Reset();
//    AD7793_GetRegisterValue(AD7793_REG_MODE, 2);
}

void WriteCONFIG(uint32_t pui32DataTx, uint32_t pui32DataTxCONF2, uint32_t pui32DataTxCONF1)
{
    WriteToCommRegister(0x10); //Binary: 00010000

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    //DelayUS(800);

    // WRITE to CONFIGURATION register
    SSIDataPut(SSI0_BASE, pui32DataTxCONF2); // Writing LSB
    SSIDataPut(SSI0_BASE, pui32DataTxCONF1); // Writing MSB

    while (SSIBusy(SSI0_BASE)) {}

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x10, 1);
//    AD7793_Reset();
//    AD7793_SetRegisterValue(AD7793_REG_CONF, 0x0180, 2);
}

void ReadCONFIG(uint32_t pui32DataTx)
{
    WriteToCommRegister(0x50); //Binary: 01010000

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    //DelayUS(800);

    // READ from CONFIGURATION register
    SSIDataPut(SSI0_BASE, 0); // dummy write
    SSIDataPut(SSI0_BASE, 0); // dummy write

    SSIDataGet(SSI0_BASE, &pui32DataRxCONF1); // Reading LSBs of 16-bit register in 8-bit int
    SSIDataGet(SSI0_BASE, &pui32DataRxCONF2); // Reading MSBs of 16-bit register in 8-bit int

    while (SSIBusy(SSI0_BASE)) {} // Execution
    DelayUS(800);

//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x50, 1);
//    AD7793_Reset();
//    AD7793_GetRegisterValue(AD7793_REG_CONF, 2);
}

void ReadDATA(uint32_t pui32DataTx)
{
    WriteToCommRegister(0x58); //Binary: 01011000 (Single conversion mode)
    //WriteToCommRegister(0x5C); //Binary: 01011100 (Continuous conversion mode)

    SSIDataGet(SSI0_BASE, &pui32DataRx); // dummy response
    //DelayUS(800);

    //Reading the 24-bit DATA register in three 8-bit ints
    SSIDataPut(SSI0_BASE, 0); // dummy write
    SSIDataPut(SSI0_BASE, 0); // dummy write
    SSIDataPut(SSI0_BASE, 0); // dummy write

    SSIDataGet(SSI0_BASE, &pui32DataRxDATA1);
    SSIDataGet(SSI0_BASE, &pui32DataRxDATA2);
    SSIDataGet(SSI0_BASE, &pui32DataRxDATA3);

    while (SSIBusy(SSI0_BASE)) {} //Execution
    pui32_comp_data &= 0x0;
    pui32_comp_data |= (pui32DataRxDATA1 << 16) | (pui32DataRxDATA2 << 8) | pui32DataRxDATA3;
    if (pui32_comp_data >= 8388608)
    {
        float uni_data = pui32_comp_data - 8388608;
        voltage = (float)uni_data*slope;
        voltage_value = voltage;
    }
    if (pui32_comp_data < 8388608)
    {
        voltage = ((float)pui32_comp_data*slope) - 0.585;
        voltage_value = voltage;
    }
    DelayUS(800);
//    AD7793_SetRegisterValue(AD7793_REG_COMM, 0x58, 1);
//    AD7793_Reset();
//    AD7793_GetRegisterValue(AD7793_REG_DATA, 3);

}

int main(void)

{

    Epic_Init();        // Board initialization

    CS = 1;             //1 - TCD, 2 - RTD
    AD77_Init(CS);

    AD77_Reset();       // Reset ADC

    DelayUS(800);

    pui32DataTx = AD7793_COMM_READ | AD7793_COMM_ADDR(AD7793_REG_STAT);
    pui32DataTx &= 0x00FF;

    while (1)
    {
        DelayUS(800);

        ReadID(pui32DataTx);

        ReadSTATUS(pui32DataTx);

        WriteMODE(pui32DataTx, pui32DataTxMODE2, pui32DataTxMODE1);

        ReadMODE(pui32DataTx);

        WriteCONFIG(pui32DataTx, pui32DataTxCONF2, pui32DataTxCONF1);

        ReadCONFIG(pui32DataTx);

        ReadDATA(pui32DataTx);

    }
    return 0;
}
