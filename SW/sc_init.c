/////////////////////////////////////
//  Generated Initialization File  //
/////////////////////////////////////

#include "C8051F330.h"

// Peripheral specific initialization functions,
// Called from the Init_Device() function
void PCA_Init()
{
    PCA0CN    = 0x40;
    PCA0MD    &= ~0x40;
    PCA0MD    = 0x02;
    PCA0CPM0  = 0x01;
    PCA0CPM2  = 0x48;
}

void Timer_Init()
{
    TCON      = 0x45;
    TMOD      = 0x21;
    CKCON     = 0x11;
    TH1       = 0xC0;
    TMR2CN    = 0x04;
}

void SMBus_Init()
{
    SMB0CF    = 0xD5;
}

void DAC_Init()
{
    IDA0CN    = 0xF2;
}

void Voltage_Reference_Init()
{
    REF0CN    = 0x08;
}

void Port_IO_Init()
{
    // P0.0  -  Skipped,     Open-Drain, Digital
    // P0.1  -  Skipped,     Open-Drain, Digital
    // P0.2  -  Skipped,     Open-Drain, Digital
    // P0.3  -  Skipped,     Open-Drain, Digital
    // P0.4  -  Skipped,     Open-Drain, Digital
    // P0.5  -  SDA (SMBus), Open-Drain, Digital
    // P0.6  -  SCL (SMBus), Open-Drain, Digital
    // P0.7  -  Unassigned,  Open-Drain, Digital

    // P1.0  -  Unassigned,  Open-Drain, Digital
    // P1.1  -  Unassigned,  Open-Drain, Digital
    // P1.2  -  Unassigned,  Open-Drain, Digital
    // P1.3  -  Unassigned,  Open-Drain, Digital
    // P1.4  -  Unassigned,  Open-Drain, Digital
    // P1.5  -  Unassigned,  Open-Drain, Digital
    // P1.6  -  Unassigned,  Open-Drain, Digital
    // P1.7  -  Unassigned,  Open-Drain, Digital

    P0SKIP    = 0x1F;
    XBR0      = 0x04;
    XBR1      = 0x40;
}

void Oscillator_Init()
{
    OSCICN    = 0x83;
}

void Interrupts_Init()
{
    IE        = 0x25;
    EIE1      = 0x11;
    IT01CF    = 0x73;
}

// Initialization function for device,
// Call Init_Device() from your main program
void Init_Device(void)
{
    PCA_Init();
    Timer_Init();
    SMBus_Init();
    DAC_Init();
    Voltage_Reference_Init();
    Port_IO_Init();
    Oscillator_Init();
    Interrupts_Init();
}
