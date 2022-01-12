//###########################################################################
//
// FILE:    sci_ex1_loopback.c
//
// TITLE:   SCI FIFO Digital Loop Back.
//
//! \addtogroup driver_example_list
//! <h1>SCI FIFO Digital Loop Back</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral.
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required.
//!
//!  This test uses the loopback test mode of the SCI module to send
//!  characters starting with 0x00 through 0xFF.  The test will send
//!  a character and then check the receive buffer for a correct match.
//!
//!  \b Watch \b Variables \n
//!  - \b loopCount - Number of characters sent
//!  - \b errorCount - Number of errors detected
//!  - \b sendChar - Character sent
//!  - \b receivedChar - Character received
//!
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.07.00.00 $
// $Release Date: Sun Sep 29 07:34:54 CDT 2019 $
// $Copyright:
// Copyright (C) 2013-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Globals
//
uint16_t loopCount;
uint16_t errorCount;

//
// Function Prototypes
//
void initSCICLoopback(void);
void initSCICFIFO(void);
void xmitSCIC(uint16_t a);
void error();

//
// Main
//
void main(void)
{
    uint16_t sendChar;
    uint16_t receivedChar;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Setup GPIO by disabling pin locks and enabling pullups
    //
    Device_initGPIO();

    // GPIO62 is the SCI Rx-485 pin that goes to the rs-485 chip.
    //
    GPIO_setMasterCore(62, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_62_SCIRXDC);
    GPIO_setDirectionMode(62, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(62, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(62, GPIO_QUAL_ASYNC);

    //
    // GPIO63 is the SCI Tx-485 pin that goes to the rs-485 chip.
    //
    GPIO_setMasterCore(63, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_63_SCITXDC);
    GPIO_setDirectionMode(63, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(63, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(63, GPIO_QUAL_ASYNC);

    //
    //



    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enables CPU interrupts
    //
    Interrupt_enableMaster();

    //
    // Initialize counts
    //
    loopCount = 0;
    errorCount = 0;

    //
    // Initialize SCIC
    //
    initSCICFIFO();
    initSCICLoopback();

    //
    // Note: Autobaud lock is not required for this example
    //

    //
    // Send a character starting with 0
    //
    sendChar = 0;

    //
    // Send Characters forever starting with 0x00 and going through 0xFF.
    // After sending each, check the receive buffer for the correct value.
    //
    for(;;)
    {
        xmitSCIC(sendChar);

        //
        // Wait for RRDY/RXFFST = 1 for 1 data available in FIFO
        //
        while(SCI_getRxFIFOStatus(SCIC_BASE) == SCI_FIFO_RX0)
        {
            ;
        }

        //
        // Check received character
        //
        receivedChar = SCI_readCharBlockingFIFO(SCIC_BASE);

        //
        // Received character not correct
        //
        if(receivedChar != sendChar)
        {
            error();
        }

        //
        // Move to the next character and repeat the test
        //
        sendChar++;

        //
        // Limit the character to 8-bits
        //
        sendChar &= 0x00FF;
        loopCount++;

        Example_PassCount++;
    }
}

//
// error - Function to count errors
//
void error()
{
    errorCount++;
    Example_Fail = 1;
    //asm("     ESTOP0");  // Uncomment to stop the test here
    for (;;);
}

//
// initSCICLoopback - Configure SCIC settings
//
void initSCICLoopback()
{
    //
    // Note: Clocks were turned on to the SCIC peripheral
    // in the Device_init() function
    //

    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIC_BASE, DEVICE_LSPCLK_FREQ, 9600, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(SCIC_BASE);

    SCI_enableFIFO(SCIC_BASE);
    SCI_performSoftwareReset(SCIC_BASE);
    SCI_disableInterrupt(SCIC_BASE, SCI_INT_RXERR);

    SCI_enableInterrupt(SCIC_BASE, SCI_INT_TXRDY);
    SCI_enableInterrupt(SCIC_BASE, SCI_INT_RXRDY_BRKDT);

    SCI_enableLoopback(SCIC_BASE);

    //
    // Relinquish SCI from Reset
    //
    SCI_enableModule(SCIC_BASE);
}

//
// xmitSCIC - Transmit a character from the SCI
//
void xmitSCIC(uint16_t a)
{
    SCI_writeCharNonBlocking(SCIC_BASE, a);
}

//
// initSCICFIFO - Initialize the SCI FIFO
//
void initSCICFIFO()
{
    SCI_clearInterruptStatus(SCIC_BASE, SCI_INT_TXFF);
    SCI_resetTxFIFO(SCIC_BASE);
    SCI_enableFIFO(SCIC_BASE);

    SCI_setFIFOInterruptLevel(SCIC_BASE, SCI_FIFO_TX0, SCI_FIFO_RX4);
    SCI_resetChannels(SCIC_BASE);

    SCI_clearInterruptStatus(SCIC_BASE, SCI_INT_RXFF);
}

//
// End of file
//
