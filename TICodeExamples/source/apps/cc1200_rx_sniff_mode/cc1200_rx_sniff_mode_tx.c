//******************************************************************************
//! @file       cc1200_rx_sniff_mode_tx.c
//! @brief      This program demonstrates Rx Sniff Mode for the 38.4 kbps
//              setting available in SmartRf Studio.
//              The transmitter sends a packet every time a button is pushed and
//              the receiver implements RX Sniff Mode to reduce the current
//              consumption.
//              DN511 (http://www.ti.com/lit/swra428) explains how the register
//              settings are found.
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//      Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//      Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//      Neither the name of Texas Instruments Incorporated nor the names of
//      its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*****************************************************************************/


/******************************************************************************
* INCLUDES
*/
#include "msp430.h"
#include "lcd_dogm128_6.h"
#include "hal_spi_rf_trxeb.h"
#include "cc120x_spi.h"
#include "stdlib.h"
#include "bsp.h"
#include "bsp_key.h"
#include "io_pin_int.h"
#include "bsp_led.h"
#include "cc1200_rx_sniff_mode_reg_config.h"


/*******************************************************************************
* DEFINES
*/
#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0
#define PKTLEN                  30 // 1 < PKTLEN < 126

#define GPIO3                   0x04
#define GPIO2                   0x08
#define GPIO0                   0x80


/*******************************************************************************
* LOCAL VARIABLES
*/
static uint8  packetSemaphore;
static uint32 packetCounter = 0;


/*******************************************************************************
* STATIC FUNCTIONS
*/
static void initMCU(void);
static void registerConfig(void);
static void runTX(void);
static void createPacket(uint8 randBuffer[]);
static void radioTxISR(void);
static void updateLcd(void);



/*******************************************************************************
*   @fn         main
*
*   @brief      Runs the main routine.
*
*   @param      none
*
*   @return     none
*/
void main(void) {

    // Initialize MCU and peripherals
    initMCU();

    // Write radio registers
    registerConfig();

    // Enter runTX, never coming back
    runTX();
}


/*******************************************************************************
*   @fn         runTX
*
*   @brief      Transmits a packet every time a button is pushed. A packet
*               counter is incremented for each packet sent and the LCD is
*               updated
*
*   @param      none
*
*   @return     none
*/
static void runTX(void) {

    static uint8 marcState;

    // Initialize packet buffer of size PKTLEN + 1
    uint8 txBuffer[PKTLEN + 1];

    // Connect ISR function to GPIO0
    ioPinIntRegister(IO_PIN_PORT_1, GPIO0, &radioTxISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, GPIO0, IO_PIN_FALLING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO0);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, GPIO0);

    // Update LCD
    updateLcd();

    // Calibrate radio
    trxSpiCmdStrobe(CC120X_SCAL);

    // Wait for calibration to be done (radio back in IDLE state)
    do {
        cc120xSpiReadReg(CC120X_MARCSTATE, &marcState, 1);
    } while (marcState != 0x41);

    // Infinite loop
    while(TRUE) {

        // Wait for button push
        while(!bspKeyPushed(BSP_KEY_ALL));

        // Create a random packet with PKTLEN + 2 byte packet counter + n x random bytes
        createPacket(txBuffer);

        // Write packet to TX FIFO
        cc120xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));

        // Strobe TX to send packet
        trxSpiCmdStrobe(CC120X_STX);

        // Wait for packet to be sent
        while(packetSemaphore != ISR_ACTION_REQUIRED);

        // Clear semaphore flag
        packetSemaphore = ISR_IDLE;

        // Update LCD
        updateLcd();
    }
}


/*******************************************************************************
* @fn          radioTxISR
*
* @brief       ISR for packet handling in TX. Sets packet semaphore
*              and clears interrupt flag
*
* @param       none
*
* @return      none
*/
static void radioTxISR(void) {

    // Set packet semaphore
    packetSemaphore = ISR_ACTION_REQUIRED;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO0);
}


/*******************************************************************************
*   @fn         initMCU
*
*   @brief      Initialize MCU and board peripherals
*
*   @param      none
*
*   @return     none
*/
static void initMCU(void) {

    // Init clocks and I/O
    bspInit(BSP_SYS_CLK_8MHZ);

    // Init LEDs
    bspLedInit();

    // Init buttons
    bspKeyInit(BSP_KEY_MODE_POLL);

    // Initialize SPI interface to LCD (shared with SPI flash)
    bspIoSpiInit(BSP_FLASH_LCD_SPI, BSP_FLASH_LCD_SPI_SPD);

    // Init LCD
    lcdInit();

    // Instantiate transceiver RF SPI interface to SCLK ~ 4 MHz
    // Input parameter is clockDivider
    // SCLK frequency = SMCLK/clockDivider
    trxRfSpiInterfaceInit(2);

    // Enable global interrupt
    _BIS_SR(GIE);
}


/*******************************************************************************
*   @fn         registerConfig
*
*   @brief      Write register settings as given by SmartRF Studio found in
*               cc1200_rx_sniff_mode_reg_config.h
*
*   @param      none
*
*   @return     none
*/
static void registerConfig(void) {

    uint8 writeByte;

    // Reset radio
    trxSpiCmdStrobe(CC120X_SRES);

    // Write registers to radio
    for(uint16 i = 0;
        i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
        writeByte = preferredSettings[i].data;
        cc120xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}


/*******************************************************************************
*   @fn         createPacket
*
*   @brief      This function is called before a packet is transmitted. It fills
*               the txBuffer with a packet consisting of a length byte, two
*               bytes packet counter and n random bytes
*
*               The packet format is as follows:
*               |--------------------------------------------------------------|
*               |           |           |           |         |       |        |
*               | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
*               |           |           |           |         |       |        |
*               |--------------------------------------------------------------|
*                txBuffer[0] txBuffer[1] txBuffer[2]  ......... txBuffer[PKTLEN]
*
*   @param       Pointer to start of txBuffer
*
*   @return      none
*/
static void createPacket(uint8 txBuffer[]) {

    txBuffer[0] = PKTLEN;                         // Length byte
    txBuffer[1] = (uint8) (packetCounter >> 8);   // MSB of packetCounter
    txBuffer[2] = (uint8) packetCounter;          // LSB of packetCounter

    // Fill rest of buffer with random bytes
    for(uint8 i = 3; i < (PKTLEN + 1); i++) {
        txBuffer[i] = (uint8)rand();
    }
}


/*******************************************************************************
*   @fn         updateLcd
*
*   @brief      Updates LCD buffer and sends bufer to LCD module
*
*   @param      none
*
*   @return     none
*/
static void updateLcd(void) {

    // Update LDC buffer and send to screen
    lcdBufferClear(0);
    lcdBufferPrintString(0, "RX Sniff Mode", 0, eLcdPage0);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 7);
    lcdBufferPrintString(0, "Sent packets:", 0, eLcdPage3);
    lcdBufferPrintInt(0, packetCounter++, 80, eLcdPage3);
    lcdBufferPrintString(0, "TX" , 0, eLcdPage7);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 55);
    lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
    lcdSendBuffer(0);
}
