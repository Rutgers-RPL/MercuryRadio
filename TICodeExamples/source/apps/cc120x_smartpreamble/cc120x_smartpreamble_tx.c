//******************************************************************************
//! @file       cc120x_smartpreamble_tx.c
//! @brief      This program sets up a SmartPreamble transmitter with respect
//!             to the settings found in cc120x_smartpreamble_settings.h.
//!
//!             The program can take any recomended register settings exported
//!             from SmartRF Studio 7 without any modification with exeption
//!             from the assumtions decribed below.
//!
//!             Please see http://www.ti.com/lit/swra438
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
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


/*******************************************************************************
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

#include "cc120x_smartpreamble_reg_config.h"
#include "cc120x_smartpreamble_settings.h"
#include "cc120x_smartpreamble_util.h"


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct {
    uint16  addr;
    uint8   data;
    uint8   keepMask;
} customRegisterSetting_t;


/*******************************************************************************
* DEFINES
*/
#define TXFIFO_THR_PIN 0x80
#define PA_PD_PIN      0x08

#define INFINITE_PACKET_LENGTH_MODE 0x40
#define FIXED_PACKET_LENGTH_MODE    0x00

#define FIFO_SIZE                   128
#define AVAILABLE_BYTES_IN_TX_FIFO  122     // # of bytes one can write to the
                                            // TX_FIFO when a falling edge occur
                                            // on IOCFGx = 0x02 and
                                            // FIFO_THR = 120
#define BYTES_IN_TX_FIFO            (FIFO_SIZE - AVAILABLE_BYTES_IN_TX_FIFO)
#define MAX_VARIABLE_LENGTH         255

/*******************************************************************************
* LOCAL VARIABLES
*/
static uint8 packetSent = FALSE;            // Flag set when packet is sent
static uint16 packetCounter = 0;            // Counter keeping track of
                                            // packets sent
static uint32 bytesLeft;                    // Keeping track of bytes left to
                                            // write to the TX FIFO
static uint8 fixedPacketLength;
static uint8 *pBufferIndex;                 // Pointer to current position in
                                            // the txBuffer
static uint8 pktFormat = INFINITE_PACKET_LENGTH_MODE;

static uint8 txBuffer[SMARTPREAMBLE_LEN + PAYLOAD_OVERHEAD + PAYLOAD_LENGTH];


/*******************************************************************************
* STATIC FUNCTIONS
*/
static void initMCU(void);
static void registerConfig(void);
static void customRegisterConfig(customRegisterSetting_t* customRegisterSettings,
                                 uint8 length);
static void manualCalibration(void);
static void updateLcd(void);
static void packetSentISR(void);
static void txFifoBelowThresholdISR(void);
static void printWelcomeMessage(void);
static void createPacket(void);
static void waitUs(uint16 uSec);
static void waitMs(uint16 mSec);


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

    uint8 writeByte;

    // Initialize MCU and peripherals
    initMCU();

    // Generates the SmartPreamble once at start up
    generateSmartPreamble(txBuffer);

    // Write SmartRF Studio exported radio registers
    registerConfig();

    /***************************************************************************
        The following array contain all changes needed to the settings exported
        from SmartRF Studio to run the SmartPreamble transmitter

        The changes are the following:
            - TX specific registers
            - FIFO_THR = 120
            - GPIO0 = TXFIFO_THR
            - GPIO2 = PA_PD (Inv)
            - No CRC
            - No preamble
            - No sync

        Note on Keep Bits:
            If the Keep Bits are set to all zero then the Data will be written
            to the specific Register, overwriting any existing register value.
            If the Keep Bits are non-zero then that Register will be first be
            read and a logical AND operation will be performed with the Keep
            Bits. The result of this operation will then be logically OR:ed with
            the Data. This means that all bits in the Keep Bits field will be
            preserved, making it possible to keep specific SmartRF exported
            Register fields and flags intact.

    =======================================================================
    |   Register               | Data                          | Keep Bits |
    =======================================================================*/
    customRegisterSetting_t customRegisterSettings[] = {
        {CC120X_SYNC_CFG0       ,0x03                           ,0x00},
        {CC120X_PREAMBLE_CFG1   ,0x00                           ,0x00},
        {CC120X_IOCFG0          ,0x02                           ,0x00},
        {CC120X_IOCFG2          ,0x59                           ,0x00},
        {CC120X_FIFO_CFG        ,0x78                           ,0x00},
        {CC120X_PKT_CFG1        ,0x05                           ,0x00},
        {CC120X_PKT_CFG0        ,INFINITE_PACKET_LENGTH_MODE    ,0x00},
    };
    /*======================================================================*/

    // Write custom register settings to radio
    customRegisterConfig(customRegisterSettings,
                        (sizeof customRegisterSettings /
                         sizeof(customRegisterSetting_t)));

    // Connect TXFIFO_THR ISR function to GPIO0
    ioPinIntRegister(IO_PIN_PORT_1, TXFIFO_THR_PIN, &txFifoBelowThresholdISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, TXFIFO_THR_PIN, IO_PIN_FALLING_EDGE);

    // Disable interrupt
    ioPinIntDisable(IO_PIN_PORT_1, TXFIFO_THR_PIN);

    // Set up and enable interrupt on GPIO2 (PA_PD (Inv))
    // The PA_PD will go low when the radio goes to sleep
    ioPinIntRegister(IO_PIN_PORT_1, PA_PD_PIN, &packetSentISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, PA_PD_PIN, IO_PIN_FALLING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, PA_PD_PIN);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, PA_PD_PIN);

    // Create the payload packet to send
    createPacket();

    printWelcomeMessage();

    // Infinite loop
    while(TRUE) {

        // Wait for button push
        while(!bspKeyPushed(BSP_KEY_ALL));

        // Send 1000 packets
        for (uint16 i = 0; i < 1000; i ++) {

            // Reset control variables
            bytesLeft = SMARTPREAMBLE_LEN + PAYLOAD_LENGTH + PAYLOAD_OVERHEAD;
            fixedPacketLength = bytesLeft % (MAX_VARIABLE_LENGTH + 1);

            // Configure the PKT_LEN register, used in fixed length mode to
            // trigger the packetSentISR the last time the FIFO has been filled
            // with data.
            writeByte = fixedPacketLength;
            cc120xSpiWriteReg(CC120X_PKT_LEN, &writeByte, 1);

            // Set initial packet length mode based on bytesLeft
            if (bytesLeft <= (MAX_VARIABLE_LENGTH + 1)) {

                // The amount of data to be sent is so small that it will fit
                // in the TX FIFO at once
                writeByte = FIXED_PACKET_LENGTH_MODE;
                pktFormat = FIXED_PACKET_LENGTH_MODE;
            } else {

                // The amount of data to be sent is larger than the TX FIFO
                writeByte = INFINITE_PACKET_LENGTH_MODE;
                pktFormat = INFINITE_PACKET_LENGTH_MODE;
            }
            cc120xSpiWriteReg(CC120X_PKT_CFG0, &writeByte, 1);

            // Fill up the TX FIFO to initialize the transfer
            if( bytesLeft <= FIFO_SIZE ) {

                // Fill up the TX FIFO with all data
                cc120xSpiWriteTxFifo(txBuffer, bytesLeft);
            } else {

                // Fill up the TX FIFO with as much data as possible
                cc120xSpiWriteTxFifo(txBuffer, FIFO_SIZE);

                // Update txBuffer pointer and bytesLeft
                pBufferIndex = txBuffer + FIFO_SIZE;
                bytesLeft -= FIFO_SIZE;

                // Clear ISR flag and enable interrupt on FIFO below threshold
                ioPinIntClear(IO_PIN_PORT_1, TXFIFO_THR_PIN);
                ioPinIntEnable(IO_PIN_PORT_1, TXFIFO_THR_PIN);
            }

            // Enter TX mode.
            trxSpiCmdStrobe(CC120X_STX);

            // All the work of re-filling the TX FIFO, to be able to handle
            // packets longer than 128 byte, is done in the
            // txFifoBelowThresholdISR

            // Wait until the entire packet is sent
            while (!packetSent);
            packetSent = FALSE;

            // Increment the packet counter on the LCD.
            updateLcd();

            // Wait for a random time to force the receiver to wake up in many
            // different timestamps.
            waitMs(300 + (rand()%400));
        }
    }
}


/*******************************************************************************
*   @fn         createPacket
*
*   @brief      Adds the data portion of the packet. Adds small preamble, the
*               data specific sync word, length, random data and CRC
*
*   @param      none
*
*   @return     none
*/
static void createPacket(void) {
    uint16 checksum;
    uint16 i;

    txBuffer[SMARTPREAMBLE_LEN]     = 0x55;            // Preamble
    txBuffer[SMARTPREAMBLE_LEN + 1] = 0x55;            // Preamble
    txBuffer[SMARTPREAMBLE_LEN + 2] = 0x55;            // Preamble
    txBuffer[SMARTPREAMBLE_LEN + 3] = 0x55;            // Preamble
    txBuffer[SMARTPREAMBLE_LEN + 4] = 0xD0;            // Payload SYNC1
    txBuffer[SMARTPREAMBLE_LEN + 5] = 0xC9;            // Payload SYNC0
    txBuffer[SMARTPREAMBLE_LEN + 6] = PAYLOAD_LENGTH;  // Length byte

    // Payload with random data
    for(uint16 i = (SMARTPREAMBLE_LEN + 7);
        i < (SMARTPREAMBLE_LEN + 7 + PAYLOAD_LENGTH); i++) {
        txBuffer[i] = rand();
    }

    // Generate CRC for the data packet
    checksum = CRC_INIT;
    for(i = (SMARTPREAMBLE_LEN + 6);
        i <= (SMARTPREAMBLE_LEN + 6 + PAYLOAD_LENGTH); i++) {
        checksum = calcCRC(txBuffer[i], checksum);
    }

    txBuffer[SMARTPREAMBLE_LEN + PAYLOAD_LENGTH + PAYLOAD_OVERHEAD - 2] =
    (uint8)(checksum >> 8);
    txBuffer[SMARTPREAMBLE_LEN + PAYLOAD_LENGTH + PAYLOAD_OVERHEAD - 1] =
    (uint8)(checksum & 0x00FF);
}


/*******************************************************************************
*   @fn         printWelcomeMessage
*
*   @brief      Print welcome message at start-up
*
*   @param      none
*
*   @return     none
*/
static void printWelcomeMessage(void) {
    lcdBufferClear(0);
    lcdBufferPrintString(0, "SmartPreamble", 0, eLcdPage0);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, eLcdPage7);
    lcdBufferPrintString(0, "Press a button", 0, eLcdPage4);
    lcdBufferPrintString(0, "to transmit", 0, eLcdPage5);
    lcdBufferPrintString(0, "Packet TX", 0, eLcdPage7);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 55);
    lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
    lcdSendBuffer(0);
}


/*******************************************************************************
*   @fn         packetSentISR
*
*   @brief      Function running every time a packet has been sent
*
*   @param      none
*
*   @return     none
*/
static void packetSentISR(void) {
    packetSent = TRUE;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, PA_PD_PIN);
}


/*******************************************************************************
*   @fn         txFifoBelowThresholdISR
*
*   @brief      Function running every time the TX FIFO is drained below
*               127 - FIFO_THR = 127 - 120 = 7
*
*   @param      none
*
*   @return     none
*/
static void txFifoBelowThresholdISR(void) {

    uint8 writeByte;

    // If less than 122 bytes to write to the TX FIFO, write remaining bytes and
    // then we are done sending this packet
    if (bytesLeft < AVAILABLE_BYTES_IN_TX_FIFO) {
        cc120xSpiWriteTxFifo(pBufferIndex, bytesLeft);

        // Disable this interrupt since we will not be filling the TX FIFO
        // any more with this packet
        ioPinIntDisable(IO_PIN_PORT_1, TXFIFO_THR_PIN);
    } else {

        // Else fill up the TX FIFO
        cc120xSpiWriteTxFifo(pBufferIndex, AVAILABLE_BYTES_IN_TX_FIFO);

        // Change to fixed packet length mode when there are less than 250
        // bytes, the max fixed packet length subtracted by the bytes in the
        // TX FIFO, left to transmit
        if ((bytesLeft < (MAX_VARIABLE_LENGTH + 1 - BYTES_IN_TX_FIFO)) &&
            (pktFormat == INFINITE_PACKET_LENGTH_MODE)) {

            pktFormat = FIXED_PACKET_LENGTH_MODE;
            writeByte = FIXED_PACKET_LENGTH_MODE;
            cc120xSpiWriteReg(CC120X_PKT_CFG0, &writeByte, 1);
        }

        // Update the variables keeping track of how many more bytes should be
        // written to the TX FIFO and where in txBuffer data should be
        // taken from
        pBufferIndex += AVAILABLE_BYTES_IN_TX_FIFO;
        bytesLeft -= AVAILABLE_BYTES_IN_TX_FIFO;
    }
    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, TXFIFO_THR_PIN);
}


/*******************************************************************************
*   @fn         updateLcd
*
*   @brief      Updates the LCD with # of packets sent
*
*   @param      none
*
*   @return     none
*/
static void updateLcd(void) {
    lcdBufferClear(0);
    lcdBufferPrintString(0, "SmartPreamble", 0, eLcdPage0);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, eLcdPage7);
    lcdBufferPrintString(0, "Sent packets:", 0, eLcdPage2);
    lcdBufferPrintInt(0, (int32)(++packetCounter), 80, eLcdPage2);
    lcdBufferPrintString(0, "Press a button", 0, eLcdPage4);
    lcdBufferPrintString(0, "to transmit", 0, eLcdPage5);
    lcdBufferPrintString(0, "Packet TX", 0, eLcdPage7);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 55);
    lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
    lcdSendBuffer(0);
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
*               cc120x_smartpreamble_reg_config.h
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
*   @fn         customRegisterConfig
*
*   @brief      Write register settings to run the SmartPreamble transmitter
*
*   @param      none
*
*   @return     none
*/
static void customRegisterConfig(customRegisterSetting_t* settings,
                                 uint8 length) {
    uint8 readByte;
    uint8 writeByte;
    uint16 addr;

    for(uint16 i = 0; i < length; i++) {

        // Respect the Keep Mask
        if( settings[i].keepMask != 0 ) {
            cc120xSpiReadReg(settings[i].addr, &readByte, 1);
            writeByte = (readByte & settings[i].keepMask) | settings[i].data;
        } else {
            writeByte = settings[i].data;
        }
        cc120xSpiWriteReg(settings[i].addr, &writeByte, 1);
    }
}


/*******************************************************************************
*   fn          waitUs
*
*   @brief      Busy wait function. Waits the specified number of microseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions. The duration of one cycle depends on MCLK.
*               In this HAL it is set to 8 MHz, thus 8 cycles per us.
*
*               NB! This function is highly dependent on architecture
*               and compiler!
*
*   @param      uint16 uSec - number of microseconds delay
*
*   @return     none
*/
#pragma optimize=none
static void waitUs(uint16 uSec) { // 5 cycles for calling

    // The least we can wait is 3 usec:
    // ~1 usec for call, 1 for first compare and 1 for return
    while(uSec > 3) {  // 2 cycles for compare
                       // 2 cycles for jump
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        uSec -= 2;     // 1 cycle for optimized decrement
    }
}                      // 4 cycles for returning


/*******************************************************************************
*   @fn         waitMs
*
*   @brief      Busy wait function. Waits the specified number of milliseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions.
*
*               NB! This function is highly dependent on architecture and
*               compiler!
*
*   @param      uint16 mSec - number of milliseconds delay
*
*   @return     none
*/
#pragma optimize=none
static void waitMs(uint16 mSec) {
    while(mSec-- > 0) {
        waitUs(1000);
    }
}