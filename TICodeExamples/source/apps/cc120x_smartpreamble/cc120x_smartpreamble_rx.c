//******************************************************************************
//! @file       cc120x_smartpreamble_rx.c
//! @brief      This program sets up a SmartPreamble receiver with respect
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

#include "math.h"

#include "cc120x_smartpreamble_reg_config.h"
#include "cc120x_smartpreamble_settings.h"
#include "cc120x_smartpreamble_util.h"


/*******************************************************************************
* DEFINES
*/
#define SYNC_TYPE_PIN           0x80
#define CRC_OK_PIN              0x08
#define WOR_EVENT1_PIN          0x04

#define FIFO_SIZE               128

#define SYNC_MODE_BM            0xE0
#define DUAL_SYNC               0xE0
#define _16_BIT                 0x40


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct {
    uint16  addr;
    uint8   data;
    uint8   keepMask;
} customRegisterSetting_t;


/*******************************************************************************
* LOCAL VARIABLES
*/
static uint8 packetReceived = FALSE;    // Flag set when a packet is received
static uint8 event1Occured = FALSE;     // Flag set when the radio enters RX
                                        // to search for the payload packet
static uint16 packetCounter = 0;        // Counter keeping track of
                                        // packets received (with CRC OK)
static uint8 rxBuffer[FIFO_SIZE];       // Used to read out the RX FIFO from
                                        // the radio

// Lookup table for EVENT0 generated at start up. This lookup tabe is used to
// determine how long the radio should sleep from the reception of a timestamp
// packet to the reception of the real packet
static struct Event0 lookupEvent0Table[NUM_TIMESTAMP_PACKETS];

// The default tEVENT0, used when in Search Mode. After a data packet has been
// received using the timestamped preamble method EVENT0 is reset to this
// value again
static struct Event0 searchEvent0;


/*******************************************************************************
* LOCAL FUNCTIONS
*/
static void initMCU(void);
static void registerConfig(void);
static void customRegisterConfig(customRegisterSetting_t* customRegisterSettings,
                                 uint8 length);
static void updateLcd(void);
static void packetReceivedISR(void);
static void event1ISR(void);
static void printWelcomeMessage(void);
static void calibrateRCOsc(void);
static void waitUs(uint16 uSec);
static void waitMs(uint16 mSec);


/*******************************************************************************
*   @fn         main
*
*   @brief      Runs the main routine
*
*   @param      none
*
*   @return     none
*/
void main(void) {

    uint8 timeStamp;
    uint32 timeout;
    uint8 writeByte;
    uint8 readByte;
    uint8 rxBytes;
    uint32 sleep;

    // Initialize MCU and peripherals
    initMCU();

    // Generate timeout lookup table once
    generateLookupEvent0Table(lookupEvent0Table);

    // Generate the EVENT0 used in Search Mode once
    generateSearchEvent0(&searchEvent0);

    // Write SmartRF Studio exported radio registers
    registerConfig();

    /***************************************************************************
        The following array contain all changes needed to the settings exported
        from SmartRF Studio to run the Smart RX Sniff Mode receiver.

        A summary of the changes to SmartRF Studio:
            - Dual Sync
                Sync of timestamp packet: 0x93, 0x0B
                Sync of data packet: 0xD0, 0xC9 (SYNC1:SYNC0)
            - Soft decision sync word threshold 00111b
            - GPIO0 = SYNC_LOW0_HIGH1
                Used only as input
                    0: Data packet
                    1: Timestamp packet
            - GPIO2 = CRC_OK
                Used to trigger packet received interrupt.
            - GPIO3 = WOR_EVENT1
                Used to trigger payload packet received interrupt.
            - AGC_WIN_SIZE, RSSI_VALID_COUNT and AGC_SETTLE_WAIT to 0
                Reduces the CS response time.
            - Terminate on bad packet
                Forces the radio back to sleep instead of idle on bad packet.
            - Use maximum length filtering (Max length byte = 125)
                Prevents RXFIFO_OVERFLOW so that the radio can always go back
                to sleep
            - Autoflush RX FIFO on CRC fail
                This, together with terminate on bad packet, makes it possible
                for the radio to go back to sleep after a bad CRC
            - CS threshold is -100 dBm
            - Auto calibration, FS_AUTOCAL, is set to never

        Note on Keep Bits:
            If the Keep Bits are set to all zero then the Data will be written
            to the specific Register, overwriting any existing register value.
            If the Keep Bits are non-zero then that Register will be first be
            read and a logical AND operation will be performed with the Keep
            Bits. The result of this operation will then be logically OR:ed with
            the Data. This means that all bits in the Keep Bits field will be
            preserved, making it possible to keep specific SmartRF exported
            Register fields and flags intact.

    ===========================================================================
    |   Register                  | Data                          | Keep Bits |
    ==========================================================================*/
    customRegisterSetting_t customRegisterSettings[] = {
        {CC120X_SYNC_CFG1           ,0xE7                           ,0x00},
        {CC120X_SYNC1               ,0xD0                           ,0x00},
        {CC120X_SYNC0               ,0xC9                           ,0x00},
        {CC120X_IOCFG0              ,0x15                           ,0x00},
        {CC120X_IOCFG2              ,0x07                           ,0x00},
        {CC120X_IOCFG3              ,0x38                           ,0x00},
        {CC120X_AGC_CFG1            ,0x00                           ,0xC0},
        {CC120X_AGC_CS_THR          ,0x02                           ,0x00},
        {CC120X_PKT_LEN             ,125                            ,0x00},
        {CC120X_FIFO_CFG            ,0x80                           ,0x00},
        {CC120X_AGC_CFG0            ,0x00                           ,0xF3},
        {CC120X_WOR_EVENT0_MSB      ,searchEvent0.WOR_EVENT0_MSB    ,0x00},
        {CC120X_WOR_EVENT0_LSB      ,searchEvent0.WOR_EVENT0_LSB    ,0x00},
        {CC120X_WOR_CFG0            ,0x20                           ,0x00},
        {CC120X_RFEND_CFG1          ,0x0E                           ,0x00},
        {CC120X_RFEND_CFG0          ,0x09                           ,0x00},
        {CC120X_WOR_CFG1            ,0x0D                           ,0x00},
        {CC120X_SETTLING_CFG        ,0x00                           ,0xE7},
    };
    /*========================================================================*/

    // Write custom register settings to radio
    customRegisterConfig(customRegisterSettings,
                        (sizeof(customRegisterSettings) /
                         sizeof(customRegisterSetting_t)));

    // Calibrates the radio frequency synthesizer
    trxSpiCmdStrobe(CC120X_SCAL);

    // Wait until calibration of radio is finished (radio returned to IDLE)
    do {
        cc120xSpiReadReg(CC120X_MARCSTATE, &readByte, 1);
    } while (readByte != 0x41);

    // Calibrate the RC Oscillator used for the cc120x wake on radio
    calibrateRCOsc();

    // Connect ISR function to GPIO2
    ioPinIntRegister(IO_PIN_PORT_1, CRC_OK_PIN, &packetReceivedISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, CRC_OK_PIN, IO_PIN_RISING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, CRC_OK_PIN);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, CRC_OK_PIN);

    // Connect ISR function to GPIO3
    ioPinIntRegister(IO_PIN_PORT_1, WOR_EVENT1_PIN, &event1ISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, WOR_EVENT1_PIN, IO_PIN_RISING_EDGE);

    // Disable the WOR_EVENT1 interrupt. This interrupt is only enabled on the
    // MCU when we are waiting for the payload packet
    ioPinIntDisable(IO_PIN_PORT_1, WOR_EVENT1_PIN);

    printWelcomeMessage();


    // Infinite loop
    while(TRUE) {

        // Reset packet received flag
        packetReceived = FALSE;

        // Enter eWOR Mode
        // (Search Mode; RX Sniff Mode is used to look for timestamp packets)
        trxSpiCmdStrobe(CC120X_SWOR);

        // Update LCD information
        updateLcd();

        // Here the MCU should enter PM and wait for packetReceivedISR

        // Wait for the reception of any packet. This will set the
        // packetReceived flag from the packetReceivedISR
        while (!packetReceived);
        packetReceived = FALSE;

        // Check status of SYNC_LOW0_HIGH1 to see if the packet is a timestamp
        // packet or a payload packet
        if (P1IN & SYNC_TYPE_PIN) {

            // This is a timestamp packet

            //##### TIME CRITICAL SECTION START ################################
            // The timing of this section is critical for two reasons and both
            // are related to the timing of the MCU response to the radio
            //
            // Not last timestamp
            // ==================
            // The time it takes for the eWOR timer to reset needs to be
            // deterministic and hard-coded as a part of t_Margin. This is due
            // to the fact that all timestamp packets are sent back-to-back and
            // the next packet is already in the air when we are reading out the
            // timestamp number. This time will be substracted from the wake
            // up time
            //
            // Last timestamp
            // ==============
            // The MCU should tell the radio to enter RX as fast as possible to
            // catch the payload sync word. Note that the radio MUST enter RX in
            // a shorter time then it takes to send the payload preamble bytes.
            // If this is not respected then the radio will not be able to enter
            // RX in time to catch the payload. Increasing the preamble of the
            // payload will increase this margin

            // Read the entire timestamp packet from the RX FIFO
            cc120xSpiReadReg(CC120X_NUM_RXBYTES, &rxBytes, 1);
            cc120xSpiReadRxFifo(rxBuffer, rxBytes);

            // Read out the timestamp number
            timeStamp = rxBuffer[1];

            // Check which timestamp packet this is
            if(timeStamp == 0) {

                // This is the last timestamp, go directly into RX and wait
                // for the payload packet since it is the next packet on the air
                trxSpiCmdStrobe(CC120X_SRX);
            } else {

                // This is not the last timestamp packet. Reschedule EVENT0,
                // enable the EVENT1 interrupt and wait for the EVENT1

                // Reschedule tEVENT0 based on the timestamp just received
                writeByte = lookupEvent0Table[timeStamp].WOR_EVENT0_MSB;
                cc120xSpiWriteReg(CC120X_WOR_EVENT0_MSB,  &writeByte, 1);
                writeByte = lookupEvent0Table[timeStamp].WOR_EVENT0_LSB;
                cc120xSpiWriteReg(CC120X_WOR_EVENT0_LSB,  &writeByte, 1);

                // Disable DualSync and look for a payload sync only
                cc120xSpiReadReg(CC120X_SYNC_CFG1, &readByte, 1);
                writeByte = (readByte & ~SYNC_MODE_BM) | _16_BIT;
                cc120xSpiWriteReg(CC120X_SYNC_CFG1, &writeByte, 1);

                // Clear ISR flag and enable interrupt on EVENT1. Next time this
                // interrupt is triggered the radio will search for the
                // payload packet
                ioPinIntClear(IO_PIN_PORT_1, WOR_EVENT1_PIN);
                ioPinIntEnable(IO_PIN_PORT_1, WOR_EVENT1_PIN);

                // Reset the WOR timer so that the next EVENT0 will be relative
                // to the current time
                trxSpiCmdStrobe(CC120X_SWORRST);
                waitUs(50);	// Wait 2 clock cycles to make sure the eWOR
                                // timer has been reset

                trxSpiCmdStrobe(CC120X_SWOR);
            }
            //##### TIME CRITICAL SECTION END ##################################

            if( timeStamp != 0 ) {

                // Here the MCU should enter PM and wait for interrupt on EVENT1
                while(!event1Occured);
                event1Occured = FALSE;
            }

            // Wait for data packet or timeout
            // Should be replaced with a configurabe timeout (using a timer)
            // How long the timout should be depends on max packet lenght of the
            // data packet. It must also be taken into account that the RCOSC
            // might wake up 0.1 % too early
            timeout = 14000;
            while((!packetReceived) && ((--timeout) > 0));

            if (packetReceived) {

                // Read packet from FIFO
                cc120xSpiReadReg(CC120X_NUM_RXBYTES, &rxBytes, 1);
                cc120xSpiReadRxFifo(rxBuffer, rxBytes);

                // Update packet counter
                packetCounter++;
            }

            // The radio is now either in
            //    - IDLE (due to packet received),
            //    - SLEEP (if RX has terminated due to a bad packet)
            //    - RX (still a carrier on the air)

            // Strobe IDLE to be sure the radio is in IDLE
            trxSpiCmdStrobe(CC120X_SIDLE);

            // Flush the RX FIFO in case it was in the middle of receiving
            // something when RX was terminated
            trxSpiCmdStrobe(CC120X_SFRX);

            // Set configuration back to look for timestamp packets
            writeByte = searchEvent0.WOR_EVENT0_MSB;
            cc120xSpiWriteReg(CC120X_WOR_EVENT0_MSB, &writeByte, 1);
            writeByte = searchEvent0.WOR_EVENT0_LSB;
            cc120xSpiWriteReg(CC120X_WOR_EVENT0_LSB, &writeByte, 1);

            // Set sync mode back to DualSync
            cc120xSpiReadReg(CC120X_SYNC_CFG1, &readByte, 1);
            writeByte = (readByte & ~SYNC_MODE_BM) | DUAL_SYNC;
            cc120xSpiWriteReg(CC120X_SYNC_CFG1, &writeByte, 1);

            // Done resetting radio, return and search for next packet.

        } else {

            // Woke right onto the data packet

            // Read packet from FIFO
            cc120xSpiReadReg(CC120X_NUM_RXBYTES, &rxBytes, 1);
            cc120xSpiReadRxFifo(rxBuffer, rxBytes);

            // Update packet counter
            packetCounter++;
        }
    }
}


/*******************************************************************************
*   @fn         calibrateRcOsc
*
*   @brief      Calibrates the RC Oclillator used for the cc120x wake on radio
*               functionality
*
*   @param      none
*
*   @return     none
*/
static void calibrateRCOsc(void) {
    uint8 writeByte;

    // Read current register value
    cc120xSpiReadReg(CC120X_WOR_CFG0,&writeByte,1);

    // Mask register bitfields and write new values
    writeByte = (writeByte & 0xF9) | (0x02 << 1);

    // Write new register value
    cc120xSpiWriteReg(CC120X_WOR_CFG0,&writeByte,1);

    // Strobe IDLE to calibrate RC osc
    trxSpiCmdStrobe(CC120X_SIDLE);

    // Disable RC calibration
    writeByte = (writeByte & 0xF9) | (0x00 << 1);
    cc120xSpiWriteReg(CC120X_WOR_CFG0, & writeByte, 1);
}


/*******************************************************************************
*   @fn         printWelcomeMessage
*
*   @brief      Prints a welcome message at start-up
*
*   @param        none
*
*   @return       none
*/
static void printWelcomeMessage(void) {
    lcdBufferClear(0);
    lcdBufferPrintString(0, "SmartPreamble", 0, eLcdPage0);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, eLcdPage7);
    lcdBufferPrintString(0, "Waiting for packets", 0, eLcdPage4);
    lcdBufferPrintString(0, "Packet RX", 0, eLcdPage7);
    lcdBufferSetHLine(0, 0, LCD_COLS - 1, 55);
    lcdBufferInvertPage(0, 0, LCD_COLS, eLcdPage7);
    lcdSendBuffer(0);
}


/*******************************************************************************
*   @fn         event1ISR
*
*   @brief      Function running every time the radio wakes up to search for a
*               payload packet
*
*   @param      none
*
*   @return     none
*/
static void event1ISR(void) {
    uint8 writeByte;

    event1Occured = TRUE;
    ioPinIntDisable(IO_PIN_PORT_1, WOR_EVENT1_PIN);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, WOR_EVENT1_PIN);
}


/*******************************************************************************
*   @fn         packetReceivedISR
*
*   @brief      Function running every time a packet has been received
*
*   @param      none
*
*   @return     none
*/
static void packetReceivedISR(void) {

    packetReceived = TRUE;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, CRC_OK_PIN);
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
    lcdBufferPrintString(0, "Packets received:", 0, eLcdPage2);
    lcdBufferPrintInt(0, (int32)(packetCounter), 102, eLcdPage2);
    lcdBufferPrintString(0, "Packet RX", 0, eLcdPage7);
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
* @fn           customRegisterConfig
*
* @brief        Write register settings to run the SmartPreamble transmitter
*
* @param        none
*
* @return       none
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
*   @fn         waitUs
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
