//******************************************************************************
//! @file       cc120x_smartpreamble_util.c
//! @brief      Utility helper functions for handling SmartPreamble.
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
#include "cc120x_smartpreamble_util.h"
#include "math.h"
#include "cc120x_smartpreamble_settings.h"


/*******************************************************************************
*   @fn         generateLookupEvent0Table
*
*   @brief      Generates a lookup table which maps timestamp numbers to EVENT0.
*               In the calculations below it is assumed that WOR_RES = 0
*
*               This lookup table is computed once at the start of the example
*               and is based on:
*                   * The number of timestamps
*                   * The reset value of the WOR timer
*                   * The time it takes to fire EVENT1
*                   * The time it takes to enter RX
*                   * An optional margin
*
*   @param      txBuffer
*                   The buffer where the smartPreamble should be saved
*
*   @return     none
*/
void generateLookupEvent0Table(struct Event0 lookupEvent0Table[]) {
    uint16 i;
    uint16 EVENT0;
    double tmp;

    for(i = 0; i < NUM_TIMESTAMP_PACKETS; i++) {

        // The time it takes to send the remaining timestamp packets
        tmp = i * TIMESTAMP_PACKET_LEN * 8.0 / BITRATE;

        // Subtract the time to go from IDLE state to RX
        tmp -= T_IDLE_TO_RX;

        // Subtract the margin
        tmp -= T_MARGIN;

        // Muliplying with 100/100.1 to account for RC oscillator
        // tolerance (0.1%)
        tmp = (100/100.1)*tmp;

        // Special case when tmp less than zero. It means that it takes more
        // time to go to sleep and wake up than there is time left
        if(tmp < 0) {
            tmp = 0;
        }

        // Multiply with the RC oscillator frequency to get floating
        // point EVENT0
        tmp *= RC_OSC_FREQ;

        // Round to get EVENT0 integer value
        EVENT0 = (uint16)floor(tmp);

        // Separate to MSB and LSB for easy EVENT0 register updates
        lookupEvent0Table[i].WOR_EVENT0_MSB = (EVENT0 >> 8);
        lookupEvent0Table[i].WOR_EVENT0_LSB = (EVENT0 & 0x00FF);
    }
}


/*******************************************************************************
*   @fn         generateSearchEvent0
*
*   @brief      Generates Search Mode EVENT0, used when searching for timestamps
*
*   @param      txBuffer
*                   The buffer where the SmartPreamble should be saved
*
*   @return     none
*/
void generateSearchEvent0(struct Event0* searchEvent0) {
    uint16 EVENT0;
    double tmp;

    // Start with the time it takes to send the SmartPreamble
    tmp = NUM_TIMESTAMP_PACKETS * TIMESTAMP_PACKET_LEN * 8.0 / BITRATE;

    // Muliplying with 100/100.1 to account for RC oscillator tolerance (0.1%)
    tmp = (100/100.1)*tmp;

    // Multiply with the RC oscillator frequency to get floating point EVENT0
    tmp *= RC_OSC_FREQ;

    // Round down to be sure to get the EVENT0 integer value
    EVENT0 = (uint16)floor(tmp);

    // Separate to MSB and LSB for easy EVENT0 register updates
    searchEvent0->WOR_EVENT0_MSB = (EVENT0 >> 8);
    searchEvent0->WOR_EVENT0_LSB = (EVENT0 & 0x00FF);
}


/*******************************************************************************
*   @fn         generateSmartPreamble
*
*   @brief      Generates the SmartPreamble and saves it in the TX buffer.
*
*               This function is called at the beginning of the TX program main
*               function and depends on the NUM_TIMESTAMP_PACKETS to determine
*               how many packets is shall generate.
*
*               Structure
*               =========
*               The structure of the total packet is as follows:
*               * X Timestamp Packets
*               * Payload
*
*               |Timestamp X|Timestamp X-1|...|Timestamp 0|Payload|
*               |-----------|-------------|---|-----------|-------|
*               |           |             |   |           |       |
*
*               Timestamp Packet
*               ----------------
*               A Timestamp Packet consists of a total of 7 bytes.
*                   * One preamble byte
*                   * Two sync bytes
*                   * One length byte
*                   * One timestamp byte
*                   * Two CRC bytes
*
*               The sync byte of the Timestamp Packet is different from that of
*               the Payload Packet. This is because it enables us to use the
*               DualSync feature of the radio to detect what kind of packet it
*               is and still discard any packet which does not have one of the
*               two configured sync fields.
*
*               Important Note
*               ==============
*               This function does not generate the payload and the input
*               buffer must be large enought to fit both the SmartPreamble and
*               the Payload. The size of the SmartPreamble is:
*
*               SmartPreamble_size = NUM_TIMESTAMP_PACKETS * 7 bytes
*
*   @param      txBuffer
*                   A pointer to the beginning of the buffer where the
*                   SmartPreamble shall be inserted. See note above.
*
*   @return     none
*/
void generateSmartPreamble(uint8* txBuffer) {
    uint16 i;
    uint16 checksum;

    // Generate the timestamps
    for(i = 0; i < NUM_TIMESTAMP_PACKETS; i++) {
        checksum = CRC_INIT;
        checksum = calcCRC(0x01, checksum);
        checksum = calcCRC((NUM_TIMESTAMP_PACKETS - 1) - i, checksum);

        txBuffer[(i * 7)]       = 0x55;
        txBuffer[(i * 7) + 1]   = 0x93;
        txBuffer[(i * 7) + 2]   = 0x0B;
        txBuffer[(i * 7) + 3]   = 0x01;
        txBuffer[(i * 7) + 4]   = (NUM_TIMESTAMP_PACKETS - 1) - i;
        txBuffer[(i * 7) + 5]   = (uint8)(checksum >> 8);
        txBuffer[(i * 7) + 6]   = (uint8)(checksum & 0x00FF);
    }
}


/*******************************************************************************
*   @fn         calcCRC
*
*   @brief      A CRC-16/CCITT implementation optimized for small code size.
*
*               The function should be called once for each byte in the data the
*               CRC is to be performed on. For the invocation on the first byte
*               the value CRC_INIT should be given for _crcReg_. The value
*               returned is the CRC-16 of the data supplied so far.
*
*   @param      crcData
*                   The data to perform the CRC-16 operation on
*
*               crcReg
*                   The current value of the CRC register
*
*   @return     The updated value of the CRC16 register
*/
uint16 calcCRC(uint8 crcData, uint16 crcReg) {
    uint8 i;
    for (i = 0; i < 8; i++) {
        if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
        crcReg = (crcReg << 1) ^ CRC16_POLY;
        else
            crcReg = (crcReg << 1);
        crcData <<= 1;
    }
    return crcReg;
}

