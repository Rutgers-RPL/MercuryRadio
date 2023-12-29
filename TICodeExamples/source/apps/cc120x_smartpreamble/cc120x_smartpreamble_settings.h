//******************************************************************************
//! @file       cc120x_smartpreamble_settings.h
//! @brief      Utility helper functions for handling timestamped preambles.
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


#ifndef CC120X_SMART_SNIFF_MODE_SETTINGS_H
#define CC120X_SMART_SNIFF_MODE_SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* DEFINES
*/
// The bitrate of the transfer. N.B. This should always match the settings
// exported from SmartRF Studio!
#define BITRATE                 38400

// The number of timestamp packets to send before the payload packet.
#define NUM_TIMESTAMP_PACKETS   256

// Length byte of the payload. Should be a number between 1 - 125
#define PAYLOAD_LENGTH          20

// An additional time margin. This has to include the time it takes for the MCU
// to receive a timestamp, set the new EVENT0 and reset the eWOR timer.

#define T_MARGIN                273e-6


// The SmartPreamble length
#define SMARTPREAMBLE_LEN      (NUM_TIMESTAMP_PACKETS * TIMESTAMP_PACKET_LEN)


/*******************************************************************************
 * CONSTANTS - Based on current SmartPreamble layout and using the CC120x radio
 */
#define RC_OSC_FREQ             40000.0

// Set slightly above the settling time of the crystal for a deterministic
// timing of when to go from IDLE to RX.
#define T_EVENT1                (24.0 / RC_OSC_FREQ)

// Time it takes to go from IDLE to RX, from datasheet
#define T_IDLE_TO_RX            133e-6

// The overhead of the payload is 4 bytes preamble, 2 bytes sync, 1 length byte
// and 2 bytes CRC
#define PAYLOAD_OVERHEAD        9

// Consists of 1 byte preamble, 2 byte sync, 1 length byte, 1 data byte and
// 2 byte CRC
#define TIMESTAMP_PACKET_LEN    7


#if( NUM_TIMESTAMP_PACKETS < 1 || NUM_TIMESTAMP_PACKETS > 256 )
  #error The number of timestamps has to be 0-256.
#endif

#ifdef  __cplusplus
}
#endif
#endif