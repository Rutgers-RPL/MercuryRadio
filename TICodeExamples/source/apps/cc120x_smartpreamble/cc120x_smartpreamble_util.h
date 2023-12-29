//******************************************************************************
//! @file       cc120x_smartpreamble_util.h
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


#ifndef CC120X_SMARTPREAMBLE_UTIL_H
#define CC120X_SMARTPREAMBLE_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* INCLUDES
*/
#include "hal_types.h"


/*******************************************************************************
* DEFINES
*/
#define CRC16_POLY      0x8005
#define CRC_INIT        0xFFFF

/*******************************************************************************
* PUBLIC TYPES
*/

/**
 * The EVENT0 structure represents one state of the two WOR_EVENT0 registers
 */
struct Event0 {
    uint8 WOR_EVENT0_MSB;
    uint8 WOR_EVENT0_LSB;
};


/*******************************************************************************
* PUBLIC FUNCTIONS
*
* See the .c file for comments on all functions
*/
void generateLookupEvent0Table(struct Event0 LookupEvent0Table[]);
void generateSearchEvent0(struct Event0* searchEvent0);
void generateSmartPreamble(uint8* txBuffer);
uint16 calcCRC(uint8 crcData, uint16 crcReg);

#ifdef  __cplusplus
}
#endif
#endif
