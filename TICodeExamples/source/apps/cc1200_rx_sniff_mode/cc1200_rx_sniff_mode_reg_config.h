//******************************************************************************
//! @file       cc1200_rx_sniff_mode_reg_config.h
//! @brief      CC1120 register export from SmartRF Studio
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

#ifndef CC1200_RX_SNIFF_MODE_REG_CONFIG_H
#define CC1200_RX_SNIFF_MODE_REG_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************
 * INCLUDES
 */
#include "hal_spi_rf_trxeb.h"
#include "cc120x_spi.h"


/******************************************************************************
 * VARIABLES
 */
// RX filter BW = 104.166667
// Address config = No address check
// Packet length = 125
// Symbol rate = 38.4
// Carrier frequency = 867.999878
// Bit rate = 38.4
// Packet bit length = 0
// Whitening = false
// Manchester enable = false
// Modulation format = 2-GFSK
// Packet length mode = Variable
// Device address = 0
// Deviation = 19.989014
// Rf settings for CC1200
static const registerSetting_t preferredSettings[] = {
    {CC120X_IOCFG2,         0x13},
    {CC120X_IOCFG0,         0x06},
    {CC120X_SYNC_CFG1,      0xA9},
    {CC120X_MODCFG_DEV_E,   0x0B},
    {CC120X_PREAMBLE_CFG1,  0x30},
    {CC120X_PREAMBLE_CFG0,  0x8A},
    {CC120X_IQIC,           0xC8},
    {CC120X_CHAN_BW,        0x10},
    {CC120X_MDMCFG1,        0x42},
    {CC120X_MDMCFG0,        0x05},
    {CC120X_SYMBOL_RATE2,   0x8F},
    {CC120X_SYMBOL_RATE1,   0x75},
    {CC120X_SYMBOL_RATE0,   0x10},
    {CC120X_AGC_REF,        0x27},
    {CC120X_AGC_CS_THR,     0xE4},
    {CC120X_AGC_CFG1,       0x00},
    {CC120X_AGC_CFG0,       0x90},
    {CC120X_SETTLING_CFG,   0x03},
    {CC120X_FS_CFG,         0x12},
    {CC120X_WOR_CFG0,       0x20},
    {CC120X_WOR_EVENT0_LSB, 0xC3},
    {CC120X_PKT_CFG2,       0x00},
    {CC120X_PKT_CFG0,       0x20},
    {CC120X_RFEND_CFG0,     0x09},
    {CC120X_PKT_LEN,        0x7D},
    {CC120X_IF_MIX_CFG,     0x1C},
    {CC120X_TOC_CFG,        0x03},
    {CC120X_MDMCFG2,        0x02},
    {CC120X_FREQ2,          0x56},
    {CC120X_FREQ1,          0xCC},
    {CC120X_FREQ0,          0xCC},
    {CC120X_IF_ADC1,        0xEE},
    {CC120X_IF_ADC0,        0x10},
    {CC120X_FS_DIG1,        0x07},
    {CC120X_FS_DIG0,        0xAF},
    {CC120X_FS_CAL1,        0x40},
    {CC120X_FS_CAL0,        0x0E},
    {CC120X_FS_DIVTWO,      0x03},
    {CC120X_FS_DSM0,        0x33},
    {CC120X_FS_DVC0,        0x17},
    {CC120X_FS_PFD,         0x00},
    {CC120X_FS_PRE,         0x6E},
    {CC120X_FS_REG_DIV_CML, 0x1C},
    {CC120X_FS_SPARE,       0xAC},
    {CC120X_FS_VCO0,        0xB5},
    {CC120X_IFAMP,          0x09},
    {CC120X_XOSC5,          0x0E},
    {CC120X_XOSC1,          0x03},
};

#ifdef  __cplusplus
}
#endif

#endif