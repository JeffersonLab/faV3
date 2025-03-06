#pragma once
/**
 * @copyright Copyright 2024, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *                                              Newport News, VA 23606
 *
 * @author    David Abbott
 *            abbottd@jlab.org                  Jefferson Lab, MS-12B3
 *            Phone: (757) 269-7190             12000 Jefferson Ave.
 *                                              Newport News, VA 23606
 * @file      faV3-HallD.h
 *
 * @brief     Header for Library to support the HallD-Production Firmware
 *
 */

#include <stdint.h>

typedef struct
{
  /* 0x0100 */ volatile uint32_t status0;
  /* 0x0104 */ volatile uint32_t status1;
  /* 0x0108 */ volatile uint32_t status2;
  /* 0x010C */ volatile uint32_t config1;
  /* 0x0110 */ volatile uint32_t config2;
  /* 0x0114 */ volatile uint32_t config4;
  /* 0x0118 */ volatile uint32_t config5;
  /* 0x011C */ volatile uint32_t ptw;
  /* 0x0120 */ volatile uint32_t pl;
  /* 0x0124 */ volatile uint32_t nsb;
  /* 0x0128 */ volatile uint32_t nsa;
  /* 0x012C */ volatile uint32_t thres[8];
  /* 0x014C */ volatile uint32_t config6;
  /* 0x0150 */ volatile uint32_t config7;
  /* 0x0154 */ volatile uint32_t test_wave;
  /* 0x0158 */ volatile uint32_t pedestal[16];
  /* 0x0198 */ volatile uint32_t config3;
  /* 0x019C */ volatile uint32_t status3;
  /* 0x01A0 */ volatile uint32_t status4;
  /* 0x01A4 */ volatile uint32_t rogue_ptw_fall_back;
} faV3_halld_adc_t;

#define FAV3_HALLD_SUPPORTED_CTRL_FIRMWARE 0x20E
#define FAV3_HALLD_SUPPORTED_PROC_FIRMWARE 0xE05

#define FAV3_HALLD_PROC_MODE_PULSE_PARAM  9
#define FAV3_HALLD_PROC_MODE_DEBUG       10

#define FAV3_HALLD_SUPPORTED_MODES                   9,10
#define FAV3_HALLD_SUPPORTED_NMODES                     2

extern const char *faV3_halld_mode_names[FAV3_MAX_PROC_MODE + 1];

#define FAV3_ENABLE_ADC_PARAMETERS_DATA        0x8000
#define FAV3_SUPPRESS_TRIGGER_TIME_DATA       0x10000
#define FAV3_SUPPRESS_TRIGGER_TIME_WORD2_DATA 0x20000
#define FAV3_SUPPRESS_TRIGGER_TIME_MASK       0x30000


/* Define MGT Control bits */
#define FAV3_MGT_FRONT_END_TO_CTP         0x2
#define FAV3_MGT_ENABLE_DATA_ALIGNMENT    0x4
#define FAV3_MGT_HITBITS_TO_CTP           0x8

#define FAV3_ADC_CONFIG1_NP_MASK    0x0030
#define FAV3_ADC_CONFIG1_MODE_MASK  0x0300
#define FAV3_ADC_CONFIG1_NSAT_MASK  0x0C00

#define FAV3_ADC_CONFIG1_CHAN_READ_ENABLE (1<<15)

#define FAV3_ADC_CONFIG6_MNPED_MASK   0x00003C00
#define FAV3_ADC_CONFIG6_PMAXPED_MASK 0x000003FF

#define FAV3_ADC_CONFIG7_NPED_MASK    0x00003C00
#define FAV3_ADC_CONFIG7_MAXPED_MASK  0x000003FF

#define FAV3_ROGUE_PTW_FALL_BACK_MASK 0x0000FFFF

int faV3HallDInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag);
int faV3HallDCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np);
int faV3HallDSetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
			 uint32_t NSB, uint32_t NSA, uint32_t NP,
			 uint32_t NPED, uint32_t MAXPED, uint32_t NSAT);
void faV3HallDGSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
			   uint32_t NSB, uint32_t NSA, uint32_t NP,
			   uint32_t NPED, uint32_t MAXPED, uint32_t NSAT);
int32_t faV3HallDGetProcMode(int id, int *pmode, uint32_t *PL, uint32_t *PTW,
			     uint32_t *NSB, uint32_t *NSA, uint32_t *NP,
			     uint32_t *NPED, uint32_t *MAXPED, uint32_t *NSAT);
int faV3HallDProcPedConfig(int id, int nsamples, int maxvalue);
int faV3HallDGProcPedConfig(int nsamples, int maxvalue);
int faV3HallDSampleConfig(int id, int nsamples, int maxvalue);
int faV3HallDGSampleConfig(int nsamples, int maxvalue);
int faV3HallDReadAllChannelSamples(int id, uint16_t data[16]);
int faV3SetMGTTestMode(int id, uint32_t mode);
int faV3SyncResetMode(int id, uint32_t mode);
int faV3GetAlignmentDebugMode();
int faV3SetHitbitsMode(int id, int enable);
void faV3GSetHitbitsMode(int enable);
int faV3HallDSetRoguePTWFallBack(int id, uint16_t enablemask);
int faV3HallDGetRoguePTWFallBack(int id, uint16_t *enablemask);
int faV3HallDDataInsertAdcParameters(int id, int enable);
void faV3HallDGDataInsertAdcParameters(int enable);
int faV3HallDDataGetInsertAdcParameters(int id);
int faV3HallDDataSuppressTriggerTime(int id, int suppress);
void faV3HallDGDataSuppressTriggerTime(int suppress);
int faV3HallDDataGetSuppressTriggerTime(int id);
int faV3HallDSetDataFormat(int id, int format);
void faV3HallDGSetDataFormat(int format);
int faV3HallDGetDataFormat(int id);
void faV3HallDGStatus(int sflag);
