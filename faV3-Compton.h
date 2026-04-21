#pragma once
/**
 * @copyright Copyright 2026, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *                                              Newport News, VA 23606
 *
 * @file      faV3-Compton.h
 *
 * @brief     Header for Library to support the Hall A Compton Polarimeter Firmware
 *
 */

#include <stdint.h>

typedef struct
{
  /* 0x0100 */ volatile uint16_t status0;
  /* 0x0102 */ volatile uint16_t status1;
  /* 0x0104 */ volatile uint16_t status2;
  /* 0x0106 */ volatile uint16_t config1;
  /* 0x0108 */ volatile uint16_t config2;
  /* 0x010A */ volatile uint16_t config4;
  /* 0x010C */ volatile uint16_t config5;
  /* 0x010E */ volatile uint16_t config6;
  /* 0x0110 */ volatile uint16_t config7;
  /* 0x0112 */ volatile uint16_t config8;
  /* 0x0114 */ volatile uint16_t config9;
  /* 0x0116 */ volatile uint16_t config10;
  /* 0x0118 */ volatile uint16_t config11;
  /* 0x011A */ volatile uint16_t config12;
  /* 0x011C */ volatile uint16_t config13;
  /* 0x011E */ volatile uint16_t config14;
  /* 0x0120 */ volatile uint16_t config15;
  /* 0x0122 */ volatile uint16_t config16;
  /* 0x0124 */ volatile uint16_t config17;
  /* 0x0126 */ volatile uint16_t config18;
  /* 0x0128 */ volatile uint16_t config_reserve[(0x15C - 0x128) >> 1];
  /* 0x015C */ volatile uint16_t config3;
  /* 0x015E */ volatile uint16_t status3;
  /* 0x0160 */ volatile uint16_t status4;
} faV3_compton_adc_t;

#define FAV3_COMPTON_SUPPORTED_CTRL_FIRMWARE 0x20E
#define FAV3_COMPTON_SUPPORTED_PROC_FIRMWARE 0xE06

#define FAV3_COMPTON_SUPPORTED_MODES                 1,9,10
#define FAV3_COMPTON_SUPPORTED_NMODES                     3

extern const char *faV3_compton_mode_names[FAV3_MAX_PROC_MODE + 1];

/* status0 */
#define FAV3_CODE_VERSION_MASK 0x7FFF

/* status1 */
#define FAV3_SAMPLE_FIFO_OVERFLOW (1 << 0)

/* config1 */
#define FAV3_COLLECT_ON  (1 << 0)

/* config6 */
#define FAV3_START_SET_MASK 0xFFF
#define FAV3_START_SET_MIN 5

/* config8 */
#define FAV3_SELF_TRIGGER_NSB_MASK 0xF

/* config9 */
#define FAV3_SELF_TRIGGER_NSA_MASK 0xFFF

/* config10 */
#define FAV3_HI_THRESHOLD_MASK 0xFFF

/* config11 */
#define FAV3_NSB1_LO_THRESHOLD_MASK 0x3FF

/* config12 */
#define FAV3_NSB2_LO_THRESHOLD_MASK 0x3FF

/* config13 */
#define FAV3_SELF_TRIGGER_THRESHOLD_MASK 0xFFF

/* config15 */
#define FAV3_LO_THRESHOLD_MASK 0xFFF

/* config16 */
#define FAV3_SELF_TRIGGER_PRESCALE_MASK 0x3FF

/* config17 */
#define FAV3_REPORT_SELF_TRIGGER_SAMPLES (1 << 0)
#define FAV3_REPORT_SELF_TRIGGER         (1 << 1)
#define FAV3_REPORT_ACCUM0               (1 << 2)
#define FAV3_REPORT_ACCUM1               (1 << 3)
#define FAV3_REPORT_ACCUM2               (1 << 4)
#define FAV3_REPORT_ACCUM3               (1 << 5)
#define FAV3_REPORT_ACCUM4               (1 << 6)
#define FAV3_REPORT_ACCUM5               (1 << 7)

/* config18 */
#define FAV3_HYSTERSIS_MASK  0x3F

/* config19 */
#define FAV3_STOP_SET_MSB_MASK  0xFFF

/* config20 */
#define FAV3_STOP_SET_LSB_MASK 0x3F

/* config3 */
#define FAV3_SYNC_DISABLE (1 << 15)

int faV3ComptonInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag);
