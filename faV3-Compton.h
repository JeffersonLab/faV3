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
  /* 0x0128 */ volatile uint16_t config19;
  /* 0x012A */ volatile uint16_t config20;
  /* 0x012C */ volatile uint16_t config21;
  /* 0x012E */ volatile uint16_t config_reserve[(0x15C - 0x12E) >> 1];
  /* 0x015C */ volatile uint16_t config3;
  /* 0x015E */ volatile uint16_t status3;
  /* 0x0160 */ volatile uint16_t status4;
} faV3_compton_adc_t;

#define FAV3_COMPTON_SUPPORTED_CTRL_FIRMWARE 0x211
#define FAV3_COMPTON_SUPPORTED_PROC_FIRMWARE 0x1101

/* status0 */
#define FAV3_CODE_VERSION_MASK 0x7FFF

/* status1 */
#define FAV3_SAMPLE_FIFO_OVERFLOW (1 << 0)

/* config1 */
#define FAV3_COLLECT_ON  (1 << 0)

/* config6 */
#define FAV3_START_SET_MASK 0xFFF
#define FAV3_START_SET_MIN 5
#define FAV3_START_SET_DEFAULT 5

/* config8 */
#define FAV3_SELF_TRIGGER_NSB_MASK 0xF
#define FAV3_NSB_DEFAULT 5

/* config9 */
#define FAV3_SELF_TRIGGER_NSA_MASK 0xFFF
#define FAV3_NSA_DEFAULT 7

/* config10 */
#define FAV3_HI_THRESHOLD_MASK 0xFFF
#define FAV3_HI_THRESHOLD_DEFAULT 4000

/* config11 */
#define FAV3_NSB1_LO_MASK 0x3FF
#define FAV3_NSB1_LO_DEFAULT 10

/* config12 */
#define FAV3_NSA2_LO_MASK 0x3FF
#define FAV3_NSA2_LO_DEFAULT 10

/* config13 */
#define FAV3_SELF_TRIGGER_THRESHOLD_MASK 0xFFF
#define FAV3_SELF_TRIGGER_THRESHOLD_DEFAULT 20

/* config15 */
#define FAV3_LO_THRESHOLD_MASK 0xFFF
#define FAV3_LO_THRESHOLD_DEFAULT 10

/* config16 */
#define FAV3_SELF_TRIGGER_PRESCALE_MASK 0x3FF
#define FAV3_SELF_TRIGGER_PRESCALE_DEFAULT 0

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
#define FAV3_HYSTERSIS_DEFAULT 0

/* config19 */
#define FAV3_STOP_SET_MSB_MASK 0xFFFF
#define FAV3_STOP_SET_DEFAULT 500

/* config20 */
#define FAV3_STOP_SET_LSB_MASK  0x7F

/* config3 */
#define FAV3_TRIGGER_SELECT_MASK 0x2
#define FAV3_TRIGGER_SELF 0
#define FAV3_TRIGGER_TI   (1 << 1)
#define FAV3_SYNC_DISABLE (1 << 15)



int faV3ComptonInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag);
void faV3ComptonGStatus(int sflag);
int32_t faV3ComptonSetMPSStartStop(int32_t id, uint16_t start, uint32_t stop);
int32_t faV3ComptonGetMPSStartStop(int32_t id, uint16_t *start, uint32_t *stop);
int32_t faV3ComptonSetProc(int32_t id, uint16_t st_nsb, uint16_t st_nsa,
			   uint16_t lo_threshold, uint16_t hi_threshold,
			   uint16_t pulse_threshold, uint16_t pulse_nsb, uint16_t pulse_nsa);
int32_t faV3ComptonGetProc(int32_t id, uint16_t *st_nsb, uint16_t *st_nsa,
			   uint16_t *lo_threshold, uint16_t *hi_threshold,
			   uint16_t *pulse_threshold, uint16_t *pulse_nsb, uint16_t *pulse_nsa);
int32_t faV3ComptonSetPrescale(int32_t id, uint16_t prescale);
int32_t faV3ComptonGetPrescale(int32_t id, uint16_t *prescale);
int32_t faV3ComptonSetHysteresis(int32_t id, uint16_t hysteresis);
int32_t faV3ComptonGetHysteresis(int32_t id, uint16_t *hysteresis);
int32_t faV3ComptonSetReportResults(int32_t id, uint16_t report);
int32_t faV3ComptonGetReportResults(int32_t id, uint16_t *report);

int32_t faV3ComptonPauseProcessingTrigger(int32_t id);
int32_t faV3ComptonResumeProcessingTrigger(int32_t id);
int32_t faV3ComptonSelectTrigger(int32_t id, uint16_t trigger);
int32_t faV3ComptonGetSelectTrigger(int32_t id, uint16_t *trigger);

int32_t faV3ComptonSetMaxTriggerCount(int32_t id, uint16_t max_count);
int32_t faV3ComptonCollectOn(int32_t id);
int32_t faV3ComptonCollectOff(int32_t id);

typedef struct faV3_compton_data_struct
{
  uint32_t new_type;
  uint32_t type;
  uint32_t slot_id_hd;
  uint32_t slot_id_tr;
  uint32_t slot_id_evh;
  uint32_t n_evts;
  uint32_t blk_num;
  uint32_t modID;
  uint32_t PL;
  uint32_t NSB;
  uint32_t NSA;
  uint32_t n_words;
  uint32_t evt_num_1;
  uint32_t evt_num_2;
  uint32_t evt_of_blk;
  uint32_t time_now;
  uint32_t time_low_10;
  uint32_t time_1;    // Type 3: MPS Rising Time
  uint32_t time_2;
  uint32_t chan;
  uint32_t nsamples;  // Type 4: Self Trigger Raw Data
  uint32_t hel;
  uint32_t pulse_start;
  uint32_t delta_hel;
  uint32_t valid_1;
  uint32_t adc_1;
  uint32_t valid_2;
  uint32_t adc_2;
  uint32_t tstop;     // Type 8: Helicity seed
  uint32_t hel_number_lsb;
  uint32_t hel_number_msb;
  uint32_t prescale;    // Type 9: Pulse Parameters
  uint32_t pulse_num;
  uint32_t over;
  uint32_t under;
  uint32_t vpeak;
  uint32_t missed;
  uint32_t nsb_nsa_overlap;
  uint32_t adc_sum;
  uint32_t scaler_data_words;
  uint32_t scaler_data_iword;
  uint32_t scaler_data;
  uint32_t acc_param_word_number;
  uint32_t acc_type;
  uint32_t acc_chan;
  uint32_t nsb_low_x_overlap;
  uint32_t no_nsa_low_x;
} faV3ComptonData_t;

void faV3SetDecodeOutput(FILE *output_file);
void faV3ComptonDataDecode(unsigned int data);
