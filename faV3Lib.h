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
 * @file      faV3Lib.h
 *
 * @brief     Header for JLAB 250MHz FLASH ADC V3 library
 *
 */


#include <stdint.h>

#define FAV3_BOARD_ID       0xadc30000
#define FAV3_MAX_BOARDS             20
#define FAV3_MAX_ADC_CHANNELS       16
#define FAV3_MAX_DATA_PER_CHANNEL  251
#define FAV3_MAX_A32_MEM      0x800000	/* 8 Meg */
#define FAV3_MAX_A32MB_SIZE   0x800000	/*  8 MB */
#define FAV3_VME_INT_LEVEL           3
#define FAV3_VME_INT_VEC          0xFA

#define FAV3_SUPPORTED_CTRL_FIRMWARE 0x0E
#define FAV3_SUPPORTED_CTRL_FIRMWARE_NUMBER 1
#define FAV3_SUPPORTED_PROC_FIRMWARE 0xE00
#define FAV3_SUPPORTED_PROC_FIRMWARE_NUMBER 1

/* Macros to help with register spacers */
#define MERGE_(a,b)  a##b
#define LABEL_(a) MERGE_(uint32_t faV3blank, a)
#define BLANK LABEL_(__LINE__)

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
  /* 0x014C */ volatile uint32_t ptw_last_adr;
  /* 0x0150 */ volatile uint32_t ptw_max_buf;
  /* 0x0154 */ volatile uint32_t test_wave;
  /* 0x0158 */ volatile uint32_t pedestal[16];
  /* 0x0198 */ volatile uint32_t config3;
  /* 0x019C */ volatile uint32_t status3;
} faV3_adc_t;

typedef struct
{
  /* 0x0200 */ volatile uint32_t status;
  /* 0x0204 */ volatile uint32_t cfg;
  /* 0x0208 */ volatile uint32_t hit_width;
  /* 0x020C */ volatile uint32_t trig_delay;
  /* 0x0210 */ volatile uint32_t trig_width;
  /* 0x0214 */ volatile uint32_t window_bits;
  /* 0x0218 */ volatile uint32_t window_width;
  /* 0x021C */ volatile uint32_t coin_bits;
  /* 0x0220 */ volatile uint32_t pattern;
  /* 0x0224 */ volatile uint32_t fifo;
  /* 0x0228 */ volatile uint32_t sum_thresh;
} faV3_hitsum_t;

typedef struct
{
  /* 0x0300 */ volatile uint32_t scaler[16];
  /* 0x0340 */ volatile uint32_t time_count;
} faV3_scalers_t;

typedef struct
{
  /* 0x0400 */ volatile uint32_t testbit;
  /* 0x0404 */ volatile uint32_t count_250;
  /* 0x0408 */ volatile uint32_t count_sync;
  /* 0x040C */ volatile uint32_t count_trig1;
  /* 0x0410 */ volatile uint32_t count_trig2;
} faV3_system_test_t;

typedef struct
{
  /* 0x0500 */ volatile uint32_t state_level;
  /* 0x0504 */ volatile uint32_t state_csr;
  /* 0x0508 */ volatile uint32_t state_value;
  /* 0x050C */ BLANK[1];
  /* 0x0510 */ volatile uint32_t berr_driven_count;
  /* 0x0514 */ volatile uint32_t retry_driven_count;
  /* 0x0518 */ BLANK[1];
  /* 0x051C */ volatile uint32_t vxs_output_status;
  /* 0x0520 */ volatile uint32_t sparsify_control;
  /* 0x0524 */ volatile uint32_t sparsify_status;
  /* 0x0528 */ volatile uint32_t first_trigger_mismatch;
  /* 0x052C */ volatile uint32_t trigger_mismatch_counter;
  /* 0x0530 */ volatile uint32_t triggers_processed;
} faV3_aux_t;

typedef struct
{
  /* 0x0000 */ volatile uint32_t version;
  /* 0x0004 */ volatile uint32_t csr;
  /* 0x0008 */ volatile uint32_t ctrl1;
  /* 0x000C */ volatile uint32_t ctrl2;
  /* 0x0010 */ volatile uint32_t blocklevel;
  /* 0x0014 */ volatile uint32_t intr;
  /* 0x0018 */ volatile uint32_t adr32;
  /* 0x001C */ volatile uint32_t adr_mb;
  /* 0x0020 */ volatile uint32_t sec_adr;
  /* 0x0024 */ BLANK[1];
  /* 0x0028 */ volatile uint32_t trig_cfg;
  /* 0x002C */ volatile uint32_t reset;
  /* 0x0030 */ volatile uint32_t trig_count;
  /* 0x0034 */ volatile uint32_t ev_count;
  /* 0x0038 */ volatile uint32_t blk_count;
  /* 0x003C */ volatile uint32_t blk_fifo_count;
  /* 0x0040 */ volatile uint32_t blk_wrd_count;
  /* 0x0044 */ volatile uint32_t trig_live_count;
  /* 0x0048 */ volatile uint32_t ram_word_count;
  /* 0x004C */ volatile uint32_t flow_status;
  /* 0x0050 */ volatile uint32_t dac_csr;
  /* 0x0054 */ volatile uint32_t dac_data;
  /* 0x0058 */ BLANK[(0x70 - 0x58) >> 2];
  /* 0x0070 */ volatile uint32_t status1;
  /* 0x0074 */ volatile uint32_t status2;
  /* 0x0078 */ volatile uint32_t status3;
  /* 0x007C */ BLANK[(0x84 - 0x7C) >> 2];
  /* 0x0084 */ volatile uint32_t trigger_control;
  /* 0x0088 */ volatile uint32_t trig21_del;
  /* 0x008C */ volatile uint32_t mem_adr;
  /* 0x0090 */ volatile uint32_t mem1_data;
  /* 0x0094 */ volatile uint32_t mem2_data;
  /* 0x0098 */ volatile uint32_t prom_reg1;
  /* 0x009C */ volatile uint32_t prom_reg2;
  /* 0x00A0 */ volatile uint32_t berr_count;
  /* 0x00A4 */ volatile uint32_t berr_in_count;
  /* 0x00A8 */ volatile uint32_t proc_words_scal;
  /* 0x00AC */ volatile uint32_t lost_trig_scal;
  /* 0x00B0 */ volatile uint32_t header_scal;
  /* 0x00B4 */ volatile uint32_t trig2_scal;
  /* 0x00B8 */ volatile uint32_t trailer_scal;
  /* 0x00BC */ volatile uint32_t syncreset_scal;
  /* 0x00C0 */ volatile uint32_t busy_level;
  /* 0x00C4 */ volatile uint32_t gen_evt_header;
  /* 0x00C8 */ volatile uint32_t gen_evt_data;
  /* 0x00CC */ volatile uint32_t gen_evt_trail;
  /* 0x00D0 */ volatile uint32_t status_mgt;
  /* 0x00D4 */ volatile uint32_t ctrl_mgt;
  /* 0x00D8 */ volatile uint32_t mem_adr_w;
  /* 0x00DC */ volatile uint32_t mem_adr_r;
  /* 0x00E0 */ volatile uint32_t scaler_ctrl;
  /* 0x00E4 */ volatile uint32_t serial_reg[3];
  /* 0x00F0 */ volatile uint32_t scaler_insert;
  /* 0x00F4 */ volatile uint32_t sum_threshold;
  /* 0x00F8 */ volatile uint32_t sum_data;
  /* 0x00FC */ volatile uint32_t sys_mon;

  /* 0x0100 */ volatile faV3_adc_t adc;
  /* 0x01A0 */ BLANK[(0x200 - 0x1A0) >> 2];

  /* 0x0200 */ volatile faV3_hitsum_t hitsum;
  /* 0x022C */ BLANK[(0x300 - 0x22C) >> 2];

  /* 0x0300 */ volatile faV3_scalers_t scalers;
  /* 0x0344 */ BLANK[(0x400 - 0x344) >> 2];

  /* 0x0400 */ volatile faV3_system_test_t system_test;
  /* 0x0414 */ BLANK[(0x500 - 0x414) >> 2];

  /* 0x0500 */ volatile faV3_aux_t aux;
} faV3_t;

typedef struct faV3_data_struct
{
  uint32_t new_type;
  uint32_t type;
  uint32_t slot_id_hd;
  uint32_t slot_id_tr;
  uint32_t n_evts;
  uint32_t blk_num;
  uint32_t n_words;
  uint32_t evt_num_1;
  uint32_t trig_time;
  uint32_t time_now;
  uint32_t time_1;
  uint32_t time_2;
  uint32_t chan;
  uint32_t width;
  uint32_t valid_1;
  uint32_t adc_1;
  uint32_t valid_2;
  uint32_t adc_2;
  uint32_t over;
  uint32_t adc_sum;
  uint32_t pulse_num;
  uint32_t thres_bin;
  uint32_t quality;
  uint32_t evt_in_blk;
  uint32_t ped_q;
  uint32_t ped_sum;
  uint32_t integral;
  uint32_t integral_q;
  uint32_t above_thres;
  uint32_t time;
  uint32_t time_q;
  uint32_t chan_a;
  uint32_t source_a;
  uint32_t chan_b;
  uint32_t source_b;
  uint32_t group;
  uint32_t time_coarse;
  uint32_t time_fine;
  uint32_t vmin;
  uint32_t vpeak;
  uint32_t adc_latency;
  uint32_t adc_nsb;
  uint32_t adc_nsa;
  uint32_t scaler[18];
} faV3data_t;


typedef struct faV3_sdc_struct
{
  volatile uint16_t csr;
  volatile uint16_t ctrl;
  volatile uint16_t busy_enable;
  volatile uint16_t busy_status;
} faV3sdc_t;


/* FADC Special Board IDs */

/* Define CSR Bits */
#define FAV3_CSR_EVENT_AVAILABLE                (1 << 0)
#define FAV3_CSR_BLOCK_LEVEL_FLAG               (1 << 1)
#define FAV3_CSR_BLOCK_READY                    (1 << 2)
#define FAV3_CSR_BERR_STATUS                    (1 << 3)
#define FAV3_CSR_TOKEN_STATUS                   (1 << 4)
#define FAV3_CSR_COMPRESSION_ERROR              (1 << 5)
#define FAV3_DAC_SERIALIZATION_ACTIVE          (1 << 10)
#define FAV3_CSR_FIFO_EMPTY                    (1 << 11)
#define FAV3_CSR_FIFO_ALMOST_EMPTY             (1 << 12)
#define FAV3_CSR_FIFO_HALF_FULL                (1 << 13)
#define FAV3_CSR_FIFO_ALMOST_FULL              (1 << 14)
#define FAV3_CSR_FIFO_FULL                     (1 << 15)
#define FAV3_CSR_SOFT_PULSE_TRIG2              (1 << 20)
#define FAV3_CSR_SOFT_CLEAR                    (1 << 21)
#define FAV3_CSR_DATA_STREAM_SCALERS           (1 << 22)
#define FAV3_CSR_FORCE_EOB_INSERT              (1 << 23)
#define FAV3_CSR_FORCE_EOB_SUCCESS             (1 << 24)
#define FAV3_CSR_FORCE_EOB_FAILED              (1 << 25)
#define FAV3_CSR_ERROR_MASK                   0x0c000000
#define FAV3_CSR_ERROR_CLEAR                   (1 << 27)
#define FAV3_CSR_SYNC                          (1 << 28)
#define FAV3_CSR_TRIGGER                       (1 << 29)
#define FAV3_CSR_SOFT_RESET                    (1 << 30)
#define FAV3_CSR_HARD_RESET                    (1 << 31)

/* Define ctrl1 Bits */
#define FAV3_REF_CLK_INTERNAL        0x0
#define FAV3_REF_CLK_FP              0x1
#define FAV3_REF_CLK_P0              0x2
#define FAV3_REF_CLK_MASK            0x3
#define FAV3_ENABLE_INTERNAL_CLK     0x8

#define FAV3_TRIG_FP                0x00
#define FAV3_TRIG_FP_ISYNC          0x10
#define FAV3_TRIG_P0                0x20
#define FAV3_TRIG_P0_ISYNC          0x30
#define FAV3_TRIG_VME_PLAYBACK      0x50
#define FAV3_TRIG_VME               0x60
#define FAV3_TRIG_INTERNAL          0x70
#define FAV3_TRIG_MASK              0x70
#define FAV3_ENABLE_SOFT_TRIG       0x80

#define FAV3_SRESET_FP             0x000
#define FAV3_SRESET_FP_ISYNC       0x100
#define FAV3_SRESET_P0             0x200
#define FAV3_SRESET_P0_ISYNC       0x300
#define FAV3_SRESET_VME            0x600
#define FAV3_SRESET_NONE           0x700
#define FAV3_SRESET_MASK           0x700
#define FAV3_ENABLE_SOFT_SRESET    0x800

#define FAV3_ENABLE_LIVE_TRIG_OUT  0x1000
#define FAV3_ENABLE_TRIG_OUT_FP    0x2000
#define FAV3_ENABLE_TRIG_OUT_P0    0x4000
#define FAV3_TRIGOUT_MASK          0x7000

#define FAV3_ENABLE_BLKLVL_INT      0x40000
#define FAV3_ENABLE_BERR           0x100000
#define FAV3_ENABLE_MULTIBLOCK     0x200000
#define FAV3_FIRST_BOARD           0x400000
#define FAV3_LAST_BOARD            0x800000
#define FAV3_ENABLE_DEBUG         0x2000000
#define FAV3_MB_TOKEN_VIA_P0     0x10000000
#define FAV3_MB_TOKEN_VIA_P2     0x20000000

/* Define ctrl2 Bits */
#define FAV3_CTRL_GO               (1 << 0)
#define FAV3_CTRL_ENABLE_TRIG      (1 << 1)
#define FAV3_CTRL_ENABLE_SRESET    (1 << 2)
#define FAV3_CTRL_ENABLE_INT_TRIG  (1 << 3)
#define FAV3_CTRL_ENABLE_MASK      0x7
#define FAV3_CTRL_ENABLED          0x7
#define FAV3_CTRL_STREAM_ENABLE    (1 << 4)
#define FAV3_CTRL_COMPRESS_DISABLE 0x000
#define FAV3_CTRL_COMPRESS_VERIFY  0x240
#define FAV3_CTRL_COMPRESS_ENABLE  0x280
#define FAV3_CTRL_COMPRESS_MASK    0x2C0
#define FAV3_CTRL_VXS_RO_ENABLE    (1 << 10)

/* Define Reset Bits */
#define FAV3_RESET_CNTL_FPGA              0x1
#define FAV3_RESET_ADC_FPGA1              0x2
#define FAV3_RESET_ADC_FIFO1            0x100
#define FAV3_RESET_DAC                  0x800
#define FAV3_RESET_EXT_RAM_PT          0x1000
#define FAV3_RESET_TOKEN              0x10000
#define FAV3_RESET_ALL                 0xFFFF

/* Define trig_scal bits */
#define FAV3_TRIG_COUNT_RESET          (1 << 31)

/* Define trigger_control bits */
#define FAV3_TRIGCTL_TRIGSTOP_EN      (1 << 31)
#define FAV3_TRIGCTL_MAX2_MASK     0x00FF0000
#define FAV3_TRIGCTL_BUSY_EN          (1 << 15)
#define FAV3_TRIGCTL_MAX1_MASK     0x000000FF

/* Define trig21 delay bits */
#define FAV3_TRIG21_DELAY_MASK     0x00000FFF

/* Define MGT Control bits */
#define FAV3_RELEASE_MGT_RESET            0x0
#define FAV3_MGT_RESET                    0x1

#define FAV3_INT_ACTIVE             0x1000000
#define FAV3_INT_IACKSTAT_WAIT      0x2000000
#define FAV3_INT_IACKSTAT_ONBT      0x4000000


/* Define RAM Bits */
#define FAV3_RAM_DATA_MASK       0x000fffff
#define FAV3_RAM_FULL            0x00100000
#define FAV3_RAM_EMPTY           0x00200000
#define FAV3_RAM_ADR_INCREMENT   0x00100000

/* Define dac_csr and dac_data bits */
#define FAV3_DAC_CHAN_MASK 0x0000000F
#define FAV3_DAC_READY     (1 << 16)
#define FAV3_DAC_SUCCESS   (1 << 17)
#define FAV3_DAC_NOT_READY (1 << 18)
#define FAV3_DAC_TIMEOUT   (1 << 19)
#define FAV3_DAC_CLEAR     (1 << 29)
#define FAV3_DAC_INIT_DONE (1 << 30)
#define FAV3_DAC_INIT      (1 << 31)

#define FAV3_DAC_DATA_MASK         0x00000FFF
#define FAV3_DAC_DATA_REG_SELECT01 0x00003000
#define FAV3_DAC_DATA_CHAN_MASK    0x0003C000
#define FAV3_DAC_DATA_REG_SELECTAB (1 << 21)
#define FAV3_DAC_MAX_VALUE         0xFFF

/* Define Bit Masks */
#define FAV3_VERSION_MASK        0x000000ff
#define FAV3_BOARD_MASK          0xffff0000
#define FAV3_CSR_MASK            0x0fffffff
#define FAV3_CONTROL_MASK        0xffffffff
#define FAV3_CONTROL2_MASK       0xc00432ff
#define FAV3_EVENT_COUNT_MASK    0xffffff
#define FAV3_BLOCK_COUNT_MASK    0xfffff
#define FAV3_BLOCK_LEVEL_MASK    0xffff
#define FAV3_INT_ENABLE_MASK     0xc0000
#define FAV3_INT_VEC_MASK        0xff
#define FAV3_INT_LEVEL_MASK      0x700
#define FAV3_SLOT_ID_MASK        0x1f0000
#define FAV3_DAC_VALUE_MASK      0x0fff
#define FAV3_THR_VALUE_MASK      0x0fff
#define FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK     0x1000
#define FAV3_THR_INVERT_MASK     0x4000
#define FAV3_THR_IGNORE_MASK     0x8000
#define FAV3_PLAYBACK_DIS_MASK   0x2000

#define FAV3_REF_CLK_SEL_MASK        0x7
#define FAV3_TRIG_SEL_MASK          0x70
#define FAV3_SRESET_SEL_MASK       0x700

#define FAV3_BUSY_LEVEL_MASK     0xfffff
#define FAV3_FORCE_BUSY       0x80000000

#define FAV3_A32_ENABLE        0x1
#define FAV3_AMB_ENABLE        0x1
#define FAV3_A32_ADDR_MASK     0xff80	/* 8 MB chunks */
#define FAV3_AMB_MIN_MASK      0xff80
#define FAV3_AMB_MAX_MASK      0xff800000

#define FAV3_SADR_AUTO_INCREMENT   0x10000
#define FAV3_PPG_WRITE_VALUE       0x8000
#define FAV3_PPG_SAMPLE_MASK       0x1fff
#define FAV3_PPG_MAX_SAMPLES       512
#define FAV3_PPG_ENABLE            0x80

/* Define system_monitor Bits/Masks */
#define FAV3_SYSMON_CTRL_TEMP_MASK        0x3FF
#define FAV3_SYSMON_FPGA_CORE_V_MASK 0x001FF800
#define FAV3_SYSMON_FPGA_AUX_V_MASK  0xFFC00000

/* Define MGT Status Bits/Masks */
#define FAV3_MGT_GTX1_LANE1_UP      0x1
#define FAV3_MGT_GTX1_LANE2_UP      0x2
#define FAV3_MGT_GTX1_CHANNEL_UP    0x4
#define FAV3_MGT_GTX1_HARD_ERROR    0x8
#define FAV3_MGT_GTX1_SOFT_ERROR    0x10
#define FAV3_MGT_GTX2_LANE1_UP      0x20
#define FAV3_MGT_GTX2_LANE2_UP      0x40
#define FAV3_MGT_GTX2_CHANNEL_UP    0x80
#define FAV3_MGT_GTX2_HARD_ERROR    0x100
#define FAV3_MGT_GTX2_SOFT_ERROR    0x200
#define FAV3_MGT_SUM_DATA_VALID     0x400
#define FAV3_MGT_RESET_ASSERTED     0x800


/* Define default ADC Processing prarmeters
   values are in clock cycles 4ns/cycle */

#define FAV3_ADC_NS_PER_CLK      4

#define FAV3_MAX_PROC_MODE         10
#define FAV3_ADC_PROC_MASK        0x7
#define FAV3_ADC_PROC_ENABLE      0x8
#define FAV3_ADC_PROC_MODE_WINDOW   1
#define FAV3_ADC_PROC_MODE_PEAK     2
#define FAV3_ADC_PROC_MODE_SUM      3
#define FAV3_ADC_PROC_MODE_TIME     7

#define FAV3_SUPPORTED_MODES                   9,10
#define FAV3_SUPPORTED_NMODES                     2

extern const char *faV3_mode_names[FAV3_MAX_PROC_MODE + 1];

#define FAV3_ADC_STATUS1_TRIG_RCV_DONE (1<<15)
#define FAV3_ADC_STATUS1_TRIGNUM_MASK  0x0FFF

#define FAV3_ADC_STATUS2_CHAN_DATA_MASK    0x3FFF

#define FAV3_ADC_CONFIG1_NSAT_MASK  0x0C00
#define FAV3_ADC_CONFIG1_TNSAT_MASK 0x3000

#define FAV3_ADC_CONFIG3_TPT_MASK   0x0FFF


#define FAV3_ADC_VERSION_MASK  0x7fff
#define FAV3_ADC_PLAYBACK_MODE 0x0080

#define FAV3_ADC_PEAK_MASK     0x0070
#define FAV3_ADC_CHAN_MASK     0xffff

#define FAV3_ADC_DEFAULT_PL     50
#define FAV3_ADC_DEFAULT_PTW    50
#define FAV3_ADC_DEFAULT_NSB     5
#define FAV3_ADC_DEFAULT_NSA    10
#define FAV3_ADC_DEFAULT_NP      4
#define FAV3_ADC_DEFAULT_THRESH  0
#define FAV3_ADC_DEFAULT_NPED    4
#define FAV3_ADC_DEFAULT_MAXPED  1
#define FAV3_ADC_DEFAULT_NSAT    1
#define FAV3_ADC_DEFAULT_MNPED   4
#define FAV3_ADC_DEFAULT_TNSA   10
#define FAV3_ADC_DEFAULT_TNSB   10
#define FAV3_ADC_DEFAULT_TNSAT   1
#define FAV3_ADC_DEFAULT_TPT     0

#define FAV3_ADC_MAX_PL       2000
#define FAV3_ADC_MAX_PTW       512
#define FAV3_ADC_MAX_NSB       500
#define FAV3_ADC_MAX_NSA       500
#define FAV3_ADC_MAX_NP          1
#define FAV3_ADC_MAX_THRESH   1023
#define FAV3_ADC_MAX_NPED       15
#define FAV3_ADC_MAX_MAXPED   1023
#define FAV3_ADC_MAX_NSAT        4
#define FAV3_ADC_MAX_MNPED      15
#define FAV3_ADC_MAX_TNSA       63
#define FAV3_ADC_MAX_TNSB       15
#define FAV3_ADC_MAX_TNSAT       4
#define FAV3_ADC_MAX_TPT      4095

#define FAV3_ADC_MIN_PL          1
#define FAV3_ADC_MIN_PTW         1
#define FAV3_ADC_MIN_NSB         1
#define FAV3_ADC_MIN_NSA         1
#define FAV3_ADC_MIN_NP          1
#define FAV3_ADC_MIN_THRESH      0
#define FAV3_ADC_MIN_NPED        4
#define FAV3_ADC_MIN_MAXPED      0
#define FAV3_ADC_MIN_NSAT        1
#define FAV3_ADC_MIN_MNPED       4
#define FAV3_ADC_MIN_TNSA        2
#define FAV3_ADC_MIN_TNSB        2
#define FAV3_ADC_MIN_TNSAT       2
#define FAV3_ADC_MIN_TPT         0

#define FAV3_ADC_NSB_READBACK_MASK    0x0000000F
#define FAV3_ADC_NSB_NEGATIVE         (1<<3)
#define FAV3_ADC_TNSB_MASK            0x00001E00

#define FAV3_ADC_NSA_READBACK_MASK    0x000001FF
#define FAV3_ADC_TNSA_MASK            0x00007E00

#define FAV3_ADC_TPT_MASK    0xFFF

#define FAV3_ADC_PEDESTAL_MASK   0xffff
#define FAV3_ADC_DELAY_MASK      0x1ff

/* Define parameters for HITSUM (Trigger) FPGA */
#define FAV3_ITRIG_VERSION_MASK     0x1fff
#define FAV3_ITRIG_CONFIG_MASK      0x001F
#define FAV3_ITRIG_ENABLE_MASK      0x0010
#define FAV3_ITRIG_P2OUT_MASK       0x0008
#define FAV3_ITRIG_MODE_MASK        0x0007

#define FAV3_ITRIG_TABLE_MODE       0
#define FAV3_ITRIG_COIN_MODE        1
#define FAV3_ITRIG_WINDOW_MODE      2
#define FAV3_ITRIG_UNDEF_MODE       3
#define FAV3_ITRIG_SUM_MODE         4

#define FAV3_ITRIG_DISABLED         0x0010
#define FAV3_ITRIG_HB_P2OUT         0x0008
#define FAV3_ITRIG_OUT_MASK         0x0000f000  /* Bits from ctrl1 register */
#define FAV3_ITRIG_OUT_FP           0x00003000

#define FAV3_ITRIG_MIN_WIDTH        0x01	/* 4 nsec */
#define FAV3_ITRIG_MAX_WIDTH        0x100	/* 1.024 microsec */
#define FAV3_ITRIG_MAX_DELAY        0xf00	/* 15.360 microsec */

#define FAV3_ITRIG_WINDOW_MAX_WIDTH 0x200	/* 2.048 microsec */

#define FAV3_ITRIG_MAX_HB_WIDTH     0xff /* 1020 ns */
#define FAV3_ITRIG_MAX_HB_DELAY     0xf  /*   60 ns */

#define FAV3_ITRIG_HB_INFO_MASK     0xffff
#define FAV3_ITRIG_HB_WIDTH_MASK    0x00ff
#define FAV3_ITRIG_HB_DELAY_MASK    0x1f00


/* Define State registers bits and masks */
#define FAV3_STATE_LEVEL_CAPTURE_MASK    0x000000FF

#define FAV3_STATE_CSR_ARM_BUFFER        (1<<31)
#define FAV3_STATE_CSR_BUFFER_FULL       (1<<27)
#define FAV3_STATE_CSR_BUFFER_EMPTY      (1<<26)
#define FAV3_STATE_CSR_BUFFER_WORDS_MASK 0x000000FF

#define FAV3_STATE_VALUE_MASK            0x0000FFFF


/* Define ADC Data Types and Masks */

#define FAV3_DATA_TYPE_DEFINE       0x80000000
#define FAV3_DATA_TYPE_MASK         0x78000000
#define FAV3_DATA_SLOT_MASK         0x07c00000

#define FAV3_DATA_BLOCK_HEADER      0x00000000
#define FAV3_DATA_BLOCK_TRAILER     0x08000000
#define FAV3_DATA_EVENT_HEADER      0x10000000
#define FAV3_DATA_TRIGGER_TIME      0x18000000
#define FAV3_DATA_WINDOW_RAW        0x20000000
#define FAV3_DATA_WINDOW_SUM        0x28000000
#define FAV3_DATA_PULSE_RAW         0x30000000
#define FAV3_DATA_PULSE_INTEGRAL    0x38000000
#define FAV3_DATA_PULSE_TIME        0x40000000
#define FAV3_DATA_STREAM            0x48000000
#define FAV3_DATA_INVALID           0x70000000
#define FAV3_DATA_FILLER            0x78000000
#define FAV3_DUMMY_DATA             0xf800fafa

#define FAV3_DATA_BLKNUM_MASK       0x0000003f
#define FAV3_DATA_WRDCNT_MASK       0x003fffff
#define FAV3_DATA_TRIGNUM_MASK      0x07ffffff

/* Define Firmware registers Data types and Masks */
#define FAV3_PROMREG1_SRAM_TO_PROM1    0x0
#define FAV3_PROMREG1_SRAM_TO_PROM2    0x1
#define FAV3_PROMREG1_PROM1_TO_SRAM    0x3
#define FAV3_PROMREG1_PROM2_TO_SRAM    0x4
#define FAV3_PROMREG1_GET_ID1          0x6
#define FAV3_PROMREG1_GET_ID2          0x7
#define FAV3_PROMREG1_ERASE_PROM1      0x9
#define FAV3_PROMREG1_ERASE_PROM2      0xA
#define FAV3_PROMREG1_REBOOT_FPGA1     0xC
#define FAV3_PROMREG1_REBOOT_FPGA2     0xD
#define FAV3_PROMREG1_READY            (1<<31)

#define FAV3_MEM_ADR_INCR_MEM1      (1<<31)
#define FAV3_MEM_ADR_INCR_MEM2      (1<<30)

/* Define Scaler Control bits */
#define FAV3_SCALER_CTRL_ENABLE     (1<<0)
#define FAV3_SCALER_CTRL_LATCH      (1<<1)
#define FAV3_SCALER_CTRL_RESET      (1<<2)
#define FAV3_SCALER_CTRL_MASK        (0x7)

/* Define Scaler Insert Mask */
#define FAV3_SCALER_INSERT_MASK   0x0000FFFF

/* Define Sum Threshold bits and masks */
#define FAV3_SUM_THRESHOLD_DREADY      (1<<31)
#define FAV3_SUM_THRESHOLD_MASK     0x0000FFFF

/* Define Sum Data bits and masks */
#define FAV3_SUM_DATA_ARM_HISTORY_BUFFER    (1<<31)
#define FAV3_SUM_DATA_SAMPLE_MASK        0x0000FFFF

/* Define Serial Number bits and masks */
#define FAV3_SERIAL_NUMBER_ACDI               0x41434449
#define FAV3_SERIAL_NUMBER_ADV_ASSEM_MASK     0xFF000000
#define FAV3_SERIAL_NUMBER_ADV_ASSEM          0x42000000
#define FAV3_SERIAL_NUMBER_ACDI_BOARDID_MASK  0x0000FFFF
#define FAV3_SERIAL_NUMBER_ADV_MNFID1         0x42323135
#define FAV3_SERIAL_NUMBER_ADV_MNFID2_MASK    0xFFFF0000
#define FAV3_SERIAL_NUMBER_ADV_MNFID2         0x39350000
#define FAV3_SERIAL_NUMBER_ADV_BOARDID2_MASK  0x000000FF


/* Define FADC Signal Distribution card bits */

#define FAV3_SDC_BOARD_ID    0x0200
#define FAV3_SDC_BOARD_MASK  0xfffe
#define FAV3_SDC_ADR_MASK    0xffc0

#define FAV3SDC_BUSY_MASK   0x00ff

#define FAV3SDC_CSR_BUSY           0x1
#define FAV3SDC_CSR_TRIG          0x20
#define FAV3SDC_CSR_SRESET        0x40
#define FAV3SDC_CSR_INIT          0x80
#define FAV3SDC_CSR_MASK        0x00e1

#define FAV3SDC_CTRL_CLK_EXT               0x1
#define FAV3SDC_CTRL_NOSYNC_TRIG          0x10
#define FAV3SDC_CTRL_ENABLE_SOFT_TRIG     0x20
#define FAV3SDC_CTRL_NOSYNC_SRESET       0x100
#define FAV3SDC_CTRL_ENABLE_SOFT_SRESET  0x200
#define FAV3SDC_CTRL_ASSERT_SOFT_BUSY   0x8000
#define FAV3SDC_CTRL_MASK               0x8331

/* Definitions for FADC Firmware Tools */
#define FAV3_FIRMWARE_LX110    0
#define FAV3_FIRMWARE_FX70T    1

#define FAV3_CTRL1_SYSTEM_TEST_MODE   (1<<31)

#define FAV3_TESTBIT_TRIGOUT          (1<<0)
#define FAV3_TESTBIT_BUSYOUT          (1<<1)
#define FAV3_TESTBIT_SDLINKOUT        (1<<2)
#define FAV3_TESTBIT_TOKENOUT         (1<<3)
#define FAV3_TESTBIT_STATBITB         (1<<8)
#define FAV3_TESTBIT_TOKENIN          (1<<9)
#define FAV3_TESTBIT_CLOCK250_STATUS  (1<<15)

#define FAV3_CLOCK250COUNT_RESET   (0<<0)
#define FAV3_CLOCK250COUNT_START   (1<<0)
#define FAV3_SYNCP0COUNT_RESET     (0<<0)
#define FAV3_TRIG1P0COUNT_RESET    (0<<0)
#define FAV3_TRIG2P0COUNT_RESET    (0<<0)

/* 0x520 sparse control */
#define FAV3_SPARSE_CONTROL_BYPASS (1 << 0)
#define FAV3_SPARSE_CONTROL_MODE_MASK 0x0000000E

/* 0x524 sparse status */
#define FAV3_SPARSE_STATUS_MASK 0x00000003
#define FAV3_SPARSE_STATUS_WAIT_CHANNEL_MASK (1 << 0)
#define FAV3_SPARSE_STATUS_ACTIVE (1 << 1)
#define FAV3_SPARSE_STATUS_CLEAR (1 << 31)



/* faInit initialization flag bits */
#define FAV3_INIT_SOFT_SYNCRESET      (0<<0)
#define FAV3_INIT_EXT_SYNCRESET       (1<<0)
#define FAV3_INIT_SOFT_TRIG           (0<<1)
#define FAV3_INIT_FP_TRIG             (1<<1)
#define FAV3_INIT_VXS_TRIG            (2<<1)
#define FAV3_INIT_INT_CLKSRC          (0<<4)
#define FAV3_INIT_FP_CLKSRC           (1<<4)
#define FAV3_INIT_VXS_CLKSRC          (2<<4)
#define FAV3_INIT_P2_CLKSRC           ((1<<4) | (2<<4))
#define FAV3_INIT_SKIP                (1<<16)
#define FAV3_INIT_USE_ADDRLIST        (1<<17)
#define FAV3_INIT_SKIP_FIRMWARE_CHECK (1<<18)

/* Define Init Flag bits for Clock Source */
#define FAV3_SOURCE_INT         FAV3_INIT_INT_CLKSRC
#define FAV3_SOURCE_SDC         FAV3_INIT_FP_CLKSRC
#define FAV3_SOURCE_VXS         FAV3_INIT_VXS_CLKSRC
#define FAV3_SOURCE_MASK        0x30


/* fadcBlockError values */
#define FAV3_BLOCKERROR_NO_ERROR          0
#define FAV3_BLOCKERROR_TERM_ON_WORDCOUNT 1
#define FAV3_BLOCKERROR_UNKNOWN_BUS_ERROR 2
#define FAV3_BLOCKERROR_ZERO_WORD_COUNT   3
#define FAV3_BLOCKERROR_DMADONE_ERROR     4
#define FAV3_BLOCKERROR_NTYPES            5

/* Number of regs to read in faDebugOutput */
#define FAV3_DEBUG_OUTPUT_NREGS 57

enum faV3_firmware_functions
  {
    FAV3_FW_CTRL = 0,
    FAV3_FW_PROC = 1,
    FAV3_FW_FUNCTION_MAX
  };

enum faV3_ctrl_firmware_types
  {
    FAV3_FW_CTRL_COMMON = 0,
    FAV3_FW_CTRL_MAX
  };

enum faV3_proc_firmware_types
  {
    FAV3_FW_PROC_COMMON = 0,
    FAV3_FW_PROC_HALLD_PRODUCTION = 1,
    FAV3_FW_PROC_MAX
  };


/* Function Prototypes */
int faV3Init(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag);
void faV3SetA32BaseAddress(uint32_t addr);
int faV3Slot(uint32_t i);
int faV3Id(uint32_t slot);
int faV3GetN();

int faV3SetClockSource(int id, int clkSrc);
int faV3GSetClockSource(int clkSrc);

void faV3Status(int id, int sflag);
void faV3GStatus(int sflag);
uint32_t faV3GetFirmwareVersions(int id, int pflag);

int faV3SetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
		    uint32_t NSB, uint32_t NSA, uint32_t NP);
void faV3GSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
		      uint32_t NSB, uint32_t NSA, uint32_t NP);
int faV3GetProcMode(int id, int *pmode, uint32_t * PL, uint32_t * PTW,
		  uint32_t * NSB, uint32_t * NSA, uint32_t * NP);


int faV3SetTriggerStopCondition(int id, int trigger_max);
int faV3SetTriggerBusyCondition(int id, int trigger_max);
int faV3SetTriggerPathSamples(int id, uint32_t TNSB, uint32_t TNSA);
void faV3GSetTriggerPathSamples(uint32_t TNSA, uint32_t TNSAT);
int faV3SetTriggerPathThreshold(int id, uint32_t TPT);
void faV3GSetTriggerPathThreshold(uint32_t TPT);

int32_t faV3SetupADC(int id, int32_t mode);

int faV3SetPPG(int id, uint16_t *sdata, int nsamples);
void faV3PPGEnable(int id);
void faV3PPGDisable(int id);

uint32_t faV3ItrigControl(int id, uint16_t itrig_width, uint16_t itrig_dt);

int faV3ReadBlock(int id, volatile uint32_t * data, int nwrds, int rflag);
int faV3GetBlockError(int pflag);
int faV3PrintBlock(int id);

uint32_t faV3ReadCSR(int id);
void faV3Clear(int id);
void faV3GClear();
void faV3ClearError(int id);
void faV3GClearError();

void faV3Reset(int id, int iFlag);
void faV3GReset(int iFlag);
void faV3SoftReset(int id, int cflag);
void faV3ResetToken(int id);
int faV3TokenStatus(int id);
int faV3GTokenStatus();
uint32_t faV3GetTokenStatus(int pflag);

int faV3SetChanDisableMask(int id, uint16_t channel);
uint32_t faV3GetChanDisableMask(int id);

void faV3SetCompression(int id, int opt);
int faV3GetCompression(int id);
void faV3SetVXSReadout(int id, int opt);
void faV3GSetVXSReadout(int opt);
int faV3GetVXSReadout(int id);

void faV3EnableSyncSrc(int id);
void faV3GEnableSyncSrc();

void faV3Enable(int id, int eflag);
void faV3GEnable(int eflag);
void faV3Disable(int id, int eflag);
void faV3GDisable(int eflag);

void faV3Trig(int id);
void faV3GTrig();
void faV3Trig2(int id);
void faV3GTrig2();
int faV3SetTrig21Delay(int id, int delay);
int faV3GetTrig21Delay(int id);
int faV3EnableInternalPlaybackTrigger(int id);
void faV3Sync(int id);
int faV3Dready(int id, int dflag);
int faV3Bready(int id);
uint32_t faV3GBready();
uint32_t faV3GBlockReady(uint32_t slotmask, int nloop);
uint32_t faV3ScanMask();
int faV3BusyLevel(int id, uint32_t val, int bflag);
int faV3Busy(int id);
void faV3EnableSoftTrig(int id);
void faV3DisableSoftTrig(int id);
void faV3EnableSoftSync(int id);
void faV3DisableSoftSync(int id);

void faV3EnableClk(int id);
void faV3DisableClk(int id);
void faV3EnableTriggerOut(int id, int output);
void faV3EnableBusError(int id);
void faV3GEnableBusError();
void faV3DisableBusError(int id);
void faV3EnableMultiBlock(int tflag);
void faV3DisableMultiBlock();
int faV3SetBlockLevel(int id, int level);
void faV3GSetBlockLevel(int level);

int faV3SetClkSource(int id, int source);
int faV3SetTrigSource(int id, int source);
int faV3SetSyncSource(int id, int source);
void faV3EnableFP(int id);
int faV3SetTrigOut(int id, int trigout);

uint32_t faV3GetTriggerCount(int id);
int faV3ResetTriggerCount(int id);

int faV3SetChannelThreshold(int id, int chan, uint16_t tvalue);
int faV3SetThresholdAll(int id, uint16_t tvalue[16]);
int faV3GetChannelThreshold(int id, int chan);
int faV3PrintThreshold(int id);


int32_t faV3DACInit(int id);
int32_t faV3DACClear(int id);
int32_t faV3DACStatus(int id);
int32_t faV3DACSet(int id, int chan, uint32_t dac_value);
int32_t faV3DACGet(int id, int chan, uint32_t *dac_value);

int faV3SetChannelPedestal(int id, uint32_t chan, uint32_t ped);
int faV3GetChannelPedestal(int id, uint32_t chan);
int faV3SetPedestal(int id, uint32_t wvalue);
int faV3PrintPedestal(int id);

/* scaler routine prototypes */
int faV3ReadScalers(int id, volatile uint32_t * data, uint32_t chmask,
		  int rflag);
int faV3PrintScalers(int id, int rflag);
int faV3ClearScalers(int id);
int faV3LatchScalers(int id);
int faV3EnableScalers(int id);
int faV3DisableScalers(int id);

uint32_t faV3GetA32(int id);
uint32_t faV3GetA32M();
uint32_t faV3GetMinA32MB(int id);
uint32_t faV3GetMaxA32MB(int id);

void faV3PrintFifoStatus(int id);
int faV3Live(int id, int sflag);
void faV3DataDecode(uint32_t data);

void faV3TestSetSystemTestMode(int id, int mode);
void faV3TestSetTrigOut(int id, int mode);
void faV3TestSetBusyOut(int id, int mode);
void faV3TestSetSdLink(int id, int mode);
void faV3TestSetTokenOut(int id, int mode);
int faV3TestGetStatBitB(int id);
int faV3TestGetTokenIn(int id);
int faV3TestGetClock250CounterStatus(int id);
uint32_t faV3TestGetClock250Counter(int id);
uint32_t faV3TestGetSyncCounter(int id);
uint32_t faV3TestGetTrig1Counter(int id);
uint32_t faV3TestGetTrig2Counter(int id);
void faV3TestResetClock250Counter(int id);
void faV3TestResetSyncCounter(int id);
void faV3TestResetTrig1Counter(int id);
void faV3TestResetTrig2Counter(int id);
uint32_t faV3TestGetTestBitReg(int id);
int faV3TestSystemClock(int id, int pflag);

int faV3GetSerialNumber(int id, char **rval, int snfix);
int faV3SetScalerBlockInterval(int id, uint32_t nblock);
int faV3GetScalerBlockInterval(int id);
int faV3ForceEndOfBlock(int id, int scalers);
void faV3GForceEndOfBlock(int scalers);

/* FLASH SDC prototypes */
int faV3SDC_Config(uint16_t cFlag, uint16_t bMask);
void faV3SDC_Status(int sFlag);
void faV3SDC_Enable(int nsync);
void faV3SDC_Disable();
void faV3SDC_Sync();
void faV3SDC_Trig();
int faV3SDC_Busy();

typedef struct
{
  double avg;
  double rms;
  double min;
  double max;
} faV3Ped;

int faV3MeasureChannelPedestal(int id, uint32_t chan, faV3Ped *ped);
