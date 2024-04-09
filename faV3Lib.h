#pragma once
/******************************************************************************
 *
 *  fadcLib.h  - Driver library header file for readout of the JLAB 250MHz FLASH
 *                ADC using a VxWorks 5.5 or later based single board
 *                computer.
 *
 *  Author: David Abbott
 *          Jefferson Lab Data Acquisition Group
 *          June 2007
 *
 *  Revision  1.0 - Initial Revision
 *                    - Supports up to 20 FADC boards in a Crate
 *                    - Programmed I/O and Block reads
 *
 *  Revision  1.1 - Updated for new firmware (0x0111 or later)
 *                  Supports FADC Signal Distribution Module
 *
 *  Revision  1.2 - Updated to support Internal (HITSUM) triggering
 *                  and Moller firmware revisions of the FADC
 *
 *  Revision  2.0 - Updated to support FADC V2.
 *
 */


#include <stdint.h>

#define CLAS12

#define FAV3_BOARD_ID       0xfadc0000
#define FAV3_MAX_BOARDS             20
#define FAV3_MAX_ADC_CHANNELS       16
#define FAV3_MAX_DATA_PER_CHANNEL  251
#define FAV3_MAX_A32_MEM      0x800000	/* 8 Meg */
#define FAV3_MAX_A32MB_SIZE   0x800000	/*  8 MB */
#define FAV3_VME_INT_LEVEL           3
#define FAV3_VME_INT_VEC          0xFA

#define FAV3_SUPPORTED_CTRL_FIRMWARE 0xBB
#define FAV3_SUPPORTED_CTRL_FIRMWARE_NUMBER 1
#define FAV3_SUPPORTED_PROC_FIRMWARE 0x1B07
#define FAV3_SUPPORTED_PROC_FIRMWARE_NUMBER 1


typedef struct faV3_struct
{
  /* 0x0000 */ volatile uint32_t version;
  /* 0x0004 */ volatile uint32_t csr;
  /* 0x0008 */ volatile uint32_t ctrl1;
  /* 0x000C */ volatile uint32_t ctrl2;
  /* 0x0010 */ volatile uint32_t blk_level;
  /* 0x0014 */ volatile uint32_t intr;
  /* 0x0018 */ volatile uint32_t adr32;
  /* 0x001C */ volatile uint32_t adr_mb;
  /* 0x0020 */ volatile uint32_t s_adr;
  /* 0x0024 */ volatile uint32_t delay;
  /* 0x0028 */ volatile uint32_t itrig_cfg;
  /* 0x002C */ volatile uint32_t reset;
  /* 0x0030 */ volatile uint32_t trig_scal;
  /* 0x0034 */ volatile uint32_t ev_count;
  /* 0x0038 */ volatile uint32_t blk_count;
  /* 0x003C */ volatile uint32_t blk_fifo_count;
  /* 0x0040 */ volatile uint32_t blk_wrdcnt_fifo;
  /* 0x0044 */ volatile uint32_t internal_trig_scal;
  /* 0x0048 */ volatile uint32_t ram_word_count;
  /* 0x004C */ volatile uint32_t dataflow_status;
  /* 0x0050 */ volatile uint16_t dac[16];
  /* 0x0070 */ volatile uint32_t status[4];
  /* 0x0080 */ volatile uint32_t aux;
  /* 0x0084 */ volatile uint32_t trigger_control;
  /* 0x0088 */ volatile uint32_t trig21_delay;
  /* 0x008C */ volatile uint32_t mem_adr;
  /* 0x0090 */ volatile uint32_t mem1_data;
  /* 0x0094 */ volatile uint32_t mem2_data;
  /* 0x0098 */ volatile uint32_t prom_reg1;
  /* 0x009C */ volatile uint32_t prom_reg2;
  /* 0x00A0 */ volatile uint32_t berr_module_scal;
  /* 0x00A4 */ volatile uint32_t berr_crate_scal;
  /* 0x00A8 */ volatile uint32_t proc_words_scal;
  /* 0x00AC */ volatile uint32_t aux_scal2;
  /* 0x00B0 */ volatile uint32_t header_scal;
  /* 0x00B4 */ volatile uint32_t trig2_scal;
  /* 0x00B8 */ volatile uint32_t trailer_scal;
  /* 0x00BC */ volatile uint32_t syncreset_scal;
  /* 0x00C0 */ volatile uint32_t busy_level;
  /* 0x00C4 */ volatile uint32_t gen_evt_header;
  /* 0x00C8 */ volatile uint32_t gen_evt_data;
  /* 0x00CC */ volatile uint32_t gen_evt_trailer;
  /* 0x00D0 */ volatile uint32_t mgt_status;
  /* 0x00D4 */ volatile uint32_t mgt_ctrl;
  /* 0x00D8 */ volatile uint32_t reserved_ctrl[2];
  /* 0x00E0 */ volatile uint32_t scaler_ctrl;
  /* 0x00E4 */ volatile uint32_t serial_number[3];
  /* 0x00F0 */ volatile uint32_t scaler_interval;
  /* 0x00F4 */ volatile uint32_t spare_ctrl[(0xFC - 0xF4) >> 2];
  /* 0x00FC */ volatile uint32_t system_monitor;

  /* 0x0100 */ volatile uint32_t adc_status[3];
  /* 0x010C */ volatile uint32_t adc_config[4];
  /* 0x011C */ volatile uint32_t adc_ptw;
  /* 0x0120 */ volatile uint32_t adc_pl;
  /* 0x0124 */ volatile uint32_t adc_nsb;
  /* 0x0128 */ volatile uint32_t adc_nsa;
  /* 0x012C */ volatile uint16_t adc_thres[16];
  /* 0x014C */ volatile uint32_t ptw_last_adr;
  /* 0x0150 */ volatile uint32_t ptw_max_buf;
  /* 0x0154 */ volatile uint32_t adc_test_data;

#ifdef CLAS12
  /* 0x0158 */ volatile uint16_t adc_pedestal[16];
  /* 0x0178 */ volatile uint16_t adc_delay[16];
  /* 0x0198 */ volatile uint32_t adc_gain[16];
  /* 0x01d8 */ volatile uint32_t hitbit_trig_mask;
  /* 0x01dc */ volatile uint32_t hitbit_trig_width;
  /* 0x01e0 */ volatile uint32_t hitbit_cfg;
  /* 0x01e4 */ volatile uint32_t spare_adc[(0x200 - 0x1e4) >> 2];

  /* 0x0200 */ volatile uint16_t la_ctrl;
  /* 0x0202 */ volatile uint16_t la_status;
  /* 0x0204 */ volatile uint16_t adc_scaler_ctrl;
  /* 0x0206 */ volatile uint16_t spare_adc0[(0x220 - 0x206) >> 1];
  /* 0x0220 */ volatile uint16_t la_cmp_mode0[16];
  /* 0x0240 */ volatile uint16_t la_cmp_thr0[16];
  /* 0x0260 */ volatile uint16_t adc_accumulator0[16];
  /* 0x0280 */ volatile uint16_t adc_accumulator1[16];
  /* 0x02A0 */ volatile uint16_t adc_accumulator2[16];
  /* 0x02C0 */ volatile uint16_t la_data[13];
  /* 0x02DA */ volatile uint16_t spare_adc1[(0x300 - 0x2DA) >> 1];
#else
  /* 0x0158 */ volatile uint32_t adc_pedestal[16];
  /* 0x0198 */ volatile uint32_t spare_adc[(0x200 - 0x198) >> 2];

  /* 0x0200 */ volatile uint32_t hitsum_status;
  /* 0x0204 */ volatile uint32_t hitsum_cfg;
  /* 0x0208 */ volatile uint32_t hitsum_hit_width;
  /* 0x020C */ volatile uint32_t hitsum_trig_delay;
  /* 0x0210 */ volatile uint32_t hitsum_trig_width;
  /* 0x0214 */ volatile uint32_t hitsum_window_bits;
  /* 0x0218 */ volatile uint32_t hitsum_window_width;
  /* 0x021C */ volatile uint32_t hitsum_coin_bits;
  /* 0x0220 */ volatile uint32_t hitsum_pattern;
  /* 0x0224 */ volatile uint32_t hitsum_fifo;
  /* 0x0228 */ volatile uint32_t hitsum_sum_thresh;
  /* 0x022C */ volatile uint32_t spare_hitsum[(0x300 - 0x22C) >> 2];
#endif


  /* 0x0300 */ volatile uint32_t scaler[16];
  /* 0x0340 */ volatile uint32_t time_count;
  /* 0x0344 */ volatile uint32_t spare_scaler[(0x400 - 0x344) >> 2];

  /* 0x0400 */ volatile uint32_t testBit;
  /* 0x0404 */ volatile uint32_t clock250count;
  /* 0x0408 */ volatile uint32_t syncp0count;
  /* 0x040C */ volatile uint32_t trig1p0count;
  /* 0x0410 */ volatile uint32_t trig2p0count;

#ifdef CLAS12
  /* 0x0414 */ volatile uint32_t spare_adc_1[(0x500 - 0x414) >> 2];
  /* 0x0500 */ volatile uint32_t gtx_ctrl;
  /* 0x0504 */ volatile uint32_t /*spare_gtx1 */ state_csr;	/* for Ed */
  /* 0x0508 */ volatile uint32_t /*trx_ctrl */ state_value;	/* for Ed */
  /* 0x050C */ volatile uint32_t spare_gtx3;
  /* 0x0510 */ volatile uint32_t gtx_status;
  /* 0x0514 */ volatile uint32_t spare_gtx4;
  /* 0x0518 */ volatile uint32_t gtx_la_ctrl;
  /* 0x051C */ volatile uint32_t gtx_la_status;

  /* 0x0520 */ volatile uint32_t sparse_control;
  /* 0x0524 */ volatile uint32_t sparse_status;
  /* 0x0528 */ volatile uint32_t first_trigger_mismatch;
  /* 0x052C */ volatile uint32_t trigger_mismatch_counter;
  /* 0x0530 */ volatile uint32_t triggers_processed;
  /* 0x0534 */ volatile uint32_t spare;

  /* 0x0538 */ volatile uint32_t gtx_la_val[2];
  /* 0x0540 */ volatile uint32_t spare_gtx[(0x580 - 0x540) >> 2];
  /* 0x0580 */ volatile uint32_t hist_ctrl;
  /* 0x0584 */ volatile uint32_t hist_time;
  /* 0x0588 */ volatile uint32_t hist_data[16];
#endif

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
  uint32_t evt_num_2;
  uint32_t time_now;
  uint32_t time_1;
  uint32_t time_2;
  uint32_t time_3;
  uint32_t time_4;
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
  uint32_t integral;
  uint32_t time;
  uint32_t chan_a;
  uint32_t source_a;
  uint32_t chan_b;
  uint32_t source_b;
  uint32_t group;
  uint32_t time_coarse;
  uint32_t time_fine;
  uint32_t vmin;
  uint32_t vpeak;
  uint32_t trig_type_int;	/* next 4 for internal trigger data */
  uint32_t trig_state_int;	/* e.g. helicity */
  uint32_t evt_num_int;
  uint32_t err_status_int;
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
#define FAV3_CSR_EVENT_AVAILABLE                    0x1
#define FAV3_CSR_BLOCK_LEVEL_FLAG                   0x2
#define FAV3_CSR_BLOCK_READY                        0x4
#define FAV3_CSR_BERR_STATUS                        0x8
#define FAV3_CSR_TOKEN_STATUS                      0x10
#define FAV3_CSR_FIFO1_EMPTY                      0x800
#define FAV3_CSR_FIFO1_ALMOST_EMPTY              0x1000
#define FAV3_CSR_FIFO1_HALF_FULL                 0x2000
#define FAV3_CSR_FIFO1_ALMOST_FULL               0x4000
#define FAV3_CSR_FIFO1_FULL                      0x8000
#define FAV3_CSR_SOFT_PULSE_TRIG2              0x100000
#define FAV3_CSR_SOFT_CLEAR                    0x200000
#define FAV3_CSR_DATA_STREAM_SCALERS           0x400000
#define FAV3_CSR_FORCE_EOB_INSERT              0x800000
#define FAV3_CSR_FORCE_EOB_SUCCESS            0x1000000
#define FAV3_CSR_FORCE_EOB_FAILED             0x2000000
#define FAV3_CSR_ERROR_MASK                  0x0c000000
#define FAV3_CSR_ERROR_CLEAR                 0x08000000
#define FAV3_CSR_SYNC                        0x10000000
#define FAV3_CSR_TRIGGER                     0x20000000
#define FAV3_CSR_SOFT_RESET                  0x40000000
#define FAV3_CSR_HARD_RESET                  0x80000000

/* Define Init Flag bits for Clock Source */
#define FAV3_SOURCE_INT         0x00
#define FAV3_SOURCE_SDC         0x10
#define FAV3_SOURCE_VXS         0x20
#define FAV3_SOURCE_MASK        0x30

/* Define Control2 Bits */
#define FAV3_CTRL_GO               0x01
#define FAV3_CTRL_ENABLE_TRIG      0x02
#define FAV3_CTRL_ENABLE_SRESET    0x04
#define FAV3_CTRL_ENABLE_INT_TRIG  0x08
#define FAV3_CTRL_ENABLE_MASK      0x07
#define FAV3_CTRL_ENABLED          0x07

#define FAV3_CTRL_COMPRESS_DISABLE 0x000
#define FAV3_CTRL_COMPRESS_VERIFY  0x240
#define FAV3_CTRL_COMPRESS_ENABLE  0x280
#define FAV3_CTRL_COMPRESS_MASK    0x2C0

#define FAV3_CTRL_VXS_RO_ENABLE    0x400

/* Define CTRL1 Bits */
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

/* Define Control 2 Bits */
#define FAV3_GO                         0x1
#define FAV3_ENABLE_EXT_TRIGGER         0x2
#define FAV3_ENABLE_EXT_SRESET          0x4
#define FAV3_ENABLE_INT_TRIGGER         0x8
#define FAV3_ENABLE_STREAM_MODE        0x10

#define FAV3_DEBUG_XFER_INFIFO_BFIFO       0x1000
#define FAV3_DEBUG_XFER_BFIFO_OFIFO        0x2000

/* Define Reset Bits */
#define FAV3_RESET_CNTL_FPGA              0x1
#define FAV3_RESET_ADC_FPGA1              0x2
#define FAV3_RESET_ADC_FIFO1            0x100
#define FAV3_RESET_DAC                  0x800
#define FAV3_RESET_EXT_RAM_PT          0x1000
#define FAV3_RESET_TOKEN              0x10000
#define FAV3_RESET_ALL                 0xFFFF

/* Define trig_scal bits */
#define FAV3_TRIG_SCAL_RESET          (1<<31)

/* Define trigger_control bits */
#define FAV3_TRIGCTL_TRIGSTOP_EN      (1<<31)
#define FAV3_TRIGCTL_MAX2_MASK     0x00FF0000
#define FAV3_TRIGCTL_BUSY_EN          (1<<15)
#define FAV3_TRIGCTL_MAX1_MASK     0x000000FF

/* Define trig21 delay bits */
#define FAV3_TRIG21_DELAY_MASK     0x00000FFF

/* Define MGT Control bits */
#define FAV3_MGT_RESET                    0x0
#define FAV3_RELEASE_MGT_RESET            0x1
#define FAV3_MGT_FRONT_END_TO_CTP         0x2
#define FAV3_MGT_ENABLE_DATA_ALIGNMENT    0x4

#define FAV3_INT_ACTIVE             0x1000000
#define FAV3_INT_IACKSTAT_WAIT      0x2000000
#define FAV3_INT_IACKSTAT_ONBT      0x4000000


/* Define RAM Bits */
#define FAV3_RAM_DATA_MASK       0x000fffff
#define FAV3_RAM_FULL            0x00100000
#define FAV3_RAM_EMPTY           0x00200000
#define FAV3_RAM_ADR_INCREMENT   0x00100000

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

#define FAV3_MAX_PROC_MODE                       10
#define FAV3_ADC_PROC_MASK        0x7
#define FAV3_ADC_PROC_ENABLE      0x8
#define FAV3_ADC_PROC_MODE_WINDOW   1
#define FAV3_ADC_PROC_MODE_PEAK     2
#define FAV3_ADC_PROC_MODE_SUM      3
#define FAV3_ADC_PROC_MODE_TIME     7

/* begin from Bryan for trigger_control */
#define FAV3_SUPPORTED_MODES                   9,10
#define FAV3_SUPPORTED_NMODES                     2

extern const char *faV3_mode_names[FAV3_MAX_PROC_MODE + 1];

#define FAV3_ADC_CONFIG0_CHAN_MASK         0x0F00
#define FAV3_ADC_CONFIG0_CHAN_READ_ENABLE (1<<15)

#define FAV3_ADC_STATUS1_TRIG_RCV_DONE (1<<15)
#define FAV3_ADC_STATUS1_TRIGNUM_MASK  0x0FFF

#define FAV3_ADC_STATUS2_CHAN_DATA_MASK    0x3FFF

#define FAV3_ADC_CONFIG1_NSAT_MASK  0x0C00
#define FAV3_ADC_CONFIG1_TNSAT_MASK 0x3000

#define FAV3_ADC_CONFIG3_TPT_MASK   0x0FFF

#define FAV3_ADC_CONFIG6_MNPED_MASK   0x00003C00
#define FAV3_ADC_CONFIG6_PMAXPED_MASK 0x000003FF

#define FAV3_ADC_CONFIG7_NPED_MASK    0x00003C00
#define FAV3_ADC_CONFIG7_MAXPED_MASK  0x000003FF
/* end from Bryan for trigger_control */


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
#define FAV3_ADC_MIN_TNSAT       2
#define FAV3_ADC_MIN_TPT         0

#define FAV3_ADC_NSB_READBACK_MASK    0x0000000F
#define FAV3_ADC_NSB_NEGATIVE         (1<<3)

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

#define FAV3_ITRIG_TABLE_MODE       1
#define FAV3_ITRIG_WINDOW_MODE      2
#define FAV3_ITRIG_COIN_MODE        3
#define FAV3_ITRIG_SUM_MODE         4

#define FAV3_ITRIG_DISABLED         0x0010
#define FAV3_ITRIG_HB_P2OUT         0x0008

#define FAV3_ITRIG_MIN_WIDTH        0x01	/* 4 nsec */
#define FAV3_ITRIG_MAX_WIDTH        0x100	/* 1.024 microsec */
#define FAV3_ITRIG_MAX_DELAY        0xf00	/* 15.360 microsec */

#define FAV3_ITRIG_WINDOW_MAX_WIDTH 0x200	/* 2.048 microsec */


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

/* Define Scaler Interval Mask */
#define FAV3_SCALER_INTERVAL_MASK   0x0000FFFF

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

/* fadcBlockError values */
#define FAV3_BLOCKERROR_NO_ERROR          0
#define FAV3_BLOCKERROR_TERM_ON_WORDCOUNT 1
#define FAV3_BLOCKERROR_UNKNOWN_BUS_ERROR 2
#define FAV3_BLOCKERROR_ZERO_WORD_COUNT   3
#define FAV3_BLOCKERROR_DMADONE_ERROR     4
#define FAV3_BLOCKERROR_NTYPES            5

/* Number of regs to read in faDebugOutput */
#define FAV3_DEBUG_OUTPUT_NREGS 57

/* Function Prototypes */
int faInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag);
int faSlot(uint32_t i);
int faSetClockSource(int id, int clkSrc);
void faStatus(int id, int sflag);
void faGStatus(int sflag);
uint32_t faGetFirmwareVersions(int id, int pflag);
int faSetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
		  uint32_t NSB, uint32_t NSA, uint32_t NP, int bank);
void faGSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
		    uint32_t NSB, uint32_t NSA, uint32_t NP, int bank);
void faSetNormalMode(int id, int opt);
int faSetPPG(int id, int pmode, uint16_t * sdata, int nsamples);
void faPPGEnable(int id);
void faPPGDisable(int id);
int faReadBlock(int id, volatile uint32_t * data, int nwrds, int rflag);
int faGetBlockError(int pflag);
int faPrintBlock(int id, int rflag);
void faClear(int id);
void faClearError(int id);
uint32_t faReadCSR(int id);
void faReset(int id, int iFlag);
void faGReset(int iFlag);
void faSoftReset(int id, int cflag);
void faResetToken(int id);
int faTokenStatus(int id);
int faGTokenStatus();
uint32_t faGetTokenStatus(int pflag);
void faSetCalib(int id, uint16_t sdelay, uint16_t tdelay);
int faSetChannelDisable(int id, int channel);
void faChanDisable(int id, uint16_t cmask);
int faSetChannelDisableMask(int id, uint16_t cmask);
int faSetChannelEnable(int id, int channel);
int faSetChannelEnableMask(int id, uint16_t enMask);
int faGetChannelMask(int id, int type);
uint32_t faGetChanMask(int id);
void faEnableSyncReset(int id);
void faEnable(int id, int eflag, int bank);
void faGEnable(int eflag, int bank);
void faDisable(int id, int eflag);
void faGDisable(int eflag);
void faTrig(int id);
void faGTrig();
void faTrig2(int id);
void faGTrig2();
int faSetTrig21Delay(int id, int delay);
int faGetTrig21Delay(int id);
int faEnableInternalPlaybackTrigger(int id);
void faSync(int id);
int faDready(int id, int dflag);
int faBready(int id);
uint32_t faGBready();
uint32_t faGBlockReady(uint32_t slotmask, int nloop);
uint32_t faScanMask();
int faBusyLevel(int id, uint32_t val, int bflag);
int faBusy(int id);
void faEnableSoftTrig(int id);
void faDisableSoftTrig(int id);
void faEnableSoftSync(int id);
void faDisableSoftSync(int id);
void faEnableClk(int id);
void faDisableClk(int id);
void faEnableTriggerOut(int id, int output);
void faEnableBusError(int id);
void faDisableBusError(int id);
void faEnableMultiBlock(int tflag);
void faDisableMultiBlock();
int faSetBlockLevel(int id, int level);
void faGSetBlockLevel(int level);
int faSetClkSource(int id, int source);
int faSetTrigSource(int id, int source);
int faSetSyncSource(int id, int source);
void faEnableFP(int id);
int faSetTrigOut(int id, int trigout);
uint32_t faGetTriggerCount(int id);
int faResetTriggerCount(int id);
int faSetThreshold(int id, uint16_t tvalue, uint16_t chmask);
int faPrintThreshold(int id);
int faSetDAC(int id, uint16_t dvalue, uint16_t chmask);
void faPrintDAC(int id);
uint32_t faGetChannelDAC(int id, uint32_t chan);
int faSetChannelPedestal(int id, uint32_t chan, uint32_t ped);
int faGetChannelPedestal(int id, uint32_t chan);
int faLive(int id, int sflag);
void faDataDecode(uint32_t data);

int faSetMGTSettings(int id, int txpre, int txswing, int rxequ);
int faSetMGTTestMode(int id, uint32_t mode);
int faSyncResetMode(int id, uint32_t mode);
/* FADC scaler routine prototypes */
int faReadScalers(int id, volatile uint32_t * data, uint32_t chmask,
		  int rflag);
int faPrintScalers(int id, int rflag);
int faClearScalers(int id);
int faLatchScalers(int id);
int faEnableScalers(int id);
int faDisableScalers(int id);

uint32_t faGetA32(int id);
uint32_t faGetA32M();
uint32_t faGetMinA32MB(int id);
uint32_t faGetMaxA32MB(int id);

/* FADC Internal Trigger Routine prototypes */
int faItrigBurstConfig(int id, uint32_t ntrig,
		       uint32_t burst_window, uint32_t busy_period);
uint32_t faItrigControl(int id, uint16_t itrig_width, uint16_t itrig_dt);
uint32_t faItrigStatus(int id, int sFlag);
int faItrigSetMode(int id, int tmode, uint32_t wMask, uint32_t wWidth,
		   uint32_t cMask, uint32_t sumThresh, uint32_t * tTable);
int faItrigInitTable(int id, uint32_t * table);
int faItrigSetHBwidth(int id, uint16_t hbWidth, uint16_t hbMask);
uint32_t faItrigGetHBwidth(int id, uint32_t chan);
void faItrigPrintHBwidth(int id);
uint32_t faItrigOutConfig(int id, uint16_t itrigDelay, uint16_t itrigWidth);
void faItrigEnable(int id);
void faItrigDisable(int id);
int faItrigGetTableVal(int id, uint16_t pMask);

/* FADC Firmware Tools Prototypes */
int fadcFirmwareLoad(int id, int chip, int pFlag);
int fadcFirmwareGLoad(int chip, int pFlag);
void fadcFirmwareDownloadConfigData(int id);
int fadcFirmwareVerifyDownload(int id);
int fadcFirmwareTestReady(int id, int n_try, int pFlag);
int fadcFirmwareZeroSRAM(int id);
int fadcFirmwareCheckSRAM(int id);
void fadcFirmwareSetFilename(char *filename, int chip);
int fadcFirmwareReadFile(char *filename);
int fadcFirmwareGetFpgaID(int pflag);
int fadcFirmwareChipFromFpgaID(int pflag);
int fadcFirmwareRevFromFpgaID(int pflag);
int fadcFirmwareReadMcsFile(char *filename);
enum faV3Args_enum
  {
    FAV3_ARGS_SHOW_ID,
    FAV3_ARGS_SHOW_PROGRESS,
    FAV3_ARGS_SHOW_DONE,
    FAV3_ARGS_SHOW_STRING,
    FAV3_ARGS_LAST
  };

typedef struct faV3UpdateWatcherArgs_struct
{
  int step;			/* 0: show id, 1: show progress, 2: show complete */
  int id;			/* slot id */
  char title[80];
} faV3UpdateWatcherArgs_t;
int fadcFirmwareAttachUpdateWatcher(VOIDFUNCPTR routine,
				    faV3UpdateWatcherArgs_t arg);
void fadcFirmwareUpdateWatcher(faV3UpdateWatcherArgs_t arg);

void faTestSetSystemTestMode(int id, int mode);
void faTestSetTrigOut(int id, int mode);
void faTestSetBusyOut(int id, int mode);
void faTestSetSdLink(int id, int mode);
void faTestSetTokenOut(int id, int mode);
int faTestGetStatBitB(int id);
int faTestGetTokenIn(int id);
int faTestGetClock250CounterStatus(int id);
uint32_t faTestGetClock250Counter(int id);
uint32_t faTestGetSyncCounter(int id);
uint32_t faTestGetTrig1Counter(int id);
uint32_t faTestGetTrig2Counter(int id);
void faTestResetClock250Counter(int id);
void faTestResetSyncCounter(int id);
void faTestResetTrig1Counter(int id);
void faTestResetTrig2Counter(int id);
uint32_t faTestGetTestBitReg(int id);
int faTestSystemClock(int id, int pflag);

int faGetSerialNumber(int id, char **rval, int snfix);
int faSetScalerBlockInterval(int id, uint32_t nblock);
int faGetScalerBlockInterval(int id);
int faForceEndOfBlock(int id, int scalers);
void faGForceEndOfBlock(int scalers);

/* FLASH SDC prototypes */
int faSDC_Config(uint16_t cFlag, uint16_t bMask);
void faSDC_Status(int sFlag);
void faSDC_Enable(int nsync);
void faSDC_Disable();
void faSDC_Sync();
void faSDC_Trig();
int faSDC_Busy();


/*sergey*/
int faGetProcMode(int id, int *pmode, uint32_t * PL, uint32_t * PTW,
		  uint32_t * NSB, uint32_t * NSA, uint32_t * NP);
int faGetNfadc();
int faId(uint32_t slot);
int faSetThresholdAll(int id, uint16_t tvalue[16]);
int faSetPedestal(int id, uint32_t wvalue);
int faPrintPedestal(int id);
#ifdef CLAS12
int faSetTriggerProcessingMode(int id, uint32_t chan, uint32_t mask);
int faGetTriggerProcessingMode(int id, uint32_t chan);
int faSetChannelDelay(int id, uint32_t chan, uint32_t delay);
int faGetChannelDelay(int id, uint32_t chan);
int faInvert(int id, uint16_t chmask);
uint32_t faGetInvertMask(int id);
int faResetMGT(int id, int reset);
int faGetMGTChannelStatus(int id);
int faSetChannelGain(int id, uint32_t chan, float gain);
float faGetChannelGain(int id, uint32_t chan);
int faGLoadChannelPedestals(char *fname, int updateThresholds);
int faThresholdIgnore(int id, uint16_t chmask);
uint32_t faGetThresholdIgnoreMask(int id);
int faPlaybackDisable(int id, uint16_t chmask);
uint32_t faGetPlaybackDisableMask(int id);
int faSetHitbitTrigMask(int id, uint16_t chmask);
uint32_t faGetHitbitTrigMask(int id);
int faSetHitbitTrigWidth(int id, uint16_t width);
uint32_t faGetHitbitTrigWidth(int id);

int faGSetHitbitMinTOT(uint16_t width);
int faGSetHitbitMinMultiplicity(uint16_t mult);
int faSetHitbitMinTOT(int id, uint16_t width);
int faSetHitbitMinMultiplicity(int id, uint16_t mult);
int faGetHitbitMinTOT(int id);
int faGetHitbitMinMultiplicity(int id);

/*Andrea*/
int faSetDelayAll(int id, uint32_t delay);
int faSetGlobalDelay(uint32_t delay);

void faSetCompression(int id, int opt);
int faGetCompression(int id);

void faSetVXSReadout(int id, int opt);
void faGSetVXSReadout(int opt);
int faGetVXSReadout(int id);

typedef struct
{
  double avg;
  double rms;
  double min;
  double max;
} fa250Ped;

int faMeasureChannelPedestal(int id, uint32_t chan, fa250Ped * ped);

int faGetChThreshold(int id, int ch);
int faSetChThreshold(int id, int ch, int threshold);

void faSetA32BaseAddress(uint32_t addr);


#endif

int faCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np);
int faSetTriggerStopCondition(int id, int trigger_max);
int faSetTriggerBusyCondition(int id, int trigger_max);
int faSetTriggerPathSamples(int id, uint32_t TNSA, uint32_t TNSAT);
void faGSetTriggerPathSamples(uint32_t TNSA, uint32_t TNSAT);
int faSetTriggerPathThreshold(int id, uint32_t TPT);
void faGSetTriggerPathThreshold(uint32_t TPT);

/* for Ed */
int faArmStatesStorage(int id);
int faDisarmStatesStorage(int id);
int faReadStatesStorage(int id);

/* Sparsification Routines for NPS */
int faSetSparsificationMode(int id, int mode);
void faGSetSparsificationMode(int mode);
int faGetSparsificationMode(int id);
int faGetSparsificationStatus(int id);
int faClearSparsificationStatus(int id);
void faGClearSparsificationStatus();
uint32_t faGetFirstTriggerMismatch(int id);
uint32_t faGetMismatchTriggerCount(int id);
uint32_t faGetTriggersProcessedCount(int id);

int faSetAccumulatorScalerMode(int id, uint16_t chmask);
int faGSetAccumulatorScalerMode(uint16_t chmask);
uint32_t faGetAccumulatorScalerMode(int id);
