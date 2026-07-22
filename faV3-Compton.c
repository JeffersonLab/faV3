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
 * @file      faV3-Compton.c
 *
 * @brief     Library to support the Hall A Compton Polarimeter Firmware
 *
 */


#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-Compton.h"

extern pthread_mutex_t faV3Mutex;

#define FAV3LOCK      if(pthread_mutex_lock(&faV3Mutex)<0) perror("pthread_mutex_lock");
#define FAV3UNLOCK    if(pthread_mutex_unlock(&faV3Mutex)<0) perror("pthread_mutex_unlock");

extern int nfaV3;
extern int faV3ID[FAV3_MAX_BOARDS];
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];	/* pointers to FAV3 memory map */
extern uint16_t faV3ChanDisableMask[(FAV3_MAX_BOARDS + 1)];
extern int faV3FwRev[(FAV3_MAX_BOARDS + 1)][FAV3_FW_FUNCTION_MAX];
volatile faV3_compton_adc_t *COMPTONp[(FAV3_MAX_BOARDS + 1)];

#define CHECKID	{							\
    if(id == 0) id = faV3ID[0];						\
    if((id <= 0) || (id > 21) || (COMPTONp[id] == NULL)) {		\
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__, id); \
      return ERROR; }}

const char *fa_compton_mode_names[FAV3_MAX_PROC_MODE+1] =
  {
    "NOT DEFINED", // 0
    "RAW WINDOW", // 1
    "NOT DEFINED",
    "NOT DEFINED",
    "NOT DEFINED",
    "NOT DEFINED", // 5
    "NOT DEFINED",
    "NOT DEFINED",
    "NOT DEFINED",
    "PULSE PARAMETER",      // 9
    "RAW + PULSE PARAMETER" // 10
  };

int
faV3ComptonInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag)
{
  int32_t rval = OK;

  rval = faV3Init(addr, addr_inc, nadc, iFlag);

  if(rval <= 0)
    return ERROR;

  /* Check Firmware Versions and Map the Hall D pointer */
  int32_t ifa;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      if((iFlag & FAV3_INIT_SKIP_FIRMWARE_CHECK) == 0)
	{
	  if(faV3FwRev[faV3Slot(ifa)][FAV3_FW_CTRL] != FAV3_COMPTON_SUPPORTED_CTRL_FIRMWARE)
	    {
	      printf("%s: Slot %d control fw not compatible with Compton library\n",
		     __func__, faV3Slot(ifa));
	      continue;
	    }

	  if(faV3FwRev[faV3Slot(ifa)][FAV3_FW_PROC] != FAV3_COMPTON_SUPPORTED_PROC_FIRMWARE)
	    {
	      printf("%s: Slot %d processing fw not compatible with Compton library\n",
		     __func__, faV3Slot(ifa));
	      continue;
	    }
	}
      COMPTONp[faV3Slot(ifa)] = (faV3_compton_adc_t *)((u_long)FAV3p[faV3Slot(ifa)] + 0x100);
      printf("%s: Slot %d: CTRL 0x%x PROC 0x%x\n",
	     __func__, faV3Slot(ifa),
	     faV3FwRev[faV3Slot(ifa)][FAV3_FW_CTRL],
	     faV3FwRev[faV3Slot(ifa)][FAV3_FW_PROC]);

      /* Disable PROC syncreset */
      vmeWrite16(&COMPTONp[faV3Slot(ifa)]->config3, vmeRead16(&COMPTONp[faV3Slot(ifa)]->config3) | FAV3_SYNC_DISABLE);

    }

  return OK;
}

int32_t
faV3ComptonCheckAddresses()
{
  faV3_t baseregs;
  u_long offset = 0, expected = 0, base = 0;

  faV3_t *v3p = (faV3_t *) &baseregs;
  faV3_compton_adc_t *compton = (faV3_compton_adc_t *)((u_long)v3p + 0x100);

  base = (u_long) v3p;

  offset = ((u_long) &compton->status0) - base;
  expected = 0x100;
  if(offset != expected)
    printf("%s: ERROR: status0 not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  offset = ((u_long) &v3p->aux.idelay_control_1) - base;
  expected = 0x540;
  if(offset != expected)
    printf("%s: ERROR: idelay_control_1 not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  offset = ((u_long) &compton->status4) - base;
  expected = 0x160;
  if(offset != expected)
    printf("%s: ERROR: status4 not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  return 0;
}

void
faV3ComptonGStatus(int sflag)
{
  int ifa, id, ii;
  faV3_t st[FAV3_MAX_BOARDS + 1];
  faV3_compton_adc_t compton_st[(FAV3_MAX_BOARDS + 1)];
  uint32_t a24addr[FAV3_MAX_BOARDS + 1];
  int nsb;
  extern u_long faV3A24Offset;

  FAV3LOCK;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      a24addr[id] = (uint32_t) ((u_long) FAV3p[id] - faV3A24Offset);
      st[id].version = vmeRead32(&FAV3p[id]->version);
      st[id].adr32 = vmeRead32(&FAV3p[id]->adr32);
      st[id].adr_mb = vmeRead32(&FAV3p[id]->adr_mb);

      st[id].ctrl1 = vmeRead32(&FAV3p[id]->ctrl1);
      st[id].ctrl2 = vmeRead32(&FAV3p[id]->ctrl2);

      st[id].csr = vmeRead32(&FAV3p[id]->csr);

      st[id].sys_mon = vmeRead32(&FAV3p[id]->sys_mon);

      compton_st[id].status0 =
	vmeRead16(&COMPTONp[id]->status0) & 0xFFFF;
      compton_st[id].status1 =
	vmeRead16(&COMPTONp[id]->status1) & 0xFFFF;
      compton_st[id].status4 =
	vmeRead16(&COMPTONp[id]->status4) & 0xFFFF;

      compton_st[id].config1 =
	vmeRead16(&COMPTONp[id]->config1) & 0xFFFF;
      compton_st[id].config2 =
	vmeRead16(&COMPTONp[id]->config2) & 0xFFFF;
      compton_st[id].config3 =
	vmeRead16(&COMPTONp[id]->config3) & 0xFFFF;
      compton_st[id].config6 =
	vmeRead16(&COMPTONp[id]->config6) & 0xFFFF;
      compton_st[id].config8 =
	vmeRead16(&COMPTONp[id]->config8) & 0xFFFF;
      compton_st[id].config9 =
	vmeRead16(&COMPTONp[id]->config9) & 0xFFFF;
      compton_st[id].config10 =
	vmeRead16(&COMPTONp[id]->config10) & 0xFFFF;
      compton_st[id].config11 =
	vmeRead16(&COMPTONp[id]->config11) & 0xFFFF;
      compton_st[id].config12 =
	vmeRead16(&COMPTONp[id]->config12) & 0xFFFF;
      compton_st[id].config13 =
	vmeRead16(&COMPTONp[id]->config13) & 0xFFFF;
      compton_st[id].config15 =
	vmeRead16(&COMPTONp[id]->config15) & 0xFFFF;
      compton_st[id].config16 =
	vmeRead16(&COMPTONp[id]->config16) & 0xFFFF;
      compton_st[id].config17 =
	vmeRead16(&COMPTONp[id]->config17) & 0xFFFF;
      compton_st[id].config18 =
	vmeRead16(&COMPTONp[id]->config18) & 0xFFFF;
      compton_st[id].config19 =
	vmeRead16(&COMPTONp[id]->config19) & 0xFFFF;
      compton_st[id].config20 =
	vmeRead16(&COMPTONp[id]->config20) & 0xFFFF;


      st[id].blk_count = vmeRead32(&FAV3p[id]->blk_count);
      st[id].blocklevel = vmeRead32(&FAV3p[id]->blocklevel);
      st[id].ram_word_count =
	vmeRead32(&FAV3p[id]->ram_word_count) & FAV3_RAM_DATA_MASK;

      st[id].trig_scal = vmeRead32(&(FAV3p[id]->trig_scal));
      st[id].trig2_scal = vmeRead32(&FAV3p[id]->trig2_scal);
      st[id].syncreset_scal = vmeRead32(&FAV3p[id]->syncreset_scal);
      st[id].aux.berr_driven_count = vmeRead32(&FAV3p[id]->aux.berr_driven_count);

      st[id].aux.sparsify_control = vmeRead32(&FAV3p[id]->aux.sparsify_control);

    }
  FAV3UNLOCK;

  printf("\n");

  printf("                      faV3 Module Configuration Summary\n\n");
  printf("     Firmware Rev   .................Addresses................\n");
  printf("Slot  Ctrl   Proc      A24        A32     A32 Multiblock Range   VXS Readout\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("0x%04x 0x%04x  ", st[id].version & 0xFFFF,
	     compton_st[id].status0 & FAV3_ADC_VERSION_MASK);

      printf("0x%06x  ", a24addr[id]);

      if(st[id].adr32 & FAV3_A32_ENABLE)
	{
	  printf("0x%08x  ", (st[id].adr32 & FAV3_A32_ADDR_MASK) << 16);
	}
      else
	{
	  printf("  Disabled  ");
	}

      if(st[id].adr_mb & FAV3_AMB_ENABLE)
	{
	  printf("0x%08x-0x%08x  ",
		 (st[id].adr_mb & FAV3_AMB_MIN_MASK) << 16,
		 (st[id].adr_mb & FAV3_AMB_MAX_MASK));
	}
      else
	{
	  printf("Disabled               ");
	}

      printf("%s",
	     (st[id].ctrl2 & FAV3_CTRL_VXS_RO_ENABLE) ? " Enabled" : "Disabled");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");


  printf("\n");
  printf("      .Signal Sources..                        ..Channel... \n");
  printf("Slot  Clk   Trig   Sync     MBlk  Token  BERR  Enabled Mask \n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("%s  ",
	     (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_INTERNAL ? " INT " : (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_P0 ? " VXS " : (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_FP ? "  FP " : " ??? ");

      printf("%s  ",
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_INTERNAL ? " INT " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_VME ? " VME " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_P0 ? " VXS " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_FP ? "  FP " : " ??? ");

      printf("%s    ",
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_VME ? " VME " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_P0 ? " VXS " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_FP ? "  FP " :
	     " ??? ");

      printf("%s   ", (st[id].ctrl1 & FAV3_ENABLE_MULTIBLOCK) ? "YES" : " NO");

      printf("%s",
	     st[id].ctrl1 & (FAV3_MB_TOKEN_VIA_P0) ? " P0" :
	     st[id].ctrl1 & (FAV3_MB_TOKEN_VIA_P2) ? " P0" : " NO");
      printf("%s  ",
	     st[id].ctrl1 & (FAV3_FIRST_BOARD) ? "-F" :
	     st[id].ctrl1 & (FAV3_LAST_BOARD) ? "-L" : "  ");

      printf("%s     ", st[id].ctrl1 & FAV3_ENABLE_BERR ? "YES" : " NO");

      printf("0x%04X        ",
	     ~(compton_st[id].config2 & FAV3_ADC_CHAN_MASK) & 0xFFFF);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("           ............faV3 Signal Scalers..........     ..System Monitor..\n");
  printf("Slot       Trig1       Trig2   SyncReset        BERR     TempC   1.0V   2.5V\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d   ", id);

      printf("%10d  ", st[id].trig_scal);

      printf("%10d  ", st[id].trig2_scal);

      printf("%10d  ", st[id].syncreset_scal);

      printf("%10d     ", st[id].aux.berr_driven_count);

      double fpga_temperature =
	(((double) (st[id].sys_mon & FAV3_SYSMON_CTRL_TEMP_MASK)) *
	 (503.975 / 1024.0)) - 273.15;
      printf("%3.1f    ", fpga_temperature);

      double fpga_1V =
	(((double)
	  ((st[id].sys_mon & FAV3_SYSMON_FPGA_CORE_V_MASK) >> 11)) *
	 (3.0 / 1024.0));
      printf("%3.1f    ", fpga_1V);

      double fpga_25V =
	(((double)
	  ((st[id].sys_mon & FAV3_SYSMON_FPGA_AUX_V_MASK) >> 22)) *
	 (3.0 / 1024.0));
      printf("%3.1f    ", fpga_25V);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                              faV3 Data Status\n\n");
  printf("      Trigger   Block                              Error Status\n");
  printf("Slot  Source    Ready  Blocks In Fifo  RAM Level   CSR     MGT\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("%s    ",
	     st[id].ctrl2 & FAV3_CTRL_ENABLE_MASK ? " Enabled" : "Disabled");

      printf("%s       ", st[id].csr & FAV3_CSR_BLOCK_READY ? "YES" : " NO");

      printf("%10d ", st[id].blk_count & FAV3_BLOCK_COUNT_MASK);

      printf("%10d  ", (st[id].ram_word_count & FAV3_RAM_DATA_MASK) * 8);

      printf("%s     ", st[id].csr & FAV3_CSR_ERROR_MASK ? "ERROR" : "  OK ");

      printf("%s  ",
	     st[id].status_mgt &
	     (FAV3_MGT_GTX1_HARD_ERROR | FAV3_MGT_GTX1_SOFT_ERROR |
	      FAV3_MGT_GTX2_HARD_ERROR | FAV3_MGT_GTX2_SOFT_ERROR) ? "ERROR" : "  OK " );

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                       faV3 Compton Configuration\n\n");

  printf("      Start   Stop      Self-Trigger  1stLoX  2ndLoX  Selected  Max\n");
  printf("Slot  Set     Set       NSB   NSA     NSB     NSA     Trigger   Trg Count\n");
  printf("--------------------------------------------------------------------------------\n");
  //       23   4095    8388607   16    4095    1023    1023    self      65535
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d   ", id);

      printf("%-4d    ",
	     compton_st[id].config6 & FAV3_START_SET_MASK);

      uint32_t stop_set = compton_st[id].config19 & FAV3_STOP_SET_LSB_MASK;
      stop_set |= (compton_st[id].config20 & FAV3_STOP_SET_MSB_MASK) << 16;

      printf("%-7d   ",
	     stop_set);

      printf("%-2d    ",
	     compton_st[id].config8 & FAV3_SELF_TRIGGER_NSB_MASK);
      printf("%-4d    ",
	     compton_st[id].config9 & FAV3_SELF_TRIGGER_NSA_MASK);

      printf("%-4d    ",
	     compton_st[id].config11 & FAV3_NSB1_LO_MASK);
      printf("%-4d    ",
	     compton_st[id].config12 & FAV3_NSA2_LO_MASK);

      printf("%s      ",
	     (compton_st[id].config3 & FAV3_TRIGGER_TI) ? " TI " : "self");

      printf("%-4d",
	     compton_st[id].config21);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                       faV3 Compton Configuration\n\n");

  printf("      ---------- Thresholds ------                        Report Results\n");
  printf("Slot  Hi      Lo      Self-Trigger  Prescale  Hysteresis  SP 0123 45\n");
  printf("--------------------------------------------------------------------------------\n");
  //       23   4095    4095    4095          1023      63          11 1111 11
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d   ", id);

      printf("%-4d    ",
	     compton_st[id].config10 & FAV3_HI_THRESHOLD_MASK);
      printf("%-4d    ",
	     compton_st[id].config15 & FAV3_LO_THRESHOLD_MASK);
      printf("%-4d          ",
	     compton_st[id].config13 & FAV3_SELF_TRIGGER_THRESHOLD_MASK);

      printf("%-4d      ",
	     compton_st[id].config16 & FAV3_SELF_TRIGGER_PRESCALE_MASK);

      printf("%-2d          ",
	     compton_st[id].config18 & FAV3_HYSTERSIS_MASK);

      printf("%d%d ",
	     (compton_st[id].config17 & (1 << 0)) ? 1 : 0,
	     (compton_st[id].config17 & (1 << 1)) ? 1 : 0);

      printf("%d%d%d%d ",
	     (compton_st[id].config17 & (1 << 2)) ? 1 : 0,
	     (compton_st[id].config17 & (1 << 3)) ? 1 : 0,
	     (compton_st[id].config17 & (1 << 4)) ? 1 : 0,
	     (compton_st[id].config17 & (1 << 5)) ? 1 : 0);

      printf("%d%d",
	     (compton_st[id].config17 & (1 << 6)) ? 1 : 0,
	     (compton_st[id].config17 & (1 << 7)) ? 1 : 0);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");


  printf("\n");
  printf("\n");
}

int32_t
faV3ComptonSetMPSStartStop(int32_t id, uint16_t start, uint32_t stop) {
  int32_t rval = OK;
  uint16_t stop_set_lsb = 0, stop_set_msb = 0;

  CHECKID;

  if((start < FAV3_START_SET_MIN) || (start > FAV3_START_SET_MASK)) {
    printf("%s: ERROR: Invalid start (%d)\n", __func__, start);
    return ERROR;
  }

  if(stop > 0x7FFFFF) {
    printf("%s: ERROR: Invalid stop (%d)\n", __func__, stop);
    return ERROR;
  }

  stop_set_lsb = stop & FAV3_STOP_SET_LSB_MASK;
  stop_set_msb = (stop >> 7) & FAV3_STOP_SET_MSB_MASK;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config6, start);
  vmeWrite16(&COMPTONp[id]->config19, stop_set_msb);
  vmeWrite16(&COMPTONp[id]->config20, stop_set_lsb);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetMPSStartStop(int32_t id, uint16_t *start, uint32_t *stop) {
  int32_t rval = OK;
  uint16_t stop_set_lsb = 0, stop_set_msb = 0;

  CHECKID;

  FAV3LOCK;
  *start = vmeRead16(&COMPTONp[id]->config6) & FAV3_START_SET_MASK;
  stop_set_msb = vmeRead16(&COMPTONp[id]->config19) & FAV3_STOP_SET_MSB_MASK;
  stop_set_lsb = vmeRead16(&COMPTONp[id]->config20) & FAV3_STOP_SET_LSB_MASK;

  *stop = (stop_set_msb << 7) | stop_set_lsb;

  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSetProc(int32_t id,
		   uint16_t st_nsb, uint16_t st_nsa,
		   uint16_t lo_threshold, uint16_t hi_threshold,
		   uint16_t st_threshold, uint16_t st_nsb_lox, uint16_t st_nsa_lox) {
  int32_t rval = OK;

  CHECKID;

  if(st_nsb > FAV3_SELF_TRIGGER_NSB_MASK) {
    printf("%s: ERROR: Invalid self trigger nsb (%d)\n", __func__, st_nsb);
    return ERROR;
  }

  if(st_nsa > FAV3_SELF_TRIGGER_NSA_MASK) {
    printf("%s: ERROR: Invalid self trigger nsa (%d)\n", __func__, st_nsa);
    return ERROR;
  }

  if(lo_threshold > FAV3_LO_THRESHOLD_MASK) {
    printf("%s: ERROR: Invalid lo_threshold (%d)\n", __func__, lo_threshold);
    return ERROR;
  }

  if(hi_threshold > FAV3_HI_THRESHOLD_MASK) {
    printf("%s: ERROR: Invalid hi_threshold (%d)\n", __func__, hi_threshold);
    return ERROR;
  }

  if(st_threshold > FAV3_SELF_TRIGGER_THRESHOLD_MASK) {
    printf("%s: ERROR: Invalid self trigger threshold (%d)\n", __func__, st_threshold);
    return ERROR;
  }

  if(st_nsb_lox > FAV3_SELF_TRIGGER_NSB_MASK) {
    printf("%s: ERROR: Invalid self trigger nsb 1st lo cross (%d)\n", __func__, st_nsb_lox);
    return ERROR;
  }

  if(st_nsa_lox > FAV3_SELF_TRIGGER_NSA_MASK) {
    printf("%s: ERROR: Invalid self trigger nsa 2nd lo cross (%d)\n", __func__, st_nsa_lox);
    return ERROR;
  }

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config8, st_nsb & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config9, st_nsa & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config15, lo_threshold & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config10, hi_threshold & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config13, st_threshold & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config11, st_nsb_lox & 0xFFFF);
  vmeWrite16(&COMPTONp[id]->config12, st_nsa_lox & 0xFFFF);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetProc(int32_t id,
		   uint16_t *st_nsb, uint16_t *st_nsa,
		   uint16_t *lo_threshold, uint16_t *hi_threshold,
		   uint16_t *st_threshold, uint16_t *st_nsb_lox, uint16_t *st_nsa_lox) {
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  *st_nsb = vmeRead16(&COMPTONp[id]->config8) & FAV3_SELF_TRIGGER_NSB_MASK;
  *st_nsa = vmeRead16(&COMPTONp[id]->config9) & FAV3_SELF_TRIGGER_NSA_MASK;
  *lo_threshold = vmeRead16(&COMPTONp[id]->config15) & FAV3_LO_THRESHOLD_MASK;
  *hi_threshold = vmeRead16(&COMPTONp[id]->config10) & FAV3_HI_THRESHOLD_MASK;
  *st_threshold = vmeRead16(&COMPTONp[id]->config13) & FAV3_SELF_TRIGGER_THRESHOLD_MASK;
  *st_nsb_lox = vmeRead16(&COMPTONp[id]->config11) & FAV3_SELF_TRIGGER_NSB_MASK;
  *st_nsa_lox = vmeRead16(&COMPTONp[id]->config12) & FAV3_SELF_TRIGGER_NSA_MASK;
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSetPrescale(int32_t id, uint16_t prescale) {
  int32_t rval = OK;

  CHECKID;

  if(prescale > FAV3_SELF_TRIGGER_PRESCALE_MASK) {
    printf("%s: ERROR: Invalid prescale (%d)\n", __func__, prescale);
    return ERROR;
  }

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config16, prescale & 0xFFFF);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetPrescale(int32_t id, uint16_t *prescale) {
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  *prescale = vmeRead16(&COMPTONp[id]->config16) & FAV3_SELF_TRIGGER_PRESCALE_MASK;
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSetHysteresis(int32_t id, uint16_t hysteresis) {
  int32_t rval = OK;

  CHECKID;

  if(hysteresis > FAV3_HYSTERSIS_MASK) {
    printf("%s: ERROR: Invalid hysteresis (%d)\n", __func__, hysteresis);
    return ERROR;
  }

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config18, hysteresis & 0xFFFF);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetHysteresis(int32_t id, uint16_t *hysteresis) {
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  *hysteresis = vmeRead16(&COMPTONp[id]->config18) & FAV3_HYSTERSIS_MASK;
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSetReportResults(int32_t id, uint16_t report) {
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config17, report);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetReportResults(int32_t id, uint16_t *report) {
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  *report = vmeRead16(&COMPTONp[id]->config17);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonPauseProcessingTrigger(int32_t id) {
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->status2, 1);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonResumeProcessingTrigger(int32_t id) {
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->status2, 0);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSelectTrigger(int32_t id, uint16_t trigger) {
  int32_t rval = OK;
  CHECKID;

  trigger = trigger ? 1 : 0;

  FAV3LOCK;

  if(trigger == 0) { // self-trigger
    vmeWrite16(&COMPTONp[id]->config3,
	       vmeRead16(&COMPTONp[id]->config3) & ~FAV3_TRIGGER_SELECT_MASK);
  } else {
    vmeWrite16(&COMPTONp[id]->config3,
	       vmeRead16(&COMPTONp[id]->config3) | FAV3_TRIGGER_TI);
  }

  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonGetSelectTrigger(int32_t id, uint16_t *trigger) {
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;

  *trigger = (vmeRead16(&COMPTONp[id]->config3) & FAV3_TRIGGER_SELECT_MASK) ? 1 : 0;

  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonSetMaxTriggerCount(int32_t id, uint16_t max_count) {
  int32_t rval = OK;
  CHECKID;

  if(max_count < 3) {
    printf("%s: ERROR: Invalid max_count (%d)\n",
	   __func__, max_count);
    return ERROR;
  }

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config21, max_count);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonCollectOn(int32_t id) {
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config1, 1);
  FAV3UNLOCK;

  return rval;
}

int32_t
faV3ComptonCollectOff(int32_t id) {
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;
  vmeWrite16(&COMPTONp[id]->config1, 0);
  FAV3UNLOCK;

  return rval;
}


FILE *fdecode_output = NULL;

void
faV3SetDecodeOutput(FILE *output_file) {fdecode_output = output_file;}

void
faV3ComptonDataDecode(unsigned int data)
{
  int i_print = 1;
  static unsigned int type_last = 15;	/* initialize to type FILLER WORD */
  static unsigned int time_last = 0;
  int idata=0;
  static faV3ComptonData_t faV3_data = {0};

  if(fdecode_output == NULL)
    fdecode_output = stdout;


  if( data & 0x80000000 )		/* data type defining word */
    {
      faV3_data.new_type = 1;
      faV3_data.type = (data & 0x78000000) >> 27;
    }
  else
    {
      faV3_data.new_type = 0;
      faV3_data.type = type_last;
    }

  fprintf(fdecode_output, "%8X - ", data);
  switch( faV3_data.type )
    {
    case 0:		/* BLOCK HEADER */
      if( faV3_data.new_type )
	{
	  faV3_data.slot_id_hd = ((data) & 0x7C00000) >> 22;
	  faV3_data.modID      = (data & 0x3C0000)>>18;
	  faV3_data.blk_num    = (data & 0x3FF00) >> 8;
	  faV3_data.n_evts     = (data & 0xFF);
	  if( i_print )
	    fprintf(fdecode_output, "BLOCK HEADER - slot = %d  modID = %d   n_evts = %d   n_blk = %d\n",
		   faV3_data.slot_id_hd,
		   faV3_data.modID, faV3_data.n_evts, faV3_data.blk_num);
	}
      else
	{
	  faV3_data.PL  = (data & 0x1FFC0000) >> 18;
	  faV3_data.NSB = (data & 0x0003FE00) >> 9;
	  faV3_data.NSA = (data & 0x000001FF) >> 0;

	  fprintf(fdecode_output, "BLOCK HEADER 2 - PL = %d  NSB = %d  NSA = %d\n",
		 faV3_data.PL,
		 faV3_data.NSB,
		 faV3_data.NSA);
	}
      break;

    case 1:		/* BLOCK TRAILER */
      faV3_data.slot_id_tr = (data & 0x7C00000) >> 22;
      faV3_data.n_words = (data & 0xFFF);
      if( i_print )
	fprintf(fdecode_output, "BLOCK TRAILER - slot = %d   n_words = %d\n",
	       faV3_data.slot_id_tr, faV3_data.n_words);
      break;

    case 2:		/* EVENT HEADER */
      faV3_data.time_low_10 = (data & 0x003FF000) >> 12;
      faV3_data.evt_num_1 = (data & 0xFFF);
      if( i_print )
	fprintf(fdecode_output, "EVENT HEADER 1 - trig time = %d   trig num = %d\n",
	       faV3_data.time_low_10, faV3_data.evt_num_1);
      break;

    case 3:		/* MPS RISING TIME */
      if( faV3_data.new_type )
	{
	  faV3_data.time_1 = (data & 0x07FFFFFF);
	  if( i_print )
	    fprintf(fdecode_output, "MPS RISING TIME 1 - time = %08x\n", faV3_data.time_1);
	  faV3_data.time_now = 1;
	  time_last = 1;
	}
      else
	{
	  if( time_last == 1 )
	    {
	      faV3_data.time_2 = (data & 0xFFFFFF);
	      if( i_print )
		fprintf(fdecode_output, "MPS RISING TIME 2 - time = %08x\n", faV3_data.time_2);
	      faV3_data.time_now = 2;
	    }
	  else
	    if( i_print )
	      fprintf(fdecode_output, "TRIGGER TIME - (ERROR)\n");

	  time_last = faV3_data.time_now;
	}
      break;

    case 4:		/* SELF TRIGGER RAW DATA */
      if( faV3_data.new_type )
	{
	  faV3_data.chan = (data & 0x07800000) >> 23;
	  faV3_data.nsamples = (data & 0x3FF);
	  faV3_data.hel = (data & (1 << 10)) ? 1 : 0;
	  if( i_print )
	    fprintf(fdecode_output, "SELF TRIGGER RAW DATA - chan = %d  hel = %d  nsamples = %d\n",
		   faV3_data.chan, faV3_data.hel, faV3_data.nsamples);
	}
      else
	{
	  faV3_data.pulse_start = (data & (1 << 29)) ? 1 : 0;
	  faV3_data.delta_hel =  (data & (1 << 30)) ? 1 : 0;

	  faV3_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  faV3_data.valid_1 = 1;

	  faV3_data.adc_2 = (data & 0x1FFF);
	  faV3_data.valid_2 = ( data & (1 << 13) ) ? 0 : 1;

	  if( i_print )
	    fprintf(fdecode_output, "RAW SAMPLES - s: %d  h: %d  valid = %d  adc = %4d   valid = %d  adc = %4d\n",
		   faV3_data.pulse_start, faV3_data.delta_hel,
		   faV3_data.valid_1, faV3_data.adc_1,
		   faV3_data.valid_2, faV3_data.adc_2);
	}
      break;

    case 5:		/* UNDEFINED TYPE */
    case 6:		/* UNDEFINED TYPE */
    case 7:		/* UNDEFINED TYPE */
      if( i_print )
	fprintf(fdecode_output, "UNDEFINED TYPE = %d\n", faV3_data.type);
      break;

    case 8:		/* Helcity Number at Tstart */
      if( faV3_data.new_type )
	{
	  faV3_data.hel = (data & (1 << 26)) ? 1 : 0;
	  faV3_data.tstop = (data & (1 << 25)) ? 1 : 0;
	  faV3_data.hel_number_lsb = (data & 0x01FFFFFF);

	  if( i_print )
	    fprintf(fdecode_output, "HEL# LSB - hel: %d  tstop: %d  n = 0x%x\n",
		   faV3_data.hel, faV3_data.tstop, faV3_data.hel_number_lsb);
	}
      else
	{
	  faV3_data.hel_number_msb = (data & 0x0000001F);

	  if( i_print )
	    fprintf(fdecode_output, "HEL# MSB - n = 0x%x\n",
		   faV3_data.hel_number_msb);
	}

      break;



    case 9:		/* SELF-TRIGGER PULSE PARAMETERS */
      if( faV3_data.new_type )
	{ /* Word 1: Channel ID */
	  faV3_data.pulse_num  = 0; /* Initialize */
	  faV3_data.evt_of_blk = (data & 0x07f80000) >> 19;
	  faV3_data.chan       = (data & 0x00078000) >> 15;
	  faV3_data.prescale   = (data & 0x000007FF);

	  if( i_print )
	    fprintf(fdecode_output, "PULSEPARAM 1 - evt = %d   chan = %d   prescale = %d\n",
		   faV3_data.evt_of_blk,
		   faV3_data.chan,
		   faV3_data.prescale);
	}
      else
	{
	  if(data & (1<<30))
	    { /* Word 2: VPeak, nsamples of n-th pulse */
	      faV3_data.pulse_num++;
	      faV3_data.nsamples = (data & 0x3ffff000)>>12;
	      faV3_data.over    = (data & (1<<10))>>10;
	      faV3_data.under   = (data & (1<<9))>>9;
	      faV3_data.vpeak = (data & 0x000001ff);

	      if( i_print )
		fprintf(fdecode_output, "PULSEPARAM 2 - P: %d  nsamples = %d  Ov/Un = %d/%d  vpeak = %d\n",
		       faV3_data.pulse_num,
		       faV3_data.nsamples,
		       faV3_data.over,
		       faV3_data.under,
		       faV3_data.vpeak);
	    }
	  else
	    { /* Word 3: Sum of n-th pulse in window */
	      faV3_data.missed = (data & (1 << 29)) ? 1 : 0;
	      faV3_data.nsb_nsa_overlap = (data & (1 << 28)) ? 1 : 0;
	      faV3_data.adc_sum = (data & 0x01FFFFFF);

	      if( i_print )
		fprintf(fdecode_output, "PULSEPARAM 3 - P: %d  missed = %d  overlap = %d  adc_sum = %d\n",
		       faV3_data.pulse_num,
		       faV3_data.missed,
		       faV3_data.nsb_nsa_overlap,
		       faV3_data.adc_sum);
	    }
	}

      break;

    case 10:		/* Accumulator Parameters */
      if( faV3_data.new_type )
	{ /* Word 1: Type */
	  faV3_data.acc_param_word_number = 1;

	  if( i_print )
	    fprintf(fdecode_output, "ACCPARAM %2d\n",
		   faV3_data.acc_param_word_number);
	}
      else
	{
	  switch(++faV3_data.acc_param_word_number) {

	  case 2:
	    faV3_data.over = (data & (1 << 8)) ? 1 : 0;
	    faV3_data.under = (data & (1 << 7)) ? 1 : 0;
	    faV3_data.acc_type = (data & 0x00000070) >> 4;
	    faV3_data.acc_chan = (data & 0xF);

	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - Ov/Un: %d/%d  type: %d  chan: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.over, faV3_data.under,
		     faV3_data.acc_type, faV3_data.acc_chan);
	    break;

	  case 3:
	  case 4:
	  case 5:
	  case 6:
	    faV3_data.time_1 = (data & 0x07000000);
	    faV3_data.time_now = (data & 0x00FFFFFF);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - t_c: %d  time: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.time_1, faV3_data.time_now);
	    break;

	  case 7:
	    faV3_data.nsamples = (data & 0x03FFFFFF);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - LSB nsamples: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.nsamples);
	    break;

	  case 8:
	    faV3_data.nsamples = (data & 0x7F);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - MSB nsamples: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.nsamples);
	    break;

	  case 9:
	    faV3_data.adc_sum = (data & 0x3FFFFFFF);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - LSB sum: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.adc_sum);
	    break;

	  case 10:
	    faV3_data.adc_sum = (data & 0x0000001F);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - MSB sum: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.adc_sum);
	    break;

	  case 11:
	    faV3_data.nsb_low_x_overlap = (data & 0x0FFFC000) >> 14;
	    faV3_data.no_nsa_low_x = (data & 0x00003FFF);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - NSB LowX: %d  No NSA2 LowX: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.nsb_low_x_overlap,
		     faV3_data.no_nsa_low_x);
	    break;

	  case 12:
	    faV3_data.missed = (data & 0x00003FFF);
	    if( i_print )
	      fprintf(fdecode_output, "ACCPARAM %2d - missed: %d\n",
		     faV3_data.acc_param_word_number,
		     faV3_data.missed);
	    faV3_data.acc_param_word_number = 1;
	    break;

	  }
	}
      break;

    case 11:		/* UNDEFINED TYPE */
      if( i_print )
	fprintf(fdecode_output, "UNDEFINED TYPE = %d\n", faV3_data.type);
      break;

    case 12:		/* SCALER HEADER */
      if( faV3_data.new_type )
	{
	  faV3_data.scaler_data_words = (data & 0x3F);
	  faV3_data.scaler_data_iword = 0;
	  if( i_print )
	    fprintf(fdecode_output, "SCALER HEADER - data words = %d\n", faV3_data.scaler_data_words);
	}
      else
	{
	  faV3_data.scaler_data_iword++;
	  faV3_data.scaler_data = (data);
	  if( i_print )
	    fprintf(fdecode_output, "SCALER DATA - word = %2d  counter = %d\n",
		   faV3_data.scaler_data_iword, faV3_data.scaler_data);
	}
      break;

    case 13:		/* UNDEFINED TYPE */
      if( i_print )
	fprintf(fdecode_output, "UNDEFINED TYPE = %d\n", faV3_data.type);
      break;

    case 14:		/* DATA NOT VALID (no data available) */
      if( i_print )
	fprintf(fdecode_output, "DATA NOT VALID = %d\n", faV3_data.type);
      break;

    case 15:		/* FILLER WORD */
      if( i_print )
	fprintf(fdecode_output, "FILLER WORD = %d\n", faV3_data.type);
      break;
    }

  type_last = faV3_data.type;	/* save type of current data word */
  fflush(fdecode_output);

}
