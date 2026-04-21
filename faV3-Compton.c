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
      compton_st[id].status3 =
	vmeRead16(&COMPTONp[id]->status3) & 0xFFFF;
      compton_st[id].status4 =
	vmeRead16(&COMPTONp[id]->status4) & 0xFFFF;

      compton_st[id].config1 =
	vmeRead16(&COMPTONp[id]->config1) & 0xFFFF;
      compton_st[id].config2 =
	vmeRead16(&COMPTONp[id]->config2) & 0xFFFF;
      compton_st[id].config3 =
	vmeRead16(&COMPTONp[id]->config3) & 0xFFFF;
      compton_st[id].config4 =
	vmeRead16(&COMPTONp[id]->config4) & 0xFFFF;
      compton_st[id].config5 =
	vmeRead16(&COMPTONp[id]->config5) & 0xFFFF;
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
      compton_st[id].config14 =
	vmeRead16(&COMPTONp[id]->config14) & 0xFFFF;
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
}
