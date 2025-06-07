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
 * @file      faV3-HallD.c
 *
 * @brief     Library to support the HallD-Production Firmware
 *
 */

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-HallD.h"

extern pthread_mutex_t faV3Mutex;

#define FAV3LOCK      if(pthread_mutex_lock(&faV3Mutex)<0) perror("pthread_mutex_lock");
#define FAV3UNLOCK    if(pthread_mutex_unlock(&faV3Mutex)<0) perror("pthread_mutex_unlock");

extern int nfaV3;
extern int faV3ID[FAV3_MAX_BOARDS];
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];	/* pointers to FAV3 memory map */
extern uint16_t faV3ChanDisableMask[(FAV3_MAX_BOARDS + 1)];
extern int faV3FwRev[(FAV3_MAX_BOARDS + 1)][FAV3_FW_FUNCTION_MAX];
volatile faV3_halld_adc_t *HallDp[(FAV3_MAX_BOARDS + 1)];

int faV3AlignmentDebug=0;                            /* Flag to send alignment sequence to CTP */

#define CHECKID	{							\
    if(id == 0) id = faV3ID[0];						\
    if((id <= 0) || (id > 21) || (HallDp[id] == NULL)) {			\
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__, id); \
      return ERROR; }}

const char *fa_halld_mode_names[FAV3_MAX_PROC_MODE+1] =
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
faV3HallDInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag)
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
	  if(faV3FwRev[faV3Slot(ifa)][FAV3_FW_CTRL] != FAV3_HALLD_SUPPORTED_CTRL_FIRMWARE)
	    {
	      printf("%s: Slot %d control fw not compatible with Hall D library\n",
		     __func__, faV3Slot(ifa));
	      continue;
	    }

	  if(faV3FwRev[faV3Slot(ifa)][FAV3_FW_PROC] != FAV3_HALLD_SUPPORTED_PROC_FIRMWARE)
	    {
	      printf("%s: Slot %d processing fw not compatible with Hall D library\n",
		     __func__, faV3Slot(ifa));
	      continue;
	    }
	}
      HallDp[faV3Slot(ifa)] = (faV3_halld_adc_t *)((u_long)FAV3p[faV3Slot(ifa)] + 0x100);
      printf("%s: Slot %d: CTRL 0x%x PROC 0x%x\n",
	     __func__, faV3Slot(ifa),
	     faV3FwRev[faV3Slot(ifa)][FAV3_FW_CTRL],
	     faV3FwRev[faV3Slot(ifa)][FAV3_FW_PROC]);
    }

  return OK;
}

int32_t
faV3HallDCheckAddresses()
{
  faV3_t baseregs;
  u_long offset = 0, expected = 0, base = 0;

  faV3_t *v3p = (faV3_t *) &baseregs;
  faV3_halld_adc_t *halld = (faV3_halld_adc_t *)((u_long)v3p + 0x100);

  base = (u_long) v3p;

  offset = ((u_long) &halld->status0) - base;
  expected = 0x100;
  if(offset != expected)
    printf("%s: ERROR: status0 not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  offset = ((u_long) &v3p->aux.idelay_control_1) - base;
  expected = 0x540;
  if(offset != expected)
    printf("%s: ERROR: idelay_control_1 not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  offset = ((u_long) &halld->rogue_ptw_fall_back) - base;
  expected = 0x1A4;
  if(offset != expected)
    printf("%s: ERROR: rogue_ptw_fall_back not at expected offset 0x%lx (@ 0x%lx)\n",
	   __func__,expected,offset);

  return 0;
}

/**
 *  @ingroup Config
 *  @brief Return the maximum number of unacknowledged triggers a specific
 *         mode can handle.
 *
 *  @param pmode  Processing Mode
 *  @param ptw  Window Width
 *  @param nsb  Number of samples before pulse over threshold
 *  @param nsa  Number of samples after pulse over threshold
 *  @param np   Number of pulses processed per window
 *
 *  @return The minimum of 9 and the calculated maximum number of triggers
 *    allowed given specified mode and window paramters.
 */

int
faV3HallDCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np)
{
  int max;
  int imode = 0, supported_modes[FAV3_SUPPORTED_NMODES] =
    { FAV3_SUPPORTED_MODES };
  int mode_supported = 0;

  for(imode = 0; imode < FAV3_SUPPORTED_NMODES; imode++)
    {
      if(mode == supported_modes[imode])
	mode_supported = 1;
    }
  if(!mode_supported)
    {
      printf("%s: ERROR: Processing Mode (%d) not supported\n",
	     __func__, mode);
      return ERROR;
    }

  switch (mode)
    {
    case 9:			/* PULSE PARAMETER */
      max = (int) (1024 / ((np * 2) + 8));
      break;

    case 10:			/* DEBUG */
      max = (int) (1024 / (((np * 2) + 8) + ptw + 1));
      break;

    default:
      printf("%s: ERROR: Mode %d is not supported\n", __func__, mode);
    }

  return ((max < 9) ? max : 9);
}

/**
 *  @ingroup Config
 *  @brief Configure the processing type/mode
 *
 *  @param id Slot number
 *  @param pmode  Processing Mode
 *     -     9 - Pulse Parameter Mode
 *     -    10 - Debug Mode (9 + Raw Samples)
 *  @param  PL  Window Latency
 *  @param PTW  Window Width
 *  @param NSB  If NSB > 0: Number of samples before pulse over threshold included in sum
 *                 NSB < 0: Number of samples after threshold excluded from sum
 *  @param NSA  Number of samples after pulse over threshold to be included in sum
 *  @param NP   Number of pulses processed per window
 *  @param NPED  Number of samples to sum for pedestal
 *  @param MAXPED Maximum value of sample to be included in pedestal sum
 *  @param NSAT Number of consecutive samples over threshold for valid pulse
 *
 *    Note:
 *     - PL must be greater than PTW
 *     - NSA+NSB must be an odd number
 *     - NPED must be less than PTW and 4 <= NPED <= 15
 *
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3HallDSetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
		     uint32_t NSB, uint32_t NSA, uint32_t NP,
		     uint32_t NPED, uint32_t MAXPED, uint32_t NSAT)
{
  int rval = OK;
  int imode=0, supported_modes[FAV3_SUPPORTED_NMODES] = {FAV3_SUPPORTED_MODES};
  int mode_supported=0;
  int mode_bit=0;

  CHECKID;

  for(imode=0; imode<FAV3_SUPPORTED_NMODES; imode++)
    {
      if(pmode == supported_modes[imode])
	mode_supported=1;
    }
  if(!mode_supported)
    {
      printf("%s: ERROR: Processing Mode (%d) not supported\n",
	     __func__, pmode);
      return ERROR;
    }

  /* Set Min/Max parameters if specified values are out of bounds */
  if((PL < FAV3_ADC_MIN_PL) || (PL > FAV3_ADC_MAX_PL))
    {
      printf("%s: WARN: PL (%d) out of bounds.  ", __func__, PL);
      PL  = (PL < FAV3_ADC_MIN_PL) ? FAV3_ADC_MIN_PL : FAV3_ADC_MAX_PL;
      printf("Setting to %d.\n", PL);
    }

  if((PTW < FAV3_ADC_MIN_PTW) || (PTW > FAV3_ADC_MAX_PTW))
    {
      printf("%s: WARN: PTW (%d) out of bounds.  ", __func__, PTW);
      PTW = (PTW < FAV3_ADC_MIN_PTW) ? FAV3_ADC_MIN_PTW : FAV3_ADC_MAX_PTW;
      printf("Setting to %d.\n", PTW);
    }

  if((NSB < FAV3_ADC_MIN_NSB) || (NSB > FAV3_ADC_MAX_NSB))
    {
      printf("%s: WARN: NSB (%d) out of bounds.  ", __func__, NSB);
      NSB = (NSB < FAV3_ADC_MIN_NSB) ? FAV3_ADC_MIN_NSB : FAV3_ADC_MAX_NSB;
      printf("Setting to %d.\n", NSB);
    }

  if((NSA < FAV3_ADC_MIN_NSA) || (NSA > FAV3_ADC_MAX_NSA))
    {
      printf("%s: WARN: NSA (%d) out of bounds.  ", __func__, NSA);
      NSA = (NSA < FAV3_ADC_MIN_NSA) ? FAV3_ADC_MIN_NSA : FAV3_ADC_MAX_NSA;
      if(((NSB + NSA) % 2)==0) /* Make sure NSA+NSB is an odd number */
	NSA = (NSA==FAV3_ADC_MIN_NSA) ? NSA + 1 : NSA - 1;
      printf("Setting to %d.\n", NSA);
    }

  if( (NSB < 0) && ((NSA - (NSB & 0x3)) < 3))
    {
      printf("%s: ERROR: NSB is negative and (NSA - (NSB & 0x3)) < 3\n",
	     __func__);
    }

  if((NP < FAV3_ADC_MIN_NP) || (NP > FAV3_ADC_MAX_NP))
    {
      printf("%s: WARN: NP (%d) out of bounds.  ",__func__,NP);
      NP = (NP < FAV3_ADC_MIN_NP) ? FAV3_ADC_MIN_NP : FAV3_ADC_MAX_NP;
      printf("Setting to %d.\n",NP);
    }

  if((NPED < FAV3_ADC_MIN_NPED) || (NPED > FAV3_ADC_MAX_NPED))
    {
      printf("%s: WARN: NPED (%d) out of bounds.  ",__func__,NPED);
      NPED = (NPED < FAV3_ADC_MIN_NPED) ? FAV3_ADC_MIN_NPED : FAV3_ADC_MAX_NPED;
      printf("Setting to %d.\n",NPED);
    }
  if(NPED >= PTW)
    {
      printf("%s: WARN: NPED (%d) >= PTW (%d)  ",__func__, NPED, PTW);
      NPED = PTW - 1;
      printf("Setting to %d.\n",NPED);
    }


  if((MAXPED < FAV3_ADC_MIN_MAXPED) || (MAXPED > FAV3_ADC_MAX_MAXPED))
    {
      printf("%s: WARN: MAXPED (%d) out of bounds.  ",__func__,MAXPED);
      MAXPED = (MAXPED < FAV3_ADC_MIN_MAXPED) ? FAV3_ADC_MIN_MAXPED : FAV3_ADC_MAX_MAXPED;
      printf("Setting to %d.\n",MAXPED);
    }

  if((NSAT < FAV3_ADC_MIN_NSAT) || (NSAT > FAV3_ADC_MAX_NSAT))
    {
      printf("%s: WARN: NSAT (%d) out of bounds.  ",__func__,NSAT);
      NSAT = (NSAT < FAV3_ADC_MIN_NSAT) ? FAV3_ADC_MIN_NSAT : FAV3_ADC_MAX_NSAT;
      printf("Setting to %d.\n",NSAT);
    }

  rval = faV3SetupADC(id, 0);

  FAV3LOCK;
  /* Disable ADC processing while writing window info */
  if(pmode == FAV3_HALLD_PROC_MODE_PULSE_PARAM)
    mode_bit = 0;
  else if(pmode == FAV3_HALLD_PROC_MODE_DEBUG)
    mode_bit = 1;
  else if(pmode == FAV3_HALLD_PROC_MODE_RAW)
    mode_bit = 3;
  else
    {
      printf("%s: ERROR: Unsupported mode (%d)\n",
	     __func__, pmode);
      return ERROR;
    }

  /* Configure the mode (mode_bit), # of pulses (NP), # samples above TET (NSAT)
     keep TNSAT, if it's already been configured */
  vmeWrite32(&HallDp[id]->config1,
	     (vmeRead32(&HallDp[id]->config1) & FAV3_ADC_CONFIG1_TNSAT_MASK) |
	     (mode_bit << 8) | ((NP-1) << 4) | ((NSAT-1) << 10) );
  /* Disable user-requested channels */
  vmeWrite32(&HallDp[id]->config2, faV3ChanDisableMask[id]);

  /* Set window parameters */
  vmeWrite32(&HallDp[id]->pl, PL);
  vmeWrite32(&HallDp[id]->ptw, PTW - 1);

  /* Set Readback NSB, NSA */
  if(NSB < 0) /* Convert value if negative */
    NSB = ((-1) * NSB) | FAV3_ADC_NSB_NEGATIVE;

  vmeWrite32(&HallDp[id]->nsb, NSB);
  vmeWrite32(&HallDp[id]->nsa,
	     (vmeRead32(&HallDp[id]->nsa) & FAV3_ADC_TNSA_MASK) |
	     NSA );

  /* Set Pedestal parameters */
  vmeWrite32(&HallDp[id]->config7, (NPED-1)<<10 | (MAXPED));

  /* Enable ADC processing */
  vmeWrite32(&HallDp[id]->config1,
	     vmeRead32(&HallDp[id]->config1) | FAV3_ADC_PROC_ENABLE );

  /* Set default value of trigger path threshold (TPT) */
  vmeWrite32(&HallDp[id]->config3, FAV3_ADC_DEFAULT_TPT);
  FAV3UNLOCK;

  faV3SetTriggerStopCondition(id, faV3HallDCalcMaxUnAckTriggers(pmode,PTW,NSA,NSB,NP));
  faV3SetTriggerBusyCondition(id, faV3HallDCalcMaxUnAckTriggers(pmode,PTW,NSA,NSB,NP));

  return(rval);
}

/**
 *  @ingroup Config
 *  @brief Configure the processing type/mode for all initialized fADC250s
 *
 *  @param id Slot number
 *  @param pmode  Processing Mode
 *     -     9 - Pulse Parameter Mode
 *     -    10 - Debug Mode (9 + Raw Samples)
 *  @param  PL  Window Latency
 *  @param PTW  Window Width
 *  @param NSB  If NSB > 0: Number of samples before pulse over threshold included in sum
 *                 NSB < 0: Number of samples after threshold excluded from sum
 *  @param NSA  Number of samples after pulse over threshold to be included in sum
 *  @param NP   Number of pulses processed per window
 *  @param NPED  Number of samples to sum for pedestal
 *  @param MAXPED Maximum value of sample to be included in pedestal sum
 *  @param NSAT Number of consecutive samples over threshold for valid pulse
 *
 *    Note:
 *     - PL must be greater than PTW
 *     - NSA+NSB must be an odd number
 *     - NPED must be less than PTW and 4 <= NPED <= 15
 *
 *  @return OK if successful, otherwise ERROR.
 */

void
faV3HallDGSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
		      uint32_t NSB, uint32_t NSA, uint32_t NP,
		      uint32_t NPED, uint32_t MAXPED, uint32_t NSAT)
{
  int ii, res;

  for (ii=0;ii<nfaV3;ii++)
    {
      res = faV3HallDSetProcMode(faV3ID[ii], pmode, PL, PTW, NSB, NSA, NP, NPED, MAXPED, NSAT);
      if(res<0)
	printf("ERROR: slot %d, in faSetProcMode()\n", faV3ID[ii]);
    }
}

int32_t
faV3HallDGetProcMode(int id, int *pmode, uint32_t *PL, uint32_t *PTW,
		     uint32_t *NSB, uint32_t *NSA, uint32_t *NP,
		     uint32_t *NPED, uint32_t *MAXPED, uint32_t *NSAT)
{
  int32_t rval = OK;
  uint32_t config1 = 0, config7 = 0, nsb = 0, mode = 0;
  CHECKID;

  FAV3LOCK;


  config1 = vmeRead32(&HallDp[id]->config1);
  mode = (config1 & FAV3_ADC_CONFIG1_MODE_MASK) >> 8;
  *pmode = (mode == 3) ? 1 : (mode + 9);
  *NP = ((config1 & FAV3_ADC_CONFIG1_NP_MASK) >> 4) + 1;
  *NSAT = ((config1 & FAV3_ADC_CONFIG1_NSAT_MASK) >> 10) + 1;

  *PL = vmeRead32(&HallDp[id]->pl) & 0xFFFF;
  *PTW = (vmeRead32(&HallDp[id]->ptw) & 0xFFFF) + 1;

  nsb = vmeRead32(&HallDp[id]->nsb) & FAV3_ADC_NSB_READBACK_MASK;
  if(nsb & FAV3_ADC_NSB_NEGATIVE)
    *NSB = (-1) * nsb * 0x3;
  else
    *NSB = nsb * 0x7;

  *NSA = vmeRead32(&HallDp[id]->nsa) & FAV3_ADC_NSA_READBACK_MASK;

  config7 = vmeRead32(&HallDp[id]->config7); // (NPED-1)<<10 | (MAXPED));
  *NPED = (config7 & FAV3_ADC_CONFIG7_NPED_MASK) >> 10;
  *MAXPED = (config7 & FAV3_ADC_CONFIG7_MAXPED_MASK);

  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Readout
 *  @brief Configure pedestal parameters to be used by processing algorythm
 *  @param id Slot number
 *  @param nsamples Number of samples to contribute to sum
 *  @param maxvalue Maximum sample value to be included in the sum
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3HallDProcPedConfig(int id, int nsamples, int maxvalue)
{
  CHECKID;

  if((nsamples < FAV3_ADC_MIN_NPED) || (nsamples > FAV3_ADC_MAX_NPED))
    {
      printf("%s: ERROR: Invalid nsamples (%d)\n",
	     __func__, nsamples);
      return ERROR;
    }

  if((maxvalue < 0) || (maxvalue > 0x3ff))
    {
      printf("%s: ERROR: Invalid maxvalue (%d)\n",
	     __func__, maxvalue);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&HallDp[id]->config7,
	     (nsamples - 1)<<10 | maxvalue);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Readout
 *  @brief Configure pedestal parameters to be used by processing algorythm
 *    for all initialized modules.
 *  @param nsamples Number of samples to contribute to sum
 *  @param maxvalue Maximum sample value to be included in the sum
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3HallDGProcPedConfig(int nsamples, int maxvalue)
{
  int ifa=0, rval=OK;


  for(ifa = 0; ifa < nfaV3; ifa++)
    rval |= faV3HallDProcPedConfig(faV3Slot(ifa), nsamples, maxvalue);

  return rval;
}

/**
 *  @ingroup Readout
 *  @brief Configure output of sample data from @faReadAllChannelSamples
 *  @param id Slot number
 *  @param nsamples Number of samples to contribute to sum
 *  @param maxvalue Maximum sample value to be included in the sum
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3HallDSampleConfig(int id, int nsamples, int maxvalue)
{
  CHECKID;

  if((nsamples < FAV3_ADC_MIN_MNPED) || (nsamples > FAV3_ADC_MAX_MNPED))
    {
      printf("%s: ERROR: Invalid nsamples (%d)\n",
	     __func__, nsamples);
      return ERROR;
    }

  if((maxvalue < 0) || (maxvalue > 0x3ff))
    {
      printf("%s: ERROR: Invalid maxvalue (%d)\n",
	     __func__, maxvalue);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&HallDp[id]->config6,
	     (nsamples - 1)<<10 | maxvalue);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Readout
 *  @brief Configure output of sample data from @faReadAllChannelSamples
 *    for all initialized modules.
 *  @param nsamples Number of samples to contribute to sum
 *  @param maxvalue Maximum sample value to be included in the sum
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3HallDGSampleConfig(int nsamples, int maxvalue)
{
  int ifa=0, rval=OK;


  for(ifa = 0; ifa < nfaV3; ifa++)
    rval |= faV3HallDSampleConfig(faV3Slot(ifa), nsamples, maxvalue);

  return rval;
}

/**
 *  @ingroup Readout
 *  @brief Read the current sample data from the specified channel and module.
 *  @param id     Slot number
 *  @param data   local memory address to place data
 *                * Least significant 16bits contain lesser channel number data
 *  @return Number of words stored in data if successful, otherwise ERROR.
 *         Sums in 'data' are valid up to 16383 (0x3fff).  Bit 15 will be high
 *         if a sample in the sum in less than zero, or greater than maxvalue
 *         configured with @faSampleConfig
 */
int
faV3HallDReadAllChannelSamples(int id, uint16_t data[16])
{
  int ichan=0, iwait = 0;
  const int nwait = 10;
  uint32_t config1 = 0, status2 = 0;

  CHECKID;

  FAV3LOCK;

  config1 = vmeRead32(&HallDp[id]->config1);
  // Set request bit
  vmeWrite32(&HallDp[id]->config1, (config1 |  FAV3_ADC_CONFIG1_CHAN_READ_ENABLE) );

  // reset request bit
  vmeWrite32(&HallDp[id]->config1, config1);


  status2 = vmeRead32(&HallDp[id]->status2);
  while( ((status2 & (1<<15)) == 0) && (iwait++ < nwait))
    status2 = vmeRead32(&HallDp[id]->status2);

  if((status2 & (1<<15)) == 0)
    {
      printf("%s(id = %d): Timeout waiting for Channel Samples\n",
	     __func__, id);
      FAV3UNLOCK;
      return ERROR;
    }

  data[0] = status2 & 0x7FFF;
  for(ichan=1; ichan<FAV3_MAX_ADC_CHANNELS; ichan++)
    {
      status2 = vmeRead32(&HallDp[id]->status2);
      data[ichan] = status2 & 0x7FFF;
    }
  FAV3UNLOCK;

  return (FAV3_MAX_ADC_CHANNELS);
}



/**
 *  @ingroup Deprec
 *  @brief Set the fa250 operation when Sync Reset is received
 *
 *   This routine is deprecated.  Use faSetAlignmentDebugMode
 *
 *  @param id Slot number
 *  @param  mode
 *    - 0:  Send a calibration sequence to the CTP for alignment purposes
 *    - 1:  Normal operation
 *  @sa faSetAlignmentDebugMode
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetMGTTestMode(int id, uint32_t mode)
{
  CHECKID;

  FAV3LOCK;
  if(mode)
    {				/* After Sync Reset (Normal mode) */
      vmeWrite32(&FAV3p[id]->ctrl_mgt, FAV3_MGT_RESET);
      vmeWrite32(&FAV3p[id]->ctrl_mgt, FAV3_MGT_FRONT_END_TO_CTP);
    }
  else
    {				/* Before Sync Reset (Calibration Mode) */
      vmeWrite32(&FAV3p[id]->ctrl_mgt, FAV3_RELEASE_MGT_RESET);
      vmeWrite32(&FAV3p[id]->ctrl_mgt, FAV3_MGT_RESET);
      vmeWrite32(&FAV3p[id]->ctrl_mgt, FAV3_MGT_ENABLE_DATA_ALIGNMENT);
    }
  FAV3UNLOCK;

  return (OK);
}



int
faV3SyncResetMode(int id, uint32_t mode)
{
  return faV3SetMGTTestMode(id, mode);
}



/**
 *  @ingroup Status
 *  @brief Return whether or not the module will send the alignment sequence to the CTP
 *  @return 1 if enabled, 0 if disabled
 */
int
faV3GetAlignmentDebugMode()
{
  return faV3AlignmentDebug;
}

/**
 *  @ingroup Config
 *  @brief Enable/Disable Hitbits mode on the module
 *  @param id Slot number
 *  @param enable
 *   -  0: Disable
 *   - >0: Enable
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3SetHitbitsMode(int id, int enable)
{
  CHECKID;

  FAV3LOCK;
  if(enable)
    {
      vmeWrite32(&FAV3p[id]->ctrl_mgt,
		 vmeRead32(&FAV3p[id]->ctrl_mgt) | FAV3_MGT_HITBITS_TO_CTP);
    }
  else
    { /* Before Sync Reset (Calibration Mode) */
      vmeWrite32(&FAV3p[id]->ctrl_mgt,
		 vmeRead32(&FAV3p[id]->ctrl_mgt) & ~FAV3_MGT_HITBITS_TO_CTP);
    }
  FAV3UNLOCK;

  return(OK);
}

/**
 *  @ingroup Config
 *  @brief Enable/Disable Hitbits mode for all initialized fADC250s
 *  @param enable
 *   -  0: Disable
 *   - >0: Enable
 */
void
faV3GSetHitbitsMode(int enable)
{
  int ifadc;

  for(ifadc=0;ifadc<nfaV3;ifadc++)
    faV3SetHitbitsMode(faV3Slot(ifadc),enable);

}

/**
 *  @ingroup Status
 *  @brief Get the enabled/disabled status of hitbits mode for the module
 *  @param id Slot number
 *  @return 1 if enabled, 0 if disabled, otherwise ERROR.
 */
int
faV3GetHitbitsMode(int id)
{
  int rval;
  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->ctrl_mgt)&FAV3_MGT_HITBITS_TO_CTP)>>3;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Enable / Disable Rogue PTW Fall Back for specified channel mask
 *
 *    When enabled, send raw data when any of the first 4 samples is
 *    above threshold.  When disabled, proceed to calculate SUM and TDC
 *
 *  @param id Slot number
 *  @param enablemask Enabled Channel Mask [0,0xffff]
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3HallDSetRoguePTWFallBack(int id, uint16_t enablemask)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&HallDp[id]->rogue_ptw_fall_back, enablemask);
  FAV3UNLOCK;

  return(OK);
}

/**
 *  @ingroup Status
 *  @brief Return mask of channels with Rogue PTW Fall Back enabled
 *
 *    When enabled, send raw data when any of the first 4 samples is
 *    above threshold.  When disabled, proceed to calculate SUM and TDC
 *
 *  @param id Slot number
 *  @return Enabled Channel Mask if successful, otherwise ERROR.
 */

int
faV3HallDGetRoguePTWFallBack(int id, uint16_t *enablemask)
{
  int rval = OK;
  CHECKID;

  FAV3LOCK;
  *enablemask = vmeRead32(&HallDp[id]->rogue_ptw_fall_back) & FAV3_ROGUE_PTW_FALL_BACK_MASK;
  FAV3UNLOCK;

  return(rval);
}


/**
 *  @ingroup Config
 *  @brief Insert ADC parameter word into datastream.
 *     The data word appears as a block header continuation word.
 *  @param id Slot number
 *  @param enable Enable flag
 *      -  0: Disable
 *      - !0: Enable
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3HallDDataInsertAdcParameters(int id, int enable)
{
  CHECKID;

  FAV3LOCK;
  if(enable)
    vmeWrite32(&FAV3p[id]->ctrl1,
	       vmeRead32(&FAV3p[id]->ctrl1) | FAV3_ENABLE_ADC_PARAMETERS_DATA);
  else
    vmeWrite32(&FAV3p[id]->ctrl1,
	       vmeRead32(&FAV3p[id]->ctrl1) & ~FAV3_ENABLE_ADC_PARAMETERS_DATA);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Insert ADC parameter word into datastream. For all initialized modules.
 *     The data word appears as a block header continuation word.
 *  @param enable Enable flag
 *      -  0: Disable
 *      - !0: Enable
 */
void
faV3HallDGDataInsertAdcParameters(int enable)
{
  int ifadc;

  for(ifadc=0;ifadc<nfaV3;ifadc++)
    faV3HallDDataInsertAdcParameters(faV3Slot(ifadc), enable);

}

/**
 *  @ingroup Status
 *  @brief Get the status of Insert ADC parameter word into datastream.
 *     The data word appears as a block header continuation word.
 *  @param id Slot number
 *  @return 1 if enabled, 0 if disabled, otherwise ERROR.
 */
int
faV3HallDDataGetInsertAdcParameters(int id)
{
  int rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->ctrl1) & FAV3_ENABLE_ADC_PARAMETERS_DATA) ? 1 : 0;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Enable/Disable suppression of one or both of the trigger time words
 *    in the data stream.
 *  @param id Slot number
 *  @param suppress Suppression Flag
 *      -  0: Trigger time words are enabled in datastream
 *      -  1: Suppress BOTH trigger time words
 *      -  2: Suppress trigger time word 2 (that with most significant bytes)
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3HallDDataSuppressTriggerTime(int id, int suppress)
{
  unsigned int suppress_bits=0;
  CHECKID;

  switch(suppress)
    {
    case 0: /* Enable trigger time words */
      suppress_bits = FAV3_SUPPRESS_TRIGGER_TIME_DATA;
      break;

    case 1: /* Suppress both trigger time words */
      suppress_bits = FAV3_SUPPRESS_TRIGGER_TIME_DATA;
      break;

    case 2: /* Suppress trigger time word 2 */
      suppress_bits = FAV3_SUPPRESS_TRIGGER_TIME_WORD2_DATA;
      break;

    default:
      printf("%s(%d): ERROR: Invalid suppress (%d)\n",
	     __func__,id,suppress);
      return ERROR;
    }

  FAV3LOCK;
  if(suppress)
    vmeWrite32(&FAV3p[id]->ctrl1,
	       vmeRead32(&FAV3p[id]->ctrl1) | suppress_bits);
  else
    vmeWrite32(&FAV3p[id]->ctrl1,
	       vmeRead32(&FAV3p[id]->ctrl1) & ~suppress_bits);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable/Disable suppression of one or both of the trigger time words
 *    in the data stream for all initialized modules.
 *  @param suppress Suppression Flag
 *      -  0: Trigger time words are enabled in datastream
 *      -  1: Suppress BOTH trigger time words
 *      -  2: Suppress trigger time word 2 (that with most significant bytes)
 */
void
faV3HallDGDataSuppressTriggerTime(int suppress)
{
  int ifadc;

  for(ifadc=0;ifadc<nfaV3;ifadc++)
    faV3HallDDataSuppressTriggerTime(faV3Slot(ifadc), suppress);

}

int
faV3HallDDataGetSuppressTriggerTime(int id)
{
  int rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->ctrl1) & FAV3_SUPPRESS_TRIGGER_TIME_MASK) >> 16;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Set the readout data form which allows for suppression of
 *         repetitious data words
 *  @param id Slot number
 *  @param format Data Format
 *      -  0: Standard Format - No data words suppressed
 *      -  1: Intermediate compression - Event headers suppressed if no data
 *      -  2: Full compression - Only first event header in the block.
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3HallDSetDataFormat(int id, int format)
{
  CHECKID;

  if((format < 0) || (format > 2))
    {
      printf("%s: ERROR: Invalid format (%d) \n",
	     __func__, format);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->ctrl1,
	     (vmeRead32(&FAV3p[id]->ctrl1) & ~FAV3_CTRL1_DATAFORMAT_MASK)
	     | (format << 26));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the readout data form for all initialized modules.
 *  @param format Data Format
 *      -  0: Standard Format - No data words suppressed
 *      -  1: Intermediate compression - Event headers suppressed if no data
 *      -  2: Full compression - Only first event header in the block.
 */
void
faV3HallDGSetDataFormat(int format)
{
  int ifadc;

  for(ifadc=0;ifadc<nfaV3;ifadc++)
    faV3HallDSetDataFormat(faV3Slot(ifadc), format);
}

int
faV3HallDGetDataFormat(int id)
{
  int32_t rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->ctrl1) & FAV3_CTRL1_DATAFORMAT_MASK) >> 26;
  FAV3UNLOCK;

  return rval;
}

void
faV3HallDGStatus(int sflag)
{
  int ifa, id, ii;
  faV3_t st[FAV3_MAX_BOARDS + 1];
  faV3_halld_adc_t hd_st[(FAV3_MAX_BOARDS + 1)];
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

      hd_st[id].status0 =
	vmeRead32(&HallDp[id]->status0) & 0xFFFF;
      hd_st[id].status1 =
	vmeRead32(&HallDp[id]->status1) & 0xFFFF;
      hd_st[id].status2 =
	vmeRead32(&HallDp[id]->status2) & 0xFFFF;

      hd_st[id].config1 =
	vmeRead32(&HallDp[id]->config1) & 0xFFFF;
      hd_st[id].config2 =
	vmeRead32(&HallDp[id]->config2) & 0xFFFF;
      hd_st[id].config3 =
	vmeRead32(&HallDp[id]->config3) & 0xFFFF;
      hd_st[id].config4 =
	vmeRead32(&HallDp[id]->config4) & 0xFFFF;
      hd_st[id].config5 =
	vmeRead32(&HallDp[id]->config5) & 0xFFFF;
      hd_st[id].config6 =
	vmeRead32(&HallDp[id]->config6) & 0xFFFF;
      hd_st[id].config7 =
	vmeRead32(&HallDp[id]->config7) & 0xFFFF;

      hd_st[id].ptw = vmeRead32(&HallDp[id]->ptw);
      hd_st[id].pl = vmeRead32(&HallDp[id]->pl);
      hd_st[id].nsb = vmeRead32(&HallDp[id]->nsb);
      hd_st[id].nsa = vmeRead32(&HallDp[id]->nsa);

      st[id].blk_count = vmeRead32(&FAV3p[id]->blk_count);
      st[id].blocklevel = vmeRead32(&FAV3p[id]->blocklevel);
      st[id].ram_word_count =
	vmeRead32(&FAV3p[id]->ram_word_count) & FAV3_RAM_DATA_MASK;

      st[id].trig_scal = vmeRead32(&(FAV3p[id]->trig_scal));
      st[id].trig2_scal = vmeRead32(&FAV3p[id]->trig2_scal);
      st[id].syncreset_scal = vmeRead32(&FAV3p[id]->syncreset_scal);
      st[id].berr_scal = vmeRead32(&FAV3p[id]->berr_scal);
      st[id].lost_trig_scal = vmeRead32(&FAV3p[id]->lost_trig_scal);
      st[id].aux.berr_driven_count = vmeRead32(&FAV3p[id]->aux.berr_driven_count);

      st[id].aux.sparsify_control = vmeRead32(&FAV3p[id]->aux.sparsify_control);
      st[id].status_mgt = vmeRead32(&FAV3p[id]->status_mgt);
      hd_st[id].rogue_ptw_fall_back = vmeRead32(&HallDp[id]->rogue_ptw_fall_back);

      for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
	{
	  hd_st[id].pedestal[ii] = vmeRead32(&HallDp[id]->pedestal[ii]);
	}

      for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS/2; ii++)
	{
	  hd_st[id].thres[ii] = vmeRead32(&HallDp[id]->thres[ii]);
	}

    }
  FAV3UNLOCK;

  printf("\n");

  printf("                      fADC250 Module Configuration Summary\n\n");
  printf("     Firmware Rev   .................Addresses................\n");
  printf("Slot  Ctrl   Proc      A24        A32     A32 Multiblock Range   VXS Readout\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("0x%04x 0x%04x  ", st[id].version & 0xFFFF,
	     hd_st[id].status0 & FAV3_ADC_VERSION_MASK);

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
  printf("      .Signal Sources..                        ..Channel...  ..Channel.\n");
  printf("Slot  Clk   Trig   Sync     MBlk  Token  BERR  Enabled Mask  Rogue Mask\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("%s  ",
	     (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_INTERNAL ? " INT " : (st[id].
						ctrl1 & FAV3_REF_CLK_MASK) ==
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
	     ~(hd_st[id].config2 & FAV3_ADC_CHAN_MASK) & 0xFFFF);

      printf("0x%04X",
	     hd_st[id].rogue_ptw_fall_back & FAV3_ADC_CHAN_MASK);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                         fADC250 Processing Mode Config\n\n");
  printf("      Block          ...[nanoseconds]...       [ns]\n");
  printf("Slot  Level  Mode    PL   PTW   NSB  NSA  NP   NPED  MAXPED  NSAT   Playback   \n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d    ", id);

      printf("%3d    ", st[id].blocklevel & FAV3_BLOCK_LEVEL_MASK);

      uint32_t mode = (hd_st[id].config1 & FAV3_ADC_CONFIG1_MODE_MASK) >> 8;

      printf("%2d   ", (mode == 3) ? 1 : (mode + 9));

      printf("%4d  ", (hd_st[id].pl & 0xFFFF) * FAV3_ADC_NS_PER_CLK);

      printf("%4d   ", ((hd_st[id].ptw & 0xFFFF) + 1) * FAV3_ADC_NS_PER_CLK);

      nsb = hd_st[id].nsb & FAV3_ADC_NSB_READBACK_MASK;
      nsb =
	(nsb & 0x7) * ((nsb & FAV3_ADC_NSB_NEGATIVE) ? -1 : 1) *
	FAV3_ADC_NS_PER_CLK;
      printf("%3d  ", nsb);

      printf("%3d   ",
	     (hd_st[id].nsa & FAV3_ADC_NSA_READBACK_MASK) * FAV3_ADC_NS_PER_CLK);

      printf("%1d     ",
	     ((hd_st[id].config1 & FAV3_ADC_PEAK_MASK) >> 4) + 1);

      printf("%2d    ", (((hd_st[id].config7 & FAV3_ADC_CONFIG7_NPED_MASK)>>10) + 1)*FAV3_ADC_NS_PER_CLK);

      printf("%4d     ", hd_st[id].config7 & FAV3_ADC_CONFIG7_MAXPED_MASK);

      printf("%d   ", ((hd_st[id].config1 & FAV3_ADC_CONFIG1_NSAT_MASK)>>10) + 1);

      printf("%s   ",
	     (hd_st[id].config1 &FAV3_ADC_PLAYBACK_MODE)>>7 ?" Enabled":"Disabled");


      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("  fADC250 Trigger Path Processing        fADC250 Pedestal Monitoring\n\n");
  printf("         [ns]               [ns]\n");
  printf("Slot     TNSA      TPT     TNSAT               MNPED     MMAXPED\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa=0; ifa<nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d       ",id);

      printf("%3d     ", ((hd_st[id].nsa & FAV3_ADC_TNSA_MASK)>>9)*FAV3_ADC_NS_PER_CLK);

      printf("%4d         ", hd_st[id].config3 & FAV3_ADC_TPT_MASK);

      printf("%d                ", (((hd_st[id].config1 &FAV3_ADC_CONFIG1_TNSAT_MASK)>>12) + 1)*FAV3_ADC_NS_PER_CLK);

      printf("%4d        ", (( (hd_st[id].config6 & 0x3C00 )>>10) + 1));

      printf("%4d   ", ( hd_st[id].config6 & 0x3FF ));

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");


  printf("\n");
  printf("                             fADC250 Signal Scalers\n\n");
  printf("Slot       Trig1       Trig2   SyncReset        BERR  Lost Triggers\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa=0; ifa<nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d   ",id);

      printf("%10d  ", st[id].trig_scal);

      printf("%10d  ", st[id].trig2_scal);

      printf("%10d  ", st[id].syncreset_scal);

      printf("%10d     ", st[id].berr_scal);

      printf("%10d  ", st[id].lost_trig_scal);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                              fADC250 Data Status\n\n");
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
  printf("\n");

}
