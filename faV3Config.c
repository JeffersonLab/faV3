/****************************************************************************
 *
 *  faV3Config.c  -  configuration library file for fADC250 V3 board
 *
 *  empty lines and line startes with # - will be ignored
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include "faV3Config.h"
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-HallD.h"

#include "config_defs.h"

static int active;

#define NBOARD     21
static FAV3_CONF faV3[NBOARD+1];

static char *expid = NULL;

int
faV3Config(char *fname)
{
  int res;
  char string[10]; /*dummy, will not be used*/

  if(strlen(fname) > 0) /* filename specified  - upload initial settings from the hardware */
    {
      faV3UploadAll(string, 0);
    }
  else /* filename not specified  - set defaults */
    {
      faV3InitGlobals();
    }

  /* reading and parsing config file */
  if( (res = faV3ReadConfigFile(fname)) < 0 )
    {
      printf("ERROR in %s: faV3ReadConfigFile() returns %d\n",
	     __func__, res);
      return(res);
    }

  /* download to all boards */
  faV3DownloadAll();

  return(0);
}

void
faV3InitGlobals()
{
  int chan, slot;

  for(slot = 0; slot < NBOARD; slot++)
    {
      faV3[slot].mode = FAV3_ADC_DEFAULT_MODE;
      faV3[slot].compression = 0;
      faV3[slot].vxsReadout = 0;
      faV3[slot].winOffset = FAV3_ADC_DEFAULT_PL * FAV3_ADC_NS_PER_CLK;
      faV3[slot].winWidth = FAV3_ADC_DEFAULT_PTW * FAV3_ADC_NS_PER_CLK;
      faV3[slot].nsa = FAV3_ADC_DEFAULT_NSA * FAV3_ADC_NS_PER_CLK;
      faV3[slot].nsb = FAV3_ADC_DEFAULT_NSB * FAV3_ADC_NS_PER_CLK;
      faV3[slot].npeak = FAV3_ADC_DEFAULT_NP;
      faV3[slot].chDisMask = 0x0;

      faV3[slot].ptw_fallback_mask = 0;

      faV3[slot].nsat = FAV3_ADC_DEFAULT_NSAT * FAV3_ADC_NS_PER_CLK;
      faV3[slot].nped = FAV3_ADC_DEFAULT_NPED;
      faV3[slot].max_ped = FAV3_ADC_DEFAULT_MAXPED;

      faV3[slot].trig_thr = FAV3_ADC_DEFAULT_TPT;
      faV3[slot].trig_nsa = FAV3_ADC_DEFAULT_TNSA * FAV3_ADC_NS_PER_CLK;
      faV3[slot].trig_nsat = FAV3_ADC_DEFAULT_TNSAT * FAV3_ADC_NS_PER_CLK;

      faV3[slot].busy = 8;
      faV3[slot].stop = 9;

      for(chan = 0; chan < NCHAN; chan++)
	{
	  faV3[slot].pedestal[chan] = 300.;
	  faV3[slot].read_thr[chan] = FAV3_ADC_DEFAULT_TET;
	  faV3[slot].dac[chan] = FAV3_ADC_DEFAULT_DAC;
	}
    }
}

/* reading and parsing config file */
int
faV3ReadConfigFile(char *filename_in)
{
  FILE   *fd;
  char   filename[FNLEN];
  char   fname[FNLEN] = { "" };  /* config file name */
  char   host[ROCLEN], ROC_name[ROCLEN];
  char *envDir = NULL;
  int do_parsing;
  SCAN_VARS;

  gethostname(host,ROCLEN);  /* obtain our hostname */

  strcpy(filename,filename_in); /* copy filename from parameter list to local string */
  do_parsing = 1;

  if((fd=fopen(filename,"r")) == NULL)
    {
      printf("%s: Can't open config file >%s<\n",
	     __func__, filename);
      return(-1);
    }

  printf("%s: Using configuration file >%s<\n",
	 __func__, filename);

  /* Parsing of config file */
  active = 0; /* by default disable crate */
  int ch = 0;
  while ((ch = getc(fd)) != EOF)
    {
      if ( ch == '#' || ch == ' ' || ch == '\t' )
	{
	  while ( getc(fd)!='\n' /*&& getc(fd)!=!= EOF*/ ) {} /*ERROR !!!*/
	  continue;
	}
      if( ch == '\n' )
	continue;

      ungetc(ch,fd);
      fgets(str_tmp, STRLEN, fd);
      sscanf (str_tmp, "%s %s", keyword, ROC_name);

      /* Start parsing real config inputs */
      if(strcmp(keyword,"FAV3_CRATE") == 0)
	{
	  if(strcmp(ROC_name,host) == 0)
	    {
	      printf("%s: FAV3_CRATE = %s  host = %s - active\n", __func__, ROC_name, host);
	      active = 1;
	    }
	  else if(strcmp(ROC_name,"all") == 0)
	    {
	      printf("%s: FAV3_CRATE = %s  host = %s - active\n",__func__, ROC_name,host);
	      active = 1;
	    }
	  else
	    {
	      printf("%s: FAV3_CRATE = %s  host = %s - not active\n", __func__, ROC_name,host);
	      active = 0;
	    }
	  continue;
	}

      SCAN_SLOT("FAV3_SLOT", slot_min, slot_max);
      SCAN_INT("FAV3_PROC_VERSION", faV3[slot].proc_version, slot_min, slot_max);

      SCAN_MASK_INV("FAV3_ADC_MASK", faV3[slot].chDisMask, slot_min, slot_max);

      SCAN_INT("FAV3_MODE", faV3[slot].mode, slot_min, slot_max);

      SCAN_INT("FAV3_COMPRESSION", faV3[slot].compression, slot_min, slot_max);
      SCAN_INT("FAV3_VXSREADOUT", faV3[slot].vxsReadout, slot_min, slot_max);

      SCAN_INT("FAV3_W_OFFSET", faV3[slot].winOffset, slot_min, slot_max);
      SCAN_INT("FAV3_W_WIDTH", faV3[slot].winWidth, slot_min, slot_max);
      SCAN_INT("FAV3_NSA", faV3[slot].nsa, slot_min, slot_max);
      SCAN_INT("FAV3_NSB", faV3[slot].nsb, slot_min, slot_max);
      SCAN_INT("FAV3_NPEAK", faV3[slot].npeak, slot_min, slot_max);
      SCAN_INT("FAV3_NSAT", faV3[slot].nsat, slot_min, slot_max);
      SCAN_INT("FAV3_NPED", faV3[slot].nped, slot_min, slot_max);
      SCAN_INT("FAV3_MAXPED", faV3[slot].max_ped, slot_min, slot_max);

      SCAN_INT("FAV3_TRIG_NSA", faV3[slot].trig_nsa, slot_min, slot_max);
      SCAN_INT("FAV3_TRIG_NSAT", faV3[slot].trig_nsat, slot_min, slot_max);

      SCAN_INT("FAV3_TRIG_THR", faV3[slot].trig_thr, slot_min, slot_max);

      SCAN_INT_ALL("FAV3_READ_THR", faV3[slot].read_thr, slot_min, slot_max);
      SCAN_INT_CH("FAV3_CH_READ_THR", faV3[slot].read_thr, slot_min, slot_max);
      SCAN_INT_ALLCH("FAV3_ALLCH_READ_THR", faV3[slot].read_thr, slot_min, slot_max);

      SCAN_FLOAT_ALL("FAV3_PED", faV3[slot].pedestal, slot_min, slot_max);
      SCAN_FLOAT_CH("FAV3_CH_PED", faV3[slot].pedestal, slot_min, slot_max);
      SCAN_FLOAT_ALLCH("FAV3_ALLCH_PED", faV3[slot].pedestal, slot_min, slot_max);

      SCAN_INT_ALL("FAV3_DAC", faV3[slot].dac, slot_min, slot_max);
      SCAN_INT_CH("FAV3_CH_DAC", faV3[slot].dac, slot_min, slot_max);
      SCAN_INT_ALLCH("FAV3_ALLCH_DAC", faV3[slot].dac, slot_min, slot_max);

      SCAN_INT("FAV3_BUSY",  faV3[slot].busy, slot_min, slot_max);
      SCAN_INT("FAV3_STOP",  faV3[slot].stop, slot_min, slot_max);

      SCAN_MASK("FAV3_PTW_FALLBACK_MASK", faV3[slot].ptw_fallback_mask, slot_min, slot_max);

      SCAN_INT("FAV3_DATA_FORMAT",  faV3[slot].data_format, slot_min, slot_max);

      SCAN_INT("FAV3_SUPPRESS_TRIG_TIME",  faV3[slot].suppress_trig_time, slot_min, slot_max);
      SCAN_INT("FAV3_INSERT_ADC_PARAMS",  faV3[slot].insert_adc_params, slot_min, slot_max);

      if(active)
	{
	  printf("%s: ERROR: Unknown keyword: %s\n", __func__, keyword);
	}

    }

  fclose(fd);

  return(0);
}


/* download setting into all found FADCs */
int
faV3DownloadAll()
{
  int slot, ichan, ifa, nfadc;
  float ped;
  extern int faV3FwRev[(FAV3_MAX_BOARDS + 1)][FAV3_FW_FUNCTION_MAX];

    /* faInit() must be called by now; get the number of boards from there */
  nfadc = faV3GetN();
  printf("%s: nfadc=%d\n", __func__, nfadc);
  for(ifa=0; ifa<nfadc; ifa++)
    {
      slot = faV3Slot(ifa);

      if(faV3FwRev[slot][FAV3_FW_PROC] == FAV3_HALLD_SUPPORTED_PROC_FIRMWARE)
	{
	  faV3HallDSetProcMode(slot,
			  faV3[slot].mode,
			  faV3[slot].winOffset / FAV3_ADC_NS_PER_CLK,
			  faV3[slot].winWidth / FAV3_ADC_NS_PER_CLK,
			  faV3[slot].nsb / FAV3_ADC_NS_PER_CLK,
			  faV3[slot].nsa / FAV3_ADC_NS_PER_CLK,
			  faV3[slot].npeak,
			  faV3[slot].nped,
			  faV3[slot].max_ped,
			  faV3[slot].nsat / FAV3_ADC_NS_PER_CLK);

	  faV3HallDSetRoguePTWFallBack(slot, faV3[slot].ptw_fallback_mask);

	  faV3HallDSetDataFormat(slot, faV3[slot].data_format);
	  faV3HallDDataSuppressTriggerTime(slot, faV3[slot].suppress_trig_time);
	  faV3HallDDataInsertAdcParameters(slot, faV3[slot].insert_adc_params);
	}

      faV3SetTriggerPathSamples(slot, faV3[slot].trig_nsa / FAV3_ADC_NS_PER_CLK,
				faV3[slot].trig_nsat / FAV3_ADC_NS_PER_CLK);
      faV3SetTriggerPathThreshold(slot, faV3[slot].trig_thr);

      faV3SetChanDisableMask(slot, faV3[slot].chDisMask);
      faV3SetCompression(slot,faV3[slot].compression);
      faV3SetVXSReadout(slot,faV3[slot].vxsReadout);

      faV3SetTriggerBusyCondition(slot, faV3[slot].busy);
      faV3SetTriggerStopCondition(slot, faV3[slot].stop);

      for(ichan=0; ichan<NCHAN; ichan++)
	{
	  faV3DACSet(slot, ichan, faV3[slot].dac[ichan]);

	  faV3SetPedestal(slot, ichan, (int) faV3[slot].pedestal[ichan]);

	  faV3SetThreshold(slot, ichan, faV3[slot].read_thr[ichan]);
	}
    }

  return(0);
}

int32_t
faV3GetModulesConfig()
{
  int slot, ichan, ifa, nfadc;
  float ped = 0.;
  int thres = 0;
  uint32_t fw_vers = 0;
  extern int faV3FwRev[(FAV3_MAX_BOARDS + 1)][FAV3_FW_FUNCTION_MAX];

  nfadc = faV3GetN();

  for(ifa = 0; ifa < nfadc; ifa++)
    {
      slot = faV3Slot(ifa);
      if(faV3FwRev[slot][FAV3_FW_PROC] == FAV3_HALLD_SUPPORTED_PROC_FIRMWARE)
	{
	  faV3HallDGetProcMode(slot,
			       &faV3[slot].mode,
			       &faV3[slot].winOffset,
			       &faV3[slot].winWidth,
			       &faV3[slot].nsb,
			       &faV3[slot].nsa,
			       &faV3[slot].npeak,
			       &faV3[slot].nped,
			       &faV3[slot].max_ped,
			       &faV3[slot].nsat);
	  faV3[slot].insert_adc_params = faV3HallDDataGetInsertAdcParameters(slot);
	  faV3[slot].suppress_trig_time = faV3HallDDataGetSuppressTriggerTime(slot);
	  faV3[slot].data_format = faV3HallDGetDataFormat(slot);

	  faV3[slot].winOffset *= FAV3_ADC_NS_PER_CLK;
	  faV3[slot].winWidth *= FAV3_ADC_NS_PER_CLK;
	  faV3[slot].nsb *= FAV3_ADC_NS_PER_CLK;
	  faV3[slot].nsa *= FAV3_ADC_NS_PER_CLK;
	  faV3[slot].nsat *= FAV3_ADC_NS_PER_CLK;
	}

      faV3GetTriggerPathSamples(slot, &faV3[slot].trig_nsa, &faV3[slot].trig_nsat);
      faV3[slot].trig_nsa *= FAV3_ADC_NS_PER_CLK;
      faV3[slot].trig_nsat *= FAV3_ADC_NS_PER_CLK;
      faV3GetTriggerPathThreshold(slot, &faV3[slot].trig_thr);


      faV3[slot].chDisMask = faV3GetChanDisableMask(slot);
      faV3[slot].compression = faV3GetCompression(slot);
      faV3[slot].vxsReadout = faV3GetVXSReadout(slot);

      faV3GetTriggerBusyCondition(slot, &faV3[slot].busy);
      faV3GetTriggerStopCondition(slot, &faV3[slot].stop);

      for(ichan = 0; ichan < FAV3_MAX_ADC_CHANNELS; ichan++)
	{
	  faV3DACGet(slot, ichan, &faV3[slot].dac[ichan]);

	  faV3[slot].pedestal[ichan] = (float) faV3GetPedestal(slot, ichan);

	  faV3[slot].read_thr[ichan] = faV3GetThreshold(slot, ichan);;
	}
    }
  return 0;
}

int32_t
faV3ConfigToString(char *string, int32_t length)
{
  int slot, ichan, ifa, nfadc;
  uint32_t adcChanEnabled = 0;
  CONFIG_STRING_VARS;

  nfadc = faV3GetN();

  str = string;
  str[0] = '\0';

  for(ifa=0; ifa<nfadc; ifa++)
    {
      slot = faV3Slot(ifa);

      sprintf(sss,"FAV3_SLOT %d\n",slot);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_MODE %d\n",      faV3[slot].mode);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_COMPRESSION %d\n", faV3[slot].compression);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_VXSREADOUT %d\n", faV3[slot].vxsReadout);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_W_OFFSET %d\n", faV3[slot].winOffset);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_W_WIDTH  %d\n", faV3[slot].winWidth);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_NSA %d\n", faV3[slot].nsa);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_NSB %d\n", faV3[slot].nsb);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_NPEAK %d\n", faV3[slot].npeak);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_NSAT %d\n", faV3[slot].nsat);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_NPED %d\n", faV3[slot].nped);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_MAXPED %d\n", faV3[slot].max_ped);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_TRIG_NSA %d\n", faV3[slot].trig_nsa);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_TRIG_NSAT %d\n", faV3[slot].trig_nsat);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_TRIG_THR %d\n", faV3[slot].trig_thr);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_BUSY %d\n", faV3[slot].busy);
      ADD_TO_STRING;

      sprintf(sss,"FAV3_STOP %d\n", faV3[slot].stop);
      ADD_TO_STRING;


      adcChanEnabled = ~faV3[slot].chDisMask & 0xFFFF;
      sprintf(sss,"FAV3_ADC_MASK");
      ADD_TO_STRING;
      for(ichan = 0; ichan < MAX_FAV3_CH; ichan++)
	{
	  sprintf(sss," %d",(adcChanEnabled>>ichan)&0x1);
	  ADD_TO_STRING;
	}
      sprintf(sss,"\n");
      ADD_TO_STRING;

      sprintf(sss,"FAV3_ALLCH_DAC");
      ADD_TO_STRING;
      for(ichan = 0; ichan < MAX_FAV3_CH; ichan++)
	{
	  sprintf(sss," %d",faV3[slot].dac[ichan]);
	  ADD_TO_STRING;
	}
      sprintf(sss,"\n");
      ADD_TO_STRING;

      sprintf(sss,"FAV3_ALLCH_READ_THR");
      ADD_TO_STRING;
      for(ichan = 0; ichan < MAX_FAV3_CH; ichan++)
	{
	  sprintf(sss," %d",faV3[slot].read_thr[ichan]);
	  ADD_TO_STRING;
	}
      sprintf(sss,"\n");
      ADD_TO_STRING;

      sprintf(sss,"FAV3_ALLCH_PED");
      ADD_TO_STRING;
      for(ichan = 0; ichan < MAX_FAV3_CH; ichan++)
	{
	  sprintf(sss," %.1f",faV3[slot].pedestal[ichan]);
	  ADD_TO_STRING;
	}
      sprintf(sss,"\n");
      ADD_TO_STRING;
    }

  CLOSE_STRING;

  return 0;
}

/* upload setting from all found FADCs */
int
faV3UploadAll(char *string, int length)
{

  faV3GetModulesConfig();

  faV3ConfigToString(string,length);

  return 0;
}

int
faV3UploadAllPrint()
{
  char str[16001];
  faV3UploadAll(str, 16000);
  printf("%s",str);

  return 0;
}
