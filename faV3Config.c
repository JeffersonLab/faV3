/****************************************************************************
 *
 *  faV3Config.c  -  configuration library file for fADC250 V3 board
 *
 *  empty lines and line startes with # - will be ignored
 *  config file format:

CRATE             <rocipname>   <- ROC/crate IP name
FAV3_ALLSLOTS             <- just keyword - all settings after this line will be implemented
#                                              for all slots, till FAV3_SLOTS will be met
FAV3_SLOTS     3  8  15   <- slot_numbers - in which next settings will be implemented
#                                              till file ends or next FAV3_SLOTS will be met
FAV3_F_REV     0x02c1     <- firmware revision  (0x0 Bits:7-0)
FAV3_B_REV     0x0a03     <- board revision     (0x0 Bits:15-8)
FAV3_ID        0xfadc     <- board type         (0x0 Bits:31-16)

FAV3_MODE         1   <- process mode: 1-4  (0x10C Bits:2-0)
FAV3_COMPRESSION  0   <- compression mode: 0-uncompressed, 1-compressed, 2-both
FAV3_VXSREADOUT   0   <- readout data through VXS: 0-disable, 1-enable
FAV3_W_OFFSET  50  <- number of ns back from trigger point. (0x120)
#                           (in Manual it is  PL=Trigger_Window(ns) * 250MHz)
FAV3_W_WIDTH   49  <- number of ns to include in trigger window. (0x11C)
#                           (in M:  PTW=Trigger_Window(ns) * 250MHz, minimum is 6)
FAV3_NSB       3   <- number of ns before trigger point to include in data processing. (0x124)
#                           This include the trigger Point. (minimum is 2 in all mode)
FAV3_NSA       6   <- number of ns after trigger point to include in data processing. (0x128)
#                           Minimum is (6 in mode 2) and ( 3 in mode 0 and 1).
#                           Number of sample report is 1 more for odd and 2 more for even NSA number.
FAV3_NPEAK     1   <- number of Pulses in Mode 2 and 3.  (0x10C Bits:6-5)

#                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 - channels ##
FAV3_ADC_MASK  1  0  1  0  1  0  1  0  1  0  1  0  1  0  1  0   <- channel enable mask
FAV3_TRG_MASK  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1   <- trigger enable mask


FAV3_TET       110        <- board Trigger Energy Threshold (TET), same for all 16 channels
FAV3_CH_TET    0    110   <- channel# and TET_value for this channel
FAV3_ALLCH_TET 111  222  2  3  4  5  6  7  8  9  10  11  12  13  14  15   <- 16 TETs (0x12C - 0x148)

FAV3_DAC       3300       <- board DAC, one and the same for all 16 channels
FAV3_CH_DAC    0    3300  <- channel# and DAC_value for this channel
FAV3_ALLCH_DAC 3300 3280 3310 3280 3310 3280 3310 3280 3300 3280 3300 3280 3310 3280 3310 3280 <- 16 DACs

FAV3_PED       210        <- board Pedestals, same for all channels
FAV3_CH_PED    0    210   <- channel# and Pedestal_value for this channel
FAV3_ALLCH_PED 210  220  210  215  215  220  220  210  210  215  215  220  220  210  215  220  <- 16 PEDs


FAV3_CONF_FILE  <filename> <- another config filename to be processed on next iteration


*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include "faV3Config.h"
#include "jvme.h"
#include "faV3Lib.h"

#include "config_defs.h"

static int active;

#define NBOARD     21
static int          nfadc;                        /* Number of FADC250s */
static FAV3_CONF faV3[NBOARD+1];

static char *expid = NULL;

int
faV3Config(char *fname)
{
  int res;
  char string[10]; /*dummy, will not be used*/

  /* faInit() must be called by now; get the number of boards from there */
  nfadc = faV3GetN();
  printf("%s: nfadc=%d\n", __func__, nfadc);

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
  int ii, jj;

  for(jj=0; jj<NBOARD; jj++)
    {
      faV3[jj].mode      = 1;
      faV3[jj].compression = 0;
      faV3[jj].vxsReadout = 0;
      faV3[jj].winOffset = 300;
      faV3[jj].winWidth  = 100;
      faV3[jj].nsa       = 6;
      faV3[jj].nsb       = 3;
      faV3[jj].npeak     = 1;
      faV3[jj].chDisMask = 0x0;

      for(ii=0; ii<NCHAN; ii++)
	{
	  faV3[jj].thr[ii] = 110;
	  faV3[jj].dac[ii] = 3300;
	  faV3[jj].ped[ii] = 0.;
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

      SCAN_MASK("FAV3_TRIG_MASK", faV3[slot].trigMask, slot_min, slot_max);

      SCAN_INT("FAV3_TRIG_NSAT", faV3[slot].trig_nsat, slot_min, slot_max);
      SCAN_INT("FAV3_TRIG_NSB", faV3[slot].trig_nsb, slot_min, slot_max);
      SCAN_INT("FAV3_TRIG_NSA", faV3[slot].trig_nsa, slot_min, slot_max);

      SCAN_INT("FAV3_TRIG_THR", faV3[slot].trig_thr, slot_min, slot_max);

      SCAN_INT_ALL("FAV3_READ_THR", faV3[slot].read_thr, slot_min, slot_max);
      SCAN_INT_CH("FAV3_CH_READ_THR", faV3[slot].read_thr, slot_min, slot_max);
      SCAN_INT_ALLCH("FAV3_ALLCH_READ_THR", faV3[slot].read_thr, slot_min, slot_max);

      SCAN_INT_ALL("FAV3_DAC", faV3[slot].dac, slot_min, slot_max);
      SCAN_INT_CH("FAV3_CH_DAC", faV3[slot].dac, slot_min, slot_max);
      SCAN_INT_ALLCH("FAV3_ALLCH_DAC", faV3[slot].dac, slot_min, slot_max);

      SCAN_INT("FAV3_BUSY",  faV3[slot].busy, slot_min, slot_max);
      SCAN_INT("FAV3_STOP",  faV3[slot].stop, slot_min, slot_max);

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
  int slot, ii, jj;
  float ped;

  printf("\n\nfadc250DownloadAll reached, nfadc=%d\n",nfadc);
  for(jj=0; jj<nfadc; jj++)
    {
      slot = faV3Slot(jj);

      faV3SetProcMode(slot,
		      faV3[slot].mode,
		      faV3[slot].winOffset,
		      faV3[slot].winWidth,
		      faV3[slot].nsb,
		      faV3[slot].nsa,
		      faV3[slot].npeak);

      faV3SetChanDisableMask(slot, faV3[slot].chDisMask);


      faV3SetCompression(slot,faV3[slot].compression);

      faV3SetVXSReadout(slot,faV3[slot].vxsReadout);


      for(ii=0; ii<NCHAN; ii++)
	{
	  faV3DACSet(slot, ii, faV3[slot].dac[ii]);

	  ped = faV3[slot].ped[ii] * (float)(faV3[slot].nsa+faV3[slot].nsb);
	  faV3SetChannelPedestal(slot, ii, (int)ped);

	  /* if threshold=0, don't add pedestal since user is disabling zero suppression */
	  if(faV3[slot].thr[ii] > 0)
	    faV3SetChannelThreshold(slot, ii, ((int)faV3[slot].ped[ii])+faV3[slot].thr[ii]);
	  else
	    faV3SetChannelThreshold(slot, ii, 0);
	}
    }

  return(0);
}

/* upload setting from all found FADCs */
int
faV3UploadAll(char *string, int length)
{
  int slot, i, jj, len1, len2;
  char *str, sss[1024];
  uint32_t adcChanEnabled;

  for(jj=0; jj<nfadc; jj++)
    {
      slot = faV3Slot(jj);

      faV3GetProcMode(slot,
		      &faV3[slot].mode,
		      &faV3[slot].winOffset,
		      &faV3[slot].winWidth,
		      &faV3[slot].nsb,
		      &faV3[slot].nsa,
		      &faV3[slot].npeak);

      faV3[slot].chDisMask = faV3GetChanDisableMask(slot);

      faV3[slot].compression = faV3GetCompression(slot);

      faV3[slot].vxsReadout = faV3GetVXSReadout(slot);

      for(i=0;i<FAV3_MAX_ADC_CHANNELS;i++)
	{
	  faV3DACGet(slot, i, &faV3[slot].dac[i]);

	  faV3[slot].ped[i] = faV3GetChannelPedestal(slot, i);
	  faV3[slot].ped[i] = ((float)faV3[slot].ped[i])/(faV3[slot].nsa+faV3[slot].nsb); /* go back from integral to amplitude */

	  faV3[slot].thr[i] = faV3GetChannelThreshold(slot, i);
	  if(faV3[slot].thr[i] > 0)
	    {
	      faV3[slot].thr[i] = faV3[slot].thr[i] - (int)faV3[slot].ped[i]; /* MUST SUBTRACT PEDESTAL TO BE CONSISTENT WITH DOWNLOADED THRESHOLD */
	    }
	}
    }


  if(length)
    {
      str = string;
      str[0] = '\0';

      for(jj=0; jj<nfadc; jj++)
	{
	  slot = faV3Slot(jj);

	  sprintf(sss,"FAV3_SLOT %d\n",slot);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_MODE %d\n",      faV3[slot].mode);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_COMPRESSION %d\n", faV3[slot].compression);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_VXSREADOUT %d\n", faV3[slot].vxsReadout);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_W_OFFSET %d\n",  faV3[slot].winOffset*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_W_WIDTH  %d\n",  faV3[slot].winWidth*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_NSA %d\n",       faV3[slot].nsa*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_NSB %d\n",       faV3[slot].nsb*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_NPEAK %d\n",     faV3[slot].npeak);
	  ADD_TO_STRING;

	  adcChanEnabled = 0xFFFF^faV3[slot].chDisMask;
	  sprintf(sss,"FAV3_ADC_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(adcChanEnabled>>(15-i))&0x1);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_ALLCH_DAC");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",faV3[slot].dac[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_ALLCH_PED");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %7.3f", faV3[slot].ped[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FAV3_ALLCH_TET");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",faV3[slot].thr[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;
	}

      CLOSE_STRING;
    }

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

void
faV3PrintConf(int slot){

  int ch;


  printf("\n===================================================\n");

  printf("Slot  =  %d \n", slot);
  printf(" FAV3_MODE      =  %d  \n", faV3[slot].mode);
  printf(" FAV3_W_OFFSET  =  %d  \n", faV3[slot].winOffset);
  printf(" FAV3_W_WIDTH   =  %d  \n", faV3[slot].winWidth);
  printf(" FAV3_NSB       =  %d  \n", faV3[slot].nsb);
  printf(" FAV3_NSA       =  %d  \n", faV3[slot].nsa);
  printf(" FAV3_NPEAK     =  %d  \n", faV3[slot].npeak);
  printf(" FAV3_NSAT      =  %d  \n", faV3[slot].nsat);
  printf(" \n");
  printf(" FAV3_NPED      =  %d  \n", faV3[slot].nped);
  printf(" FAV3_MAX_PED   =  %d  \n", faV3[slot].max_ped);
  printf(" \n");
  printf(" FAV3_BUSY      =  %d  \n", faV3[slot].busy);
  printf(" FAV3_STOP      =  %d  \n", faV3[slot].stop);
  printf(" \n");
  printf(" FAV3_FORMAT    =  %d  \n", faV3[slot].data_format);
  printf(" \n");
  printf(" FAV3_ADC_MASK  =  0x%x  \n", faV3[slot].chDisMask);
  printf(" \n");

  printf(" FAV3_DAC  =  ");
  for(ch = 0; ch <  NCHAN; ch++)
    printf("  %d", faV3[slot].dac[ch]);
  printf("\n");


  printf(" FAV3_TET  =  ");
  for(ch = 0; ch <  NCHAN; ch++)
    printf("  %d", faV3[slot].read_thr[ch]);
  printf("\n");


  printf("\n");

  printf(" FAV3_TRIG_THR     =  %d  \n",   faV3[slot].trig_thr);
  printf(" FAV3_TRIG_NSB     =  %d  \n",   faV3[slot].trig_nsb);
  printf(" FAV3_TRIG_NSA     =  %d  \n",   faV3[slot].trig_nsa);
  printf(" FAV3_TRIG_NSAT    =  %d  \n",   faV3[slot].trig_nsat);
  printf(" FAV3_TRIG_MASK    =  0x%x  \n", faV3[slot].trigMask);

  printf(" FAV3_TRIG_BL  =  ");
  for(ch = 0; ch <  NCHAN; ch++)
    printf("  %d", faV3[slot].trig_bl[ch]);
  printf("\n");

  printf("\n");


}
