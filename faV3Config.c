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

static int active;

#define NBOARD     21
static int          nfadc;                        /* Number of FADC250s */
static FAV3_CONF faV3[NBOARD+1];


#define SCAN_MSK						\
  args = sscanf (str_tmp, "%*s %d %d %d %d %d %d %d %d   \
                                     %d %d %d %d %d %d %d %d",	\
		 &msk[ 0], &msk[ 1], &msk[ 2], &msk[ 3],	\
		 &msk[ 4], &msk[ 5], &msk[ 6], &msk[ 7],	\
		 &msk[ 8], &msk[ 9], &msk[10], &msk[11],	\
		 &msk[12], &msk[13], &msk[14], &msk[15])

#define SCAN_FMSK						\
  args = sscanf (str_tmp, "%*s %f %f %f %f %f %f %f %f   \
                                     %f %f %f %f %f %f %f %f",	\
		 &fmsk[ 0], &fmsk[ 1], &fmsk[ 2], &fmsk[ 3],	\
		 &fmsk[ 4], &fmsk[ 5], &fmsk[ 6], &fmsk[ 7],	\
		 &fmsk[ 8], &fmsk[ 9], &fmsk[10], &fmsk[11],	\
		 &fmsk[12], &fmsk[13], &fmsk[14], &fmsk[15])

#define GET_READ_MSK							\
  SCAN_MSK;								\
  if(args != 16)							\
    {									\
      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args); \
      return(-8);							\
    }									\
  ui1 = 0;								\
  for(jj=0; jj<NCHAN; jj++)						\
    {									\
      if((msk[jj] < 0) || (msk[jj] > 1))				\
	{								\
	  CFG_ERR("Invalid mask bit value, %d\n", msk[jj]);		\
	  return(-6);							\
	}								\
      if(strcmp(keyword,"FAV3_ADC_MASK") == 0) msk[jj] = ~(msk[jj])&0x1; \
      ui1 |= (msk[jj]<<jj);						\
    }

/* Macros for error ouput */
#define CFG_ERR(format, ...) {						\
    fprintf(stdout, "\n%s: ERROR: ", "ReadConfigFile");			\
    if(slot1==0)							\
      fprintf(stdout, "ALL SLOTS: ");					\
    else								\
      fprintf(stdout, "SLOT %d: ", slot1);				\
    fprintf(stdout, "%s\n\t", keyword);					\
    fprintf(stdout, format, ## __VA_ARGS__);				\
    fprintf(stdout, "\n");						\
  }



static char *expid = NULL;

void
fadc250SetExpid(char *string)
{
  expid = strdup(string);
}

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

/* to set host externally */
static char hosthost[1024];
void
faV3Sethost(char *host)
{
  strcpy(hosthost,host);
}

/* reading and parsing config file */
int
faV3ReadConfigFile(char *filename_in)
{
  FILE   *fd;
  char   filename[FNLEN];
  char   fname[FNLEN] = { "" };  /* config file name */
  int    ii, jj, ch;
  char   str_tmp[STRLEN], str2[STRLEN], keyword[ROCLEN];
  char   host[ROCLEN], ROC_name[ROCLEN];
  int    args, i1, msk[16];
  int    slot, slot1, slot2, chan;
  uint32_t  ui1;
  float f1, fmsk[16];
  /*char *getenv();*/
  char *envDir = NULL;
  int do_parsing;

#ifndef OFFLINE
  gethostname(host,ROCLEN);  /* obtain our hostname */
#else
  strcpy(host,hosthost);
#endif

#ifdef FAV3_CONFIG_GET_ENV
  envDir = getenv(FAV3_CONFIG_GET_ENV);
  if(envDir == NULL)
    {
      strcpy((char *)str_tmp,"./");
      envDir = (char *)str_tmp;
      printf("%s: INFO: %s not found. Using %s\n",
	     __func__,FAV3_CONFIG_GET_ENV,envDir);
    }
  else
    {
      printf("%s: FADC250-V3 Config Environment Variable:\n"
	     " %s = %s\n",
	     __func__,
	     FAV3_CONFIG_GET_ENV, envDir);
    }
#else
  strcpy((char *)str_tmp,"./");
  envDir = (char *)str_tmp;
#endif

  if(expid==NULL)
    {
      expid = getenv("EXPID");
      printf("\nNOTE: use EXPID=>%s< from environment\n",expid);
    }
  else
    {
      printf("\nNOTE: use EXPID=>%s< from CODA\n",expid);
    }

  strcpy(filename,filename_in); /* copy filename from parameter list to local string */
  do_parsing = 1;

  while(do_parsing)
    {
      if(strlen(filename)!=0) /* filename specified */
	{
	  if ( filename[0]=='/' || (filename[0]=='.' && filename[1]=='/') )
	    {
	      sprintf(fname, "%s", filename);
	    }
	  else
	    {
	      sprintf(fname, "%s/fav3/%s", envDir, filename);
	    }

	  if((fd=fopen(fname,"r")) == NULL)
	    {
	      printf("\nReadConfigFile: Can't open config file >%s<\n",fname);
	      return(-1);
	    }
	}
      else if(do_parsing<2) /* filename does not specified */
	{
	  sprintf(fname, "%s/fav3/%s.cnf", envDir, host);
	  if((fd=fopen(fname,"r")) == NULL)
	    {
	      sprintf(fname, "%s/fav3/%s.cnf", envDir, expid);
	      if((fd=fopen(fname,"r")) == NULL)
		{
		  printf("\nReadConfigFile: Can't open config file >%s<\n",fname);
		  return(-2);
		}
	    }
	}
      else
	{
	  printf("\nReadConfigFile: ERROR: since do_parsing=%d (>1), filename must be specified\n",do_parsing);
	  return(-1);
	}

      printf("\nReadConfigFile: Using configuration file >%s<\n",fname);

      /* Parsing of config file */
      active = 0; /* by default disable crate */
      do_parsing = 0; /* will parse only one file specified above, unless it changed during parsing */
      while ((ch = getc(fd)) != EOF)
	{
	  if ( ch == '#' || ch == ' ' || ch == '\t' )
	    {
	      while ( getc(fd)!='\n' /*&& getc(fd)!=!= EOF*/ ) {} /*ERROR !!!*/
	    }
	  else if( ch == '\n' ) {}
	  else
	    {
	      ungetc(ch,fd);
	      fgets(str_tmp, STRLEN, fd);
	      sscanf (str_tmp, "%s %s", keyword, ROC_name);
#ifdef DEBUG
	      printf("\nfgets returns %s so keyword=%s\n\n",str_tmp,keyword);
#endif

	      /* Start parsing real config inputs */
	      if(strcmp(keyword,"FAV3_CRATE") == 0)
		{
		  if(strcmp(ROC_name,host) == 0)
		    {
		      printf("\nReadConfigFile: crate = %s  host = %s - activated\n",ROC_name,host);
		      active = 1;
		    }
		  else if(strcmp(ROC_name,"all") == 0)
		    {
		      printf("\nReadConfigFile: crate = %s  host = %s - activated\n",ROC_name,host);
		      active = 1;
		    }
		  else
		    {
		      printf("\nReadConfigFile: crate = %s  host = %s - disactivated\n",ROC_name,host);
		      active = 0;
		    }
		}

	      else if(active && (strcmp(keyword,"FAV3_CONF_FILE")==0))
		{
		  sscanf (str_tmp, "%*s %s", str2);
		  /*printf("str2=%s\n",str2);*/
		  strcpy(filename,str2);
		  do_parsing = 2;
		}

	      else if(active && ((strcmp(keyword,"FAV3_SLOT")==0) || (strcmp(keyword,"FAV3_SLOTS")==0)))
		{
		  sscanf (str_tmp, "%*s %s", str2);
		  /*printf("str2=%s\n",str2);*/
		  if(isdigit(str2[0]))
		    {
		      slot1 = atoi(str2);
		      slot2 = slot1 + 1;
		      if(slot1<2 && slot1>21)
			{
			  printf("\nReadConfigFile: ERROR: Invalid slot number %d\n\n",slot1);
			  return(-4);
			}
		    }
		  else if(!strcmp(str2,"all"))
		    {
		      slot1 = 0;
		      slot2 = NBOARD;
		    }
		  else
		    {
		      printf("\nReadConfigFile: ERROR: Invalid slot >%s<, must be 'all' or actual slot number\n\n",str2);
		      return(-4);
		    }
		  /*printf("slot1=%d slot2=%d\n",slot1,slot2);*/
		}

	      else if(active && (strcmp(keyword,"FAV3_MODE") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].mode = i1;
		}

	      else if(active && (strcmp(keyword,"FAV3_COMPRESSION") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].compression = i1;
		}

	      else if(active && (strcmp(keyword,"FAV3_VXSREADOUT") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].vxsReadout = i1;
		}

	      else if(active && (strcmp(keyword,"FAV3_W_OFFSET") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].winOffset = i1/4;
		}

	      else if(active && (strcmp(keyword,"FAV3_W_WIDTH") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  i1 = i1/4; /* convert ns to samples */
#ifdef HALLB
		  i1 = ((i1+15)/16)*16; /* round up to 16 samples */
#endif
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].winWidth = i1;
		}

	      else if(active && (strcmp(keyword,"FAV3_NSA") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].nsa = i1/4;
		}

	      else if(active && (strcmp(keyword,"FAV3_NSB") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].nsb = i1/4;
		}

	      else if(active && (strcmp(keyword,"FAV3_NPEAK") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].npeak = i1;
		}

	      else if(active && (strcmp(keyword,"FAV3_ADC_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].chDisMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}

	      else if(active && (strcmp(keyword,"FAV3_TET") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &ui1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].thr[ii] = ui1;
		}

	      else if(active && (strcmp(keyword,"FAV3_CH_TET") == 0))
		{
		  sscanf (str_tmp, "%*s %d %d", &chan, &ui1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].thr[chan] = ui1;
		}

	      else if(active && (strcmp(keyword,"FAV3_ALLCH_TET") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].thr[ii] = msk[ii];
		}

	      else if(active && (strcmp(keyword,"FAV3_DAC") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &ui1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].dac[ii] = ui1;
		}

	      else if(active && (strcmp(keyword,"FAV3_CH_DAC") == 0))
		{
		  sscanf (str_tmp, "%*s %d %d", &chan, &ui1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].dac[chan] = ui1;
		}

	      else if(active && (strcmp(keyword,"FAV3_ALLCH_DAC") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].dac[ii] = msk[ii];
		}

	      else if(active && (strcmp(keyword,"FAV3_PED") == 0))
		{
		  sscanf (str_tmp, "%*s %f", &f1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].ped[ii] = f1;
		}

	      else if(active && (strcmp(keyword,"FAV3_CH_PED") == 0))
		{
		  sscanf (str_tmp, "%*s %d %f", &chan, &f1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) faV3[slot].ped[chan] = f1;
		}

	      else if(active && (strcmp(keyword,"FAV3_ALLCH_PED") == 0))
		{
		  SCAN_FMSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) faV3[slot].ped[ii] = fmsk[ii];

		}

	      else if(active)
		{
		  printf("Error: FADC250 unknown line: fgets returns %s so keyword=%s\n\n",str_tmp,keyword);
		}

	    }
	}
      fclose(fd);
    }


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
