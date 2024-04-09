/****************************************************************************
 *
 *  faV3Config.c  -  configuration library file for fADC250 V3 board
 *
 *  SP, 07-Nov-2013
 * Sergey Boyarinov Nov 2013 - simplify/adjust for Hall B
 *
 *  empty lines and line startes with # - will be ignored
 *  config file format:

CRATE             <rocipname>   <- ROC/crate IP name
FADC250_ALLSLOTS             <- just keyword - all settings after this line will be implemented
#                                              for all slots, till FADC250_SLOTS will be met
FADC250_SLOTS     3  8  15   <- slot_numbers - in which next settings will be implemented
#                                              till file ends or next FADC250_SLOTS will be met
FADC250_F_REV     0x02c1     <- firmware revision  (0x0 Bits:7-0)
FADC250_B_REV     0x0a03     <- board revision     (0x0 Bits:15-8)
FADC250_ID        0xfadc     <- board type         (0x0 Bits:31-16)

FADC250_MODE         1   <- process mode: 1-4  (0x10C Bits:2-0)
FADC250_COMPRESSION  0   <- compression mode: 0-uncompressed, 1-compressed, 2-both
FADC250_VXSREADOUT   0   <- readout data through VXS: 0-disable, 1-enable
FADC250_W_OFFSET  50  <- number of ns back from trigger point. (0x120)
#                           (in Manual it is  PL=Trigger_Window(ns) * 250MHz)
FADC250_W_WIDTH   49  <- number of ns to include in trigger window. (0x11C)
#                           (in M:  PTW=Trigger_Window(ns) * 250MHz, minimum is 6)
FADC250_NSB       3   <- number of ns before trigger point to include in data processing. (0x124)
#                           This include the trigger Point. (minimum is 2 in all mode)
FADC250_NSA       6   <- number of ns after trigger point to include in data processing. (0x128)
#                           Minimum is (6 in mode 2) and ( 3 in mode 0 and 1).
#                           Number of sample report is 1 more for odd and 2 more for even NSA number.
FADC250_NPEAK     1   <- number of Pulses in Mode 2 and 3.  (0x10C Bits:6-5)

#                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 - channels ##
FADC250_ADC_MASK  1  0  1  0  1  0  1  0  1  0  1  0  1  0  1  0   <- channel enable mask
FADC250_TRG_MASK  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1   <- trigger enable mask

FADC250_TRG_WIDTH   1  <- stretches pulse width of channel over threshold in 4ns ticks
FADC250_TRG_MINTOT  1  <- minimum number of 4ns clocks channel must be over threshold to count towards multiplicity for FADC
FADC250_TRG_MINMULT 1  <- minimum number of channels triggered simultaneously for FADC to send trigger to SD

FADC250_TET       110        <- board Trigger Energy Threshold (TET), same for all 16 channels
FADC250_CH_TET    0    110   <- channel# and TET_value for this channel
FADC250_ALLCH_TET 111  222  2  3  4  5  6  7  8  9  10  11  12  13  14  15   <- 16 TETs (0x12C - 0x148)

FADC250_DAC       3300       <- board DAC, one and the same for all 16 channels
FADC250_CH_DAC    0    3300  <- channel# and DAC_value for this channel
FADC250_ALLCH_DAC 3300 3280 3310 3280 3310 3280 3310 3280 3300 3280 3300 3280 3310 3280 3310 3280 <- 16 DACs

FADC250_PED       210        <- board Pedestals, same for all channels
FADC250_CH_PED    0    210   <- channel# and Pedestal_value for this channel
FADC250_ALLCH_PED 210  220  210  215  215  220  220  210  210  215  215  220  220  210  215  220  <- 16 PEDs

FADC250_GAIN       0.5        <- board Gains, same for all channels
FADC250_CH_GAIN    0    0.5   <- channel# and Gain_value for this channel
FADC250_ALLCH_GAIN 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5  <- 16 GAINs

FADC250_CH_DELAY    0   0     <- channel# and delay in ns
FADC250_ALLCH_DELAY 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

FADC250_TRG_MODE_MASK 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 <- 0=normal pulse trigger mode, 1=discriminator mode

FADC250_INVERT_MASK 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 <- 0=ADC values not changed, 1=inverted ADC input polarity (i.e. ADC=4095-ADC)

FADC250_SPARSIFICATION 0      <- 0=Bypassed, 1=Enabled
FADC250_ACCUMATOR_SCALER_MODE_MASK 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 <- 0=Default, TET based pulse integration, 1=Sum all samples

FADC250_CONF_FILE  <filename> <- another config filename to be processed on next iteration


 cc -rdynamic -shared -o fadc250Config.so fadc250Config.c -I/home/halld/test_setup/coda/linuxvme/include /home/halld/test_setup/coda/linuxvme/jvme/libjvme.a /home/halld/test_setup/coda/linuxvme/fadcV2/libfadc.a -ldl -lpthread -lrt

*/

#if defined(VXWORKS) || defined(Linux)

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>

#include "faV3Config.h"
#include "jvme.h"
#include "faV3Lib.h"

#undef DEBUG

static int active;

#define NBOARD     21
static int          nfadc;                        /* Number of FADC250s */
static FADC250_CONF fa250[NBOARD+1];


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
      if(strcmp(keyword,"FADC250_ADC_MASK") == 0) msk[jj] = ~(msk[jj])&0x1; \
      ui1 |= (msk[jj]<<jj);						\
    }


static char *expid = NULL;

void
fadc250SetExpid(char *string)
{
  expid = strdup(string);
}



#ifndef OFFLINE

int
faV3Config(char *fname)
{
  int res;
  char string[10]; /*dummy, will not be used*/

  /* faInit() must be called by now; get the number of boards from there */
  nfadc = faV3GetNfadc();
  printf("fadc250Config: nfadc=%d\n",nfadc);

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

#endif /*OFFLINE*/


void
faV3InitGlobals()
{
  int ii, jj;

  printf("%s reached\n", __func__);

  /*nfadc = 0;*/
#ifndef OFFLINE
  nfadc = faV3GetNfadc();
#endif
  for(jj=0; jj<NBOARD; jj++)
    {
      fa250[jj].mode      = 1;
      fa250[jj].compression = 0;
      fa250[jj].vxsReadout = 0;
      fa250[jj].winOffset = 300;
      fa250[jj].winWidth  = 100;
      fa250[jj].nsa       = 6;
      fa250[jj].nsb       = 3;
      fa250[jj].npeak     = 1;
      fa250[jj].chDisMask = 0x0;
      fa250[jj].trigMask  = 0xffff;
      fa250[jj].trigWidth = 0xff;
      fa250[jj].trigMinTOT = 1;
      fa250[jj].trigMinMult = 1;
      fa250[jj].thrIgnoreMask = 0;
      fa250[jj].invertMask = 0;
      fa250[jj].playbackDisableMask = 0;
      fa250[jj].sparsification = 0;
      fa250[jj].accumulatorMask = 0;

      for(ii=0; ii<NCHAN; ii++)
	{
	  fa250[jj].thr[ii] = 110;
	  fa250[jj].dac[ii] = 3300;
	  fa250[jj].ped[ii] = 0.;
	  fa250[jj].gain[ii] = 0.5;
	  fa250[jj].delay[ii] = 0;
          fa250[jj].trigMode[ii] = 0;
	}
    }
}

void
faV3GetParamsForOffline(float ped[6][22][16], int tet[6][22][16], float gain[6][22][16], int nsa[6][22], int nsb[6][22])
{
  int ii,jj;

  for(jj=0; jj<NBOARD; jj++)
    {
      nsa[0][jj] = fa250[jj].nsa;
      nsb[0][jj] = fa250[jj].nsb;

      for(ii=0; ii<NCHAN; ii++)
	{
	  ped[0][jj][ii]  = fa250[jj].ped[ii];
	  tet[0][jj][ii]  = fa250[jj].thr[ii];
	  gain[0][jj][ii] = fa250[jj].gain[ii];
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
  unsigned int  ui1;
  float f1, fmsk[16];
  /*char *getenv();*/
  char *envDir = NULL;
  int do_parsing;

#ifndef OFFLINE
  gethostname(host,ROCLEN);  /* obtain our hostname */
#else
  strcpy(host,hosthost);
#endif

#ifdef FADC250_CONFIG_GET_ENV
  envDir = getenv(FADC250_CONFIG_GET_ENV);
  if(envDir == NULL)
    {
      strcpy((char *)str_tmp,"./");
      envDir = (char *)str_tmp;
      printf("%s: INFO: %s not found. Using %s\n",
	     __func__,FADC250_CONFIG_GET_ENV,envDir);
    }
  else
    {
      printf("%s: FADC250 Config Environment Variable:\n"
	     " %s = %s\n",
	     __func__,
	     FADC250_CONFIG_GET_ENV, envDir);
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
	      sprintf(fname, "%s/fadc250/%s", envDir, filename);
	    }

	  if((fd=fopen(fname,"r")) == NULL)
	    {
	      printf("\nReadConfigFile: Can't open config file >%s<\n",fname);
	      return(-1);
	    }
	}
      else if(do_parsing<2) /* filename does not specified */
	{
	  sprintf(fname, "%s/fadc250/%s.cnf", envDir, host);
	  if((fd=fopen(fname,"r")) == NULL)
	    {
	      sprintf(fname, "%s/fadc250/%s.cnf", envDir, expid);
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

      /* Macros for error ouput */
#define CFG_ERR(format, ...) {						\
	fprintf(stdout, "\n%s: ERROR: ", "ReadConfigFile");		\
	if(slot1==0) fprintf(stdout, "ALL SLOTS: ");			\
	else fprintf(stdout, "SLOT %d: ", slot1);			\
	fprintf(stdout, "%s\n\t", keyword);				\
	fprintf(stdout, format, ## __VA_ARGS__);			\
	fprintf(stdout, "\n");						\
      }


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
	      if(strcmp(keyword,"FADC250_CRATE") == 0)
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

	      else if(active && (strcmp(keyword,"FADC250_CONF_FILE")==0))
		{
		  sscanf (str_tmp, "%*s %s", str2);
		  /*printf("str2=%s\n",str2);*/
		  strcpy(filename,str2);
		  do_parsing = 2;
		}

	      else if(active && ((strcmp(keyword,"FADC250_SLOT")==0) || (strcmp(keyword,"FADC250_SLOTS")==0)))
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

	      else if(active && (strcmp(keyword,"FADC250_MODE") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].mode = i1;
		}

	      else if(active && (strcmp(keyword,"FADC250_COMPRESSION") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].compression = i1;
		}

	      else if(active && (strcmp(keyword,"FADC250_VXSREADOUT") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].vxsReadout = i1;
		}

	      else if(active && (strcmp(keyword,"FADC250_W_OFFSET") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].winOffset = i1/4;
		}

	      else if(active && (strcmp(keyword,"FADC250_W_WIDTH") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  i1 = i1/4; /* convert ns to samples */
#ifdef HALLB
		  i1 = ((i1+15)/16)*16; /* round up to 16 samples */
#endif
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].winWidth = i1;
		}

	      else if(active && (strcmp(keyword,"FADC250_NSA") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].nsa = i1/4;
		}

	      else if(active && (strcmp(keyword,"FADC250_NSB") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].nsb = i1/4;
		}

	      else if(active && (strcmp(keyword,"FADC250_NPEAK") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].npeak = i1;
		}

	      else if(active && (strcmp(keyword,"FADC250_ADC_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].chDisMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}

	      else if(active && (strcmp(keyword,"FADC250_TRG_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].trigMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}
	      else if(active && (strcmp(keyword,"FADC250_TRG_WIDTH") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].trigWidth = i1/4;
		}
	      else if(active && (strcmp(keyword,"FADC250_TRG_MINTOT") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].trigMinTOT = i1;
		}
	      else if(active && (strcmp(keyword,"FADC250_TRG_MINMULT") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].trigMinMult = i1;
		}
	      else if(active && (strcmp(keyword,"FADC250_TET_IGNORE_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].thrIgnoreMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}
	      else if(active && (strcmp(keyword,"FADC250_PLAYBACK_DISABLE_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].playbackDisableMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}
	      else if(active && (strcmp(keyword,"FADC250_INVERT_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].invertMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
		}
	      else if(active && (strcmp(keyword,"FADC250_TRG_MODE_MASK") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].trigMode[ii] = msk[ii];
		}
	      else if(active && (strcmp(keyword,"FADC250_TET") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &ui1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].thr[ii] = ui1;
		}

	      else if(active && (strcmp(keyword,"FADC250_CH_TET") == 0))
		{
		  sscanf (str_tmp, "%*s %d %d", &chan, &ui1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].thr[chan] = ui1;
		}

	      else if(active && (strcmp(keyword,"FADC250_CH_DELAY") == 0))
		{
		  sscanf (str_tmp, "%*s %d %d", &chan, &ui1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].delay[chan] = ui1;
		}

	      else if(active && (strcmp(keyword,"FADC250_ALLCH_DELAY") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].delay[ii] = msk[ii];
		}

	      else if(active && (strcmp(keyword,"FADC250_ALLCH_TET") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].thr[ii] = msk[ii];
		}

	      else if(active && (strcmp(keyword,"FADC250_DAC") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &ui1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].dac[ii] = ui1;
		}

	      else if(active && (strcmp(keyword,"FADC250_CH_DAC") == 0))
		{
		  sscanf (str_tmp, "%*s %d %d", &chan, &ui1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].dac[chan] = ui1;
		}

	      else if(active && (strcmp(keyword,"FADC250_ALLCH_DAC") == 0))
		{
		  SCAN_MSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].dac[ii] = msk[ii];
		}

	      else if(active && (strcmp(keyword,"FADC250_PED") == 0))
		{
		  sscanf (str_tmp, "%*s %f", &f1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].ped[ii] = f1;
		}

	      else if(active && (strcmp(keyword,"FADC250_CH_PED") == 0))
		{
		  sscanf (str_tmp, "%*s %d %f", &chan, &f1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].ped[chan] = f1;
		}

	      else if(active && (strcmp(keyword,"FADC250_ALLCH_PED") == 0))
		{
		  SCAN_FMSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].ped[ii] = fmsk[ii];

		}

	      else if(active && (strcmp(keyword,"FADC250_GAIN") == 0))
		{
		  sscanf (str_tmp, "%*s %f", &f1);
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].gain[ii] = f1;
		}

	      else if(active && (strcmp(keyword,"FADC250_CH_GAIN") == 0))
		{
		  sscanf (str_tmp, "%*s %d %f", &chan, &f1);
		  if((chan<0) || (chan>NCHAN))
		    {
		      CFG_ERR("Invalid channel number %d, %s\n", chan, str_tmp);
		      return(-7);
		    }
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].gain[chan] = f1;
		}

	      else if(active && (strcmp(keyword,"FADC250_ALLCH_GAIN") == 0))
		{
		  SCAN_FMSK;
		  if(args != 16)
		    {
		      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args);
		      return(-8);
		    }
		  for(slot=slot1; slot<slot2; slot++) for(ii=0; ii<NCHAN; ii++) fa250[slot].gain[ii] = fmsk[ii];
		}

	      else if(active && (strcmp(keyword,"FADC250_SPARSIFICATION") == 0))
		{
		  sscanf (str_tmp, "%*s %d", &i1);
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].sparsification = i1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,i1);
#endif
		}

	      else if(active && (strcmp(keyword,"FADC250_ACCUMULATOR_SCALER_MODE_MASK") == 0))
		{
		  GET_READ_MSK;
		  for(slot=slot1; slot<slot2; slot++) fa250[slot].accumulatorMask = ui1;
#ifdef DEBUG
		  printf("\nReadConfigFile: %s = 0x%04x \n",keyword,ui1);
#endif
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




#ifndef OFFLINE


/* download setting into all found FADCs */
int
faV3DownloadAll()
{
  int slot, ii, jj;
  float ped;
#if 0
  int updateThresholds = 1;
#endif

  printf("\n\nfadc250DownloadAll reached, nfadc=%d\n",nfadc);
  for(jj=0; jj<nfadc; jj++)
    {
      slot = faV3Slot(jj);

      faV3SetProcMode(slot,
		    fa250[slot].mode,
		    fa250[slot].winOffset,
		    fa250[slot].winWidth,
		    fa250[slot].nsb,
		    fa250[slot].nsa,
		    fa250[slot].npeak, 0);

      faV3ChanDisable(slot, fa250[slot].chDisMask);

      faV3ThresholdIgnore(slot, fa250[slot].thrIgnoreMask);

      faV3Invert(slot, fa250[slot].invertMask);

      faV3PlaybackDisable(slot, fa250[slot].playbackDisableMask);

      faV3SetHitbitTrigWidth(slot, fa250[slot].trigWidth);

      faV3SetHitbitTrigMask(slot, fa250[slot].trigMask);

      faV3SetHitbitMinTOT(slot, fa250[slot].trigMinTOT);

      faV3SetHitbitMinMultiplicity(slot, fa250[slot].trigMinMult);

      faV3SetCompression(slot,fa250[slot].compression);

      faV3SetVXSReadout(slot,fa250[slot].vxsReadout);

      faV3SetSparsificationMode(slot, fa250[slot].sparsification);

      faV3SetAccumulatorScalerMode(slot, fa250[slot].accumulatorMask);

      for(ii=0; ii<NCHAN; ii++)
	{
	  faV3SetChannelDelay(slot, ii, fa250[slot].delay[ii] / 4);
	  faV3SetDAC(slot, fa250[slot].dac[ii], (1<<ii));
	  faV3SetChannelGain(slot, ii, fa250[slot].gain[ii]);
   	  faV3SetTriggerProcessingMode(slot, ii, fa250[slot].trigMode[ii]);

	  ped = fa250[slot].ped[ii] * (float)(fa250[slot].nsa+fa250[slot].nsb);
	  faV3SetChannelPedestal(slot, ii, (int)ped);

	  /* if threshold=0, don't add pedestal since user is disabling zero suppression */
	  if(fa250[slot].thr[ii] > 0)
	    faV3SetChThreshold(slot, ii, ((int)fa250[slot].ped[ii])+fa250[slot].thr[ii]);
	  else
	    faV3SetChThreshold(slot, ii, 0);
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
  unsigned int adcChanEnabled;

  for(jj=0; jj<nfadc; jj++)
    {
      slot = faV3Slot(jj);

      faV3GetProcMode(slot,
		    &fa250[slot].mode,
		    &fa250[slot].winOffset,
		    &fa250[slot].winWidth,
		    &fa250[slot].nsb,
		    &fa250[slot].nsa,
		    &fa250[slot].npeak);

      fa250[slot].chDisMask = faV3GetChanMask(slot);
      fa250[slot].thrIgnoreMask = faV3GetThresholdIgnoreMask(slot);
      fa250[slot].invertMask = faV3GetInvertMask(slot);
      fa250[slot].playbackDisableMask = faV3GetPlaybackDisableMask(slot);

      fa250[slot].trigWidth = faV3GetHitbitTrigWidth(slot);

      fa250[slot].trigMask = faV3GetHitbitTrigMask(slot);

      fa250[slot].trigMinTOT = faV3GetHitbitMinTOT(slot);

      fa250[slot].trigMinMult = faV3GetHitbitMinMultiplicity(slot);

      fa250[slot].compression = faV3GetCompression(slot);

      fa250[slot].vxsReadout = faV3GetVXSReadout(slot);

      fa250[slot].sparsification = faV3GetSparsificationMode(slot);

      fa250[slot].accumulatorMask = faV3GetAccumulatorScalerMode(slot);

      for(i=0;i<FAV3_MAX_ADC_CHANNELS;i++)
	{
	  fa250[slot].delay[i] = 4*faV3GetChannelDelay(slot, i);
	  fa250[slot].dac[i] = faV3GetChannelDAC(slot, i);

	  fa250[slot].ped[i] = faV3GetChannelPedestal(slot, i);
	  fa250[slot].ped[i] = ((float)fa250[slot].ped[i])/(fa250[slot].nsa+fa250[slot].nsb); /* go back from integral to amplitude */

	  fa250[slot].thr[i] = faV3GetChThreshold(slot, i);
	  if(fa250[slot].thr[i] > 0)
	    {
	      fa250[slot].thr[i] = fa250[slot].thr[i] - (int)fa250[slot].ped[i]; /* MUST SUBTRACT PEDESTAL TO BE CONSISTENT WITH DOWNLOADED THRESHOLD */
	    }
	  fa250[slot].gain[i] = faV3GetChannelGain(slot, i);
          fa250[slot].trigMode[i] = faV3GetTriggerProcessingMode(slot, i);
	}
    }


  if(length)
    {
      str = string;
      str[0] = '\0';

      for(jj=0; jj<nfadc; jj++)
	{
	  slot = faV3Slot(jj);

	  sprintf(sss,"FADC250_SLOT %d\n",slot);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_MODE %d\n",      fa250[slot].mode);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_COMPRESSION %d\n", fa250[slot].compression);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_VXSREADOUT %d\n", fa250[slot].vxsReadout);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_W_OFFSET %d\n",  fa250[slot].winOffset*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_W_WIDTH  %d\n",  fa250[slot].winWidth*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_NSA %d\n",       fa250[slot].nsa*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_NSB %d\n",       fa250[slot].nsb*FAV3_ADC_NS_PER_CLK);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_NPEAK %d\n",     fa250[slot].npeak);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_TRG_MASK %d\n",  fa250[slot].trigMask);
	  ADD_TO_STRING;

	  sprintf(sss, "FADC250_TRG_WIDTH %d\n", fa250[slot].trigWidth);
	  ADD_TO_STRING;

	  sprintf(sss, "FADC250_TRG_MINTOT %d\n", fa250[slot].trigMinTOT);
	  ADD_TO_STRING;

	  sprintf(sss, "FADC250_TRG_MINMULT %d\n", fa250[slot].trigMinMult);
	  ADD_TO_STRING;

	  adcChanEnabled = 0xFFFF^fa250[slot].chDisMask;
	  sprintf(sss,"FADC250_ADC_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(adcChanEnabled>>(15-i))&0x1);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_TET_IGNORE_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(fa250[slot].thrIgnoreMask>>(15-i))&0x1);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_INVERT_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(fa250[slot].invertMask>>(15-i))&0x1);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_PLAYBACK_DISABLE_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(fa250[slot].playbackDisableMask>>(15-i))&0x1);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_TRG_MODE_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",fa250[slot].trigMode[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ALLCH_DAC");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",fa250[slot].dac[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ALLCH_PED");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %7.3f", fa250[slot].ped[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ALLCH_TET");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",fa250[slot].thr[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ALLCH_DELAY");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",fa250[slot].delay[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ALLCH_GAIN");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %7.3f",fa250[slot].gain[i]);
	      ADD_TO_STRING;
	    }
	  sprintf(sss,"\n");
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_SPARSIFICATION %d\n", fa250[slot].sparsification);
	  ADD_TO_STRING;

	  sprintf(sss,"FADC250_ACCUMULATOR_SCALER_MODE_MASK");
	  ADD_TO_STRING;
	  for(i=0; i<16; i++)
	    {
	      sprintf(sss," %d",(fa250[slot].accumulatorMask>>(15-i))&0x1);
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



/* print board registers; if slot is zero, print all boards */
void
faV3Mon(int slot)
{
  int id, start, end, kk;


  nfadc = faV3GetNfadc();
  if(slot==0)
    {
      start = 0;
      end = nfadc;
    }
  else if((id = faV3Id(slot)) >= 0)
    {
      start = id;
      end = start + 1;
    }
  else
    {
      return;
    }

  printf("nfadc=%d\n",nfadc);
  for(kk=start; kk<end; kk++)
    {
      faV3Status(faV3Slot(kk),0);
    }

  return;
}

#endif /*OFFLINE*/




#else /* dummy version*/

void
faV3Config_dummy()
{
  return;
}

#endif
