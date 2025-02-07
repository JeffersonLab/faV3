/*
 * File:
 *    faV3debug.c
 *
 * Description:
 *    Evolving program to debug various features of the fadc250 V3
 *
 *
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-HallD.h"
#include "faV3Config.h"

#define NDAC 40

char *progName;
char serial_number[16];
int32_t FAV3_SLOT = 0;

void
Usage()
{
  printf("Usage:\n");
  printf("\n");
  printf(" %s <slotnumber>\n", progName);
  printf("     <slotnumber>      Slot number to scan.\n");
  printf("                       If not specified, scan entire crate\n");
  printf("\n");
  printf("\n");
}

// initalialize the fav3 library with a provided slot number
int32_t
init(char *choice)
{
  int32_t user_slotnumber = 0;
  uint32_t vme_addr = 0;
  int32_t ninit = 1;

  printf("\n");

  if(strlen(choice) > 0)
    {
      sscanf(choice, "%d", &user_slotnumber);
    }
  else
    {
      printf(" Slot number: ");
      fflush(stdout);
      scanf("%d", &user_slotnumber);
    }

  if(user_slotnumber == 0)
    {
      vme_addr = 3 << 19;
      ninit = 18;
    }
  else if((user_slotnumber < 3) || (user_slotnumber > 21))
    {
      printf("%s: Invalid slotnumber (%d)\n",
	     progName, user_slotnumber);
      return -1;
    }
  else
    vme_addr = user_slotnumber << 19;

  extern int32_t nfaV3;
  int32_t iflag = 0;
  iflag |= FAV3_INIT_SKIP_IDELAY_CONFIG;
  faV3HallDInit(vme_addr, 1<<19, ninit, iflag);
  if(nfaV3 <= 0)
    {
      printf("%s: ERROR: Initialization returned %d\n", progName, nfaV3);
      return -1;
    }
  faV3HallDGStatus(0);
  FAV3_SLOT = faV3Slot(0);
  faV3GetSerialNumber(FAV3_SLOT, (char **)&serial_number);


  return 0;
}

int32_t
status(char *choice)
{
  faV3HallDGStatus(0);
  return 0;
}

int32_t
idelay(char *choice)
{
  printf("\n");

  if(faV3LoadIdelay(FAV3_SLOT, 1) != OK)
    {
      printf("%s(%d): ERROR from faV3LoadDelay\n",
	     __func__, FAV3_SLOT);
      return -1;
    }

  return 0;
}


// set the DAC for the specified channel
int32_t
setdac(char *choice)
{
  int32_t dac_value = 0;
  int32_t channel_number = 0;

  printf("\n");
  if(strlen(choice) > 0)
    {
      sscanf(choice, "%d %d", &channel_number, &dac_value);
    }
  else
    {
      printf(" %s: channel number [1, 16] (0 for all): ", __func__);
      fflush(stdout);
      scanf("%d", &channel_number);

      printf(" %s: dac value [1, 4095]: ", __func__);
      fflush(stdout);
      scanf("%d", &dac_value);
    }

  int32_t ichan = 0, nchan = 16;
  if(channel_number == 0)
    {
      channel_number = 1;
      nchan = 16;
    }
  else
    nchan = channel_number;

  for(ichan = (channel_number-1); ichan < nchan; ichan++)
    {
      if(faV3DACSet(FAV3_SLOT, ichan, (uint16_t)dac_value) != OK)
	{
	  printf("%s(%d): ERROR from faV3DACSet\n",
		 __func__, FAV3_SLOT);
	  return -1;
	}
    }

  return 0;
}

// printout the DAC for each channel
int32_t
getdac(char *choice)
{
  int32_t ichan = 0;
  uint32_t dac_value[16];

  printf("\n");

  for(ichan = 0; ichan < 16; ichan++)
    {
      if(faV3DACGet(FAV3_SLOT, ichan, &dac_value[ichan]) != OK)
	{
	  printf("%s: ERROR from faV3DACGet\n",
		 __func__);
	  return -1;
	}
    }

  printf("# Slot %2d: %s \n", FAV3_SLOT, serial_number);

  ichan = 0;
  printf("Ch 1: %4d\n", dac_value[ichan]);
  for(ichan = 1; ichan < 16; ichan ++)
    printf("  %2d: %4d\n", ichan+1, dac_value[ichan]);

  printf("\n");

  return 0;
}

// configure the pedestal monitor parameters
int32_t
setped(char *choice)
{
  int32_t nsamples = 0, maxped = 0;

  if(strlen(choice) > 0)
    {
      sscanf(choice, "%d %d", &nsamples, &maxped);
    }
  else
    {
      printf(" number of samples [4, 15]: ");
      fflush(stdout);
      scanf("%d", &nsamples);

      printf("  maximum pedestal [0, 1023]: ");
      fflush(stdout);
      scanf("%d", &maxped);
    }

  if(faV3HallDSampleConfig(FAV3_SLOT, nsamples, maxped) != OK)
    {
      printf("%s: ERROR from faV3HallDSampleConfig\n", __func__);
      return ERROR;
    }

  return 0;
}

// readback and print pedestal monitor
int32_t
getped(char *choice)
{
  uint16_t channel_data[16];
  if(faV3HallDReadAllChannelSamples(FAV3_SLOT, channel_data) <= 0)
    {
      printf("%s: ERROR from faV3HallDReadAllChannelSamples\n",
	     __func__);
      return -1;
    }

  int ichan = 0;
  printf("# Slot %2d: %s \n", FAV3_SLOT, serial_number);

  printf("Ch 1: %4d\n", (channel_data[ichan] & 0x3fff) >> 2);
  for(ichan = 1; ichan < 16; ichan ++)
    printf("  %2d: %4d\n", ichan+1, (channel_data[ichan] & 0x3fff) >> 2);

  printf("\n");

  return 0;
}


#include <readline/readline.h>
int com_quit(char *arg);
int com_help(char *arg);

typedef struct
{
  char *name;			/* User printable name of the function. */
  rl_icpfunc_t *func;		/* Function to call to do the job. */
  char *doc;			/* Documentation for this function.  */
} COMMAND;

COMMAND commands[] = {
  {"help", com_help, "Display this text"},
  {"?", com_help, "Synonym for `help'"},
  {"init", init, "Initialize module: init <slotnumber>"},
  {"status", status, "Print status of initialized modules"},
  {"idelay", idelay, "Configure IDelay"},
  {"setdac", setdac, "Set DAC for Channel: setdac <channel> <dac value>"},
  {"getdac", getdac, "Print DAC values for all channels"},
  {"setped", setped, "Set Pedestal Monitor parameters: setped <nsamples> <maxped>"},
  {"getped", getped, "Print out Pedestal Monitor"},
  {"quit", com_quit, "Quit"},
  {(char *) NULL, (rl_icpfunc_t *) NULL, (char *) NULL}
};
#include "readline_menu.h"


int
main(int argc, char *argv[])
{
  char config_filename[256] = "./dacScan.cfg";
  int32_t user_slotnumber = -1;

  progName = argv[0];

  faV3InitGlobals();
  faV3ReadConfigFile(config_filename);

  int status;
  status = vmeOpenDefaultWindows();
  if(status != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);

  init("0");

  initialize_readline(progName);	/* Bind our completer. */
  com_help("");
  strcat(progName, ": ");
  readline_menu_loop(progName);


 CLOSE:
  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faV3DACScan "
  End:
*/
