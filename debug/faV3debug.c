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
#include "toBin.h"
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3-Compton.h"
#include "faV3Config.h"

#define NDAC 40

char progName[256];
char serial_number[16];
int32_t FAV3_SLOT = 0;
int32_t prog_nsamples = 0;
char config_filename[256] = "./debug.cfg";
const char *filecheck = "%faV3debug";

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
  int32_t iflag = FAV3_INIT_SKIP_FIRMWARE_CHECK;

  vmeSetQuietFlag(1);
  faV3ComptonInit(vme_addr, 1<<19, ninit, iflag);
  if(nfaV3 <= 0)
    {
      printf("%s: ERROR: Initialization returned %d\n", progName, nfaV3);
      return -1;
    }

  FAV3_SLOT = faV3Slot(0);
  faV3GetSerialNumber(FAV3_SLOT, (char **)&serial_number);

  faV3EnableSoftSync(FAV3_SLOT);
  faV3EnableSoftTrig(FAV3_SLOT);
  faV3ComptonGStatus(0);


  return 0;
}

int32_t
config(char *choice)
{
  faV3Config(config_filename);

  faV3ComptonGStatus(0);
  return 0;
}

int32_t
status(char *choice)
{
  faV3ComptonGStatus(0);
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


int32_t
hard_reset(char *choice)
{
  if(faV3Reset(FAV3_SLOT, 1) != OK)
    {
      printf("%s: ERROR from faV3Reset\n",
	     __func__);
      return -1;
    }
  printf("%s: done\n\n", __func__);

  return 0;
}

int32_t
soft_reset(char *choice)
{
  if(faV3SoftReset(FAV3_SLOT, 0) != OK)
    {
      printf("%s: ERROR from faV3SoftReset(0) - soft reset\n",
	     __func__);
      return -1;
    }
  printf("%s: done\n\n", __func__);

  return 0;
}

int32_t
soft_clear(char *choice)
{
  if(faV3SoftReset(FAV3_SLOT, 1) != OK)
    {
      printf("%s: ERROR from faV3SoftReset(1) - soft clear\n",
	     __func__);
      return -1;
    }
  printf("%s: done\n\n", __func__);

  return 0;
}

int32_t
syncreset(char *choice)
{
  if(faV3Sync(FAV3_SLOT) != OK)
    {
      printf("%s: ERROR from faV3Sync\n",
	     __func__);
      return -1;
    }

  printf("%s: done\n\n", __func__);

  return 0;
}

int32_t
enable(char *choice)
{
  if(faV3Enable(FAV3_SLOT, 0) != OK)
    {
      printf("%s: ERROR from faV3Enable\n",
	     __func__);
      return -1;
    }

  printf("%s: done\n\n", __func__);
  return 0;
}

int32_t
disable(char *choice)
{
  if(faV3Disable(FAV3_SLOT, 0) != OK)
    {
      printf("%s: ERROR from faV3Disable\n",
	     __func__);
      return -1;
    }
  printf("%s: done\n\n", __func__);
  return 0;
}

int32_t
trigger(char *choice)
{
  if(faV3Trig(FAV3_SLOT) != OK)
    {
      printf("%s: ERROR from faV3Trig\n",
	     __func__);
      return -1;
    }
  printf("%s: done\n\n", __func__);
  return 0;
}

extern int nfaV3;
extern int faV3ID[FAV3_MAX_BOARDS];
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];
extern volatile faV3_compton_adc_t *COMPTONp[(FAV3_MAX_BOARDS + 1)];

int32_t
read16(char *choice)
{
  uint32_t offset = 0, value = 0;
  char arg1[256], arg2[256];
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];


  if(strlen(choice) > 0)
    {
      offset = strtoll(choice, NULL, 16);
      local_reg = local_reg_base + offset;
    }
  else
    {
      printf(" Usage:\n%s <reg offset>\n", __func__);
      return -1;
    }

  value = vmeRead16((volatile uint16_t *)local_reg);

  printf("  %s:  0x%04x := 0x%04X\n",
	 __func__,
	 offset, value);

  return 0;
}

int32_t
write16(char *choice)
{
  uint32_t offset = 0, value = 0;
  char arg1[256], arg2[256];
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];


  if(strlen(choice) > 0)
    {
      sscanf(choice, "%s %s", arg1, arg2);
      offset = strtoll(arg1, NULL, 16);
      value = strtoll(arg2, NULL, 16);
      local_reg = local_reg_base + offset;
    }
  else
    {
      printf(" Usage:\n%s <reg offset> <value>\n", __func__);
      return -1;
    }

  vmeWrite16((volatile uint16_t *)local_reg, (uint16_t)value);

  printf("  %s:  0x%04x <- 0x%04X\n",
	 __func__,
	 offset, value);

  return 0;
}

int32_t
read32(char *choice)
{
  uint32_t offset = 0, value = 0;
  char arg1[256], arg2[256];
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];


  if(strlen(choice) > 0)
    {
      offset = strtoll(choice, NULL, 16);
      local_reg = local_reg_base + offset;
    }
  else
    {
      printf(" Usage:\n%s <reg offset>\n", __func__);
      return -1;
    }

  value = vmeRead32((volatile uint32_t *)local_reg);

  printf("  %s:  0x%04x := 0x%08X\n",
	 __func__,
	 offset, value);

  return 0;
}

int32_t
write32(char *choice)
{
  uint32_t offset = 0, value = 0;
  char arg1[256], arg2[256];
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];


  if(strlen(choice) > 0)
    {
      sscanf(choice, "%s %s", arg1, arg2);
      offset = strtoll(arg1, NULL, 16);
      value = strtoll(arg2, NULL, 16);
      local_reg = local_reg_base + offset;
    }
  else
    {
      printf(" Usage:\n%s <reg offset> <value>\n", __func__);
      return -1;
    }

  vmeWrite32((volatile uint32_t *)local_reg, (uint32_t)value);

  printf("  %s:  0x%04x <- 0x%08X\n",
	 __func__,
	 offset, value);

  return 0;
}

int32_t
show32(char *choice)
{
  uint32_t offset = 0;
  uint32_t value = 0;
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];

  printf("%s\n\n", __func__);
  int32_t ireg = 0;
  for(ireg = 0x0; ireg < 0x100; ireg += 0x4)
    {
      if((ireg % 0x10)==0) printf("\n0x%04X: ", ireg);

      local_reg = local_reg_base + ireg;
      value = vmeRead32((volatile uint32_t *)local_reg);
      printf("  0x%08X", value);
    }
  printf("\n\n");

  return 0;
}

int32_t
show16(char *choice)
{
  uint32_t offset = 0;
  uint16_t value = 0;
  unsigned long local_reg = 0,
    local_reg_base = (unsigned long) FAV3p[FAV3_SLOT];

  printf("%s\n\n", __func__);
  int32_t ireg = 0;
  for(ireg = 0x100; ireg < 0x200; ireg += 0x2)
    {
      if((ireg % 0x10)==0) printf("\n0x%04X: ", ireg);

      local_reg = local_reg_base + ireg;
      value = vmeRead16((volatile uint16_t *)local_reg);
      printf("  0x%04X", value);
    }
  printf("\n\n");

  return 0;
}

int32_t
readout(char *choice)
{
  FILE *outFile;
  if(strlen(choice) > 0)
    {
      outFile = fopen(choice, "w");
    }
  else
    {
      printf(" Usage: %s <data filename>\n", __func__);
      return -1;
    }

  if(!outFile) {
    perror("fopen");
    return -1;
  }

  uint32_t data[1024];
  uint32_t nwords;

  nwords = faV3ReadBlock(faV3Slot(0), data, 1024, 0);
  faV3ResetToken(faV3Slot(0));

  fprintf(outFile, "%s\n", filecheck);

  int32_t idata;
  for(idata = 0; idata < nwords; idata++) {
    fprintf(outFile, "0x%08x\n", bswap_32(data[idata]));
  }

  printf(" Wrote %d bytes to %s\n", nwords*4, choice);
  fclose(outFile);

  return 0;
}

int32_t
decode(char *choice)
{
  FILE *outFile;
  if(strlen(choice) > 0)
    {
      outFile = fopen(choice, "r");
    }
  else
    {
      printf(" Usage: %s <data filename>\n", __func__);
      return -1;
    }

  if(!outFile) {
    perror("fopen");
    return -1;
  }

  uint32_t data = 0, i = 0;
  char testname[256];
  fscanf(outFile, "%s\n", testname);
  if(strcmp(testname, filecheck) != 0) {
    printf("%s: ERROR: Invalid file header >%s<\n",
	   __func__, testname);
    return -1;
  }

  while(fscanf(outFile, "0x%08x\n", &data) > 0) {
    faV3ComptonDataDecode(data);
  }

  fclose(outFile);

  return 0;
}

int32_t
showbin(char *choice)
{
  unsigned int input = 0;
  if(strlen(choice) > 0)
    {
      input = (unsigned int) strtoll(choice,NULL,16);
    }
  else
    {
      printf(" Usage: showbin <hex number>\n");
      return -1;
    }

  print_bit((struct b_field *) &input);
  print_hex(input);

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
  {"help", com_help, "Display Commands:\t help <command>"},
  {"?", com_help, "Synonym for `help'\n"},
  {"init", init, "Initialize module:\t init <slotnumber>"},
  {"config", config, "Configure module with debug.cfg"},
  {"status", status, "Print status of initialized modules\n"},
  {"setdac", setdac, "Set DAC for Channel:\t setdac <channel> <dac value>"},
  {"getdac", getdac, "Print DAC values for all channels"},
  {"hard_reset", hard_reset, "Hard Reset"},
  {"soft_reset", soft_reset, "Soft Reset"},
  {"soft_clear", soft_clear, "Soft Clear"},
  {"syncreset", syncreset, "Generate Soft SyncReset"},
  {"enable", enable, "Enable Trigger Source"},
  {"disable", disable, "Disable Trigger Source"},
  {"trigger", trigger, "Generate Soft Trigger"},
  {"readout", readout, "Readout Event:\t readout <filename>"},
  {"decode", decode, "Decode Event:\t\t decode <filename>\n"},
  {"read16", read16, "16bit Read:\t\t read16 <reg offset>"},
  {"write16", write16, "16bit Write:\t\t write16 <reg offset> <value>"},
  {"read32", read32, "32bit Read:\t\t read32 <reg offset>"},
  {"write32", write32, "32bit Write:\t\t write32 <reg offset> <value>\n"},
  {"show16", show16, "Show values of 16bit registers"},
  {"show32", show32, "Show values of 32bit registers\n"},
  {"showbin", showbin, "Show the bits:\t showbin <hex>"},
  {"quit", com_quit, "Quit"},
  {(char *) NULL, (rl_icpfunc_t *) NULL, (char *) NULL}
};
#include "readline_menu.h"

int
main(int argc, char *argv[])
{
  int32_t user_slotnumber = -1;

  strncpy(progName, argv[0], 256);

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

  printf("quit\n");

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faV3debug "
  End:
*/
