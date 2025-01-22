/*
 * File:
 *    faV3FirmwareUpdate
 *
 * Description:
 *    JLab Flash250 V3 firmware updating
 *     Single board
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <getopt.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3FirmwareTools.h"

char *progName;

void
usage()
{
  printf("\n");
  printf("%s <option> <firmware file> <FAV3 VME ADDRESS>\n", progName);
  printf("\n");
  printf("\n");
  printf(" options:\n");
  printf("     -p                  program <firmware file> to ROM (-v included)\n");
  printf("     -s                  save ROM to <firmware file> (not compatible with -p, -v)\n");
  printf("     -v                  verify/compare ROM with <firmware file>\n");
  printf("     -y                  assume 'yes' to all prompts\n");
  printf("\n");

}

int32_t
filecheck(char *filename)
{
  int32_t rval = 0;
  struct stat file_stat;
  int32_t status;

  status = stat(filename, &file_stat);
  if(status < 0)
    {
      rval = 0;
    }
  else if(file_stat.st_size > 0)
    {
      rval = 1;
    }

  return rval;
}

int
main(int argc, char *argv[])
{

  int32_t stat = 0;
  char *fw_filename;
  uint32_t fadc_address = 0;
  char inputchar[16];

  printf("\nJLAB fav3 firmware update\n");
  printf("----------------------------\n");

  progName = argv[0];

  int32_t program = 0, save = 0, verify = 0, force = 0, opt;

  while ((opt = getopt(argc, argv, "psvy")) != -1) {
    switch (opt) {
    case 'p':
      program = 1;
      break;
    case 's':
      save = 1;
      break;
    case 'v':
      verify = 1;
      break;
    case 'y':
      force = 1;
      break;
    default: /* '?' */
      usage();
      exit(EXIT_FAILURE);
    }
  }

  if ((optind + 2) != argc) {
    usage();
    exit(EXIT_FAILURE);
  }

  fw_filename = argv[optind++];
  fadc_address = (uint32_t) strtoll(argv[optind++], NULL, 16) & 0xffffffff;

  vmeSetQuietFlag(1);
  stat = vmeOpenDefaultWindows();

  if(stat < 0)
    {
      printf(" Unable to initialize VME driver\n");
      exit(-1);
    }

  vmeCheckMutexHealth(10);
  vmeBusLock();

  int iFlag = (1 << 18);	/* Do not perform firmware check */
  stat = faV3Init(fadc_address, 0x0, 1, iFlag);
  if(stat < 0)
    {
      printf(" Unable to initialize faV3.\n");
      goto CLOSE;
    }

  if(program)
    {
      printf("Update ROM with file:  %s\n", fw_filename);
      printf(" for FADC250 V3 with VME address = 0x%08x\n", fadc_address);
      if(filecheck(fw_filename) != 1)
	{
	  printf("ERROR: %s file not found\n", fw_filename);
	  goto CLOSE;
	}
    }

  if(save)
    {
      printf("Save ROM to file:  %s\n", fw_filename);
      printf(" for FADC250 V3 with VME address = 0x%08x\n", fadc_address);
      if(filecheck(fw_filename) == 1)
	{
	  printf("WARNING: File exists.\n");
	  if(!force)
	    {
	    REPEAT1:
	      printf(" Press y and <ENTER> to overwrite... n or q and <ENTER> to quit\n");

	      scanf("%s", (char *) inputchar);

	      if((strcmp(inputchar, "q") == 0) || (strcmp(inputchar, "Q") == 0) ||
		 (strcmp(inputchar, "n") == 0) || (strcmp(inputchar, "N") == 0))
		{
		  goto CLOSE;
		}
	      else if((strcmp(inputchar, "y") == 0) || (strcmp(inputchar, "Y") == 0))
		{
		}
	      else
		goto REPEAT1;
	    }
	}
    }

  if(verify)
    {
      printf("Verify ROM with file:  %s\n", fw_filename);
      printf(" for FADC250 V3 with VME address = 0x%08x\n", fadc_address);
      if(filecheck(fw_filename) != 1)
	{
	  printf("ERROR: %s file not found\n", fw_filename);
	  goto CLOSE;
	}
    }

 REPEAT2:
  if(!force)
    {
      printf(" Press y and <ENTER> to continue... n or q and <ENTER> to quit\n");

      scanf("%s", (char *) inputchar);

      if((strcmp(inputchar, "q") == 0) || (strcmp(inputchar, "Q") == 0) ||
	 (strcmp(inputchar, "n") == 0) || (strcmp(inputchar, "N") == 0))
	{
	  printf(" Exiting without update\n");
	  goto CLOSE;
	}
      else if((strcmp(inputchar, "y") == 0) || (strcmp(inputchar, "Y") == 0))
	{
	}
      else
	goto REPEAT2;
    }

  if(program)
    {
      if(faV3FirmwareReadFile(fw_filename) != OK)
	goto CLOSE;

      if(faV3FirmwareLoad(0, 1) != OK)
	goto CLOSE;
    }
  else if(verify)
    {
      if(faV3FirmwareReadFile(fw_filename) != OK)
	goto CLOSE;

      if(faV3FirmwareDownload(0, 1) != OK)
	goto CLOSE;

      if(faV3FirmwareVerify(0, 1) != OK)
	goto CLOSE;
    }
  else if(save)
    {
      if(faV3FirmwareDownload(0, 1) != OK)
	goto CLOSE;

      if(faV3FirmwareWriteFile(fw_filename) != OK)
	goto CLOSE;
    }

 CLOSE:

  faV3FirmwareDone(0);

  vmeBusUnlock();

  stat = vmeCloseDefaultWindows();
  if(stat != OK)
    {
      printf("vmeCloseDefaultWindows failed: code 0x%08x\n", stat);
      exit(-1);
    }

  exit(0);
}
