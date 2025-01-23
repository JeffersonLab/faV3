/*
 * File:
 *    faV3ReloadFpga.c
 *
 * Description:
 *    Reload the specified FPGA(s) for the specified fADC250 V3
 *
 *
 */


#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3FirmwareTools.h"

char *progName;
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];


void
Usage()
{
  printf("\n");
  printf("%s <FADC VME ADDRESS>\n", progName);
  printf("\n");
}

int
main(int argc, char *argv[])
{
  int stat = 0, rval = OK, iFlag = 0, pFlag = 0;
  extern int nfaV3;
  uint32_t fadc_address = 0;
  int ifadc = 0;

  progName = argv[0];

  printf("\nfADC250-V3 FPGA Reload\n");
  printf
    ("--------------------------------------------------------------------------------\n\n");

  vmeSetQuietFlag(1);
  stat = vmeOpenDefaultWindows();

  if (argc != 2)
    {
      printf(" ERROR: Must specify one argument\n");
      Usage();
      goto CLOSE;
    }
  else
    {
      fadc_address = (uint32_t) strtoll(argv[1], NULL, 16) & 0xffffffff;
    }


  vmeCheckMutexHealth(10);
  vmeBusLock();


  iFlag = FAV3_INIT_SKIP | FAV3_INIT_SKIP_FIRMWARE_CHECK;
  stat = faV3Init(fadc_address, 0, 1, iFlag);

  if(nfaV3 < 0)
    {
      printf(" ERROR: Unable to initialize FAV3s.\n");
      vmeBusUnlock();
      goto CLOSE;
    }

  int32_t id = 0;
  printf("REBOOT FPGA\n");
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      printf(" %2d: ", id);
      fflush(stdout);
      faV3FirmwareReboot(id);
    }

  sleep(1);
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(faV3FirmwareWaitForReboot(id, 60000, pFlag) < OK) /* Wait til it's done */
	{
	  printf("%2d: ERROR: Timeout after FPGA Reboot\n",
		 id);
	  rval = ERROR;
	}
    }

  printf("\n");

  vmeBusUnlock();

 CLOSE:

  vmeCloseDefaultWindows();
  printf("\n");
  printf
    ("--------------------------------------------------------------------------------\n");


  exit(0);
}
