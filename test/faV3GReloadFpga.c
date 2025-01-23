/*
 * File:
 *    faV3GReloadFpga.c
 *
 * Description:
 *    Reload the specified FPGA(s) for all fADC250 V3s found in the crate
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

int  TestReady(int id, int n_try, int pFlag);
void
Usage()
{
  printf("\n");
  printf("%s\n", progName);
  printf("\n");
}

int
main(int argc, char *argv[])
{
  int stat = 0, rval = OK, iFlag = 0, pFlag = 0;
  extern int nfaV3;
  int ifadc = 0, id = 0, user_choice = 0, fpga_choice = 0, doBoth = 0;

  progName = argv[0];

  printf("\nfaV3250-V2 FPGA Reload\n");
  printf
    ("--------------------------------------------------------------------------------\n\n");

  vmeSetQuietFlag(1);
  stat = vmeOpenDefaultWindows();


  if (argc != 1)
    {
      printf(" ERROR: Must specify no arguments\n");
      Usage();
      goto CLOSE;
    }

  vmeCheckMutexHealth(10);
  vmeBusLock();


  iFlag = FAV3_INIT_SKIP | FAV3_INIT_SKIP_FIRMWARE_CHECK;
  stat = faV3Init((3<<19) , (1<<19), 20, iFlag);

  if(nfaV3 < 0)
    {
      printf(" ERROR: Unable to initialize FADCs.\n");
      vmeBusUnlock();
      goto CLOSE;
    }

  printf("REBOOT FPGA\n");
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      faV3FirmwareReboot(id);
    }

  sleep(1);
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      printf(" %2d: ", id);
      fflush(stdout);
      if(faV3FirmwareWaitForReboot(id, 60000, pFlag) < OK) /* Wait til it's done */
	{
	  printf("%2d: ERROR: Timeout after FPGA %d Reboot\n",
		 id, fpga_choice);
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


int
TestReady(int id, int n_try, int pFlag)
{
  int ii;
  int result;
  unsigned int value = 0;

  result = ERROR;

  for(ii = 0; ii < n_try; ii++)	/* poll for ready bit */
    {
      taskDelay(1);		/* wait */

      value = vmeRead32(&FAV3p[id]->prom_reg1);


      if( value == 0xFFFFFFFF)
	continue;

      if(value & FAV3_PROMREG1_READY)
	{
	  result = OK;
	  break;
	}
    }

  if(pFlag)
    {
      if( ii == n_try )		/* failed to detect ready asserted */
	printf("%s: FADC %2d NOT READY after %d wait cycles (1/60 sec)\n",
	       __FUNCTION__,id,n_try);
      else
	printf("%s: FADC %2d READY after %d wait cycles (1/60 sec)\n",
	       __FUNCTION__,id,(ii + 1));
    }

  return result;
}
