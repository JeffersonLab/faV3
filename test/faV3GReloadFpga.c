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

char *progName;
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];

int  TestReady(int id, int n_try, int pFlag);
void
Usage()
{
  printf("\n");
  printf("%s <FPGA Number>\n", progName);
  printf("\n");
  printf("     FPGA Number = 0 for both FPGAs\n");
  printf("                   1 for Control FPGA\n");
  printf("                   2 for Processing FPGA\n");
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


  if (argc != 2)
    {
      printf(" ERROR: Must specify one argument\n");
      Usage();
      goto CLOSE;
    }
  else
    {
      user_choice = (int) strtol(argv[1], NULL, 10);
    }

  if((user_choice < 0) || (user_choice > 2))
    {
      printf(" ERROR: Invalid FPGA Number (%d)\n",
	     fpga_choice);
      Usage();
      goto CLOSE;
    }

  switch(user_choice)
    {
    case 0:
      doBoth = 1;
      fpga_choice = FAV3_FIRMWARE_FX70T;
      break;

    case 1:
      fpga_choice = FAV3_FIRMWARE_FX70T;
      break;

    case 2:
      fpga_choice = FAV3_FIRMWARE_LX110;
      break;

    }


  vmeCheckMutexHealth(10);
  vmeBusLock();


  iFlag = FAV3_INIT_SKIP | FAV3_INIT_SKIP_FIRMWARE_CHECK;
  stat = faInit((3<<19) , (1<<19), 20, iFlag);

  if(nfaV3 < 0)
    {
      printf(" ERROR: Unable to initialize FADCs.\n");
      vmeBusUnlock();
      goto CLOSE;
    }

  const char *fpga_names[2] =
    {
      "LX110 (Processing FPGA)",
      "FX70T (Control FPGA)"
    };

 RELOAD:
  printf("Reloading %s: ", fpga_names[fpga_choice]);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faSlot(ifadc);
      printf(" %2d ", id);
      fflush(stdout);

      if(fpga_choice == FAV3_FIRMWARE_LX110)
	{
	  vmeWrite32(&FAV3p[id]->prom_reg1,FAV3_PROMREG1_REBOOT_FPGA1);
	  if(doBoth)
	    doBoth = 0;
	}
      else if (fpga_choice == FAV3_FIRMWARE_FX70T)
	{
	  vmeWrite32(&FAV3p[id]->prom_reg1,FAV3_PROMREG1_REBOOT_FPGA2);
	}
    }

  taskDelay(1);
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faSlot(ifadc);
      if(TestReady(id, 60000, pFlag) != OK) /* Wait til it's done */
	{
	  printf("%2d: ERROR: Timeout after FPGA %d Reboot\n",
		 id, fpga_choice);
	  rval = ERROR;
	}
    }

  printf("\n");

  if(doBoth)
    {
      fpga_choice = FAV3_FIRMWARE_LX110;
      goto RELOAD;
    }

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
