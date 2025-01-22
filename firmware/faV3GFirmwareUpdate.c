/*
 * File:
 *    faV3GFirmwareUpdate
 *
 * Description:
 *    JLab Flash250 V3 firmware updating
 *     Multiple boards, using slot number addressing convention to ID faV3 in crate
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3FirmwareTools.h"

#define FADC_ADDR (3<<19)
#define NFADC     16
#define SKIPSS

extern int nfaV3;
char *progName;

void Usage();

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

  int32_t status;
  char *fw_filename;
  char inputchar[16];
  int ifa = 0;
  uint32_t cfw = 0;

  printf("\nJLAB fav3 firmware update\n");
  printf("----------------------------\n");

  progName = argv[0];

  if(argc < 2)
    {
      printf(" ERROR: Must specify one argument\n");
      Usage();
      exit(-1);
    }
  else
    {
      fw_filename = argv[1];
    }

  vmeSetQuietFlag(1);
  status = vmeOpenDefaultWindows();

  vmeCheckMutexHealth(10);
  vmeBusLock();
  int iFlag = (1 << 18);	// Skip firmware check

#ifdef SKIPSS
  faV3Init((uint32_t) (FADC_ADDR), (1 << 19), NFADC + 2, iFlag);
#else
  faV3Init((uint32_t) (FADC_ADDR), (1 << 19), NFADC, iFlag);
#endif

  if(nfaV3 == 0)
    {
      printf(" Unable to initialize any FADCs.\n");
      goto CLOSE;
    }

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      cfw = faV3GetFirmwareVersions(faV3Slot(ifa), 0);
      printf("%2d: Control Firmware Version: 0x%04x   Proc Firmware Version: 0x%04x\n",
	     faV3Slot(ifa), cfw & 0xFFFF, (cfw >> 16) & 0xFFFF);
    }

  printf(" Will update firmware with file: \n   %s\n", fw_filename);

 REPEAT2:
  printf(" Press y and <ENTER> to continue... n or q and <ENTER> to quit without update\n");

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

  if(faV3FirmwareReadFile(fw_filename) != OK)
    goto CLOSE;

  faV3FirmwareGLoad(0);

 CLOSE:

  vmeBusUnlock();
  status = vmeCloseDefaultWindows();
  if(status != OK)
    {
      printf("vmeCloseDefaultWindows failed: code 0x%08x\n", status);
      return -1;
    }

  exit(0);
}


void
Usage()
{
  printf("\n");
  printf("%s <firmware file>\n", progName);
  printf("\n");

}
