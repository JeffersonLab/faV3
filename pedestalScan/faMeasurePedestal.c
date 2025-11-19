/*
 * File:
 *    faMeasurePedestal.c
 *
 * Description:
 *    JLab Flash ADC pedestal measurement
 *
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "jvme.h"
#include "faV3Lib.h"
#include "faV3Config.h"

#define FADC_ADDR (3<<19)
#define NFAV3     16

#ifndef FNLEN
#define FNLEN 256
#endif

extern int nfaV3;
char *progName;

void Usage();

int
main(int argc, char *argv[])
{
  int status;
  char configFilename[FNLEN] = "", pedestalFilename[FNLEN] = "";
  int inputchar = 10;
  int ch, ifa = 0;
  unsigned int cfw = 0;
  FILE *f;
  faV3Ped ped;

  char myhostname[128];
  gethostname(myhostname, 128);

  printf("\nJLAB fadc pedestal measurement on host %s\n", myhostname);
  printf("----------------------------\n");

  progName = argv[0];

  switch (argc)
    {
    case 2:
      strncpy(pedestalFilename, argv[1], FNLEN);
      break;

    case 3:
      strncpy(configFilename, argv[1], FNLEN);
      strncpy(pedestalFilename, argv[2], FNLEN);
      break;

    default:
      Usage();
      exit(-1);
    }

  status = vmeOpenDefaultWindows();
  if(status != OK)
    goto CLOSE;

  /* Sync Source */
  int iFlag = FAV3_INIT_SOFT_SYNCRESET;
  /* Trigger Source */
  iFlag |= FAV3_INIT_SOFT_TRIG;
  /* Clock Source */
  iFlag |= FAV3_INIT_INT_CLKSRC;

  vmeBusLock();
  faV3Init((unsigned int) (FADC_ADDR), (1 << 19), NFAV3 + 2, iFlag);

  faV3Config(configFilename);

  if(nfaV3 == 0)
    {
      printf(" Unable to initialize any FADCs.\n");
      vmeBusUnlock();
      goto CLOSE;
    }

  f = fopen(pedestalFilename, "wt");

  if(f)
    {
      fprintf(f, "FAV3_CRATE %s\n", myhostname);
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  fprintf(f, "FAV3_SLOT %d\nFAV3_ALLCH_PED", faV3Slot(ifa));

	  for(ch = 0; ch < 16; ch++)
	    {
	      if(faV3MeasureChannelPedestal(faV3Slot(ifa), ch, &ped) != OK)
		{
		  printf(" Unable to measure pedestal on slot %d, ch %d...\n",
			 faV3Slot(ifa), ch);
		  fclose(f);
		  vmeBusUnlock();
		  goto CLOSE;
		}
	      fprintf(f, " %8.3f", ped.avg);
	    }
	  fprintf(f, "\n");
	}
      fprintf(f, "FAV3_CRATE end\n");

      fclose(f);
      printf("Wrote to : %s\n", pedestalFilename);
    }
  else
    {
      perror("fopen");
      printf(" Unable to open pedestal file %s\n", pedestalFilename);
    }

  vmeBusUnlock();

CLOSE:

  vmeCloseDefaultWindows();
  exit(0);
}


void
Usage()
{
  printf("\n");
  printf("%s <pedestal filename>\n", progName);
  printf("    OR\n");
  printf("%s <config filename> <pedestal filename>\n", progName);
  printf("\n");
}
