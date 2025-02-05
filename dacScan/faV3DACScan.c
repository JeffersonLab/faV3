/*
 * File:
 *    faV3DACScan.c
 *
 * Description:
 *    Test scanning the DAC for each channel
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

int32_t
faV3HallDDACScan(int32_t id, uint16_t dac_value[NDAC], uint16_t channel_data[NDAC][16])
{
  int32_t ichan = 0, idac = 0, rval = OK;

  /* Setup the sample monitor, 4 samples with max 12 bits */
  if(faV3HallDSampleConfig(id, 4, 0x3ff) != OK)
    {
      printf("%s(id = %d): ERROR from faV3HallDSampleConfig\n", __func__, id);
      return ERROR;
    }

  for(idac = 0; idac < NDAC; idac++)
    {
      dac_value[idac] = 100 + (idac * 100);
      for(ichan = 0; ichan < 16; ichan++)
	{
	  faV3DACSet(id, ichan, dac_value[idac]);
	}
      taskDelay(1);
      faV3HallDReadAllChannelSamples(id, channel_data[idac]);
    }


  return rval;
}

int
main(int argc, char *argv[])
{
  char config_filename[256] = "./dacScan.cfg";
  int32_t user_slotnumber = -1;

  /* grab filename using arguments */
  progName = argv[0];
  if(argc == 2)
    {
      user_slotnumber = atoi(argv[1]);
      if((user_slotnumber < 3) || (user_slotnumber > 21))
	{
	  printf("%s: Invalid slotnumber (%d)\n",
		 progName, user_slotnumber);
	  Usage();
	  exit(-1);
	}
    }

  faV3InitGlobals();
  faV3ReadConfigFile(config_filename);

  int status;
  status = vmeOpenDefaultWindows();
  if(status != OK)
    goto CLOSE;

  vmeCheckMutexHealth(1);
  vmeBusLock();

  extern int32_t nfaV3;

  uint32_t vme_addr = 3 << 19;
  int32_t ninit = 18;

  if(user_slotnumber > 0)
    {
      vme_addr = user_slotnumber << 19;
      ninit = 1;
    }

  faV3HallDInit(vme_addr, 1<<19, ninit, 0);
  if(nfaV3 <= 0)
    goto CLOSE;

  faV3DownloadAll();

  char serial_number[16];
  uint16_t channel_data[NDAC][16];
  uint16_t dac_value[NDAC];
  char output_filename[256];

  int id, ifa;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);

      faV3HallDDACScan(id, dac_value, channel_data);
      faV3GetSerialNumber(id, (char **)&serial_number);

      sprintf(output_filename, "output/slot%d_%s.txt", id, serial_number);

      FILE *outfp = fopen(output_filename, "w");
      if(outfp == NULL)
	{
	  perror("fopen");
	  goto CLOSE;
	}

      int idata = 0, ichan = 0;
      fprintf(outfp, "# %s \n", serial_number);

      fprintf(outfp, "# Ch/DAC 0    1    2    3    4    5    6    7    8    9   10   11   12   13   14   15\n");
      for(idata = 0; idata < NDAC; idata++)
	{
	  fprintf(outfp, "%4d  ", dac_value[idata]);
	  for(ichan = 0; ichan < 16; ichan ++)
	    fprintf(outfp, "%4d ", (channel_data[idata][ichan] & 0x3fff) >> 2);

	  fprintf(outfp, "\n");
	}
      fprintf(outfp, "\n");
      fclose(outfp);
      printf("File saved: %s\n", output_filename);
    }



 CLOSE:
  vmeBusUnlock();
  vmeCloseDefaultWindows();

  exit(0);
}

/*
  Local Variables:
  compile-command: "make -k faV3DACScan "
  End:
*/
