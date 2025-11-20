/*
 * File:
 *    faReadoutTest.c
 *
 * Description:
 *    Test readingout out the fADC250 v3 with a pipeline TI as the
 *     trigger source.
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "tiLib.h"
#include "tiConfig.h"
#include "sdLib.h"
#include "faV3Lib.h"
#include "faV3Config.h"

/* Event Buffer definitions */
#define MAX_EVENT_POOL     100
#define MAX_EVENT_LENGTH   1024*2000      /* Size in Bytes */
DMA_MEM_ID vmeIN,vmeOUT;
int nfaV3 = 0;
int MAXFADCWORDS = 0;
int blockLevel = 1;

/* Interrupt Service routine */
void
mytiISR(int arg)
{
  int printout = 1;
  int ifa = 0, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roType = 2, roCount = 0, blockError = 0;
  uint32_t tiIntCount = tiGetIntCount();
  extern DMANODE *the_event;
  extern unsigned int *dma_dabufp;

  GETEVENT(vmeIN,tiIntCount);
  //  faV3Trig(0);
  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      //      dma_dabufp += dCnt;
    }

  /* Readout FADC */
  /* Mask of initialized modules */
  scanmask = faV3ScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faV3GBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      if(nfaV3 == 1)
	roType = 1;   /* otherwise roType = 2   multiboard reaodut with token passing */
      nwords = faV3ReadBlock(0, dma_dabufp, 4*MAXFADCWORDS, roType);

      /* Check for ERROR in block read */
      blockError = faV3GetBlockError(1);

      if(blockError)
	{
	  printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		 faV3Slot(ifa), roCount, nwords);

	  for(ifa = 0; ifa < nfaV3; ifa++)
	    faV3ResetToken(faV3Slot(ifa));

	  if(nwords > 0)
	    dma_dabufp += nwords;
	}
      else
	{
	  dma_dabufp += nwords;
	  faV3ResetToken(faV3Slot(0));
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  PUTEVENT(vmeOUT);

  DMANODE *outEvent = dmaPGetItem(vmeOUT);
  if(tiIntCount%printout==0)
    {
      printf("Received %d triggers...\n",
	     tiIntCount);
      int idata;
      for(idata=0; idata < outEvent->length; idata++)
	{
	  faV3DataDecode(bswap_32(outEvent->data[idata]));
	}
      printf("\n\n");
    }
  dmaPFreeItem(outEvent);

}


int
main(int argc, char *argv[]) {

  int stat;
  char ti_configfile[1024] = "ti-master.ini",
    fav3_configfile[1024] = "faV3.cfg";

  if(argc == 2) {
    strncpy(fav3_configfile, argv[1], 1024);
  }


  printf("\n fadc250 v3 readout test\n");

  printf("----------------------------\n");

  vmeSetQuietFlag(1);
  vmeOpen();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* INIT dmaPList */

  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",MAX_EVENT_LENGTH,MAX_EVENT_POOL,0);
  vmeOUT = dmaPCreate("vmeOUT",0,0,0);

  dmaPStatsAll();

  dmaPReInitAll();

  tiConfigInitGlobals();

  tiSetFiberLatencyOffset_preInit(0x20);
  tiInit(0,TI_READOUT_EXT_POLL,TI_INIT_SKIP_FIRMWARE_CHECK);
  tiCheckAddresses();

  tiConfig(ti_configfile);
  tiConfigFree();

  stat = tiIntConnect(TI_INT_VEC, mytiISR, 0);
  if (stat != OK)
    {
      printf("ERROR: tiIntConnect failed \n");
      goto CLOSE;
    }
  else
    {
      printf("INFO: Attached TI Interrupt\n");
    }

  int NFAV3 = 16+2;   /* 16 slots + 2 (for the switch slots) */

  /* Setup the iFlag.. flags for FADC initialization */
  int iflag=0;
  iflag |= FAV3_INIT_EXT_SYNCRESET;  /* vxs sync-reset */
  iflag |= FAV3_INIT_VXS_TRIG;       /* VXS trigger source */
  iflag |= FAV3_INIT_INT_CLKSRC;     /* Internal 250MHz Clock source, switch to VXS in prestart */

  iflag |= FAV3_INIT_A32_SLOTNUMBER;

  faV3Init( 3 << 19 , 1 << 19, NFAV3, iflag);
  vmeSetQuietFlag(0);
  nfaV3 = faV3GetN();

  /* Just one FADC250 */
  if(nfaV3 > 1)
    faV3EnableMultiBlock(1);

  /* configure all modules based on config file */
  faV3Config(fav3_configfile);

  int ifa;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faV3EnableBusError(faV3Slot(ifa));
    }

  /* Get the FADC mode and window size to determine max data size */
  int fadc_mode = 0;
  uint32_t pl=0, ptw=0, nsb=0, nsa=0, np=0, nped=0, maxped=0, nsat=0;

  faV3GetProcMode(0, &fadc_mode, &pl, &ptw,
		  &nsb, &nsa, &np);

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfaV3 * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */
  MAXFADCWORDS = nfaV3 * (4 + blockLevel * (4 + 16 * (1 + (ptw / 2))) + 18);

  /***************************************
   *   SD SETUP
   ***************************************/
  sdInit(0);   /* Initialize the SD library */
  sdSetActiveVmeSlots(faV3ScanMask()); /* Tell the sd where to find the fadcs */

  tiClockReset();
  taskDelay(1);
  tiTrigLinkReset();
  taskDelay(1);
  tiEnableVXSSignals();
  taskDelay(1);

  // PRESTART
  /* Set Clock Source to VXS */
  faV3GSetClockSource(2);
  faV3GEnableSyncSrc();

  for(ifa=0; ifa < nfaV3; ifa++)
    {
      faV3SoftReset(faV3Slot(ifa),0);
      faV3ResetToken(faV3Slot(ifa));
      faV3ResetTriggerCount(faV3Slot(ifa));
    }

  taskDelay(1);
  tiSyncReset(1);
  taskDelay(1);

  /* TI Status */
  tiStatus(0);
  sdStatus(0);
  faV3GStatus(0);

  printf("Hit enter to start triggers\n");
  getchar();

  // GO
  /*  Enable FADC */
  faV3GEnable(0);

  tiIntEnable(0);
#define SOFTTRIG
#ifdef SOFTTRIG
  tiSetRandomTrigger(1,0xf);
/*   taskDelay(10); */
/*   tiSoftTrig(1,0x1,0x700,0); */
#endif

  printf("Hit any key to Disable triggers and exit.\n");
  getchar();

#ifdef SOFTTRIG
  /* No more soft triggers */
  /*     tidSoftTrig(0x0,0x8888,0); */
  tiSoftTrig(1,0,0x700,0);
  tiDisableRandomTrigger();
#endif

  tiIntDisable();
  tiIntDisconnect();

  /* FADC Disable */
  faV3GDisable(0);

  tiStatus(0);
  faV3GStatus(0);


 CLOSE:

  /* faV3GReset(1); */
  dmaPFreeAll();
  vmeCloseDefaultWindows();

  exit(0);
}
