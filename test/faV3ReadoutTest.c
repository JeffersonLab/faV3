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
extern DMANODE *the_event;
extern unsigned int *dma_dabufp;

#define BLOCKLEVEL 1

/* Interrupt Service routine */
void
mytiISR(int arg)
{
  volatile unsigned short reg;
  int dCnt, len=0,idata;
  DMANODE *outEvent;
  int tibready=0, timeout=0;
  int printout = 1/BLOCKLEVEL;

  unsigned int tiIntCount = tiGetIntCount();

  GETEVENT(vmeIN,tiIntCount);

  dCnt = tiReadBlock(dma_dabufp,3*BLOCKLEVEL+10,1);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }

    /* Readout FADC */
  if(NFADC!=0)
    {
      FA_SLOT = fadcID[0];
      int itime=0, stat=0, roflag=1, islot=0;
      unsigned int gbready=0;
      for(itime=0;itime<10000;itime++)
	{
	  gbready = faGBready();
	  stat = (gbready == fadcSlotMask);
	  if (stat>0)
	    {
	      break;
	    }
	}
      if(stat>0)
	{
	  if(NFADC>1) roflag=2; /* Use token passing scheme to readout all modules */
	  dCnt = faReadBlock(FA_SLOT,dma_dabufp,MAXFADCWORDS,roflag);
	  if(dCnt<=0)
	    {
	      printf("FADC%d: No data or error.  dCnt = %d\n",FA_SLOT,dCnt);
	    }
	  else
	    {
	      if(dCnt>MAXFADCWORDS)
		{
		  printf("%s: WARNING.. faReadBlock returned dCnt >= MAXFADCWORDS (%d >= %d)\n",
			 __FUNCTION__,dCnt, MAXFADCWORDS);
		}
	      else
		dma_dabufp += dCnt;
	    }
	}
      else
	{
	  printf ("FADC%d: no events   stat=%d  intcount = %d   gbready = 0x%08x  fadcSlotMask = 0x%08x\n",
		  FA_SLOT,stat,tiGetIntCount(),gbready,fadcSlotMask);
	}

      /* Reset the Token */
      if(roflag==2)
	{
	  for(islot=0; islot<NFADC; islot++)
	    {
	      FA_SLOT = fadcID[islot];
	      faResetToken(FA_SLOT);
	    }
	}
    }




  PUTEVENT(vmeOUT);

  outEvent = dmaPGetItem(vmeOUT);
  if(tiIntCount%printout==0)
    {
      printf("Received %d triggers...\n",
	     tiIntCount);

      len = outEvent->length;

      for(idata=0;idata<len;idata++)
	{
	  faDataDecode(LSWAP(outEvent->data[idata]));
	}
      printf("\n\n");
    }
  dmaPFreeItem(outEvent);

}


int
main(int argc, char *argv[]) {

  int stat;

  printf("\n fadc250 v3 readout test\n");
  printf("----------------------------\n");

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

  tiA32Base=0x09000000;
  tiSetFiberLatencyOffset_preInit(0x20);
  tiInit(0,TI_READOUT_EXT_POLL,TI_INIT_SKIP_FIRMWARE_CHECK);
  tiCheckAddresses();

  if(argc == 2)
    tiConfig(argv[1]);
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
  int iFlag=0;
  iflag |= FAV3_INIT_EXT_SYNCRESET;  /* vxs sync-reset */
  iflag |= FAV3_INIT_VXS_TRIG;       /* VXS trigger source */
  iflag |= FAV3_INIT_INT_CLKSRC;     /* Internal 250MHz Clock source, switch to VXS in prestart */

  vmeSetQuietFlag(1);
  faV3Init( 3 << 19 , 1 << 19, NFAV3, iflag);
  vmeSetQuietFlag(0);

  /* Just one FADC250 */
  if(nfaV3 == 1)
    faV3DisableMultiBlock();
  else
    faV3EnableMultiBlock(1);

  /* configure all modules based on config file */
  faV3Config(fav3_configfile);

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faV3EnableBusError(faV3Slot(ifa));
    }
  faV3GStatus(0);

  /***************************************
   *   SD SETUP
   ***************************************/
  sdInit(0);   /* Initialize the SD library */
  sdSetActiveVmeSlots(faV3ScanMask()); /* Tell the sd where to find the fadcs */
  sdStatus(0);

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
  faGStatus(0);

  printf("Hit enter to start triggers\n");
  getchar();

  // GO
  /*  Enable FADC */
  faV3GEnable(0);

  tiIntEnable(0);
  tiStatus(0);
#define SOFTTRIG
#ifdef SOFTTRIG
  tiSetRandomTrigger(1,0xf);
/*   taskDelay(10); */
/*   tiSoftTrig(1,0x1,0x700,0); */
#endif

  printf("Hit any key to Disable TID and exit.\n");
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

  /* FADC Event status - Is all data read out */
  tiStatus(0);
  faV3GStatus(0);


 CLOSE:

  dmaPFreeAll();
  vmeCloseDefaultWindows();

  exit(0);
}
