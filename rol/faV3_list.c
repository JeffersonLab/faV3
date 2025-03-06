/*************************************************************************
 *
 *  faV3_list.c - Library of routines for the user to write for
 *                readout and buffering of events from JLab FADC V3 using
 *                a JLab pipeline TI module and Linux VME controller.
 *
 *                In this example, clock, syncreset, and trigger are
 *                output from the TI then distributed using a Front Panel SDC.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1024*60      /* Size in Bytes */

/* Define Interrupt source and address */
#define TI_MASTER
#define TI_READOUT TI_READOUT_EXT_POLL  /* Poll for available data, external triggers */
#define TI_ADDR    (21<<19)          /* GEO slot 21 */

#define FIBER_LATENCY_OFFSET 0x4A  /* measured longest fiber length */

#include "dmaBankTools.h"
#include "tiprimary_list.c" /* source required for CODA */
#include "faV3Lib.h"        /* library of FADC250 routines */
#include "faV3-HallD.h"     /* Hall D firmware */
#include "faV3Config.h"

#define BUFFERLEVEL 1

/* FADC Library Variables */
extern int nfaV3;

/* Number of fadc250 to initialize */
#define NFAV3     1
/* Address of first fADC250 (set to slot# << 19 )*/
#define FADC_ADDR (4 << 19)
/* Increment address to find next fADC250 (increment by 1 slot) */
#define FADC_INCR (1 << 19)

#define FADC_BANK 0x250

#define FAV3_READ_CONF_FILE {			\
    if(rol->usrConfig)				\
      faV3Config(rol->usrConfig);		\
  }

/* for the calculation of maximum data words in the block transfer */
unsigned int MAXFADCWORDS=0;

/* function prototype */
void rocTrigger(int arg);

void
rocDownload()
{
  unsigned short iflag;
  int ifa, stat;

  /* Configure TI */
#ifdef TI_MASTER
  /*
   * Set Trigger source
   *    For the TI-Master, valid sources:
   *      TI_TRIGGER_FPTRG     2  Front Panel "TRG" Input
   *      TI_TRIGGER_TSINPUTS  3  Front Panel "TS" Inputs
   *      TI_TRIGGER_TSREV2    4  Ribbon cable from Legacy TS module
   *      TI_TRIGGER_PULSER    5  TI Internal Pulser (Fixed rate and/or random)
   */
  tiSetTriggerSource(TI_TRIGGER_TSINPUTS); /* TS Inputs enabled */

  /* Enable set specific TS input bits (1-6) */
  tiEnableTSInput( TI_TSINPUT_1 | TI_TSINPUT_2 );

  /* Load the trigger table that associates
   *    - TS#1,2,3,4,5,6 : Physics trigger,
   */
  tiLoadTriggerTable(3);

  tiSetTriggerHoldoff(1,10,0);
  tiSetTriggerHoldoff(2,10,0);

  /* Set the SyncReset width to 4 microSeconds */
  tiSetSyncResetType(1);

  /* Set initial number of events per block */
  tiSetBlockLevel(blockLevel);

  /* Set Trigger Buffer Level */
  tiSetBlockBufferLevel(BUFFERLEVEL);

  /* Sync event every 1000 blocks */
  tiSetSyncEventInterval(1000);

  /* Set L1A prescale ... rate/(x+1) */
  tiSetPrescale(0);

  /* Set TS input #1 prescale rate/(2^(x-1) + 1)*/
  tiSetInputPrescale(1, 0);
#endif

  /* Add trigger latch pattern to datastream */
  tiSetFPInputReadout(1);

  /* Program/Init FADC Modules Here */
  iflag = 0xea00; /* SDC Board address */
  iflag |= FAV3_INIT_EXT_SYNCRESET;  /* Front panel sync-reset */
  iflag |= FAV3_INIT_FP_TRIG;  /* Front Panel Input trigger source */
  iflag |= FAV3_INIT_FP_CLKSRC;  /* Internal 250MHz Clock source */

  extern uint32_t faV3A32Base;
  faV3A32Base = 0x09000000;

  vmeSetQuietFlag(1);
  faV3Init(FADC_ADDR, FADC_INCR, NFAV3, iflag);
  vmeSetQuietFlag(0);

  /* configure all modules based on config file */
  FAV3_READ_CONF_FILE;

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      /* Bus errors to terminate block transfers (preferred) */
      faV3EnableBusError(faV3Slot(ifa));
    }

  tiStatus(0);
  faV3SDC_Status(0);
  faV3GStatus(0);

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  int ifa;

  /* Program/Init VME Modules Here */
  faV3GEnableSyncSrc();
  for(ifa=0; ifa < nfaV3; ifa++)
    {
      faV3SoftReset(faV3Slot(ifa),0);
      faV3ResetTriggerCount(faV3Slot(ifa));
    }

  /* Set number of events per block (broadcasted to all connected TI Slaves)*/
  tiSetBlockLevel(blockLevel);
  printf("rocPrestart: Block Level set to %d\n",blockLevel);

  /* Sync Reset to synchronize TI and fadc250 timestamps and their internal buffers */
  faV3SDC_Sync();

  tiStatus(0);
  faV3GStatus(0);

  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{

  /* Get the current block level */
  blockLevel = tiGetCurrentBlockLevel();
  printf("%s: Current Block Level = %d\n",
	 __FUNCTION__,blockLevel);

  faV3GSetBlockLevel(blockLevel);

  int fadc_mode = 0;
  uint32_t pl=0, ptw=0, nsb=0, nsa=0, np=0, nped=0, maxped=0, nsat=0;
  faV3HallDGetProcMode(0, &fadc_mode, &pl, &ptw,
		       &nsb, &nsa, &np,
		       &nped, &maxped, &nsat);

  /* Set Max words from fadc (proc mode == 1 produces the most)
     nfaV3 * ( Block Header + Trailer + 2  # 2 possible filler words
               blockLevel * ( Event Header + Header2 + Timestamp1 + Timestamp2 +
	                      nchan * (Channel Header + (WindowSize / 2) )
             ) +
     scaler readout # 16 channels + header/trailer
   */
  MAXFADCWORDS = nfaV3 * (4 + blockLevel * (4 + 16 * (1 + (ptw / 2))) + 18);

  /*  Enable FADC */
  faV3GEnable(0);

  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{

  /* FADC Disable */
  faV3GDisable(0);

  /* FADC Event status - Is all data read out */
  faV3GStatus(0);

  tiStatus(0);

  printf("rocEnd: Ended after %d events\n",tiGetIntCount());

}

void
rocTrigger(int arg)
{
  int ifa, stat, nwords, dCnt;
  unsigned int datascan, scanmask;
  int roCount = 0, blockError = 0;

  roCount = tiGetIntCount();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }

  /* fADC250 Readout */
  BANKOPEN(FADC_BANK ,BT_UI4,0);

  /* Mask of initialized modules */
  scanmask = faV3ScanMask();
  /* Check scanmask for block ready up to 100 times */
  datascan = faV3GBlockReady(scanmask, 100);
  stat = (datascan == scanmask);

  if(stat)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  nwords = faV3ReadBlock(faV3Slot(ifa), dma_dabufp, MAXFADCWORDS, 1);

	  /* Check for ERROR in block read */
	  blockError = faV3GetBlockError(1);

	  if(blockError)
	    {
	      printf("ERROR: Slot %d: in transfer (event = %d), nwords = 0x%x\n",
		     faV3Slot(ifa), roCount, nwords);

	      if(nwords > 0)
		dma_dabufp += nwords;
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      printf("ERROR: Event %d: Datascan != Scanmask  (0x%08x != 0x%08x)\n",
	     roCount, datascan, scanmask);
    }
  BANKCLOSE;


  /* Check for SYNC Event */
  if(tiGetSyncEventFlag() == 1)
    {
      /* Check for data available */
      int davail = tiBReady();
      if(davail > 0)
	{
	  printf("%s: ERROR: TI Data available (%d) after readout in SYNC event \n",
		 __func__, davail);

	  while(tiBReady())
	    {
	      vmeDmaFlush(tiGetAdr32());
	    }
	}

      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  davail = faV3Bready(faV3Slot(ifa));
	  if(davail > 0)
	    {
	      printf("%s: ERROR: fADC250 Data available (%d) after readout in SYNC event \n",
		     __func__, davail);

	      while(faV3Bready(faV3Slot(ifa)))
		{
		  vmeDmaFlush(faV3GetA32(faV3Slot(ifa)));
		}
	    }
	}
    }

}

void
rocCleanup()
{

  printf("%s: Reset all FADCs\n",__FUNCTION__);
  faV3GReset(1);

}

/*
  Local Variables:
  compile-command: "make -k faV3_list.so"
  End:
 */
