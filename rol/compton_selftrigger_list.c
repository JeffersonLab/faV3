/*************************************************************************
 *
 *  compton_selftrigger_list.c - Library of routines for readout of
 *                    faV3 in stand-alone readout mode (without TI)
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     10
#define MAX_EVENT_LENGTH   1152*32      /* Size in Bytes */

#include "GEN_source.h" /* source required for CODA */
#include <unistd.h>

#include "jvme.h"
#include "dmaPList.h"
/* DMA memory for VME Readout */
DMA_MEM_ID vmeIN;

#include "faV3Lib.h"
#include "faV3-Compton.h"
#include "faV3Config.h"

char config_filename[256] = "./faV3-Compton.cfg";
#define FAV3_ADDR 0xed0000
extern int32_t nfaV3;


int blklevel = 1;

/*
  Type 0xff10 is RAW trigger No timestamps
  Type 0xff11 is RAW trigger with timestamps (64 bits)
*/
int trigBankType = 0xff10;

/* Global Flag for debug printing */
int usrDebugFlag=0;

/****************************************
 *  DOWNLOAD
 ****************************************/
void
rocDownload()
{

  /* Initialize memory partition library */
  dmaPartInit();

  /* Allocate memory for DMA */
  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",MAX_EVENT_LENGTH,1,0);

  if(vmeIN == 0)
    daLogMsg("ERROR", "Unable to allocate memory for event buffers");

  /* Reinitialize the Buffer memory */
  dmaPReInitAll();
  dmaPStatsAll();

  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  int32_t iflag = 0; // default with internal clock

  vmeSetQuietFlag(1);
  faV3ComptonInit(FAV3_ADDR, 1<<19, 1, iflag);
  if(nfaV3 <= 0)
    {
      daLogMsg("ERROR", "faV3 not initialized");
    }

  faV3ComptonSelectTrigger(faV3Slot(0), 0);  // 0: self trigger
  faV3SetSyncSource(faV3Slot(0), 6);
  faV3SetTrigSource(faV3Slot(0), 3);

  faV3EnableBusError(faV3Slot(0));

  faV3ComptonGStatus(0);



  printf("rocDownload: User Download Executed\n");

}

/****************************************
 *  PRESTART
 ****************************************/
void
rocPrestart()
{
  usrDebugFlag=0;

  int32_t rval = faV3Config(config_filename);
  if(rval < 0) {
    daLogMsg("ERROR", "faV3 configuration failed");
  }

  int ifa;
  for(ifa=0; ifa < nfaV3; ifa++)
    {
      faV3SoftReset(faV3Slot(ifa),0);
      faV3ResetTriggerCount(faV3Slot(ifa));
    }

  faV3GEnableSyncSrc();
  faV3ComptonGStatus(0);


  faV3Sync(faV3Slot(0));

    printf("rocPrestart: User Prestart Executed\n");

}

/****************************************
 *  GO
 ****************************************/
void
rocGo()
{
  /*  Enable FADC */
  faV3ComptonEnable(faV3Slot(0));

}

/****************************************
 *  END
 ****************************************/
void
rocEnd()
{

  dmaPStatsAll();

  /* FADC Disable */
  faV3ComptonDisable(faV3Slot(0));

  /* FADC Event status - Is all data read out */
  faV3ComptonGStatus(0);

  printf("rocEnd: Ended after %d events\n",*(rol->nevents));

}

/****************************************
 *  EVENT TYPE
 ****************************************/
int
rocType()
{
  int rval = 1;

  return rval;
}

/****************************************
 *  POLLING ROUTINE
 ****************************************/
int
rocPoll()
{
  int rval = 0;

  rval = faV3Bready(faV3Slot(0)) ? 1 : 0;

  return rval;
}


/****************************************
 *  TRIGGER
 ****************************************/
void
rocTrigger(int evno, int evtype)
{

  CEOPEN(ROCID, BT_BANK, blklevel);

  /* Create Dummy trigger Bank */
  CBOPEN(trigBankType, BT_SEG, blklevel);
  int ii;
  for(ii = 0; ii < blklevel; ii++)
    {
      if(trigBankType == 0xff11)
	{
	  *rol->dabufp++ = (evtype << 24) | (0x01 << 16) | (3);
	}
      else
	{
	  *rol->dabufp++ = (evtype << 24) | (0x01 << 16) | (1);
	}
      *rol->dabufp++ = (blklevel * (evno - 1) + (ii + 1));
      if(trigBankType == 0xff11)
	{
	  *rol->dabufp++ = 0x12345678;
	  *rol->dabufp++ = 0;
	}
    }
  CBCLOSE;

  /* Grab a buffer for DMA */
  unsigned int *dma_dabufp = NULL;

  DMANODE *the_event = dmaPGetItem(vmeIN);
  if(the_event == (DMANODE *) 0) {
    daLogMsg("ERROR","DMA BUFFER ERROR: no pool buffer available for part %s\n",
	     vmeIN->name);
  } else {
    dma_dabufp = (unsigned int *) &(the_event->data[0]);
  }

  int nwords = faV3ReadBlock(faV3Slot(0), dma_dabufp, 1024, 0);
  faV3ResetToken(faV3Slot(0));
  if(nwords >= 0)
    dma_dabufp += nwords;

  /* Copy it into the event buffer */
  long length = (((long)(dma_dabufp) - (long)(&the_event->data[0]))>>2);

  CBOPEN(4, BT_UI4, blklevel);
  int ii;
  for(ii = 0; ii < length; ii++)
    *rol->dabufp++ = the_event->data[ii];
  CBCLOSE;

  dmaPFreeItem(the_event);

  CECLOSE;

}

void
rocReset()
{
  /* Free all allocated memory for DMA */
  dmaPFreeAll();
}

void
rocCleanup()
{
  /* Free all allocated memory for DMA */
  dmaPFreeAll();
  printf("%s: Reset all Modules\n",__FUNCTION__);
  faV3GReset(1);

}

int
tsLive(int sflag)
{
  return 100;
}
