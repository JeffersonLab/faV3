/**
 * @copyright Copyright 2024, Jefferson Science Associates, LLC.
 *            Subject to the terms in the LICENSE file found in the
 *            top-level directory.
 *
 * @author    Bryan Moffit
 *            moffit@jlab.org                   Jefferson Lab, MS-12B3
 *            Phone: (757) 269-5660             12000 Jefferson Ave.
 *                                              Newport News, VA 23606
 *
 * @author    David Abbott
 *            abbottd@jlab.org                  Jefferson Lab, MS-12B3
 *            Phone: (757) 269-7190             12000 Jefferson Ave.
 *                                              Newport News, VA 23606
 * @file      faV3Lib.c
 *
 * @brief     Library for JLAB configuration and readout of JLAB 250MHz
 *            FLASH ADC V3
 *
 */


#ifdef VXWORKS
#include <vxWorks.h>
#else
#include <stddef.h>
#include <pthread.h>
#include "jvme.h"
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef VXWORKS
#include <logLib.h>
#include <taskLib.h>
#include <intLib.h>
#include <iv.h>
#include <semLib.h>
#include <vxLib.h>
#else
#include <unistd.h>
#endif


/* Include ADC definitions */
#include "faV3Lib.h"

#ifdef VXWORKS
#define FAV3LOCK
#define FAV3UNLOCK
#else
/* Mutex to guard flexio read/writes */
pthread_mutex_t faV3Mutex = PTHREAD_MUTEX_INITIALIZER;
#define FAV3LOCK      if(pthread_mutex_lock(&faV3Mutex)<0) perror("pthread_mutex_lock");
#define FAV3UNLOCK    if(pthread_mutex_unlock(&faV3Mutex)<0) perror("pthread_mutex_unlock");
#endif

/* Define external Functions */
#ifdef VXWORKS
IMPORT STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT STATUS intDisconnect(int);
IMPORT STATUS sysIntEnable(int);
IMPORT STATUS sysIntDisable(int);
IMPORT STATUS sysVmeDmaDone(int, int);
IMPORT STATUS sysVmeDmaSend(uint32_t, uint32_t, int, BOOL);

#define EIEIO    __asm__ volatile ("eieio")
#define SYNC     __asm__ volatile ("sync")
#endif

/* Define Interrupts variables */
BOOL faV3IntRunning = FALSE;	/* running flag */
int faV3IntID = -1;		/* id number of ADC generating interrupts */
LOCAL VOIDFUNCPTR faV3IntRoutine = NULL;	/* user interrupt service routine */
LOCAL int faV3IntArg = 0;	/* arg to user routine */
LOCAL uint32_t faV3IntLevel = FAV3_VME_INT_LEVEL;	/* default VME interrupt level */
LOCAL uint32_t faV3IntVec = FAV3_VME_INT_VEC;	/* default interrupt Vector */

/* Define global variables */
int nfaV3 = 0;			/* Number of FAV3s in Crate */
uint32_t faV3A32Base = 0x09000000;	/* Minimum VME A32 Address for use by FAV3s */
u_long faV3A32Offset = 0x08000000;	/* Difference in CPU A32 Base - VME A32 Base */
u_long faV3A24Offset = 0x0;	/* Difference in CPU A24 Base - VME A24 Base */
u_long faV3A16Offset = 0x0;	/* Difference in CPU A16 Base - VME A16 Base */
volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];	/* pointers to FAV3 memory map */
volatile faV3sdc_t *FAV3SDCp;	/* pointer to FAV3 Signal distribution card */
volatile uint32_t *FAV3pd[(FAV3_MAX_BOARDS + 1)];	/* pointers to FAV3 FIFO memory */
volatile uint32_t *FAV3pmb;	/* pointer to Multblock window */
int faV3ID[FAV3_MAX_BOARDS];	/* array of slot numbers for FAV3s */
uint32_t faV3AddrList[FAV3_MAX_BOARDS];	/* array of a24 addresses for FAV3s */
int faV3FwRev[(FAV3_MAX_BOARDS + 1)][FAV3_FW_FUNCTION_MAX];  /* control+proc version numbers */


uint16_t faV3ChanDisableMask[(FAV3_MAX_BOARDS + 1)];	/* Disabled Channel Mask for each Module */
int faV3Inited = 0;		/* >0 if Library has been Initialized before */
int faV3MaxSlot = 0;		/* Highest Slot hold an FAV3 */
int faV3MinSlot = 0;		/* Lowest Slot holding an FAV3 */
int faV3Source = 0;		/* Signal source for FAV3 system control */
int faV3UseSDC = 0;		/* If > 0 then Use Signal Distribution board */
int faV3SDCPassthrough = 0;	/* If > 0 SDC in level translate / passthrough mode */
faV3data_t faV3_data;
int faV3BlockError = FAV3_BLOCKERROR_NO_ERROR;	/* Whether (>0) or not (0) Block Transfer had an error */

#define CHECKID	{							\
    if(id == 0) id = faV3ID[0];						\
    if((id <= 0) || (id > 21) || (FAV3p[id] == NULL)) {			\
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__, id); \
      return ERROR; }}


/**
 * @defgroup Config Initialization/Configuration
 * @defgroup SDCConfig SDC Initialization/Configuration
 *   @ingroup Config
 * @defgroup Status Status
 * @defgroup SDCStatus SDC Status
 *   @ingroup Status
 * @defgroup Readout Data Readout
 * @defgroup IntPoll Interrupt/Polling
 * @defgroup Deprec Deprecated - To be removed
 */

/**
 *  @ingroup Config
 *  @brief Initialize JLAB FADC250 V3 Library.
 *
 * @param addr
 *  - A24 VME Address of the fADC250 V3
 * @param addr_inc
 *  - Amount to increment addr to find the next fADC250 V3
 * @param nadc
 *  - Number of times to increment
 *
 *  @param iFlag 18 bit integer
 * <pre>
 *       Low 6 bits - Specifies the default Signal distribution (clock,trigger)
 *                    sources for the board (Internal, FrontPanel, VXS, VME(Soft))
 *       bit    0:  defines Sync Reset source
 *                     0  VME (Software Sync-Reset)
 *                     1  Front Panel/VXS/P2 (Depends on Clk/Trig source selection)
 *       bits 3-1:  defines Trigger source
 *               0 0 0  VME (Software Triggers)
 *               0 0 1  Front Panel Input
 *               0 1 0  VXS (P0)
 *               1 0 0  Internal Trigger Logic (HITSUM FPGA)
 *               (all others Undefined - default to VME/Software)
 *       bits 5-4:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  P2 Connector (Backplane)
 * </pre>
 *
 * <pre>
 *       Common Modes of Operation:
 *           Value = 0  CLK (Int)  TRIG (Soft)   SYNC (Soft)    (Debug/Test Mode)
 *                   2  CLK (Int)  TRIG (FP)     SYNC (Soft)    (Single Board
 *                   3  CLK (Int)  TRIG (FP)     SYNC (FP)         Modes)
 *                0x10  CLK (FP)   TRIG (Soft)   SYNC (Soft)
 *                0x13  CLK (FP)   TRIG (FP)     SYNC (FP)      (VME SDC Mode)
 *                0x20  CLK (VXS)  TRIG (Soft)   SYNC (Soft)
 *                0x25  CLK (VXS)  TRIG (VXS)    SYNC (VXS)     (VXS SD Mode)
 *
 *
 *      High 10bits - A16 Base address of FADC Signal Distribution Module
 *                    This board can control up to 7 FADC Boards.
 *                    Clock Source must be set to Front Panel (bit4 = 1)
 *
 *      bit 16:  Exit before board initialization
 *             0 Initialize FADC (default behavior)
 *             1 Skip initialization (just setup register map pointers)
 *
 *      bit 17:  Use fadcAddrList instead of addr and addr_inc
 *               for VME addresses.
 *             0 Initialize with addr and addr_inc
 *             1 Use fadcAddrList
 *
 *      bit 18:  Skip firmware check.  Useful for firmware updating.
 *             0 Perform firmware check
 *             1 Skip firmware check
 * </pre>
 *
 *
 * @return OK, or ERROR if the address is invalid or a board is not present.
 */

int
faV3Init(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc = 0, clkSrc = 0, srSrc = 0;
  uint32_t rdata, a32addr, a16addr = 0;
  u_long laddr = 0, laddr_inc = 0;
  volatile faV3_t *fa;
  uint16_t sdata;
  int noBoardInit = 0;
  int useList = 0;
  int multiBlockOnly = 0;
  int vxsReadoutOnly = 0;
  uint16_t ctrl_version = 0, proc_version = 0;

  /* Check if we have already Initialized boards before */
  if((faV3Inited > 0) && (faV3ID[0] != 0))
    {
      /* Hard Reset of all FADC boards in the Crate */
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->csr), FAV3_CSR_HARD_RESET);
	}
      taskDelay(5);
    }

  /* Check if we are to exit when pointers are setup */
  noBoardInit = (iFlag & FAV3_INIT_SKIP) ? 1 : 0;

  /* Check if we're initializing using a list */
  useList = (iFlag & FAV3_INIT_USE_ADDRLIST) ? 1 : 0;

  /* Check if we're only using token passing for readout */
  multiBlockOnly = (iFlag & FAV3_INIT_MULTIBLOCK_ONLY) ? 1 : 0;

  /* Check if we're reading out through the VXS (VTP) */
  vxsReadoutOnly = (iFlag & FAV3_INIT_VXS_READOUT_ONLY) ? 1 : 0;

  /* Check for valid address */
  if(addr == 0)
    {
      printf("faInit: ERROR: Must specify a Bus (VME-based A24) address for FADC 0\n");
      return (ERROR);
    }
  else if(addr > 0x00ffffff)
    {				/* A24 Addressing */
      printf("faInit: ERROR: A32 Addressing not allowed for FADC configuration space\n");
      return (ERROR);
    }
  else
    {				/* A24 Addressing */
      if(((addr_inc == 0) || (nadc == 0)) && (useList == 0))
	nadc = 1;		/* assume only one FADC to initialize */

      /* get the FADC address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x39, (char *) addr, (char **) &laddr);
#else
      res = vmeBusToLocalAdrs(0x39, (char *) (u_long) addr, (char **) &laddr);
#endif
      if(res != 0)
	{
#ifdef VXWORKS
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 addr);
#else
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 addr);
#endif
	  return (ERROR);
	}
      faV3A24Offset = laddr - addr;
    }

  /* Init Some Global variables */
  faV3Source = iFlag & FAV3_SOURCE_MASK;
  faV3Inited = nfaV3 = 0;
  faV3UseSDC = 0;
  memset((char *) FAV3p, 0, sizeof(FAV3p));
  memset((char *) FAV3pd, 0, sizeof(FAV3pd));
  FAV3pmb = NULL;
  memset((char *) faV3ID, 0, sizeof(faV3ID));
  memset((char *) faV3FwRev, 0, sizeof(faV3FwRev));
  memset((char *) faV3ChanDisableMask, 0, sizeof(faV3ChanDisableMask));


  for(ii = 0; ii < nadc; ii++)
    {
      if(useList == 1)
	{
	  laddr_inc = faV3AddrList[ii] + faV3A24Offset;
	}
      else
	{
	  laddr_inc = laddr + ii * addr_inc;
	}
      fa = (faV3_t *) laddr_inc;
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(fa->version), VX_READ, 4, (char *) &rdata);
#else
      res = vmeMemProbe((char *) &(fa->version), 4, (char *) &rdata);
#endif
      if(res < 0)
	{
#ifdef VXWORKS
	  printf("faInit: WARN: No addressable board at addr=0x%x\n",
		 (uint32_t) fa);
#else
	  printf("faInit: WARN: No addressable board at VME (Local) addr=0x%x (0x%lx)\n",
		 (uint32_t) (laddr_inc - faV3A24Offset), (u_long) fa);
#endif
	  errFlag = 1;
	  continue;
	}
      else
	{
	  /* Check that it is an FA board */
	  if((rdata & FAV3_BOARD_MASK) != FAV3_BOARD_ID)
	    {
	      printf("%s: WARN: For board at 0x%lx, Invalid Board ID: 0x%x\n",
		     __func__, (u_long) fa - faV3A24Offset, rdata);
	      continue;
	    }
	  else
	    {
	      /* Check if this is board has a valid slot number */
	      boardID = ((vmeRead32(&(fa->intr))) & FAV3_SLOT_ID_MASK) >> 16;

	      if((boardID <= 0) || (boardID > 21))
		{
		  printf(" ERROR: Board Slot ID is not in range: %d\n",
			 boardID);
		  continue;
		  /*          return(ERROR); */
		}
	      else
		{
		  /* Check Control FPGA firmware version */
		  ctrl_version = rdata & FAV3_VERSION_MASK;

		  /* Check Processing FPGA firmware version */
		  proc_version =
		    (uint16_t) (vmeRead32(&fa->adc.status0) &
				FAV3_ADC_VERSION_MASK);

		  FAV3p[boardID] = (faV3_t *) (laddr_inc);

		  faV3FwRev[boardID][FAV3_FW_CTRL] = ctrl_version;
		  faV3FwRev[boardID][FAV3_FW_PROC] = proc_version;

		  faV3ID[nfaV3] = boardID;
		  if(boardID >= maxSlot)
		    maxSlot = boardID;
		  if(boardID <= minSlot)
		    minSlot = boardID;

		  printf("Initialized FADC %2d  Slot #%2d at VME (Local) address 0x%06x (0x%lx) \n",
			 nfaV3, faV3ID[nfaV3],
			 (uint32_t) (((u_long) FAV3p[(faV3ID[nfaV3])]) - faV3A24Offset),
			 (u_long) FAV3p[(faV3ID[nfaV3])]);
		}
	      nfaV3++;
	    }
	}
    }				// End loop through fadcs


  /* Check if we are using a JLAB FADC Signal Distribution Card (SDC)
     NOTE the SDC board only supports 7 FADCs - so if there are
     more than 7 FADCs in the crate they can only be controlled by daisychaining
     multiple SDCs together - or by using a VXS Crate with SD switch card
  */
  a16addr = iFlag & FAV3_SDC_ADR_MASK;
  if(a16addr)
    {
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x29, (char *) a16addr, (char **) &laddr);
      if(res != 0)
	{
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x29,0x%x,&laddr) \n",
		 a16addr);
	  return (ERROR);
	}

      res = vxMemProbe((char *) laddr, VX_READ, 2, (char *) &sdata);
#else
      res =
	vmeBusToLocalAdrs(0x29, (char *) (u_long) a16addr, (char **) &laddr);
      if(res != 0)
	{
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x29,0x%x,&laddr) \n",
		 a16addr);
	  return (ERROR);
	}
      res = vmeMemProbe((char *) laddr, 2, (char *) &sdata);
#endif
      if(res < 0)
	{
	  printf("faInit: ERROR: No addressable SDC board at addr=0x%x\n",
		 (uint32_t) laddr);
	}
      else
	{
	  faV3A16Offset = laddr - a16addr;
	  FAV3SDCp = (faV3sdc_t *) laddr;
	  if(!noBoardInit)
	    vmeWrite16(&(FAV3SDCp->ctrl), FAV3SDC_CSR_INIT);	/* Reset the Module */

	  if(nfaV3 > 7)
	    {
	      printf("WARN: A Single JLAB FADC Signal Distribution Module only supports 7 FADCs\n");
	      printf("WARN: You must use multiple SDCs to support more FADCs - this must be configured in hardware\n");
	    }
#ifdef VXWORKS
	  printf("Using JLAB FADC Signal Distribution Module at address 0x%x\n",
		 (uint32_t) FAV3SDCp);
#else
	  printf("Using JLAB FADC Signal Distribution Module at VME (Local) address 0x%x (0x%lx)\n",
		 (uint32_t) a16addr, (u_long) FAV3SDCp);
#endif
	  faV3UseSDC = 1;
	}

      if(faV3Source == FAV3_SOURCE_SDC)
	{			/* Check if SDC will be used */
	  faV3UseSDC = 1;
	  printf("faInit: JLAB FADC Signal Distribution Card is Assumed in Use\n");
	  printf("faInit: Front Panel Inputs will be enabled. \n");
	}
      else
	{
	  faV3UseSDC = 0;
	  printf("faInit: JLAB FADC Signal Distribution Card will not be Used\n");
	}
    }				// end if a16addr

  /* Hard Reset of all FADC boards in the Crate */
  if(!noBoardInit)
    {
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->reset), FAV3_RESET_ALL);
	}
      taskDelay(60);
    }

  /* Initialize Interrupt variables */
  faV3IntID = -1;
  faV3IntRunning = FALSE;
  faV3IntLevel = FAV3_VME_INT_LEVEL;
  faV3IntVec = FAV3_VME_INT_VEC;
  faV3IntRoutine = NULL;
  faV3IntArg = 0;

  /* Calculate the A32 Offset for use in Block Transfers */
#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x09, (char *) faV3A32Base, (char **) &laddr);
  if(res != 0)
    {
      printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     faV3A32Base);
      return (ERROR);
    }
  else
    {
      faV3A32Offset = laddr - faV3A32Base;
    }
#else
  res =
    vmeBusToLocalAdrs(0x09, (char *) (u_long) faV3A32Base, (char **) &laddr);
  if(res != 0)
    {
      printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     faV3A32Base);
      return (ERROR);
    }
  else
    {
      faV3A32Offset = laddr - faV3A32Base;
    }
#endif

  if(!noBoardInit)
    {
      /* what are the Trigger Sync Reset and Clock sources */
      if(faV3Source == FAV3_SOURCE_VXS)
	{
	  printf("faInit: Enabling FADC for VXS Clock ");
	  clkSrc = FAV3_REF_CLK_P0;
	  switch (iFlag & 0xf)
	    {
	    case 0:
	    case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_VME | FAV3_ENABLE_SOFT_TRIG;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    case 4:
	    case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_P0_ISYNC;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 5:
	    case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = FAV3_TRIG_P0_ISYNC;
	      srSrc = FAV3_SRESET_P0_ISYNC;
	      break;
	    case 8:
	    case 10:
	    case 12:
	    case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 9:
	    case 11:
	    case 13:
	    case 15:
	      printf("and Internal Trigger Logic (VXS Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    }
	}
      else if(faV3Source == FAV3_SOURCE_SDC)
	{
	  printf("faInit: Enabling FADC for SDC Clock (Front Panel) ");
	  clkSrc = FAV3_REF_CLK_FP;
	  switch (iFlag & 0xf)
	    {
	    case 0:
	    case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_VME | FAV3_ENABLE_SOFT_TRIG;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	    case 4:
	    case 6:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	    case 5:
	    case 7:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    case 8:
	    case 10:
	    case 12:
	    case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 9:
	    case 11:
	    case 13:
	    case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    }
	  faV3SDC_Config(0, 0);
	}
      else
	{			/* Use internal Clk */
	  printf("faInit: Enabling FADC Internal Clock, ");
	  clkSrc = FAV3_REF_CLK_INTERNAL;
	  switch (iFlag & 0xf)
	    {
	    case 0:
	    case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_VME | FAV3_ENABLE_SOFT_TRIG;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = FAV3_TRIG_FP_ISYNC;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    case 4:
	    case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_P0_ISYNC;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 5:
	    case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = FAV3_TRIG_P0_ISYNC;
	      srSrc = FAV3_SRESET_P0_ISYNC;
	      break;
	    case 8:
	    case 10:
	    case 12:
	    case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_VME | FAV3_ENABLE_SOFT_SRESET;
	      break;
	    case 9:
	    case 11:
	    case 13:
	    case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = FAV3_TRIG_INTERNAL;
	      srSrc = FAV3_SRESET_FP_ISYNC;
	      break;
	    }
	}

      /* Enable Clock source - Internal Clk enabled by default */
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&FAV3p[faV3ID[ii]]->ctrl1,
		     (clkSrc | FAV3_ENABLE_INTERNAL_CLK));
	}
      taskDelay(20);


      /* Hard Reset FPGAs and FIFOs */
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&FAV3p[faV3ID[ii]]->reset,
		     (FAV3_RESET_HARD_CNTL | FAV3_RESET_HARD_PROC |
		      FAV3_RESET_ADC_FIFO | FAV3_RESET_HITSUM_FIFO |
		      FAV3_RESET_DAC | FAV3_RESET_EXT_RAM_PT));

	  /* Release reset on MGTs */
	  vmeWrite32(&FAV3p[faV3ID[ii]]->ctrl_mgt, FAV3_MGT_RESET);
	  vmeWrite32(&FAV3p[faV3ID[ii]]->ctrl_mgt, FAV3_RELEASE_MGT_RESET);
	  vmeWrite32(&FAV3p[faV3ID[ii]]->ctrl_mgt, FAV3_MGT_RESET);

	}
      taskDelay(5);
    }

  /* Write configuration registers with default/defined Sources */
  for(ii = 0; ii < nfaV3; ii++)
    {
      if((!multiBlockOnly) || (!vxsReadoutOnly))
	{
	  /* Program an A32 access address for this FADC's FIFO */
	  a32addr = faV3A32Base + ii * FAV3_MAX_A32_MEM;
#ifdef VXWORKS
	  res = sysBusToLocalAdrs(0x09, (char *) a32addr, (char **) &laddr);
	  if(res != 0)
	    {
	      printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		     a32addr);
	      return (ERROR);
	    }
#else
	  res =
	    vmeBusToLocalAdrs(0x09, (char *) (u_long) a32addr, (char **) &laddr);
	  if(res != 0)
	    {
	      printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		     a32addr);
	      return (ERROR);
	    }
#endif
	  FAV3pd[faV3ID[ii]] = (uint32_t *) (laddr);	/* Set a pointer to the FIFO */
	}

      if(!noBoardInit)
	{
	  /* Write a32 address and enable */
	  if((!multiBlockOnly) || (!vxsReadoutOnly))
	    vmeWrite32(&FAV3p[faV3ID[ii]]->adr32, (a32addr >> 16) + 1);

	  /* Set Default Block Level to 1 */
	  vmeWrite32(&FAV3p[faV3ID[ii]]->blocklevel, 1);

	  /* Setup Trigger and Sync Reset sources */
	  vmeWrite32(&FAV3p[faV3ID[ii]]->ctrl1,
		     (vmeRead32(&FAV3p[faV3ID[ii]]->ctrl1) &
		      ~(FAV3_REF_CLK_MASK | FAV3_TRIG_MASK | FAV3_SRESET_MASK)) |
		     (clkSrc | srSrc | trigSrc) );
	}
    }				//End loop through fadcs

  /* If there are more than 1 FADC in the crate then setup the Muliblock Address
     window. This must be the same on each board in the crate */
  if((nfaV3 > 1) && (!vxsReadoutOnly))
    {
      if(multiBlockOnly)
	a32addr = faV3A32Base;
      else
	a32addr = faV3A32Base + (nfaV3 + 1) * FAV3_MAX_A32_MEM;	/* set MB base above individual board base */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09, (char *) a32addr, (char **) &laddr);
      if(res != 0)
	{
	  printf("faInit: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 a32addr);
	  return (ERROR);
	}
#else
      res =
	vmeBusToLocalAdrs(0x09, (char *) (u_long) a32addr, (char **) &laddr);
      if(res != 0)
	{
	  printf("faInit: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 a32addr);
	  return (ERROR);
	}
#endif
      FAV3pmb = (uint32_t *) (laddr);	/* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  for(ii = 0; ii < nfaV3; ii++)
	    {
	      /* Write the register and enable */
	      vmeWrite32(&(FAV3p[faV3ID[ii]]->adr_mb),
			 (a32addr + FAV3_MAX_A32MB_SIZE) + (a32addr >> 16) +
			 FAV3_A32_ENABLE);
	    }
	}
      /* Set First Board and Last Board */
      faV3MaxSlot = maxSlot;
      faV3MinSlot = minSlot;
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAV3p[minSlot]->ctrl1),
		     vmeRead32(&(FAV3p[minSlot]->ctrl1)) | FAV3_FIRST_BOARD);
	  vmeWrite32(&(FAV3p[maxSlot]->ctrl1),
		     vmeRead32(&(FAV3p[maxSlot]->ctrl1)) | FAV3_LAST_BOARD);
	}
    }

  faV3Inited = nfaV3;

  if(nfaV3 <= 0)
    {
      printf("%s: ERROR: No FADCs initialized\n", __func__);

      return (ERROR);
    }

  printf("%s: %d FADC(s) successfully initialized\n",
	 __func__, nfaV3);

  return nfaV3;

}				//End of faInit

void
faV3SetA32BaseAddress(uint32_t addr)
{
  faV3A32Base = addr;
  printf("fadc A32 base address set to 0x%08X\n", faV3A32Base);
}

/**
 * @ingroup Status
 * @brief Convert an index into a slot number, where the index is
 *          the element of an array of FADCs in the order in which they were
 *          initialized.
 *
 * @param[in] i Initialization number
 * @return Slot number if Successfull, otherwise ERROR.
 *
 */

int
faV3Slot(uint32_t i)
{
  if(i >= nfaV3)
    {
      printf("%s: ERROR: Index (%d) >= FADCs initialized (%d).\n",
	     __func__, i, nfaV3);
      return ERROR;
    }

  return faV3ID[i];
}


/**
 * @ingroup Status
 * @brief Convert a slot number into the index, where the index is
 *          the element of an array of FADCs in the order in which they were
 *          initialized.
 *
 * @param[in] slot faV3 slot number
 * @return Index if Successfull, otherwise ERROR.
 *
 */
int
faV3Id(uint32_t slot)
{
  int id;

  for(id = 0; id < nfaV3; id++)
    {
      if(faV3ID[id] == slot)
	{
	  return (id);
	}
    }

  printf("%s: ERROR: FADC in slot %d does not exist or not initialized.\n",
	 __func__, slot);
  return (ERROR);
}

int
faV3GetN()
{
  return (nfaV3);
}


/**
 *  @ingroup Config
 *  @brief Set all sram registers to zero
 *
 *  @param id Slot Number
 *
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3ZeroSram(int id)
{
  int rval = OK;

  CHECKID;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Set the clock source
 *
 *   This routine should be used in the case that the source clock
 *   is NOT set in faInit (and defaults to Internal).  Such is the case
 *   when clocks are synchronized in a many crate system.  The clock source
 *   of the FADC should ONLY be set AFTER those clocks have been set and
 *   synchronized.
 *
 *  @param id Slot Number
 *  @param clkSrc 2 bit integer
 * <pre>
 *       bits 1-0:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  VXS (P0)
 * </pre>
 *
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3SetClockSource(int id, int clkSrc)
{
  CHECKID;

  if(clkSrc > 0x3)
    {
      printf("%s: ERROR: Invalid Clock Source specified (0x%x)\n",
	     __func__, clkSrc);
      return ERROR;
    }

  /* Enable Clock source - Internal Clk enabled by default */
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     (vmeRead32(&FAV3p[id]->ctrl1) & ~(FAV3_REF_CLK_MASK)) |
	     (clkSrc | FAV3_ENABLE_INTERNAL_CLK));
  taskDelay(20);
  FAV3UNLOCK;

  switch (clkSrc)
    {
    case FAV3_REF_CLK_INTERNAL:
      printf("%s: FADC id %d clock source set to INTERNAL\n", __func__, id);
      break;

    case FAV3_REF_CLK_FP:
      printf("%s: FADC id %d clock source set to FRONT PANEL\n",
	     __func__, id);
      break;

    case FAV3_REF_CLK_P0:
      printf("%s: FADC id %d clock source set to VXS (P0)\n", __func__, id);
      break;

    case FAV3_REF_CLK_MASK:
      printf("%s: FADC id %d clock source set to VXS (P0)\n", __func__, id);
      break;
    }

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the clock source for all initialized modules
 *
 *   This routine should be used in the case that the source clock
 *   is NOT set in faInit (and defaults to Internal).  Such is the case
 *   when clocks are synchronized in a many crate system.  The clock source
 *   of the FADC should ONLY be set AFTER those clocks have been set and
 *   synchronized.
 *
 *  @param clkSrc 2 bit integer
 * <pre>
 *       bits 1-0:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  VXS (P0)
 * </pre>
 *
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3GSetClockSource(int clkSrc)
{
  int ifa, id;
  if(clkSrc > 0x3)
    {
      printf("%s: ERROR: Invalid Clock Source specified (0x%x)\n",
	     __func__, clkSrc);
      return ERROR;
    }

  /* Enable Clock source - Internal Clk enabled by default */
  FAV3LOCK;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      vmeWrite32(&(FAV3p[id]->ctrl1),
		 (vmeRead32(&FAV3p[id]->ctrl1) & ~(FAV3_REF_CLK_MASK)) |
		 (clkSrc | FAV3_ENABLE_INTERNAL_CLK));
    }
  taskDelay(20);
  FAV3UNLOCK;

  switch (clkSrc)
    {
    case FAV3_REF_CLK_INTERNAL:
      printf("%s: FADC clock source set to INTERNAL\n", __func__);
      break;

    case FAV3_REF_CLK_FP:
      printf("%s: FADC clock source set to FRONT PANEL\n", __func__);
      break;

    case FAV3_REF_CLK_P0:
      printf("%s: FADC clock source set to VXS (P0)\n", __func__);
      break;

    case FAV3_REF_CLK_MASK:
      printf("%s: FADC clock source set to VXS (P0)\n", __func__);
      break;
    }

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Print Status of fADC250 to standard out
 *  @param id Slot Number
 *  @param sflag Reserved for future use
 *
 */

int
faV3Status(int id, int sflag)
{
  int ii;
  uint32_t a32Base, ambMin, ambMax, vers;
  uint32_t csr, ctrl1, ctrl2, count, bcount, blevel, intr, addr32, addrMB;
  uint32_t adcStat[3], adcConf[3],
    PTW, PL, NSB, NSA, NP, adcChanDisabled, playbackMode;
  uint32_t adc_enabled, adc_version, adc_option;
  uint32_t trigCnt, trig2Cnt, srCnt, itrigCnt, ramWords;
  uint32_t mgtStatus, mgtCtrl;
  uint32_t berr_count = 0;
  uint32_t scaler_interval = 0;
  uint32_t trigger_control = 0;
  uint32_t lost_trig_scal = 0;
  uint32_t tet_trg[16], tet_readout[16], delay[16];
  float gain_trg[16], ped_trg[16];
  uint32_t val, trig_mode[16], inverted[16], playback_ch[16];
  char *trig_mode_string;

  CHECKID;

  FAV3LOCK;
  vers = vmeRead32(&FAV3p[id]->version);

  csr = (vmeRead32(&(FAV3p[id]->csr))) & FAV3_CSR_MASK;
  ctrl1 = (vmeRead32(&(FAV3p[id]->ctrl1))) & FAV3_CONTROL_MASK;
  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
  count = (vmeRead32(&(FAV3p[id]->ev_count))) & FAV3_EVENT_COUNT_MASK;
  bcount = (vmeRead32(&(FAV3p[id]->blk_count))) & FAV3_BLOCK_COUNT_MASK;
  blevel = (vmeRead32(&(FAV3p[id]->blocklevel))) & FAV3_BLOCK_LEVEL_MASK;
  ramWords = (vmeRead32(&(FAV3p[id]->ram_word_count))) & FAV3_RAM_DATA_MASK;
  trigCnt = vmeRead32(&(FAV3p[id]->trig_count));
  trig2Cnt = vmeRead32(&FAV3p[id]->trig2_scal);
  srCnt = vmeRead32(&FAV3p[id]->syncreset_scal);
  itrigCnt = vmeRead32(&(FAV3p[id]->trig_live_count));
  intr = vmeRead32(&(FAV3p[id]->intr));
  addr32 = vmeRead32(&(FAV3p[id]->adr32));
  a32Base = (addr32 & FAV3_A32_ADDR_MASK) << 16;
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));
  ambMin = (addrMB & FAV3_AMB_MIN_MASK) << 16;
  ambMax = (addrMB & FAV3_AMB_MAX_MASK);
  berr_count = vmeRead32(&(FAV3p[id]->aux.berr_driven_count));

  adcStat[0] = (vmeRead32(&(FAV3p[id]->adc.status0)) & 0xFFFF);
  adcStat[1] = (vmeRead32(&(FAV3p[id]->adc.status1)) & 0xFFFF);
  adcStat[2] = (vmeRead32(&(FAV3p[id]->adc.status2)) & 0xFFFF);
  adcConf[0] = (vmeRead32(&(FAV3p[id]->adc.config1)) & 0xFFFF);
  adcConf[1] = (vmeRead32(&(FAV3p[id]->adc.config2)) & 0xFFFF);
  adcConf[2] = (vmeRead32(&(FAV3p[id]->adc.config4)) & 0xFFFF);

  PTW = (vmeRead32(&(FAV3p[id]->adc.ptw)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  PL = (vmeRead32(&(FAV3p[id]->adc.pl)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  NSB = (vmeRead32(&(FAV3p[id]->adc.nsb)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  NSA = (vmeRead32(&(FAV3p[id]->adc.nsa)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  adc_version = adcStat[0] & FAV3_ADC_VERSION_MASK;
  adc_option = (adcConf[0] & FAV3_ADC_PROC_MASK) + 1;
  NP = (adcConf[0] & FAV3_ADC_PEAK_MASK) >> 4;
  adc_enabled = (adcConf[0] & FAV3_ADC_PROC_ENABLE);
  playbackMode = (adcConf[0] & FAV3_ADC_PLAYBACK_MODE) >> 7;
  adcChanDisabled = (adcConf[1] & FAV3_ADC_CHAN_MASK);

  mgtStatus = vmeRead32(&(FAV3p[id]->status_mgt));

  scaler_interval =
    vmeRead32(&FAV3p[id]->scaler_insert) & FAV3_SCALER_INSERT_MASK;

  FAV3UNLOCK;

#ifdef VXWORKS
  printf("\nSTATUS for FADC in slot %d at base address 0x%x \n",
	 id, (uint32_t) FAV3p[id]);
#else
  printf("\nSTATUS for FADC in slot %d at VME (Local) base address 0x%x (0x%lx)\n",
	 id, (uint32_t) (u_long) (FAV3p[id] - faV3A24Offset), (u_long) FAV3p[id]);
#endif
  printf("---------------------------------------------------------------------- \n");

  printf(" Board Firmware Rev/ID = 0x%04x : ADC Processing Rev = 0x%04x\n",
	 (vers) & 0xffff, adc_version);
  if(addrMB & FAV3_AMB_ENABLE)
    {
      printf(" Alternate VME Addressing: Multiblock Enabled\n");
      if(addr32 & FAV3_A32_ENABLE)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n", a32Base,
	       (u_long) FAV3pd[id]);
      else
	printf("   A32 Disabled\n");

      printf("   Multiblock VME Address Range 0x%08x - 0x%08x\n", ambMin,
	     ambMax);
    }
  else
    {
      printf(" Alternate VME Addressing: Multiblock Disabled\n");
      if(addr32 & FAV3_A32_ENABLE)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n", a32Base,
	       (u_long) FAV3pd[id]);
      else
	printf("   A32 Disabled\n");
    }

  if(ctrl1 & FAV3_INT_ENABLE_MASK)
    {
      printf("\n  Interrupts ENABLED: ");
      if(ctrl1 & FAV3_ENABLE_BLKLVL_INT)
	printf(" on Block Level(%d)", blevel);

      printf("\n");
      printf("  Interrupt Reg: 0x%08x\n", intr);
      printf("  VME INT Vector = 0x%x  Level = %d\n",
	     (intr & FAV3_INT_VEC_MASK), ((intr & FAV3_INT_LEVEL_MASK) >> 8));
    }

  printf("\n Signal Sources: \n");

  if((ctrl1 & FAV3_REF_CLK_MASK) == FAV3_REF_CLK_INTERNAL)
    {
      printf("   Ref Clock : Internal\n");
    }
  else if((ctrl1 & FAV3_REF_CLK_MASK) == FAV3_REF_CLK_P0)
    {
      printf("   Ref Clock : VXS\n");
    }
  else if((ctrl1 & FAV3_REF_CLK_MASK) == FAV3_REF_CLK_FP)
    {
      printf("   Ref Clock : Front Panel\n");
    }
  else
    {
      printf("   Ref Clock : %d (Undefined)\n", (ctrl1 & FAV3_REF_CLK_MASK));
    }

  switch (ctrl1 & FAV3_TRIG_MASK)
    {
    case FAV3_TRIG_INTERNAL:
      printf("   Trig Src  : Internal\n");
      break;
    case FAV3_TRIG_VME:
      printf("   Trig Src  : VME (Software)\n");
      break;
    case FAV3_TRIG_P0_ISYNC:
      printf("   Trig Src  : VXS (Async)\n");
      break;
    case FAV3_TRIG_P0:
      printf("   Trig Src  : VXS (Sync)\n");
      break;
    case FAV3_TRIG_FP_ISYNC:
      printf("   Trig Src  : Front Panel (Async)\n");
      break;
    case FAV3_TRIG_FP:
      printf("   Trig Src  : Front Panel (Sync)\n");
    }

  switch (ctrl1 & FAV3_SRESET_MASK)
    {
    case FAV3_SRESET_VME:
      printf("   Sync Reset: VME (Software)\n");
      break;
    case FAV3_SRESET_P0_ISYNC:
      printf("   Sync Reset: VXS (Async)\n");
      break;
    case FAV3_SRESET_P0:
      printf("   Sync Reset: VXS (Sync)\n");
      break;
    case FAV3_SRESET_FP_ISYNC:
      printf("   Sync Reset: Front Panel (Async)\n");
      break;
    case FAV3_SRESET_FP:
      printf("   Sync Reset: Front Panel (Sync)\n");
    }

  if(faV3UseSDC)
    {
      printf("   SDC       : In Use\n");
    }


  printf("\n Configuration: \n");

  if(ctrl1 & FAV3_ENABLE_INTERNAL_CLK)
    printf("   Internal Clock ON\n");
  else
    printf("   Internal Clock OFF\n");

  if(ctrl1 & FAV3_ENABLE_BERR)
    printf("   Bus Error ENABLED\n");
  else
    printf("   Bus Error DISABLED\n");


  if(ctrl1 & FAV3_ENABLE_MULTIBLOCK)
    {
      int tP0, tP2;
      tP0 = ctrl1 & FAV3_MB_TOKEN_VIA_P0;
      tP2 = ctrl1 & FAV3_MB_TOKEN_VIA_P2;

      if(tP0)
	{
	  if(ctrl1 & FAV3_FIRST_BOARD)
	    printf("   MultiBlock transfer ENABLED (First Board - token via VXS)\n");
	  else if(ctrl1 & FAV3_LAST_BOARD)
	    printf("   MultiBlock transfer ENABLED (Last Board  - token via VXS)\n");
	  else
	    printf("   MultiBlock transfer ENABLED (Token via VXS)\n");
	  /* #ifdef VERSION1 */
	}
      else if(tP2)
	{
	  if(ctrl1 & FAV3_FIRST_BOARD)
	    printf("   MultiBlock transfer ENABLED (First Board - token via P2)\n");
	  else if(ctrl1 & FAV3_LAST_BOARD)
	    printf("   MultiBlock transfer ENABLED (Last Board  - token via P2)\n");
	  else
	    printf("   MultiBlock transfer ENABLED (Token via P2)\n");
	  /* #endif */
	}
      else
	{
	  printf("   MultiBlock transfer ENABLED (**NO Tokens enabled**)\n");
	}
    }
  else
    {
      printf("   MultiBlock transfer DISABLED\n");
    }

  if(ctrl1 & FAV3_ENABLE_SOFT_TRIG)
    printf("   Software Triggers   ENABLED\n");
  if(ctrl1 & FAV3_ENABLE_SOFT_SRESET)
    printf("   Software Sync Reset ENABLED\n");


  printf("\n ADC Processing Configuration: \n");
  printf("   Channel Disable Mask = 0x%04x\n", adcChanDisabled);
  if(adc_enabled)
    printf("   Mode = %d  (ENABLED)\n", adc_option);
  else
    printf("   Mode = %d  (Disabled)\n", adc_option);

  printf("   Lookback (PL)    = %d ns   Time Window (PTW) = %d ns\n", PL,
	 PTW);
  printf("   Time Before Peak = %d ns   Time After Peak   = %d ns\n", NSB,
	 NSA);
  printf("   Max Peak Count   = %d \n", NP);
  printf("   Playback Mode    = %d \n", playbackMode);



  printf("\n");
  printf(" Unacknowleged Trigger Stop: %s (%d)\n",
	 (trigger_control & FAV3_TRIGCTL_TRIGSTOP_EN) ? " ENABLED" : "DISABLED",
	 (trigger_control & FAV3_TRIGCTL_MAX2_MASK) >> 16);
  printf(" Unacknowleged Trigger Busy: %s (%d)\n",
	 (trigger_control & FAV3_TRIGCTL_BUSY_EN) ? " ENABLED" : "DISABLED",
	 trigger_control & FAV3_TRIGCTL_MAX1_MASK);



  printf("\n");
  if(csr & FAV3_CSR_ERROR_MASK)
    {
      printf("  CSR       Register = 0x%08x - **Error Condition**\n", csr);
    }
  else
    {
      printf("  CSR       Register = 0x%08x\n", csr);
    }

  printf("  Control 1 Register = 0x%08x \n", ctrl1);


  if((ctrl2 & FAV3_CTRL_ENABLE_MASK) == FAV3_CTRL_ENABLED)
    {
      printf("  Control 2 Register = 0x%08x - Enabled for triggers\n", ctrl2);
    }
  else
    {
      printf("  Control 2 Register = 0x%08x - Disabled\n", ctrl2);
    }



  if((ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_DISABLE)
    {
      printf("  Control 2 Register = 0x%08x - Compress disabled\n", ctrl2);
    }
  else if((ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_ENABLE)
    {
      printf("  Control 2 Register = 0x%08x - Compress enabled\n", ctrl2);
    }
  else if((ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_VERIFY)
    {
      printf("  Control 2 Register = 0x%08x - Compress verify\n", ctrl2);
    }
  else
    printf("  Control 2 Register = 0x%08x - Compress error\n", ctrl2);


  printf("  Internal Triggers (Live) = %d\n", itrigCnt);
  printf("  Trigger   Scaler         = %d\n", trigCnt);
  printf("  Trigger 2 Scaler         = %d\n", trig2Cnt);
  printf("  SyncReset Scaler         = %d\n", srCnt);
  printf("  Trigger Control          = 0x%08x\n", trigger_control);
  if(trigger_control & (FAV3_TRIGCTL_TRIGSTOP_EN | FAV3_TRIGCTL_BUSY_EN))
    {
      printf("  Lost Trigger Scaler      = %d\n", lost_trig_scal);
    }

  if(scaler_interval)
    {
      printf("  Block interval for scaler events = %d\n", scaler_interval);
    }

  if(csr & FAV3_CSR_BLOCK_READY)
    {
      printf("  Blocks in FIFO           = %d  (Block level = %d) - Block Available\n",
	     bcount, blevel);
      printf("  RAM Level (Bytes)        = %d \n", (ramWords * 8));
    }
  else if(csr & FAV3_CSR_EVENT_AVAILABLE)
    {
      printf("  Events in FIFO           = %d  (Block level = %d) - Data Available\n",
	     count, blevel);
      printf("  RAM Level (Bytes)        = %d \n", (ramWords * 8));
    }
  else
    {
      printf("  Events in FIFO           = %d  (Block level = %d)\n", count,
	     blevel);
    }

  printf("  BERR count (from module) = %d\n", berr_count);

  printf("  MGT Status Register      = 0x%08x ", mgtStatus);
  if(mgtStatus & (FAV3_MGT_GTX1_HARD_ERROR | FAV3_MGT_GTX1_SOFT_ERROR |
		  FAV3_MGT_GTX2_HARD_ERROR | FAV3_MGT_GTX2_SOFT_ERROR))
    printf(" - **Error Condition**\n");
  else
    printf("\n");


  return OK;
}

/**
 *  @ingroup Status
 *  @brief Print a summary of all initialized fADC250s
 *  @param sflag reserved for future use
 */

void
faV3GStatus(int sflag)
{
  int ifa, id, ii;
  faV3_t st[FAV3_MAX_BOARDS + 1];
  uint32_t a24addr[FAV3_MAX_BOARDS + 1];
  int nsb;

  FAV3LOCK;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      a24addr[id] = (uint32_t) ((u_long) FAV3p[id] - faV3A24Offset);
      st[id].version = vmeRead32(&FAV3p[id]->version);
      st[id].adr32 = vmeRead32(&FAV3p[id]->adr32);
      st[id].adr_mb = vmeRead32(&FAV3p[id]->adr_mb);

      st[id].ctrl1 = vmeRead32(&FAV3p[id]->ctrl1);
      st[id].ctrl2 = vmeRead32(&FAV3p[id]->ctrl2);

      st[id].csr = vmeRead32(&FAV3p[id]->csr);

      st[id].sys_mon = vmeRead32(&FAV3p[id]->sys_mon);

      st[id].adc.status0 =
	vmeRead32(&FAV3p[id]->adc.status0) & 0xFFFF;
      st[id].adc.status1 =
	vmeRead32(&FAV3p[id]->adc.status0) & 0xFFFF;
      st[id].adc.status2 =
	vmeRead32(&FAV3p[id]->adc.status0) & 0xFFFF;

      st[id].adc.config1 =
	vmeRead32(&FAV3p[id]->adc.config1) & 0xFFFF;
      st[id].adc.config2 =
	vmeRead32(&FAV3p[id]->adc.config2) & 0xFFFF;
      st[id].adc.config4 =
	vmeRead32(&FAV3p[id]->adc.config4) & 0xFFFF;

      st[id].adc.ptw = vmeRead32(&FAV3p[id]->adc.ptw);
      st[id].adc.pl = vmeRead32(&FAV3p[id]->adc.pl);
      st[id].adc.nsb = vmeRead32(&FAV3p[id]->adc.nsb);
      st[id].adc.nsa = vmeRead32(&FAV3p[id]->adc.nsa);

      st[id].blk_count = vmeRead32(&FAV3p[id]->blk_count);
      st[id].blocklevel = vmeRead32(&FAV3p[id]->blocklevel);
      st[id].ram_word_count =
	vmeRead32(&FAV3p[id]->ram_word_count) & FAV3_RAM_DATA_MASK;

      st[id].trig_count = vmeRead32(&(FAV3p[id]->trig_count));
      st[id].trig2_scal = vmeRead32(&FAV3p[id]->trig2_scal);
      st[id].syncreset_scal = vmeRead32(&FAV3p[id]->syncreset_scal);
      st[id].aux.berr_driven_count = vmeRead32(&FAV3p[id]->aux.berr_driven_count);

      st[id].aux.sparsify_control = vmeRead32(&FAV3p[id]->aux.sparsify_control);

      for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
	{
	  st[id].adc.pedestal[ii] = vmeRead32(&FAV3p[id]->adc.pedestal[ii]);
	}

      for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS/2; ii++)
	{
	  // FIXME: 2 channels per reg?
	  st[id].adc.thres[ii] = vmeRead32(&FAV3p[id]->adc.thres[ii]);
	}

    }
  FAV3UNLOCK;

  printf("\n");

  printf("                      fADC250 Module Configuration Summary\n\n");
  printf("     Firmware Rev   .................Addresses................\n");
  printf("Slot  Ctrl   Proc      A24        A32     A32 Multiblock Range   VXS Readout\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("0x%04x 0x%04x  ", st[id].version & 0xFFFF,
	     st[id].adc.status0 & FAV3_ADC_VERSION_MASK);

      printf("0x%06x  ", a24addr[id]);

      if(st[id].adr32 & FAV3_A32_ENABLE)
	{
	  printf("0x%08x  ", (st[id].adr32 & FAV3_A32_ADDR_MASK) << 16);
	}
      else
	{
	  printf("  Disabled  ");
	}

      if(st[id].adr_mb & FAV3_AMB_ENABLE)
	{
	  printf("0x%08x-0x%08x  ",
		 (st[id].adr_mb & FAV3_AMB_MIN_MASK) << 16,
		 (st[id].adr_mb & FAV3_AMB_MAX_MASK));
	}
      else
	{
	  printf("Disabled               ");
	}

      printf("%s",
	     (st[id].
	      ctrl2 & FAV3_CTRL_VXS_RO_ENABLE) ? " Enabled" : "Disabled");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");


  printf("\n");
  printf("      .Signal Sources..                        ..Channel...\n");
  printf("Slot  Clk   Trig   Sync     MBlk  Token  BERR  Enabled Mask\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("%s  ",
	     (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_INTERNAL ? " INT " : (st[id].
						ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_P0 ? " VXS " : (st[id].ctrl1 & FAV3_REF_CLK_MASK) ==
	     FAV3_REF_CLK_FP ? "  FP " : " ??? ");

      printf("%s  ",
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_INTERNAL ? " INT " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_VME ? " VME " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_P0 ? " VXS " :
	     (st[id].ctrl1 & FAV3_TRIG_MASK) == FAV3_TRIG_FP ? "  FP " : " ??? ");

      printf("%s    ",
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_VME ? " VME " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_P0_ISYNC ? " VXS " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_FP_ISYNC ? "  FP " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_P0 ? " VXS " :
	     (st[id].ctrl1 & FAV3_SRESET_MASK) == FAV3_SRESET_FP ? "  FP " :
	     " ??? ");

      printf("%s   ", (st[id].ctrl1 & FAV3_ENABLE_MULTIBLOCK) ? "YES" : " NO");

      printf("%s",
	     st[id].ctrl1 & (FAV3_MB_TOKEN_VIA_P0) ? " P0" :
	     st[id].ctrl1 & (FAV3_MB_TOKEN_VIA_P2) ? " P0" : " NO");
      printf("%s  ",
	     st[id].ctrl1 & (FAV3_FIRST_BOARD) ? "-F" :
	     st[id].ctrl1 & (FAV3_LAST_BOARD) ? "-L" : "  ");

      printf("%s     ", st[id].ctrl1 & FAV3_ENABLE_BERR ? "YES" : " NO");

      printf("0x%04X", ~(st[id].adc.config2 & FAV3_ADC_CHAN_MASK) & 0xFFFF);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                         fADC250 Processing Mode Config\n\n");
  printf("      Block          ...[nanoseconds]...     \n");
  printf("Slot  Level  Mode    PL   PTW   NSB  NSA  NP   Compression  Playback  Sparse\n");
  printf("--------------------------------------------------------------------------------\n");

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d    ", id);

      printf("%3d    ", st[id].blocklevel & FAV3_BLOCK_LEVEL_MASK);

      printf("%2d   ", (st[id].adc.config1 & FAV3_ADC_PROC_MASK) + 1);

      printf("%4d  ", (st[id].adc.pl & 0xFFFF) * FAV3_ADC_NS_PER_CLK);

      printf("%4d   ", ((st[id].adc.ptw & 0xFFFF) + 1) * FAV3_ADC_NS_PER_CLK);

      nsb = st[id].adc.nsb & FAV3_ADC_NSB_READBACK_MASK;
      nsb =
	(nsb & 0x7) * ((nsb & FAV3_ADC_NSB_NEGATIVE) ? -1 : 1) *
	FAV3_ADC_NS_PER_CLK;
      printf("%3d  ", nsb);

      printf("%3d   ",
	     (st[id].adc.nsa & FAV3_ADC_NSA_READBACK_MASK) * FAV3_ADC_NS_PER_CLK);

      printf("%1d      ",
	     ((st[id].adc.config1 & FAV3_ADC_PEAK_MASK) >> 4) + 1);


      printf("%s  ",
	     ((st[id].ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_DISABLE) ?
	     "Disabled" :

	     ((st[id].ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_ENABLE) ?
	     " Enabled" :

	     ((st[id].ctrl2 & FAV3_CTRL_COMPRESS_MASK) == FAV3_CTRL_COMPRESS_VERIFY) ?
	     "  Verify" :

	     "UNKNOWN");

      printf("%s ",
	     (st[id].adc.config1 & FAV3_ADC_PLAYBACK_MODE) >> 7 ? " Enabled" :
	     "Disabled");

      printf("%s",
	     (st[id].aux.sparsify_control & FAV3_SPARSE_CONTROL_BYPASS) ? "Bypassed" :
	     " Enabled");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("           .........fADC250 Signal Scalers..........     ..System Monitor..\n");
  printf("Slot       Trig1       Trig2   SyncReset        BERR     TempC   1.0V   2.5V\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d   ", id);

      printf("%10d  ", st[id].trig_count);

      printf("%10d  ", st[id].trig2_scal);

      printf("%10d  ", st[id].syncreset_scal);

      printf("%10d     ", st[id].aux.berr_driven_count);

      double fpga_temperature =
	(((double) (st[id].sys_mon & FAV3_SYSMON_CTRL_TEMP_MASK)) *
	 (503.975 / 1024.0)) - 273.15;
      printf("%3.1f    ", fpga_temperature);

      double fpga_1V =
	(((double)
	  ((st[id].sys_mon & FAV3_SYSMON_FPGA_CORE_V_MASK) >> 11)) *
	 (3.0 / 1024.0));
      printf("%3.1f    ", fpga_1V);

      double fpga_25V =
	(((double)
	  ((st[id].sys_mon & FAV3_SYSMON_FPGA_AUX_V_MASK) >> 22)) *
	 (3.0 / 1024.0));
      printf("%3.1f    ", fpga_25V);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                              fADC250 Data Status\n\n");
  printf("                                                  .......Error Status.......\n");
  printf("      Trigger   Block                             Local   ....... MGT ......\n");
  printf("Slot  Source    Ready  Blocks In Fifo  RAM Level   Bus    Reset  Lane  Chan\n");
  printf("--------------------------------------------------------------------------------\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      printf(" %2d  ", id);

      printf("%s    ",
	     st[id].ctrl2 & FAV3_CTRL_ENABLE_MASK ? " Enabled" : "Disabled");

      printf("%s       ", st[id].csr & FAV3_CSR_BLOCK_READY ? "YES" : " NO");

      printf("%10d ", st[id].blk_count & FAV3_BLOCK_COUNT_MASK);

      printf("%10d  ", (st[id].ram_word_count & FAV3_RAM_DATA_MASK) * 8);

      printf("%s     ", st[id].csr & FAV3_CSR_ERROR_MASK ? "ERROR" : "  OK ");

      /* printf("%s  ", (st[id].gtx_ctrl & 0x1) ? " ON" : "OFF"); */

      /* printf("%s  ", ((st[id].gtx_status & 0x1) != 0x1) ? "Down" : " Up "); */

      /* printf("%s ", ((st[id].gtx_status & 0x6) != 0x6) ? "Down" : " Up "); */

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                      fADC250 Trigger Path Processing\n\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      printf("           .......TET.......                                           \n");
      printf("Slot  Ch   Readout   Trigger      Ped\n");
      printf("--------------------------------------------------------------------------------\n");

      id = faV3Slot(ifa);

      int ichan;
      for(ichan = 0; ichan < FAV3_MAX_ADC_CHANNELS; ichan++)
	{
	  if(ichan == 0)
	    printf(" %2d", id);
	  else
	    printf("   ");

	  printf("   ");
	  printf("%2d      ", ichan);

	  int NSB = (st[id].adc.nsb & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
	  int NSA = (st[id].adc.nsa & 0xFFFF) * FAV3_ADC_NS_PER_CLK;

	  float ped_trg =
	    4.0 *
	    ((float) (st[id].adc.pedestal[ichan] & FAV3_ADC_PEDESTAL_MASK)) /
	    ((float) (NSA + NSB));

	  int tet_trg =
	    (st[id].adc.thres[ichan] & FAV3_THR_VALUE_MASK) - (int) ped_trg;

	  int tet_readout = (st[id].adc.thres[ichan] & FAV3_THR_IGNORE_MASK) ? 0
	    : ((st[id].adc.thres[ichan] & FAV3_THR_VALUE_MASK) - (int) ped_trg);

	  printf("%4d      ", tet_readout);

	  printf("%4d   ", tet_trg);

	  printf("%8.3f     ", ped_trg);


	  printf("\n");
	}

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("\n");

}

/**
 *  @ingroup Status
 *  @brief Get the firmware versions of each FPGA
 *
 *  @param id Slot number
 *  @param  pval
 *    -  0: Print nothing to stdout
 *    - !0: Print firmware versions to stdout
 *
 *  @return (fpga_control.version) | (fpga_processing.version<<16)
 *            or -1 if error
 */

uint32_t
faV3GetFirmwareVersions(int id, int pflag)
{
  uint32_t cntl = 0, proc = 0, rval = 0;
  CHECKID;

  FAV3LOCK;
  /* Control FPGA firmware version */
  cntl = vmeRead32(&FAV3p[id]->version) & 0xFFFF;

  /* Processing FPGA firmware version */
  proc = vmeRead32(&(FAV3p[id]->adc.status0)) & FAV3_ADC_VERSION_MASK;
  FAV3UNLOCK;

  rval = (cntl) | (proc << 16);

  if(pflag)
    {
      printf("%s:  Board Firmware Rev/ID = 0x%04x : ADC Processing Rev = 0x%04x\n",
	     __func__, cntl, proc);
    }

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Configure the processing type/mode
 *
 *  @param id Slot number
 *  @param pmode  Processing Mode
 *  @param  PL  Window Latency
 *  @param PTW  Window Width
 *  @param NSB  If NSB > 0: Number of samples before pulse over threshold included in sum
 *                 NSB < 0: Number of samples after threshold excluded from sum
 *  @param NSA  Number of samples after pulse over threshold to be included in sum
 *  @param NP   Number of pulses processed per window
 *
 *    Note:
 *     - PL must be greater than PTW
 *     - NSA+NSB must be an odd number
 *
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
		uint32_t NSB, uint32_t NSA, uint32_t NP)
{

  int err = 0;
  uint32_t ptw_last_adr, ptw_max_buf;


  CHECKID;

  if((pmode <= 0) || (pmode > 8))
    {
      printf("faV3SetProcMode: ERROR: Processing mode (%d) out of range (pmode= 1-8)\n",
	     pmode);
      return (ERROR);
    }
  else
    {
      /*       if((pmode>3)&&(pmode<8))  */
      /*        { */
      /*          printf("faV3SetProcMode: ERROR: Processing mode (%d) not implemented \n",pmode); */
      /*        } */
    }

  if(NP > 4)
    {
      printf("faV3SetProcMode: ERROR: Invalid Peak count %d (must be 0-4)\n",
	     NP);
      return (ERROR);
    }

  /*Defaults */
  if((PL == 0) || (PL > FAV3_ADC_MAX_PL))
    PL = FAV3_ADC_DEFAULT_PL;
  if((PTW == 0) || (PTW > FAV3_ADC_MAX_PTW))
    PTW = FAV3_ADC_DEFAULT_PTW;
  if((NSB == 0) || (NSB > FAV3_ADC_MAX_NSB))
    NSB = FAV3_ADC_DEFAULT_NSB;
  if((NSA == 0) || (NSA > FAV3_ADC_MAX_NSA))
    NSA = FAV3_ADC_DEFAULT_NSA;
  if((NP == 0) && (pmode != FAV3_ADC_PROC_MODE_WINDOW))
    NP = FAV3_ADC_DEFAULT_NP;

  /* Consistancy check */
  if(PTW > PL)
    {
      err++;
      printf("faV3SetProcMode: ERROR: Window must be <= Latency\n");
    }
  if(((NSB + NSA) % 2) == 0)
    {
      err++;
      printf("faV3SetProcMode: ERROR: NSB+NSA must be an odd number\n");
    }

  /* Calculate Proc parameters */
  ptw_max_buf = (uint32_t) (2016 / (PTW + 8));
  ptw_last_adr = ptw_max_buf * (PTW + 8) - 1;

  faV3SetupADC(id, 0);

  FAV3LOCK;
  /* Disable ADC processing while writing window info */
  vmeWrite32(&(FAV3p[id]->adc.config1), ((pmode - 1) | (NP << 4)));
  vmeWrite32(&(FAV3p[id]->adc.config2), faV3ChanDisableMask[id]);
  vmeWrite32(&(FAV3p[id]->adc.pl), PL);
  vmeWrite32(&(FAV3p[id]->adc.ptw), PTW);
  vmeWrite32(&(FAV3p[id]->adc.nsb), NSB);
  vmeWrite32(&(FAV3p[id]->adc.nsa), NSA);
  vmeWrite32(&(FAV3p[id]->adc.ptw_max_buf), ptw_max_buf);
  vmeWrite32(&(FAV3p[id]->adc.ptw_last_adr), ptw_last_adr);
  /* Enable ADC processing */
  vmeWrite32(&(FAV3p[id]->adc.config1),
	     ((pmode - 1) | (NP << 4) | FAV3_ADC_PROC_ENABLE));

  FAV3UNLOCK;

  return (OK);
}

/**
 *  @ingroup Config
 *  @brief Configure the processing type/mode for all initialized fADC250s
 *
 *  @param id Slot number
 *  @param pmode  Processing Mode
 *  @param  PL  Window Latency
 *  @param PTW  Window Width
 *  @param NSB  If NSB > 0: Number of samples before pulse over threshold included in sum
 *                 NSB < 0: Number of samples after threshold excluded from sum
 *  @param NSA  Number of samples after pulse over threshold to be included in sum
 *  @param NP   Number of pulses processed per window
 *
 *    Note:
 *     - PL must be greater than PTW
 *     - NSA+NSB must be an odd number
 *
 *  @return OK if successful, otherwise ERROR.
 */

void
faV3GSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
		 uint32_t NSB, uint32_t NSA, uint32_t NP)
{
  int ii, res;

  for (ii=0;ii<nfaV3;ii++)
    {
      res = faV3SetProcMode(faV3ID[ii], pmode, PL, PTW, NSB, NSA, NP);
      if(res<0)
	printf("ERROR: slot %d, in faV3SetProcMode()\n", faV3ID[ii]);
    }
}

int
faV3GetProcMode(int id, int *pmode, uint32_t * PL, uint32_t * PTW,
		uint32_t * NSB, uint32_t * NSA, uint32_t * NP)
{
  uint32_t tmp;

  CHECKID;

  *PTW = (vmeRead32(&(FAV3p[id]->adc.ptw)) & 0xFFFF);
  *PL = (vmeRead32(&(FAV3p[id]->adc.pl)) & 0xFFFF);
  *NSB = (vmeRead32(&(FAV3p[id]->adc.nsb)) & 0xFFFF);
  *NSA = (vmeRead32(&(FAV3p[id]->adc.nsa)) & 0xFFFF);

  tmp = (vmeRead32(&(FAV3p[id]->adc.config1)) & 0xFFFF);
  *pmode = (tmp & FAV3_ADC_PROC_MASK) + 1;
  *NP = (tmp & FAV3_ADC_PEAK_MASK) >> 4;

  return (0);
}


/**
 *  @ingroup Config
 *  @brief Set the maximum number of unacknowledged triggers before module
 *         stops accepting incoming triggers.
 *  @param id Slot number
 *  @param trigger_max Limit for maximum number of unacknowledged triggers.
 *         If 0, disables the condition.
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetTriggerStopCondition(int id, int trigger_max)
{
  CHECKID;

  if(trigger_max > 0xFF)
    {
      printf("%s: ERROR: Invalid trigger_max (%d)\n", __func__, trigger_max);
      return ERROR;
    }

  FAV3LOCK;
  if(trigger_max > 0)
    {
      vmeWrite32(&FAV3p[id]->trigger_control,
		 (vmeRead32(&FAV3p[id]->trigger_control) &
		  ~(FAV3_TRIGCTL_TRIGSTOP_EN | FAV3_TRIGCTL_MAX2_MASK)) |
		 (FAV3_TRIGCTL_TRIGSTOP_EN | (trigger_max << 16)));
    }
  else
    {
      vmeWrite32(&FAV3p[id]->trigger_control,
		 (vmeRead32(&FAV3p[id]->trigger_control) &
		  ~(FAV3_TRIGCTL_TRIGSTOP_EN | FAV3_TRIGCTL_MAX2_MASK)));
    }
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the maximum number of unacknowledged triggers before module
 *         asserts BUSY.
 *  @param id Slot number
 *  @param trigger_max Limit for maximum number of unacknowledged triggers
 *         If 0, disables the condition
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetTriggerBusyCondition(int id, int trigger_max)
{
  CHECKID;

  if(trigger_max > 0xFF)
    {
      printf("%s: ERROR: Invalid trigger_max (%d)\n", __func__, trigger_max);
      return ERROR;
    }

  FAV3LOCK;
  if(trigger_max > 0)
    {
      vmeWrite32(&FAV3p[id]->trigger_control,
		 (vmeRead32(&FAV3p[id]->trigger_control) &
		  ~(FAV3_TRIGCTL_BUSY_EN | FAV3_TRIGCTL_MAX1_MASK)) |
		 (FAV3_TRIGCTL_BUSY_EN | (trigger_max)));
    }
  else
    {
      vmeWrite32(&FAV3p[id]->trigger_control,
		 (vmeRead32(&FAV3p[id]->trigger_control) &
		  ~(FAV3_TRIGCTL_BUSY_EN | FAV3_TRIGCTL_MAX1_MASK)));
    }
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the number of samples that are included before and after
 *    threshold crossing that are sent through the trigger path
 *  @param id Slot number
 *  @param TNSB Number of samples before threshold crossing
 *  @param TNSA Number of samples after threshold crossing
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3SetTriggerPathSamples(int id, uint32_t TNSB, uint32_t TNSA)
{
  uint32_t readback_nsb = 0, readback_nsa = 0;

  CHECKID;

  // FIXME: Check for updated v3 values
  if((TNSA < FAV3_ADC_MIN_TNSA) || (TNSA > FAV3_ADC_MAX_TNSA))
    {
      printf("%s: WARN: TNSA (%d) out of range. Setting to %d\n",
	     __func__, TNSA, FAV3_ADC_DEFAULT_TNSA);
      TNSA = FAV3_ADC_DEFAULT_TNSA;
    }

  if((TNSB < FAV3_ADC_MIN_TNSB) || (TNSB > FAV3_ADC_MAX_TNSB))
    {
      printf("%s: WARN: TNSB (%d) out of range. Setting to %d\n",
	     __func__, TNSB, FAV3_ADC_DEFAULT_TNSB);
      TNSB = FAV3_ADC_DEFAULT_TNSB;
    }

  FAV3LOCK;

  readback_nsb = vmeRead32(&FAV3p[id]->adc.nsb) & FAV3_ADC_NSB_READBACK_MASK;
  readback_nsa = vmeRead32(&FAV3p[id]->adc.nsa) & FAV3_ADC_NSA_READBACK_MASK;

  vmeWrite32(&FAV3p[id]->adc.nsb, (TNSB << 9) | readback_nsb);
  vmeWrite32(&FAV3p[id]->adc.nsa, (TNSA << 9) | readback_nsa);

  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the number of samples that are included before and after
 *    threshold crossing that are sent through the trigger path for
 *    all initialized fADC250s
 *  @param NSB Number of samples before threshold crossing
 *  @param NSA Number of samples after threshold crossing
 *  @sa faV3SetTriggerPathSamples
 */
void
faV3GSetTriggerPathSamples(uint32_t TNSA, uint32_t TNSAT)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faV3SetTriggerPathSamples(faV3ID[ii], TNSA, TNSAT);
      if(res < 0)
	printf("ERROR: slot %d, in faV3SetTriggerPathSamples()\n", faV3ID[ii]);
    }

}

/**
 *  @ingroup Config
 *  @brief Set the threshold used to determine what samples are sent through the
 *     trigger path
 *  @param id Slot number
 *  @param threshold Trigger Path Threshold
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3SetTriggerPathThreshold(int id, uint32_t TPT)
{
  CHECKID;

  if(TPT > FAV3_ADC_MAX_TPT)
    {
      printf("%s: WARN: TPT (%d) greater than MAX.  Setting to %d\n",
	     __func__, TPT, FAV3_ADC_MAX_TPT);
      TPT = FAV3_ADC_MAX_TPT;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->adc.config3,
	     (vmeRead32(&FAV3p[id]->adc.config3) & ~FAV3_ADC_CONFIG3_TPT_MASK) | TPT);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the threshold used to determine what samples are sent through the
 *     trigger path for all initialized fADC250s
 *  @param threshold Trigger Path Threshold
 *  @sa faV3SetTriggerPathThreshold
 */
void
faV3GSetTriggerPathThreshold(uint32_t TPT)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faV3SetTriggerPathThreshold(faV3ID[ii], TPT);
      if(res < 0)
	printf("ERROR: slot %d, in faV3SetTriggerPathThreshold()\n",
	       faV3ID[ii]);
    }
}



/**
 *  @ingroup Config
 *  @brief Static routine, to wait for the ADC processing chip ready bit
 *     before proceeding with further programming
 *  @param id Slot number
 *
 */

static int32_t
faV3ADCTestReady(int id)
{
  /* returns positive value (number of attempts) if ADC chips are ready to be written to */
  /* otherwise returns 0; makes 100 attempts */
  int32_t test, ii, adc_ready = 0;

  for(ii=1; ii<=100; ii++)
    {
      test = vmeRead32(&FAV3p[id]->adc.status0) & 0x8000;
      if( test == 0x8000 )
	{
	  adc_ready = ii;
	  break;
	}
    }

  return adc_ready;
}

static int32_t
faV3ADCWriteAll(int id, uint32_t value)
{
  /* broadcasts 'value' to all ADC chips */
  /* 'value' contains ADC register address (bits 15-8) and data (bits 7-0)  */
  /* 'value' bits 31-16 are ignored */
  int32_t rval = OK, adc_ready;

  adc_ready = faV3ADCTestReady(id);
  printf("+++++ adc_ready (start) = %d\n", adc_ready);

  vmeWrite32(&FAV3p[id]->adc.config5, value);		/* set up address & data */

  vmeWrite32(&FAV3p[id]->adc.config4, 0x40);		/* write all */
  adc_ready = faV3ADCTestReady(id);
  printf("+++++ adc_ready (1) = %d\n", adc_ready);

  vmeWrite32(&FAV3p[id]->adc.config4, 0xC0);
  adc_ready = faV3ADCTestReady(id);
  printf("+++++ adc_ready (2) = %d\n", adc_ready);

  vmeWrite32(&FAV3p[id]->adc.config4, 0x40);
  adc_ready = faV3ADCTestReady(id);
  printf("+++++ adc_ready (end) = %d\n", adc_ready);

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Configure the ADC Processing in "Normal Mode"
 *
 *  @param id Slot number
 *  @param mode Reserved for future use
 *
 */

// FIXME: Call this from SetProcMode

int32_t
faV3SetupADC(int id, int32_t mode)
{
  int32_t rval = OK;

  CHECKID;

  mode = 0;
  taskDelay(1);

  printf("%s(%d): ---- Initializing ADC chips ----\n",
	 __func__, id);

  vmeWrite32(&FAV3p[id]->adc.config4, 0x0);			/* reset adc chip */
  taskDelay(1);

  vmeWrite32(&FAV3p[id]->adc.config4, 0x10);			/* reset adc chip */
  taskDelay(1);

  vmeWrite32(&FAV3p[id]->adc.config4, 0x0);			/* reset adc chip */
  taskDelay(1);

  faV3ADCWriteAll(id, 0x0F02);			/* CML enable */

  faV3ADCWriteAll(id, 0x179E);			/* output clk delay */

  faV3ADCWriteAll(id, 0xFF01);			/* transfer register values */

  printf("%s(%d):   ---- ADC chips initialized ----\n",
	 __func__, id);

  printf("%s(%d):   ---- Put ADC chips in normal running mode ----\n",
	 __func__, id);

  faV3ADCWriteAll(id, 0x0D00);

  faV3ADCWriteAll(id, 0xFF01);			/* transfer register values */

  printf("%s(%d):   ---- ADC chips in normal running mode ----\n",
	 __func__, id);

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Setup FADC Progammable Pulse Generator
 *
 *  @param id Slot number
 *  @param sdata  Array of sample data to be programmed
 *  @param nsamples Number of samples contained in sdata
 *
 *  @sa faPPGEnable faPPGDisable
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetPPG(int id, uint16_t *sdata, int nsamples)
{

  int ii;
  uint16_t rval;

  CHECKID;

  if(sdata == NULL)
    {
      printf("faV3SetPPG: ERROR: Invalid Pointer to sample data\n");
      return (ERROR);
    }

  /*Defaults */
  if((nsamples <= 0) || (nsamples > FAV3_PPG_MAX_SAMPLES))
    nsamples = FAV3_PPG_MAX_SAMPLES;

  FAV3LOCK;
  for(ii = 0; ii < (nsamples - 2); ii++)
    {
      vmeWrite32(&FAV3p[id]->adc.test_wave, (sdata[ii] | FAV3_PPG_WRITE_VALUE));
      rval = vmeRead32(&FAV3p[id]->adc.test_wave);
      if((rval & FAV3_PPG_SAMPLE_MASK) != sdata[ii])
	printf("faV3SetPPG: ERROR: Write error %x != %x (ii=%d)\n", rval,
	       sdata[ii], ii);

    }

  vmeWrite32(&FAV3p[id]->adc.test_wave,
	     (sdata[(nsamples - 2)] & FAV3_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAV3p[id]->adc.test_wave);
  if(rval != sdata[(nsamples - 2)])
    printf("faV3SetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples - 2]);
  vmeWrite32(&FAV3p[id]->adc.test_wave,
	     (sdata[(nsamples - 1)] & FAV3_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAV3p[id]->adc.test_wave);
  if(rval != sdata[(nsamples - 1)])
    printf("faV3SetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples - 1]);

  /*   vmeWrite32(&FAV3p[id]->adc.test_wave, (sdata[(nsamples-2)]&FAV3_PPG_SAMPLE_MASK)); */
  /*   vmeWrite32(&FAV3p[id]->adc.test_wave, (sdata[(nsamples-1)]&FAV3_PPG_SAMPLE_MASK)); */

  FAV3UNLOCK;

  return (OK);
}

/**
 *  @ingroup Config
 *  @brief Enable the programmable pulse generator
 *  @param id Slot number
 *  @sa faV3SetPPG faV3PPGDisable
 */

int
faV3PPGEnable(int id)
{
  uint16_t val1;

  CHECKID;

  FAV3LOCK;
  val1 = (vmeRead32(&FAV3p[id]->adc.config1) & 0xFFFF);
  val1 |= (FAV3_PPG_ENABLE | 0xff00);
  vmeWrite32(&FAV3p[id]->adc.config1, val1);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable the programmable pulse generator
 *  @param id Slot number
 *  @sa faV3SetPPG faV3PPGEnable
 */

int
faV3PPGDisable(int id)
{
  uint16_t val1;

  CHECKID;

  FAV3LOCK;
  val1 = (vmeRead32(&FAV3p[id]->adc.config1) & 0xFFFF);
  val1 &= ~FAV3_PPG_ENABLE;
  val1 &= ~(0xff00);
  vmeWrite32(&FAV3p[id]->adc.config1, val1);
  FAV3UNLOCK;

  return OK;
}

/**
 * @ingroup Config
 * @brief Summary
 * @param id Slot number
 * @param itrig_width Internal pulse width [1,255] 4ns -> 1020ns
 * @param itrig_dt Deadtime between triggers [1,255] 4ns -> 1020ns
 * @return Current value in trig_cfg register: (itrig_width << 16) | (itrig_dt)
 */

uint32_t
faV3ItrigControl(int id, uint16_t itrig_width, uint16_t itrig_dt)
{
  uint32_t retval = 0;

  CHECKID;

  /* If both parameters = 0 then just return the current value */
  FAV3LOCK;
  if((itrig_width == 0) && (itrig_dt == 0))
    {
      retval = vmeRead32(&(FAV3p[id]->trig_cfg));
    }
  else
    {
      if((itrig_width == 0) || (itrig_width > 255))
	itrig_width = 0xc;	/* default 48ns */
      if((itrig_dt == 0) || (itrig_dt > 255))
	itrig_dt = 0xa;		/* default 40ns */

      vmeWrite32(&(FAV3p[id]->trig_cfg), (itrig_width << 16) | itrig_dt);
      retval = vmeRead32(&(FAV3p[id]->trig_cfg));
    }
  FAV3UNLOCK;

  return (retval);
}



/**
 *  @ingroup Readout
 *  @brief General Data readout routine
 *
 *  @param  id     Slot number of module to read
 *  @param  data   local memory address to place data
 *  @param  nwrds  Max number of words to transfer
 *  @param  rflag  Readout Flag
 * <pre>
 *              0 - programmed I/O from the specified board
 *              1 - DMA transfer using Universe/Tempe DMA Engine
 *                    (DMA VME transfer Mode must be setup prior)
 *              2 - Multiblock DMA transfer (Multiblock must be enabled
 *                     and daisychain in place or SD being used)
 * </pre>
 *  @return Number of words inserted into data if successful.  Otherwise ERROR.
 */

int
faV3ReadBlock(int id, volatile uint32_t *data, int nwrds, int rflag)
{
  int ii;
  int stat, retVal, xferCount, rmode, async;
  int dCnt, berr = 0;
  int dummy = 0;
  volatile uint32_t *laddr;
  uint32_t bhead, ehead, val;
  uint32_t vmeAdr, csr;

  CHECKID;

  if(data == NULL)
    {
      logMsg("faV3ReadBlock: ERROR: Invalid Destination address\n", 0, 0, 0, 0,
	     0, 0);
      return (ERROR);
    }

  faV3BlockError = FAV3_BLOCKERROR_NO_ERROR;
  if(nwrds <= 0)
    nwrds = (FAV3_MAX_ADC_CHANNELS * FAV3_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag & 0x0f;

  if(rmode >= 1)
    {				/* Block Transfers */

      /*Assume that the DMA programming is already setup. */
      /* Don't Bother checking if there is valid data - that should be done prior
         to calling the read routine */

      /* Check for 8 byte boundary for address - insert dummy word (Slot 0 FADC Dummy DATA) */
      if((u_long) (data) & 0x7)
	{
#ifdef VXWORKS
	  *data = FAV3_DUMMY_DATA;
#else
	  *data = LSWAP(FAV3_DUMMY_DATA);
#endif
	  dummy = 1;
	  laddr = (data + 1);
	}
      else
	{
	  dummy = 0;
	  laddr = data;
	}

      if(rmode == 1)
	{
	  /* Check for valid A32 data pointer */
	  if(FAV3pd[id] == NULL)
	    {
	      logMsg("faV3ReadBlock(id = %d): ERROR: A32 Data Pointer not initialized\n",
		     id, 0, 0, 0, 0, 0);
	      return ERROR;
	    }
	}

      FAV3LOCK;
      if(rmode == 2)
	{			/* Multiblock Mode */
	  if((vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_FIRST_BOARD) == 0)
	    {
	      logMsg("faV3ReadBlock: ERROR: FADC in slot %d is not First Board\n",
		 id, 0, 0, 0, 0, 0);
	      FAV3UNLOCK;
	      return (ERROR);
	    }
	  vmeAdr = (uint32_t) ((u_long) (FAV3pmb) - faV3A32Offset);
	}
      else
	{
	  vmeAdr = (uint32_t) ((u_long) (FAV3pd[id]) - faV3A32Offset);
	}

#ifdef HALLB
      /*
	printf("faReadBlock: faV3A32Offset=0x%08x vmeAdr=0x%08x laddr=0x%08x nwrds=%d\n",faV3A32Offset,vmeAdr,laddr,nwrds);fflush(stdout);
      */
      retVal = usrVme2MemDmaStart(vmeAdr, (uint32_t) laddr, (nwrds << 2));
#else
#ifdef VXWORKS
      retVal = sysVmeDmaSend((uint32_t) laddr, vmeAdr, (nwrds << 2), 0);
#else
      retVal = vmeDmaSend((u_long) laddr, vmeAdr, (nwrds << 2));
#endif
#endif
      if(retVal != 0)
	{
	  logMsg("faV3ReadBlock: ERROR in DMA transfer Initialization 0x%x\n",
		 retVal, 0, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (retVal);
	}

      /* Wait until Done or Error */
#ifdef HALLB
      retVal = usrVme2MemDmaDone();
#else
#ifdef VXWORKS
      retVal = sysVmeDmaDone(10000, 1);
#else
      retVal = vmeDmaDone();
#endif
#endif

      if(retVal > 0)
	{
	  /* Check to see that Bus error was generated by FADC */
	  if(rmode == 2)
	    {
	      csr = vmeRead32(&(FAV3p[faV3MaxSlot]->csr));	/* from Last FADC */
	    }
	  else
	    {
	      csr = vmeRead32(&(FAV3p[id]->csr));	/* from Last FADC */
	    }
	  stat = (csr) & FAV3_CSR_BERR_STATUS;

	  if((retVal > 0) && (stat))
	    {
#ifdef HALLB
	      xferCount = ((retVal >> 2) + dummy);	/* Number of Longwords transfered */
#else
#ifdef VXWORKS
	      xferCount = (nwrds - (retVal >> 2) + dummy);	/* Number of Longwords transfered */
#else
	      xferCount = ((retVal >> 2) + dummy);	/* Number of Longwords transfered */
#endif
#endif
	      FAV3UNLOCK;
	      return (xferCount);	/* Return number of data words transfered */
	    }
	  else
	    {
#if defined(VXWORKS) && !defined(HALLB)
	      xferCount = (nwrds - (retVal >> 2) + dummy);	/* Number of Longwords transfered */
	      logMsg
		("faReadBlock: DMA transfer terminated by unknown BUS Error (csr=0x%x xferCount=%d id=%d)\n",
		 csr, xferCount, id, 0, 0, 0);
	      faV3BlockError = FAV3_BLOCKERROR_UNKNOWN_BUS_ERROR;
#else
	      xferCount = ((retVal >> 2) + dummy);	/* Number of Longwords transfered */
	      if((retVal >> 2) == nwrds)
		{
		  logMsg
		    ("faReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",
		     nwrds, 0, 0, 0, 0, 0);
		  faV3BlockError = FAV3_BLOCKERROR_TERM_ON_WORDCOUNT;
		}
	      else
		{
		  logMsg
		    ("faReadBlock: DMA transfer terminated by unknown BUS Error (csr=0x%x xferCount=%d id=%d)\n",
		     csr, xferCount, id, 0, 0, 0);
		  faV3BlockError = FAV3_BLOCKERROR_UNKNOWN_BUS_ERROR;
		}
#endif
	      FAV3UNLOCK;
	      if(rmode == 2)
		faV3GetTokenStatus(1);

	      return (xferCount);
	    }
	}
      else if(retVal == 0)
	{			/* Block Error finished without Bus Error */
#if defined(VXWORKS) && !defined(HALLB)
	  logMsg
	    ("faReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",
	     nwrds, 0, 0, 0, 0, 0);
#else
	  logMsg
	    ("faReadBlock: WARN: DMA transfer returned zero word count 0x%x\n",
	     nwrds, 0, 0, 0, 0, 0);
#endif
	  faV3BlockError = FAV3_BLOCKERROR_ZERO_WORD_COUNT;
	  FAV3UNLOCK;

	  if(rmode == 2)
	    faV3GetTokenStatus(1);

	  return (nwrds);
	}
      else
	{			/* Error in DMA */
#if defined(VXWORKS) && !defined(HALLB)
	  logMsg("faV3ReadBlock: ERROR: sysVmeDmaDone returned an Error\n", 0,
		 0, 0, 0, 0, 0);
#else
	  logMsg("faV3ReadBlock: ERROR: vmeDmaDone returned an Error\n", 0, 0,
		 0, 0, 0, 0);
#endif
	  faV3BlockError = FAV3_BLOCKERROR_DMADONE_ERROR;
	  FAV3UNLOCK;

	  if(rmode == 2)
	    faV3GetTokenStatus(1);

	  return (retVal >> 2);
	}

    }
  else
    {				/*Programmed IO */

      /* Check if Bus Errors are enabled. If so then disable for Prog I/O reading */
      FAV3LOCK;
      berr = vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_ENABLE_BERR;
      if(berr)
	vmeWrite32(&(FAV3p[id]->ctrl1),
		   vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_BERR);

      dCnt = 0;
      /* Read Block Header - should be first word */
      bhead = (uint32_t) * FAV3pd[id];
#ifndef VXWORKS
      bhead = LSWAP(bhead);
#endif
      if((bhead & FAV3_DATA_TYPE_DEFINE)
	 && ((bhead & FAV3_DATA_TYPE_MASK) == FAV3_DATA_BLOCK_HEADER))
	{
	  ehead = (uint32_t) * FAV3pd[id];
#ifndef VXWORKS
	  ehead = LSWAP(ehead);
#endif

#ifdef VXWORKS
	  data[dCnt] = bhead;
#else
	  data[dCnt] = LSWAP(bhead);	/* Swap back to little-endian */
#endif
	  dCnt++;
#ifdef VXWORKS
	  data[dCnt] = ehead;
#else
	  data[dCnt] = LSWAP(ehead);	/* Swap back to little-endian */
#endif
	  dCnt++;
	}
      else
	{
	  /* We got bad data - Check if there is any data at all */
	  if((vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK) == 0)
	    {
	      logMsg("faV3ReadBlock: FIFO Empty (0x%08x)\n", bhead, 0, 0, 0, 0,
		     0);
	      FAV3UNLOCK;
	      return (0);
	    }
	  else
	    {
	      logMsg("faV3ReadBlock: ERROR: Invalid Header Word 0x%08x\n",
		     bhead, 0, 0, 0, 0, 0);
	      FAV3UNLOCK;
	      return (ERROR);
	    }
	}

      ii = 0;
      while(ii < nwrds)
	{
	  val = (uint32_t) * FAV3pd[id];
	  data[ii + 2] = val;
#ifndef VXWORKS
	  val = LSWAP(val);
#endif
	  if((val & FAV3_DATA_TYPE_DEFINE)
	     && ((val & FAV3_DATA_TYPE_MASK) == FAV3_DATA_BLOCK_TRAILER))
	    break;
	  ii++;
	}
      ii++;
      dCnt += ii;

      if(berr)
	vmeWrite32(&(FAV3p[id]->ctrl1),
		   vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_ENABLE_BERR);

      FAV3UNLOCK;
      return (dCnt);
    }

  FAV3UNLOCK;
  return (OK);

}				//End faReadBlock

/**
 *  @ingroup Status
 *  @brief Return the type of error that occurred while attempting a
 *    block read from faV3ReadBlock.
 *  @param pflag
 *     - >0: Print error message to standard out
 *  @sa faReadBlock
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3GetBlockError(int pflag)
{
  int rval = 0;
  const char *block_error_names[FAV3_BLOCKERROR_NTYPES] = {
    "NO ERROR",
    "DMA Terminated on Word Count",
    "Unknown Bus Error",
    "Zero Word Count",
    "DmaDone Error"
  };

  rval = faV3BlockError;
  if(pflag)
    {
      if(rval != FAV3_BLOCKERROR_NO_ERROR)
	{
	  logMsg("faV3GetBlockError: Block Transfer Error: %s\n",
		 block_error_names[rval], 2, 3, 4, 5, 6);
	}
    }

  return rval;
}

/**
 *  @ingroup Readout
 *  @brief Print the current available block to standard out
 *  @param id Slot number
 *  @return Number of words read if successful, otherwise ERROR.
 */
int
faV3PrintBlock(int id)
{

  int ii;
  int nwrds = 32768, dCnt, berr = 0;
  uint32_t data, bhead, ehead;

  CHECKID;

  /* Check for valid A32 data pointer */
  if(FAV3pd[id] == NULL)
    {
      logMsg("faV3PrintBlock(id = %d): ERROR: A32 Data Pointer not initialized\n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  /* Check if data available */
  FAV3LOCK;
  if((vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK) == 0)
    {
      printf("faV3PrintBlock: ERROR: FIFO Empty\n");
      FAV3UNLOCK;
      return (0);
    }

  /* Check if Bus Errors are enabled. If so then disable for reading */
  berr = vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_ENABLE_BERR;
  if(berr)
    vmeWrite32(&(FAV3p[id]->ctrl1),
	       vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_BERR);

  dCnt = 0;
  /* Read Block Header - should be first word */
  bhead = (uint32_t) * FAV3pd[id];
#ifndef VXWORKS
  bhead = LSWAP(bhead);
#endif
  if((bhead & FAV3_DATA_TYPE_DEFINE)
     && ((bhead & FAV3_DATA_TYPE_MASK) == FAV3_DATA_BLOCK_HEADER))
    {
      ehead = (uint32_t) * FAV3pd[id];
#ifndef VXWORKS
      ehead = LSWAP(ehead);
#endif
      printf("%4d: ", dCnt + 1);
      faV3DataDecode(bhead);
      dCnt++;
      printf("%4d: ", dCnt + 1);
      faV3DataDecode(ehead);
      dCnt++;
    }
  else
    {
      /* We got bad data - Check if there is any data at all */
      if((vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK) == 0)
	{
	  logMsg("faV3PrintBlock: FIFO Empty (0x%08x)\n", bhead, 0, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (0);
	}
      else
	{
	  logMsg("faV3PrintBlock: ERROR: Invalid Header Word 0x%08x\n", bhead,
		 0, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (ERROR);
	}
    }

  ii = 0;
  while(ii < nwrds)
    {
      data = (uint32_t) * FAV3pd[id];
#ifndef VXWORKS
      data = LSWAP(data);
#endif
      printf("%4d: ", dCnt + 1 + ii);
      faV3DataDecode(data);
      if((data & FAV3_DATA_TYPE_DEFINE)
	 && ((data & FAV3_DATA_TYPE_MASK) == FAV3_DATA_BLOCK_TRAILER))
	break;

      if((data & FAV3_DATA_TYPE_DEFINE)
	 && ((data & FAV3_DATA_TYPE_MASK) == FAV3_DATA_INVALID))
	break;

      ii++;
    }
  ii++;
  dCnt += ii;

  if(berr)
    vmeWrite32(&(FAV3p[id]->ctrl1),
	       vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_ENABLE_BERR);

  FAV3UNLOCK;
  return (dCnt);

}


/**
 *  @ingroup Status
 *  @brief Get the value of the Control/Status Register
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

uint32_t
faV3ReadCSR(int id)
{
  uint32_t rval;

  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&(FAV3p[id]->csr));
  FAV3UNLOCK;

  return (rval);
}

/**
 *  @ingroup Config
 *  @brief Perform a soft reset.
 *  @param id Slot number
 */

int
faV3Clear(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Perform a soft reset of all initialized fADC250s
 */

void
faV3GClear()
{

  int ii, id;

  for(ii = 0; ii < nfaV3; ii++)
    {
      faV3Clear(faV3Slot(ii));
    }
}

/**
 *  @ingroup Config
 *  @brief Clear latched errors
 *  @param id Slot number
 */

int
faV3ClearError(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_ERROR_CLEAR);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Clear latched errors of all initialized fADC250s
 */

void
faV3GClearError()
{

  int ii, id;

  for(ii = 0; ii < nfaV3; ii++)
    {
      faV3ClearError(faV3Slot(ii));
    }

}

/**
 *  @ingroup Config
 *  @brief Perform a hard reset
 *  @param id Slot number
 *  @param iFlag Decision to restore A32 readout after reset.
 *     -  0: Restore A32 readout after reset.
 *     - !0: Do not restore A32 readout after reset. (Useful for configuration changes)
 */

int
faV3Reset(int id, int iFlag)
{
  uint32_t a32addr, addrMB;

  CHECKID;

  FAV3LOCK;
  if(iFlag == 0)
    {
      a32addr = vmeRead32(&(FAV3p[id]->adr32));
      addrMB = vmeRead32(&(FAV3p[id]->adr_mb));
    }

  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_HARD_RESET);
  taskDelay(2);

  if(iFlag == 0)
    {
      vmeWrite32(&(FAV3p[id]->adr32), a32addr);
      vmeWrite32(&(FAV3p[id]->adr_mb), addrMB);
    }
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Perform a hard reset on all initialized fADC250s
 *  @param iFlag Decision to restore A32 readout after reset.
 *     -  0: Restore A32 readout after reset.
 *     - !0: Do not restore A32 readout after reset. (Useful for configuration changes)
 */
void
faV3GReset(int iFlag)
{
  uint32_t a32addr[(FAV3_MAX_BOARDS + 1)], addrMB[(FAV3_MAX_BOARDS + 1)];
  int ifa = 0, id = 0;

  FAV3LOCK;
  if(iFlag == 0)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  id = faV3Slot(ifa);
	  a32addr[id] = vmeRead32(&(FAV3p[id]->adr32));
	  addrMB[id] = vmeRead32(&(FAV3p[id]->adr_mb));
	}
    }

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faV3Slot(ifa);
      vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_HARD_RESET);
    }

  taskDelay(10);

  if(iFlag == 0)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  id = faV3Slot(ifa);
	  vmeWrite32(&(FAV3p[id]->adr32), a32addr[id]);
	  vmeWrite32(&(FAV3p[id]->adr_mb), addrMB[id]);
	}
    }

  FAV3UNLOCK;

}

/**
 *  @ingroup Config
 *  @brief Perform either a soft clear or soft reset
 *  @param id Slot number
 *  @param cflag
 *    -  0: Soft Clear
 *    - >0: Soft Reset
 */

int
faV3SoftReset(int id, int cflag)
{
  CHECKID;

  FAV3LOCK;
  if(cflag)			/* perform soft clear */
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_CLEAR);
  else				/* normal soft reset */
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Reset the token
 *
 *     A call to this routine will cause the module to have the token if it
 *     has been configured to the the FIRST module in the MultiBlock chain.
 *     This routine has no effect on any other module in the chain.
 *
 *  @param id Slot number
 */

int
faV3ResetToken(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->reset), FAV3_RESET_TOKEN);
  FAV3UNLOCK;

  return OK;
}


/**
 *  @ingroup Status
 *  @brief Return the status of the token
 *  @param id Slot number
 *  @return 1 if module has the token, 0 if not, otherwise ERROR.
 */

int
faV3TokenStatus(int id)
{
  int rval = 0;

  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->csr) & FAV3_CSR_TOKEN_STATUS) >> 4;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the slotmask of those modules that have the token.
 *  @return Token Slotmask
 */

int
faV3GTokenStatus()
{
  int ifa = 0, bit = 0, rval = 0;

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      bit = faV3TokenStatus(faV3Slot(ifa));
      rval |= (bit << (faV3Slot(ifa)));
    }

  return rval;
}

/**
 * @ingroup Status
 *  @brief Return slot mask of modules with token
 *  @param pflag Option to print status to standard out.
 *  @return Mask of slots with the token, if successful. Otherwise ERROR.
 */

uint32_t
faV3GetTokenStatus(int pflag)
{
  uint32_t rval = 0;
  int ifa = 0;

  if(pflag)
    logMsg("faV3GetTokenStatus: Token in slot(s) ", 1, 2, 3, 4, 5, 6);

  rval = faV3GTokenStatus();

  if(pflag)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  if(rval & (1 << faV3ID[ifa]))
	    logMsg("%2d ", faV3ID[ifa], 2, 3, 4, 5, 6);
	}
    }

  if(pflag)
    logMsg("\n", 1, 2, 3, 4, 5, 6);

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Disable the specified channel
 *  @param id Slot number
 *  @param channel Channel Number to Disable
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetChanDisableMask(int id, uint16_t cmask)
{

  CHECKID;

  faV3ChanDisableMask[id] = cmask;	/* Set Global Variable */

  FAV3LOCK;
  /* Write New Disable Mask */
  vmeWrite32(&(FAV3p[id]->adc.config2), cmask);
  FAV3UNLOCK;

  return OK;
}


/**
 *  @ingroup Status
 *  @brief Get the Disabled Channel Mask
 *  @param id Slot number
 *  @return Specified mask if successful, otherwise ERROR.
 */

uint32_t
faV3GetChanDisableMask(int id)
{
  uint32_t tmp, cmask = 0;

  CHECKID;

  FAV3LOCK;
  tmp = vmeRead32(&(FAV3p[id]->adc.config2)) & 0xFFFF;
  cmask = (tmp & FAV3_ADC_CHAN_MASK);
  faV3ChanDisableMask[id] = cmask;	/* Set Global Variable */
  FAV3UNLOCK;


  return (cmask);
}


/* opt=0 - disable, 1-enable, 2-verify */
int
faV3SetCompression(int id, int opt)
{
  uint32_t ctrl2;

  CHECKID;

  FAV3LOCK;

  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faV3SetCompression: read ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & ~FAV3_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faV3SetCompression: masked ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  if(opt == 0)
    {
      ;
    }
  else if(opt == 1)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_ENABLE;
      printf("faV3SetCompression: setting mode 1 ctrl2=0x%08x\n", ctrl2);
    }
  else if(opt == 2)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_VERIFY;
      printf("faV3SetCompression: setting mode 2 ctrl2=0x%08x\n", ctrl2);
    }
  else
    printf("faV3SetCompression: illegal opt=%d\n", opt);

#ifdef DEBUG_COMPRESSION
  printf("faV3SetCompression: writing ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */
  vmeWrite32(&(FAV3p[id]->ctrl2), ctrl2);

  FAV3UNLOCK;

  return OK;
}


int
faV3GetCompression(int id)
{
  uint32_t ctrl2;
  int opt;

  CHECKID;

  FAV3LOCK;

  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faV3GetCompression: read ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & FAV3_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faV3GetCompression: masked ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  if(ctrl2 == FAV3_CTRL_COMPRESS_DISABLE)
    opt = 0;
  else if(ctrl2 == FAV3_CTRL_COMPRESS_ENABLE)
    opt = 1;
  else if(ctrl2 == FAV3_CTRL_COMPRESS_VERIFY)
    opt = 2;
  else
    opt = -2;

  FAV3UNLOCK;

  return (opt);
}

/* opt=0 - disable, 1-enable */
int
faV3SetVXSReadout(int id, int opt)
{
  uint32_t ctrl2;

  CHECKID;

  FAV3LOCK;

  ctrl2 = vmeRead32(&FAV3p[id]->ctrl2);

  if(opt == 0)
    {
      ctrl2 = ctrl2 & ~FAV3_CTRL_VXS_RO_ENABLE;
    }
  else
    {
      ctrl2 = ctrl2 | FAV3_CTRL_VXS_RO_ENABLE;
    }

  vmeWrite32(&(FAV3p[id]->ctrl2), ctrl2);

  FAV3UNLOCK;
  return OK;
}

void
faV3GSetVXSReadout(int opt)
{
  int ifa = 0;

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      faV3SetVXSReadout(faV3ID[ifa], opt);
    }

}


int
faV3GetVXSReadout(int id)
{
  uint32_t ctrl2;
  int opt;

  CHECKID;

  FAV3LOCK;

  ctrl2 = vmeRead32(&FAV3p[id]->ctrl2) & FAV3_CTRL_VXS_RO_ENABLE;

  if(ctrl2)
    opt = 1;
  else
    opt = 0;

  FAV3UNLOCK;

  return (opt);
}

/**
 *  @ingroup Config
 *  @brief Enable the SyncReset source
 *  @param id Slot number
 */

int
faV3EnableSyncSrc(int id)
{
  uint32_t ctrl2;

  CHECKID;

  FAV3LOCK;
  ctrl2 = vmeRead32(&FAV3p[id]->ctrl2) | FAV3_CTRL_ENABLE_SRESET;
  vmeWrite32(&FAV3p[id]->ctrl2, ctrl2);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable the SyncReset Source of all initialized fADC250s
 */

void
faV3GEnableSyncSrc()
{
  int id = 0;
  for(id = 0; id < nfaV3; id++)
    faV3EnableSyncSrc(faV3ID[id]);

}

/**
 *  @ingroup Config
 *  @brief Enable data acquisition, trigger, and SyncReset on the module
 *  @param id Slot number
 *  @param eflag Enable Internal Trigger Logic, as well
 */

int
faV3Enable(int id, int eflag)
{
  uint32_t ctrl2;
  int compress_opt, vxsro_opt;

  CHECKID;

  /* call it BEFORE 'FAV3LOCK' !!! */
  compress_opt = faV3GetCompression(id);
  vxsro_opt = faV3GetVXSReadout(id);

  FAV3LOCK;

  ctrl2 = FAV3_CTRL_GO | FAV3_CTRL_ENABLE_TRIG | FAV3_CTRL_ENABLE_SRESET;

  if(eflag)			/* Enable Internal Trigger logic as well */
    {
      ctrl2 = ctrl2 | FAV3_CTRL_ENABLE_INT_TRIG;
    }

  if(compress_opt == 1)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_ENABLE;
    }
  else if(compress_opt == 2)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_VERIFY;
    }

  if(vxsro_opt == 1)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_VXS_RO_ENABLE;
    }

  vmeWrite32(&(FAV3p[id]->ctrl2), ctrl2);

  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable data acquisition, trigger, and SyncReset on all initialized fADC250s
 *
 *    Also enables the SDC if it is initalized and used.
 *
 *  @param eflag Enable Internal Trigger Logic, as well
 */
void
faV3GEnable(int eflag)
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faV3Enable(faV3ID[ii], eflag);

  if(faV3UseSDC && !faV3SDCPassthrough)
    faV3SDC_Enable(1);

}

/**
 *  @ingroup Config
 *  @brief Disable data acquisition, triggers, and SyncReset on the module
 *  @param id Slot number
 *  @param eflag
 *    - >0: Turn off FIFO transfer as well.
 */

int
faV3Disable(int id, int eflag)
{

  CHECKID;

  FAV3LOCK;
  if(eflag)
    vmeWrite32(&(FAV3p[id]->ctrl2), 0);	/* Turn FIFO Transfer off as well */
  else
    vmeWrite32(&(FAV3p[id]->ctrl2), (FAV3_CTRL_GO | FAV3_CTRL_ENABLE_SRESET));	/* Keep SYNC RESET detection enabled */
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable data acquisition, triggers, and SyncReset on all initialized fADC250s
 *  @param eflag
 *    - >0: Turn off FIFO transfer as well.
 */

void
faV3GDisable(int eflag)
{
  int ii;

  if(faV3UseSDC && !faV3SDCPassthrough)
    faV3SDC_Disable();

  for(ii = 0; ii < nfaV3; ii++)
    faV3Disable(faV3ID[ii], eflag);

}

/**
 *  @ingroup Readout
 *  @brief Pulse a software trigger to the module.
 *  @param id Slot number
 */

int
faV3Trig(int id)
{

  CHECKID;

  FAV3LOCK;

  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_TRIG))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_TRIGGER);
  else
    logMsg("faV3Trig: ERROR: Software Triggers not enabled", 0, 0, 0, 0, 0, 0);

  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Readout
 *  @brief Pulse a software trigger to all initialized fADC250s
 */

void
faV3GTrig()
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faV3Trig(faV3ID[ii]);
}

/**
 *  @ingroup Readout
 *  @brief Pulse a software playback trigger to the module.
 *  @param id Slot number
 */

int
faV3Trig2(int id)
{
  CHECKID;

  FAV3LOCK;
  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_TRIG))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_PULSE_TRIG2);
  else
    logMsg("faV3Trig2: ERROR: Software Triggers not enabled", 0, 0, 0, 0, 0, 0);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Readout
 *  @brief Pulse a software playback trigger to all initialized fADC250s
 */

void
faV3GTrig2()
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faV3Trig2(faV3ID[ii]);
}

/**
 *  @ingroup Config
 *  @brief Configure the delay between the software playback trigger and trigger
 *  @param id Slot number
 *  @param delay Delay between the playback trigger and trigger in units of 4 ns
 *  @sa faV3EnableInternalPlaybackTrigger
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetTrig21Delay(int id, int delay)
{
  CHECKID;

  if(delay > FAV3_TRIG21_DELAY_MASK)
    {
      printf("%s: ERROR: Invalid value for delay (%d).\n", __func__, delay);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig21_del, delay);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Return the value of the delay between the software playback trigger and trigger
 *  @param id Slot number
 *  @return Trigger delay, otherwise ERROR.
 */

int
faV3GetTrig21Delay(int id)
{
  int rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->trig21_del) & FAV3_TRIG21_DELAY_MASK;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Enable the software playback trigger and trigger
 *  @param id Slot number
 *  @sa fV3aSetTrig21Delay
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3EnableInternalPlaybackTrigger(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->ctrl1,
	     (vmeRead32(&FAV3p[id]->ctrl1) & ~FAV3_TRIG_MASK) |
	     FAV3_TRIG_VME_PLAYBACK);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Pulse a software SyncReset
 *  @param id Slot number
 */

int
faV3Sync(int id)
{

  CHECKID;

  FAV3LOCK;
  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_SRESET))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SYNC);
  else
    logMsg("faV3Sync: ERROR: Software Sync Resets not enabled\n", 0, 0, 0, 0, 0,
	   0);
  FAV3UNLOCK;

  return OK;
}


/**
 *  @ingroup Readout
 *  @brief  Return Event/Block count
 *  @param id Slot number
 *  @param dflag
 *   -  0: Event Count
 *   - >0: Block count
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3Dready(int id, int dflag)
{
  uint32_t dcnt = 0;

  CHECKID;

  FAV3LOCK;
  if(dflag)
    dcnt = vmeRead32(&(FAV3p[id]->blk_count)) & FAV3_BLOCK_COUNT_MASK;
  else
    dcnt = vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK;
  FAV3UNLOCK;


  return (dcnt);
}

/**
 *  @ingroup Readout
 *  @brief Return a Block Ready status
 *  @param id Slot number
 *  @return 1 if block is ready for readout, 0 if not, otherwise ERROR.
 */

int
faV3Bready(int id)
{
  int stat = 0;

  CHECKID;

  FAV3LOCK;
  stat = (vmeRead32(&(FAV3p[id]->csr))) & FAV3_CSR_BLOCK_READY;
  FAV3UNLOCK;

  if(stat)
    return (1);
  else
    return (0);
}

/**
 *  @ingroup Readout
 *  @brief Return a Block Ready status mask for all initialized fADC250s
 *  @return block ready mask, otherwise ERROR.
 */

uint32_t
faV3GBready()
{
  int ii, id, stat = 0;
  uint32_t dmask = 0;

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];

      stat = vmeRead32(&(FAV3p[id]->csr)) & FAV3_CSR_BLOCK_READY;

      if(stat)
	dmask |= (1 << id);
    }
  FAV3UNLOCK;

  return (dmask);
}

/**
 *  @ingroup Readout
 *  @brief Return a Block Ready status mask for fADCs indicated in supplied slotmask
 *  @param slotmask Slotmask Slotmask of fADCs to check for block ready
 *  @param nloop Number of times to iterate through slotmask.
 *  @return block ready mask, otherwise ERROR.
*/

uint32_t
faV3GBlockReady(uint32_t slotmask, int nloop)
{
  int iloop, islot, stat = 0;
  uint32_t dmask = 0;

  FAV3LOCK;
  for(iloop = 0; iloop < nloop; iloop++)
    {

      for(islot = 0; islot < 21; islot++)
	{

	  if(slotmask & (1 << islot))
	    {			/* slot used */

	      if(!(dmask & (1 << islot)))
		{		/* No block ready yet. */

		  stat = vmeRead32(&FAV3p[islot]->csr) & FAV3_CSR_BLOCK_READY;

		  if(stat)
		    dmask |= (1 << islot);

		  if(dmask == slotmask)
		    {		/* Blockready mask matches user slotmask */
		      FAV3UNLOCK;
		      return (dmask);
		    }
		}
	    }
	}
    }
  FAV3UNLOCK;

  return (dmask);

}

/**
 *  @ingroup Status
 *  @brief Return the vme slot mask of all initialized fADC250s
 *  @return VME Slot mask, otherwise ERROR.
 */

uint32_t
faV3ScanMask()
{
  int ifadc, id, dmask = 0;

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      dmask |= (1 << id);
    }

  return (dmask);
}

/**
 *  @ingroup Config
 *  @brief Set/Readback Busy Level
 *  @param id Slot number
 *  @param val
 *    - >0: set the busy level to val
 *    -  0: read back busy level
 *  @param bflag   i
 *    - >0: force the module Busy
 *
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3BusyLevel(int id, uint32_t val, int bflag)
{
  uint32_t blreg = 0;

  CHECKID;

  if(val > FAV3_BUSY_LEVEL_MASK)
    return (ERROR);

  /* if Val > 0 then set the Level else leave it alone */
  FAV3LOCK;
  if(val)
    {
      if(bflag)
	vmeWrite32(&(FAV3p[id]->busy_level), (val | FAV3_FORCE_BUSY));
      else
	vmeWrite32(&(FAV3p[id]->busy_level), val);
    }
  else
    {
      blreg = vmeRead32(&(FAV3p[id]->busy_level));
      if(bflag)
	vmeWrite32(&(FAV3p[id]->busy_level), (blreg | FAV3_FORCE_BUSY));
    }
  FAV3UNLOCK;

  return ((blreg & FAV3_BUSY_LEVEL_MASK));
}

/**
 *  @ingroup Status
 *  @brief Get the busy status
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3Busy(int id)
{
  uint32_t blreg = 0;
  uint32_t dreg = 0;

  CHECKID;

  FAV3LOCK;
  blreg = vmeRead32(&(FAV3p[id]->busy_level)) & FAV3_BUSY_LEVEL_MASK;
  dreg = vmeRead32(&(FAV3p[id]->ram_word_count)) & FAV3_RAM_DATA_MASK;
  FAV3UNLOCK;

  if(dreg >= blreg)
    return (1);
  else
    return (0);
}

/**
 *  @ingroup Config
 *  @brief Enable software triggers
 *  @param id Slot number
 */

int
faV3EnableSoftTrig(int id)
{
  CHECKID;

  /* Clear the source */
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_TRIG_MASK);
  /* Set Source and Enable */
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_TRIG_VME |
					       FAV3_ENABLE_SOFT_TRIG));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable Software Triggers for all initialized fADC250s
 */

void
faV3GEnableSoftTrig()
{
  int ii, id;

  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];
      faV3EnableSoftTrig(id);
    }

}

/**
 *  @ingroup Config
 *  @brief Disable Software Triggers
 *  @param id Slot number
 */

int
faV3DisableSoftTrig(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_SOFT_TRIG);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable Software SyncReset
 *  @param id Slot number
 */

int
faV3EnableSoftSync(int id)
{

  CHECKID;

  /* Clear the source */
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_SRESET_MASK);
  /* Set Source and Enable */
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_SRESET_VME |
					       FAV3_ENABLE_SOFT_SRESET));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable Software SyncReset
 *  @param id Slot number
 */

int
faV3DisableSoftSync(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_SOFT_SRESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable the internal clock
 *  @param id Slot number
 */

int
faV3EnableClk(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_REF_CLK_INTERNAL |
					       FAV3_ENABLE_INTERNAL_CLK));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable the internal clock
 *  @param id Slot number
 */

int
faV3DisableClk(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_INTERNAL_CLK);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable trigger out for front panel or p0
 *  @param id Slot number
 *  @param output
 *    - 0: FP trigger out
 *    - 1: P0 trigger out
 *    - 2: FP and P0 trigger out
 */

int
faV3EnableTriggerOut(int id, int output)
{
  int bitset = 0;

  CHECKID;

  if(output > 2)
    {
      logMsg("faV3EnableTriggerOut: ERROR: output (%d) out of range.  Must be less than 3",
	     output, 2, 3, 4, 5, 6);
      return ERROR;

    }

  switch (output)
    {
    case 0:
      bitset = FAV3_ENABLE_TRIG_OUT_FP;
      break;
    case 1:
      bitset = FAV3_ENABLE_TRIG_OUT_P0;
      break;
    case 2:
      bitset = FAV3_ENABLE_TRIG_OUT_FP | FAV3_ENABLE_TRIG_OUT_P0;
      break;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) | bitset);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable bus errors to terminate a block transfer
 *  @param id Slot number
 */

int
faV3EnableBusError(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_ENABLE_BERR);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable bus errors to terminate a block transfer for all initialized fADC250s
 */

void
faV3GEnableBusError()
{
  int ii;

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    {
      vmeWrite32(&(FAV3p[faV3ID[ii]]->ctrl1),
		 vmeRead32(&(FAV3p[faV3ID[ii]]->ctrl1)) | FAV3_ENABLE_BERR);
    }
  FAV3UNLOCK;

}

/**
 *  @ingroup Config
 *  @brief Disable bus errors
 *  @param id Slot number
 */

int
faV3DisableBusError(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_BERR);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable and setup multiblock transfers for all initialized fADC250s
 *  @param tflag Token Flag
 *    - >0: Token via P0/VXS
 *    -  0: Token via P2
 */

int
faV3EnableMultiBlock(int tflag)
{
  int ii, id;
  uint32_t mode;

  if((nfaV3 <= 1) || (FAV3p[faV3ID[0]] == NULL))
    {
      logMsg("faV3EnableMultiBlock: ERROR : Cannot Enable MultiBlock mode \n",
	     0, 0, 0, 0, 0, 0);
      return ERROR;
    }

  /* if token = 0 then send via P2 else via VXS */
  if(tflag)
    mode = (FAV3_ENABLE_MULTIBLOCK | FAV3_MB_TOKEN_VIA_P0);
  else
    mode = (FAV3_ENABLE_MULTIBLOCK | FAV3_MB_TOKEN_VIA_P2);

  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];
      FAV3LOCK;
      vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) | mode);
      FAV3UNLOCK;
      faV3DisableBusError(id);
      if(id == faV3MinSlot)
	{
	  FAV3LOCK;
	  vmeWrite32(&(FAV3p[id]->ctrl1),
		     vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_FIRST_BOARD);
	  FAV3UNLOCK;
	}
      if(id == faV3MaxSlot)
	{
	  FAV3LOCK;
	  vmeWrite32(&(FAV3p[id]->ctrl1),
		     vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_LAST_BOARD);
	  FAV3UNLOCK;
	  faV3EnableBusError(id);	/* Enable Bus Error only on Last Board */
	}
    }

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable multiblock transfer for all initialized fADC250s
 */

int
faV3DisableMultiBlock()
{
  int ii;

  if((nfaV3 <= 1) || (FAV3p[faV3ID[0]] == NULL))
    {
      logMsg("faV3DisableMultiBlock: ERROR : Cannot Disable MultiBlock Mode\n",
	     0, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    vmeWrite32(&(FAV3p[faV3ID[ii]]->ctrl1),
	       vmeRead32(&(FAV3p[faV3ID[ii]]->ctrl1)) & ~FAV3_ENABLE_MULTIBLOCK);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the block level for the module
 *  @param id Slot number
 *  @param level block level
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetBlockLevel(int id, int level)
{
  int rval;

  CHECKID;

  if(level <= 0)
    level = 1;

  logMsg("faV3SetBlockLevel: INFO: Set ADC slot %d block level to %d \n", id,
	 level, 0, 0, 0, 0);

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->blocklevel), level);
  rval = vmeRead32(&(FAV3p[id]->blocklevel)) & FAV3_BLOCK_LEVEL_MASK;
  FAV3UNLOCK;

  return (rval);
}

/**
 *  @ingroup Config
 *  @brief Set the block level for all initialized fADC250s
 *  @param level block level
 */

void
faV3GSetBlockLevel(int level)
{
  int ii;

  if(level <= 0)
    level = 1;
  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    vmeWrite32(&(FAV3p[faV3ID[ii]]->blocklevel), level);
  FAV3UNLOCK;

}

/**
 *  @ingroup Config
 *  @brief Set the Clock Source for the module
 *  @param id Slot number
 *  @param source Clock Source
 *    - 0: internal
 *    - 1: front panel
 *    - 2: P0/VXS
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetClkSource(int id, int source)
{
  int rval;

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_REF_CLK_SEL_MASK);
  if((source < 0) || (source > 7))
    source = FAV3_REF_CLK_INTERNAL;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) | source);
  rval = vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_REF_CLK_SEL_MASK;
  FAV3UNLOCK;


  return (rval);
}

/**
 *  @ingroup Config
 *  @brief Set the trigger source for the module
 *  @param id Slot number
 *  @param source Trigger Source
 *   - 0: Front Panel
 *   - 1: Front Panel (Synchronized)
 *   - 2: P0/VXS
 *   - 3: P0/VXS (Synchronized)
 *   - 4: Not used
 *   - 5: Software (with playback)
 *   - 6: Software
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetTrigSource(int id, int source)
{
  int rval;

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_TRIG_SEL_MASK);
  if((source < 0) || (source > 7))
    source = FAV3_TRIG_FP_ISYNC;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) | source);
  rval = vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_TRIG_SEL_MASK;
  FAV3UNLOCK;

  return (rval);

}

/**
 *  @ingroup Config
 *  @brief Set the SyncReset source for the module
 *  @param id Slot number
 *  @param source
 *   - 0: FP
 *   - 1: FP (synchronized)
 *   - 2: P0/VXS
 *   - 3: P0/VXS (synchronized)
 *   - 4: Not used
 *   - 5: Not used
 *   - 6: Software
 *   - 7: No Source
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetSyncSource(int id, int source)
{
  int rval;

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_SRESET_SEL_MASK);
  if((source < 0) || (source > 7))
    source = FAV3_SRESET_FP_ISYNC;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) | source);
  rval = vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_SRESET_SEL_MASK;
  FAV3UNLOCK;

  return (rval);

}

/**
 *  @ingroup Config
 *  @brief Enable Front Panel Inputs
 *
 *    Also disables software triggers/syncs but leaves the clock source alone
 *
 *  @param id Slot number
 */
int
faV3EnableFP(int id)
{

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) &
	     ~(FAV3_TRIG_SEL_MASK | FAV3_SRESET_SEL_MASK | FAV3_ENABLE_SOFT_SRESET |
	       FAV3_ENABLE_SOFT_TRIG));
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_TRIG_FP_ISYNC |
					       FAV3_SRESET_FP_ISYNC));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set trigger output options
 *  @param id Slot number
 *  @param trigout bits:
 * <pre>
 *      0  1  0  Enable Front Panel Trigger Output
 *      1  0  0  Enable VXS Trigger Output
 * </pre>
 *
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetTrigOut(int id, int trigout)
{
  CHECKID;

  if(trigout < 0 || trigout > 7)
    {
      printf("faV3SetTrigOut: ERROR : Invalid trigout value (%d) \n", trigout);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     (vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_TRIGOUT_MASK) |
	     trigout << 12);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Get the trigger count for the module
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

uint32_t
faV3GetTriggerCount(int id)
{
  uint32_t rval = 0;

  CHECKID;

  /* Just reading - not need to Lock mutex */
  rval = vmeRead32(&FAV3p[id]->trig_count);

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Reset the trigger count for the module
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3ResetTriggerCount(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig_count, FAV3_TRIG_COUNT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the readout threshold value for specified channel mask
 *  @param id Slot number
 *  @param tvalue Threshold value
 *  @param chmask Mask of channels to set
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetChannelThreshold(int id, int chan, uint16_t tvalue)
{
  CHECKID;

  int index = chan / 2;
  int hibyte = chan % 2;
  uint32_t regval = 0;

  FAV3LOCK;
  regval = vmeRead32(&FAV3p[id]->adc.thres[index]);
  if(hibyte)
    vmeWrite32(&FAV3p[id]->adc.thres[index],
	       (regval & 0xFFFF) | (tvalue << 16));
  else
    vmeWrite32(&FAV3p[id]->adc.thres[index],
	       (regval & 0xFFFF0000) | tvalue);

  FAV3UNLOCK;

  return (OK);
}

int
faV3SetThresholdAll(int id, uint16_t tvalue[16])
{
  int ii;

  CHECKID;

  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      faV3SetChannelThreshold(id, ii, tvalue[ii]);
    }

  return (OK);
}

int
faV3GetChannelThreshold(int id, int chan)
{
  int rval = 0;

  CHECKID;

  int index = chan / 2;
  int hibyte = chan % 2;
  uint32_t regval = 0;

  FAV3LOCK;
  regval = vmeRead32(&FAV3p[id]->adc.thres[index]);
  FAV3UNLOCK;

  if(hibyte)
    rval = (regval >> 16) & 0xFFFF;
  else
    rval = (regval & 0xFFFF);

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Print the thresholds of all channels to standard out
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3PrintThreshold(int id)
{
  int ii;
  uint16_t tval[FAV3_MAX_ADC_CHANNELS];

  CHECKID;

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS/2; ii++)
    {
      tval[2*ii] = vmeRead32(&(FAV3p[id]->adc.thres[ii])) & 0xFFFF;
      tval[2*ii + 1] = (vmeRead32(&(FAV3p[id]->adc.thres[ii])) & 0xFFFF0000) >> 16;
    }
  FAV3UNLOCK;

  printf(" Threshold Settings for FADC in slot %d:", id);
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if((ii % 4) == 0)
	{
	  printf("\n");
	}
      printf("Chan %2d: %5d(%d)   ", (ii + 1), tval[ii] & FAV3_THR_VALUE_MASK,
	     (tval[ii] & FAV3_THR_IGNORE_MASK) >> 15);
    }
  printf("\n");

  return (OK);
}

int32_t
faV3DACInit(int id)
{
  uint32_t csr_value = 0, init_done = 0;
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->dac_csr, FAV3_DAC_INIT);
  taskDelay(1);		// wait
  csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
  init_done = (csr_value & FAV3_DAC_INIT_DONE) >> 30;
  FAV3UNLOCK;

  if(!init_done)
    {
      printf("%s(id = %d): ERROR: Init Failed.  DAC_CSR: 0x%08x\n",
	     __func__, id, csr_value);
      rval = ERROR;
    }

  return rval;
}

int32_t
faV3DACClear(int id)
{
  uint32_t csr_value;
  uint32_t ready, success, not_ready_since_clear, timeout_since_clear;
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->dac_csr, FAV3_DAC_CLEAR);
  csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
  ready = (csr_value & FAV3_DAC_READY >> 16);
  success = (csr_value & FAV3_DAC_SUCCESS) >> 17;
  not_ready_since_clear = (csr_value & FAV3_DAC_NOT_READY) >> 18;
  timeout_since_clear = (csr_value & FAV3_DAC_TIMEOUT) >> 19;
  FAV3UNLOCK;

  if(!ready || !success || not_ready_since_clear || timeout_since_clear)
    {
      printf("%s(id = %d): ERROR: Clear Failed.  DAC_CSR: 0x%08x\n",
	     __func__, id, csr_value);
      printf("    Ready: %d  Success: %d  NotReadySinceClear: %d  Timeout Since Clear %d\n",
	     ready, success, not_ready_since_clear, timeout_since_clear);
      rval = ERROR;
    }

  return rval;
}

int32_t
faV3DACStatus(int id)
{
  uint32_t csr_value;
  uint32_t ready, success, not_ready_since_clear, timeout_since_clear;
  int32_t rval = OK;

  CHECKID;

  FAV3LOCK;
  csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
  ready = (csr_value & FAV3_DAC_READY >> 16);
  success = (csr_value & FAV3_DAC_SUCCESS) >> 17;
  not_ready_since_clear = (csr_value & FAV3_DAC_NOT_READY) >> 18;
  timeout_since_clear = (csr_value & FAV3_DAC_TIMEOUT) >> 19;
  FAV3UNLOCK;

  printf("%s(id = %d): DAC_CSR: 0x%08x\n",
	 __func__, id, csr_value);
  printf("    Ready: %d  Success: %d  NotReadySinceClear: %d  Timeout Since Clear %d\n",
	 ready, success, not_ready_since_clear, timeout_since_clear);

  return rval;
}


int32_t
faV3DACSet(int id, int chan, uint32_t dac_value)
{
  uint32_t csr_value;
  uint32_t ready, success, not_ready_since_clear, timeout_since_clear;
  int32_t rval = OK;

  CHECKID;

  if(chan > FAV3_MAX_ADC_CHANNELS)
    {
      printf("%s: ERROR: Invalid chan (%d)\n",
	     __func__, chan);
      return ERROR;
    }

  if(dac_value > FAV3_DAC_MAX_VALUE)
    {
      printf("%s(id = %d, chan = %d): ERROR: Invalid dac_value 0x%x (%d)\n",
	     __func__, id, chan, dac_value, dac_value);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->dac_csr, chan);

  csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
  ready = (csr_value & FAV3_DAC_READY >> 16);
  success = (csr_value & FAV3_DAC_SUCCESS) >> 17;

  if(ready && success)
    {
      vmeWrite32(&FAV3p[id]->dac_data, dac_value);
      csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
      ready = (csr_value & FAV3_DAC_READY >> 16);
      success = (csr_value & FAV3_DAC_SUCCESS) >> 17;
    }
  FAV3UNLOCK;

  if(!ready || !success)
    {
      printf("%s(id = %d, chan = %d): ERROR: Write 0x%x Failed.  DAC_CSR: 0x%08x\n",
	     __func__, id, chan, dac_value, csr_value);
      printf("    Ready: %d  Success: %d\n",
	     ready, success);
      rval = ERROR;
    }

  return rval;
}

int32_t
faV3DACGet(int id, int chan, uint32_t *dac_value)
{
  uint32_t csr_value, data_value, chan_value;
  uint32_t ready, success, not_ready_since_clear, timeout_since_clear;
  int32_t rval = OK;

  CHECKID;

  if(chan > FAV3_MAX_ADC_CHANNELS)
    {
      printf("%s: ERROR: Invalid chan (%d)\n",
	     __func__, chan);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->dac_csr, chan);

  csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
  ready = (csr_value & FAV3_DAC_READY >> 16);
  success = (csr_value & FAV3_DAC_SUCCESS) >> 17;

  if(ready && success)
    {
      data_value = vmeRead32(&FAV3p[id]->dac_data);
      *dac_value = data_value & FAV3_DAC_DATA_MASK;
      csr_value = vmeRead32(&FAV3p[id]->dac_csr);	// read back value
      ready = (csr_value & FAV3_DAC_READY >> 16);
      success = (csr_value & FAV3_DAC_SUCCESS) >> 17;
    }
  FAV3UNLOCK;

  if(!ready || !success)
    {
      printf("%s(id = %d, chan = %d): ERROR: Read 0x%x Failed.  DAC_CSR: 0x%08x\n",
	     __func__, id, chan, *dac_value, csr_value);
      printf("    Ready: %d  Success: %d\n",
	     ready, success);
      rval = ERROR;
    }

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Set the pedestal value of specified channel
 *
 *    The pedestal is the value that will be subtracted from specified channel
 *    for each sample before it is sent through the trigger path
 *
 *  @param id Slot number
 *  @param chan Channel Number
 *  @param ped Pedestal value
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetChannelPedestal(int id, uint32_t chan, uint32_t ped)
{
  uint32_t lovalue = 0, hivalue = 0;
  CHECKID;

  if(chan > 16)
    {
      logMsg("faV3SetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(ped > 0xffff)
    {
      logMsg("faV3SetChannelPedestal: ERROR : PED value (%d) out of range (0-65535) \n",
	     ped, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->adc.pedestal[chan], ped);
  FAV3UNLOCK;

  return (OK);
}

/**
 *  @ingroup Status
 *  @brief Get the pedestal value of specified channel
 *  @param id Slot number
 *  @param chan Channel Number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3GetChannelPedestal(int id, uint32_t chan)
{
  uint32_t rval = 0;

  CHECKID;

  if(chan > 16)
    {
      logMsg("faV3SetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->adc.pedestal[chan]) & FAV3_ADC_PEDESTAL_MASK;
  FAV3UNLOCK;

  return (rval);
}


int
faV3SetPedestal(int id, uint32_t wvalue)
{
  int ii;

  CHECKID;

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(!(ii & 0x1))
	vmeWrite32((uint32_t *) & (FAV3p[id]->adc.pedestal[ii]),
		   wvalue | (wvalue << 16));
    }
  FAV3UNLOCK;

  return (OK);
}

int
faV3PrintPedestal(int id)
{
  int ii;
  uint32_t tval[FAV3_MAX_ADC_CHANNELS];

  CHECKID;

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tval[ii] = vmeRead32(&(FAV3p[id]->adc.pedestal[ii]));
    }
  FAV3UNLOCK;


  printf(" Pedestal Settings for FADC in slot %d:", id);
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if((ii % 4) == 0)
	{
	  printf("\n");
	}
      printf("chan %2d: %3d   ", (ii + 1), tval[ii]);
    }
  printf("\n");

  return (OK);
}

/**
 *  @ingroup Readout
 *  @brief Scaler Data readout routine
 *
 *        Readout the desired scalers (indicated by the channel mask), as well
 *        as the timer counter.  The timer counter will be the last word
 *        in the "data" array.
 *
 *  @param id Slot number
 *  @param data   - local memory address to place data
 *  @param chmask - Channel Mask (indicating which channels to read)
 *  @param rflag  - Readout Flag
 *    - bit 0: Latch Scalers before read
 *    - bit 1: Clear Scalers after read
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3ReadScalers(int id, volatile uint32_t * data, uint32_t chmask, int rflag)
{
  int doLatch = 0, doClear = 0, ichan = 0;
  int dCnt = 0;

  CHECKID;

  if(rflag & ~(FAV3_SCALER_CTRL_MASK))
    {
      logMsg("faV3ReadScalers: WARN : rflag (0x%x) has undefined bits \n",
	     rflag, 0, 0, 0, 0, 0);
    }

  doLatch = rflag & (1 << 0);
  doClear = rflag & (1 << 1);

  FAV3LOCK;
  if(doLatch)
    vmeWrite32(&FAV3p[id]->scaler_ctrl,
	       FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_LATCH);

  for(ichan = 0; ichan < 16; ichan++)
    {
      if((1 << ichan) & chmask)
	{
	  data[dCnt] = vmeRead32(&FAV3p[id]->scalers.scaler[ichan]);
	  dCnt++;
	}
    }

  data[dCnt] = vmeRead32(&FAV3p[id]->scalers.time_count);
  dCnt++;

  if(doClear)
    vmeWrite32(&FAV3p[id]->scaler_ctrl,
	       FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_RESET);
  FAV3UNLOCK;

  return dCnt;

}

/**
 *  @ingroup Readout
 *  @brief Scaler Print Out routine
 *
 *        Print out the scalers as well as the timer counter.
 *
 *  @param id Slot number
 *  @param rflag  - Printout Flag
 *     - bit 0: Latch Scalers before read
 *     - bit 1: Clear Scalers after read

 *  @return OK if successful, otherwise ERROR.
 */

int
faV3PrintScalers(int id, int rflag)
{
  int doLatch = 0, doClear = 0, ichan = 0;
  uint32_t data[16], time_count;

  CHECKID;

  if(rflag & ~(FAV3_SCALER_CTRL_MASK))
    {
      logMsg("faV3PrintScalers: WARN : rflag (0x%x) has undefined bits \n",
	     rflag, 0, 0, 0, 0, 0);
    }

  doLatch = rflag & (1 << 0);
  doClear = rflag & (1 << 1);

  FAV3LOCK;
  if(doLatch)
    vmeWrite32(&FAV3p[id]->scaler_ctrl,
	       FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_LATCH);

  for(ichan = 0; ichan < 16; ichan++)
    {
      data[ichan] = vmeRead32(&FAV3p[id]->scalers.scaler[ichan]);
    }

  time_count = vmeRead32(&FAV3p[id]->scalers.time_count);

  if(doClear)
    vmeWrite32(&FAV3p[id]->scaler_ctrl,
	       FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_RESET);
  FAV3UNLOCK;

  printf("%s: Scaler Counts\n", __func__);
  for(ichan = 0; ichan < 16; ichan++)
    {
      if((ichan % 4) == 0)
	printf("\n");

      printf("%2d: %10d ", ichan, data[ichan]);
    }
  printf("\n  timer: %10d\n", time_count);

  return OK;

}

/**
 *  @ingroup Config
 *  @brief Clear the scalers (and enable, if disabled)
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3ClearScalers(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl,
	     FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Latch the current scaler count
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3LatchScalers(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl,
	     FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_LATCH);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable the scalers to count
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3EnableScalers(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl, FAV3_SCALER_CTRL_ENABLE);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Disable counting in the scalers
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3DisableScalers(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl, ~FAV3_SCALER_CTRL_ENABLE);
  FAV3UNLOCK;

  return OK;
}


/**
 * @ingroup Status
 *  @brief Return the base address of the A32 for specified module
 *  @param id
 *   - Slot Number
 *  @return A32 address base, if successful. Otherwise ERROR.
 */

uint32_t
faV3GetA32(int id)
{
  uint32_t rval = 0;
  CHECKID;

  if(FAV3pd[id])
    {
      rval = (uint32_t) ((u_long) (FAV3pd[id]) - faV3A32Offset);
    }
  else
    {
      logMsg("faV3GetA32(%d): A32 pointer not initialized\n",
	     id, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  return rval;
}


/**
 * @ingroup Status
 *  @brief Return the base address of the A32 Multiblock
 *  @return A32 multiblock address base, if successful. Otherwise ERROR.
 */

uint32_t
faV3GetA32M()
{
  uint32_t rval = 0;

  if(FAV3pmb)
    {
      rval = (uint32_t) ((u_long) (FAV3pmb) - faV3A32Offset);
    }
  else
    {
      logMsg("faV3GetA32M: A32M pointer not initialized\n", 1, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  return rval;
}



/**
 *  @ingroup Status
 *  @brief Get the minimum address used for multiblock
 *  @param id Slot number
 *  @return multiblock min address if successful, otherwise ERROR.
 */
uint32_t
faV3GetMinA32MB(int id)
{
  uint32_t rval = 0, a32addr, addrMB;
  CHECKID;

  FAV3LOCK;

  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));

  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;
  addrMB = (addrMB & FAV3_AMB_MIN_MASK) << 16;

#ifdef DEBUG
  printf("faV3GetMinA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n", a32addr,
	 addrMB, id);
#endif

  id = faV3ID[0];
  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;

  rval = a32addr;
#ifdef DEBUG
  printf("faV3GetMinA32MB: rval=0x%08x\n", rval);
#endif

  FAV3UNLOCK;


  return rval;
}

/**
 *  @ingroup Status
 *  @brief Get the maximum address used for multiblock
 *  @param id Slot number
 *  @return multiblock max address if successful, otherwise ERROR.
 */

uint32_t
faV3GetMaxA32MB(int id)
{
  uint32_t rval = 0, a32addr, addrMB;

  CHECKID;

  FAV3LOCK;

  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));

  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;
  addrMB = addrMB & FAV3_AMB_MAX_MASK;

  rval = addrMB;

#ifdef DEBUG
  printf("faV3GetMaxA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n", a32addr,
	 addrMB, id);
  printf("faV3GetMaxA32MB: rval=0x%08x\n", rval);
#endif

  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Print the status of the FIFO to standard out
 *  @param id Slot number
 */

int
faV3PrintFifoStatus(int id)
{
  uint32_t ibuf, bbuf, obuf, dflow;
  uint32_t wc[2], mt[2], full[2];
  uint32_t rdy[2];

  CHECKID;

  FAV3LOCK;
  dflow = vmeRead32(&(FAV3p[id]->flow_status));
  ibuf = vmeRead32(&(FAV3p[id]->status1)) & 0xdfffdfff;
  bbuf = vmeRead32(&(FAV3p[id]->status2)) & 0x1fff1fff;
  obuf = vmeRead32(&(FAV3p[id]->status3)) & 0x3fff3fff;
  FAV3UNLOCK;

  printf("%s: Fifo Buffers Status (DataFlow Status = 0x%08x\n",
	 __func__, dflow);

  mt[1] = full[1] = 0;
  wc[1] = (ibuf & 0x7ff0000) >> 16;
  rdy[1] = (ibuf & 0x80000000) >> 31;
  if(ibuf & 0x8000000)
    full[1] = 1;
  if(ibuf & 0x10000000)
    mt[1] = 1;

  printf("  Input Buffer : 0x%08x \n", ibuf);
  printf("    FPGA : wc=%d   Empty=%d Full=%d Ready=%d\n", wc[1], mt[1],
	 full[1], rdy[1]);

  mt[0] = full[0] = 0;
  wc[0] = bbuf & 0x7ff;
  if(bbuf & 0x800)
    full[0] = 1;
  if(bbuf & 0x1000)
    mt[0] = 1;

  mt[1] = full[1] = 0;
  wc[1] = (bbuf & 0x7ff0000) >> 16;
  if(bbuf & 0x8000000)
    full[1] = 1;
  if(bbuf & 0x10000000)
    mt[1] = 1;

  printf("  Build Buffer : 0x%08x \n", bbuf);
  printf("    BUF_A: wc=%d   Empty=%d Full=%d \n", wc[1], mt[1], full[1]);
  printf("    BUF_B: wc=%d   Empty=%d Full=%d \n", wc[0], mt[0], full[0]);

  mt[0] = full[0] = 0;
  wc[0] = obuf & 0xfff;
  if(obuf & 0x1000)
    full[0] = 1;
  if(obuf & 0x2000)
    mt[0] = 1;

  mt[1] = full[1] = 0;
  wc[1] = (obuf & 0xfff0000) >> 16;
  if(obuf & 0x10000000)
    full[1] = 1;
  if(obuf & 0x20000000)
    mt[1] = 1;

  printf("  Output Buffer: 0x%08x \n", obuf);
  printf("    BUF_A: wc=%d   Empty=%d Full=%d \n", wc[1], mt[1], full[1]);
  printf("    BUF_B: wc=%d   Empty=%d Full=%d \n", wc[0], mt[0], full[0]);


  return OK;

}

/**
 *  @ingroup Status
 *  @brief Return the internal live trigger count
 *  @param id Slot number
 *  @param reset
 *    - >0: Reset count after read
 *  @return Internal Live trigger count if Successful, otherwise ERROR
 */

int
faV3Live(int id, int sflag)
{
  int rval = 0;

  CHECKID;

  /* Read Current Scaler values */
  FAV3LOCK;
  rval = (int)vmeRead32(&(FAV3p[id]->trig_live_count));

  /* Reset if requested */
  if(sflag)
    vmeWrite32(&(FAV3p[id]->trig_live_count), 0x80000000);
  FAV3UNLOCK;

  return (rval);
}


/**
 *  @ingroup Status
 *  @brief Decode a data word from an fADC250 and print to standard out.
 *  @param data 32bit fADC250 data word
 */

void
faV3DataDecode(uint32_t data)
{
  int i_print = 1;
  static uint32_t type_last = 15;	/* initialize to type FILLER WORD */
  static uint32_t time_last = 0;

  if(data & 0x80000000)		/* data type defining word */
    {
      faV3_data.new_type = 1;
      faV3_data.type = (data & 0x78000000) >> 27;
    }
  else
    {
      faV3_data.new_type = 0;
      faV3_data.type = type_last;
    }

  switch (faV3_data.type)
    {
    case 0:			/* BLOCK HEADER */
      faV3_data.slot_id_hd = (data & 0x7C00000) >> 22;
      faV3_data.n_evts = (data & 0x3FF800) >> 11;
      faV3_data.blk_num = (data & 0x7FF);
      if(i_print)
	printf("%8X - BLOCK HEADER - slot = %d   n_evts = %d   n_blk = %d\n",
	       data, faV3_data.slot_id_hd, faV3_data.n_evts,
	       faV3_data.blk_num);
      break;
    case 1:			/* BLOCK TRAILER */
      faV3_data.slot_id_tr = (data & 0x7C00000) >> 22;
      faV3_data.n_words = (data & 0x3FFFFF);
      if(i_print)
	printf("%8X - BLOCK TRAILER - slot = %d   n_words = %d\n",
	       data, faV3_data.slot_id_tr, faV3_data.n_words);
      break;
    case 2:			/* EVENT HEADER */
      if(faV3_data.new_type)
	{
	  faV3_data.evt_num_1 = (data & 0x7FFFFFF);
	  if(i_print)
	    printf("%8X - EVENT HEADER 1 - evt_num = %d\n", data,
		   faV3_data.evt_num_1);
	}
      /* else */
      /* 	{ */
      /* 	  faV3_data.evt_num_2 = (data & 0x7FFFFFF); */
      /* 	  if(i_print) */
      /* 	    printf("%8X - EVENT HEADER 2 - evt_num = %d\n", data, */
      /* 		   faV3_data.evt_num_2); */
      /* 	} */
      break;
    case 3:			/* TRIGGER TIME */
      if(faV3_data.new_type)
	{
	  faV3_data.time_1 = (data & 0xFFFFFF);
	  if(i_print)
	    printf("%8X - TRIGGER TIME 1 - time = %08x\n", data,
		   faV3_data.time_1);
	  faV3_data.time_now = 1;
	  time_last = 1;
	}
      else
	{
	  if(time_last == 1)
	    {
	      faV3_data.time_2 = (data & 0xFFFFFF);
	      if(i_print)
		printf("%8X - TRIGGER TIME 2 - time = %08x\n", data,
		       faV3_data.time_2);
	      faV3_data.time_now = 2;
	    }
	  /* else if(time_last == 2) */
	  /*   { */
	  /*     faV3_data.time_3 = (data & 0xFFFFFF); */
	  /*     if(i_print) */
	  /* 	printf("%8X - TRIGGER TIME 3 - time = %08x\n", data, */
	  /* 	       faV3_data.time_3); */
	  /*     faV3_data.time_now = 3; */
	  /*   } */
	  /* else if(time_last == 3) */
	  /*   { */
	  /*     faV3_data.time_4 = (data & 0xFFFFFF); */
	  /*     if(i_print) */
	  /* 	printf("%8X - TRIGGER TIME 4 - time = %08x\n", data, */
	  /* 	       faV3_data.time_4); */
	  /*     faV3_data.time_now = 4; */
	  /*   } */
	  else if(i_print)
	    printf("%8X - TRIGGER TIME - (ERROR)\n", data);

	  time_last = faV3_data.time_now;
	}
      break;
    case 4:			/* WINDOW RAW DATA */
      if(faV3_data.new_type)
	{
	  faV3_data.chan = (data & 0x7800000) >> 23;
	  faV3_data.width = (data & 0xFFF);
	  if(i_print)
	    printf("%8X - WINDOW RAW DATA - chan = %d   nsamples = %d\n",
		   data, faV3_data.chan, faV3_data.width);
	}
      else
	{
	  faV3_data.valid_1 = 1;
	  faV3_data.valid_2 = 1;
	  faV3_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if(data & 0x20000000)
	    faV3_data.valid_1 = 0;
	  faV3_data.adc_2 = (data & 0x1FFF);
	  if(data & 0x2000)
	    faV3_data.valid_2 = 0;
	  if(i_print)
	    printf
	      ("%8X - RAW SAMPLES - valid = %d  adc = %4d   valid = %d  adc = %4d\n",
	       data, faV3_data.valid_1, faV3_data.adc_1, faV3_data.valid_2,
	       faV3_data.adc_2);
	}
      break;
    case 5:			/* WINDOW SUM */
      faV3_data.over = 0;
      faV3_data.chan = (data & 0x7800000) >> 23;
      faV3_data.adc_sum = (data & 0x3FFFFF);
      if(data & 0x400000)
	faV3_data.over = 1;
      if(i_print)
	printf("%8X - WINDOW SUM - chan = %d   over = %d   adc_sum = %08x\n",
	       data, faV3_data.chan, faV3_data.over, faV3_data.adc_sum);
      break;
    case 6:			/* PULSE RAW DATA */
      if(faV3_data.new_type)
	{
	  faV3_data.chan = (data & 0x7800000) >> 23;
	  faV3_data.pulse_num = (data & 0x600000) >> 21;
	  faV3_data.thres_bin = (data & 0x3FF);
	  if(i_print)
	    printf("%8X - PULSE RAW DATA - chan = %d   pulse # = %d   threshold bin = %d\n",
		   data, faV3_data.chan, faV3_data.pulse_num,
		   faV3_data.thres_bin);
	}
      else
	{
	  faV3_data.valid_1 = 1;
	  faV3_data.valid_2 = 1;
	  faV3_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if(data & 0x20000000)
	    faV3_data.valid_1 = 0;
	  faV3_data.adc_2 = (data & 0x1FFF);
	  if(data & 0x2000)
	    faV3_data.valid_2 = 0;
	  if(i_print)
	    printf("%8X - PULSE RAW SAMPLES - valid = %d  adc = %d   valid = %d  adc = %d\n",
		   data, faV3_data.valid_1, faV3_data.adc_1, faV3_data.valid_2,
		   faV3_data.adc_2);
	}
      break;
    case 7:			/* PULSE INTEGRAL */
      faV3_data.chan = (data & 0x7800000) >> 23;
      faV3_data.pulse_num = (data & 0x600000) >> 21;
      faV3_data.quality = (data & 0x180000) >> 19;
      faV3_data.integral = (data & 0x7FFFF);
      if(i_print)
	printf("%8X - PULSE INTEGRAL - chan = %d   pulse # = %d   quality = %d   integral = %d\n",
	       data, faV3_data.chan, faV3_data.pulse_num, faV3_data.quality,
	       faV3_data.integral);
      break;
    case 8:			/* PULSE TIME */
      faV3_data.chan = (data & 0x7800000) >> 23;
      faV3_data.pulse_num = (data & 0x600000) >> 21;
      faV3_data.quality = (data & 0x180000) >> 19;
      faV3_data.time = (data & 0xFFFF);
      if(i_print)
	printf("%8X - PULSE TIME - chan = %d   pulse # = %d   quality = %d   time = %d\n",
	       data, faV3_data.chan, faV3_data.pulse_num, faV3_data.quality,
	       faV3_data.time);
      break;
    case 9:			/* STREAMING RAW DATA */
      if(faV3_data.new_type)
	{
	  faV3_data.chan_a = (data & 0x3C00000) >> 22;
	  faV3_data.source_a = (data & 0x4000000) >> 26;
	  faV3_data.chan_b = (data & 0x1E0000) >> 17;
	  faV3_data.source_b = (data & 0x200000) >> 21;
	  if(i_print)
	    printf("%8X - STREAMING RAW DATA - ena A = %d  chan A = %d   ena B = %d  chan B = %d\n",
		   data, faV3_data.source_a, faV3_data.chan_a, faV3_data.source_b,
		   faV3_data.chan_b);
	}
      else
	{
	  faV3_data.valid_1 = 1;
	  faV3_data.valid_2 = 1;
	  faV3_data.adc_1 = (data & 0x1FFF0000) >> 16;
	  if(data & 0x20000000)
	    faV3_data.valid_1 = 0;
	  faV3_data.adc_2 = (data & 0x1FFF);
	  if(data & 0x2000)
	    faV3_data.valid_2 = 0;
	  faV3_data.group = (data & 0x40000000) >> 30;
	  if(faV3_data.group)
	    {
	      if(i_print)
		printf("%8X - RAW SAMPLES B - valid = %d  adc = %d   valid = %d  adc = %d\n",
		       data, faV3_data.valid_1, faV3_data.adc_1,
		       faV3_data.valid_2, faV3_data.adc_2);
	    }
	  else if(i_print)
	    printf("%8X - RAW SAMPLES A - valid = %d  adc = %d   valid = %d  adc = %d\n",
		   data, faV3_data.valid_1, faV3_data.adc_1, faV3_data.valid_2,
		   faV3_data.adc_2);
	}
      break;
    case 10:			/* PULSE AMPLITUDE DATA */
      faV3_data.chan = (data & 0x7800000) >> 23;
      faV3_data.pulse_num = (data & 0x600000) >> 21;
      faV3_data.vmin = (data & 0x1FF000) >> 12;
      faV3_data.vpeak = (data & 0xFFF);
      if(i_print)
	printf("%8X - PULSE V - chan = %d   pulse # = %d   vmin = %d   vpeak = %d\n",
	       data, faV3_data.chan, faV3_data.pulse_num, faV3_data.vmin,
	       faV3_data.vpeak);
      break;

    case 11:			/* INTERNAL TRIGGER WORD */
      /* faV3_data.trig_type_int = data & 0x7; */
      /* faV3_data.trig_state_int = (data & 0x8) >> 3; */
      /* faV3_data.evt_num_int = (data & 0xFFF0) >> 4; */
      /* faV3_data.err_status_int = (data & 0x10000) >> 16; */
      /* if(i_print) */
      /* 	printf("%8X - INTERNAL TRIGGER - type = %d   state = %d   num = %d   error = %d\n", */
      /* 	       data, faV3_data.trig_type_int, faV3_data.trig_state_int, */
      /* 	       faV3_data.evt_num_int, faV3_data.err_status_int); */
    case 12:			/* UNDEFINED TYPE */
      if(i_print)
	printf("%8X - UNDEFINED TYPE = %d\n", data, faV3_data.type);
      break;
    case 13:			/* END OF EVENT */
      if(i_print)
	printf("%8X - END OF EVENT = %d\n", data, faV3_data.type);
      break;
    case 14:			/* DATA NOT VALID (no data available) */
      if(i_print)
	printf("%8X - DATA NOT VALID = %d\n", data, faV3_data.type);
      break;
    case 15:			/* FILLER WORD */
      if(i_print)
	printf("%8X - FILLER WORD = %d\n", data, faV3_data.type);
      break;
    }

  type_last = faV3_data.type;	/* save type of current data word */

}

/**
 *  @ingroup Config
 *  @brief Enable/Disable System test mode
 *  @param id Slot number
 *  @param mode
 *    -  0: Disable Test Mode
 *    - >0: Enable Test Mode
 */

int
faV3TestSetSystemTestMode(int id, int mode)
{
  int reg = 0;
  CHECKID;

  if(mode >= 1)
    reg = FAV3_CTRL1_SYSTEM_TEST_MODE;
  else
    reg = 0;

  FAV3LOCK;

  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&FAV3p[id]->ctrl1) | reg);

  /*   printf(" ctrl1 = 0x%08x\n",vmeRead32(&FAV3p[id]->ctrl1)); */
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the level of Trig Out to the SD
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @param mode
 *    -  0: Not asserted
 *    - >0: Asserted
 */

int
faV3TestSetTrigOut(int id, int mode)
{
  int reg = 0;
  CHECKID;

  if(mode >= 1)
    reg = FAV3_TESTBIT_TRIGOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->system_test.testbit), reg);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the level of Busy Out to the SD
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @param mode
 *    -  0: Not asserted
 *    - >0: Asserted
 */

int
faV3TestSetBusyOut(int id, int mode)
{
  int reg = 0;
  CHECKID;

  if(mode >= 1)
    reg = FAV3_TESTBIT_BUSYOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->system_test.testbit), reg);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the level of the SD Link
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @param mode
 *    -  0: Not asserted
 *    - >0: Asserted
 */

int
faV3TestSetSdLink(int id, int mode)
{
  int reg = 0;
  CHECKID;

  if(mode >= 1)
    reg = FAV3_TESTBIT_SDLINKOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->system_test.testbit), reg);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the level of Token Out to the SD
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @param mode
 *    -  0: Not asserted
 *    - >0: Asserted
 */

int
faV3TestSetTokenOut(int id, int mode)
{
  int reg = 0;
  CHECKID;

  if(mode >= 1)
    reg = FAV3_TESTBIT_TOKENOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->system_test.testbit), reg);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Get the level of the StatBitB to the SD
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return 1 if asserted, 0 if not, otherwise ERROR.
 */

int
faV3TestGetStatBitB(int id)
{
  int reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->system_test.testbit) & FAV3_TESTBIT_STATBITB) >> 8;
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Get the level of the Token In from the SD
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return 1 if asserted, 0 if not, otherwise ERROR.
 */

int
faV3TestGetTokenIn(int id)
{
  int reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->system_test.testbit) & FAV3_TESTBIT_TOKENIN) >> 9;
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Return the status of the 250Mhz Clock Counter
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return 1 if counting, 0 if not counting, otherwise ERROR.
 */

int
faV3TestGetClock250CounterStatus(int id)
{
  int reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->system_test.testbit) & FAV3_TESTBIT_CLOCK250_STATUS) >> 15;
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Return the value of the 250Mhz Clock scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return 250Mhz Clock scaler counter if successful, otherwise ERROR.
 */

uint32_t
faV3TestGetClock250Counter(int id)
{
  uint32_t reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->system_test.count_250);
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Return the value of the SyncReset scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return SyncReset scaler counter if successful, otherwise ERROR.
 */

uint32_t
faV3TestGetSyncCounter(int id)
{
  uint32_t reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->system_test.count_sync);
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Return the value of the trig1 scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return trig1 scaler counter if successful, otherwise ERROR.
 */

uint32_t
faV3TestGetTrig1Counter(int id)
{
  uint32_t reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->system_test.count_trig1);
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Return the value of the trig2 scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return trig2 scaler counter if successful, otherwise ERROR.
 */

uint32_t
faV3TestGetTrig2Counter(int id)
{
  uint32_t reg = 0;
  CHECKID;

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->system_test.count_trig2);
  FAV3UNLOCK;

  return reg;

}

/**
 *  @ingroup Status
 *  @brief Reset the counter of the 250MHz Clock scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 */

int
faV3TestResetClock250Counter(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->system_test.count_250, FAV3_CLOCK250COUNT_RESET);
  vmeWrite32(&FAV3p[id]->system_test.count_250, FAV3_CLOCK250COUNT_START);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Reset the counter of the SyncReset scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 */

int
faV3TestResetSyncCounter(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->system_test.count_sync, FAV3_SYNCP0COUNT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Reset the counter of the trig1 scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 */

int
faV3TestResetTrig1Counter(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->system_test.count_trig1, FAV3_TRIG1P0COUNT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Reset the counter of the trig2 scaler
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 */

int
faV3TestResetTrig2Counter(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->system_test.count_trig2, FAV3_TRIG2P0COUNT_RESET);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Return the current value of the testBit register
 *
 *   Available only in System Test Mode
 *
 *  @sa faV3TestSetSystemTestMode
 *  @param id Slot number
 *  @return testBit register value if successful, otherwise ERROR.
 */

uint32_t
faV3TestGetTestBitReg(int id)
{
  uint32_t rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->system_test.testbit);
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the status of the current system clock
 *
 *   Enable and disables test mode during operation
 *
 *  @sa faV3TestSetSystemTestMode
 *  @sa faV3TestGetClock250CounterStatus
 *  @sa faV3TestGetClock250Counter
 *  @sa faV3TestResetClock250Counter
 *
 *  @param id Slot number
 *  @param pflag print to standard output if > 0
 *
 *  @return OK if system clock is available, otherwise ERROR.
 */

int
faV3TestSystemClock(int id, int pflag)
{
  uint32_t rval = OK;

  CHECKID;

  /* Enable test mode */
  faV3TestSetSystemTestMode(id, 1);

  /* reset clock counter */
  faV3TestResetClock250Counter(id);

  int iwait = 0;
  /* Wait for the 20us internal timer */
  while(iwait++ < 50)
    {
      if(faV3TestGetClock250CounterStatus(id) == 0)
	break;
    }

  /* Counter should return 5000 if the system clock is 250Mhz */
  int expected = 5000, measured = 0, diff = 0;

  measured = faV3TestGetClock250Counter(id);
  diff = abs(expected - measured);

  if(diff < 5)
    rval = OK;
  else
    rval = ERROR;

  /* Disable test mode */
  faV3TestSetSystemTestMode(id, 0);

  if(pflag)
    {
      printf("%s: System Clock is %s\n",
	     __func__, (rval == OK) ? "Present" : "NOT PRESENT");
    }

  return rval;
}


/**
 *  @ingroup Status
 *  @brief Fills 'rval' with a character array containing the fa250 serial number.
 *  @param id Slot number
 *  @param rval Where to return Serial number string
 *  @param snfix
 *<pre>
 *      If snfix >= 1, will attempt to format the serial number to maintain
 *        consistency between serials made by the same vendor.
 *        e.g. Advanced Assembly:
 *          snfix=0: B21595-02R  B2159515R1
 *          snfix=1: B21595-02R  B21595-15R1
 *        e.g. ACDI
 *          snfix=0: ACDI002
 *          snfix=1: ACDI-002
 *</pre>
 *  @return length of character array 'rval' if successful, otherwise ERROR
 */

int
faV3GetSerialNumber(int id, char **rval, int snfix)
{
  uint32_t sn[3];
  int i = 0, ivme = 0, ibyte = 0;
  uint32_t byte;
  uint32_t shift = 0, mask = 0;
  uint32_t boardID;
  char boardID_c[4];
  char byte_c[2];
  char adv_str[12], acdi_str[12];
  char ret[12];
  int ret_len;

  CHECKID;

  FAV3LOCK;
  for(i = 0; i < 3; i++)
    sn[i] = vmeRead32(&FAV3p[id]->serial_reg[i]);
  FAV3UNLOCK;

  if(sn[0] == FAV3_SERIAL_NUMBER_ACDI)
    {				/* ACDI */
      strcpy(acdi_str, "");
      for(ibyte = 3; ibyte >= 0; ibyte--)
	{
	  shift = (ibyte * 8);
	  mask = (0xFF) << shift;
	  byte = (sn[ivme] & mask) >> shift;
	  sprintf(byte_c, "%c", byte);
	  strcat(acdi_str, byte_c);
	}
      boardID = (sn[1] & FAV3_SERIAL_NUMBER_ACDI_BOARDID_MASK);
      if(boardID > 999)
	{
	  printf("%s: WARN: Invalid Board ACDI Board ID (%d)\n",
		 __func__, boardID);
	}

      if(snfix > 0)		/* If needed, Add '-' after the ACDI */
	sprintf(boardID_c, "-%03d", boardID);
      else
	sprintf(boardID_c, "%03d", boardID);

      strcat(acdi_str, boardID_c);
#ifdef DEBUGSN
      printf("acdi_str = %s\n", acdi_str);
#endif
      strcpy(ret, acdi_str);

    }

  else if((sn[0] & FAV3_SERIAL_NUMBER_ADV_ASSEM_MASK) ==
	  FAV3_SERIAL_NUMBER_ADV_ASSEM)

    {				/* ADV ASSEM */
      /* Make sure manufacture's ID is correct */
      if((sn[0] == FAV3_SERIAL_NUMBER_ADV_MNFID1) &&
	 ((sn[1] & FAV3_SERIAL_NUMBER_ADV_MNFID2_MASK) ==
	  FAV3_SERIAL_NUMBER_ADV_MNFID2))
	{
	  strcpy(adv_str, "");
	  for(ivme = 0; ivme < 3; ivme++)
	    {
	      for(ibyte = 3; ibyte >= 0; ibyte--)
		{
		  shift = (ibyte * 8);
		  mask = (0xFF) << shift;
		  byte = (sn[ivme] & mask) >> shift;
		  if(byte == 0xFF)
		    {
		      break;
		    }
		  if(snfix > 0)
		    {		/* If needed, Add '-' after the B21595 */
		      if(ivme == 1 && ibyte == 1)
			{
			  if(byte != 0x2D)	/* 2D = - */
			    {
			      strcat(adv_str, "-");
			    }
			}
		    }

		  sprintf(byte_c, "%c", byte);
		  strcat(adv_str, byte_c);
		}
	    }
#ifdef DEBUGSN
	  printf("adv_str = %s\n", adv_str);
#endif
	  strcpy(ret, adv_str);
	}
      else
	{
	  printf("%s: ERROR: Unable to determine manufacture's ID.  SN regs:\n",
		 __func__);
	  for(i = 0; i < 3; i++)
	    printf("\t%d: 0x%08x\n", i, sn[i]);
	  return -1;
	}
    }
  else
    {
      printf("%s: ERROR: Unable to determine manufacture's ID. SN regs:\n",
	     __func__);
      for(i = 0; i < 3; i++)
	printf("\t%d: 0x%08x\n", i, sn[i]);
      return -1;
    }

#ifdef DEBUGSN
  printf("ret = %s\n", ret);
#endif
  strcpy((char *) rval, ret);

  ret_len = (int) strlen(ret);

  return (ret_len);

}


/**
 *  @ingroup Config
 *  @brief Set the block interval of scaler data insertion
 *
 *   Data from scalers may be inserted into the readout data stream at
 *   regular event count intervals.  The interval is specified in
 *   multiples of blocks.
 *    Note: Scalers are NOT reset after their values are captured.
 *
 *  @param id Slot number
 *  @param nblock
 *    -   0: No insertion of scaler data into the data stream
 *    - >=1: The current scaler values are appended to the last event of the appropriate n'th block of events.
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SetScalerBlockInterval(int id, uint32_t nblock)
{
  CHECKID;

  if(nblock > FAV3_SCALER_INSERT_MASK)
    {
      printf("%s: ERROR: Invalid value of nblock (%d).\n", __func__, nblock);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_insert, nblock);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3GetScalerBlockInterval(int id)
{
  int rval = 0;

  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->scaler_insert) & FAV3_SCALER_INSERT_MASK;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Allows for the insertion of a block trailer into the data stream.
 *
 *      Allows for the insertion of a block trailer into the data stream.  This is
 *      useful for the efficient extraction of a partial block of events
 *      from the FADC (e.g. for an end of run event, or the resynchonize with
 *      other modules).
 *      Event count within block is reset, after successful execution.
 *
 *  @param id Slot number
 *  @param scalers If set to > 0, scalers will also be inserted with the End of Block
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3ForceEndOfBlock(int id, int scalers)
{
  int rval = OK, icheck = 0, timeout = 1000, csr = 0;
  int proc_config = 0;

  CHECKID;

  FAV3LOCK;
  /* Disable triggers to Processing FPGA (if enabled) */
  proc_config = vmeRead32(&FAV3p[id]->adc.config1);
  vmeWrite32(&FAV3p[id]->adc.config1, proc_config & ~(FAV3_ADC_PROC_ENABLE));

  csr = FAV3_CSR_FORCE_EOB_INSERT;
  if(scalers > 0)
    csr |= FAV3_CSR_DATA_STREAM_SCALERS;

  vmeWrite32(&FAV3p[id]->csr, csr);

  for(icheck = 0; icheck < timeout; icheck++)
    {
      csr = vmeRead32(&FAV3p[id]->csr);
      if(csr & FAV3_CSR_FORCE_EOB_SUCCESS)
	{
	  logMsg("faV3ForceEndOfBlock: Block trailer insertion successful\n",
		 1, 2, 3, 4, 5, 6);
	  rval = ERROR;
	  break;
	}

      if(csr & FAV3_CSR_FORCE_EOB_FAILED)
	{
	  logMsg("faV3ForceEndOfBlock: Block trailer insertion FAILED\n",
		 1, 2, 3, 4, 5, 6);
	  rval = ERROR;
	  break;
	}
    }

  if(icheck == timeout)
    {
      logMsg("faV3ForceEndOfBlock: Block trailer insertion FAILED on timeout\n",
	     1, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  /* Restore the original state of the Processing FPGA */
  vmeWrite32(&FAV3p[id]->adc.config1, proc_config);

  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Allows for the insertion of a block trailer into the data stream for all
 *    initialized fADC250s
 *
 *      Allows for the insertion of a block trailer into the data stream.  This is
 *      useful for the efficient extraction of a partial block of events
 *      from the FADC (e.g. for an end of run event, or the resynchonize with
 *      other modules).
 *      Event count within block is reset, after successful execution.
 *
 *  @param scalers If set to > 0, scalers will also be inserted with the End of Block
 */

void
faV3GForceEndOfBlock(int scalers)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faV3ForceEndOfBlock(faV3ID[ii], scalers);
      if(res < 0)
	printf("%s: ERROR: slot %d, in faForceEndOfBlock()\n",
	       __func__, faV3ID[ii]);
    }

}

/**
 *  @ingroup Config
 *  @brief Set the threshold to trigger for the history buffer to be saved for readout
 *  @param id Slot number
 *  @param thres History Buffer Threshold
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3SetHistoryBufferThreshold(int id, int thres)
{
  CHECKID;

  if(thres>FAV3_SUM_THRESHOLD_MASK)
    {
      printf("%s: ERROR: Invalid value for threshold (%d)\n",
	     __FUNCTION__,thres);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->sum_threshold,thres);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the threshold to trigger for the history buffer to be saved for readout
 *     for all initialized fADC250s
 *  @param thres History Buffer Threshold
 */
void
faV3GSetHistoryBufferThreshold(int thres)
{
  int ifa=0;

  for (ifa=0;ifa<nfaV3;ifa++)
    {
      faV3SetHistoryBufferThreshold(faV3Slot(ifa),thres);
    }
}

/**
 *  @ingroup Status
 *  @brief Get the history buffer threshold
 *  @param id Slot number
 *  @return threshold if successful, otherwise ERROR.
 */
int
faV3GetHistoryBufferThreshold(int id)
{
  int rval=0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->sum_threshold) & FAV3_SUM_THRESHOLD_MASK;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Config
 *  @brief Enable the history buffer for data acquisition for the module
 *  @param id Slot number
 *  @return OK if successful, otherwise ERROR.
 */
int
faV3ArmHistoryBuffer(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->sum_data, FAV3_SUM_DATA_ARM_HISTORY_BUFFER);
  FAV3UNLOCK;
  return OK;
}

/**
 *  @ingroup Config
 *  @brief Enable the history buffer for data acquisition for all initialized fADC250s
 */
void
faV3GArmHistoryBuffer()
{
  int ifa=0;

  for (ifa=0;ifa<nfaV3;ifa++)
    {
      faV3ArmHistoryBuffer(faV3Slot(ifa));
    }
}

/**
 *  @ingroup Readout
 *  @brief Return whether or not the history buffer has been triggered
 *  @param id Slot number
 *  @return 1 if history buffer data is ready for readout, 0 if not, otherwise ERROR.
 */
int
faV3HistoryBufferDReady(int id)
{
  int rval=0;
  CHECKID;

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->sum_threshold) & FAV3_SUM_THRESHOLD_DREADY)>>31;
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Readout
 *  @brief Read out history buffer from the module
 *  @param id Slot number
 *  @param  data   local memory address to place data
 *  @param  nwrds  Max number of words to transfer
 *  @return Number of words read if successful, otherwise ERROR.
 */
int
faV3ReadHistoryBuffer(int id, volatile unsigned int *data, int nwrds)
{
  int idata=0, dCnt=0;
  CHECKID;

  FAV3LOCK;
  while(idata<nwrds)
    {
      data[idata] = vmeRead32(&FAV3p[id]->sum_data) & FAV3_SUM_DATA_SAMPLE_MASK;
#ifndef VXWORKS
      data[idata] = LSWAP(data[idata]);
#endif
      idata++;
    }
  idata++;

  /* Use this to clear the data ready bit (dont set back to zero) */
  vmeWrite32(&FAV3p[id]->sum_data,FAV3_SUM_DATA_ARM_HISTORY_BUFFER);

  FAV3UNLOCK;
  dCnt += idata;

  return dCnt;
}



/**
 *  @ingroup Config
 *  @brief Enable/Disable Buffer to store state machine diagnostics
 *  @param id Slot number
 *  @param enable If enable != 0, enable buffer, otherwise disable.
 *  @return OK successful, otherwise ERROR.
 */

int
faV3StateArmBuffer(int id, int enable)
{
  CHECKID;

  FAV3LOCK;
  if(enable)
    vmeWrite32(&FAV3p[id]->aux.state_csr, FAV3_STATE_CSR_ARM_BUFFER);
  else
    vmeWrite32(&FAV3p[id]->aux.state_csr, 0);
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Read state machine buffer.
 *  @param id Slot number
 *  @param  data   local memory address to place data
 *  @param  nwrds  Max number of words to transfer
 *  @return Number of words read if successful, otherwise ERROR.
 */
int
faV3StateReadBuffer(int id, volatile unsigned int *data, int nwords)
{
  int rval=0, idata=0, ndata=0;
  CHECKID;

  FAV3LOCK;
  /* Read in how many words are available */
  ndata = vmeRead32(&FAV3p[id]->aux.state_csr) & FAV3_STATE_CSR_BUFFER_WORDS_MASK;

  if(ndata == 0)
    {
      logMsg("faV3StateReadBuffer(%d): WARN: No words in State Machine buffer\n",
	     id, 2, 3, 4, 5, 6);
      rval = 0;
    }
  else
    {
      if(ndata > nwords)
	{
	  logMsg("faV3StateReadBuffer(%d): WARN: %d words remain in State Machine buffer\n",
		 id, ndata, 3, 4, 5, 6);

	}
      for(idata = 0; idata < ndata; idata++)
	{
	  data[idata] = vmeRead32(&FAV3p[id]->aux.state_value) & FAV3_STATE_VALUE_MASK;
	}
      rval = ndata;
    }

  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Convert state value to mapped identifier
 *  @param state_value State Value to convert
 *  @return Mapped id if successful, otherwise ERROR.
 */
int
faV3StateMap(unsigned int state_value)
{
  int rval=0;

  switch(state_value)
    {
    case 0x0:
      rval = 0;
      break;

    case 0x02000:
      rval = 4;
      break;

    case 0x04000:
      rval = 1001;
      break;

    case 0x06000:
      rval = 134;
      break;

    case 0x08000:
      rval = 135;
      break;

    case 0x0A000:
      rval = 102;
      break;

    case 0x0C000:
      rval = 1003;
      break;

    case 0x0E000:
      rval = 104;
      break;

    case 0x10000:
      rval = 1005;
      break;

    case 0x12000:
      rval = 106;
      break;

    case 0x14000:
      rval = 722;
      break;

    case 0x16000:
      rval = 155;
      break;

    case 0x18000:
      rval = 1009;
      break;

    case 0x00002:
      rval = 1;
      break;

    case 0x02002:
      rval = 101;
      break;

    case 0x00020:
      rval = 2;
      break;

    case 0x02020:
      rval = 6;
      break;

    case 0x00024:
      rval = 3;
      break;

    case 0x00008:
      rval = 5;
      break;

    case 0x02008:
      rval = 105;
      break;

    case 0x00100:
      rval = 55;
      break;

    case 0x00071:
      rval = 7;
      break;

    case 0x02071:
      rval = 14;
      break;

    case 0x00011:
      rval = 9;
      break;

    case 0x02011:
      rval = 130;
      break;

    case 0x04011:
      rval = 131;
      break;

    case 0x06011:
      rval = 1010;
      break;

    case 0x08011:
      rval = 1011;
      break;

    case 0x00051:
      rval = 10;
      break;

    case 0x00031:
      rval = 12;
      break;

    case 0x02031:
      rval = 22;
      break;

    case 0x04031:
      rval = 23;
      break;

    case 0x06031:
      rval = 27;
      break;

    case 0x08031:
      rval = 121;
      break;

    case 0x00211:
      rval = 20;
      break;

    case 0x02211:
      rval = 129;
      break;

    case 0x00231:
      rval = 21;
      break;

    case 0x02231:
      rval = 128;
      break;

    case 0x00531:
      rval = 24;
      break;

    case 0x000B1:
      rval = 32;
      break;

    case 0x020B1:
      rval = 33;
      break;

    case 0x001B1:
      rval = 34;
      break;

    case 0x021B1:
      rval = 35;
      break;

    case 0x025B1:
      rval = 36;
      break;

    case 0x00800:
      rval = 132;
      break;

    case 0x02800:
      rval = 336;
      break;

    case 0x00C00:
      rval = 133;
      break;

    case 0x00004:
      rval = 103;
      break;

    case 0x01000:
      rval = 109;
      break;

    case 0x03000:
      rval = 1012;
      break;

    default:
      rval = -1;		// no valid state

    }

  return rval;
}


/**
 *  @ingroup Status
 *  @brief Print the contents of the State Machine buffer to standard out.
 *  @param state_value State Value to convert
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3StatePrintBuffer(int id)
{
  unsigned int data[0xff];
  int nwords = 0, idata = 0;

  CHECKID;

  nwords = faV3StateReadBuffer(id, (volatile unsigned int *)&data, 0xff);
  if(nwords < 0)
    {
      logMsg("faV3StatePrintBuffer(%d): ERROR: Unable to retreive state machine data\n", id, 2, 3, 4, 5, 6);
      return ERROR;
    }

  printf("\n--- number of state values saved = %d\n\n", nwords);
  for(idata = 0; idata < nwords; idata++)
    {
      printf("state %4d   value = %5X   id = %4d\n",
	     (idata + 1),
	     data[idata],
	     faV3StateMap(data[idata]));
    }

  return OK;
}

/**
 * @ingroup Config
 * @brief Enable / Disable sparsification
 * @details Enable or disable the sparsification logic for the specified module
 * @param[in] id fadc slot number
 * @param[in] mode sparsification mode
 *     0 : Bypass sparsification
 *     1 : Enable
 * @return OK if successful, otherwise ERROR
 */
int
faV3SetSparsificationMode(int id, int mode)
{
  CHECKID;

  /* logic in register is reversed */
  mode = mode ? 0 : 1;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->aux.sparsify_control, mode);
  FAV3UNLOCK;

  return (OK);
}


/**
 * @ingroup Status
 * @brief Enable / Disable sparsification
 * @details Enable or disable the sparsification logic for all initialized modules
 * @param[in] mode sparsification mode
 *     0 : Bypass sparsification
 *     1 : Enable
 * @return OK if successful, otherwise ERROR
 */
void
faV3GSetSparsificationMode(int mode)
{
  int id = 0;

  /* logic in register is reversed */
  mode = mode ? 0 : 1;

  FAV3LOCK;

  for(id = 0; id < nfaV3; id++)
    vmeWrite32(&FAV3p[id]->aux.sparsify_control, mode);

  FAV3UNLOCK;
}

/**
 * @ingroup Status
 * @brief Sparisification is Enabled / Disabled
 * @details Return the state of the sparsification logic for specified module
 * @param[in] id fadc slot number
 * @return 1 if sparsification is enabled, 0 if bypassed, otherwise ERROR
 */
int
faV3GetSparsificationMode(int id)
{
  int mode = 0, rval = 0;
  CHECKID;

  FAV3LOCK;
  mode =
    (int) (vmeRead32(&FAV3p[id]->aux.sparsify_control) & FAV3_SPARSE_CONTROL_BYPASS);

  /* logic in register is reversed */
  rval = mode ? 0 : 1;
  FAV3UNLOCK;

  return (rval);
}

int
faV3GetSparsificationStatus(int id)
{
  int rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = (int) (vmeRead32(&FAV3p[id]->aux.sparsify_status) & FAV3_SPARSE_STATUS_MASK);
  FAV3UNLOCK;

  return rval;
}

int
faV3ClearSparsificationStatus(int id)
{
  CHECKID;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->aux.sparsify_control, FAV3_SPARSE_STATUS_CLEAR);
  FAV3UNLOCK;

  return (OK);
}

void
faV3GClearSparsificationStatus()
{
  int id = 0;

  FAV3LOCK;
  for(id = 0; id < nfaV3; id++)
    vmeWrite32(&FAV3p[id]->aux.sparsify_control, FAV3_SPARSE_STATUS_CLEAR);

  FAV3UNLOCK;
}


/**
 *  @ingroup Status
 *  @brief Print to standard out some auxillary scalers
 *
 *   Prints out
 *     - Total number of words generated
 *     - Total number of headers generated
 *     - Total number of trailers generated
 *     - Total number of lost triggers
 *
 *  @param id Slot number
 */

int
faV3PrintAuxScal(int id)
{
  CHECKID;

  FAV3LOCK;
  printf("Auxillary Scalers:\n");
  printf("       Word Count:         %d\n",
	 vmeRead32(&FAV3p[id]->proc_words_scal));
  printf("       Headers   :         %d\n", vmeRead32(&FAV3p[id]->header_scal));
  printf("       Trailers  :         %d\n",
	 vmeRead32(&FAV3p[id]->trailer_scal));
  printf("  Lost Triggers  :         %d\n",
	 vmeRead32(&FAV3p[id]->lost_trig_scal));
  FAV3UNLOCK;

  return OK;
}

/**
 *  @ingroup Status
 *  @brief Return the first trigger mismatch count
 *
 *  @param id Slot number
 *  @return mismatch count, if successful.  Otherwise ERROR
 */

uint32_t
faV3GetFirstTriggerMismatch(int id)
{
  uint32_t rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->aux.first_trigger_mismatch);
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the trigger mismatch count
 *
 *  @param id Slot number
 *  @return mismatch count, if successful.  Otherwise ERROR
 */

uint32_t
faV3GetMismatchTriggerCount(int id)
{
  uint32_t rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->aux.trigger_mismatch_counter);
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the triggers processed count
 *
 *  @param id Slot number
 *  @return triggers processed count, if successful.  Otherwise ERROR
 */

uint32_t
faV3GetTriggersProcessedCount(int id)
{
  uint32_t rval = 0;
  CHECKID;

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->aux.triggers_processed);
  FAV3UNLOCK;

  return rval;
}

/***************************************************************************************
   JLAB FADC Signal Distribution Card (SDC) Routines
***************************************************************************************/

/**
 *  @ingroup SDCConfig
 *  @brief Configure the Signal Distribution Card (SDC)
 *  @param id Slot number
 *  @param  cFlag  controls the configuation of the SDC
 *   -  0:  Default Mode  Internal CLK, Sync External Trigger and Sync Reset
 *   - >0:  Pass through mode
 *  @param bMask:  mask of Busy enables for the SDC - Do not Enable busy if there is no FADC
 *  @return OK if successful, otherwise ERROR.
 */

int
faV3SDC_Config(uint16_t cFlag, uint16_t bMask)
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Config: ERROR : Cannot Configure FADC Signal Board \n", 0,
	     0, 0, 0, 0, 0);
      return (ERROR);
    }

  /* Reset the Board */
  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->csr), FAV3SDC_CSR_INIT);

  if(cFlag == 0)
    {
      /* Default - Enable Internal Clock, Sync Trigger and Sync-Reset */
      vmeWrite16(&(FAV3SDCp->ctrl),
		 (FAV3SDC_CTRL_ENABLE_SOFT_TRIG |
		  FAV3SDC_CTRL_ENABLE_SOFT_SRESET));
      faV3SDCPassthrough = 0;
    }
  else if(cFlag == 1)
    {
      /* Pass Through - */
      vmeWrite16(&(FAV3SDCp->ctrl),
		 (FAV3SDC_CTRL_CLK_EXT | FAV3SDC_CTRL_NOSYNC_TRIG |
		  FAV3SDC_CTRL_NOSYNC_SRESET));
      faV3SDCPassthrough = 1;
    }
  else
    {
      /* Level Translator - re-sync the signals coming in to the SDC */
      vmeWrite16(&(FAV3SDCp->ctrl), (FAV3SDC_CTRL_CLK_EXT));
      faV3SDCPassthrough = 1;
    }

  vmeWrite16(&(FAV3SDCp->busy_enable), bMask);
  FAV3UNLOCK;

  return (OK);
}

/**
 *  @ingroup SDCStatus
 *  @brief Print status of SDC to standard out
 *  @param sFlag Not used
 */

void
faV3SDC_Status(int sFlag)
{

  uint16_t sdc[4];
  int ibit = 0;

  if(FAV3SDCp == NULL)
    {
      printf("faV3SDC_Status: ERROR : No FADC SDC available \n");
      return;
    }

  FAV3LOCK;
  sdc[0] = vmeRead16(&(FAV3SDCp->csr));
  sdc[1] = vmeRead16(&(FAV3SDCp->ctrl)) & FAV3SDC_CTRL_MASK;
  sdc[2] = vmeRead16(&(FAV3SDCp->busy_enable)) & FAV3SDC_BUSY_MASK;
  sdc[3] = vmeRead16(&(FAV3SDCp->busy_status));
  FAV3UNLOCK;


#ifdef VXWORKS
  printf("\nSTATUS for FADC Signal Distribution Card at base address 0x%x \n",
	 (uint32_t) FAV3SDCp);
#else
  printf("\nSTATUS for FADC Signal Distribution Card at\n VME (Local) base address 0x%x (0x%lx)\n",
	 (uint32_t) ((u_long) FAV3SDCp - faV3A16Offset), (u_long) FAV3SDCp);
#endif
  printf("---------------------------------------------------------------- \n");

  printf(" Board Firmware Rev/ID = 0x%02x\n", ((sdc[0] & 0xff00) >> 8));
  printf(" Registers: \n");
  printf("   CSR         = 0x%04x     Control     = 0x%04x\n", sdc[0],
	 sdc[1]);
  printf("   Busy Enable = 0x%04x     Busy Status = 0x%04x\n", sdc[2],
	 sdc[3]);
  printf("\n");

  if((sdc[1] & FAV3SDC_CTRL_CLK_EXT))
    printf(" Ref Clock : External\n");
  else
    printf(" Ref Clock : Internal\n");


  printf("   Trigger :");
  if((sdc[1] & FAV3SDC_CTRL_ENABLE_SOFT_TRIG))
    {
      printf(" Internal (Software)\n");
    }
  else
    {
      if((sdc[1] & FAV3SDC_CTRL_NOSYNC_TRIG))
	printf(" External (Pass through)\n");
      else
	printf(" External (Sync with clock)\n");
    }

  printf(" SyncReset :");
  if((sdc[1] & FAV3SDC_CTRL_ENABLE_SOFT_SRESET))
    {
      printf(" Internal (Software)\n");
    }
  else
    {
      if((sdc[1] & FAV3SDC_CTRL_NOSYNC_SRESET))
	printf(" External (Pass through)\n");
      else
	printf(" External (Sync with clock)\n");
    }
  printf("\n");
  printf(" Busy Ports\n  Enabled  :");
  for(ibit = 0; ibit < 7; ibit++)
    if((1 << ibit) & sdc[2])
      printf(" %d", ibit + 1);

  printf("\n");

  printf("\n");
  printf(" Busy Ports\n  Asserted :");
  for(ibit = 0; ibit < 7; ibit++)
    if((1 << ibit) & sdc[3])
      printf(" %d", ibit + 1);

  printf("\n");

  printf("\n");

}

/**
 *  @ingroup SDCConfig
 *  @brief Enable Triggers and/or SyncReset on the SDC
 *  @param nsync
 *    -  0: Front panel triggers and syncreset
 *    - !0: Front panel triggers only
 */

void
faV3SDC_Enable(int nsync)
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Enable: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  if(nsync != 0)		/* FP triggers only */
    vmeWrite16(&(FAV3SDCp->ctrl), FAV3SDC_CTRL_ENABLE_SOFT_SRESET);
  else				/* both FP triggers and sync reset */
    vmeWrite16(&(FAV3SDCp->ctrl), 0);
  FAV3UNLOCK;
}

/**
 *  @ingroup SDCConfig
 *  @brief Disable Triggers and SyncReset on the SDC
 */

void
faV3SDC_Disable()
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Disable: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->ctrl),
	     (FAV3SDC_CTRL_ENABLE_SOFT_TRIG | FAV3SDC_CTRL_ENABLE_SOFT_SRESET));
  FAV3UNLOCK;
}

/**
 *  @ingroup SDCConfig
 *  @brief Perform a SyncReset from the SDC
 */

void
faV3SDC_Sync()
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Sync: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->csr), FAV3SDC_CSR_SRESET);
  FAV3UNLOCK;
}

/**
 *  @ingroup SDCConfig
 *  @brief Perform a trigger pulse from the SDC
 */

void
faV3SDC_Trig()
{
  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Trig: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->csr), FAV3SDC_CSR_TRIG);
  FAV3UNLOCK;
}

/**
 *  @ingroup SDCStatus
 *  @brief Return Busy status of the SDC
 *  @return 1 if busy, 0 if not, otherwise ERROR.
 */

int
faV3SDC_Busy()
{
  int busy = 0;

  if(FAV3SDCp == NULL)
    {
      logMsg("faV3SDC_Busy: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return -1;
    }

  FAV3LOCK;
  busy = vmeRead16(&(FAV3SDCp->csr)) & FAV3SDC_CSR_BUSY;
  FAV3UNLOCK;

  return (busy);
}
