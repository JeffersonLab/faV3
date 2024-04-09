/********************************************************************************
 *  faV3Lib.c  -    Library for JLAB configuration and readout of JLAB 250MHz FLASH
 *                  ADC V3 using a VxWorks >=5.4 or Linux >=2.6.18 based Single
 *                  Board computer.
 *
 *  Author: David Abbott & Bryan Moffit
 *          Jefferson Lab Data Acquisition Group
 *          April 2024
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
int faV3Rev[(FAV3_MAX_BOARDS + 1)];	/* Board Revision Info for each module */
int faV3ProcRev[(FAV3_MAX_BOARDS + 1)];	/* Processing FPGA Revision Info for each module */
uint16_t faV3ChanDisable[(FAV3_MAX_BOARDS + 1)];	/* Disabled Channel Mask for each Module */
int faV3Inited = 0;		/* >0 if Library has been Initialized before */
int faV3MaxSlot = 0;		/* Highest Slot hold an FAV3 */
int faV3MinSlot = 0;		/* Lowest Slot holding an FAV3 */
int faV3Source = 0;		/* Signal source for FAV3 system control */
int faV3BlockLevel = 0;		/* Block Level for ADCs */
int faV3IntCount = 0;		/* Count of interrupts from FAV3 */
int faV3UseSDC = 0;		/* If > 0 then Use Signal Distribution board */
int faV3SDCPassthrough = 0;	/* If > 0 SDC in level translate / passthrough mode */
faV3data_t faV3_data;
int faV3BlockError = FAV3_BLOCKERROR_NO_ERROR;	/* Whether (>0) or not (0) Block Transfer had an error */

/*sergey*/
static int proc_mode[(FAV3_MAX_BOARDS + 1)];

/* Include Firmware Tools */
#include "faV3FirmwareTools.c"

/*******************************************************************************
 *
 * faInit - Initialize JLAB FADC V3 Library.
 *
 *
 *   iFlag: 18 bit integer
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
 *               (all others Undefined - default to Internal)
 *       bits 5-4:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  P2 Connector (Backplane)
 *
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
 *      bit 17:  Use faV3AddrList instead of addr and addr_inc
 *               for VME addresses.
 *             0 Initialize with addr and addr_inc
 *             1 Use faV3AddrList
 *
 *      bit 18:  Skip firmware check.  Useful for firmware updating.
 *             0 Perform firmware check
 *             1 Skip firmware check
 *
 *
 * RETURNS: OK, or ERROR if the address is invalid or a board is not present.
 */

STATUS
faInit(uint32_t addr, uint32_t addr_inc, int nadc, int iFlag)
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
  int noFirmwareCheck = 0;
  uint16_t supported_ctrl[FAV3_SUPPORTED_CTRL_FIRMWARE_NUMBER]
    = { FAV3_SUPPORTED_CTRL_FIRMWARE };
  uint16_t supported_proc[FAV3_SUPPORTED_PROC_FIRMWARE_NUMBER]
    = { FAV3_SUPPORTED_PROC_FIRMWARE };
  uint16_t ctrl_version = 0, proc_version = 0;
  int icheck = 0, ctrl_supported = 0, proc_supported = 0;

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
  noBoardInit = (iFlag & FAV3_INIT_SKIP) >> 16;

  /* Check if we're initializing using a list */
  useList = (iFlag & FAV3_INIT_USE_ADDRLIST) >> 17;

  /* Are we skipping the firmware check? */
  noFirmwareCheck = (iFlag & FAV3_INIT_SKIP_FIRMWARE_CHECK) >> 18;

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
  memset((char *) faV3ChanDisable, 0, sizeof(faV3ChanDisable));
  memset((char *) faV3ID, 0, sizeof(faV3ID));

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
	      ctrl_supported = 0;
	      proc_supported = 0;

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

		  for(icheck = 0; icheck < FAV3_SUPPORTED_CTRL_FIRMWARE_NUMBER;
		      icheck++)
		    {
		      if(ctrl_version == supported_ctrl[icheck])
			ctrl_supported = 1;
		    }

		  if(ctrl_supported == 0)
		    {
		      printf("%s: %s: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __func__, (noFirmwareCheck) ? "WARN" : "ERROR",
			     boardID, ctrl_version);

		      printf("\tSupported Control Firmware:  ");
		      for(icheck = 0;
			  icheck < FAV3_SUPPORTED_CTRL_FIRMWARE_NUMBER;
			  icheck++)
			{
			  printf("0x%02x ", supported_ctrl[icheck]);
			}
		      printf("\n");

		      if(!noFirmwareCheck)
			{	/* Skip to the next fADC */
			  continue;
			}
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version =
		    (uint16_t) (vmeRead32(&fa->adc_status[0]) &
				FAV3_ADC_VERSION_MASK);

		  for(icheck = 0; icheck < FAV3_SUPPORTED_PROC_FIRMWARE_NUMBER;
		      icheck++)
		    {
		      if(proc_version == supported_proc[icheck])
			proc_supported = 1;
		    }

		  if(proc_supported == 0)
		    {
		      printf("%s: %s: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __func__, (noFirmwareCheck) ? "WARN" : "ERROR",
			     boardID, proc_version);

		      printf("\tSupported Proc Firmware:  ");
		      for(icheck = 0;
			  icheck < FAV3_SUPPORTED_PROC_FIRMWARE_NUMBER;
			  icheck++)
			{
			  printf("0x%04x ", supported_proc[icheck]);
			}
		      printf("\n");

		      if(!noFirmwareCheck)
			{	/* Skip to the next fADC */
			  continue;
			}
		    }

		  FAV3p[boardID] = (faV3_t *) (laddr_inc);
		  faV3Rev[boardID] = rdata & FAV3_VERSION_MASK;
		  faV3ProcRev[boardID] = proc_version;
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
	      /*          printf("Initialized FADC %2d  Slot # %2d at address 0x%08x \n", */
	      /*                 ii,faV3ID[ii],(uint32_t) FAV3p[(faV3ID[ii])]); */
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
	  faSDC_Config(0, 0);
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
    }

  /* Enable Clock source - Internal Clk enabled by default */
  if(!noBoardInit)
    {
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->ctrl1),
		     (clkSrc | FAV3_ENABLE_INTERNAL_CLK));
	}
      taskDelay(20);


      /* Hard Reset FPGAs and FIFOs */
      for(ii = 0; ii < nfaV3; ii++)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->reset),
		     (FAV3_RESET_ADC_FPGA1 | FAV3_RESET_ADC_FIFO1 |
		      FAV3_RESET_DAC | FAV3_RESET_EXT_RAM_PT));

#ifdef CLAS12
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->gtx_ctrl), 0x203);	/*put reset */
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->gtx_ctrl), 0x800);	/*release reset */
#else
	  /* #ifdef USEMGTCTRL */
	  /* Release reset on MGTs */
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->mgt_ctrl), FAV3_MGT_RESET);
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->mgt_ctrl), FAV3_RELEASE_MGT_RESET);
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->mgt_ctrl), FAV3_MGT_RESET);
	  /* #endif */
#endif

	}
      taskDelay(5);
    }

  /* Write configuration registers with default/defined Sources */
  for(ii = 0; ii < nfaV3; ii++)
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
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->adr32), (a32addr >> 16) + 1);	/* Write the register and enable */

	  /* Set Default Block Level to 1 */
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->blk_level), 1);
	}
      faV3BlockLevel = 1;

      /* Setup Trigger and Sync Reset sources */
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAV3p[faV3ID[ii]]->ctrl1),
		     vmeRead32(&(FAV3p[faV3ID[ii]]->ctrl1)) |
		     (srSrc | trigSrc));
	}
    }				//End loop through fadcs

  /* If there are more than 1 FADC in the crate then setup the Muliblock Address
     window. This must be the same on each board in the crate */
  if(nfaV3 > 1)
    {
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
  if(errFlag > 0)
    {
#ifdef DEBUG
      printf("faInit: WARN: Unable to initialize all requested FADC Modules (%d)\n",
	     nadc);
#endif
      if(nfaV3 > 0)
	printf("faInit: %d FADC(s) successfully initialized\n", nfaV3);
      return (ERROR);
    }
  else
    {
      return (OK);
    }
}				//End of faInit

void
faSetA32BaseAddress(uint32_t addr)
{
  faV3A32Base = addr;
  printf("fadc A32 base address set to 0x%08X\n", faV3A32Base);
}

/*******************************************************************************
 *
 * faSlot - Convert an index into a slot number, where the index is
 *          the element of an array of FADCs in the order in which they were
 *          initialized.
 *
 * RETURNS: Slot number if Successfull, otherwise ERROR.
 *
 */

int
faSlot(uint32_t i)
{
  if(i >= nfaV3)
    {
      printf("%s: ERROR: Index (%d) >= FADCs initialized (%d).\n",
	     __func__, i, nfaV3);
      return ERROR;
    }

  return faV3ID[i];
}

/*******************************************************************************
 *
 * faSetClockSource - Set the clock source
 *
 *   This routine should be used in the case that the source clock
 *   is NOT set in faInit (and defaults to Internal).  Such is the case
 *   when clocks are synchronized in a many crate system.  The clock source
 *   of the FADC should ONLY be set AFTER those clocks have been set and
 *   synchronized.
 *
 *   clkSrc: 2 bit integer
 *       bits 1-0:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel
 *           1 0  VXS (P0)
 *           1 1  VXS (P0)
 *
 */

int
faSetClockSource(int id, int clkSrc)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

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

int
faGSetClockSource(int clkSrc)
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
      id = faSlot(ifa);
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

void
faStatus(int id, int sflag)
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

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return;
    }

  FAV3LOCK;
  vers = vmeRead32(&FAV3p[id]->version);

  csr = (vmeRead32(&(FAV3p[id]->csr))) & FAV3_CSR_MASK;
  ctrl1 = (vmeRead32(&(FAV3p[id]->ctrl1))) & FAV3_CONTROL_MASK;
  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
  count = (vmeRead32(&(FAV3p[id]->ev_count))) & FAV3_EVENT_COUNT_MASK;
  bcount = (vmeRead32(&(FAV3p[id]->blk_count))) & FAV3_BLOCK_COUNT_MASK;
  blevel = (vmeRead32(&(FAV3p[id]->blk_level))) & FAV3_BLOCK_LEVEL_MASK;
  ramWords = (vmeRead32(&(FAV3p[id]->ram_word_count))) & FAV3_RAM_DATA_MASK;
  trigCnt = vmeRead32(&(FAV3p[id]->trig_scal));
  trig2Cnt = vmeRead32(&FAV3p[id]->trig2_scal);
  srCnt = vmeRead32(&FAV3p[id]->syncreset_scal);
  itrigCnt = vmeRead32(&(FAV3p[id]->internal_trig_scal));
  intr = vmeRead32(&(FAV3p[id]->intr));
  addr32 = vmeRead32(&(FAV3p[id]->adr32));
  a32Base = (addr32 & FAV3_A32_ADDR_MASK) << 16;
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));
  ambMin = (addrMB & FAV3_AMB_MIN_MASK) << 16;
  ambMax = (addrMB & FAV3_AMB_MAX_MASK);
  berr_count = vmeRead32(&(FAV3p[id]->berr_module_scal));

  for(ii = 0; ii < 3; ii++)
    {
      adcStat[ii] = (vmeRead32(&(FAV3p[id]->adc_status[ii])) & 0xFFFF);
      adcConf[ii] = (vmeRead32(&(FAV3p[id]->adc_config[ii])) & 0xFFFF);
    }
  PTW = (vmeRead32(&(FAV3p[id]->adc_ptw)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  PL = (vmeRead32(&(FAV3p[id]->adc_pl)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  NSB = (vmeRead32(&(FAV3p[id]->adc_nsb)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  NSA = (vmeRead32(&(FAV3p[id]->adc_nsa)) & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
  adc_version = adcStat[0] & FAV3_ADC_VERSION_MASK;
  adc_option = (adcConf[0] & FAV3_ADC_PROC_MASK) + 1;
  NP = (adcConf[0] & FAV3_ADC_PEAK_MASK) >> 4;
  adc_enabled = (adcConf[0] & FAV3_ADC_PROC_ENABLE);
  playbackMode = (adcConf[0] & FAV3_ADC_PLAYBACK_MODE) >> 7;
  adcChanDisabled = (adcConf[1] & FAV3_ADC_CHAN_MASK);

#ifdef CLAS12
  mgtStatus = vmeRead32(&(FAV3p[id]->gtx_status));
  mgtCtrl = vmeRead32(&(FAV3p[id]->gtx_ctrl));

  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      val = vmeRead32(&FAV3p[id]->adc_gain[ii]) & 0xFFFF;
      if(val & 0x8000)
	trig_mode[ii] = 1;
      else
	trig_mode[ii] = 0;
      gain_trg[ii] = ((float) (val & 0x7FFF)) / 256.0f;
      delay[ii] = vmeRead16(&FAV3p[id]->adc_delay[ii]) & FAV3_ADC_DELAY_MASK;

      ped_trg[ii] =
	4.0 *
	((float)
	 (vmeRead16(&FAV3p[id]->adc_pedestal[ii]) & FAV3_ADC_PEDESTAL_MASK)) /
	((float) (NSA + NSB));

      val = vmeRead16(&(FAV3p[id]->adc_thres[ii]));
      tet_trg[ii] = (val & FAV3_THR_VALUE_MASK) - (int) ped_trg[ii];
      tet_readout[ii] =
	(val & FAV3_THR_IGNORE_MASK) ? 0 : ((val & FAV3_THR_VALUE_MASK) -
					  (int) ped_trg[ii]);
      inverted[ii] = (val & FAV3_THR_INVERT_MASK) ? 1 : 0;
      playback_ch[ii] = (val & FAV3_PLAYBACK_DIS_MASK) ? 1 : 0;
    }

#else
  mgtStatus = vmeRead32(&(FAV3p[id]->mgt_status));
#endif

  scaler_interval =
    vmeRead32(&FAV3p[id]->scaler_interval) & FAV3_SCALER_INTERVAL_MASK;
  trigger_control = vmeRead32(&FAV3p[id]->trigger_control);

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

#ifdef CLAS12
  printf("  GTX Ctrl   Register      = 0x%08x\n", mgtCtrl);
  printf("  GTX Status Register      = 0x%08x, Errors:", mgtStatus);
  if(mgtCtrl & 0x1)
    printf(" Reset");
  if((mgtStatus & 0x2) == 0 || (mgtStatus & 0x4) == 0)
    printf(" LaneErr");
  if((mgtStatus & 0x1) == 0)
    printf(" ChannelErr");
  printf("\n\n");

  printf("  Ch| Readout - TET | Trigger - TET | GAIN   | PED     | DELAY  | TRGMODE| INVERT | PLAYBACK_DIS |\n");
  printf("  --|---------------|---------------|--------|---------|--------|--------|------- | ------------ |\n");
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(trig_mode[ii])
	trig_mode_string = "DISC";
      else
	trig_mode_string = "PULSE";
      printf("  %2d|          %4d |          %4d |%7.3f |%8.3f |%3d | %s | %5d | %3d\n",
	     ii, tet_readout[ii], tet_trg[ii], gain_trg[ii], ped_trg[ii],
	     delay[ii], trig_mode_string, inverted[ii], playback_ch[ii]);
    }

  printf("\n");
#else
  printf("  MGT Status Register      = 0x%08x ", mgtStatus);
  if(mgtStatus & (FAV3_MGT_GTX1_HARD_ERROR | FAV3_MGT_GTX1_SOFT_ERROR |
		  FAV3_MGT_GTX2_HARD_ERROR | FAV3_MGT_GTX2_SOFT_ERROR))
    printf(" - **Error Condition**\n");
  else
    printf("\n");
#endif



}

void
faGStatus(int sflag)
{
  int ifa, id, ii;
  faV3_t st[FAV3_MAX_BOARDS + 1];
  uint32_t a24addr[FAV3_MAX_BOARDS + 1];
  int nsb;

  FAV3LOCK;
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faSlot(ifa);
      a24addr[id] = (uint32_t) ((u_long) FAV3p[id] - faV3A24Offset);
      st[id].version = vmeRead32(&FAV3p[id]->version);
      st[id].adr32 = vmeRead32(&FAV3p[id]->adr32);
      st[id].adr_mb = vmeRead32(&FAV3p[id]->adr_mb);

      st[id].ctrl1 = vmeRead32(&FAV3p[id]->ctrl1);
      st[id].ctrl2 = vmeRead32(&FAV3p[id]->ctrl2);

      st[id].csr = vmeRead32(&FAV3p[id]->csr);

      st[id].system_monitor = vmeRead32(&FAV3p[id]->system_monitor);

      for(ii = 0; ii < 3; ii++)
	{
	  st[id].adc_status[ii] =
	    vmeRead32(&FAV3p[id]->adc_status[ii]) & 0xFFFF;
	  st[id].adc_config[ii] =
	    vmeRead32(&FAV3p[id]->adc_config[ii]) & 0xFFFF;
	}
      st[id].adc_ptw = vmeRead32(&FAV3p[id]->adc_ptw);
      st[id].adc_pl = vmeRead32(&FAV3p[id]->adc_pl);
      st[id].adc_nsb = vmeRead32(&FAV3p[id]->adc_nsb);
      st[id].adc_nsa = vmeRead32(&FAV3p[id]->adc_nsa);

      st[id].blk_count = vmeRead32(&FAV3p[id]->blk_count);
      st[id].blk_level = vmeRead32(&FAV3p[id]->blk_level);
      st[id].ram_word_count =
	vmeRead32(&FAV3p[id]->ram_word_count) & FAV3_RAM_DATA_MASK;

      st[id].trig_scal = vmeRead32(&(FAV3p[id]->trig_scal));
      st[id].trig2_scal = vmeRead32(&FAV3p[id]->trig2_scal);
      st[id].syncreset_scal = vmeRead32(&FAV3p[id]->syncreset_scal);
      st[id].berr_module_scal = vmeRead32(&FAV3p[id]->berr_module_scal);

      st[id].gtx_ctrl = vmeRead32(&FAV3p[id]->gtx_ctrl);
      st[id].gtx_status = vmeRead32(&FAV3p[id]->gtx_status);
      st[id].sparse_control = vmeRead32(&FAV3p[id]->sparse_control);

      for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
	{
	  st[id].adc_gain[ii] = vmeRead32(&FAV3p[id]->adc_gain[ii]);
	  st[id].adc_delay[ii] = vmeRead16(&FAV3p[id]->adc_delay[ii]);
	  st[id].adc_pedestal[ii] = vmeRead16(&FAV3p[id]->adc_pedestal[ii]);
	  st[id].adc_thres[ii] = vmeRead16(&FAV3p[id]->adc_thres[ii]);
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
      id = faSlot(ifa);
      printf(" %2d  ", id);

      printf("0x%04x 0x%04x  ", st[id].version & 0xFFFF,
	     st[id].adc_status[0] & FAV3_ADC_VERSION_MASK);

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
      id = faSlot(ifa);
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

      printf("0x%04X", ~(st[id].adc_config[1] & FAV3_ADC_CHAN_MASK) & 0xFFFF);

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
      id = faSlot(ifa);
      printf(" %2d    ", id);

      printf("%3d    ", st[id].blk_level & FAV3_BLOCK_LEVEL_MASK);

      printf("%2d   ", (st[id].adc_config[0] & FAV3_ADC_PROC_MASK) + 1);

      printf("%4d  ", (st[id].adc_pl & 0xFFFF) * FAV3_ADC_NS_PER_CLK);

      printf("%4d   ", ((st[id].adc_ptw & 0xFFFF) + 1) * FAV3_ADC_NS_PER_CLK);

      nsb = st[id].adc_nsb & FAV3_ADC_NSB_READBACK_MASK;
      nsb =
	(nsb & 0x7) * ((nsb & FAV3_ADC_NSB_NEGATIVE) ? -1 : 1) *
	FAV3_ADC_NS_PER_CLK;
      printf("%3d  ", nsb);

      printf("%3d   ",
	     (st[id].adc_nsa & FAV3_ADC_NSA_READBACK_MASK) * FAV3_ADC_NS_PER_CLK);

      printf("%1d      ",
	     ((st[id].adc_config[0] & FAV3_ADC_PEAK_MASK) >> 4) + 1);


      printf("%s  ",
	     ((st[id].ctrl2 & FAV3_CTRL_COMPRESS_MASK) ==
	      FAV3_CTRL_COMPRESS_DISABLE) ? "Disabled" : ((st[id].
							 ctrl2 &
							 FAV3_CTRL_COMPRESS_MASK)
							==
							FAV3_CTRL_COMPRESS_ENABLE)
	     ? " Enabled" : ((st[id].ctrl2 & FAV3_CTRL_COMPRESS_MASK) ==
			     FAV3_CTRL_COMPRESS_VERIFY) ? "  Verify" :
	     "UNKNOWN");

      printf("%s ",
	     (st[id].
	      adc_config[0] & FAV3_ADC_PLAYBACK_MODE) >> 7 ? " Enabled" :
	     "Disabled");

      printf("%s",
	     (st[id].
	      sparse_control & FAV3_SPARSE_CONTROL_BYPASS) ? "Bypassed" :
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
      id = faSlot(ifa);
      printf(" %2d   ", id);

      printf("%10d  ", st[id].trig_scal);

      printf("%10d  ", st[id].trig2_scal);

      printf("%10d  ", st[id].syncreset_scal);

      printf("%10d     ", st[id].berr_module_scal);

      double fpga_temperature =
	(((double) (st[id].system_monitor & FAV3_SYSMON_CTRL_TEMP_MASK)) *
	 (503.975 / 1024.0)) - 273.15;
      printf("%3.1f    ", fpga_temperature);

      double fpga_1V =
	(((double)
	  ((st[id].system_monitor & FAV3_SYSMON_FPGA_CORE_V_MASK) >> 11)) *
	 (3.0 / 1024.0));
      printf("%3.1f    ", fpga_1V);

      double fpga_25V =
	(((double)
	  ((st[id].system_monitor & FAV3_SYSMON_FPGA_AUX_V_MASK) >> 22)) *
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
      id = faSlot(ifa);
      printf(" %2d  ", id);

      printf("%s    ",
	     st[id].ctrl2 & FAV3_CTRL_ENABLE_MASK ? " Enabled" : "Disabled");

      printf("%s       ", st[id].csr & FAV3_CSR_BLOCK_READY ? "YES" : " NO");

      printf("%10d ", st[id].blk_count & FAV3_BLOCK_COUNT_MASK);

      printf("%10d  ", (st[id].ram_word_count & FAV3_RAM_DATA_MASK) * 8);

      printf("%s     ", st[id].csr & FAV3_CSR_ERROR_MASK ? "ERROR" : "  OK ");

      printf("%s  ", (st[id].gtx_ctrl & 0x1) ? " ON" : "OFF");

      printf("%s  ", ((st[id].gtx_status & 0x1) != 0x1) ? "Down" : " Up ");

      printf("%s ", ((st[id].gtx_status & 0x6) != 0x6) ? "Down" : " Up ");

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                      fADC250 Trigger Path Processing\n\n");
  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      printf("           .......TET.......                                           \n");
      printf("Slot  Ch   Readout   Trigger      Gain      Ped   Delay  TrigMode  Invert  Accum\n");
      printf("--------------------------------------------------------------------------------\n");

      id = faSlot(ifa);

      int ichan;
      for(ichan = 0; ichan < FAV3_MAX_ADC_CHANNELS; ichan++)
	{
	  if(ichan == 0)
	    printf(" %2d", id);
	  else
	    printf("   ");

	  printf("   ");
	  printf("%2d      ", ichan);

	  int NSB = (st[id].adc_nsb & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
	  int NSA = (st[id].adc_nsa & 0xFFFF) * FAV3_ADC_NS_PER_CLK;
	  float gain_trg = (st[id].adc_gain[ichan] & 0x7FFF) / 256.0f;
	  float ped_trg =
	    4.0 *
	    ((float) (st[id].adc_pedestal[ichan] & FAV3_ADC_PEDESTAL_MASK)) /
	    ((float) (NSA + NSB));

	  int tet_trg =
	    (st[id].adc_thres[ichan] & FAV3_THR_VALUE_MASK) - (int) ped_trg;

	  int tet_readout = (st[id].adc_thres[ichan] & FAV3_THR_IGNORE_MASK) ? 0
	    : ((st[id].adc_thres[ichan] & FAV3_THR_VALUE_MASK) - (int) ped_trg);

	  printf("%4d      ", tet_readout);

	  printf("%4d   ", tet_trg);

	  printf("%7.3f ", gain_trg * 1.);

	  printf("%8.3f     ", ped_trg);

	  printf("%3d     ", st[id].adc_delay[ichan]);

	  printf("%s       ",
		 (st[id].adc_gain[ichan] & 0x8000) ? " DISC" : "PULSE");

	  printf("%d      ",
		 (st[id].adc_thres[ichan] & FAV3_THR_INVERT_MASK) ? 1 : 0);

	  printf("%d",
		 (st[id].
		  adc_thres[ichan] & FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK) ? 1
		 : 0);

	  printf("\n");
	}

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("\n");

}

/***********************
 *
 *  faGetFirmwareVersions - Get the firmware versions of each FPGA
 *
 *    ARG:   pval
 *             0: Print nothing to stdout
 *            !0: Print firmware versions to stdout
 *
 *   RETURNS: (fpga_control.version) | (fpga_processing.version<<16)
 *            or -1 if error
 */
uint32_t
faGetFirmwareVersions(int id, int pflag)
{
  uint32_t cntl = 0, proc = 0, rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n", __func__,
	     id);
      return (ERROR);
    }

  FAV3LOCK;
  /* Control FPGA firmware version */
  cntl = vmeRead32(&FAV3p[id]->version) & 0xFFFF;

  /* Processing FPGA firmware version */
  proc = vmeRead32(&(FAV3p[id]->adc_status[0])) & FAV3_ADC_VERSION_MASK;
  FAV3UNLOCK;

  rval = (cntl) | (proc << 16);

  if(pflag)
    {
      printf("%s:  Board Firmware Rev/ID = 0x%04x : ADC Processing Rev = 0x%04x\n",
	     __func__, cntl, proc);
    }

  return rval;
}

/***********************
 *
 *  faSetProcMode - Setup ADC processing modes.
 *
 *   VERSION2: bank is ignored
 */
int
faSetProcMode(int id, int pmode, uint32_t PL, uint32_t PTW,
	      uint32_t NSB, uint32_t NSA, uint32_t NP, int bank)
{

  int err = 0;
  uint32_t ptw_last_adr, ptw_max_buf;


  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if((pmode <= 0) || (pmode > 8))
    {
      printf("faSetProcMode: ERROR: Processing mode (%d) out of range (pmode= 1-8)\n",
	     pmode);
      return (ERROR);
    }
  else
    {
      /*       if((pmode>3)&&(pmode<8))  */
      /*        { */
      /*          printf("faSetProcMode: ERROR: Processing mode (%d) not implemented \n",pmode); */
      /*        } */
    }

  /*sergey */
  proc_mode[id] = pmode;

  if(NP > 4)
    {
      printf("faSetProcMode: ERROR: Invalid Peak count %d (must be 0-4)\n",
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
      printf("faSetProcMode: ERROR: Window must be <= Latency\n");
    }
  if(((NSB + NSA) % 2) == 0)
    {
      err++;
      printf("faSetProcMode: ERROR: NSB+NSA must be an odd number\n");
    }

  /* Calculate Proc parameters */
  ptw_max_buf = (uint32_t) (2016 / (PTW + 8));
  ptw_last_adr = ptw_max_buf * (PTW + 8) - 1;

  /* Current firmware (version<=0x0208) requires a call to faSetNormalMode
     before enabling the window registers */
  faSetNormalMode(id, 0);

  FAV3LOCK;
  /* Disable ADC processing while writing window info */
  vmeWrite32(&(FAV3p[id]->adc_config[0]), ((pmode - 1) | (NP << 4)));
  vmeWrite32(&(FAV3p[id]->adc_config[1]), faV3ChanDisable[id]);
  vmeWrite32(&(FAV3p[id]->adc_pl), PL);
  vmeWrite32(&(FAV3p[id]->adc_ptw), PTW);
  vmeWrite32(&(FAV3p[id]->adc_nsb), NSB);
  vmeWrite32(&(FAV3p[id]->adc_nsa), NSA);
  vmeWrite32(&(FAV3p[id]->ptw_max_buf), ptw_max_buf);
  vmeWrite32(&(FAV3p[id]->ptw_last_adr), ptw_last_adr);
  /* Enable ADC processing */
  vmeWrite32(&(FAV3p[id]->adc_config[0]),
	     ((pmode - 1) | (NP << 4) | FAV3_ADC_PROC_ENABLE));

  FAV3UNLOCK;

  return (OK);
}

int
faGetNSA(int id)
{
  int ret;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  ret = vmeRead32(&(FAV3p[id]->adc_nsa)) & 0xFFFF;
  FAV3UNLOCK;

  /*  printf("faGetNSA returns %d\n",ret); */

  return (ret);
}

int
faGetNSB(int id)
{
  int ret;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  ret = vmeRead32(&(FAV3p[id]->adc_nsb)) & 0xFFFF;
  FAV3UNLOCK;

  /*  printf("faGetNSB returns %d\n",ret); */

  return (ret);
}


void
faGSetProcMode(int pmode, uint32_t PL, uint32_t PTW,
	       uint32_t NSB, uint32_t NSA, uint32_t NP, int bank)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faSetProcMode(faV3ID[ii], pmode, PL, PTW, NSB, NSA, NP, bank);

      if(res < 0)
	printf("ERROR: slot %d, in faSetProcMode()\n", faV3ID[ii]);
    }

}

/*********************************************************************/
/*********************************************************************/
/* sergey: begin new functions from Bryan Moffit for trigger_control */

/**
 *  @ingroup Config
 *  @brief Return the maximum number of unacknowledged triggers a specific
 *         mode can handle.
 *
 *  @param pmode  Processing Mode
 *  @param ptw  Window Width
 *  @param nsb  Number of samples before pulse over threshold
 *  @param nsa  Number of samples after pulse over threshold
 *  @param np   Number of pulses processed per window
 *
 *  @return The minimum of 9 and the calculated maximum number of triggers
 *    allowed given specified mode and window paramters.
 */

int
faCalcMaxUnAckTriggers(int mode, int ptw, int nsa, int nsb, int np)
{
  int max;
  int imode = 0, supported_modes[FAV3_SUPPORTED_NMODES] =
    { FAV3_SUPPORTED_MODES };
  int mode_supported = 0;

  for(imode = 0; imode < FAV3_SUPPORTED_NMODES; imode++)
    {
      if(mode == supported_modes[imode])
	mode_supported = 1;
    }
  if(!mode_supported)
    {
      printf("%s: ERROR: Processing Mode (%d) not supported\n",
	     __func__, mode);
      return ERROR;
    }

  switch (mode)
    {
    case 9:			/* PULSE PARAMETER */
      max = (int) (1024 / ((np * 2) + 8));
      break;

    case 10:			/* DEBUG */
      max = (int) (1024 / (((np * 2) + 8) + ptw + 1));
      break;

    default:
      printf("%s: ERROR: Mode %d is not supported\n", __func__, mode);
    }

  return ((max < 9) ? max : 9);
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
faSetTriggerStopCondition(int id, int trigger_max)
{
  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

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
faSetTriggerBusyCondition(int id, int trigger_max)
{
  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

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
 *  @param NSB Number of samples before threshold crossing
 *  @param NSA Number of samples after threshold crossing
 *  @return OK if successful, otherwise ERROR.
 */
int
faSetTriggerPathSamples(int id, uint32_t TNSA, uint32_t TNSAT)
{
  uint32_t readback_nsa = 0, readback_config1 = 0;

  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  if(faV3ProcRev[id] < 0x90B)
    {
      printf("%s: ERROR: Processing Firmware does not support this function\n",
	     __func__);
      printf("      Requires 0x90B and above\n");
      return ERROR;
    }

  if((TNSA < FAV3_ADC_MIN_TNSA) || (TNSA > FAV3_ADC_MAX_TNSA))
    {
      printf("%s: WARN: TNSA (%d) out of range. Setting to %d\n",
	     __func__, TNSA, FAV3_ADC_DEFAULT_TNSA);
      TNSA = FAV3_ADC_DEFAULT_TNSA;
    }

  if((TNSAT < FAV3_ADC_MIN_TNSAT) || (TNSAT > FAV3_ADC_MAX_TNSAT))
    {
      printf("%s: WARN: TNSAT (%d) out of range. Setting to %d\n",
	     __func__, TNSAT, FAV3_ADC_DEFAULT_TNSAT);
      TNSAT = FAV3_ADC_DEFAULT_TNSAT;
    }

  FAV3LOCK;

  readback_nsa = vmeRead32(&FAV3p[id]->adc_nsa) & FAV3_ADC_NSA_READBACK_MASK;
  readback_config1 =
    vmeRead32(&FAV3p[id]->adc_config[0]) & ~FAV3_ADC_CONFIG1_TNSAT_MASK;

  vmeWrite32(&FAV3p[id]->adc_nsa, (TNSA << 9) | readback_nsa);
  vmeWrite32(&FAV3p[id]->adc_config[0], ((TNSAT - 1) << 12) | readback_config1);

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
 *  @sa faSetTriggerPathSamples
 */
void
faGSetTriggerPathSamples(uint32_t TNSA, uint32_t TNSAT)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faSetTriggerPathSamples(faV3ID[ii], TNSA, TNSAT);
      if(res < 0)
	printf("ERROR: slot %d, in faSetTriggerPathSamples()\n", faV3ID[ii]);
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
faSetTriggerPathThreshold(int id, uint32_t TPT)
{
  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : FADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  if(faV3ProcRev[id] < 0x90B)
    {
      printf("%s: ERROR: Processing Firmware does not support this function\n",
	     __func__);
      printf("      Requires 0x90B and above\n");
      return ERROR;
    }

  if(TPT > FAV3_ADC_MAX_TPT)
    {
      printf("%s: WARN: TPT (%d) greater than MAX.  Setting to %d\n",
	     __func__, TPT, FAV3_ADC_MAX_TPT);
      TPT = FAV3_ADC_MAX_TPT;
    }

#if 0
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->config3,
	     (vmeRead32(&FAV3p[id]->config3) & ~FAV3_ADC_CONFIG3_TPT_MASK) | TPT);
  FAV3UNLOCK;
#endif

  return OK;
}

/**
 *  @ingroup Config
 *  @brief Set the threshold used to determine what samples are sent through the
 *     trigger path for all initialized fADC250s
 *  @param threshold Trigger Path Threshold
 *  @sa faSetTriggerPathThreshold
 */
void
faGSetTriggerPathThreshold(uint32_t TPT)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faSetTriggerPathThreshold(faV3ID[ii], TPT);
      if(res < 0)
	printf("ERROR: slot %d, in faSetTriggerPathThreshold()\n",
	       faV3ID[ii]);
    }
}

/* sergey: end new functions from Bryan Moffit for trigger_control */
/*******************************************************************/
/*******************************************************************/





/*
 * faWaitForAdcReady()
 *   - Static routine, to wait for the ADC processing chip ready bit
 *     before proceeding with further programming
 *
 */
static void
faWaitForAdcReady(int id)
{
  int iwait = 0;

  while((iwait < 100) && (vmeRead32(&FAV3p[id]->adc_status[0]) & 0x8000) == 0)
    {
      iwait++;
    }

  if(iwait == 100)
    printf("%s: ERROR: Wait timeout.\n", __func__);

}

/* faSetNormalMode
 *    - Configure the ADC Processing in "Normal Mode"
 *      This is temporary until the firmware is confirmed to be stable
 *
 */
void
faSetNormalMode(int id, int opt)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetProcMode: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[3], 0x0F02);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);

  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[3], 0x179F);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);

  /* 01dec2011 This portion commented out... would change the input gain */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAV3p[id]->adc_config[3], 0x1811); */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAV3p[id]->adc_config[2], 0x40); */
  /*   faWaitForAdcReady(id); */
  /*   vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);        */

  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[3], 0xFF01);	/* transfer register values */
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);
  /*
    printf("%s: ---- FADC %2d ADC chips initialized ----\n",
    __func__,id);
  */
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[3], 0x0D00);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);

  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[3], 0xFF01);	/* transfer register values */
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0x40);
  faWaitForAdcReady(id);
  vmeWrite32(&FAV3p[id]->adc_config[2], 0xC0);

  FAV3UNLOCK;


}

/***********************
 *
 *  faSetPPG - Setup FADC Progammable Pulse Generator
 *
 *
 */
int
faSetPPG(int id, int pmode, uint16_t * sdata, int nsamples)
{

  int ii;
  uint16_t rval;


  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetPPG: ERROR : FADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (ERROR);
    }

  if(sdata == NULL)
    {
      printf("faSetPPG: ERROR: Invalid Pointer to sample data\n");
      return (ERROR);
    }

  /*Defaults */
  if((nsamples <= 0) || (nsamples > FAV3_PPG_MAX_SAMPLES))
    nsamples = FAV3_PPG_MAX_SAMPLES;

  FAV3LOCK;
  for(ii = 0; ii < (nsamples - 2); ii++)
    {
      vmeWrite32(&FAV3p[id]->adc_test_data, (sdata[ii] | FAV3_PPG_WRITE_VALUE));
      rval = vmeRead32(&FAV3p[id]->adc_test_data);
      if((rval & FAV3_PPG_SAMPLE_MASK) != sdata[ii])
	printf("faSetPPG: ERROR: Write error %x != %x (ii=%d)\n", rval,
	       sdata[ii], ii);

    }

  vmeWrite32(&FAV3p[id]->adc_test_data,
	     (sdata[(nsamples - 2)] & FAV3_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAV3p[id]->adc_test_data);
  if(rval != sdata[(nsamples - 2)])
    printf("faSetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples - 2]);
  vmeWrite32(&FAV3p[id]->adc_test_data,
	     (sdata[(nsamples - 1)] & FAV3_PPG_SAMPLE_MASK));
  rval = vmeRead32(&FAV3p[id]->adc_test_data);
  if(rval != sdata[(nsamples - 1)])
    printf("faSetPPG: ERROR: Write error %x != %x\n",
	   rval, sdata[nsamples - 1]);

  /*   vmeWrite32(&FAV3p[id]->adc_test_data, (sdata[(nsamples-2)]&FAV3_PPG_SAMPLE_MASK)); */
  /*   vmeWrite32(&FAV3p[id]->adc_test_data, (sdata[(nsamples-1)]&FAV3_PPG_SAMPLE_MASK)); */

  FAV3UNLOCK;

  return (OK);
}

void
faPPGEnable(int id)
{
  uint16_t val1;

  if(id == 0)
    id = faV3ID[0];

  FAV3LOCK;
  val1 = (vmeRead32(&FAV3p[id]->adc_config[0]) & 0xFFFF);
  val1 |= (FAV3_PPG_ENABLE | 0xff00);
  vmeWrite32(&FAV3p[id]->adc_config[0], val1);
  FAV3UNLOCK;

}

void
faPPGDisable(int id)
{
  uint16_t val1;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPPGDisable: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  val1 = (vmeRead32(&FAV3p[id]->adc_config[0]) & 0xFFFF);
  val1 &= ~FAV3_PPG_ENABLE;
  val1 &= ~(0xff00);
  vmeWrite32(&FAV3p[id]->adc_config[0], val1);
  FAV3UNLOCK;

}


#ifdef VERSION1
/*************************************************************************************
 *
 *  faItrigBurstConfig - Setup Internal Trigger Burst control Parameters
 *
 *   ntrig        = max triggers (1-128) allowed in Burst Window
 *   burst_window = size (in clock ticks 4ns/tick) of Burst window (1 - 4 microsec)
 *   busy_period  = size (in clocks) of busy period to wait after max triggers reached
 *                   (0 - 262 microsec)
 */
int
faItrigBurstConfig(int id, uint32_t ntrig,
		   uint32_t burst_window, uint32_t busy_period)
{


  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faItrigBurstConfig: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  /* Set Defaults */
  if((ntrig == 0) || (ntrig > 128))
    ntrig = 4;
  if((burst_window < 0x100) || (burst_window > 0x3ff))
    burst_window = 0x200;
  if((busy_period == 0) || (busy_period > 0xffff))
    busy_period = 0x800;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->itrig_burst_count), ntrig);
  vmeWrite32(&(FAV3p[id]->itrig_burst_ctrl),
	     ((busy_period) << 16) | burst_window);
  FAV3UNLOCK;

  return (OK);
}
#endif

/*
 * Set Internal trigger pulse width and deadtime between triggers
 *   Range for each :   4ns <-> 1020ns
 *
 *    Units are in clock ticks (4ns/tick)
 */
uint32_t
faItrigControl(int id, uint16_t itrig_width, uint16_t itrig_dt)
{
  uint32_t retval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faItrigControl: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (0xffffffff);
    }

  /* If both parameters = 0 then just return the current value */
  FAV3LOCK;
  if((itrig_width == 0) && (itrig_dt == 0))
    {
      retval = vmeRead32(&(FAV3p[id]->itrig_cfg));
    }
  else
    {
      if((itrig_width == 0) || (itrig_width > 255))
	itrig_width = 0xc;	/* default 48ns */
      if((itrig_dt == 0) || (itrig_dt > 255))
	itrig_dt = 0xa;		/* default 40ns */

      vmeWrite32(&(FAV3p[id]->itrig_cfg), (itrig_width << 16) | itrig_dt);
      retval = vmeRead32(&(FAV3p[id]->itrig_cfg));
    }
  FAV3UNLOCK;

  return (retval);
}



/**************************************************************************************
 *
 *  faReadBlock - General Data readout routine
 *
 *    id    - Slot number of module to read
 *    data  - local memory address to place data
 *    nwrds - Max number of words to transfer
 *    rflag - Readout Flag
 *              0 - programmed I/O from the specified board
 *              1 - DMA transfer using Universe/Tempe DMA Engine
 *                    (DMA VME transfer Mode must be setup prior)
 *              2 - Multiblock DMA transfer (Multiblock must be enabled
 *                     and daisychain in place or SD being used)
 */
int
faReadBlock(int id, volatile uint32_t * data, int nwrds, int rflag)
{
  int ii;
  int stat, retVal, xferCount, rmode, async;
  int dCnt, berr = 0;
  int dummy = 0;
  volatile uint32_t *laddr;
  uint32_t bhead, ehead, val;
  uint32_t vmeAdr, csr;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faReadBlock: ERROR : FADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(data == NULL)
    {
      logMsg("faReadBlock: ERROR: Invalid Destination address\n", 0, 0, 0, 0,
	     0, 0);
      return (ERROR);
    }

  faV3BlockError = FAV3_BLOCKERROR_NO_ERROR;
  if(nwrds <= 0)
    nwrds = (FAV3_MAX_ADC_CHANNELS * FAV3_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag & 0x0f;
  async = rflag & 0x80;

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

      FAV3LOCK;
      if(rmode == 2)
	{			/* Multiblock Mode */
	  if((vmeRead32(&(FAV3p[id]->ctrl1)) & FAV3_FIRST_BOARD) == 0)
	    {
	      logMsg
		("faReadBlock: ERROR: FADC in slot %d is not First Board\n",
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
	  logMsg("faReadBlock: ERROR in DMA transfer Initialization 0x%x\n",
		 retVal, 0, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (retVal);
	}

      if(async)
	{			/* Asynchronous mode - return immediately - don't wait for done!! */
	  FAV3UNLOCK;
	  return (OK);
	}
      else
	{
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
	}

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
		faGetTokenStatus(1);

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
	    faGetTokenStatus(1);

	  return (nwrds);
	}
      else
	{			/* Error in DMA */
#if defined(VXWORKS) && !defined(HALLB)
	  logMsg("faReadBlock: ERROR: sysVmeDmaDone returned an Error\n", 0,
		 0, 0, 0, 0, 0);
#else
	  logMsg("faReadBlock: ERROR: vmeDmaDone returned an Error\n", 0, 0,
		 0, 0, 0, 0);
#endif
	  faV3BlockError = FAV3_BLOCKERROR_DMADONE_ERROR;
	  FAV3UNLOCK;

	  if(rmode == 2)
	    faGetTokenStatus(1);

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
	      logMsg("faReadBlock: FIFO Empty (0x%08x)\n", bhead, 0, 0, 0, 0,
		     0);
	      FAV3UNLOCK;
	      return (0);
	    }
	  else
	    {
	      logMsg("faReadBlock: ERROR: Invalid Header Word 0x%08x\n",
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
 *    block read from faReadBlock.
 *  @param pflag
 *     - >0: Print error message to standard out
 *  @sa faReadBlock
 *  @return OK if successful, otherwise ERROR.
 */
int
faGetBlockError(int pflag)
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
	  logMsg("faGetBlockError: Block Transfer Error: %s\n",
		 block_error_names[rval], 2, 3, 4, 5, 6);
	}
    }

  return rval;
}

int
faReadBlockStatus(int id, volatile uint32_t * data, int nwrds, int rflag)
{

  int stat, retVal, xferCount, rmode;
  int dummy = 0;
  uint32_t csr = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faReadBlockStatus: ERROR : FADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(nwrds <= 0)
    nwrds = (FAV3_MAX_ADC_CHANNELS * FAV3_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag & 0x0f;

  /* Check for 8 byte boundary for address - insert dummy word (Slot 0 FADC Dummy DATA) */
  if((u_long) (data) & 0x7)
    {
      dummy = 1;
    }
  else
    {
      dummy = 0;
    }

#ifdef HALLB
  retVal = usrVme2MemDmaDone();
#else
#ifdef VXWORKS
  retVal = sysVmeDmaDone(10000, 1);
#else
  retVal = vmeDmaDone();
#endif
#endif

  FAV3LOCK;
  if(retVal > 0)
    {
      /* Check to see that Bus error was generated by FADC */
      if(rmode == 2)
	{
	  csr = vmeRead32(&(FAV3p[faV3MaxSlot]->csr));	/* from Last FADC */
	  stat = (csr) & FAV3_CSR_BERR_STATUS;	/* from Last FADC */
	}
      else
	{
	  stat = vmeRead32(&(FAV3p[id]->csr)) & FAV3_CSR_BERR_STATUS;	/* from FADC id */
	}

      if((retVal > 0) && (stat))
	{
#ifdef HALLB
	  xferCount = ((retVal >> 2) + dummy);	/* Number of Longwords transfered */
#else
	  xferCount = (nwrds - (retVal >> 2) + dummy);	/* Number of Longwords transfered */
#endif
	  FAV3UNLOCK;
	  return (xferCount);	/* Return number of data words transfered */
	}
      else
	{
#ifdef HALLB
	  xferCount = ((retVal >> 2) + dummy);	/* Number of Longwords transfered */
#else
	  xferCount = (nwrds - (retVal >> 2) + dummy);	/* Number of Longwords transfered */
#endif
	  logMsg
	    ("faReadBlockStatus: DMA transfer terminated by unknown BUS Error (csr=0x%x nwrds=%d)\n",
	     csr, xferCount, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (ERROR);
	}
    }
  else if(retVal == 0)
    {				/* Block Error finished without Bus Error */
      logMsg
	("faReadBlockStatus: WARN: DMA transfer terminated by word count 0x%x\n",
	 nwrds, 0, 0, 0, 0, 0);
      FAV3UNLOCK;
      return (nwrds);
    }
  else
    {				/* Error in DMA */
      logMsg("faReadBlockStatus: ERROR: sysVmeDmaDone returned an Error\n", 0,
	     0, 0, 0, 0, 0);
      FAV3UNLOCK;
      return (retVal);
    }

}

int
faPrintBlock(int id, int rflag)
{

  int ii;
  int nwrds = 32768, dCnt, berr = 0;
  uint32_t data, bhead, ehead;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faPrintEvent: ERROR : FADC in slot %d is not initialized \n",
	     id);
      return (ERROR);
    }

  /* Check if data available */
  FAV3LOCK;
  if((vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK) == 0)
    {
      printf("faPrintEvent: ERROR: FIFO Empty\n");
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
      faDataDecode(bhead);
      dCnt++;
      printf("%4d: ", dCnt + 1);
      faDataDecode(ehead);
      dCnt++;
    }
  else
    {
      /* We got bad data - Check if there is any data at all */
      if((vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK) == 0)
	{
	  logMsg("faPrintBlock: FIFO Empty (0x%08x)\n", bhead, 0, 0, 0, 0, 0);
	  FAV3UNLOCK;
	  return (0);
	}
      else
	{
	  logMsg("faPrintBlock: ERROR: Invalid Header Word 0x%08x\n", bhead,
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
      faDataDecode(data);
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

/*****************************************************************************/

uint32_t
faReadCSR(int id)
{
  uint32_t rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faReadCSR: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  rval = vmeRead32(&(FAV3p[id]->csr));
  FAV3UNLOCK;

  return (rval);
}


void
faClear(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faClear: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_RESET);
  FAV3UNLOCK;
}


void
faGClear()
{

  int ii, id;

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];
      if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
	{
	  logMsg("faGClear: ERROR : ADC in slot %d is not initialized \n", id,
		 0, 0, 0, 0, 0);
	}
      else
	{
	  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_RESET);
	}
    }
  FAV3UNLOCK;

}

void
faClearError(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faClearErr: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_ERROR_CLEAR);
  FAV3UNLOCK;

}


void
faGClearError()
{

  int ii, id;

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];
      if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
	{
	  logMsg("faGClearErr: ERROR : ADC in slot %d is not initialized \n",
		 id, 0, 0, 0, 0, 0);
	}
      else
	{
	  vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_ERROR_CLEAR);
	}
    }
  FAV3UNLOCK;

}


void
faReset(int id, int iFlag)
{
  uint32_t a32addr, addrMB;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faReset: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

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

}

/**
 *  @ingroup Config
 *  @brief Perform a hard reset on all initialized fADC250s
 *  @param iFlag Decision to restore A32 readout after reset.
 *     -  0: Restore A32 readout after reset.
 *     - !0: Do not restore A32 readout after reset. (Useful for configuration changes)
 */
void
faGReset(int iFlag)
{
  uint32_t a32addr[(FAV3_MAX_BOARDS + 1)], addrMB[(FAV3_MAX_BOARDS + 1)];
  int ifa = 0, id = 0;

  FAV3LOCK;
  if(iFlag == 0)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  id = faSlot(ifa);
	  a32addr[id] = vmeRead32(&(FAV3p[id]->adr32));
	  addrMB[id] = vmeRead32(&(FAV3p[id]->adr_mb));
	}
    }

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      id = faSlot(ifa);
      vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_HARD_RESET);
    }

  taskDelay(10);

  if(iFlag == 0)
    {
      for(ifa = 0; ifa < nfaV3; ifa++)
	{
	  id = faSlot(ifa);
	  vmeWrite32(&(FAV3p[id]->adr32), a32addr[id]);
	  vmeWrite32(&(FAV3p[id]->adr_mb), addrMB[id]);
	}
    }

  FAV3UNLOCK;

}

void
faSoftReset(int id, int cflag)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faReset: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  if(cflag)			/* perform soft clear */
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_CLEAR);
  else				/* normal soft reset */
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_RESET);
  FAV3UNLOCK;

}

void
faResetToken(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faResetToken: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->reset), FAV3_RESET_TOKEN);
  FAV3UNLOCK;
}

int
faTokenStatus(int id)
{
  int rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faResetToken: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  rval = (vmeRead32(&FAV3p[id]->csr) & FAV3_CSR_TOKEN_STATUS) >> 4;
  FAV3UNLOCK;

  return rval;
}

int
faGTokenStatus()
{
  int ifa = 0, bit = 0, rval = 0;

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      bit = faTokenStatus(faSlot(ifa));
      rval |= (bit << (faSlot(ifa)));
    }

  return rval;
}

uint32_t
faGetTokenStatus(int pflag)
{
  uint32_t rval = 0;
  int ifa = 0;

  if(pflag)
    logMsg("faGetTokenStatus: Token in slot(s) ", 1, 2, 3, 4, 5, 6);

  rval = faGTokenStatus();

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

void
faSetCalib(int id, uint16_t sdelay, uint16_t tdelay)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetCalib: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->delay), (sdelay << 16) | tdelay);
  FAV3UNLOCK;

}

void
faChanDisable(int id, uint16_t cmask)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faChanDisable: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  faV3ChanDisable[id] = cmask;	/* Set Global Variable */

  FAV3LOCK;
  /* Write New Disable Mask */
  vmeWrite32(&(FAV3p[id]->adc_config[1]), cmask);
  FAV3UNLOCK;

}


uint32_t
faGetChanMask(int id)
{
  uint32_t tmp, cmask = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetChanMask: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (0);
    }


  FAV3LOCK;
  tmp = vmeRead32(&(FAV3p[id]->adc_config[1])) & 0xFFFF;
  cmask = (tmp & FAV3_ADC_CHAN_MASK);
  faV3ChanDisable[id] = cmask;	/* Set Global Variable */
  FAV3UNLOCK;


  return (cmask);
}


/* opt=0 - disable, 1-enable, 2-verify */
void
faSetCompression(int id, int opt)
{
  uint32_t ctrl2;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetCompression: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;

  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: read ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & ~FAV3_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: masked ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  if(opt == 0)
    {
      ;
    }
  else if(opt == 1)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_ENABLE;
      printf("faSetCompression: setting mode 1 ctrl2=0x%08x\n", ctrl2);
    }
  else if(opt == 2)
    {
      ctrl2 = ctrl2 | FAV3_CTRL_COMPRESS_VERIFY;
      printf("faSetCompression: setting mode 2 ctrl2=0x%08x\n", ctrl2);
    }
  else
    printf("faSetCompression: illegal opt=%d\n", opt);

#ifdef DEBUG_COMPRESSION
  printf("faSetCompression: writing ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */
  vmeWrite32(&(FAV3p[id]->ctrl2), ctrl2);

  FAV3UNLOCK;
}


int
faGetCompression(int id)
{
  uint32_t ctrl2;
  int opt;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetCompression: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;

  ctrl2 = (vmeRead32(&(FAV3p[id]->ctrl2))) & FAV3_CONTROL2_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faGetCompression: read ctrl2=0x%08x\n", ctrl2);
#endif /* DEBUG_COMPRESSION */

  ctrl2 = ctrl2 & FAV3_CTRL_COMPRESS_MASK;
#ifdef DEBUG_COMPRESSION
  printf("faGetCompression: masked ctrl2=0x%08x\n", ctrl2);
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
void
faSetVXSReadout(int id, int opt)
{
  uint32_t ctrl2;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetVXSReadout: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

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
}

void
faGSetVXSReadout(int opt)
{
  int ifa = 0;

  for(ifa = 0; ifa < nfaV3; ifa++)
    {
      faSetVXSReadout(faV3ID[ifa], opt);
    }

}


int
faGetVXSReadout(int id)
{
  uint32_t ctrl2;
  int opt;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetVXSReadout: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;

  ctrl2 = vmeRead32(&FAV3p[id]->ctrl2) & FAV3_CTRL_VXS_RO_ENABLE;

  if(ctrl2)
    opt = 1;
  else
    opt = 0;

  FAV3UNLOCK;

  return (opt);
}


void
faEnableSyncReset(int id)
{
  uint32_t ctrl2;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnable: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  ctrl2 = vmeRead32(&FAV3p[id]->ctrl2) | FAV3_CTRL_ENABLE_SRESET;
  vmeWrite32(&FAV3p[id]->ctrl2, ctrl2);
  FAV3UNLOCK;
}

void
faEnable(int id, int eflag, int bank)
{
  uint32_t ctrl2;
  int compress_opt, vxsro_opt;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnable: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  /* call it BEFORE 'FAV3LOCK' !!! */
  compress_opt = faGetCompression(id);
  vxsro_opt = faGetVXSReadout(id);

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
}

void
faGEnable(int eflag, int bank)
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faEnable(faV3ID[ii], eflag, bank);

  if(faV3UseSDC && !faV3SDCPassthrough)
    faSDC_Enable(1);

}

void
faDisable(int id, int eflag)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faDisable: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  if(eflag)
    vmeWrite32(&(FAV3p[id]->ctrl2), 0);	/* Turn FIFO Transfer off as well */
  else
    vmeWrite32(&(FAV3p[id]->ctrl2), (FAV3_CTRL_GO | FAV3_CTRL_ENABLE_SRESET));	/* Keep SYNC RESET detection enabled */
  FAV3UNLOCK;
}

void
faGDisable(int eflag)
{
  int ii;

  if(faV3UseSDC && !faV3SDCPassthrough)
    faSDC_Disable();

  for(ii = 0; ii < nfaV3; ii++)
    faDisable(faV3ID[ii], eflag);

}


void
faTrig(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faTrig: ERROR : ADC in slot %d is not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }

  FAV3LOCK;
  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_TRIG))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_TRIGGER);
  else
    logMsg("faTrig: ERROR: Software Triggers not enabled", 0, 0, 0, 0, 0, 0);
  FAV3UNLOCK;
}

void
faGTrig()
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faTrig(faV3ID[ii]);
}

void
faTrig2(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faTrig2: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }
  FAV3LOCK;
  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_TRIG))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SOFT_PULSE_TRIG2);
  else
    logMsg("faTrig2: ERROR: Software Triggers not enabled", 0, 0, 0, 0, 0, 0);
  FAV3UNLOCK;
}

void
faGTrig2()
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faTrig2(faV3ID[ii]);
}

int
faSetTrig21Delay(int id, int delay)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  if(delay > FAV3_TRIG21_DELAY_MASK)
    {
      printf("%s: ERROR: Invalid value for delay (%d).\n", __func__, delay);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig21_delay, delay);
  FAV3UNLOCK;

  return OK;
}

int
faGetTrig21Delay(int id)
{
  int rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->trig21_delay) & FAV3_TRIG21_DELAY_MASK;
  FAV3UNLOCK;

  return rval;
}


int
faEnableInternalPlaybackTrigger(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->ctrl1,
	     (vmeRead32(&FAV3p[id]->ctrl1) & ~FAV3_TRIG_MASK) |
	     FAV3_TRIG_VME_PLAYBACK);
  FAV3UNLOCK;

  return OK;
}

void
faSync(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSync: ERROR : ADC in slot %d is not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }

  FAV3LOCK;
  if(vmeRead32(&(FAV3p[id]->ctrl1)) & (FAV3_ENABLE_SOFT_SRESET))
    vmeWrite32(&(FAV3p[id]->csr), FAV3_CSR_SYNC);
  else
    logMsg("faSync: ERROR: Software Sync Resets not enabled\n", 0, 0, 0, 0, 0,
	   0);
  FAV3UNLOCK;
}



/* Return Event/Block count for ADC in slot id */
int
faDready(int id, int dflag)
{
  uint32_t dcnt = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faDready: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  if(dflag)
    dcnt = vmeRead32(&(FAV3p[id]->blk_count)) & FAV3_BLOCK_COUNT_MASK;
  else
    dcnt = vmeRead32(&(FAV3p[id]->ev_count)) & FAV3_EVENT_COUNT_MASK;
  FAV3UNLOCK;


  return (dcnt);
}

/* Return a Block Ready status for ADC. If Block Level is =1 then return Event Ready status */
int
faBready(int id)
{
  int stat = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faBready: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  stat = (vmeRead32(&(FAV3p[id]->csr))) & FAV3_CSR_BLOCK_READY;
  FAV3UNLOCK;

  if(stat)
    return (1);
  else
    return (0);
}

uint32_t
faGBready()
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

uint32_t
faGBlockReady(uint32_t slotmask, int nloop)
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

/* return Scan mask for all initialized FADCs */
uint32_t
faScanMask()
{
  int ifadc, id, dmask = 0;

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      dmask |= (1 << id);
    }

  return (dmask);
}


/* if val>0 then set the busy level, if val=0 then read it back.
   if bflag>0 then force the module Busy */
int
faBusyLevel(int id, uint32_t val, int bflag)
{
  uint32_t blreg = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faBusyLevel: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return (ERROR);
    }
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

int
faBusy(int id)
{
  uint32_t blreg = 0;
  uint32_t dreg = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faBusy: ERROR : ADC in slot %d is not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  blreg = vmeRead32(&(FAV3p[id]->busy_level)) & FAV3_BUSY_LEVEL_MASK;
  dreg = vmeRead32(&(FAV3p[id]->ram_word_count)) & FAV3_RAM_DATA_MASK;
  FAV3UNLOCK;

  if(dreg >= blreg)
    return (1);
  else
    return (0);
}


void
faEnableSoftTrig(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableSoftTrig: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  /* Clear the source */
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_TRIG_MASK);
  /* Set Source and Enable */
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_TRIG_VME |
					     FAV3_ENABLE_SOFT_TRIG));
  FAV3UNLOCK;
}


void
faGEnableSoftTrig()
{
  int ii, id;

  for(ii = 0; ii < nfaV3; ii++)
    {
      id = faV3ID[ii];
      faEnableSoftTrig(id);
    }

}


void
faDisableSoftTrig(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faDisableSoftTrig: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_SOFT_TRIG);
  FAV3UNLOCK;

}

void
faEnableSoftSync(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableSoftSync: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  /* Clear the source */
  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_SRESET_MASK);
  /* Set Source and Enable */
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_SRESET_VME |
					     FAV3_ENABLE_SOFT_SRESET));
  FAV3UNLOCK;
}

void
faDisableSoftSync(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faDisableSoftSync: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_SOFT_SRESET);
  FAV3UNLOCK;

}

void
faEnableClk(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableClk: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_REF_CLK_INTERNAL |
					     FAV3_ENABLE_INTERNAL_CLK));
  FAV3UNLOCK;

}

void
faDisableClk(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faDisableClk: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_INTERNAL_CLK);
  FAV3UNLOCK;

}

/*************************************************************************************
 *
 *  faEnableTriggerOut - Enable trigger out for front panel or p0
 *
 *   output = 0 for FP trigger out
 *            1 for P0 trigger out
 *            2 for FP and P0 trigger out
 */

void
faEnableTriggerOut(int id, int output)
{
  int bitset = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableBusError: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  if(output > 2)
    {
      logMsg
	("faEnableTriggerOut: ERROR: output (%d) out of range.  Must be less than 3",
	 output, 2, 3, 4, 5, 6);
      return;

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

}

void
faEnableBusError(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableBusError: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | FAV3_ENABLE_BERR);
  FAV3UNLOCK;

}


void
faGEnableBusError()
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


void
faDisableBusError(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faDisableBusError: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_ENABLE_BERR);
  FAV3UNLOCK;

}


void
faEnableMultiBlock(int tflag)
{
  int ii, id;
  uint32_t mode;

  if((nfaV3 <= 1) || (FAV3p[faV3ID[0]] == NULL))
    {
      logMsg("faEnableMultiBlock: ERROR : Cannot Enable MultiBlock mode \n",
	     0, 0, 0, 0, 0, 0);
      return;
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
      faDisableBusError(id);
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
	  faEnableBusError(id);	/* Enable Bus Error only on Last Board */
	}
    }

}

void
faDisableMultiBlock()
{
  int ii;

  if((nfaV3 <= 1) || (FAV3p[faV3ID[0]] == NULL))
    {
      logMsg("faDisableMultiBlock: ERROR : Cannot Disable MultiBlock Mode\n",
	     0, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    vmeWrite32(&(FAV3p[faV3ID[ii]]->ctrl1),
	       vmeRead32(&(FAV3p[faV3ID[ii]]->ctrl1)) & ~FAV3_ENABLE_MULTIBLOCK);
  FAV3UNLOCK;

}



int
faSetBlockLevel(int id, int level)
{
  int rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetBlockLevel: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(level <= 0)
    level = 1;

  logMsg("faSetBlockLevel: INFO: Set ADC slot %d block level to %d \n", id,
	 level, 0, 0, 0, 0);

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->blk_level), level);
  faV3BlockLevel = level;
  rval = vmeRead32(&(FAV3p[id]->blk_level)) & FAV3_BLOCK_LEVEL_MASK;
  FAV3UNLOCK;

  return (rval);

}

void
faGSetBlockLevel(int level)
{
  int ii;

  if(level <= 0)
    level = 1;
  FAV3LOCK;
  for(ii = 0; ii < nfaV3; ii++)
    vmeWrite32(&(FAV3p[faV3ID[ii]]->blk_level), level);
  FAV3UNLOCK;

  faV3BlockLevel = level;
}


int
faSetClkSource(int id, int source)
{
  int rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetClkSource: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

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

int
faSetTrigSource(int id, int source)
{
  int rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetTrigSource: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

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

int
faSetSyncSource(int id, int source)
{
  int rval;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetSyncSource: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

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

/* Enable Front Panel Inputs (and Disable software triggers/syncs
   but leave the clock source alone */
void
faEnableFP(int id)
{

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableFP: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) &
	     ~(FAV3_TRIG_SEL_MASK | FAV3_SRESET_SEL_MASK | FAV3_ENABLE_SOFT_SRESET |
	       FAV3_ENABLE_SOFT_TRIG));
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     vmeRead32(&(FAV3p[id]->ctrl1)) | (FAV3_TRIG_FP_ISYNC |
					     FAV3_SRESET_FP_ISYNC));
  FAV3UNLOCK;

}

/* Set trigger output options
 *   trigout bits:
 *      0  0  1  Live Internal Trigger to Output
 *      0  1  0  Enable Front Panel Trigger Output
 *      1  0  0  Enable VXS Trigger Output
 *
 * RETURNS: OK, or ERROR if unsuccessful
 */

int
faSetTrigOut(int id, int trigout)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("faSetTrigOut: ERROR : ADC in slot %d is not initialized \n",
	     id);
      return ERROR;
    }

  if(trigout < 0 || trigout > 7)
    {
      printf("faSetTrigOut: ERROR : Invalid trigout value (%d) \n", trigout);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->ctrl1),
	     (vmeRead32(&(FAV3p[id]->ctrl1)) & ~FAV3_TRIGOUT_MASK) |
	     trigout << 12);
  FAV3UNLOCK;

  return OK;
}

/* Routine to get the Trigger Counter value */
uint32_t
faGetTriggerCount(int id)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetTrigCount: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return 0xffffffff;
    }

  /* Just reading - not need to Lock mutex */
  rval = vmeRead32(&FAV3p[id]->trig_scal);

  return rval;

}

int
faResetTriggerCount(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faResetTriggerCount: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig_scal, FAV3_TRIG_SCAL_RESET);
  FAV3UNLOCK;

  return OK;
}

int
faSetThreshold(int id, uint16_t tvalue, uint16_t chmask)
{

  int ii, doWrite = 0;
  uint32_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetThreshold: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chmask == 0)
    chmask = 0xffff;		/* Set All channels the same */

  /*printf("faSetThreshold: slot %d, value %d, mask 0x%04X\n", id, tvalue, chmask); */

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->adc_thres[ii + 1]));

	  if((1 << ii) & chmask)
	    {
	      lovalue = (lovalue & ~FAV3_THR_VALUE_MASK) | tvalue;
	      doWrite = 1;
	    }
	  if((1 << (ii + 1)) & chmask)
	    {
	      hivalue = (hivalue & ~FAV3_THR_VALUE_MASK) | tvalue;
	      doWrite = 1;
	    }

	  if(doWrite)
	    vmeWrite32((uint32_t *) & (FAV3p[id]->adc_thres[ii]),
		       lovalue << 16 | hivalue);

	  lovalue = 0;
	  hivalue = 0;
	  doWrite = 0;
	}
    }
  FAV3UNLOCK;

  return (OK);
}


int
faPrintThreshold(int id)
{
  int ii;
  uint16_t tval[FAV3_MAX_ADC_CHANNELS];

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPrintThreshold: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tval[ii] = vmeRead16(&(FAV3p[id]->adc_thres[ii]));
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


int
faSetDAC(int id, uint16_t dvalue, uint16_t chmask)
{
  int ii, doWrite = 0;
  uint32_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetDAC: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (ERROR);
    }

  if(chmask == 0)
    chmask = 0xffff;		/* Set All channels the same */

  if(dvalue > 0xfff)
    {
      logMsg("faSetDAC: ERROR : DAC value (%d) out of range (0-255) \n",
	     dvalue, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {

      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->dac[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->dac[ii + 1]));

	  if((1 << ii) & chmask)
	    {
	      lovalue = dvalue & FAV3_DAC_VALUE_MASK;
	      doWrite = 1;
	    }
	  if((1 << (ii + 1)) & chmask)
	    {
	      hivalue = (dvalue & FAV3_DAC_VALUE_MASK);
	      doWrite = 1;
	    }

	  if(doWrite)
	    vmeWrite32((uint32_t *) & (FAV3p[id]->dac[ii]),
		       lovalue << 16 | hivalue);

	  lovalue = 0;
	  hivalue = 0;
	  doWrite = 0;
	}

    }
  FAV3UNLOCK;

  return (OK);
}


void
faPrintDAC(int id)
{
  int ii;
  uint16_t dval[FAV3_MAX_ADC_CHANNELS];

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPrintDAC: ERROR : ADC in slot %d is not initialized \n", id,
	     0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    dval[ii] = vmeRead16(&(FAV3p[id]->dac[ii])) & FAV3_DAC_VALUE_MASK;
  FAV3UNLOCK;


  printf(" DAC Settings for FADC in slot %d:", id);
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if((ii % 4) == 0)
	printf("\n");
      printf("Chan %2d: %5d   ", (ii + 1), dval[ii]);
    }
  printf("\n");

}

uint32_t
faGetChannelDAC(int id, uint32_t chan)
{
  uint32_t val;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetChannelDAC: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  val = vmeRead16(&(FAV3p[id]->dac[chan])) & FAV3_DAC_VALUE_MASK;
  FAV3UNLOCK;


  return (val);
}

int
faSetChannelPedestal(int id, uint32_t chan, uint32_t ped)
{
  uint32_t lovalue = 0, hivalue = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetChannelPedestal: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg
	("faSetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	 chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(ped > 0xffff)
    {
      logMsg
	("faSetChannelPedestal: ERROR : PED value (%d) out of range (0-65535) \n",
	 ped, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  lovalue = vmeRead16(&FAV3p[id]->adc_pedestal[(chan & 0xE) + 0]);
  hivalue = vmeRead16(&FAV3p[id]->adc_pedestal[(chan & 0xE) + 1]);

  if(chan & 0x1)
    hivalue = ped;
  else
    lovalue = ped;

  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_pedestal[chan & 0xE]),
	     (lovalue << 16) | hivalue);
  FAV3UNLOCK;

  return (OK);
}

int
faGetChannelPedestal(int id, uint32_t chan)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetChannelPedestal: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg
	("faSetChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	 chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead16(&FAV3p[id]->adc_pedestal[chan]) & FAV3_ADC_PEDESTAL_MASK;
  FAV3UNLOCK;

  return (rval);
}



#ifdef CLAS12

int
faSetChannelDelay(int id, uint32_t chan, uint32_t delay)
{
  uint32_t lovalue = 0, hivalue = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetChannelDelay: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg("faSetChannelDelay: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(delay > 512)
    {
      logMsg
	("faSetChannelDelay: ERROR : Delay value (%d) out of range (0-31) \n",
	 delay, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  lovalue = vmeRead16(&FAV3p[id]->adc_delay[(chan & 0xE) + 0]);
  hivalue = vmeRead16(&FAV3p[id]->adc_delay[(chan & 0xE) + 1]);

  if(chan & 0x1)
    hivalue = delay;
  else
    lovalue = delay;

  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_delay[chan & 0xE]),
	     (lovalue << 16) | hivalue);
  FAV3UNLOCK;

  return (OK);
}

/*Begin Andrea*/
int
faSetDelayAll(int id, uint32_t delay)
{
  int ch = 0;
  int ret;
  for(ch = 0; ch < 16; ch++)
    {
      ret = faSetChannelDelay(id, ch, delay);
      if(ret != OK)
	{
	  logMsg("faSetDelayAll: ERROR for slot %d ch %d\n", id, ch, 3, 4, 5,
		 6);
	  break;
	}
    }
  return ret;
}

int
faSetGlobalDelay(uint32_t delay)
{
  int fadc;
  int id;
  int ret;
  for(fadc = 0; fadc < nfaV3; fadc++)
    {
      id = faV3ID[fadc];
      ret = faSetDelayAll(id, delay);
      if(ret != OK)
	{
	  logMsg("faSetGlobalDelay: ERROR for slot %d \n", fadc, 2, 3, 4, 5,
		 6);
	  break;
	}
    }
  return ret;
}

/*End Andrea*/
int
faGetChannelDelay(int id, uint32_t chan)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetChannelDelay: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg("faSetChannelDelay: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead16(&FAV3p[id]->adc_delay[chan]) & FAV3_ADC_DELAY_MASK;
  FAV3UNLOCK;

  return (rval);
}



int
faInvert(int id, uint16_t chmask)
{
  int ii;
  uint32_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faInvert: ERROR : ADC in slot %d is not initialized \n", id, 0,
	     0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->adc_thres[ii + 1]));

	  if((1 << ii) & chmask)
	    lovalue |= FAV3_THR_INVERT_MASK;
	  else
	    lovalue &= ~FAV3_THR_INVERT_MASK;

	  if((1 << (ii + 1)) & chmask)
	    hivalue |= FAV3_THR_INVERT_MASK;
	  else
	    hivalue &= ~FAV3_THR_INVERT_MASK;

	  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_thres[ii]),
		     lovalue << 16 | hivalue);
	}
    }
  FAV3UNLOCK;
  return (OK);
}

uint32_t
faGetInvertMask(int id)
{
  int ii;
  uint32_t tmp, cmask = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetInvertMask: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tmp = vmeRead16(&FAV3p[id]->adc_thres[ii]);
      if(tmp & FAV3_THR_INVERT_MASK)
	cmask |= (1 << ii);
    }
  FAV3UNLOCK;

  return (cmask);
}

int
faSetHitbitTrigMask(int id, uint16_t chmask)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetHitbitTrigMask: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->hitbit_trig_mask), chmask);
  FAV3UNLOCK;
  return (OK);
}

uint32_t
faGetHitbitTrigMask(int id)
{
  uint32_t rvalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetHitbitTrigMask: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  rvalue = vmeRead32(&(FAV3p[id]->hitbit_trig_mask)) & 0xFFFF;
  FAV3UNLOCK;
  return (rvalue);
}

int
faGSetHitbitMinTOT(uint16_t width)
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faSetHitbitMinTOT(faV3ID[ii], width);

  return (OK);
}


int
faGSetHitbitMinMultiplicity(uint16_t mult)
{
  int ii;

  for(ii = 0; ii < nfaV3; ii++)
    faSetHitbitMinMultiplicity(faV3ID[ii], mult);

  return OK;
}

int
faGetHitbitMinTOT(int id)
{
  uint32_t val;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  val = vmeRead32(&(FAV3p[id]->hitbit_cfg));
  FAV3UNLOCK;
  return (val & 0xFF);
}

int
faSetHitbitMinTOT(int id, uint16_t width)
{
  uint32_t val;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  val = vmeRead32(&(FAV3p[id]->hitbit_cfg));
  val = (val & 0xFFFFFF00) | (width & 0xFF);
  vmeWrite32(&(FAV3p[id]->hitbit_cfg), val);
  FAV3UNLOCK;
  return (OK);
}

int
faGetHitbitMinMultiplicity(int id)
{
  uint32_t val;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  val = vmeRead32(&(FAV3p[id]->hitbit_cfg));
  FAV3UNLOCK;
  return (val >> 8) & 0x1F;
}

int
faSetHitbitMinMultiplicity(int id, uint16_t mult)
{
  uint32_t val;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  val = vmeRead32(&(FAV3p[id]->hitbit_cfg));
  val = (val & 0xFFFFE0FF) | ((mult & 0x1F) << 8);
  vmeWrite32(&(FAV3p[id]->hitbit_cfg), val);
  FAV3UNLOCK;
  return (OK);
}

int
faSetHitbitTrigWidth(int id, uint16_t width)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->hitbit_trig_width), width);
  FAV3UNLOCK;
  return (OK);
}

uint32_t
faGetHitbitTrigWidth(int id)
{
  uint32_t rvalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetHitbitTrigWidth: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  rvalue = vmeRead32(&(FAV3p[id]->hitbit_trig_width)) & 0xFFFF;
  FAV3UNLOCK;
  return (rvalue);
}

int
faThresholdIgnore(int id, uint16_t chmask)
{
  int ii;
  uint32_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faThresholdIgnore: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->adc_thres[ii + 1]));

	  if((1 << ii) & chmask)
	    lovalue |= FAV3_THR_IGNORE_MASK;
	  else
	    lovalue &= ~FAV3_THR_IGNORE_MASK;

	  if((1 << (ii + 1)) & chmask)
	    hivalue |= FAV3_THR_IGNORE_MASK;
	  else
	    hivalue &= ~FAV3_THR_IGNORE_MASK;

	  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_thres[ii]),
		     lovalue << 16 | hivalue);
	}
    }
  FAV3UNLOCK;
  return (OK);
}

uint32_t
faGetThresholdIgnoreMask(int id)
{
  int ii;
  uint32_t tmp, cmask = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetThresholdIgnoreMask: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tmp = vmeRead16(&FAV3p[id]->adc_thres[ii]);
      if(tmp & FAV3_THR_IGNORE_MASK)
	cmask |= (1 << ii);
    }
  FAV3UNLOCK;

  return (cmask);
}

int
faPlaybackDisable(int id, uint16_t chmask)
{
  int ii;
  uint32_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faPlaybackDisable: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (-1);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->adc_thres[ii + 1]));

	  if((1 << ii) & chmask)
	    lovalue |= FAV3_PLAYBACK_DIS_MASK;
	  else
	    lovalue &= ~FAV3_PLAYBACK_DIS_MASK;

	  if((1 << (ii + 1)) & chmask)
	    hivalue |= FAV3_PLAYBACK_DIS_MASK;
	  else
	    hivalue &= ~FAV3_PLAYBACK_DIS_MASK;

	  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_thres[ii]),
		     lovalue << 16 | hivalue);
	}
    }
  FAV3UNLOCK;
  return (OK);
}

uint32_t
faGetPlaybackDisableMask(int id)
{
  int ii;
  uint32_t tmp, cmask = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetPlaybackDisableMask: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tmp = vmeRead16(&FAV3p[id]->adc_thres[ii]);
      if(tmp & FAV3_PLAYBACK_DIS_MASK)
	cmask |= (1 << ii);
    }
  FAV3UNLOCK;

  return (cmask);
}


int
faGetChThreshold(int id, int ch)
{
  uint32_t rvalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetThresholdAll: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rvalue = vmeRead16(&(FAV3p[id]->adc_thres[ch])) & FAV3_THR_VALUE_MASK;
  FAV3UNLOCK;

  return rvalue;
}

int
faSetChThreshold(int id, int ch, int threshold)
{
  /*  printf("faSetChThreshold: slot %d, ch %d, threshold=%d\n",id,ch,threshold); */

  return faSetThreshold(id, threshold, (1 << ch));
}

int
faGLoadChannelPedestals(char *fname, int updateThresholds)
{
  FILE *fd = NULL;
  float ped, adc_ped[FAV3_MAX_BOARDS + 1][FAV3_MAX_ADC_CHANNELS];
  int offset, slot, chan, ii, nsamples, threshold, status;
  float sigma;

  fd = fopen(fname, "r");
  if(fd == NULL)
    {
      printf("faGLoadChannelPedestals: ERROR: cannot open pedestal file >%s<\n",
	     fname);
      return (ERROR);
    }
  else
    {				/* pedestal file opened */
      printf("faGLoadChannelPedestals: pedestal file >%s< is opened for reading\n",
	     fname);

      memset(adc_ped, 0, sizeof(adc_ped));

      while((status =
	     fscanf(fd, "%d %d %f %f %d\n", &slot, &chan, &ped, &sigma,
		    &offset)) > 0)
	{
	  /*      printf("status=%d -> slot=%2d chan=%2d ped=%7.3f sigma=%6.3f\n",status,slot,chan,ped,sigma); */
	  if(slot >= 2 && slot < 21 && chan >= 0 && chan < 16)
	    {
	      adc_ped[slot][chan] = ped;
	      /*        printf("PED=%f\n",adc_ped[slot][chan]); */
	    }
	  else
	    {
	      printf("bad slot=%d or chan=%d\n", slot, chan);
	    }
	}

      if(status == EOF)
	{			/*printf("EOF reached\n"); */
	}
      else
	printf("fscanf() returned error %d\n", status);

      fclose(fd);
      printf("faGLoadChannelPedestals: pedestal file >%s< is closed\n",
	     fname);
    }

  for(ii = 0; ii < nfaV3; ii++)
    {
      slot = faV3ID[ii];
      nsamples = faGetNSA(slot) + faGetNSB(slot);
      /*    printf("faGLoadChannelPedestals: slot=%d, nsamples=%d\n",slot,nsamples); */
      for(chan = 0; chan < FAV3_MAX_ADC_CHANNELS; chan++)
	{
	  ped = adc_ped[slot][chan] * ((float) nsamples);
	  /*      printf("faGLoadChannelPedestals: chan=%d, ped=%d\n",chan,(int)ped); */
	  faSetChannelPedestal(slot, chan, (int) ped);

	  if(updateThresholds)
	    {
	      threshold = faGetChThreshold(slot, chan);
	      /* if threshold=0, don't add pedestal since user is disabling zero suppression */
	      if(threshold)
		threshold += (int) adc_ped[slot][chan];
	      /*        printf("faGLoadChannelPedestals: chan=%d, threshold=%d\n",chan,threshold); */
	      faSetChThreshold(slot, chan, threshold);
	    }
	}
    }

  return OK;
}

#define FAV3_MEASURE_PED_NTIMES		10

int
faMeasureChannelPedestal(int id, uint32_t chan, fa250Ped * ped)
{
  int status, i, n;
  uint32_t sample0, sample1;
  double adc_val, nsamples;
  fa250Ped p;

  p.avg = 0.0;
  p.rms = 0.0;
  p.min = 4095.0;
  p.max = 0.0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faMeasureChannelPedestal: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg
	("faMeasureChannelPedestal: ERROR : Channel (%d) out of range (0-15) \n",
	 chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  for(n = 0; n < FAV3_MEASURE_PED_NTIMES; n++)
    {
      FAV3LOCK;
      vmeWrite16(&FAV3p[id]->la_ctrl, 0);	/* disable logic analyzer */
      for(i = 0; i < 16; i++)
	{
	  vmeWrite16(&FAV3p[id]->la_cmp_mode0[i], 0);	/* setup a don't care trigger */
	  vmeWrite16(&FAV3p[id]->la_cmp_thr0[i], 0);	/* setup a don't care trigger */
	}
      vmeWrite16(&FAV3p[id]->la_ctrl, 1);	/* enable logic analyzer */
      FAV3UNLOCK;

      taskDelay(1);

      FAV3LOCK;
      status = vmeRead16(&FAV3p[id]->la_status);
      vmeWrite16(&FAV3p[id]->la_ctrl, 0);	/* disable logic analyzer */
      FAV3UNLOCK;

      if(!status)
	{
	  logMsg("faMeasureChannelPedestal: ERROR : timeout \n");
	  return (ERROR);
	}

      FAV3LOCK;
      for(i = 0; i < 512; i++)
	{
	  uint32_t idx = (chan * 13) / 16;
	  uint32_t shift = (chan * 13) % 16;
	  sample0 = (uint32_t) vmeRead16(&FAV3p[id]->la_data[idx]);
	  if(idx < 12)
	    sample1 = (uint32_t) vmeRead16(&FAV3p[id]->la_data[idx + 1]);

	  adc_val =
	    (double) (((sample0 >> shift) | (sample1 << (16 - shift))) &
		      0xFFF);

	  p.avg += adc_val;

	  p.rms += adc_val * adc_val;

	  if(adc_val < p.min)
	    p.min = adc_val;

	  if(adc_val > p.max)
	    p.max = adc_val;
	}
      FAV3UNLOCK;
    }

  nsamples = 512.0 * (double) FAV3_MEASURE_PED_NTIMES;

  p.avg /= nsamples;
  p.rms = sqrt(p.rms / nsamples - p.avg * p.avg);

  printf("faMeasureChannelPedestal: slot %d, chan %d => avg %6.3f, rms %6.3f, min %.0f, max %.0f\n",
	 id, chan, p.avg, p.rms, p.min, p.max);

  if(ped)
    *ped = p;

  return (OK);
}

// Mode=0: normal pulse integration trigger mode
// Mode=1: discriminator trigger mode
int
faSetTriggerProcessingMode(int id, uint32_t chan, uint32_t mode)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetTriggerProcessingMode: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg
	("faSetTriggerProcessingMode: ERROR : Channel (%d) out of range (0-15) \n",
	 chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->adc_gain[chan]);
  if(mode)
    rval |= 0x8000;
  else
    rval &= 0x7FFF;
  vmeWrite32(&FAV3p[id]->adc_gain[chan], rval);
  FAV3UNLOCK;

  return (OK);
}

int
faGetTriggerProcessingMode(int id, uint32_t chan)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faGetTriggerProcessingMode: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg
	("faGetTriggerProcessingMode: ERROR : Channel (%d) out of range (0-15) \n",
	 chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->adc_gain[chan]);
  if(rval & 0x8000)
    rval = 1;
  else
    rval = 0;
  FAV3UNLOCK;

  return (rval);
}

int
faSetChannelGain(int id, uint32_t chan, float gain)
{
  uint32_t rval = 0;
  int igain;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetChannelGain: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg("faSetChannelGain: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(gain >= 127.0 || gain < 0.0)
    {
      logMsg
	("faSetChannelGain: ERROR : GAIN value (%f) out of range (0.0-127.0) \n",
	 gain, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  igain = (int) (gain * 256.0);
  /*  printf("gain=%f igain=%d\n",gain,igain); */

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->adc_gain[chan]) & 0x8000;
  rval |= igain & 0x7FFF;
  vmeWrite32(&FAV3p[id]->adc_gain[chan], igain);
  FAV3UNLOCK;

  return (OK);
}

float
faGetChannelGain(int id, uint32_t chan)
{
  uint32_t rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetChannelGain: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  if(chan > 16)
    {
      logMsg("faSetChannelGain: ERROR : Channel (%d) out of range (0-15) \n",
	     chan, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->adc_gain[chan]) & 0x7FFF;
  FAV3UNLOCK;

  return (((float) rval) / 256.0);
}

int
faResetMGT(int id, int reset)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  //  faSetMGTSettings(id, 0, 2, 2);

  FAV3LOCK;
  if(reset)
    vmeWrite32(&(FAV3p[id]->gtx_ctrl), 0x203);	/*put reset */
  else
    vmeWrite32(&(FAV3p[id]->gtx_ctrl), 0x800);	/*release reset */
  FAV3UNLOCK;

  taskDelay(2);

  return (OK);
}

int
faGetMGTChannelStatus(int id)
{
  int status = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (0);
    }

  FAV3LOCK;
  status = vmeRead32(&(FAV3p[id]->gtx_status));
  //status = vmeRead32(&(FAV3p[id]->trx_ctrl));
  FAV3UNLOCK;

  if(status & 0x1)
    //  if(status & 0x1000)
    return (1);

  return (0);
}

/*
  int
  faSetMGTSettings(int id, int txpre, int txswing, int rxequ)
  {
  int val;

  if(id==0) id=faV3ID[0];

  if((id<=0) || (id>21) || (FAV3p[id] == NULL))
  {
  printf("%s: ERROR : ADC in slot %d is not initialized \n",
  __func__,id);
  return(ERROR);
  }

  val = ((txpre & 0x1F)<<0) |
  ((txswing & 0xF)<<10) |
  ((rxequ & 0x3)<<14);

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->trx_ctrl),val);
  FAV3UNLOCK;

  taskDelay(2);

  return(OK);
  }
*/


#endif



/**************************************************************************************
 *
 *  faSetMGTTestMode -
 *  faSyncResetMode  -  Set the fa250 operation when Sync Reset is received
 *
 *        When SYNC RESET is received by the module, the module may:
 *      mode : 0  - Send a calibration sequence to the CTP for alignment purposes
 *             1  - Normal operation
 *
 *        In both instances, timestamps and counters will be reset.
 *
 *   RETURNS OK, or ERROR if unsuccessful.
 */

int
faSetMGTTestMode(int id, uint32_t mode)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  if(mode)
    {				/* After Sync Reset (Normal mode) */
      vmeWrite32(&FAV3p[id]->mgt_ctrl, FAV3_MGT_RESET);
      vmeWrite32(&FAV3p[id]->mgt_ctrl, FAV3_MGT_FRONT_END_TO_CTP);
    }
  else
    {				/* Before Sync Reset (Calibration Mode) */
      vmeWrite32(&FAV3p[id]->mgt_ctrl, FAV3_RELEASE_MGT_RESET);
      vmeWrite32(&FAV3p[id]->mgt_ctrl, FAV3_MGT_RESET);
      vmeWrite32(&FAV3p[id]->mgt_ctrl, FAV3_MGT_ENABLE_DATA_ALIGNMENT);
    }
  FAV3UNLOCK;

  return (OK);
}

int
faSyncResetMode(int id, uint32_t mode)
{
  return faSetMGTTestMode(id, mode);
}

/**************************************************************************************
 *
 *  faReadScalers - Scaler Data readout routine
 *        Readout the desired scalers (indicated by the channel mask), as well
 *        as the timer counter.  The timer counter will be the last word
 *        in the "data" array.
 *
 *    id     - Slot number of module to read
 *    data   - local memory address to place data
 *    chmask - Channel Mask (indicating which channels to read)
 *    rflag  - Readout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS the number of 32bit words read, or ERROR if unsuccessful.
 */
int
faReadScalers(int id, volatile uint32_t * data, uint32_t chmask, int rflag)
{
  int doLatch = 0, doClear = 0, ichan = 0;
  int dCnt = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faReadScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  if(rflag & ~(FAV3_SCALER_CTRL_MASK))
    {
      logMsg("faReadScalers: WARN : rflag (0x%x) has undefined bits \n",
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
	  data[dCnt] = vmeRead32(&FAV3p[id]->scaler[ichan]);
	  dCnt++;
	}
    }

  data[dCnt] = vmeRead32(&FAV3p[id]->time_count);
  dCnt++;

  if(doClear)
    vmeWrite32(&FAV3p[id]->scaler_ctrl,
	       FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_RESET);
  FAV3UNLOCK;

  return dCnt;

}

/**************************************************************************************
 *
 *  faPrintScalers - Scaler Print Out routine
 *        Print out the scalers as well as the timer counter.
 *
 *    id     - Slot number of module to read
 *    rflag  - Printout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS ok if successful , or ERROR if unsuccessful.
 */
int
faPrintScalers(int id, int rflag)
{
  int doLatch = 0, doClear = 0, ichan = 0;
  uint32_t data[16], time_count;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPrintScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  if(rflag & ~(FAV3_SCALER_CTRL_MASK))
    {
      logMsg("faPrintScalers: WARN : rflag (0x%x) has undefined bits \n",
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
      data[ichan] = vmeRead32(&FAV3p[id]->scaler[ichan]);
    }

  time_count = vmeRead32(&FAV3p[id]->time_count);

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

int
faClearScalers(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faClearScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl,
	     FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_RESET);
  FAV3UNLOCK;

  return OK;
}


int
faLatchScalers(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faLatchScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl,
	     FAV3_SCALER_CTRL_ENABLE | FAV3_SCALER_CTRL_LATCH);
  FAV3UNLOCK;

  return OK;
}

int
faEnableScalers(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faEnableScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl, FAV3_SCALER_CTRL_ENABLE);
  FAV3UNLOCK;

  return OK;
}

int
faDisableScalers(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faDisableScalers: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_ctrl, ~FAV3_SCALER_CTRL_ENABLE);
  FAV3UNLOCK;

  return OK;
}


/**************************************************************************************
 *
 *  faReadChargeScalers - Scaler Data readout routine
 *        Readout the desired scalers (indicated by the channel mask), as well
 *        as the timer counter.  The timer counter will be the last word
 *        in the "data" array.
 *
 *    id     - Slot number of module to read
 *    data   - local memory address to place data
 *    chmask - Channel Mask (indicating which channels to read)
 *    rflag  - Readout Flag
 *            bit 0 - Latch Scalers before read
 *            bit 1 - Clear Scalers after read
 *
 *   RETURNS the number of 32bit words read, or ERROR if unsuccessful.
 */
int
faReadChargeScalers(int id, volatile uint64_t * data, uint32_t chmask)
{
  int ichan = 0;
  int dCnt = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faReadChargeScalers: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite16(&FAV3p[id]->adc_scaler_ctrl, 1);

  for(ichan = 0; ichan < 16; ichan++)
    {
      if((1 << ichan) & chmask)
	{
	  data[dCnt] =
	    (uint64_t) vmeRead16(&FAV3p[id]->adc_accumulator0[ichan]);
	  data[dCnt] |=
	    ((uint64_t) vmeRead16(&FAV3p[id]->adc_accumulator1[ichan])) << 16;
	  data[dCnt] |=
	    ((uint64_t) vmeRead16(&FAV3p[id]->adc_accumulator2[ichan])) << 32;
	  dCnt++;
	}
    }

  FAV3UNLOCK;

  return dCnt;

}

/**
 * @ingroup Status
 *  @brief Return the base address of the A32 for specified module
 *  @param id
 *   - Slot Number
 *  @return A32 address base, if successful. Otherwise ERROR.
 */

uint32_t
faGetA32(int id)
{
  uint32_t rval = 0;
  if(FAV3pd[id])
    {
      rval = (uint32_t) ((u_long) (FAV3pd[id]) - faV3A32Offset);
    }
  else
    {
      logMsg("faGetA32(%d): A32 pointer not initialized\n",
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
faGetA32M()
{
  uint32_t rval = 0;
  if(FAV3pmb)
    {
      rval = (uint32_t) ((u_long) (FAV3pmb) - faV3A32Offset);
    }
  else
    {
      logMsg("faGetA32M: A32M pointer not initialized\n", 1, 2, 3, 4, 5, 6);
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
faGetMinA32MB(int id)
{
  uint32_t rval = 0, a32addr, addrMB;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  FAV3LOCK;

  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));

  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;
  addrMB = (addrMB & FAV3_AMB_MIN_MASK) << 16;

#ifdef DEBUG
  printf("faGetMinA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n", a32addr,
	 addrMB, id);
#endif

  id = faV3ID[0];
  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;

  rval = a32addr;
#ifdef DEBUG
  printf("faGetMinA32MB: rval=0x%08x\n", rval);
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
faGetMaxA32MB(int id)
{
  uint32_t rval = 0, a32addr, addrMB;

  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  FAV3LOCK;

  a32addr = vmeRead32(&(FAV3p[id]->adr32));
  addrMB = vmeRead32(&(FAV3p[id]->adr_mb));

  a32addr = (a32addr & FAV3_A32_ADDR_MASK) << 16;
  addrMB = addrMB & FAV3_AMB_MAX_MASK;

  rval = addrMB;

#ifdef DEBUG
  printf("faGetMaxA32MB: a32addr=0x%08x addrMB=0x%08x for slot %d\n", a32addr,
	 addrMB, id);
  printf("faGetMaxA32MB: rval=0x%08x\n", rval);
#endif

  FAV3UNLOCK;

  return rval;
}






/*********************************************
 *
 *  FADC Internal Trigger FADC Configuration and Control
 *  Routines.
 */

// FIXME: Need itrig regs defined to uncomment
/* #include "faV3Itrig.c" */



/* -------------------------------------------------------------------------------------

   Utility routines
*/

void
faPrintAuxScal(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPrintAuxScal: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  printf("Auxillary Scalers:\n");
  printf("       Word Count:         %d\n",
	 vmeRead32(&FAV3p[id]->proc_words_scal));
  printf("       Headers   :         %d\n", vmeRead32(&FAV3p[id]->header_scal));
  printf("       Trailers  :         %d\n",
	 vmeRead32(&FAV3p[id]->trailer_scal));
  FAV3UNLOCK;

  return;
}

void
faPrintFifoStatus(int id)
{
  uint32_t ibuf, bbuf, obuf, dflow;
  uint32_t wc[2], mt[2], full[2];
  uint32_t rdy[2];

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faPrintFifoStatus: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return;
    }

  FAV3LOCK;
  dflow = vmeRead32(&(FAV3p[id]->dataflow_status));
  ibuf = vmeRead32(&(FAV3p[id]->status[0])) & 0xdfffdfff;
  bbuf = vmeRead32(&(FAV3p[id]->status[1])) & 0x1fff1fff;
  obuf = vmeRead32(&(FAV3p[id]->status[2])) & 0x3fff3fff;
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


  return;

}

int
faLive(int id, int sflag)
{
  int ilt = 0;
  uint32_t live;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faLive: ERROR : ADC in slot %d is not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (ERROR);
    }

  /* Read Current Scaler values */
  FAV3LOCK;
  live = vmeRead32(&(FAV3p[id]->internal_trig_scal));

  vmeWrite32(&(FAV3p[id]->internal_trig_scal), 0x80000000);
  FAV3UNLOCK;

  if(live == 0)			/* scaler is zero or disabled */
    return (0);
  ilt = live;

  return (ilt);
}


void
faDataDecode(uint32_t data)
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
      else
	{
	  faV3_data.evt_num_2 = (data & 0x7FFFFFF);
	  if(i_print)
	    printf("%8X - EVENT HEADER 2 - evt_num = %d\n", data,
		   faV3_data.evt_num_2);
	}
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
	  else if(time_last == 2)
	    {
	      faV3_data.time_3 = (data & 0xFFFFFF);
	      if(i_print)
		printf("%8X - TRIGGER TIME 3 - time = %08x\n", data,
		       faV3_data.time_3);
	      faV3_data.time_now = 3;
	    }
	  else if(time_last == 3)
	    {
	      faV3_data.time_4 = (data & 0xFFFFFF);
	      if(i_print)
		printf("%8X - TRIGGER TIME 4 - time = %08x\n", data,
		       faV3_data.time_4);
	      faV3_data.time_now = 4;
	    }
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
      faV3_data.trig_type_int = data & 0x7;
      faV3_data.trig_state_int = (data & 0x8) >> 3;
      faV3_data.evt_num_int = (data & 0xFFF0) >> 4;
      faV3_data.err_status_int = (data & 0x10000) >> 16;
      if(i_print)
	printf("%8X - INTERNAL TRIGGER - type = %d   state = %d   num = %d   error = %d\n",
	       data, faV3_data.trig_type_int, faV3_data.trig_state_int,
	       faV3_data.evt_num_int, faV3_data.err_status_int);
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

void
faTestSetSystemTestMode(int id, int mode)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  if(mode >= 1)
    reg = FAV3_CTRL1_SYSTEM_TEST_MODE;
  else
    reg = 0;

  FAV3LOCK;

  vmeWrite32(&(FAV3p[id]->ctrl1), vmeRead32(&FAV3p[id]->ctrl1) | reg);

  /*   printf(" ctrl1 = 0x%08x\n",vmeRead32(&FAV3p[id]->ctrl1)); */
  FAV3UNLOCK;

}

void
faTestSetTrigOut(int id, int mode)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  if(mode >= 1)
    reg = FAV3_TESTBIT_TRIGOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->testBit), reg);
  FAV3UNLOCK;

}

void
faTestSetBusyOut(int id, int mode)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  if(mode >= 1)
    reg = FAV3_TESTBIT_BUSYOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->testBit), reg);
  FAV3UNLOCK;

}

void
faTestSetSdLink(int id, int mode)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  if(mode >= 1)
    reg = FAV3_TESTBIT_SDLINKOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->testBit), reg);
  FAV3UNLOCK;

}

void
faTestSetTokenOut(int id, int mode)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  if(mode >= 1)
    reg = FAV3_TESTBIT_TOKENOUT;
  else
    reg = 0;

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->testBit), reg);
  FAV3UNLOCK;

}

int
faTestGetStatBitB(int id)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->testBit) & FAV3_TESTBIT_STATBITB) >> 8;
  FAV3UNLOCK;

  return reg;

}

int
faTestGetTokenIn(int id)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->testBit) & FAV3_TESTBIT_TOKENIN) >> 9;
  FAV3UNLOCK;

  return reg;

}

int
faTestGetClock250CounterStatus(int id)
{
  int reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = (vmeRead32(&FAV3p[id]->testBit) & FAV3_TESTBIT_CLOCK250_STATUS) >> 15;
  FAV3UNLOCK;

  return reg;

}

uint32_t
faTestGetClock250Counter(int id)
{
  uint32_t reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->clock250count);
  FAV3UNLOCK;

  return reg;

}

uint32_t
faTestGetSyncCounter(int id)
{
  uint32_t reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->syncp0count);
  FAV3UNLOCK;

  return reg;

}

uint32_t
faTestGetTrig1Counter(int id)
{
  uint32_t reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->trig1p0count);
  FAV3UNLOCK;

  return reg;

}

uint32_t
faTestGetTrig2Counter(int id)
{
  uint32_t reg = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  reg = vmeRead32(&FAV3p[id]->trig2p0count);
  FAV3UNLOCK;

  return reg;

}

void
faTestResetClock250Counter(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->clock250count, FAV3_CLOCK250COUNT_RESET);
  vmeWrite32(&FAV3p[id]->clock250count, FAV3_CLOCK250COUNT_START);
  FAV3UNLOCK;

}

void
faTestResetSyncCounter(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->syncp0count, FAV3_SYNCP0COUNT_RESET);
  FAV3UNLOCK;

}

void
faTestResetTrig1Counter(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig1p0count, FAV3_TRIG1P0COUNT_RESET);
  FAV3UNLOCK;

}

void
faTestResetTrig2Counter(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->trig2p0count, FAV3_TRIG2P0COUNT_RESET);
  FAV3UNLOCK;

}

uint32_t
faTestGetTestBitReg(int id)
{
  uint32_t rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return 0;
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->testBit);
  FAV3UNLOCK;

  return rval;
}

/**
 *  @ingroup Status
 *  @brief Return the status of the current system clock
 *
 *   Enable and disables test mode during operation
 *
 *  @sa faTestSetSystemTestMode
 *  @sa faTestGetClock250CounterStatus
 *  @sa faTestGetClock250Counter
 *  @sa faTestResetClock250Counter
 *
 *  @param id Slot number
 *  @param pflag print to standard output if > 0
 *
 *  @return OK if system clock is available, otherwise ERROR.
 */

int
faTestSystemClock(int id, int pflag)
{
  uint32_t rval = OK;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  /* Enable test mode */
  faTestSetSystemTestMode(id, 1);

  /* reset clock counter */
  faTestResetClock250Counter(id);

  int iwait = 0;
  /* Wait for the 20us internal timer */
  while(iwait++ < 50)
    {
      if(faTestGetClock250CounterStatus(id) == 0)
	break;
    }

  /* Counter should return 5000 if the system clock is 250Mhz */
  int expected = 5000, measured = 0, diff = 0;

  measured = faTestGetClock250Counter(id);
  diff = abs(expected - measured);

  if(diff < 5)
    rval = OK;
  else
    rval = ERROR;

  /* Disable test mode */
  faTestSetSystemTestMode(id, 0);

  if(pflag)
    {
      printf("%s: System Clock is %s\n",
	     __func__, (rval == OK) ? "Present" : "NOT PRESENT");
    }

  return rval;
}


/**************************************************************************************
 *
 *  faGetSerialNumber - Available for firmware>=0x0211
 *      Fills 'rval' with a character array containing the fa250 serial number.
 *
 *      If snfix >= 1, will attempt to format the serial number to maintain
 *        consistency between serials made by the same vendor.
 *        e.g. Advanced Assembly:
 *          snfix=0: B21595-02R  B2159515R1
 *          snfix=1: B21595-02R  B21595-15R1
 *        e.g. ACDI
 *          snfix=0: ACDI002
 *          snfix=1: ACDI-002
 *
 *
 *   RETURNS length of character array 'rval' if successful, otherwise ERROR
 */

int
faGetSerialNumber(int id, char **rval, int snfix)
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

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  for(i = 0; i < 3; i++)
    sn[i] = vmeRead32(&FAV3p[id]->serial_number[i]);
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


/**************************************************************************************
 *
 *  faSetScalerBlockInterval
 *      Data from scalers may be inserted into the readout data stream
 *      at regular event count intervals.  The interval is specified in
 *      multiples of blocks.
 *
 *    Argument:
 *        nblock:
 *             0 : No insertion of scaler data into the data stream
 *           >=1 : The current scaler values are appended to the last event
 *                  of the appropriate n'th block of events.
 *
 *    Note: Scalers are NOT reset after their values are captured.
 *
 *   RETURNS OK if successful, otherwise ERROR.
 */

int
faSetScalerBlockInterval(int id, uint32_t nblock)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  if(nblock > FAV3_SCALER_INTERVAL_MASK)
    {
      printf("%s: ERROR: Invalid value of nblock (%d).\n", __func__, nblock);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->scaler_interval, nblock);
  FAV3UNLOCK;

  return OK;
}

int
faGetScalerBlockInterval(int id)
{
  int rval = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__,
	     id);
      return ERROR;
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->scaler_interval) & FAV3_SCALER_INTERVAL_MASK;
  FAV3UNLOCK;

  return rval;
}

/**************************************************************************************
 *
 *  faForceEndOfBlock
 *      Allows for the insertion of a block trailer into the data stream.  This is
 *      useful for the efficient extraction of a partial block of events
 *      from the FADC (e.g. for an end of run event, or the resynchonize with
 *      other modules).
 *      Event count within block is reset, after successful execution.
 *
 *   ARG:
 *     scalers:  If set to > 0, scalers will also be inserted with the End of Block
 *
 *   RETURNS OK if successful, otherwise ERROR.
 */

int
faForceEndOfBlock(int id, int scalers)
{
  int rval = OK, icheck = 0, timeout = 1000, csr = 0;
  int proc_config = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faForceEndOfBlock: ERROR : ADC in slot %d is not initialized \n",
	 id, 2, 3, 4, 5, 6);
      return ERROR;
    }

  FAV3LOCK;
  /* Disable triggers to Processing FPGA (if enabled) */
  proc_config = vmeRead32(&FAV3p[id]->adc_config[0]);
  vmeWrite32(&FAV3p[id]->adc_config[0], proc_config & ~(FAV3_ADC_PROC_ENABLE));

  csr = FAV3_CSR_FORCE_EOB_INSERT;
  if(scalers > 0)
    csr |= FAV3_CSR_DATA_STREAM_SCALERS;

  vmeWrite32(&FAV3p[id]->csr, csr);

  for(icheck = 0; icheck < timeout; icheck++)
    {
      csr = vmeRead32(&FAV3p[id]->csr);
      if(csr & FAV3_CSR_FORCE_EOB_SUCCESS)
	{
	  logMsg("faForceEndOfBlock: Block trailer insertion successful\n",
		 1, 2, 3, 4, 5, 6);
	  rval = ERROR;
	  break;
	}

      if(csr & FAV3_CSR_FORCE_EOB_FAILED)
	{
	  logMsg("faForceEndOfBlock: Block trailer insertion FAILED\n",
		 1, 2, 3, 4, 5, 6);
	  rval = ERROR;
	  break;
	}
    }

  if(icheck == timeout)
    {
      logMsg("faForceEndOfBlock: Block trailer insertion FAILED on timeout\n",
	     1, 2, 3, 4, 5, 6);
      rval = ERROR;
    }

  /* Restore the original state of the Processing FPGA */
  vmeWrite32(&FAV3p[id]->adc_config[0], proc_config);

  FAV3UNLOCK;

  return rval;
}

void
faGForceEndOfBlock(int scalers)
{
  int ii, res;

  for(ii = 0; ii < nfaV3; ii++)
    {
      res = faForceEndOfBlock(faV3ID[ii], scalers);
      if(res < 0)
	printf("%s: ERROR: slot %d, in faForceEndOfBlock()\n",
	       __func__, faV3ID[ii]);
    }

}

/***************************************************************************************
   JLAB FADC Signal Distribution Card (SDC) Routines

   cFlag:  controls the configuation of the SDC

          0:  Default Mode  Internal CLK, Sync External Trigger and Sync Reset
        > 0:  Pass through mode

   bMask:  mask of Busy enables for the SDC - Do not Enable busy if there is no FADC

*/

int
faSDC_Config(uint16_t cFlag, uint16_t bMask)
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Config: ERROR : Cannot Configure FADC Signal Board \n", 0,
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

void
faSDC_Status(int sFlag)
{

  uint16_t sdc[4];
  int ibit = 0;

  if(FAV3SDCp == NULL)
    {
      printf("faSDC_Status: ERROR : No FADC SDC available \n");
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


void
faSDC_Enable(int nsync)
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Enable: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
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

void
faSDC_Disable()
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Disable: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->ctrl),
	     (FAV3SDC_CTRL_ENABLE_SOFT_TRIG | FAV3SDC_CTRL_ENABLE_SOFT_SRESET));
  FAV3UNLOCK;
}



void
faSDC_Sync()
{

  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Sync: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->csr), FAV3SDC_CSR_SRESET);
  FAV3UNLOCK;
}

void
faSDC_Trig()
{
  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Trig: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return;
    }

  FAV3LOCK;
  vmeWrite16(&(FAV3SDCp->csr), FAV3SDC_CSR_TRIG);
  FAV3UNLOCK;
}

int
faSDC_Busy()
{
  int busy = 0;

  if(FAV3SDCp == NULL)
    {
      logMsg("faSDC_Busy: ERROR : No FADC SDC available \n", 0, 0, 0, 0, 0,
	     0);
      return -1;
    }

  FAV3LOCK;
  busy = vmeRead16(&(FAV3SDCp->csr)) & FAV3SDC_CSR_BUSY;
  FAV3UNLOCK;

  return (busy);
}



/********************************/
/* sergey: returns some globals */



int
faGetProcMode(int id, int *pmode, uint32_t * PL, uint32_t * PTW,
	      uint32_t * NSB, uint32_t * NSA, uint32_t * NP)
{
  uint32_t tmp;

  if(id == 0)
    id = faV3ID[0];
  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetProcMode: ERROR : FADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  *PTW = (vmeRead32(&(FAV3p[id]->adc_ptw)) & 0xFFFF);
  *PL = (vmeRead32(&(FAV3p[id]->adc_pl)) & 0xFFFF);
  *NSB = (vmeRead32(&(FAV3p[id]->adc_nsb)) & 0xFFFF);
  *NSA = (vmeRead32(&(FAV3p[id]->adc_nsa)) & 0xFFFF);

  tmp = (vmeRead32(&(FAV3p[id]->adc_config[0])) & 0xFFFF);
  *pmode = (tmp & FAV3_ADC_PROC_MASK) + 1;
  *NP = (tmp & FAV3_ADC_PEAK_MASK) >> 4;

  return (0);
}




int
faGetNfadc()
{
  return (nfaV3);
}

/*
  int
  faSlot(uint32_t id)
  {
  if(id>=nfaV3)
  {
  printf("%s: ERROR: Index (%d) >= FADCs initialized (%d).\n",__func__,id,nfaV3);
  return(ERROR);
  }

  return(faV3ID[id]);
  }
*/


int
faId(uint32_t slot)
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
faSetThresholdAll(int id, uint16_t tvalue[16])
{
  int ii;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faSetThresholdAll: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      faSetChThreshold(id, ii, tvalue[ii]);
    }

  return (OK);
}



/*sergey: set same pedestal for all channels, will change it later*/

/*
  todo
  setgain
  printgain
*/

int
faSetPedestal(int id, uint32_t wvalue)
{
  int ii;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faSetPedestal: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(!(ii & 0x1))
	vmeWrite32((uint32_t *) & (FAV3p[id]->adc_pedestal[ii]),
		   wvalue | (wvalue << 16));
    }
  FAV3UNLOCK;

  return (OK);
}

int
faPrintPedestal(int id)
{
  int ii;
  uint32_t tval[FAV3_MAX_ADC_CHANNELS];

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faPrintPedestal: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tval[ii] = vmeRead16(&(FAV3p[id]->adc_pedestal[ii]));
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



/**************************/
/* begin debugging for Ed */

/* Ed's email:

   I am adding state machine tracing code to the firmware.  This will save the last 500 state changes of the data flow control state machine into a FIFO.  Please modify your code to do the following:


   STATE_CSR register:  address = 0x504

   STATE_VALUE register:  address = 0x508


   Before 'GO':

   - Arm the storage of states by writing value 0x80000000 to STATE_CSR  (0x504)


   After the error occurs and all status registers have been read and printed:

   - Disarm storage of states by writing 0x0 to STATE_CSR  (0x504)

   - Read (and print) STATE_CSR (= value) to get the number of valid states stored

   - Print all valid stored state values:

   num_states = 0x1FF & value;

   for(ii = 0; ii < num_states; ii++)

   {

   read and print STATE_VALUE register  (0x508);      // read gets next value in FIFO

   }


   ----------------

   Since the data flow state machine will stop changing when the triggers stop due to the busy assertion I can trace back the history of this state machine.  The Xilinx Chipscope tool records state values on each clock edge and thus can only show a limited history of the state machine.

   ----------------
*/

int
faArmStatesStorage(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faArmStatesStorage: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->state_csr), 0x80000000);
  FAV3UNLOCK;

  printf("faArmStatesStorage: ARMED slot %d\n", id);

  return (OK);
}

int
faDisarmStatesStorage(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faDisarmStatesStorage: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&(FAV3p[id]->state_csr), 0x0);
  FAV3UNLOCK;

  printf("faDisarmStatesStorage: DISARMED slot %d\n", id);

  return (OK);
}

int
faReadStatesStorage(int id)
{
  int ii;
  uint32_t value, fifo;
  int num_states;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg
	("faReadStatesStorage: ERROR : ADC in slot %d is not initialized \n",
	 id, 0, 0, 0, 0, 0);
      return (ERROR);
    }

  FAV3LOCK;

  value = vmeRead32(&(FAV3p[id]->state_csr));

  num_states = value & 0x1FF;
  printf("\nfaReadStatesStorage: slot %d, state_csr = 0x%08x, num_states = %d\n",
	 id, value, num_states);

  /* read and print STATE_VALUE fifo */
  for(ii = 0; ii < num_states; ii++)
    {
      fifo = vmeRead32(&(FAV3p[id]->state_value));
      printf("faReadStatesStorage: fifo[%4d] = 0x%08x\n", ii, fifo);
    }

  FAV3UNLOCK;

  printf("faReadStatesStorage: done printing fifo\n\n");

  return (OK);
}


/*  end debugging for Ed  */
/**************************/

/*
 * Sparsification Routines for NPS
 *
 */




/**
 * @brief Enable / Disable sparsification
 * @details Enable or disable the sparsification logic for the specified module
 * @param[in] id fadc slot number
 * @param[in] mode sparsification mode
 *     0 : Bypass sparsification
 *     1 : Enable
 * @return OK if successful, otherwise ERROR
 */
int
faSetSparsificationMode(int id, int mode)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  /* logic in register is reversed */
  mode = mode ? 0 : 1;

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->sparse_control, mode);
  FAV3UNLOCK;

  return (OK);
}


/**
 * @brief Enable / Disable sparsification
 * @details Enable or disable the sparsification logic for all initialized modules
 * @param[in] mode sparsification mode
 *     0 : Bypass sparsification
 *     1 : Enable
 * @return OK if successful, otherwise ERROR
 */
void
faGSetSparsificationMode(int mode)
{
  int id = 0;

  /* logic in register is reversed */
  mode = mode ? 0 : 1;

  FAV3LOCK;

  for(id = 0; id < nfaV3; id++)
    vmeWrite32(&FAV3p[id]->sparse_control, mode);

  FAV3UNLOCK;
}

/**
 * @brief Sparisification is Enabled / Disabled
 * @details Return the state of the sparsification logic for specified module
 * @param[in] id fadc slot number
 * @return 1 if sparsification is enabled, 0 if bypassed, otherwise ERROR
 */
int
faGetSparsificationMode(int id)
{
  int mode = 0, rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  mode =
    (int) (vmeRead32(&FAV3p[id]->sparse_control) & FAV3_SPARSE_CONTROL_BYPASS);

  /* logic in register is reversed */
  rval = mode ? 0 : 1;
  FAV3UNLOCK;

  return (rval);
}

int
faGetSparsificationStatus(int id)
{
  int rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  rval = (int) (vmeRead32(&FAV3p[id]->sparse_status) & FAV3_SPARSE_STATUS_MASK);
  FAV3UNLOCK;

  return rval;
}

int
faClearSparsificationStatus(int id)
{
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->sparse_control, FAV3_SPARSE_STATUS_CLEAR);
  FAV3UNLOCK;

  return (OK);
}

void
faGClearSparsificationStatus()
{
  int id = 0;

  FAV3LOCK;
  for(id = 0; id < nfaV3; id++)
    vmeWrite32(&FAV3p[id]->sparse_control, FAV3_SPARSE_STATUS_CLEAR);

  FAV3UNLOCK;
}

uint32_t
faGetFirstTriggerMismatch(int id)
{
  uint32_t rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->first_trigger_mismatch);
  FAV3UNLOCK;

  return rval;
}

uint32_t
faGetMismatchTriggerCount(int id)
{
  uint32_t rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->trigger_mismatch_counter);
  FAV3UNLOCK;

  return rval;
}

uint32_t
faGetTriggersProcessedCount(int id)
{
  uint32_t rval = 0;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  rval = vmeRead32(&FAV3p[id]->triggers_processed);
  FAV3UNLOCK;

  return rval;
}

/**
 * @brief Set the scaler mode
 * @details Set the scaler mode using a channel mask for the specified module
 * @param[in] id fadc slot number
 * @param[in] chmask Channel Mask, (bit=0, chan=0; bit 15, chan=15),
 * If the bit is set, using the accumulator mode to summ alal samples.
 * If the bit is not set, use the default TET based pulse integration.
 * @return OK if successful, otherwise ERROR
 */
int
faSetAccumulatorScalerMode(int id, uint16_t chmask)
{
  int ii;
  uint16_t lovalue = 0, hivalue = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return (ERROR);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      if(ii % 2 == 0)
	{
	  lovalue = (vmeRead16(&FAV3p[id]->adc_thres[ii]));
	  hivalue = (vmeRead16(&FAV3p[id]->adc_thres[ii + 1]));

	  if((1 << ii) & chmask)
	    lovalue |= FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK;
	  else
	    lovalue &= ~FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK;

	  if((1 << (ii + 1)) & chmask)
	    hivalue |= FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK;
	  else
	    hivalue &= ~FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK;

	  vmeWrite32((uint32_t *) & (FAV3p[id]->adc_thres[ii]),
		     lovalue << 16 | hivalue);
	}
    }
  FAV3UNLOCK;

  return (OK);
}

/**
 * @brief Set the scaler mode
 * @details Set the scaler mode using a channel mask for all initialized modules
 * @param[in] chmask Channel Mask, (bit=0, chan=0; bit 15, chan=15),
 * If the bit is set, using the accumulator mode to summ alal samples.
 * If the bit is not set, use the default TET based pulse integration.
 * @return OK if successful, otherwise ERROR
 */
int
faGSetAccumulatorScalerMode(uint16_t chmask)
{
  int ifa = 0, rval = OK;

  for(ifa = 0; ifa < nfaV3; ifa++)
    rval |= faSetAccumulatorScalerMode(ifa, chmask);

  return (rval);
}


uint32_t
faGetAccumulatorScalerMode(int id)
{
  int ii;
  uint32_t tmp, cmask = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      logMsg("faGetInvertMask: ERROR : ADC in slot %d is not initialized \n",
	     id, 0, 0, 0, 0, 0);
      return (0);
    }

  FAV3LOCK;
  for(ii = 0; ii < FAV3_MAX_ADC_CHANNELS; ii++)
    {
      tmp = vmeRead16(&FAV3p[id]->adc_thres[ii]);
      if(tmp & FAV3_THR_ACCUMULATOR_SCALER_MODE_MASK)
	cmask |= (1 << ii);
    }
  FAV3UNLOCK;

  return (cmask);
}
