/* Module: faV3FirmwareTools.c
 *
 * Description: FADC250 Firmware Tools Library
 *              Firmware specific functions.
 *
 * Author:
 *        Bryan Moffit
 *        JLab Data Acquisition Group
 *
 * Revision History:
 *      Revision 1.0   2011/07/18  moffit
 *         - Initial version for FADC250 v2
 *
 * SVN: $Rev$
 *
 */

#include <ctype.h>

#define        FAV3_FPGAID_MASK     0xFFFFF000
#define        FAV3_FPGAID_CTRL     0xf2501000
#define        FAV3_FPGAID_PROC     0xf2502000
#define        FAV3_FPGAID_REV_MASK 0x00000FFF

#define        MSC_MAX_SIZE    8000000
uint32_t MCS_FPGAID = -1;	/* FPGA ID defined in the MCS file */
uint32_t MSC_arraySize = 0;	/* Size of the array holding the firmware */
unsigned char MSC_ARRAY[MSC_MAX_SIZE];	/* The array holding the firmware */
char *MSC_filename_LX110 = "LX110_firmware.dat";	/* Default firmware for LX110 */
char *MSC_filename_FX70T = "FX70T_firmware.dat";	/* Default firmware for FX70T */
int MSC_loaded = 0;		/* 1(0) if firmware loaded (not loaded) */
VOIDFUNCPTR faV3UpdateWatcherRoutine = NULL;
faV3UpdateWatcherArgs_t faV3UpdateWatcherArgs;

/*************************************************************
 * faV3FirmwareLoad
 *   - main routine to load up firmware for FADC with specific id
 *
 *  NOTE: Make call to
 *     faV3FirmwareSetFilename(...);
 *   if not using firmware from the default
 */

int
faV3FirmwareLoad(int id, int chip, int pFlag)
{
  faV3UpdateWatcherArgs_t updateArgs;
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  if(chip < 0 || chip > 2)
    {
      printf("%s: ERROR:  Invalid chip parameter %d\n", __func__, chip);
      return ERROR;
    }

  if(chip == 2)			/* Fix for discrepancy between Linux and vxWorks implementation */
    chip = FAV3_FIRMWARE_LX110;

  /* Perform a hardware and software reset */
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->reset, 0xFFFF);
  FAV3UNLOCK;
  taskDelay(60);

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  /* Check if FADC is Ready */
  if(faV3FirmwareTestReady(id, 60, pFlag) != OK)
    {
      printf("%s: ERROR: FADC %2d not ready after reset\n", __func__, id);
      return ERROR;
    }

  /* Check if FADC has write access to SRAM from U90 JTAG DIP-switches */
  if(faV3FirmwareCheckSRAM(id) != OK)
    {
      printf("%s: ERROR: FADC %2d not configured to read/write PROM.\n\tCheck U90 DIP Switches.\n",
	     __func__, id);
      updateArgs.step = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FADC %2d FAILED PROM READ/WRITE TEST\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Data to SRAM */
  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  faV3FirmwareDownloadConfigData(id);
  if(faV3FirmwareVerifyDownload(id) != OK)
    {
      printf("%s: ERROR: FADC %2d Failed data verification at SRAM\n",
	     __func__, id);
      return ERROR;
    }


  /* SRAM TO PROM */
  taskDelay(1);
  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading PROM with SRAM data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  FAV3LOCK;
  if(chip == FAV3_FIRMWARE_LX110)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_SRAM_TO_PROM1);
  else if(chip == FAV3_FIRMWARE_FX70T)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_SRAM_TO_PROM2);
  FAV3UNLOCK;
  taskDelay(1);

  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout SRAM -> PROM\n",
	     __func__, id);
      return ERROR;
    }

  /* PROM TO SRAM (For verification) */
  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with PROM data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  faV3FirmwareZeroSRAM(id);

  FAV3LOCK;
  if(chip == FAV3_FIRMWARE_LX110)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_PROM1_TO_SRAM);
  else if(chip == FAV3_FIRMWARE_FX70T)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_PROM2_TO_SRAM);
  FAV3UNLOCK;
  taskDelay(1);

  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
	     __func__, id);
      return ERROR;
    }

  /* Compare SRAM to Data Array */
  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Verifying data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  if(faV3FirmwareVerifyDownload(id) != OK)
    {
      printf("%s: ERROR: FADC %d PROM data not verified\n", __func__, id);
      return ERROR;
    }

  /* PROM to FPGA (Reboot FPGA) */
  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Rebooting FPGA \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  FAV3LOCK;
  if(chip == FAV3_FIRMWARE_LX110)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_REBOOT_FPGA1);
  else if(chip == FAV3_FIRMWARE_FX70T)
    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_REBOOT_FPGA2);
  FAV3UNLOCK;
  taskDelay(1);

  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> FPGA\n",
	     __func__, id);
      return ERROR;
    }

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
  faV3FirmwareUpdateWatcher(updateArgs);

  return OK;

}

static uint32_t passed[FAV3_MAX_BOARDS + 1], stepfail[FAV3_MAX_BOARDS + 1];

int
faV3FirmwarePassedMask()
{
  uint32_t retMask = 0;
  int id, ifadc;

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id] == 1)
	retMask |= (1 << id);
    }

  return retMask;
}

/*************************************************************
 * faV3FirmwareGLoad
 *   - load up firmware for all initialized modules
 *
 *  NOTE: Make call to
 *     faV3FirmwareSetFilename(...);
 *   if not using firmware from the default
 */
int
faV3FirmwareGLoad(int chip, int pFlag)
{
  int ifadc = 0, id = 0, step = 0;
  faV3UpdateWatcherArgs_t updateArgs;

  /*   uint32_t passed[FAV3_MAX_BOARDS+1], stepfail[FAV3_MAX_BOARDS+1]; */

  if(chip < 0 || chip > 2)
    {
      printf("%s: ERROR:  Invalid chip parameter %d\n", __func__, chip);
      return ERROR;
    }

  if(chip == 2)			/* Fix for discrepancy between Linux and vxWorks implementation */
    chip = FAV3_FIRMWARE_LX110;

  /* Perform a hardware and software reset */
  step = 0;
  FAV3LOCK;
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
	{
	  printf("%s: ERROR : ADC in slot %d is not initialized \n",
		 __func__, id);
	  passed[id] = 0;
	  stepfail[id] = step;
	}
      else
	{
	  passed[id] = 1;
	  vmeWrite32(&FAV3p[id]->reset, 0xFFFF);
	}
    }
  FAV3UNLOCK;
  taskDelay(60);

  /* Check if FADC is Ready */
  step = 1;

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(faV3FirmwareTestReady(id, 60, pFlag) != OK)
	{
	  printf("%s: ERROR: FADC %2d not ready after reset\n", __func__, id);
	  passed[id] = 0;
	  stepfail[id] = step;
	}
    }

  /* Check if FADC has write access to SRAM from U90 JTAG DIP-switches */
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(faV3FirmwareCheckSRAM(id) != OK)
	{
	  printf("%s: ERROR: FADC %2d not configured to read/write PROM.\n\tCheck U90 DIP Switches.\n",
		 __func__, id);
	  passed[id] = 0;
	  stepfail[id] = step;
	  updateArgs.step = FAV3_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title,
		  "ERROR: FADC %2d FAILED PROM READ/WRITE TEST\n", id);
	  faV3FirmwareUpdateWatcher(updateArgs);
	}
    }


  /* Data to SRAM */
  step = 2;

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading SRAM with data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];

      updateArgs.step = FAV3_ARGS_SHOW_ID;
      updateArgs.id = id;
      faV3FirmwareUpdateWatcher(updateArgs);

      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  faV3FirmwareDownloadConfigData(id);

	  if(faV3FirmwareVerifyDownload(id) != OK)
	    {
	      printf("%s: ERROR: FADC %2d Failed data verification at SRAM\n",
		     __func__, id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	  else
	    {
	      updateArgs.step = FAV3_ARGS_SHOW_DONE;
	      faV3FirmwareUpdateWatcher(updateArgs);
	    }
	}
    }

  /* SRAM TO PROM */
  step = 3;
  taskDelay(1);

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Loading PROM with SRAM data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  FAV3LOCK;
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  if(chip == FAV3_FIRMWARE_LX110)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_SRAM_TO_PROM1);
	  else if(chip == FAV3_FIRMWARE_FX70T)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_SRAM_TO_PROM2);
	}
    }
  FAV3UNLOCK;
  taskDelay(1);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout SRAM -> PROM\n",
		     __func__, id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  /* PROM TO SRAM (For verification) */
  step = 4;

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  faV3FirmwareUpdateWatcher(updateArgs);
  sprintf(updateArgs.title, "Loading SRAM with PROM data \n");

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  faV3FirmwareZeroSRAM(id);
	  FAV3LOCK;
	  if(chip == FAV3_FIRMWARE_LX110)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_PROM1_TO_SRAM);
	  else if(chip == FAV3_FIRMWARE_FX70T)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_PROM2_TO_SRAM);
	  FAV3UNLOCK;
	}
    }

  taskDelay(1);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
		     __func__, id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  /* Compare SRAM to Data Array */
  step = 5;

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Verifying data \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];

      updateArgs.step = FAV3_ARGS_SHOW_ID;
      updateArgs.id = id;
      faV3FirmwareUpdateWatcher(updateArgs);

      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareVerifyDownload(id) != OK)
	    {
	      printf("%s: ERROR: FADC %d PROM data not verified\n",
		     __func__, id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	  else
	    {
	      updateArgs.step = FAV3_ARGS_SHOW_DONE;
	      faV3FirmwareUpdateWatcher(updateArgs);
	    }
	}
    }

  /* PROM to FPGA (Reboot FPGA) */
  step = 6;

  updateArgs.step = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Rebooting FPGA \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  FAV3LOCK;
	  if(chip == FAV3_FIRMWARE_LX110)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_REBOOT_FPGA1);
	  else if(chip == FAV3_FIRMWARE_FX70T)
	    vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_REBOOT_FPGA2);
	  FAV3UNLOCK;
	}
    }
  taskDelay(1);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareTestReady(id, 60000, pFlag) != OK)	/* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout PROM -> FPGA\n",
		     __func__, id);
	      passed[id] = 0;
	      stepfail[id] = step;
	    }
	}
    }

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id])		/* Skip the ones that have previously failed */
	{
	  updateArgs.step = FAV3_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
	  faV3FirmwareUpdateWatcher(updateArgs);
	}
      else
	{
	  printf("%s: FAILED programming FADC %2d at step %d\n",
		 __func__, id, stepfail[id]);
	}
    }

  return OK;

}

void
faV3FirmwareDownloadConfigData(int id)
{
  uint32_t ArraySize;
  uint32_t ByteCount, ByteIndex, ByteNumber;
  uint32_t Word32Bits;
#ifdef DEBUG
  uint32_t value;
#endif

  if(MSC_loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n", __func__);
      return;
    }

  ArraySize = MSC_arraySize;
  ByteIndex = 0;

  /* write SRAM address register */
  /* start at 0 and increment address after write to mem1 data register */
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->mem_adr, 0x80000000);
#ifdef DEBUG
  value = vmeRead32(&FAV3p[id]->mem_adr);
#endif
  FAV3UNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of writes = 0x%08x\n\n",
	 __func__, id, value);
#endif
  taskDelay(1);			/* wait */

/*   printf("Download Config Data... \n"); */
  for(ByteCount = 0; ByteCount < ArraySize; ByteCount += 4)
    {
      Word32Bits = 0;
      for(ByteNumber = 0; ByteNumber < 4; ++ByteNumber)
	{
	  Word32Bits =
	    (MSC_ARRAY[ByteIndex] << (8 * ByteNumber)) | Word32Bits;
	  ++ByteIndex;
	  if(ByteIndex > MSC_MAX_SIZE)
	    printf("**** TOO BIG! ****\n");
	}

      /* write 32-bit data word to  mem1 data register */
      FAV3LOCK;
      vmeWrite32(&FAV3p[id]->mem1_data, Word32Bits);
      FAV3UNLOCK;
    }

#ifdef DEBUG
  FAV3LOCK;
  value = vmeRead32(&FAV3p[id]->mem_adr);
  FAV3UNLOCK;
  printf("%s: FADC %2d memory address after write = 0x%08x\n\n",
	 __func__, id, value);
#endif
  taskDelay(1);			/* wait */

}


int
faV3FirmwareVerifyDownload(int id)
{
  uint32_t ArraySize;
  uint32_t ByteCount, ByteIndex, ByteNumber;
  uint32_t ExpWord32Bits, RdWord32Bits;
  int ErrorCount = 0, stopPrint = 0;
#ifdef DEBUG
  uint32_t value;
#endif

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  if(MSC_loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n", __func__);
      return ERROR;
    }

  ArraySize = MSC_arraySize;
  ByteIndex = 0;

  /* write SRAM address register */
  /* start at 0 and increment address after read from mem1 data register */
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->mem_adr, 0x0000 | FAV3_MEM_ADR_INCR_MEM1);
#ifdef DEBUG
  value = vmeRead32(&FAV3p[id]->mem_adr);
#endif
  FAV3UNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of read = 0x%08x\n\n",
	 __func__, id, value);
#endif
  taskDelay(1);			/* wait */

  for(ByteCount = 0; ByteCount < ArraySize; ByteCount += 4)
    {
      /* get expected value */
      ExpWord32Bits = 0;
      for(ByteNumber = 0; ByteNumber < 4; ++ByteNumber)
	{
	  ExpWord32Bits =
	    (MSC_ARRAY[ByteIndex] << (8 * ByteNumber)) | ExpWord32Bits;
	  ++ByteIndex;
	}

      /* read 32-bit data word from mem1 data register */
      FAV3LOCK;
      RdWord32Bits = (uint32_t) vmeRead32(&FAV3p[id]->mem1_data);
      FAV3UNLOCK;

#ifdef DEBUG
      if(ByteCount < 40)
	printf("RdWord32Bits = 0x%08x\n", RdWord32Bits);
#endif

      /* test if read value = expected value */
      if(RdWord32Bits != ExpWord32Bits)
	{
	  ErrorCount++;
	  if(!stopPrint)
	    printf("%s: ERROR: FADC %2d ByteCount %8d  Expect %08X  Read %08X\n",
		   __func__, id, ByteCount, ExpWord32Bits, RdWord32Bits);
	  if(ErrorCount == 2)
	    {
	      printf("%s: Further errors for FADC %2d will not be displayed\n",
		     __func__, id);
/* 	      getchar(); */
	      stopPrint = 1;
	    }
/* 	  if( ErrorCount>1000 ) */
/* 	    stopPrint=1; */
	}
    }

#ifdef DEBUG
  FAV3LOCK;
  value = vmeRead32(&FAV3p[id]->mem_adr);
  FAV3UNLOCK;
  printf("%s: memory address after read = 0x%08x\n\n", __func__, value);
#endif
  if(ErrorCount)
    printf("%s: ErrorCount = %d\n", __func__, ErrorCount);
  taskDelay(1);			/* wait */

  if(ErrorCount)
    return ERROR;

  return OK;
}

int
faV3FirmwareTestReady(int id, int n_try, int pFlag)
{
  int ii;
  int result;
  faV3UpdateWatcherArgs_t updateArgs;
  uint32_t value = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  result = ERROR;

  updateArgs.step = FAV3_ARGS_SHOW_ID;
  updateArgs.id = id;
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ii = 0; ii < n_try; ii++)	/* poll for ready bit */
    {
      updateArgs.step = FAV3_ARGS_SHOW_PROGRESS;
      faV3FirmwareUpdateWatcher(updateArgs);

      taskDelay(1);		/* wait */
      FAV3LOCK;
      value = vmeRead32(&FAV3p[id]->prom_reg1);
      FAV3UNLOCK;

      if(value == 0xFFFFFFFF)
	continue;

      if(value & FAV3_PROMREG1_READY)
	{
	  result = OK;
	  break;
	}
    }
  updateArgs.step = FAV3_ARGS_SHOW_DONE;
  faV3FirmwareUpdateWatcher(updateArgs);

  if(pFlag)
    {
      if(ii == n_try)		/* failed to detect ready asserted */
	printf("%s: FADC %2d NOT READY after %d wait cycles (1/60 sec)\n",
	       __func__, id, n_try);
      else
	printf("%s: FADC %2d READY after %d wait cycles (1/60 sec)\n",
	       __func__, id, (ii + 1));
    }

  return result;
}

int
faV3FirmwareZeroSRAM(int id)
{
  int ii, value = 0, value_1 = 0, value_2 = 0;
  int ErrorCount = 0, stopPrint = 0;

  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  /* set address = 0; allow increment on mem2 access */
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->mem_adr, 0x0000 | FAV3_MEM_ADR_INCR_MEM2);

  for(ii = 0; ii < 0x80000; ii++)	/* write ZERO to entire memory */
    {
      vmeWrite32(&FAV3p[id]->mem1_data, 0);
      vmeWrite32(&FAV3p[id]->mem2_data, 0);
    }

  /* reset address = 0; allow increment on mem2 access */
  vmeWrite32(&FAV3p[id]->mem_adr, 0x0000 | FAV3_MEM_ADR_INCR_MEM2);

  FAV3UNLOCK;

  /* read and test expected memory data */
  for(ii = 0; ii < 0x80000; ii++)
    {
      FAV3LOCK;
      value_1 = vmeRead32(&FAV3p[id]->mem1_data);
      value_2 = vmeRead32(&FAV3p[id]->mem2_data);
      FAV3UNLOCK;

      if((value_1 != 0) || (value_2 != 0))
	{
	  ErrorCount++;
	  FAV3LOCK;
	  value = vmeRead32(&FAV3p[id]->mem_adr) & 0xFFFFF;
	  FAV3UNLOCK;
	  if(!stopPrint)
	    {
	      printf("%s: ERROR: FADC %2d  address = %8X    mem1 read = %8X    mem2 read = %8X\n",
		     __func__, id, value, value_1, value_2);
	      taskDelay(1);	/* wait */
	    }
	  if(ErrorCount == 1)
	    {
	      printf("%s: Further errors for FADC %2d will not be displayed\n",
		     __func__, id);
	      stopPrint = 1;
	    }
	}
    }

  if(ErrorCount)
    return ERROR;

  return OK;
}

int
faV3FirmwareCheckSRAM(int id)
{
  int NonZeroCount = 0;
  uint32_t ByteCount;
  uint32_t RdWord32Bits;
#ifdef DEBUG
  uint32_t value;
#endif
  if(id == 0)
    id = faV3ID[0];

  if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
    {
      printf("%s: ERROR : ADC in slot %d is not initialized \n",
	     __func__, id);
      return ERROR;
    }

  faV3FirmwareZeroSRAM(id);

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->prom_reg1, FAV3_PROMREG1_PROM2_TO_SRAM);
  FAV3UNLOCK;
  taskDelay(1);

  if(faV3FirmwareTestReady(id, 60000, 0) != OK)	/* Wait til it's done */
    {
      printf("%s: ERROR: FADC %2d ready timeout PROM -> SRAM\n",
	     __func__, id);
      return ERROR;
    }

  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->mem_adr, 0x0000 | FAV3_MEM_ADR_INCR_MEM1);
#ifdef DEBUG
  value = vmeRead32(&FAV3p[id]->mem_adr);
#endif
  FAV3UNLOCK;
#ifdef DEBUG
  printf("%s: FADC %2d memory address at start of read = 0x%08x\n\n",
	 __func__, id, value);
#endif
  taskDelay(1);			/* wait */

  for(ByteCount = 0; ByteCount < MSC_arraySize; ByteCount += 4)
    {
      FAV3LOCK;
      RdWord32Bits = (uint32_t) vmeRead32(&FAV3p[id]->mem1_data);
      FAV3UNLOCK;

      if(RdWord32Bits != 0)
	{
	  NonZeroCount++;
	  break;
	}
    }

  if(NonZeroCount == 0)
    {
      return ERROR;
    }

  return OK;
}

void
faV3FirmwareSetFilename(char *filename, int chip)
{
  if(chip == FAV3_FIRMWARE_LX110)
    MSC_filename_LX110 = filename;
  else if(chip == FAV3_FIRMWARE_FX70T)
    MSC_filename_FX70T = filename;

}


int
faV3FirmwareReadFile(char *filename)
{
  uint32_t arraySize;

/* #define DEBUGFILE */
#ifdef DEBUGFILE
  int ichar = 0;
#endif
  FILE *arrayFile = NULL;
  arrayFile = fopen(filename, "r");

  if(arrayFile == NULL)
    {
      printf("%s: ERROR opening file (%s) for reading\n", __func__, filename);
      return ERROR;
    }

  /* First 32bits is the size of the array */
  fread(&arraySize, sizeof(uint32_t), 1, arrayFile);

#ifdef VXWORKS
  /* Made this file in Linux... so byte swap it for VXWORKS */
  MSC_arraySize = LONGSWAP(arraySize);
#else
  MSC_arraySize = arraySize;
#endif

  if(MSC_arraySize > MSC_MAX_SIZE)
    {
      printf("%s: ERROR: Firmware size (%d) from %s greater than MAX allowed (%d)\n",
	     __func__, MSC_arraySize, filename, MSC_MAX_SIZE);
      return ERROR;
    }


#ifdef DEBUGFILE
  printf("MSC_arraySize = %d\n", MSC_arraySize);
#endif

  fread(&MSC_ARRAY, MSC_arraySize, 1, arrayFile);

  fclose(arrayFile);

#ifdef DEBUGFILE
  for(ichar = 0; ichar < 16 * 10; ichar++)
    {
      if((ichar % 16) == 0)
	printf("\n");
      printf("0x%02x ", MSC_ARRAY[ichar]);
    }
  printf("\n\n");
#endif
  MSC_loaded = 1;

  printf("%s: Reading Firmware from %s\n", __func__, filename);

  return OK;
}

int
hex2num(char c)
{
  c = toupper(c);

  if(c > 'F')
    return 0;

  if(c >= 'A')
    return 10 + c - 'A';

  if((c > '9') || (c < '0'))
    return 0;

  return c - '0';
}

int
faV3FirmwareGetFpgaID(int pflag)
{
  if(pflag)
    {
      printf("%s: FPGA ID = 0x%x (%d)\n", __func__, MCS_FPGAID, MCS_FPGAID);
    }

  return MCS_FPGAID;
}

int
faV3FirmwareChipFromFpgaID(int pflag)
{
  int rval = 0;
  uint32_t id;

  id = MCS_FPGAID & FAV3_FPGAID_MASK;

  switch (id)
    {
    case FAV3_FPGAID_PROC:
      rval = FAV3_FIRMWARE_LX110;
      break;
    case FAV3_FPGAID_CTRL:
      rval = FAV3_FIRMWARE_FX70T;
      break;
    case -1:
    default:
      rval = ERROR;
    }

  if(pflag)
    {
      if(rval != ERROR)
	{
	  printf("%s: ID = 0x%08x FPGA Chip = %d\n",
		 __func__, MCS_FPGAID, rval);
	}
      else
	{
	  printf("%s: ID (0x%08x) does not match any available FPGAs for this program\n",
		 __func__, MCS_FPGAID);
	}
    }

  return rval;
}

int
faV3FirmwareRevFromFpgaID(int pflag)
{
  int rval = 0;

  rval = MCS_FPGAID & FAV3_FPGAID_REV_MASK;

  if(pflag)
    {
      printf("%s: Rev = 0x%x\n", __func__, rval);
    }
  return rval;
}

int
faV3FirmwareReadMcsFile(char *filename)
{
  FILE *mscFile = NULL;
  char ihexLine[200], *pData;
  int len = 0, datalen = 0, byte_shift;
  uint32_t nbytes = 0, line = 0, hiChar = 0, loChar = 0;
  uint32_t readMSC = 0;
#ifdef DEBUGFILE
  int ichar, thisChar[0];
#endif

  mscFile = fopen(filename, "r");
  if(mscFile == NULL)
    {
      perror("fopen");
      printf("%s: ERROR opening file (%s) for reading\n", __func__, filename);
      return ERROR;
    }

  while(!feof(mscFile))
    {
      /* Get the current line */
      if(!fgets(ihexLine, sizeof(ihexLine), mscFile))
	break;

      /* Get the the length of this line */
      len = strlen(ihexLine);

      if(len >= 5)
	{
	  /* Check for the start code */
	  if(ihexLine[0] != ':')
	    {
	      printf("%s: ERROR parsing file at line %d\n", __func__, line);
	      return ERROR;
	    }

	  /* Get the byte count */
	  hiChar = hex2num(ihexLine[1]);
	  loChar = hex2num(ihexLine[2]);
	  datalen = (hiChar) << 4 | loChar;

	  if(strncmp("00", &ihexLine[7], 2) == 0)	/* Data Record */
	    {
	      pData = &ihexLine[9];	/* point to the beginning of the data */
	      while(datalen--)
		{
		  hiChar = hex2num(*pData++);
		  loChar = hex2num(*pData++);
		  MSC_ARRAY[readMSC] = ((hiChar) << 4) | (loChar);
		  if(readMSC >= MSC_MAX_SIZE)
		    {
		      printf("%s: ERROR: TOO BIG!\n", __func__);
		      return ERROR;
		    }
		  readMSC++;
		  nbytes++;
		}
	    }
	  else if(strncmp("01", &ihexLine[7], 2) == 0)	/* End of File, contains FPGA ID */
	    {
	      byte_shift = 24;
	      MCS_FPGAID = 0;
	      pData = &ihexLine[9];	/* point to the beginning of the data */
	      while(datalen--)
		{
		  if(byte_shift < 0)
		    {
		      printf("%s: ERROR: FPGA ID too large!\n", __func__);
		      return ERROR;
		    }
		  hiChar = hex2num(*pData++);
		  loChar = hex2num(*pData++);

		  MCS_FPGAID |= ((hiChar << 4) | (loChar)) << byte_shift;
#ifdef DEBUGFILE
		  printf("%2d: MCS_FPGAID = 0x%08x\n", datalen, MCS_FPGAID);
#endif
		  byte_shift -= 8;
		}
	    }
	}
      line++;
    }

  MSC_arraySize = readMSC;

#ifdef DEBUGFILE
  printf("MSC_arraySize = %d\n", MSC_arraySize);

  for(ichar = 0; ichar < 16 * 10; ichar++)
    {
      if((ichar % 16) == 0)
	printf("\n");
      printf("0x%02x ", MSC_ARRAY[ichar]);
    }
  printf("\n\n");
#endif
  MSC_loaded = 1;

  fclose(mscFile);
  return OK;
}

int
faV3FirmwareAttachUpdateWatcher(VOIDFUNCPTR routine,
				faV3UpdateWatcherArgs_t arg)
{

  if(routine)
    {
      faV3UpdateWatcherRoutine = routine;
      faV3UpdateWatcherArgs = arg;
    }
  else
    {
      faV3UpdateWatcherRoutine = NULL;
      memset(&faV3UpdateWatcherArgs, 0, sizeof(faV3UpdateWatcherArgs_t));
    }

  return OK;
}

void
faV3FirmwareUpdateWatcher(faV3UpdateWatcherArgs_t arg)
{
  faV3UpdateWatcherArgs_t rArg;
  static int step1_ticks = 0;

  if((arg.step >= FAV3_ARGS_SHOW_ID) && (arg.step < FAV3_ARGS_LAST))
    rArg = arg;
  else
    rArg = faV3UpdateWatcherArgs;

  if(faV3UpdateWatcherRoutine != NULL)
    {
      (*faV3UpdateWatcherRoutine) (rArg);
    }
  else
    {
      switch (rArg.step)
	{
	case FAV3_ARGS_SHOW_ID:
	  step1_ticks = 0;
	  printf("%2d: ", arg.id);
	  fflush(stdout);
	  break;

	case FAV3_ARGS_SHOW_PROGRESS:
	  step1_ticks++;
	  if((step1_ticks % 180) == 0)
	    {
	      printf("+");
	      fflush(stdout);
	    }
	  break;

	case FAV3_ARGS_SHOW_DONE:
	  step1_ticks = 0;
	  printf(" Done\n");
	  fflush(stdout);
	  break;

	case FAV3_ARGS_SHOW_STRING:
	  printf("%s", arg.title);
	  fflush(stdout);
	  break;

	}

    }
}
