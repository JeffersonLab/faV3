/* Module: faV3FirmwareTools.c
 *
 * Description: FADC250 Firmware Tools Library
 *              Firmware specific functions.
 *
 * Author:
 *        Bryan Moffit
 *        JLab Data Acquisition Group
 *
 */

#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <ctype.h>
#include "faV3Lib.h"
#include "faV3FirmwareTools.h"

extern pthread_mutex_t faV3Mutex;

#define FAV3LOCK      if(pthread_mutex_lock(&faV3Mutex)<0) perror("pthread_mutex_lock");
#define FAV3UNLOCK    if(pthread_mutex_unlock(&faV3Mutex)<0) perror("pthread_mutex_unlock");

extern int32_t nfaV3;
extern int32_t faV3ID[FAV3_MAX_BOARDS];
extern volatile faV3_t *FAV3p[(FAV3_MAX_BOARDS + 1)];	/* pointers to FAV3 memory map */

#define CHECKID	{							\
    if(id == 0) id = faV3ID[0];						\
    if((id <= 0) || (id > 21) || (FAV3p[id] == NULL)) {			\
      printf("%s: ERROR : ADC in slot %d is not initialized \n", __func__, id); \
      return ERROR; }}

#define WAITFORREADY {							\
    int32_t ready_ret = faV3FirmwareWaitForReady(id, FAV3_FIRMWARE_WAIT); \
    if(ready_ret == ERROR) {						\
      printf("%s: ERROR: Timeout\n", __func__);				\
      return ERROR;							\
    } else {								\
      printf("%s: INFO: Ready after %d \n", __func__, ready_ret);	\
    }}

const uint32_t ConfigRomHostEndOfCmd = 0x0100;
const uint32_t ConfigRomHostExec = 0x0200;
const uint8_t ConfigRom_RDID = 0x9F;  // Read ROM Identification registers For S256 the bytes are 0x0102194D
const uint8_t ConfigRom_RDSR1 = 0x05;  // Read status register 1
const uint8_t ConfigRom_WRDI = 0x04;  // Disable write to ROM non-Volatile memory
const uint8_t ConfigRom_WREN = 0x06;  // Enable write to ROM non-Volatilde memory
const uint8_t ConfigRom_BE = 0x60;   // BULK Erase
const uint8_t ConfigRom_PP4 = 0x12;   // Write to ROM volatalile memory. Content store to ROM on rising edge of ROM CSN
const uint8_t ConfigRom_AREAD = 0X13;   // Read FLASH Array Low memory then higher memory
const uint32_t ConfigRom_SR1V_WEL   = 2; // Write Enable Latch 1= Can write to memory, registers
const uint32_t ConfigRom_SR1V_WIP   = 1; // 1 = Write in progress
const uint32_t ConfigRomReadyForCommand = 0x2;

#define MAX_MCS_SIZE 8000000
#define FAV3_FIRMWARE_WAIT 200

#define VME_CONFIG6_ADR &FAV3p[id]->config_rom_control0
#define VME_CONFIG7_ADR &FAV3p[id]->config_rom_control1
#define VME_CONFIG8_ADR &FAV3p[id]->config_rom_control2
#define VME_STATUS5_ADR &FAV3p[id]->config_rom_status0

typedef struct {
  char filename[256];
  int32_t loaded;
  uint32_t size;
  uint32_t data[MAX_MCS_SIZE];
} firmware_t;

firmware_t *file_firmware = NULL;
firmware_t *rom_firmware = NULL;

VOIDFUNCPTR faV3UpdateWatcherRoutine = NULL;
faV3UpdateWatcherArgs_t faV3UpdateWatcherArgs;


/**
 * @brief Wait for config rom to report ReadyForCmd
 * @param[in] id faV3 slot ID
 * @param[in] nwait Max number of tries
 * @return If ready, number of tries before ready, otherwise ERROR.
 */
int32_t
faV3FirmwareWaitForReady(int32_t id, int32_t nwait)
{
  int32_t rval = OK;
  uint32_t regval = 0, iwait = 0;
  CHECKID;

  while( ((regval & ConfigRomReadyForCommand)==0) && (iwait++ < nwait) )
    {
      FAV3LOCK;
      regval = vmeRead32(&FAV3p[id]->config_rom_status1);
      FAV3UNLOCK;
    }

  if(regval & ConfigRomReadyForCommand)
    {
      rval = iwait;
    }
  else
    {
      printf("%s: ERROR:  timeout after %d tries\n",
	     __func__, iwait);
      rval = ERROR;
    }

  return rval;
}

/**
 * @brief Read ROM ID from ROM Chip
 * @param[in] id faV3 Slot ID
 * @return ROM ID, if successful.
 */

uint32_t
faV3FirmwareRomID(int32_t id)
{
  uint32_t rval = 0;
  uint32_t cmd, romadr, romdata = 0;
  CHECKID;

  WAITFORREADY;
  cmd = ConfigRomHostEndOfCmd | ConfigRom_RDID;

  FAV3LOCK;

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command

  rval = vmeRead32(VME_STATUS5_ADR);
  FAV3UNLOCK;

  return rval;
}

/**
 * @brief Read ROM STATUS1 register from ROM Chip
 * @param[in] id faV3 Slot ID
 * @return ROM STATUS1, if successful.
 */

uint32_t
faV3FirmwareRomStatus1(int32_t id)
{
  uint32_t rval;
  uint32_t cmd, romdata = 0;
  CHECKID;

  WAITFORREADY;
  cmd = ConfigRomHostEndOfCmd | ConfigRom_RDSR1;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Rom Command

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command

  rval = vmeRead32(VME_STATUS5_ADR);
  FAV3UNLOCK;

  return rval;
}

/**
 * @brief Enable or Disable Writing to ROM
 * @param[in] id faV3 Slot ID
 * @param[in] enable
 *      0: disable
 *      1: enable
 * @return ROM Status1
 */

uint32_t
faV3FirmwareSetMemoryWrite(int32_t id, int32_t enable)
{
  uint32_t rval;
  uint32_t cmd, romdata = 0;
  CHECKID;

  WAITFORREADY;
  if(enable)
    cmd = ConfigRomHostEndOfCmd | ConfigRom_WREN;
  else
    cmd = ConfigRomHostEndOfCmd | ConfigRom_WRDI;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Rom Command

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command

  if(!enable)
    rval = vmeRead32(VME_STATUS5_ADR);
  FAV3UNLOCK;

  if(enable)
    rval = faV3FirmwareRomStatus1(id) & ConfigRom_SR1V_WEL;

  return rval;

}

/**
 * @brief Erase the ROM
 * @param[in] id faV3 Slot ID
 * @return OK if successful, otherwise ERROR
 */

int32_t
faV3FirmwareRomErase(int32_t id)
{
  uint32_t rval;
  uint32_t cmd, romdata = 0;
  CHECKID;

  rval = faV3FirmwareSetMemoryWrite(id, 1);
  if(rval == 0)
    {
      printf("%s: ERROR: Write not enabled\n",
	     __func__);
      return -1;
    }

  WAITFORREADY;
  cmd = ConfigRomHostEndOfCmd | ConfigRom_BE;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Rom Command

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command

  rval = vmeRead32(VME_STATUS5_ADR);
  FAV3UNLOCK;

  usleep(10);
  romdata = faV3FirmwareRomStatus1(id) & ConfigRom_SR1V_WIP;
  if(romdata == 0)
    {
      printf("%s: ERROR: Failed to erase Config ROM.  ROM Write-In-Progress bit not set\n",
	     __func__);
      return -1;
    }

  int32_t wait = 0, maxwait = 1300;
  while((wait++ < maxwait) && (romdata != 0))
    {
      usleep(100);
      romdata = faV3FirmwareRomStatus1(id) & ConfigRom_SR1V_WIP;
    }

  if(romdata != 0)
    {
      printf("%s: Failed to erase Config ROM.\n",
	     __func__);
      return -1;
    }
  else
    rval = 0;

  return rval;
}

/**
 * @brief Read the contents of the ROM at specified address
 * @param[in] id faV3 slot ID
 * @param[in] romadr ROM Address
 * @param[in] last Whether or not this is the end of the page
 * @return Data from ROM, if successful
 */

int32_t
faV3FirmwareReadRomAdr(int32_t id, uint32_t romadr, int32_t last)
{
  uint32_t rval;
  uint32_t cmd, romdata = 0;
  CHECKID;

  WAITFORREADY;
  cmd = ConfigRom_AREAD;
  if(last)
    cmd |= ConfigRomHostEndOfCmd;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Rom Command
  vmeWrite32(VME_CONFIG7_ADR, romadr);	// Rom Address

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command
  FAV3UNLOCK;

  WAITFORREADY;

  FAV3LOCK;
  rval = vmeRead32(VME_STATUS5_ADR);
  FAV3UNLOCK;

  return rval;
}

/**
 * @brief Write data to the ROM at specified address
 * @param[in] id faV3 slot ID
 * @param[in] romadr ROM Address
 * @param[in] romdata Data to write
 * @param[in] last Whether or not this is the end of the page
 * @return 0
 */

int32_t
faV3FirmwareWriteRomAdr(int32_t id, uint32_t romadr, uint32_t romdata, int32_t last)
{
  uint32_t cmd;
  CHECKID;

  WAITFORREADY;
  cmd = ConfigRom_PP4;
  if(last)
    cmd |= ConfigRomHostEndOfCmd;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Rom Command
  vmeWrite32(VME_CONFIG7_ADR, romadr);	// Rom Address
  vmeWrite32(VME_CONFIG8_ADR, romdata);	// Rom Data

  cmd = ConfigRomHostExec | cmd;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Execute Command

  cmd = cmd & ~ConfigRomHostExec;
  vmeWrite32(VME_CONFIG6_ADR, cmd);	// Ready for Next Command

  FAV3UNLOCK;

  return 0;
}

/**
 * @brief Download the contents of the ROM to local memory
 * @param[in] id faV3 slot ID
 * @param[in] size maximum size of local memory
 * @return OK if successful
 */

int32_t
faV3FirmwareDownloadRom(int32_t id, int32_t size)
{
  uint32_t rval = 0;
  int32_t idata = 0, iword = 0, last_of_page = 0;
  CHECKID;

  // FIXME: Not sure write enable is needed here
  rval = faV3FirmwareSetMemoryWrite(id, 1);
  if(rval == 0)
    {
      printf("%s: ERROR: Write not enabled\n",
	     __func__);
      return -1;
    }

  if(rom_firmware == NULL)
    rom_firmware = (firmware_t *)malloc(sizeof(firmware_t));

  rom_firmware->loaded = 0;

  while(idata < size)
    {
      if((idata & (256 >> 2)) == (256 >> 2))
	last_of_page = 1;
      else
	last_of_page = 0;

      rom_firmware->data[idata] = faV3FirmwareReadRomAdr(id, (idata << 2), last_of_page);

      idata++;
    }
  rom_firmware->data[idata] = 0xFFFFFFFF;

  rom_firmware->size = idata;
  rom_firmware->loaded = 1;

  return OK;
}

/**
 * @brief Program the ROM with the firmware in local memory
 * @param[in] id faV3 slot ID
 * @return OK if successfull
 */
int32_t
faV3FirmwareProgramRom(int32_t id)
{
  uint32_t rval = 0;
  int32_t idata = 0, iword = 0, last_of_page = 0;
  CHECKID;
  if(file_firmware->loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n", __func__);
      return ERROR;
    }

  rval = faV3FirmwareSetMemoryWrite(id, 1);
  if(rval == 0)
    {
      printf("%s: ERROR: Write not enabled\n",
	     __func__);
      return -1;
    }

  while(idata < file_firmware->size)
    {
      if((idata & (256 >> 2)) == (256 >> 2))
	last_of_page = 1;
      else
	last_of_page = 0;

      faV3FirmwareWriteRomAdr(id, (idata << 2), file_firmware->data[idata], last_of_page);

      idata++;
    }
  faV3FirmwareWriteRomAdr(id, (idata << 2), 0xFFFFFFFF, 1);

  return OK;
}

/**
 * @brief Compare the local memory contents from FILE and ROM
 * @return OK if successful, otherwise ERROR
 */

int32_t
faV3FirmwareVerifyDownload()
{
  if(file_firmware->loaded != 1)
    {
      printf("%s: ERROR : File Firmware was not loaded\n", __func__);
      return ERROR;
    }
  if(rom_firmware->loaded != 1)
    {
      printf("%s: ERROR : ROM Firmware was not loaded\n", __func__);
      return ERROR;
    }

  if(file_firmware->size != rom_firmware->size)
    {
      printf("%s: ERROR: File size != Rom size (0x%x != 0x%x)\n",
	     __func__, file_firmware->size, rom_firmware->size);
      return ERROR;
    }

  int32_t idata = 0, errorCount = 0;
  while(idata < file_firmware->size)
    {
      if(file_firmware->data[idata] != rom_firmware->data[idata])
	{
	  errorCount++;
	  if(errorCount == 1)
	    {
	      printf("%s: ERROR: word 0x%x  File 0x%08x  ROM 0x%08x\n",
		     __func__, idata, file_firmware->data[idata], rom_firmware->data[idata]);
	    }
	}
      idata++;
    }

  if(errorCount)
    {
      printf("%s: errorCount = 0x%x (%d)\n", __func__, errorCount, errorCount);
      return ERROR;
    }

  return OK;
}



static uint32_t passed[FAV3_MAX_BOARDS + 1], stepfail[FAV3_MAX_BOARDS + 1];

int
faV3FirmwarePassedMask()
{
  uint32_t retMask = 0;
  int32_t id, ifadc;

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(passed[id] == 1)
	retMask |= (1 << id);
    }

  return retMask;
}

#ifdef OLDCODE
/*************************************************************
 * faV3FirmwareLoad
 *   - main routine to load up firmware for FADC with specific id
 *
 *  NOTE: Make call to
 *     faV3FirmwareSetFilename(...);
 *   if not using firmware from the default
 */

int
faV3FirmwareLoad(int32_t id, int32_t pFlag)
{
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

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

/*************************************************************
 * faV3FirmwareGLoad
 *   - load up firmware for all initialized modules
 *
 *  NOTE: Make call to
 *     faV3FirmwareSetFilename(...);
 *   if not using firmware from the default
 */
int
faV3FirmwareGLoad(int32_t pFlag)
{
  int32_t ifadc = 0, id = 0, step = 0;
  faV3UpdateWatcherArgs_t updateArgs;

  /*   uint32_t passed[FAV3_MAX_BOARDS+1], stepfail[FAV3_MAX_BOARDS+1]; */

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
#endif

int
faV3FirmwareReadFile(char *filename)
{
  FILE *firmwareFD = fopen(filename, "r");

  if(firmwareFD == NULL)
    {
      printf("%s: ERROR opening file (%s) for reading\n", __func__, filename);
      return ERROR;
    }

  file_firmware = (firmware_t *)malloc(sizeof(firmware_t));

  strcpy(file_firmware->filename, filename);

  uint32_t idata = 0, data = 0;
  while(!feof(firmwareFD))
    {
      fread(&data, sizeof(uint32_t), 1, firmwareFD);
      file_firmware->data[idata++];
    }
  fclose(firmwareFD);

  file_firmware->size = idata;
  file_firmware->loaded = 1;

  printf("%s: Read Firmware from %s\n", __func__, file_firmware->filename);

  return OK;
}

int
faV3FirmwareWriteFile(char *filename)
{
  FILE *firmwareFD = fopen(filename, "w");

  if(firmwareFD == NULL)
    {
      printf("%s: ERROR opening file (%s) for writing\n", __func__, filename);
      return ERROR;
    }

  uint32_t idata = 0, data = 0;
  while(idata < rom_firmware->size)
    {
      data = rom_firmware->data[idata++];
      fwrite(&data, sizeof(uint32_t), 1, firmwareFD);
    }
  data = 0xffffffff;
  fwrite(&data, sizeof(uint32_t), 1, firmwareFD);
  fclose(firmwareFD);

  printf("%s: Wrote Firmware to %s\n", __func__, filename);

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

#ifdef OLDCODE
// FIXME: Needs modified for uint32_t elements from uint8_t
int
faV3FirmwareReadMcsFile(char *filename)
{
  FILE *mcsFile = NULL;
  char ihexLine[200], *pData;
  int32_t len = 0, datalen = 0, byte_shift;
  uint32_t nbytes = 0, line = 0, hiChar = 0, loChar = 0;
  uint32_t readMCS = 0;

  mcsFile = fopen(filename, "r");
  if(mcsFile == NULL)
    {
      perror("fopen");
      printf("%s: ERROR opening file (%s) for reading\n", __func__, filename);
      return ERROR;
    }

  file_firmware = (firmware_t *)malloc(sizeof(firmware_t));

  strcpy(file_firmware->filename, filename);

  while(!feof(mcsFile))
    {
      /* Get the current line */
      if(!fgets(ihexLine, sizeof(ihexLine), mcsFile))
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
		  file_firmware->data[readMCS] = ((hiChar) << 4) | (loChar);
		  if(readMCS >= MAX_MCS_SIZE)
		    {
		      printf("%s: ERROR: TOO BIG!\n", __func__);
		      return ERROR;
		    }
		  readMCS++;
		  nbytes++;
		}
	    }
	  else if(strncmp("01", &ihexLine[7], 2) == 0)	/* End of File, contains FPGA ID */
	    {
	    }
	}
      line++;
    }
  fclose(mcsFile);

  file_firmware->size = readMCS;
  file_firmware->loaded = 1;

  return OK;
}
#endif

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
  static int32_t step1_ticks = 0;

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
