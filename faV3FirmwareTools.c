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
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <ctype.h>
#include <byteswap.h>
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
    int32_t ready_ret = faV3FirmwareWaitForReady(id, FAV3_FIRMWARE_WAIT, 0); \
    if(ready_ret == ERROR) {						\
      printf("%s: ERROR: Timeout\n", __func__);				\
      return ERROR;							\
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
const uint32_t ConfigRomRebootFPGA = (1 << 11);

#define MAX_FW_SIZE 0x1800000
#define FAV3_FW_SIZE 0x1701DEC
#define FAV3_FIRMWARE_WAIT 200

#define VME_CONFIG6_ADR &FAV3p[id]->config_rom_control0
#define VME_CONFIG7_ADR &FAV3p[id]->config_rom_control1
#define VME_CONFIG8_ADR &FAV3p[id]->config_rom_control2
#define VME_STATUS5_ADR &FAV3p[id]->config_rom_status0

typedef struct {
  char filename[256];
  int32_t loaded;
  uint32_t size;
  uint32_t data[MAX_FW_SIZE>>2];
} faV3fw_t;

faV3fw_t *file_firmware = NULL;
faV3fw_t *rom_firmware = NULL;

VOIDFUNCPTR faV3UpdateWatcherRoutine = NULL;
faV3UpdateWatcherArgs_t faV3UpdateWatcherArgs;

/**
 * @brief Wait for config rom to report ReadyForCmd
 * @param[in] id faV3 slot ID
 * @param[in] nwait Max number of tries
 * @return If ready, number of tries before ready, otherwise ERROR.
 */
int32_t
faV3FirmwareWaitForReady(int32_t id, int32_t nwait, int32_t pflag)
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

  if(pflag)
    {
      printf("%s: INFO: Ready after %d tries\n",
	     __func__, iwait);
    }

  return rval;
}

/**
 * @brief Wait for config rom to report write in progress is done
 * @param[in] id faV3 slot ID
 * @param[in] nwait Max number of tries
 * @return If ready, number of tries before ready, otherwise ERROR.
 */
int32_t
faV3FirmwareWaitForWIP(int32_t id, int32_t nwait, int32_t pflag)
{
  int32_t rval = OK;
  uint32_t wipval = 0, iwait = 0;
  faV3UpdateWatcherArgs_t updateArgs;

  CHECKID;

  wipval = faV3FirmwareRomStatus1(id) & ConfigRom_SR1V_WIP;

  while((iwait++ < nwait) && (wipval != 0))
    {
      if(nwait > 1000)
	{
	  updateArgs.step = FAV3_UPDATE_STEP_ERASE;
	  updateArgs.show = FAV3_ARGS_SHOW_PROGRESS;
	  faV3FirmwareUpdateWatcher(updateArgs);
	}

      usleep(1000);
      wipval = faV3FirmwareRomStatus1(id) & ConfigRom_SR1V_WIP;
    }

  if(wipval == 0)
    {
      rval = iwait;
    }
  else
    {
      printf("%s: ERROR:  timeout after %d tries\n",
	     __func__, iwait);
      rval = ERROR;
    }

  if(nwait > 1000)
    {
      updateArgs.step = FAV3_UPDATE_STEP_ERASE;
      updateArgs.show = FAV3_ARGS_SHOW_DONE;
      faV3FirmwareUpdateWatcher(updateArgs);
    }

  if(pflag)
    {
      printf("%s: INFO: Ready after %d tries\n",
	     __func__, iwait);
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

  FAV3UNLOCK;

  WAITFORREADY;

  FAV3LOCK;
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
  FAV3UNLOCK;

  WAITFORREADY;

  FAV3LOCK;
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
  FAV3UNLOCK;

  WAITFORREADY;

  FAV3LOCK;
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
faV3FirmwareRomErase(int32_t id, int32_t waitforWIP)
{
  int32_t rval;
  uint32_t cmd;
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
  FAV3UNLOCK;

  if(waitforWIP)
    {
      rval = faV3FirmwareWaitForWIP(id, 200000, 0);
      if(rval < 0)
	{
	  printf("%s: Failed to erase Config ROM.  romstatus1 = 0x%08x\n",
		 __func__, faV3FirmwareRomStatus1(id));
	  return -1;
	}
      else
	rval = 0;
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
  int32_t rval = 0;
  uint32_t cmd;
  CHECKID;

  WAITFORREADY;

  if((romadr & 0xff) == 0)
    {
      rval = faV3FirmwareSetMemoryWrite(id, 1);
      if(rval == 0)
	{
	  printf("%s: ERROR: Write not enabled for romadr = 0x%x\n",
		 __func__, romadr);
	  return -1;
	}
    }

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

  if(last)
    {
      faV3FirmwareWaitForWIP(id, 20, 0);
    }

  return rval;
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
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

  if(rom_firmware == NULL)
    rom_firmware = (faV3fw_t *)malloc(sizeof(faV3fw_t));
  else
    memset(&rom_firmware->data, 0, (MAX_FW_SIZE>>2)*sizeof(uint32_t));

  rom_firmware->loaded = 0;

  updateArgs.step = FAV3_UPDATE_STEP_DOWNLOAD;
  updateArgs.show = FAV3_ARGS_SHOW_PROGRESS;
  while(idata < (size>>2))
    {
      if((idata & (256 >> 2)) == (256 >> 2))
	last_of_page = 1;
      else
	last_of_page = 0;

      rom_firmware->data[idata] = faV3FirmwareReadRomAdr(id, (idata << 2), last_of_page);

      idata++;

      faV3FirmwareUpdateWatcher(updateArgs);

    }

  updateArgs.show = FAV3_ARGS_SHOW_DONE;
  faV3FirmwareUpdateWatcher(updateArgs);

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
  uint32_t rval = 0, romadr = 0;
  int32_t idata = 0, iword = 0, last_of_page = 0;
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;
  if(file_firmware->loaded != 1)
    {
      printf("%s: ERROR : Firmware was not loaded\n", __func__);
      return ERROR;
    }

  updateArgs.step = FAV3_UPDATE_STEP_PROGRAM;
  updateArgs.show = FAV3_ARGS_SHOW_PROGRESS;
  while(idata < file_firmware->size)
    {
      romadr = idata << 2;
      if((romadr & 0xFC) == 0xFC)
	last_of_page = 1;
      else
	last_of_page = 0;

      faV3FirmwareWriteRomAdr(id, romadr, file_firmware->data[idata], last_of_page);

      idata++;

      faV3FirmwareUpdateWatcher(updateArgs);

    }
  romadr = idata << 2;
  faV3FirmwareWriteRomAdr(id, romadr, 0xFFFFFFFF, 1);

  updateArgs.show = FAV3_ARGS_SHOW_DONE;
  faV3FirmwareUpdateWatcher(updateArgs);

  return OK;
}

/**
 * @brief Compare the local memory contents from FILE and ROM
 * @return OK if successful, otherwise ERROR
 */

int32_t
faV3FirmwareCompare()
{
  faV3UpdateWatcherArgs_t updateArgs;

  if((file_firmware == NULL) || (file_firmware->loaded != 1))
    {
      printf("%s: ERROR : File Firmware was not loaded\n", __func__);
      return ERROR;
    }
  if((rom_firmware == NULL) || (rom_firmware->loaded != 1))
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
  updateArgs.step = FAV3_UPDATE_STEP_VERIFY;
  updateArgs.show = FAV3_ARGS_SHOW_PROGRESS;
  while(idata < file_firmware->size)
    {
      if(file_firmware->data[idata] != rom_firmware->data[idata])
	{
	  errorCount++;
	  if(errorCount < 16)
	    {
	      printf("%s: ERROR: word 0x%x  File 0x%08x  ROM 0x%08x\n",
		     __func__, idata, file_firmware->data[idata], rom_firmware->data[idata]);
	    }
	}
      idata++;
      faV3FirmwareUpdateWatcher(updateArgs);

    }

  if(errorCount)
    {
      printf("%s: errorCount = 0x%x (%d)\n", __func__, errorCount, errorCount);
      return ERROR;
    }

  updateArgs.show = FAV3_ARGS_SHOW_DONE;
  faV3FirmwareUpdateWatcher(updateArgs);

  return OK;
}

int32_t
faV3FirmwareReboot(int32_t id)
{
  int32_t rval = OK;
  CHECKID;

  FAV3LOCK;
  vmeWrite32(VME_CONFIG6_ADR, ConfigRomRebootFPGA);
  FAV3UNLOCK;
  return rval;
}

/**
 * @brief Wait for fav3 to reload firmware after reboot
 * @param[in] id faV3 slot ID
 * @param[in] nwait Max number of tries
 * @return If ready, number of tries before ready, otherwise ERROR.
 */
int32_t
faV3FirmwareWaitForReboot(int32_t id, int32_t nwait, int32_t pflag)
{
  int32_t rval = OK, res = 0;
  uint32_t rdata = -1, iwait = 0;
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

  updateArgs.step = FAV3_UPDATE_STEP_REBOOT;
  updateArgs.show = FAV3_ARGS_SHOW_PROGRESS;

  res = vmeMemProbe((char *) &FAV3p[id]->version, 4, (char *) &rdata);
  while( (res < 0) && (iwait++ < nwait) )
    {
      FAV3LOCK;
      res = vmeMemProbe((char *) &FAV3p[id]->version, 4, (char *) &rdata);
      FAV3UNLOCK;
      if(res >= 0)
	if(rdata == -1) res = -1;
      faV3FirmwareUpdateWatcher(updateArgs);
      usleep(1000);
    }

  if(res >= 0)
    {
      rval = iwait;
      updateArgs.show = FAV3_ARGS_SHOW_DONE;
      faV3FirmwareUpdateWatcher(updateArgs);
    }
  else
    {
      printf("%s: ERROR:  timeout after %d tries\n",
	     __func__, iwait);
      rval = ERROR;
    }

  if(pflag)
    {
      printf("%s: INFO: Ready after %d tries\n",
	     __func__, iwait);
    }

  return rval;
}


typedef struct
{
  int32_t skip;
  int32_t passed;
  int32_t stepfail;
} fwBoardUpdate_t;

static fwBoardUpdate_t fwStatus[FAV3_MAX_BOARDS + 1];

int
faV3FirmwarePassedMask()
{
  uint32_t retMask = 0;
  int32_t id, ifadc;

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3ID[ifadc];
      if(fwStatus[id].passed == 1)
	retMask |= (1 << id);
    }

  return retMask;
}

/*************************************************************
 * faV3FirmwareLoad
 *   - main routine to load up firmware for FADC with specific id
 *
 *  NOTE: Make call to
 *     faV3FirmwareSetFilename(...);
 *   if not using firmware from the default
 */

int32_t
faV3FirmwareLoad(int32_t id, int32_t pFlag)
{
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

  if(id==0) id = faV3Slot(id);
  updateArgs.id = id;
  updateArgs.step = FAV3_UPDATE_STEP_INIT;

  /* Perform a hardware and software reset */
  FAV3LOCK;
  vmeWrite32(&FAV3p[id]->reset, 0xFFFF);
  FAV3UNLOCK;
  taskDelay(60);

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);

  /* Check for ROM Ready after reset */
  if(faV3FirmwareWaitForReady(id, 60, 0) < OK)
    {
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d not ready after reset\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  updateArgs.show = FAV3_ARGS_SHOW_DONE;
  faV3FirmwareUpdateWatcher(updateArgs);

  /* ERASE ROM */
  updateArgs.step = FAV3_UPDATE_STEP_ERASE;

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "ERASE ROM \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);

  if(faV3FirmwareRomErase(id, 1) != OK)
    {
      printf("%s: ERROR: faV3 %2d Failed to erase ROM\n",
	     __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM ERASE\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Program ROM */
  updateArgs.step = FAV3_UPDATE_STEP_PROGRAM;
  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Program ROM\n");
  faV3FirmwareUpdateWatcher(updateArgs);

  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);
  if(faV3FirmwareProgramRom(id) != OK)
    {
      printf("%s: ERROR: faV3 %2d Failed to program ROM\n",
	     __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM PROGRAM\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Download ROM data */
  updateArgs.step = FAV3_UPDATE_STEP_DOWNLOAD;
  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Download ROM data\n");
  faV3FirmwareUpdateWatcher(updateArgs);
  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);

  if(faV3FirmwareDownload(id, 1) != OK)
    {
      printf("%s: ERROR: faV3 %2d Failed to download ROM data\n",
	     __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM DATA DOWNLOAD\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Verify ROM data with file */
  updateArgs.step = FAV3_UPDATE_STEP_VERIFY;
  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Verify ROM data\n");
  faV3FirmwareUpdateWatcher(updateArgs);

  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);

  if(faV3FirmwareVerify(id, 1) != OK)
    {
      printf("%s: ERROR: faV3 %2d ROM Data not verified\n",
	     __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM DATA VERIFICATION\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  /* Reboot */
  updateArgs.step = FAV3_UPDATE_STEP_REBOOT;
  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Reboot FPGA\n");
  faV3FirmwareUpdateWatcher(updateArgs);

  updateArgs.show = FAV3_ARGS_SHOW_ID;
  faV3FirmwareUpdateWatcher(updateArgs);

  if(faV3FirmwareReboot(id) != OK)
    {
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED TO REBOOT FPGA\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }
  sleep(1);

  if(faV3FirmwareWaitForReboot(id, 60000, 0) < OK)	/* Wait til it's done */
    {
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d TIMEOUT AFTER REBOOT FPGA\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
  faV3FirmwareUpdateWatcher(updateArgs);

  return OK;
}

int32_t
faV3FirmwareDownload(int32_t id, int32_t pFlag)
{
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

  /* PROM to data */
  taskDelay(1);

  if(faV3FirmwareDownloadRom(id, FAV3_FW_SIZE) != OK)
    {
      printf("%s: ERROR: faV3 %2d Failed to download ROM\n",
	     __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM DOWNLOAD\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  return OK;
}

int32_t
faV3FirmwareVerify(int32_t id, int32_t pFlag)
{
  faV3UpdateWatcherArgs_t updateArgs;
  CHECKID;

  /* Verify ROM */

  if(faV3FirmwareCompare() != OK)
    {
      printf("%s: ERROR: faV3 %d PROM data not verified\n", __func__, id);
      updateArgs.show = FAV3_ARGS_SHOW_STRING;
      sprintf(updateArgs.title,
	      "ERROR: FAV3 %2d FAILED ROM VERIFICATION\n", id);
      faV3FirmwareUpdateWatcher(updateArgs);
      return ERROR;
    }

  return OK;
}

int32_t
faV3FirmwareDone(int32_t pFlag)
{

  if(rom_firmware)
    free(rom_firmware);

  if(file_firmware)
    free(file_firmware);

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
int32_t
faV3FirmwareGLoad(int32_t pFlag, int32_t force)
{
  int32_t ifadc = 0, id = 0;
  faV3UpdateWatcherArgs_t updateArgs;

  memset(fwStatus, 0, sizeof(fwStatus));

  if(force == 0)
    {
      /* Skip the modules that are already programmed correctly */
      int32_t print_once = 1;
      for(ifadc = 0; ifadc < nfaV3; ifadc++)
	{
	  uint32_t fw_vers = 0;
	  uint32_t fw_supported = ((FAV3_SUPPORTED_PROC_FIRMWARE << 16) | (FAV3_SUPPORTED_CTRL_FIRMWARE) );

	  id = faV3Slot(ifadc);
	  fw_vers = faV3GetFirmwareVersions(id, 0);

	  if(fw_vers == fw_supported)
	    {
	      if(print_once)
		printf("Skip slot ");

	      printf(" %d", id);
	      print_once = 0;
	      fwStatus[id].skip = 1;
	    }
	}
      if(!print_once)
	printf("\n");
    }

  /* Perform a hardware and software reset */
  updateArgs.step = FAV3_UPDATE_STEP_INIT;
  FAV3LOCK;
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if((id <= 0) || (id > 21) || (FAV3p[id] == NULL))
	{
	  printf("%s: ERROR : ADC in slot %d is not initialized \n",
		 __func__, id);
	  fwStatus[id].passed = 0;
	  fwStatus[id].stepfail = updateArgs.step;
	}
      else
	{
	  fwStatus[id].passed = 1;
	  vmeWrite32(&FAV3p[id]->reset, 0xFFFF);
	}
    }
  FAV3UNLOCK;
  taskDelay(60);

  /* Check if FADC is Ready */

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Check if ready \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      updateArgs.id = id;
      updateArgs.show = FAV3_ARGS_SHOW_ID;
      faV3FirmwareUpdateWatcher(updateArgs);

      if(faV3FirmwareWaitForReady(id, 60, pFlag) < OK)
	{
	  updateArgs.show = FAV3_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title,
		  "ERROR: FAV3 %2d not ready after reset\n", id);
	  faV3FirmwareUpdateWatcher(updateArgs);
	  fwStatus[id].passed = 0;
	  fwStatus[id].stepfail = updateArgs.step;
	}
      else
	{
	  updateArgs.show = FAV3_ARGS_SHOW_DONE;
	  faV3FirmwareUpdateWatcher(updateArgs);
	}
    }

  /* ERASE ROM */
  updateArgs.step = FAV3_UPDATE_STEP_ERASE;

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "ERASE ROM \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  /* Execute erase command */
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareRomErase(id, 0) != OK)
	    {
	      updateArgs.show = FAV3_ARGS_SHOW_STRING;
	      sprintf(updateArgs.title,
		      "ERROR: FAV3 %2d FAILED TO EXEC ROM ERASE\n", id);
	      faV3FirmwareUpdateWatcher(updateArgs);
	      fwStatus[id].passed = 0;
	      fwStatus[id].stepfail = updateArgs.step;
	    }
	}
    }
  /* Wait for Erase to Complete */
  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  updateArgs.id = id;
	  updateArgs.show = FAV3_ARGS_SHOW_ID;
	  faV3FirmwareUpdateWatcher(updateArgs);

	  if(faV3FirmwareWaitForWIP(id, 200000, 0) < OK)
	    {
	      updateArgs.show = FAV3_ARGS_SHOW_STRING;
	      sprintf(updateArgs.title,
		      "ERROR: FAV3 %2d FAILED ROM ERASE\n", id);
	      faV3FirmwareUpdateWatcher(updateArgs);
	      fwStatus[id].passed = 0;
	      fwStatus[id].stepfail = updateArgs.step;
	    }
	}
    }

  /* Program ROM */
  updateArgs.step = FAV3_UPDATE_STEP_PROGRAM;

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Program ROM\n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  updateArgs.id = id;
	  updateArgs.show = FAV3_ARGS_SHOW_ID;
	  faV3FirmwareUpdateWatcher(updateArgs);
	  if(faV3FirmwareProgramRom(id) != OK)
	    {
	      updateArgs.show = FAV3_ARGS_SHOW_STRING;
	      sprintf(updateArgs.title,
		      "ERROR: FAV3 %2d FAILED ROM PROGRAM\n", id);
	      faV3FirmwareUpdateWatcher(updateArgs);
	      fwStatus[id].passed = 0;
	      fwStatus[id].stepfail = updateArgs.step;
	    }
	}
    }

  /* Download and Verify ROM data */
  updateArgs.step = FAV3_UPDATE_STEP_DOWNLOAD;
  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Download and Verify ROM data\n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  updateArgs.id = id;
	  updateArgs.show = FAV3_ARGS_SHOW_ID;
	  faV3FirmwareUpdateWatcher(updateArgs);

	  if(faV3FirmwareDownload(id, 1) != OK)
	    {
	      updateArgs.show = FAV3_ARGS_SHOW_STRING;
	      sprintf(updateArgs.title,
		      "ERROR: FAV3 %2d FAILED ROM DATA DOWNLOAD\n", id);
	      faV3FirmwareUpdateWatcher(updateArgs);
	      fwStatus[id].passed = 0;
	      fwStatus[id].stepfail = updateArgs.step;
	    }
	  else
	    {
	      /* Verify ROM data */
	      updateArgs.step = FAV3_UPDATE_STEP_VERIFY;
	      updateArgs.show = FAV3_ARGS_SHOW_ID;
	      faV3FirmwareUpdateWatcher(updateArgs);

	      if(faV3FirmwareVerify(id, 1) != OK)
		{
		  updateArgs.show = FAV3_ARGS_SHOW_STRING;
		  sprintf(updateArgs.title,
			  "ERROR: FAV3 %2d FAILED ROM DATA VERIFICATION\n", id);
		  faV3FirmwareUpdateWatcher(updateArgs);
		  fwStatus[id].passed = 0;
		  fwStatus[id].stepfail = updateArgs.step;

		}
	    }
	}
    }

  /* PROM to FPGA (Reboot FPGA) */

  updateArgs.show = FAV3_ARGS_SHOW_STRING;
  sprintf(updateArgs.title, "Rebooting FPGA \n");
  faV3FirmwareUpdateWatcher(updateArgs);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  faV3FirmwareReboot(id);
	}
    }
  sleep(1);

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  if(faV3FirmwareWaitForReboot(id, 60000, 0) < OK)	/* Wait til it's done */
	    {
	      printf("%s: ERROR: FADC %2d ready timeout after reboot\n",
		     __func__, id);
	      fwStatus[id].passed = 0;
	      fwStatus[id].stepfail = updateArgs.step;
	    }
	}
    }

  for(ifadc = 0; ifadc < nfaV3; ifadc++)
    {
      id = faV3Slot(ifadc);
      if(fwStatus[id].skip)
	continue;

      if(fwStatus[id].passed)		/* Skip the ones that have previously failed */
	{
	  updateArgs.show = FAV3_ARGS_SHOW_STRING;
	  sprintf(updateArgs.title, "Done programming FADC %2d\n", id);
	  faV3FirmwareUpdateWatcher(updateArgs);
	}
      else
	{
	  printf("%s: FAILED programming FADC %2d at step %d\n",
		 __func__, id, fwStatus[id].stepfail);
	}
    }

  return OK;

}

int
faV3FirmwareReadFile(char *filename)
{
  FILE *firmwareFD = fopen(filename, "r");

  if(firmwareFD == NULL)
    {
      perror("fopen");
      printf("%s: ERROR opening file (%s) for reading\n", __func__, filename);
      return ERROR;
    }

  file_firmware = (faV3fw_t *)malloc(sizeof(faV3fw_t));

  strcpy(file_firmware->filename, filename);

  uint32_t idata = 0, data = 0;
  while(!feof(firmwareFD))
    {
      fread(&data, sizeof(uint32_t), 1, firmwareFD);
      file_firmware->data[idata++] = bswap_32(data);
    }
  fclose(firmwareFD);

  file_firmware->size = idata-1;
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
      perror("fopen");
      printf("%s: ERROR opening file (%s) for writing\n", __func__, filename);
      return ERROR;
    }

  uint32_t idata = 0, data = 0;
  while(idata < rom_firmware->size)
    {
      data = bswap_32(rom_firmware->data[idata++]);
      fwrite(&data, sizeof(uint32_t), 1, firmwareFD);
    }
  fclose(firmwareFD);

  printf("%s: Wrote Firmware to %s\n", __func__, filename);

  return OK;
}

int
faV3FirmwareAttachUpdateWatcher(VOIDFUNCPTR routine, faV3UpdateWatcherArgs_t arg)
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
  static int32_t progress_ticks = 0;
  /* prescale to normalize to show max 20 +'s */
  int32_t erase_prescale = 1000 * 100 / 20; /* erase is about 100 seconds, each tick is 1 ms*/
  int32_t rw_prescale = (0x1701DEC / 4) / 20; /* Firmware is 0x1701DEC bytes, each tick is 4 bytes */
  int32_t progress_prescale = 20;

  if((arg.show >= FAV3_ARGS_SHOW_ID) && (arg.show < FAV3_ARGS_LAST))
    rArg = arg;
  else
    rArg = faV3UpdateWatcherArgs;

  if(faV3UpdateWatcherRoutine != NULL)
    {
      (*faV3UpdateWatcherRoutine) (rArg);
    }
  else
    {
      switch(rArg.step)
	{
	case FAV3_UPDATE_STEP_ERASE:
	case FAV3_UPDATE_STEP_REBOOT:
	  progress_prescale = erase_prescale;
	  break;

	case FAV3_UPDATE_STEP_PROGRAM:
	case FAV3_UPDATE_STEP_DOWNLOAD:
	case FAV3_UPDATE_STEP_VERIFY:
	  progress_prescale = rw_prescale;
	  break;

	}


      switch (rArg.show)
	{
	case FAV3_ARGS_SHOW_ID:
	  progress_ticks = 0;
	  printf("%2d: ", arg.id);
	  fflush(stdout);
	  break;

	case FAV3_ARGS_SHOW_PROGRESS:
	  progress_ticks++;
	  if((progress_ticks % progress_prescale) == 0)
	    {
	      printf("+");
	      fflush(stdout);
	    }
	  break;

	case FAV3_ARGS_SHOW_DONE:
	  progress_ticks = 0;
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
