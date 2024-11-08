#pragma once

#include <stdint.h>
#include "jvme.h"

/* FAV3DC Firmware Tools Prototypes */
int32_t faV3FirmwareWaitForReady(int32_t id, int32_t nwait, int32_t pflag);
uint32_t faV3FirmwareRomID(int32_t id);
uint32_t faV3FirmwareRomStatus1(int32_t id);
uint32_t faV3FirmwareSetMemoryWrite(int32_t id, int32_t enable);
int32_t  faV3FirmwareRomErase(int32_t id);
int32_t  faV3FirmwareReadRomAdr(int32_t id, uint32_t romadr, int32_t last);
int32_t  faV3FirmwareWriteRomAdr(int32_t id, uint32_t romadr, uint32_t romdata, int32_t last);
int32_t  faV3FirmwareDownloadRom(int32_t id, int32_t size);
int32_t  faV3FirmwareProgramRom(int32_t id);
int32_t  faV3FirmwareVerifyDownload();

int32_t faV3FirmwarePassedMask();
int32_t faV3FirmwareLoad(int32_t id, int32_t pFlag);
int32_t faV3FirmwareDownload(int32_t id, int32_t pFlag);
int32_t faV3FirmwareVerify(int32_t id, int32_t pFlag);
int32_t faV3FirmwareDone(int32_t pFlag);

int32_t faV3FirmwareReadFile(char *filename);
int32_t faV3FirmwareWriteFile(char *filename);
int32_t faV3FirmwareReadMcsFile(char *filename);

enum faV3Args_enum
  {
    FAV3_ARGS_SHOW_ID,
    FAV3_ARGS_SHOW_PROGRESS,
    FAV3_ARGS_SHOW_DONE,
    FAV3_ARGS_SHOW_STRING,
    FAV3_ARGS_LAST
  };

typedef struct faV3UpdateWatcherArgs_struct
{
  int32_t step;			/* 0: show id, 1: show progress, 2: show complete */
  int32_t id;			/* slot id */
  char title[80];
} faV3UpdateWatcherArgs_t;
int32_t faV3FirmwareAttachUpdateWatcher(VOIDFUNCPTR routine,
				    faV3UpdateWatcherArgs_t arg);
void faV3FirmwareUpdateWatcher(faV3UpdateWatcherArgs_t arg);
