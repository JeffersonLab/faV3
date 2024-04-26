#pragma once

#include <stdint.h>
#include "jvme.h"

/* FAV3DC Firmware Tools Prototypes */
int faV3FirmwareLoad(int id, int chip, int pFlag);
int faV3FirmwareGLoad(int chip, int pFlag);
void faV3FirmwareDownloadConfigData(int id);
int faV3FirmwareVerifyDownload(int id);
int faV3FirmwareTestReady(int id, int n_try, int pFlag);
int faV3FirmwareZeroSRAM(int id);
int faV3FirmwareCheckSRAM(int id);
void faV3FirmwareSetFilename(char *filename, int chip);
int faV3FirmwareReadFile(char *filename);
int faV3FirmwareGetFpgaID(int pflag);
int faV3FirmwareChipFromFpgaID(int pflag);
int faV3FirmwareRevFromFpgaID(int pflag);
int faV3FirmwareReadMcsFile(char *filename);
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
  int step;			/* 0: show id, 1: show progress, 2: show complete */
  int id;			/* slot id */
  char title[80];
} faV3UpdateWatcherArgs_t;
int faV3FirmwareAttachUpdateWatcher(VOIDFUNCPTR routine,
				    faV3UpdateWatcherArgs_t arg);
void faV3FirmwareUpdateWatcher(faV3UpdateWatcherArgs_t arg);
