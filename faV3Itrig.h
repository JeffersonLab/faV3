#pragma once

#include <stdint.h>

/* FAV3DC Internal Trigger Routine prototypes */
int faV3ItrigBurstConfig(int id, uint32_t ntrig,
		       uint32_t burst_window, uint32_t busy_period);
uint32_t faV3ItrigControl(int id, uint16_t itrig_width, uint16_t itrig_dt);
uint32_t faV3ItrigStatus(int id, int sFlag);
int faV3ItrigSetMode(int id, int tmode, uint32_t wMask, uint32_t wWidth,
		   uint32_t cMask, uint32_t sumThresh, uint32_t * tTable);
int faV3ItrigInitTable(int id, uint32_t * table);
int faV3ItrigSetHBwidth(int id, uint16_t hbWidth, uint16_t hbMask);
uint32_t faV3ItrigGetHBwidth(int id, uint32_t chan);
int faV3ItrigPrintHBwidth(int id);
uint32_t faV3ItrigOutConfig(int id, uint16_t itrigDelay, uint16_t itrigWidth);
int faV3ItrigEnable(int id);
int faV3ItrigDisable(int id);
int faV3ItrigGetTableVal(int id, uint16_t pMask);
