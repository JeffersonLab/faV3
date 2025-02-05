#pragma once

#ifndef FAV3_CONFIG_GET_ENV
#define FAV3_CONFIG_GET_ENV "FAV3_PARAMS"
#endif

/****************************************************************************
 *
 *  fadc250Config.h  -  configuration library header file for fADC250 board
 *
 *  SP, 07-Nov-2013
 *
 */


#include <stdint.h>

#define MAX_FAV3_CH 16

/** FADC250 configuration parameters **/
typedef struct {
  uint32_t proc_version;
  int32_t mode;
  int32_t compression;
  int32_t vxsReadout;
  uint32_t winOffset;
  uint32_t winWidth;
  int32_t nsb;
  uint32_t nsa;
  uint32_t npeak;

  uint32_t chDisMask;

  uint32_t dac[MAX_FAV3_CH];
  uint32_t read_thr[MAX_FAV3_CH];
  float pedestal[MAX_FAV3_CH];

  uint32_t nsat;

  uint32_t nped;
  uint32_t max_ped;

  uint32_t trig_thr;
  uint32_t trig_nsa;
  uint32_t trig_nsat;

  /* Busy and stop processing conditions */
  int32_t busy;
  int32_t stop;

  uint32_t ptw_fallback_mask;

  int32_t data_format;
  int32_t suppress_trig_time;
  int32_t insert_adc_params;

} FAV3_CONF;


/* functions */

#ifdef	__cplusplus
extern "C" {
#endif

void faV3SetExpid(char *string);
void faV3Sethost(char *host);
void faV3InitGlobals();
int faV3ReadConfigFile(char *filename);
int faV3DownloadAll();
int faV3Config(char *fname);
int faV3UploadAll(char *string, int length);

#ifdef	__cplusplus
}
#endif
