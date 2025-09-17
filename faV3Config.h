#pragma once
/****************************************************************************
 *
 *  faV3Config.h  -  configuration library header file for faV3
 *
 */


#include <stdint.h>

#define MAX_FAV3_CH 16

/** FADC250 configuration parameters **/
typedef struct {
  uint32_t proc_version;

  uint32_t chDisMask;

  int32_t mode;

  uint32_t winOffset;
  uint32_t winWidth;
  uint32_t nsb;
  uint32_t nsa;
  uint32_t npeak;

  uint32_t nsat;
  uint32_t nped;
  uint32_t max_ped;

  uint16_t ptw_fallback_mask;

  uint32_t trig_thr;
  uint32_t trig_nsa;
  uint32_t trig_nsat;

  uint32_t trigMask;
  uint32_t trigWidth;
  uint32_t trigMinTOT;
  uint32_t trigMinMult;

  uint32_t thrIgnoreMask;
  uint32_t invertMask;
  uint32_t playbackDisableMask;
  uint32_t sparsification;
  uint32_t accumulatorMask;
  uint32_t trigModeMask;

  float pedestal[MAX_FAV3_CH];
  uint32_t thr[MAX_FAV3_CH];
  uint32_t dac[MAX_FAV3_CH];
  float gain[MAX_FAV3_CH];
  uint32_t delay[MAX_FAV3_CH];

  int32_t data_format;
  int32_t suppress_trig_time;
  int32_t insert_adc_params;
  int32_t compression;
  int32_t vxsReadout;

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
