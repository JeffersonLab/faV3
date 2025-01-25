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


#define ADD_TO_STRING				\
  len1 = strlen(str);				\
  len2 = strlen(sss);				\
  if((len1+len2) < length) strcat(str,sss);	\
  else return(len1)

#define CLOSE_STRING				\
  len1 = strlen(str);				\
  return(len1)

#define FNLEN     128       /* length of config. file name */
#define STRLEN    250       /* length of str_tmp */
#define ROCLEN     80       /* length of ROC_name */
#define NCHAN      16

#include <stdint.h>

/** FADC250 configuration parameters **/
typedef struct {
  int          mode;
  int          compression;
  int          vxsReadout;
  uint32_t winOffset;
  uint32_t winWidth;
  uint32_t nsb;
  uint32_t nsa;
  uint32_t npeak;

  uint32_t chDisMask;
  uint32_t thr[NCHAN];
  uint32_t dac[NCHAN];
  float        ped[NCHAN];

  uint32_t trigMask;

  uint32_t read_thr[NCHAN];

  uint32_t nsat;

  uint32_t nped;
  uint32_t max_ped;

  uint32_t trig_bl[NCHAN];

  uint32_t trig_thr;
  uint32_t trig_nsb;
  uint32_t trig_nsa;
  uint32_t trig_nsat;

  /* Busy and stop processing conditions */
  uint32_t busy;
  uint32_t stop;

  uint32_t data_format;

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
