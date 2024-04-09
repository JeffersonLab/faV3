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


/** FADC250 configuration parameters **/
typedef struct {
  int f_rev;
  int b_rev;
  int b_ID;

  int          mode;
  int          compression;
  int          vxsReadout;
  unsigned int winOffset;
  unsigned int winWidth;
  unsigned int nsb;
  unsigned int nsa;
  unsigned int npeak;

  unsigned int chDisMask;
  unsigned int trigMask;
  unsigned int trigWidth;
  unsigned int trigMinTOT;
  unsigned int trigMinMult;
  unsigned int thr[NCHAN];
  unsigned int dac[NCHAN];
  unsigned int delay[NCHAN];
  float        ped[NCHAN];
  unsigned int thrIgnoreMask;
  unsigned int invertMask;
  unsigned int playbackDisableMask;
  float gain[NCHAN];
  unsigned int trigMode[NCHAN];
  unsigned int sparsification;
  unsigned int accumulatorMask;

} FAV3_CONF;


/* functions */

#ifdef	__cplusplus
extern "C" {
#endif

void faV3SetExpid(char *string);
void faV3GetParamsForOffline(float ped[6][22][16], int tet[6][22][16], float gain[6][22][16], int nsa[6][22], int nsb[6][22]);
void faV3Sethost(char *host);
void faV3InitGlobals();
int faV3ReadConfigFile(char *filename);
int faV3DownloadAll();
int faV3Config(char *fname);
void faV3Mon(int slot);
int faV3UploadAll(char *string, int length);

#ifdef	__cplusplus
}
#endif
