#pragma once

#define CONFIG_STRING_VARS			\
  int len1, len2;				\
  char *str, sss[1024];

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
#define CONFIG_DEBUG 0

#define SCAN_VARS						\
  int    slot, slot_min, slot_max, chan;			\
  int    args, i1, msk[16];					\
  uint32_t  ui1, val;						\
  char   str_tmp[STRLEN], str2[STRLEN], keyword[ROCLEN];	\
  float f1, fmsk[16];


#define SCAN_MSK						\
  args = sscanf (str_tmp, "%*s %d %d %d %d %d %d %d %d   \
                                     %d %d %d %d %d %d %d %d",	\
		 &msk[ 0], &msk[ 1], &msk[ 2], &msk[ 3],	\
		 &msk[ 4], &msk[ 5], &msk[ 6], &msk[ 7],	\
		 &msk[ 8], &msk[ 9], &msk[10], &msk[11],	\
		 &msk[12], &msk[13], &msk[14], &msk[15])

#define SCAN_FMSK						\
  args = sscanf (str_tmp, "%*s %f %f %f %f %f %f %f %f   \
                                     %f %f %f %f %f %f %f %f",	\
		 &fmsk[ 0], &fmsk[ 1], &fmsk[ 2], &fmsk[ 3],	\
		 &fmsk[ 4], &fmsk[ 5], &fmsk[ 6], &fmsk[ 7],	\
		 &fmsk[ 8], &fmsk[ 9], &fmsk[10], &fmsk[11],	\
		 &fmsk[12], &fmsk[13], &fmsk[14], &fmsk[15])

#define GET_READ_MSK							\
  SCAN_MSK;								\
  if(args != 16)							\
    {									\
      CFG_ERR("Invalid number of arguments (%d), should be 16\n", args); \
      return(-8);							\
    }									\
  val = 0;								\
  for(chan=0; chan<NCHAN; chan++)					\
    {									\
      if((msk[chan] < 0) || (msk[chan] > 1))				\
	{								\
	  CFG_ERR("Invalid mask bit value, %d\n", msk[chan]);		\
	  return(-6);							\
	}								\
    }

/* Macros for error ouput */
#define CFG_ERR(format, ...) {				\
    fprintf(stdout, "\n%s: ERROR: ", __func__);		\
    if(slot_min==0)					\
      fprintf(stdout, "ALL SLOTS: ");			\
    else						\
      fprintf(stdout, "SLOT %d: ", slot_min);		\
    fprintf(stdout, "%s\n\t", keyword);			\
    fprintf(stdout, format, ## __VA_ARGS__);		\
    fprintf(stdout, "\n");				\
  }

#define SCAN_SLOT(BKEYWORD,SL_MIN,SL_MAX)				\
  if(active && (strcmp(keyword, BKEYWORD) == 0)) {			\
   slot = -1;								\
   sscanf(str_tmp, "%*s %s", str2);					\
   if(isdigit(str2[0])) {						\
    SL_MIN = atoi(str2);						\
    SL_MAX = slot_min + 1;						\
    if((SL_MIN < 2) || (SL_MIN > 21)) {					\
      printf("%s: ERROR: Invalid slot number %d\n\n", __func__, SL_MIN); \
      return(-4);}							\
   } else if(!strcmp(str2,"all")) {					\
     SL_MIN = 0;							\
     SL_MAX = NBOARD;							\
   } else {								\
     printf("%s: ERROR: Invalid slot >%s<, must be 'all' or actual slot number\n\n", __func__, str2); \
     return(-4);							\
   }									\
   if(CONFIG_DEBUG) printf("%s: keyword = %s  SL_MIN = %d\n", __func__, keyword, SL_MIN);	\
   continue;}

#define SCAN_INT(BKEYWORD,BSTRUCT,SL_MIN,SL_MAX)		\
  if(active && (strcmp(keyword,(BKEYWORD)) == 0)) {		\
    sscanf (str_tmp, "%*s %d", &val);				\
    for(slot = SL_MIN; slot < SL_MAX; slot++) (BSTRUCT) = val;	\
    if(CONFIG_DEBUG) printf("%s: keyword = %s  val = %d\n", __func__, keyword, val);	\
    continue;}					\


#define SCAN_INT_HEX(BKEYWORD,BSTRUCT,SL_MIN,SL_MAX)		\
  if(active && (strcmp(keyword,(BKEYWORD)) == 0)) {		\
    sscanf (str_tmp, "%*s %x", &val);				\
    for (slot = SL_MIN; slot < SL_MAX; slot++) (BSTRUCT) = val;	\
    continue;}

#define SCAN_MASK(BKEYWORD,BSTRUCT,SL_MIN,SL_MAX)		\
  if(active && (strcmp(keyword,(BKEYWORD)) == 0)) {		\
    GET_READ_MSK;						\
    for(chan = 0; chan < NCHAN; chan++)					\
      val |= msk[chan] << chan;						\
    for (slot = SL_MIN; slot < SL_MAX; slot++)				\
      (BSTRUCT) = val;							\
    if(CONFIG_DEBUG) printf("%s: keyword = %s  val = %d\n", __func__, keyword, val);\
    continue;}

#define SCAN_MASK_INV(BKEYWORD,BSTRUCT,SL_MIN,SL_MAX)		\
  if(active && (strcmp(keyword,(BKEYWORD)) == 0)) {		\
    GET_READ_MSK;						\
    for(chan = 0; chan < NCHAN; chan++)					\
      val |= msk[chan] << chan;						\
    for (slot = SL_MIN; slot < SL_MAX; slot++)				\
      (BSTRUCT) = ~val & 0xffff;					\
    if(CONFIG_DEBUG) printf("%s: keyword = %s  val = %d\n", __func__, keyword, val);\
    continue;}



#define SCAN_INT_ALL(TDP_K, TDP_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP_K)) == 0)) {			\
    sscanf (str_tmp, "%*s %d", &val);					\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      for(chan = 0; chan < NCHAN; chan++)   (TDP_S)[chan] = val;	\
    continue;}

#define SCAN_INT_CH(TDP3_K,TDP3_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP3_K)) == 0))	{			\
    sscanf (str_tmp, "%*s %d %d", &chan, &val);				\
    if((chan < 0) || (chan > NCHAN)) {				\
      printf("%s: ERROR: Invalid channel number %d\n\n", __func__, chan); \
      return(-4);}							\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      (TDP3_S)[chan] = val;						\
    continue;}

#define SCAN_INT_ALLCH(TDP3_K,TDP3_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP3_K)) == 0))	{			\
    SCAN_MSK;								\
    if(args != 16) {							\
      printf("%s: Wrong argument's number %d, should be 16\n\n", __func__, args); \
      return(-8);							\
    }									\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      for(chan = 0; chan < NCHAN; chan++)   (TDP3_S)[chan] = msk[chan]; \
    continue;}

#define SCAN_FLOAT_ALL(TDP_K, TDP_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP_K)) == 0)) {			\
    sscanf (str_tmp, "%*s %f", &f1);					\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      for(chan = 0; chan < NCHAN; chan++)   (TDP_S)[chan] = f1;		\
    continue;}

#define SCAN_FLOAT_CH(TDP3_K,TDP3_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP3_K)) == 0))	{			\
    sscanf (str_tmp, "%*s %d %f", &chan, &f1);				\
    if((chan < 0) || (chan > NCHAN)) {				\
      printf("%s: ERROR: Invalid channel number %d\n\n", __func__, chan); \
      return(-4);}							\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      (TDP3_S)[chan] = f1;						\
    continue;}

#define SCAN_FLOAT_ALLCH(TDP3_K,TDP3_S,SL_MIN,SL_MAX)			\
  if(active && (strcmp(keyword,(TDP3_K)) == 0))	{			\
    SCAN_FMSK;								\
    if(args != 16) {							\
      printf("%s: Wrong argument's number %d, should be 16\n\n", __func__, args); \
      return(-8);							\
    }									\
    for(slot = SL_MIN; slot < SL_MAX; slot++)				\
      for(chan = 0; chan < NCHAN; chan++)   (TDP3_S)[chan] = fmsk[chan]; \
    continue;}

#define SCAN_B_MSKS(BKEYWORD,BSTRUCT)			\
  if(active && (strcmp(keyword,(BKEYWORD)) == 0)) {	\
    GET_READ_MSK;					\
    (BSTRUCT) = ui1;					\
    continue;}
