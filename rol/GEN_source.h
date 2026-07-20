/******************************************************************************
*
* header file for use General USER defined rols with CODA 3
*
*                             DJA   Nov 2000
*
*******************************************************************************/
#ifndef __GEN_ROL__
#define __GEN_ROL__

#include "dmaBankTools.h"
#include <stdio.h>
#include <rol.h>

static int GEN_handlers,GENflag;
static int GEN_isAsync;

/* POLLING_MODE */
#define POLLING___
#define POLLING_MODE

/* INIT_NAME, INIT_NAME_POLL must be defined at compilation.
   Check readout list Makefile */
#ifndef INIT_NAME
#warning "INIT_NAME undefined. Set to readout list filename base with gcc flag -DINIT_NAME"
#endif
#ifndef INIT_NAME_POLL
#warning "INIT_NAME_POLL undefined. Set to readout list filename base with gcc flag -DINIT_NAME_POLL"
#endif
extern int bigendian_out;

/* ROC Function prototypes defined by the user */
void rocDownload();
void rocPrestart();
void rocGo();
void rocEnd();
int rocPoll();
int rocType();
void rocTrigger(int evno, int evtype);
void rocCleanup();

static void
gentenable(int val)
{
  GENflag = 1;
}

static void
gentdisable(int val)
{
  GENflag = 0;
}

static int
genttest(int code)
{
  int rval = 0;
  if(GENflag)
    rval = rocPoll();

  return rval;
}

/* Define CODA readout list specific Macro routines/definitions */

#define GEN_TEST  genttest

#define GEN_INIT { GEN_handlers =0;GEN_isAsync = 0;GENflag = 0;}

#define GEN_ASYNC(code,id)  {printf("*** No Async mode is available for GEN ***\n"); \
                              printf("linking sync GEN trigger to id %d \n",id); \
			       GEN_handlers = (id);GEN_isAsync = 0;}

#define GEN_SYNC(code,id)   {printf("linking sync GEN trigger to id %d \n",id); \
			       GEN_handlers = (id);GEN_isAsync = 0;}

#define GEN_ENA(code,val) gentenable(val);

#define GEN_DIS(code,val) gentdisable(val);

#define GEN_GETID(code) GEN_handlers

#define GEN_TTYPE rocType

#define GEN_START(val)	 {;}


/**
 *  DOWNLOAD
 */
static void __download()
{
  int status;

  daLogMsg("INFO","Readout list compiled %s", DAYTIME);
#ifdef POLLING___
  rol->poll = 1;
#endif
  *(rol->async_roc) = 0; /* Normal ROC */

  bigendian_out=1;

  /* Execute User defined download */
  rocDownload();

  daLogMsg("INFO","Download Executed");

} /*end download */

/**
 *  PRESTART
 */
static void __prestart()
{
  CTRIGINIT;
  *(rol->nevents) = 0;

  daLogMsg("INFO","Entering Prestart");


  GEN_INIT;
  CTRIGRSS(GEN,1,usrtrig,usrtrig_done);
  CRTTYPE(1,GEN,1);

  /* Execute User defined prestart */
  rocPrestart();

  daLogMsg("INFO","Prestart Executed");

  if (__the_event__) WRITE_EVENT_;
  *(rol->nevents) = 0;
  rol->recNb = 0;
} /*end prestart */

/**
 *  PAUSE
 */
static void __pause()
{
  CDODISABLE(GEN,1,0);
  daLogMsg("INFO","Pause Executed");

  if (__the_event__) WRITE_EVENT_;
} /*end pause */

/**
 *  GO
 */
static void __go()
{
  daLogMsg("INFO","Entering Go");

  CDOENABLE(GEN,1,1);
  rocGo();

  if (__the_event__) WRITE_EVENT_;
}

static void __end()
{
  /* Execute User defined end */
  rocEnd();

  CDODISABLE(GEN,1,0);

  daLogMsg("INFO","End Executed");

  if (__the_event__) WRITE_EVENT_;
} /* end end block */

void usrtrig(unsigned long EVTYPE,unsigned long EVSOURCE)
{
  int ev_num = *(rol->nevents);
  long length = 0, size = 0;

  /* Execute user defined Trigger Routine */
  rocTrigger(ev_num, EVTYPE);

} /*end trigger */

void usrtrig_done()
{
} /*end done */

void __done()
{
  poolEmpty = 0; /* global Done, Buffers have been freed */
} /*end done */

static void __reset()
{
  int iemp=0;

  printf(" **Reset Called** \n");

} /* end reset */

/* This routine is automatically executed just before the shared libary
   is unloaded.

   Clean up memory that was allocated
*/
__attribute__((destructor)) void end (void)
{
  static int ended=0;

  if(ended==0)
    {
      printf("ROC Cleanup\n");
      rocCleanup();

      ended=1;
    }

}

#endif /* __GEN_ROL__ */
