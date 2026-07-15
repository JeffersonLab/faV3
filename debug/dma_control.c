#define MAX_EVENT_POOL     1
#define MAX_EVENT_LENGTH   1024*20      /* Size in Bytes */
DMA_MEM_ID vmeIN,vmeOUT;


int32_t
dma_init()
{
  /* Setup Address and data modes for DMA transfers
   *
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* INIT dmaPList */

  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",MAX_EVENT_LENGTH,MAX_EVENT_POOL,0);
  vmeOUT = dmaPCreate("vmeOUT",0,0,0);

  dmaPStatsAll();

  dmaPReInitAll();

  GETEVENT(vmeIN, 0);

  return 0;
}

int32_t
dma_close()
{
  PUTEVENT(vmeOUT);
  DMANODE *outEvent = dmaPGetItem(vmeOUT);
  dmaPFreeItem(outEvent);

  dmaPFreeAll();
  return 0;
}

pthread_t      readout_thread;

void
start_readout_thread(void)
{
  int status;

  status = pthread_create(&readout_thread, NULL,
			  (void*(*)(void *)) readout_loop, (void *)NULL);
  if(status!=0)
    {
      perror("pthread_create");

      printf("%s: ERROR: Readout Thread could not be started.\n",
	     __func__);
      printf("\t pthread_create returned: %d\n", status);
    }

}

void
cancel_readout_thread(void)
{
  int status = 0;
  if(readout_thread)
    {
      if(pthread_cancel(readout_thread)<0)
	perror("pthread_cancel");
      if(pthread_join(readout_thread, NULL)<0)
	perror("pthread_join");
    }

}
