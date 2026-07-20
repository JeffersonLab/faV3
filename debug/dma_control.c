#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "jvme.h"
#include "dmaPList.h"
#include "dma_control.h"

#define MAX_EVENT_POOL     1
#define MAX_EVENT_LENGTH   1024*20	/* Size in Bytes */


void *dma_poll_thread(void *arg);

pthread_t
dma_control_init(dma_control_t *ctrl)
{
  pthread_mutex_init(&ctrl->mutex, NULL);
  pthread_cond_init(&ctrl->cond, NULL);
  ctrl->keep_running = true;
  ctrl->is_polling = false; // start paused
  ctrl->dma_buffer = NULL;

  dmaPFreeAll();

  ctrl->vmeIN = dmaPCreate("vmeIN", MAX_EVENT_LENGTH, MAX_EVENT_POOL, 0);

  dmaPReInitAll();

  extern DMANODE *the_event;
  extern unsigned int *dma_dabufp;
  GETEVENT(ctrl->vmeIN, 0);


  ctrl->dma_event = the_event;
  ctrl->dma_buffer = (uint32_t *) &(ctrl->dma_event->data[0]);

  pthread_t rval_thread_id;
  pthread_create(&rval_thread_id, NULL, dma_poll_thread, ctrl);

  return rval_thread_id;
}

void
dma_control_pause(dma_control_t *ctrl)
{
  pthread_mutex_lock(&ctrl->mutex);
  ctrl->is_polling = false;
  pthread_mutex_unlock(&ctrl->mutex);
}

void
dma_control_resume(dma_control_t *ctrl)
{
  pthread_mutex_lock(&ctrl->mutex);
  ctrl->is_polling = true;
  pthread_cond_signal(&ctrl->cond);
  pthread_mutex_unlock(&ctrl->mutex);
}

void
dma_control_shutdown(dma_control_t *ctrl, pthread_t thread_id)
{
  pthread_mutex_lock(&ctrl->mutex);
  ctrl->keep_running = false;
  ctrl->is_polling = false;
  pthread_cond_signal(&ctrl->cond);	// Wake up if asleep
  pthread_mutex_unlock(&ctrl->mutex);

  // Wait for the thread to completely finish current iteration and exit
  pthread_join(thread_id, NULL);

  pthread_mutex_destroy(&ctrl->mutex);
  pthread_cond_destroy(&ctrl->cond);
}

// Inline helper for the thread loop checkpoint
static inline bool
dma_control_checkpoint(dma_control_t *ctrl)
{
  pthread_mutex_lock(&ctrl->mutex);

  while(!ctrl->is_polling && ctrl->keep_running)
    {
      pthread_cond_wait(&ctrl->cond, &ctrl->mutex);
    }

  bool should_continue = ctrl->keep_running;
  pthread_mutex_unlock(&ctrl->mutex);

  return should_continue;
}

void
dma_cleanup_handler(void *arg)
{
  dma_control_t *ctrl = (dma_control_t *) arg;
  printf("[Cleanup] Executing cleanup stack handler...\n");

  // Safety check: ensure mutex isn't left locked if canceled inside checkpoint
  pthread_mutex_unlock(&ctrl->mutex);

  if(ctrl->dma_buffer)
    {
      dmaPFreeItem(ctrl->dma_event);
      dmaPFreeAll();
      printf("[Cleanup] DMA Buffer released.\n");
    }
}

void *
dma_poll_thread(void *arg)
{
  dma_control_t *ctrl = (dma_control_t *) arg;

  pthread_cleanup_push(dma_cleanup_handler, ctrl);

  while(dma_control_checkpoint(ctrl))
    {
      if(ctrl->readout_function != NULL)
	{
	  (*ctrl->readout_function) (ctrl->readout_argument);
	}
    }

  pthread_cleanup_pop(1);
  return NULL;
}
