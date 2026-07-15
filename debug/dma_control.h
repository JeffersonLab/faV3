#pragma once

#include <stdbool.h>
#include <pthread.h>

typedef struct {
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  bool keep_running;
  bool is_polling;
  DMA_MEM_ID vmeIN;
  DMANODE *dma_event;
  uint32_t *dma_buffer;
  void (*readout_function) (void *arg);
  void *readout_argument;
} dma_control_t;

pthread_t dma_control_init(dma_control_t *ctrl);
void dma_control_pause(dma_control_t *ctrl);
void dma_control_resume(dma_control_t *ctrl);
void dma_control_shutdown(dma_control_t *ctrl, pthread_t thread_id);
