#pragma once

#include <stdint.h>
#include <pthread.h>
#include <time.h>

typedef struct {
  uint64_t total_transfers;         // Count of successful DMAs
  uint64_t total_bytes;             // Cumulative payload size in bytes
  double total_active_dma_time_sec; // Cumulative time spent *actively* transferring data
  struct timespec start_time;       // Reference point for overall elapsed time
  pthread_mutex_t lock;             // Protects the cumulative sums
} dma_metrics_t;

void dma_metrics_init(dma_metrics_t *m);
void dma_metrics_reset(dma_metrics_t *m);
void dma_metrics_record(dma_metrics_t *m, size_t bytes, struct timespec start, struct timespec end);
void dma_metrics_print_report(dma_metrics_t *m);
