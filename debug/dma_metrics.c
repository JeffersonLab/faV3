#include "dma_metrics.h"
#include <stdio.h>

static double
timespec_diff_sec(struct timespec start, struct timespec end)
{
  return (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) * 1e-9;
}

void
dma_metrics_init(dma_metrics_t *m)
{
  pthread_mutex_init(&m->lock, NULL);
  dma_metrics_reset(m);
}

void
dma_metrics_reset(dma_metrics_t *m)
{
  pthread_mutex_lock(&m->lock);
  m->total_transfers = 0;
  m->total_bytes = 0;
  m->total_active_dma_time_sec = 0.0;
  clock_gettime(CLOCK_MONOTONIC, &m->start_time);
  pthread_mutex_unlock(&m->lock);
}

// Called by the polling thread immediately after a DMA finishes
void
dma_metrics_record(dma_metrics_t *m, size_t bytes, struct timespec start, struct timespec end)
{
  double duration = timespec_diff_sec(start, end);
  if(duration < 0.0)
    duration = 0.0;

  pthread_mutex_lock(&m->lock);
  m->total_transfers++;
  m->total_bytes += bytes;
  m->total_active_dma_time_sec += duration;
  pthread_mutex_unlock(&m->lock);
}

// Called on demand to display statistics since the last reset
void
dma_metrics_print_report(dma_metrics_t *m)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  pthread_mutex_lock(&m->lock);

  double total_elapsed = timespec_diff_sec(m->start_time, now);
  if(total_elapsed <= 0.0)
    total_elapsed = 1e-9;	// Prevent division by zero

  uint64_t transfers = m->total_transfers;
  uint64_t bytes = m->total_bytes;
  double active_time = m->total_active_dma_time_sec;

  pthread_mutex_unlock(&m->lock);

  // Convert bytes to Megabytes (1024 * 1024)
  double total_mb = (double) bytes / (1024.0 * 1024.0);

  // Calculate rates
  double avg_hz = (double) transfers / total_elapsed;
  double avg_mb_per_sec = total_mb / total_elapsed;

  // Average speed while the hardware was actively busy
  double hardware_mb_per_sec = (active_time > 0.0) ? (total_mb / active_time) : 0.0;

  printf("\n============= DMA RUNNING STATS =============\n");
  printf("Elapsed Time:         %.3f seconds\n", total_elapsed);
  printf("Total Active Time:    %.6f seconds\n", active_time);
  printf("Total Transfers:      %lu\n", transfers);
  printf("Total Data Read:      %lu B\n", bytes);
  printf("---------------------------------------------\n");
  printf("Average DMA Freq:     %.2f Hz\n", avg_hz);
  printf("Average Data Rate:    %.3f MB/s (overall rate)\n", avg_mb_per_sec);
  printf("Hardware Speed:       %.3f MB/s (during active DMA)\n", hardware_mb_per_sec);
  printf("=============================================\n");
}
