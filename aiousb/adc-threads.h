#ifndef ADC_THREADS_H
#define ADC_THREADS_H
#include <pthread.h>
#include <semaphore.h>
#include "aiousb.h"
enum bcs_style {bcs_adc, bcs_dio};

static const int adc_worker_min_block = 1024 * 1024;
struct adc_worker_context
{
  aiousb_device_handle device;
  unsigned int pipe_index;
  int bytes_left;
  uint16_t *tar;
  enum bcs_style bcs_style;
  uint32_t block_size;
  int b_abort;
  pthread_mutex_t mutex;
  pthread_mutex_t cond_mutex;
  pthread_cond_t cond;
};

enum buf_state {buf_state_blank, buf_state_accessing, buf_state_got_data};
struct adc_continuous_buffer
{
  uint8_t *ad_buff;
  enum buf_state buf_state;
  int index;
  uint32_t used_size; //should this be size_t?
  uint32_t flags;
};

typedef struct adc_continuous_buffer * adc_continuous_buffer_handle;

struct adc_cont_buff_worker_context
{
  uint32_t bytes_per_buff;
  pthread_mutex_t buff_mutex;
  sem_t blank_buf_sem;
  sem_t data_buf_sem;
  sem_t kill_sem;
  adc_cont_callback *callback;
  uint32_t callback_context;
  int num_buffs;
  adc_continuous_buffer_handle *buf_buf;
  int next_index_in;
  int next_index_out;
  int terminate;
};
struct adc_cont_acq_worker_context
{
  aiousb_device_handle device;
  unsigned int pipe_index;
  int b_counter_control;
  uint16_t divisor_a, divisor_b;
  uint32_t io_status;
  enum bcs_style bcs_style;
  struct adc_cont_buff_worker_context *adc_cont_buff_worker_context;
  pthread_cond_t start_cond;
  pthread_mutex_t start_cond_mutex;
  int terminate;
};

void *adc_worker_execute (void *context);

#endif