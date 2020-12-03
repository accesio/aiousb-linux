#ifndef ADC_THREADS_H
#define ADC_THREADS_H
#include <atomic>
#include <thread>
#include <pthread.h>
#include <semaphore.h>
#include "safe-queue.h"
#include "aiousb.h"
namespace AIOUSB {
enum bcs_style {bcs_adc, bcs_dio};

static const int adc_worker_min_block = 1024 * 1024;
struct adc_worker_context
{
  aiousb_device_handle device;
  unsigned int pipe_index;
  unsigned int bytes_left;
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

struct adc_cont_acq_worker_context;

struct adc_cont_buff_worker_context
{
  uint32_t bytes_per_buff;
  pthread_mutex_t buff_mutex;
  sem_t blank_buf_sem;
  sem_t data_buf_sem;
  sem_t kill_sem;
  adc_cont_callback callback;
  void *callback_context;
  int num_buffs;
  adc_continuous_buffer_handle *buf_buf;
  int next_index_in;
  int next_index_out;
  int terminate;
  struct adc_cont_acq_worker_context *adc_cont_acq_worker_context;
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

void *adc_cont_buff_worker_execute(void *context);
void *adc_worker_execute (void *context);


class ContinuousBufferManager
{
  public:
    ContinuousBufferManager(int Count, size_t Size);
    ~ContinuousBufferManager();

    //get or create empty buffer
    uint16_t* EmptyBufferGet();
    //store empty buffer
    void EmptyBufferPut(uint16_t *Buff);

    //get data buffer. wait if there isn't one
    void DataBufferGet(uint16_t **Buff, uint32_t *Used);
    //store data buffer
    void DataBufferPut(uint16_t *Buff, uint32_t Used);

    //get size of buffer
    size_t SizeGet() {return mSize;};

    void Stop();


  private:
    struct ContBuff
    {
      uint16_t *data;
      uint32_t used;
    };
    SafeQueue<ContBuff *> *mEmptyBuffers;
    SafeQueue<ContBuff *> *mDataBuffers;
    size_t mSize;
};

class ContinuousAdcWorker
{
  public:
    ContinuousAdcWorker(aiousb_device_handle Device, uint32_t BuffSize,
            uint32_t BaseBuffCount, void *Context, adc_cont_callback Callback);
    ~ContinuousAdcWorker();

  int Execute ();
  void Terminate();

  private:
    void ExecuteCapture();
    void ExecuteCallback();
    std::atomic<bool> mTerminated;
    std::thread *mCaptureThread;
    std::thread *mCallbackThread;
    ContinuousBufferManager *mBuffManager;
    aiousb_device_handle mDevice;
    void *mContext;
    adc_cont_callback mCallback;
    double mHertz;
};

} //namespace AIOUSB

#endif