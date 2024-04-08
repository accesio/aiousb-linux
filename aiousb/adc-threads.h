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

void *adc_worker_execute (void *context);

class ContinuousBufferManager
{
  public:
    ContinuousBufferManager(int Count, size_t Size);
    ~ContinuousBufferManager();

    //get or create empty buffer
    uint16_t* EmptyBufferGet(uint32_t *created);
    //store empty buffer
    void EmptyBufferPut(uint16_t *Buff);

    //get data buffer. wait if there isn't one
    void DataBufferGet(uint16_t **Buff, uint32_t *Used, uint32_t *Flags);
    //store data buffer
    void DataBufferPut(uint16_t* Buff, uint32_t Used, uint32_t Flags);

    //get size of buffer
    size_t SizeGet() {return mSize;};

    void Stop();


  private:
    struct ContBuff
    {
      uint16_t *data;
      uint32_t used;
      uint32_t flags;
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