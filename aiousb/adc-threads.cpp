#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "aiousb-private.h"

namespace AIOUSB {

///////////////////adc_worker thread start
void *adc_worker_execute (void *context)
{
  struct adc_worker_context *adc_worker_context =
                            (struct adc_worker_context *)context;
  uint8_t *ad_buff = NULL;
  uint8_t *current_tar = (uint8_t *)adc_worker_context->tar;
  uint32_t bc_data;
  int this_transfer;
  int status;

  uint32_t transfer_length = MAX(adc_worker_min_block,
          MIN(adc_worker_context->bytes_left, adc_worker_context->block_size));

  ad_buff =(uint8_t *) malloc(transfer_length);

  if (ad_buff == NULL)
    {
      aiousb_library_err_print("Couldn't allocate buffer in adc_worker");
      return NULL;
    }

  switch (adc_worker_context->bcs_style)
  {
    case bcs_adc:
      bc_data = 0x5;
      GenericVendorWrite(adc_worker_context->device,
                            0xbc,
                            adc_worker_context->bytes_left >> 17,
                            adc_worker_context->bytes_left >> 1,
                            sizeof(bc_data),
                            &bc_data);

      break;
    case bcs_dio:
      GenericVendorWrite(adc_worker_context->device,
                            0xbc,
                            0x3,
                            0,
                            0,
                            NULL);
      break;
    default:
      aiousb_library_err_print("Default fault");
      return NULL;
      break;
  }

  do
  {
    pthread_mutex_lock(&adc_worker_context->mutex);
    if (adc_worker_context->b_abort)
      {
        goto ERR_ABORT;
      }
    pthread_mutex_unlock(&adc_worker_context->mutex);

    this_transfer = 0;

    status = AWU_GenericBulkIn(adc_worker_context->device,
                                0,
                                ad_buff,
                                transfer_length,
                                &this_transfer);

    if (status)
      {
        aiousb_library_err_print("AWU_GenericBulkIn returned %d", status);
        goto ERR_XFER;
      }

    pthread_mutex_lock(&adc_worker_context->mutex);

    if (adc_worker_context->bytes_left < (unsigned int)this_transfer)
      {
        this_transfer = adc_worker_context->bytes_left;
      }

    memcpy(current_tar, ad_buff, this_transfer);
    current_tar += this_transfer;
    adc_worker_context->bytes_left -= this_transfer;

    pthread_mutex_unlock(&adc_worker_context->mutex);

  }while (adc_worker_context->bytes_left > 0);

  GenericVendorWrite(adc_worker_context->device,
                          0xbc,
                          0,
                          0,
                          0,
                          NULL);

return NULL;

ERR_ABORT:
  free(ad_buff);
  pthread_mutex_unlock(&adc_worker_context->mutex);
  return NULL;

ERR_XFER:
  free(ad_buff);
  return NULL;

}

///////////////////adc_worker thread end

///////////////////adc_continuous buffer definitions



struct adc_continuous_buffer *
adc_cont_buff_worker_extra_buf (struct adc_cont_buff_worker_context *context)
{
  struct adc_continuous_buffer *retval = NULL;
  pthread_mutex_lock(&context->buff_mutex);

  retval =(struct adc_continuous_buffer*) malloc(sizeof(struct adc_continuous_buffer));

  if (retval == NULL)
    {
      aiousb_library_err_print("Unable to allocate memory");
      exit(-ENOMEM);
    }

  context->num_buffs++;
  context->buf_buf = (adc_continuous_buffer_handle *) realloc(context->buf_buf,
              sizeof (struct adc_continuous_buffer *) * context->num_buffs);

  if (context->buf_buf == NULL)
    {
      aiousb_library_err_print("Unable to allocate_memory");
      exit(-ENOMEM);
    }

  context->buf_buf[context->num_buffs - 1] = retval;
  retval->buf_state = buf_state_accessing;

  pthread_mutex_unlock(&context->buff_mutex);

  return retval;

}

struct adc_continuous_buffer *
adc_cont_buff_worker_get_blank_buff (struct adc_cont_buff_worker_context *context,
                             uint32_t flags)
{
  struct adc_continuous_buffer *ret_val = NULL;
  int i;
  if (sem_trywait(&context->blank_buf_sem) == -EAGAIN)
    {
      ret_val = adc_cont_buff_worker_extra_buf(context);
      ret_val->flags |= ADC_CONT_CALLBACK_FLAG_INSERTED;
      return ret_val;
    }

  pthread_mutex_lock(&context->buff_mutex);

  for ( i = 0 ; i < context->num_buffs ; i++)
    {
      if (context->buf_buf[i]->buf_state == buf_state_blank)
        {
          ret_val = context->buf_buf[i];
          ret_val->flags |= flags;
          break;
        }
    }

  pthread_mutex_unlock(&context->buff_mutex);

  if (ret_val == NULL)
    {
      aiousb_library_err_print("Exiting on condition that should never occur");
      exit(-1);
    }

  return ret_val;

}

void adc_cont_buff_worker_put_blank_buff(struct adc_cont_buff_worker_context *context,
                                        struct adc_continuous_buffer *cont_buf)
{
  pthread_mutex_lock(&context->buff_mutex);
  cont_buf->buf_state = buf_state_blank;
  pthread_mutex_unlock(&context->buff_mutex);
  sem_post(&context->blank_buf_sem);
}

void adc_cont_buff_worker_put_data_buff(struct adc_cont_buff_worker_context *context,
                                        struct adc_continuous_buffer *cont_buff)
{
  pthread_mutex_lock(&context->buff_mutex);
  cont_buff->buf_state = buf_state_got_data;
  cont_buff->index=context->next_index_in;
  context->next_index_in++;
  pthread_mutex_unlock(&context->buff_mutex);
  sem_post(&context->data_buf_sem);
}

struct adc_continuous_buffer *
adc_cont_buff_worker_get_data_buf_or_killed (struct adc_cont_buff_worker_context *context)
{
  struct adc_continuous_buffer * ret_val = NULL;
  int i;

  do
    {
      sem_wait(&context->data_buf_sem);

      pthread_mutex_lock(&context->buff_mutex);

      for (i=0; i < context->num_buffs ; i++)
        {
          if ((context->buf_buf[i]->index == context->next_index_out) &&
              (context->buf_buf[i]->buf_state == buf_state_got_data))
          {
            ret_val = context->buf_buf[i];
            ret_val->buf_state = buf_state_accessing;
            context->next_index_out++;
          }
        }
      if (__sync_and_and_fetch(&context->terminate, 0x1))
        {
            return NULL;
        }
    }while (ret_val == NULL);
    return ret_val;
}

void *adc_cont_buff_worker_execute(void *context)
{
  struct adc_cont_buff_worker_context *this_context =
                            (struct adc_cont_buff_worker_context *)context;
  struct adc_continuous_buffer *this_buff = NULL;

  do
    {
      this_buff = adc_cont_buff_worker_get_data_buf_or_killed(this_context);

      if (this_buff == NULL)
        {
          (*this_context->callback)(NULL,
                                  0,
                                  ADC_CONT_CALLBACK_FLAG_END_STREAM,
                                  this_context->callback_context);
        }
      else if ((this_buff->used_size) || (this_buff->flags))
        {
          (*this_context->callback)((uint16_t*)this_buff->ad_buff,
                                    this_buff->used_size,
                                    this_buff->flags,
                                    this_context->callback_context);
        }

    }while (!__sync_and_and_fetch(&this_context->terminate, 0x1));
    return NULL;
}

///////////////////adc continuous worker definitions

void adc_cont_acq_worker_do_bc_control(aiousb_device_handle device,
                                    uint32_t size, uint32_t control_data)
{
  GenericVendorWrite (device,
                              0xbc,
                              size >> 16, size,
                              sizeof(control_data),
                              &control_data);
}

void *adc_cont_acq_worker_execute (void *context)
{
  struct adc_cont_acq_worker_context *this_context =
                                  (struct adc_cont_acq_worker_context *)context;

  uint32_t bytes_left, data_size, status;
  struct adc_continuous_buffer *this_buff;


  this_context->io_status = 0;

  if (this_context->bcs_style == bcs_dio)
    {
      GenericVendorWrite(this_context->device,
                                    0xbc,
                                    0x3,
                                    0,
                                    0,
                                    NULL);
    }
  else
    {
      //counter control is always zero
      if (this_context->b_counter_control)
        {
          GenericVendorWrite(this_context->device,
                                        0xc5,
                                        0x3,
                                        0,
                                        0,
                                        NULL);
        }
      else
        {
          adc_cont_acq_worker_do_bc_control(this_context->device,
                                              0,
                                              0x01000007);
        }
    }
  pthread_mutex_lock(&this_context->start_cond_mutex);
  pthread_cond_signal(&this_context->start_cond);
  pthread_mutex_unlock(&this_context->start_cond_mutex);


  while (!__sync_and_and_fetch (&this_context->terminate, 0x1))
  {
    this_buff = adc_cont_buff_worker_get_blank_buff(
      this_context->adc_cont_buff_worker_context,
      0);
    this_buff->used_size = 0;

    status = AWU_GenericBulkIn(this_context->device,
      0,
      this_buff->ad_buff,
      this_context->adc_cont_buff_worker_context->bytes_per_buff,
      (int *)&this_buff->used_size);

     if (status)
       {
         if (this_context->bcs_style == bcs_dio)
            {
              GenericVendorWrite(this_context->device,
                                            0xbc,
                                            0x10,
                                            0,
                                            0,
                                            NULL);
            }
         else
            {
              adc_cont_acq_worker_do_bc_control(this_context->device, 0, 0x00020002);
            }
            //TODO: Is there a clean way out of this?
            aiousb_library_err_print("error in bulk in call. Bailing");
            pthread_exit(NULL);
       }
      if (this_buff->used_size != 0)
        {
          adc_cont_buff_worker_put_data_buff(
                                    this_context->adc_cont_buff_worker_context,
                                    this_buff);
        }
      else
        {
          adc_cont_buff_worker_put_blank_buff(
                                    this_context->adc_cont_buff_worker_context,
                                    this_buff);
        }
  }

  if (this_context->bcs_style == bcs_dio)
    {
      GenericVendorWrite(this_context->device,
                                    0xbc,
                                    0x10,
                                    0,
                                    0,
                                    NULL);
    }
  else
    {
      adc_cont_acq_worker_do_bc_control(this_context->device, 0, 0x00020002);
    }

  bytes_left = 0;
  data_size = sizeof(bytes_left);

  GenericVendorRead(this_context->device,
                          0xbc,
                          0,
                          0,
                          data_size, &bytes_left);

  bytes_left &= 0xffff;

  if ( 0 != bytes_left)
    {
      this_buff = adc_cont_buff_worker_get_blank_buff(
                                this_context->adc_cont_buff_worker_context,
                                0);
      this_buff->used_size = 0;
      data_size=this_context->adc_cont_buff_worker_context->bytes_per_buff;

      bytes_left = data_size < bytes_left ? data_size : bytes_left;

      status = AWU_GenericBulkIn(this_context->device,
                                          0,
                                          this_buff->ad_buff,
                                          bytes_left,
                                          (int *)&this_buff->used_size);

      if ((status) || (this_buff->used_size == 0))
        {
          adc_cont_buff_worker_put_blank_buff(
                              this_context->adc_cont_buff_worker_context,
                              this_buff);
        }
      else
        {
          adc_cont_buff_worker_put_data_buff(
                                this_context->adc_cont_buff_worker_context,
                                this_buff);
        }
    }
  return 0;
}

ContinuousBufferManager::ContinuousBufferManager(int Count, size_t Size) : mSize(Size)
{
  mEmptyBuffers = new SafeQueue<ContBuff *>();
  for (int i = 0 ; i < Count ; i++)
    {
      ContBuff *Current = new ContBuff;
      Current->data = new uint16_t[Size];
      Current->used = 0;

      mEmptyBuffers->enqueue(Current);
    }
  mDataBuffers = new SafeQueue<ContBuff *>();
}

ContinuousBufferManager::~ContinuousBufferManager()
{
  ContBuff *Current;
  do
    {
      Current = mEmptyBuffers->tryDequeue();
      if (Current != NULL )
      {
        //Sometimes this causes a segfault for double free
        //delete[] Current->data;
        delete Current;
      }
    }while (Current != NULL);
    delete mEmptyBuffers;

  do
    {
      Current = mDataBuffers->tryDequeue();
      if (Current != NULL )
      {
        //Sometimes this causes a segfault for double free
        //delete[] Current->data;
        delete Current;
      }
    }while (Current != NULL);
    delete mDataBuffers;
}

uint16_t* ContinuousBufferManager::EmptyBufferGet()
{
  ContBuff *Buff;
  uint16_t *retval;

  Buff = mEmptyBuffers->tryDequeue();

  if (Buff == NULL)
    {
      retval = new uint16_t[mSize];
    }
  else
  {
    retval = Buff->data;
    delete Buff;
  }


  return retval;
}

void ContinuousBufferManager::EmptyBufferPut(uint16_t* Buff)
{
  ContBuff *Current = new ContBuff;
  Current->data = Buff;
  Current->used = 0;
  mEmptyBuffers->enqueue(Current);
}

void ContinuousBufferManager::DataBufferGet(uint16_t **Buff, uint32_t *Used)
{
  ContBuff *Current = mDataBuffers->dequeue();
  if (Current != nullptr)
  {
    *Buff = Current->data;
    *Used = Current->used;
    delete Current;
  }
}

void ContinuousBufferManager::DataBufferPut(uint16_t* Buff, uint32_t Used)
{
  ContBuff *Current = new ContBuff;
  Current->data = Buff;
  Current->used = Used;
  mDataBuffers->enqueue(Current);
}

void ContinuousBufferManager::Stop()
{
  mDataBuffers->Stop();
}

ContinuousAdcWorker::ContinuousAdcWorker(aiousb_device_handle Device,
    uint32_t BuffSize, uint32_t BaseBuffCount, void *Context,
    adc_cont_callback Callback)
    : mDevice(Device)
    , mContext(Context)
    , mCallback(Callback)
{
  mBuffManager = new ContinuousBufferManager(BaseBuffCount, BuffSize);
  mTerminated = false;
}

ContinuousAdcWorker::~ContinuousAdcWorker()
{
  if (!mTerminated) Terminate();
  delete mBuffManager;
}

int ContinuousAdcWorker::Execute()
{
  mCaptureThread = new std::thread(&ContinuousAdcWorker::ExecuteCapture, this);
  mCallbackThread = new std::thread(&ContinuousAdcWorker::ExecuteCallback, this);
  return 0;
}

void ContinuousAdcWorker::Terminate()
{
  mTerminated = true;
  mBuffManager->Stop();
  mCallbackThread->join();
  mCaptureThread->join();
}

void ContinuousAdcWorker::ExecuteCapture ()
{
  uint32_t bytes_left, status, control_data;
  uint16_t *this_buff;
  uint32_t used;

  if (mDevice->descriptor.b_adc_dio_stream) //bcs_dio in reference code
    {
      GenericVendorWrite(mDevice,
                                    0xbc,
                                    0x3,
                                    0,
                                    0,
                                    NULL);
    }
  else
    {
      control_data = 0x01000007;
      GenericVendorWrite(mDevice,
                                      0xbc,
                                      0,
                                      0,
                                      sizeof(control_data),
                                      &control_data);
    }

    while (!mTerminated)
      {
        this_buff = mBuffManager->EmptyBufferGet();

        status = AWU_GenericBulkIn(mDevice,
                                      0,
                                      this_buff,
                                      mBuffManager->SizeGet(),
                                      (int *)&used);

        if (status)
        {
          aiousb_library_err_print("bulk_in fail");
        }
        if (used != 0)
          {
            mBuffManager->DataBufferPut(this_buff, used);
          }
        else
        {
          mBuffManager->EmptyBufferPut(this_buff);
        }
      }

    if (mDevice->descriptor.b_adc_dio_stream) //bcs_dio in reference code
    {
      GenericVendorWrite(mDevice,
                                    0xbc,
                                    0x10,
                                    0,
                                    0,
                                    NULL);
    }
  else
    {
      control_data = 0x00020002;
      GenericVendorWrite(mDevice,
                                      0xbc,
                                      0,
                                      0,
                                      sizeof(control_data),
                                      &control_data);
    }

    bytes_left = 0;

    GenericVendorRead(mDevice,
                          0xbc,
                          0,
                          0,
                          sizeof(bytes_left), &bytes_left);

    bytes_left &= 0xffff;

    if ( bytes_left )
      {
        this_buff = mBuffManager->EmptyBufferGet();

        status = AWU_GenericBulkIn(mDevice,
                                        0,
                                        this_buff,
                                        bytes_left,
                                        (int *)&used);

        if ((status) || (!used))
          {
            mBuffManager->EmptyBufferPut(this_buff);
          }
        else
          {
            mBuffManager->DataBufferPut(this_buff, used);
          }
      }
}


#include <unistd.h>
void ContinuousAdcWorker::ExecuteCallback ()
{
  uint16_t *buff = nullptr;
  uint32_t used;
  while (!mTerminated)
  {
    mBuffManager->DataBufferGet(&buff, &used);
    //TODO: For now Not doing the flags. Need to discuss with other team members
    //about need.
    if (buff != nullptr)
    {
      mCallback(buff, used, 0, mContext);

      mBuffManager->EmptyBufferPut(buff);
    }
  }
  sleep(1);

}

} //namespace AIOUSB

