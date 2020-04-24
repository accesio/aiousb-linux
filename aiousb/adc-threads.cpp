#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "aiousb-private.h"


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
      aiousb_generic_vendor_write(adc_worker_context->device,
                            0xbc,
                            adc_worker_context->bytes_left >> 17,
                            adc_worker_context->bytes_left >> 1,
                            sizeof(bc_data),
                            &bc_data);

      break;
    case bcs_dio:
      aiousb_generic_vendor_write(adc_worker_context->device,
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

    status = aiousb_generic_bulk_in(adc_worker_context->device,
                                0,
                                ad_buff,
                                transfer_length,
                                &this_transfer);

    if (status)
      {
        aiousb_library_err_print("aiousb_generic_bulk_in returned %d", status);
        goto ERR_XFER;
      }

    pthread_mutex_lock(&adc_worker_context->mutex);

    if (adc_worker_context->bytes_left < this_transfer)
      {
        this_transfer = adc_worker_context->bytes_left;
      }

    memcpy(current_tar, ad_buff, this_transfer);
    current_tar += this_transfer;
    adc_worker_context->bytes_left -= this_transfer;

    pthread_mutex_unlock(&adc_worker_context->mutex);

  }while (adc_worker_context->bytes_left > 0);

  aiousb_generic_vendor_write(adc_worker_context->device,
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
}

///////////////////adc continuous worker definitions

void adc_cont_acq_worker_do_bc_control(aiousb_device_handle device,
                                    uint32_t size, uint32_t control_data)
{
  aiousb_generic_vendor_write (device,
                              0xbc,
                              size >> 16, size,
                              sizeof(control_data),
                              &control_data);
}

void *adc_cont_acq_worker_execute (void *context)
{
  struct adc_cont_acq_worker_context *adc_cont_acq_worker_context = 
                                  (struct adc_cont_acq_worker_context *)context;

  uint32_t bytes_left, data_size, status;
  struct adc_continuous_buffer *this_buff;
  

  adc_cont_acq_worker_context->io_status = 0;

  if (adc_cont_acq_worker_context->bcs_style == bcs_dio)
    {
      aiousb_generic_vendor_write(adc_cont_acq_worker_context->device,
                                    0xbc,
                                    0x3,
                                    0,
                                    0,
                                    NULL);
    }
  else
    {
      if (adc_cont_acq_worker_context->b_counter_control)
        {
          aiousb_generic_vendor_write(adc_cont_acq_worker_context->device,
                                        0xc5,
                                        0x3,
                                        0,
                                        0,
                                        NULL);
        }
      else
        {
          adc_cont_acq_worker_do_bc_control(adc_cont_acq_worker_context->device,
                                              0,
                                              0x01000007);
        }
    }
  pthread_mutex_lock(&adc_cont_acq_worker_context->start_cond_mutex);
  pthread_cond_signal(&adc_cont_acq_worker_context->start_cond);
  pthread_mutex_unlock(&adc_cont_acq_worker_context->start_cond_mutex);

  
  while (!__sync_and_and_fetch (&adc_cont_acq_worker_context->terminate, 0x1))
  {

  }

}

