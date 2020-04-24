/*
 * Copyright (c), ACCES I/O Products, Inc.
 * All rights reserved.
 * 
 * Contributor(s):
 * Zach Perez, zach.perez@accesio.com
 * 
 * PERMISSION TO USE, COPY, MODIFY, AND/OR DISTRIBUTE THIS SOFTWARE FOR ANY
 * PURPOSE WITH OR WITHOUT FEE IS HEREBY GRANTED, PROVIDED THAT THE ABOVE
 * COPYRIGHT NOTICE AND THIS PERMISSION NOTICE APPEAR IN ALL COPIES.
 * 
 * THIS SOFTWARE IS PROVIDED BY ACCES I/O AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ACCES I/O AND CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define MODULE_NAME "ACCESIO_USB"
#define MODULE_PREFIX MODULE_NAME": "

#define ACCESIO_MAJOR  160
#define ACCESIO_USB_BUF_SZ 64




// #include "../common/driver.h"
// #include "../common/ioctl.h"
// #include "../common/usbdev.h"

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/completion.h>

#include <linux/version.h>
#include <linux/firmware.h>

#include "ids.h"
//#include "module/declarations.h"
//#include "module/types.h"
//#include "module/driver.h"

#include "accesio_usb_ioctl.h"

#define ACCESIO_USB_MINOR	192

#define ACCESIO_USB_ANCHOR_TIMEOUT 1000

#define aio_driver_err_print(fmt, ...) \
				do { printk( "%s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__); } while (0);

#ifndef AIO_DEBUG
#define AIO_DEBUG 1
#endif

#define aio_driver_debug_print(fmt, ...) \
				do { if (AIO_DEBUG) printk ("%s:%d:%s(): " fmt "\n" , __FILE__, \
																__LINE__, __func__, ##__VA_ARGS__); } while (0)

//TODO: remove when done with initial driver development
#define aio_driver_dev_print(fmt, ...) \
				do { if (AIO_DEBUG) printk ("%s:%d:%s(): " fmt "\n" , __FILE__, \
																__LINE__, __func__, ##__VA_ARGS__); } while (0)





/*
 * These are the requests (bRequest) that the bootstrap loader is expected
 * to recognize.  The codes are reserved by Cypress, and these values match
 * what EZ-USB hardware, or "Vend_Ax" firmware (2nd stage loader) uses.
 * Cypress' "a3load" is nice because it supports both FX and FX2, although
 * it doesn't have the EEPROM support (subset of "Vend_Ax").
 */
//TODO: Look into using the firmware loader built into the kernel
//cypress_firmware.h
#define ACCESIO_USB_REQ_INT 0xA0  // hardware implements this one
#define ACCESIO_USB_REQ_MEM 0xA3
#define ACCESIO_USB_WRITE_RETRY 5
#define ACCESIO_USB_RAM_REG 0xE600

static int accesio_major_num;
static dev_t accesio_first_dev = MKDEV(ACCESIO_MAJOR,0);



//TODO: Figure out if we should make the epd a pointer or a copy.
struct accesio_usb_endpoint {
    struct usb_endpoint_descriptor *epd;
} accesio_usb_endpoint;

#define ACCES_MAX_BULK_ENDPOINTS 2

struct accesio_usb_device_info {
    struct usb_device* udev;         /* the usb kernel device for this device */
    struct usb_interface* interface; /* the interface for this device */
//    struct semaphore limit_sem;      /* limiting the number of writes in progress */
    struct usb_anchor submitted;     /* in case we need to retract our submissions */
    struct accesio_usb_endpoint bulk_in[ACCES_MAX_BULK_ENDPOINTS];
    struct accesio_usb_endpoint bulk_out[ACCES_MAX_BULK_ENDPOINTS];
    unsigned int bulk_in_count;
    unsigned int bulk_out_count;
    int errors;                      /* the last request tanked */
//    bool ongoing_read;               /* a read is going on */
//    spinlock_t err_lock;             /* lock for errors */
    struct kref kref;                /* kernel reference object */
//    struct mutex io_mutex;           /* synchronize I/O with disconnect */
    
//    bool ctrl_msg;                    /* true if currently sending a urb */
//    uint32_t device_index;            /* the device index of the dev tree */
    //TODO: Maybe replace this with it's acces_usb_device_descriptor or add the
    //descriptor, but maybe only used in probe
    uint32_t product_id;              /* the devices product id */
    struct acces_usb_device_descriptor const *acces_usb_device_descriptor;
    struct urb *urb;
    struct completion urb_completion;
} accesio_usb_device_info;






#define DRIVER_NAME "accesio_usb"
#define DRIVER_VERSION "2.0"
#define DRIVER_AUTHOR "Zach Perez <zach.perez@accesio.com>"
#define DRIVER_DESC "ACCES I/O Products, Inc. USB driver"
#define DRIVER_LICENSE "Dual MIT/GPL"

static char* accesio_usb_get_devnode(struct device* dev, umode_t* mode);
static int accesio_usb_probe(struct usb_interface* interface, const struct usb_device_id* id);
static void accesio_usb_disconnect(struct usb_interface* interface);
static void accesio_usb_draw_down(struct accesio_usb_device_info* dev);
#if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)
    static int accesio_usb_suspend(struct usb_interface* intf, pm_message_t message);
    static int accesio_usb_resume(struct usb_interface* intf);
#endif
static int accesio_usb_pre_reset(struct usb_interface* intf);
static int accesio_usb_post_reset(struct usb_interface* intf);
static void accesio_usb_draw_down(struct accesio_usb_device_info* dev);
static void accesio_usb_delete(struct kref* kref);
static int accesio_usb_open(struct inode* inode, struct file* file);
static int accesio_usb_release(struct inode* inode, struct file* file);
static int accesio_usb_flush(struct file* file, fl_owner_t id);
static ssize_t accesio_usb_read(struct file* file, char* buffer, size_t count, loff_t* ppos);
static ssize_t accesio_usb_write(struct file* file, const char* user_buffer, size_t count, loff_t* ppos);
static loff_t accesio_usb_seek(struct file* filp, loff_t off, int origin);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
static int accesio_usb_ioctl(struct inode* inode, struct file* filp, unsigned int cmd, unsigned long arg);
#else 
static long accesio_usb_ioctl(struct file* filp, unsigned int cmd, unsigned long arg);
#endif

static struct usb_driver accesio_usb_driver = {
    .name = DRIVER_NAME,
    .id_table = NULL,
    .probe = accesio_usb_probe,
    .disconnect = accesio_usb_disconnect,
#if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)
    .suspend = accesio_usb_suspend,
    .resume = accesio_usb_resume,
    .supports_autosuspend = 1,
#endif
    .pre_reset = accesio_usb_pre_reset,
    .post_reset = accesio_usb_post_reset,
    .no_dynamic_id = 1,
};

static const struct file_operations accesio_usb_file_ops = {
    .owner = THIS_MODULE,
    .read = accesio_usb_read,
    .write = accesio_usb_write,
    .open = accesio_usb_open,
    .release = accesio_usb_release,
    .flush = accesio_usb_flush,
    .llseek = accesio_usb_seek,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
    .ioctl          = accesio_usb_ioctl,
#else
    .unlocked_ioctl = accesio_usb_ioctl,
#endif
};
#define DRIVER_CLASS_NAME "accesio"
/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver accesio_usb_class = {
    .name = DRIVER_CLASS_NAME,
    .fops = &accesio_usb_file_ops,
    .devnode = &accesio_usb_get_devnode,
    .minor_base = ACCESIO_USB_MINOR,
};

/*
 * For writing to RAM using a first (hardware) or second (software)
 * stage loader and 0xA0 or 0xA3 vendor requests
 */
typedef enum {
    _undef = 0,
    internal_only = 1,  // hardware first-stage loader
    skip_internal = 2,  // first phase, second-stage loader
    skip_external = 3   // second phase, second-stage loader
} ram_mode;

typedef struct ram_poke_context {
    struct accesio_usb_device_info* device;
    ram_mode mode;
    unsigned int total;
    unsigned int count;
} ram_poke_context;

// static void accesio_usb_print_ep(const char* epname, accesio_usb_endpoint* ep)
// {
//     printk(KERN_INFO KBUILD_MODNAME ": [%s=(I)0x%04X:%zu|(O)0x%04X:%zu].\n", epname,
//                             ep->in.address, ep->in.buffer_size,
//                             ep->out.address, ep->out.buffer_size);
// }

// static void accesio_usb_print_dev(accesio_usb_device_info* dev, const char* msg, bool pep)
// {
//     printk(KERN_INFO KBUILD_MODNAME ": %s0x%04X=%s_%d[:%d].\n",
//         ((msg != NULL) ? msg : ""),
//         dev->product_id,
//         get_name_from_id(dev->product_id),
//         dev->device_index,
//         ((dev->interface != NULL) ? dev->interface->minor : -1));
//     if (pep) {
//         accesio_usb_print_ep("bulk", &dev->endpoints.bulk);
//         accesio_usb_print_ep("ctrl", &dev->endpoints.control);
//         accesio_usb_print_ep("iso", &dev->endpoints.isochronous);
//         accesio_usb_print_ep("int", &dev->endpoints.interrupt);
//     }
// }

// static int accesio_usb_bulk_read(accesio_usb_device_info* dev, void* data, uint16_t len)
// {
//     int actual_length = 0;
//     int rc = mutex_lock_interruptible(&dev->io_mutex);
//     ACCESIO_VMAP_DATA_DECL
//     if (rc < 0) { return rc; }
//     // disconnect() was called
//     if (!dev->interface) {
//         mutex_unlock(&dev->io_mutex);
//         return -ENODEV;
//     }
//     ACCESIO_VMAP_ALLOC(len)
//     rc = usb_bulk_msg(dev->udev,                                                    // usb_device
//                       usb_rcvbulkpipe(dev->udev, dev->endpoints.bulk.in.address),   // pipe
//                       ACCESIO_VMAP_DATA_REF(data),                                  // data
//                       len,                                                          // len
//                       &actual_length,                                               // xfr/rcv'd
//                       USB_CTRL_SET_TIMEOUT);                                        // timeout
//     ACCESIO_VMAP_COPY_DATA(data, len)
//     ACCESIO_VMAP_FREE
//     mutex_unlock(&dev->io_mutex);
//     if (actual_length == len) {
//         rc = 0;
//     } else {
//         printk(KERN_INFO KBUILD_MODNAME ": error reading bulk %d, data len = %u, actually sent = %u.\n", rc, len, actual_length);
//     }
//     return rc;
// }

// static int accesio_usb_bulk_msg(accesio_usb_device_info* dev, void* data, uint16_t len)
// {
//     int actual_length = 0;
//     int rc = mutex_lock_interruptible(&dev->io_mutex);
//     ACCESIO_VMAP_DATA_DECL
//     if (rc < 0) { return rc; }
//     // disconnect() was called
//     if (!dev->interface) {
//         mutex_unlock(&dev->io_mutex);
//         return -ENODEV;
//     }
//     dev->ctrl_msg = true;
//     ACCESIO_VMAP_ALLOC(len)
//     ACCESIO_VMAP_DATA_COPY(data, len)
//     rc = usb_bulk_msg(dev->udev,                                                    // usb_device
//                       usb_sndbulkpipe(dev->udev, dev->endpoints.bulk.out.address),  // pipe
//                       ACCESIO_VMAP_DATA_REF(data),                                  // data
//                       len,                                                          // len
//                       &actual_length,                                               // xfr/rcv'd
//                       USB_CTRL_SET_TIMEOUT);                                        // timeout
//     ACCESIO_VMAP_FREE
//     if (actual_length == len) {
//         rc = 0;
//     } else {
//         printk(KERN_INFO KBUILD_MODNAME ": error sending bulk %d, data len = %u, actually sent = %u.\n", rc, len, actual_length);
//     }
//     dev->ctrl_msg = false;
//     mutex_unlock(&dev->io_mutex);
//     return rc;
// }

// static int accesio_usb_ctrl_read(accesio_usb_device_info* dev, uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t len)
// {
//     int rc = mutex_lock_interruptible(&dev->io_mutex);
//     ACCESIO_VMAP_DATA_DECL
//     if (rc < 0) { return rc; }
//     // disconnect() was called
//     if (!dev->interface) {
//         mutex_unlock(&dev->io_mutex);
//         return -ENODEV;
//     }
//     ACCESIO_VMAP_ALLOC(len)
//     rc = usb_control_msg(dev->udev,                                                      // usb_device
//                          usb_rcvctrlpipe(dev->udev, dev->endpoints.control.in.address),  // pipe
//                          request,                                                        // request 
//                          (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_RECIP_DEVICE),     // req_type
//                          value,                                                         // value
//                          index,                                                         // index
//                          ACCESIO_VMAP_DATA_REF(data),                                   // data
//                          len,                                                           // len
//                          USB_CTRL_SET_TIMEOUT);                                         // timeout
//     ACCESIO_VMAP_COPY_DATA(data, len)
//     ACCESIO_VMAP_FREE
//     mutex_unlock(&dev->io_mutex);
//     return rc;
// }


static int accesio_usb_ctrl_msg(struct accesio_usb_device_info* dev, uint8_t request, uint16_t value, uint16_t index, void* data, uint16_t len)
{
    //TODO: See if we can do a run time check of the memory to see if it is dma capable
    //If not have this function call the accesio_usb_ctrl_msg_user
//    int rc = mutex_lock_interruptible(&dev->io_mutex);
    int rc;
    void *dma_capable_buffer = NULL; //usb_control_message doesn't like stack data
//    if (rc < 0) { return rc; }
    // disconnect() was called
    // if (!dev->interface) {
    //     mutex_unlock(&dev->io_mutex);
    //     return -ENODEV;
    // }
//    dev->ctrl_msg = true;
    dma_capable_buffer = kmalloc(len, GFP_KERNEL);
    memcpy(dma_capable_buffer, data, len);
    rc = usb_control_msg(dev->udev,                                                      // usb_device
                         usb_sndctrlpipe(dev->udev, 0),                                                              // pipe
                         request,                                                        // request 
                         (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_RECIP_DEVICE),     // req_type
                         value,                                                         // value
                         index,                                                         // index
                         dma_capable_buffer         ,                                   // data
                         len,                                                           // len
                         USB_CTRL_SET_TIMEOUT);                                         // timeout
    kfree(dma_capable_buffer);
//    dev->ctrl_msg = false;
//    mutex_unlock(&dev->io_mutex);
    return rc;
}

static int accesio_usb_write_ram(ram_poke_context* ctx, unsigned short addr, unsigned char* data, size_t len)
{
    int rc = 0;
    unsigned int retry = 0;
    // addr+len should be < 16k
    if ((addr + len) > 0x4000) {
        return -EINVAL;
    }
    ctx->total += len;
    ctx->count++;
    do {
        // Retry this till we get a real error. Control messages are not
        // NAK'd (just dropped) so time out means is a real problem.
        rc = accesio_usb_ctrl_msg(ctx->device, ACCESIO_USB_REQ_INT, addr, 0, data, len);
    } while ((rc < 0) && (++retry < ACCESIO_USB_WRITE_RETRY) && (rc != ETIMEDOUT));

    return (rc < 0) ? rc : 0;
}

/*
 * Parse an Intel HEX image file and invoke the poke() function on the
 * various segments to implement policies such as writing to RAM (with
 * a one or two stage loader setup, depending on the firmware) or to
 * EEPROM (two stages required).
 *
 * image - the hex image file
 * context - for use by poke()
 * is_external - if non-null, used to check which segments go into
 *    external memory (writable only by software loader)
 * poke  - called with each memory segment; errors indicated
 *    by returning negative values.
 *
 * Caller is responsible for halting CPU as needed, such as when
 * overwriting a second stage loader.
 */
static int accessio_parse_and_send_ihex(const uint8_t* sdata, size_t slen, struct accesio_usb_device_info* udev)
{
    const size_t BUF_SZ = ACCESIO_USB_BUF_SZ; // udev->endpoints.control.out.buffer_size;
    const size_t DATA_SZ = (BUF_SZ * 2) - 1;
    ram_poke_context context;
    unsigned char* data = kzalloc(DATA_SZ, GFP_KERNEL);
    char* buf = kzalloc(BUF_SZ, GFP_KERNEL);
    unsigned short data_addr = 0;
    size_t data_len = 0;
    size_t pos = 0;
    int rc;
    int first_line = 1;

    memset(data, 0, DATA_SZ);
    context.mode = internal_only;
    context.device = udev;
    context.total = 0;
    context.count = 0;

    /* Read the input file as an IHEX file, and report the memory segments
     * as we go.  Each line holds a max of 16 bytes, but downloading is
     * faster (and EEPROM space smaller) if we merge those lines into larger
     * chunks.  Most hex files keep memory segments together, which makes
     * such merging all but free.  (But it may still be worth sorting the
     * hex files to make up for undesirable behavior from tools.)
     *
     * Note that EEPROM segments max out at 1023 bytes; the download protocol
     * allows segments of up to 64 KBytes (more than a loader could handle).
     */
    for (;;) {
        size_t len = 0;
        size_t idx = 0;
        size_t off = 0;
        size_t type = 0;
        char* cp = NULL;
        char tmp = 0;
        memset(buf, 0, BUF_SZ);

        // cp = fgets(buf, sizeof buf, image);
        for (len = 0; len < BUF_SZ && pos < slen; ++len, ++pos) {
            idx = len;
            buf[len] = sdata[pos];
            if (buf[len] == '\n') {
                buf[len] = 0;
                ++pos;
                break;
            }
            if (buf[len] == '\0') {
                printk(KERN_INFO KBUILD_MODNAME ": EOF w/o EOF record found in FW @ pos = %zu with len = %zu.\n", pos, len);
                rc = -EFAULT;
                goto error;
            }
        }
        if (idx == 0) {
            // EOF w/o EOF record
            printk(KERN_INFO KBUILD_MODNAME ": EOF w/o EOF record found in FW @ pos = %zu.\n", pos);
            rc = -EFAULT;
            goto error;
        }

        // EXTENSION: "# comment-till-end-of-line", for copyrights etc
        if (buf[0] == '#') { continue; }
        if (buf[0] != ':') { // invalid IHEX record
            rc = -EINVAL;
            goto error;
        }

        // this is done in the for loop
        /* ignore any newline
        cp = strchr(buf, '\n');
        if (cp) { *cp = 0; } */

        // Read the length field (up to 16 bytes)
        tmp = buf[3];
        buf[3] = 0;
        //len = strtoul(buf+1, 0, 16);
        rc = kstrtoul(buf+1, 16, &len);
        if (rc < 0) {
            goto error;
        }
        buf[3] = tmp;
        // Read the target offset (address up to 64KB)
        tmp = buf[7];
        buf[7] = 0;
        //off = strtoul(buf+3, 0, 16);
        rc = kstrtoul(buf+3, 16, &off);
        if (rc < 0) {
            goto error;
        }
        buf[7] = tmp;
        // Initialize data_addr
        if (first_line) {
            data_addr = off;
            first_line = 0;
        }
        // Read the record type
        tmp = buf[9];
        buf[9] = 0;
        //type = strtoul(buf+7, 0, 16);
        rc = kstrtoul(buf+7, 16, &type);
        if (rc < 0) {
            goto error;
        }
        buf[9] = tmp;

        // If this is an EOF record, then make it so.
        if (type == 1) { break; }
        if (type != 0) { // unsupported record type
            rc = -EINVAL;
            goto error;
        }
        
        if ((len * 2) + 11 > strnlen(buf, BUF_SZ)) { // record too short
            rc = -ENXIO;
            goto error;
        }

        /* FIXME check for _physically_ contiguous not just virtually
        e.g. on FX2 0x1F00-0x2100 includes both on-chip and external
        memory so it's not really contiguous */

        // flush the saved data if it's not contiguous, or when we've buffered as much as we can.
        if ((data_len != 0) && (off != (data_addr + data_len) || ((data_len + len) > DATA_SZ))) {
            rc = accesio_usb_write_ram(&context, data_addr, data, data_len);
            if (rc < 0) {
                goto error;
            }
            data_addr = off;
            data_len = 0;
        }
        // append to saved data, flush later
        for (idx = 0, cp = buf+9; idx < len; ++idx, cp += 2) {
            tmp = cp[2];
            cp[2] = 0;
            //data[data_len + idx] = strtoul(cp, 0, 16);
            rc = kstrtoul(cp, 16, &off);
            if (rc < 0) {
                goto error;
            }
            data[data_len + idx] = off;
            cp[2] = tmp;
        }
        data_len += len;
    }

    // flush any data remaining
    if (data_len != 0) {
        rc =  accesio_usb_write_ram(&context, data_addr, data, data_len);
        if (rc < 0) {
            goto error;
        }
    }
    rc = 0;
    
    error:
    kfree(data);
    kfree(buf);
    return rc;
}

/*
 * Modifies the CPUCS register to stop or reset the CPU.
 * Returns false on error.
 */
static int accesio_usb_set_cpu_runstate(struct accesio_usb_device_info* device, bool run)
{
    unsigned char data = run ? 0 : 1;
    int rc = accesio_usb_ctrl_msg(device, ACCESIO_USB_REQ_INT, ACCESIO_USB_RAM_REG, 0, &data, 1);
    if (rc < 1) {
        printk(KERN_INFO KBUILD_MODNAME ": error modifying run state %d.\n", rc);
        return rc;
    }
    return 0;
}

static int accesio_usb_load_fw(struct accesio_usb_device_info* udev)
{
    /*
        The following search paths are used to look for firmware on your root filesystem.
        -fw_path_para - module parameter - default is empty so this is ignored
        -/lib/firmware/updates/UTS_RELEASE/
        -/lib/firmware/updates/
        -/lib/firmware/UTS_RELEASE/
        -/lib/firmware/
    */
    int rc = 0;
    const struct firmware* fw = NULL;
    const char* fwname = udev->acces_usb_device_descriptor->fw_fname;
    if (fwname == NULL) {
        printk(KERN_INFO KBUILD_MODNAME ": could not find firmware name for product 0x%04X.\n", udev->product_id);
        return -ENODEV;
    }
//    printk(KERN_INFO KBUILD_MODNAME ": requesting firmware %s for device[%d] 0x%04X\n", fwname, udev->device_index, udev->product_id);
    rc = request_firmware(&fw, fwname, &udev->udev->dev);
    if (rc || fw == NULL) {
        // printk(KERN_INFO KBUILD_MODNAME ": error requesting firmware %s for device[%d] 0x%04X, error: %d.\n",
        //        fwname, udev->device_index, udev->product_id, rc);
        return ((rc < 0) ? rc : -EIO);
    }
    printk(KERN_INFO KBUILD_MODNAME ": firmware size = %zu bytes.\n", fw->size);
    // don't let CPU run while we overwrite its code/data
    rc = accesio_usb_set_cpu_runstate(udev, false);
    if (rc == 0) {
        // validate fw image and send to device
        rc = accessio_parse_and_send_ihex(fw->data, fw->size, udev);
        if (rc != 0) {
//            printk(KERN_INFO KBUILD_MODNAME ": error loading firmware %s for device[%d] 0x%04X, error %d\n", fwname, udev->device_index, udev->product_id, rc);
        } else {
            printk(KERN_INFO KBUILD_MODNAME ": firmware uploaded, reseting device state.\n");
        }
        // reset the CPU so it runs what we just downloaded
        rc = accesio_usb_set_cpu_runstate(udev, true);
    }
    release_firmware(fw);
    return rc;
}

static void accesio_usb_free_endpoint_info(struct accesio_usb_endpoint* ep)
{
    aio_driver_dev_print ("Stub called");
}

// static bool accesio_usb_match_endpoint(struct usb_endpoint_descriptor *epd,
//                                        struct usb_endpoint_descriptor **bulk_in,
//                                        struct usb_endpoint_descriptor **bulk_out,
//                                        struct usb_endpoint_descriptor **ctrl_in,
//                                        struct usb_endpoint_descriptor **ctrl_out,
//                                        struct usb_endpoint_descriptor **iso_in,
//                                        struct usb_endpoint_descriptor **iso_out,
//                                        struct usb_endpoint_descriptor **int_in,
//                                        struct usb_endpoint_descriptor **int_out)
// {
//     switch (usb_endpoint_type(epd)) {
//         case USB_ENDPOINT_XFER_BULK:
//             if (usb_endpoint_dir_in(epd)) {
//                 if (bulk_in && !*bulk_in) {
//                     *bulk_in = epd;
//                     break;
//                 }
//             } else {
//                 if (bulk_out && !*bulk_out) {
//                     *bulk_out = epd;
//                     break;
//                 }
//             }
//             return false;

//         case USB_ENDPOINT_XFER_CONTROL:
//             if (usb_endpoint_dir_in(epd)) {
//                 if (ctrl_in && !*ctrl_in) {
//                     *ctrl_in = epd;
//                     break;
//                 }
//             } else {
//                 if (ctrl_out && !*ctrl_out) {
//                     *ctrl_out = epd;
//                     break;
//                 }
//             }
//             return false;
//         case USB_ENDPOINT_XFER_ISOC:
//             if (usb_endpoint_dir_in(epd)) {
//                 if (iso_in && !*iso_in) {
//                     *iso_in = epd;
//                     break;
//                 }
//             } else {
//                 if (iso_out && !*iso_out) {
//                     *iso_out = epd;
//                     break;
//                 }
//             }
//             return false;
        
//         case USB_ENDPOINT_XFER_INT:
//             if (usb_endpoint_dir_in(epd)) {
//                 if (int_in && !*int_in) {
//                     *int_in = epd;
//                     break;
//                 }
//             } else {
//                 if (int_out && !*int_out) {
//                     *int_out = epd;
//                     break;
//                 }
//             }
//             return false;
//         default:
//             return false;
//     }

//     return (!bulk_in || *bulk_in) && (!bulk_out || *bulk_out) &&
//            (!ctrl_in || *ctrl_in) && (!ctrl_out || *ctrl_out) &&
//            (!iso_in || *iso_in) && (!iso_out || *iso_out) &&
//            (!int_in || *int_in) && (!int_out || *int_out);
// }

// static int accesio_usb_find_endpoints(struct usb_interface* interface,
//                                       struct usb_endpoint_descriptor **bulk_in,
//                                       struct usb_endpoint_descriptor **bulk_out,
//                                       struct usb_endpoint_descriptor **ctrl_in,
//                                       struct usb_endpoint_descriptor **ctrl_out,
//                                       struct usb_endpoint_descriptor **iso_in,
//                                       struct usb_endpoint_descriptor **iso_out,
//                                       struct usb_endpoint_descriptor **int_in,
//                                       struct usb_endpoint_descriptor **int_out)
// {
//     int i = 0;
//     int x = 0;
//     unsigned int found = 0;
//     *bulk_in = NULL; *bulk_out = NULL;
//     *ctrl_in = NULL; *ctrl_out = NULL;
//     *iso_in = NULL; *iso_out = NULL;
//     *int_in = NULL; *int_out = NULL;
//     for (i = 0; i < interface->num_altsetting; ++i) {
//         for (x = 0; x < interface->altsetting[i].desc.bNumEndpoints; ++x) {
//             accesio_usb_match_endpoint(&interface->altsetting[i].endpoint[x].desc,
//                     bulk_in, bulk_out, ctrl_in, ctrl_out, iso_in, iso_out, int_in, int_out);
//             if (bulk_in != NULL) { found |= 1; }
//             if (bulk_out != NULL) { found |= 2; }
//             if (ctrl_in != NULL) { found |= 4; }
//             if (ctrl_out != NULL) { found |= 8; }
//             if (iso_in != NULL) { found |= 16; }
//             if (iso_out != NULL) { found |= 32; }
//             if (int_in != NULL) { found |= 64; }
//             if (int_out != NULL) { found |= 128; }
//         }
//     }
//     return found;
// }


static int accesio_usb_find_endpoints(struct accesio_usb_device_info* dev)
{
    int i, j;
    struct usb_endpoint_descriptor *epd;

    for (i = 0; i < dev->interface->num_altsetting; i++) 
    {
        for (j = 0; j < dev->interface->altsetting[i].desc.bNumEndpoints; j++) 
        {
            epd = &dev->interface->altsetting[i].endpoint[j].desc;

            if (usb_endpoint_type(epd) == USB_ENDPOINT_XFER_BULK)
            {
                aio_driver_dev_print("endpoint number is %d", usb_endpoint_num(epd));
                if ((usb_endpoint_dir_in(epd)) && dev->bulk_in_count < ACCES_MAX_BULK_ENDPOINTS)
                {
                    dev->bulk_in[dev->bulk_in_count].epd = epd;
                    dev->bulk_in_count++;
                    aio_driver_debug_print("bulk in endpoint found");
                }
                else if (dev->bulk_out_count < ACCES_MAX_BULK_ENDPOINTS)
                {
                    dev->bulk_out[dev->bulk_out_count].epd = epd;
                    dev->bulk_out_count++;
                    aio_driver_debug_print ("bulk out endpoint found");
                }
            }
        }
    }
    return 0;            
}

static char* accesio_usb_get_devnode(struct device* dev, umode_t* mode)
{
    return kasprintf(GFP_KERNEL,
                     DRIVER_CLASS_NAME"/%s_%u",
                     dev_name(dev), MINOR(dev->devt));
}

static void accesio_usb_delete(struct kref* kobj)
{
    struct accesio_usb_device_info* dev = container_of(kobj, struct accesio_usb_device_info, kref);
//    accesio_usb_print_dev(dev, "unregistering device ", false);
    if (dev->interface != NULL) {
        usb_set_intfdata(dev->interface, NULL);
        // give back our minor
        if (dev->acces_usb_device_descriptor->pid_loaded == dev->product_id)
        {
            usb_deregister_dev(dev->interface, &accesio_usb_class);
        }
        // prevent more I/O from starting
//        mutex_lock(&dev->io_mutex);
        dev->interface = NULL;
//        mutex_unlock(&dev->io_mutex);
        usb_kill_anchored_urbs(&dev->submitted);
    }
//    mutex_destroy(&dev->io_mutex);
    accesio_usb_free_endpoint_info(dev->bulk_in);
    accesio_usb_free_endpoint_info(dev->bulk_out);
    usb_put_dev(dev->udev);
    kfree(dev);
}

static int accesio_usb_open(struct inode* inode, struct file* filp)
{
    struct usb_interface *interface;
    struct accesio_usb_device_info* dev;
    int minor;
    int retval = 0;

    minor = iminor(inode);
    interface = usb_find_interface(&accesio_usb_driver, minor);

    if (!interface) 
    {
        aio_driver_err_print("could not find interface for minor %d", minor);
        retval = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(interface);

    aio_driver_dev_print("dev = %p", dev);

    filp->private_data = dev;

exit:
    return retval;


    
    // int tmp = iminor(inode);
    // struct usb_interface* interface = usb_find_interface(&accesio_usb_driver, tmp);
    // if (!interface) {
    //     printk(KERN_INFO KBUILD_MODNAME ": error finding device for minor %d.\n", tmp);
    //     return -ENODEV;
    // }
    // dev = usb_get_intfdata(interface);
    // if (!dev) {
    //     printk(KERN_INFO KBUILD_MODNAME ": error retrieving interface data.\n");
    //     return -ENODEV;
    // }
    // #if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)
    //     tmp = usb_autopm_get_interface(interface);
    //     if (tmp != 0) {
    //         printk(KERN_INFO KBUILD_MODNAME ": error retrieving PM interface data %d.\n", tmp);
    //         return tmp;
    //     }
    // #endif
    // // increment our usage count for the device
    // kref_get(&dev->kref);
    // // save our object in the file's private structure
    // filp->private_data = dev;

}

static int accesio_usb_release(struct inode* inode, struct file* filp)
{
    // accesio_usb_device_info* dev = filp->private_data;
    // if (dev == NULL) { return -ENODEV; }
    // #if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)
    //     // allow the device to be autosuspended
    //     mutex_lock(&dev->io_mutex);
    //     if (dev->interface) {
    //         usb_autopm_put_interface(dev->interface);
    //     }
    //     mutex_unlock(&dev->io_mutex);
    // #endif
    // // decrement the count on our device
    // kref_put(&dev->kref, accesio_usb_delete);
    printk("%s:%dSTUB CALLED %s", __FILE__, __LINE__, __FUNCTION__);
    return 0;
}

static int accesio_usb_flush(struct file* filp, fl_owner_t id)
{
    int res = 0;
    // accesio_usb_device_info* dev = filp->private_data;
    // if (dev == NULL) { return -ENODEV; }
    // // wait for io to stop
    // mutex_lock(&dev->io_mutex);
    // accesio_usb_draw_down(dev);
    // // read out errors, leave subsequent opens in a clean slate
    // spin_lock_irq(&dev->err_lock);
    // if (dev->errors) {
    //     res = (dev->errors == -EPIPE ? -EPIPE : -EIO);
    // }
    // dev->errors = 0;
    // spin_unlock_irq(&dev->err_lock);
    // mutex_unlock(&dev->io_mutex);
    aio_driver_dev_print("%s:%dSTUB CALLED %s", __FILE__, __LINE__, __FUNCTION__);
    return res;
}

static ssize_t accesio_usb_read(struct file* filp, char* buffer, size_t count, loff_t* ppos)
{
    // int rv = 0;
    // bool ongoing_io = false;
    // accesio_usb_device_info* dev = filp->private_data;
    // // if we cannot read at all, return EOF
    // if (!dev->endpoints.bulk.in.urb || !count) { return 0; }
    // // no concurrent readers
    // rv = mutex_lock_interruptible(&dev->io_mutex);
    // if (rv < 0) { return rv; }
    // // disconnect() was called
    // if (!dev->interface) {
    //     mutex_unlock(&dev->io_mutex);
    //     return -ENODEV;
    // }

    // // if IO is under way, we must not touch things
    // for (;;) { //retry:
    //     spin_lock_irq(&dev->err_lock);
    //     ongoing_io = dev->ongoing_read;
    //     spin_unlock_irq(&dev->err_lock);
    //     if (ongoing_io) {
    //         // nonblocking IO shall not wait
    //         if (filp->f_flags & O_NONBLOCK) {
    //             rv = -EAGAIN;
    //             break; // goto exit;
    //         }
    //         // IO may take forever hence wait in an interruptible state
    //         rv = wait_event_interruptible(dev->io_wait, (!dev->ongoing_read));
    //         if (rv < 0) {
    //             break; // goto exit;
    //         }
    //     }
    //     // errors must be reported
    //     rv = dev->errors;
    //     if (rv < 0) {
    //         // any error is reported once
    //         dev->errors = 0;
    //         // to preserve notifications about reset
    //         if (rv != -EPIPE) { rv = -EIO; }
    //         break; // goto exit;
    //     }
    //     // if the buffer is filled we may satisfy the read else we need to start IO
    //     if (dev->endpoints.bulk.in.filled) {
    //         // we had read data
    //         size_t available = (dev->endpoints.bulk.in.filled - dev->endpoints.bulk.in.copied);
    //         size_t chunk = min(available, count);
    //         if (!available) {
    //             // all data has been used actual IO needs to be done
    //             rv = accesio_usb_do_read_io(dev, count);
    //             if (rv < 0) {
    //                 break;    // goto exit;
    //             } else {
    //                 continue; // goto retry;
    //             }
    //         }
    //         // data is available chunk tells us how much shall be copied
    //         if (copy_to_user(buffer, (dev->endpoints.bulk.in.buffer + dev->endpoints.bulk.in.copied), chunk)) {
    //             rv = -EFAULT;
    //         } else {
    //             rv = chunk;
    //         }
    //         dev->endpoints.bulk.in.copied += chunk;
    //         // if we are asked for more than we have, we start IO but don't wait
    //         if (available < count) {
    //             accesio_usb_do_read_io(dev, (count - chunk));
    //         }
    //     } else {
    //         // no data in the buffer
    //         rv = accesio_usb_do_read_io(dev, count);
    //         if (rv < 0) {
    //             break;    // goto exit;
    //         } else {
    //             continue; // goto retry;
    //         }
    //     }
    // } // exit:
    // mutex_unlock(&dev->io_mutex);
    // return rv;
    printk("%s:%dSTUB CALLED %s", __FILE__, __LINE__, __FUNCTION__);
    return 0;

}

static ssize_t accesio_usb_write(struct file* filp, const char* user_buffer, size_t count, loff_t* ppos)
{
    // int retval = 0;
    // struct urb* urb = NULL;
    // char* buf = NULL;
    // size_t writesize = min(count, (size_t)ACCESIO_USB_MAX_XFR);
    // accesio_usb_device_info* dev = filp->private_data;
    // // verify that we actually have some data to write
    // if (count == 0) {
    //     goto exit;
    // }
    // // limit the number of URBs in flight to stop a user from using up all RAM
    // if (!(filp->f_flags & O_NONBLOCK)) {
    //     if (down_interruptible(&dev->limit_sem)) {
    //         retval = -ERESTARTSYS;
    //         goto exit;
    //     }
    // } else {
    //     if (down_trylock(&dev->limit_sem)) {
    //         retval = -EAGAIN;
    //         goto exit;
    //     }
    // }
    // spin_lock_irq(&dev->err_lock);
    // retval = dev->errors;
    // if (retval < 0) {
    //     // any error is reported once
    //     dev->errors = 0;
    //     // to preserve notifications about reset
    //     if (retval != -EPIPE) { retval = -EIO; }
    // }
    // spin_unlock_irq(&dev->err_lock);
    // if (retval < 0) {
    //     goto error;
    // }
    // // create a urb, and a buffer for it, and copy the data to the urb
    // urb = usb_alloc_urb(0, GFP_KERNEL);
    // if (!urb) {
    //     retval = -ENOMEM;
    //     goto error;
    // }
    // buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
    // if (!buf) {
    //     retval = -ENOMEM;
    //     goto error;
    // }
    // if (copy_from_user(buf, user_buffer, writesize)) {
    //     retval = -EFAULT;
    //     goto error;
    // }

    // // this lock makes sure we don't submit URBs to gone devices
    // mutex_lock(&dev->io_mutex);
    // if (!dev->interface) {
    //     // disconnect() was called
    //     mutex_unlock(&dev->io_mutex);
    //     retval = -ENODEV;
    //     goto error;
    // }
    // // initialize the urb properly
    // usb_fill_bulk_urb(urb,
    //                   dev->udev,
    //                   usb_sndbulkpipe(dev->udev, dev->endpoints.bulk.out.address),
    //                   buf,
    //                   writesize,
    //                   accesio_usb_write_bulk_callback,
    //                   dev);
    // urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    // usb_anchor_urb(urb, &dev->submitted);
    // // send the data out the bulk port
    // retval = usb_submit_urb(urb, GFP_KERNEL);
    // mutex_unlock(&dev->io_mutex);

    // if (retval) {
    //     printk(KERN_INFO KBUILD_MODNAME ": error submitting write urb %d.\n", retval);
    //     usb_unanchor_urb(urb);
    //     goto error;
    // }
    // // release our reference to this urb, the USB core will eventually free it entirely
    // usb_free_urb(urb);
    // return writesize;
    // error:
    //     if (urb) {
    //         usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
    //         usb_free_urb(urb);
    //     }
    //     up(&dev->limit_sem);
    // exit:
    //     return retval;
    return 0;
}

// static void accesio_usb_ioctl_set_endpoint_info(accesio_usb_ep* ep, accesio_usb_endpoint* devep)
// {
//     if (ep != NULL && devep != NULL) {
//         ep->in_address = devep->in.address;
//         ep->in_buffer_size = devep->in.buffer_size;
//         ep->in_buffer_filled = devep->in.filled;
//         ep->in_buffer_copied = devep->in.copied;
//         ep->out_address = devep->out.address;
//         ep->out_buffer_size = devep->out.buffer_size;
//         ep->out_buffer_filled = devep->out.filled;
//         ep->out_buffer_copied = devep->out.copied;
//     }
// }

// static inline int accesio_usb_ioctl_internal_get_device_info(accesio_usb_device_info* ddata, unsigned long arg)
// {
//     accesio_usb_info info;
//     if (ACCES_AOK(VERIFY_WRITE, arg, sizeof(accesio_usb_info)) == 0) { return -EACCES; }
//     // set endpoint information
//     info.endpoints[ACCESIO_USB_BULK_EP].endpoint_type = ACCESIO_USB_BULK_EP;
//     accesio_usb_ioctl_set_endpoint_info(&info.endpoints[ACCESIO_USB_BULK_EP], &ddata->endpoints.bulk);
//     info.endpoints[ACCESIO_USB_CTRL_EP].endpoint_type = ACCESIO_USB_CTRL_EP;
//     accesio_usb_ioctl_set_endpoint_info(&info.endpoints[ACCESIO_USB_CTRL_EP], &ddata->endpoints.control);
//     info.endpoints[ACCESIO_USB_INT_EP].endpoint_type = ACCESIO_USB_INT_EP;
//     accesio_usb_ioctl_set_endpoint_info(&info.endpoints[ACCESIO_USB_INT_EP], &ddata->endpoints.interrupt);
//     info.endpoints[ACCESIO_USB_ISO_EP].endpoint_type = ACCESIO_USB_ISO_EP;
//     accesio_usb_ioctl_set_endpoint_info(&info.endpoints[ACCESIO_USB_ISO_EP], &ddata->endpoints.isochronous);
//     // usb_device_info data
//     info.errors = ddata->errors;
//     info.ongoing_read = ddata->ongoing_read;
//     info.ctrl_msg = ddata->ctrl_msg;
//     info.device_index = ddata->device_index;
//     info.product_id = ddata->product_id;
//     // kernel device information
//     info.kernel_devnum = ddata->udev->devnum;
//     memcpy(ddata->udev->devpath, info.kernel_devpath, sizeof(info.kernel_devpath));
//     info.kernel_route = ddata->udev->route;
//     info.kernel_usb_state = ddata->udev->state;
//     info.kernel_usb_speed = ddata->udev->speed;
//     info.kernel_usb_bus_mA = ddata->udev->bus_mA;
//     info.kernel_usb_portnum = ddata->udev->portnum;
//     info.kernel_usb_level = ddata->udev->level;
//     info.kernel_usb_can_submit = ddata->udev->can_submit;
//     info.kernel_usb_persist_enabled = ddata->udev->persist_enabled;
//     info.kernel_usb_have_langid = ddata->udev->have_langid;
//     info.kernel_usb_authorized = ddata->udev->authorized;
//     info.kernel_usb_authenticated = ddata->udev->authenticated;
//     info.kernel_usb_wusb = ddata->udev->wusb;
//     info.kernel_usb_lpm_capable = ddata->udev->lpm_capable;
//     info.kernel_usb2_hw_lpm_capable = ddata->udev->usb2_hw_lpm_capable;
//     info.kernel_usb2_hw_lpm_besl_capable = ddata->udev->usb2_hw_lpm_besl_capable;
//     info.kernel_usb2_hw_lpm_enabled = ddata->udev->usb2_hw_lpm_enabled;
//     info.kernel_usb2_hw_lpm_allowed = ddata->udev->usb2_hw_lpm_allowed;
//     info.kernel_usb_string_langid = ddata->udev->string_langid;
//     info.kernel_usb_quirks = ddata->udev->quirks;
//     info.kernel_usb_urbnum = ddata->udev->urbnum.counter;
//     info.kernel_usb_active_duration = ddata->udev->active_duration;
//     #if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)
//         info.kernel_usb_connect_time = ddata->udev->connect_time;
//         info.kernel_usb_do_remote_wakeup = ddata->udev->do_remote_wakeup;
//         info.kernel_usb_reset_resume = ddata->udev->reset_resume;
//         info.kernel_usb_port_is_suspended = ddata->udev->port_is_suspended;
//     #endif
//     info.kernel_usb_slot_id = ddata->udev->slot_id;
//     info.kernel_usb_lpm_disable_count = ddata->udev->lpm_disable_count;
    
//     if (copy_to_user((accesio_usb_info*)arg, &info, sizeof(accesio_usb_info)) != 0) { return -EIO; };
//     return 0;
// }

// static inline int accesio_usb_ioctl_internal_write(accesio_usb_device_info* ddata, unsigned long arg)
// {
//     accesio_usb_ioctl_packet iodata;
//     if (ACCES_AOK(VERIFY_READ, arg, sizeof(accesio_usb_ioctl_packet)) == 0) { return -EACCES; }
//     if (copy_from_user(&iodata, (accesio_usb_ioctl_packet*)arg, sizeof(accesio_usb_ioctl_packet)) != 0) { return -EIO; }
//     switch (iodata.msg_type) {
//         case ACCESIO_USB_IOCTL_BULK_MSG:
//             return accesio_usb_bulk_msg(ddata, iodata.data, iodata.data_len);
//         case ACCESIO_USB_IOCTL_CTRL_MSG:
//             return accesio_usb_ctrl_msg(ddata, iodata.request, iodata.value, iodata.index, iodata.data, iodata.data_len);
//     }
//     return -EINVAL;
// }

// static inline int accesio_usb_ioctl_internal_read(accesio_usb_device_info* ddata, unsigned long arg)
// {
//     int rc = -EIO;
//     accesio_usb_ioctl_packet iodata;
//     if (ACCES_AOK(VERIFY_READ, arg, sizeof(accesio_usb_ioctl_packet)) == 0) { return -EACCES; }
//     if (copy_from_user(&iodata, (accesio_usb_ioctl_packet*)arg, sizeof(accesio_usb_ioctl_packet)) != 0) { return -EIO; }
//     switch (iodata.msg_type) {
//         case ACCESIO_USB_IOCTL_BULK_MSG:
//             rc = accesio_usb_bulk_read(ddata, iodata.data, iodata.data_len);
//         case ACCESIO_USB_IOCTL_CTRL_MSG:
//             rc = accesio_usb_ctrl_read(ddata, iodata.request, iodata.value, iodata.index, iodata.data, iodata.data_len);
//     }
//     if (rc < 0) { return rc; }
//     if (copy_to_user((accesio_usb_ioctl_packet*)arg, &iodata, sizeof(accesio_usb_ioctl_packet)) != 0) { return -EIO; }
//     return 0;
// }

static int ioctl_ACCESIO_USB_CONTROL_XFER (struct accesio_usb_device_info *dev, unsigned long arg)
{
    struct accesio_usb_control_transfer *context = (struct accesio_usb_control_transfer *) arg;
    int status = 0;
    void *dma_capable_buffer = NULL; //TODO: Maybe put in device structure to avoid allocation every transfer


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))
    if (!access_ok(context->data, context->size))
#else
    if (!access_ok(context->read ? VERIFY_READ : VERIFY_WRITE, context->data, context->size))
#endif
    {
        aio_driver_err_print("access_ok returned err");
        return -EPERM;
    }

    aio_driver_dev_print("passed access_ok");

    dma_capable_buffer = kmalloc(context->size, GFP_KERNEL);
    copy_from_user(dma_capable_buffer, context->data, context->size);

    // {
    //     uint8_t *temp = (char *) dma_capable_buffer;
    //     aio_driver_dev_print("%x %x %x %x %x", temp[0], temp[1], temp[2], temp[3], temp[4]);
    // }

    aio_driver_dev_print("Calling contrl_msg");


    status = usb_control_msg(dev->udev,
                              usb_sndctrlpipe(dev->udev, 0),
                              context->request,
                              (context->read ? USB_DIR_IN : USB_DIR_OUT) | USB_TYPE_VENDOR,
                              context->value,
                              context->index,
                              dma_capable_buffer,
                              context->size,
                              1000);

    if (context->read)
    {
        copy_to_user(context->data, dma_capable_buffer, context->size);
    }

    if (status < 0)
    {
        aio_driver_err_print("usb_ctrol_msg returned %d", status);
    }

    return status;
}

void accesio_urb_complete(struct urb *urb)
{
    struct completion *completion = (struct completion *)urb->context;
    aio_driver_dev_print("Reached competion callback");
    complete(completion);
}

static int ioctl_ACCESIO_USB_BULK_XFER (struct accesio_usb_device_info *dev, unsigned long arg)
{
    struct accesio_usb_bulk_transfer *context = (struct accesio_usb_bulk_transfer *)arg;
    int status = 0;
    void *dma_capable_buffer = NULL; //TODO: Maybe put in device structure to avoid allocation every transfer
    unsigned int endpoint;
    unsigned int pipe;


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 0, 0))
    if (!access_ok(context->data, context->size))
#else
    if (!access_ok(context->read ? VERIFY_READ : VERIFY_WRITE, context->data, context->size))
#endif
    {
        aio_driver_err_print("access_ok returned err");
        return -EPERM;
    }

    aio_driver_dev_print("passed access_ok");

    dma_capable_buffer = usb_alloc_coherent (dev->udev, context->size, GFP_KERNEL, &dev->urb->transfer_dma);
    copy_from_user(dma_capable_buffer, context->data, context->size);

    if (context->read)
    {
        endpoint = usb_endpoint_num(dev->bulk_in[context->pipe_index].epd);
        pipe = usb_rcvbulkpipe(dev->udev, endpoint);
    }
    else
    {
        endpoint = usb_endpoint_num(dev->bulk_out[context->pipe_index].epd);
        pipe = usb_sndbulkpipe(dev->udev, endpoint);
    }

    aio_driver_dev_print("filling urb");

    usb_fill_bulk_urb(dev->urb, 
                    dev->udev,
                    pipe,
                    dma_capable_buffer,
                    context->size,
                    accesio_urb_complete,
                    &dev->urb_completion);


    status = usb_submit_urb(dev->urb, GFP_KERNEL);

    if (status)
    {
        aio_driver_err_print("Unable to submit urb status = %d", status);
        goto ERR_OUT;
    }


        wait_for_completion(&dev->urb_completion);


        status = dev->urb->status;
        aio_driver_dev_print("urb status = %d", status);

    if (context->read && !status )
    {
        aio_driver_dev_print("Copying data to user. transferred = %d", dev->urb->actual_length);
        copy_to_user(context->data, dma_capable_buffer, dev->urb->actual_length);
        put_user(dev->urb->actual_length, context->transferred);
    }

ERR_OUT:

     usb_free_coherent(dev->udev,
                    dev->urb->transfer_buffer_length,
                    dev->urb->transfer_buffer,
                    dev->urb->transfer_dma);

    return status;

}



static int accesio_usb_ioctl_internal(struct file* filp, unsigned int cmd, unsigned long arg)
{
    struct accesio_usb_device_info* dev = filp->private_data;
    int status;
    aio_driver_debug_print("enter");

    if (!dev) 
    { 
        aio_driver_err_print("dev is NULL");
        status = -ENOSYS;
        goto err_out;
    }
    switch (cmd) {
        case ACCESIO_USB_CONTROL_XFER:
            aio_driver_debug_print("ACCES_USB_CONTROL_XFER");
            status = ioctl_ACCESIO_USB_CONTROL_XFER(dev, arg);
            break;
        case ACCESIO_USB_BULK_XFER:
            aio_driver_debug_print("ACCESIO_USB_BULK_XFER");
            status = ioctl_ACCESIO_USB_BULK_XFER(dev, arg);
            break;
        case ACCESIO_USB_AIOUSB_INFO:
            aio_driver_debug_print("ACCESIO_USB_AIOUSB_INFO");
            {
                struct accesio_usb_aiousb_info *info = (struct accesio_usb_aiousb_info *)arg;
                info->pid = dev->product_id;
                status = 0;
            }
            break;
        case ACCESIO_USB_ABORT_PIPE:
            aio_driver_debug_print("ACCESIO_USB_ABORT_PIPE");
            {
                //TODO: Should we be tracking whether or not there's a transfer in 
                // flight? Do we care if the USB core doesn't care?
                status = usb_unlink_urb(dev->urb);
            }
            break;
        default:
            aio_driver_err_print("invalid ioctl cmd");
            status = -EINVAL;
            break;
    
    };
err_out:
    return status;
}

static loff_t accesio_usb_seek(struct file* filp, loff_t offset, int origin)
{
    /* NOTE: this function could be utilized to signal different modes of
    I/O for the device, but for now it is essentially a 'noop_llseek' */
    //accesio_usb_device_info* dev = (accesio_usb_device_info*)filp->private_data;
    // origin doesn't matter, since we can't go back or forward
    //if (offset < 0 || offset >= ddata->regions[bar].length) { return -ESPIPE; }
    printk("%s:%dSTUB CALLED %s", __FILE__, __LINE__, __FUNCTION__);
    filp->f_pos = offset;
    return filp->f_pos;
}

//TODO: Determine how far back we want to try to keep code compatible
//https://www.kernel.org/category/releases.html
//CentOS 7 is using 3.10 and probably will until 2024
//https://wiki.centos.org/FAQ/General
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
    int accesio_usb_ioctl(struct inode* inode, struct file* filp, unsigned int cmd, unsigned long arg)
    { (void*)inode; return accesio_pci_ioctl_internal(filp, cmd, arg); }
#else 
    long accesio_usb_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
    { return accesio_usb_ioctl_internal(filp, cmd, arg); }
#endif

// kernel driver functions (defined in driver.h)

static int accesio_usb_probe(struct usb_interface* interface, const struct usb_device_id* id)
{
    int retval;
    int i;
    struct accesio_usb_device_info* dev = kzalloc(sizeof(accesio_usb_device_info), GFP_KERNEL);
    aio_driver_debug_print("Enter probe");
    if (!dev) { return -ENOMEM; }
    dev->product_id = id->idProduct;
    kref_init(&dev->kref);
//    sema_init(&dev->limit_sem, ACCESIO_USB_WIF);
//    mutex_init(&dev->io_mutex);
//    spin_lock_init(&dev->err_lock);
    init_usb_anchor(&dev->submitted);
    dev->udev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;
    // set up the endpoint information
//    retval = accesio_usb_find_endpoints(dev);
    // if (retval) {
    //     printk(KERN_INFO KBUILD_MODNAME ": error setting up I/O endpoints, %d.\n", retval);
    //     goto error;
    // }
    // save our data pointer in this interface device

    for ( i = 0 ; i < NUM_ACCES_USB_DEVICES ; i++)
    {
        if ((acces_usb_device_table[i].pid_unloaded == dev->product_id) || 
            (acces_usb_device_table[i].pid_loaded == dev->product_id))
        {
            dev->acces_usb_device_descriptor = &acces_usb_device_table[i];
            break;
        }
    }



    usb_set_intfdata(interface, dev);
    // we can register the device now, as it is ready
    if (dev->acces_usb_device_descriptor->pid_unloaded == dev->product_id) 
    {
        retval = accesio_usb_load_fw(dev);
        if (retval != 0) {
            goto error;
        }
    }
    else 
    {
        accesio_usb_class.name = dev->acces_usb_device_descriptor->name;
        retval = usb_register_dev(interface, &accesio_usb_class);
        if (retval) {
            // something prevented us from registering this driver
            printk(KERN_INFO KBUILD_MODNAME ": register device failed getting minor, error %d.\n", retval);
            goto error;
        }
        accesio_usb_find_endpoints(dev);
        dev->urb = usb_alloc_urb(0, GFP_KERNEL);
        init_completion(&dev->urb_completion);

        aio_driver_dev_print("dev = %p", dev);

    }
//    accesio_usb_print_dev(dev, "registered device ", true);
    
    return 0;

    error:
        // this frees allocated memory
        kref_put(&dev->kref, accesio_usb_delete);
        return retval;
}

static void accesio_usb_disconnect(struct usb_interface* interface)
{
    struct accesio_usb_device_info* dev = usb_get_intfdata(interface);
    // decrement our usage count
    kref_put(&dev->kref, accesio_usb_delete);
}

static void accesio_usb_draw_down(struct accesio_usb_device_info* dev)
{
    // int time = usb_wait_anchor_empty_timeout(&dev->submitted, ACCESIO_USB_ANCHOR_TIMEOUT);
    // if (!time) { usb_kill_anchored_urbs(&dev->submitted); }
    // usb_kill_urb(dev->endpoints.bulk.in.urb);
    aio_driver_dev_print("Stub called");
}

#if defined(CONFIG_PM) || defined(ACCESIO_USB_AUTOSUSPEND)

static int accesio_usb_suspend(struct usb_interface* intf, pm_message_t message)
{
    struct accesio_usb_device_info* dev = usb_get_intfdata(intf);
    if (!dev) { return 0; }
    accesio_usb_draw_down(dev);
    return 0;
}

static int accesio_usb_resume(struct usb_interface* intf)
{
    return 0;
}

#endif

static int accesio_usb_pre_reset(struct usb_interface* intf)
{
    struct accesio_usb_device_info* dev = usb_get_intfdata(intf);
//    mutex_lock(&dev->io_mutex);
    accesio_usb_draw_down(dev);
    return 0;
}

static int accesio_usb_post_reset(struct usb_interface* intf)
{
    struct accesio_usb_device_info* dev = usb_get_intfdata(intf);
    /* we are sure no URBs are active - no locking needed */
    dev->errors = -EPIPE;
//    mutex_unlock(&dev->io_mutex);
    return 0;
}

static int __init accesio_usb_init(void)
{
    struct usb_device_id *id_table = NULL;
    int i = 0;
    int ret;

    id_table = kmalloc(sizeof(struct usb_device_id) * NUM_ACCES_USB_DEVICES * 2, GFP_KERNEL);

    for (i=0; i < NUM_ACCES_USB_DEVICES ; i++)
    {
        id_table[i*2].match_flags = USB_DEVICE_ID_MATCH_DEVICE;
        id_table[i*2].idVendor = ACCESIO_USB_VID;
        id_table[i*2].idProduct = acces_usb_device_table[i].pid_unloaded;
        
        id_table[i*2+1].match_flags = USB_DEVICE_ID_MATCH_DEVICE;
        id_table[i*2+1].idVendor = ACCESIO_USB_VID;
        id_table[i*2+1].idProduct = acces_usb_device_table[i].pid_loaded;
    }

    accesio_usb_driver.id_table = id_table;

    ret = usb_register(&accesio_usb_driver);
    // NOTE: the current Linux USB serial driver for the FX chips works
    // usb_serial_register(&option_1port_device);
    accesio_major_num = MAJOR(accesio_first_dev);
    if (ret) {
        printk(KERN_INFO KBUILD_MODNAME ": could not register driver, error %d.\n", ret);
        return ret;
    }
    printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ": " DRIVER_DESC "\n");
    return 0;
}

static void __exit accesio_usb_exit(void)
{
    accesio_major_num = 0;
    usb_deregister(&accesio_usb_driver);
    kfree(accesio_usb_driver.id_table);
    accesio_usb_driver.id_table = NULL;
    printk("ACCES USB driver unloaded\n");
}

module_init(accesio_usb_init);
module_exit(accesio_usb_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE(DRIVER_LICENSE);
