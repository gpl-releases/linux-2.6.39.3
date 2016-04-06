/*
 *  puma6_gpio_ctrl
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2012 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution
 *  in the file called LICENSE.GPL.
 *
 *  Contact Information:
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 * The file contains the main data structure and API definitions for Linux GPIO driver
 *
 */

/** \file   puma6_gpio_ctrl.c
 *  \brief  GPIO config control APIs. 
 *          
 *  \author     Intel
 *
 *  \version    0.1     Created
 */

#include <pal.h> 
#include <hw_mutex_ctrl.h> 
#include "puma6_gpio_ctrl.h"
#include "puma6_dect_page_button_ctrl.h"

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/module.h>   /* for modules */
#include <linux/interrupt.h>
#include <linux/fs.h>       /* file_operations */
#include <linux/uaccess.h>  /* copy_(to,from)_user */
#include <linux/init.h>     /* module_init, module_exit */
#include <linux/cdev.h>     /* cdev utilities */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/ioctl.h>
#include <linux/proc_fs.h>





/****************************************************************************/
/****************************************************************************/
/*                          GPIO local definitions                          */
/****************************************************************************/
/****************************************************************************/
static dev_t dect_page_button_dev_t;
struct tasklet_struct   gDectIrqTasklet;     /* Tx completion processing tasklet */
static unsigned int count = 1;
static unsigned int ref = 0;
static struct cdev *dect_page_button_cdev;
static struct class *dect_page_button_udev_class;
struct semaphore dectPageSem;

static struct proc_dir_entry *dect_page_proc_dir;

/****************************************************************************/
/****************************************************************************/
/*                          GPIO Defines                                    */
/****************************************************************************/
/****************************************************************************/
#define DEV_NAME  DECT_PAGE_BUTTON_INTERFACE_DRIVER_NAME
#define DECT_PAGE_BUTTON_OK                 (0)
#define DECT_PAGE_BUTTON_FAIL               (-1)

/* Debug */
#ifdef  GPIO_DEBUG_OUTPUT_ON
	#define GPIO_DEBUG_OUTPUT(fmt, args...) printk("Puma6 GPIO Debug %s: " fmt, __FUNCTION__ , ## args)
#else
	#define GPIO_DEBUG_OUTPUT(fmt, args...)
#endif

/* Info */
#ifdef  GPIO_INFO_OUTPUT_ON
	#define GPIO_INFO_OUTPUT(fmt, args...) printk("Puma6 GPIO Info %s: " fmt, __FUNCTION__ , ## args)
#else	
	#define GPIO_INFO_OUTPUT(fmt, args...)
#endif	

/**************************************************************************/
/*! \fn static int dect_page_button_drv_open(struct inode *inode, struct file *filp)
 **************************************************************************
 *  \brief This function is opens the gpio device.
 *  \param struct inode *inode - device node pointer
 *  \param struct file *filp - device file pointer
 *  \return int - DECT_PAGE_BUTTON_OK on correcet access type  otherwise  DECT_PAGE_BUTTON_FAIL.
 **************************************************************************/
static int dect_page_button_drv_open(struct inode *inode, struct file *filp)
{
        ++ref;
        GPIO_DEBUG_OUTPUT("%s, dect_page_button_drv_open: ref %d\n", __FUNCTION__, ref);
        return DECT_PAGE_BUTTON_OK;
}

/**************************************************************************/
/*! \fn static int dect_page_button_drv_close(struct inode *inode, struct file *filp)
 **************************************************************************
 *  \brief This function is closes the gpio device.
 *  \param struct inode *inode - device node pointer
 *  \param struct file *filp - device file pointer
 *  \return int - DECT_PAGE_BUTTON_OK on correcet access type  otherwise  DECT_PAGE_BUTTON_FAIL.
 **************************************************************************/
static int dect_page_button_drv_close(struct inode *inode, struct file *filp)
{
    --ref;
    GPIO_DEBUG_OUTPUT("%s, dect_page_button_drv_close: ref %d\n", __FUNCTION, --ref);
    return DECT_PAGE_BUTTON_OK;
}

/**************************************************************************/
/*! \fn static long page_button_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief This function is activate the the gpio device requests.
 *  \param struct file *filp - the device file pointer
 *  \param unsigned int cmd - the command to be performed
 *  \param unsigned long arg - pointer to the user request
 *  \return long - 0 on success else negative number.
 **************************************************************************/
static long page_button_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct dect_page_info  gpio_info;
    int ret = 0;

    /* Check for valid pointer to the parameter list */
    if (0 == arg)
    {
        printk(KERN_ERR "page_button arg == 0\n");
        return -EINVAL;
    }
    if (copy_from_user(&gpio_info, (void __user *)arg, sizeof(struct gpio_user_info)))
    {
        printk(KERN_ERR "page_button copy from user failed\n");
        return -EFAULT;
        /* Execute ioctl request */
    }
        
    switch (cmd)
    {
        /*---------------------------------------------------------------------------*/
    case DECT_GET_IRQ_STATUS:
            GPIO_DEBUG_OUTPUT("%s, PAL_sysGpioCtrlGetIrqStatus(gpio_pin=%d) requested.\n",__FUNCTION__,gpio_info.gpio_pin);
            if ((gpio_info.value = PAL_sysGpioCtrlGetIrqStatus(gpio_info.gpio_pin)) < 0)
            {
                ret = -ENOSYS;
            }
            else
            {
                if (copy_to_user((void __user *)arg, &gpio_info, sizeof(struct gpio_user_info)))
                {
                    ret = -EFAULT;
                }
            }

            break;
        /*---------------------------------------------------------------------------*/
    case DECT_CLEAR_IRQ_STATUS:
            GPIO_DEBUG_OUTPUT("%s, PAL_sysGpioCtrlClearIrqStatus(gpio_pin=%d) requested.\n",__FUNCTION__,gpio_info.gpio_pin);
            if ((gpio_info.value = PAL_sysGpioCtrlClearIrqStatus(gpio_info.gpio_pin)) < 0)
            {
                ret = -ENOSYS;
            }
            else
            {
                if (copy_to_user((void __user *)arg, &gpio_info, sizeof(struct gpio_user_info)))
                {
                    ret = -EFAULT;
                }
            }
 
            break;
    case DECT_GET_LINE_STATUS:
            GPIO_DEBUG_OUTPUT("%s, PAL_sysGpioInBit(gpio_pin=%d) requested.\n",__FUNCTION__,gpio_info.gpio_pin);
            if ((gpio_info.value = PAL_sysGpioInBit(gpio_info.gpio_pin)) == DECT_PAGE_BUTTON_FAIL)
            {
                ret = -ENOSYS;
            }
            else
            {
                if (copy_to_user((void __user *)arg, &gpio_info, sizeof(struct gpio_user_info)))
                {
                    ret = -EFAULT;
                }
            }
            break;
        /*---------------------------------------------------------------------------*/
    case DECT_ENABLE_IRQ:
            GPIO_DEBUG_OUTPUT("%s, enable DECT page IRQ (gpio_pin=%d) requested, using falling edge mode.\n",__FUNCTION__,gpio_info.gpio_pin);
            PAL_sysGpioCtrlEnableIrq(gpio_info.gpio_pin);
            PAL_sysGpioCtrlSetIrqConfig(gpio_info.gpio_pin, IDL_GPIO_FALLING_DOWN_EDGE);
            gpio_info.value = 0;
            ret = 0;
            if (copy_to_user((void __user *)arg, &gpio_info, sizeof(struct gpio_user_info)))
            {
                ret = -EFAULT;
            }

            break;
    default:
            printk(KERN_ERR "iosfsb no legal command given\n");
            ret = -ENOSYS;
            break;
        }
    return ret;
}

irqreturn_t dect_button_interrupt(int irq, void *dev)
{
    /* Since the INTD interrupts are level, need to disable the IRQ in order to run the tasklet */
    disable_irq_nosync(AVALANCHE_INT_83);

    GPIO_DEBUG_OUTPUT("dect_button_interrupt(), irq=%d\n", irq);
    tasklet_schedule(&gDectIrqTasklet);

    return IRQ_RETVAL(1);
}

static void do_dect_page_irq_complete(unsigned long data)
{

    /* First clear the IRQ in order not to get a false interrupt since INTD is level */
    ack_irq(AVALANCHE_INT_83);

    GPIO_DEBUG_OUTPUT("do_dect_page_irq_complete()\n");
    /* Send INTD EOI */
    // avalanche_intd_write_eoi(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM);

    /* It could be that between INTD count decrement and EOI the accumulator will issue another interrupt.
       The logic of INTD is such that level will remain active high even after EOI is set, so INTC will
       lose the interrupt after ack_irq is done (it now expects INTD polarity change).
       Therefore we must check INTD count and if it is not 0 - reschedule the tasklet */
#if 0
    for (priority = PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT - 1; priority >= 0; priority--)
    {
        if (avalanche_intd_get_interrupt_count(0, PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority)))
        {
            tasklet_schedule(&gTxCompleteTasklet);
            return;
        }
    }
#endif
    /* Now enable the IRQ */
    enable_irq(AVALANCHE_INT_83);
}

/**************************************************************************/
/*! \fn static int __init dect_page_drv_init(void)
 ***************************************************************************
 **  \brief This function is the gpio device module init function.
 **  \return long - 0 on success else negative number.
 ***************************************************************************/
int setupInterrupt() {

    tasklet_init(&gDectIrqTasklet, do_dect_page_irq_complete, 0);

    if(request_irq(AVALANCHE_INT_83, dect_button_interrupt, (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ), "dect_button", NULL))
    {
        printk(KERN_ERR "%s, unable to get IRQ #%d for DECT button task\n", __FUNCTION__, AVALANCHE_INT_83);
        return -1;
    }

    return 0;
}

/* Structure to map driver functions to kernel */
struct file_operations dect_page_button_drv_fops = {
        .owner   = THIS_MODULE,
        .unlocked_ioctl   = page_button_unlocked_ioctl,
        .open    = dect_page_button_drv_open,
        .release = dect_page_button_drv_close,
};

int getGpioLineStatus(char *buf,char **start,off_t offset,int count,int *eof,void *data ) 
{
    int gpioNum=39;
    printk("In getGpioStatus(), gpio %d is %d\n", gpioNum, PAL_sysGpioInBit(gpioNum) );

    return 0;
}

int getGpioIrqStatus(char *buf,char **start,off_t offset,int count,int *eof,void *data)
{
    int gpioNum=39;
    printk(" GPIO %d irq status: 0x%X\n", gpioNum, PAL_sysGpioCtrlGetIrqStatus(gpioNum));
    return 0;
}

int setGpioIrqEnable(char *buf,char **start,off_t offset,int count,int *eof,void *data)
{
    int gpioNum=39;
    printk(" GPIO %d irq enable: 0x%X\n", gpioNum, PAL_sysGpioCtrlEnableIrq(gpioNum));
    printk(" GPIO %d irq enable: 0x%X\n", gpioNum, PAL_sysGpioCtrlSetIrqConfig(gpioNum, IDL_GPIO_FALLING_DOWN_EDGE ));
    return 0;
}

int clearGpioIrq(char *buf,char **start,off_t offset,int count,int *eof,void *data)
{
    int gpioNum=39;
    printk(" GPIO %d irq clear: 0x%X\n", gpioNum, PAL_sysGpioCtrlClearIrqStatus(gpioNum));
    return 0;
}


/**************************************************************************/
/*! \fn static int __init dect_page_drv_init(void)
 **************************************************************************
 *  \brief This function is the gpio device module init function.
 *  \return long - 0 on success else negative number.
 **************************************************************************/
static int __init dect_page_drv_init(void)
{
    sema_init(&dectPageSem,1);

    printk("DECT drv initializing\n");
    if (setupInterrupt() != 0) {
        printk(KERN_ERR "\nDECT Page drv failed to setup DECT Page button GPIO as interrupt\n");
        return DECT_PAGE_BUTTON_FAIL;
    }

    dect_page_proc_dir = proc_mkdir(DECT_PAGE_BUTTON_INTERFACE_DRIVER_NAME, NULL);
    create_proc_read_entry("getGpioLineStatus",0,dect_page_proc_dir,getGpioLineStatus,NULL);
    create_proc_read_entry("getGpioIrqStatus",0,dect_page_proc_dir,getGpioIrqStatus,NULL);
    create_proc_read_entry("setGpioIrqEnable",0,dect_page_proc_dir,setGpioIrqEnable,NULL);
    create_proc_read_entry("clearGpioIrq",0,dect_page_proc_dir,clearGpioIrq,NULL);

    if (alloc_chrdev_region(&dect_page_button_dev_t, 0, count, DEV_NAME) < 0)
    { /*count indicates how many minors we get*/
        printk(KERN_ERR "\nDECT Page drv failed to register character device region %s\n",DEV_NAME);
        return DECT_PAGE_BUTTON_FAIL;
    }

    if (!(dect_page_button_cdev = cdev_alloc()))
    {
        printk(KERN_ERR "\nDECT Page drv failed in cdev_alloc %s\n",DEV_NAME);
        unregister_chrdev_region(dect_page_button_dev_t, count);
        return DECT_PAGE_BUTTON_FAIL;
    }
    /* Connect the file operations with the cdev */
    cdev_init(dect_page_button_cdev, &dect_page_button_drv_fops);
    /* Connect the major/minor number to the cdev  - Activates the device*/
    if (cdev_add(dect_page_button_cdev, dect_page_button_dev_t, count) < 0)
    {
        printk(KERN_ERR "\nDECT Page drv failed in add character device %s\n",DEV_NAME);
        cdev_del(dect_page_button_cdev);
        unregister_chrdev_region(dect_page_button_dev_t, count);
        return DECT_PAGE_BUTTON_FAIL;
    }
    /* connection to the udev *******************************************************************/
    /* ceates a class directory under /sys/class */
    dect_page_button_udev_class = class_create(THIS_MODULE, "page_button_class");
    /* ceates a class directory under /sys/class/DEV_NAME named DEV_NAME  */
    /* creates 3 file: dev, uevent, subsystem*/
    device_create(dect_page_button_udev_class, NULL, dect_page_button_dev_t, NULL, "%s", "page_button");

    printk(KERN_INFO "DECT Page drv succeeded in registering character device %s Major = %d, Minor = %d\n",
            DEV_NAME,MAJOR(dect_page_button_dev_t),MINOR(dect_page_button_dev_t));
    printk("DECT Page drv succeeded in registering character device %s Major = %d, Minor = %d\n",
            DEV_NAME,MAJOR(dect_page_button_dev_t),MINOR(dect_page_button_dev_t));

    return DECT_PAGE_BUTTON_OK;

}


/**************************************************************************/
/*! \fn static void __exit dect_page_drv_exit(void)
 **************************************************************************
 *  \brief This function is the gpio device module exit function.
 **************************************************************************/
static void __exit dect_page_drv_exit(void)
{
    if (dect_page_button_cdev)
        cdev_del(dect_page_button_cdev);
    unregister_chrdev_region(dect_page_button_dev_t, count);
    printk(KERN_INFO "\nDECT Page device unregistered\n");


    device_destroy(dect_page_button_udev_class, dect_page_button_dev_t);
    class_destroy(dect_page_button_udev_class);
}
/*************************************************************************************/

module_init(dect_page_drv_init);
module_exit(dect_page_drv_exit);


