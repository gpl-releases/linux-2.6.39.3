/*
 *
 * p_unit_drv.c
 * Description:
 * power control unit device driver
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
 */

/*------------------------------------------------------------------------------
 * File Name: p_unit_drv.c
 *------------------------------------------------------------------------------
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/module.h>   /* for modules */
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
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include "puma6.h"
#include "iosfsb_api.h"
#include "puma6_bootcfg_ctrl.h"
#include "p_unit_api.h"

#define DEV_NAME  "p_unit"

//#define P_UNIT_DEBUG
#ifdef  P_UNIT_DEBUG
/* note: prints function name for you */
#  define DPRINTK(fmt, args...) printk("%-40s:%5d " fmt, __FUNCTION__,__LINE__, ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

static dev_t            p_unit_dev_t;
static unsigned int     count = 1;
static struct cdev *    p_unit_cdev;
static struct class *   p_unit_udev_class;
struct semaphore        P_Unit_Sem;
static atomic_t         p_unit_enter_bbu_mode_done_flag;
static DECLARE_WAIT_QUEUE_HEAD(wq);
static int              gPunitResetEvent;
struct workqueue_struct *gPunitWorkqueue;
struct work_struct      gPunitWork;

typedef union
{
    struct
    {
        unsigned char   Stepping;
        unsigned char   NumMajor;
        unsigned char   NumMinor;
        unsigned char   NumRelease;
    }
    field;
    unsigned int reg;
}
P_Unit_Version_t;

#define ADDR_FW_VERSION         (0x00000087)
#define ADDR_DOCSIS_PM_CMD      (0x000000D8)
#define ADDR_DOCSIS_PM_DATA     (0x000000D9)
#define ADDR_PM_DOCSIS_CMD      (0x000000DA)
#define ADDR_PM_DOCSIS_DATA     (0x000000DB)

#define P_UNIT_CMD_TYPE_IPC         (0x0 << 8)
#define P_UNIT_CMD_TYPE_BBU         (0x1 << 8)
#define P_UNIT_CMD_TYPE_WATCHDOG    (0x2 << 8)
#define P_UNIT_CMD_TYPE_RESET       (0x3 << 8)

#define P_UNIT_CMD_DATA_ATTACHED    (0x1 << 14)
#define P_UNIT_CMD_DATA_EXPECTED    (0x1 << 15)

#define P_UNIT_CMD_IDLE             (0x0000)
#define P_UNIT_CMD_ACK              (0x0001)

#define P_UNIT_CMD_RESET_REQ_COLD_RESET                         (P_UNIT_CMD_TYPE_RESET | 0x00) // Initiate a cold reset
#define P_UNIT_CMD_RESET_REQ_WARM_RESET                         (P_UNIT_CMD_TYPE_RESET | 0x01) // Initiate a warm reset
#define P_UNIT_CMD_RESET_REQ_DOCSIS_RESET                       (P_UNIT_CMD_TYPE_RESET | 0x03) // Initiate a docsis only reset
#define P_UNIT_CMD_RESET_GET_LAST_RESET_CAUSE                   (P_UNIT_CMD_TYPE_RESET | 0x04 | P_UNIT_CMD_DATA_EXPECTED) // Returns the last reset cause indicated by firmware. If a hardware reset(not under firmware control) occurs the state indicated here is a "cold boot"
#define P_UNIT_CMD_RESET_CLR_LAST_RESET_CAUSE                   (P_UNIT_CMD_TYPE_RESET | 0x05)
#define P_UNIT_CMD_RESET_GET_DURATION_RESET_HELD                (P_UNIT_CMD_TYPE_RESET | 0x06 | P_UNIT_CMD_DATA_EXPECTED) // Returns the amount of time the reset button was held, if the last reset cause was a reset button. Unit is in milliseconds (valid range: 0-60,000 milliseconds)
#define P_UNIT_CMD_RESET_SET_WARM_RESET_ON_BUTTON               (P_UNIT_CMD_TYPE_RESET | 0x08) // When set, on a reset button press the firmware will start the warm reset sequence (warm reset by default)
#define P_UNIT_CMD_RESET_SET_COLD_RESET_ON_BUTTON               (P_UNIT_CMD_TYPE_RESET | 0x09) // When set, on a reset button press the firmware will start the cold reset sequence (warm reset by default)
#define P_UNIT_CMD_RESET_EN_DOCSIS_RESET_INDICATION             (P_UNIT_CMD_TYPE_RESET | 0x0E | P_UNIT_CMD_DATA_ATTACHED) // When enabled, if a reset request is sent to Punit firmware, an IPC will be sent to DOCSIS to inform it a reset will occur. Data Sent: 0 = Disable (default), 1 = Enable
#define P_UNIT_CMD_RESET_ALLOW_DOCSIS_INDICATION_IF_SELF_RESET  (P_UNIT_CMD_TYPE_RESET | 0x0F | P_UNIT_CMD_DATA_ATTACHED) // When enabled, if a reset request is sent to Punit firmware from DOCSIS and if enabled an IPC will be sent to DOCSIS to inform it a reset will occur. Suggested to use this mode only in a debug scenario and not in production code. Data Sent: 0 = Disable (default), 1 = Enable
#define P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION                (P_UNIT_CMD_TYPE_RESET | 0x12 | P_UNIT_CMD_DATA_ATTACHED) // If a reset request is sent to firmware and if enabled this IPC will be sent to DOCSIS to inform it a reset will occur.
#define P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION_ACK            (P_UNIT_CMD_TYPE_RESET | 0x13) // After receiving the IPC of a reset request, the DOCSIS should respond with this ACK IPC when it gives the firmware the OK to continue with the reset.
#define P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION_ACK_TIMEOUT    (P_UNIT_CMD_TYPE_RESET | 0x15 | P_UNIT_CMD_DATA_ATTACHED) // Set the timeout time in milliseconds for the DOCSIS to ack, if this time expires the reset sequence will proceed without an ack. Unit is in milliseconds (valid range: 0-60,000 milliseconds) (default 2,000  milliseconds)

#define P_UNIT_CMD_WATCHDOG_DO_COLD_RESET   (P_UNIT_CMD_TYPE_WATCHDOG | 0x8)
#define P_UNIT_CMD_WATCHDOG_DO_WARM_RESET   (P_UNIT_CMD_TYPE_WATCHDOG | 0x9)
#define P_UNIT_CMD_WATCHDOG_DO_CPU_RESET    (P_UNIT_CMD_TYPE_WATCHDOG | 0xA)
#define P_UNIT_CMD_WATCHDOG_DO_NOTHING      (P_UNIT_CMD_TYPE_WATCHDOG | 0xB)

#define P_UNIT_CMD_BBU_EXIT_BBU_MODE        (P_UNIT_CMD_TYPE_BBU | 0x2)
#define P_UNIT_CMD_BBU_ENTER_BBU_MODE_DONE  (P_UNIT_CMD_TYPE_BBU | 0x1)

#define ATOM_REBOOT_WAIT_TOTAL_TIME_MSEC    5000
#define ATOM_REBOOT_POLL_INTERVAL_MSEC      100
#define ATOM_REBOOT_POLL_NUM_INTERVALS      ((ATOM_REBOOT_WAIT_TOTAL_TIME_MSEC) / (ATOM_REBOOT_POLL_INTERVAL_MSEC))

/* ******************************************************************** */
/*                                                                      */
/*                                                                      */
/*           P-UNIT Low Level Building Blocks ....                      */
/*                                                                      */
/*                                                                      */
/* ******************************************************************** */

/* ******************************************************************** */
static int p_unit_getFwVersion(P_Unit_Version_t * version)
{
    if (iosfsb_read(IOSFSB_PUNIT_PORT, ADDR_FW_VERSION, (unsigned int *)version, IOSFSB_REG_READ))
    {
        return (-1);
    }
    return 0;
}
/* ******************************************************************** */


/* ******************************************************************** */
static int p_unit_cmd(unsigned int command)
{
    unsigned int data = command;

    DPRINTK("Enter cmd = 0x%08X \n", command);
    //Send Command to Punit
    if (iosfsb_write(IOSFSB_PUNIT_PORT, ADDR_DOCSIS_PM_CMD, data, IOSFSB_REG_WRITE))
    {
        return (-1);
    }


    PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT1_SET, BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR);


    //Complete Initiator Handshake with Punit
    do
    {
        if (iosfsb_read(IOSFSB_PUNIT_PORT, ADDR_DOCSIS_PM_CMD, &data, IOSFSB_REG_READ))
        {
            PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT1_CLR, BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR);
            return (-1);
        }

#ifdef CONFIG_MACH_PUMA6_FPGA
    data = P_UNIT_CMD_ACK;
#endif

    }
    while (P_UNIT_CMD_ACK != data);

    if (iosfsb_write(IOSFSB_PUNIT_PORT, ADDR_DOCSIS_PM_CMD, P_UNIT_CMD_IDLE, IOSFSB_REG_WRITE))
    {
        PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT1_CLR, BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR);
        return (-1);
    }
    PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT1_CLR, BOOTCFG_REG_SW_INT1_ARM11_2_PUNIT_ISR);
    DPRINTK("Exit  cmd = 0x%08X \n", command);

    return 0;
}
/* ******************************************************************** */


/* ******************************************************************** */
static int p_unit_cmd_wr_data(unsigned int  command, unsigned int  data)
{
    int rc;

    DPRINTK("Enter cmd = 0x%08X data = 0x%08X\n", command, data);
    //Send Data
    if(iosfsb_write(IOSFSB_PUNIT_PORT, ADDR_DOCSIS_PM_DATA, data, IOSFSB_REG_WRITE))
    {
        return (-1);
    }

    //Send Command and Wait for ACK
    rc = p_unit_cmd(command);
    DPRINTK("Exit  cmd = 0x%08X data = 0x%08X\n", command, data);

    return rc;
}
/* ******************************************************************** */


/* ******************************************************************** */
static int p_unit_cmd_rd_data(unsigned int command, unsigned int * data)
{
    int rc;

    DPRINTK("Enter cmd = 0x%08X data_ptr = 0x%p\n", command, data);
    //Send Command and Wait for ACK
    if (p_unit_cmd(command))
    {
        return (-1);
    }

    //Get Returned Data
    rc = iosfsb_read(IOSFSB_PUNIT_PORT, ADDR_DOCSIS_PM_DATA, data, IOSFSB_REG_READ);
    DPRINTK("Exit  cmd = 0x%08X data_ptr = 0x%p\n", command, data);

    return rc;
}
/* ******************************************************************** */



/* ******************************************************************** */
/*                                                                      */
/*                                                                      */
/*           KERNEL   APIs    ...                                       */
/*                                                                      */
/*                                                                      */
/* ******************************************************************** */


/* ******************************************************************** */
int p_unit_reset_soc( void )
{
    unsigned int atomRebootStatus;
    int atomRebootNumIntervals = ATOM_REBOOT_POLL_NUM_INTERVALS;

    DPRINTK("Enter, gPunitResetEvent=%d\n", gPunitResetEvent);

    /* If gPunitResetEvent is set it means that reset came from Punit - no need to wait for APP to finish its reboot process, only need to ACK the Punit that we are done */
    /* Otherwise, we wait for APP to indicate it finished its reboot process and than we issue Punit cold reset */
    if (!gPunitResetEvent)
    {
        might_sleep();

        atomRebootStatus = PAL_sysBootCfgCtrl_ReadReg(BOOTCFG_REG_SW_INT_STAT);
        while ((atomRebootStatus & BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR) && atomRebootNumIntervals)
        {
            atomRebootStatus = PAL_sysBootCfgCtrl_ReadReg(BOOTCFG_REG_SW_INT_STAT);
            msleep(ATOM_REBOOT_POLL_INTERVAL_MSEC); /* Sleep for 100msec */
            atomRebootNumIntervals--;
        }
    }

    down(&P_Unit_Sem);

    /* In case of reset event from Punit, we just need to ACK it. Otherwise, we issue cold reset */
    if (gPunitResetEvent)
    {
        p_unit_cmd(P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION_ACK);
    }
    else
    {
        p_unit_cmd(P_UNIT_CMD_RESET_REQ_COLD_RESET);
    }

    up(&P_Unit_Sem);
    DPRINTK("Exit, gPunitResetEvent=%d\n", gPunitResetEvent);
    return 0;
}
EXPORT_SYMBOL(p_unit_reset_soc);
/* ******************************************************************** */

/* Wait for APP CPU suspend done event from pUnit*/
/* return success = 0, failure = -1*/
/* ******************************************************************** */
int p_unit_get_app_cpu_bbu_mode( unsigned long timeInMilliseconds )
{
    int retval = 0;
    unsigned long timeInJiffies = msecs_to_jiffies(timeInMilliseconds);

    DPRINTK("timeInJiffies = %lu\n", timeInJiffies);

    /*Wait for pUint interrupt that App CPU suspend done*/
    retval = wait_event_interruptible_timeout(wq, (atomic_read(&p_unit_enter_bbu_mode_done_flag) != 0), timeInJiffies);

    DPRINTK("retval = %d \n", retval);

    if (retval != 0)
    {
        return 0;
    }

    return (-1);
}
EXPORT_SYMBOL(p_unit_get_app_cpu_bbu_mode);
/* ******************************************************************** */

/* Resume APP CPU in case suspend done event recieved*/
/* return success = 0, failure = -1*/
/* ******************************************************************** */
int p_unit_app_cpu_exit_bbu_mode( unsigned long timeInMilliseconds )
{
    if (p_unit_get_app_cpu_bbu_mode(timeInMilliseconds) == 0)
    {
        /*Change p_unit_enter_bbu_mode_done_flag in case suspend done event recieved*/
        atomic_set(&p_unit_enter_bbu_mode_done_flag, 0);
        down(&P_Unit_Sem);
        /*resume App cpu*/
        p_unit_cmd( P_UNIT_CMD_BBU_EXIT_BBU_MODE );
        up(&P_Unit_Sem);
        DPRINTK("Exit\n");
        return 0;
    }

    return (-1);
}
EXPORT_SYMBOL(p_unit_app_cpu_exit_bbu_mode);
/* ******************************************************************** */


/* ******************************************************************** */
int p_unit_get_reset_reason ( unsigned long * reason )
{
    int rc;

    DPRINTK("Enter\n");
    down(&P_Unit_Sem);

    rc = p_unit_cmd_rd_data( P_UNIT_CMD_RESET_GET_LAST_RESET_CAUSE, (unsigned int *)reason );

    up(&P_Unit_Sem);
    DPRINTK("Exit\n");

    return rc;
}
EXPORT_SYMBOL(p_unit_get_reset_reason);
/* ******************************************************************** */


/* ******************************************************************** */
int p_unit_get_reset_duration ( unsigned long * duration )
{
    int rc;

    DPRINTK("Enter\n");
    down(&P_Unit_Sem);

    rc = p_unit_cmd_rd_data( P_UNIT_CMD_RESET_GET_DURATION_RESET_HELD, (unsigned int *)duration );

    up(&P_Unit_Sem);
    DPRINTK("Exit\n");

    return rc;
}
EXPORT_SYMBOL(p_unit_get_reset_duration);
/* ******************************************************************** */


/* ******************************************************************** */
int p_unit_set_watchdog_action(P_UnitWatchdogAction_e action)
{
    unsigned int command;

    DPRINTK("Enter\n");
    switch (action)
    {
        case WATCHDOG_DO_COLD_RESET:    command = P_UNIT_CMD_WATCHDOG_DO_COLD_RESET;    break;
        case WATCHDOG_DO_WARM_RESET:    command = P_UNIT_CMD_WATCHDOG_DO_WARM_RESET;    break;
        case WATCHDOG_DO_CPU_RESET:     command = P_UNIT_CMD_WATCHDOG_DO_CPU_RESET ;    break;
        case WATCHDOG_DO_NOTHING:       command = P_UNIT_CMD_WATCHDOG_DO_NOTHING   ;    break;
        default:
            DPRINTK("Exit\n");
            return (-1);
    }

    down(&P_Unit_Sem);

    p_unit_cmd( command );

    up(&P_Unit_Sem);
    DPRINTK("Exit\n");
    return 0;
}
EXPORT_SYMBOL(p_unit_set_watchdog_action);
/* ******************************************************************** */



/* ******************************************************************** */
/*                                                                      */
/*                                                                      */
/*           IOCTL Implementation                                       */
/*                                                                      */
/*                                                                      */
/* ******************************************************************** */

/**************************************************************************/
/*! \fn static long p_unit_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
 **************************************************************************
 *  \brief This function handles device requests.
 *  \param struct file *filp - the device file pointer
 *  \param unsigned int cmd - the command to be performed
 *  \param unsigned long arg - pointer to the user request
 *  \return long - 0 on success else negative number.
 **************************************************************************/
static long p_unit_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch (cmd)
    {
        case P_UNIT_RESET_SOC_REG_CMD:
        {
            if (p_unit_reset_soc())
            {
                ret = -EFAULT;
            }
            break;
        }

        case P_UNIT_GET_LAST_RESET_REASON_CMD:
        {
            unsigned long rstReason;

            if (p_unit_get_reset_reason(&rstReason))
            {
                ret = -EFAULT;
            }
            else
            {
                if (copy_to_user((void __user *)arg, &rstReason, sizeof(unsigned long)))
                {
                     ret = -EFAULT;
                }
            }
            break;
        }

        case P_UNIT_GET_LAST_RESET_DURATION_CMD:
        {
            unsigned long rstDuration;

            if (p_unit_get_reset_duration(&rstDuration))
            {
                ret = -EFAULT;
            }
            else
            {
                if (copy_to_user((void __user *)arg, &rstDuration, sizeof(unsigned long)))
                {
                     ret = -EFAULT;
                }
            }
            break;
        }

        case P_UNIT_SET_WATCHDOG_ACTION_CMD:
        {
            unsigned long watchdogAction;

            if (copy_from_user(&watchdogAction, (void __user *)arg, sizeof(watchdogAction)))
            {
                return -EFAULT;
            }

            if (p_unit_set_watchdog_action((P_UnitWatchdogAction_e)watchdogAction))
            {
                return -EFAULT;
            }

            break;
        }

        case P_UNIT_SET_APP_CPU_EXIT_BBU_MODE:
        {
            if (p_unit_app_cpu_exit_bbu_mode(arg))
            {
                DPRINTK("Failed to p_unit_exit_bbu_mode \n");
                ret = -EFAULT;
            }
            break;
        }

        case P_UNIT_GET_APP_CPU_BBU_MODE:
        {
            if (p_unit_get_app_cpu_bbu_mode(arg))
            {
                DPRINTK("Failed to p_unit_get_bbu_mode \n");
                ret = -EFAULT;
            }
            break;
        }

        default:
            printk(KERN_ERR "%s:%d Invalid IOCTL(0x%08X) has been received \n",__FUNCTION__,__LINE__,cmd);
            ret = -ENOSYS;
            break;
    }
    return ret;
}


/* ******************************************************************** */
/*                                                                      */
/*                                                                      */
/*           Proc File System ...                                       */
/*                                                                      */
/*                                                                      */
/* ******************************************************************** */

P_UNIT_RESET_STR(rst_reason_names);
P_UNIT_RESET_ORIGIN_STR(rst_origin_names);

#define     P_UNIT_PROC_FS_BUFF_SZ          (32*128)
static char p_unit_proc_fs_buffer[ P_UNIT_PROC_FS_BUFF_SZ ];

int p_unit_proc_dump_fw_version(char* buf, char **start, off_t offset, int count, int *eof, void *data)
{
    static   int    buff_size;
             int    len = 0;

     if (0 == offset)
     {
         P_Unit_Version_t   version;

         if (p_unit_getFwVersion( &version ))
         {
             len += sprintf (&p_unit_proc_fs_buffer[len]," ERROR: function p_unit_getFwVersion has failed\n");
         }
         else
         {
             len += sprintf (&p_unit_proc_fs_buffer[len],
                             "P-UNIT : FW version is [ %d.%d.%d ] Silicon Stepping [%02X]\n",
                             version.field.NumMajor,
                             version.field.NumMinor,
                             version.field.NumRelease,
                             version.field.Stepping
                            );
         }

         buff_size = len;
     }
     else
     {
         len = buff_size - offset;
     }

     if (len > count)
     {
         len = count;
     }

     memcpy(buf, &p_unit_proc_fs_buffer[offset], len);

     return len;
}

int p_unit_proc_dump_status(char* buf, char **start, off_t offset, int count, int *eof, void *data)
{
    static   int    buff_size;
             int    len = 0;
    unsigned int    rstReason;
    unsigned int    rstOrigin;
    unsigned int    rstDuration;


    if (0 == offset)
    {
        do
        {
            unsigned int tmp;

            p_unit_get_reset_reason((unsigned long *)&tmp);

            rstReason = (tmp & 0x00FF);
            rstOrigin = (tmp & 0xFF00) >> 8;

            len += sprintf (&p_unit_proc_fs_buffer[len], "P-UNIT : Last reset reason   = 0x%08X [ %-30s ]\n",rstReason, (rstReason < RESET_REASONS_MAX) ? rst_reason_names[ rstReason ] : "UNKNOWN");

            if (len + 128 > P_UNIT_PROC_FS_BUFF_SZ)
            {
                sprintf(&p_unit_proc_fs_buffer[len - 6],"\n...\n");
                break ;
            }

            len += sprintf (&p_unit_proc_fs_buffer[len], "P-UNIT : Last reset origin   = 0x%08X [ %-30s ]\n",rstOrigin, (rstOrigin < RESET_ORIGIN_MAX) ? rst_origin_names[ rstOrigin ] : "UNKNOWN");

            if (len + 128 > P_UNIT_PROC_FS_BUFF_SZ)
            {
                sprintf(&p_unit_proc_fs_buffer[len - 6],"\n...\n");
                break ;
            }

            p_unit_get_reset_duration((unsigned long *)&rstDuration);

            len += sprintf (&p_unit_proc_fs_buffer[len], "P-UNIT : Last reset duration = %d.%d Seconds\n",rstDuration/1000, rstDuration%1000);

            if (len + 128 > P_UNIT_PROC_FS_BUFF_SZ)
            {
                sprintf(&p_unit_proc_fs_buffer[len - 6],"\n...\n");
                break ;
            }
        }
        while (0);

        buff_size = len;
    }
    else
    {
        len = buff_size - offset;
    }

    if (len > count)
    {
        len = count;
    }

    memcpy(buf, &p_unit_proc_fs_buffer[offset], len);

    return len;
}



/**************************************************************************/
/*! \fn int p_unit_proc_control(struct file *fp, const char * buf, unsigned long count, void * data)
 **************************************************************************
 *  \brief proc file to configure P-Unit
 **************************************************************************/
static int p_unit_proc_control(struct file *fp, const char * buf, unsigned long count, void * data)
{
    unsigned char local_buf[50];
    int ret_val = 0;
    unsigned int operation;

    if (count > 50)
    {
        printk(KERN_ERR "Buffer Overflow\n");
        return -EFAULT;
    }

    if(copy_from_user(local_buf,buf,count))
        return -EFAULT;;
    local_buf[count-1]='\0'; /* Ignoring last \n char */
    ret_val = count;

    if (strcmp(local_buf,"wd"))
    {
        sscanf(local_buf+2,"%d",&operation);
        p_unit_set_watchdog_action((P_UnitWatchdogAction_e)operation);
    }
    else
    {
        printk(KERN_ERR "Unknown operation, must be read ('r') or write ('w') \n");
        return -EFAULT;
    }

    return ret_val;
}


/* ******************************************************************** */
/*                                                                      */
/*                                                                      */
/*           Module init ...                                            */
/*                                                                      */
/*                                                                      */
/* ******************************************************************** */

/* Structure to map driver functions to kernel */
struct file_operations p_unit_drv_fops =
{
        .owner   = THIS_MODULE,
        .unlocked_ioctl   = p_unit_ioctl,
        .open    = NULL,
        .release = NULL,
};

static void p_unit_isr_cmd_ack(void)
{
    if (iosfsb_write(IOSFSB_PUNIT_PORT, ADDR_PM_DOCSIS_DATA, 0, IOSFSB_REG_WRITE))
    {
        printk("Error in first iosfsb_write of p_unit_isr_cmd_ack \n");
    }
    if (iosfsb_write(IOSFSB_PUNIT_PORT, ADDR_PM_DOCSIS_CMD, P_UNIT_CMD_ACK, IOSFSB_REG_WRITE))
    {
        printk("Error in second iosfsb_write of p_unit_isr_cmd_ack \n");
    }
}

static void p_unit_workq_func(struct work_struct *work)
{
    unsigned int command, data;

    if (iosfsb_read(IOSFSB_PUNIT_PORT, ADDR_PM_DOCSIS_CMD, &command, IOSFSB_REG_READ))
    {
        printk("Error in first iosfsb_read \n");
    }
    if (iosfsb_read(IOSFSB_PUNIT_PORT, ADDR_PM_DOCSIS_DATA, &data, IOSFSB_REG_READ))
    {
        printk("Error in second iosfsb_read \n");
    }

    switch (command)
    {
    case P_UNIT_CMD_BBU_ENTER_BBU_MODE_DONE:
        DPRINTK("#########  Interrupted - APP CPU enter bbu mode done ##########\n");
        p_unit_isr_cmd_ack();
        atomic_set(&p_unit_enter_bbu_mode_done_flag, 1);
        wake_up_interruptible(&wq);
        break;

    case P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION:
        DPRINTK("#########  Interrupted - reset indication ##########\n");
        gPunitResetEvent = 1;
        /* We will start reboot process by simulating APP->NP reboot interrupt */
        PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT_SET, BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR);
        break;

    default:
        printk("#########  Interrupted - unknown command 0x%04X, data 0x%08X ##########\n", command, data);
        break;
    }
}

static irqreturn_t p_unit_isr(int irq, void *dev_id)
{
    /* Que work */
    queue_work(gPunitWorkqueue, &gPunitWork);

    return IRQ_HANDLED;
}


/**************************************************************************/
/*! \fn static int __init p_unit_drv_init(void)
 **************************************************************************
 *  \brief This function is the power control unit device module init function.
 *  \return long - 0 on success else negative number.
 **************************************************************************/
static int __init p_unit_drv_init(void)
{
    int res;
    gPunitResetEvent = 0; /* Indication if reset event has been received from Punit. Default is FALSE */

    if (alloc_chrdev_region(&p_unit_dev_t, 0, count, DEV_NAME) < 0)
    { /* count indicates how many minors we get */
        printk(KERN_ERR "%s:%d Failed to register character device region %s\n",__FUNCTION__,__LINE__,DEV_NAME);
        return (-1);
    }

    if (!(p_unit_cdev = cdev_alloc()))
    {
        printk(KERN_ERR "%s:%d Failed to allocate character device %s\n",__FUNCTION__,__LINE__,DEV_NAME);
        unregister_chrdev_region(p_unit_dev_t, count);
        return (-1);
    }

    /* Connect the file operations with the cdev                            */
    cdev_init(p_unit_cdev, &p_unit_drv_fops);

    /* Connect the major/minor number to the cdev  - Activates the device   */
    if (cdev_add(p_unit_cdev, p_unit_dev_t, count) < 0)
    {
        printk(KERN_ERR "%s:%d Failed to add character device %s\n",__FUNCTION__,__LINE__,DEV_NAME);
        cdev_del(p_unit_cdev);
        unregister_chrdev_region(p_unit_dev_t, count);
        return (-1);
    }
    /* ******************************************************************** */
    /* connection to the udev                                               */
    /* ceates a class directory under /sys/class                            */
    p_unit_udev_class = class_create(THIS_MODULE, DEV_NAME);
    /* ceates a class directory under /sys/class/DEV_NAME named DEV_NAME    */
    /* creates 3 file: dev, uevent, subsystem                               */
    device_create(p_unit_udev_class, NULL, p_unit_dev_t, NULL, "%s", DEV_NAME);


    printk(KERN_INFO "%s:%d Character device %s Major = %d, Minor = %d has been registered\n",__FUNCTION__,__LINE__,DEV_NAME,MAJOR(p_unit_dev_t),MINOR(p_unit_dev_t));

    sema_init(&P_Unit_Sem,1);

    /* Proc filesystem utilities.... */
    {
        struct proc_dir_entry * dir;

        if (NULL == (dir = proc_mkdir("P-UNIT", NULL)))
        {
            printk(KERN_ERR "%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
            return -1;
        }

        if (NULL == (create_proc_read_entry( "status" , 0, dir, p_unit_proc_dump_status, NULL )))
        {
            printk(KERN_ERR "%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
            return -1;
        }

        if (NULL == (create_proc_read_entry( "fw_version" , 0, dir, p_unit_proc_dump_fw_version, NULL )))
        {
            printk(KERN_ERR "%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
            return -1;
        }

        if (NULL == (dir = create_proc_entry("control",0 ,dir)))
        {
            printk(KERN_ERR "%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
            return -1;
        }
        dir->write_proc = p_unit_proc_control;
    }

    if (p_unit_set_watchdog_action( WATCHDOG_DO_COLD_RESET ))
    {
        printk(KERN_ERR "%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
        return -1;
    }

    gPunitWorkqueue = create_workqueue("gPunitWorkqueue");
    if (gPunitWorkqueue == NULL)
    {
        printk(KERN_ERR "%s:%d ERROR ....Failed to create gPunitWorkqueue\n",__FUNCTION__,__LINE__);
        return -1;
    }
    /* Init the work */
    INIT_WORK(&gPunitWork, p_unit_workq_func);

    atomic_set(&p_unit_enter_bbu_mode_done_flag, 0);
    res = request_irq(AVALANCHE_PUNIT_INT, p_unit_isr, IRQF_TRIGGER_RISING | IRQF_DISABLED, "punit_int", &count );
    printk("%d = result from request_irq \n", res);
    if (res)
    {
        printk(KERN_ERR "PUNIT: Unable to allocate pUnit IRQ\n");
        return -1;
    }

    {
        P_Unit_Version_t   version;

        if (p_unit_getFwVersion( &version ))
        {
            printk(KERN_ERR " ERROR: function p_unit_getFwVersion has failed\n");
            return  -1;
        }
        else
        {
            if (  version.reg & 0xFFFFFF >= 0x010006  )
            {
                printk(KERN_INFO "P-UNIT : Enable reset event handling ... \n");
                down(&P_Unit_Sem);
                p_unit_cmd_wr_data(P_UNIT_CMD_RESET_DOCSIS_RESET_INDICATION_ACK_TIMEOUT, 5000); // ACK timeout is 5 seconds
                p_unit_cmd_wr_data(P_UNIT_CMD_RESET_EN_DOCSIS_RESET_INDICATION, 1);
                up(&P_Unit_Sem);
            }
        }
    }

    {
        char    print_buf[P_UNIT_PROC_FS_BUFF_SZ];
        char *  print_ptr;
        int     print_len = 0;

        print_len += p_unit_proc_dump_fw_version( print_buf,             NULL, 0, P_UNIT_PROC_FS_BUFF_SZ, NULL, NULL );
        print_len += p_unit_proc_dump_status(     print_buf + print_len, NULL, 0, P_UNIT_PROC_FS_BUFF_SZ - print_len, NULL, NULL );

        printk("\n==============================================================================\n");
        for ( print_ptr = &print_buf[0]; print_len;  print_ptr++, print_len-- )  { printk("%c",*print_ptr); }
        printk("==============================================================================\n");
    }

    return(0);
}


/**************************************************************************/
/*! \fn static void __exit p_unit_drv_exit(void)
 **************************************************************************
 *  \brief This function is the power control unit device module exit function.
 **************************************************************************/
static void __exit p_unit_drv_exit(void)
{
    if (p_unit_cdev)
    {
        cdev_del(p_unit_cdev);
    }

    unregister_chrdev_region(p_unit_dev_t, count);
    device_destroy(p_unit_udev_class, p_unit_dev_t);
    class_destroy(p_unit_udev_class);
    free_irq(AVALANCHE_PUNIT_INT, NULL);

    flush_workqueue(gPunitWorkqueue);
    destroy_workqueue(gPunitWorkqueue);

    printk(KERN_INFO "%s:%d %s device has been unregistered\n",__FUNCTION__,__LINE__,DEV_NAME);
}
/*************************************************************************************/

module_init(p_unit_drv_init);
module_exit(p_unit_drv_exit);

/* Driver identification */
MODULE_DESCRIPTION("Power Control Unit Device Driver");
MODULE_LICENSE("GPL");


