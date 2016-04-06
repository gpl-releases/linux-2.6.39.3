/*
 * kernel/arm_atom_mbx.c
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
 *  Contact Information: liat.guedj@intel.com
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <hw_mutex_ctrl.h> 
#include "arm_atom_mbx.h"
#include <puma6.h>
#include <puma6_bootcfg_ctrl.h>
#include <linux/interrupt.h>
#include <linux/reboot.h>



#define ARM_MBX_STRUCT_BASE_ADDR        (AVALANCHE_SRAM_BASE + 0x00001E00)
#define ATOM_MBX_STRUCT_BASE_ADDR       (AVALANCHE_SRAM_BASE + 0x00001E04)

#define ARM_MBX_STRUCT_EVENT_MASK_ADDR  (ARM_MBX_STRUCT_BASE_ADDR)
#define ARM_MBX_STRUCT_ACK_MASK_ADDR    (ARM_MBX_STRUCT_BASE_ADDR + 2)


#define ATOM_MBX_STRUCT_EVENT_MASK_ADDR  (ATOM_MBX_STRUCT_BASE_ADDR)
#define ATOM_MBX_STRUCT_ACK_MASK_ADDR    (ATOM_MBX_STRUCT_BASE_ADDR + 2)

/* RPC-IF INFO */
/* Data format in this address would be in LE byte order */
#define NPCPU_RPC_IPV4_ADDR				(ARM_MBX_STRUCT_BASE_ADDR + 0x8)
#define APPCPU_RPC_IPV4_ADDR			(ARM_MBX_STRUCT_BASE_ADDR + 0xC)
#define RPC_NETWORK_MASK				(ARM_MBX_STRUCT_BASE_ADDR + 0x10)
#define VLAN_ID							(ARM_MBX_STRUCT_BASE_ADDR + 0x14)

/* 15 seconds max delay */
#define MBX_MAX_POLLING_DELAY		(msecs_to_jiffies(3000))

#define reg_write_32(addr, data) (( *(volatile unsigned int *) (addr) ) = (data))
#define reg_read_32(addr)        ( *(volatile unsigned int *) (addr) )
#define reg_write_16(addr, data) (( *(volatile unsigned short *) (addr) ) = (data))
#define reg_read_16(addr)        ( *(volatile unsigned short *) (addr) )

static unsigned int ref = 0;

static int mbx_Major;

static struct class *mbx_class;

/* Polloing on specivec ACK/Event bit - max timeout is 1 sec */ 
static int mbx_wait_till_ready(volatile unsigned int regAddressToPoll, unsigned short eventId)
{
	unsigned long timeout = jiffies + MBX_MAX_POLLING_DELAY;

    do{
		if( (le16_to_cpu( reg_read_16(regAddressToPoll) ) & eventId) == 0 )
        {
			continue;
		}
        else
        {
			return 0;  /* Normal exit */
		}

	}while(time_after(timeout, jiffies));

    printk( "*****************************************************************\n" );
	printk( "*** mbx_wait_till_ready Wait for ACK/EVENT Fail on timeout     **\n" );
	printk( "*****************************************************************\n" );

	return 1;
}

static int mbx_open(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "arm_atom_mbx driver open ref %d\n", ++ref);
    return 0;
}

static int mbx_close(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "arm_atom_mbx driver close ref %d\n", --ref);
    return 0;
}

static long mbx_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct arm11_atom_mbx_user  mbx_usr_info;
	int err = 0;

    /* copy from the user the event ID and if need to copy parameter */
    if (arg)
    {
        if (copy_from_user(&mbx_usr_info, (struct arm11_atom_mbx_user *)arg, sizeof(struct arm11_atom_mbx_user)) )
        {
            return -EFAULT;
        }
    }

    switch (cmd)
    {
    case MBX_SEND_EVENT_CMD:
        {
            err = arm_atom_mbx_send_notification(mbx_usr_info.eventId,(unsigned int *)&mbx_usr_info.parameter);
            break;
        }

    case MBX_GET_EVENT_CMD:
        {
            err = arm_atom_mbx_receive_event_notification(mbx_usr_info.eventId,(unsigned int *)&mbx_usr_info.parameter);
            if(mbx_usr_info.isParamRequired)
            {
                if (copy_to_user(&(((struct arm11_atom_mbx_user *)arg)->parameter ), &mbx_usr_info.parameter, sizeof(mbx_usr_info.parameter) ))
                {
                    return -EFAULT;
                }
            }
            break;
        }

    case MBX_SEND_ACK_CMD:
        {
            err = arm_atom_mbx_send_ack(mbx_usr_info.eventId);
            break;
        }

    case MBX_RECEIVE_ACK_CMD:
        {
            err = arm_atom_mbx_receive_specific_ack(mbx_usr_info.eventId);
            break;
        }

    case MBX_REBOOT:
        {
            /* Disable Atom->ARM interrupt since we are already in reboot process */
            disable_irq(AVALANCHE_ATOM2ARM_INT);
            printk("Disable IRQ #%d for device named %s\n", AVALANCHE_ATOM2ARM_INT, DEVICE_NAME);

            /* Set the Atom->ARM interrupt. We depend on this bit to be cleared by Atom at the end of its reboot process, but it may be that when we
               check the status (when we finish our own reboot), Atom didn't even started (doesn't sound logical, but just in case...) */
            PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT_SET, BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR);

            /* Notify ATOM that we are starting a reboot process */
            PAL_sysBootCfgCtrl_WriteReg(BOOTCFG_REG_SW_INT1_SET, BOOTCFG_REG_SW_INT1_ARM11_2_ATOM_REBOOT_ISR);
            break;
        }
    default:
        {
            printk(KERN_ERR "ARM/ATOM Mailbox driver receive Wrong IOCTL command = 0x%x \n",cmd);
            return -EFAULT;
        }
    }

    return err;
}
/*
 * arm_atom_mbx_send_notification 
 * @eventId: Event need send to ATOM
 * @ParamPtr - pointer to parameter for this event (optional)
 * 
 * Send Specific event notification to the ATOM. 
 * If Parameter is requiered update the corresponding parameter (in specific address of ARM SRAM)
 * Returns 0 if success, negative if error / timeout
 */
long arm_atom_mbx_send_notification(unsigned short eventID, unsigned int *paramPtr)
{
    volatile unsigned short eventMask;

    /* Take the HW mutex */
    if (hw_mutex_lock_interruptible(HW_MUTEX_ARM_MBX) != 0)
    {
        printk("arm_atom_mbx_send_notification- Can't lock HW mutex\n");
        return -ENOSYS;
    }

    /* Get the event mask */
    eventMask = le16_to_cpu( reg_read_16(ARM_MBX_STRUCT_EVENT_MASK_ADDR) );
    eventMask |= eventID;

    switch(eventID)
        {
	    case NPCPU_EVENT_RPC_IF_OBTAIN_ADDR:
		{
			struct npcpu_rpc_info *rpc_info = (struct npcpu_rpc_info *)paramPtr;
			BUG_ON(!rpc_info);
	        reg_write_32( NPCPU_RPC_IPV4_ADDR, (rpc_info->npcpu_ipv4_addr) );
			reg_write_32( APPCPU_RPC_IPV4_ADDR,(rpc_info->appcpu_ipv4_addr) );
	        reg_write_32( RPC_NETWORK_MASK,    (rpc_info->netmask) );
	        reg_write_32( VLAN_ID,             cpu_to_le32(rpc_info->vlan_id) );
	        break;
	    }

    case ARM11_EVENT_GPIO_INIT_EXIT:
    case ARM11_EVENT_SPI_INIT_EXIT:
    case ARM11_EVENT_EMMC_INIT_EXIT:
    case ARM11_EVENT_EMMC_ADVANCE_INIT_EXIT:
        {
            /* Event w/o parameters */
            break;
        }
    default:
        {
            /* Release the HW Mutex */
            hw_mutex_unlock(HW_MUTEX_ARM_MBX);		
            printk( "ERROR : arm_atom_mbx_send_notification Event=0x%x is invalid \n", eventID );
            return -ENOSYS;
        }
    }

    /* set the appropiated bit of the event mask on ARM structur */
    reg_write_16(ARM_MBX_STRUCT_EVENT_MASK_ADDR, cpu_to_le16(eventMask));

    /* Release the HW Mutex */
    hw_mutex_unlock(HW_MUTEX_ARM_MBX);

    return 0;
}
/*
 * arm_atom_mbx_send_ack 
 * @eventId: Event need to ACK on
 *  
 * Send ACK to ATOM on specific event received by ARM 
 * Set the apropriate ACK bit in the ATOM mbox struct indicates ARM was processing this event
 * Returns 0 if success, negative if error / timeout
 */
long arm_atom_mbx_send_ack(unsigned short eventID)
{
    volatile unsigned short ackMask;
    /* Take the HW mutex */
    if (hw_mutex_lock_interruptible(HW_MUTEX_ATOM_MBX) != 0)
    {
        printk("arm_atom_mbx_send_ack- Can't lock HW mutex\n");
        return -ENOSYS;
    }

    ackMask = le16_to_cpu( reg_read_16(ATOM_MBX_STRUCT_ACK_MASK_ADDR) );
    printk("arm_atom_mbx_send_ack- ack Mask = 0x%x\n",ackMask);

    ackMask |= eventID;
    /* Set the ACK bit on ATOM ACK bit mask structure*/
    reg_write_16(ATOM_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));
    printk("arm_atom_mbx_send_ack- Writing ack Mask = 0x%x for event = 0x%x\n",ackMask,eventID);

    /* Release the HW Mutex */
    hw_mutex_unlock(HW_MUTEX_ATOM_MBX);

    return 0;
}
/*
 * arm_atom_mbx_receive_specific_ack 
 * @eventId: Wait for ack on Event Sent earlier by the ARM
 * 
 * Wait for specific  ACK from the ATOM - indicates ATOM got this event
 * polling the ARM Mbx structure (ACK bit Mask)- DO not wait forever exit on timeout
 * Returns 0 if success, negative if error / timeout
 */
long arm_atom_mbx_receive_specific_ack(unsigned short eventId)
{
    volatile unsigned short ackMask;
    volatile unsigned short eventMask;


     if( mbx_wait_till_ready(ARM_MBX_STRUCT_ACK_MASK_ADDR, eventId) )
     {
         printk( "ATOM-ARM MBX is stuck - ACK from ATOM on eventId=0x%x NOT arrived \n", eventId );
         return -ENOSYS;
     }
     printk( "\nGOT ACK from ATOM on eventId=0x%x arrived \n", eventId );

    /* ACK was received - Need to Clear the event and ACK bit*/
    /* Take the HW mutex */
    if (hw_mutex_lock_interruptible(HW_MUTEX_ARM_MBX) != 0)
    {
        printk("arm_atom_mbx_send_ack- Can't lock HW mutex\n");
        return -ENOSYS;
    }

    printk( "Clear the ACK Vector from ATOM on eventId=0x%x \n", eventId );

    /* Clear ARM ACK vector */
    ackMask = le16_to_cpu( reg_read_16(ARM_MBX_STRUCT_ACK_MASK_ADDR) );
    ackMask = (ackMask & ~(eventId));
    reg_write_16(ARM_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));

    /* Clear ARM event vector */
    eventMask = le16_to_cpu( reg_read_16(ARM_MBX_STRUCT_EVENT_MASK_ADDR) );
    eventMask = (eventMask & ~(eventId));
    reg_write_16(ARM_MBX_STRUCT_EVENT_MASK_ADDR,cpu_to_le16(eventMask));

    /* Release the HW Mutex */
    hw_mutex_unlock(HW_MUTEX_ARM_MBX);

    return 0;
}

/*
 * arm_atom_mbx_receive_event_notification
 * @eventId: Wait for this event
 * param - if the event requiered parameter - this is the output parameter
 * 
 * Wait for specific event from the ATOM - 
 * polling the ATOM Mbx structure - DO not wait forever exit on timeout
 * Returns 0 if success, negative if error / timeout
 */
long arm_atom_mbx_receive_event_notification(unsigned short eventId, unsigned int *param)
{
    volatile unsigned short ackMask;
    bool ackRequiered = 0;

    if( mbx_wait_till_ready(ATOM_MBX_STRUCT_EVENT_MASK_ADDR, eventId) )
    {
        printk( "ATOM-ARM MBX is stuck - Wait for Event=0x%x from ATOM fail on timeout \n", eventId );
        return -ENOSYS;
    }

    switch(eventId)
    {
        case ATOM_EVENT_SPI_ADVANCE_EXIT:
        case ATOM_EVENT_EMMC_ADVANCE_EXIT:
        {
            break;
        }

        default:
        {
            printk( "ERROR : receive_specific_event_notification Event=0x%x is invalid \n", eventId );
            return -ENOSYS;
        }
        
    }

    printk( "receive_specific_event_notification Event=0x%x  parameter = 0x%x \n", eventId,(param==NULL?'0':*param) );

    if (ackRequiered)
    {
        /* Take the HW mutex */
        if (hw_mutex_lock_interruptible(HW_MUTEX_ATOM_MBX) != 0)
        {
            printk("ERROR - arm_atom_mbx_receive_event_notification- Can't lock HW mutex\n");
            return -ENOSYS;
        }

        ackMask = le16_to_cpu(reg_read_16(ATOM_MBX_STRUCT_ACK_MASK_ADDR)) | eventId ;
        reg_write_16(ATOM_MBX_STRUCT_ACK_MASK_ADDR,cpu_to_le16(ackMask));

        /* Set the appropiated ACK bit  */
        /* Release the HW Mutex */
        hw_mutex_unlock(HW_MUTEX_ATOM_MBX);
    }

    return 0;
}

static struct file_operations mbx_fops = {
    .owner   = THIS_MODULE,
    .unlocked_ioctl   = mbx_unlocked_ioctl,
	.open = mbx_open,
	.release = mbx_close,
};

static void argv_cleanup(struct subprocess_info *info)
{
	argv_free(info->argv);
}

/*
 * atom2arm_interrupt
 * @irq: interrupt number 
 * @dev: device
 * 
 * interrupt handler of interrupts coming from ATOM. 
 * Currently the only interrupt is issued when ATOM start reboot process
 */
static irqreturn_t atom2arm_interrupt(int irq, void *dev)
{
    int ret = -ENOMEM;
    int cause;

    cause = PAL_sysBootCfgCtrl_ReadReg(BOOTCFG_REG_SW_INT_STAT);
    if (cause & BOOTCFG_REG_SW_INT_ATOM_2_ARM11_INTC_REBOOT_ISR)
    {
        int argc;
        char **argv = argv_split(GFP_ATOMIC, "/sbin/reboot", &argc);
        static char *envp[] = {"HOME=/", "PATH=/sbin:/bin:/usr/sbin:/usr/bin", NULL};
        struct subprocess_info *info;

        printk(KERN_CRIT "Got reboot interrupt from ATOM. Shutting down the system now\n");

        info = call_usermodehelper_setup(argv[0], argv, envp, GFP_ATOMIC);
        if (info == NULL)
        {
            argv_free(argv);
        }
        else
        {
            call_usermodehelper_setfns(info, NULL, argv_cleanup, NULL);
            ret = call_usermodehelper_exec(info, UMH_NO_WAIT);
        }

        if ((info == NULL) || ret)
        {
            printk(KERN_ERR "Failed to reboot: forcing orderly_poweroff\n");
            orderly_poweroff(true);
        }
    }
    else
    {
        printk(KERN_ERR "Got unknown interrupt from ATOM\n");
    }

    return IRQ_RETVAL(IRQ_HANDLED);
}


/* Init the module */
static int __init arm_atom_mbx_init(void)
{
    struct device *mbx_dev;

    // register mbx as a device
    mbx_Major = register_chrdev(0, DEVICE_NAME, &mbx_fops);

    if (mbx_Major < 0) 
    {
        printk(KERN_ERR "ERROR - Failed to register device %s with error %d\n", DEVICE_NAME, mbx_Major);
        return mbx_Major;
    }

    /* create /dev/arm_atom_mbx
    * we use udev to make the file
    */
   mbx_class = class_create(THIS_MODULE,DEVICE_NAME);
   mbx_dev = device_create(mbx_class, NULL, MKDEV(mbx_Major,0),NULL,DEVICE_NAME);

   if(request_irq(AVALANCHE_ATOM2ARM_INT, atom2arm_interrupt, IRQF_ONESHOT, DEVICE_NAME, mbx_dev))
   {
       printk(KERN_ERR "unable to get IRQ #%d for device named %s\n", AVALANCHE_ATOM2ARM_INT, DEVICE_NAME);
       return -ENOMEM;
   }
   printk("enable IRQ #%d for device named %s\n", AVALANCHE_ATOM2ARM_INT, DEVICE_NAME);

    return 0;
}

/* remove the module */
static void __exit arm_atom_mbx_exit(void)
{
    device_destroy(mbx_class,MKDEV(mbx_Major,0));
    class_unregister(mbx_class);
    class_destroy(mbx_class);
    unregister_chrdev(mbx_Major, DEVICE_NAME);
}

module_init(arm_atom_mbx_init);
module_exit(arm_atom_mbx_exit); 


EXPORT_SYMBOL(arm_atom_mbx_send_notification);
EXPORT_SYMBOL(arm_atom_mbx_send_ack);
EXPORT_SYMBOL(arm_atom_mbx_receive_specific_ack);
EXPORT_SYMBOL(arm_atom_mbx_receive_event_notification);

MODULE_DESCRIPTION("ARM11 <-> ATOM Event Mailbox Device Driver");
MODULE_LICENSE("GPL"); 


