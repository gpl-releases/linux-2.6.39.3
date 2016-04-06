/*
 *
 * avalanche_generic_setup.c 
 * Description:
 * avalanche generic initialization
 *
 *
 * Copyright (C) 2008, Texas Instruments, Incorporated
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */
#include <linux/platform_device.h>
#include <asm/arch/update_tag.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <asm/mach/time.h>

#include <asm/cpu.h>
#include <asm/irq.h>
#include <asm/serial.h>
#include <asm/mach-types.h>

#include <linux/proc_fs.h>

#include <asm/uaccess.h>

#include <asm-arm/arch-avalanche/generic/pal.h>
#include <asm-arm/arch-avalanche/generic/release.h>

#ifdef CONFIG_SERIAL_8250
#include <linux/tty.h>
#include <linux/kgdb.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
extern int early_serial_setup(struct uart_port *port);
#endif /* CONFIG_SERIAL_8250 */

#ifdef CONFIG_ARM_AVALANCHE_VLYNQ
int __init vlynq_bus_init(void);
#endif

extern void avalanche_soc_platform_init(void);
extern unsigned int cpu_freq;
extern void avalanche_proc_entries(void);

static int avalanche_p_read_base_psp_version(char* buf, char **start, off_t offset, 
                                             int count, int *eof, void *data)
{
    int len   = 0;
    int limit = count - 80;
    char *cache_mode[4] = {"cached, write through", \
                           "cached, write back", \
                           "uncached"};


    int cache_index = 1; /* default is write back */

	/* write through mode */
	#if defined(CONFIG_CPU_DCACHE_WRITETHROUGH)
		cache_index = 0;
	#endif

	/* uncached mode */
	#if defined(CONFIG_CPU_DCACHE_DISABLE) && defined(CONFIG_CPU_ICACHE_DISABLE)
		cache_index = 2;
	#endif

 

    if(len<=limit)
        len+= sprintf(buf+len, "\nLinux OS version %s\n"\
			       "Avalanche SOC Version: 0x%x operating in %s mode\n"\
			       "Cpu Frequency: %u MHZ\nSystem Bus frequency: %u MHZ\n\n", 
					PSP_RELEASE_TYPE, 
					avalanche_get_chip_version_info(), cache_mode[cache_index],
					cpu_freq/1000000, 
#if defined (CONFIG_MACH_PUMA5)
                    2*avalanche_get_vbus_freq()/1000000);
#else    /* CONFIG_MACH_PUMA6  For Puma-6 SoC */
                    PAL_sysClkcGetFreq(PAL_SYS_CLKC_SSX)/1000000);
#endif

    
    return (len);
}

int avalanche_proc_init(void)
{
    struct proc_dir_entry *avalanche_proc_root;

    avalanche_proc_root = proc_mkdir("avalanche",NULL);
    if(!avalanche_proc_root)
        return -ENOMEM;

    create_proc_read_entry("avalanche/base_psp_version", 0, NULL, avalanche_p_read_base_psp_version, NULL);

	/* Create other proc entries - implemented in avalanche_intc.c */
	avalanche_proc_entries();
    return (0);
}
fs_initcall(avalanche_proc_init);

#ifdef CONFIG_ARM_AVALANCHE_VLYNQ
//arch_initcall(vlynq_bus_init);
#endif


const char *get_system_type(void)
{
	return "Texas Instruments Cable SoC";
}

/* This structure is a subset of old_serial_port in 8250.h. Redefined here
 * because old_serial_port happens to be a private structure of 8250 and 
 * his structure here has got nothing to do with the 8250 concept of 
 * old_serial_port.
 */
struct serial_port_dfns 
{
    unsigned int irq;
    unsigned int iomem_base;
};

/* from asm/serial.h */

/* Standard COM flags (except for COM4, because of the 8514 problem) */
#if 0
#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ)
#define STD_COM4_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF | ASYNC_SKIP_TEST)
#define STD_COM4_FLAGS ASYNC_BOOT_AUTOCONF
#endif
#endif
#define STD_COM_FLAGS UPF_BOOT_AUTOCONF

#ifdef CONFIG_CPU_BIG_ENDIAN
#define AVALANCHE_SERIAL_OFFSET 3
#else
#define AVALANCHE_SERIAL_OFFSET 0
#endif

static struct serial_port_dfns serial_port_dfns[] = {
        {.irq = AVALANCHE_UART0_INT, .iomem_base = (AVALANCHE_UART0_REGS_BASE + AVALANCHE_SERIAL_OFFSET)},
#if (CONFIG_AVALANCHE_NUM_SER_PORTS > 1)
        {.irq = AVALANCHE_UART1_INT, .iomem_base = (AVALANCHE_UART1_REGS_BASE + AVALANCHE_SERIAL_OFFSET)},
#if (CONFIG_AVALANCHE_NUM_SER_PORTS > 2)
        {.irq = AVALANCHE_UART2_INT, .iomem_base = (AVALANCHE_UART2_REGS_BASE + AVALANCHE_SERIAL_OFFSET)},
#endif
#endif
};

int  ti_avalanche_setup(void)
{
    int i, j;
    struct uart_port av_serial[CONFIG_AVALANCHE_NUM_SER_PORTS];

	/* Initialize the platform first up */
    avalanche_soc_platform_init();

#ifdef CONFIG_SERIAL_8250

	memset(&av_serial, 0, sizeof(av_serial));

    for ( i = 1, j = 0; j < CONFIG_AVALANCHE_NUM_SER_PORTS; j++)
    {
        if(j == CONFIG_AVALANCHE_CONSOLE_PORT){        
	        av_serial[j].line		= 0;
        } else {
	        av_serial[j].line		= i++;
        }
	    av_serial[j].irq		= LNXINTNUM(serial_port_dfns[j].irq);
	    av_serial[j].flags		= STD_COM_FLAGS;
#ifdef CONFIG_MACH_PUMA5 /* For Puma-5 SoC */
        av_serial[j].uartclk	= avalanche_get_vbus_freq();
#else /* CONFIG_MACH_PUMA6  For Puma-6 SoC */
        av_serial[j].uartclk	= PAL_sysClkcGetFreq(PAL_SYS_CLKC_UART0); /* UART0-2 use the same Clkc 1x */
#endif
	    av_serial[j].iotype	    = UPIO_MEM;
	    av_serial[j].mapbase 	= IO_VIRT2PHY(serial_port_dfns[j].iomem_base);
	    av_serial[j].membase  	= (char*)serial_port_dfns[j].iomem_base;
	    av_serial[j].regshift	= 2;

#ifdef CONFIG_KGDB_8250
		kgdb8250_add_port(j, &av_serial[j]);
#endif
    } 
 
    for ( i = 0; i < CONFIG_AVALANCHE_NUM_SER_PORTS; i++)
	    if (early_serial_setup(&av_serial[i]) != 0) 
	        printk(KERN_ERR "early_serial_setup on port %d failed.\n", i);		
#endif

	return 0;
}


