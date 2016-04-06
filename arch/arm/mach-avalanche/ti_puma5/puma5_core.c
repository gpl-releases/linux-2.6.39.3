/*
 * puma5_core.c
 * Description:
 * Architecture specific stuff.
 *
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

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#endif
#include <linux/irq.h>
#include <asm/setup.h>
#include <asm/param.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm-arm/arch-avalanche/generic/pal.h>
extern unsigned int cpu_freq;

void arch_init_irq(void);
extern void create_mapping(struct map_desc *md);
extern int ti_avalanche_setup(void);

static unsigned int no_linux_mem_size  = CONFIG_ARM_AVALANCHE_TOP_MEM_RESERVED;
static unsigned int no_linux_mem_last;
static unsigned int no_linux_mem_start;

struct NO_OPERSYS_MEM_DESC_T no_OperSys_memory_desc[eNO_OperSys_END];

#ifdef CONFIG_HIGH_RES_TIMERS
static void config_timer2(u32);
#endif
static struct map_desc puma5_io_desc[] __initdata = {
	{
		.virtual	= IO_VIRT,
		.pfn		= __phys_to_pfn(IO_PHY),
		.length		= IO_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= EMIF3E_VIRT,
		.pfn		= __phys_to_pfn(EMIF3E_PHY),
		.length		= EMIF3E_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= FLASH_0_VIRT,
		.pfn		= __phys_to_pfn(FLASH_0_PHY),
		.length		= FLASH_0_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= FLASH_1_VIRT,
		.pfn		= __phys_to_pfn(FLASH_1_PHY),
		.length		= FLASH_1_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= MM_SPI_0_VIRT,
		.pfn		= __phys_to_pfn(MM_SPI_0_PHY),
		.length		= MM_SPI_0_SIZE,
		.type		= MT_MEMORY
	},
	{
		.virtual	= MM_SPI_1_VIRT,
		.pfn		= __phys_to_pfn(MM_SPI_1_PHY),
		.length		= MM_SPI_1_SIZE,
		.type		= MT_MEMORY
	},
	{
		.virtual	= INTC_VIRT,
		.pfn		= __phys_to_pfn(INTC_PHY),
		.length		= INTC_SIZE,
		.type		=MT_DEVICE_NONSHARED  
	},
#if defined(CONFIG_MACH_PUMA5_VOLCANO)
	{
		.virtual	= VOLCANO_VIRT,
		.pfn		= __phys_to_pfn(VOLCANO_PHY),
		.length		= VOLCANO_SIZE,
		.type		= MT_DEVICE
	}
#endif
};

static void __init puma5_map_io(void)
{
	iotable_init(puma5_io_desc, ARRAY_SIZE(puma5_io_desc));
    printk("Reserved %dk DSP memory starting from Physical 0x%p\n", 
			(int)(no_linux_mem_size/1024), 
			(void*)(no_linux_mem_start));    
}


#define TIMER16_CNTRL_PRESCALE_ENABLE       0x8000
#define TIMER16_CNTRL_PRESCALE              0x003C
#define TIMER16_CNTRL_MODE                  0x0002

#define TIMER16_MINPRESCALE                 2
#define TIMER16_MAXPRESCALE                 8192
#define TIMER16_PRESCALE_DEFAULT       0x05
#define TIMER16_MIN_LOAD_VALUE              1
#define TIMER16_MAX_LOAD_VALUE              0xFFFF

#define MHZ                                 1000000

/* set min clock divisor to a little higher value
 * so that we are not close to the edge.
 * so multiply by factor 2
 */
#define TIMER16_MAX_CLK_DIVISOR (TIMER16_MAX_LOAD_VALUE * TIMER16_MAXPRESCALE)
#define TIMER16_MIN_CLK_DIVISOR (TIMER16_MIN_LOAD_VALUE * TIMER16_MINPRESCALE * 2)

typedef struct {
                        u32 ctrl_reg;   /* Timer Control Register */
                        u32 load_reg;   /* Timer Load value register */
                        u32 count_reg;  /* Timer count register */
                        u32 intr_reg;   /* Timer Interrupt register */

} puma_timer_regs_t;

typedef enum
{
    TIMER16_MODE_ONESHOT  = 0,
    TIMER16_MODE_AUTOLOAD = 2
} puma_timer_mode;

typedef enum
{
    TIMER16_STATUS_STOP = 0,
    TIMER16_STATUS_START
} puma_timer_status;

extern unsigned int system_rev;

#ifdef CONFIG_HIGH_RES_TIMERS
/****************************************************************************
 * FUNCTION: puma_config_timer
 ****************************************************************************
 * Description: The routine is called to configure the timer mode and
 *              time period (in micro seconds).
 ***************************************************************************/
int puma_config_timer(u32 base_address,u32 refclk_freq,
                        puma_timer_mode mode, u32 usec)
{
    volatile puma_timer_regs_t *p_timer;
    u32 prescale;
    u32 count;
    u32 refclk_mhz = (refclk_freq / MHZ);

    if ((base_address == 0) || (usec == 0))
        return -1;

	if ((mode != TIMER16_CNTRL_ONESHOT) && (mode != TIMER16_CNTRL_AUTOLOAD))
		return -1;

	/* The min time period is 1 usec and since the reference clock freq is always going
		to be more than "min" divider value, minimum value is not checked.
		Check the max time period that can be derived from the timer in micro-seconds
	*/
	if (usec > ((TIMER16_MAX_CLK_DIVISOR) / refclk_mhz)) {
		return -1;      /* input argument speed, out of range */
	}

	p_timer = (puma_timer_regs_t *) (base_address);
	count = refclk_mhz * usec;

	for (prescale = 0; prescale < 12; prescale++) {
		count = count >> 1;
		if (count <= TIMER16_MAX_LOAD_VALUE) {
				break;
		}
	}

	printk("load_value = %d prescale = %d\n", count, prescale);
	/*write the load counter value */
	p_timer->load_reg = count;

	/* write prescalar and mode to control reg */
	p_timer->ctrl_reg =
	mode | TIMER16_CNTRL_PRESCALE_ENABLE | (prescale << 2);

	return 0;
}

static void config_timer2(u32 base_address)
{
   volatile puma_timer_regs_t *p_timer;

   p_timer = (puma_timer_regs_t *) (base_address);

   /*write the load counter value */
   p_timer->load_reg = TIMER16_MAX_LOAD_VALUE;

   /* write prescalar and mode to control reg */
   p_timer->ctrl_reg = TIMER16_CNTRL_AUTOLOAD
           | TIMER16_CNTRL_PRESCALE_ENABLE |(TIMER16_PRESCALE_DEFAULT << 2);
}

/****************************************************************************
 * FUNCTION: puma_timer_ctrl
 ****************************************************************************
 * Description: The routine is called to start/stop the timer
 *
 ***************************************************************************/
void puma_timer_ctrl(u32 base_address, puma_timer_status status)
{
	volatile puma_timer_regs_t *p_timer;

    if (base_address) {
    	p_timer = (puma_timer_regs_t *) (base_address);

        if (status == TIMER16_CTRL_START) {
        	p_timer->ctrl_reg |= TIMER16_CTRL_START;
        } else {
            p_timer->ctrl_reg &= ~(TIMER16_CTRL_START);
        }
    }

}

/****************************************************************************
 * FUNCTION: puma_timer_read
 ****************************************************************************
 * Description: The routine is called to read the current value of timer.
 *
 ***************************************************************************/

static cycle_t timer_read(void)
{
   volatile puma_timer_regs_t *p_timer;
   u32 timer_value;

   p_timer = (puma_timer_regs_t *)(AVALANCHE_TIMER2_BASE);
   timer_value = (TIMER16_MAX_LOAD_VALUE - (p_timer->count_reg &0xffff));

   return (cycles_t)timer_value;
}


/*
 * clocksource
 */
static struct clocksource clocksource_puma = {
        .name           = "timer16bit",
        .rating         = 300,
        .read           = timer_read,
        .mask           = CLOCKSOURCE_MASK(16),
        .shift          = 16,
        .is_continuous  = 1,
};

static void puma_timer_set_next_event(unsigned long cycles,
                                    struct clock_event_device *evt)
{
   volatile puma_timer_regs_t *p_timer;
   p_timer = (puma_timer_regs_t *)(AVALANCHE_TIMER0_BASE);

   /* First stop the timer */
   p_timer->ctrl_reg &= ~(TIMER16_CTRL_START);
   /* Load the value being passed */
        p_timer->load_reg = cycles;

   /* Now start the timer */
   p_timer->ctrl_reg |= TIMER16_CTRL_START;
}

static void puma_timer_set_mode(enum clock_event_mode mode,
                              struct clock_event_device *evt)
{
   volatile puma_timer_regs_t *p_timer;
   p_timer = (puma_timer_regs_t *)(AVALANCHE_TIMER0_BASE);

   switch (mode) {
        case CLOCK_EVT_PERIODIC:
       		/* write mode to control reg */
           	p_timer->ctrl_reg |= TIMER16_CNTRL_AUTOLOAD;
        	break;
        case CLOCK_EVT_ONESHOT:
       		/* write mode to control reg */
           	p_timer->ctrl_reg &= ~(TIMER16_CNTRL_AUTOLOAD);
        	break;
        case CLOCK_EVT_SHUTDOWN:
       		/* stop the timer */
       		p_timer->ctrl_reg &= ~(TIMER16_CTRL_START);
        	break;
   }
}

/*
 * clockevent
 */
static struct clock_event_device clockevent_puma = {
        .name           = "timer16bit",
        .capabilities   = CLOCK_CAP_NEXTEVT | CLOCK_CAP_TICK |
                          CLOCK_CAP_UPDATE,
        .shift          = 32,
        .set_next_event = puma_timer_set_next_event,
        .set_mode       = puma_timer_set_mode,
        .event_handler  = NULL,
};
#endif

#ifdef CONFIG_HIGH_RES_TIMERS
static irqreturn_t puma_timer0_interrupt(int irq, void *dev_id,
                                            struct pt_regs *regs)
{

        clockevent_puma.event_handler(regs);
        return IRQ_HANDLED;
}
#else

extern seqlock_t xtime_lock;

static irqreturn_t
puma_timer0_interrupt(int irq, void *dev_id)
{
	timer_tick();
	return IRQ_HANDLED;
}
#endif

static struct irqaction puma_timer0_irq = {
        .name = "Puma5 Timer Tick",
        .flags          = IRQF_DISABLED | IRQF_TIMER,
        .handler        = puma_timer0_interrupt
};


#ifdef CONFIG_HIGH_RES_TIMERS
/*
 * Set up timer interrupt, and return the current time in seconds.
 */
static void __init puma5_timer_init(void)
{
   int timer_clk;
   int scale;

   /* update PG info */
   system_rev = *(volatile unsigned int*)(AVALANCHE_PG_INFO_REG);
   system_rev &= AVALANCHE_PG_INFO_MASK;
   system_rev >>= AVALANCHE_PG_INFO_SHIFT;
   
   ti_avalanche_setup();

   /* get the input clock frequency */
   timer_clk = PAL_sysClkcGetFreq(CLKC_VBUS);

   /* timer 1 - enable timer and auto load, and go off every 1 ms */
   PAL_sysResetCtrl(AVALANCHE_TIMER0_RESET_BIT, OUT_OF_RESET);
   puma_config_timer(AVALANCHE_TIMER0_BASE, timer_clk,
               TIMER16_CNTRL_AUTOLOAD,
               (int)((1.0 / (float)(HZ)) * 1000000.0));
   puma_timer_ctrl(AVALANCHE_TIMER0_BASE, TIMER16_CTRL_START);
   setup_irq(AVALANCHE_TIMER_0_INT, &puma_timer0_irq);

   PAL_sysResetCtrl(AVALANCHE_TIMER2_RESET_BIT, OUT_OF_RESET);
   config_timer2(AVALANCHE_TIMER2_BASE);
   puma_timer_ctrl(AVALANCHE_TIMER2_BASE, TIMER16_CTRL_START);

   /* Get the scaler value for dividing timer frequency */
   scale = (2*(1<<TIMER16_PRESCALE_DEFAULT));

        /* setup clocksource */
        clocksource_puma.mult = clocksource_hz2mult(timer_clk/scale,
                                     clocksource_puma.shift);
   printk("Mult = %u\n", clocksource_puma.mult);
   if (clocksource_register(&clocksource_puma))
		printk(KERN_ERR "%s: can't register clocksource!\n",
           clocksource_puma.name);

   /* setup clockevent */
   clockevent_puma.mult = div_sc(timer_clk/scale, NSEC_PER_SEC,
                                         clockevent_puma.shift);
   clockevent_puma.max_delta_ns =
            clockevent_delta2ns(0xfffe, &clockevent_puma);
   clockevent_puma.min_delta_ns =
           clockevent_delta2ns(1, &clockevent_puma);

   register_global_clockevent(&clockevent_puma);

   printk("Puma5 Timer0 & Timer2 initialized\n");
}
#else
static void __init puma5_timer_init(void)
{
	int timer_clk;

    /* update PG info */
    system_rev = *(volatile unsigned int*)(AVALANCHE_PG_INFO_REG);
    system_rev &= AVALANCHE_PG_INFO_MASK;
    system_rev >>= AVALANCHE_PG_INFO_SHIFT;
    
    ti_avalanche_setup();

	/* get the input clock frequency */
	timer_clk = PAL_sysClkcGetFreq(CLKC_VBUS);

	/* timer 1 - enable timer and auto load, and go off every 1 ms */
	PAL_sysResetCtrl(AVALANCHE_TIMER0_RESET_BIT, OUT_OF_RESET);
	PAL_sysTimer16SetParams(AVALANCHE_TIMER0_BASE, timer_clk, 
			TIMER16_CNTRL_AUTOLOAD, (int)((1.0/(float)(HZ)) * 1000000.0));
	PAL_sysTimer16Ctrl(AVALANCHE_TIMER0_BASE, TIMER16_CTRL_START);
	setup_irq(AVALANCHE_TIMER_0_INT, &puma_timer0_irq);

	printk("Puma5 Timer0 initialized\n");
}
#endif

static struct sys_timer puma5_timer = {
   .init = puma5_timer_init,
};

/* This API is used to allocate specified size of memory for the 
 * specified module (DSP) from the reserved region. Currently size
 * of reserved region is decided at compile time using Kconfig variable
*/
int avalanche_alloc_no_OperSys_memory(AVALANCHE_NO_OPERSYS_MOD_T mod, 
										unsigned int size, 
										unsigned int *phys_start)
{
    unsigned int cookie;
    int ret = 0;

    if(mod >= eNO_OperSys_END)
        return -1;

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &cookie);

	/* If the memory was already allocated then simply return it */ 
    if(no_OperSys_memory_desc[mod].reserved)
    {
        *phys_start = no_OperSys_memory_desc[mod].phys_start;
        goto topmem_done;
    }
	/* 32-bit align */
    size = ((size + 0x3) & (~0x3));

	/* we do not have that much reserved memory */
    if(size > no_linux_mem_size)
    {
        ret = -1;
        goto topmem_done;
    }

    no_linux_mem_size -= size;

	switch(mod)
	{
		case eNO_OperSys_VDSP:
	        no_OperSys_memory_desc[mod].reserved = 1;
    	    no_OperSys_memory_desc[mod].phys_start = no_linux_mem_start;
        	*phys_start           = no_linux_mem_start;
	        no_linux_mem_start += size;
			break;

#if 0 /* This block might be required for future SOCs */
		case eNO_OperSys_DSLDSP:
	        no_linux_mem_last  -= size;
    	    *phys_start           = no_linux_mem_last;
        	no_OperSys_memory_desc[mod].reserved = 1;
	        no_OperSys_memory_desc[mod].phys_start = no_linux_mem_last;
#endif			

		default:
			ret = -1;
	}

topmem_done:
    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, cookie);

    return ret;
}
EXPORT_SYMBOL(avalanche_alloc_no_OperSys_memory);

/* variable used by vlynq */
unsigned int avalanche_mem_size;
EXPORT_SYMBOL(avalanche_mem_size);

static void __init puma5_fixup(struct machine_desc *desc, struct tag *tag,
		 char **cmdline, struct meminfo *mi)
{
	for (; tag->hdr.size; tag = tag_next(tag)) {
	    if (tag->hdr.tag == ATAG_MEM) {
            unsigned long size = tag->u.mem.size, start =  tag->u.mem.start;               

        	size -= start & ~PAGE_MASK;

			/* This variable is used by DSP memory allocation API */
			no_linux_mem_last = start + size;

            size -= no_linux_mem_size;

        	mi->bank[mi->nr_banks].start = PAGE_ALIGN(start);
        	mi->bank[mi->nr_banks].size  = size & PAGE_MASK;
        	mi->bank[mi->nr_banks].highmem = PHYS_TO_NID(start);

            /* dsp memory starts where normal memory ends */
			no_linux_mem_start = start + size;
            avalanche_mem_size += size;
        	mi->nr_banks += 1;            
            break;    
        }
    }
}

#ifdef CONFIG_AVALANCHE_ADAM2
struct puma5_cmd {
	char cmd[COMMAND_LINE_SIZE];
};

/* set default memory size to 32M */
#ifndef MEM_SIZE
#define MEM_SIZE	(32*1024*1024)
#endif

static struct init_tags {
	struct tag_header hdr1;
	struct tag_core   core;
	struct tag_header hdr2;
	struct tag_mem32  mem;
	struct tag_header hdr3;
	struct puma5_cmd  cmd;
	struct tag_header hdr4;
} puma5_command_line_tag __initdata = {
	{ tag_size(tag_core), ATAG_CORE },
	{ 1, PAGE_SIZE, 0xff },
	{ tag_size(tag_mem32), ATAG_MEM },
	{ MEM_SIZE, PHYS_OFFSET },
	{ tag_size(puma5_cmd), ATAG_CMDLINE },
	{ { 0 } },
	{ 0, ATAG_NONE }
};


void __init volcano_adam2_init(void)
{
	int i, count, end_idx;
	char *cmdline;
	char *cmd_ptr;
	char *end_mark = "-PM5-";

	cmdline = (char*) AVALANCHE_CMDLINE_BASE;
	memcpy(puma5_command_line_tag.cmd.cmd, cmdline, COMMAND_LINE_SIZE);
	cmd_ptr = puma5_command_line_tag.cmd.cmd;
	count = 0;
	end_idx = 0;
	for(i=0; i<COMMAND_LINE_SIZE; i++) {
		if(cmd_ptr[i] == end_mark[end_idx]) {
			end_idx++;
			if (end_idx >= strlen(end_mark))
				break;
		}
		else {
			if(cmd_ptr[i] == 0x0)
				cmd_ptr[i] = ' ';
			count = i;
			end_idx = 0;
		}
	}

	for(i=count; i<COMMAND_LINE_SIZE; i++) {
		cmd_ptr[i] = 0x0;
	}

	/* set machine type - this is typically setup by bootloader */
	/* this value came from arch/arm/tools/mach-types */
	__asm__ volatile ("mov	r1, #0x0400\n\t"
			"add	r1, r1, #0x07b\n\t");
}

#endif /* CONFIG_AVALANCHE_ADAM2 */


MACHINE_START(PUMA5, "puma5")
    /* Maintainer:  Mansoor Ahamed */
    /*.phys_io = IO_START,.io_pg_offst = ((IO_BASE) >> 18) & 0xfffc,*/
#if defined(CONFIG_AVALANCHE_U_BOOT)
    .boot_params = PUMA5_BOOT_PARAM_BASE,
#else
    .boot_params = (unsigned long)&puma5_command_line_tag,
#endif
    .map_io = puma5_map_io,.init_irq = arch_init_irq,.timer =
    &puma5_timer,.fixup = puma5_fixup, MACHINE_END

