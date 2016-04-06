/* -------------------------------------------------------------------------
 * i2c-algo-bit.c i2c driver algorithms for bit-shift adapters
 * -------------------------------------------------------------------------
 *   Copyright (C) 1995-2000 Simon G. Vogl

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ------------------------------------------------------------------------- */

/* With some changes from Frodo Looijaard <frodol@dds.nl>, Kyösti Mälkki
   <kmalkki@cc.hut.fi> and Jean Delvare <khali@linux-fr.org> */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <asm-arm/arch-avalanche/puma6/hw_mutex_ctrl.h>
#include <asm-arm/arch-avalanche/puma6/puma6_gpio_ctrl.h>

/* externs shared with puma6_gpio_ctrl.c */
struct semaphore gpioSem;
EXPORT_SYMBOL(gpioSem);
DEFINE_SPINLOCK(gpioSemSpinlock);
EXPORT_SYMBOL(gpioSemSpinlock);
bool gpioSemaphoreInitialized = false;
EXPORT_SYMBOL(gpioSemaphoreInitialized);

/* gpio functions. Mostly do nothing */
int  gpio_direction_input (unsigned gpio);
int  gpio_direction_output(unsigned gpio, int value);
int  gpio_get_value       (unsigned gpio);
void gpio_set_value       (unsigned gpio, int value);
int  gpio_request         (unsigned gpio, const char *label);
void gpio_free            (unsigned gpio);

/* macros for manipulating the gpio pins directly as calling the Intel functions */
/* in puma6_gpio_ctrl.c directly leads to unacceptably slow performance          */
#define PUMA6_GPIO_REG_GET(reg)                   (*((volatile unsigned int *)(reg)))
#define PUMA6_GPIO_REG_SET(reg, val)              ((*((volatile unsigned int *)(reg))) = (val))

#define __scllo__(adap) \
{ \
    PUMA6_GPIO_REG_SET(0xdffe0490, 0x00100000 ); \
    PUMA6_GPIO_REG_SET(0xdffe0424, PUMA6_GPIO_REG_GET(0xdffe0424) | (0x00100000) ); \
    /*ndelay(NDELAY);*/ \
}

#define __sclhi__(adap) \
{ \
    /* write register twice so that clock line will stay high for at least 500 ns */ \
    PUMA6_GPIO_REG_SET(0xdffe0424, (PUMA6_GPIO_REG_GET(0xdffe0424) & (~0x00100000) ) ); \
    PUMA6_GPIO_REG_SET(0xdffe0424, (PUMA6_GPIO_REG_GET(0xdffe0424) & (~0x00100000) ) ); \
    /*ndelay(NDELAY)*/; \
}

#define __setsda__(adap,bit) \
{ \
    if ((bit)) \
        PUMA6_GPIO_REG_SET(0xdffe0494, 0x00200000 ); \
    else \
        PUMA6_GPIO_REG_SET(0xdffe0490, 0x00200000 ); \
    PUMA6_GPIO_REG_SET(0xdffe0424, PUMA6_GPIO_REG_GET(0xdffe0424) | (0x00200000) ); \
}

#define __setscl__(adap,bit) \
{ \
    if (bit ) \
        PUMA6_GPIO_REG_SET(0xdffe0494, 0x00100000 ); \
    else \
        PUMA6_GPIO_REG_SET(0xdffe0490, 0x00100000 ); \
    PUMA6_GPIO_REG_SET(0xdffe0424, PUMA6_GPIO_REG_GET(0xdffe0424) | (0x00100000) ); \
}

#define __sdahi__(adap) \
{ \
    PUMA6_GPIO_REG_SET(0xdffe0424, (PUMA6_GPIO_REG_GET(0xdffe0424) & (~0x00200000) ) ); \
}

#define __getsda__(adap) \
    ((PUMA6_GPIO_REG_GET(0xdffe0428) & 0x00200000) == 0x00200000)

#define __getscl__(adap) \
    ((PUMA6_GPIO_REG_GET(0xdffe0428) & 0x00100000) == 0x00100000)

#define __sdalo__(adap) __setsda__((adap),0)

/* ----- global defines ----------------------------------------------- */

#ifdef DEBUG
#define bit_dbg(level, dev, format, args...) \
	do { \
		if (i2c_debug >= level) \
			dev_dbg(dev, format, ##args); \
	} while (0)
#else
#define bit_dbg(level, dev, format, args...) \
	do {} while (0)
#endif /* DEBUG */

/* ----- global variables ---------------------------------------------	*/

static int bit_test;	/* see if the line-setting functions work	*/
module_param(bit_test, bool, 0);
MODULE_PARM_DESC(bit_test, "Test the lines of the bus to see if it is stuck");

#ifdef DEBUG
static int i2c_debug = 1;
module_param(i2c_debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(i2c_debug,
		 "debug level - 0 off; 1 normal; 2 verbose; 3 very verbose");
#endif

/* --- other auxiliary functions --------------------------------------	*/
static void i2c_start(struct i2c_algo_bit_data *adap)
{
	/* assert: scl, sda are high */
	__setsda__(adap, 0);
//	ndelay(NDELAY);
	__scllo__(adap);
}

static void i2c_repstart(struct i2c_algo_bit_data *adap)
{
	/* assert: scl is low */
	__sdahi__(adap);
	__sclhi__(adap);
	__setsda__(adap, 0);
//	ndelay(NDELAY);
	__scllo__(adap);
}


static void i2c_stop(struct i2c_algo_bit_data *adap)
{
	/* assert: scl is low */
	__sdalo__(adap);
	__sclhi__(adap);
	__setsda__(adap, 1);
//	ndelay(NDELAY);
}



/* send a byte without start cond., look for arbitration,
   check ackn. from slave */
/* returns:
 * 1 if the device acknowledged
 * 0 if the device did not ack
 * -ETIMEDOUT if an error occurred (while raising the scl line)
 */
static int i2c_outb(struct i2c_adapter *i2c_adap, unsigned char c)
{
	int i;
	int ack;
//	int sb;
//	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	/* assert: scl is low */
	for (i = 7; i >= 0; i--) 
    {
		__setsda__(adap, (c>>i)&1);
        __sclhi__(adap);
		__scllo__(adap);
	}
    /* need to check if this is really necessary */
	__sdahi__(adap);
    __sclhi__(adap);
    /* */

	/* read ack: SDA should be pulled down by slave, or it may
	 * NAK (usually to report problems with the data we wrote).
	 */
	ack = !__getsda__(adap);
	__scllo__(adap);

	return ack;
}


static int i2c_inb(struct i2c_adapter *i2c_adap)
{
	/* read byte via i2c port, without start/stop sequence	*/
	/* acknowledge is sent in i2c_read.			*/
	int i;
	unsigned char indata = 0;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	/* assert: scl is low */
	__sdahi__(adap);
	for (i = 0; i < 8; i++) {
		__sclhi__(adap);
		indata *= 2;
		if (__getsda__(adap))
			indata |= 0x01;
		__setscl__(adap, 0);
		udelay(i == 7 ? adap->udelay / 2 : adap->udelay);
	}
	/* assert: scl is low */
	return indata;
}

/*
 * Sanity check for the adapter hardware - check the reaction of
 * the bus lines only if it seems to be idle.
 */
static int test_bus(struct i2c_adapter *i2c_adap)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	const char *name = i2c_adap->name;
	int scl, sda, ret;

	if (adap->pre_xfer) {
		ret = adap->pre_xfer(i2c_adap);
		if (ret < 0)
			return -ENODEV;
	}

	if (adap->getscl == NULL)
		pr_info("%s: Testing SDA only, SCL is not readable\n", name);

	sda = __getsda__(adap);
	scl = __getscl__(adap);
	if (!scl || !sda) {
		printk(KERN_WARNING "%s: bus seems to be busy\n", name);
		goto bailout;
	}

	__sdalo__(adap);
	sda = __getsda__(adap);
	scl = __getscl__(adap);
	if (sda) {
		printk(KERN_WARNING "%s: SDA stuck high!\n", name);
		goto bailout;
	}
	if (!scl) {
		printk(KERN_WARNING "%s: SCL unexpected low "
		       "while pulling SDA low!\n", name);
		goto bailout;
	}

	__sdahi__(adap);
	sda = __getsda__(adap);
	scl = __getscl__(adap);
	if (!sda) {
		printk(KERN_WARNING "%s: SDA stuck low!\n", name);
		goto bailout;
	}
	if (!scl) {
		printk(KERN_WARNING "%s: SCL unexpected low "
		       "while pulling SDA high!\n", name);
		goto bailout;
	}

	__scllo__(adap);
	sda = __getsda__(adap);
	scl = __getscl__(adap);
	if (scl) {
		printk(KERN_WARNING "%s: SCL stuck high!\n", name);
		goto bailout;
	}
	if (!sda) {
		printk(KERN_WARNING "%s: SDA unexpected low "
		       "while pulling SCL low!\n", name);
		goto bailout;
	}

	__sclhi__(adap);
	sda = __getsda__(adap);
	scl = __getscl__(adap);
	if (!scl) {
		printk(KERN_WARNING "%s: SCL stuck low!\n", name);
		goto bailout;
	}
	if (!sda) {
		printk(KERN_WARNING "%s: SDA unexpected low "
		       "while pulling SCL high!\n", name);
		goto bailout;
	}

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);

	pr_info("%s: Test OK\n", name);
	return 0;
bailout:
	__sdahi__(adap);
	__sclhi__(adap);

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);

	return -ENODEV;
}

/* ----- Utility functions
 */

/* try_address tries to contact a chip for a number of
 * times before it gives up.
 * return values:
 * 1 chip answered
 * 0 chip did not answer
 * -x transmission error
 */
static int try_address(struct i2c_adapter *i2c_adap,
		       unsigned char addr, int retries)
{
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	int i, ret = 0;

	for (i = 0; i <= retries; i++) {
		ret = i2c_outb(i2c_adap, addr);
		if (ret == 1 || i == retries)
			break;
		bit_dbg(3, &i2c_adap->dev, "emitting stop condition\n");
		i2c_stop(adap);
		udelay(adap->udelay);
		yield();
		bit_dbg(3, &i2c_adap->dev, "emitting start condition\n");
		i2c_start(adap);
	}
	if (i && ret)
		bit_dbg(1, &i2c_adap->dev, "Used %d tries to %s client at "
			"0x%02x: %s\n", i + 1,
			addr & 1 ? "read from" : "write to", addr >> 1,
			ret == 1 ? "success" : "failed, timeout?");
	return ret;
}

static int sendbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	const unsigned char *temp = msg->buf;
	int count = msg->len;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	int retval;
	int wrcount = 0;

	while (count > 0) {
		retval = i2c_outb(i2c_adap, *temp);

		/* OK/ACK; or ignored NAK */
		if ((retval > 0) || (nak_ok && (retval == 0))) {
			count--;
			temp++;
			wrcount++;

		/* A slave NAKing the master means the slave didn't like
		 * something about the data it saw.  For example, maybe
		 * the SMBus PEC was wrong.
		 */
		} else if (retval == 0) {
			dev_err(&i2c_adap->dev, "sendbytes: NAK bailout.\n");
			return -EIO;

		/* Timeout; or (someday) lost arbitration
		 *
		 * FIXME Lost ARB implies retrying the transaction from
		 * the first message, after the "winning" master issues
		 * its STOP.  As a rule, upper layer code has no reason
		 * to know or care about this ... it is *NOT* an error.
		 */
		} else {
			dev_err(&i2c_adap->dev, "sendbytes: error %d\n",
					retval);
			return retval;
		}
	}
	return wrcount;
}

static int acknak(struct i2c_adapter *i2c_adap, int is_ack)
{
//	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	/* assert: sda is high */
	if (is_ack)		/* send ack */
		__setsda__(adap, 0);
//	ndelay(NDELAY);
	__sclhi__(adap);
	__scllo__(adap);
	return 0;
}

static int readbytes(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	int inval;
	int rdcount = 0;	/* counts bytes read */
	unsigned char *temp = msg->buf;
	int count = msg->len;
	const unsigned flags = msg->flags;

	while (count > 0) {
		inval = i2c_inb(i2c_adap);
		if (inval >= 0) {
			*temp = inval;
			rdcount++;
		} else {   /* read timed out */
			break;
		}

		temp++;
		count--;

		/* Some SMBus transactions require that we receive the
		   transaction length as the first read byte. */
		if (rdcount == 1 && (flags & I2C_M_RECV_LEN)) {
			if (inval <= 0 || inval > I2C_SMBUS_BLOCK_MAX) {
				if (!(flags & I2C_M_NO_RD_ACK))
					acknak(i2c_adap, 0);
				dev_err(&i2c_adap->dev, "readbytes: invalid "
					"block length (%d)\n", inval);
				return -EREMOTEIO;
			}
			/* The original count value accounts for the extra
			   bytes, that is, either 1 for a regular transaction,
			   or 2 for a PEC transaction. */
			count += inval;
			msg->len += inval;
		}

		bit_dbg(2, &i2c_adap->dev, "readbytes: 0x%02x %s\n",
			inval,
			(flags & I2C_M_NO_RD_ACK)
				? "(no ack/nak)"
				: (count ? "A" : "NA"));

		if (!(flags & I2C_M_NO_RD_ACK)) {
			inval = acknak(i2c_adap, count);
			if (inval < 0)
				return inval;
		}
	}
	return rdcount;
}

/* doAddress initiates the transfer by generating the start condition (in
 * try_address) and transmits the address in the necessary format to handle
 * reads, writes as well as 10bit-addresses.
 * returns:
 *  0 everything went okay, the chip ack'ed, or IGNORE_NAK flag was set
 * -x an error occurred (like: -EREMOTEIO if the device did not answer, or
 *	-ETIMEDOUT, for example if the lines are stuck...)
 */
static int bit_doAddress(struct i2c_adapter *i2c_adap, struct i2c_msg *msg)
{
	unsigned short flags = msg->flags;
	unsigned short nak_ok = msg->flags & I2C_M_IGNORE_NAK;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;

	unsigned char addr;
	int ret, retries;

	retries = nak_ok ? 0 : i2c_adap->retries;

	if (flags & I2C_M_TEN) {
		/* a ten bit address */
		addr = 0xf0 | ((msg->addr >> 7) & 0x03);
		bit_dbg(2, &i2c_adap->dev, "addr0: %d\n", addr);
		/* try extended address code...*/
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)  {
			dev_err(&i2c_adap->dev,
				"died at extended address code\n");
			return -EREMOTEIO;
		}
		/* the remaining 8 bit address */
		ret = i2c_outb(i2c_adap, msg->addr & 0x7f);
		if ((ret != 1) && !nak_ok) {
			/* the chip did not ack / xmission error occurred */
			dev_err(&i2c_adap->dev, "died at 2nd address code\n");
			return -EREMOTEIO;
		}
		if (flags & I2C_M_RD) {
			bit_dbg(3, &i2c_adap->dev, "emitting repeated "
				"start condition\n");
			i2c_repstart(adap);
			/* okay, now switch into reading mode */
			addr |= 0x01;
			ret = try_address(i2c_adap, addr, retries);
			if ((ret != 1) && !nak_ok) {
				dev_err(&i2c_adap->dev,
					"died at repeated address code\n");
				return -EREMOTEIO;
			}
		}
	} else {		/* normal 7bit address	*/
		addr = msg->addr << 1;
		if (flags & I2C_M_RD)
			addr |= 1;
		if (flags & I2C_M_REV_DIR_ADDR)
			addr ^= 1;
		ret = try_address(i2c_adap, addr, retries);
		if ((ret != 1) && !nak_ok)
			return -ENXIO;
	}

	return 0;
}

static int __bit_xfer(struct i2c_adapter *i2c_adap,
		    struct i2c_msg msgs[], int num)
{
	struct i2c_msg *pmsg;
	struct i2c_algo_bit_data *adap = i2c_adap->algo_data;
	int i, ret;
	unsigned short nak_ok;

	if (adap->pre_xfer) {
		ret = adap->pre_xfer(i2c_adap);
		if (ret < 0)
			return ret;
	}

	bit_dbg(3, &i2c_adap->dev, "emitting start condition\n");
	i2c_start(adap);
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;
		if (!(pmsg->flags & I2C_M_NOSTART)) {
			if (i) {
				bit_dbg(3, &i2c_adap->dev, "emitting "
					"repeated start condition\n");
				i2c_repstart(adap);
			}
			ret = bit_doAddress(i2c_adap, pmsg);
			if ((ret != 0) && !nak_ok) {
				bit_dbg(1, &i2c_adap->dev, "NAK from "
					"device addr 0x%02x msg #%d\n",
					msgs[i].addr, i);
				goto bailout;
			}
		}
		if (pmsg->flags & I2C_M_RD) {
			/* read bytes into buffer*/
			ret = readbytes(i2c_adap, pmsg);
			if (ret >= 1)
				bit_dbg(2, &i2c_adap->dev, "read %d byte%s\n",
					ret, ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EREMOTEIO;
				goto bailout;
			}
		} else {
			/* write bytes from buffer */
			ret = sendbytes(i2c_adap, pmsg);
			if (ret >= 1)
				bit_dbg(2, &i2c_adap->dev, "wrote %d byte%s\n",
					ret, ret == 1 ? "" : "s");
			if (ret < pmsg->len) {
				if (ret >= 0)
					ret = -EREMOTEIO;
				goto bailout;
			}
		}
	}
	ret = i;

bailout:
	bit_dbg(3, &i2c_adap->dev, "emitting stop condition\n");
	i2c_stop(adap);

	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);
	return ret;
}

static int bit_xfer(struct i2c_adapter *i2c_adap,
		    struct i2c_msg msgs[], int num)
{
    int ret = -EIO;

	if ( down_interruptible(&gpioSem) )
	{
		return -ERESTARTSYS;
	}

    if ( ! hw_mutex_lock_interruptible( HW_MUTEX_GPIO ) )
    {
        ret = __bit_xfer( i2c_adap, msgs, num );
        hw_mutex_unlock( HW_MUTEX_GPIO );
    }
    up(&gpioSem);

    return ret;
}

static u32 bit_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_SMBUS_READ_BLOCK_DATA |
	       I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
	       I2C_FUNC_10BIT_ADDR | I2C_FUNC_PROTOCOL_MANGLING;
}


/* -----exported algorithm data: -------------------------------------	*/

static const struct i2c_algorithm i2c_bit_algo = {
	.master_xfer	= bit_xfer,
	.functionality	= bit_func,
};

/*
 * registering functions to load algorithms at runtime
 */
static int __i2c_bit_add_bus(struct i2c_adapter *adap,
			     int (*add_adapter)(struct i2c_adapter *))
{
//	struct i2c_algo_bit_data *bit_adap = adap->algo_data;
	int ret;

	if (bit_test) {
		ret = test_bus(adap);
		if (ret < 0)
			return -ENODEV;
	}

	/* register new adapter to i2c module... */
	adap->algo = &i2c_bit_algo;
	adap->retries = 3;

	ret = add_adapter(adap);
	if (ret < 0)
		return ret;

#if 0
	/* Complain if SCL can't be read */
	if (bit_adap->getscl == NULL) {
		dev_warn(&adap->dev, "Not I2C compliant: can't read SCL\n");
		dev_warn(&adap->dev, "Bus may be unreliable\n");
	}
#endif
	return 0;
}

int i2c_bit_add_bus(struct i2c_adapter *adap)
{
	return __i2c_bit_add_bus(adap, i2c_add_adapter);
}
EXPORT_SYMBOL(i2c_bit_add_bus);

int i2c_bit_add_numbered_bus(struct i2c_adapter *adap)
{
	return __i2c_bit_add_bus(adap, i2c_add_numbered_adapter);
}
EXPORT_SYMBOL(i2c_bit_add_numbered_bus);

/*************************************************************************************/

int gpio_direction_input( unsigned gpio )
{
    int ret = 0;

    return ret;
}
EXPORT_SYMBOL(gpio_direction_input);

/*************************************************************************************/

int gpio_direction_output( unsigned gpio, int value )
{
    int ret = 0;

    return ret;
}
EXPORT_SYMBOL(gpio_direction_output);

/*************************************************************************************/

int gpio_get_value( unsigned gpio )
{
    int ret = 0;

    return ret;
}
EXPORT_SYMBOL(gpio_get_value);

/*************************************************************************************/

void gpio_set_value( unsigned gpio, int value )
{
}
EXPORT_SYMBOL(gpio_set_value);

/*************************************************************************************/

int gpio_request( unsigned gpio, const char *label )
{
    int ret = -EIO;
    unsigned long _flags = 0;

    spin_lock_irqsave( &gpioSemSpinlock, _flags );
    if ( gpioSemaphoreInitialized == false )
    {
        sema_init(&gpioSem,1);
        gpioSemaphoreInitialized = true;
    }
    spin_unlock_irqrestore( &gpioSemSpinlock, _flags );

	if ( down_interruptible(&gpioSem) )
	{
		return -ERESTARTSYS;
	}

    if ( ! hw_mutex_lock_interruptible( HW_MUTEX_GPIO ) )
    {
        if ( gpio == 44 ) /*scl*/
        {
            __sclhi__(0);
        }
        else if ( gpio == 45 ) /* sda */
        {
            __sdahi__(0);
        }
        hw_mutex_unlock( HW_MUTEX_GPIO );

        ret = 0;
    }
    up(&gpioSem);

    return ret;
}
EXPORT_SYMBOL(gpio_request);

/*************************************************************************************/

void gpio_free( unsigned gpio )
{
    down(&gpioSem);
    if ( ! hw_mutex_lock_interruptible( HW_MUTEX_GPIO ) )
    {
        if ( gpio == 44 ) /*scl*/
        {
            __sclhi__(0);
        }
        else if ( gpio == 45 ) /* sda */
        {
            __sdahi__(0);
        }
        hw_mutex_unlock( HW_MUTEX_GPIO );
    }
    up(&gpioSem);
}
EXPORT_SYMBOL(gpio_free);

/*************************************************************************************/

MODULE_AUTHOR("Simon G. Vogl <simon@tk.uni-linz.ac.at>");
MODULE_DESCRIPTION("I2C-Bus bit-banging algorithm");
MODULE_LICENSE("GPL");

