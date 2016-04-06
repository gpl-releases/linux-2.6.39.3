/*
 * Bitbanging I2C bus driver using the GPIO API
 *
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

//#include <asm/gpio.h>
#include <asm-arm/arch-avalanche/puma6/puma6_gpio_ctrl.h>

/* functions defined in i2c-bit-algo.c */
extern int  gpio_direction_input (unsigned gpio);
extern int  gpio_direction_output(unsigned gpio, int value);
extern int  gpio_get_value       (unsigned gpio);
extern void gpio_set_value       (unsigned gpio, int value);
extern int  gpio_request         (unsigned gpio, const char *label);
extern void gpio_free            (unsigned gpio);

static struct i2c_gpio_platform_data gpio_state =
{
    .sda_pin = 45, /*0x2d */
    .scl_pin = 44, /*0x2c */
    .udelay  = 1,
};

static struct platform_device *i2c_gpio_device;

/* Toggle SDA by changing the direction of the pin */
static void i2c_gpio_setsda_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->sda_pin);
	else
		gpio_direction_output(pdata->sda_pin, 0);
}

/*
 * Toggle SDA by changing the output value of the pin. This is only
 * valid for pins configured as open drain (i.e. setting the value
 * high effectively turns off the output driver.)
 */
static void i2c_gpio_setsda_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->sda_pin, state);
}

/* Toggle SCL by changing the direction of the pin. */
static void i2c_gpio_setscl_dir(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	if (state)
		gpio_direction_input(pdata->scl_pin);
	else
		gpio_direction_output(pdata->scl_pin, 0);
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void i2c_gpio_setscl_val(void *data, int state)
{
	struct i2c_gpio_platform_data *pdata = data;

	gpio_set_value(pdata->scl_pin, state);
}

static int i2c_gpio_getsda(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->sda_pin);
}

static int i2c_gpio_getscl(void *data)
{
	struct i2c_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->scl_pin);
}

static int __devinit i2c_gpio_probe(struct platform_device *pdev)
{
	struct i2c_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret;

	pdata = pdev->dev.platform_data;
	if (!pdata)
    {
        pdev->dev.platform_data = (void *)&gpio_state;
        pdata = &gpio_state;
        printk( KERN_ERR " no platform_data, using static structure\n" );
    }

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
    {
        printk( KERN_ERR " could not allocate i2c_adapter structure\n" );
		goto err_alloc_adap;
    }

	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
    {
        printk( KERN_ERR " could not allocate i2c_algp_bit_data structure\n" );
		goto err_alloc_bit_data;
    }

	ret = gpio_request(pdata->sda_pin, "sda");
	if (ret)
    {
        printk( KERN_ERR "gpio_request for sda failed\n" );
		goto err_request_sda;
    }

	ret = gpio_request(pdata->scl_pin, "scl");
	if (ret)
    {
        printk( KERN_ERR "gpio_request for scl failed\n" );
		goto err_request_scl;
    }

	if (pdata->sda_is_open_drain) 
    {
		gpio_direction_output(pdata->sda_pin, 1);
		bit_data->setsda = i2c_gpio_setsda_val;
	} 
    else 
    {
		gpio_direction_input(pdata->sda_pin);
		bit_data->setsda = i2c_gpio_setsda_dir;
	}

	if (pdata->scl_is_open_drain || pdata->scl_is_output_only) 
    {
		gpio_direction_output(pdata->scl_pin, 1);
		bit_data->setscl = i2c_gpio_setscl_val;
	} 
    else 
    {
		gpio_direction_input(pdata->scl_pin);
		bit_data->setscl = i2c_gpio_setscl_dir;
	}

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;
	bit_data->getsda = i2c_gpio_getsda;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else if (pdata->scl_is_output_only)
		bit_data->udelay = 50;			/* 10 kHz */
	else
		bit_data->udelay = 5;			/* 100 kHz */

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-gpio%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
    adap->nr = (pdev->id != -1) ? pdev->id : 1; /* for puma6 i2c-0 is used for a 'real' i2c controller */

	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
    {
        printk( KERN_ERR " i2c_bit_add_numbered_bus failed, return=0x%x\n", ret );
		goto err_add_bus;
    }
	platform_set_drvdata(pdev, adap);

	dev_info(&pdev->dev, "using pins %u (SDA) and %u (SCL%s)\n",
		 pdata->sda_pin, pdata->scl_pin,
		 pdata->scl_is_output_only
		 ? ", no clock stretching" : "");

	return 0;

err_add_bus:
	gpio_free(pdata->scl_pin);
err_request_scl:
	gpio_free(pdata->sda_pin);
err_request_sda:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	return ret;
}

static int __devexit i2c_gpio_remove(struct platform_device *pdev)
{
	struct i2c_gpio_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	gpio_free(pdata->scl_pin);
	gpio_free(pdata->sda_pin);
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

#define I2C_GPIO_NAME "i2c-gpio"

static struct platform_driver i2c_gpio_driver = {
	.driver		= {
		.name	= I2C_GPIO_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_gpio_probe,
	.remove		= __devexit_p(i2c_gpio_remove),
};

static int __init i2c_gpio_init(void)
{
	int ret;

    /* Allocate device */
    i2c_gpio_device = platform_device_alloc( I2C_GPIO_NAME, -1 );
    if ( i2c_gpio_device )
    {
        /* initialize device structure */
        i2c_gpio_device->dev.platform_data = (void *)&gpio_state;
        i2c_gpio_device->id                = 1;

        ret = platform_device_add( i2c_gpio_device );
        if ( ret == 0 )
        {
            ret = platform_driver_register( &i2c_gpio_driver );
            if ( ret == 0 )
            {
                /* success */
            }
            else
            {
                printk( KERN_ERR "i2c-gpio: platform_driver_register failed\n" );
                platform_device_del( i2c_gpio_device );
            }
        }
        else
        {
            printk( KERN_ERR "i2c-gpio: platform_device_add failed.\n" );
        }
    }
    else
    {
        printk(KERN_ERR "i2c-gpio: platform_device_alloc failed.\n");
        ret = -ENOMEM;
    }

	return ret;
}


subsys_initcall(i2c_gpio_init);

static void __exit i2c_gpio_exit(void)
{
	platform_driver_unregister(&i2c_gpio_driver);
    platform_device_del( i2c_gpio_device );
}
module_exit(i2c_gpio_exit);

MODULE_AUTHOR("Haavard Skinnemoen <hskinnemoen@atmel.com>");
MODULE_DESCRIPTION("Platform-independent bitbanging I2C driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:i2c-gpio");

