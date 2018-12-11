/*
 * Driver for the FT232H USB to SPI and GPIO adapter based on the driver spi-FT232H-usb from Gunar Schorcht
 *
 * Copyright (c) 2017 Gunar Schorcht (gunar@schorcht.net)
 * Copyright (c) 2018 Francisco Quintana
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

// uncomment following line to activate kernel debug handling
//#define DEBUG
#define DEBUG_PRINTK

#ifdef DEBUG_PRINTK
#define PRINTK(fmt,...) printk("%s: "fmt"\n", __func__, ##__VA_ARGS__)
#else
#define PRINTK(fmt,...)
#endif

#define FT232H_IF_ADDR (&(ft232h_dev->usb_if->dev))
#define DEV_ERR(d,f,...)  dev_err (d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_DBG(d,f,...)  dev_dbg (d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_INFO(d,f,...) dev_info(d,"%s: "f"\n", __FUNCTION__, ##__VA_ARGS__)

// check for condition and return with or without err code if it fails
#define CHECK_PARAM_RET(cond,err) if (!(cond)) return err;
#define CHECK_PARAM(cond)         if (!(cond)) return;

#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
#error The driver requires at least kernel version 3.10
#else

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include "mpsse.h"


#define FT232H_SPI_MAX_NUM_DEVICES   1
#define FT232H_SPI_BUS_NUM           0
#define FT232H_SPI_MODALIAS          "spidev"
#define FT232H_SPI_MODE              SPI_MODE_0
#define FT232H_SPI_MIN_FREQ          400
#define FT232H_SPI_MAX_FREQ          1e6
#define FT232H_SPI_MIN_BITS_PER_WORD 4
#define FT232H_SPI_MAX_BITS_PER_WORD 32

#define FT232H_OK                    0

#define FT232H_IF_ADDR (&(ft232h_dev->usb_if->dev))

/**
 *
 *  Change the default values in *FT232H_board_config* for your configuraton
 *
 *  Configurable are:
 *
 *  - Pin 15 (D0/CS0  ) as input/output/CS (FT232H_PIN_MODE_IN/FT232H_PIN_MODE_OUT/FT232H_PIN_MODE_CS)
 *  - Pin 16 (D1/CS1  ) as input/output/CS (FT232H_PIN_MODE_IN/FT232H_PIN_MODE_OUT/FT232H_PIN_MODE_CS)
 *  - Pin 17 (D2/CS2  ) as input/output/CS (FT232H_PIN_MODE_IN/FT232H_PIN_MODE_OUT/FT232H_PIN_MODE_CS)
 *  - Pin 19 (D4/DOUT2) as input/output    (FT232H_PIN_MODE_IN/FT232H_PIN_MODE_OUT)
 *  - Pin 21 (D6/DIN2 ) as input           (FT232H_PIN_MODE_IN)
 *
 *  Pins 18, 20, 22 have fix configuraton and are used as SPI signals.
 */

static struct spi_board_info FT232H_spi_devices[FT232H_SPI_MAX_NUM_DEVICES];

struct spi_board_info FT232H_spi_device_template =
{
    .modalias     = "spidev",
    .max_speed_hz = FT232H_SPI_MAX_FREQ,
    .bus_num      = 0,
    .chip_select  = 0,
    .mode         = SPI_MODE_0,
};

// device specific structure
struct ft232h_device
{
    // USB device description
    struct usb_device*    usb_dev;  // usb device
    struct usb_interface* usb_if;   // usb interface


    // SPI device description
    struct spi_master*  master;   // spi master
    struct spi_device*  slaves[FT232H_SPI_MAX_NUM_DEVICES];
    int                 slave_num;
    struct mpsse_context *mpsse; 

};

// ----- function prototypes ---------------------------------------------

static struct mutex ft232h_lock;

#define ft232h_spi_maser_to_dev(m) *((struct ft232h_device**)spi_master_get_devdata(m))

static int FT232H_usb_transfer (struct ft232h_device *dev, int out_len, int in_len);

static int FT232H_spi_set_cs (struct spi_device *spi, bool active)
{
    /*struct ft232h_device* ft232h_dev;
      int result;

      CHECK_PARAM_RET (spi, -EINVAL);
      CHECK_PARAM_RET (ft232h_dev = FT232H_spi_maser_to_dev(spi->master), -EINVAL);

    // DEV_DBG (FT232H_IF_ADDR, "active %s", active ? "true" : "false");

    if (spi->chip_select > FT232H_SPI_MAX_NUM_DEVICES)
    {
    DEV_ERR (FT232H_IF_ADDR, "invalid CS value %d, 0~%d are available", 
    spi->chip_select, FT232H_SPI_MAX_NUM_DEVICES-1);
    return -EINVAL;
    }

    if (active)
    ft232h_dev->gpio_io_data &= ~cs_bits[spi->chip_select];
    else
    ft232h_dev->gpio_io_data |= cs_bits[spi->chip_select];

    ft232h_dev->out_buf[0]  = FT232H_CMD_UIO_STREAM;
    ft232h_dev->out_buf[1]  = FT232H_CMD_UIO_STM_DIR | ft232h_dev->gpio_mask;
    ft232h_dev->out_buf[2]  = FT232H_CMD_UIO_STM_OUT | (ft232h_dev->gpio_io_data & ft232h_dev->gpio_mask);
    ft232h_dev->out_buf[3]  = FT232H_CMD_UIO_STM_END;

    result = FT232H_usb_transfer(ft232h_dev, 4, 0);

    return (result < 0) ? result : FT232H_OK;*/
    return FT232H_OK;
}

static int FT232H_spi_transfer_one(struct spi_master *master,
        struct spi_device *spi, 
        struct spi_transfer* t)
{
    struct ft232h_device* ft232h_dev = ft232h_spi_maser_to_dev(spi->master);
    const uint8_t* tx;
    uint8_t* rx;
    bool lsb; 
    int result = 0;
    int i;

    CHECK_PARAM_RET (ft232h_dev, EIO);
    CHECK_PARAM_RET (master   , EIO)
        CHECK_PARAM_RET (spi      , EIO)
        CHECK_PARAM_RET (t        , EIO); 

    // DEV_DBG (FT232H_IF_ADDR, "");

    mutex_lock (&ft232h_lock);

    lsb = spi->mode & SPI_LSB_FIRST;
    tx  = t->tx_buf;
    rx  = t->rx_buf;

    u32 speed = spi->max_speed_hz;
    if(t->speed_hz != 0)
        speed = t->speed_hz;

    SetClock(ft232h_dev->mpsse, speed);
    SetMode(ft232h_dev->mpsse, (lsb ? LSB : MSB));

    Start(ft232h_dev->mpsse);
    Write(ft232h_dev->mpsse, tx, t->len);
    rx = Read(ft232h_dev->mpsse, t->len);
    Stop(ft232h_dev->mpsse);


    spi_finalize_current_transfer(master);

    mutex_unlock (&ft232h_lock);

    return result;
}


static int ft232h_spi_probe (struct ft232h_device* ft232h_dev)
{
    int bus = 0;
    int result;
    int i;

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);

    DEV_DBG (FT232H_IF_ADDR, "start");

    // search for next free bus number
    while ((ft232h_dev->master = spi_busnum_to_master(bus)))
    { 
        // returns a refcounted pointer to an existing master
        spi_master_put (ft232h_dev->master);
        bus++;
    }

    // allocate a new SPI master with a pointer to ft232h_device as device data
    ft232h_dev->master = spi_alloc_master(FT232H_IF_ADDR, sizeof(struct ft232h_device*));
    if (!ft232h_dev->master)
    {
        DEV_ERR (FT232H_IF_ADDR, "SPI master allocation failed");
        return -ENOMEM;
    }

    // save the pointer to ft232h_dev in the SPI master device data field
    ft232h_spi_maser_to_dev (ft232h_dev->master) = ft232h_dev;

    DEV_INFO (FT232H_IF_ADDR, "SPI master connected to SPI bus %d", bus);

    // set SPI master configuration
    ft232h_dev->master->bus_num = bus;
    ft232h_dev->master->num_chipselect = FT232H_SPI_MAX_NUM_DEVICES;
    ft232h_dev->master->mode_bits = SPI_MODE_3;
    ft232h_dev->master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
    ft232h_dev->master->bits_per_word_mask = SPI_BIT_MASK(8);
    ft232h_dev->master->transfer_one = FT232H_spi_transfer_one;
    ft232h_dev->master->max_speed_hz = FT232H_SPI_MAX_FREQ;
    ft232h_dev->master->min_speed_hz = FT232H_SPI_MIN_FREQ;

    ft232h_dev->mpsse = OpenIndex(ft232h_dev->usb_dev, ft232h_dev->usb_if, SPI0, 500000, MSB, IFACE_A);

    // register the new master
    if ((result = spi_register_master (ft232h_dev->master)))
    {
        DEV_ERR(FT232H_IF_ADDR, "could not register SPI master");
        spi_master_put(ft232h_dev->master);
        // in case of error, reset the master to avoid crash during free
        ft232h_dev->master = 0;
        return result;
    }
    ft232h_dev->slave_num = FT232H_SPI_MAX_NUM_DEVICES;
    // create SPI slaves
    for (i = 0; i < ft232h_dev->slave_num; i++)
    {
        FT232H_spi_devices[i] = FT232H_spi_device_template;
        FT232H_spi_devices[i].bus_num = bus;
        if ((ft232h_dev->slaves[i] = spi_new_device(ft232h_dev->master, &FT232H_spi_devices[i])))
        {
            DEV_INFO (FT232H_IF_ADDR, "SPI device /dev/spidev%d.%d created", 
                    bus, FT232H_spi_devices[i].chip_select);
            FT232H_spi_set_cs (ft232h_dev->slaves[i], false);
        }
        DEV_ERR(FT232H_IF_ADDR, "register?");
    }

    mutex_init (&ft232h_lock);

    DEV_DBG (FT232H_IF_ADDR, "done");

    return FT232H_OK;
}

static void ft232h_spi_remove (struct ft232h_device* ft232h_dev)
{
    int i;

    CHECK_PARAM (ft232h_dev);

    for (i = 0; i < ft232h_dev->slave_num; i++)
        if (ft232h_dev->slaves[i])
            spi_unregister_device (ft232h_dev->slaves[i]);

    if (ft232h_dev->master)
    {
        spi_unregister_master (ft232h_dev->master);
        spi_master_put (ft232h_dev->master);
    }

    return;
}
// ----- spi layer end ---------------------------------------------------
// ----- usb layer begin -------------------------------------------------

static const struct usb_device_id ft232h_usb_table[] = {
    { USB_DEVICE(0x403, 0x6014) },
    { }
};

MODULE_DEVICE_TABLE(usb, ft232h_usb_table);

static void ft232h_usb_free_device (struct ft232h_device* ft232h_dev)
{
    CHECK_PARAM (ft232h_dev)
        ft232h_spi_remove (ft232h_dev);

    kfree (ft232h_dev);
}

static int ft232h_usb_probe (struct usb_interface* usb_if, const struct usb_device_id* usb_id)
{
    struct usb_device* usb_dev = usb_get_dev(interface_to_usbdev(usb_if));
    struct usb_endpoint_descriptor *epd;
    struct usb_host_interface *settings;
    struct ft232h_device* ft232h_dev;
    int i;
    int error;

    //DEV_DBG (&usb_if->dev, "connect device");

    // create and initialize a new device data structure
    if (!(ft232h_dev = kzalloc(sizeof(struct ft232h_device), GFP_KERNEL)))
    {
        DEV_ERR (&usb_if->dev, "could not allocate device memor");
        usb_put_dev (ft232h_dev->usb_dev);
        return -ENOMEM;
    }
    ft232h_dev->usb_dev = usb_dev;
    ft232h_dev->usb_if = usb_if;
    // save the pointer to the new ft232h_device in USB interface device data
    usb_set_intfdata(usb_if, ft232h_dev);

    if (error = ft232h_spi_probe (ft232h_dev))  // initialize SPI master and slaves
    {
        ft232h_usb_free_device (ft232h_dev);
        return error;
    }

    DEV_INFO (FT232H_IF_ADDR, "connected");

    return FT232H_OK;
}

static void ft232h_usb_disconnect(struct usb_interface *usb_if)
{
    struct ft232h_device* ft232h_dev = usb_get_intfdata(usb_if);

    DEV_INFO (FT232H_IF_ADDR, "disconnected");

    ft232h_usb_free_device (ft232h_dev);
}

static struct usb_driver ft232h_usb_driver = {
    .name       = "spi-ft232h-usb",
    .id_table   = ft232h_usb_table,
    .probe      = ft232h_usb_probe,
    .disconnect = ft232h_usb_disconnect
};

module_usb_driver(ft232h_usb_driver);

// ----- usb layer end ---------------------------------------------------

MODULE_ALIAS("spi:ft232h");
MODULE_AUTHOR("franciscoquintana <francisco.quintana.dosa@gmail.com>");
MODULE_DESCRIPTION("spi-ft232h-usb driver v1.0.0");
MODULE_LICENSE("GPL");

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)

