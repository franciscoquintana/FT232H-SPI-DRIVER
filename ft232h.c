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
#include "support.h"

#define FT232H_POLL_PERIOD_MS        10
#define FT232H_GPIO_NUM_PINS         1

#define FT232H_PIN_MODE_OUT          0
#define FT232H_PIN_MODE_IN           1
#define FT232H_PIN_MODE_CS           2

#define FT232H_SPI_MAX_NUM_DEVICES   1
#define FT232H_SPI_BUS_NUM           0
#define FT232H_SPI_MODALIAS          "spidev"
#define FT232H_SPI_MODE              SPI_MODE_0
#define FT232H_SPI_MIN_FREQ          400
#define FT232H_SPI_MAX_FREQ          30000000
#define FT232H_SPI_MIN_BITS_PER_WORD 4
#define FT232H_SPI_MAX_BITS_PER_WORD 32

#define FT232H_OK                    0

#define FT232H_IF_ADDR (&(ft232h_dev->usb_if->dev))

struct ft232h_pin_config {
    uint8_t pin;    // pin number of FT232H chip
    uint8_t mode;   // GPIO mode
    char*   name;   // GPIO name
    bool    value;
};

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

struct ft232h_pin_config ft232h_board_config[FT232H_GPIO_NUM_PINS] = 
{
    // pin  GPIO mode           GPIO name   VALUE
    {   GPIOH0, FT232H_PIN_MODE_OUT , "gpio4"     , 0 }

};

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

    struct mpsse_context     *mpsse_gpio;
    struct gpio_chip         gpio;                              // chip descriptor for GPIOs
    uint8_t                  gpio_num;                          // number of pins used as GPIOs    
    struct task_struct *     gpio_thread;                       // GPIO poll thread
    struct ft232h_pin_config* gpio_pins   [FT232H_GPIO_NUM_PINS]; // pin configurations (gpio_num elements)
    char*                    gpio_names  [FT232H_GPIO_NUM_PINS]; // pin names  (gpio_num elements)
    int                      gpio_irq_map[FT232H_GPIO_NUM_PINS]; // GPIO to IRQ map (gpio_num elements)

    // IRQ device description
    struct irq_chip   irq;                                // chip descriptor for IRQs
    uint8_t           irq_num;                            // number of pins with IRQs
    int               irq_base;                           // base IRQ allocated
    struct irq_desc * irq_descs    [FT232H_GPIO_NUM_PINS]; // IRQ descriptors used (irq_num elements)
    int               irq_types    [FT232H_GPIO_NUM_PINS]; // IRQ types (irq_num elements)
    bool              irq_enabled  [FT232H_GPIO_NUM_PINS]; // IRQ enabled flag (irq_num elements)
    int               irq_gpio_map [FT232H_GPIO_NUM_PINS]; // IRQ to GPIO pin map (irq_num elements)

};

static int ft232h_cfg_probe (struct ft232h_device* ft232h_dev)
{
    struct ft232h_pin_config* cfg;
    int i;

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);

    ft232h_dev->gpio_num    = 0;
    ft232h_dev->gpio_thread = 0;

    ft232h_dev->irq_num     = 0;
    ft232h_dev->irq_base    = 0;

    for (i = 0; i < FT232H_GPIO_NUM_PINS; i++)
    {
        cfg = ft232h_board_config + i;

        // --- read in pin configuration

        if (cfg->mode == FT232H_PIN_MODE_CS)
        {   
        }
        else // FT232H_PIN_MODE_IN || FT232H_PIN_MODE_OUT
        {
            // if pin is not configured as CS signal, set GPIO configuration
            ft232h_dev->gpio_names  [ft232h_dev->gpio_num] = cfg->name;
            ft232h_dev->gpio_pins   [ft232h_dev->gpio_num] = cfg;
            ft232h_dev->gpio_irq_map[ft232h_dev->gpio_num] = -1; // no valid IRQ
         
            // GPIO pins can generate IRQs when set to input mode
            ft232h_dev->gpio_irq_map[ft232h_dev->gpio_num] = ft232h_dev->irq_num;
            ft232h_dev->irq_gpio_map[ft232h_dev->irq_num]  = ft232h_dev->gpio_num;
               
            DEV_INFO (FT232H_IF_ADDR, "%s %s gpio=%d irq=%d", 
                      cfg->mode == FT232H_PIN_MODE_IN ? "input " : "output",
                      cfg->name, ft232h_dev->gpio_num, ft232h_dev->irq_num);

            ft232h_dev->irq_num++;
            ft232h_dev->gpio_num++;
        }
    }

    return FT232H_OK;
}

static uint poll_period = FT232H_POLL_PERIOD_MS;       // module parameter poll period

static struct mutex ft232h_lock;

#define ft232h_spi_maser_to_dev(m) *((struct ft232h_device**)spi_master_get_devdata(m))

/**
* TODO 
**/

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

int contador = 0;
static int FT232H_spi_transfer_one(struct spi_master *master,
        struct spi_device *spi, 
        struct spi_transfer* t)
{
    struct ft232h_device* ft232h_dev = ft232h_spi_maser_to_dev(spi->master);
    const uint8_t* tx;
    uint8_t* rx;
    bool lsb; 
    int result = 0;

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
    if(ft232h_dev->mpsse->clock != speed) {
	//DEV_INFO (FT232H_IF_ADDR, "speed1%d ", ft232h_dev->mpsse->clock );
	SetClock(ft232h_dev->mpsse, speed);
	//DEV_INFO (FT232H_IF_ADDR, "speed%d ", speed);
    }
    /**
    * TODO FIX GPIO RESET ON CHANGE
    **/

    //enum modes mode = (lsb ? LSB : MSB);
    //if(ft232h_dev->mpsse->mode != mode)
    //SetMode(ft232h_dev->mpsse, mode);


    Start(ft232h_dev->mpsse);
    FastTransfer(ft232h_dev->mpsse, tx, rx, t->len);

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


// ----- irq layer begin -------------------------------------------------

void ft232h_irq_enable_disable (struct irq_data *data, bool enable)
{    
    struct ft232h_device *ft232h_dev;
    int irq;
    
    CHECK_PARAM (data && (ft232h_dev = irq_data_get_irq_chip_data(data)));

    // calculate local IRQ
    irq = data->irq - ft232h_dev->irq_base;

    // valid IRQ is in range 0 ... ft232h_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ft232h_dev->irq_num) return;
    
    // enable local IRQ
    ft232h_dev->irq_enabled[irq] = enable;

    DEV_INFO (FT232H_IF_ADDR, "irq=%d enabled=%d", 
              data->irq, ft232h_dev->irq_enabled[irq] ? 1 : 0);
}

void ft232h_irq_enable (struct irq_data *data)
{
    ft232h_irq_enable_disable (data, true);
}

void ft232h_irq_disable (struct irq_data *data)
{
    ft232h_irq_enable_disable (data, false);
}

int ft232h_irq_set_type (struct irq_data *data, unsigned int type)
{
    struct ft232h_device *ft232h_dev;
    int irq;
    
    CHECK_PARAM_RET (data && (ft232h_dev = irq_data_get_irq_chip_data(data)), -EINVAL);
    
    // calculate local IRQ
    irq = data->irq - ft232h_dev->irq_base;

    // valid IRQ is in range 0 ... ft232h_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ft232h_dev->irq_num) return -EINVAL;
    
    ft232h_dev->irq_types[irq] = type;    

    DEV_INFO (FT232H_IF_ADDR, "irq=%d flow_type=%d", data->irq, type);
    
    return FT232H_OK;
}

static int ft232h_irq_check (struct ft232h_device* ft232h_dev, uint8_t irq,
                            uint8_t old, uint8_t new, bool hardware)
{
    int type;

    CHECK_PARAM_RET (old != new, FT232H_OK)
    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    CHECK_PARAM_RET (irq < ft232h_dev->irq_num, -EINVAL);

    // valid IRQ is in range 0 ... ft232h_dev->irq_num-1, invalid IRQ is -1
    if (irq < 0 || irq >= ft232h_dev->irq_num) return -EINVAL;

    // if IRQ is disabled, just return with success
    if (!ft232h_dev->irq_enabled[irq]) return FT232H_OK;
    
    type = ft232h_dev->irq_types[irq];

    // for software IRQs dont check if IRQ is the hardware IRQ for rising edges
    if (!hardware && new > old)
        return FT232H_OK;

    if ((type & IRQ_TYPE_EDGE_FALLING && old > new) ||
        (type & IRQ_TYPE_EDGE_RISING  && new > old))
    {
        // DEV_DBG (FT232H_IF_ADDR, "%s irq=%d %d %s", 
        //          hardware ? "hardware" : "software", 
        //          irq, type, (old > new) ? "falling" : "rising");

        #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,3,0)
		handle_simple_irq (ft232h_dev->irq_descs[irq]);
        #else
		handle_simple_irq (ft232h_dev->irq_base+irq, ft232h_dev->irq_descs[irq]);
        #endif
    }
    
    return FT232H_OK;
}

static int ft232h_irq_probe (struct ft232h_device* ft232h_dev)
{
    int i;
    int result;

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);

    DEV_DBG (FT232H_IF_ADDR, "start");

    ft232h_dev->irq.name         = "ft232h";
    ft232h_dev->irq.irq_enable   = ft232h_irq_enable;
    ft232h_dev->irq.irq_disable  = ft232h_irq_disable;
    ft232h_dev->irq.irq_set_type = ft232h_irq_set_type;

    if (!ft232h_dev->irq_num) return FT232H_OK;

    if ((result = irq_alloc_descs(-1, 0, ft232h_dev->irq_num, 0)) < 0)
    {
        DEV_ERR (FT232H_IF_ADDR, "failed to allocate IRQ descriptors");
        return result;
    }
    
    ft232h_dev->irq_base = result;
    
    DEV_DBG (FT232H_IF_ADDR, "irq_base=%d", ft232h_dev->irq_base);

    for (i = 0; i < ft232h_dev->irq_num; i++)
    {
        ft232h_dev->irq_descs[i]   = irq_to_desc(ft232h_dev->irq_base + i);
        ft232h_dev->irq_enabled[i] = false;
        
        irq_set_chip          (ft232h_dev->irq_base + i, &ft232h_dev->irq);
        irq_set_chip_data     (ft232h_dev->irq_base + i, ft232h_dev);
        irq_clear_status_flags(ft232h_dev->irq_base + i, IRQ_NOREQUEST | IRQ_NOPROBE);
    }
    
    DEV_DBG (FT232H_IF_ADDR, "done");
   
    return FT232H_OK;
}

static void ft232h_irq_remove (struct ft232h_device* ft232h_dev)
{
    CHECK_PARAM (ft232h_dev);

    if (ft232h_dev->irq_base)
        irq_free_descs (ft232h_dev->irq_base, ft232h_dev->irq_num);
        
    return;
}

// ----- irq layer end ---------------------------------------------------

// ----- gpio layer begin ------------------------------------------------

void ft232h_gpio_read_inputs (struct ft232h_device* ft232h_dev)
{
    uint8_t old_value;
    uint8_t new_value;
    uint8_t gpio;
    int i;

    CHECK_PARAM (ft232h_dev);

    // DEV_DBG (FT232H_IF_ADDR, "start");

    for (i = 0; i < ft232h_dev->irq_num; i++)
    {
        // determine local GPIO for each IRQ
        gpio = ft232h_dev->irq_gpio_map[i];
            
        // determin old an new value of the bit
        old_value = ft232h_dev->gpio_pins[gpio]->value;
        //mutex_lock (&ft232h_lock);

        new_value = PinState(ft232h_dev->mpsse_gpio, ft232h_dev->gpio_pins[gpio]->pin, -1);
     
        //mutex_unlock (&ft232h_lock);

        ft232h_dev->gpio_pins[gpio]->value = new_value;
            
        // check for interrupt
        ft232h_irq_check (ft232h_dev, i, old_value, new_value, false);
    }
    
    // DEV_DBG (FT232H_IF_ADDR, "done");
}

// #define FT232H_POLL_WITH_SLEEP

static int ft232h_gpio_poll_function (void* argument)
{
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)argument;
    unsigned int next_poll_ms = jiffies_to_msecs(jiffies);
    unsigned int jiffies_ms;
    int drift_ms = 0;
    int corr_ms  = 0;
    int sleep_ms = 0;

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    
    DEV_DBG (FT232H_IF_ADDR, "start");

    while (!kthread_should_stop())
    {
        // current time in ms
        jiffies_ms = jiffies_to_msecs(jiffies);
        drift_ms   = jiffies_ms - next_poll_ms;
        
        if (poll_period == 0)
        {
            poll_period = FT232H_POLL_PERIOD_MS;
            DEV_ERR (FT232H_IF_ADDR,
                     "Poll period 0 ms is invalid, set back to the default of %d ms",
                     FT232H_POLL_PERIOD_MS);
        }

        if (drift_ms < 0)
        {
            // period was to short, increase corr_ms by 1 ms
            // DEV_DBG (FT232H_IF_ADDR, "polling GPIO is %u ms too early", -drift_ms); 
            corr_ms = (corr_ms > 0) ? corr_ms - 1 : 0;
        }   
        else if (drift_ms > 0 && drift_ms < poll_period)
        {
            // period was to long, decrease corr_ms by 1 ms
            // DEV_DBG (FT232H_IF_ADDR, "polling GPIO is %u ms too late", drift_ms); 
            corr_ms = (corr_ms < poll_period) ? corr_ms + 1 : 0;
        }

        next_poll_ms = jiffies_ms + poll_period;

        // DEV_DBG (FT232H_IF_ADDR, "read FT232H GPIOs");
        ft232h_gpio_read_inputs (ft232h_dev);

        jiffies_ms = jiffies_to_msecs(jiffies);
        
        // if GPIO read took longer than poll period, do not sleep
        if (jiffies_ms > next_poll_ms)
        {
            DEV_ERR (FT232H_IF_ADDR, 
                     "GPIO poll period is too short by at least %u msecs", 
                     jiffies_ms - next_poll_ms);
        }
        else
        {
            sleep_ms = next_poll_ms - jiffies_ms - corr_ms;
            
            #ifdef FT232H_POLL_WITH_SLEEP
            msleep ((sleep_ms <= 0) ? 1 : sleep_ms);
            #else
            set_current_state(TASK_UNINTERRUPTIBLE);
            schedule_timeout(msecs_to_jiffies((sleep_ms <= 0) ? 1 : sleep_ms));
            #endif
        }
    }
    #ifndef FT232H_POLL_WITH_SLEEP
    __set_current_state(TASK_RUNNING);
    #endif
    
    DEV_DBG (FT232H_IF_ADDR, "stop");

    return 0;
}

int ft232h_gpio_get (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)gpiochip_get_data(chip);
    #else
    struct ft232h_device* ft232h_dev = container_of(chip, struct ft232h_device, gpio);
    #endif
    int value;
    
    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ft232h_dev->gpio_num, -EINVAL);

    value = ft232h_dev->gpio_pins[offset]->value;
    
    return value;
}

void ft232h_gpio_set (struct gpio_chip *chip, unsigned offset, int value)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)gpiochip_get_data(chip);
    #else
    struct ft232h_device* ft232h_dev = container_of(chip, struct ft232h_device, gpio);
    #endif
    
    CHECK_PARAM (ft232h_dev);
    CHECK_PARAM (offset < ft232h_dev->gpio_num);

    mutex_lock (&ft232h_lock);
    //DEV_INFO (FT232H_IF_ADDR, "gpio=%d value=%i", offset, value);
    gpio_write(ft232h_dev->mpsse_gpio, ft232h_dev->gpio_pins[offset]->pin, value);
    mutex_unlock (&ft232h_lock);
}

int ft232h_gpio_get_direction (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)gpiochip_get_data(chip);
    #else
    struct ft232h_device* ft232h_dev = container_of(chip, struct ft232h_device, gpio);
    #endif
    int mode;
    
    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ft232h_dev->gpio_num, -EINVAL);

    mode = (ft232h_dev->gpio_pins[offset]->mode == FT232H_PIN_MODE_IN) ? 1 : 0;

    DEV_DBG (FT232H_IF_ADDR, "gpio=%d dir=%d", offset, mode);
 

    return mode;
}

int ft232h_gpio_set_direction (struct gpio_chip *chip, unsigned offset, bool input)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)gpiochip_get_data(chip);
    #else
    struct ft232h_device* ft232h_dev = container_of(chip, struct ft232h_device, gpio);
    #endif

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ft232h_dev->gpio_num, -EINVAL);

    DEV_INFO (FT232H_IF_ADDR, "gpio=%d direction=%s", offset, input ? "input" :  "output");

    ft232h_dev->gpio_pins[offset]->mode = input ? FT232H_PIN_MODE_IN : FT232H_PIN_MODE_OUT;
    
    return FT232H_OK;
}

int ft232h_gpio_direction_input (struct gpio_chip *chip, unsigned offset)
{
    return ft232h_gpio_set_direction (chip, offset, true);
}

int ft232h_gpio_direction_output (struct gpio_chip *chip, unsigned offset, int value)
{
    int result = FT232H_OK;

    if ((result = ft232h_gpio_set_direction (chip, offset, false)) == FT232H_OK)
        // set initial output value
        ft232h_gpio_set (chip, offset, value);
    
    return result;
}

int ft232h_gpio_to_irq (struct gpio_chip *chip, unsigned offset)
{
    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    struct ft232h_device* ft232h_dev = (struct ft232h_device*)gpiochip_get_data(chip);
    #else
    struct ft232h_device* ft232h_dev = container_of(chip, struct ft232h_device, gpio);
    #endif
    int irq;
        
    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    CHECK_PARAM_RET (offset < ft232h_dev->gpio_num, -EINVAL);

    // valid IRQ is in range 0 ... ft232h_dev->irq_num, invalid IRQ is -1
    irq = ft232h_dev->gpio_irq_map[offset];
    irq = (irq >= 0 ? ft232h_dev->irq_base + irq : 0);

    DEV_DBG (FT232H_IF_ADDR, "gpio=%d irq=%d", offset, irq);

    return irq;
}


static int ft232h_gpio_probe (struct ft232h_device* ft232h_dev)
{
    struct gpio_chip *gpio = &ft232h_dev->gpio;
    int result;
    int i, j = 0;

    CHECK_PARAM_RET (ft232h_dev, -EINVAL);
    
    DEV_DBG (FT232H_IF_ADDR, "start");
    
    ft232h_dev->mpsse_gpio = OpenIndex(ft232h_dev->usb_dev, ft232h_dev->usb_if, GPIO, 0, 0, IFACE_A);

    gpio->label     = "ft232h";

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    gpio->parent = &ft232h_dev->usb_dev->dev;
    #else
    gpio->dev    = &ft232h_dev->usb_dev->dev;
    #endif

    gpio->owner  = THIS_MODULE;
    gpio->request= NULL;
    gpio->free   = NULL;
    
    gpio->base   = -1;   // request dynamic ID allocation
    gpio->ngpio  = ft232h_dev->gpio_num;
    
    gpio->can_sleep = 1;
    gpio->names     = (void*)ft232h_dev->gpio_names;

    gpio->get_direction     = ft232h_gpio_get_direction;
    gpio->direction_input   = ft232h_gpio_direction_input;
    gpio->direction_output  = ft232h_gpio_direction_output;
    gpio->get               = ft232h_gpio_get;
    gpio->set               = ft232h_gpio_set;
    gpio->to_irq            = ft232h_gpio_to_irq;

    #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,5,0)
    if ((result = gpiochip_add_data (gpio, ft232h_dev)))
    #else
    if ((result = gpiochip_add (gpio)))
    #endif
    {
        DEV_ERR (FT232H_IF_ADDR, "failed to register GPIOs: %d", result);
        // in case of error, reset gpio->base to avoid crashes during free 
        gpio->base   = -1;
        return result;
    }

    DEV_DBG (FT232H_IF_ADDR, "registered GPIOs from %d to %d", 
             gpio->base, gpio->base + gpio->ngpio - 1);

    for (i = 0; i < FT232H_GPIO_NUM_PINS; i++)
        // in case the pin is not a CS signal, it is an GPIO pin
        if (ft232h_board_config[i].mode != FT232H_PIN_MODE_CS)
        {
            // add and export the GPIO pin
            if ((result = gpio_request(gpio->base + j, ft232h_board_config[i].name)) ||
                (result = gpio_export (gpio->base + j, true)))
            {
                DEV_ERR (FT232H_IF_ADDR, "failed to export GPIO %s: %d", 
                         ft232h_board_config[i].name, result);
                // reduce number of GPIOs to avoid crashes during free in case of error
                ft232h_dev->gpio_num = j ? j-1 : 0;
                return result;
            }
            j++;
        }

    //ft232h_dev->gpio_thread = kthread_run (&ft232h_gpio_poll_function, ft232h_dev, "spi-ft232h-usb-poll");

    DEV_DBG (FT232H_IF_ADDR, "done");

    return 0;
}

static void ft232h_gpio_remove (struct ft232h_device* ft232h_dev)
{
    int i;

    CHECK_PARAM (ft232h_dev);

    if (ft232h_dev->gpio_thread)
    {
        kthread_stop(ft232h_dev->gpio_thread);
        wake_up_process (ft232h_dev->gpio_thread);
    }
        
    if (ft232h_dev->gpio.base > 0)
    {
        for (i = 0; i < ft232h_dev->gpio_num; ++i)
           gpio_free(ft232h_dev->gpio.base + i);

        gpiochip_remove(&ft232h_dev->gpio);
    }

    Close(ft232h_dev->mpsse_gpio);
       
    return;
}

// ----- gpio layer end --------------------------------------------------

// ----- usb layer begin -------------------------------------------------

static const struct usb_device_id ft232h_usb_table[] = {
    { USB_DEVICE(0x403, 0x6014) },
    { }
};

MODULE_DEVICE_TABLE(usb, ft232h_usb_table);

static void ft232h_usb_free_device (struct ft232h_device* ft232h_dev)
{
    CHECK_PARAM (ft232h_dev)
    ft232h_gpio_remove (ft232h_dev);
    ft232h_irq_remove  (ft232h_dev);
    ft232h_spi_remove  (ft232h_dev);

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

    if ((error = ft232h_cfg_probe (ft232h_dev)) ||
        (error = ft232h_spi_probe (ft232h_dev)) ||
        (error = ft232h_irq_probe (ft232h_dev)) ||
        (error = ft232h_gpio_probe (ft232h_dev)))  // initialize SPI master and slaves
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

