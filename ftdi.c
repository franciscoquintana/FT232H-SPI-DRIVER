/***************************************************************************
                          ftdi.c  -  description
                             -------------------
    begin                : Fri Apr 4 2003
    copyright            : (C) 2003-2017 by Intra2net AG and the libftdi developers
    email                : opensource@intra2net.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License           *
 *   version 2.1 as published by the free Software Foundation;             *
 *                                                                         *
 ***************************************************************************/

/**
    \mainpage libftdi API documentation

    Library to talk to FTDI chips. You find the latest versions of libftdi at
    https://www.intra2net.com/en/developer/libftdi/

    The library is easy to use. Have a look at this short example:
    \include simple.c

    More examples can be found in the "examples" directory.
*/
/** \addtogroup libftdi */
/* @{ */

#include "ftdi_i.h"
#include "ftdi.h"
#include "ftdi_version_i.h"

#define ftdi_error_return(code, str) do {  \
printk(KERN_ALERT "%s", str);\
        return code;                       \
   } while(0);

/**
    Initializes a ftdi_context.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: couldn't allocate read buffer
    \retval -2: couldn't allocate struct  buffer

    \remark This should be called before all functions
*/
int ftdi_init(struct ftdi_context *ftdi)
{
    struct ftdi_eeprom* eeprom;
    ftdi->usb_dev = NULL;
    ftdi->usb_if = NULL;
    ftdi->usb_read_timeout = 5000;
    ftdi->usb_write_timeout = 5000;

    ftdi->type = TYPE_BM;    /* chip type */
    ftdi->baudrate = -1;
    ftdi->bitbang_enabled = 0;  /* 0: normal mode 1: any of the bitbang modes enabled */

    ftdi->readbuffer = NULL;
    ftdi->readbuffer_offset = 0;
    ftdi->readbuffer_remaining = 0;
    ftdi->writebuffer_chunksize = 4096;
    ftdi->max_packet_size = 0;
    ftdi->error_str = NULL;

    ftdi_set_interface(ftdi, INTERFACE_ANY);
    ftdi->bitbang_mode = 1; /* when bitbang is enabled this holds the number of the mode  */

    eeprom = (struct ftdi_eeprom *)kmalloc(sizeof(struct ftdi_eeprom), GFP_KERNEL);
    if (eeprom == 0)
        ftdi_error_return(-2, "Can't kmalloc struct ftdi_eeprom");
    memset(eeprom, 0, sizeof(struct ftdi_eeprom));
    ftdi->eeprom = eeprom;

    /* All fine. Now allocate the readbuffer */
    return ftdi_read_data_set_chunksize(ftdi, 4096);
}

/**
    Allocate and initialize a new ftdi_context

    \return a pointer to a new ftdi_context, or NULL on failure
*/
struct ftdi_context *ftdi_new(void)
{
    struct ftdi_context * ftdi = (struct ftdi_context *)kmalloc(sizeof(struct ftdi_context), GFP_KERNEL);

    if (ftdi == NULL)
    {
        return NULL;
    }

    if (ftdi_init(ftdi) != 0)
    {
        kfree(ftdi);
        return NULL;
    }

    return ftdi;
}

/**
    Open selected channels on a chip, otherwise use first channel.

    \param ftdi pointer to ftdi_context
    \param interface Interface to use for FT2232C/2232H/4232H chips.

    \retval  0: all fine
    \retval -1: unknown interface
    \retval -2: USB device unavailable
    \retval -3: Device already open, interface can't be set in that state
*/
int ftdi_set_interface(struct ftdi_context *ftdi, enum ftdi_interface interface)
{
    if (ftdi == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (ftdi->usb_dev != NULL)
    {
        int check_interface = interface;
        if (check_interface == INTERFACE_ANY)
            check_interface = INTERFACE_A;

        if (ftdi->index != check_interface)
            ftdi_error_return(-3, "Interface can not be changed on an already open device");
    }

    switch (interface)
    {
        case INTERFACE_ANY:
        case INTERFACE_A:
            ftdi->index     = INTERFACE_A;
            ftdi->in_ep     = 0x02;
            ftdi->out_ep    = 0x81;
            break;
        case INTERFACE_B:
            ftdi->index     = INTERFACE_B;
            ftdi->in_ep     = 0x04;
            ftdi->out_ep    = 0x83;
            break;
        case INTERFACE_C:
            ftdi->index     = INTERFACE_C;
            ftdi->in_ep     = 0x06;
            ftdi->out_ep    = 0x85;
            break;
        case INTERFACE_D:;
            ftdi->index     = INTERFACE_D;
            ftdi->in_ep     = 0x08;
            ftdi->out_ep    = 0x87;
            break;
        default:
            ftdi_error_return(-1, "Unknown interface");
    }
    return 0;
}

/**
    Deinitializes a ftdi_context.

    \param ftdi pointer to ftdi_context
*/
void ftdi_deinit(struct ftdi_context *ftdi)
{
    if (ftdi == NULL)
        return;

    if (ftdi->readbuffer != NULL)
    {
        kfree(ftdi->readbuffer);
        ftdi->readbuffer = NULL;
    }

    if (ftdi->eeprom != NULL)
    {
        if (ftdi->eeprom->manufacturer != 0)
        {
            kfree(ftdi->eeprom->manufacturer);
            ftdi->eeprom->manufacturer = 0;
        }
        if (ftdi->eeprom->product != 0)
        {
            kfree(ftdi->eeprom->product);
            ftdi->eeprom->product = 0;
        }
        if (ftdi->eeprom->serial != 0)
        {
            kfree(ftdi->eeprom->serial);
            ftdi->eeprom->serial = 0;
        }
        kfree(ftdi->eeprom);
        ftdi->eeprom = NULL;
    }
}

/**
    Deinitialize and kfree an ftdi_context.

    \param ftdi pointer to ftdi_context
*/
void ftdi_kfree(struct ftdi_context *ftdi)
{
    ftdi_deinit(ftdi);
    kfree(ftdi);
}

/**
 * @brief Get libftdi library version
 *
 * @return ftdi_version_info Library version information
 **/
struct ftdi_version_info ftdi_get_library_version(void)
{
    struct ftdi_version_info ver;

    ver.major = FTDI_MAJOR_VERSION;
    ver.minor = FTDI_MINOR_VERSION;
    ver.micro = FTDI_MICRO_VERSION;
    ver.version_str = FTDI_VERSION_STRING;
    ver.snapshot_str = FTDI_SNAPSHOT_VERSION;

    return ver;
}

/**
 * Internal function to determine the maximum packet size.
 * \param ftdi pointer to ftdi_context
 * \param dev usb_if to use
 * \retval Maximum packet size for this device
 */
static unsigned int _ftdi_determine_max_packet_size(struct ftdi_context *ftdi, struct usb_interface *usb_if)
{
    struct usb_interface_descriptor *config0;
    unsigned int packet_size;

    // Sanity check
    if (ftdi == NULL || usb_if == NULL)
        return 64;

    // Determine maximum packet size. Init with default value.
    // New hi-speed devices from FTDI use a packet size of 512 bytes
    // but could be connected to a normal speed USB hub -> 64 bytes packet size.
    if (ftdi->type == TYPE_2232H || ftdi->type == TYPE_4232H || ftdi->type == TYPE_232H)
        packet_size = 512;
    else
        packet_size = 64;
    

                struct usb_host_interface *settings= usb_if->cur_altsetting;
                if (settings->desc.bNumEndpoints > 0)
                {
                    packet_size = settings->endpoint[0].desc.wMaxPacketSize;
                }

    return packet_size;
}

/**
    Opens a ftdi device given by an usb_device.

    \param ftdi pointer to ftdi_context
    \param dev libusb usb_dev to use

    \retval  0: all fine
    \retval -3: unable to config device
    \retval -4: unable to open device
    \retval -5: unable to claim device
    \retval -6: reset failed
    \retval -7: set baudrate failed
    \retval -8: ftdi context invalid
*/
int ftdi_usb_open_dev(struct ftdi_context *ftdi, struct usb_device* usb_dev, struct usb_interface* usb_if)
{
    struct usb_device_descriptor desc;
    int cfg, cfg0, detach_errno = 0;

    if (ftdi == NULL)
        ftdi_error_return(-8, "ftdi context invalid");

    ftdi->usb_dev = usb_dev;
    ftdi->usb_if = usb_if;

    if (ftdi_usb_reset (ftdi) != 0)
    {
        ftdi_error_return(-6, "ftdi_usb_reset failed");
    }
    desc = usb_dev->descriptor;
    //printk(KERN_ALERT "bcdDevice = 0x%x", desc.bcdDevice);
    // Try to guess chip type
    // Bug in the BM type chips: bcdDevice is 0x200 for serial == 0
    if (desc.bcdDevice == 0x400 || (desc.bcdDevice == 0x200
                                    && desc.iSerialNumber == 0))
        ftdi->type = TYPE_BM;
    else if (desc.bcdDevice == 0x200)
        ftdi->type = TYPE_AM;
    else if (desc.bcdDevice == 0x500)
        ftdi->type = TYPE_2232C;
    else if (desc.bcdDevice == 0x600)
        ftdi->type = TYPE_R;
    else if (desc.bcdDevice == 0x700)
        ftdi->type = TYPE_2232H;
    else if (desc.bcdDevice == 0x800)
        ftdi->type = TYPE_4232H;
    else if (desc.bcdDevice == 0x900)
        ftdi->type = TYPE_232H;
    else if (desc.bcdDevice == 0x1000)
        ftdi->type = TYPE_230X;

    // Determine maximum packet size
    ftdi->max_packet_size = _ftdi_determine_max_packet_size(ftdi, usb_if);

    if (ftdi_set_baudrate (ftdi, 9600) != 0)
    {
        ftdi_error_return(-7, "set baudrate failed");
    }

    ftdi_error_return(0, "all fine");
}

/**
    Resets the ftdi device.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: FTDI reset failed
    \retval -2: USB device unavailable
*/
int ftdi_usb_reset(struct ftdi_context *ftdi)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0),
                                SIO_RESET_REQUEST, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_SIO,
                                ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1,"FTDI reset failed");

    // Invalidate data in the readbuffer
    ftdi->readbuffer_offset = 0;
    ftdi->readbuffer_remaining = 0;

    return 0;
}

/**
    Clears the read buffer on the chip and the internal read buffer.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: read buffer purge failed
    \retval -2: USB device unavailable
*/
int ftdi_usb_purge_rx_buffer(struct ftdi_context *ftdi)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_RESET_REQUEST, FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_PURGE_RX,
                                ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "FTDI purge of RX buffer failed");

    // Invalidate data in the readbuffer
    ftdi->readbuffer_offset = 0;
    ftdi->readbuffer_remaining = 0;

    return 0;
}

/**
    Clears the write buffer on the chip.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: write buffer purge failed
    \retval -2: USB device unavailable
*/
int ftdi_usb_purge_tx_buffer(struct ftdi_context *ftdi)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_RESET_REQUEST, FTDI_DEVICE_OUT_REQTYPE
                                , SIO_RESET_PURGE_TX,
                                ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "FTDI purge of TX buffer failed");

    return 0;
}

/**
    Clears the buffers on the chip and the internal read buffer.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: read buffer purge failed
    \retval -2: write buffer purge failed
    \retval -3: USB device unavailable
*/
int ftdi_usb_purge_buffers(struct ftdi_context *ftdi)
{
    int result;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-3, "USB device unavailable");

    result = ftdi_usb_purge_rx_buffer(ftdi);
    if (result < 0)
        return -1;

    result = ftdi_usb_purge_tx_buffer(ftdi);
    if (result < 0)
        return -2;

    return 0;
}



/**
    Closes the ftdi device. Call ftdi_deinit() if you're cleaning up.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -3: ftdi context invalid
*/
int ftdi_usb_close(struct ftdi_context *ftdi)
{
    int rtn = 0;

    if (ftdi == NULL)
        ftdi_error_return(-3, "ftdi context invalid");


    return rtn;
}

/*  ftdi_to_clkbits_AM For the AM device, convert a requested baudrate
                    to encoded divisor and the achievable baudrate
    Function is only used internally
    \internal

    See AN120
   clk/1   -> 0
   clk/1.5 -> 1
   clk/2   -> 2
   From /2, 0.125/ 0.25 and 0.5 steps may be taken
   The fractional part has frac_code encoding
*/
static int ftdi_to_clkbits_AM(int baudrate, unsigned long *encoded_divisor)

{
    static const char frac_code[8] = {0, 3, 2, 4, 1, 5, 6, 7};
    static const char am_adjust_up[8] = {0, 0, 0, 1, 0, 3, 2, 1};
    static const char am_adjust_dn[8] = {0, 0, 0, 1, 0, 1, 2, 3};
    int divisor, best_divisor, best_baud, best_baud_diff;
    int i;
    divisor = 24000000 / baudrate;

    // Round down to supported fraction (AM only)
    divisor -= am_adjust_dn[divisor & 7];

    // Try this divisor and the one above it (because division rounds down)
    best_divisor = 0;
    best_baud = 0;
    best_baud_diff = 0;
    for (i = 0; i < 2; i++)
    {
        int try_divisor = divisor + i;
        int baud_estimate;
        int baud_diff;

        // Round up to supported divisor value
        if (try_divisor <= 8)
        {
            // Round up to minimum supported divisor
            try_divisor = 8;
        }
        else if (divisor < 16)
        {
            // AM doesn't support divisors 9 through 15 inclusive
            try_divisor = 16;
        }
        else
        {
            // Round up to supported fraction (AM only)
            try_divisor += am_adjust_up[try_divisor & 7];
            if (try_divisor > 0x1FFF8)
            {
                // Round down to maximum supported divisor value (for AM)
                try_divisor = 0x1FFF8;
            }
        }
        // Get estimated baud rate (to nearest integer)
        baud_estimate = (24000000 + (try_divisor / 2)) / try_divisor;
        // Get absolute difference from requested baud rate
        if (baud_estimate < baudrate)
        {
            baud_diff = baudrate - baud_estimate;
        }
        else
        {
            baud_diff = baud_estimate - baudrate;
        }
        if (i == 0 || baud_diff < best_baud_diff)
        {
            // Closest to requested baud rate so far
            best_divisor = try_divisor;
            best_baud = baud_estimate;
            best_baud_diff = baud_diff;
            if (baud_diff == 0)
            {
                // Spot on! No point trying
                break;
            }
        }
    }
    // Encode the best divisor value
    *encoded_divisor = (best_divisor >> 3) | (frac_code[best_divisor & 7] << 14);
    // Deal with special cases for encoded value
    if (*encoded_divisor == 1)
    {
        *encoded_divisor = 0;    // 3000000 baud
    }
    else if (*encoded_divisor == 0x4001)
    {
        *encoded_divisor = 1;    // 2000000 baud (BM only)
    }
    return best_baud;
}

/*  ftdi_to_clkbits Convert a requested baudrate for a given system clock  and predivisor
                    to encoded divisor and the achievable baudrate
    Function is only used internally
    \internal

    See AN120
   clk/1   -> 0
   clk/1.5 -> 1
   clk/2   -> 2
   From /2, 0.125 steps may be taken.
   The fractional part has frac_code encoding

   value[13:0] of value is the divisor
   index[9] mean 12 MHz Base(120 MHz/10) rate versus 3 MHz (48 MHz/16) else

   H Type have all features above with
   {index[8],value[15:14]} is the encoded subdivisor

   FT232R, FT2232 and FT232BM have no option for 12 MHz and with
   {index[0],value[15:14]} is the encoded subdivisor

   AM Type chips have only four fractional subdivisors at value[15:14]
   for subdivisors 0, 0.5, 0.25, 0.125
*/
static int ftdi_to_clkbits(int baudrate, unsigned int clk, int clk_div, unsigned long *encoded_divisor)
{
    static const char frac_code[8] = {0, 3, 2, 4, 1, 5, 6, 7};
    int best_baud = 0;
    int divisor, best_divisor;
    if (baudrate >=  clk/clk_div)
    {
        *encoded_divisor = 0;
        best_baud = clk/clk_div;
    }
    else if (baudrate >=  clk/(clk_div + clk_div/2))
    {
        *encoded_divisor = 1;
        best_baud = clk/(clk_div + clk_div/2);
    }
    else if (baudrate >=  clk/(2*clk_div))
    {
        *encoded_divisor = 2;
        best_baud = clk/(2*clk_div);
    }
    else
    {
        /* We divide by 16 to have 3 fractional bits and one bit for rounding */
        divisor = clk*16/clk_div / baudrate;
        if (divisor & 1) /* Decide if to round up or down*/
            best_divisor = divisor /2 +1;
        else
            best_divisor = divisor/2;
        if(best_divisor > 0x20000)
            best_divisor = 0x1ffff;
        best_baud = clk*16/clk_div/best_divisor;
        if (best_baud & 1) /* Decide if to round up or down*/
            best_baud = best_baud /2 +1;
        else
            best_baud = best_baud /2;
        *encoded_divisor = (best_divisor >> 3) | (frac_code[best_divisor & 0x7] << 14);
    }
    return best_baud;
}
/**
    ftdi_convert_baudrate returns nearest supported baud rate to that requested.
    Function is only used internally
    \internal
*/
static int ftdi_convert_baudrate(int baudrate, struct ftdi_context *ftdi,
                                 unsigned short *value, unsigned short *index)
{
    int best_baud;
    unsigned long encoded_divisor;

    if (baudrate <= 0)
    {
        // Return error
        return -1;
    }

#define H_CLK 120000000
#define C_CLK  48000000
    if ((ftdi->type == TYPE_2232H) || (ftdi->type == TYPE_4232H) || (ftdi->type == TYPE_232H))
    {
        if(baudrate*10 > H_CLK /0x3fff)
        {
            /* On H Devices, use 12 000 000 Baudrate when possible
               We have a 14 bit divisor, a 1 bit divisor switch (10 or 16)
               three fractional bits and a 120 MHz clock
               Assume AN_120 "Sub-integer divisors between 0 and 2 are not allowed" holds for
               DIV/10 CLK too, so /1, /1.5 and /2 can be handled the same*/
            best_baud = ftdi_to_clkbits(baudrate, H_CLK, 10, &encoded_divisor);
            encoded_divisor |= 0x20000; /* switch on CLK/10*/
        }
        else
            best_baud = ftdi_to_clkbits(baudrate, C_CLK, 16, &encoded_divisor);
    }
    else if ((ftdi->type == TYPE_BM) || (ftdi->type == TYPE_2232C) || (ftdi->type == TYPE_R) || (ftdi->type == TYPE_230X))
    {
        best_baud = ftdi_to_clkbits(baudrate, C_CLK, 16, &encoded_divisor);
    }
    else
    {
        best_baud = ftdi_to_clkbits_AM(baudrate, &encoded_divisor);
    }
    // Split into "value" and "index" values
    *value = (unsigned short)(encoded_divisor & 0xFFFF);
    if (ftdi->type == TYPE_2232H || ftdi->type == TYPE_4232H || ftdi->type == TYPE_232H)
    {
        *index = (unsigned short)(encoded_divisor >> 8);
        *index &= 0xFF00;
        *index |= ftdi->index;
    }
    else
        *index = (unsigned short)(encoded_divisor >> 16);

    // Return the nearest baud rate
    return best_baud;
}

/**
 * @brief Wrapper function to export ftdi_convert_baudrate() to the unit test
 * Do not use, it's only for the unit test framework
 **/
int convert_baudrate_UT_export(int baudrate, struct ftdi_context *ftdi,
                               unsigned short *value, unsigned short *index)
{
    return ftdi_convert_baudrate(baudrate, ftdi, value, index);
}

/**
    Sets the chip baud rate

    \param ftdi pointer to ftdi_context
    \param baudrate baud rate to set

    \retval  0: all fine
    \retval -1: invalid baudrate
    \retval -2: setting baudrate failed
    \retval -3: USB device unavailable
*/
int ftdi_set_baudrate(struct ftdi_context *ftdi, int baudrate)
{
    unsigned short value, index;
    int actual_baudrate;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-3, "USB device unavailable");

    if (ftdi->bitbang_enabled)
    {
        baudrate = baudrate*4;
    }

    actual_baudrate = ftdi_convert_baudrate(baudrate, ftdi, &value, &index);
    if (actual_baudrate <= 0)
        ftdi_error_return (-1, "Silly baudrate <= 0.");

    // Check within tolerance (about 5%)
    if ((actual_baudrate * 2 < baudrate /* Catch overflows */ )
            || ((actual_baudrate < baudrate)
                ? (actual_baudrate * 21 < baudrate * 20)
                : (baudrate * 21 < actual_baudrate * 20)))
        ftdi_error_return (-1, "Unsupported baudrate. Note: bitbang baudrates are automatically multiplied by 4");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_BAUDRATE_REQUEST, FTDI_DEVICE_OUT_REQTYPE, value,
                                index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return (-2, "Setting new baudrate failed");

    ftdi->baudrate = baudrate;
    return 0;
}

/**
    Set (RS232) line characteristics.
    The break type can only be set via ftdi_set_line_property2()
    and defaults to "off".

    \param ftdi pointer to ftdi_context
    \param bits Number of bits
    \param sbit Number of stop bits
    \param parity Parity mode

    \retval  0: all fine
    \retval -1: Setting line property failed
*/
int ftdi_set_line_property(struct ftdi_context *ftdi, enum ftdi_bits_type bits,
                           enum ftdi_stopbits_type sbit, enum ftdi_parity_type parity)
{
    return ftdi_set_line_property2(ftdi, bits, sbit, parity, BREAK_OFF);
}

/**
    Set (RS232) line characteristics

    \param ftdi pointer to ftdi_context
    \param bits Number of bits
    \param sbit Number of stop bits
    \param parity Parity mode
    \param break_type Break type

    \retval  0: all fine
    \retval -1: Setting line property failed
    \retval -2: USB device unavailable
*/
int ftdi_set_line_property2(struct ftdi_context *ftdi, enum ftdi_bits_type bits,
                            enum ftdi_stopbits_type sbit, enum ftdi_parity_type parity,
                            enum ftdi_break_type break_type)
{
    unsigned short value = bits;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    switch (parity)
    {
        case NONE:
            value |= (0x00 << 8);
            break;
        case ODD:
            value |= (0x01 << 8);
            break;
        case EVEN:
            value |= (0x02 << 8);
            break;
        case MARK:
            value |= (0x03 << 8);
            break;
        case SPACE:
            value |= (0x04 << 8);
            break;
    }

    switch (sbit)
    {
        case STOP_BIT_1:
            value |= (0x00 << 11);
            break;
        case STOP_BIT_15:
            value |= (0x01 << 11);
            break;
        case STOP_BIT_2:
            value |= (0x02 << 11);
            break;
    }

    switch (break_type)
    {
        case BREAK_OFF:
            value |= (0x00 << 14);
            break;
        case BREAK_ON:
            value |= (0x01 << 14);
            break;
    }

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0),
                                SIO_SET_DATA_REQUEST, FTDI_DEVICE_OUT_REQTYPE, value,
                                ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return (-1, "Setting new line property failed");

    return 0;
}

/**
    Writes data in chunks (see ftdi_write_data_set_chunksize()) to the chip

    \param ftdi pointer to ftdi_context
    \param buf Buffer with the data
    \param size Size of the buffer

    \retval -666: USB device unavailable
    \retval <0: error code from usb_bulk_write()
    \retval >0: number of bytes written
*/
int ftdi_write_data(struct ftdi_context *ftdi, const unsigned char *buf, int size)
{
    int offset = 0;
    int actual_length;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-666, "USB device unavailable");

    while (offset < size)
    {
        int write_size = ftdi->writebuffer_chunksize;

        if (offset+write_size > size)
            write_size = size-offset;
        if (usb_bulk_msg(ftdi->usb_dev, usb_sndbulkpipe(ftdi->usb_dev, ftdi->in_ep), (unsigned char *)buf+offset, write_size, &actual_length, ftdi->usb_write_timeout) < 0)
            ftdi_error_return(-1, "usb bulk write failed");

        offset += actual_length;
    }

    return offset;
}

/**
    Configure write buffer chunk size.
    Default is 4096.

    \param ftdi pointer to ftdi_context
    \param chunksize Chunk size

    \retval 0: all fine
    \retval -1: ftdi context invalid
*/
int ftdi_write_data_set_chunksize(struct ftdi_context *ftdi, unsigned int chunksize)
{
    if (ftdi == NULL)
        ftdi_error_return(-1, "ftdi context invalid");

    ftdi->writebuffer_chunksize = chunksize;
    return 0;
}

/**
    Get write buffer chunk size.

    \param ftdi pointer to ftdi_context
    \param chunksize Pointer to store chunk size in

    \retval 0: all fine
    \retval -1: ftdi context invalid
*/
int ftdi_write_data_get_chunksize(struct ftdi_context *ftdi, unsigned int *chunksize)
{
    if (ftdi == NULL)
        ftdi_error_return(-1, "ftdi context invalid");

    *chunksize = ftdi->writebuffer_chunksize;
    return 0;
}

/**
    Reads data in chunks (see ftdi_read_data_set_chunksize()) from the chip.

    Automatically strips the two modem status bytes transfered during every read.

    \param ftdi pointer to ftdi_context
    \param buf Buffer to store data in
    \param size Size of the buffer

    \retval -666: USB device unavailable

    \retval  0: no data was available
    \retval >0: number of bytes read

*/
int ftdi_read_data(struct ftdi_context *ftdi, unsigned char *buf, int size)
{
    int offset = 0, ret, i, num_of_chunks, chunk_remains;
    int packet_size;
    int actual_length = 1;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-666, "USB device unavailable");

    // Packet size sanity check (avoid division by zero)
    packet_size = ftdi->max_packet_size;
    if (packet_size == 0)
        ftdi_error_return(-1, "max_packet_size is bogus (zero)");

    // everything we want is still in the readbuffer?
    if (size <= (int)ftdi->readbuffer_remaining)
    {
        memcpy (buf, ftdi->readbuffer+ftdi->readbuffer_offset, size);

        // Fix offsets
        ftdi->readbuffer_remaining -= size;
        ftdi->readbuffer_offset += size;

        /* printf("Returning bytes from buffer: %d - remaining: %d\n", size, ftdi->readbuffer_remaining); */

        return size;
    }
    // something still in the readbuffer, but not enough to satisfy 'size'?
    if (ftdi->readbuffer_remaining != 0)
    {
        memcpy (buf, ftdi->readbuffer+ftdi->readbuffer_offset, ftdi->readbuffer_remaining);

        // Fix offset
        offset += ftdi->readbuffer_remaining;
    }
    // do the actual USB read
    while (offset < size && actual_length > 0)
    {
        ftdi->readbuffer_remaining = 0;
        ftdi->readbuffer_offset = 0;
        /* returns how much received */
        ret = usb_bulk_msg(ftdi->usb_dev, usb_rcvbulkpipe(ftdi->usb_dev, ftdi->out_ep), ftdi->readbuffer, ftdi->readbuffer_chunksize, &actual_length, ftdi->usb_read_timeout);
        if (ret < 0)
            ftdi_error_return(ret, "usb bulk read failed");

        if (actual_length > 2)
        {
            // skip FTDI status bytes.
            // Maybe stored in the future to enable modem use
            num_of_chunks = actual_length / packet_size;
            chunk_remains = actual_length % packet_size;
            //printf("actual_length = %X, num_of_chunks = %X, chunk_remains = %X, readbuffer_offset = %X\n", actual_length, num_of_chunks, chunk_remains, ftdi->readbuffer_offset);

            ftdi->readbuffer_offset += 2;
            actual_length -= 2;

            if (actual_length > packet_size - 2)
            {
                for (i = 1; i < num_of_chunks; i++)
                    memmove (ftdi->readbuffer+ftdi->readbuffer_offset+(packet_size - 2)*i,
                             ftdi->readbuffer+ftdi->readbuffer_offset+packet_size*i,
                             packet_size - 2);
                if (chunk_remains > 2)
                {
                    memmove (ftdi->readbuffer+ftdi->readbuffer_offset+(packet_size - 2)*i,
                             ftdi->readbuffer+ftdi->readbuffer_offset+packet_size*i,
                             chunk_remains-2);
                    actual_length -= 2*num_of_chunks;
                }
                else
                    actual_length -= 2*(num_of_chunks-1)+chunk_remains;
            }
        }
        else if (actual_length <= 2)
        {
            // no more data to read?
            return offset;
        }
        if (actual_length > 0)
        {
            // data still fits in buf?
            if (offset+actual_length <= size)
            {
                memcpy (buf+offset, ftdi->readbuffer+ftdi->readbuffer_offset, actual_length);
                //printf("buf[0] = %X, buf[1] = %X\n", buf[0], buf[1]);
                offset += actual_length;

                /* Did we read exactly the right amount of bytes? */
                if (offset == size)
                    //printf("read_data exact rem %d offset %d\n",
                    //ftdi->readbuffer_remaining, offset);
                    return offset;
            }
            else
            {
                // only copy part of the data or size <= readbuffer_chunksize
                int part_size = size-offset;
                memcpy (buf+offset, ftdi->readbuffer+ftdi->readbuffer_offset, part_size);

                ftdi->readbuffer_offset += part_size;
                ftdi->readbuffer_remaining = actual_length-part_size;
                offset += part_size;

                /* printf("Returning part: %d - size: %d - offset: %d - actual_length: %d - remaining: %d\n",
                part_size, size, offset, actual_length, ftdi->readbuffer_remaining); */

                return offset;
            }
        }
    }
    // never reached
    return -127;
}

/**
    Configure read buffer chunk size.
    Default is 4096.

    Automatically reallocates the buffer.

    \param ftdi pointer to ftdi_context
    \param chunksize Chunk size

    \retval 0: all fine
    \retval -1: ftdi context invalid
*/
int ftdi_read_data_set_chunksize(struct ftdi_context *ftdi, unsigned int chunksize)
{
    unsigned char *new_buf;

    if (ftdi == NULL)
        ftdi_error_return(-1, "ftdi context invalid");

    // Invalidate all remaining data
    ftdi->readbuffer_offset = 0;
    ftdi->readbuffer_remaining = 0;
#ifdef __linux__
    /* We can't set readbuffer_chunksize larger than MAX_BULK_BUFFER_LENGTH,
       which is defined in libusb-1.0.  Otherwise, each USB read request will
       be divided into multiple URBs.  This will cause issues on Linux kernel
       older than 2.6.32.  */
    if (chunksize > 16384)
        chunksize = 16384;
#endif

	if (ftdi->readbuffer != NULL)
	kfree(ftdi->readbuffer); 
    if ((new_buf = (unsigned char *)kmalloc(chunksize, GFP_KERNEL)) == NULL)
        ftdi_error_return(-1, "out of memory for readbuffer");

    ftdi->readbuffer = new_buf;
    ftdi->readbuffer_chunksize = chunksize;

    return 0;
}

/**
    Get read buffer chunk size.

    \param ftdi pointer to ftdi_context
    \param chunksize Pointer to store chunk size in

    \retval 0: all fine
    \retval -1: FTDI context invalid
*/
int ftdi_read_data_get_chunksize(struct ftdi_context *ftdi, unsigned int *chunksize)
{
    if (ftdi == NULL)
        ftdi_error_return(-1, "FTDI context invalid");

    *chunksize = ftdi->readbuffer_chunksize;
    return 0;
}

/**
    Enable/disable bitbang modes.

    \param ftdi pointer to ftdi_context
    \param bitmask Bitmask to configure lines.
           HIGH/ON value configures a line as output.
    \param mode Bitbang mode: use the values defined in \ref ftdi_mpsse_mode

    \retval  0: all fine
    \retval -1: can't enable bitbang mode
    \retval -2: USB device unavailable
*/
int ftdi_set_bitmode(struct ftdi_context *ftdi, unsigned char bitmask, unsigned char mode)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    usb_val = bitmask; // low byte: bitmask
    usb_val |= (mode << 8);
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_SET_BITMODE_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "unable to configure bitbang mode. Perhaps not a BM/2232C type chip?");

    ftdi->bitbang_mode = mode;
    ftdi->bitbang_enabled = (mode == BITMODE_RESET) ? 0 : 1;
    return 0;
}

/**
    Disable bitbang mode.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: can't disable bitbang mode
    \retval -2: USB device unavailable
*/
int ftdi_disable_bitbang(struct ftdi_context *ftdi)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_SET_BITMODE_REQUEST, FTDI_DEVICE_OUT_REQTYPE, 0, ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "unable to leave bitbang mode. Perhaps not a BM type chip?");

    ftdi->bitbang_enabled = 0;
    return 0;
}


/**
    Directly read pin state, circumventing the read buffer. Useful for bitbang mode.

    \param ftdi pointer to ftdi_context
    \param pins Pointer to store pins into

    \retval  0: all fine
    \retval -1: read pins failed
    \retval -2: USB device unavailable
*/
int ftdi_read_pins(struct ftdi_context *ftdi, unsigned char *pins)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_READ_PINS_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, ftdi->index, (unsigned char *)pins, 1, ftdi->usb_read_timeout) != 1)
        ftdi_error_return(-1, "read pins failed");

    return 0;
}

/**
    Set latency timer

    The FTDI chip keeps data in the internal buffer for a specific
    amount of time if the buffer is not full yet to decrease
    load on the usb bus.

    \param ftdi pointer to ftdi_context
    \param latency Value between 1 and 255

    \retval  0: all fine
    \retval -1: latency out of range
    \retval -2: unable to set latency timer
    \retval -3: USB device unavailable
*/
int ftdi_set_latency_timer(struct ftdi_context *ftdi, unsigned char latency)
{
    unsigned short usb_val;

    if (latency < 1)
        ftdi_error_return(-1, "latency out of range. Only valid for 1-255");

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-3, "USB device unavailable");

    usb_val = latency;
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_SET_LATENCY_TIMER_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-2, "unable to set latency timer");

    return 0;
}

/**
    Get latency timer

    \param ftdi pointer to ftdi_context
    \param latency Pointer to store latency value in

    \retval  0: all fine
    \retval -1: unable to get latency timer
    \retval -2: USB device unavailable
*/
int ftdi_get_latency_timer(struct ftdi_context *ftdi, unsigned char *latency)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_GET_LATENCY_TIMER_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, ftdi->index, (unsigned char *)&usb_val, 1, ftdi->usb_read_timeout) != 1)
        ftdi_error_return(-1, "reading latency timer failed");

    *latency = (unsigned char)usb_val;
    return 0;
}

/**
    Poll modem status information

    This function allows the retrieve the two status bytes of the device.
    The device sends these bytes also as a header for each read access
    where they are discarded by ftdi_read_data(). The chip generates
    the two stripped status bytes in the absence of data every 40 ms.

    Layout of the first byte:
    - B0..B3 - must be 0
    - B4       Clear to send (CTS)
                 0 = inactive
                 1 = active
    - B5       Data set ready (DTS)
                 0 = inactive
                 1 = active
    - B6       Ring indicator (RI)
                 0 = inactive
                 1 = active
    - B7       Receive line signal detect (RLSD)
                 0 = inactive
                 1 = active

    Layout of the second byte:
    - B0       Data ready (DR)
    - B1       Overrun error (OE)
    - B2       Parity error (PE)
    - B3       Framing error (FE)
    - B4       Break interrupt (BI)
    - B5       Transmitter holding register (THRE)
    - B6       Transmitter empty (TEMT)
    - B7       Error in RCVR FIFO

    \param ftdi pointer to ftdi_context
    \param status Pointer to store status information in. Must be two bytes.

    \retval  0: all fine
    \retval -1: unable to retrieve status information
    \retval -2: USB device unavailable
*/
int ftdi_poll_modem_status(struct ftdi_context *ftdi, unsigned short *status)
{
    char usb_val[2];

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_POLL_MODEM_STATUS_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, ftdi->index, (unsigned char *)usb_val, 2, ftdi->usb_read_timeout) != 2)
        ftdi_error_return(-1, "getting modem status failed");

    *status = (usb_val[1] << 8) | (usb_val[0] & 0xFF);

    return 0;
}

/**
    Set flowcontrol for ftdi chip

    Note: Do not use this function to enable XON/XOFF mode, use ftdi_setflowctrl_xonxoff() instead.

    \param ftdi pointer to ftdi_context
    \param flowctrl flow control to use. should be
           SIO_DISABLE_FLOW_CTRL, SIO_RTS_CTS_HS, SIO_DTR_DSR_HS

    \retval  0: all fine
    \retval -1: set flow control failed
    \retval -2: USB device unavailable
*/
int ftdi_setflowctrl(struct ftdi_context *ftdi, int flowctrl)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_FLOW_CTRL_REQUEST, FTDI_DEVICE_OUT_REQTYPE, 0, (flowctrl | ftdi->index),
                                NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "set flow control failed");

    return 0;
}

/**
    Set XON/XOFF flowcontrol for ftdi chip

    \param ftdi pointer to ftdi_context
    \param xon character code used to resume transmission
    \param xoff character code used to pause transmission

    \retval  0: all fine
    \retval -1: set flow control failed
    \retval -2: USB device unavailable
*/
int ftdi_setflowctrl_xonxoff(struct ftdi_context *ftdi, unsigned char xon, unsigned char xoff)
{
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    uint16_t xonxoff = xon | (xoff << 8);
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_FLOW_CTRL_REQUEST, FTDI_DEVICE_OUT_REQTYPE, xonxoff, (SIO_XON_XOFF_HS | ftdi->index),
                                NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "set flow control failed");

    return 0;
}

/**
    Set dtr line

    \param ftdi pointer to ftdi_context
    \param state state to set line to (1 or 0)

    \retval  0: all fine
    \retval -1: set dtr failed
    \retval -2: USB device unavailable
*/
int ftdi_setdtr(struct ftdi_context *ftdi, int state)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (state)
        usb_val = SIO_SET_DTR_HIGH;
    else
        usb_val = SIO_SET_DTR_LOW;

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_MODEM_CTRL_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index,
                                NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "set dtr failed");

    return 0;
}

/**
    Set rts line

    \param ftdi pointer to ftdi_context
    \param state state to set line to (1 or 0)

    \retval  0: all fine
    \retval -1: set rts failed
    \retval -2: USB device unavailable
*/
int ftdi_setrts(struct ftdi_context *ftdi, int state)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (state)
        usb_val = SIO_SET_RTS_HIGH;
    else
        usb_val = SIO_SET_RTS_LOW;

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_MODEM_CTRL_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index,
                                NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "set of rts failed");

    return 0;
}

/**
    Set dtr and rts line in one pass

    \param ftdi pointer to ftdi_context
    \param dtr  DTR state to set line to (1 or 0)
    \param rts  RTS state to set line to (1 or 0)

    \retval  0: all fine
    \retval -1: set dtr/rts failed
    \retval -2: USB device unavailable
 */
int ftdi_setdtr_rts(struct ftdi_context *ftdi, int dtr, int rts)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (dtr)
        usb_val = SIO_SET_DTR_HIGH;
    else
        usb_val = SIO_SET_DTR_LOW;

    if (rts)
        usb_val |= SIO_SET_RTS_HIGH;
    else
        usb_val |= SIO_SET_RTS_LOW;

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), 
                                SIO_SET_MODEM_CTRL_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index,
                                NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "set of rts/dtr failed");

    return 0;
}

/**
    Set the special event character

    \param ftdi pointer to ftdi_context
    \param eventch Event character
    \param enable 0 to disable the event character, non-zero otherwise

    \retval  0: all fine
    \retval -1: unable to set event character
    \retval -2: USB device unavailable
*/
int ftdi_set_event_char(struct ftdi_context *ftdi,
                        unsigned char eventch, unsigned char enable)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    usb_val = eventch;
    if (enable)
        usb_val |= 1 << 8;

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_SET_EVENT_CHAR_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "setting event character failed");

    return 0;
}

/**
    Set error character

    \param ftdi pointer to ftdi_context
    \param errorch Error character
    \param enable 0 to disable the error character, non-zero otherwise

    \retval  0: all fine
    \retval -1: unable to set error character
    \retval -2: USB device unavailable
*/
int ftdi_set_error_char(struct ftdi_context *ftdi,
                        unsigned char errorch, unsigned char enable)
{
    unsigned short usb_val;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    usb_val = errorch;
    if (enable)
        usb_val |= 1 << 8;

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_SET_ERROR_CHAR_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, ftdi->index, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "setting error character failed");

    return 0;
}

/**
    Init eeprom with default values for the connected device
    \param ftdi pointer to ftdi_context
    \param manufacturer String to use as Manufacturer
    \param product String to use as Product description
    \param serial String to use as Serial number description

    \retval  0: all fine
    \retval -1: No struct ftdi_context
    \retval -2: No struct ftdi_eeprom
    \retval -3: No connected device or device not yet opened
*/
int ftdi_eeprom_initdefaults(struct ftdi_context *ftdi, char * manufacturer,
                             char * product, char * serial)
{
    struct ftdi_eeprom *eeprom;

    if (ftdi == NULL)
        ftdi_error_return(-1, "No struct ftdi_context");

    if (ftdi->eeprom == NULL)
        ftdi_error_return(-2,"No struct ftdi_eeprom");

    eeprom = ftdi->eeprom;
    memset(eeprom, 0, sizeof(struct ftdi_eeprom));

    if (ftdi->usb_dev == NULL)
        ftdi_error_return(-3, "No connected device or device not yet opened");

    eeprom->vendor_id = 0x0403;
    eeprom->use_serial = 1;
    if ((ftdi->type == TYPE_AM) || (ftdi->type == TYPE_BM) ||
            (ftdi->type == TYPE_R))
        eeprom->product_id = 0x6001;
    else if (ftdi->type == TYPE_4232H)
        eeprom->product_id = 0x6011;
    else if (ftdi->type == TYPE_232H)
        eeprom->product_id = 0x6014;
    else if (ftdi->type == TYPE_230X)
        eeprom->product_id = 0x6015;
    else
        eeprom->product_id = 0x6010;

    if (ftdi->type == TYPE_AM)
        eeprom->usb_version = 0x0101;
    else
        eeprom->usb_version = 0x0200;
    eeprom->max_power = 100;

    if (eeprom->manufacturer)
        kfree (eeprom->manufacturer);
    eeprom->manufacturer = NULL;
    if (manufacturer)
    {
        eeprom->manufacturer = (char *)kmalloc(strlen(manufacturer)+1, GFP_KERNEL);
        if (eeprom->manufacturer)
            strcpy(eeprom->manufacturer, manufacturer);
    }

    if (eeprom->product)
        kfree (eeprom->product);
    eeprom->product = NULL;
    if(product)
    {
        eeprom->product = (char *)kmalloc(strlen(product)+1, GFP_KERNEL);
        if (eeprom->product)
            strcpy(eeprom->product, product);
    }
    else
    {
        const char* default_product;
        switch(ftdi->type)
        {
            case TYPE_AM:    default_product = "AM"; break;
            case TYPE_BM:    default_product = "BM"; break;
            case TYPE_2232C: default_product = "Dual RS232"; break;
            case TYPE_R:     default_product = "FT232R USB UART"; break;
            case TYPE_2232H: default_product = "Dual RS232-HS"; break;
            case TYPE_4232H: default_product = "FT4232H"; break;
            case TYPE_232H:  default_product = "Single-RS232-HS"; break;
            case TYPE_230X:  default_product = "FT230X Basic UART"; break;
            default:
                ftdi_error_return(-3, "Unknown chip type");
        }
        eeprom->product = (char *)kmalloc(strlen(default_product) +1, GFP_KERNEL);
        if (eeprom->product)
            strcpy(eeprom->product, default_product);
    }

    if (eeprom->serial)
        kfree (eeprom->serial);
    eeprom->serial = NULL;
    if (serial)
    {
        eeprom->serial = (char *)kmalloc(strlen(serial)+1, GFP_KERNEL);
        if (eeprom->serial)
            strcpy(eeprom->serial, serial);
    }

    if (ftdi->type == TYPE_R)
    {
        eeprom->max_power = 90;
        eeprom->size = 0x80;
        eeprom->cbus_function[0] = CBUS_TXLED;
        eeprom->cbus_function[1] = CBUS_RXLED;
        eeprom->cbus_function[2] = CBUS_TXDEN;
        eeprom->cbus_function[3] = CBUS_PWREN;
        eeprom->cbus_function[4] = CBUS_SLEEP;
    }
    else if (ftdi->type == TYPE_230X)
    {
        eeprom->max_power = 90;
        eeprom->size = 0x100;
        eeprom->cbus_function[0] = CBUSX_TXDEN;
        eeprom->cbus_function[1] = CBUSX_RXLED;
        eeprom->cbus_function[2] = CBUSX_TXLED;
        eeprom->cbus_function[3] = CBUSX_SLEEP;
    }
    else
    {
        if(ftdi->type == TYPE_232H)
        {
            int i;
            for (i=0; i<10; i++)
                eeprom->cbus_function[i] = CBUSH_TRISTATE;
        }
        eeprom->size = -1;
    }
    switch (ftdi->type)
    {
        case TYPE_AM:
            eeprom->release_number = 0x0200;
            break;
        case TYPE_BM:
            eeprom->release_number = 0x0400;
            break;
        case TYPE_2232C:
            eeprom->release_number = 0x0500;
            break;
        case TYPE_R:
            eeprom->release_number = 0x0600;
            break;
        case TYPE_2232H:
            eeprom->release_number = 0x0700;
            break;
        case TYPE_4232H:
            eeprom->release_number = 0x0800;
            break;
        case TYPE_232H:
            eeprom->release_number = 0x0900;
            break;
        case TYPE_230X:
            eeprom->release_number = 0x1000;
            break;
        default:
            eeprom->release_number = 0x00;
    }
    return 0;
}

int ftdi_eeprom_set_strings(struct ftdi_context *ftdi, char * manufacturer,
                            char * product, char * serial)
{
    struct ftdi_eeprom *eeprom;

    if (ftdi == NULL)
        ftdi_error_return(-1, "No struct ftdi_context");

    if (ftdi->eeprom == NULL)
        ftdi_error_return(-2,"No struct ftdi_eeprom");

    eeprom = ftdi->eeprom;

    if (ftdi->usb_dev == NULL)
        ftdi_error_return(-3, "No connected device or device not yet opened");

    if (manufacturer)
    {
        if (eeprom->manufacturer)
            kfree (eeprom->manufacturer);
        eeprom->manufacturer = (char *)kmalloc(strlen(manufacturer)+1, GFP_KERNEL);
        if (eeprom->manufacturer)
            strcpy(eeprom->manufacturer, manufacturer);
    }

    if(product)
    {
        if (eeprom->product)
            kfree (eeprom->product);
        eeprom->product = (char *)kmalloc(strlen(product)+1, GFP_KERNEL);
        if (eeprom->product)
            strcpy(eeprom->product, product);
    }

    if (serial)
    {
        if (eeprom->serial)
            kfree (eeprom->serial);
        eeprom->serial = (char *)kmalloc(strlen(serial)+1, GFP_KERNEL);
        if (eeprom->serial)
        {
            strcpy(eeprom->serial, serial);
            eeprom->use_serial = 1;
        }
    }
    return 0;
}

/**
    Return device ID strings from the eeprom. Device needs to be connected.

    The parameters manufacturer, description and serial may be NULL
    or pointer to buffers to store the fetched strings.

    \param ftdi pointer to ftdi_context
    \param manufacturer Store manufacturer string here if not NULL
    \param mnf_len Buffer size of manufacturer string
    \param product Store product description string here if not NULL
    \param prod_len Buffer size of product description string
    \param serial Store serial string here if not NULL
    \param serial_len Buffer size of serial string

    \retval   0: all fine
    \retval  -1: ftdi context invalid
    \retval  -2: ftdi eeprom buffer invalid
*/
int ftdi_eeprom_get_strings(struct ftdi_context *ftdi,
                            char *manufacturer, int mnf_len,
                            char *product, int prod_len,
                            char *serial, int serial_len)
{
    struct ftdi_eeprom *eeprom;

    if (ftdi == NULL)
        ftdi_error_return(-1, "No struct ftdi_context");
    if (ftdi->eeprom == NULL)
        ftdi_error_return(-2, "No struct ftdi_eeprom");

    eeprom = ftdi->eeprom;

    if (manufacturer)
    {
        strncpy(manufacturer, eeprom->manufacturer, mnf_len);
        if (mnf_len > 0)
            manufacturer[mnf_len - 1] = '\0';
    }

    if (product)
    {
        strncpy(product, eeprom->product, prod_len);
        if (prod_len > 0)
            product[prod_len - 1] = '\0';
    }

    if (serial)
    {
        strncpy(serial, eeprom->serial, serial_len);
        if (serial_len > 0)
            serial[serial_len - 1] = '\0';
    }

    return 0;
}

/*FTD2XX doesn't check for values not fitting in the ACBUS Signal options*/
void set_ft232h_cbus(struct ftdi_eeprom *eeprom, unsigned char * output)
{
    int i;
    for(i=0; i<5; i++)
    {
        int mode_low, mode_high;
        if (eeprom->cbus_function[2*i]> CBUSH_CLK7_5)
            mode_low = CBUSH_TRISTATE;
        else
            mode_low = eeprom->cbus_function[2*i];
        if (eeprom->cbus_function[2*i+1]> CBUSH_CLK7_5)
            mode_high = CBUSH_TRISTATE;
        else
            mode_high = eeprom->cbus_function[2*i+1];

        output[0x18+i] = (mode_high <<4) | mode_low;
    }
}
/* Return the bits for the encoded EEPROM Structure of a requested Mode
 *
 */
static unsigned char type2bit(unsigned char type, enum ftdi_chip_type chip)
{
    switch (chip)
    {
        case TYPE_2232H:
        case TYPE_2232C:
        {
            switch (type)
            {
                case CHANNEL_IS_UART: return 0;
                case CHANNEL_IS_FIFO: return 0x01;
                case CHANNEL_IS_OPTO: return 0x02;
                case CHANNEL_IS_CPU : return 0x04;
                default: return 0;
            }
        }
        case TYPE_232H:
        {
            switch (type)
            {
                case CHANNEL_IS_UART   : return 0;
                case CHANNEL_IS_FIFO   : return 0x01;
                case CHANNEL_IS_OPTO   : return 0x02;
                case CHANNEL_IS_CPU    : return 0x04;
                case CHANNEL_IS_FT1284 : return 0x08;
                default: return 0;
            }
        }
        case TYPE_R:
        {
            switch (type)
            {
                case CHANNEL_IS_UART   : return 0;
                case CHANNEL_IS_FIFO   : return 0x01;
                default: return 0;
            }
        }
        case TYPE_230X: /* FT230X is only UART */
        default: return 0;
    }
    return 0;
}

/**
    Build binary buffer from ftdi_eeprom structure.
    Output is suitable for ftdi_write_eeprom().

    \param ftdi pointer to ftdi_context

    \retval >=0: size of eeprom user area in bytes
    \retval -1: eeprom size (128 bytes) exceeded by custom strings
    \retval -2: Invalid eeprom or ftdi pointer
    \retval -3: Invalid cbus function setting     (FIXME: Not in the code?)
    \retval -4: Chip doesn't support invert       (FIXME: Not in the code?)
    \retval -5: Chip doesn't support high current drive         (FIXME: Not in the code?)
    \retval -6: No connected EEPROM or EEPROM Type unknown
*/
int ftdi_eeprom_build(struct ftdi_context *ftdi)
{
    unsigned char i, j, eeprom_size_mask;
    unsigned short checksum, value;
    unsigned char manufacturer_size = 0, product_size = 0, serial_size = 0;
    int user_area_size, kfree_start, kfree_end;
    struct ftdi_eeprom *eeprom;
    unsigned char * output;

    if (ftdi == NULL)
        ftdi_error_return(-2,"No context");
    if (ftdi->eeprom == NULL)
        ftdi_error_return(-2,"No eeprom structure");

    eeprom= ftdi->eeprom;
    output = eeprom->buf;

    if (eeprom->chip == -1)
        ftdi_error_return(-6,"No connected EEPROM or EEPROM type unknown");

    if (eeprom->size == -1)
    {
        if ((eeprom->chip == 0x56) || (eeprom->chip == 0x66))
            eeprom->size = 0x100;
        else
            eeprom->size = 0x80;
    }

    if (eeprom->manufacturer != NULL)
        manufacturer_size = strlen(eeprom->manufacturer);
    if (eeprom->product != NULL)
        product_size = strlen(eeprom->product);
    if (eeprom->serial != NULL)
        serial_size = strlen(eeprom->serial);

    // eeprom size check
    switch (ftdi->type)
    {
        case TYPE_AM:
        case TYPE_BM:
        case TYPE_R:
            user_area_size = 96;    // base size for strings (total of 48 characters)
            break;
        case TYPE_2232C:
            user_area_size = 90;     // two extra config bytes and 4 bytes PnP stuff
            break;
        case TYPE_230X:
            user_area_size = 88;     // four extra config bytes + 4 bytes PnP stuff
            break;
        case TYPE_2232H:            // six extra config bytes + 4 bytes PnP stuff
        case TYPE_4232H:
            user_area_size = 86;
            break;
        case TYPE_232H:
            user_area_size = 80;
            break;
        default:
            user_area_size = 0;
            break;
    }
    user_area_size  -= (manufacturer_size + product_size + serial_size) * 2;

    if (user_area_size < 0)
        ftdi_error_return(-1,"eeprom size exceeded");

    // empty eeprom
    if (ftdi->type == TYPE_230X)
    {
        /* FT230X have a reserved section in the middle of the MTP,
           which cannot be written to, but must be included in the checksum */
        memset(ftdi->eeprom->buf, 0, 0x80);
        memset((ftdi->eeprom->buf + 0xa0), 0, (FTDI_MAX_EEPROM_SIZE - 0xa0));
    }
    else
    {
        memset(ftdi->eeprom->buf, 0, FTDI_MAX_EEPROM_SIZE);
    }

    // Bytes and Bits set for all Types

    // Addr 02: Vendor ID
    output[0x02] = eeprom->vendor_id;
    output[0x03] = eeprom->vendor_id >> 8;

    // Addr 04: Product ID
    output[0x04] = eeprom->product_id;
    output[0x05] = eeprom->product_id >> 8;

    // Addr 06: Device release number (0400h for BM features)
    output[0x06] = eeprom->release_number;
    output[0x07] = eeprom->release_number >> 8;

    // Addr 08: Config descriptor
    // Bit 7: always 1
    // Bit 6: 1 if this device is self powered, 0 if bus powered
    // Bit 5: 1 if this device uses remote wakeup
    // Bit 4-0: reserved - 0
    j = 0x80;
    if (eeprom->self_powered)
        j |= 0x40;
    if (eeprom->remote_wakeup)
        j |= 0x20;
    output[0x08] = j;

    // Addr 09: Max power consumption: max power = value * 2 mA
    output[0x09] = eeprom->max_power / MAX_POWER_MILLIAMP_PER_UNIT;

    if ((ftdi->type != TYPE_AM) && (ftdi->type != TYPE_230X))
    {
        // Addr 0A: Chip configuration
        // Bit 7: 0 - reserved
        // Bit 6: 0 - reserved
        // Bit 5: 0 - reserved
        // Bit 4: 1 - Change USB version
        // Bit 3: 1 - Use the serial number string
        // Bit 2: 1 - Enable suspend pull downs for lower power
        // Bit 1: 1 - Out EndPoint is Isochronous
        // Bit 0: 1 - In EndPoint is Isochronous
        //
        j = 0;
        if (eeprom->in_is_isochronous)
            j = j | 1;
        if (eeprom->out_is_isochronous)
            j = j | 2;
        output[0x0A] = j;
    }

    // Dynamic content
    // Strings start at 0x94 (TYPE_AM, TYPE_BM)
    // 0x96 (TYPE_2232C), 0x98 (TYPE_R) and 0x9a (TYPE_x232H)
    // 0xa0 (TYPE_232H)
    i = 0;
    switch (ftdi->type)
    {
        case TYPE_2232H:
        case TYPE_4232H:
            i += 2;
        case TYPE_R:
            i += 2;
        case TYPE_2232C:
            i += 2;
        case TYPE_AM:
        case TYPE_BM:
            i += 0x94;
            break;
        case TYPE_232H:
        case TYPE_230X:
            i = 0xa0;
            break;
    }
    /* Wrap around 0x80 for 128 byte EEPROMS (Internale and 93x46) */
    eeprom_size_mask = eeprom->size -1;
    kfree_end = i & eeprom_size_mask;

    // Addr 0E: Offset of the manufacturer string + 0x80, calculated later
    // Addr 0F: Length of manufacturer string
    // Output manufacturer
    output[0x0E] = i;  // calculate offset
    output[i & eeprom_size_mask] = manufacturer_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++; // type: string
    for (j = 0; j < manufacturer_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->manufacturer[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }
    output[0x0F] = manufacturer_size*2 + 2;

    // Addr 10: Offset of the product string + 0x80, calculated later
    // Addr 11: Length of product string
    output[0x10] = i | 0x80;  // calculate offset
    output[i & eeprom_size_mask] = product_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++;
    for (j = 0; j < product_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->product[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }
    output[0x11] = product_size*2 + 2;

    // Addr 12: Offset of the serial string + 0x80, calculated later
    // Addr 13: Length of serial string
    output[0x12] = i | 0x80; // calculate offset
    output[i & eeprom_size_mask] = serial_size*2 + 2, i++;
    output[i & eeprom_size_mask] = 0x03, i++;
    for (j = 0; j < serial_size; j++)
    {
        output[i & eeprom_size_mask] = eeprom->serial[j], i++;
        output[i & eeprom_size_mask] = 0x00, i++;
    }

    // Legacy port name and PnP fields for FT2232 and newer chips
    if (ftdi->type > TYPE_BM)
    {
        output[i & eeprom_size_mask] = 0x02; /* as seen when written with FTD2XX */
        i++;
        output[i & eeprom_size_mask] = 0x03; /* as seen when written with FTD2XX */
        i++;
        output[i & eeprom_size_mask] = eeprom->is_not_pnp; /* as seen when written with FTD2XX */
        i++;
    }

    output[0x13] = serial_size*2 + 2;

    if (ftdi->type > TYPE_AM) /* use_serial not used in AM devices */
    {
        if (eeprom->use_serial)
            output[0x0A] |= USE_SERIAL_NUM;
        else
            output[0x0A] &= ~USE_SERIAL_NUM;
    }

    /* Bytes and Bits specific to (some) types
       Write linear, as this allows easier fixing*/
    switch (ftdi->type)
    {
        case TYPE_AM:
            break;
        case TYPE_BM:
            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;
            if (eeprom->use_usb_version)
                output[0x0A] |= USE_USB_VERSION_BIT;
            else
                output[0x0A] &= ~USE_USB_VERSION_BIT;

            break;
        case TYPE_2232C:

            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_2232C);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;

            if ( eeprom->high_current_a == HIGH_CURRENT_DRIVE)
                output[0x00] |= HIGH_CURRENT_DRIVE;
            else
                output[0x00] &= ~HIGH_CURRENT_DRIVE;

            output[0x01] = type2bit(eeprom->channel_b_type, TYPE_2232C);
            if ( eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;

            if ( eeprom->high_current_b == HIGH_CURRENT_DRIVE)
                output[0x01] |= HIGH_CURRENT_DRIVE;
            else
                output[0x01] &= ~HIGH_CURRENT_DRIVE;

            if (eeprom->in_is_isochronous)
                output[0x0A] |= 0x1;
            else
                output[0x0A] &= ~0x1;
            if (eeprom->out_is_isochronous)
                output[0x0A] |= 0x2;
            else
                output[0x0A] &= ~0x2;
            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;
            if (eeprom->use_usb_version)
                output[0x0A] |= USE_USB_VERSION_BIT;
            else
                output[0x0A] &= ~USE_USB_VERSION_BIT;

            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;
            output[0x14] = eeprom->chip;
            break;
        case TYPE_R:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_R);
            if (eeprom->high_current == HIGH_CURRENT_DRIVE_R)
                output[0x00] |= HIGH_CURRENT_DRIVE_R;
            if (eeprom->external_oscillator)
                output[0x00] |= 0x02;
            output[0x01] = 0x40; /* Hard coded Endpoint Size*/

            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;
            output[0x0B] = eeprom->invert;
            output[0x0C] = eeprom->usb_version & 0xff;
            output[0x0D] = (eeprom->usb_version>>8) & 0xff;

            if (eeprom->cbus_function[0] > CBUS_BB_RD)
                output[0x14] = CBUS_TXLED;
            else
                output[0x14] = eeprom->cbus_function[0];

            if (eeprom->cbus_function[1] > CBUS_BB_RD)
                output[0x14] |= CBUS_RXLED<<4;
            else
                output[0x14] |= eeprom->cbus_function[1]<<4;

            if (eeprom->cbus_function[2] > CBUS_BB_RD)
                output[0x15] = CBUS_TXDEN;
            else
                output[0x15] = eeprom->cbus_function[2];

            if (eeprom->cbus_function[3] > CBUS_BB_RD)
                output[0x15] |= CBUS_PWREN<<4;
            else
                output[0x15] |= eeprom->cbus_function[3]<<4;

            if (eeprom->cbus_function[4] > CBUS_CLK6)
                output[0x16] = CBUS_SLEEP;
            else
                output[0x16] = eeprom->cbus_function[4];
            break;
        case TYPE_2232H:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_2232H);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;

            output[0x01] = type2bit(eeprom->channel_b_type, TYPE_2232H);
            if ( eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;
            if (eeprom->suspend_dbus7 == SUSPEND_DBUS7_BIT)
                output[0x01] |= SUSPEND_DBUS7_BIT;
            else
                output[0x01] &= ~SUSPEND_DBUS7_BIT;

            if (eeprom->suspend_pull_downs)
                output[0x0A] |= 0x4;
            else
                output[0x0A] &= ~0x4;

            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA<<4;
            else
                output[0x0c] |= eeprom->group1_drive<<4;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT<<4;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW<<4;

            if (eeprom->group2_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group2_drive;
            if (eeprom->group2_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group2_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            if (eeprom->group3_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA<<4;
            else
                output[0x0d] |= eeprom->group3_drive<<4;
            if (eeprom->group3_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT<<4;
            if (eeprom->group3_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW<<4;

            output[0x18] = eeprom->chip;

            break;
        case TYPE_4232H:
            if (eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCP;
            else
                output[0x00] &= ~DRIVER_VCP;
            if (eeprom->channel_b_driver == DRIVER_VCP)
                output[0x01] |= DRIVER_VCP;
            else
                output[0x01] &= ~DRIVER_VCP;
            if (eeprom->channel_c_driver == DRIVER_VCP)
                output[0x00] |= (DRIVER_VCP << 4);
            else
                output[0x00] &= ~(DRIVER_VCP << 4);
            if (eeprom->channel_d_driver == DRIVER_VCP)
                output[0x01] |= (DRIVER_VCP << 4);
            else
                output[0x01] &= ~(DRIVER_VCP << 4);

            if (eeprom->suspend_pull_downs)
                output[0x0a] |= 0x4;
            else
                output[0x0a] &= ~0x4;

            if (eeprom->channel_a_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 0;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 0);
            if (eeprom->channel_b_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 1;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 1);
            if (eeprom->channel_c_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 2;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 2);
            if (eeprom->channel_d_rs485enable)
                output[0x0b] |= CHANNEL_IS_RS485 << 3;
            else
                output[0x0b] &= ~(CHANNEL_IS_RS485 << 3);

            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA<<4;
            else
                output[0x0c] |= eeprom->group1_drive<<4;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT<<4;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW<<4;

            if (eeprom->group2_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group2_drive;
            if (eeprom->group2_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group2_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            if (eeprom->group3_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA<<4;
            else
                output[0x0d] |= eeprom->group3_drive<<4;
            if (eeprom->group3_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT<<4;
            if (eeprom->group3_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW<<4;

            output[0x18] = eeprom->chip;

            break;
        case TYPE_232H:
            output[0x00] = type2bit(eeprom->channel_a_type, TYPE_232H);
            if ( eeprom->channel_a_driver == DRIVER_VCP)
                output[0x00] |= DRIVER_VCPH;
            else
                output[0x00] &= ~DRIVER_VCPH;
            if (eeprom->powersave)
                output[0x01] |= POWER_SAVE_DISABLE_H;
            else
                output[0x01] &= ~POWER_SAVE_DISABLE_H;

            if (eeprom->suspend_pull_downs)
                output[0x0a] |= 0x4;
            else
                output[0x0a] &= ~0x4;

            if (eeprom->clock_polarity)
                output[0x01] |= FT1284_CLK_IDLE_STATE;
            else
                output[0x01] &= ~FT1284_CLK_IDLE_STATE;
            if (eeprom->data_order)
                output[0x01] |= FT1284_DATA_LSB;
            else
                output[0x01] &= ~FT1284_DATA_LSB;
            if (eeprom->flow_control)
                output[0x01] |= FT1284_FLOW_CONTROL;
            else
                output[0x01] &= ~FT1284_FLOW_CONTROL;
            if (eeprom->group0_drive > DRIVE_16MA)
                output[0x0c] |= DRIVE_16MA;
            else
                output[0x0c] |= eeprom->group0_drive;
            if (eeprom->group0_schmitt == IS_SCHMITT)
                output[0x0c] |= IS_SCHMITT;
            if (eeprom->group0_slew == SLOW_SLEW)
                output[0x0c] |= SLOW_SLEW;

            if (eeprom->group1_drive > DRIVE_16MA)
                output[0x0d] |= DRIVE_16MA;
            else
                output[0x0d] |= eeprom->group1_drive;
            if (eeprom->group1_schmitt == IS_SCHMITT)
                output[0x0d] |= IS_SCHMITT;
            if (eeprom->group1_slew == SLOW_SLEW)
                output[0x0d] |= SLOW_SLEW;

            set_ft232h_cbus(eeprom, output);

            output[0x1e] = eeprom->chip;
            printk(KERN_ERR "FIXME: Build FT232H specific EEPROM settings\n");
            break;
        case TYPE_230X:
            output[0x00] = 0x80; /* Actually, leave the default value */
            /*FIXME: Make DBUS & CBUS Control configurable*/
            output[0x0c] = 0;    /* DBUS drive 4mA, CBUS drive 4 mA like factory default */
            for (j = 0; j <= 6; j++)
            {
                output[0x1a + j] = eeprom->cbus_function[j];
            }
            output[0x0b] = eeprom->invert;
            break;
    }

    /* First address without use */
    kfree_start = 0;
    switch (ftdi->type)
    {
        case TYPE_230X:
            kfree_start += 2;
        case TYPE_232H:
            kfree_start += 6;
        case TYPE_2232H:
        case TYPE_4232H:
            kfree_start += 2;
        case TYPE_R:
            kfree_start += 2;
        case TYPE_2232C:
            kfree_start++;
        case TYPE_AM:
        case TYPE_BM:
            kfree_start += 0x14;
    }

    /* Arbitrary user data */
    if (eeprom->user_data && eeprom->user_data_size >= 0)
    {
        if (eeprom->user_data_addr < kfree_start)
            printk(KERN_ERR "Warning, user data starts inside the generated data!\n");
        if (eeprom->user_data_addr + eeprom->user_data_size >= kfree_end)
            printk(KERN_ERR "Warning, user data overlaps the strings area!\n");
        if (eeprom->user_data_addr + eeprom->user_data_size > eeprom->size)
            ftdi_error_return(-1,"eeprom size exceeded");
        memcpy(output + eeprom->user_data_addr, eeprom->user_data, eeprom->user_data_size);
    }

    // calculate checksum
    checksum = 0xAAAA;

    for (i = 0; i < eeprom->size/2-1; i++)
    {
        if ((ftdi->type == TYPE_230X) && (i == 0x12))
        {
            /* FT230X has a user section in the MTP which is not part of the checksum */
            i = 0x40;
        }
        if ((ftdi->type == TYPE_230X) && (i >=  0x40) && (i < 0x50)) {
            uint16_t data;
            if (ftdi_read_eeprom_location(ftdi, i, &data)) {
                printk(KERN_ERR  "Reading Factory Configuration Data failed\n");
                i = 0x50;
            }
            value = data;
        }
        else {
            value = output[i*2];
            value += output[(i*2)+1] << 8;
        }
        checksum = value^checksum;
        checksum = (checksum << 1) | (checksum >> 15);
    }

    output[eeprom->size-2] = checksum;
    output[eeprom->size-1] = checksum >> 8;

    eeprom->initialized_for_connected_device = 1;
    return user_area_size;
}
/* Decode the encoded EEPROM field for the FTDI Mode into a value for the abstracted
 * EEPROM structure
 *
 * FTD2XX doesn't allow to set multiple bits in the interface mode bitfield, and so do we
 */
static unsigned char bit2type(unsigned char bits)
{
    switch (bits)
    {
        case   0: return CHANNEL_IS_UART;
        case   1: return CHANNEL_IS_FIFO;
        case   2: return CHANNEL_IS_OPTO;
        case   4: return CHANNEL_IS_CPU;
        case   8: return CHANNEL_IS_FT1284;
        default:
            printk(KERN_ERR " Unexpected value %d for Hardware Interface type\n",
                    bits);
    }
    return 0;
}
/* Decode 230X / 232R type chips invert bits
 * Prints directly to stdout.
*/
static void print_inverted_bits(int invert)
{
    const char *r_bits[] = {"TXD","RXD","RTS","CTS","DTR","DSR","DCD","RI"};
    int i;

    printk(KERN_INFO"Inverted bits:");
    for (i=0; i<8; i++)
        if ((invert & (1<<i)) == (1<<i))
            printk(KERN_INFO" %s",r_bits[i]);

    printk(KERN_INFO"\n");
}
/**
   Decode binary EEPROM image into an ftdi_eeprom structure.

   For FT-X devices use AN_201 FT-X MTP memory Configuration to decode.

   \param ftdi pointer to ftdi_context
   \param verbose Decode EEPROM on stdout

   \retval 0: all fine
   \retval -1: something went wrong

   FIXME: How to pass size? How to handle size field in ftdi_eeprom?
   FIXME: Strings are kmalloc'ed here and should be kfreed somewhere
*/
int ftdi_eeprom_decode(struct ftdi_context *ftdi, int verbose)
{
    int i, j;
    unsigned short checksum, eeprom_checksum, value;
    unsigned char manufacturer_size = 0, product_size = 0, serial_size = 0;
    int eeprom_size;
    struct ftdi_eeprom *eeprom;
    unsigned char *buf = NULL;

    if (ftdi == NULL)
        ftdi_error_return(-1,"No context");
    if (ftdi->eeprom == NULL)
        ftdi_error_return(-1,"No eeprom structure");

    eeprom = ftdi->eeprom;
    eeprom_size = eeprom->size;
    buf = ftdi->eeprom->buf;

    // Addr 02: Vendor ID
    eeprom->vendor_id = buf[0x02] + (buf[0x03] << 8);

    // Addr 04: Product ID
    eeprom->product_id = buf[0x04] + (buf[0x05] << 8);

    // Addr 06: Device release number
    eeprom->release_number = buf[0x06] + (buf[0x07]<<8);

    // Addr 08: Config descriptor
    // Bit 7: always 1
    // Bit 6: 1 if this device is self powered, 0 if bus powered
    // Bit 5: 1 if this device uses remote wakeup
    eeprom->self_powered = buf[0x08] & 0x40;
    eeprom->remote_wakeup = buf[0x08] & 0x20;

    // Addr 09: Max power consumption: max power = value * 2 mA
    eeprom->max_power = MAX_POWER_MILLIAMP_PER_UNIT * buf[0x09];

    // Addr 0A: Chip configuration
    // Bit 7: 0 - reserved
    // Bit 6: 0 - reserved
    // Bit 5: 0 - reserved
    // Bit 4: 1 - Change USB version on BM and 2232C
    // Bit 3: 1 - Use the serial number string
    // Bit 2: 1 - Enable suspend pull downs for lower power
    // Bit 1: 1 - Out EndPoint is Isochronous
    // Bit 0: 1 - In EndPoint is Isochronous
    //
    eeprom->in_is_isochronous  = buf[0x0A]&0x01;
    eeprom->out_is_isochronous = buf[0x0A]&0x02;
    eeprom->suspend_pull_downs = buf[0x0A]&0x04;
    eeprom->use_serial         = !!(buf[0x0A] & USE_SERIAL_NUM);
    eeprom->use_usb_version    = !!(buf[0x0A] & USE_USB_VERSION_BIT);

    // Addr 0C: USB version low byte when 0x0A
    // Addr 0D: USB version high byte when 0x0A
    eeprom->usb_version = buf[0x0C] + (buf[0x0D] << 8);

    // Addr 0E: Offset of the manufacturer string + 0x80, calculated later
    // Addr 0F: Length of manufacturer string
    manufacturer_size = buf[0x0F]/2;
    if (eeprom->manufacturer)
        kfree(eeprom->manufacturer);
    if (manufacturer_size > 0)
    {
        eeprom->manufacturer = (char *)kmalloc(manufacturer_size, GFP_KERNEL);
        if (eeprom->manufacturer)
        {
            // Decode manufacturer
            i = buf[0x0E] & (eeprom_size -1); // offset
            for (j=0; j<manufacturer_size-1; j++)
            {
                eeprom->manufacturer[j] = buf[2*j+i+2];
            }
            eeprom->manufacturer[j] = '\0';
        }
    }
    else eeprom->manufacturer = NULL;

    // Addr 10: Offset of the product string + 0x80, calculated later
    // Addr 11: Length of product string
    if (eeprom->product)
        kfree(eeprom->product);
    product_size = buf[0x11]/2;
    if (product_size > 0)
    {
        eeprom->product = (char *)kmalloc(product_size, GFP_KERNEL);
        if (eeprom->product)
        {
            // Decode product name
            i = buf[0x10] & (eeprom_size -1); // offset
            for (j=0; j<product_size-1; j++)
            {
                eeprom->product[j] = buf[2*j+i+2];
            }
            eeprom->product[j] = '\0';
        }
    }
    else eeprom->product = NULL;

    // Addr 12: Offset of the serial string + 0x80, calculated later
    // Addr 13: Length of serial string
    if (eeprom->serial)
        kfree(eeprom->serial);
    serial_size = buf[0x13]/2;
    if (serial_size > 0)
    {
        eeprom->serial = (char *)kmalloc(serial_size, GFP_KERNEL);
        if (eeprom->serial)
        {
            // Decode serial
            i = buf[0x12] & (eeprom_size -1); // offset
            for (j=0; j<serial_size-1; j++)
            {
                eeprom->serial[j] = buf[2*j+i+2];
            }
            eeprom->serial[j] = '\0';
        }
    }
    else eeprom->serial = NULL;

    // verify checksum
    checksum = 0xAAAA;

    for (i = 0; i < eeprom_size/2-1; i++)
    {
        if ((ftdi->type == TYPE_230X) && (i == 0x12))
        {
            /* FT230X has a user section in the MTP which is not part of the checksum */
            i = 0x40;
        }
        value = buf[i*2];
        value += buf[(i*2)+1] << 8;

        checksum = value^checksum;
        checksum = (checksum << 1) | (checksum >> 15);
    }

    eeprom_checksum = buf[eeprom_size-2] + (buf[eeprom_size-1] << 8);

    if (eeprom_checksum != checksum)
    {
        printk(KERN_ERR  "Checksum Error: %04x %04x\n", checksum, eeprom_checksum);
        ftdi_error_return(-1,"EEPROM checksum error");
    }

    eeprom->channel_a_type   = 0;
    if ((ftdi->type == TYPE_AM) || (ftdi->type == TYPE_BM))
    {
        eeprom->chip = -1;
    }
    else if (ftdi->type == TYPE_2232C)
    {
        eeprom->channel_a_type   = bit2type(buf[0x00] & 0x7);
        eeprom->channel_a_driver = buf[0x00] & DRIVER_VCP;
        eeprom->high_current_a   = buf[0x00] & HIGH_CURRENT_DRIVE;
        eeprom->channel_b_type   = buf[0x01] & 0x7;
        eeprom->channel_b_driver = buf[0x01] & DRIVER_VCP;
        eeprom->high_current_b   = buf[0x01] & HIGH_CURRENT_DRIVE;
        eeprom->chip = buf[0x14];
    }
    else if (ftdi->type == TYPE_R)
    {
        /* TYPE_R flags D2XX, not VCP as all others*/
        eeprom->channel_a_driver = ~buf[0x00] & DRIVER_VCP;
        eeprom->high_current     = buf[0x00] & HIGH_CURRENT_DRIVE_R;
        eeprom->external_oscillator = buf[0x00] & 0x02;
        if ( (buf[0x01]&0x40) != 0x40)
            printk(KERN_ERR 
                    "TYPE_R EEPROM byte[0x01] Bit 6 unexpected Endpoint size."
                    " If this happened with the\n"
                    " EEPROM programmed by FTDI tools, please report "
                    "to libftdi@developer.intra2net.com\n");

        eeprom->chip = buf[0x16];
        // Addr 0B: Invert data lines
        // Works only on FT232R, not FT245R, but no way to distinguish
        eeprom->invert = buf[0x0B];
        // Addr 14: CBUS function: CBUS0, CBUS1
        // Addr 15: CBUS function: CBUS2, CBUS3
        // Addr 16: CBUS function: CBUS5
        eeprom->cbus_function[0] = buf[0x14] & 0x0f;
        eeprom->cbus_function[1] = (buf[0x14] >> 4) & 0x0f;
        eeprom->cbus_function[2] = buf[0x15] & 0x0f;
        eeprom->cbus_function[3] = (buf[0x15] >> 4) & 0x0f;
        eeprom->cbus_function[4] = buf[0x16] & 0x0f;
    }
    else if ((ftdi->type == TYPE_2232H) || (ftdi->type == TYPE_4232H))
    {
        eeprom->channel_a_driver = buf[0x00] & DRIVER_VCP;
        eeprom->channel_b_driver = buf[0x01] & DRIVER_VCP;

        if (ftdi->type == TYPE_2232H)
        {
            eeprom->channel_a_type   = bit2type(buf[0x00] & 0x7);
            eeprom->channel_b_type   = bit2type(buf[0x01] & 0x7);
            eeprom->suspend_dbus7    = buf[0x01] & SUSPEND_DBUS7_BIT;
        }
        else
        {
            eeprom->channel_c_driver = (buf[0x00] >> 4) & DRIVER_VCP;
            eeprom->channel_d_driver = (buf[0x01] >> 4) & DRIVER_VCP;
            eeprom->channel_a_rs485enable = buf[0x0b] & (CHANNEL_IS_RS485 << 0);
            eeprom->channel_b_rs485enable = buf[0x0b] & (CHANNEL_IS_RS485 << 1);
            eeprom->channel_c_rs485enable = buf[0x0b] & (CHANNEL_IS_RS485 << 2);
            eeprom->channel_d_rs485enable = buf[0x0b] & (CHANNEL_IS_RS485 << 3);
        }

        eeprom->chip = buf[0x18];
        eeprom->group0_drive   =  buf[0x0c]       & DRIVE_16MA;
        eeprom->group0_schmitt =  buf[0x0c]       & IS_SCHMITT;
        eeprom->group0_slew    =  buf[0x0c]       & SLOW_SLEW;
        eeprom->group1_drive   = (buf[0x0c] >> 4) & 0x3;
        eeprom->group1_schmitt = (buf[0x0c] >> 4) & IS_SCHMITT;
        eeprom->group1_slew    = (buf[0x0c] >> 4) & SLOW_SLEW;
        eeprom->group2_drive   =  buf[0x0d]       & DRIVE_16MA;
        eeprom->group2_schmitt =  buf[0x0d]       & IS_SCHMITT;
        eeprom->group2_slew    =  buf[0x0d]       & SLOW_SLEW;
        eeprom->group3_drive   = (buf[0x0d] >> 4) & DRIVE_16MA;
        eeprom->group3_schmitt = (buf[0x0d] >> 4) & IS_SCHMITT;
        eeprom->group3_slew    = (buf[0x0d] >> 4) & SLOW_SLEW;
    }
    else if (ftdi->type == TYPE_232H)
    {
        eeprom->channel_a_type   = buf[0x00] & 0xf;
        eeprom->channel_a_driver = (buf[0x00] & DRIVER_VCPH)?DRIVER_VCP:0;
        eeprom->clock_polarity =  buf[0x01]       & FT1284_CLK_IDLE_STATE;
        eeprom->data_order     =  buf[0x01]       & FT1284_DATA_LSB;
        eeprom->flow_control   =  buf[0x01]       & FT1284_FLOW_CONTROL;
        eeprom->powersave      =  buf[0x01]       & POWER_SAVE_DISABLE_H;
        eeprom->group0_drive   =  buf[0x0c]       & DRIVE_16MA;
        eeprom->group0_schmitt =  buf[0x0c]       & IS_SCHMITT;
        eeprom->group0_slew    =  buf[0x0c]       & SLOW_SLEW;
        eeprom->group1_drive   =  buf[0x0d]       & DRIVE_16MA;
        eeprom->group1_schmitt =  buf[0x0d]       & IS_SCHMITT;
        eeprom->group1_slew    =  buf[0x0d]       & SLOW_SLEW;

        for(i=0; i<5; i++)
        {
            eeprom->cbus_function[2*i  ] =  buf[0x18+i] & 0x0f;
            eeprom->cbus_function[2*i+1] = (buf[0x18+i] >> 4) & 0x0f;
        }
        eeprom->chip = buf[0x1e];
        /*FIXME: Decipher more values*/
    }
    else if (ftdi->type == TYPE_230X)
    {
        for(i=0; i<4; i++)
        {
            eeprom->cbus_function[i] =  buf[0x1a + i] & 0xFF;
        }
        eeprom->group0_drive   =  buf[0x0c]       & 0x03;
        eeprom->group0_schmitt =  buf[0x0c]       & IS_SCHMITT;
        eeprom->group0_slew    =  buf[0x0c]       & SLOW_SLEW;
        eeprom->group1_drive   = (buf[0x0c] >> 4) & 0x03;
        eeprom->group1_schmitt = (buf[0x0c] >> 4) & IS_SCHMITT;
        eeprom->group1_slew    = (buf[0x0c] >> 4) & SLOW_SLEW;

        eeprom->invert = buf[0xb];
    }

    if (verbose)
    {
        const char *channel_mode[] = {"UART", "FIFO", "CPU", "OPTO", "FT1284"};
        printk(KERN_INFO "VID:     0x%04x\n",eeprom->vendor_id);
        printk(KERN_INFO "PID:     0x%04x\n",eeprom->product_id);
        printk(KERN_INFO "Release: 0x%04x\n",eeprom->release_number);

        if (eeprom->self_powered)
            printk(KERN_INFO "Self-Powered%s", (eeprom->remote_wakeup)?", USB Remote Wake Up\n":"\n");
        else
            printk(KERN_INFO "Bus Powered: %3d mA%s", eeprom->max_power,
                    (eeprom->remote_wakeup)?" USB Remote Wake Up\n":"\n");
        if (eeprom->manufacturer)
            printk(KERN_INFO "Manufacturer: %s\n",eeprom->manufacturer);
        if (eeprom->product)
            printk(KERN_INFO "Product:      %s\n",eeprom->product);
        if (eeprom->serial)
            printk(KERN_INFO "Serial:       %s\n",eeprom->serial);
        printk(KERN_INFO     "Checksum      : %04x\n", checksum);
        if (ftdi->type == TYPE_R) {
            printk(KERN_INFO     "Internal EEPROM\n");
            printk(KERN_INFO"Oscillator: %s\n", eeprom->external_oscillator?"External":"Internal");
        }
        else if (eeprom->chip >= 0x46)
            printk(KERN_INFO     "Attached EEPROM: 93x%02x\n", eeprom->chip);
        if (eeprom->suspend_dbus7)
            printk(KERN_INFO "Suspend on DBUS7\n");
        if (eeprom->suspend_pull_downs)
            printk(KERN_INFO "Pull IO pins low during suspend\n");
        if(eeprom->powersave)
        {
            if(ftdi->type >= TYPE_232H)
                printk(KERN_INFO"Enter low power state on ACBUS7\n");
        }
        if (eeprom->remote_wakeup)
            printk(KERN_INFO "Enable Remote Wake Up\n");
        printk(KERN_INFO "PNP: %d\n",(eeprom->is_not_pnp)?0:1);
        if (ftdi->type >= TYPE_2232C)
            printk(KERN_INFO"Channel A has Mode %s%s%s\n",
                    channel_mode[eeprom->channel_a_type],
                    (eeprom->channel_a_driver)?" VCP":"",
                    (eeprom->high_current_a)?" High Current IO":"");
        if (ftdi->type == TYPE_232H)
        {
            printk(KERN_INFO"FT1284 Mode Clock is idle %s, %s first, %sFlow Control\n",
                    (eeprom->clock_polarity)?"HIGH":"LOW",
                    (eeprom->data_order)?"LSB":"MSB",
                    (eeprom->flow_control)?"":"No ");
        }
        if ((ftdi->type == TYPE_2232H) || (ftdi->type == TYPE_4232H))
            printk(KERN_INFO"Channel B has Mode %s%s%s\n",
                    channel_mode[eeprom->channel_b_type],
                    (eeprom->channel_b_driver)?" VCP":"",
                    (eeprom->high_current_b)?" High Current IO":"");
        if (((ftdi->type == TYPE_BM) || (ftdi->type == TYPE_2232C)) &&
                eeprom->use_usb_version)
            printk(KERN_INFO"Use explicit USB Version %04x\n",eeprom->usb_version);

        if ((ftdi->type == TYPE_2232H) || (ftdi->type == TYPE_4232H))
        {
            printk(KERN_INFO"%s has %d mA drive%s%s\n",
                    (ftdi->type == TYPE_2232H)?"AL":"A",
                    (eeprom->group0_drive+1) *4,
                    (eeprom->group0_schmitt)?" Schmitt Input":"",
                    (eeprom->group0_slew)?" Slow Slew":"");
            printk(KERN_INFO"%s has %d mA drive%s%s\n",
                    (ftdi->type == TYPE_2232H)?"AH":"B",
                    (eeprom->group1_drive+1) *4,
                    (eeprom->group1_schmitt)?" Schmitt Input":"",
                    (eeprom->group1_slew)?" Slow Slew":"");
            printk(KERN_INFO"%s has %d mA drive%s%s\n",
                    (ftdi->type == TYPE_2232H)?"BL":"C",
                    (eeprom->group2_drive+1) *4,
                    (eeprom->group2_schmitt)?" Schmitt Input":"",
                    (eeprom->group2_slew)?" Slow Slew":"");
            printk(KERN_INFO"%s has %d mA drive%s%s\n",
                    (ftdi->type == TYPE_2232H)?"BH":"D",
                    (eeprom->group3_drive+1) *4,
                    (eeprom->group3_schmitt)?" Schmitt Input":"",
                    (eeprom->group3_slew)?" Slow Slew":"");
        }
        else if (ftdi->type == TYPE_232H)
        {
            const char *cbush_mux[] = {"TRISTATE","TXLED","RXLED", "TXRXLED","PWREN",
                                 "SLEEP","DRIVE_0","DRIVE_1","IOMODE","TXDEN",
                                 "CLK30","CLK15","CLK7_5"
                                };
            printk(KERN_INFO"ACBUS has %d mA drive%s%s\n",
                    (eeprom->group0_drive+1) *4,
                    (eeprom->group0_schmitt)?" Schmitt Input":"",
                    (eeprom->group0_slew)?" Slow Slew":"");
            printk(KERN_INFO"ADBUS has %d mA drive%s%s\n",
                    (eeprom->group1_drive+1) *4,
                    (eeprom->group1_schmitt)?" Schmitt Input":"",
                    (eeprom->group1_slew)?" Slow Slew":"");
            for (i=0; i<10; i++)
            {
                if (eeprom->cbus_function[i]<= CBUSH_CLK7_5 )
                    printk(KERN_INFO"C%d Function: %s\n", i,
                            cbush_mux[eeprom->cbus_function[i]]);
            }
        }
        else if (ftdi->type == TYPE_230X)
        {
            const char *cbusx_mux[] = {"TRISTATE","TXLED","RXLED", "TXRXLED","PWREN",
                                 "SLEEP","DRIVE_0","DRIVE_1","IOMODE","TXDEN",
                                 "CLK24","CLK12","CLK6","BAT_DETECT","BAT_DETECT#",
                                 "I2C_TXE#", "I2C_RXF#", "VBUS_SENSE", "BB_WR#",
                                 "BBRD#", "TIME_STAMP", "AWAKE#",
                                };
            printk(KERN_INFO"DBUS has %d mA drive%s%s\n",
                    (eeprom->group0_drive+1) *4,
                    (eeprom->group0_schmitt)?" Schmitt Input":"",
                    (eeprom->group0_slew)?" Slow Slew":"");
            printk(KERN_INFO"CBUS has %d mA drive%s%s\n",
                    (eeprom->group1_drive+1) *4,
                    (eeprom->group1_schmitt)?" Schmitt Input":"",
                    (eeprom->group1_slew)?" Slow Slew":"");
            for (i=0; i<4; i++)
            {
                if (eeprom->cbus_function[i]<= CBUSX_AWAKE)
                    printk(KERN_INFO"CBUS%d Function: %s\n", i, cbusx_mux[eeprom->cbus_function[i]]);
            }

            if (eeprom->invert)
                print_inverted_bits(eeprom->invert);
        }

        if (ftdi->type == TYPE_R)
        {
            const char *cbus_mux[] = {"TXDEN","PWREN","RXLED", "TXLED","TX+RXLED",
                                "SLEEP","CLK48","CLK24","CLK12","CLK6",
                                "IOMODE","BB_WR","BB_RD"
                               };
            const char *cbus_BB[] = {"RXF","TXE","RD", "WR"};

            if (eeprom->invert)
                print_inverted_bits(eeprom->invert);

            for (i=0; i<5; i++)
            {
                if (eeprom->cbus_function[i]<=CBUS_BB_RD)
                    printk(KERN_INFO"C%d Function: %s\n", i,
                            cbus_mux[eeprom->cbus_function[i]]);
                else
                {
                    if (i < 4)
                        /* Running MPROG show that C0..3 have fixed function Synchronous
                           Bit Bang mode */
                        printk(KERN_INFO"C%d BB Function: %s\n", i,
                                cbus_BB[i]);
                    else
                        printk(KERN_INFO "Unknown CBUS mode. Might be special mode?\n");
                }
            }
        }
    }
    return 0;
}

/**
   Get a value from the decoded EEPROM structure

   \param ftdi pointer to ftdi_context
   \param value_name Enum of the value to query
   \param value Pointer to store read value

   \retval 0: all fine
   \retval -1: Value doesn't exist
*/
int ftdi_get_eeprom_value(struct ftdi_context *ftdi, enum ftdi_eeprom_value value_name, int* value)
{
    switch (value_name)
    {
        case VENDOR_ID:
            *value = ftdi->eeprom->vendor_id;
            break;
        case PRODUCT_ID:
            *value = ftdi->eeprom->product_id;
            break;
        case RELEASE_NUMBER:
            *value = ftdi->eeprom->release_number;
            break;
        case SELF_POWERED:
            *value = ftdi->eeprom->self_powered;
            break;
        case REMOTE_WAKEUP:
            *value = ftdi->eeprom->remote_wakeup;
            break;
        case IS_NOT_PNP:
            *value = ftdi->eeprom->is_not_pnp;
            break;
        case SUSPEND_DBUS7:
            *value = ftdi->eeprom->suspend_dbus7;
            break;
        case IN_IS_ISOCHRONOUS:
            *value = ftdi->eeprom->in_is_isochronous;
            break;
        case OUT_IS_ISOCHRONOUS:
            *value = ftdi->eeprom->out_is_isochronous;
            break;
        case SUSPEND_PULL_DOWNS:
            *value = ftdi->eeprom->suspend_pull_downs;
            break;
        case USE_SERIAL:
            *value = ftdi->eeprom->use_serial;
            break;
        case USB_VERSION:
            *value = ftdi->eeprom->usb_version;
            break;
        case USE_USB_VERSION:
            *value = ftdi->eeprom->use_usb_version;
            break;
        case MAX_POWER:
            *value = ftdi->eeprom->max_power;
            break;
        case CHANNEL_A_TYPE:
            *value = ftdi->eeprom->channel_a_type;
            break;
        case CHANNEL_B_TYPE:
            *value = ftdi->eeprom->channel_b_type;
            break;
        case CHANNEL_A_DRIVER:
            *value = ftdi->eeprom->channel_a_driver;
            break;
        case CHANNEL_B_DRIVER:
            *value = ftdi->eeprom->channel_b_driver;
            break;
        case CHANNEL_C_DRIVER:
            *value = ftdi->eeprom->channel_c_driver;
            break;
        case CHANNEL_D_DRIVER:
            *value = ftdi->eeprom->channel_d_driver;
            break;
        case CHANNEL_A_RS485:
            *value = ftdi->eeprom->channel_a_rs485enable;
            break;
        case CHANNEL_B_RS485:
            *value = ftdi->eeprom->channel_b_rs485enable;
            break;
        case CHANNEL_C_RS485:
            *value = ftdi->eeprom->channel_c_rs485enable;
            break;
        case CHANNEL_D_RS485:
            *value = ftdi->eeprom->channel_d_rs485enable;
            break;
        case CBUS_FUNCTION_0:
            *value = ftdi->eeprom->cbus_function[0];
            break;
        case CBUS_FUNCTION_1:
            *value = ftdi->eeprom->cbus_function[1];
            break;
        case CBUS_FUNCTION_2:
            *value = ftdi->eeprom->cbus_function[2];
            break;
        case CBUS_FUNCTION_3:
            *value = ftdi->eeprom->cbus_function[3];
            break;
        case CBUS_FUNCTION_4:
            *value = ftdi->eeprom->cbus_function[4];
            break;
        case CBUS_FUNCTION_5:
            *value = ftdi->eeprom->cbus_function[5];
            break;
        case CBUS_FUNCTION_6:
            *value = ftdi->eeprom->cbus_function[6];
            break;
        case CBUS_FUNCTION_7:
            *value = ftdi->eeprom->cbus_function[7];
            break;
        case CBUS_FUNCTION_8:
            *value = ftdi->eeprom->cbus_function[8];
            break;
        case CBUS_FUNCTION_9:
            *value = ftdi->eeprom->cbus_function[9];
            break;
        case HIGH_CURRENT:
            *value = ftdi->eeprom->high_current;
            break;
        case HIGH_CURRENT_A:
            *value = ftdi->eeprom->high_current_a;
            break;
        case HIGH_CURRENT_B:
            *value = ftdi->eeprom->high_current_b;
            break;
        case INVERT:
            *value = ftdi->eeprom->invert;
            break;
        case GROUP0_DRIVE:
            *value = ftdi->eeprom->group0_drive;
            break;
        case GROUP0_SCHMITT:
            *value = ftdi->eeprom->group0_schmitt;
            break;
        case GROUP0_SLEW:
            *value = ftdi->eeprom->group0_slew;
            break;
        case GROUP1_DRIVE:
            *value = ftdi->eeprom->group1_drive;
            break;
        case GROUP1_SCHMITT:
            *value = ftdi->eeprom->group1_schmitt;
            break;
        case GROUP1_SLEW:
            *value = ftdi->eeprom->group1_slew;
            break;
        case GROUP2_DRIVE:
            *value = ftdi->eeprom->group2_drive;
            break;
        case GROUP2_SCHMITT:
            *value = ftdi->eeprom->group2_schmitt;
            break;
        case GROUP2_SLEW:
            *value = ftdi->eeprom->group2_slew;
            break;
        case GROUP3_DRIVE:
            *value = ftdi->eeprom->group3_drive;
            break;
        case GROUP3_SCHMITT:
            *value = ftdi->eeprom->group3_schmitt;
            break;
        case GROUP3_SLEW:
            *value = ftdi->eeprom->group3_slew;
            break;
        case POWER_SAVE:
            *value = ftdi->eeprom->powersave;
            break;
        case CLOCK_POLARITY:
            *value = ftdi->eeprom->clock_polarity;
            break;
        case DATA_ORDER:
            *value = ftdi->eeprom->data_order;
            break;
        case FLOW_CONTROL:
            *value = ftdi->eeprom->flow_control;
            break;
        case CHIP_TYPE:
            *value = ftdi->eeprom->chip;
            break;
        case CHIP_SIZE:
            *value = ftdi->eeprom->size;
            break;
        case EXTERNAL_OSCILLATOR:
            *value = ftdi->eeprom->external_oscillator;
            break;
        default:
            ftdi_error_return(-1, "Request for unknown EEPROM value");
    }
    return 0;
}

/**
   Set a value in the decoded EEPROM Structure
   No parameter checking is performed

   \param ftdi pointer to ftdi_context
   \param value_name Enum of the value to set
   \param value to set

   \retval 0: all fine
   \retval -1: Value doesn't exist
   \retval -2: Value not user settable
*/
int ftdi_set_eeprom_value(struct ftdi_context *ftdi, enum ftdi_eeprom_value value_name, int value)
{
    switch (value_name)
    {
        case VENDOR_ID:
            ftdi->eeprom->vendor_id = value;
            break;
        case PRODUCT_ID:
            ftdi->eeprom->product_id = value;
            break;
        case RELEASE_NUMBER:
            ftdi->eeprom->release_number = value;
            break;
        case SELF_POWERED:
            ftdi->eeprom->self_powered = value;
            break;
        case REMOTE_WAKEUP:
            ftdi->eeprom->remote_wakeup = value;
            break;
        case IS_NOT_PNP:
            ftdi->eeprom->is_not_pnp = value;
            break;
        case SUSPEND_DBUS7:
            ftdi->eeprom->suspend_dbus7 = value;
            break;
        case IN_IS_ISOCHRONOUS:
            ftdi->eeprom->in_is_isochronous = value;
            break;
        case OUT_IS_ISOCHRONOUS:
            ftdi->eeprom->out_is_isochronous = value;
            break;
        case SUSPEND_PULL_DOWNS:
            ftdi->eeprom->suspend_pull_downs = value;
            break;
        case USE_SERIAL:
            ftdi->eeprom->use_serial = value;
            break;
        case USB_VERSION:
            ftdi->eeprom->usb_version = value;
            break;
        case USE_USB_VERSION:
            ftdi->eeprom->use_usb_version = value;
            break;
        case MAX_POWER:
            ftdi->eeprom->max_power = value;
            break;
        case CHANNEL_A_TYPE:
            ftdi->eeprom->channel_a_type = value;
            break;
        case CHANNEL_B_TYPE:
            ftdi->eeprom->channel_b_type = value;
            break;
        case CHANNEL_A_DRIVER:
            ftdi->eeprom->channel_a_driver = value;
            break;
        case CHANNEL_B_DRIVER:
            ftdi->eeprom->channel_b_driver = value;
            break;
        case CHANNEL_C_DRIVER:
            ftdi->eeprom->channel_c_driver = value;
            break;
        case CHANNEL_D_DRIVER:
            ftdi->eeprom->channel_d_driver = value;
            break;
        case CHANNEL_A_RS485:
            ftdi->eeprom->channel_a_rs485enable = value;
            break;
        case CHANNEL_B_RS485:
            ftdi->eeprom->channel_b_rs485enable = value;
            break;
        case CHANNEL_C_RS485:
            ftdi->eeprom->channel_c_rs485enable = value;
            break;
        case CHANNEL_D_RS485:
            ftdi->eeprom->channel_d_rs485enable = value;
            break;
        case CBUS_FUNCTION_0:
            ftdi->eeprom->cbus_function[0] = value;
            break;
        case CBUS_FUNCTION_1:
            ftdi->eeprom->cbus_function[1] = value;
            break;
        case CBUS_FUNCTION_2:
            ftdi->eeprom->cbus_function[2] = value;
            break;
        case CBUS_FUNCTION_3:
            ftdi->eeprom->cbus_function[3] = value;
            break;
        case CBUS_FUNCTION_4:
            ftdi->eeprom->cbus_function[4] = value;
            break;
        case CBUS_FUNCTION_5:
            ftdi->eeprom->cbus_function[5] = value;
            break;
        case CBUS_FUNCTION_6:
            ftdi->eeprom->cbus_function[6] = value;
            break;
        case CBUS_FUNCTION_7:
            ftdi->eeprom->cbus_function[7] = value;
            break;
        case CBUS_FUNCTION_8:
            ftdi->eeprom->cbus_function[8] = value;
            break;
        case CBUS_FUNCTION_9:
            ftdi->eeprom->cbus_function[9] = value;
            break;
        case HIGH_CURRENT:
            ftdi->eeprom->high_current = value;
            break;
        case HIGH_CURRENT_A:
            ftdi->eeprom->high_current_a = value;
            break;
        case HIGH_CURRENT_B:
            ftdi->eeprom->high_current_b = value;
            break;
        case INVERT:
            ftdi->eeprom->invert = value;
            break;
        case GROUP0_DRIVE:
            ftdi->eeprom->group0_drive = value;
            break;
        case GROUP0_SCHMITT:
            ftdi->eeprom->group0_schmitt = value;
            break;
        case GROUP0_SLEW:
            ftdi->eeprom->group0_slew = value;
            break;
        case GROUP1_DRIVE:
            ftdi->eeprom->group1_drive = value;
            break;
        case GROUP1_SCHMITT:
            ftdi->eeprom->group1_schmitt = value;
            break;
        case GROUP1_SLEW:
            ftdi->eeprom->group1_slew = value;
            break;
        case GROUP2_DRIVE:
            ftdi->eeprom->group2_drive = value;
            break;
        case GROUP2_SCHMITT:
            ftdi->eeprom->group2_schmitt = value;
            break;
        case GROUP2_SLEW:
            ftdi->eeprom->group2_slew = value;
            break;
        case GROUP3_DRIVE:
            ftdi->eeprom->group3_drive = value;
            break;
        case GROUP3_SCHMITT:
            ftdi->eeprom->group3_schmitt = value;
            break;
        case GROUP3_SLEW:
            ftdi->eeprom->group3_slew = value;
            break;
        case CHIP_TYPE:
            ftdi->eeprom->chip = value;
            break;
        case POWER_SAVE:
            ftdi->eeprom->powersave = value;
            break;
        case CLOCK_POLARITY:
            ftdi->eeprom->clock_polarity = value;
            break;
        case DATA_ORDER:
            ftdi->eeprom->data_order = value;
            break;
        case FLOW_CONTROL:
            ftdi->eeprom->flow_control = value;
            break;
        case CHIP_SIZE:
            ftdi_error_return(-2, "EEPROM Value can't be changed");
            break;
        case EXTERNAL_OSCILLATOR:
            ftdi->eeprom->external_oscillator = value;
            break;
        case USER_DATA_ADDR:
            ftdi->eeprom->user_data_addr = value;
            break;

        default :
            ftdi_error_return(-1, "Request to unknown EEPROM value");
    }
    ftdi->eeprom->initialized_for_connected_device = 0;
    return 0;
}

/** Get the read-only buffer to the binary EEPROM content

    \param ftdi pointer to ftdi_context
    \param buf buffer to receive EEPROM content
    \param size Size of receiving buffer

    \retval 0: All fine
    \retval -1: struct ftdi_contxt or ftdi_eeprom missing
    \retval -2: Not enough room to store eeprom
*/
int ftdi_get_eeprom_buf(struct ftdi_context *ftdi, unsigned char * buf, int size)
{
    if (!ftdi || !(ftdi->eeprom))
        ftdi_error_return(-1, "No appropriate structure");

    if (!buf || size < ftdi->eeprom->size)
        ftdi_error_return(-1, "Not enough room to store eeprom");

    // Only copy up to FTDI_MAX_EEPROM_SIZE bytes
    if (size > FTDI_MAX_EEPROM_SIZE)
        size = FTDI_MAX_EEPROM_SIZE;

    memcpy(buf, ftdi->eeprom->buf, size);

    return 0;
}

/** Set the EEPROM content from the user-supplied prefilled buffer

    \param ftdi pointer to ftdi_context
    \param buf buffer to read EEPROM content
    \param size Size of buffer

    \retval 0: All fine
    \retval -1: struct ftdi_context or ftdi_eeprom or buf missing
*/
int ftdi_set_eeprom_buf(struct ftdi_context *ftdi, const unsigned char * buf, int size)
{
    if (!ftdi || !(ftdi->eeprom) || !buf)
        ftdi_error_return(-1, "No appropriate structure");

    // Only copy up to FTDI_MAX_EEPROM_SIZE bytes
    if (size > FTDI_MAX_EEPROM_SIZE)
        size = FTDI_MAX_EEPROM_SIZE;

    memcpy(ftdi->eeprom->buf, buf, size);

    return 0;
}

/** Set the EEPROM user data content from the user-supplied prefilled buffer

    \param ftdi pointer to ftdi_context
    \param buf buffer to read EEPROM user data content
    \param size Size of buffer

    \retval 0: All fine
    \retval -1: struct ftdi_context or ftdi_eeprom or buf missing
*/
int ftdi_set_eeprom_user_data(struct ftdi_context *ftdi, const char * buf, int size)
{
    if (!ftdi || !(ftdi->eeprom) || !buf)
        ftdi_error_return(-1, "No appropriate structure");

    ftdi->eeprom->user_data_size = size;
    ftdi->eeprom->user_data = buf;
    return 0;
}

/**
    Read eeprom location

    \param ftdi pointer to ftdi_context
    \param eeprom_addr Address of eeprom location to be read
    \param eeprom_val Pointer to store read eeprom location

    \retval  0: all fine
    \retval -1: read failed
    \retval -2: USB device unavailable
*/
int ftdi_read_eeprom_location (struct ftdi_context *ftdi, int eeprom_addr, unsigned short *eeprom_val)
{
    unsigned char buf[2];

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_READ_EEPROM_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, eeprom_addr, buf, 2, ftdi->usb_read_timeout) != 2)
        ftdi_error_return(-1, "reading eeprom failed");

    *eeprom_val = (0xff & buf[0]) | (buf[1] << 8);

    return 0;
}

/**
    Read eeprom

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: read failed
    \retval -2: USB device unavailable
*/
int ftdi_read_eeprom(struct ftdi_context *ftdi)
{
    int i;
    unsigned char *buf;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");
    buf = ftdi->eeprom->buf;

    for (i = 0; i < FTDI_MAX_EEPROM_SIZE/2; i++)
    {
        if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_READ_EEPROM_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, i,
                    buf+(i*2), 2, ftdi->usb_read_timeout) != 2)
            ftdi_error_return(-1, "reading eeprom failed");
    }

    if (ftdi->type == TYPE_R)
        ftdi->eeprom->size = 0x80;
    /*    Guesses size of eeprom by comparing halves
          - will not work with blank eeprom */
    else if (strrchr((const char *)buf, 0xff) == ((const char *)buf +FTDI_MAX_EEPROM_SIZE -1))
        ftdi->eeprom->size = -1;
    else if (memcmp(buf,&buf[0x80],0x80) == 0)
        ftdi->eeprom->size = 0x80;
    else if (memcmp(buf,&buf[0x40],0x40) == 0)
        ftdi->eeprom->size = 0x40;
    else
        ftdi->eeprom->size = 0x100;
    return 0;
}

/*
    ftdi_read_chipid_shift does the bitshift operation needed for the FTDIChip-ID
    Function is only used internally
    \internal
*/
static unsigned char ftdi_read_chipid_shift(unsigned char value)
{
    return ((value & 1) << 1) |
           ((value & 2) << 5) |
           ((value & 4) >> 2) |
           ((value & 8) << 4) |
           ((value & 16) >> 1) |
           ((value & 32) >> 1) |
           ((value & 64) >> 4) |
           ((value & 128) >> 2);
}

/**
    Read the FTDIChip-ID from R-type devices

    \param ftdi pointer to ftdi_context
    \param chipid Pointer to store FTDIChip-ID

    \retval  0: all fine
    \retval -1: read failed
    \retval -2: USB device unavailable
*/
int ftdi_read_chipid(struct ftdi_context *ftdi, unsigned int *chipid)
{
    unsigned int a = 0, b = 0;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_READ_EEPROM_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, 0x43, (unsigned char *)&a, 2, ftdi->usb_read_timeout) == 2)
    {
        a = a << 8 | a >> 8;
        if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_READ_EEPROM_REQUEST, FTDI_DEVICE_IN_REQTYPE, 0, 0x44, (unsigned char *)&b, 2, ftdi->usb_read_timeout) == 2)
        {
            b = b << 8 | b >> 8;
            a = (a << 16) | (b & 0xFFFF);
            a = ftdi_read_chipid_shift(a) | ftdi_read_chipid_shift(a>>8)<<8
                | ftdi_read_chipid_shift(a>>16)<<16 | ftdi_read_chipid_shift(a>>24)<<24;
            *chipid = a ^ 0xa5f0f7d1;
            return 0;
        }
    }

    ftdi_error_return(-1, "read of FTDIChip-ID failed");
}

/**
    Write eeprom location

    \param ftdi pointer to ftdi_context
    \param eeprom_addr Address of eeprom location to be written
    \param eeprom_val Value to be written

    \retval  0: all fine
    \retval -1: write failed
    \retval -2: USB device unavailable
    \retval -3: Invalid access to checksum protected area below 0x80
    \retval -4: Device can't access unprotected area
    \retval -5: Reading chip type failed
*/
int ftdi_write_eeprom_location(struct ftdi_context *ftdi, int eeprom_addr,
                               unsigned short eeprom_val)
{
    int chip_type_location;
    unsigned short chip_type;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if (eeprom_addr <0x80)
        ftdi_error_return(-2, "Invalid access to checksum protected area  below 0x80");


    switch (ftdi->type)
    {
        case TYPE_BM:
        case  TYPE_2232C:
            chip_type_location = 0x14;
            break;
        case TYPE_2232H:
        case TYPE_4232H:
            chip_type_location = 0x18;
            break;
        case TYPE_232H:
            chip_type_location = 0x1e;
            break;
        default:
            ftdi_error_return(-4, "Device can't access unprotected area");
    }

    if (ftdi_read_eeprom_location( ftdi, chip_type_location>>1, &chip_type))
        ftdi_error_return(-5, "Reading failed");
    printk(KERN_ERR " loc 0x%04x val 0x%04x\n", chip_type_location,chip_type);
    if ((chip_type & 0xff) != 0x66)
    {
        ftdi_error_return(-6, "EEPROM is not of 93x66");
    }

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_WRITE_EEPROM_REQUEST, FTDI_DEVICE_OUT_REQTYPE, eeprom_val, eeprom_addr,
                                NULL, 0, ftdi->usb_write_timeout) != 0)
        ftdi_error_return(-1, "unable to write eeprom");

    return 0;
}

/**
    Write eeprom

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: read failed
    \retval -2: USB device unavailable
    \retval -3: EEPROM not initialized for the connected device;
*/
int ftdi_write_eeprom(struct ftdi_context *ftdi)
{
    unsigned short usb_val, status;
    int i, ret;
    unsigned char *eeprom;

    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if(ftdi->eeprom->initialized_for_connected_device == 0)
        ftdi_error_return(-3, "EEPROM not initialized for the connected device");

    eeprom = ftdi->eeprom->buf;

    /* These commands were traced while running MProg */
    if ((ret = ftdi_usb_reset(ftdi)) != 0)
        return ret;
    if ((ret = ftdi_poll_modem_status(ftdi, &status)) != 0)
        return ret;
    if ((ret = ftdi_set_latency_timer(ftdi, 0x77)) != 0)
        return ret;

    for (i = 0; i < ftdi->eeprom->size/2; i++)
    {
        /* Do not try to write to reserved area */
        if ((ftdi->type == TYPE_230X) && (i == 0x40))
        {
            i = 0x50;
        }
        usb_val = eeprom[i*2];
        usb_val += eeprom[(i*2)+1] << 8;
        if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_WRITE_EEPROM_REQUEST, FTDI_DEVICE_OUT_REQTYPE, usb_val, i,
                                    NULL, 0, ftdi->usb_write_timeout) < 0)
            ftdi_error_return(-1, "unable to write eeprom");
    }

    return 0;
}

/**
    Erase eeprom

    This is not supported on FT232R/FT245R according to the MProg manual from FTDI.

    \param ftdi pointer to ftdi_context

    \retval  0: all fine
    \retval -1: erase failed
    \retval -2: USB device unavailable
    \retval -3: Writing magic failed
    \retval -4: Read EEPROM failed
    \retval -5: Unexpected EEPROM value
*/
#define MAGIC 0x55aa
int ftdi_erase_eeprom(struct ftdi_context *ftdi)
{
    unsigned short eeprom_value;
    if (ftdi == NULL || ftdi->usb_dev == NULL)
        ftdi_error_return(-2, "USB device unavailable");

    if ((ftdi->type == TYPE_R) || (ftdi->type == TYPE_230X))
    {
        ftdi->eeprom->chip = 0;
        return 0;
    }

    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_ERASE_EEPROM_REQUEST, FTDI_DEVICE_OUT_REQTYPE, 
                                0, 0, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "unable to erase eeprom");


    /* detect chip type by writing 0x55AA as magic at word position 0xc0
       Chip is 93x46 if magic is read at word position 0x00, as wraparound happens around 0x40
       Chip is 93x56 if magic is read at word position 0x40, as wraparound happens around 0x80
       Chip is 93x66 if magic is only read at word position 0xc0*/
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0),
                                SIO_WRITE_EEPROM_REQUEST, FTDI_DEVICE_OUT_REQTYPE, MAGIC, 0xc0,
                                NULL, 0, ftdi->usb_write_timeout) != 0)
        ftdi_error_return(-3, "Writing magic failed");
    if (ftdi_read_eeprom_location( ftdi, 0x00, &eeprom_value))
        ftdi_error_return(-4, "Reading failed");
    if (eeprom_value == MAGIC)
    {
        ftdi->eeprom->chip = 0x46;
    }
    else
    {
        if (ftdi_read_eeprom_location( ftdi, 0x40, &eeprom_value))
            ftdi_error_return(-4, "Reading failed");
        if (eeprom_value == MAGIC)
            ftdi->eeprom->chip = 0x56;
        else
        {
            if (ftdi_read_eeprom_location( ftdi, 0xc0, &eeprom_value))
                ftdi_error_return(-4, "Reading failed");
            if (eeprom_value == MAGIC)
                ftdi->eeprom->chip = 0x66;
            else
            {
                ftdi->eeprom->chip = -1;
            }
        }
    }
    if (usb_control_msg(ftdi->usb_dev, usb_sndctrlpipe(ftdi->usb_dev, 0), SIO_ERASE_EEPROM_REQUEST, FTDI_DEVICE_OUT_REQTYPE,
                                0, 0, NULL, 0, ftdi->usb_write_timeout) < 0)
        ftdi_error_return(-1, "unable to erase eeprom");
    return 0;
}

/**
    Get string representation for last error code

    \param ftdi pointer to ftdi_context

    \retval Pointer to error string
*/
const char *ftdi_get_error_string (struct ftdi_context *ftdi)
{
    if (ftdi == NULL)
        return "";

    return ftdi->error_str;
}

/* @} end of doxygen libftdi group */
