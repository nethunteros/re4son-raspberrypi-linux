/*
 * Framebuffer for 4D Systems displays
 * ili9481 and hx8357 drivers are supported
 * Copyright (c) 2015 Andrej Strancar
 *
 * Original: Copyright (c) 2009 Jean-Christian de Rivaz
 *
 * SPI mods, console support, 320x240 instead of 240x320:
 * Copyright (c) 2012 Jeroen Domburg <jeroen@spritesmods.com>
 *
 * Bits and pieces borrowed from the fsl-lcdpi.c:
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * Author: Alison Wang <b18965@freescale.com>
 *         Jason Jin <Jason.jin@freescale.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

//#define DEBUG

//#define CONFIG_FB_BACKLIGHT

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/fb.h>
#include <asm/io.h>
#include <linux/spi/spi.h>
#include <linux/pinctrl/consumer.h>
#include <linux/backlight.h>
#include <linux/dma-mapping.h>

#include <linux/version.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>

#include <linux/clk.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/semaphore.h>

#include <mach/platform.h>

/* ILI9341 commands */
#define ILI9341_READ_DISPLAY_PIXEL_FORMAT 0x0C
#define ILI9341_SLEEP_OUT 0x11
#define ILI9341_GAMMA_SET 0x26
#define ILI9341_DISPLAY_OFF 0x28
#define ILI9341_DISPLAY_ON 0x29
#define ILI9341_SET_COLUMN_ADDRESS 0x2A
#define ILI9341_SET_PAGE_ADDRESS 0x2B
#define ILI9341_WRITE_MEMORY 0x2C
#define ILI9341_READ_MEMORY 0x2E
#define ILI9341_MEMORY_ACCESS_CONTROL 0x36
#define ILI9341_WRITE_MEMORY_CONTINUE 0x3C
#define ILI9341_READ_MEMORY_CONTINUE 0x3E
#define ILI9341_PIXEL_FORMAT_SET 0x3A
#define ILI9341_FRAME_RATE_CONTROL 0xB1
#define ILI9341_DISPLAY_FUNCTION_CONTROL 0xB6
#define ILI9341_POWER_CONTROL_1 0xC0
#define ILI9341_POWER_CONTROL_2 0xC1
#define ILI9341_VCOM_CONTROL_1 0xC5
#define ILI9341_VCOM_CONTROL_2 0xC7
#define ILI9341_POWER_CONTROL_A 0xCB
#define ILI9341_POWER_CONTROL_B 0xCF
#define ILI9341_POSITIVE_GAMMA_CORRECTION 0xE0
#define ILI9341_NEGATIVE_GAMMA_CORRECTION 0xE1
#define ILI9341_DRIVER_TIMING_CONTROL_A 0xE8
#define ILI9341_DRIVER_TIMING_CONTROL_B 0xEA
#define ILI9341_POWER_ON_SEQUENCE_CONTROL 0xED
#define ILI9341_UNDOCUMENTED_0xEF 0xEF
#define ILI9341_ENABLE_3G 0xF2
#define ILI9341_INTERFACE_CONTROL 0xF6
#define ILI9341_DPUMP_RATIO_CONTROL 0xF7

/* HX8357 commands */
#define HX8357_EXIT_SLEEP_MODE          0x11
#define HX8357_SET_DISPLAY_OFF          0x28
#define HX8357_SET_DISPLAY_ON           0x29
#define HX8357_SET_COLUMN_ADDRESS       0x2a
#define HX8357_SET_PAGE_ADDRESS         0x2b
#define HX8357_WRITE_MEMORY_START       0x2c
#define HX8357_READ_MEMORY_START        0x2e
#define HX8357_SET_TEAR_ON              0x35
#define HX8357_SET_ADDRESS_MODE         0x36
#define HX8357_SET_PIXEL_FORMAT         0x3a
#define HX8357_WRITE_MEMORY_CONTINUE    0x3c
#define HX8357_READ_MEMORY_CONTINUE     0x3e
#define HX8357_SET_INTERNAL_OSCILLATOR   0xB0
#define HX8357_SET_POWER_CONTROL         0xB1
#define HX8357_SET_DISPLAY_MODE          0xb4
#define HX8357_SET_VCOM_VOLTAGE         0xB6
#define HX8357_ENABLE_EXTENSION_COMMAND 0xB9
#define HX8357_SET_PANEL_DRIVING        0xc0    // not documented!
#define HX8357_SET_PANEL_CHARACTERISTIC  0xCC
#define HX8357_SET_GAMMA_CURVE           0xe0


enum lcdpi_model { HAT35 = 0xab, HAT32, HAT24 };

struct lcdpi_page {
    unsigned short x;
    unsigned short y;
    unsigned short *buffer;
    unsigned short len;
    int must_update;
};

struct lcdpi {
    struct device *dev;
    spinlock_t spi_lock;
    struct spi_device *spidev;
    struct fb_info *info;
    unsigned int pages_count;
    struct lcdpi_page *pages;
    unsigned long pseudo_palette[17];
    int backlight;
    int brightness;
    int update_brightness;
};

struct spiblock_dma {
    u8 *buf;
    dma_addr_t dma;
    size_t len;
};

/* touchscreen parameters */
static u16 touchscreens[][4] = {
    {0x07, 0x01, 0x04, 0x02}, // 24-HAT
    {0x04, 0x02, 0x07, 0x01}, // 32-HAT
    {0x07, 0x01, 0x04, 0x02}, // 35-HAT
};

/* touchscreen parameters form DT */
static u8 rotate_codes[4] = {0xff, 0xff, 0xff, 0xff };

/* module parameters */
static int rotate = 360;
module_param(rotate, int, 0444);
MODULE_PARM_DESC(rotate, "Screen rotation: 0/90/180/270");

static int compress = 1;
module_param(compress, int, 0444);
MODULE_PARM_DESC(compress, "SPI compresion 1/0");

static ushort product_code = 0;
module_param(product_code, ushort, 0444);
MODULE_PARM_DESC(product_code, "Force product code");

static ulong sclk = 48000000;
module_param(sclk, ulong, 0444);
MODULE_PARM_DESC(sclk, "SPI clock frequency");

static ulong frames = 0;
module_param(frames, ulong, 0444);
MODULE_PARM_DESC(frames, "Count of drawn frames");

/* exports */
u16 lcdpi_touchparams;
EXPORT_SYMBOL(lcdpi_touchparams);

/* horizontal and vertical resolution */
static u16 HDP=320;
static u16 VDP=240;

/* LCD width and height, depend on rotation! */
static u16 width;
static u16 height;

/* ILI9341 init sequences for 24-HAT */
static u8 ili9341_24_seq_interface[] =         { ILI9341_INTERFACE_CONTROL,
                                              0x01, 0x01, 0x00, };
static u8 ili9341_24_seq_power_b[] =           { ILI9341_POWER_CONTROL_B,
                                              0x00, 0xC1, 0x30, };
static u8 ili9341_24_seq_power_on[] =          { ILI9341_POWER_ON_SEQUENCE_CONTROL,
                                              0x64, 0x03, 0x12, 0x81, };
static u8 ili9341_24_seq_power_a[] =           { ILI9341_POWER_CONTROL_A,
                                              0x39, 0x2C, 0x00, 0x34, 0x02, };
static u8 ili9341_24_seq_timing_b[] =          { ILI9341_DRIVER_TIMING_CONTROL_B,
                                              0x00, 0x00, };
static u8 ili9341_24_seq_timing_a[] =          { ILI9341_DRIVER_TIMING_CONTROL_A,
                                              0x85, 0x00, 0x7A, };
static u8 ili9341_24_seq_power1[] =            { ILI9341_POWER_CONTROL_1,
                                              0x26, };
static u8 ili9341_24_seq_power2[] =            { ILI9341_POWER_CONTROL_2,
                                              0x11, };
static u8 ili9341_24_seq_vcom1[] =             { ILI9341_VCOM_CONTROL_1,
                                              0x39, 0x37, };
static u8 ili9341_24_seq_vcom2[] =             { ILI9341_VCOM_CONTROL_2,
                                              0xA6, };
static u8 ili9341_24_seq_pixel_format[] =      { ILI9341_PIXEL_FORMAT_SET,
                                              0x55, };
static u8 ili9341_24_seq_memory_access[] =     { ILI9341_MEMORY_ACCESS_CONTROL,
                                              0x08, };
static u8 ili9341_24_seq_frame_rate[] =        { ILI9341_FRAME_RATE_CONTROL,
                                              0x00, 0x1B, };
static u8 ili9341_24_seq_display_function[] =  { ILI9341_DISPLAY_FUNCTION_CONTROL,
                                              0x0A, 0xA2, };
static u8 ili9341_24_seq_enable_3g[] =         { ILI9341_ENABLE_3G,
                                              0x00, };
static u8 ili9341_24_seq_gama_set[] =          { ILI9341_GAMMA_SET,
                                              0x01, };
/*
static u8 ili9341_24_seq_positive_gama[] =     { ILI9341_POSITIVE_GAMMA_CORRECTION,
                                               0x0f, 0x2D, 0x0E, 0x08, 0x12, 0x0A, 0x3D, 0x95, 0x31, 0x04, 0x10, 0x09, 0x09, 0x0D, 0x00, };
static u8 ili9341_24_seq_negative_gama[] =     { ILI9341_NEGATIVE_GAMMA_CORRECTION,
                                               0x00, 0x12, 0x17, 0x03, 0x0d, 0x05, 0x2c, 0x44, 0x41, 0x05, 0x0F, 0x0a, 0x30, 0x32, 0x0F, };
*/
static u8 ili9341_24_seq_write_memory[] =      { ILI9341_WRITE_MEMORY, };
static u8 ili9341_24_seq_sleep_out[] =         { ILI9341_SLEEP_OUT, 0x06};
static u8 ili9341_24_seq_display_on[] =        { ILI9341_DISPLAY_ON };
static u8 ili9341_24_seq_pump_ratio[] =        { ILI9341_DPUMP_RATIO_CONTROL,
                                              0x20};


/* ILI9341 init sequences for 32-HAT */
static u8 ili9341_32_seq_interface[] =      { ILI9341_INTERFACE_CONTROL,
                                              0x01, 0x01, 0x00, };
static u8 ili9341_32_seq_undocumented[] =   { ILI9341_UNDOCUMENTED_0xEF,
                                              0x03, 0x80, 0x02, };
static u8 ili9341_32_seq_power_b[] =        { ILI9341_POWER_CONTROL_B,
                                              0x00, 0xF2, 0xA0, };
static u8 ili9341_32_seq_power_on[] =       { ILI9341_POWER_ON_SEQUENCE_CONTROL,
                                              0x64, 0x03, 0x12, 0x81, };
static u8 ili9341_32_seq_power_a[] =        { ILI9341_POWER_CONTROL_A,
                                              0x39, 0x2C, 0x00, 0x34, 0x02, };
static u8 ili9341_32_seq_timing_b[] =       { ILI9341_DRIVER_TIMING_CONTROL_B,
                                              0x00, 0x00, };
static u8 ili9341_32_seq_timing_a[] =       { ILI9341_DRIVER_TIMING_CONTROL_A,
                                              0x85, 0x10, 0x7A, };
static u8 ili9341_32_seq_power1[] =         { ILI9341_POWER_CONTROL_1,
                                              0x21, };
static u8 ili9341_32_seq_power2[] =         { ILI9341_POWER_CONTROL_2,
                                              0x11, };
static u8 ili9341_32_seq_vcom1[] =          { ILI9341_VCOM_CONTROL_1,
                                              0x3F, 0x3C, };
static u8 ili9341_32_seq_vcom2[] =          { ILI9341_VCOM_CONTROL_2,
                                              0xC6, };
static u8 ili9341_32_seq_pixel_format[] =   { ILI9341_PIXEL_FORMAT_SET,
                                              0x55, };
static u8 ili9341_32_seq_memory_access[] =  { ILI9341_MEMORY_ACCESS_CONTROL,
                                              0xA8, };
static u8 ili9341_32_seq_frame_rate[] =     { ILI9341_FRAME_RATE_CONTROL,
                                              0x00, 0x1B, };
static u8 ili9341_32_seq_display_function[] = { ILI9341_DISPLAY_FUNCTION_CONTROL,
                                                0x0A, 0xA2, };
static u8 ili9341_32_seq_enable_3g[] =      { ILI9341_ENABLE_3G,
                                              0x00, };
static u8 ili9341_32_seq_gama_set[] =       { ILI9341_GAMMA_SET,
                                              0x01, };
static u8 ili9341_32_seq_positive_gama[] =  { ILI9341_POSITIVE_GAMMA_CORRECTION,
                                              0x0f, 0x24, 0x21, 0x0F, 0x13, 0x0A, 0x52, 0xC9, 0x3B, 0x05, 0x00, 0x00,
                                              0x00, 0x00, 0x00, };
static u8 ili9341_32_seq_negative_gama[] =  { ILI9341_NEGATIVE_GAMMA_CORRECTION,
                                              0x00, 0x1B, 0x1E, 0x00, 0x0C, 0x04, 0x2F, 0x36, 0x44, 0x0a, 0x1F, 0x0F,
                                              0x3F, 0x3F, 0x0F, };
static u8 ili9341_32_seq_write_memory[] =   { ILI9341_WRITE_MEMORY, };
static u8 ili9341_32_seq_sleep_out[] =      { ILI9341_SLEEP_OUT, };
static u8 ili9341_32_seq_display_on[] =     { ILI9341_DISPLAY_ON };


/* HX8357 init sequences for 35-HAT */
static u8 hx8357_seq_exit_sleep[] =         { HX8357_EXIT_SLEEP_MODE, };
static u8 hx8357_seq_enable_extension_command[] = { HX8357_ENABLE_EXTENSION_COMMAND,
                                                    0xFF, 0x83, 0x57, };
static u8 hx8357_seq_set_power_control[] =  { HX8357_SET_POWER_CONTROL, 0x00,
                                              0x12, 0x12, 0x12, 0xC3, 0x44, };
static u8 hx8357_seq_set_display_mode[] =   { HX8357_SET_DISPLAY_MODE, 0x02, 0x40,
                                              0x00, 0x2A, 0x2A, 0x20, 0x91, };
static u8 hx8357_seq_set_vcom_voltage[] =   { HX8357_SET_VCOM_VOLTAGE, 0x38, };
static u8 hx8357_seq_set_internal_oscillator[] = { HX8357_SET_INTERNAL_OSCILLATOR,
                                                   0x68, 0xE3, 0x2F, 0x1F, 0xB5, 0x01, 0x01, 0x67, };
static u8 hx8357_seq_set_panel_driving[] =  { HX8357_SET_PANEL_DRIVING, 0x70,
                                              0x70, 0x01, 0x3C, 0xC8, 0x08, };
static u8 hx8357_seq_c2[] =                 { 0xC2,
                                              0x00, 0x08, 0x04, };
static u8 hx8357_seq_set_panel_characteristic[] = { HX8357_SET_PANEL_CHARACTERISTIC,
                                                    0x09, };
static u8 hx8357_seq_set_gama_curve[] =     { HX8357_SET_GAMMA_CURVE,
                                              0x01, 0x02, 0x03, 0x05, 0x0E, 0x22, 0x32, 0x3B, 0x5C, 0x54, 0x4C, 0x41, 0x3D, 0x37,
                                              0x31, 0x21, 0x01, 0x02, 0x03, 0x05, 0x0E, 0x22, 0x32, 0x3B, 0x5C, 0x54,
                                              0x4C, 0x41, 0x3D, 0x37, 0x31, 0x21, 0x00, 0x01, };
static u8 hx8357_seq_set_pixel_format[] =   { HX8357_SET_PIXEL_FORMAT,
                                              0x55, };
static u8 hx8357_seq_set_address_mode[] =   { HX8357_SET_ADDRESS_MODE,
                                              0x00, };
static u8 hx8357_seq_set_tear_on[] =        { HX8357_SET_TEAR_ON,
                                              0x00, };
static u8 hx8357_seq_set_display_on[] =     { HX8357_SET_DISPLAY_ON,
                                            };
static u8 hx8357_seq_write_memory_start[] = { HX8357_WRITE_MEMORY_START };


/* ioctl for reading tactile switches */
#define QUERY_GET_KEYS _IOR('K', 1, unsigned char *)

/* ioctl to force LCD module initialization */
#define QUERY_INIT _IOR('I', 1, unsigned char *)

/* ioctl for reading tactile product code */
#define QUERY_GET_PRODUCT_CODE _IOR('P', 1, unsigned char *)

static int lcdpi_spi_write(struct lcdpi *item, unsigned short value,
                           unsigned int isdata);

static int lcdpi_backlight_update_status(struct backlight_device *bd) {
    struct lcdpi *item = bl_get_data(bd);

    if(bd->props.fb_blank == 0)
        item->brightness = bd->props.brightness;
    else
        item->brightness = 0;

    item->update_brightness = 1;
    item->pages[0].must_update=1;
    schedule_delayed_work(&item->info->deferred_work, HZ / 20);

    return 0;
}

static int lcdpi_backlight_get_brightness(struct backlight_device *bd)
{
    return bd->props.brightness;
}

static void lcdpi_unregister_backlight(struct lcdpi *item)
{
    const struct backlight_ops *bl_ops;

    if (item->info->bl_dev) {
        item->info->bl_dev->props.power = FB_BLANK_POWERDOWN;
        backlight_update_status(item->info->bl_dev);
        bl_ops = item->info->bl_dev->ops;
        backlight_device_unregister(item->info->bl_dev);
        item->info->bl_dev = NULL;
        kfree(bl_ops);
    }
}

static void lcdpi_register_backlight(struct lcdpi *item) {
    struct backlight_device *bd;
    struct backlight_properties bl_props = {0, 1, };
    struct backlight_ops *bl_ops;

    bl_ops = kzalloc(sizeof(struct backlight_ops), GFP_KERNEL);
    if (!bl_ops) {
        dev_err(item->info->device,
                "%s: could not allocate memeory for backlight operations.\n",
                __func__);
        return;
    }

    bl_ops->get_brightness = lcdpi_backlight_get_brightness;
    bl_ops->update_status = lcdpi_backlight_update_status;
    bl_props.type = BACKLIGHT_RAW;
    /* Assume backlight is off, get polarity from current state of pin */
    bl_props.power = FB_BLANK_POWERDOWN;

    bd = backlight_device_register(dev_driver_string(item->info->device),
                                   item->info->device, item, bl_ops, &bl_props);

    if (IS_ERR(bd)) {
        dev_err(item->info->device, "cannot register backlight device (%ld)\n",
                PTR_ERR(bd));
        goto failed;
    }
    item->info->bl_dev = bd;

    item->info->bl_dev->props.brightness = 1;
    return;

failed: kfree(bl_ops);
}


static struct spi_device *lcdpi_spi_device;
static struct lcdpi *lcdpi_item;
static void ili9341_24_setup(struct lcdpi *item);
static void ili9341_32_setup(struct lcdpi *item);
static void hx8357_35_setup(struct lcdpi *item);
static void lcdpi_update_all(struct lcdpi *item);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int ssd1289_ioctl(struct inode *i, struct file *f, unsigned int cmd, unsigned long arg)
#else
static int lcdpi_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
#endif
{
    unsigned char keys;

    switch (cmd)
    {
    case QUERY_GET_KEYS:
        lcdpi_spi_device->max_speed_hz=1000000;
        lcdpi_spi_device->master->setup(lcdpi_spi_device);
        spi_read(lcdpi_spi_device, &keys, sizeof(unsigned char));
        lcdpi_spi_device->max_speed_hz=sclk;
        lcdpi_spi_device->master->setup(lcdpi_spi_device);
        keys = (keys >> 3) & 0x1f;
        if (copy_to_user((unsigned char *)arg, &keys, sizeof(unsigned char)))
        {
            return -EACCES;
        }
        break;

    case QUERY_GET_PRODUCT_CODE:
        if (copy_to_user((unsigned char *)arg, &product_code, sizeof(unsigned char)))
        {
            return -EACCES;
        }
        break;

    case QUERY_INIT:
        switch(product_code) {
        case HAT24:
            ili9341_24_setup(lcdpi_item);
            break;
        case HAT32:
            ili9341_32_setup(lcdpi_item);
            break;
        case HAT35:
            hx8357_35_setup(lcdpi_item);
            break;
        }
        lcdpi_update_all(lcdpi_item);
        break;

    default:
        return -EINVAL;
    }

    return 0;
}


#define BLOCKLEN (HDP * VDP * 2 + 10)
static u8 *lcdpi_spiblock;

#define MAX_DMA_PIXELS 32767
static struct spiblock_dma lcdpi_spiblock_dma;

static int write_spi_dma(struct lcdpi *item, size_t len)
{
    struct spi_transfer t = {
        .tx_buf = lcdpi_spiblock_dma.buf,
                .len = len,
    };
    struct spi_message m;

    spi_message_init(&m);
    t.tx_dma = lcdpi_spiblock_dma.dma;
    m.is_dma_mapped = 1;
    spi_message_add_tail(&t, &m);
    return spi_sync(item->spidev, &m);
}

/*
This routine will write a single 16-bit value, either as data or as a command
(depends on isdata). The LCD will clock this in because the SPIs /CS goes high.
*/

/*
 * bits in control byte
 */
#define LCDPI_LONG (1<<7)
#define LCDPI_BLOCK (1<<6)
#define LCDPI_RESET (1<<5)
#define LCDPI_RS (1<<4)
#define LCDPI_BL (1<<3)
#define LCDPI_RD (1<<2)

static int lcdpi_spi_write(struct lcdpi *item, unsigned short value,
                           unsigned int isdata)
{

    item->backlight = item->info->bl_dev->props.brightness;

    /*
     * control byte
     */
    lcdpi_spiblock[0]=((isdata?LCDPI_RS:0)|(item->backlight?LCDPI_BL:0))|LCDPI_RESET|LCDPI_RD;
    /*
     * 16-bit data / command
     */
    lcdpi_spiblock[1]=(value>>8)&0xff;
    lcdpi_spiblock[2]=(value)&0xff;

    spi_write(item->spidev, lcdpi_spiblock, 3);
    return 0;
}

static inline void lcdpi_reg_set(struct lcdpi *item, unsigned char reg,
                                 unsigned short value)
{
    lcdpi_spi_write(item, reg&0xff, 0);
    lcdpi_spi_write(item, value, 1);
}

static inline void lcdpi_spi_write_array(struct lcdpi *item,
                                         u8 *value, u8 len)
{
    u8 i;

    lcdpi_spi_write(item, value[0]&0xff, 0);
    for(i=1; i < len; i++)
        lcdpi_spi_write(item, value[i], 1);
}

static void lcdpi_update_all(struct lcdpi *item)
{
    unsigned short i;
    struct fb_deferred_io *fbdefio = item->info->fbdefio;
    for (i = 0; i < item->pages_count; i++) {
        item->pages[i].must_update=1;
    }
    schedule_delayed_work(&item->info->deferred_work, fbdefio->delay);
}

extern void obj_update_compress(struct fb_info *info, struct list_head *pagelist, struct spiblock_dma *lcdpi_spiblock_dma,
                           ulong sclk, int compress, u16 width, u16 height, ulong *frames);

static void lcdpi_update_compress(struct fb_info *info, struct list_head *pagelist)
{
    obj_update_compress(info, pagelist, &lcdpi_spiblock_dma, sclk, compress, width, height, &frames);
}

static void lcdpi_update(struct fb_info *info, struct list_head *pagelist)
{
    struct lcdpi *item = (struct lcdpi *)info->par;
    struct page *page;
    int i;
    int x, lasti, ptr;
    unsigned short *buffer, value;

    if(item->update_brightness == 1) {
        /* nop */
        lcdpi_spi_write(item, 0x00, 0);
        mdelay(1);
        item->update_brightness = 0;
    }

    /*
     * We can be called because of pagefaults (mmap'ed framebuffer, pages
     * returned in *pagelist) or because of kernel activity
     * (pages[i]/must_update!=0). Add the former to the list of the latter.
     */
    list_for_each_entry(page, pagelist, lru) {
        item->pages[page->index].must_update=1;
    }

    frames++;

    /* Copy changed pages. */
    lasti = -2;
    ptr = 0;

    for (i=0; i<item->pages_count; i++) {
        /*
         * ToDo: Small race here between checking and setting must_update,
         * maybe lock?
         */
        if (item->pages[i].must_update) {
            item->pages[i].must_update=0;

            buffer = item->pages[i].buffer;

            if(ptr >= MAX_DMA_PIXELS - item->pages[i].len) {
                pr_debug("Transfering block of max=%d bytes\n", ptr*4);
                lasti = -2;
            }

            if(i != lasti + 1) {
                if(ptr) {
                    write_spi_dma(item, ptr*2+2);
                    ptr = 0;
                    pr_debug("Transfering block of %d bytes\n", ptr*4);
                }

                lcdpi_spi_write(item, HX8357_SET_COLUMN_ADDRESS, 0);
                lcdpi_spi_write(item, 0, 1);
                lcdpi_spi_write(item, 0, 1);
                lcdpi_spi_write(item, width >> 8, 1);
                lcdpi_spi_write(item, width & 0x00ff, 1);
                lcdpi_spi_write(item, HX8357_SET_PAGE_ADDRESS, 0);
                lcdpi_spi_write(item, item->pages[i].y >> 8, 1);
                lcdpi_spi_write(item, item->pages[i].y &0x0ff, 1);
                lcdpi_spi_write(item, height >> 8, 1);
                lcdpi_spi_write(item, height & 0x00ff, 1);
                lcdpi_spi_write(item, HX8357_WRITE_MEMORY_START, 0);


                /*
                 * control byte for long data transfer. Any number of 16-bit data can follow.
                 */
                lcdpi_spiblock_dma.buf[0]=LCDPI_RS|(item->backlight?LCDPI_BL:0)|LCDPI_RESET|LCDPI_RD|LCDPI_LONG;
            }

            if((ptr == 0) && (i != 0)) {
                for (x=0; x<item->pages[i].x; x++) {
                    value=item->pages[i - 1].buffer[item->pages[i - 1].len - item->pages[i].x + x];
                    lcdpi_spiblock_dma.buf[(ptr*2)+1]=(value>>8)&0xff;
                    lcdpi_spiblock_dma.buf[(ptr*2)+2]=(value)&0xff;
                    ptr++;
                }
            }

            for (x=0; x<item->pages[i].len; x++) {
                value=buffer[x];
                lcdpi_spiblock_dma.buf[(ptr*2)+1]=(value>>8)&0xff;
                lcdpi_spiblock_dma.buf[(ptr*2)+2]=(value)&0xff;
                ptr++;
            }

            lasti = i;
            pr_debug("Added page %d, ptr=%d\n", i, ptr);
        }
    }

    if(ptr) {
        pr_debug("Transfering block of %d bytes [%2x, %2x, %2x, ...]\n", ptr*2+1, lcdpi_spiblock[0],
                lcdpi_spiblock[1], lcdpi_spiblock[2]);

        /*
         * long transfer. Control byte + ptr of 16-bit data
         */
        write_spi_dma(item, ptr*2+2);
        ptr = 0;
    }

}

/* ILI9341 init for 24-HAT */
static void ili9341_24_setup(struct lcdpi *item)
{
    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

    item->spidev->max_speed_hz=sclk/4;
    item->spidev->mode=0;
    item->spidev->master->setup(item->spidev);

    lcdpi_spiblock[1]=0;
    lcdpi_spiblock[2]=0;
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(100);
    lcdpi_spiblock[0]=LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(100);
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(150);

    lcdpi_spi_write_array(item, ili9341_24_seq_interface, ARRAY_SIZE(ili9341_24_seq_interface));
    lcdpi_spi_write_array(item, ili9341_24_seq_power_b, ARRAY_SIZE(ili9341_24_seq_power_b));
    lcdpi_spi_write_array(item, ili9341_24_seq_power_on, ARRAY_SIZE(ili9341_24_seq_power_on));
    lcdpi_spi_write_array(item, ili9341_24_seq_timing_a, ARRAY_SIZE(ili9341_24_seq_timing_a));
    lcdpi_spi_write_array(item, ili9341_24_seq_power_a, ARRAY_SIZE(ili9341_24_seq_power_a));
    lcdpi_spi_write_array(item, ili9341_24_seq_pump_ratio, ARRAY_SIZE(ili9341_24_seq_pump_ratio));
    lcdpi_spi_write_array(item, ili9341_24_seq_timing_b, ARRAY_SIZE(ili9341_24_seq_timing_b));
    lcdpi_spi_write_array(item, ili9341_24_seq_power1, ARRAY_SIZE(ili9341_24_seq_power1));
    lcdpi_spi_write_array(item, ili9341_24_seq_power2, ARRAY_SIZE(ili9341_24_seq_power2));
    lcdpi_spi_write_array(item, ili9341_24_seq_vcom1, ARRAY_SIZE(ili9341_24_seq_vcom1));
    lcdpi_spi_write_array(item, ili9341_24_seq_vcom2, ARRAY_SIZE(ili9341_24_seq_vcom2));
    switch(rotate) {
    case 90:
        ili9341_24_seq_memory_access[1] = 0xC8;
        break;

    case 270:
        ili9341_24_seq_memory_access[1] = 0x08;
        break;

    case 180:
        ili9341_24_seq_memory_access[1] = 0x68;
        break;

    default:
        ili9341_24_seq_memory_access[1] = 0xA8;
        break;
    }


    lcdpi_spi_write_array(item, ili9341_24_seq_memory_access, ARRAY_SIZE(ili9341_24_seq_memory_access));
    lcdpi_spi_write_array(item, ili9341_24_seq_display_function, ARRAY_SIZE(ili9341_24_seq_display_function));
    lcdpi_spi_write_array(item, ili9341_24_seq_frame_rate, ARRAY_SIZE(ili9341_24_seq_frame_rate));
    lcdpi_spi_write_array(item, ili9341_24_seq_enable_3g, ARRAY_SIZE(ili9341_24_seq_enable_3g));
    lcdpi_spi_write_array(item, ili9341_24_seq_gama_set, ARRAY_SIZE(ili9341_24_seq_gama_set));
    lcdpi_spi_write_array(item, ili9341_24_seq_pixel_format, ARRAY_SIZE(ili9341_24_seq_pixel_format));
//    lcdpi_spi_write_array(item, ili9341_24_seq_positive_gama, ARRAY_SIZE(ili9341_24_seq_positive_gama));
//    lcdpi_spi_write_array(item, ili9341_24_seq_negative_gama, ARRAY_SIZE(ili9341_24_seq_negative_gama));
    lcdpi_spi_write_array(item, ili9341_32_seq_positive_gama, ARRAY_SIZE(ili9341_32_seq_positive_gama));
    lcdpi_spi_write_array(item, ili9341_32_seq_negative_gama, ARRAY_SIZE(ili9341_32_seq_negative_gama));

    lcdpi_spi_write_array(item, ili9341_24_seq_write_memory, ARRAY_SIZE(ili9341_24_seq_write_memory));
    lcdpi_spi_write_array(item, ili9341_24_seq_sleep_out, ARRAY_SIZE(ili9341_24_seq_sleep_out));
    msleep(120);
    lcdpi_spi_write_array(item, ili9341_24_seq_display_on, ARRAY_SIZE(ili9341_24_seq_display_on));
    usleep_range(5000, 7000);
    lcdpi_spi_write(item, HX8357_WRITE_MEMORY_START, 0);

    lcdpi_spiblock_dma.buf[0]=LCDPI_RS|LCDPI_BL|LCDPI_RESET|LCDPI_RD;

    item->spidev->max_speed_hz=sclk;
    item->spidev->mode=0;
    item->spidev->master->setup(item->spidev);
}


/* ILI9341 init for 32-HAT*/
static void ili9341_32_setup(struct lcdpi *item)
{
    lcdpi_spiblock[1]=0;
    lcdpi_spiblock[2]=0;
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);
    lcdpi_spiblock[0]=LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);

    lcdpi_spi_write_array(item, ili9341_32_seq_interface, ARRAY_SIZE(ili9341_32_seq_interface));
    lcdpi_spi_write_array(item, ili9341_32_seq_undocumented, ARRAY_SIZE(ili9341_32_seq_undocumented));
    lcdpi_spi_write_array(item, ili9341_32_seq_power_b, ARRAY_SIZE(ili9341_32_seq_power_b));
    lcdpi_spi_write_array(item, ili9341_32_seq_power_on, ARRAY_SIZE(ili9341_32_seq_power_on));
    lcdpi_spi_write_array(item, ili9341_32_seq_power_a, ARRAY_SIZE(ili9341_32_seq_power_a));
    lcdpi_spi_write_array(item, ili9341_32_seq_timing_b, ARRAY_SIZE(ili9341_32_seq_timing_b));
    lcdpi_spi_write_array(item, ili9341_32_seq_timing_a, ARRAY_SIZE(ili9341_32_seq_timing_a));
    lcdpi_spi_write_array(item, ili9341_32_seq_power1, ARRAY_SIZE(ili9341_32_seq_power1));
    lcdpi_spi_write_array(item, ili9341_32_seq_power2, ARRAY_SIZE(ili9341_32_seq_power2));
    lcdpi_spi_write_array(item, ili9341_32_seq_vcom1, ARRAY_SIZE(ili9341_32_seq_vcom1));
    lcdpi_spi_write_array(item, ili9341_32_seq_vcom2, ARRAY_SIZE(ili9341_32_seq_vcom2));
    lcdpi_spi_write_array(item, ili9341_32_seq_pixel_format, ARRAY_SIZE(ili9341_32_seq_pixel_format));

    switch(rotate) {
    case 90:
        ili9341_32_seq_memory_access[1] = 0xC8;
        break;

    case 270:
        ili9341_32_seq_memory_access[1] = 0x08;
        break;

    case 180:
        ili9341_32_seq_memory_access[1] = 0x68;
        break;

    default:
        ili9341_32_seq_memory_access[1] = 0xA8;
        break;
    }

    lcdpi_spi_write_array(item, ili9341_32_seq_memory_access, ARRAY_SIZE(ili9341_32_seq_memory_access));
    lcdpi_spi_write_array(item, ili9341_32_seq_frame_rate, ARRAY_SIZE(ili9341_32_seq_frame_rate));
    lcdpi_spi_write_array(item, ili9341_32_seq_display_function, ARRAY_SIZE(ili9341_32_seq_display_function));
    lcdpi_spi_write_array(item, ili9341_32_seq_enable_3g, ARRAY_SIZE(ili9341_32_seq_enable_3g));
    lcdpi_spi_write_array(item, ili9341_32_seq_gama_set, ARRAY_SIZE(ili9341_32_seq_gama_set));
    lcdpi_spi_write_array(item, ili9341_32_seq_positive_gama, ARRAY_SIZE(ili9341_32_seq_positive_gama));
    lcdpi_spi_write_array(item, ili9341_32_seq_negative_gama, ARRAY_SIZE(ili9341_32_seq_negative_gama));
    lcdpi_spi_write_array(item, ili9341_32_seq_write_memory, ARRAY_SIZE(ili9341_32_seq_write_memory));
    lcdpi_spi_write_array(item, ili9341_32_seq_sleep_out, ARRAY_SIZE(ili9341_32_seq_sleep_out));
    msleep(120);
    lcdpi_spi_write_array(item, ili9341_32_seq_display_on, ARRAY_SIZE(ili9341_32_seq_display_on));
    usleep_range(5000, 7000);
    lcdpi_spi_write(item, HX8357_WRITE_MEMORY_START, 0);

    lcdpi_spiblock_dma.buf[0]=LCDPI_RS|LCDPI_BL|LCDPI_RESET|LCDPI_RD;
}

/* HX8357 init for 35-HAT*/
static void hx8357_35_setup(struct lcdpi *item)
{
    lcdpi_spiblock[1]=0;
    lcdpi_spiblock[2]=0;
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);
    lcdpi_spiblock[0]=LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);

    lcdpi_spi_write_array(item, hx8357_seq_exit_sleep, ARRAY_SIZE(hx8357_seq_exit_sleep));
    mdelay(150);
    lcdpi_spi_write_array(item, hx8357_seq_enable_extension_command, ARRAY_SIZE(hx8357_seq_enable_extension_command));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_power_control, ARRAY_SIZE(hx8357_seq_set_power_control));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_display_mode, ARRAY_SIZE(hx8357_seq_set_display_mode));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_vcom_voltage, ARRAY_SIZE(hx8357_seq_set_vcom_voltage));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_internal_oscillator, ARRAY_SIZE(hx8357_seq_set_internal_oscillator));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_panel_driving, ARRAY_SIZE(hx8357_seq_set_panel_driving));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_c2, ARRAY_SIZE(hx8357_seq_c2));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_panel_characteristic, ARRAY_SIZE(hx8357_seq_set_panel_characteristic));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_gama_curve, ARRAY_SIZE(hx8357_seq_set_gama_curve));
    mdelay(1);
    lcdpi_spi_write_array(item, hx8357_seq_set_pixel_format, ARRAY_SIZE(hx8357_seq_set_pixel_format));

    switch(rotate) {
    case 90:
        hx8357_seq_set_address_mode[1] = 0x00;
        break;

    case 270:
        hx8357_seq_set_address_mode[1] = 0xC0;
        break;

    case 180:
        hx8357_seq_set_address_mode[1] = 0xA0;
        break;

    default:
        hx8357_seq_set_address_mode[1] = 0x60;
        break;
    }

    lcdpi_spi_write_array(item, hx8357_seq_set_address_mode, ARRAY_SIZE(hx8357_seq_set_address_mode));
    lcdpi_spi_write_array(item, hx8357_seq_set_tear_on, ARRAY_SIZE(hx8357_seq_set_tear_on));
    mdelay(10);
    lcdpi_spi_write_array(item, hx8357_seq_set_display_on, ARRAY_SIZE(hx8357_seq_set_display_on));
    mdelay(10);
    lcdpi_spi_write_array(item, hx8357_seq_write_memory_start, ARRAY_SIZE(hx8357_seq_write_memory_start));
}

/*
 * This routine will allocate the buffer for the complete framebuffer. This
 * is one continuous chunk of 16-bit pixel values; userspace programs
 * will write here.
 */
static int __init lcdpi_video_alloc(struct lcdpi *item)
{
    unsigned int frame_size;

    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

    frame_size = item->info->fix.line_length * item->info->var.yres;
    dev_dbg(item->dev, "%s: item=0x%p frame_size=%u\n",
            __func__, (void *)item, frame_size);

    item->pages_count = frame_size / PAGE_SIZE;
    if ((item->pages_count * PAGE_SIZE) < frame_size) {
        item->pages_count++;
    }
    dev_dbg(item->dev, "%s: item=0x%p pages_count=%u\n",
            __func__, (void *)item, item->pages_count);

    item->info->fix.smem_len = item->pages_count * PAGE_SIZE;
    item->info->fix.smem_start =
            (unsigned long)vmalloc(item->info->fix.smem_len);
    if (!item->info->fix.smem_start) {
        dev_err(item->dev, "%s: unable to vmalloc\n", __func__);
        return -ENOMEM;
    }
    memset((void *)item->info->fix.smem_start, 0, item->info->fix.smem_len);

    return 0;
}

static void lcdpi_video_free(struct lcdpi *item)
{
    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);
    vfree((void *)item->info->fix.smem_start);
}

/*
 * This routine will allocate a lcdpi_page struct for each vm page in the
 * main framebuffer memory. Each struct will contain a pointer to the page
 * start, an x- and y-offset, and the length of the pagebuffer which is in the framebuffer.
 */
static int __init lcdpi_pages_alloc(struct lcdpi *item)
{
    unsigned short pixels_per_page;
    unsigned short yoffset_per_page;
    unsigned short xoffset_per_page;
    unsigned short index;
    unsigned short x = 0;
    unsigned short y = 0;
    unsigned short *buffer;
    unsigned int len;

    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

    item->pages = kmalloc(item->pages_count * sizeof(struct lcdpi_page),
                          GFP_KERNEL);
    if (!item->pages) {
        dev_err(item->dev, "%s: unable to kmalloc for lcdpi_page\n",
                __func__);
        return -ENOMEM;
    }

    pixels_per_page = PAGE_SIZE / (item->info->var.bits_per_pixel / 8);
    yoffset_per_page = pixels_per_page / item->info->var.xres;
    xoffset_per_page = pixels_per_page -
            (yoffset_per_page * item->info->var.xres);
    dev_dbg(item->dev, "%s: item=0x%p pixels_per_page=%hu "
                       "yoffset_per_page=%hu xoffset_per_page=%hu\n",
            __func__, (void *)item, pixels_per_page,
            yoffset_per_page, xoffset_per_page);

    buffer = (unsigned short *)item->info->fix.smem_start;
    for (index = 0; index < item->pages_count; index++) {
        len = (item->info->var.xres * item->info->var.yres) -
                (index * pixels_per_page);
        if (len > pixels_per_page) {
            len = pixels_per_page;
        }
        dev_dbg(item->dev,
                "%s: page[%d]: x=%3hu y=%3hu buffer=0x%p len=%3hu\n",
                __func__, index, x, y, buffer, len);
        item->pages[index].x = x;
        item->pages[index].y = y;
        item->pages[index].buffer = buffer;
        item->pages[index].len = len;

        x += xoffset_per_page;
        if (x >= item->info->var.xres) {
            y++;
            x -= item->info->var.xres;
        }
        y += yoffset_per_page;
        buffer += pixels_per_page;
    }

    return 0;
}

static void lcdpi_pages_free(struct lcdpi *item)
{
    dev_dbg(item->dev, "%s: item=0x%p\n", __func__, (void *)item);

    kfree(item->pages);
}

static inline __u32 CNVT_TOHW(__u32 val, __u32 width)
{
    return ((val<<width) + 0x7FFF - val)>>16;
}

/* This routine is needed because the console driver won't work without it. */
static int lcdpi_setcolreg(unsigned regno,
                           unsigned red, unsigned green, unsigned blue,
                           unsigned transp, struct fb_info *info)
{
    int ret = 1;

    /*
     * If greyscale is true, then we convert the RGB value
     * to greyscale no matter what visual we are using.
     */
    if (info->var.grayscale)
        red = green = blue = (19595 * red + 38470 * green +
                              7471 * blue) >> 16;
    switch (info->fix.visual) {
    case FB_VISUAL_TRUECOLOR:
        if (regno < 16) {
            u32 *pal = info->pseudo_palette;
            u32 value;

            red = CNVT_TOHW(red, info->var.red.length);
            green = CNVT_TOHW(green, info->var.green.length);
            blue = CNVT_TOHW(blue, info->var.blue.length);
            transp = CNVT_TOHW(transp, info->var.transp.length);

            value = (red << info->var.red.offset) |
                    (green << info->var.green.offset) |
                    (blue << info->var.blue.offset) |
                    (transp << info->var.transp.offset);

            pal[regno] = value;
            ret = 0;
        }
        break;
    case FB_VISUAL_STATIC_PSEUDOCOLOR:
    case FB_VISUAL_PSEUDOCOLOR:
        break;
    }
    return ret;
}

static int lcdpi_blank(int blank_mode, struct fb_info *info)
{
    struct lcdpi *item = (struct lcdpi *)info->par;
    if (blank_mode == FB_BLANK_UNBLANK)
        item->backlight=1;
    else
        item->backlight=0;
    /*
     * Item->backlight won't take effect until the LCD is written to. Force that
     * by dirty'ing a page.
     */
    item->pages[0].must_update=1;
    schedule_delayed_work(&info->deferred_work, 0);
    return 0;
}

static void lcdpi_touch(struct fb_info *info, int x, int y, int w, int h)
{
    struct fb_deferred_io *fbdefio = info->fbdefio;
    struct lcdpi *item = (struct lcdpi *)info->par;
    int i, ystart, yend;
    if (fbdefio) {
        /* Touch the pages the y-range hits, so the deferred io will update them. */
        for (i=0; i<item->pages_count; i++) {
            ystart=item->pages[i].y;
            yend=item->pages[i].y+(item->pages[i].len/info->fix.line_length)+1;
            if (!((y+h)<ystart || y>yend)) {
                item->pages[i].must_update=1;
            }
        }
        /* Schedule the deferred IO to kick in after a delay. */
        schedule_delayed_work(&info->deferred_work, fbdefio->delay);
    }
}

static void lcdpi_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
    sys_fillrect(p, rect);
    lcdpi_touch(p, rect->dx, rect->dy, rect->width, rect->height);
}

static void lcdpi_imageblit(struct fb_info *p, const struct fb_image *image)
{
    sys_imageblit(p, image);
    lcdpi_touch(p, image->dx, image->dy, image->width, image->height);
}

static void lcdpi_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
    sys_copyarea(p, area);
    lcdpi_touch(p, area->dx, area->dy, area->width, area->height);
}

static ssize_t lcdpi_write(struct fb_info *p, const char __user *buf,
                           size_t count, loff_t *ppos)
{
    ssize_t res;
    res = fb_sys_write(p, buf, count, ppos);
    lcdpi_touch(p, 0, 0, p->var.xres, p->var.yres);
    return res;
}

static struct fb_ops lcdpi_fbops = {
    .owner        = THIS_MODULE,
    .fb_read      = fb_sys_read,
    .fb_write     = lcdpi_write,
    .fb_fillrect  = lcdpi_fillrect,
    .fb_copyarea  = lcdpi_copyarea,
    .fb_imageblit = lcdpi_imageblit,
    .fb_setcolreg	= lcdpi_setcolreg,
    .fb_blank	= lcdpi_blank,
    .fb_ioctl = lcdpi_ioctl,
};

static struct fb_fix_screeninfo lcdpi_fix __initdata = {
    .id          = "lcdpi",
    .type        = FB_TYPE_PACKED_PIXELS,
    .visual      = FB_VISUAL_TRUECOLOR,
    .accel       = FB_ACCEL_NONE,
    /*	.line_length =  HDP * 2, */
};

static struct fb_var_screeninfo lcdpi_var __initdata = {
    /*		.xres		= HDP,
     * 		.yres		= VDP,
     * 		.xres_virtual	= HDP,
     *  	.yres_virtual	= VDP,
     */
    .width		= 74,
    .height		= 49,
    .bits_per_pixel	= 16,
    .red		= {11, 5, 0},
    .green		= {5, 6, 0},
    .blue		= {0, 5, 0},
    .activate	= FB_ACTIVATE_NOW,
    .vmode		= FB_VMODE_NONINTERLACED,
};

static struct fb_deferred_io lcdpi_defio = {
    .delay          = HZ / 60,
    .deferred_io    = &lcdpi_update_compress,
};

static u32 lcdpi_of_value(struct device_node *node, const char *propname)
{
    u32 val = -1;

    of_property_read_u32(node, propname, &val);
    return val;
}

static const struct of_device_id lcdpi_dt_ids[];

static int lcdpi_parse_dt(struct device *dev)
{
    struct device_node *node = dev->of_node;
    const struct of_device_id *of_id;
    u32 dummy;

    if(!node)
        return -1;

    of_id = of_match_device(lcdpi_dt_ids, dev);
    if(!product_code)
        product_code = (int)of_id->data;

     //of_property_read_u8(node, "product-code", (u8*)&product_code);

    dummy = lcdpi_of_value(node, "rotate");
    if((dummy != -1) && (rotate == 360))
        rotate = dummy;

/*
    dummy = lcdpi_of_value(node, "spi-max-frequency");
    if(dummy != -1)
        sclk = dummy;
*/

    of_property_read_u8_array(node, "rotate-codes", rotate_codes, 4);

    return 0;
}

static int __init lcdpi_probe(struct spi_device *dev)
{
    int ret = 0;
    struct lcdpi *item;
    struct fb_info *info;

    dev_dbg(&dev->dev, "%s\n", __func__);

    lcdpi_parse_dt(&dev->dev);

    item = kzalloc(sizeof(struct lcdpi), GFP_KERNEL);
    if (!item) {
        dev_err(&dev->dev, "%s: unable to kzalloc for lcdpi\n", __func__);
        ret = -ENOMEM;
        goto out;
    }
    item->dev = &dev->dev;
    dev_set_drvdata(&dev->dev, item);
    item->backlight=1;

    lcdpi_item = item;
    lcdpi_spi_device=item->spidev=dev;
    spin_lock_init(&item->spi_lock);
    item->dev=&dev->dev;
    spi_set_drvdata(dev, item);
    dev_info(&dev->dev, "4d-hat registered, product code = %x\n", product_code);

    lcdpi_spiblock = (u8*)kmalloc(10, GFP_KERNEL);

    item->spidev->max_speed_hz=16000000;
    item->spidev->mode=0;
    item->spidev->master->setup(item->spidev);

    /* reset display and get the pruduct code */
    lcdpi_spiblock[1]=0;
    lcdpi_spiblock[2]=0;
    lcdpi_spiblock[0]=LCDPI_RESET|LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);
    lcdpi_spiblock[0]=LCDPI_RD;
    spi_write(item->spidev, lcdpi_spiblock, 3);
    mdelay(50);

    switch(product_code) {
    case HAT24:
    case HAT32:
    case HAT35:
        break;
    default:
        spi_read(lcdpi_spi_device, &product_code, sizeof(unsigned char));
        break;
    }

    rotate %= 360;

    switch(product_code) {
    case HAT24:
        HDP = 320;
        VDP = 240;
        lcdpi_touchparams = touchscreens[0][rotate / 90];
        break;
    case HAT32:
        HDP = 320;
        VDP = 240;
        lcdpi_touchparams = touchscreens[1][rotate / 90];
        break;
    case HAT35:
        HDP = 480;
        VDP = 320;
        lcdpi_touchparams = touchscreens[2][rotate / 90];
        break;
    }

    /* override with DT touch parameters */
    if(rotate_codes[rotate / 90] != 0xff)
        lcdpi_touchparams = rotate_codes[rotate / 90];

    dev_dbg(&dev->dev, "%s: product_code=0x%x\n", __func__, product_code);

    switch(rotate) {
    case 90:
    case 270: lcdpi_var.yres = HDP;
        lcdpi_var.xres = VDP;
        lcdpi_var.yres_virtual = HDP;
        lcdpi_var.xres_virtual = VDP;
        lcdpi_var.width = 49;
        lcdpi_var.height = 74;
        lcdpi_fix.line_length =  VDP * 2;
        width = VDP - 1;
        height = HDP - 1;
        break;

    default: lcdpi_var.xres = HDP;
        lcdpi_var.yres = VDP;
        lcdpi_var.xres_virtual = HDP;
        lcdpi_var.yres_virtual = VDP;
        lcdpi_var.width = 74;
        lcdpi_var.height = 49;
        lcdpi_fix.line_length =  HDP * 2;
        width = HDP - 1;
        height = VDP - 1;
        break;
    }

    info = framebuffer_alloc(sizeof(struct lcdpi), &dev->dev);
    if (!info) {
        ret = -ENOMEM;
        dev_err(&dev->dev, "%s: unable to framebuffer_alloc\n", __func__);
        goto out_item;
    }

    info->pseudo_palette = &item->pseudo_palette;
    item->info = info;
    info->par = item;
    info->dev = &dev->dev;
    info->fbops = &lcdpi_fbops;
    info->flags = FBINFO_FLAG_DEFAULT|FBINFO_VIRTFB;
    info->fix = lcdpi_fix;
    info->var = lcdpi_var;

    ret = lcdpi_video_alloc(item);
    if (ret) {
        dev_err(&dev->dev, "%s: unable to lcdpi_video_alloc\n", __func__);
        goto out_info;
    }
    info->screen_base = (char __iomem *)item->info->fix.smem_start;

    ret = lcdpi_pages_alloc(item);
    if (ret < 0) {
        dev_err(&dev->dev, "%s: unable to lcdpi_pages_init\n", __func__);
        goto out_video;
    }

    lcdpi_register_backlight(item);

    switch(compress) {
    case 0:
        lcdpi_defio.deferred_io = &lcdpi_update;
        break;

    default:
        lcdpi_defio.deferred_io = &lcdpi_update_compress;
        break;
    }

    info->fbdefio = &lcdpi_defio;
    fb_deferred_io_init(info);

    item->dev->coherent_dma_mask = ~0;
    lcdpi_spiblock_dma.buf = dma_alloc_coherent(item->dev, BLOCKLEN * 4, &lcdpi_spiblock_dma.dma, GFP_DMA);
    lcdpi_spiblock_dma.len = BLOCKLEN * 4;

    if(!lcdpi_spiblock){
        dev_err(&dev->dev, "%s: unable to allocate spi buffer\n", __func__);
        goto out_pages;
    }

    switch(product_code) {
    case HAT24:
        rotate = (rotate + 180) % 360;
        ili9341_24_setup(item);
        break;
    case HAT32:
        ili9341_32_setup(item);
        break;
    case HAT35:
        hx8357_35_setup(item);
        break;
    default:
        dev_err(&dev->dev, "%s: product not supporterd!\n", __func__);
        break;
    }

    item->spidev->max_speed_hz=sclk;
    item->spidev->master->setup(item->spidev);

    lcdpi_update_all(item);

    ret = register_framebuffer(info);
    if (ret < 0) {
        dev_err(&dev->dev, "%s: unable to register_frambuffer\n", __func__);
        goto out_pages;
    }

    return ret;

out_pages:
    lcdpi_unregister_backlight(item);
    lcdpi_pages_free(item);
out_video:
    lcdpi_video_free(item);
out_info:
    framebuffer_release(info);
out_item:
    kfree(item);
out:
    return ret;
}

static int __exit lcdpi_remove(struct spi_device *dev)
{
    struct lcdpi *item = dev_get_drvdata(&dev->dev);
    struct fb_info *info;

    dev_dbg(&dev->dev, "%s\n", __func__);

    dev_set_drvdata(&dev->dev, NULL);
    if (item) {
        lcdpi_unregister_backlight(item);
        info = item->info;
        if (info) {
            fb_deferred_io_cleanup(info);
            unregister_framebuffer(info);
            framebuffer_release(info);
        }
        lcdpi_pages_free(item);
        lcdpi_video_free(item);

        if(lcdpi_spiblock_dma.buf)
            dma_free_coherent(item->dev, BLOCKLEN * 4, lcdpi_spiblock_dma.buf, lcdpi_spiblock_dma.dma);

        kfree(item);
    }

    if(lcdpi_spiblock)
        kfree(lcdpi_spiblock);

    return 0;

}

/*
#ifdef CONFIG_PM
static int lcdpi_suspend(struct spi_device *spi, pm_message_t state)
{
    struct fb_info *info = dev_get_drvdata(&spi->dev);
    struct lcdpi *item = (struct lcdpi *)info->par;
    // enter into sleep mode
    lcdpi_reg_set(item, lcdpi_REG_SLEEP_MODE, 0x0001);
    return 0;
}

static int lcdpi_resume(struct spi_device *spi)
{
    struct fb_info *info = dev_get_drvdata(&spi->dev);
    struct lcdpi *item = (struct lcdpi *)info->par;
    // leave sleep mode
    lcdpi_reg_set(item, lcdpi_REG_SLEEP_MODE, 0x0000);
    return 0;
}
#else
*/
#define lcdpi_suspend NULL
#define lcdpi_resume NULL
/* #endif */

static const struct of_device_id lcdpi_dt_ids[] = {
{ .compatible = "4dsystems,24-hat", .data = (void *)HAT24 },
{ .compatible = "4dsystems,32-hat", .data = (void *)HAT32  },
{ .compatible = "4dsystems,35-hat", .data = (void *)HAT35  },
{ .compatible = "4dsystems,4dpi-3x", .data = (void *)0  }, /* product code is read from Xilinx */
{},
};

MODULE_DEVICE_TABLE(of, lcdpi_dt_ids);

static struct spi_driver spi_lcdpi_driver = {
    .driver = {
        .name	= "4d-hats",
        .owner	= THIS_MODULE,
        .of_match_table = of_match_ptr(lcdpi_dt_ids),
    },
    .probe = lcdpi_probe,
    .remove = lcdpi_remove,

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4,1,0))
    .suspend = lcdpi_suspend,
    .resume = lcdpi_resume,
#endif

};

static int __init lcdpi_init(void)
{
    int ret = 0;

    ret = spi_register_driver(&spi_lcdpi_driver);
    if (ret) {
        pr_err("%s: unable to platform_driver_register\n", __func__);
    }

    return ret;
}
static void __exit lcdpi_exit(void)
{
    pr_debug("%s\n", __func__);

    spi_unregister_driver(&spi_lcdpi_driver);
}

module_init(lcdpi_init);
module_exit(lcdpi_exit);

MODULE_DESCRIPTION("4D-HAT LCD Driver");
MODULE_AUTHOR("Andrej Strancar <info@hwlevel.com>");
MODULE_LICENSE("GPL");
