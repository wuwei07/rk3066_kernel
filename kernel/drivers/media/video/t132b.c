/*
 * t132b.c - Terawins T132BT video decoder driver
 *
 * Copyright (c) 2012 Terawins Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <media/soc_camera.h>
#include <linux/platform_device.h>
#include <mach/iomux.h>
#include <mach/io.h>

//#ifndef DEBUG
#define DEBUG
//#endif

#ifdef DEBUG
#define DBG(format, ...) \
		printk(KERN_INFO "T132BT: " format "\n", ## __VA_ARGS__)
#define ERR(format, ...) \
		printk(KERN_ERR "T132BT: " format "\n", ## __VA_ARGS__)
#else
#define DBG(format, ...)
#define ERR(format, ...)
#endif

#define DRIVER_FOR_ROCKCHIP	1

#if DRIVER_FOR_ROCKCHIP
#include <plat/rk_camera.h>
#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)
#define SENSOR_NAME RK29_CAM_SENSOR_T132B
#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))

#define DRIVER_NAME SENSOR_NAME_STRING()

// for PX2 SDK V10
#define T132B_CVBS_IN_DET 	RK30_PIN6_PA0
#define T132B_YCRCB_IN_DET 	RK30_PIN0_PB7
#endif

/**
 * T132B registers definition
 */
#define T132B_CHIPID_REG				0xF4
	#define T132B_CHIPID_T132				0x32

#define T132B_VS_TIMING_MEAS_REG		0x50
	#define	T132B_SHORT_VS_FREERUN			0x04
#define T132B_VS_PERIOD_LSB_REG			0x5A
#define T132B_VS_PERIOD_MSB_REG			0x5B

#define T132B_CVD_STATUS_REG			0x3A
	#define T132B_CVD_NO_SIGNAL				(1 << 0)
	#define T132B_CVD_H_LOCK				(1 << 1)
	#define T132B_CVD_V_LOCK				(1 << 2)
	#define T132B_CVD_C_LOCK				(1 << 3)
#define T132B_CVD_AUTO_MODE_REG			0x41

#define T132B_INT_STATUS_REG			0x12
	#define T132B_STA_LOST_VSYNC			(1 << 0)
	#define T132B_STA_LOST_HSYNC			(1 << 1)
	#define T132B_STA_HSYNC_CHANG			(1 << 2)
	#define T132B_STA_VSYNC_CHANG			(1 << 3)
	#define T132B_STA_VSYNC_LEADING			(1 << 5)

/**
 * input mode command
 */
#define T132B_INPUT_CVBS_ALL	0x132F
#define T132B_INPUT_CVBS_NTSC	0x1320
#define T132B_INPUT_CVBS_PAL	0x1321
#define T132B_INPUT_YCBCR_480I	0x1322
#define T132B_INPUT_YCBCR_576I	0x1323
#define T132B_INPUT_YUV_480P	0x1324
#define T132B_INPUT_YUV_576P	0x1325
#define T132B_INPUT_VGA			0x1326
#define T132B_INPUT_SVGA		0x1327

/**
 * output mode command
 */
#define T132B_OUTPUT_CCIR656	0x1328
#define T132B_OUTPUT_CCIR601	0x1329

/**
 * output control command
 */
#define T132B_OUTPUT_START		0x0132
#define T132B_OUTPUT_STOP		0x1132

struct t132b_init_array {
	u8 reg;
	u8 val;
};

static const
struct t132b_init_array init_601_out[]=
{
	// encode init
	{0xFF,0x03},	// page3
	{0x1B,0x08},
	{0x1C,0x80},

	{0xFF,0x00},	// page0
	{0xCF,0x02},

};

static const
struct t132b_init_array init_656_out[]=
{
	// encode init
	{0xFF,0x03},	// page3
	{0x1A,0x10},

	{0xFF,0x00},	// page0
	{0xCF,0x07},

};

static const
struct t132b_init_array init_cvbs_ntsc_656[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x2F},
	{0x07,0x28},
	{0x08,0x28},
	{0x09,0x28},
	{0x0A,0x80},
	{0x0B,0x80},
	{0x0C,0x80},
	{0x10,0x28},
	{0x11,0x98},
	{0x13,0x08},
	{0x14,0xD1},
	{0x15,0x5A},
	{0x17,0x48},
	{0x1B,0x00},
	{0x0F,0xF1},
	{0x16,0xF3},
	{0x18,0x00},
	{0x19,0x04},
	{0x1A,0x02},
	{0x20,0x00},
	{0x30,0x82},
	{0x31,0x10},
	{0x40,0xC0},

	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x0F},
	{0x84,0xC7},
	{0x85,0x01},

	{0x90,0x00},
	{0xB0,0x20},
	{0xB1,0x00},
	{0xB2,0x14},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xF0},
	{0xB7,0x00},
	{0xB8,0x4A},
	{0xB9,0x03},
	{0xBA,0x07},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x1E},
	{0xC9,0x00},
	{0xCA,0x23},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xF0},
	{0xDF,0x00},
	{0xE0,0xB0},
	{0xE3,0x80},

	// cvd init
	{0xFF,0x02},	// page2
	{0x00,0x00},
	{0x01,0x09},
	{0x02,0x4B},
	{0x03,0x0B},
	{0x07,0xA0},
	{0x08,0x78},
	{0x09,0x2B},
	{0x0A,0x64},
	{0x0C,0x67},
	{0x18,0x21},
	{0x19,0xF0},
	{0x1A,0x7C},
	{0x1B,0x1F},
	{0x40,0x03},
	{0x30,0xFF},
	{0x31,0x00},
	{0x39,0x09},
	{0x67,0x0B},
	{0x3F,0x01},
	{0x3F,0x00},

	{0xFF,0x00},	// page0

};

static const
struct t132b_init_array init_cvbs_pal_656[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x00,0x02},
	{0x01,0x02},
	{0x02,0x02},
	{0x03,0x00},
	{0x04,0x2F},
	{0x07,0x28},
	{0x08,0x28},
	{0x09,0x28},
	{0x0A,0x80},
	{0x0B,0x80},
	{0x0C,0x80},
	{0x10,0x28},
	{0x11,0x98},
	{0x13,0x08},
	{0x14,0xD1},
	{0x15,0x5A},
	{0x17,0x48},
	{0x1B,0x00},
	{0x0F,0xF1},
	{0x16,0xF3},
	{0x18,0x00},
	{0x19,0x04},
	{0x1A,0x02},
	{0x20,0x00},
	{0x30,0x82},
	{0x31,0x10},
	{0x40,0xC0},

	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x0F},
	{0x84,0xC7},
	{0x85,0x17},

	{0x90,0x00},
	{0xB0,0x20},
	{0xB1,0x00},
	{0xB2,0x14},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0x20},
	{0xB7,0x01},
	{0xB8,0x55},
	{0xB9,0x03},
	{0xBA,0x07},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x03},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x1E},
	{0xC9,0x00},
	{0xCA,0x23},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0x20},
	{0xDF,0x01},
	{0xE0,0xB0},
	{0xE3,0x80},

	// cvd init
	{0xFF,0x02},	// page2
	{0x00,0x32},
	{0x01,0x08},
	{0x02,0x4B},
	{0x03,0x0B},
	{0x07,0xA3},
	{0x08,0x78},
	{0x09,0x2B},
	{0x0A,0x64},
	{0x0C,0x67},
	{0x10,0x2A},
	{0x11,0x89},
	{0x18,0x2A},
	{0x19,0x09},
	{0x1A,0x8A},
	{0x1B,0xCB},
	{0x40,0x03},	
	{0x30,0xFF},
	{0x31,0x00},
	{0x39,0x09},
	{0x67,0x0B},
	{0x3F,0x01},
	{0x3F,0x00},

	{0xFF,0x00},	// page0

};

static const
struct t132b_init_array init_cvbs_ntsc_601[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x2F},
	{0x07,0x28},
	{0x08,0x28},
	{0x09,0x28},
	{0x0A,0x80},
	{0x0B,0x80},
	{0x0C,0x80},
	{0x10,0x28},
	{0x11,0x98},
	{0x13,0x08},
	{0x14,0xD1},
	{0x15,0x5A},
	{0x1B,0x00},
	{0x17,0x48},
	{0x0F,0xF1},
	{0x16,0xF3},
	{0x18,0x00},
	{0x19,0x04},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x10},
	{0x40,0xC0},

	// timing
	{0x70,0x80},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x10},
	{0x79,0x80},
	{0x84,0x00},
	{0x85,0x01},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x28},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xE0},
	{0xB7,0x01},
	{0xB8,0x58},
	{0xB9,0x03},
	{0xBA,0x07},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xE0},
	{0xDF,0x01},
	{0xE0,0xB0},
	{0xE3,0x80},

	// cvd init
	{0xFF,0x02},	// page2
	{0x00,0x00},
	{0x01,0x01},
	{0x02,0x4B},
	{0x18,0x21},
	{0x19,0xF0},
	{0x1A,0x7C},
	{0x1B,0x1F},
	{0x30,0x22},
	{0x31,0xC1},
	{0x39,0x09},
	{0x3F,0x01},
	{0x3F,0x00},

	{0xFF,0x00},	// page0

};

static const
struct t132b_init_array init_cvbs_pal_601[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x2F},
	{0x07,0x28},
	{0x08,0x28},
	{0x09,0x28},
	{0x0A,0x80},
	{0x0B,0x80},
	{0x0C,0x80},
	{0x10,0x28},
	{0x11,0x98},
	{0x13,0x08},
	{0x14,0xD1},
	{0x15,0x5A},
	{0x17,0x48},
	{0x1B,0x00},
	{0x0F,0xF1},
	{0x16,0xF3},
	{0x18,0x00},
	{0x19,0x04},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x10},
	{0x40,0xC0},

	// timing
	{0x70,0x80},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x10},
	{0x79,0x80},
	{0x84,0xC7},
	{0x85,0x17},

	{0x90,0x00},
	{0xB0,0x20},
	{0xB1,0x00},
	{0xB2,0x2C},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0x40},
	{0xB7,0x02},
	{0xB8,0x55},
	{0xB9,0x03},
	{0xBA,0x07},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x03},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0x40},
	{0xDF,0x02},
	{0xE0,0xB0},
	{0xE3,0x80},

	// cvd init
	{0xFF,0x02},	// page2
	{0x00,0x32},
	{0x01,0x02},
	{0x02,0x4B},
	{0x03,0x00},
	{0x0C,0x67},
	{0x18,0x2A},
	{0x19,0x09},
	{0x1A,0x8A},
	{0x1B,0xCB},
	{0x30,0x2C},
	{0x31,0xC1},
	{0x39,0x09},
	{0x3F,0x01},
	{0x3F,0x00},

	{0xFF,0x00},	// page0

};

static const
struct t132b_init_array init_ycbcr_480i_656[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x26},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x5C},
	{0x0B,0x78},
	{0x0C,0x74},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDE},
	{0x15,0xB7},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x86},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x82},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x60},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0xA0},
	{0x49,0x05},
	{0x4A,0xF0},
	{0x4B,0x00},
	{0x4C,0x8A},
	{0x4D,0x00},
	{0x4E,0x0F},
	{0x4F,0x00},


	// timing
	{0x70,0x90},	// [5] Inv_VideoF
	{0x72,0x00},
	{0x73,0x40},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x0B},
	{0x84,0xC7},
	{0x85,0x01},

	{0x90,0x00},
	{0xB0,0x20},
	{0xB1,0x00},
	{0xB2,0x12},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xF0},
	{0xB7,0x00},
	{0xB8,0x4A},
	{0xB9,0x03},
	{0xBA,0x07},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x1E},
	{0xC9,0x00},
	{0xCA,0x23},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xF0},
	{0xDF,0x00},
	{0xE0,0xA0},
	{0xE3,0x80},

};

static const
struct t132b_init_array init_ycbcr_576i_656[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x26},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x58},
	{0x0B,0x7C},
	{0x0C,0x74},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDE},
	{0x15,0xC9},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x86},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x82},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x7C},
	{0x46,0x03},
	{0x47,0x00},
	{0x48,0xA0},
	{0x49,0x05},
	{0x4A,0x20},
	{0x4B,0x01},
	{0x4C,0x9E},
	{0x4D,0x00},
	{0x4E,0x11},
	{0x4F,0x00},


	// timing
	{0x70,0x90},	// [5] Inv_VideoF
	{0x72,0x00},
	{0x73,0x40},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x0B},
	{0x84,0xC7},
	{0x85,0x17},

	{0x90,0x00},
	{0xB0,0x20},
	{0xB1,0x00},
	{0xB2,0x14},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0x20},
	{0xB7,0x01},
	{0xB8,0x50},
	{0xB9,0x03},
	{0xBA,0x20},
	{0xBB,0x01},
	{0xBC,0x10},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x1E},
	{0xC9,0x00},
	{0xCA,0x23},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0x20},
	{0xDF,0x01},
	{0xE0,0xA0},
	{0xE3,0x80},

	{0xFF,0x03},	// page3
	{0x1A,0xD0},	// only for ycbcr576i 656out
	{0xFF,0x00},	// page3
};

static const
struct t132b_init_array init_ycbcr_480i_601[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x26},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x5C},
	{0x0B,0x78},
	{0x0C,0x74},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDE},
	{0x15,0xB7},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x86},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x60},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0xA0},
	{0x49,0x05},
	{0x4A,0xF0},
	{0x4B,0x00},
	{0x4C,0x8A},
	{0x4D,0x00},
	{0x4E,0x0F},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x40},
	{0x74,0x00},
	{0x75,0x10},
	{0x79,0x80},
	{0x84,0x15},
	{0x85,0x3D},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x10},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xE0},
	{0xB7,0x01},
	{0xB8,0x58},
	{0xB9,0x03},
	{0xBA,0xF0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xE0},
	{0xDF,0x01},
	{0xE0,0xA0},
	{0xE3,0x80},

};

static const
struct t132b_init_array init_ycbcr_576i_601[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x26},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x58},
	{0x0B,0x7C},
	{0x0C,0x74},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDE},
	{0x15,0xC9},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x86},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x7C},
	{0x46,0x03},
	{0x47,0x00},
	{0x48,0xA0},
	{0x49,0x05},
	{0x4A,0x20},
	{0x4B,0x01},
	{0x4C,0x9E},
	{0x4D,0x00},
	{0x4E,0x11},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x40},
	{0x74,0x00},
	{0x75,0x10},
	{0x79,0x80},
	{0x84,0x15},
	{0x85,0x4B},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x12},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0x40},
	{0xB7,0x02},
	{0xB8,0x62},
	{0xB9,0x03},
	{0xBA,0xF0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0x40},
	{0xDF,0x02},
	{0xE0,0xA0},
	{0xE3,0x80},
};

static const
struct t132b_init_array init_yuv_480p[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x24},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x5C},
	{0x0B,0x78},
	{0x0C,0x74},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDE},
	{0x15,0xB7},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x83},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x3A},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0xD0},
	{0x49,0x02},
	{0x4A,0xE0},
	{0x4B,0x01},
	{0x4C,0x74},
	{0x4D,0x00},
	{0x4E,0x20},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x80},
	{0x84,0x15},
	{0x85,0x20},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x17},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xE0},
	{0xB7,0x01},
	{0xB8,0x59},
	{0xB9,0x03},
	{0xBA,0xD0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xE0},
	{0xDF,0x01},
	{0xE0,0xA0},
	{0xE3,0x80},

};

static const
struct t132b_init_array init_yuv_576p[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x24},
	{0x07,0x30},
	{0x08,0x30},
	{0x09,0x30},
	{0x0A,0x58},
	{0x0B,0x74},
	{0x0C,0x70},
	{0x10,0x09},
	{0x11,0x7D},
	{0x13,0xAA},
	{0x14,0xDC},
	{0x15,0x6B},
	{0x17,0x38},
	{0x1B,0x1F},
	{0x0F,0xF5},
	{0x16,0x83},
	{0x18,0x3F},
	{0x19,0x34},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x10},
	{0x46,0x02},
	{0x47,0x00},
	{0x48,0xD0},
	{0x49,0x02},
	{0x4A,0x40},
	{0x4B,0x02},
	{0x4C,0x84},
	{0x4D,0x00},
	{0x4E,0x30},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x00},
	{0x73,0x20},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x80},
	{0x84,0x15},
	{0x85,0x3A},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x21},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0x40},
	{0xB7,0x02},
	{0xB8,0x63},
	{0xB9,0x03},
	{0xBA,0xF0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2E},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0x40},
	{0xDF,0x02},
	{0xE0,0xA0},
	{0xE3,0x80},

};

static const
struct t132b_init_array init_vga_640x480[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x00},
	{0x04,0x24},
	{0x07,0x2E},
	{0x08,0x2E},
	{0x09,0x2E},
	{0x0A,0x58},
	{0x0B,0x78},
	{0x0C,0x70},
	{0x10,0x09},
	{0x11,0x25},
	{0x13,0xAA},
	{0x14,0xF0},
	{0x15,0x20},
	{0x17,0x38},
	{0x1B,0x0D},
	{0x0F,0xF5},
	{0x16,0x83},
	{0x18,0x15},
	{0x19,0x13},
	{0x1A,0x00},
	{0x20,0x00},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x10},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0x80},
	{0x49,0x02},
	{0x4A,0xE0},
	{0x4B,0x01},
	{0x4C,0x7F},
	{0x4D,0x00},
	{0x4E,0x24},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x71},
	{0x73,0x1C},
	{0x74,0x00},
	{0x75,0x20},
	{0x79,0x80},
	{0x84,0x15},
	{0x85,0x20},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x1A},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xE0},
	{0xB7,0x01},
	{0xB8,0x14},
	{0xB9,0x03},
	{0xBA,0xF0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2A},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xE0},
	{0xDF,0x01},
	{0xE0,0xA0},
	{0xE3,0x80},

};

static const
struct t132b_init_array init_svga_800x600[]=
{
	// ADC init
	{0xFF,0x00},	// page0
	{0x03,0x08},
	{0x04,0x24},
	{0x07,0x2E},
	{0x08,0x2E},
	{0x09,0x2E},
	{0x0A,0x58},
	{0x0B,0x78},
	{0x0C,0x70},
	{0x10,0x09},
	{0x11,0x25},
	{0x13,0xAA},
	{0x14,0xF0},
	{0x15,0x20},
	{0x17,0x38},
	{0x1B,0x0D},
	{0x0F,0xF5},
	{0x16,0x84},
	{0x18,0x15},
	{0x19,0x13},
	{0x1A,0x00},
	{0x20,0x20},
	{0x30,0x80},
	{0x31,0x30},

	{0x40,0xC2},
	{0x43,0x01},
	{0x44,0x00},
	{0x45,0x10},
	{0x46,0x01},
	{0x47,0x00},
	{0x48,0x20},
	{0x49,0x03},
	{0x4A,0x58},
	{0x4B,0x02},
	{0x4C,0x46},
	{0x4D,0x00},
	{0x4E,0x10},
	{0x4F,0x00},


	// timing
	{0x70,0xB0},
	{0x72,0x8E},
	{0x73,0x23},
	{0x74,0x00},
	{0x75,0x28},
	{0x79,0x80},
	{0x84,0x00},
	{0x85,0x23},

	{0x90,0x00},
	{0xB0,0x16},
	{0xB1,0x00},
	{0xB2,0x1A},
	{0xB3,0x00},
	{0xB4,0xD0},
	{0xB5,0x02},
	{0xB6,0xE0},
	{0xB7,0x01},
	{0xB8,0x1D},
	{0xB9,0x03},
	{0xBA,0xF0},
	{0xBB,0x02},
	{0xBC,0x20},
	{0xBD,0x00},
	{0xBE,0x02},
	{0xBF,0x00},

	{0xC0,0x01},
	{0xC1,0x08},
	{0xC8,0x2B},
	{0xC9,0x01},
	{0xCA,0x22},
	{0xDC,0xD0},
	{0xDD,0x02},
	{0xDE,0xE0},
	{0xDF,0x01},
	{0xE0,0xA0},
	{0xE3,0x80},
};

struct t132b_state {
	struct v4l2_subdev	sd;
	struct mutex		mutex; /* mutual excl. when accessing chip */
#if DRIVER_FOR_ROCKCHIP
	int			cvbs_irq;
	int         ycrcb_irq;
	struct work_struct	ycrcb_work;
	struct work_struct	cvbs_work;
#endif
	v4l2_std_id		curr_norm;
	bool			autodetect;
	bool			en_output;
	struct soc_camera_device *icd;
	unsigned int	input_mode;
	unsigned int 	output_mode;
};

static inline void t132b_config_array(struct i2c_client *client, const struct t132b_init_array *tab, size_t cnt)
{
	int i;

	for(i = 0; i < cnt; i++) {
		i2c_smbus_write_byte_data(client, tab[i].reg, tab[i].val);
	}
}

static inline void t132b_config_601(struct i2c_client *client)
{
	t132b_config_array(client, init_601_out, sizeof(init_601_out)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_656(struct i2c_client *client)
{
	t132b_config_array(client, init_656_out, sizeof(init_656_out)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_cvbs_ntsc_656(struct i2c_client *client)
{
	t132b_config_array(client, init_cvbs_ntsc_656, sizeof(init_cvbs_ntsc_656)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_cvbs_pal_656(struct i2c_client *client)
{
	t132b_config_array(client, init_cvbs_pal_656, sizeof(init_cvbs_pal_656)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_cvbs_ntsc_601(struct i2c_client *client)
{
	t132b_config_array(client, init_cvbs_ntsc_601, sizeof(init_cvbs_ntsc_601)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_cvbs_pal_601(struct i2c_client *client)
{
	t132b_config_array(client, init_cvbs_pal_601, sizeof(init_cvbs_pal_601)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_ycbcr_480i_656(struct i2c_client *client)
{
	t132b_config_array(client, init_ycbcr_480i_656, sizeof(init_ycbcr_480i_656)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_ycbcr_576i_656(struct i2c_client *client)
{
	t132b_config_array(client, init_ycbcr_576i_656, sizeof(init_ycbcr_576i_656)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_ycbcr_480i_601(struct i2c_client *client)
{
	t132b_config_array(client, init_ycbcr_480i_601, sizeof(init_ycbcr_480i_601)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_ycbcr_576i_601(struct i2c_client *client)
{
	t132b_config_array(client, init_ycbcr_576i_601, sizeof(init_ycbcr_576i_601)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_yuv_480p(struct i2c_client *client)
{
	t132b_config_array(client, init_yuv_480p, sizeof(init_yuv_480p)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_yuv_576p(struct i2c_client *client)
{
	t132b_config_array(client, init_yuv_576p, sizeof(init_yuv_576p)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_vga_640x480(struct i2c_client *client)
{
	t132b_config_array(client, init_vga_640x480, sizeof(init_vga_640x480)/sizeof(struct t132b_init_array));
}

static inline void t132b_config_svga_800x600(struct i2c_client *client)
{
	t132b_config_array(client, init_svga_800x600, sizeof(init_svga_800x600)/sizeof(struct t132b_init_array));
}

static void change_input_config(struct i2c_client *client, int input, int output, int std)
{
	switch(input) {
	case T132B_INPUT_CVBS_ALL:
		if(std == V4L2_STD_NTSC) {
			if(output == T132B_OUTPUT_CCIR656) {
				t132b_config_cvbs_ntsc_656(client);
			} else {
				t132b_config_cvbs_ntsc_601(client);
			}
		} else {
			if(output == T132B_OUTPUT_CCIR656) {
				t132b_config_cvbs_pal_656(client);
			} else {
				t132b_config_cvbs_pal_601(client);
			}
		}
		break;
	case T132B_INPUT_CVBS_NTSC:
		if(output == T132B_OUTPUT_CCIR656) {
			t132b_config_cvbs_ntsc_656(client);
		} else {
			t132b_config_cvbs_ntsc_601(client);
		}
		break;
	case T132B_INPUT_CVBS_PAL:
		if(output == T132B_OUTPUT_CCIR656) {
			t132b_config_cvbs_pal_656(client);
		} else {
			t132b_config_cvbs_pal_601(client);
		}
		break;
	case T132B_INPUT_YCBCR_480I:
		if(output == T132B_OUTPUT_CCIR656) {
			t132b_config_ycbcr_480i_656(client);
		} else {
			t132b_config_ycbcr_480i_601(client);
		}
		break;
	case T132B_INPUT_YCBCR_576I:
		if(output == T132B_OUTPUT_CCIR656) {
			t132b_config_ycbcr_576i_656(client);
		} else {
			t132b_config_ycbcr_576i_601(client);
		}
		break;
	case T132B_INPUT_YUV_480P:
		t132b_config_yuv_480p(client);
		break;
	case T132B_INPUT_YUV_576P:
		t132b_config_yuv_576p(client);
		break;
	case T132B_INPUT_VGA:
		t132b_config_vga_640x480(client);
		break;
	case T132B_INPUT_SVGA:
		t132b_config_svga_800x600(client);
		break;
	default:
		ERR("unknown input mode!\n");
		break;
	}
}

static void change_output_config(struct i2c_client *client, int input, int output, int std)
{
	switch(output) {
	case T132B_OUTPUT_CCIR656:
		t132b_config_656(client);
		break;
	case T132B_OUTPUT_CCIR601:
		t132b_config_601(client);
		break;
	default:
		ERR("unknown output mode!\n");
		break;
	}
}

static int cvd_get_vs_period(struct i2c_client *client)
{
	int vs_period = 0;
	int temp = 0;
	int i = 0, count = 0;

	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
	for(i = 0; i < 20; i++) {
		vs_period = i2c_smbus_read_byte_data(client, T132B_VS_PERIOD_LSB_REG);
		vs_period |= (i2c_smbus_read_byte_data(client, T132B_VS_PERIOD_MSB_REG) << 8);
		if((temp == (vs_period + 1)) || (temp == (vs_period - 1))
				|| (temp == vs_period)) {
			count++;
			if(count >= 5)
				break;
		} else {
			count = 0;
			temp = vs_period;
			if(i2c_smbus_read_byte_data(client, T132B_VS_PERIOD_LSB_REG)
					& T132B_SHORT_VS_FREERUN)
				break;
		}
		msleep(10);
	}
	return vs_period;
}

static v4l2_std_id cvbs_get_std(struct i2c_client *client)
{
	int vtotal;

#if 0	/* TODO: support all std. */
	int color_cubcarrire;

	i2c_smbus_write_byte_data(client, 0xFF, 0x02);	// select page2
	color_cubcarrire = i2c_smbus_read_byte_data(client, T132B_CVD_AUTO_MODE_REG);
	color_cubcarrire |= 0x30;
	vtotal = cvd_get_vs_period(client);
	if((vtotal >= 261) && (vtotal <= 264)
		&& (color_cubcarrire == 0x10)) {
		return V4L2_STD_NTSC;
	} else if((vtotal >= 261) && (vtotal <= 264)
		&& (color_cubcarrire == 0x20)) {
		return V4L2_STD_NTSC_443;
	} else if((vtotal >= 311) && (vtotal <= 314)
		&& (color_cubcarrire == 0x20)) {
		return V4L2_STD_PAL;
	} else if((vtotal >= 311) && (vtotal <= 314)
		&& (color_cubcarrire == 0x10)) {
		return V4L2_STD_PAL_N;
	} else {
		return V4L2_STD_UNKNOWN;
	}
#else
	vtotal = cvd_get_vs_period(client);
	if((vtotal >= 259) && (vtotal <= 264)) {
		DBG("%s V4L2_STD_NTSC 1", __FUNCTION__);
		return V4L2_STD_NTSC;
	} else if((vtotal >= 310) && (vtotal <= 314)) {
		DBG("%s V4L2_STD_PAL 1", __FUNCTION__);
		return V4L2_STD_PAL;
	} else if((vtotal >= 520) && (vtotal <= 528)) {
		DBG("%s V4L2_STD_NTSC 2", __FUNCTION__);
		return V4L2_STD_NTSC;
	} else if((vtotal >= 620) && (vtotal <= 628)) {
		DBG("%s V4L2_STD_PAL 2", __FUNCTION__);
		return V4L2_STD_PAL;
	} else {
		DBG("%s V4L2_STD_UNKNOWN", __FUNCTION__);
		return V4L2_STD_UNKNOWN;
	}
#endif
}

static v4l2_std_id t132b_std_to_v4l2(struct i2c_client *client, u8 status1)
{
	/* now no signal */
	if ((status1 & (T132B_CVD_H_LOCK | T132B_CVD_V_LOCK)) == 0x00) {
		return V4L2_STD_UNKNOWN;
	}
	/* now have signal*/
	else if((status1 & (T132B_CVD_H_LOCK | T132B_CVD_V_LOCK)) ==
			(T132B_CVD_H_LOCK | T132B_CVD_V_LOCK)) {
		return cvbs_get_std(client);
	}
	else {
		return V4L2_STD_UNKNOWN;
	}
}

static u32 t132b_status_to_v4l2(u8 status1)
{
	if ((status1 & T132B_CVD_NO_SIGNAL))
		return V4L2_IN_ST_NO_SIGNAL;

	return 0;
}

static int __t132b_status2(struct i2c_client *client, u32 *status,
	v4l2_std_id *std, int input)
{
	int status1;

	i2c_smbus_write_byte_data(client, 0xFF, 0x03);	// select page3
	status1 = i2c_smbus_read_byte_data(client, T132B_INT_STATUS_REG);
	i2c_smbus_write_byte_data(client, T132B_INT_STATUS_REG, 0x3F);	// w1c
	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0

	/* check signal */
	if((status1 & (T132B_STA_LOST_HSYNC | T132B_STA_LOST_VSYNC))
			== (T132B_STA_LOST_HSYNC | T132B_STA_LOST_VSYNC)
			|| (status1 & (T132B_STA_LOST_HSYNC | T132B_STA_VSYNC_LEADING))
			== T132B_STA_LOST_HSYNC) {
		/* no signal */
		status1 = V4L2_IN_ST_NO_SIGNAL;
		/* close panel (enable build-in pattern and freerun) */
		i2c_smbus_write_byte_data(client, 0x91, 0x87);
		i2c_smbus_write_byte_data(client, 0xC2, 0x12);
	} else {
		status1 = 0;
		/* open panel (close build-in pattern and freerun) */
		i2c_smbus_write_byte_data(client, 0x91, 0x07);
		i2c_smbus_write_byte_data(client, 0xC2, 0x00);
	}

	/* YCbCr 480i/576i */
	if(input == T132B_INPUT_YCBCR_480I
			|| input == T132B_INPUT_YCBCR_576I) {
		if (std) {
			if(input == T132B_INPUT_YCBCR_480I) {
				*std = V4L2_STD_NTSC;
			} else {
				*std = V4L2_STD_PAL;
			}
		}
		if (status)
			*status = status1;
	/* YUV 480p/576p */
	} else if(input == T132B_INPUT_YUV_480P
			|| input == T132B_INPUT_YUV_576P) {
		if (status)
			*status = status1;
		if (std)
			*std = V4L2_STD_UNKNOWN;
	/* VGA/SVGA */
	} else if(input == T132B_INPUT_VGA
			|| input == T132B_INPUT_SVGA) {
		if (status)
			*status = status1;
		if (std)
			*std = V4L2_STD_UNKNOWN;
	/* unknown input */
	} else {
		if (status)
			*status = V4L2_IN_ST_NO_SIGNAL;
		if (std)
			*std = V4L2_STD_UNKNOWN;
	}

	return 0;
}

static int __t132b_status(struct i2c_client *client, u32 *status,
	v4l2_std_id *std)
{
	int status1;

	i2c_smbus_write_byte_data(client, 0xFF, 0x02);	// select page2
	status1 = i2c_smbus_read_byte_data(client, T132B_CVD_STATUS_REG);
	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0

	if (status1 < 0)
		return status1;

	if (status)
		*status = t132b_status_to_v4l2(status1);
	if (std)
		*std = t132b_std_to_v4l2(client, status1);

	return 0;
}

static inline struct t132b_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct t132b_state, sd);
}

static void t132b_out_ctl(struct v4l2_subdev *sd, bool onoff)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct t132b_state *state = to_state(sd);
	int val = 0;

	DBG("%s onoff %d", __FUNCTION__, onoff);

	if(onoff == 0) {
		// disable output
		i2c_smbus_write_byte_data(client, 0xFF, 0x03);	// select page3
		val = i2c_smbus_read_byte_data(client, 0x1A);
		i2c_smbus_write_byte_data(client, 0x1A, val & 0xFE);	// disable CCIR656 output
		val = i2c_smbus_read_byte_data(client, 0x1B);
		i2c_smbus_write_byte_data(client, 0x1B, val & 0xFE);	// disable CCIR601 output
		i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
		state->en_output = 0;
	} else {
		if(state->input_mode == T132B_INPUT_CVBS_ALL
			|| state->input_mode == T132B_INPUT_CVBS_NTSC
			|| state->input_mode == T132B_INPUT_CVBS_PAL) {
			i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
			/* open panel (close build-in pattern and freerun) */
			i2c_smbus_write_byte_data(client, 0x91, 0x07);
			i2c_smbus_write_byte_data(client, 0xC2, 0x00);
		} else {
			i2c_smbus_write_byte_data(client, 0xFF, 0x03);	// select page3
			i2c_smbus_write_byte_data(client, T132B_INT_STATUS_REG, 0x3F);	// w1c
			/* check signal and if no signal enable build-in pattern */
			__t132b_status2(client, NULL, NULL, state->input_mode);
		}
		// enable output
		if(state->output_mode == T132B_OUTPUT_CCIR601) {
			i2c_smbus_write_byte_data(client, 0xFF, 0x03);	// select page3
			val = i2c_smbus_read_byte_data(client, 0x1B);
			i2c_smbus_write_byte_data(client, 0x1B, val | 0x01); // enable CCIR601 output
			i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
		} else {
			i2c_smbus_write_byte_data(client, 0xFF, 0x03);	// select page3
			val = i2c_smbus_read_byte_data(client, 0x1A);
			i2c_smbus_write_byte_data(client, 0x1A, val | 0x01); // enable CCIR656 output
			i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
		}
		state->en_output = 1;
	}
}

static void t132b_pwr_ctl(struct soc_camera_device *icd, bool onoff)
{
	return ;
#if 0	/* TODO: power control */
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int enable;

	if(onoff == 0)
		enable = 1;
	else
		enable = 0;

	if (icl->powerdown) {
		icl->powerdown(icd->pdev, enable);
		msleep(10);
	}
#endif
}

static int t132b_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct t132b_state *state = to_state(sd);
	int err = mutex_lock_interruptible(&state->mutex);
	if (err)
		return err;
	if(state->output_mode == T132B_OUTPUT_CCIR601
		/* state->input_mode != T132B_INPUT_CVBS_ALL
			&& state->input_mode != T132B_INPUT_CVBS_NTSC
			&& state->input_mode != T132B_INPUT_CVBS_PAL */) {
		err = -1;	/* CCIR656 configure for rockchip  */
	} else {
		if(state->input_mode == T132B_INPUT_CVBS_ALL
				|| state->input_mode == T132B_INPUT_CVBS_NTSC
				|| state->input_mode == T132B_INPUT_CVBS_PAL) {
			err = __t132b_status(v4l2_get_subdevdata(sd), NULL, std);
		} else {
			err = __t132b_status2(v4l2_get_subdevdata(sd), NULL, std, state->input_mode);
		}
	}
	mutex_unlock(&state->mutex);
	return err;
}

static int t132b_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct t132b_state *state = to_state(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	if(state->input_mode == T132B_INPUT_CVBS_ALL
			|| state->input_mode == T132B_INPUT_CVBS_NTSC
			|| state->input_mode == T132B_INPUT_CVBS_PAL) {
		ret = __t132b_status(v4l2_get_subdevdata(sd), status, NULL);
	} else {
		ret = __t132b_status2(v4l2_get_subdevdata(sd), status, NULL, state->input_mode);
	}

	mutex_unlock(&state->mutex);
	return ret;
}

static int t132b_v4l2_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
	struct t132b_state *state = to_state(sd);

	DBG("%s val %d", __FUNCTION__, val);
	t132b_pwr_ctl(icd, 1);

	/* clear state */
	state->input_mode = 0;
	state->output_mode = 0;
	state->curr_norm = 0;
	state->en_output = 0;

	/* disable output */
	t132b_out_ctl(sd, 0);

	/* Initialize t132b */
	// disable power saving
	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
	i2c_smbus_write_byte_data(client, 0xFC, 0x0F);
	i2c_smbus_write_byte_data(client, 0xFD, 0x07);

	// pad init
	i2c_smbus_write_byte_data(client, 0xFF, 0x01);	// select page1
	i2c_smbus_write_byte_data(client, 0xE5, 0x10);
	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0

	// set blue color for build-in pattern
	i2c_smbus_write_byte_data(client, 0x9D, 0x1D);	// Y
	i2c_smbus_write_byte_data(client, 0x9E, 0xF0);	// U
	i2c_smbus_write_byte_data(client, 0x9F, 0x6C);	// V

	// set input & output mode
	DBG("%s devnum %d init", __FUNCTION__, icd->devnum);
	switch(icd->devnum) {
	case 2:
		printk("input CVBS NTSC mode and 656 output mode\n");
		state->input_mode = T132B_INPUT_CVBS_NTSC;
		state->output_mode = T132B_OUTPUT_CCIR656;
		change_output_config(client, state->input_mode, state->output_mode, 0);
		change_input_config(client, state->input_mode, state->output_mode, T132B_INPUT_CVBS_NTSC);
		break;
	case 1:
		printk("input YUV 480P mode and 601 output mode\n");
		state->input_mode = T132B_INPUT_YUV_480P;
		state->output_mode = T132B_OUTPUT_CCIR601;
		change_output_config(client, state->input_mode, state->output_mode, 0);
		change_input_config(client, state->input_mode, state->output_mode, 0);
		break;
	case 0:
	default:
		printk("t132b default init, input cvbs mode and 656 output mode\n");
		state->input_mode = T132B_INPUT_CVBS_ALL;
		state->output_mode = T132B_OUTPUT_CCIR656;
		change_output_config(client, state->input_mode, state->output_mode, 0);
		change_input_config(client, state->input_mode, state->output_mode, V4L2_STD_NTSC);
		msleep(100);
		/* check current std. */
		__t132b_status(client, NULL, &state->curr_norm);
		if(state->curr_norm == V4L2_STD_PAL) {
			change_input_config(client, state->input_mode, state->output_mode, state->curr_norm);
		}
		break;
	}

	/* enable output */
	t132b_out_ctl(sd, 1);

	return 0;

#if 0	// TODO: auto detection
err_unreg_subdev:
	return ret;
#endif
}

static long t132b_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct t132b_state *state = to_state(sd);

	DBG("%s cmd %x", __FUNCTION__, cmd);

	switch(cmd) {
#if DRIVER_FOR_ROCKCHIP
	case RK29_CAM_SUBDEV_DEACTIVATE:
		t132b_pwr_ctl(icd, 0);
		return 0;
#endif
	case T132B_OUTPUT_CCIR656:
	case T132B_OUTPUT_CCIR601:
		state->output_mode = cmd;
		change_output_config(client, state->input_mode, state->output_mode, state->curr_norm);
		return 0;
	case T132B_INPUT_CVBS_ALL:
		__t132b_status(client, NULL, &state->curr_norm);
		break;
	case T132B_INPUT_CVBS_NTSC:
		state->curr_norm = V4L2_STD_NTSC;
		break;
	case T132B_INPUT_CVBS_PAL:
		state->curr_norm = V4L2_STD_PAL;
		break;
	case T132B_INPUT_YCBCR_480I:
	case T132B_INPUT_YCBCR_576I:
	case T132B_INPUT_YUV_480P:
	case T132B_INPUT_YUV_576P:
	case T132B_INPUT_VGA:
	case T132B_INPUT_SVGA:
		state->curr_norm = V4L2_STD_UNKNOWN;
		break;
	case T132B_OUTPUT_START:
		t132b_out_ctl(sd, 1);
		break;
	case T132B_OUTPUT_STOP:
		t132b_out_ctl(sd, 0);
		break;
	default:
		ERR("unknown ioctl cmd!\n");
		return -EINVAL;
	}
	state->input_mode = cmd;
	change_input_config(client, state->input_mode, state->output_mode, state->curr_norm);

	return 0;
}

static int t132b_g_chip_ident(struct v4l2_subdev *sd,
	struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_T132B, 0);
}

static int t132b_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct t132b_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = mutex_lock_interruptible(&state->mutex);
	if (ret)
		return ret;

	/* all standards -> autodetect */
	if (std == V4L2_STD_ALL) {
		state->input_mode = T132B_INPUT_CVBS_ALL;
		/* read current norm */
		__t132b_status(client, NULL, &state->curr_norm);
		change_input_config(client, state->input_mode, state->output_mode, state->curr_norm);
	} else {
		switch(std) {
		case V4L2_STD_NTSC:
			state->input_mode = T132B_INPUT_CVBS_NTSC;
			change_input_config(client, state->input_mode, state->output_mode, std);
			break;
		case V4L2_STD_PAL:
			state->input_mode = T132B_INPUT_CVBS_PAL;
			change_input_config(client, state->input_mode, state->output_mode, std);
			break;
		default:
			goto out;
		}
		state->curr_norm = std;
	}
	ret = 0;
out:
	mutex_unlock(&state->mutex);
	return ret;
}

struct t132b_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

struct t132b_picsize {
	unsigned int width;
	unsigned int height;
};

static const struct t132b_picsize t132b_pic_sizes[] = {
	//Default
	{720, 480},
	//NTSC & 480i & 480p
	{720, 480},
	//PAL & 576i & 576p
	{720, 576},
	// VGA
	{720, 480}, //{640, 480},
	// SVGA
	{720, 480}, //{800, 600},

};

static const struct t132b_datafmt t132b_colour_fmts[] = {
    {V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_SMPTE170M},
};

static int t132b_set_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct t132b_state *state = to_state(sd);
	int index = 0;
//	struct i2c_client *client = v4l2_get_subdevdata(sd);

	DBG("%s width %d height %d code 0x%x colorspace 0x%x",
		__FUNCTION__, mf->width, mf->height, mf->code, mf->colorspace);

	switch(state->input_mode) {
	case T132B_INPUT_CVBS_ALL:
//		__t132b_status(client, NULL, &state->curr_norm);
		if(state->curr_norm == V4L2_STD_PAL)
			index = 2;
		else if(state->curr_norm == V4L2_STD_NTSC)
			index = 1;
		else
			index = 0;
		break;
	case T132B_INPUT_CVBS_NTSC:
	case T132B_INPUT_YCBCR_480I:
	case T132B_INPUT_YUV_480P:
		index = 1;
		break;
	case T132B_INPUT_CVBS_PAL:
	case T132B_INPUT_YCBCR_576I:
	case T132B_INPUT_YUV_576P:
		index = 2;
		break;
	case T132B_INPUT_VGA:
		index = 3;
		break;
	case T132B_INPUT_SVGA:
		index = 4;
		break;
	default:
		index = 0;
		break;
	}
	mf->width	= t132b_pic_sizes[index].width;
	mf->height	= t132b_pic_sizes[index].height;

	DBG("%s index:%d mf->width %d mf->height %d code 0x%x colorspace 0x%x",
		__FUNCTION__,index, mf->width, mf->height, mf->code, mf->colorspace);
	return 0;
}

static int t132b_get_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	mf->width	= t132b_pic_sizes[0].width;
	mf->height	= t132b_pic_sizes[0].height;
	mf->code	=  t132b_colour_fmts[0].code;
	mf->colorspace	= t132b_colour_fmts[0].colorspace;
//	mf->pixelformat = V4L2_PIX_FMT_UYVY;
	mf->field	= V4L2_FIELD_NONE;
	DBG("%s width %d height %d code 0x%x colorspace 0x%x",
		__FUNCTION__, mf->width, mf->height, mf->code, mf->colorspace);
	return 0;
}

static int t132b_try_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct t132b_state *state = to_state(sd);
	int index;
//	struct i2c_client *client = v4l2_get_subdevdata(sd);

	DBG("%s width %d height %d code 0x%x colorspace 0x%x",
		__FUNCTION__, mf->width, mf->height, mf->code, mf->colorspace);
	//If input picture size is larger than supported picture size, just query max supported picture size.
	switch(state->input_mode) {
	case T132B_INPUT_CVBS_ALL:
//		__t132b_status(client, NULL, &state->curr_norm);
		if(state->curr_norm == V4L2_STD_PAL)
			index = 2;
		else if(state->curr_norm == V4L2_STD_NTSC)
			index = 1;
		else
			index = 0;
		break;
	case T132B_INPUT_CVBS_NTSC:
	case T132B_INPUT_YCBCR_480I:
	case T132B_INPUT_YUV_480P:
		index = 1;
		break;
	case T132B_INPUT_CVBS_PAL:
	case T132B_INPUT_YCBCR_576I:
	case T132B_INPUT_YUV_576P:
		index = 2;
		break;
	case T132B_INPUT_VGA:
		index = 3;
		break;
	case T132B_INPUT_SVGA:
		index = 4;
		break;
	default:
		index = 0;
		break;
	}
	if(mf->width > t132b_pic_sizes[index].width || mf->height > t132b_pic_sizes[index].height) {
		mf->width = t132b_pic_sizes[index].width;
		mf->height = t132b_pic_sizes[index].height;
		return 0;
	}
	else
		return 0;
//	for(int i = 0; i < ARRAY_SIZE(t132b_colour_fmts); i++)
//	{
//		if( mf->width == t132b_colour_fmts[i].width &&
//			mf->height == t132b_colour_fmts[i].height &&
//			mf->code == t132b_colour_fmts[i].code )
//			return 0;
//	}

	return -EINVAL;
}

static int t132b_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	DBG("%s index %d", __FUNCTION__, index);

	if (index >= ARRAY_SIZE(t132b_colour_fmts))
		return -EINVAL;

	*code = t132b_colour_fmts[index].code;
	return 0;
}

static int t132b_enum_mbus_pixelfmt(struct v4l2_subdev *sd, unsigned int index,
			    int *code)
{
	DBG("%s index %d", __FUNCTION__, index);

	if(index > 0)
		return -EINVAL;

	*code = V4L2_PIX_FMT_NV16;
	return 0;
}

static int t132b_set_bus_param(struct soc_camera_device *icd, unsigned long flags)
{
    return 0;
}

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING|\
                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |\
                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_27MHZ)

static unsigned long t132b_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SENSOR_BUS_PARAM;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int t132b_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
	DBG("%s", __FUNCTION__);
	if (pm_msg.event == PM_EVENT_SUSPEND) {
		t132b_pwr_ctl(icd, 0);
	}
	return 0;
}

static int t132b_resume(struct soc_camera_device *icd)
{
	DBG("%s", __FUNCTION__);
	t132b_pwr_ctl(icd, 1);
	return 0;
}

static struct soc_camera_ops t132b_sensor_ops =
{
	.suspend            = t132b_suspend,
	.resume 			= t132b_resume,
	.set_bus_param		= t132b_set_bus_param,
	.query_bus_param	= t132b_query_bus_param,
};

static const struct v4l2_subdev_video_ops t132b_video_ops = {
	.querystd = t132b_querystd,
	.g_input_status = t132b_g_input_status,
	.s_mbus_fmt = t132b_set_mbus_fmt,
	.g_mbus_fmt = t132b_get_mbus_fmt,
	.try_mbus_fmt = t132b_try_mbus_fmt,
	.enum_mbus_fmt = t132b_enum_mbus_fmt,
//	.enum_mbus_pixelfmt = t132b_enum_mbus_pixelfmt,
};

static const struct v4l2_subdev_core_ops t132b_core_ops = {
	.init = t132b_v4l2_init,
	.ioctl = t132b_ioctl,
	.g_chip_ident = t132b_g_chip_ident,
	.s_std = t132b_s_std,
};

static const struct v4l2_subdev_ops t132b_ops = {
	.core = &t132b_core_ops,
	.video = &t132b_video_ops,
};

#if DRIVER_FOR_ROCKCHIP
static void t132b_cvbs_work(struct work_struct *cvbs_work)
{
	struct t132b_state *state = container_of(cvbs_work, struct t132b_state,
		cvbs_work);
//	struct i2c_client *client = v4l2_get_subdevdata(&state->sd);

	mutex_lock(&state->mutex);
#if 0	// TODO: auto detection input mode

#endif
	mutex_unlock(&state->mutex);

	enable_irq(state->cvbs_irq);
}

static void t132b_ycrcb_work(struct work_struct *ycrcb_work)
{
	struct t132b_state *state = container_of(ycrcb_work, struct t132b_state,
		ycrcb_work);
//	struct i2c_client *client = v4l2_get_subdevdata(&state->sd);

	mutex_lock(&state->mutex);
#if 0	// TODO: auto detection input mode

#endif
	mutex_unlock(&state->mutex);

	enable_irq(state->ycrcb_irq);
}

static irqreturn_t t132b_cvbs_irq(int irq, void *devid)
{
	struct t132b_state *state = devid;

	schedule_work(&state->cvbs_work);

	disable_irq_nosync(state->cvbs_irq);

	return IRQ_HANDLED;
}


static irqreturn_t t132b_ycrcb_irq(int irq, void *devid)
{
	struct t132b_state *state = devid;

	schedule_work(&state->ycrcb_work);

	disable_irq_nosync(state->ycrcb_irq);

	return IRQ_HANDLED;
}
#endif

static ssize_t store_dbg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int reg, val;
    struct i2c_client *client = to_i2c_client(dev);

    sscanf(buf, "%x %x", &reg, &val);
    i2c_smbus_write_byte_data(client, reg, val);
    printk("write 0x%02x = 0x%02x, ret = 0x%02x\n", reg, val, i2c_smbus_read_byte_data(client, reg));

    return count;
}

static struct device_attribute t132b_dev_attr = {
    .attr = {
         .name = "reg",
         .mode = S_IRUSR | S_IWUSR,
         },
    .store = store_dbg,
};

/*
 * Generic i2c probe
 * concerning the addresses: i2c wants 7 bit (without the r/w bit), so '>>1'
 */

static int t132b_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct t132b_state *state;
	struct v4l2_subdev *sd;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	state = kzalloc(sizeof(struct t132b_state), GFP_KERNEL);
	if (state == NULL) {
		ret = -ENOMEM;
		goto err;
	}

#if 0 /* TODO: add auto detected from interrupt with GPIO pins */
	rk30_mux_api_set(GPIO0B7_I2S8CHSDO3_NAME, GPIO0B_GPIO0B7);
	ret = gpio_request(T132B_CVBS_IN_DET, "TW_CVBS_INT");
	if (ret != 0)
	{
		gpio_free(T132B_CVBS_IN_DET);
		printk(KERN_ERR "request T132B_CVBS_IN_DET pin fail!\n");
		return -1;
	}
	gpio_direction_input(T132B_CVBS_IN_DET);

	ret = gpio_request(T132B_YCRCB_IN_DET, "TW_YCRCB_INT");
	if (ret != 0)
	{
		gpio_free(T132B_YCRCB_IN_DET);
		printk(KERN_ERR "request T132B_YCRCB_IN_DET  pin fail!\n");
		return -1;
	}
	gpio_direction_input(T132B_YCRCB_IN_DET);

	state->cvbs_irq = gpio_to_irq(T132B_CVBS_IN_DET);
	state->ycrcb_irq = gpio_to_irq(T132B_YCRCB_IN_DET);

	INIT_WORK(&state->cvbs_work, t132b_cvbs_work);
	INIT_WORK(&state->ycrcb_work, t132b_ycrcb_work);
#endif

	mutex_init(&state->mutex);
	/* TODO: add auto detected from interrupt with GPIO pins */
	state->autodetect = false;	// true;
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &t132b_ops);

	state->icd = icd;
	icd->ops = &t132b_sensor_ops;
	icd->user_width = t132b_pic_sizes[0].width;
	icd->user_height = t132b_pic_sizes[0].height;
	icd->colorspace = t132b_colour_fmts[0].colorspace;

	t132b_pwr_ctl(icd, 1);
	ret = 0;

	i2c_smbus_write_byte_data(client, 0xFF, 0x00);	// select page0
	ret = i2c_smbus_read_byte_data(client, T132B_CHIPID_REG);
	if (ret != T132B_CHIPID_T132) {
		printk("ID: 0x%02X not is T132B\n", ret);
		goto err_unreg_subdev;
	}

	t132b_pwr_ctl(icd, 0);

    ret = device_create_file(&client->dev, &t132b_dev_attr);
    if (ret) {
        printk("create device file failed!\n");
    }

	v4l_info(client, "chip probe success @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	return 0;

err_unreg_subdev:
	t132b_pwr_ctl(icd, 0);
	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(state);
err:
	printk(KERN_ERR DRIVER_NAME ": Failed to probe: %d\n", ret);
	return ret;
}

static int t132b_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct t132b_state *state = to_state(sd);

#if DRIVER_FOR_ROCKCHIP
	if (state->cvbs_irq > 0) {
		free_irq(state->cvbs_irq,state);
		if (cancel_work_sync(&state->cvbs_work)) {
			/*
			 * Work was pending, therefore we need to enable
			 * IRQ here to balance the disable_irq() done in the
			 * interrupt handler.
			 */
			enable_irq(state->cvbs_irq);
		}
	}

	if (state->ycrcb_irq > 0) {
		free_irq(state->ycrcb_irq,state);
		if (cancel_work_sync(&state->ycrcb_work)) {
			/*
			 * Work was pending, therefore we need to enable
			 * IRQ here to balance the disable_irq() done in the
			 * interrupt handler.
			 */
			enable_irq(state->ycrcb_irq);
		}
	}
#endif

	mutex_destroy(&state->mutex);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id t132b_id[] = {
	{DRIVER_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, t132b_id);

static struct i2c_driver t132b_driver = {
	.driver = {
//		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
	.probe		= t132b_probe,
	.remove		= __devexit_p(t132b_remove),
	.id_table	= t132b_id,
};

static __init int t132b_init(void)
{
    printk("\n%s..%s.. add driver\n",__FUNCTION__, DRIVER_NAME);
	return i2c_add_driver(&t132b_driver);
}

static __exit void t132b_exit(void)
{
	i2c_del_driver(&t132b_driver);
}

module_init(t132b_init);
module_exit(t132b_exit);

MODULE_DESCRIPTION("Analog Devices T132BT video decoder driver");
MODULE_AUTHOR("Ken Terawins");
MODULE_LICENSE("GPL v2");

