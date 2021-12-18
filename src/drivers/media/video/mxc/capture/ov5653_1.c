/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"
#include <linux/proc_fs.h>

#define OV5653_VOLTAGE_ANALOG               2800000
#define OV5653_VOLTAGE_DIGITAL_CORE         1500000
#define OV5653_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 15

#define OV5653_XCLK_MIN 6000000
#define OV5653_XCLK_MAX 24000000

/* OV5653 Camera Auto Focus Registers */
#define REG_CMD_MAIN			0x3024
#define REG_STA_FOCUS			0x3027
#define REG_STA_ZONE			0x3026
#define REG_CMD_TAG				0x3025
#define REG_CMD_PARA3 			0x5082
#define REG_CMD_PARA2 			0x5083
#define REG_CMD_PARA1			0x5084
#define REG_CMD_PARA0 			0x5085


/* ov5653 Auto Focus Commands and Responses */
#define	S_STARTUP			0xFA
#define	S_FIRWARE			0xFF
#define	S_STARTUP			0xFA
#define	S_ERROR				0xFE
#define	S_DRVICERR			0xEE
#define	S_IDLE				0x00
#define	S_FOCUSING			0x01
#define	S_FOCUSED			0x02
#define	S_CAPTURE			0x12
#define	S_STEP				0x20

/*OV5653 AWB Registers*/
#define REG_AWB_MANUAL			0x3406
#define REG_AWB_R_GAIN_HIGH		0x3400
#define REG_AWB_R_GAIN_LOW		0x3401
#define REG_AWB_G_GAIN_HIGH		0x3402
#define REG_AWB_G_GAIN_LOW		0x3403
#define REG_AWB_B_GAIN_HIGH		0x3404
#define REG_AWB_B_GAIN_LOW		0x3405

#define	CMD_ENABLE_OVERLAY		0x01
#define	CMD_DISABLE_OVERLAY		0x02
#define	CMD_SINGLE_FOCUS_MODE		0x03
#define	CMD_CONST_FOCUS_MODE		0x04
#define	CMD_STEP_FOCUS_MODE		0x05
#define	CMD_PAUSE			0x06
#define	CMD_IDLE_MODE			0x08
#define	CMD_SET_ZONE_MODE		0x10
#define	CMD_UPDATE_ZONE_MODE		0x12
#define	CMD_MOTOR_MODE			0x20
#define	CMD_SCAN_MODE			0x30

#define OV5653_CHIP_ID_HIGH_BYTE	0x300A
#define OV5653_CHIP_ID_LOW_BYTE		0x300B

#define OV5653_MAX_EXP_LINES 1984
 // Interrupts for mode setting tables
#define INTERRUPT_FUNC_AEC_POLYGON 0xDEAD
#define INTERRUPT_FUNC_AEC_CONTROL 0xBABE
#define INTERRUPT_FUNC_AWB_CONTROL 0xFACE

#define MIN_GAIN_VALUE 1
#define MAX_GAIN_VALUE 31
#define MAX_EXPOSURE_VALUE 20000

enum ov5653_mode {
	ov5653_mode_MIN = 0,
	ov5653_mode_VGA_640_480 = 0,
	ov5653_mode_QVGA_320_240 = 1,
	ov5653_mode_NTSC_720_480 = 2,
	ov5653_mode_PAL_720_576 = 3,
	ov5653_mode_720P_1280_720 = 4,
	ov5653_mode_1080P_1920_1080 = 5,
	ov5653_mode_QSXGA_2592_1944 = 6,
	ov5653_mode_QCIF_176_144 = 7,
	ov5653_mode_XGA_1024_768 = 8,
	ov5653_mode_800_600 = 9,
	ov5653_mode_1600_1200 = 10,
	ov5653_mode_160_120 = 11,
	ov5653_mode_MAX = 11
};

enum ov5653_frame_rate {
	ov5653_15_fps,
	ov5653_30_fps
};

static unsigned int prev_exposure_table[ov5653_mode_MAX] = {4000,};

static int ov5653_framerates[] = {
	[ov5653_15_fps] = 15,
	[ov5653_30_fps] = 30,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov5653_mode_info {
	enum ov5653_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

static char procname[0xff];

static int prev_width = 0;
static int prev_height = 0;

static unsigned int req_auto_exp = 1;
static unsigned short req_exposure = 0;
static unsigned short req_gain = 0;
// Gain lookup table, gain request values:             
static char gain_bits_table[MAX_GAIN_VALUE+1] = {
//  0      1    2     3    4      5     6     7     8     9     10    11    12    13    14    15    16  etc....
    0,     0,   0x10, 0x18, 0x30, 0x34, 0x38, 0x3c, 0x70, 0x72, 0x74, 0x76, 0x78, 0x7a, 0x7c, 0x7d, 0x7f,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff 
};
 

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ov5653_data;

static struct reg_value ov5653_rotate_none_VGA[] = {
	{0x3818, 0xc1, 0x00, 0x00}, {0x3621, 0x87, 0x00, 0x00},
};

static struct reg_value ov5653_rotate_vert_flip_VGA[] = {
	{0x3818, 0x20, 0xbf, 0x00}, {0x3621, 0x20, 0xff, 0x00},
};

static struct reg_value ov5653_rotate_horiz_flip_VGA[] = {
	{0x3818, 0x81, 0x00, 0x01}, {0x3621, 0xa7, 0x00, 0x00},
};

static struct reg_value ov5653_rotate_180_VGA[] = {
	{0x3818, 0x60, 0xff, 0x00}, {0x3621, 0x00, 0xdf, 0x00},
};


static struct reg_value ov5653_rotate_none_FULL[] = {
	{0x3818, 0xc0, 0x00, 0x00}, {0x3621, 0x09, 0x00, 0x00},
};

static struct reg_value ov5653_rotate_vert_flip_FULL[] = {
	{0x3818, 0x20, 0xbf, 0x01}, {0x3621, 0x20, 0xff, 0x00},
};

static struct reg_value ov5653_rotate_horiz_flip_FULL[] = {
	{0x3818, 0x80, 0x00, 0x01}, {0x3621, 0x29, 0x00, 0x00},
};

static struct reg_value ov5653_rotate_180_FULL[] = {
	{0x3818, 0x60, 0xff, 0x00}, {0x3621, 0x00, 0xdf, 0x00},
};


static struct reg_value ov5653_initial_setting[] = {
	{0x3103, 0x93, 0, 0}, {0x3008, 0x82, 0, 0}, {0x3017, 0x7f, 0, 0},
	{0x3018, 0xfc, 0, 0}, {0x3810, 0xc2, 0, 0}, {0x3615, 0xf0, 0, 0},
	{0x3000, 0x00, 0, 0}, {0x3001, 0x00, 0, 0}, {0x3002, 0x5c, 0, 0},
	{0x3003, 0x00, 0, 0}, {0x3004, 0xff, 0, 0}, {0x3005, 0xff, 0, 0},
	{0x3006, 0x43, 0, 0}, {0x3007, 0x37, 0, 0}, {0x3011, 0x08, 0, 0},
	{0x3010, 0x00, 0, 0}, {0x460c, 0x22, 0, 0}, {0x3815, 0x04, 0, 0},
	{0x370c, 0xa0, 0, 0}, {0x3602, 0xfc, 0, 0}, {0x3612, 0xff, 0, 0},
	{0x3634, 0xc0, 0, 0}, {0x3613, 0x00, 0, 0}, {0x3605, 0x7c, 0, 0},
	{0x3621, 0x09, 0, 0}, {0x3622, 0x60, 0, 0}, {0x3604, 0x40, 0, 0},
	{0x3603, 0xa7, 0, 0}, {0x3603, 0x27, 0, 0}, {0x4000, 0x21, 0, 0},
	{0x401d, 0x22, 0, 0}, {0x3600, 0x54, 0, 0}, {0x3605, 0x04, 0, 0},
	{0x3606, 0x3f, 0, 0}, {0x3c00, 0x04, 0, 0}, {0x3c01, 0x80, 0, 0},
	{0x5000, 0x4f, 0, 0}, {0x5020, 0x04, 0, 0}, {0x5181, 0x79, 0, 0},
	{0x5182, 0x00, 0, 0}, {0x5185, 0x22, 0, 0}, {0x5197, 0x01, 0, 0},
	{0x5001, 0xff, 0, 0}, {0x5500, 0x0a, 0, 0}, {0x5504, 0x00, 0, 0},
	{0x5505, 0x7f, 0, 0}, {0x5080, 0x08, 0, 0}, {0x300e, 0x18, 0, 0},
	{0x4610, 0x00, 0, 0}, {0x471d, 0x05, 0, 0}, {0x4708, 0x06, 0, 0},
	{0x3808, 0x02, 0, 0}, {0x3809, 0x80, 0, 0}, {0x380a, 0x01, 0, 0},
	{0x380b, 0xe0, 0, 0}, {0x380e, 0x07, 0, 0}, {0x380f, 0xd0, 0, 0},
	{0x501f, 0x00, 0, 0}, {0x5000, 0x4f, 0, 0}, {0x4300, 0x30, 0, 0},
	{0x3503, 0x07, 0, 0}, {0x3501, 0x73, 0, 0}, {0x3502, 0x80, 0, 0},
	{0x350b, 0x00, 0, 0}, {0x3503, 0x07, 0, 0}, {0x3824, 0x11, 0, 0},
	{0x3501, 0x1e, 0, 0}, {0x3502, 0x80, 0, 0}, {0x350b, 0x7f, 0, 0},
	{0x380c, 0x0c, 0, 0}, {0x380d, 0x80, 0, 0}, {0x380e, 0x03, 0, 0},
	{0x380f, 0xe8, 0, 0}, {0x3a0d, 0x04, 0, 0}, {0x3a0e, 0x03, 0, 0},
	{0x3818, 0xc1, 0, 0}, {0x3705, 0xdb, 0, 0}, {0x370a, 0x81, 0, 0},
	{0x3801, 0x80, 0, 0}, {0x3621, 0x87, 0, 0}, {0x3801, 0x50, 0, 0},
	{0x3803, 0x08, 0, 0}, {0x3827, 0x08, 0, 0}, {0x3810, 0x40, 0, 0},
	{0x3804, 0x05, 0, 0}, {0x3805, 0x00, 0, 0}, {0x5682, 0x05, 0, 0},
	{0x5683, 0x00, 0, 0}, {0x3806, 0x03, 0, 0}, {0x3807, 0xc0, 0, 0},
	{0x5686, 0x03, 0, 0}, {0x5687, 0xbc, 0, 0}, {0x3a00, 0x78, 0, 0},
	{0x3a1a, 0x05, 0, 0}, {0x3a13, 0x30, 0, 0}, {0x3a18, 0x00, 0, 0},
	{0x3a19, 0x7c, 0, 0}, {0x3a08, 0x12, 0, 0}, {0x3a09, 0xc0, 0, 0},
	{0x3a0a, 0x0f, 0, 0}, {0x3a0b, 0xa0, 0, 0}, {0x350c, 0x07, 0, 0},
	{0x350d, 0xd0, 0, 0}, {0x3500, 0x00, 0, 0}, {0x3501, 0x00, 0, 0},
	{0x3502, 0x00, 0, 0}, {0x350a, 0x00, 0, 0}, {0x350b, 0x00, 0, 0},
	{0x3503, 0x00, 0, 0}, {0x528a, 0x02, 0, 0}, {0x528b, 0x04, 0, 0},
	{0x528c, 0x08, 0, 0}, {0x528d, 0x08, 0, 0}, {0x528e, 0x08, 0, 0},
	{0x528f, 0x10, 0, 0}, {0x5290, 0x10, 0, 0}, {0x5292, 0x00, 0, 0},
	{0x5293, 0x02, 0, 0}, {0x5294, 0x00, 0, 0}, {0x5295, 0x02, 0, 0},
	{0x5296, 0x00, 0, 0}, {0x5297, 0x02, 0, 0}, {0x5298, 0x00, 0, 0},
	{0x5299, 0x02, 0, 0}, {0x529a, 0x00, 0, 0}, {0x529b, 0x02, 0, 0},
	{0x529c, 0x00, 0, 0}, {0x529d, 0x02, 0, 0}, {0x529e, 0x00, 0, 0},
	{0x529f, 0x02, 0, 0}, //{0x3a0f, 0x3c, 0, 0}, {0x3a10, 0x30, 0, 0},
	{0x3a1b, 0x3c, 0, 0}, {0x3a1e, 0x30, 0, 0}, {0x3a11, 0x70, 0, 0},
	{0x3a1f, 0x10, 0, 0}, {0x3030, 0x0b, 0, 0}, {0x3a02, 0x00, 0, 0},
	{0x3a03, 0x7d, 0, 0}, {0x3a04, 0x00, 0, 0}, {0x3a14, 0x00, 0, 0},
	{0x3a15, 0x7d, 0, 0}, {0x3a16, 0x00, 0, 0}, {0x3a00, 0x78, 0, 0},
	{0x5193, 0x70, 0, 0}, {0x589b, 0x04, 0, 0}, {0x589a, 0xc5, 0, 0},
	{0x401e, 0x20, 0, 0}, {0x4001, 0x42, 0, 0}, {0x401c, 0x04, 0, 0},
	{0x528a, 0x01, 0, 0}, {0x528b, 0x04, 0, 0}, {0x528c, 0x08, 0, 0},
	{0x528d, 0x10, 0, 0}, {0x528e, 0x20, 0, 0}, {0x528f, 0x28, 0, 0},
	{0x5290, 0x30, 0, 0}, {0x5292, 0x00, 0, 0}, {0x5293, 0x01, 0, 0},
	{0x5294, 0x00, 0, 0}, {0x5295, 0x04, 0, 0}, {0x5296, 0x00, 0, 0},
	{0x5297, 0x08, 0, 0}, {0x5298, 0x00, 0, 0}, {0x5299, 0x10, 0, 0},
	{0x529a, 0x00, 0, 0}, {0x529b, 0x20, 0, 0}, {0x529c, 0x00, 0, 0},
	{0x529d, 0x28, 0, 0}, {0x529e, 0x00, 0, 0}, {0x529f, 0x30, 0, 0},
	{0x5282, 0x00, 0, 0}, {0x5300, 0x00, 0, 0}, {0x5301, 0x20, 0, 0},
	{0x5302, 0x00, 0, 0}, {0x5303, 0x7c, 0, 0}, {0x530c, 0x00, 0, 0},
	{0x530d, 0x0c, 0, 0}, {0x530e, 0x20, 0, 0}, {0x530f, 0x80, 0, 0},
	{0x5310, 0x20, 0, 0}, {0x5311, 0x80, 0, 0}, {0x5308, 0x20, 0, 0},
	{0x5309, 0x40, 0, 0}, {0x5304, 0x00, 0, 0}, {0x5305, 0x30, 0, 0},
	{0x5306, 0x00, 0, 0}, {0x5307, 0x80, 0, 0}, {0x5314, 0x08, 0, 0},
	{0x5315, 0x20, 0, 0}, {0x5319, 0x30, 0, 0}, {0x5316, 0x10, 0, 0},
	{0x5317, 0x00, 0, 0}, {0x5318, 0x02, 0, 0}, {0x5380, 0x01, 0, 0},
	{0x5381, 0x00, 0, 0}, {0x5382, 0x00, 0, 0}, {0x5383, 0x4e, 0, 0},
	{0x5384, 0x00, 0, 0}, {0x5385, 0x0f, 0, 0}, {0x5386, 0x00, 0, 0},
	{0x5387, 0x00, 0, 0}, {0x5388, 0x01, 0, 0}, {0x5389, 0x15, 0, 0},
	{0x538a, 0x00, 0, 0}, {0x538b, 0x31, 0, 0}, {0x538c, 0x00, 0, 0},
	{0x538d, 0x00, 0, 0}, {0x538e, 0x00, 0, 0}, {0x538f, 0x0f, 0, 0},
	{0x5390, 0x00, 0, 0}, {0x5391, 0xab, 0, 0}, {0x5392, 0x00, 0, 0},
	{0x5393, 0xa2, 0, 0}, {0x5394, 0x08, 0, 0}, {0x5480, 0x14, 0, 0},
	{0x5481, 0x21, 0, 0}, {0x5482, 0x36, 0, 0}, {0x5483, 0x57, 0, 0},
	{0x5484, 0x65, 0, 0}, {0x5485, 0x71, 0, 0}, {0x5486, 0x7d, 0, 0},
	{0x5487, 0x87, 0, 0}, {0x5488, 0x91, 0, 0}, {0x5489, 0x9a, 0, 0},
	{0x548a, 0xaa, 0, 0}, {0x548b, 0xb8, 0, 0}, {0x548c, 0xcd, 0, 0},
	{0x548d, 0xdd, 0, 0}, {0x548e, 0xea, 0, 0}, {0x548f, 0x1d, 0, 0},
	{0x5490, 0x05, 0, 0}, {0x5491, 0x00, 0, 0}, {0x5492, 0x04, 0, 0},
	{0x5493, 0x20, 0, 0}, {0x5494, 0x03, 0, 0}, {0x5495, 0x60, 0, 0},
	{0x5496, 0x02, 0, 0}, {0x5497, 0xb8, 0, 0}, {0x5498, 0x02, 0, 0},
	{0x5499, 0x86, 0, 0}, {0x549a, 0x02, 0, 0}, {0x549b, 0x5b, 0, 0},
	{0x549c, 0x02, 0, 0}, {0x549d, 0x3b, 0, 0}, {0x549e, 0x02, 0, 0},
	{0x549f, 0x1c, 0, 0}, {0x54a0, 0x02, 0, 0}, {0x54a1, 0x04, 0, 0},
	{0x54a2, 0x01, 0, 0}, {0x54a3, 0xed, 0, 0}, {0x54a4, 0x01, 0, 0},
	{0x54a5, 0xc5, 0, 0}, {0x54a6, 0x01, 0, 0}, {0x54a7, 0xa5, 0, 0},
	{0x54a8, 0x01, 0, 0}, {0x54a9, 0x6c, 0, 0}, {0x54aa, 0x01, 0, 0},
	{0x54ab, 0x41, 0, 0}, {0x54ac, 0x01, 0, 0}, {0x54ad, 0x20, 0, 0},
	{0x54ae, 0x00, 0, 0}, {0x54af, 0x16, 0, 0}, {0x54b0, 0x01, 0, 0},
	{0x54b1, 0x20, 0, 0}, {0x54b2, 0x00, 0, 0}, {0x54b3, 0x10, 0, 0},
	{0x54b4, 0x00, 0, 0}, {0x54b5, 0xf0, 0, 0}, {0x54b6, 0x00, 0, 0},
	{0x54b7, 0xdf, 0, 0}, {0x5402, 0x3f, 0, 0}, {0x5403, 0x00, 0, 0},
	{0x3406, 0x00, 0, 0}, {0x5180, 0xff, 0, 0}, {0x5181, 0x52, 0, 0},
	{0x5182, 0x11, 0, 0}, {0x5183, 0x14, 0, 0}, {0x5184, 0x25, 0, 0},
	{0x5185, 0x24, 0, 0}, {0x5186, 0x06, 0, 0}, {0x5187, 0x08, 0, 0},
	{0x5188, 0x08, 0, 0}, {0x5189, 0x7c, 0, 0}, {0x518a, 0x60, 0, 0},
	{0x518b, 0xb2, 0, 0}, {0x518c, 0xb2, 0, 0}, {0x518d, 0x44, 0, 0},
	{0x518e, 0x3d, 0, 0}, {0x518f, 0x58, 0, 0}, {0x5190, 0x46, 0, 0},
	{0x5191, 0xf8, 0, 0}, {0x5192, 0x04, 0, 0}, {0x5193, 0x70, 0, 0},
	{0x5194, 0xf0, 0, 0}, {0x5195, 0xf0, 0, 0}, {0x5196, 0x03, 0, 0},
	{0x5197, 0x01, 0, 0}, {0x5198, 0x04, 0, 0}, {0x5199, 0x12, 0, 0},
	{0x519a, 0x04, 0, 0}, {0x519b, 0x00, 0, 0}, {0x519c, 0x06, 0, 0},
	{0x519d, 0x82, 0, 0}, {0x519e, 0x00, 0, 0}, {0x5025, 0x80, 0, 0},
//	{0x3a0f, 0x38, 0, 0}, {0x3a10, 0x30, 0, 0}, {0x3a1b, 0x3a, 0, 0},
	{0x3a1e, 0x2e, 0, 0}, {0x3a11, 0x60, 0, 0}, {0x3a1f, 0x10, 0, 0},
	{0x5688, 0xa6, 0, 0}, {0x5689, 0x6a, 0, 0}, {0x568a, 0xea, 0, 0},
	{0x568b, 0xae, 0, 0}, {0x568c, 0xa6, 0, 0}, {0x568d, 0x6a, 0, 0},
	{0x568e, 0x62, 0, 0}, {0x568f, 0x26, 0, 0}, {0x5583, 0x40, 0, 0},
	{0x5584, 0x40, 0, 0}, {0x5580, 0x02, 0, 0}, {0x5000, 0xcf, 0, 0},
	{0x5800, 0x27, 0, 0}, {0x5801, 0x19, 0, 0}, {0x5802, 0x12, 0, 0},
	{0x5803, 0x0f, 0, 0}, {0x5804, 0x10, 0, 0}, {0x5805, 0x15, 0, 0},
	{0x5806, 0x1e, 0, 0}, {0x5807, 0x2f, 0, 0}, {0x5808, 0x15, 0, 0},
	{0x5809, 0x0d, 0, 0}, {0x580a, 0x0a, 0, 0}, {0x580b, 0x09, 0, 0},
	{0x580c, 0x0a, 0, 0}, {0x580d, 0x0c, 0, 0}, {0x580e, 0x12, 0, 0},
	{0x580f, 0x19, 0, 0}, {0x5810, 0x0b, 0, 0}, {0x5811, 0x07, 0, 0},
	{0x5812, 0x04, 0, 0}, {0x5813, 0x03, 0, 0}, {0x5814, 0x03, 0, 0},
	{0x5815, 0x06, 0, 0}, {0x5816, 0x0a, 0, 0}, {0x5817, 0x0f, 0, 0},
	{0x5818, 0x0a, 0, 0}, {0x5819, 0x05, 0, 0}, {0x581a, 0x01, 0, 0},
	{0x581b, 0x00, 0, 0}, {0x581c, 0x00, 0, 0}, {0x581d, 0x03, 0, 0},
	{0x581e, 0x08, 0, 0}, {0x581f, 0x0c, 0, 0}, {0x5820, 0x0a, 0, 0},
	{0x5821, 0x05, 0, 0}, {0x5822, 0x01, 0, 0}, {0x5823, 0x00, 0, 0},
	{0x5824, 0x00, 0, 0}, {0x5825, 0x03, 0, 0}, {0x5826, 0x08, 0, 0},
	{0x5827, 0x0c, 0, 0}, {0x5828, 0x0e, 0, 0}, {0x5829, 0x08, 0, 0},
	{0x582a, 0x06, 0, 0}, {0x582b, 0x04, 0, 0}, {0x582c, 0x05, 0, 0},
	{0x582d, 0x07, 0, 0}, {0x582e, 0x0b, 0, 0}, {0x582f, 0x12, 0, 0},
	{0x5830, 0x18, 0, 0}, {0x5831, 0x10, 0, 0}, {0x5832, 0x0c, 0, 0},
	{0x5833, 0x0a, 0, 0}, {0x5834, 0x0b, 0, 0}, {0x5835, 0x0e, 0, 0},
	{0x5836, 0x15, 0, 0}, {0x5837, 0x19, 0, 0}, {0x5838, 0x32, 0, 0},
	{0x5839, 0x1f, 0, 0}, {0x583a, 0x18, 0, 0}, {0x583b, 0x16, 0, 0},
	{0x583c, 0x17, 0, 0}, {0x583d, 0x1e, 0, 0}, {0x583e, 0x26, 0, 0},
	{0x583f, 0x53, 0, 0}, {0x5840, 0x10, 0, 0}, {0x5841, 0x0f, 0, 0},
	{0x5842, 0x0d, 0, 0}, {0x5843, 0x0c, 0, 0}, {0x5844, 0x0e, 0, 0},
	{0x5845, 0x09, 0, 0}, {0x5846, 0x11, 0, 0}, {0x5847, 0x10, 0, 0},
	{0x5848, 0x10, 0, 0}, {0x5849, 0x10, 0, 0}, {0x584a, 0x10, 0, 0},
	{0x584b, 0x0e, 0, 0}, {0x584c, 0x10, 0, 0}, {0x584d, 0x10, 0, 0},
	{0x584e, 0x11, 0, 0}, {0x584f, 0x10, 0, 0}, {0x5850, 0x0f, 0, 0},
	{0x5851, 0x0c, 0, 0}, {0x5852, 0x0f, 0, 0}, {0x5853, 0x10, 0, 0},
	{0x5854, 0x10, 0, 0}, {0x5855, 0x0f, 0, 0}, {0x5856, 0x0e, 0, 0},
	{0x5857, 0x0b, 0, 0}, {0x5858, 0x10, 0, 0}, {0x5859, 0x0d, 0, 0},
	{0x585a, 0x0d, 0, 0}, {0x585b, 0x0c, 0, 0}, {0x585c, 0x0c, 0, 0},
	{0x585d, 0x0c, 0, 0}, {0x585e, 0x0b, 0, 0}, {0x585f, 0x0c, 0, 0},
	{0x5860, 0x0c, 0, 0}, {0x5861, 0x0c, 0, 0}, {0x5862, 0x0d, 0, 0},
	{0x5863, 0x08, 0, 0}, {0x5864, 0x11, 0, 0}, {0x5865, 0x18, 0, 0},
	{0x5866, 0x18, 0, 0}, {0x5867, 0x19, 0, 0}, {0x5868, 0x17, 0, 0},
	{0x5869, 0x19, 0, 0}, {0x586a, 0x16, 0, 0}, {0x586b, 0x13, 0, 0},
	{0x586c, 0x13, 0, 0}, {0x586d, 0x12, 0, 0}, {0x586e, 0x13, 0, 0},
	{0x586f, 0x16, 0, 0}, {0x5870, 0x14, 0, 0}, {0x5871, 0x12, 0, 0},
	{0x5872, 0x10, 0, 0}, {0x5873, 0x11, 0, 0}, {0x5874, 0x11, 0, 0},
	{0x5875, 0x16, 0, 0}, {0x5876, 0x14, 0, 0}, {0x5877, 0x11, 0, 0},
	{0x5878, 0x10, 0, 0}, {0x5879, 0x0f, 0, 0}, {0x587a, 0x10, 0, 0},
	{0x587b, 0x14, 0, 0}, {0x587c, 0x13, 0, 0}, {0x587d, 0x12, 0, 0},
	{0x587e, 0x11, 0, 0}, {0x587f, 0x11, 0, 0}, {0x5880, 0x12, 0, 0},
	{0x5881, 0x15, 0, 0}, {0x5882, 0x14, 0, 0}, {0x5883, 0x15, 0, 0},
	{0x5884, 0x15, 0, 0}, {0x5885, 0x15, 0, 0}, {0x5886, 0x13, 0, 0},
	{0x5887, 0x17, 0, 0}, {0x3710, 0x10, 0, 0}, {0x3632, 0x51, 0, 0},
	{0x3702, 0x10, 0, 0}, {0x3703, 0xb2, 0, 0}, {0x3704, 0x18, 0, 0},
	{0x370b, 0x40, 0, 0}, {0x370d, 0x03, 0, 0}, {0x3631, 0x01, 0, 0},
	{0x3632, 0x52, 0, 0}, {0x3606, 0x24, 0, 0}, {0x3620, 0x96, 0, 0},
	{0x5785, 0x07, 0, 0}, {0x3a13, 0x30, 0, 0}, {0x3600, 0x52, 0, 0},
	{0x3604, 0x48, 0, 0}, {0x3606, 0x1b, 0, 0}, {0x370d, 0x0b, 0, 0},
	{0x370f, 0xc0, 0, 0}, {0x3709, 0x01, 0, 0}, {0x3823, 0x00, 0, 0},
	{0x5007, 0x00, 0, 0}, {0x5009, 0x00, 0, 0}, {0x5011, 0x00, 0, 0},
	{0x5013, 0x00, 0, 0}, {0x519e, 0x00, 0, 0}, {0x5086, 0x00, 0, 0},
	{0x5087, 0x00, 0, 0}, {0x5088, 0x00, 0, 0}, {0x5089, 0x00, 0, 0},
	{0x302b, 0x00, 0, 300},
};



static struct reg_value ov5653_setting_15fps_QCIF_176_144[] = {
	{0x3a00, 0x78, 0, 0},
};

static struct reg_value ov5653_setting_30fps_QCIF_176_144[] = {
	{0x3a00, 0x78, 0, 0},
};


static struct reg_value ov5653_setting_15fps_QSXGA_2592_1944[] = {
{0x3008, 0x82, 0, 0},
{0x3008, 0x42, 0, 0},
{0x3103, 0x93, 0, 0},
{0x3b07, 0x0c, 0, 0},
{0x3017, 0xff, 0, 0},
{0x3018, 0xfc, 0, 0},
{0x3706, 0x41, 0, 0},
{0x3703, 0xe6, 0, 0}, 
{0x3613, 0x44, 0, 0},
{0x3630, 0x22, 0, 0},
{0x3605, 0x04, 0, 0},
{0x3606, 0x3f, 0, 0},
{0x3712, 0x13, 0, 0},
{0x370e, 0x00, 0, 0},
{0x370b, 0x40, 0, 0},
{0x3600, 0x54, 0, 0},
{0x3601, 0x05, 0, 0},
{0x3713, 0x22, 0, 0},
{0x3714, 0x27, 0, 0},
{0x3631, 0x22, 0, 0},
{0x3612, 0x1a, 0, 0},
{0x3604, 0x40, 0, 0},
{0x3705, 0xda, 0, 0},
{0x3709, 0x40, 0, 0},
{0x370a, 0x40, 0, 0},
{0x370c, 0x0, 0, 0},
{0x3710, 0x28, 0, 0},
{0x3702, 0x3a, 0, 0},
{0x3704, 0x18, 0, 0},
//{0x3a18, 0x00, 0, 0},
//{0x3a19, 0xf8, 0, 0},
{0x3a00, 0x38, 0, 0}, //38
{0x3800, 0x02, 0, 0}, //2
{0x3801, 0x54, 0, 0}, //54
{0x3803, 0x0c, 0, 0}, //c
{0x380c, 0x0c, 0, 0},
{0x380d, 0xb4, 0, 0},
{0x380e, 0x07, 0, 0},
{0x380f, 0xb0, 0, 0},
{0x3830, 0x50, 0, 0},
{0x3a08, 0x12, 0, 0},
{0x3a09, 0x70, 0, 0},
{0x3a0a, 0x0f, 0, 0},
{0x3a0b, 0x60, 0, 0},
{0x3a0d, 0x06, 0, 0},
{0x3a0e, 0x06, 0, 0},
{0x3a13, 0x54, 0, 0}, // changed from 54
{0x3815, 0x82, 0, 0},
{0x5059, 0x80, 0, 0},
{0x3615, 0x52, 0, 0},
{0x505a, 0x0a, 0, 0},
{0x505b, 0x2e, 0, 0},
{0x3a1a, 0x06, 0, 0},
{INTERRUPT_FUNC_AEC_CONTROL, 0, 0, 0},
{0x3623, 0x01, 0, 0},
{0x3633, 0x24, 0, 0},
{0x3c01, 0x34, 0, 0},
{0x3c04, 0x28, 0, 0},
{0x3c05, 0x98, 0, 0},
{0x3c07, 0x07, 0, 0},
{0x3c09, 0xc2, 0, 0},
{0x3804, 0x0a, 0, 0}, //4, a
{0x3805, 0x20, 0, 0}, //0, 20
{0x3806, 0x07, 0, 0}, //3, 7
{0x3807, 0x98, 0, 0}, //0, 98
{0x3808, 0x0a, 0, 0},
{0x3809, 0x20, 0, 0},
{0x380a, 0x07, 0, 0},
{0x380b, 0x98, 0, 0},
{INTERRUPT_FUNC_AEC_POLYGON, 0, 0, 0},
{0x4000, 0x05, 0, 0}, // 0x05 // BLC enable/disable
{0x401d, 0x28, 0, 0},//0x28
{0x4001, 0x02, 0, 0}, // brightness
{0x401c, 0x46, 0, 0},
{0x5046, 0x09, 0, 0},
{0x3810, 0x40, 0, 0},
{0x3836, 0x41, 0, 0},
{0x505f, 0x04, 0, 0},
{0x5000, 0xfe, 0, 0}, //fe
{0x5001, 0x01, 0, 0}, //1
{0x5002, 0x00, 0, 0},
{0x503d, 0x00, 0, 0},
{0x5901, 0x00, 0, 0}, //0
{0x585a, 0x01, 0, 0},
{0x585b, 0x2c, 0, 0},
{0x585c, 0x01, 0, 0},
{0x585d, 0x93, 0, 0},
{0x585e, 0x01, 0, 0},
{0x585f, 0x90, 0, 0},
{0x5860, 0x01, 0, 0},
{0x5861, 0x0d, 0, 0},
{0x5180, 0xc0, 0, 0},
{0x5184, 0x01, 0, 0},
{0x470a, 0x00, 0, 0},
{0x470b, 0x00, 0, 0},
{0x470c, 0x00, 0, 0},
{0x300f, 0x8e, 0, 0},
{0x3603, 0xa7, 0, 0},
{0x3632, 0x55, 0, 0},
{0x3620, 0x56, 0, 0},
{0x3621, 0x2f, 0, 0},
{0x381a, 0x3c, 0, 0},
{0x3818, 0xc0, 0, 0},
{0x3631, 0x36, 0, 0},
{0x3632, 0x5f, 0, 0},
{0x3711, 0x24, 0, 0},
{0x401f, 0x03, 0, 0},
{INTERRUPT_FUNC_AWB_CONTROL, 0, 0, 0},
{0x3008, 0x02, 0, 0},

{0x3406, 0x00, 0, 0}, // Back to auto AWB. Have to set it here!
// Needed 3 sec delay to stabilize
//{0x3008, 0x02, 0, 3000},
};

static struct reg_value ov5653_setting_30fps_800_600[] = {
 {0x3010, 0x02, 0, 0}, // orig
};

static struct reg_value ov5653_setting_30fps_1600_1200[] = {
  {0x3008,0x02, 0, 0}
};



static struct reg_value ov5653_setting_QSXGA_2_VGA[] = {
	{0x3012, 0x02, 0, 0}, {0x3010, 0x00, 0, 0},
};

static struct reg_value ov5653_setting_30fps_VGA_640_480[] = {
{0x3008, 0x82, 0, 0},
{0x3008, 0x42, 0, 0},
{0x3103, 0x93, 0, 0},
{0x3b07, 0x0c, 0, 0},
{0x3017, 0xff, 0, 0},
{0x3018, 0xfc, 0, 0},
{0x3706, 0x41, 0, 0},
{0x3613, 0xc4, 0, 0},
{0x370d, 0x42, 0, 0},
{0x3703, 0x9a, 0, 0},
{0x3630, 0x22, 0, 0},
{0x3605, 0x04, 0, 0},
{0x3606, 0x3f, 0, 0},
{0x3712, 0x13, 0, 0},
{0x370e, 0x00, 0, 0},
{0x370b, 0x40, 0, 0},
{0x3600, 0x54, 0, 0},
{0x3601, 0x05, 0, 0},
{0x3713, 0x22, 0, 0},
{0x3714, 0x27, 0, 0},
{0x3631, 0x22, 0, 0},
{0x3612, 0x1a, 0, 0},
{0x3604, 0x40, 0, 0},
{0x3705, 0xdc, 0, 0},
{0x3709, 0x40, 0, 0},
{0x370a, 0x41, 0, 0},
{0x370c, 0xc8, 0, 0},
{0x3710, 0x28, 0, 0},
{0x3702, 0x3a, 0, 0},
{0x3704, 0x18, 0, 0},
//{0x3a18, 0x00, 0, 0},
//{0x3a19, 0xf8, 0, 0},
{0x3a00, 0x38, 0, 0},
{0x3800, 0x02, 0, 0},
{0x3801, 0x54, 0, 0},
{0x3803, 0x08, 0, 0},
{0x380c, 0x08, 0, 0},
{0x380d, 0x78, 0, 0},
{0x380e, 0x01, 0, 0},
{0x380f, 0xec, 0, 0},
{0x3830, 0x50, 0, 0},
{0x3a08, 0x12, 0, 0},
{0x3a09, 0x70, 0, 0},
{0x3a0a, 0x0f, 0, 0},
{0x3a0b, 0x60, 0, 0},
{0x3a0d, 0x01, 0, 0},
{0x3a0e, 0x01, 0, 0},
{0x3a13, 0x54, 0, 0},
{0x3815, 0x81, 0, 0},
{0x5059, 0x80, 0, 0},
{0x3615, 0x52, 0, 0},
{0x505a, 0x0a, 0, 0},
{0x505b, 0x2e, 0, 0},
{0x3703, 0x9a, 0, 0},
//{0x3010, 0x20, 0, 0},
{0x3010, 0x70, 0, 0},
//{0x3011, 0x10, 0, 0},
{0x3011, 0x10, 0, 0},
{0x3714, 0x17, 0, 0},

{0x3804, 0x05, 0, 0},
{0x3805, 0x00, 0, 0},
{0x3806, 0x01, 0, 0},
{0x3807, 0xe0, 0, 0},

{0x3808, 0x02, 0, 0},
{0x3809, 0x80, 0, 0},
{0x380a, 0x01, 0, 0},
{0x380b, 0xe0, 0, 0},
{0x380c, 0x08, 0, 0},
{0x380d, 0x78, 0, 0},
{INTERRUPT_FUNC_AEC_POLYGON, 0, 0, 0},
{0x3a08, 0x12, 0, 0},
{0x3a09, 0x70, 0, 0},
{0x3a0a, 0x0f, 0, 0},
{0x3a0b, 0x60, 0, 0},
{0x3a0d, 0x01, 0, 0},
{0x3a0e, 0x01, 0, 0},
{0x380e, 0x01, 0, 0},
{0x380f, 0xec, 0, 0},
{0x3815, 0x81, 0, 0},
{0x3824, 0x23, 0, 0},
{0x3825, 0x20, 0, 0},
{0x3803, 0x08, 0, 0},
{0x3826, 0x00, 0, 0},
{0x3827, 0x08, 0, 0},
{0x3a1a, 0x06, 0, 0},
{INTERRUPT_FUNC_AEC_CONTROL, 0, 0, 0},
{0x3623, 0x01, 0, 0},
{0x3633, 0x24, 0, 0},
{0x3c01, 0x34, 0, 0},
{0x3c04, 0x28, 0, 0},
{0x3c05, 0x98, 0, 0},
{0x3c07, 0x07, 0, 0},
{0x3c09, 0xc2, 0, 0},
{0x4000, 0x05, 0, 0},
{0x401d, 0x28, 0, 0},
{0x4001, 0x02, 0, 0},
{0x401c, 0x42, 0, 0},
{0x5046, 0x09, 0, 0}, // orig
{0x3810, 0x40, 0, 0},
{0x3836, 0x41, 0, 0},
{0x505f, 0x04, 0, 0},
{0x5000, 0xfe, 0, 0}, // orig
{0x5001, 0x01, 0, 0}, //orig
{0x5002, 0x02, 0, 0},
{0x503d, 0x00, 0, 0}, // TEST PATTERN - color bar
{0x5901, 0x04, 0, 0},
{0x585a, 0x01, 0, 0},
{0x585b, 0x2c, 0, 0},
{0x585c, 0x01, 0, 0},
{0x585d, 0x93, 0, 0},
{0x585e, 0x01, 0, 0},
{0x585f, 0x90, 0, 0},
{0x5860, 0x01, 0, 0},
{0x5861, 0x0d, 0, 0},
{0x5180, 0xc0, 0, 0},
{0x5184, 0x00, 0, 0},
{0x470a, 0x00, 0, 0},
{0x470b, 0x00, 0, 0},
{0x470c, 0x00, 0, 0},
{0x300f, 0x8e, 0, 0},
{0x3603, 0xa7, 0, 0},
{0x3632, 0x55, 0, 0},
{0x3620, 0x56, 0, 0},
{0x3621, 0xaf, 0, 0},
{0x3818, 0xc2, 0, 0},
{0x3631, 0x36, 0, 0},
{0x3632, 0x5f, 0, 0},
{0x3711, 0x24, 0, 0},
{0x401f, 0x03, 0, 0},
{0x3010, 0x70, 0, 0}, // orig
{INTERRUPT_FUNC_AWB_CONTROL, 0, 0, 0},
{0x3008, 0x02, 0, 0},
{0x3406, 0x00, 0, 0}, // Back to auto AWB. Have to set it here!
};

static struct reg_value ov5653_setting_15fps_VGA_640_480[] = {
{0x3008, 0x82, 0, 0},
{0x3008, 0x42, 0, 0},
{0x3103, 0x93, 0, 0},
{0x3b07, 0x0c, 0, 0},
{0x3017, 0xff, 0, 0},
{0x3018, 0xfc, 0, 0},
{0x3706, 0x41, 0, 0},
{0x3613, 0xc4, 0, 0},
{0x370d, 0x42, 0, 0},
{0x3703, 0x9a, 0, 0},
{0x3630, 0x22, 0, 0},
{0x3605, 0x04, 0, 0},
{0x3606, 0x3f, 0, 0},
{0x3712, 0x13, 0, 0},
{0x370e, 0x00, 0, 0},
{0x370b, 0x40, 0, 0},
{0x3600, 0x54, 0, 0},
{0x3601, 0x05, 0, 0},
{0x3713, 0x22, 0, 0},
{0x3714, 0x27, 0, 0},
{0x3631, 0x22, 0, 0},
{0x3612, 0x1a, 0, 0},
{0x3604, 0x40, 0, 0},
{0x3705, 0xdc, 0, 0},
{0x3709, 0x40, 0, 0},
{0x370a, 0x41, 0, 0},
{0x370c, 0xc8, 0, 0},
{0x3710, 0x28, 0, 0},
{0x3702, 0x3a, 0, 0},
{0x3704, 0x18, 0, 0},
//{0x3a18, 0x00, 0, 0},
//{0x3a19, 0xf8, 0, 0},
{0x3a00, 0x38, 0, 0},
{0x3800, 0x02, 0, 0},
{0x3801, 0x54, 0, 0},
{0x3803, 0x08, 0, 0},
{0x380c, 0x08, 0, 0},
{0x380d, 0x78, 0, 0},
{0x380e, 0x01, 0, 0},
{0x380f, 0xec, 0, 0},
{0x3830, 0x50, 0, 0},
{0x3a08, 0x12, 0, 0},
{0x3a09, 0x70, 0, 0},
{0x3a0a, 0x0f, 0, 0},
{0x3a0b, 0x60, 0, 0},
{0x3a0d, 0x01, 0, 0},
{0x3a0e, 0x01, 0, 0},
{0x3a13, 0x54, 0, 0},
{0x3815, 0x81, 0, 0},
{0x5059, 0x80, 0, 0},
{0x3615, 0x52, 0, 0},
{0x505a, 0x0a, 0, 0},
{0x505b, 0x2e, 0, 0},
{0x3703, 0x9a, 0, 0},
//{0x3010, 0x20, 0, 0},
{0x3010, 0x70, 0, 0},
//{0x3011, 0x10, 0, 0},
{0x3011, 0x10, 0, 0},
{0x3714, 0x17, 0, 0},

{0x3804, 0x05, 0, 0},
{0x3805, 0x00, 0, 0},
{0x3806, 0x01, 0, 0},
{0x3807, 0xe0, 0, 0},

{0x3808, 0x02, 0, 0},
{0x3809, 0x80, 0, 0},
{0x380a, 0x01, 0, 0},
{0x380b, 0xe0, 0, 0},
{0x380c, 0x08, 0, 0},
{0x380d, 0x78, 0, 0},
{INTERRUPT_FUNC_AEC_POLYGON, 0, 0, 0},
{0x3a08, 0x12, 0, 0},
{0x3a09, 0x70, 0, 0},
{0x3a0a, 0x0f, 0, 0},
{0x3a0b, 0x60, 0, 0},
{0x3a0d, 0x01, 0, 0},
{0x3a0e, 0x01, 0, 0},
{0x380e, 0x01, 0, 0},
{0x380f, 0xec, 0, 0},
{0x3815, 0x81, 0, 0},
{0x3824, 0x23, 0, 0},
{0x3825, 0x20, 0, 0},
{0x3803, 0x08, 0, 0},
{0x3826, 0x00, 0, 0},
{0x3827, 0x08, 0, 0},
{0x3a1a, 0x06, 0, 0},
{INTERRUPT_FUNC_AEC_CONTROL, 0, 0, 0},
{0x3623, 0x01, 0, 0},
{0x3633, 0x24, 0, 0},
{0x3c01, 0x34, 0, 0},
{0x3c04, 0x28, 0, 0},
{0x3c05, 0x98, 0, 0},
{0x3c07, 0x07, 0, 0},
{0x3c09, 0xc2, 0, 0},
{0x4000, 0x05, 0, 0},
{0x401d, 0x28, 0, 0},
{0x4001, 0x02, 0, 0},
{0x401c, 0x42, 0, 0},
{0x5046, 0x09, 0, 0}, // orig
{0x3810, 0x40, 0, 0},
{0x3836, 0x41, 0, 0},
{0x505f, 0x04, 0, 0},
{0x5000, 0xfe, 0, 0}, // orig
{0x5001, 0x01, 0, 0}, //orig
{0x5002, 0x02, 0, 0},
{0x503d, 0x00, 0, 0}, // TEST PATTERN - color bar
{0x5901, 0x04, 0, 0},
{0x585a, 0x01, 0, 0},
{0x585b, 0x2c, 0, 0},
{0x585c, 0x01, 0, 0},
{0x585d, 0x93, 0, 0},
{0x585e, 0x01, 0, 0},
{0x585f, 0x90, 0, 0},
{0x5860, 0x01, 0, 0},
{0x5861, 0x0d, 0, 0},
{0x5180, 0xc0, 0, 0},
{0x5184, 0x00, 0, 0},
{0x470a, 0x00, 0, 0},
{0x470b, 0x00, 0, 0},
{0x470c, 0x00, 0, 0},
{0x300f, 0x8e, 0, 0},
{0x3603, 0xa7, 0, 0},
{0x3632, 0x55, 0, 0},
{0x3620, 0x56, 0, 0},
{0x3621, 0xaf, 0, 0},
{0x3818, 0xc2, 0, 0},
{0x3631, 0x36, 0, 0},
{0x3632, 0x5f, 0, 0},
{0x3711, 0x24, 0, 0},
{0x401f, 0x03, 0, 0},
{0x3010, 0x70, 0, 0}, // orig
{INTERRUPT_FUNC_AWB_CONTROL, 0, 0, 0},
{0x3008, 0x02, 0, 0},
{0x3406, 0x00, 0, 0}, // Back to auto AWB. Have to set it here!
};


static struct reg_value ov5653_setting_30fps_XGA_1024_768[] = {
 {0x3010, 0x02, 0, 0}, // orig  
};

static struct reg_value ov5653_setting_15fps_XGA_1024_768[] = {
 {0x3010, 0x02, 0, 0}, // orig  
};

static struct reg_value ov5653_setting_30fps_QVGA_320_240[] = {
	{0x3010, 0x10, 0, 0}, {0x3012, 0x00, 0, 0},
};

static struct reg_value ov5653_setting_30fps_NTSC_720_480[] = {
	{0x302c, 0x60, 0x60, 0},
};

static struct reg_value ov5653_setting_30fps_PAL_720_576[] = {
	{0x302c, 0x60, 0x60, 0},
};

static struct reg_value ov5653_setting_15fps_720P_1280_720[] = {
	{0x3a0b, 0x50, 0, 0}, {0x3a0d, 0x08, 0, 0}, {0x3a0e, 0x07, 0, 0},
};

static struct reg_value ov5653_setting_30fps_720P_1280_720[] = {
	{0x3819, 0x80, 0, 0}, {0x5002, 0xe0, 0, 0},
};

static struct reg_value ov5653_setting_15fps_1080P_1920_1080[] = {
	{0x5002, 0xe0, 0, 0},
};

static struct reg_value ov5653_setting_15fps_QVGA_320_240[] = {
	{0x380b, 0xf0, 0, 0}, {0x3a00, 0x78, 0, 0},
};

static struct reg_value ov5653_setting_15fps_NTSC_720_480[] = {
	{0x5682, 0x05, 0, 0}, {0x5683, 0x00, 0, 0},
};

static struct reg_value ov5653_setting_15fps_PAL_720_576[] = {
	{0x5682, 0x04, 0, 0}, {0x5683, 0xb0, 0, 0},
};

static struct reg_value ov5653_setting_30fps_160_120[] = {
        {0x3010, 0x10, 0, 0}, {0x3012, 0x00, 0, 0},
};

static struct ov5653_mode_info ov5653_mode_info_data[2][ov5653_mode_MAX + 1] = {
	{
		{ov5653_mode_VGA_640_480,    640,  480,
		ov5653_setting_15fps_VGA_640_480,
		ARRAY_SIZE(ov5653_setting_15fps_VGA_640_480)},
		{ov5653_mode_QVGA_320_240,   320,  240,
		ov5653_setting_15fps_QVGA_320_240,
		ARRAY_SIZE(ov5653_setting_15fps_QVGA_320_240)},
		{ov5653_mode_NTSC_720_480,   720,  480,
		ov5653_setting_15fps_NTSC_720_480,
		ARRAY_SIZE(ov5653_setting_15fps_NTSC_720_480)},
		{ov5653_mode_PAL_720_576,   720,  576,
		ov5653_setting_15fps_PAL_720_576,
		ARRAY_SIZE(ov5653_setting_15fps_PAL_720_576)},
		{ov5653_mode_720P_1280_720,  1280, 720,
		ov5653_setting_15fps_720P_1280_720,
		ARRAY_SIZE(ov5653_setting_15fps_720P_1280_720)},
		{ov5653_mode_1080P_1920_1080, 1920, 1080,
		ov5653_setting_15fps_1080P_1920_1080,
		ARRAY_SIZE(ov5653_setting_15fps_1080P_1920_1080)},
		{ov5653_mode_QSXGA_2592_1944, 2592, 1944,
		ov5653_setting_15fps_QSXGA_2592_1944,
		ARRAY_SIZE(ov5653_setting_15fps_QSXGA_2592_1944)},
		{ov5653_mode_QCIF_176_144, 176, 144,
		ov5653_setting_15fps_QCIF_176_144,
		ARRAY_SIZE(ov5653_setting_15fps_QCIF_176_144)},
		{ov5653_mode_XGA_1024_768, 1024, 768,
		ov5653_setting_15fps_XGA_1024_768,
		ARRAY_SIZE(ov5653_setting_15fps_XGA_1024_768)},
		{ov5653_mode_800_600, 800, 600, ov5653_setting_30fps_800_600,
                ARRAY_SIZE(ov5653_setting_30fps_800_600)},
		{ov5653_mode_1600_1200, 1600, 1200,
		ov5653_setting_30fps_1600_1200,
		ARRAY_SIZE(ov5653_setting_30fps_1600_1200)},
		{ov5653_mode_160_120, 160, 120,
		ov5653_setting_30fps_160_120,
		ARRAY_SIZE(ov5653_setting_30fps_160_120)},
	},
	{
		{ov5653_mode_VGA_640_480,    640,  480,
		ov5653_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov5653_setting_30fps_VGA_640_480)},
		{ov5653_mode_QVGA_320_240,   320,  240,
		ov5653_setting_30fps_QVGA_320_240,
		ARRAY_SIZE(ov5653_setting_30fps_QVGA_320_240)},
		{ov5653_mode_NTSC_720_480,   720, 480,
		ov5653_setting_30fps_NTSC_720_480,
		ARRAY_SIZE(ov5653_setting_30fps_NTSC_720_480)},
		{ov5653_mode_PAL_720_576,    720, 576,
		ov5653_setting_30fps_PAL_720_576,
		ARRAY_SIZE(ov5653_setting_30fps_PAL_720_576)},
		{ov5653_mode_720P_1280_720,  1280, 720,
		ov5653_setting_30fps_720P_1280_720,
		ARRAY_SIZE(ov5653_setting_30fps_720P_1280_720)},
		{ov5653_mode_1080P_1920_1080, 0, 0, NULL, 0},
    {ov5653_mode_QSXGA_2592_1944, 2592, 1944,
    ov5653_setting_15fps_QSXGA_2592_1944,
    ARRAY_SIZE(ov5653_setting_15fps_QSXGA_2592_1944)},
		{ov5653_mode_QCIF_176_144, 176, 144,
		ov5653_setting_30fps_QCIF_176_144,
		ARRAY_SIZE(ov5653_setting_30fps_QCIF_176_144)},
		{ov5653_mode_XGA_1024_768, 1024, 768,
		ov5653_setting_30fps_XGA_1024_768,
		ARRAY_SIZE(ov5653_setting_30fps_XGA_1024_768)},
		{ov5653_mode_800_600, 800, 600,
		ov5653_setting_30fps_800_600,
		ARRAY_SIZE(ov5653_setting_30fps_800_600)},
		{ov5653_mode_1600_1200, 1600, 1200,
		ov5653_setting_30fps_1600_1200,
		ARRAY_SIZE(ov5653_setting_30fps_1600_1200)},
		{ov5653_mode_160_120, 160, 120,
		ov5653_setting_30fps_160_120,
		ARRAY_SIZE(ov5653_setting_30fps_160_120)},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct fsl_mxc_camera_platform_data *camera_plat;

static struct aec_polygon aec_poly = {{-1,-1},{-1,-1}};

static int ov5653_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov5653_remove(struct i2c_client *client);

static s32 ov5653_read_reg(u16 reg, u8 *val);
static s32 ov5653_write_reg(u16 reg, u8 val);

static int ov5653_set_gain(unsigned int gain);
static void ov5653_set_exposure(unsigned short exposure_in_lines);
static unsigned int ov5653_get_exposure(void);
static void ov5653_set_aec_poly(void);
static void ov5653_set_aec_control(unsigned int exposure);
static void ov5653_set_awb_control(unsigned short red_gain, 
                                  unsigned short green_gain, 
                                  unsigned short blue_gain);
static void ov5653_get_awb_gains( unsigned short *red_gain, 
                                  unsigned short *green_gain, 
                                  unsigned short *blue_gain);



static const struct i2c_device_id ov5653_id[] = {
	{"ov5653_1", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5653_id);

static struct i2c_driver ov5653_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov5653_1",
		  },
	.probe  = ov5653_probe,
	.remove = ov5653_remove,
	.id_table = ov5653_id,
};

static s32 ov5653_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};
	int ret = 0;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if ((ret = i2c_master_send(ov5653_data.i2c_client, au8Buf, 3)) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x, ret=%d\n",
			__func__, reg, val, ret);
		return -1;
	}

	return 0;
}

static s32 ov5653_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov5653_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov5653_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}


static enum ov5653_frame_rate get_frame_rate_id(int fps)
{
        if (fps == 15)
                return ov5653_15_fps;
        else if (fps == 30)
                return ov5653_30_fps;
        else {
                pr_warning(" No valid frame rate set!\n");
        }
        return ov5653_30_fps;
}

static int ov5653_init_sensor(void)
{
  struct reg_value *pModeSetting = NULL;
  s32 i = 0;
  s32 iModeSettingArySize = 0;
  register u32 Delay_ms = 0;
  register u16 RegAddr = 0;
  register u8 Mask = 0;
  register u8 Val = 0;
  u8 RegVal = 0;
  int retval = 0;

 
  pModeSetting = ov5653_initial_setting;
//  pModeSetting = ov5653_mode_start;
  iModeSettingArySize = ARRAY_SIZE(ov5653_initial_setting);
//  iModeSettingArySize = ARRAY_SIZE(ov5653_mode_start);
  prev_width = prev_height = 0;

  for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
    Delay_ms = pModeSetting->u32Delay_ms;
    RegAddr = pModeSetting->u16RegAddr;
    Val = pModeSetting->u8Val;
    Mask = pModeSetting->u8Mask;
    if (Mask) {
      retval = ov5653_read_reg(RegAddr, &RegVal);
      if (retval < 0)
        goto err;

      RegVal &= ~(u8)Mask;
      Val &= Mask;
      Val |= RegVal;
    }

    retval = ov5653_write_reg(RegAddr, Val);
    if (retval < 0)
      goto err;

    if (Delay_ms)
      msleep(Delay_ms);
  }
err:
  return retval;
}


static int ov5653_transition_mode(void)
{
  struct reg_value *pModeSetting = NULL;
  s32 i = 0;
  s32 iModeSettingArySize = 0;
  register u32 Delay_ms = 0;
  register u16 RegAddr = 0;
  register u8 Mask = 0;
  register u8 Val = 0;
  u8 RegVal = 0;
  int retval = 0;

 
  pModeSetting = ov5653_setting_QSXGA_2_VGA;
//  pModeSetting = ov5653_mode_start;
  iModeSettingArySize = ARRAY_SIZE(ov5653_setting_QSXGA_2_VGA);
//  iModeSettingArySize = ARRAY_SIZE(ov5653_mode_start);

  for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
    Delay_ms = pModeSetting->u32Delay_ms;
    RegAddr = pModeSetting->u16RegAddr;
    Val = pModeSetting->u8Val;
    Mask = pModeSetting->u8Mask;
    if (Mask) {
      retval = ov5653_read_reg(RegAddr, &RegVal);
      if (retval < 0)
        goto err;

      RegVal &= ~(u8)Mask;
      Val &= Mask;
      Val |= RegVal;
    }

    retval = ov5653_write_reg(RegAddr, Val);
    if (retval < 0)
      goto err;

    if (Delay_ms)
      msleep(Delay_ms);
  }
err:
  return retval;
}



static int ov5653_set_idle_mode(void) {
	register u16 RegAddr = 0;
	u8 ReadVal = 0;
	u8 WriteVal = 0;
	int retval = 0;
	int lc = 0;

  printk(KERN_INFO "<< OV5653 1: SET IDLE MODE>>\n");

	ReadVal = -1;
	RegAddr = REG_STA_FOCUS;
	retval = ov5653_read_reg(RegAddr, &ReadVal);
	if (retval < 0) {
		pr_err("%s, read reg 0x%x failed\n", __FUNCTION__, RegAddr);
	}

	for (lc = 0; (lc < 100) && (ReadVal != S_IDLE); ++lc) {
		WriteVal = CMD_IDLE_MODE;
		RegAddr = REG_CMD_MAIN;
		retval = ov5653_write_reg(RegAddr, WriteVal);
		if (retval < 0) {
			pr_err("%s, write reg 0x%x failed\n", __FUNCTION__, RegAddr);
		}

		mdelay(1);

		ReadVal = -1;
		RegAddr = REG_STA_FOCUS;
		retval = ov5653_read_reg(RegAddr, &ReadVal);
		if (retval < 0) {
			pr_err("%s, read reg 0x%x failed\n", __FUNCTION__, RegAddr);
		}
	}

	if (ReadVal != S_IDLE)
		retval = -1;
	else
		retval = 0;
	return retval;
}

static int ov5653_config_auto_focus(void){

  printk(KERN_INFO "<< OV5653 0: CONFIG AUTO FOCUS>>\n");

	ov5653_write_reg(REG_CMD_TAG, 0x01);
	ov5653_write_reg(REG_CMD_MAIN, 0x10);
	return 0;
}

static int ov5653_set_rotate_mode(struct reg_value *rotate_mode)
{
	s32 i = 0;
	s32 iModeSettingArySize = 2;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;

  printk(KERN_INFO "<< OV5653 0: SET ROTATE MODE>>\n");

	for (i = 0; i < iModeSettingArySize; ++i, ++rotate_mode) {
		Delay_ms = rotate_mode->u32Delay_ms;
		RegAddr = rotate_mode->u16RegAddr;
		Val = rotate_mode->u8Val;
		Mask = rotate_mode->u8Mask;

		if (Mask) {
			retval = ov5653_read_reg(RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("%s, read reg 0x%x failed\n", __FUNCTION__, RegAddr);
				goto err;
			}

			Val |= RegVal;
			Val &= Mask;
		}

		retval = ov5653_write_reg(RegAddr, Val);
		if (retval < 0) {
			pr_err("%s, write reg 0x%x failed\n", __FUNCTION__, RegAddr);
			goto err;
		}

		if (Delay_ms)
			mdelay(Delay_ms);
	}
err:
	return retval;
}

static int ov5653_auto_focus_start(void) {
	register u16 RegAddr = 0;
	u8 RegVal = 0;
	int retval = 0;

  printk(KERN_INFO "<< OV5653 0: AUTO FOCUS START>>\n");
	retval = ov5653_set_idle_mode();
	ov5653_config_auto_focus();

	if (retval > -1) {
		RegVal = CMD_SINGLE_FOCUS_MODE;
		RegAddr = REG_CMD_MAIN;
		retval = ov5653_write_reg(RegAddr, RegVal);
		if (retval < 0) {
			pr_err("%s, write reg 0x%x failed\n", __FUNCTION__, RegAddr);
		}
	} else {
		pr_err("Could not get camera into idle mode. Abandoning focus attempt");
	}

	return retval;
}

static int ov5653_init_mode(enum ov5653_frame_rate frame_rate,
		enum ov5653_mode mode, enum ov5653_mode orig_mode);
static int ov5653_write_snapshot_para(enum ov5653_frame_rate frame_rate,
		enum ov5653_mode mode, enum ov5653_mode orig_mode, struct v4l2_framepos *pfpos);
static int ov5653_change_mode(enum ov5653_frame_rate new_frame_rate,
		enum ov5653_frame_rate old_frame_rate,
		enum ov5653_mode new_mode,
		enum ov5653_mode orig_mode, struct v4l2_framepos *pfpos)
{
	int retval = 0;


	if (new_mode > ov5653_mode_MAX || new_mode < ov5653_mode_MIN) {
		pr_err("Wrong ov5653 mode detected!\n");
		return -1;
	}

  ov5653_data.frame_rate = new_frame_rate;
#if 0
	if ((new_frame_rate == old_frame_rate) &&
	    (new_mode == ov5653_mode_VGA_640_480) &&
		(orig_mode == ov5653_mode_QSXGA_2592_1944)) {
		pModeSetting = ov5653_setting_QSXGA_2_VGA;
		iModeSettingArySize = ARRAY_SIZE(ov5653_setting_QSXGA_2_VGA);
		ov5653_data.pix.width = 640;
		ov5653_data.pix.height = 480;
	} else if ((new_frame_rate == old_frame_rate) &&
	    (new_mode == ov5653_mode_QVGA_320_240) &&
		(orig_mode == ov5653_mode_VGA_640_480)) {
		pModeSetting = ov5653_setting_VGA_2_QVGA;
		iModeSettingArySize = ARRAY_SIZE(ov5653_setting_VGA_2_QVGA);
		ov5653_data.pix.width = 320;
		ov5653_data.pix.height = 240;
	} else {
#endif
    printk(KERN_INFO "<< WRITE SNAPSHOT PARAMS >>\n");
//	ov5653_init_sensor();
		retval = ov5653_write_snapshot_para(new_frame_rate, new_mode, orig_mode, pfpos);
//	}

#if 0
  printk(KERN_INFO "<<<<<< CHANGE MODE >>>>>>\n"); 
	if (ov5653_data.pix.width == 0 || ov5653_data.pix.height == 0 ||
			pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov5653_read_reg(RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("read reg error addr=0x%x", RegAddr);
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov5653_write_reg(RegAddr, Val);
		if (retval < 0) {
			pr_err("write reg error addr=0x%x", RegAddr);
			goto err;
		}

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
#endif
	return retval;
}


static void dump_register_range(unsigned int rs, unsigned int re)
{
   unsigned int r;
   u8 RegVal = 0;
  
   for(r = rs; r <= re; r++) {
      ov5653_read_reg(r, &RegVal);
      printk(KERN_INFO "Register: 0x%x, Value: 0x%x\n", r, RegVal);
   }  
}

static void dump_registers(void)
{
  dump_register_range(0x3000, 0x3037);
  dump_register_range(0x3040, 0x3043);
  dump_register_range(0x3400, 0x3406);
  dump_register_range(0x3500, 0x3503);
  dump_register_range(0x3508, 0x350d);
  dump_register_range(0x3600, 0x3634);
  dump_register_range(0x3700, 0x3715);
  dump_register_range(0x3800, 0x3851);
  dump_register_range(0x3a00, 0x3a20);
  dump_register_range(0x3b00, 0x3b09);
  dump_register_range(0x3c00, 0x3c0c);
  dump_register_range(0x3d00, 0x3d00);
  dump_register_range(0x3d04, 0x3d04);
  dump_register_range(0x3e00, 0x3e14);
  dump_register_range(0x4000, 0x401e);
  dump_register_range(0x4100, 0x4100);
  dump_register_range(0x4200, 0x4203);
  dump_register_range(0x4700, 0x470c);
  dump_register_range(0x4800, 0x4805);
  dump_register_range(0x4810, 0x4815);
  dump_register_range(0x4818, 0x483f);
  dump_register_range(0x4846, 0x4854);
  dump_register_range(0x4860, 0x4865);
  dump_register_range(0x5000, 0x5002);
  dump_register_range(0x5005, 0x501b);
  dump_register_range(0x5025, 0x5025);
  dump_register_range(0x503d, 0x503e);
  dump_register_range(0x5046, 0x5046);
  dump_register_range(0x5180, 0x51be);
  dump_register_range(0x5680, 0x5693);
  dump_register_range(0x5780, 0x578f);
  dump_register_range(0x505a, 0x505d);
  dump_register_range(0x5800, 0x5876);
  dump_register_range(0x5900, 0x5901);

}


static int ov5653_init_mode(enum ov5653_frame_rate frame_rate,
			    enum ov5653_mode mode, enum ov5653_mode orig_mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 i = 0;
	s32 iModeSettingArySize = 0;
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int retval = 0;
  unsigned short red_gain, green_gain, blue_gain;

  printk(KERN_INFO "<< OV5653 1: INIT MODE>>\n");

if (mode > ov5653_mode_MAX || mode < ov5653_mode_MIN) {
		pr_err("Wrong ov5653 mode detected!\n");
		return -1;
	}

	pModeSetting = ov5653_mode_info_data[frame_rate][mode].init_data_ptr;
	iModeSettingArySize =
		ov5653_mode_info_data[frame_rate][mode].init_data_size;

	ov5653_data.pix.width = ov5653_mode_info_data[frame_rate][mode].width;
	ov5653_data.pix.height = ov5653_mode_info_data[frame_rate][mode].height;


	if (ov5653_data.pix.width == 0 || ov5653_data.pix.height == 0 ||
	    pModeSetting == NULL || iModeSettingArySize == 0)
		return -EINVAL;

  printk(KERN_INFO "<< OV5653 1: INIT MODE FOR Width: %d, Height: %d>>\n", ov5653_data.pix.width, ov5653_data.pix.height);
  if((ov5653_data.pix.width == prev_width) && (ov5653_data.pix.height == prev_height))
      return retval;

  prev_width = ov5653_data.pix.width;
  prev_height = ov5653_data.pix.height;

  // Get previous exposure for original mode and color gains values
  prev_exposure_table[orig_mode] = ov5653_get_exposure();
  ov5653_get_awb_gains(&red_gain, &green_gain, &blue_gain);
  printk(KERN_INFO "OV5653 0: Exposure: %d, AWB Gain values: Red: %X, Green: %X, Blue: %X\n", 
          prev_exposure_table[orig_mode], red_gain, green_gain, blue_gain);

  // Change mode
	for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

    if(RegAddr == INTERRUPT_FUNC_AEC_POLYGON) {
      ov5653_set_aec_poly();
    }else if(RegAddr == INTERRUPT_FUNC_AEC_CONTROL) {
      ov5653_set_aec_control(prev_exposure_table[mode]);
    }else if(RegAddr == INTERRUPT_FUNC_AWB_CONTROL) {
      ov5653_set_awb_control(red_gain, green_gain, blue_gain);
    }else {

  		if (Mask) {
  			retval = ov5653_read_reg(RegAddr, &RegVal);
  			if (retval < 0) {
  				pr_err("read reg error addr=0x%x", RegAddr);
  				goto err;
  			}

  			RegVal &= ~(u8)Mask;
  			Val &= Mask;
  			Val |= RegVal;
  		}

  		retval = ov5653_write_reg(RegAddr, Val);
  		if (retval < 0) {
  			pr_err("write reg error addr=0x%x", RegAddr);
  			goto err;
  		}
    }
		if (Delay_ms)
			mdelay(Delay_ms);
//			msleep(Delay_ms);
	}
 // dump_registers();
  // Set exposure if manual was requested. We have to do it now because INIT mode
  // reqets sensor chip and we end up with defaqult value of the 0x3503 AEC/AGC control
  // register which is 0 (auto ON). If we requested prior to this frame manual exposure -
  // make sure it is set.
  if(!req_auto_exp) {
      ov5653_set_exposure(req_exposure); // In this case exposure is used to set exposure+gain
  }
   mdelay(100);
   printk(KERN_INFO "<< OV5653 1: MODE INIT DONE >>\n");

err:
	return retval;
}


/* -=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==
* Set start for ov5653 camera
*/
static inline void ov5653_set_frame_start (int x, int y, int offs_x, int offs_y) {
        int X = x + offs_x;        /* offset to valid pixel region */
        int Y = y + offs_y;
        u8 RegVal = 0;

 
        ov5653_read_reg(0x3800, &RegVal);
        printk(KERN_INFO "ov5653: 0x3800: %X\n", RegVal);
        ov5653_read_reg(0x3801, &RegVal);
        printk(KERN_INFO "ov5653: 0x3801: %X\n", RegVal);
        ov5653_read_reg(0x3802, &RegVal);
        printk(KERN_INFO "ov5653: 0x3802: %X\n", RegVal);
        ov5653_read_reg(0x3803, &RegVal);
        printk(KERN_INFO "ov5653: 0x3803: %X\n", RegVal);


        ov5653_read_reg(0x3824, &RegVal);
        printk(KERN_INFO "ov5653: 0x3824: %X\n", RegVal);
        ov5653_read_reg(0x3825, &RegVal);
        printk(KERN_INFO "ov5653: 0x3825: %X\n", RegVal);
        ov5653_read_reg(0x3826, &RegVal);
        printk(KERN_INFO "ov5653: 0x3826: %X\n", RegVal);
        ov5653_read_reg(0x3827, &RegVal);
        printk(KERN_INFO "ov5653: 0x3827: %X\n", RegVal);


        printk(KERN_INFO "Setting Frame start: X: %X, Y: %X\n", X, Y); 
        ov5653_write_reg(0x3800, (unsigned char)((X>>8)&0x0f));/* sensor x start point(h)*/
        ov5653_write_reg(0x3801, (unsigned char)(X&0xff));         /* sensor x start point(l)*/
        ov5653_write_reg(0x3802, (unsigned char)((Y>>8)&0x0f));    /* sensor y start point(h)*/
        ov5653_write_reg(0x3803, (unsigned char)(Y&0xff));         /* sensor y start point(l) */
#if 0
        ov5653_write_reg(0x3824, (unsigned char)((X>>8)&0x0f));/* sensor x start point(h)*/
        ov5653_write_reg(0x3825, (unsigned char)(X&0xff));         /* sensor x start point(l)*/
        ov5653_write_reg(0x3826, (unsigned char)((Y>>8)&0x0f));    /* sensor y start point(h)*/
        ov5653_write_reg(0x3827, (unsigned char)(Y&0xff));         /* sensor y start point(l) */
#endif
}

/* -=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==
* Set input size for ov5653
*/
static inline void ov5653_set_frame_input_size (int x, int y)
{
        u8 RegVal = 0;
        ov5653_read_reg(0x3804, &RegVal);
        printk(KERN_INFO "ov5653: 0x3804: %X\n", RegVal);
        ov5653_read_reg(0x3805, &RegVal);
        printk(KERN_INFO "ov5653: 0x3805: %X\n", RegVal);
        ov5653_read_reg(0x3806, &RegVal);
        printk(KERN_INFO "ov5653: 0x3806: %X\n", RegVal);
        ov5653_read_reg(0x3807, &RegVal);
        printk(KERN_INFO "ov5653: 0x3807: %X\n", RegVal);
        /* set Sensor output size */
        printk(KERN_INFO "Setting Frame Size: SW: %X, SH: %X\n", x, y); 
        ov5653_write_reg(0x3804, (unsigned char)((x>>8)&0x0f));    /* sensor x output size(h)*/
        ov5653_write_reg(0x3805, (unsigned char)(x&0xff));         /* sensor x output size(l)*/
        ov5653_write_reg(0x3806, (unsigned char)((y>>8)&0x0f));    /* sensor y output size(h)*/
        ov5653_write_reg(0x3807, (unsigned char)(y&0xff));         /* sensor y output size(l)*/

}


static int ov5653_write_snapshot_para(enum ov5653_frame_rate frame_rate,
       enum ov5653_mode mode, enum ov5653_mode orig_mode, struct v4l2_framepos *pfpos)
{
	int ret = 0;
#if 0  
	bool m_60Hz = false;
	u16 capture_frame_rate = 50;
	u16 g_preview_frame_rate = 225;
        int sx = pfpos->xstart;
        int sy = pfpos->ystart;
        int sw = pfpos->width;
        int sh = pfpos->height;
        int off_x, off_y;

	u8 exposure_low, exposure_mid, exposure_high;
	u8 ret_l, ret_m, ret_h, gain, lines_10ms;
	u16 ulcapture_exposure, icapture_gain, preview_maxlines;
	u32 ulcapture_exposure_gain, capture_maxlines, g_preview_exposure;

	ov5653_write_reg(0x3503, 0x07);

	ret_h = ret_m = ret_l = 0;
	g_preview_exposure = 0;
	ov5653_read_reg(0x3500, &ret_h);
	ov5653_read_reg(0x3501, &ret_m);
	ov5653_read_reg(0x3502, &ret_l);
	g_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);

	ret_h = ret_m = ret_l = 0;
	preview_maxlines = 0;
	ov5653_read_reg(0x380e, &ret_h);
	ov5653_read_reg(0x380f, &ret_l);
	preview_maxlines = (ret_h << 8) + ret_l;
	/*Read back AGC Gain for preview*/
	gain = 0;
	ov5653_read_reg(0x350b, &gain);
#endif



	ret = ov5653_init_mode(frame_rate, mode, orig_mode);
	if (ret < 0)
		return ret;

#if 0
	ret_h = ret_m = ret_l = 0;
	ov5653_read_reg(0x380e, &ret_h);
	ov5653_read_reg(0x380f, &ret_l);
	capture_maxlines = (ret_h << 8) + ret_l;
	if (m_60Hz == true)
		lines_10ms = capture_frame_rate * capture_maxlines/12000;
	else
		lines_10ms = capture_frame_rate * capture_maxlines/10000;

	if (preview_maxlines == 0)
		preview_maxlines = 1;

	ulcapture_exposure = (g_preview_exposure*(capture_frame_rate)*(capture_maxlines))/
		(((preview_maxlines)*(g_preview_frame_rate)));
	icapture_gain = (gain & 0x0f) + 16;
	if (gain & 0x10)
		icapture_gain = icapture_gain << 1;

	if (gain & 0x20)
		icapture_gain = icapture_gain << 1;

	if (gain & 0x40)
		icapture_gain = icapture_gain << 1;

	if (gain & 0x80)
		icapture_gain = icapture_gain << 1;

	ulcapture_exposure_gain = 2 * ulcapture_exposure * icapture_gain;

	if (ulcapture_exposure_gain < capture_maxlines*16) {
		ulcapture_exposure = ulcapture_exposure_gain/16;
		if (ulcapture_exposure > lines_10ms) {
			ulcapture_exposure /= lines_10ms;
			ulcapture_exposure *= lines_10ms;
		}
	} else
		ulcapture_exposure = capture_maxlines;

	if (ulcapture_exposure == 0)
		ulcapture_exposure = 1;

	icapture_gain = (ulcapture_exposure_gain*2/ulcapture_exposure + 1)/2;
	exposure_low = ((unsigned char)ulcapture_exposure)<<4;
	exposure_mid = (unsigned char)(ulcapture_exposure >> 4) & 0xff;
	exposure_high = (unsigned char)(ulcapture_exposure >> 12);

	gain = 0;
	if (icapture_gain > 31) {
		gain |= 0x10;
		icapture_gain = icapture_gain >> 1;
	}
	if (icapture_gain > 31) {
		gain |= 0x20;
		icapture_gain = icapture_gain >> 1;
	}
	if (icapture_gain > 31) {
		gain |= 0x40;
		icapture_gain = icapture_gain >> 1;
	}
	if (icapture_gain > 31) {
		gain |= 0x80;
		icapture_gain = icapture_gain >> 1;
	}
	if (icapture_gain > 16)
		gain |= ((icapture_gain - 16) & 0x0f);

	if (gain == 0x10)
		gain = 0x11;

	ov5653_write_reg(0x350b, gain);
	ov5653_write_reg(0x3502, exposure_low);
	ov5653_write_reg(0x3501, exposure_mid);
	ov5653_write_reg(0x3500, exposure_high);
//	msleep(500);

  /* Finaly, set internal position of a frame */
  if(sx < 0) sx = 0;
  if(sy < 0) sy = 0;
  if(sw > 2592) sw = 2592;
  if(sh > 1944) sw = 1944;
  if(mode == ov5653_mode_VGA_640_480) {
      sw = 0x500;
      sh = 0x3c0;
      off_x = 0x50;
      off_y = 0x08;
	}
  else {
      off_x = 0x95;
      off_y = 0x0E;
  }
//	ov5653_set_frame_start(sx, sy, off_x, off_y);
//	ov5653_set_frame_input_size(sw, sh);
#endif
	return ret;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
* Get AWB color gains
*/
static void ov5653_get_awb_gains( unsigned short *red_gain, 
                                  unsigned short *green_gain, 
                                  unsigned short *blue_gain)
{
  unsigned char regval;

  *red_gain = 0;
  ov5653_read_reg(0x3400, &regval); /* msb */
  *red_gain = regval;
  ov5653_read_reg(0x3401, &regval); /* lsb */
  *red_gain = ((*red_gain << 8) | regval);
  *green_gain = 0;
  ov5653_read_reg(0x3402, &regval); /* msb */
  *green_gain = regval;
  ov5653_read_reg(0x3403, &regval); /* lsb */
  *green_gain = ((*green_gain << 8) | regval);
  *blue_gain = 0;
  ov5653_read_reg(0x3404, &regval); /* msb */
  *blue_gain = regval;
  ov5653_read_reg(0x3405, &regval); /* lsb */
  *blue_gain = ((*blue_gain << 8) | regval);
}



/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= 
* This sets manual exposure in lines for ov5653 5MP sensor
*/
static void ov5653_set_exposure(unsigned short exposure_in_lines)
{
        unsigned int exposure = exposure_in_lines;
        unsigned int explines;
        unsigned int gain;

    // Save exposure
    if(exposure > MAX_EXPOSURE_VALUE) 
        exposure = MAX_EXPOSURE_VALUE;
    req_exposure = exposure_in_lines;
    // Just in case enable manual exposure and gain
    ov5653_write_reg(0x3503, 0x03);
    // Calculate gain and exposure lines.
    // Calculate gain = exposure*MAX_GAIN_VAL/MAX_EXPOSURE_IN_LINES
//    gain = ((exposure << 5) >> 14)+1;
    gain = (exposure >> 9)+1;
    if(gain > MAX_GAIN_VALUE)
        gain = MAX_GAIN_VALUE;
    // Calculate exposure for this chip = exposure*MAX_EXPOSURE_VAL/MAX_EXPOSURE_IN_LINES
//    explines = (((exposure << 11) + (exposure << 8) ) >> 14); // Values are approximate
    // Simplify
    explines = (exposure >> 3); // Values are approximate from 0 - 2500

    printk(KERN_INFO "OV5653_1: Set Exposure: %d gain: %d\n",explines, gain);
    ov5653_set_gain(gain);
    explines = (explines << 4); // Exposure to set is in 1/16th of a line
    ov5653_write_reg(0x3502, (explines & 0x00FF));
    ov5653_write_reg(0x3501, ((explines >> 8) & 0x00FF));
    ov5653_write_reg(0x3500, ((explines >> 16) & 0x000F));

    //dump_registers();
}



static unsigned int ov5653_get_exposure(void)
{
        unsigned int exposure = 0;
        unsigned char regval;

        ov5653_read_reg(0x3500, &regval);       /* exposure[19:16] */
        exposure = (regval & 0x0F);
        ov5653_read_reg(0x3501, &regval);       /* exposure[15:8] */
        exposure = ((exposure << 8) | regval);
        ov5653_read_reg(0x3502, &regval);       /* exposure[7:0] */
        exposure = ((exposure << 8) | regval);
//        printk(KERN_INFO "OV5653_1: [Got exposure: %d]\n", exposure);
        return exposure;
}

static int ov5653_set_gain(unsigned int gain)
{
        unsigned char gain_bits;

  printk(KERN_INFO "<< OV5653 1: IOCTL SET GAIN: %d>>\n", gain);
        if(gain < MIN_GAIN_VALUE || gain > MAX_GAIN_VALUE) {
            printk(KERN_ERR "OV5653: gain value is out of limits\n");
            return -EINVAL;
        }
        gain_bits = gain_bits_table[gain];
        ov5653_write_reg(0x350a, ((gain_bits >> 7) & 0x01));
        ov5653_write_reg(0x350b, (gain_bits & 0x7F));
        req_gain = gain;
        return 0;
}

static unsigned int ov5653_get_gain(void)
{
        unsigned char regval1;
        unsigned char regval2;
        unsigned int gain;

        // read the high bit:
        ov5653_read_reg(0x350a, &regval1);
        // read the low bits:
        ov5653_read_reg(0x350b, &regval2);
        // put them together in this weird way
        gain = ((regval1 & 0x01)+1)*(((regval2 >> 6) & 0x01)+1)*(((regval2 >> 5) & 0x01)+1)*
                (((regval2 >> 4) & 0x01)+1)*(((regval2 & 0x0F) >> 4) +1);

        return gain;
}
 
 
/* 0 - turn auto-exposure OFF, 1 - turn AE ON*/
static void ov5653_set_autoexposure(int on)
{
    unsigned char val;

    printk(KERN_INFO "<< OV5653 0: IOCTL SET AUTOEXPOSURE, on: %d>>\n", on);

    if(on){
        /* Auto-exposure */
        ov5653_read_reg(0x3503, &val);
        val &= 0xFE;
        ov5653_write_reg(0x3503, val);
        req_auto_exp = 1;
//        ov5653_write_reg(0x3a00, 0x78);
    }else{
        /* Manual exposure */
     ov5653_read_reg(0x3503, &val);
     val |= 0x01;
     ov5653_write_reg(0x3503, val);
     req_auto_exp = 0;
//       ov5653_write_reg(0x3a00, 0x08); 
    }

    ov5653_read_reg(0x3503, &val);
    printk(KERN_INFO "<< OV5653 0: AUTOEXPOSURE value: %X>>\n", val);
}

/* Returns: 0 - auto-exposure is OFF, 1 - auto-exposure is ON.*/
static int ov5653_get_autoexposure(void)
{
        char result;
        ov5653_read_reg(0x3503, &result);
        return (result & 0x01)?0:1;
}


/* 0 - turn auto-gain OFF, 1 - turn AG ON*/
static void ov5653_set_autogain(int on)
{
        unsigned char val;
  printk(KERN_INFO "<< OV5653 0: IOCTL SET AUTOGAIN, on: %d>>\n", on);


    if(on){
                  /* Auto-exposure */
     ov5653_read_reg(0x3503, &val);
     val &= 0xFD;
     ov5653_write_reg(0x3503, val);
    }else{
        /* Manual exposure */
               ov5653_read_reg(0x3503, &val);
               val |= 0x02;
        ov5653_write_reg(0x3503, val);
    }
    printk(KERN_INFO "<< OV5653 0: AUTOGAIN value: %X>>\n", val);
}

/* Returns: 0 - auto-gain is OFF, 1 - auto-gain is ON.*/
static int ov5653_get_autogain(void)
{
        char result;
        ov5653_read_reg(0x3503, &result);
        return (result & 0x02)?0:1;
}

#if 0
{0x3406, 0x01, 0, 0}, // Manual AWB and set values for color gains
{0x3400, 0x05, 0, 0},
{0x3401, 0x72, 0, 0},
{0x3402, 0x04, 0, 0},
{0x3403, 0x00, 0, 0},
{0x3404, 0x05, 0, 0},
{0x3405, 0x72, 0, 0}, 
#endif
/* 
*  Set AWB values. Manual mode + prev values.
*/
static void ov5653_set_awb_control( unsigned short red_gain,
                                    unsigned short green_gain,
                                    unsigned short blue_gain)
{
  // Set AWB to manual
  ov5653_write_reg(0x3406, 0x01);
  // Set gain value = previous before changing mode
  ov5653_write_reg(0x3400, ((red_gain >> 8) & 0xFF));
  ov5653_write_reg(0x3401, (red_gain & 0xFF));
  ov5653_write_reg(0x3402, ((green_gain >> 8) & 0xFF));
  ov5653_write_reg(0x3403, (green_gain & 0xFF));
  ov5653_write_reg(0x3404, ((blue_gain >> 8) & 0xFF));
  ov5653_write_reg(0x3405, (blue_gain & 0xFF));
}


/* 
*  Set AEC values. First, manual mode + prev value.
*  Second - auto AEC
*/
static void ov5653_set_aec_control(unsigned int exposure)
{
  unsigned char exp_h, exp_m, exp_l;
  unsigned char val;
  ov5653_read_reg(0x3503, &val);
  printk(KERN_INFO "OV5653: READING EXPOSURE VAL: %X\n", val);
  // Set only if we are in auto-exposure mode
  if(!(val & 0x01)) {
      printk(KERN_INFO "OV5653: Reset exposure to original: %d\n", exposure);
      // Break exposure to register settings
      exp_h = ((exposure >> 16) & 0x0F);
      exp_m = ((exposure >> 8) & 0xFF);
      exp_l = (exposure & 0xFF);

  // Set AEC to manual
      ov5653_write_reg(0x3503, 0x01);
  // Set exposure value = previous before changing mode
//  ov5653_set_exposure(exposure);
      ov5653_write_reg(0x3500, exp_h);
      ov5653_write_reg(0x3501, exp_m);
      ov5653_write_reg(0x3502, exp_l);
      // Set AEC to auto if it was set before
      ov5653_write_reg(0x3503, 0x00);
   } else {
      printk(KERN_INFO "OV5653: manual exposure detected.\n");
  }
}

/* Set AEC polygon Upper left and lower right corners */
static void ov5653_set_aec_poly(void)
{
    // Completed combined polygon, set it
    unsigned int ulx, uly, lrx, lry;
    if(aec_poly.UL.X == -1 && aec_poly.UL.Y == -1)
      return;
    ulx = aec_poly.UL.X;
    uly = aec_poly.UL.Y;
    lrx = aec_poly.LR.X;
    lry = aec_poly.LR.Y;
    // Calculate based on width and height
    if((ov5653_data.pix.width > 1024) && (ov5653_data.pix.height > 768)) {
        /* Scale by multiplying by 2 to accomodate for internal size 2592x1944 */
        ulx = ulx * 2;
        uly = uly * 2;
        lrx = lrx * 2;
        lry = lry * 2;
    }
    // X start
    ov5653_write_reg(0x5681, (ulx & 0xFF));
    ov5653_write_reg(0x5680, (ulx >> 8) & 0x0F);
    // Y start
    ov5653_write_reg(0x5685, (uly & 0xFF));
    ov5653_write_reg(0x5684, (uly >> 8) & 0x07);
    // X end
    ov5653_write_reg(0x5683, (lrx & 0xFF));
    ov5653_write_reg(0x5682, (lrx >> 8) & 0x0F);
    // Y end
    ov5653_write_reg(0x5687, (lry & 0xFF));
    ov5653_write_reg(0x5686, (lry >> 8) & 0x07);
}

void ov5653_set_aec_polygon_ul(__u32 pt)
{
    aec_poly.UL.X = ((pt >> 16) & 0xFFFF);
    aec_poly.UL.Y = (pt & 0xFFFF);
}

void ov5653_set_aec_polygon_lr(__u32 pt)
{
    aec_poly.LR.X = ((pt >> 16) & 0xFFFF);
    aec_poly.LR.Y = (pt & 0xFFFF);
   // Completed combined polygon, set it by reinitilizing sensor mode table
    printk(KERN_INFO "OV5653 1: Setting mode: %d\n", ov5653_data.streamcap.capturemode);
    ov5653_init_mode(ov5653_data.frame_rate, ov5653_data.streamcap.capturemode, ov5653_data.streamcap.capturemode);
//    ov5653_set_aec_poly();
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov5653_data.mclk;
	pr_debug("   clock_curr=mclk=%d\n", ov5653_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV5653_XCLK_MIN;
	p->u.bt656.clock_max = OV5653_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
		/* Make sure power on */
		if (camera_plat->pwdn)
			camera_plat->pwdn(0);

	} else if (!on && sensor->on) {
		if (analog_regulator)
			regulator_disable(analog_regulator);
		if (core_regulator)
			regulator_disable(core_regulator);
		if (io_regulator)
			regulator_disable(io_regulator);
		if (gpo_regulator)
			regulator_disable(gpo_regulator);

		if (camera_plat->pwdn)
			camera_plat->pwdn(1);
	}

	sensor->on = on;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps, old_fps;	/* target frames per secound */
	enum ov5653_frame_rate new_frame_rate, old_frame_rate;
	int ret = 0;


	/* Make sure power on */
	if (camera_plat->pwdn)
		camera_plat->pwdn(0);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 15)
			new_frame_rate = ov5653_15_fps;
		else if (tgt_fps == 30)
			new_frame_rate = ov5653_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		if (sensor->streamcap.timeperframe.numerator != 0)
			old_fps = sensor->streamcap.timeperframe.denominator /
				sensor->streamcap.timeperframe.numerator;
		else
			old_fps = 30;

		if (old_fps == 15)
			old_frame_rate = ov5653_15_fps;
		else if (old_fps == 30)
			old_frame_rate = ov5653_30_fps;
		else {
			pr_warning(" No valid frame rate set!\n");
			old_frame_rate = ov5653_30_fps;
		}


		ret = ov5653_change_mode(new_frame_rate, old_frame_rate,
				a->parm.capture.capturemode,
				sensor->streamcap.capturemode, &a->fpos);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
				(u32)a->parm.capture.capturemode;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov5653_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov5653_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov5653_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov5653_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov5653_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov5653_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov5653_get_exposure();
		break;
	case V4L2_CID_AUTOEXPOSURE:
		vc->value = ov5653_get_autoexposure();
		break;
	case V4L2_CID_GAIN:
		vc->value = ov5653_get_gain();
		break;
	case V4L2_CID_AUTOGAIN:
		vc->value = ov5653_get_autogain();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = 0;
	struct sensor_data *sensor = s->priv;
	__u32 captureMode = sensor->streamcap.capturemode;

	pr_debug("In ov5653:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_AUTO_FOCUS_START:
		retval = ov5653_auto_focus_start();
		break;
	case V4L2_CID_AUTO_FOCUS_STOP:
		retval = ov5653_set_idle_mode();
		break;
	case V4L2_CID_CONTRAST:
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_AUTOEXPOSURE:
		ov5653_set_autoexposure(vc->value);
		break;
	case V4L2_CID_EXPOSURE:
		ov5653_set_exposure(vc->value);	
		break;
	case V4L2_CID_AUTOGAIN:
		ov5653_set_autogain(vc->value);
		break;
	case V4L2_CID_GAIN:
		retval = ov5653_set_gain(vc->value);
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
  case V4L2_CID_AEC_POLYGON_UL:
    ov5653_set_aec_polygon_ul((__u32)vc->value);
    break;
  case V4L2_CID_AEC_POLYGON_LR:
    ov5653_set_aec_polygon_lr((__u32)vc->value);
    break;
	case V4L2_CID_MXC_ROT:
	case V4L2_CID_MXC_VF_ROT:
		switch (vc->value) {
		case V4L2_MXC_ROTATE_NONE:
			if (captureMode == ov5653_mode_QSXGA_2592_1944) {
				if (ov5653_set_rotate_mode(ov5653_rotate_none_FULL))
					retval = -EPERM;
			} else {
				if (ov5653_set_rotate_mode(ov5653_rotate_none_VGA))
					retval = -EPERM;
			}
			break;
		case V4L2_MXC_ROTATE_VERT_FLIP:
			if (captureMode == ov5653_mode_QSXGA_2592_1944) {
				if (ov5653_set_rotate_mode(ov5653_rotate_vert_flip_FULL))
					retval = -EPERM;
			} else {
				if (ov5653_set_rotate_mode(ov5653_rotate_vert_flip_VGA))
					retval = -EPERM;
			}
			break;
		case V4L2_MXC_ROTATE_HORIZ_FLIP:
			if (captureMode == ov5653_mode_QSXGA_2592_1944) {
				if (ov5653_set_rotate_mode(ov5653_rotate_horiz_flip_FULL))
					retval = -EPERM;
			} else {
				if (ov5653_set_rotate_mode(ov5653_rotate_horiz_flip_VGA))
					retval = -EPERM;
			}
			break;
		case V4L2_MXC_ROTATE_180:
			if (captureMode == ov5653_mode_QSXGA_2592_1944) {
				if (ov5653_set_rotate_mode(ov5653_rotate_180_FULL))
					retval = -EPERM;
			} else {
				if (ov5653_set_rotate_mode(ov5653_rotate_180_VGA))
					retval = -EPERM;
			}
			break;
		default:
			retval = -EPERM;
			break;
		}
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

static int ioctl_send_command(struct v4l2_int_device *s, struct v4l2_send_command_control *vc) {
	int ret = -1;
	int retval1,retval2;
	u8 loca_val=0;

   printk(KERN_INFO "<< OV5653 0: IOCTL SEND COMMAND: %d>>\n", vc->id);

	ret = ov5653_set_idle_mode();
	if (0 != ret)
		pr_err("error %d setting idle mode\n", ret);
	ov5653_config_auto_focus();
	switch (vc->id) {
		case 101: //step to near
			pr_debug("Stepping to near object\n");
			retval1=ov5653_write_reg(REG_CMD_TAG, 0x01);
			retval2=ov5653_write_reg(REG_CMD_MAIN, 0x05);
			if(retval1 == 0 && retval2 == 0)
				ret = 0;
			break;
		case 102: //step to far
			pr_debug("Stepping to far object\n");
			retval1=ov5653_write_reg(REG_CMD_TAG, 0x02);
			retval2=ov5653_write_reg(REG_CMD_MAIN, 0x05);
			if(retval1 == 0 && retval2 == 0)
				ret = 0;
			break;

		case 103: //step to furthest
			pr_debug("Stepping to furthest object\n");
			retval1=ov5653_write_reg(REG_CMD_TAG, 0x03);
			retval2=ov5653_write_reg(REG_CMD_MAIN, 0x05);
			if(retval1 == 0 && retval2 == 0)
				ret = 0;
			break;

		case 104: //step to nearest
			pr_debug("Stepping to nearest object\n");
			retval1=ov5653_write_reg(REG_CMD_TAG, 0x04);
			retval2=ov5653_write_reg(REG_CMD_MAIN, 0x05);
			if(retval1 == 0 && retval2 == 0)
				ret = 0;
			break;


		case 105: //step to specified position
			pr_debug("Stepping to position: %d\n", vc->value0);
			if(vc->value0 < 0 || vc->value0 > 255)
				return ret;
			loca_val = vc->value0;
			retval1=ov5653_write_reg(REG_CMD_TAG, 0x10);
			retval2=ov5653_write_reg(REG_CMD_PARA0, loca_val);
			ret=ov5653_write_reg(REG_CMD_MAIN, 0x05);
			if(retval1 != 0 && retval2 != 0 && ret != 0)
				ret = -1;
			break;
		default:
			break;
	}

	return ret;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov5653_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov5653_data.pix.pixelformat;
	fsize->discrete.width =
			max(ov5653_mode_info_data[0][fsize->index].width,
			    ov5653_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ov5653_mode_info_data[0][fsize->index].height,
			    ov5653_mode_info_data[1][fsize->index].height);
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;

	if (fival->index < 0 || fival->index > ov5653_mode_MAX)
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov5653_mode_info_data); i++) {
		for (j = 0; j < (ov5653_mode_MAX + 1); j++) {
			if (fival->pixel_format == ov5653_data.pix.pixelformat
			 && fival->width == ov5653_mode_info_data[i][j].width
			 && fival->height == ov5653_mode_info_data[i][j].height
			 && ov5653_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						ov5653_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov5653_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{

	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)	/* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = ov5653_data.pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	int retval = 0;
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov5653_frame_rate frame_rate;


   printk(KERN_INFO "<< OV5653 1: DEVICE INIT>>\n");
	ov5653_data.on = true;

	/* mclk */
	tgt_xclk = ov5653_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV5653_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV5653_XCLK_MIN);
	ov5653_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov5653_data.mclk, ov5653_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

  frame_rate = get_frame_rate_id(tgt_fps);
  ov5653_data.frame_rate = frame_rate;
  ov5653_data.pix.width = 640;
  ov5653_data.pix.height = 480;
  ov5653_data.streamcap.capturemode = ov5653_mode_VGA_640_480;

  retval = ov5653_init_sensor();
	return retval;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov5653_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
	{vidioc_int_send_command_num,
				(v4l2_int_ioctl_func *) ioctl_send_command},
};

static struct v4l2_int_slave ov5653_slave = {
	.ioctls = ov5653_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov5653_ioctl_desc),
};

static struct v4l2_int_device ov5653_int_device = {
	.module = THIS_MODULE,
	.name = "ov5653_1",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov5653_slave,
	},
};

static int write_proc(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char localbuf[256];
	if( count < sizeof(localbuf) ){
		if(copy_from_user(localbuf,buffer,count)){
			printk(KERN_ERR "Error reading user buf\n" );
		} else {
			int addr0 ;
			int value ;
			int numScanned ;
			if(2 == (numScanned = sscanf(localbuf,"%04x %02x", &addr0, &value)) ){
				if( (0xFFFF >= addr0) && (0xff >= value) ){
					s32 rval ;

					rval = ov5653_write_reg(addr0,value);
                                        if (rval < 0)
						pr_err("%s, write reg 0x%x failed: %d\n", __func__, addr0, rval);
					else
                                                pr_err("ov5653[%04x] = %02x\n", addr0,value);
				}
				else
					printk(KERN_ERR "Invalid data: %s\n", localbuf);
			} else if(1 == numScanned){
				if(0xFFFF > addr0){
					s32 rval;
					u8 value;
					rval = ov5653_read_reg(addr0,&value);
					if (0 == rval) {
						pr_err("ov5653[%04x] == 0x%02x\n", addr0,value);
					} else {
						pr_err("%s, read reg 0x%x failed: %d\n", __func__, addr0, rval);
					}
				}
			}
			else
				printk(KERN_ERR "Invalid data: %s\n", localbuf);
		}
	}

	return count ;
}

/*!
 * ov5653 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5653_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	u8 chip_id_high, chip_id_low;
	struct proc_dir_entry *pde ;

	/* Set initial values for the sensor struct. */
	memset(&ov5653_data, 0, sizeof(ov5653_data));
	ov5653_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov5653_data.mclk = plat_data->mclk;
	ov5653_data.mclk_source = plat_data->mclk_source;
	ov5653_data.csi = plat_data->csi;
	ov5653_data.io_init = plat_data->io_init;
	printk("OV5653_1: Probing sensor on CSI: %d, clock: %d\n", ov5653_data.csi, ov5653_data.mclk_source);

	ov5653_data.i2c_client = client;
	ov5653_data.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
  printk(KERN_INFO "%s: pixel format: %X\n", __func__, ov5653_data.pix.pixelformat); 
  ov5653_data.pix.width = 640;
  ov5653_data.pix.height = 480;
	ov5653_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov5653_data.streamcap.capturemode = ov5653_mode_VGA_640_480;
	ov5653_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov5653_data.streamcap.timeperframe.numerator = 1;
  ov5653_data.frame_rate = get_frame_rate_id(DEFAULT_FPS);  
  // Set some default parameters
  prev_exposure_table[ov5653_mode_QSXGA_2592_1944] = 20000;
  prev_exposure_table[ov5653_mode_VGA_640_480] = 4000;

	if (plat_data->io_regulator) {
		io_regulator = regulator_get(&client->dev,
					     plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator,
					      OV5653_VOLTAGE_DIGITAL_IO,
					      OV5653_VOLTAGE_DIGITAL_IO);
			retval = regulator_enable(io_regulator);
			if (retval) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		core_regulator = regulator_get(&client->dev,
					       plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					      OV5653_VOLTAGE_DIGITAL_CORE,
					      OV5653_VOLTAGE_DIGITAL_CORE);
			retval = regulator_enable(core_regulator);
			if (retval) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		analog_regulator = regulator_get(&client->dev,
						 plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator,
					      OV5653_VOLTAGE_ANALOG,
					      OV5653_VOLTAGE_ANALOG);
			retval = regulator_enable(analog_regulator);
			if (retval) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err3;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			analog_regulator = NULL;
	}

	if (plat_data->io_init)
		plat_data->io_init();

	if (plat_data->pwdn)
		plat_data->pwdn(0);

	retval = ov5653_read_reg(OV5653_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x56) {
		pr_warning("camera ov5653 is not found\n");
		retval = -ENODEV;
		goto err4;
	}
	retval = ov5653_read_reg(OV5653_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x51) {
		pr_warning("camera ov5653 is not found\n");
		retval = -ENODEV;
		goto err4;
	}

//	if (plat_data->pwdn)
//		plat_data->pwdn(1);

	camera_plat = plat_data;

	ov5653_int_device.priv = &ov5653_data;

	sprintf(procname, "driver/ov5653_%d", plat_data->csi);
	pde = create_proc_entry(procname, 0, 0);
	if( pde ) {
		pde->write_proc = write_proc ;
		pde->data = &ov5653_int_device ;
	}
	else
		printk( KERN_ERR "Error creating ov5653 proc entry\n" );

	retval = v4l2_int_device_register(&ov5653_int_device);

	pr_info("Camera ov5653_1 is found\n");
//	if (plat_data->pwdn)
//		plat_data->pwdn(1);
	return retval;

err4:
	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}
err3:
	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}
err2:
	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}
err1:
	return retval;
}

/*!
 * ov5653 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5653_remove(struct i2c_client *client)
{
	remove_proc_entry(procname, NULL);

	v4l2_int_device_unregister(&ov5653_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	return 0;
}

/*!
 * ov5653 init function
 * Called by insmod ov5653_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov5653_1_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov5653_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * ov5653 cleanup function
 * Called on rmmod ov5653_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov5653_1_clean(void)
{
	i2c_del_driver(&ov5653_i2c_driver);
}

module_init(ov5653_1_init);
module_exit(ov5653_1_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV5653 1 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
