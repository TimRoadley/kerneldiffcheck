/*
 * Copyright 2005-2013 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @defgroup IPU MXC Image Processing Unit (IPU) Driver
 */
/*!
 * @file arch-mxc/ipu.h
 *
 * @brief This file contains the IPU driver API declarations.
 *
 * @ingroup IPU
 */

#ifndef __ASM_ARCH_IPU_H__
#define __ASM_ARCH_IPU_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#ifndef __cplusplus
typedef unsigned char bool;
#endif
#define irqreturn_t int
#define dma_addr_t int
#define uint32_t unsigned int
#define uint16_t unsigned short
#define uint8_t unsigned char
#define u32 unsigned int
#define u8 unsigned char
#define __u32 u32

/*!
 * Enumeration of IPU rotation modes
 */
typedef enum {
	/* Note the enum values correspond to BAM value */
	IPU_ROTATE_NONE = 0,
	IPU_ROTATE_VERT_FLIP = 1,
	IPU_ROTATE_HORIZ_FLIP = 2,
	IPU_ROTATE_180 = 3,
	IPU_ROTATE_90_RIGHT = 4,
	IPU_ROTATE_90_RIGHT_VFLIP = 5,
	IPU_ROTATE_90_RIGHT_HFLIP = 6,
	IPU_ROTATE_90_LEFT = 7,
} ipu_rotate_mode_t;

/*!
 * Enumeration of VDI MOTION select
 */
typedef enum {
	MED_MOTION = 0,
	LOW_MOTION = 1,
	HIGH_MOTION = 2,
} ipu_motion_sel;

/*!
 * Enumeration of DI ports for ADC.
 */
typedef enum {
	DISP0,
	DISP1,
	DISP2,
	DISP3
} display_port_t;

/*  IPU Pixel format definitions */
/*  Four-character-code (FOURCC) */
#define fourcc(a, b, c, d)\
	 (((__u32)(a)<<0)|((__u32)(b)<<8)|((__u32)(c)<<16)|((__u32)(d)<<24))

/*!
 * @name IPU Pixel Formats
 *
 * Pixel formats are defined with ASCII FOURCC code. The pixel format codes are
 * the same used by V4L2 API.
 */

/*! @{ */
/*! @name Generic or Raw Data Formats */
/*! @{ */
#define IPU_PIX_FMT_GENERIC fourcc('I', 'P', 'U', '0')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_GENERIC_32 fourcc('I', 'P', 'U', '1')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_LVDS666 fourcc('L', 'V', 'D', '6')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_LVDS888 fourcc('L', 'V', 'D', '8')	/*!< IPU Generic Data */
#define IPU_PIX_FMT_BT656    fourcc('B', 'T', '6', '5')	/*!< BT656, 16 UYVY */
#define IPU_PIX_FMT_BT1120    fourcc('B', 'T', '1', '1')	/*!< BT1120, 16 UYVY */
/*! @} */
/*! @name RGB Formats */
/*! @{ */
#define IPU_PIX_FMT_RGB332  fourcc('R', 'G', 'B', '1')	/*!<  8  RGB-3-3-2    */
#define IPU_PIX_FMT_RGB555  fourcc('R', 'G', 'B', 'O')	/*!< 16  RGB-5-5-5    */
#define IPU_PIX_FMT_RGB565  fourcc('R', 'G', 'B', 'P')	/*!< 1 6  RGB-5-6-5   */
#define IPU_PIX_FMT_RGB666  fourcc('R', 'G', 'B', '6')	/*!< 18  RGB-6-6-6    */
#define IPU_PIX_FMT_BGR666  fourcc('B', 'G', 'R', '6')	/*!< 18  BGR-6-6-6    */
#define IPU_PIX_FMT_BGR24   fourcc('B', 'G', 'R', '3')	/*!< 24  BGR-8-8-8    */
#define IPU_PIX_FMT_RGB24   fourcc('R', 'G', 'B', '3')	/*!< 24  RGB-8-8-8    */
#define IPU_PIX_FMT_GBR24   fourcc('G', 'B', 'R', '3')	/*!< 24  GBR-8-8-8    */
#define IPU_PIX_FMT_BGR32   fourcc('B', 'G', 'R', '4')	/*!< 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_BGRA32  fourcc('B', 'G', 'R', 'A')	/*!< 32  BGR-8-8-8-8  */
#define IPU_PIX_FMT_RGB32   fourcc('R', 'G', 'B', '4')	/*!< 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_RGBA32  fourcc('R', 'G', 'B', 'A')	/*!< 32  RGB-8-8-8-8  */
#define IPU_PIX_FMT_ABGR32  fourcc('A', 'B', 'G', 'R')	/*!< 32  ABGR-8-8-8-8 */
/*! @} */
/*! @name YUV Interleaved Formats */
/*! @{ */
#define IPU_PIX_FMT_YUYV    fourcc('Y', 'U', 'Y', 'V')	/*!< 16 YUV 4:2:2 */
#define IPU_PIX_FMT_UYVY    fourcc('U', 'Y', 'V', 'Y')	/*!< 16 YUV 4:2:2 */
#define IPU_PIX_FMT_YVYU    fourcc('Y', 'V', 'Y', 'U')  /*!< 16 YVYU 4:2:2 */
#define IPU_PIX_FMT_VYUY    fourcc('V', 'Y', 'U', 'Y')  /*!< 16 VYYU 4:2:2 */
#define IPU_PIX_FMT_Y41P    fourcc('Y', '4', '1', 'P')	/*!< 12 YUV 4:1:1 */
#define IPU_PIX_FMT_YUV444  fourcc('Y', '4', '4', '4')	/*!< 24 YUV 4:4:4 */
#define IPU_PIX_FMT_VYU444  fourcc('V', '4', '4', '4')	/*!< 24 VYU 4:4:4 */
/* two planes -- one Y, one Cb + Cr interleaved  */
#define IPU_PIX_FMT_NV12    fourcc('N', 'V', '1', '2') /* 12  Y/CbCr 4:2:0  */
/* two planes -- 12  tiled Y/CbCr 4:2:0  */
#define IPU_PIX_FMT_TILED_NV12    fourcc('T', 'N', 'V', 'P')
#define IPU_PIX_FMT_TILED_NV12F   fourcc('T', 'N', 'V', 'F')

/*! @} */
/*! @name YUV Planar Formats */
/*! @{ */
#define IPU_PIX_FMT_GREY    fourcc('G', 'R', 'E', 'Y')	/*!< 8  Greyscale */
#define IPU_PIX_FMT_YVU410P fourcc('Y', 'V', 'U', '9')	/*!< 9  YVU 4:1:0 */
#define IPU_PIX_FMT_YUV410P fourcc('Y', 'U', 'V', '9')	/*!< 9  YUV 4:1:0 */
#define IPU_PIX_FMT_YVU420P fourcc('Y', 'V', '1', '2')	/*!< 12 YVU 4:2:0 */
#define IPU_PIX_FMT_YUV420P fourcc('I', '4', '2', '0')	/*!< 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YUV420P2 fourcc('Y', 'U', '1', '2')	/*!< 12 YUV 4:2:0 */
#define IPU_PIX_FMT_YVU422P fourcc('Y', 'V', '1', '6')	/*!< 16 YVU 4:2:2 */
#define IPU_PIX_FMT_YUV422P fourcc('4', '2', '2', 'P')	/*!< 16 YUV 4:2:2 */
/* non-interleaved 4:4:4 */
#define IPU_PIX_FMT_YUV444P fourcc('4', '4', '4', 'P')	/*!< 24 YUV 4:4:4 */
/*! @} */
#define IPU_PIX_FMT_TILED_NV12_MBALIGN	(16)
#define TILED_NV12_FRAME_SIZE(w, h)	\
		(ALIGN((w) * (h), SZ_4K) + ALIGN((w) * (h) / 2, SZ_4K))
/* IPU device */
typedef enum {
	RGB_CS,
	YUV_CS,
	NULL_CS
} cs_t;

struct ipu_pos {
	u32 x;
	u32 y;
};

struct ipu_crop {
	struct ipu_pos pos;
	u32 w;
	u32 h;
};

struct ipu_deinterlace {
	bool	enable;
	u8	motion; /*see ipu_motion_sel*/
#define IPU_DEINTERLACE_FIELD_TOP	0
#define IPU_DEINTERLACE_FIELD_BOTTOM	1
#define IPU_DEINTERLACE_FIELD_MASK	\
		(IPU_DEINTERLACE_FIELD_TOP | IPU_DEINTERLACE_FIELD_BOTTOM)
	/* deinterlace frame rate double flags */
#define IPU_DEINTERLACE_RATE_EN		0x80
#define IPU_DEINTERLACE_RATE_FRAME1	0x40
#define IPU_DEINTERLACE_RATE_MASK	\
		(IPU_DEINTERLACE_RATE_EN | IPU_DEINTERLACE_RATE_FRAME1)
#define IPU_DEINTERLACE_MAX_FRAME	2
	u8	field_fmt;
};

struct ipu_input {
	u32 width;
	u32 height;
	u32 format;
	struct ipu_crop crop;
	dma_addr_t paddr;

	struct ipu_deinterlace deinterlace;
	dma_addr_t paddr_n; /*valid when deinterlace enable*/
};

struct ipu_alpha {
#define IPU_ALPHA_MODE_GLOBAL	0
#define IPU_ALPHA_MODE_LOCAL	1
	u8 mode;
	u8 gvalue; /* 0~255 */
	dma_addr_t loc_alp_paddr;
};

struct ipu_colorkey {
	bool enable;
	u32 value; /* RGB 24bit */
};

struct ipu_overlay {
	u32	width;
	u32	height;
	u32	format;
	struct ipu_crop crop;
	struct ipu_alpha alpha;
	struct ipu_colorkey colorkey;
	dma_addr_t paddr;
};

struct ipu_output {
	u32	width;
	u32	height;
	u32	format;
	u8	rotate;
	struct ipu_crop crop;
	dma_addr_t paddr;
};

struct ipu_task {
	struct ipu_input input;
	struct ipu_output output;

	bool overlay_en;
	struct ipu_overlay overlay;

#define IPU_TASK_PRIORITY_NORMAL 0
#define IPU_TASK_PRIORITY_HIGH	1
	u8	priority;

#define	IPU_TASK_ID_ANY	0
#define	IPU_TASK_ID_VF	1
#define	IPU_TASK_ID_PP	2
#define	IPU_TASK_ID_MAX 3
	u8	task_id;

	int	timeout;
};

enum {
	IPU_CHECK_OK = 0,
	IPU_CHECK_WARN_INPUT_OFFS_NOT8ALIGN = 0x1,
	IPU_CHECK_WARN_OUTPUT_OFFS_NOT8ALIGN = 0x2,
	IPU_CHECK_WARN_OVERLAY_OFFS_NOT8ALIGN = 0x4,
	IPU_CHECK_ERR_MIN,
	IPU_CHECK_ERR_INPUT_CROP,
	IPU_CHECK_ERR_OUTPUT_CROP,
	IPU_CHECK_ERR_OVERLAY_CROP,
	IPU_CHECK_ERR_INPUT_OVER_LIMIT,
	IPU_CHECK_ERR_OV_OUT_NO_FIT,
	IPU_CHECK_ERR_OVERLAY_WITH_VDI,
	IPU_CHECK_ERR_PROC_NO_NEED,
	IPU_CHECK_ERR_SPLIT_INPUTW_OVER,
	IPU_CHECK_ERR_SPLIT_INPUTH_OVER,
	IPU_CHECK_ERR_SPLIT_OUTPUTW_OVER,
	IPU_CHECK_ERR_SPLIT_OUTPUTH_OVER,
	IPU_CHECK_ERR_SPLIT_WITH_ROT,
	IPU_CHECK_ERR_NOT_SUPPORT,
	IPU_CHECK_ERR_NOT16ALIGN,
};

/* IOCTL commands */
#define IPU_CHECK_TASK		_IOWR('I', 0x1, struct ipu_task)
#define IPU_QUEUE_TASK		_IOW('I', 0x2, struct ipu_task)
#define IPU_ALLOC		_IOWR('I', 0x3, int)
#define IPU_FREE		_IOW('I', 0x4, int)
#define IPU_DMA_MAP		_IOWR('I', 0x5, struct ipu_task)
#define IPU_DMA_UNMAP		_IOW('I', 0x6, int)

/* export functions */

#endif
