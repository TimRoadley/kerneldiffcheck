#ifndef __PARALLAX_H__
#define __PARALLAX_H__

#ifndef PLAT_PHYS_OFFSET
   #define PLAT_PHYS_OFFSET        UL(0x10000000)
#endif

#ifdef CONFIG_PA_PARALLAX_REDUCED_MEM
/* Offset for boards with 256MB of mem */
#if defined(CONFIG_MXC_CAMERA_OV5653) || defined(CONFIG_MXC_CAMERA_OV5653_MODULE)
#define PARALLAX_DMA_MEM_OFFS   (144*1024*1024) 
#define PARALLAX_CSI_MEM_OFFS   (124*1024*1024) 
#define PARALLAX_VPU_MEM_OFFS   (114*1024*1024) 
#define PARALLAX_LVPU_MEM_OFFS  (110*1024*1024) 
#else
#define PARALLAX_DMA_MEM_OFFS   (174*1024*1024) 
#define PARALLAX_CSI_MEM_OFFS   (154*1024*1024) 
#define PARALLAX_VPU_MEM_OFFS   (144*1024*1024) 
#define PARALLAX_LVPU_MEM_OFFS  (140*1024*1024) 
#endif
#else
/* Offset for boards with 512MB of mem */
#define PARALLAX_DMA_MEM_OFFS   (400*1024*1024) 
#endif

/* Offset for boards with 256MB of mem */
//#define PARALLAX_DMA_MEM_OFFS   (180*1024*1024)

/* Offset with boards with 512MB of mem */
/*
#define PARALLAX_DMA_MEM_OFFS   (400*1024*1024)
*/
#if defined(CONFIG_MXC_CAMERA_OV5653) || defined(CONFIG_MXC_CAMERA_OV5653_MODULE)
#define PARALLAX_DMA_BUF_SIZE   (15*1024*1024) 
#else
#define PARALLAX_DMA_BUF_SIZE   (10*1024*1024)
#endif
#define PARALLAX_VPU_BUF_SIZE   (128*4096 + 4096*16)
#define PARALLAX_LVPU_BUF_SIZE   (2*1024*1024)
#define PARALLAX_DMA_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_DMA_MEM_OFFS)
#define PARALLAX_CSI_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_CSI_MEM_OFFS)
#define PARALLAX_VPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_VPU_MEM_OFFS)
#define PARALLAX_LVPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_LVPU_MEM_OFFS)

#define PARALLAX_VPU_BUF_NUM	16
#define PARALLAX_LVPU_BUF_NUM	2
#define V4L2_BUF_TYPE_DMAMAP 0xaa55

#endif
