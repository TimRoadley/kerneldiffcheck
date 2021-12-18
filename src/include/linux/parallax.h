#ifndef __PARALLAX_H__
#define __PARALLAX_H__

#ifndef PLAT_PHYS_OFFSET
   #define PLAT_PHYS_OFFSET        0x10000000
#endif
#define PARALLAX_GPU_MEM_OFFS   (224*1024*1024) 
#define PARALLAX_DMA_MEM_OFFS   (196*1024*1024) 
#define PARALLAX_CSI_MEM_OFFS   (176*1024*1024) 
#define PARALLAX_VPU_MEM_OFFS   (164*1024*1024) 
#define PARALLAX_HVPU_MEM_OFFS  (156*1024*1024) 
#define PARALLAX_DVPU_MEM_OFFS  (152*1024*1024) 
#define PARALLAX_JVPU_MEM_OFFS  (136*1024*1024) 

#define PARALLAX_DMA_BUF_SIZE   (10*1024*1024)
#define PARALLAX_VPU_BUF_SIZE   (128*4096 + 4096*16)
#define PARALLAX_DVPU_BUF_SIZE  (2*1024*1024)
#define PARALLAX_JVPU_BUF_SIZE  (8*1024*1024)
#define PARALLAX_HVPU_BUF_SIZE  (4*1024*1024)
#define PARALLAX_GPU_BUF_SIZE   (32*1024*1024)
#define PARALLAX_DMA_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_DMA_MEM_OFFS)
#define PARALLAX_CSI_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_CSI_MEM_OFFS)
#define PARALLAX_VPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_VPU_MEM_OFFS)
#define PARALLAX_DVPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_DVPU_MEM_OFFS)
#define PARALLAX_JVPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_JVPU_MEM_OFFS)
#define PARALLAX_HVPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_HVPU_MEM_OFFS)
#define PARALLAX_GPU_MEM_START  (PLAT_PHYS_OFFSET+PARALLAX_GPU_MEM_OFFS)

#define PARALLAX_VPU_BUF_NUM	20
#define PARALLAX_JVPU_BUF_NUM	2
#define PARALLAX_HVPU_BUF_NUM	2
#define V4L2_BUF_TYPE_DMAMAP 0xaa55
#define DMA_FRAME_BUFFER_NUM 3
#define M3_VPU_BUFFER_IC_ID 2

/* Parallax snapshot system */
#define PARALLAX_SNAPSHOT_MAX_BUFFERS  16
/* Frame Buffer type */
#define PARALLAX_SNAPSHOT_BUF_TYPE_NONE 0x00
#define PARALLAX_SNAPSHOT_BUF_TYPE_DMA  0x01
/* Frame Buffer Mode */
#define PARALLAX_SNAPSHOT_BUF_MODE_ALLOC 0x02 /* Buffer allocated */	
#define PARALLAX_SNAPSHOT_BUF_MODE_MAPD  0x04 /* Buffer mapped */	
#define PARALLAX_SNAPSHOT_BUF_MODE_QUED  0x08 /* Buffer queued */  
#define PARALLAX_SNAPSHOT_BUF_MODE_DONE  0x10 /* Buffer done */  
#define PARALLAX_SNAPSHOT_BUF_MODE_ERR   0x80 /* Buffer error */  

/* Generic control */
#define PARALLAX_SNAPSHOT_MXCV4L_SUSPEND 0x01 /* MXC V4L suspended/low power */
#define PARALLAX_SNAPSHOT_MXCV4L_POWER   0x02 /* MXC V4L iface power up/down */

/* DEQUEUE errors */
#define PARALLAX_SNAPSHOT_DEQUE_ERROR_BNF		(-9001) /* Buffer not filled */
#define PARALLAX_SNAPSHOT_DEQUE_ERROR_BNQ		(-9002) /* Buffer not queued */
#define PARALLAX_SNAPSHOT_DEQUE_ERROR_CBNQ		(-9003) /* Camera Callback Buffer not queued */
#define PARALLAX_SNAPSHOT_DEQUE_ERROR_BAQ		(-9004) /* Buffer allready queued */
#define PARALLAX_SNAPSHOT_DEQUE_ERROR_ODB		(-9005) /* Overwrite done buffer */

struct mxcv4loverlay_kernel_entry {
    unsigned long csi_interrupts;
    unsigned char buffer_count;
    unsigned char buffer_control[PARALLAX_SNAPSHOT_MAX_BUFFERS];
    unsigned char mxcv4l_control;
    unsigned char csi_id;
    unsigned char ipu_id;
    int           dqueue_err;
}__attribute__((packed));

#endif
