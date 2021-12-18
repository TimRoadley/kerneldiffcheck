/*
 * Copyright 2006-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_vpu.c
 *
 * @brief VPU system initialization and file operation implementation
 *
 * @ingroup VPU
 */

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/dma-mapping.h>
#include <linux/iram_alloc.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fsl_devices.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/regulator/consumer.h>
#include <linux/page-flags.h>
#include <linux/mm_types.h>
#include <linux/types.h>
#include <linux/memblock.h>
#include <linux/memory.h>
#include <linux/parallax.h>
#include <asm/page.h>
#include <asm/sizes.h>
#include <mach/clock.h>
#include <mach/hardware.h>

#include <mach/mxc_vpu.h>

#define MXC_VPU_MAJOR 252
#define MXC_VPU_ASIGNED_DMA_BUFFERS 6

#define USE_PA_DMA_MEM_MANAGER 1

/* Define one new pgprot which combined uncached and XN(never executable) */
#define pgprot_noncachedxn(prot) \
	__pgprot_modify(prot, L_PTE_MT_MASK, L_PTE_MT_UNCACHED | L_PTE_XN)

struct vpu_priv {
	struct fasync_struct *async_queue;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
};

/* To track the allocated memory buffer */
typedef struct memalloc_record {
	struct list_head list;
	struct vpu_mem_desc mem;
} memalloc_record;

struct iram_setting {
	u32 start;
	u32 end;
};


static LIST_HEAD(head);

static int vpu_major = 0;
static int vpu_clk_usercount;
static struct class *vpu_class;
static struct vpu_priv vpu_data;
static u8 open_count;
static struct clk *vpu_clk;
static struct vpu_mem_desc bitwork_mem = { 0 };
static struct vpu_mem_desc pic_para_mem = { 0 };
static struct vpu_mem_desc user_data_mem = { 0 };
static struct vpu_mem_desc share_mem = { 0 };
static struct vpu_mem_desc vshare_mem = { 0 };

static void __iomem *vpu_base;
static int vpu_ipi_irq;
static u32 phy_vpu_base_addr;
static phys_addr_t top_address_DRAM;
static struct mxc_vpu_platform_data *vpu_plat;

/* IRAM setting */
static struct iram_setting iram;

/* implement the blocking ioctl */
static int irq_status;
static int codec_done;
static wait_queue_head_t vpu_queue;


#ifdef CONFIG_SOC_IMX6Q
#define MXC_VPU_HAS_JPU
#endif

#ifdef MXC_VPU_HAS_JPU
static int vpu_jpu_irq;
#endif


static unsigned int regBk[64];
static struct regulator *vpu_regulator;
static unsigned int pc_before_suspend;
static struct vpu_mem_desc dma_free_pool[PARALLAX_VPU_BUF_NUM];
static struct vpu_mem_desc vpu_dma_free_pool[MXC_VPU_ASIGNED_DMA_BUFFERS] = {
 {PARALLAX_HVPU_BUF_SIZE, PARALLAX_HVPU_MEM_START, 0, 0, 0},
 {PARALLAX_HVPU_BUF_SIZE, PARALLAX_HVPU_MEM_START+PARALLAX_HVPU_BUF_SIZE, 0, 0, 0},
 {PARALLAX_JVPU_BUF_SIZE, PARALLAX_JVPU_MEM_START, 0, 0, 0},
 {PARALLAX_JVPU_BUF_SIZE, PARALLAX_JVPU_MEM_START+PARALLAX_JVPU_BUF_SIZE, 0, 0, 0},
 {PARALLAX_DVPU_BUF_SIZE, PARALLAX_DVPU_MEM_START, 0, 0, 0},
 {PARALLAX_DVPU_BUF_SIZE, PARALLAX_DVPU_MEM_START+PARALLAX_DVPU_BUF_SIZE, 0, 0, 0}
};


#define	READ_REG(x)		__raw_readl(vpu_base + x)
#define	WRITE_REG(val, x)	__raw_writel(val, vpu_base + x)

/*!
* Custom DMA buffer assignment through Parallax memory.
* @return physical memory address or 0 for failure
*/

static void parallax_vpu_fill_dma_buffers(void)
{
    int i;
    memset(&dma_free_pool[0], 0x00, sizeof(struct vpu_mem_desc)*PARALLAX_VPU_BUF_NUM);
    for(i = 0; i < PARALLAX_VPU_BUF_NUM; i++) {
        dma_free_pool[i].phy_addr = PARALLAX_VPU_MEM_START + i*PARALLAX_VPU_BUF_SIZE;
        dma_free_pool[i].size = PARALLAX_VPU_BUF_SIZE;
        dma_free_pool[i].flags = 0;
    }
}


static struct vpu_mem_desc *parallax_vpu_alloc_dma_buf(u32 size)
{
    int i;
    /* We cannot allocate more than VPU DMA buffer size. check it here */
    if(size > PARALLAX_VPU_BUF_SIZE) {
        for(i = 0; i < MXC_VPU_ASIGNED_DMA_BUFFERS; i++) {
            if((vpu_dma_free_pool[i].flags == 0) && (vpu_dma_free_pool[i].size == size)) {
#ifdef IMX_VPU_DEBUG
	        printk(KERN_INFO "++++mxc_vpu: Found Internal VPU DMA location: %d, addr: %X for size: %d\n", 
                       i, (u32)vpu_dma_free_pool[i].phy_addr, size);
#endif
                vpu_dma_free_pool[i].flags = (VPU_MEM_DESCR_DMAMAP | VPU_MEM_DESCR_VPU_DMA);
                return &vpu_dma_free_pool[i];
            } 
        }
    }
    for(i = 0; i < PARALLAX_VPU_BUF_NUM; i++) {
        if(dma_free_pool[i].flags == 0) {
#ifdef IMX_VPU_DEBUG
	    	printk(KERN_INFO "++++mxc_vpu: Found DMA location: %d, addr: %X for size: %d\n", i, (u32)dma_free_pool[i].phy_addr, size);
#endif
	    	dma_free_pool[i].flags = VPU_MEM_DESCR_DMAMAP;
            	return &dma_free_pool[i];
	}
    }	
    return NULL;
}

static void parallax_vpu_free_dma_buf(dma_addr_t addr, u32 flags)
{
    u32 i = 0;
    /* Find index */
    if((flags & VPU_MEM_DESCR_DMAMAP) && (addr != 0)) {
        if(flags & VPU_MEM_DESCR_VPU_DMA) {
            for(i = 0; i < MXC_VPU_ASIGNED_DMA_BUFFERS; i++) {
                if(vpu_dma_free_pool[i].phy_addr == addr) {
#ifdef IMX_VPU_DEBUG
                    printk(KERN_INFO "++++mxc_vpu: Clearing Internal VPU DMA location: %d, addr: %X for size: %d\n", 
                           i, (u32)vpu_dma_free_pool[i].phy_addr, vpu_dma_free_pool[i].size);
#endif
                    vpu_dma_free_pool[i].flags = 0;
					if(vpu_dma_free_pool[i].cpu_addr != 0) {
						iounmap((void *)(vpu_dma_free_pool[i].cpu_addr));
						vpu_dma_free_pool[i].cpu_addr = 0;
					}                    
                    return;
                }
            }
        }
        else {
            for(i = 0; i < PARALLAX_VPU_BUF_NUM; i++) {
                if(dma_free_pool[i].phy_addr == addr) {
#ifdef IMX_VPU_DEBUG
                    printk(KERN_INFO "++++mxc_vpu: clearing Parallax DMA location: %d, addr: %X\n", i, addr);
#endif
    	            dma_free_pool[i].flags = 0;
					if(dma_free_pool[i].cpu_addr != 0) {
						iounmap((void *)(dma_free_pool[i].cpu_addr));
						dma_free_pool[i].cpu_addr = 0;
					}                    
                    return;
	        }
            }	
        }
    }
}


/*!
 * Private function to alloc dma buffer
 * @return status  0 success.
 */
/* 
*  KP: OK, here we need to implement static DMA memory allocator. 
*  We are using OpenCV2 which fragments memory a lot and causing failure on contiguous 
*  pages requests.
*/
static int vpu_alloc_dma_buffer(struct vpu_mem_desc *mem)
{
    struct vpu_mem_desc *vpu_mem = NULL;
//    printk(KERN_INFO "++++mxc_vpu: allocating coherent buf, size: %d\n", mem->size);
    /* First, try to use DMA buffer pool */
#ifdef USE_PA_DMA_MEM_MANAGER
    if((vpu_mem = parallax_vpu_alloc_dma_buf(PAGE_ALIGN(mem->size))) != NULL) {
        vpu_mem->cpu_addr = (unsigned long)ioremap(vpu_mem->phy_addr, PAGE_ALIGN(vpu_mem->size));
        if(vpu_mem->cpu_addr == 0) {
            printk(KERN_INFO "VPU driver: ERROR: Unable to map CPU addr for: %X, size: %d\n",
                   vpu_mem->phy_addr, PAGE_ALIGN(vpu_mem->size));
            return -1;
        }        
        *mem = *vpu_mem;
    }
    else {
#endif
 	    mem->cpu_addr = (unsigned long)
	    dma_alloc_coherent(NULL, PAGE_ALIGN(mem->size),
			       (dma_addr_t *) (&mem->phy_addr),
			       GFP_DMA | GFP_KERNEL);
		pr_debug("[ALLOC] mem alloc cpu_addr = 0x%x\n", mem->cpu_addr);
#ifdef IMX_VPU_DEBUG
		printk(KERN_INFO "+++++mxc_vpu: [ALLOC] mem alloc cpu_addr = 0x%x, size: %d\n", 
               mem->cpu_addr, mem->size);
#endif
		if ((void *)(mem->cpu_addr) == NULL) {
			printk(KERN_ERR "Physical memory allocation error!\n");
			return -1;
		}
		mem->flags = VPU_MEM_DESCR_ALLOC;
#ifdef USE_PA_DMA_MEM_MANAGER
    }
#endif
    return 0;
}

/*!
 * Private function to free dma buffer
 */
static void vpu_free_dma_buffer(struct vpu_mem_desc *mem)
{
#ifdef IMX_VPU_DEBUG
        printk(KERN_INFO "+++++Freeing DMA buffer: phy: %X, cpu: 0x%x, size: %d, flags: 0x%x\n", 
		mem->phy_addr, mem->cpu_addr, mem->size, mem->flags);
#endif
#ifdef USE_PA_DMA_MEM_MANAGER
	if (mem->flags != 0) {
	    /* Check if we've got custom DMA allocation */
	    if(mem->flags & (VPU_MEM_DESCR_DMAMAP | VPU_MEM_DESCR_VPU_DMA)) {
			parallax_vpu_free_dma_buf(mem->phy_addr, mem->flags);
	    }
        else {
#endif
#ifdef IMX_VPU_DEBUG
            printk(KERN_INFO "++++mxc_vpu: [FREE] freeing dynamic DMA buf, cpuaddr: 0x%x\n", mem->cpu_addr);
#endif
			dma_free_coherent(0, PAGE_ALIGN(mem->size),
				  (void *)mem->cpu_addr, mem->phy_addr);
#ifdef USE_PA_DMA_MEM_MANAGER
	    }
	}
#endif
}

/*!
 * Private function to free buffers
 * @return status  0 success.
 */
static int vpu_free_buffers(void)
{
	struct memalloc_record *rec, *n;
	struct vpu_mem_desc mem;

#ifdef IMX_VPU_DEBUG
    printk(KERN_INFO "++++mxc_vpu: Freeing ALL dynamic DMA buffers\n");
#endif
	list_for_each_entry_safe(rec, n, &head, list) {
		mem = rec->mem;
		if (mem.phy_addr != 0) {
			vpu_free_dma_buffer(&mem);
			pr_debug("[FREE] freed paddr=0x%08X\n", mem.phy_addr);
			/* delete from list */
			list_del(&rec->list);
			kfree(rec);
		}
	}

	return 0;
}

static inline void vpu_worker_callback(struct work_struct *w)
{
	struct vpu_priv *dev = container_of(w, struct vpu_priv,
				work);

	if (dev->async_queue)
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);

	irq_status = 1;
	/*
	 * Clock is gated on when dec/enc started, gate it off when
	 * codec is done.
	 */
	if (codec_done) {
		codec_done = 0;
	}

	wake_up_interruptible(&vpu_queue);
}

/*!
 * @brief vpu interrupt handler
 */
static irqreturn_t vpu_ipi_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;
	unsigned long reg;

	reg = READ_REG(BIT_INT_REASON);
	if (reg & 0x8)
		codec_done = 1;
	WRITE_REG(0x1, BIT_INT_CLEAR);

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}

/*!
 * @brief vpu jpu interrupt handler
 */
#ifdef MXC_VPU_HAS_JPU
static irqreturn_t vpu_jpu_irq_handler(int irq, void *dev_id)
{
	struct vpu_priv *dev = dev_id;
	unsigned long reg;

	reg = READ_REG(MJPEG_PIC_STATUS_REG);
	if (reg & 0x3)
		codec_done = 1;

	queue_work(dev->workqueue, &dev->work);

	return IRQ_HANDLED;
}
#endif

/*!
 * @brief check phy memory prepare to pass to vpu is valid or not, we
 * already address some issue that if pass a wrong address to vpu
 * (like virtual address), system will hang.
 *
 * @return true return is a valid phy memory address, false return not.
 */
bool vpu_is_valid_phy_memory(u32 paddr)
{
#ifdef IMX_VPU_DEBUG
    printk("++++checking PHY mem: 0x%X, top: 0x%X\n", paddr, top_address_DRAM);
#endif
	if (paddr > top_address_DRAM)
		return false;

	return true;
}

/*!
 * @brief open function for vpu file operation
 *
 * @return  0 on success or negative error code on error
 */
static int vpu_open(struct inode *inode, struct file *filp)
{
        printk(KERN_INFO "-----MX VPU OPEN -----\n");
	mutex_lock(&vpu_data.lock);

	if (open_count++ == 0) {
		if (!IS_ERR(vpu_regulator))
			regulator_enable(vpu_regulator);

#ifdef CONFIG_SOC_IMX6Q
		clk_enable(vpu_clk);
		if (READ_REG(BIT_CUR_PC))
			printk(KERN_DEBUG "Not power off before vpu open!\n");
		clk_disable(vpu_clk);
#endif
	}

	filp->private_data = (void *)(&vpu_data);
	mutex_unlock(&vpu_data.lock);
	return 0;
}

/*!
 * @brief IO ctrl function for vpu file operation
 * @param cmd IO ctrl command
 * @return  0 on success or negative error code on error
 */
static long vpu_ioctl(struct file *filp, u_int cmd,
		     u_long arg)
{
	int ret = 0;

	switch (cmd) {
	case VPU_IOC_PHYMEM_ALLOC:
		{
			struct memalloc_record *rec;

			rec = kzalloc(sizeof(*rec), GFP_KERNEL);
			if (!rec)
				return -ENOMEM;

			ret = copy_from_user(&(rec->mem),
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				return -EFAULT;
			}

			pr_debug("[ALLOC] mem alloc size = 0x%x\n",
				 rec->mem.size);

                        mutex_lock(&vpu_data.lock);
			ret = vpu_alloc_dma_buffer(&(rec->mem));
                        mutex_unlock(&vpu_data.lock);
			if (ret == -1 || rec->mem.phy_addr == 0) {
				kfree(rec);
				printk(KERN_ERR
				       "Physical memory allocation error!\n");
				break;
			}
			ret = copy_to_user((void __user *)arg, &(rec->mem),
					   sizeof(struct vpu_mem_desc));
			if (ret) {
				kfree(rec);
				ret = -EFAULT;
				break;
			}

			mutex_lock(&vpu_data.lock);
			list_add(&rec->list, &head);
			mutex_unlock(&vpu_data.lock);

			break;
		}
	case VPU_IOC_PHYMEM_FREE:
		{
			struct memalloc_record *rec, *n;
			struct vpu_mem_desc vpu_mem;

			ret = copy_from_user(&vpu_mem,
					     (struct vpu_mem_desc *)arg,
					     sizeof(struct vpu_mem_desc));
			if (ret)
				return -EACCES;
			mutex_lock(&vpu_data.lock);

			pr_debug("[FREE] mem freed cpu_addr = 0x%x\n",
				 vpu_mem.phy_addr);
			if (vpu_mem.phy_addr != 0) {
				vpu_free_dma_buffer(&vpu_mem);
			}
			list_for_each_entry_safe(rec, n, &head, list) {
				if (rec->mem.phy_addr == vpu_mem.phy_addr) {
					/* delete from list */
					list_del(&rec->list);
					kfree(rec);
					break;
				}
			}
			mutex_unlock(&vpu_data.lock);
			break;
		}
        case VPU_IOC_DMAMEM_FREE:
		{
			/* This forces to free DMA memory */
                mutex_lock(&vpu_data.lock);
		vpu_free_buffers();
                mutex_unlock(&vpu_data.lock);
			break;
		}
	case VPU_IOC_WAIT4INT:
		{
			u_long timeout = (u_long) arg;
			if (!wait_event_interruptible_timeout
			    (vpu_queue, irq_status != 0,
			     msecs_to_jiffies(timeout))) {
				printk(KERN_WARNING "VPU blocking: timeout.\n");
				ret = -ETIME;
			} else if (signal_pending(current)) {
				printk(KERN_WARNING
				       "VPU interrupt received.\n");
				ret = -ERESTARTSYS;
			} else
				irq_status = 0;
			break;
		}
	case VPU_IOC_IRAM_SETTING:
		{
			ret = copy_to_user((void __user *)arg, &iram,
					   sizeof(struct iram_setting));
			if (ret)
				ret = -EFAULT;

			break;
		}
	case VPU_IOC_CLKGATE_SETTING:
		{
			u32 clkgate_en;

			if (get_user(clkgate_en, (u32 __user *) arg))
				return -EFAULT;

			if (clkgate_en) {
				clk_enable(vpu_clk);
			} else {
				clk_disable(vpu_clk);
			}

			break;
		}
	case VPU_IOC_GET_SHARE_MEM:
		{
			mutex_lock(&vpu_data.lock);
			if (share_mem.cpu_addr != 0) {
				ret = copy_to_user((void __user *)arg,
						   &share_mem,
						   sizeof(struct vpu_mem_desc));
				mutex_unlock(&vpu_data.lock);
				break;
			} else {
				if (copy_from_user(&share_mem,
						   (struct vpu_mem_desc *)arg,
						 sizeof(struct vpu_mem_desc))) {
					mutex_unlock(&vpu_data.lock);
					return -EFAULT;
				}
				if (vpu_alloc_dma_buffer(&share_mem) == -1)
					ret = -EFAULT;
				else {
					if (copy_to_user((void __user *)arg,
							 &share_mem,
							 sizeof(struct
								vpu_mem_desc)))
						ret = -EFAULT;
				}
			}
			mutex_unlock(&vpu_data.lock);
			break;
		}
	case VPU_IOC_REQ_VSHARE_MEM:
		{
			mutex_lock(&vpu_data.lock);
			if (vshare_mem.cpu_addr != 0) {
				ret = copy_to_user((void __user *)arg,
						   &vshare_mem,
						   sizeof(struct vpu_mem_desc));
				mutex_unlock(&vpu_data.lock);
				break;
			} else {
				if (copy_from_user(&vshare_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct
							  vpu_mem_desc))) {
					mutex_unlock(&vpu_data.lock);
					return -EFAULT;
				}
				/* vmalloc shared memory if not allocated */
				if (!vshare_mem.cpu_addr)
					vshare_mem.cpu_addr =
					    (unsigned long)
					    vmalloc_user(vshare_mem.size);
				if (copy_to_user
				     ((void __user *)arg, &vshare_mem,
				     sizeof(struct vpu_mem_desc)))
					ret = -EFAULT;
			}
			mutex_unlock(&vpu_data.lock);
			break;
		}
	case VPU_IOC_GET_WORK_ADDR:
		{
			if (bitwork_mem.cpu_addr != 0) {
				ret =
				    copy_to_user((void __user *)arg,
						 &bitwork_mem,
						 sizeof(struct vpu_mem_desc));
				break;
			} else {
				if (copy_from_user(&bitwork_mem,
						   (struct vpu_mem_desc *)arg,
						   sizeof(struct vpu_mem_desc)))
					return -EFAULT;

				if (vpu_alloc_dma_buffer(&bitwork_mem) == -1)
					ret = -EFAULT;
				else if (copy_to_user((void __user *)arg,
						      &bitwork_mem,
						      sizeof(struct
							     vpu_mem_desc)))
					ret = -EFAULT;
			}
			break;
		}
	/*
	 * The following two ioctl is used when user allocates working buffer
	 * and register it to vpu driver.
	 */
	case VPU_IOC_QUERY_BITWORK_MEM:
		{
			if (copy_to_user((void __user *)arg,
					 &bitwork_mem,
					 sizeof(struct vpu_mem_desc)))
				ret = -EFAULT;
			break;
		}
	case VPU_IOC_SET_BITWORK_MEM:
		{
			if (copy_from_user(&bitwork_mem,
					   (struct vpu_mem_desc *)arg,
					   sizeof(struct vpu_mem_desc)))
				ret = -EFAULT;
			break;
		}
	case VPU_IOC_SYS_SW_RESET:
		{
			if (vpu_plat->reset)
				vpu_plat->reset();

			break;
		}
	case VPU_IOC_REG_DUMP:
		break;
	case VPU_IOC_PHYMEM_DUMP:
		break;
	case VPU_IOC_PHYMEM_CHECK:
	{
		struct vpu_mem_desc check_memory;
		ret = copy_from_user(&check_memory,
				     (void __user *)arg,
				     sizeof(struct vpu_mem_desc));
		if (ret != 0) {
			printk(KERN_ERR "copy from user failure:%d\n", ret);
			ret = -EFAULT;
			break;
		}
		ret = vpu_is_valid_phy_memory((u32)check_memory.phy_addr);

		pr_debug("vpu: memory phy:0x%x %s phy memory\n",
		       check_memory.phy_addr, (ret ? "is" : "isn't"));
		/* borrow .size to pass back the result. */
		check_memory.size = ret;
		ret = copy_to_user((void __user *)arg, &check_memory,
				   sizeof(struct vpu_mem_desc));
		if (ret) {
			ret = -EFAULT;
			break;
		}
		break;
	}
        case VPU_IOC_RESET_WARM_REBOOT:
        {
            /* Disable SRC warm reset to work around system reboot issue */
            u32 reg = __raw_readl(IO_ADDRESS(SRC_BASE_ADDR));
            reg &= ~0x1;
            __raw_writel(reg, IO_ADDRESS(SRC_BASE_ADDR));
             break;
        }
	default:
		{
			printk(KERN_ERR "No such IOCTL, cmd is %d\n", cmd);
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}

/*!
 * @brief Release function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_release(struct inode *inode, struct file *filp)
{
	int i;
	unsigned long timeout;
        struct vpu_mem_desc mem;
         struct memalloc_record *rec, *n;

        printk(KERN_INFO "-----MX VPU RELEASE -----\n");
	mutex_lock(&vpu_data.lock);
        
        printk(KERN_INFO "[VPU DMA BUFS INFO]\n");

        list_for_each_entry_safe(rec, n, &head, list) {
                mem = rec->mem;
                printk(KERN_INFO " [buf: phy: %X, cpu: %X, flags: %X, size: %d]\n",
                  mem.phy_addr, mem.cpu_addr, mem.flags, mem.size);
        }


	if (open_count > 0 && !(--open_count)) {

		/* Wait for vpu go to idle state */
		clk_enable(vpu_clk);
		if (READ_REG(BIT_CUR_PC)) {

			timeout = jiffies + HZ;
			while (READ_REG(BIT_BUSY_FLAG)) {
				msleep(1);
				if (time_after(jiffies, timeout)) {
					printk(KERN_WARNING "VPU timeout during release\n");
					break;
				}
			}
			clk_disable(vpu_clk);

			/* Clean up interrupt */
			cancel_work_sync(&vpu_data.work);
			flush_workqueue(vpu_data.workqueue);
			irq_status = 0;

			clk_enable(vpu_clk);
			if (READ_REG(BIT_BUSY_FLAG)) {

				if (cpu_is_mx51() || cpu_is_mx53()) {
					printk(KERN_ERR
						"fatal error: can't gate/power off when VPU is busy\n");
					clk_disable(vpu_clk);
					mutex_unlock(&vpu_data.lock);
					return -EFAULT;
				}

#ifdef CONFIG_SOC_IMX6Q
				if (cpu_is_mx6dl() || cpu_is_mx6q()) {
					WRITE_REG(0x11, 0x10F0);
					timeout = jiffies + HZ;
					while (READ_REG(0x10F4) != 0x77) {
						msleep(1);
						if (time_after(jiffies, timeout))
							break;
					}

					if (READ_REG(0x10F4) != 0x77) {
						printk(KERN_ERR
							"fatal error: can't gate/power off when VPU is busy\n");
						WRITE_REG(0x0, 0x10F0);
						clk_disable(vpu_clk);
						mutex_unlock(&vpu_data.lock);
						return -EFAULT;
					} else {
						if (vpu_plat->reset)
							vpu_plat->reset();
					}
				}
#endif
			}
		}
		clk_disable(vpu_clk);

		vpu_free_buffers();

		/* Free shared memory when vpu device is idle */
		vpu_free_dma_buffer(&share_mem);
		share_mem.cpu_addr = 0;
		vfree((void *)vshare_mem.cpu_addr);
		vshare_mem.cpu_addr = 0;

		vpu_clk_usercount = clk_get_usecount(vpu_clk);
		for (i = 0; i < vpu_clk_usercount; i++)
			clk_disable(vpu_clk);

		if (!IS_ERR(vpu_regulator))
			regulator_disable(vpu_regulator);

	}
	mutex_unlock(&vpu_data.lock);

	return 0;
}

/*!
 * @brief fasync function for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_fasync(int fd, struct file *filp, int mode)
{
	struct vpu_priv *dev = (struct vpu_priv *)filp->private_data;
	return fasync_helper(fd, filp, mode, &dev->async_queue);
}

/*!
 * @brief memory map function of harware registers for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_hwregs(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long pfn;

	vm->vm_flags |= VM_IO | VM_RESERVED;
	/*
	 * Since vpu registers have been mapped with ioremap() at probe
	 * which L_PTE_XN is 1, and the same physical address must be
	 * mapped multiple times with same type, so set L_PTE_XN to 1 here.
	 * Otherwise, there may be unexpected result in video codec.
	 */
	vm->vm_page_prot = pgprot_noncachedxn(vm->vm_page_prot);
	pfn = phy_vpu_base_addr >> PAGE_SHIFT;
	pr_debug("size=0x%x,  page no.=0x%x\n",
		 (int)(vm->vm_end - vm->vm_start), (int)pfn);
	return remap_pfn_range(vm, vm->vm_start, pfn, vm->vm_end - vm->vm_start,
			       vm->vm_page_prot) ? -EAGAIN : 0;
}

/*!
 * @brief memory map function of memory for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_map_dma_mem(struct file *fp, struct vm_area_struct *vm)
{
	int request_size;
	request_size = vm->vm_end - vm->vm_start;

	pr_debug(" start=0x%x, pgoff=0x%x, size=0x%x\n",
		 (unsigned int)(vm->vm_start), (unsigned int)(vm->vm_pgoff),
		 request_size);

	vm->vm_flags |= VM_IO | VM_RESERVED;
	vm->vm_page_prot = pgprot_writecombine(vm->vm_page_prot);

	return remap_pfn_range(vm, vm->vm_start, vm->vm_pgoff,
			       request_size, vm->vm_page_prot) ? -EAGAIN : 0;

}

/* !
 * @brief memory map function of vmalloced share memory
 * @return  0 on success or negative error code on error
 */
static int vpu_map_vshare_mem(struct file *fp, struct vm_area_struct *vm)
{
	int ret = -EINVAL;

	ret = remap_vmalloc_range(vm, (void *)(vm->vm_pgoff << PAGE_SHIFT), 0);
	vm->vm_flags |= VM_IO;

	return ret;
}
/*!
 * @brief memory map interface for vpu file operation
 * @return  0 on success or negative error code on error
 */
static int vpu_mmap(struct file *fp, struct vm_area_struct *vm)
{
	unsigned long offset;

	offset = vshare_mem.cpu_addr >> PAGE_SHIFT;

	if (vm->vm_pgoff && (vm->vm_pgoff == offset))
		return vpu_map_vshare_mem(fp, vm);
	else if (vm->vm_pgoff)
		return vpu_map_dma_mem(fp, vm);
	else
		return vpu_map_hwregs(fp, vm);
}

struct file_operations vpu_fops = {
	.owner = THIS_MODULE,
	.open = vpu_open,
	.unlocked_ioctl = vpu_ioctl,
	.release = vpu_release,
	.fasync = vpu_fasync,
	.mmap = vpu_mmap,
};

/*!
 * This function is called by the driver framework to initialize the vpu device.
 * @param   dev The device structure for the vpu passed in by the framework.
 * @return   0 on success or negative error code on error
 */
static int vpu_dev_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *temp_class;
	struct resource *res;
	unsigned long addr = 0;

	vpu_plat = pdev->dev.platform_data;


	if (vpu_plat && vpu_plat->iram_enable && vpu_plat->iram_size)
		iram_alloc(vpu_plat->iram_size, &addr);
	printk(KERN_INFO "vpu: IRAM allocation: ADDR: %lX, SIZE: %d\n",addr,vpu_plat->iram_size);
	if (addr == 0)
		iram.start = iram.end = 0;
	else {
		iram.start = addr;
		iram.end = addr +  vpu_plat->iram_size - 1;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vpu_regs");
	if (!res) {
		printk(KERN_ERR "vpu: unable to get vpu base addr\n");
		return -ENODEV;
	}
	phy_vpu_base_addr = res->start;
	vpu_base = ioremap(res->start, res->end - res->start);

	vpu_major = register_chrdev(MXC_VPU_MAJOR, "mxc_vpu", &vpu_fops);
	if (vpu_major < 0) {
		printk(KERN_ERR "vpu: unable to get a major for VPU\n");
		err = -EBUSY;
		goto error;
	}
	vpu_major = MXC_VPU_MAJOR;
	vpu_class = class_create(THIS_MODULE, "mxc_vpu");
	if (IS_ERR(vpu_class)) {
		err = PTR_ERR(vpu_class);
		goto err_out_chrdev;
	}

	temp_class = device_create(vpu_class, NULL, MKDEV(vpu_major, 0),
				   NULL, "mxc_vpu");
	if (IS_ERR(temp_class)) {
		err = PTR_ERR(temp_class);
		goto err_out_class;
	}

	vpu_clk = clk_get(&pdev->dev, "vpu_clk");
	if (IS_ERR(vpu_clk)) {
		err = -ENOENT;
		goto err_out_class;
	}

	vpu_ipi_irq = platform_get_irq_byname(pdev, "vpu_ipi_irq");
	if (vpu_ipi_irq < 0) {
		printk(KERN_ERR "vpu: unable to get vpu interrupt\n");
		err = -ENXIO;
		goto err_out_class;
	}
	err = request_irq(vpu_ipi_irq, vpu_ipi_irq_handler, 0, "VPU_CODEC_IRQ",
			  (void *)(&vpu_data));
	if (err)
		goto err_out_class;
	vpu_regulator = regulator_get(NULL, "cpu_vddvpu");
	if (IS_ERR(vpu_regulator)) {
		if (!(cpu_is_mx51() || cpu_is_mx53())) {
			printk(KERN_ERR
				"%s: failed to get vpu regulator\n", __func__);
			goto err_out_class;
		} else {
			/* regulator_get will return error on MX5x,
			 * just igore it everywhere*/
			printk(KERN_WARNING
				"%s: failed to get vpu regulator\n", __func__);
		}
	}

#ifdef MXC_VPU_HAS_JPU
	vpu_jpu_irq = platform_get_irq_byname(pdev, "vpu_jpu_irq");
	if (vpu_jpu_irq < 0) {
		printk(KERN_ERR "vpu: unable to get vpu jpu interrupt\n");
		err = -ENXIO;
		free_irq(vpu_ipi_irq, &vpu_data);
		goto err_out_class;
	}
	err = request_irq(vpu_jpu_irq, vpu_jpu_irq_handler, IRQF_TRIGGER_RISING,
			  "VPU_JPG_IRQ", (void *)(&vpu_data));
	if (err) {
		free_irq(vpu_ipi_irq, &vpu_data);
		goto err_out_class;
	}
#endif

	vpu_data.workqueue = create_workqueue("vpu_wq");
	INIT_WORK(&vpu_data.work, vpu_worker_callback);
	mutex_init(&vpu_data.lock);

	printk(KERN_INFO "VPU initialized\n");
	goto out;

      err_out_class:
	device_destroy(vpu_class, MKDEV(vpu_major, 0));
	class_destroy(vpu_class);
      err_out_chrdev:
	unregister_chrdev(vpu_major, "mxc_vpu");
      error:
	iounmap(vpu_base);
      out:
	return err;
}

static int vpu_dev_remove(struct platform_device *pdev)
{
	free_irq(vpu_ipi_irq, &vpu_data);
#ifdef MXC_VPU_HAS_JPU
	free_irq(vpu_jpu_irq, &vpu_data);
#endif
	cancel_work_sync(&vpu_data.work);
	flush_workqueue(vpu_data.workqueue);
	destroy_workqueue(vpu_data.workqueue);

	iounmap(vpu_base);
	if (vpu_plat && vpu_plat->iram_enable && vpu_plat->iram_size)
		iram_free(iram.start,  vpu_plat->iram_size);
	if (!IS_ERR(vpu_regulator))
		regulator_put(vpu_regulator);
	return 0;
}

#ifdef CONFIG_PM
static int vpu_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i;
	unsigned long timeout;

	mutex_lock(&vpu_data.lock);
	if (open_count == 0) {
		/* VPU is released (all instances are freed),
		 * clock is already off, context is no longer needed,
		 * power is already off on MX6,
		 * gate power on MX51 */
		if (cpu_is_mx51()) {
			if (vpu_plat->pg)
				vpu_plat->pg(1);
		}
	} else {
		/* Wait for vpu go to idle state, suspect vpu cannot be changed
		   to idle state after about 1 sec */
		timeout = jiffies + HZ;
		clk_enable(vpu_clk);
		while (READ_REG(BIT_BUSY_FLAG)) {
			msleep(1);
			if (time_after(jiffies, timeout)) {
				clk_disable(vpu_clk);
				mutex_unlock(&vpu_data.lock);
				return -EAGAIN;
			}
		}
		clk_disable(vpu_clk);

		/* Make sure clock is disabled before suspend */
		vpu_clk_usercount = clk_get_usecount(vpu_clk);
		for (i = 0; i < vpu_clk_usercount; i++)
			clk_disable(vpu_clk);

		if (cpu_is_mx53()) {
			mutex_unlock(&vpu_data.lock);
			return 0;
		}

		if (bitwork_mem.cpu_addr != 0) {
			clk_enable(vpu_clk);
			/* Save 64 registers from BIT_CODE_BUF_ADDR */
			for (i = 0; i < 64; i++)
				regBk[i] = READ_REG(BIT_CODE_BUF_ADDR + (i * 4));
			pc_before_suspend = READ_REG(BIT_CUR_PC);
			clk_disable(vpu_clk);
		}

		if (vpu_plat->pg)
			vpu_plat->pg(1);

		/* If VPU is working before suspend, disable
		 * regulator to make usecount right. */
		if (!IS_ERR(vpu_regulator))
			regulator_disable(vpu_regulator);
	}

	mutex_unlock(&vpu_data.lock);
	return 0;
}

static int vpu_resume(struct platform_device *pdev)
{
	int i;

	mutex_lock(&vpu_data.lock);
	if (open_count == 0) {
		/* VPU is released (all instances are freed),
		 * clock should be kept off, context is no longer needed,
		 * power should be kept off on MX6,
		 * disable power gating on MX51 */
		if (cpu_is_mx51()) {
			if (vpu_plat->pg)
				vpu_plat->pg(0);
		}
	} else {
		if (cpu_is_mx53())
			goto recover_clk;

		/* If VPU is working before suspend, enable
		 * regulator to make usecount right. */
		if (!IS_ERR(vpu_regulator))
			regulator_enable(vpu_regulator);

		if (vpu_plat->pg)
			vpu_plat->pg(0);

		if (bitwork_mem.cpu_addr != 0) {
			u32 *p = (u32 *) bitwork_mem.cpu_addr;
			u32 data, pc;
			u16 data_hi;
			u16 data_lo;

			clk_enable(vpu_clk);

			pc = READ_REG(BIT_CUR_PC);
			if (pc) {
				printk(KERN_WARNING "Not power off after suspend (PC=0x%x)\n", pc);
				clk_disable(vpu_clk);
				goto recover_clk;
			}

			/* Restore registers */
			for (i = 0; i < 64; i++)
				WRITE_REG(regBk[i], BIT_CODE_BUF_ADDR + (i * 4));

			WRITE_REG(0x0, BIT_RESET_CTRL);
			WRITE_REG(0x0, BIT_CODE_RUN);
			/* MX6 RTL has a bug not to init MBC_SET_SUBBLK_EN on reset */
#ifdef CONFIG_SOC_IMX6Q
			WRITE_REG(0x0, MBC_SET_SUBBLK_EN);
#endif

			/*
			 * Re-load boot code, from the codebuffer in external RAM.
			 * Thankfully, we only need 4096 bytes, same for all platforms.
			 */
			for (i = 0; i < 2048; i += 4) {
				data = p[(i >> 1) + 1];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG((i << 16) | data_hi, BIT_CODE_DOWN);
				WRITE_REG(((i + 1) << 16) | data_lo,
						BIT_CODE_DOWN);

				data = p[(i >> 1)];
				data_hi = (data >> 16) & 0xFFFF;
				data_lo = data & 0xFFFF;
				WRITE_REG(((i + 2) << 16) | data_hi,
						BIT_CODE_DOWN);
				WRITE_REG(((i + 3) << 16) | data_lo,
						BIT_CODE_DOWN);
			}

			if (pc_before_suspend) {
				WRITE_REG(0x1, BIT_BUSY_FLAG);
				WRITE_REG(0x1, BIT_CODE_RUN);
				while (READ_REG(BIT_BUSY_FLAG))
					;
			} else {
				printk(KERN_WARNING "PC=0 before suspend\n");
			}
			clk_disable(vpu_clk);
		}

recover_clk:
		/* Recover vpu clock */
		for (i = 0; i < vpu_clk_usercount; i++)
			clk_enable(vpu_clk);
	}

	mutex_unlock(&vpu_data.lock);
	return 0;
}
#else
#define	vpu_suspend	NULL
#define	vpu_resume	NULL
#endif				/* !CONFIG_PM */

/*! Driver definition
 *
 */
static struct platform_driver mxcvpu_driver = {
	.driver = {
		   .name = "mxc_vpu",
		   },
	.probe = vpu_dev_probe,
	.remove = vpu_dev_remove,
	.suspend = vpu_suspend,
	.resume = vpu_resume,
};

static int __init vpu_init(void)
{
        int ret = 0;

    parallax_vpu_fill_dma_buffers();

	ret = platform_driver_register(&mxcvpu_driver);

	init_waitqueue_head(&vpu_queue);


	memblock_analyze();
	top_address_DRAM = memblock_end_of_DRAM_with_reserved();

	return ret;
}

static void __exit vpu_exit(void)
{
	if (vpu_major > 0) {
		device_destroy(vpu_class, MKDEV(vpu_major, 0));
		class_destroy(vpu_class);
		unregister_chrdev(vpu_major, "mxc_vpu");
		vpu_major = 0;
	}

	vpu_free_dma_buffer(&bitwork_mem);
	vpu_free_dma_buffer(&pic_para_mem);
	vpu_free_dma_buffer(&user_data_mem);

	clk_put(vpu_clk);

	platform_driver_unregister(&mxcvpu_driver);
	return;
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Linux VPU driver for Freescale i.MX/MXC");
MODULE_LICENSE("GPL");

module_init(vpu_init);
module_exit(vpu_exit);

