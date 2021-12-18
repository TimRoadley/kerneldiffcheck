/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @defgroup Framebuffer Framebuffer Driver for surface sharing.
 */

/*!
 * @file virtual_fb.c 
 *
 * @brief Virtual Frame buffer driver for surface sharing
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <asm/mach-types.h>
#include <linux/parallax.h>

/*
 * Driver name
 */
#define VIRT_FB_NAME      "virtual_fb"

static int vfbcount = 6;
module_param(vfbcount, int, 0); 
static struct fb_info ** g_fb_list;

static int virtfb_map_video_memory(struct fb_info *fbi);
static int virtfb_unmap_video_memory(struct fb_info *fbi);

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int virtfb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ywrapstep = 1;
	fix->ypanstep = 1;

	return 0;
}


/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int virtfb_set_par(struct fb_info *fbi)
{
	int retval = 0;
	u32 mem_len;

	dev_dbg(fbi->device, "Reconfiguring framebuffer\n");

	virtfb_set_fix(fbi);

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			virtfb_unmap_video_memory(fbi);

		if (virtfb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}


	return retval;
}


/*
 * Check framebuffer variable parameters and adjust to valid values.
 *
 * @param       var      framebuffer variable parameters
 *
 * @param       info     framebuffer information pointer
 */
static int virtfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{

	/* fg should not bigger than bg */
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 12) &&
	    (var->bits_per_pixel != 8))
		var->bits_per_pixel = 16;


	printk(KERN_INFO "Virtual Buffer: BPP: %d\n", var->bits_per_pixel);

	switch (var->bits_per_pixel) {
	case 8:
		var->red.length = 3;
		var->red.offset = 5;
		var->red.msb_right = 0;

		var->green.length = 3;
		var->green.offset = 2;
		var->green.msb_right = 0;

		var->blue.length = 2;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 24;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 16;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 8;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 *
 * @param               var     Variable screen buffer information
 * @param               info    Framebuffer information pointer
 */
static int
virtfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (info->var.yoffset == var->yoffset)
		return 0;	/* No change, do nothing */

	if ((var->yoffset + info->var.yres) > info->var.yres_virtual)
		return -EINVAL;

	info->var.yoffset = var->yoffset;

	return 0;
}

/*
 * Function to handle custom mmap for virtual framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       vma     Pointer to vm_area_struct
 */
static int virtfb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory */
		len = fbi->fix.smem_len - offset;
		vma->vm_pgoff = (fbi->fix.smem_start + offset) >> PAGE_SHIFT;
	} else {
		return -EINVAL;
	}

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(fbi->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core framebuffer driver to perform operations like
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops virtfb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = virtfb_set_par,
	.fb_check_var = virtfb_check_var,
	.fb_pan_display = virtfb_pan_display,
	.fb_mmap = virtfb_mmap,
};


/*
 * Main framebuffer functions
 */

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int virtfb_map_video_memory(struct fb_info *fbi)
{
	/* Extract and check ID Should be 0 - 3 range */
	unsigned int dev_offset = 0;
	unsigned int ipu_offset = M3_VPU_BUFFER_IC_ID*PARALLAX_DMA_BUF_SIZE;
	if(fbi->fix.fbid > 5) {
            printk(KERN_INFO "VFB: ERROR: VFB Device ID: %d is not supported\n", fbi->fix.fbid);
            dev_err(fbi->device, "Unable to map device ID: %d. Not supported\n", fbi->fix.fbid);
            fbi->fix.smem_len = 0;
            fbi->fix.smem_start = 0;
            return -ENXIO;
	}
	/* Calculate offset to DMA area for IPU operations */
	dev_offset = (fbi->fix.fbid >= 3)?1:0;
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;
	/* 
	*  Calculate DMA memory offset for frame buffer operations: 
	*  DMA start + Camera DMA zone + IPU IC buffer offset 
	*/
	fbi->fix.smem_start = PARALLAX_DMA_MEM_START + (dev_offset*((M3_VPU_BUFFER_IC_ID+1)*PARALLAX_DMA_BUF_SIZE)) +
				ipu_offset;
        printk(KERN_INFO "Virt Buffer Map: %X for dev offset: %d, FBID: %d!\n", fbi->fix.smem_start, dev_offset, fbi->fix.fbid);
	fbi->screen_base = ioremap(fbi->fix.smem_start,
                                  fbi->fix.smem_len);

#if 0
	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA);
#endif
	if (fbi->screen_base == 0) {
		printk(KERN_INFO "VFB: ERROR: DMA memory for Image Conversion map failed\n");
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}
        else {
		printk(KERN_INFO "VFB: DMA memory for Image Conversion remmapped OK\n");
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);
	printk(KERN_INFO "Virtual FB: Allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int virtfb_unmap_video_memory(struct fb_info *fbi)
{
#if 0
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
#endif
	iounmap(fbi->screen_base);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info *virtfb_init_fbinfo(struct fb_ops *ops)
{
	struct fb_info *fbi;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	fbi = framebuffer_alloc(sizeof(unsigned int), NULL);
	if (!fbi)
		return NULL;


	fbi->var.activate = FB_ACTIVATE_NOW;

	fbi->fbops = ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;


	return fbi;
}


static int virtfb_register(struct fb_info *fbi, unsigned int id, 
                           unsigned int width, unsigned int height, unsigned int depth)
{
	struct fb_videomode m;
	int ret = 0;

	//TODO: Set framebuffer ID
	sprintf(fbi->fix.id, "virt_fb%d", id);
	fbi->fix.fbid = (__u16)id;
	//Setup small default resolution
//	fbi->var.xres_virtual = fbi->var.xres = fbi->var.yres_virtual = fbi->var.yres  = 128;
//	fbi->var.bits_per_pixel = 16;
	fbi->var.xres_virtual = fbi->var.xres = width;
    	fbi->var.yres_virtual = fbi->var.yres  = height;
	fbi->var.bits_per_pixel = depth;

	virtfb_check_var(&fbi->var, fbi);

	virtfb_set_fix(fbi);

	/*added first mode to fbi modelist*/
	if (!fbi->modelist.next || !fbi->modelist.prev)
		INIT_LIST_HEAD(&fbi->modelist);
	fb_var_to_videomode(&m, &fbi->var);
	fb_add_videomode(&m, &fbi->modelist);

	fbi->var.activate |= FB_ACTIVATE_FORCE;
	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &fbi->var);
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();


	ret = register_framebuffer(fbi);
	if (ret < 0)
		goto err0;

	return ret;
err0:
	return ret;
}

static void virtfb_unregister(struct fb_info *fbi)
{

	unregister_framebuffer(fbi);
}

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MXCFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
int __init virtfb_init(void)
{
	
        u32 *  fbNum;
        int  i, ret = 0;

        /*
         * Initialize FB structures
         */

	g_fb_list = kzalloc(sizeof(struct fb_info*) * vfbcount, GFP_KERNEL);
	/* Allocate for ALL cameras - two types of images */
        /* Allocate for LPR images */
        g_fb_list[0] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[0]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[0]->par;
        *fbNum = 0;

        ret = virtfb_register(g_fb_list[0], 0, 2592, 1944, 32);
        if (ret < 0)
                goto virtfb_register_failed;

	/* Allocate for VGA detection */
        g_fb_list[1] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[1]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[1]->par;
        *fbNum = 1;

        ret = virtfb_register(g_fb_list[1], 1, 640, 480, 32);
        if (ret < 0)
                goto virtfb_register_failed;

        /* Allocate for Filter operations on detection/H264 frames */
        g_fb_list[2] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[2]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[2]->par;
        *fbNum = 2;

        ret = virtfb_register(g_fb_list[2], 2, 640, 480, 32);
        if (ret < 0)
                goto virtfb_register_failed;


        /* Camera 2 - Allocate for LPR images */
        g_fb_list[3] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[3]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[3]->par;
        *fbNum = 3;

        ret = virtfb_register(g_fb_list[3], 3, 2592, 1944, 32);
        if (ret < 0)
                goto virtfb_register_failed;

        /* Camera 2 - Allocate for VGA detection */
        g_fb_list[4] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[4]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[4]->par;
        *fbNum = 4;

        ret = virtfb_register(g_fb_list[4], 4, 640, 480, 32);
        if (ret < 0)
                goto virtfb_register_failed;

        /* Camera 2 - Allocate for Filter for detection/H264 frames */
        g_fb_list[5] = virtfb_init_fbinfo(&virtfb_ops);
        if (!g_fb_list[5]) {
                ret = -ENOMEM;
                goto init_fbinfo_failed;
        }
        fbNum = (u32*)g_fb_list[5]->par;
        *fbNum = 5;

        ret = virtfb_register(g_fb_list[5], 5, 640, 480, 32);
        if (ret < 0)
                goto virtfb_register_failed;


#if 0
        for(i=0;i<vfbcount;i++)
        {
                g_fb_list[i] = virtfb_init_fbinfo(&virtfb_ops);
                if (!g_fb_list[i]) {
                        ret = -ENOMEM;
                        goto init_fbinfo_failed;
                }

                fbNum = (u32*)g_fb_list[i]->par;
                *fbNum = i;

                ret = virtfb_register(g_fb_list[i], i);
                if (ret < 0)
                        goto virtfb_register_failed;
        }
#endif

        return 0;
virtfb_register_failed:
init_fbinfo_failed:
        for(i=0;i<vfbcount;i++)
	{
		if(g_fb_list[i])
		{
			virtfb_unregister(g_fb_list[i]);
        		framebuffer_release(g_fb_list[i]);
		}
	}
        return ret;

}

void virtfb_exit(void)
{

	int i;

        for(i=0;i<vfbcount;i++)
	{
		if(g_fb_list[i])
		{
			virtfb_unregister(g_fb_list[i]);
			virtfb_unmap_video_memory(g_fb_list[i]);

			framebuffer_release(g_fb_list[i]);
		}
	}


}

module_init(virtfb_init);
module_exit(virtfb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Virtual framebuffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");