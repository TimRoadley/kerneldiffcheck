/*
 * Copyright 2009-2012 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file ipu_csi0_enc.c
 *
 * @brief CSI0 Use case for video capture
 *
 * @ingroup IPU
 */
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/ipu.h>
#include <linux/parallax.h>
#include <mach/mipi_csi2.h>
#include "mxc_v4l2_capture.h"
#include "ipu_prp_sw.h"

#ifdef CAMERA_DBG
	#define CAMERA_TRACE(x) printk(x)
#else
	#define CAMERA_TRACE(x)
#endif

/*
 * Function definitions
 */

/*!
 * csi ENC callback function.
 *
 * @param irq       int irq line
 * @param dev_id    void * device id
 *
 * @return status   IRQ_HANDLED for handled
 */
static irqreturn_t csi_enc_callback(int irq, void *dev_id)
{
	cam_data *cam = (cam_data *) dev_id;

	if (cam->enc_callback == NULL)
		return IRQ_HANDLED;

	cam->enc_callback(irq, dev_id);
	return IRQ_HANDLED;
}

/*!
 * CSI ENC enable channel setup function
 *
 * @param cam       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_setup(cam_data *cam)
{
	ipu_channel_params_t params;
	u32 pixel_fmt;
	int err = 0, sensor_protocol = 0;
	dma_addr_t dummy = cam->dummy_frame.buffer.m.offset;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	if (!cam) {
		printk(KERN_ERR "cam private is NULL\n");
		return -ENXIO;
	}

	memset(&params, 0, sizeof(ipu_channel_params_t));
	params.csi_mem.csi = cam->csi;

	sensor_protocol = ipu_csi_get_sensor_protocol(cam->ipu, cam->csi);
	switch (sensor_protocol) {
	case IPU_CSI_CLK_MODE_GATED_CLK:
	case IPU_CSI_CLK_MODE_NONGATED_CLK:
	case IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR:
		params.csi_mem.interlaced = false;
		break;
	case IPU_CSI_CLK_MODE_CCIR656_INTERLACED:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR:
		params.csi_mem.interlaced = true;
		break;
	default:
		printk("CSI0 ENC: ERROR: invalid sensor protocol\n");
		return -EINVAL;
	}

	if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420)
		pixel_fmt = IPU_PIX_FMT_YUV420P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YVU420)
		pixel_fmt = IPU_PIX_FMT_YVU420P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUV422P)
		pixel_fmt = IPU_PIX_FMT_YUV422P;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY)
		pixel_fmt = IPU_PIX_FMT_UYVY;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
		pixel_fmt = IPU_PIX_FMT_YUYV;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_NV12)
		pixel_fmt = IPU_PIX_FMT_NV12;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR24)
		pixel_fmt = IPU_PIX_FMT_BGR24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
		pixel_fmt = IPU_PIX_FMT_RGB24;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565)
		pixel_fmt = IPU_PIX_FMT_RGB565;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_BGR32)
		pixel_fmt = IPU_PIX_FMT_BGR32;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB32)
		pixel_fmt = IPU_PIX_FMT_RGB32;
	else if (cam->v2f.fmt.pix.pixelformat == V4L2_PIX_FMT_SBGGR10) {
		pixel_fmt = IPU_PIX_FMT_GENERIC;
		printk("++++CSI ENC SETUP: FORMAT SBGGR10\n");
	}
	else {
		printk("CSI0 ENC: ERROR: invalid pixel format\n");
		return -EINVAL;
	}

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id) {
				params.csi_mem.mipi_en = true;
				params.csi_mem.mipi_vc =
				mipi_csi2_get_virtual_channel(mipi_csi2_info);
				params.csi_mem.mipi_id =
				mipi_csi2_get_datatype(mipi_csi2_info);

				mipi_csi2_pixelclk_enable(mipi_csi2_info);
			} else {
				params.csi_mem.mipi_en = false;
				params.csi_mem.mipi_vc = 0;
				params.csi_mem.mipi_id = 0;
			}
		} else {
			params.csi_mem.mipi_en = false;
			params.csi_mem.mipi_vc = 0;
			params.csi_mem.mipi_id = 0;
		}
	} else {
		return -EPERM;
	}
#endif

	err = ipu_init_channel(cam->ipu, CSI_MEM0, &params);
	if (err != 0) {
		return err;
	}

        printk("++++CSI ENC SETUP: pixel_fmt: %X, bpl: %d, width: %d\n", 
		pixel_fmt, cam->v2f.fmt.pix.bytesperline, cam->v2f.fmt.pix.width);

	err = ipu_init_channel_buffer(cam->ipu, CSI_MEM0, IPU_OUTPUT_BUFFER,
				      pixel_fmt, cam->v2f.fmt.pix.width,
				      cam->v2f.fmt.pix.height,
				      cam->v2f.fmt.pix.bytesperline,
				      IPU_ROTATE_NONE,
				      dummy, dummy, 0,
				      cam->offset.u_offset,
				      cam->offset.v_offset);
	if (err != 0) {
		return err;
	}
	err = ipu_enable_channel(cam->ipu, CSI_MEM0);
	if (err < 0) {
		return err;
	}

	return err;
}

/*!
 * function to update physical buffer address for encorder IDMA channel
 *
 * @param eba         physical buffer address for encorder IDMA channel
 * @param buffer_num  int buffer 0 or buffer 1
 *
 * @return  status
 */
static int csi_enc_eba_update(struct ipu_soc *ipu, dma_addr_t eba, int *buffer_num)
{
	int err = 0;

	pr_debug("eba %x\n", eba);
	err = ipu_update_channel_buffer(ipu, CSI_MEM0, IPU_OUTPUT_BUFFER,
					*buffer_num, eba);
	if (err != 0) {
		ipu_clear_buffer_ready(ipu, CSI_MEM0, IPU_OUTPUT_BUFFER,
				       *buffer_num);

		err = ipu_update_channel_buffer(ipu, CSI_MEM0, IPU_OUTPUT_BUFFER,
						*buffer_num, eba);
		if (err != 0) {
			pr_err("ERROR: v4l2 capture: fail to update "
			       "buf%d\n", *buffer_num);
			return err;
		}
	}

	ipu_select_buffer(ipu, CSI_MEM0, IPU_OUTPUT_BUFFER, *buffer_num);

	*buffer_num = (*buffer_num == 0) ? 1 : 0;

	return 0;
}

/*!
 * Enable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
	CAMERA_TRACE("IPU:In csi_enc_enabling_tasks\n");

#if 0
	cam->dummy_frame.vaddress = dma_alloc_coherent(0,
			       PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage),
			       &cam->dummy_frame.paddress,
			       GFP_DMA | GFP_KERNEL);
	if (cam->dummy_frame.vaddress == 0) {
		printk("ERROR: CSI0 ENC: v4l2 capture: Allocate dummy frame "
		       "failed.\n");
		return -ENOBUFS;
	}
	cam->dummy_frame.buffer.type = V4L2_BUF_TYPE_PRIVATE;
	cam->dummy_frame.buffer.length =
	    PAGE_ALIGN(cam->v2f.fmt.pix.sizeimage);
	cam->dummy_frame.buffer.m.offset = cam->dummy_frame.paddress;
#endif
	/* Set static DMA address fir CSI IPU */
	cam->dummy_frame.paddress = PARALLAX_CSI_MEM_START+(PARALLAX_DMA_BUF_SIZE*cam->csi);
	cam->dummy_frame.vaddress = ioremap(cam->dummy_frame.paddress, PARALLAX_DMA_BUF_SIZE);
        if (cam->dummy_frame.vaddress == 0) {
                printk("ERROR: CSI0 ENC: v4l2 capture: Allocate dummy frame "
                       "failed.\n");
                return -ENOBUFS;
        }
        cam->dummy_frame.buffer.type = V4L2_BUF_TYPE_DMAMAP;
        cam->dummy_frame.buffer.length = PARALLAX_DMA_BUF_SIZE;
        cam->dummy_frame.buffer.m.offset = cam->dummy_frame.paddress;

	ipu_clear_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF);
	err = ipu_request_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF,
			      csi_enc_callback, 0, "Mxc Camera 0", cam);
	if (err != 0) {
		printk("CSI0 ENC: ERROR: ipu_request_irq\n");
		return err;
	}

	err = csi_enc_setup(cam);
	if (err != 0) {
		printk("CSI0 ENC: ERROR: csi_enc_setup\n");
		return err;
	}

	return err;
}

/*!
 * Disable encoder task
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
static int csi_enc_disabling_tasks(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;
#ifdef CONFIG_MXC_MIPI_CSI2
	void *mipi_csi2_info;
	int ipu_id;
	int csi_id;
#endif

	err = ipu_disable_channel(cam->ipu, CSI_MEM0, true);

	ipu_uninit_channel(cam->ipu, CSI_MEM0);

	if (cam->dummy_frame.vaddress != 0) {
		if(cam->dummy_frame.buffer.type == V4L2_BUF_TYPE_DMAMAP) {
			iounmap(cam->dummy_frame.vaddress);
		}
		else {
			dma_free_coherent(0, cam->dummy_frame.buffer.length,
					  cam->dummy_frame.vaddress,
					  cam->dummy_frame.paddress);
		}
		cam->dummy_frame.vaddress = 0;
	}

#ifdef CONFIG_MXC_MIPI_CSI2
	mipi_csi2_info = mipi_csi2_get_info();

	if (mipi_csi2_info) {
		if (mipi_csi2_get_status(mipi_csi2_info)) {
			ipu_id = mipi_csi2_get_bind_ipu(mipi_csi2_info);
			csi_id = mipi_csi2_get_bind_csi(mipi_csi2_info);

			if (cam->ipu == ipu_get_soc(ipu_id)
				&& cam->csi == csi_id)
				mipi_csi2_pixelclk_disable(mipi_csi2_info);
		}
	} else {
		return -EPERM;
	}
#endif

	return err;
}

/*!
 * Enable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_enable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	return ipu_enable_csi(cam->ipu, cam->csi);
}

/*!
 * Disable csi
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  status
 */
static int csi_enc_disable_csi(void *private)
{
	cam_data *cam = (cam_data *) private;

	/* free csi eof irq firstly.
	 * when disable csi, wait for idmac eof.
	 * it requests eof irq again */
	ipu_free_irq(cam->ipu, IPU_IRQ_CSI0_OUT_EOF, cam);

	return ipu_disable_csi(cam->ipu, cam->csi);
}

/*!
 * function to select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi0_enc_select(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = csi_enc_eba_update;
		cam->enc_enable = csi_enc_enabling_tasks;
		cam->enc_disable = csi_enc_disabling_tasks;
		cam->enc_enable_csi = csi_enc_enable_csi;
		cam->enc_disable_csi = csi_enc_disable_csi;
	} else {
		err = -EIO;
	}

	return err;
}

/*!
 * function to de-select CSI ENC as the working path
 *
 * @param private       struct cam_data * mxc capture instance
 *
 * @return  int
 */
int csi0_enc_deselect(void *private)
{
	cam_data *cam = (cam_data *) private;
	int err = 0;

	if (cam) {
		cam->enc_update_eba = NULL;
		cam->enc_enable = NULL;
		cam->enc_disable = NULL;
		cam->enc_enable_csi = NULL;
		cam->enc_disable_csi = NULL;
	}

	return err;
}

/*!
 * Init the Encorder channels
 *
 * @return  Error code indicating success or failure
 */
__init int csi0_enc_init(void)
{
	return 0;
}

/*!
 * Deinit the Encorder channels
 *
 */
void __exit csi0_enc_exit(void)
{
}

module_init(csi0_enc_init);
module_exit(csi0_enc_exit);

EXPORT_SYMBOL(csi0_enc_select);
EXPORT_SYMBOL(csi0_enc_deselect);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("CSI0 ENC Driver");
MODULE_LICENSE("GPL");
