/*
 * SRC basic driver to read/write SRC registers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/pasrc.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>

#define SRC_MAJOR 126


static int parallax_src_open(struct inode * inode, struct file * file)
{
        int ret = 0;
	return ret;
}

static int parallax_src_release(struct inode * inode, struct file * file)
{
	return 0;
}

static long parallax_src_ioctl(struct file *file, unsigned int cmd,
                        unsigned long arg)
{
        unsigned int ret = 0;
        struct pasrc_control src;

        switch (cmd) {
        case PASRC_IOCTL_READ:
              if (copy_from_user(&src, (struct pasrc_control __user *)arg,
                                   sizeof(struct pasrc_control))) {
                        return -EFAULT;
                }
                src.val = __raw_readl(IO_ADDRESS(SRC_BASE_ADDR+src.reg));

                if(copy_to_user((struct pasrc_control __user *)arg, &src,
                                        sizeof(struct pasrc_control)) != 0) {
                        return -EFAULT;
                } 
                break;
        case PASRC_IOCTL_WRITE:
             if (copy_from_user(&src, (struct pasrc_control __user *)arg,
                                   sizeof(struct pasrc_control))) {
                        return -EFAULT;
                }
	        __raw_writel(src.val, IO_ADDRESS(SRC_BASE_ADDR+src.reg));

                break;
        default:
		ret = -ENOTTY;
                break;
        }
        return ret;
}


static const struct file_operations parallax_src_fops = {
        .owner          = THIS_MODULE,
        .unlocked_ioctl = parallax_src_ioctl,
        .open           = parallax_src_open,
        .release        = parallax_src_release,
};


static int parallax_src_probe(struct platform_device *pdev)
{

	printk(KERN_INFO "PARALLAX SRC: Initializing SRC interface\n");

        /* platform_set_drvdata(pdev, src_data); */
	// Register as character device
        if (register_chrdev (SRC_MAJOR, "parallax_src", &parallax_src_fops)) {
                printk (KERN_ERR "parallax src: unable to get major %d\n", SRC_MAJOR);
                return -EIO;
        }
	printk(KERN_INFO "PARALLAX SRC: SRC interface initialized\n");
        return 0;
}

static int parallax_src_remove(struct platform_device *pdev)
{
	// Unregister device
	unregister_chrdev(SRC_MAJOR, "parallax_src");
	return 0;
}

static struct platform_driver parallax_src_driver = {
	.driver		= {
		.name	= "parallax_src",
		.owner	= THIS_MODULE,
	},
	.probe		= parallax_src_probe,
	.remove		= __devexit_p(parallax_src_remove),
};

static int __init parallax_src_init(void)
{
	return platform_driver_register(&parallax_src_driver);
}
module_init(parallax_src_init);

static void __exit parallax_src_exit(void)
{
	platform_driver_unregister(&parallax_src_driver);
}
module_exit(parallax_src_exit);

MODULE_DESCRIPTION("Parallax SRC Board Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:parallax-src");

