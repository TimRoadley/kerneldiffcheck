/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/slab.h>
#include <linux/sensled.h>
#include <asm/uaccess.h>

#define LEDS_MAJOR 125
//#define PARALLAX_PWM_MAX 0x1FFFFF
#define PARALLAX_PWM_MAX 0x3A5270
//#define PARALLAX_PWM_MAX_SHIFT 21

struct parallax_leds_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	char			name[32];
	unsigned int		period;
	unsigned int		max_brightness;
	unsigned int		max_shift;
	unsigned int		active_low;
};

static DEFINE_SPINLOCK(led_iface_lock); 
static struct sensled_color_config cur_color;
static struct parallax_leds_data *leds_data;

static void set_color(struct sensled_color_config *led_color)
{
	unsigned int val;
        unsigned int red = (led_color->red*led_color->brightness)/USER_MAX_BRIGHTNESS;
        unsigned int green = (led_color->green*led_color->brightness)/USER_MAX_BRIGHTNESS;
        unsigned int blue = (led_color->blue*led_color->brightness)/USER_MAX_BRIGHTNESS;
	// Scale colors 
	// B
//	printk("PARALLAX LED: Configuring %d/%d/%d (original: %d/%d/%d), brightness: %d\n", red, green, blue, 
//		led_color->red, led_color->green, led_color->blue, led_color->brightness);
	val = (blue*leds_data[0].period)/leds_data[0].max_brightness;
//	printk("PARALLAX LED: Configureing BLue: %X, max: %X, color: %d\n", val, leds_data[0].max_brightness, led_color->blue);
//        pwm_disable(leds_data[0].pwm);
        pwm_config(leds_data[0].pwm, val, leds_data[0].period);
        pwm_enable(leds_data[0].pwm);
	// G
 //       pwm_disable(leds_data[1].pwm);
	val = (green*leds_data[1].period)/leds_data[1].max_brightness;
//	printk("PARALLAX LED: Configureing Green: %X, max: %X, color: %d\n", val, leds_data[1].max_brightness, led_color->green);
        pwm_config(leds_data[1].pwm, val, leds_data[1].period);
        pwm_enable(leds_data[1].pwm);
	// R
//        pwm_disable(leds_data[2].pwm);
	val = (red*leds_data[2].period)/leds_data[2].max_brightness;
//	printk("PARALLAX LED: Configureing Red: %X, max: %X, color: %d\n", val, leds_data[2].max_brightness, led_color->red);
        pwm_config(leds_data[2].pwm, val, leds_data[2].period);
        pwm_enable(leds_data[2].pwm);

}

static ssize_t parallax_leds_read(struct file * file, char __user * buf,
                       size_t count, loff_t *ppos)
{
	return -EFAULT;
}

static ssize_t parallax_leds_write(struct file * file, const char __user * buf,
                        size_t count, loff_t *ppos)
{
	return -EFAULT;
}

static int parallax_leds_open(struct inode * inode, struct file * file)
{
        int ret = 0;
	return ret;
}

static int parallax_leds_release(struct inode * inode, struct file * file)
{
	return 0;
}

static long parallax_leds_ioctl(struct file *file, unsigned int cmd,
                        unsigned long arg)
{
        int ret = 0;
        struct sensled_color_config color;
        unsigned long flags = 0;

        switch (cmd) {
        case SENSLED_IOCTL_ON:
                break;
        case SENSLED_IOCTL_OFF:
                break;
        case SENSLED_IOCTL_SET_BRIGHTNESS:
                break;
        case SENSLED_IOCTL_GET_BRIGHTNESS:
                break;
        case SENSLED_IOCTL_SET_COLOR:
               if (copy_from_user(&color, (struct sensled_color_config __user *)arg,
                                   sizeof(struct sensled_color_config))) {
                        //printk(KERN_ERR "Unable to copy color configuration structure from user space\n");
                        return -EFAULT;
                }
		spin_lock_irqsave(&led_iface_lock, flags);
                /* Copy the last color set */
                memcpy(&cur_color, &color, sizeof(struct sensled_color_config));
//		printk(KERN_INFO "LED BOARD: IOCTL_SET: %d/%d/%d/%d, size: %d\n", color.red, color.green, 
//				color.blue, color.brightness, sizeof(struct sensled_color_config));
                set_color(&color);
		spin_unlock_irqrestore(&led_iface_lock, flags);
                break;
        case SENSLED_IOCTL_GET_COLOR:
                if(copy_to_user((struct sensled_color_config __user *)arg, &cur_color,
                                        sizeof(struct sensled_color_config)) != 0) {
                        return -EFAULT;
                }
//		printk(KERN_INFO "LED BOARD: IOCTL_GET: %d/%d/%d/%d\n", cur_color.red, 
//			cur_color.green, cur_color.blue, cur_color.brightness);
                break;
        default:
		ret = -ENOTTY;
                break;
        }
        return ret;
}


static const struct file_operations parallax_leds_fops = {
        .owner          = THIS_MODULE,
        .write          = parallax_leds_write,
        .read           = parallax_leds_read,
        .unlocked_ioctl = parallax_leds_ioctl,
        .open           = parallax_leds_open,
        .release        = parallax_leds_release,
};


static int parallax_leds_probe(struct platform_device *pdev)
{
        struct led_pwm_platform_data *pdata = pdev->dev.platform_data;
        struct led_pwm *cur_led;
        struct  parallax_leds_data *led_dat;
        int i, ret = 0;
	struct sensled_color_config led_color;

        if (!pdata)
                return -EBUSY;

	printk("PARALLAX LED: Initializing LED interface\n");

        leds_data = (struct parallax_leds_data *)kzalloc(sizeof(struct parallax_leds_data) * pdata->num_leds,
                                GFP_KERNEL);
        if (!leds_data)
                return -ENOMEM;

        i = 0;
        while (i < pdata->num_leds) {
                cur_led = &pdata->leds[i];
                led_dat = &leds_data[i];

                led_dat->pwm = pwm_request(cur_led->pwm_id,
                                cur_led->name);
                if (IS_ERR(led_dat->pwm)) {
                        ret = PTR_ERR(led_dat->pwm);
                        dev_err(&pdev->dev, "unable to request PWM %d\n",
                                        cur_led->pwm_id);
                        goto err;
                }
		strcpy(led_dat->name, cur_led->name);
                led_dat->active_low = cur_led->active_low;
                led_dat->period = cur_led->pwm_period_ns;
                led_dat->max_brightness = cur_led->max_brightness;
                led_dat->max_shift = cur_led->max_shift;
		/* Turn ON PWM with low brightness */
//                pwm_config(led_dat->pwm, 40000, led_dat->period);
//                pwm_enable(led_dat->pwm);

                i++;
        }
	// Initialize color to kernel boot - orange
	led_color.red = 255;
	led_color.green = 163;
	led_color.blue = 0;
	led_color.brightness = 10;
	set_color(&led_color);
	memcpy(&cur_color, &led_color, sizeof(struct sensled_color_config));

        platform_set_drvdata(pdev, leds_data);
	// Register as character device
        if (register_chrdev (LEDS_MAJOR, "parallax_leds", &parallax_leds_fops)) {
                printk (KERN_ERR "parallax leds: unable to get major %d\n", LEDS_MAJOR);
                return -EIO;
        }
	spin_lock_init(&led_iface_lock);
	printk("PARALLAX LED: LED interface initialized\n");
        return 0;
err:
        if (i > 0) {
                for (i = i - 1; i >= 0; i--) {
                       pwm_free(leds_data[i].pwm);
                }
        }

        kfree(leds_data);
	return ret;
}

static int parallax_leds_remove(struct platform_device *pdev)
{
	int i;
	struct led_pwm_platform_data *data = pdev->dev.platform_data;

	// Free PWM
        for (i = 0; i < data->num_leds; i++) {
                pwm_free(leds_data[i].pwm);
        }
	// Unregister device
	unregister_chrdev(LEDS_MAJOR, "parallax_leds");
        kfree(leds_data);
	return 0;
}

static struct platform_driver parallax_leds_driver = {
	.driver		= {
		.name	= "parallax_leds",
		.owner	= THIS_MODULE,
	},
	.probe		= parallax_leds_probe,
	.remove		= __devexit_p(parallax_leds_remove),
};

static int __init parallax_leds_init(void)
{
	return platform_driver_register(&parallax_leds_driver);
}
module_init(parallax_leds_init);

static void __exit parallax_leds_exit(void)
{
	platform_driver_unregister(&parallax_leds_driver);
}
module_exit(parallax_leds_exit);

MODULE_DESCRIPTION("Parallax LED Board Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:parallax-leds");

