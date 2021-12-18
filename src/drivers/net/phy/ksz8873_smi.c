/* Micrel KSZ8873 SMI interface driver 
* 
* Copyright (C) 2010 ParkAssist. Konstantyn Prokopenko
* 
* This driver is based on the driver for Realtek RTL8366 by
* Gabor Juhos <juhosg@openwrt.org> 
* 
* This program is free software; you can redistribute it and/or modify it 
* under the terms of the GNU General Public License version 2 as published 
* by the Free Software Foundation. 
*/ 
  
#include <linux/kernel.h> 
#include <linux/module.h> 
#include <linux/device.h> 
#include <linux/delay.h> 
#include <mach/gpio.h> 
#include <linux/spinlock.h> 
#include <linux/skbuff.h> 
  
#include <linux/ksz8873_smi.h>
  
#define KSZ8873_SMI_ACK_RETRY_COUNT         1000 
#define KSZ8873_SMI_CLK_DELAY               10 /* nsec */ 
#define KSZ8873_SMI_DATA_DELAY              20 /* nsec */ 
#define KSZ8873_SMI_READ_DELAY              250 /* nsec */ 


#define KSZ8873_PIN_SCL IMX_GPIO_NR(1, 31)
#define KSZ8873_PIN_SDA IMX_GPIO_NR(1, 22)
#define KSZ8873_OFF_SCK 31
#define KSZ8873_OFF_SDA 22

#define GPIO_DR         0x00
#define GPIO_GDIR       0x04
#define GPIO_PSR        0x08

/* Create static table for MII/SMI protocols */
static struct ksz8873_smi_frame_control fctl_table[] = {
	{28, 1, 23, 18, 0},	/* PHY 1 MII operations */
	{28, 2, 23, 18, 0},	/* PHY 2 MII operations */
	{26, 0, 23, 18, 0}	/* PHY 3 SMI operations */
};

static inline void my_delay(unsigned int d) {
	unsigned int __d = d;
	while(__d) {
	    asm("mov r0,r0");
	    __d--;
	}
}

static inline void ksz8873_smi_clk_delay(struct ksz8873_smi *smi) 
{ 
//        my_delay(KSZ8873_SMI_CLK_DELAY); 
        ndelay(KSZ8873_SMI_CLK_DELAY); 
} 
static inline void ksz8873_smi_data_delay(struct ksz8873_smi *smi) 
{ 
//        my_delay(KSZ8873_SMI_DATA_DELAY); 
        ndelay(KSZ8873_SMI_DATA_DELAY); 

} 
static inline void ksz8873_smi_read_delay(struct ksz8873_smi *smi) 
{ 
//        my_delay(KSZ8873_SMI_READ_DELAY); 
        ndelay(KSZ8873_SMI_READ_DELAY); 
}

static inline void ksz8873_smi_set_direction(struct ksz8873_smi *smi, u32 dir, u32 offset)
{
	u32 l = __raw_readl(smi->base + GPIO_GDIR);
        if (dir)
                l |= 1 << offset;
        else
                l &= ~(1 << offset);
        __raw_writel(l, smi->base + GPIO_GDIR);
//	printk(KERN_INFO "KSZ8873_SMI: direction register: %X\n", __raw_readl(smi->base + GPIO_GDIR));
}

static inline void ksz8873_smi_set_gpio(struct ksz8873_smi *smi, u32 offset, u32 val)
{
	u32 l = __raw_readl(smi->base + GPIO_DR);
	if(val)
                l |= 1 << offset;
        else
                l &= ~(1 << offset);
        __raw_writel(l, smi->base + GPIO_DR);
}
 
static inline u32 ksz8873_smi_get_gpio(struct ksz8873_smi *smi, u32 offset)
{
	u32 l = __raw_readl(smi->base + GPIO_DR);
//	printk("++OFF: %d, GPIO_DR: %X\n", offset, l);
	return ((l >> offset) & 1);
}

static void ksz8873_smi_start(struct ksz8873_smi *smi) 
{ 
        unsigned int sda = smi->gpio_sda; 
        unsigned int sck = smi->gpio_sck; 
 
        /* 
         * Set GPIO pins to output mode, with initial state: 
         * SCK = 0, SDA = 1 
         */ 
        gpio_direction_output(sck, 0); 
        gpio_direction_output(sda, 1); 
        ksz8873_smi_clk_delay(smi); 
 
        /* CLK 1: 0 -> 1, 1 -> 0 */ 
        __gpio_set_value(sck, 1); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 0); 
        ksz8873_smi_clk_delay(smi); 
  
        /* CLK 2: */ 
        __gpio_set_value(sck, 1); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sda, 0); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 0); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sda, 1); 
} 
  
static void ksz8873_smi_stop(struct ksz8873_smi *smi) 
{ 
        unsigned int sda = smi->gpio_sda; 
        unsigned int sck = smi->gpio_sck; 
  
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sda, 0); 
        __gpio_set_value(sck, 1); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sda, 1); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 1); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 0); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 1); 
  
         /* add a click */ 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 0); 
        ksz8873_smi_clk_delay(smi); 
        __gpio_set_value(sck, 1); 
 
        /* set GPIO pins to input mode */ 
        gpio_direction_input(sda); 
        gpio_direction_input(sck); 
} 
  
static void ksz8873_smi_write_bit(struct ksz8873_smi *smi, u8 bit) 
{ 
        unsigned int sda = smi->gpio_sda; 
        unsigned int sck = smi->gpio_sck; 
 
        /* set bit */ 
//        gpio_set_value(sda, bit & 0x01);
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, (bit & 0x01));
	/* Delay while waiting for data to be ON: Tmd1*/
        ksz8873_smi_clk_delay(smi); 
        /* Set clock bit to write the value bit */ 
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 1);
//        gpio_set_value(sck, 1); 
	/* Delay while waiting for data to be ON: Tmd1*/
        ksz8873_smi_data_delay(smi); 
	/* Reset clock bit */
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 0);
//         gpio_set_value(sck, 0); 
	/* Make sure clock is de-asserted */
        ksz8873_smi_clk_delay(smi); 
//        __gpio_set_value(sda, 0);
         
 } 
 

static void ksz8873_smi_write_clock_bit(struct ksz8873_smi *smi)
{
        unsigned int sda = smi->gpio_sda;
        unsigned int sck = smi->gpio_sck;

        /* set bit */
        /* Delay while waiting for data to be ON: Tmd1*/
        ksz8873_smi_clk_delay(smi);
        /* Set clock bit to write the value bit */
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 1);
//        gpio_set_value(sck, 1);
        ksz8873_smi_data_delay(smi);
        /* Reset clock bit */
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 0);
//         gpio_set_value(sck, 0);
        /* Make sure clock is de-asserted */
        ksz8873_smi_clk_delay(smi);
}


static void ksz8873_smi_write_bits(struct ksz8873_smi *smi, u32 data, u32 len)
{
	int i;
	
        for(i = len-1; i >=0; i--) {
		ksz8873_smi_write_bit(smi, (u8)(data  >> i));
         }
 }

static void ksz8873_smi_write_clock(struct ksz8873_smi *smi, u32 data, u32 len)
{
        int i;

        for(i = len-1; i >=0; i--) {
                ksz8873_smi_write_clock_bit(smi);
         }
 }

 
static void ksz8873_smi_read_bits(struct ksz8873_smi *smi, u32 len, u32 *data) 
{ 
        unsigned int sda = smi->gpio_sda; 
        unsigned int sck = smi->gpio_sck; 
  
	/* Delay before clocking for valid data */ 
//       ksz8873_smi_read_delay(smi); 
       ksz8873_smi_clk_delay(smi); 
        for (*data = 0; len > 0; len--) { 
                u32 u = 0; 
                /* Clock the value */ 
		ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 1);
//                gpio_set_value(sck, 1); 
		/* Wait for riding clock edge */
//                ksz8873_smi_clk_delay(smi); 
		/* Get data bit */
//		u = ksz8873_smi_get_gpio(smi, KSZ8873_OFF_SDA);
//                u = gpio_get_value(sda);
		/* Delay a little again */
                ksz8873_smi_read_delay(smi); 
		/* Set clock back */
		ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 0);
//                gpio_set_value(sck, 0); 
		/* Delay before next operation */
//                ksz8873_smi_clk_delay(smi); 
		/* Get data bit */
		u = ksz8873_smi_get_gpio(smi, KSZ8873_OFF_SDA);
	 	*data |= ((u & 1) << (len - 1));
         } 
} 
  
static int ksz8873_smi_wait_for_ack(struct ksz8873_smi *smi) 
{ 
        int retry_cnt; 
 
        retry_cnt = 0; 
        do { 
                u32 ack; 
                ksz8873_smi_clk_delay(smi); 
                ksz8873_smi_read_bits(smi, 1, &ack); 
                if (ack == 0) 
                        break; 
 
                if (++retry_cnt > KSZ8873_SMI_ACK_RETRY_COUNT) 
                        return -EIO; 
        } while (1); 
 
        return 0; 
} 
  
static int ksz8873_smi_write_byte(struct ksz8873_smi *smi, u8 data) 
{ 
        ksz8873_smi_write_bits(smi, data, 8); 
        return 0; 
} 

/* 
 * Helper function to send the preamble, address, and
 * register (common to read and write).
*/
static void ksz8873_cmd(struct ksz8873_smi *smi, int opcode, u8 phy, u8 reg)
{
	/* Send preamble 32 1's */
        ksz8873_smi_write_bits(smi, 0xFFFFFFFF, 32);
	/* Send start frame 01 */
        ksz8873_smi_write_bit(smi, 0);
        ksz8873_smi_write_bit(smi, 1);
	/* Standard MII */
	/* Opcode is 00 for everything in Micrel chip */
        ksz8873_smi_write_bit(smi, 0);
        ksz8873_smi_write_bit(smi, 0);
	/* PHY address. Opcode is a part of this address  */
        ksz8873_smi_write_bit(smi, opcode);	/* Opcode bit 1 - read, 0 - write */
        ksz8873_smi_write_bit(smi, 0);	/* Undefined bit after that */
	/* Register address */
        ksz8873_smi_write_bits(smi, reg, 8); /* Reg addr */
}

 
/* Read MII/SMI register */
int ksz8873_smi_read_reg(struct ksz8873_smi *smi, u32 phy_id, u32 addr, u32 *data) 
{ 
	u32 smi_data = 0;
        unsigned long flags = 0;
        int ret;
        struct ksz8873_smi_frame_control *fctl = NULL;
        u32 preamble = 0xFFFFFFFF;
        u32 frame = 0;

        if(phy_id < 1 || phy_id > 3) {
                printk(KERN_INFO "KSZ8873_SMI: ERROR: Invalid PHY ID: %d\n", phy_id);
                return -1;
        }
        fctl = &fctl_table[phy_id-1];


        spin_lock_irqsave(&smi->lock, flags); 
 
//        ksz8873_smi_start(smi); 

	/* Set direction to output */
//        gpio_direction_output(smi->gpio_sda, 1); 
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
//	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, 1);
        /* Assemble MII/SMI command to the Micrel KSZ8873 chip. 
        *
        * We are creating frame here based on PHY ID passed. For PHY 1 and 2, use MII frames.
        */
        /* SOF bits are generic for all frames */
        frame = (1L << 30);
        /* OPCODE is different and shifts to different location in the frame */
        /* Note that for SMI it shifts to the bit 26-27. Bit 26 is don't care . */
        frame |= (((unsigned int)KSZ8873_MII_READ) << fctl->opcode_shift);
        /* SHift PHY address. For PHY3 it is no-op. */
        frame |= ((fctl->phy_addr & 0x03) << fctl->phy_addr_shift);
        /* Shift register address */
        frame |= ((addr & 0xFF) << fctl->reg_addr_shift);
//        frame |= (1L << 17);	// TA
//	frame |= 0xFFFF;
	/* No data! */
        /* Send preamble */
//        printk(KERN_INFO "KSZ8873_SMI: READ command: PHY: %d, ADDR: %X, PREAMBLE: %X, FRAME: %X\n", phy_id, addr, preamble, frame);
	ksz8873_smi_write_bits(smi, preamble, 32);
//	ksz8873_smi_write_clock(smi, preamble, 32);
        /* send Write command */
	ksz8873_smi_write_bits(smi, (frame >> 18), 14);
//	ksz8873_smi_write_bits(smi, frame, 32);

	/* Check for turnaround bit which should be '0' */
	/* Set direction to input */
//         gpio_direction_input(smi->gpio_sda); 
	ksz8873_smi_set_direction(smi, 0, KSZ8873_OFF_SDA);

	if((ret = ksz8873_smi_wait_for_ack(smi))) {
		printk(KERN_INFO "Ack wait failure\n");
		goto out;
	}
        ksz8873_smi_read_bits(smi, 16, &smi_data); 
// 	printk(KERN_INFO "KSZ8873_SMI: READ data: %X\n", smi_data);

	*data = smi_data;
	 
	/* Read last idle */
        ksz8873_smi_read_bits(smi, 1, &smi_data); 
        ret = 0; 
 
 out: 
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
//	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, 1);
//        gpio_direction_output(smi->gpio_sda, 1); 
//        ksz8873_smi_stop(smi); 
        spin_unlock_irqrestore(&smi->lock, flags); 
 
        return ret; 
} 

EXPORT_SYMBOL_GPL(ksz8873_smi_read_reg); 
  
int ksz8873_smi_write_reg(struct ksz8873_smi *smi, u32 phy_id, u32 addr, u32 data) 
{ 
        unsigned long flags = 0; 
        int ret; 
	struct ksz8873_smi_frame_control *fctl = NULL;
	u32 preamble = 0xFFFFFFFF;
	u32 frame = 0;
	int frame_len = sizeof(u32);
	unsigned char *frame_ptr;

	if(phy_id < 1 || phy_id > 3) {
		printk(KERN_INFO "KSZ8873_SMI: ERROR: Invalid PHY ID: %d\n", phy_id);
		return -EINVAL;
	}
	fctl = &fctl_table[phy_id-1];

        spin_lock_irqsave(&smi->lock, flags); 
	// Set SDA to output high
//        gpio_direction_output(smi->gpio_sda, 1); 
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
//        ksz8873_smi_start(smi); 
 
 
	/* Assemble MII/SMI command to the Micrel KSZ8873 chip. 
 	*
 	* We are creating frame here based on PHY ID passed. For PHY 1 and 2, use MII frames.
 	*/
	/* SOF bits are generic for all frames */
	frame = (1L << 30);
	/* OPCODE is different and shifts to different location in the frame */
	/* Note that for SMI it shifts to the bit 26-27. Bit 26 is don't care . */
	frame |= (KSZ8873_MII_WRITE << fctl->opcode_shift);
	/* SHift PHY address. For PHY3 it is no-op. */
	frame |= ((fctl->phy_addr & 0x03) << fctl->phy_addr_shift);
	/* Shift register address */
	frame |= ((addr & 0xFF) << fctl->reg_addr_shift);
        frame |= (1L << 17);	// TA
	/* Shift data. For SMI data[15:8] are not defined, can be anything */
	frame |= ((data & 0xFFFF) << fctl->data_shift);
//	if(phy_id == 3)
//	        printk(KERN_INFO "KSZ8873_SMI: WRITE command: ADDR: %X, FRAME: %X\n", addr, frame);

        ksz8873_smi_write_bits(smi, preamble, 32);
        /* send Write command */
        ksz8873_smi_write_bits(smi, frame, 32);

//	if((ret = ksz8873_smi_wait_for_ack(smi))) {
//		printk(KERN_INFO "Ack wait failure\n");
//		goto out;
//	}
	
        ret = 0; 
 
 out: 
//       ksz8873_smi_stop(smi); 
	/* Switch to high-Z state */
//        gpio_direction_input(smi->gpio_sda); 
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
//	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, 1);

        spin_unlock_irqrestore(&smi->lock, flags); 
  
        return ret; 
} 
EXPORT_SYMBOL_GPL(ksz8873_smi_write_reg); 

#if 0 
int ksz8873_smi_rmwr(struct ksz8873_smi *smi, u8 mii_id, u8 addr, u32 mask, u32 data) 
{ 
        u32 t; 
        int err; 
 
    //    err = ksz8873_smi_read_reg(smi, mii_id, addr, &t); 
        if (err) 
                return err; 
 
     //   err = ksz8873_smi_write_reg(smi, addr, (t & ~mask) | data); 
        return err; 
 
} 
EXPORT_SYMBOL_GPL(ksz8873_smi_rmwr); 
#endif


int ksz8873_smi_init(struct ksz8873_smi *smi) 
{
	int err = 0;

	/* Initialize SMI pins */ 
	smi->gpio_sda = KSZ8873_PIN_SDA;
	smi->gpio_sck = KSZ8873_PIN_SCL;
	printk(KERN_INFO "KSZ8873_SMI: Init GPIO: %d/%d\n", smi->gpio_sda, smi->gpio_sck);

	err = gpio_request(smi->gpio_sda, "mdio"); 
        if (err) { 
                dev_err(smi->parent, "gpio_request failed for %u, err=%d\n", 
                        smi->gpio_sda, err); 
                goto err_out; 
        } 
 
        err = gpio_request(smi->gpio_sck, "mdc"); 
        if (err) { 
                dev_err(smi->parent, "gpio_request failed for %u, err=%d\n", 
                        smi->gpio_sck, err); 
                goto err_out; 
        }
	printk(KERN_INFO "<<GPIO Request OK>>\n");
	// OK, request kernel address for GPIO base
//	smi->base = phys_to_virt(0x209C000);
	smi->base = 0xF409C000;
	
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SCK);
	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, 0);
	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SCK, 0);

//	ksz8873_smi_set_direction(smi, 1, KSZ8873_OFF_SDA);
//	ksz8873_smi_set_gpio(smi, KSZ8873_OFF_SDA, 1);

//        gpio_direction_output(smi->gpio_sck, 0); 
//        gpio_direction_input(smi->gpio_sda); 

	spin_lock_init(&smi->lock);
	// Reset switch
//        ksz8873_smi_write_reg(smi, 2, 0x43, 0x10);
  //      msleep(100);
  //      ksz8873_smi_write_reg(smi, 2, 0x43, 0x00);

	err = 0;
err_out:
	if(err != 0) {
		printk(KERN_INFO "KSZ8873_SMI: ERROR in initializing SMI GPIO %d/%d: %d\n", smi->gpio_sda, smi->gpio_sck, err);
	}
        return err; 
 
} 
EXPORT_SYMBOL_GPL(ksz8873_smi_init); 
 
MODULE_DESCRIPTION("Micrel KSZ8873 SMI interface driver"); 
MODULE_AUTHOR("Konstantyn Prokopenko kp@parkassist.com"); 
MODULE_LICENSE("GPL v2"); 

