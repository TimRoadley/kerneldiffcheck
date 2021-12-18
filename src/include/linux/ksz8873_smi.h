/* SMI bus specific definitions for KSZ8873RLL chip. 
  * 
  * Copyright (C) 2010 ParkAssist Inc.  Konstantyn Prokopenko 
  *
  * This software is based on device driver for RTL8366
  * by Gabor Juhos <juhosg@openwrt.org>
  * 
  * This program is free software; you can redistribute it and/or modify it 
  * under the terms of the GNU General Public License version 2 as published 
  * by the Free Software Foundation. 
  */ 
  
#ifndef __KSZ8873_SMI_H__
#define __KSZ8873_SMI_H__
  
#include <linux/phy.h> 


#define KSZ8873_SMI_READ        0x10 
#define KSZ8873_SMI_WRITE        0x01 

#define KSZ8873_MII_READ        0x02 
#define KSZ8873_MII_WRITE       0x01 
  
struct ksz8873_smi_ops; 
struct mii_bus; 

/* Micrel chip SMI frame */
struct ksz8873_smi_frame_control {
	u32 opcode_shift;	/* Opcode shift */
	u32 phy_addr;		/* PHY address to set */
	u32 phy_addr_shift;	/* PHY address shift in the command word */
	u32 reg_addr_shift;	/* Register address shift */
	u32 data_shift;		/* Data shift */
};
 
 
struct ksz8873_smi { 
	struct device           *parent; 
	void __iomem		*base;
        unsigned int            gpio_sda; 
        unsigned int            gpio_sck; 
        spinlock_t              lock; 
        struct mii_bus          *mii_bus; 
        struct ksz8873_smi_ops  *ops; 
}; 
  
struct ksz8873_smi_ops { 
        int     (*detect)(struct ksz8873_smi *smi); 
 
        int     (*mii_read)(struct mii_bus *bus, int addr, int reg); 
        int     (*mii_write)(struct mii_bus *bus, int addr, int reg, u16 val); 
}; 
 
  
int ksz8873_smi_init(struct ksz8873_smi *smi); 
void ksz8873_smi_cleanup(struct ksz8873_smi *smi); 
int ksz8873_smi_write_reg(struct ksz8873_smi *smi, u32 phy_id, u32 addr, u32 data); 
int ksz8873_smi_read_reg(struct ksz8873_smi *smi, u32 phy_id, u32 addr, u32 *data); 
//int ksz8873_smi_rmwr(struct ksz8873_smi *smi, u32 phy_id, u32 addr, u32 mask, u32 data); 
 
#endif 

