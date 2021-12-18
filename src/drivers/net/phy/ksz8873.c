/*
 * drivers/net/phy/ksz8873.c
 *
 * Driver for Micrel PHYs
 *
 * Author: Konstantyn Prokopenko
 *
 * Copyright (C) ParkAssist 2010. 
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This device driver is designed based on phy_device.c generic PHY module.
 * We have to access certain generic phy funcitonality from this module
 * as well.
 *
 * Support : ksz8873RLL RMII interface. This interface emulates PHY and 
 * is controlled by accessing 8 bit memory mapped registers via MDIO bus 
 * interface using Micrel's special SMI frames. Check Micrel's KSZ8873RLL
 * specification for more details
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unistd.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/ksz8873_smi.h>
#include <linux/ksz8873.h>
#include <linux/ksz8873phy.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

char *init_names[] = {"KSZ8873_0","KSZ8873_1", "KSZ8873_2", "KSZ8873_3", "KSZ8873_4"};
MODULE_DESCRIPTION("KSZ8873RLL RMII PHY");
MODULE_AUTHOR("Konstantyn Prokopenko");
MODULE_LICENSE("GPL");

static struct phy_driver ksz8873_driver;
static 	struct	cdev *ksz8873_cdev; /* Character devce structure */
static dev_t ksz8873_devno;
extern int mdio_bus_init(void);
extern void mdio_bus_exit(void);
static struct phy_device *phy_device;
static int mac_table_choice;
static int __read_phy_MIB_counters(struct phy_device *phydev, struct KSZ8873_MIB_entry *mibentry);
static void get_snapshot_entry(struct KSZ8873_snapshot_entry *snap);


static void ksz8873_reset(struct phy_device *phydev) 
{
	phy_write_smi(phy_device, KSZ8873_SMI_RESET, 0x10);
	msleep(100);
	phy_write_smi(phy_device, KSZ8873_SMI_RESET, 0x00);
}

void ksz8873_device_free(struct phy_device *phydev)
{
	kfree(phydev);
}

static void ksz8873_device_release(struct device *dev)
{
	phy_device_free(to_phy_device(dev));
}


/*
 * Create PHY device for Port 3 of Micrel chip. This port operates in RMII mode
 */
struct phy_device* ksz8873_device_create(struct mii_bus *bus, int addr, int phy_id)
{
	struct phy_device *dev;
	/* We allocate the device, and initialize the
	 * default values */
	printk(KERN_INFO "%s: Creating device: ID: %d, addr: %d\n", DRV_NAME, phy_id, addr);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);

	if (NULL == dev)
		return (struct phy_device*) PTR_ERR((void*)-ENOMEM);

	dev->dev.release = ksz8873_device_release;
	dev_set_name(&dev->dev, PHY_ID_FMT, bus->id, addr);

	/* 
	 * Hard-code speed and duplex here. Note that Micrel chip is connected directly to
	 * the Atmel MAC using RMII port 3. There is no auto-negotiation, all values are
	 * hard-coded and they are not changed on connection with different hubs and network
	 * interfaces. The outside world is connected to the two PHY ports for which we
	 * advertise auto-negotiation and other benefits...
	*/
//	dev->speed = SPEED_10;
	dev->speed = SPEED_100;
	dev->duplex = DUPLEX_FULL;
	dev->pause = dev->asym_pause = 0;
	dev->link = 1;
	dev->interface = PHY_INTERFACE_MODE_RMII; 
	/* No auto-negotiation on Port 3 of Micrel chip */
	dev->autoneg = AUTONEG_ENABLE;
//	dev->autoneg = AUTONEG_DISABLE;
	/* Set PHY IDs and addresses */
	dev->addr = addr;
	dev->phy_id = phy_id;
	dev->bus = bus;
        dev->dev.parent = bus->parent;
        dev->dev.bus = &mdio_bus_type;
        dev->irq = bus->irq != NULL ? bus->irq[addr] : PHY_POLL;

	dev->state = PHY_DOWN;

	mutex_init(&dev->lock);
	request_module(MDIO_MODULE_PREFIX MDIO_ID_FMT, MDIO_ID_ARGS(phy_id));
	return dev;
}

/**
 * ksz8873_get_phy_device - reads the specified PHY device and returns its @phy_device struct
 * @bus: the target MII bus. Note that we are reading ID from MII bus interface which should
 * give us 32 bit Micrel ID rather than SMI 8 bit IDs of the chip.
 * @addr: PHY address on the MII bus should be 0x03 for RMII Port 3.
 *
 * Description: Reads the ID registers of the PHY at @addr on the
 *   @bus, then allocates and returns the phy_device to represent it.
 */
struct phy_device * ksz8873_get_phy_device(struct mii_bus *bus, int addr)
{
	struct phy_device *dev = NULL;
	u32 phy_id;
	int r;

	printk(KERN_INFO "[PARALLAX]: KSZ8873: Get PHY device\n");
	/* 
	* Note, we are executing generic PHY function here which reads standard MII frame 
 	* to get 32 bit ID. The ID is read on three bus IDs of Micrel and all of them should 
	* return valid 16 bit ID assembled into PHY_ID_KSZ8873 32 bit value.
 	*/
	r = get_phy_id(bus, addr, &phy_id);
	if (r)
		return ERR_PTR(r);

	/* If the phy_id is mostly Fs, there is no device there */
	if ((phy_id & 0x1fffffff) == 0x1fffffff)
		return NULL;
	/* Create device for the PHY ID */
	dev = ksz8873_device_create(bus, addr, phy_id);

	return dev;
}
EXPORT_SYMBOL(ksz8873_get_phy_device);

/**
 * ksz8873_prepare_link - prepares the PHY layer to monitor link status
 * @phydev: target phy_device struct
 * @handler: callback function for link status change notifications
 *
 * Sets handler for the link adjustments
 */
void ksz8873_prepare_link(struct phy_device *phydev,
		void (*handler)(struct net_device *))
{
	phydev->adjust_link = handler;
}
EXPORT_SYMBOL(ksz8873_prepare_link);


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 user API functions
*/

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 open call. USER API
*/
static int ksz8873_open(struct inode *inode, struct file *file) {
	/* Nothing to be done here */
	//printk(KERN_INFO "KSZ8873 Open\n");
	return 0;
}
/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 close call. USER API
*/
static int ksz8873_release(struct inode *inode, struct file *file) {
	/* Nothing to be done here */
	//printk(KERN_INFO "KSZ8873 Close\n");
	return 0;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * Enable or disable PHY
*/
static int __set_PHY_enable(int phyID, bool enable) {
	u32 ctl32 = 0;

	if(enable == true) {
	        ctl32 |= (BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_ANENABLE);
		/* Write PHY function to BMCR register for both PHY IDs 1 and 2 */
	}
	else {
	        ctl32 = BMCR_PDOWN;
	}
        phy_write_ext(phy_device, phyID, MII_BMCR, ctl32);
	return 0;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
* Enable or disable TAIL TAG feature
*/
static int __set_tailtag_enable(bool enable) {
        u8 regVal = 0;

	regVal = phy_read_smi(phy_device, 0x3);
        if(enable == true) {
		regVal |= 0x40;
        }
        else {
		regVal &= ~0x40;
        }
	phy_write_smi(phy_device, 0x3, regVal);  
        return 0;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
* Set sniffing feature for a port
*/
static int __set_port_sniffer(struct _port_sniffer_control *sniff) {
        u8 monitor_regVal = 0;
	u8 monitor_regID = 0;
	u8 rx_regID = 0;
	u8 tx_regID = 0;
	int idx = 0;

	// Check if we need to disable all sniffing activities 
	if(sniff->enable == 0) {
		// Disable all sniffing on all ports
		for(idx = 0; idx < 3; idx++) {
	        	monitor_regVal = phy_read_smi(phy_device, (0x11+(0x10*idx)));
			monitor_regVal &= ~0xE0;
        		phy_write_smi(phy_device, (0x11+(0x10*idx)), monitor_regVal);
		}
	}
	else {
		/* Validate the monitor port */
		if((sniff->port_monitor < 1) || (sniff->port_monitor > 3)) {
			return -EINVAL;
		}
		/* Get port monitor */
		monitor_regID = 0x11 + (0x10 * (sniff->port_monitor - 1));
		/* Get correct RX forwarding port */
		if((sniff->port_rx_fwd >= 1) && (sniff->port_rx_fwd <= 3)) {
			rx_regID = 0x11 + (0x10 * (sniff->port_rx_fwd - 1));
		}
		/* Get correct TX forwarding port */
		if((sniff->port_tx_fwd >= 1) && (sniff->port_tx_fwd <= 3)) {
			tx_regID = 0x11 + (0x10 * (sniff->port_tx_fwd - 1));
		}
		
		/* Get and set value of the monitor port register */
        	monitor_regVal = phy_read_smi(phy_device, monitor_regID);
        	monitor_regVal |= 0x80;
	        phy_write_smi(phy_device, monitor_regID, monitor_regVal);
		if(rx_regID != 0) {
		       /* Get and set value of RX formward port */	
        		monitor_regVal = phy_read_smi(phy_device, rx_regID);
        		monitor_regVal |= 0x40;
		        phy_write_smi(phy_device, rx_regID, monitor_regVal);
		}
		if(tx_regID != 0) {
		       /* Get and set value of TX formward port */	
        		monitor_regVal = phy_read_smi(phy_device, tx_regID);
        		monitor_regVal |= 0x20;
		        phy_write_smi(phy_device, tx_regID, monitor_regVal);
		}
	}
        return 0;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
* Set spanning tree feature for a port
*/
static int __set_port_stp(struct _port_stp_control *stp) {
        u8 regID = 0;
	u8 regVal = 0;

	if((stp->portID < 1) || (stp->portID > 2)) {
		return -EINVAL;
	}
	regID = (stp->portID == 1)?0x12:0x22;
	/* Get value of the register */
	regVal =  phy_read_smi(phy_device, regID);

	if(stp->learning_enable == 0) {
		/* Disable STP */
		regVal |= 0x01;
	}
	else {
		regVal &= ~0x01;
		if(stp->rx_enable) {
			regVal |= 0x02;
		}
		if(stp->tx_enable) {
			regVal |= 0x04;
		}
	}
	phy_write_smi(phy_device, regID, regVal);
	return 0;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
* Get PHY port link status
*/
static int __get_port_link_stat(int phyID) {
	int status;
	if(phyID == 1)
		status = phy_read_smi(phy_device, 0x1E);
	else
		status = phy_read_smi(phy_device, 0x2E);
	if(status & 0x20) {
		return 0;
	}
	return -1;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 IOCTL call. USER API
*/
static long ksz8873_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	int phyID;
	int status = -1;
	int err;
	struct _port_sniffer_control sniff;
	struct _port_stp_control stp;
	struct KSZ8873_MIB_entry mib;
        struct KSZ8873_snapshot_entry snap;

	/* Nothing to be done here */
	//printk(KERN_INFO "KSZ8873 IOCTL\n");
	switch(cmd){
	case  KSZ8873_IOCTL_PHY_ENABLE:
		/* Get PHY ID from user */
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		__set_PHY_enable(phyID, true);
		break;
	case  KSZ8873_IOCTL_PHY_DISABLE:
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		__set_PHY_enable(phyID, false);
		break;
	case  KSZ8873_IOCTL_TAILTAG_ENABLE:
		__set_tailtag_enable(true);
		break;
	case  KSZ8873_IOCTL_TAILTAG_DISABLE:
		__set_tailtag_enable(false);
		break;
	case KSZ8873_IOCTL_SET_SNIFFING:
                if (copy_from_user(&sniff, (struct _port_sniffer_control __user *)arg, sizeof(struct _port_sniffer_control))) {
                        return -EFAULT;
		}
		if(__set_port_sniffer(&sniff) != 0) {
			return -EINVAL;
		}
		break;
	case KSZ8873_IOCTL_SET_STP:
                if (copy_from_user(&stp, (struct _port_stp_control __user *)arg, sizeof(struct _port_stp_control))) {
                        return -EFAULT;
		}
		if(__set_port_stp(&stp) != 0) {
			return -EINVAL;
		}
		break;
	case KSZ8873_IOCTL_CHOOSE_DYNMAC:
		mac_table_choice = KSZ8873_MAC_TABLE_DYNAMIC;
		break;
	case KSZ8873_IOCTL_CHOOSE_STATMAC:
		mac_table_choice = KSZ8873_MAC_TABLE_STATIC;
		break;
	case KSZ8873_IOCTL_PHY_GET_LINK_STAT:
		/* Get PHY ID from user */
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		if(phyID < 0 || phyID > 2) {
			return -EINVAL;
		}
		/* Get link status */	
		status = __get_port_link_stat(phyID);
		if(copy_to_user((int __user *)arg, &status, sizeof(int)) != 0) {
			 return -EFAULT;
		}	
		break;
        case KSZ8873_IOCTL_GET_PHY_MIB_COUNTERS:
                if (copy_from_user(&mib, (struct KSZ8873_MIB_entry __user *)arg, sizeof(struct KSZ8873_MIB_entry))) {
                        return -EFAULT;
                }
                /* Allow for 3 ports */
                if((mib.portID < 1) || (mib.portID > 3)) {
                        return -EINVAL;
                }
                if((status = __read_phy_MIB_counters(phy_device, &mib)) != 0) {
                        return status;
                }
                if(copy_to_user((struct KSZ8873_MIB_entry __user *)arg, &mib, sizeof(struct KSZ8873_MIB_entry)) != 0) {
                         return -EFAULT;
                }
                break;
	case KSZ8873_IOCTL_PHY_RESET:
		printk(KERN_INFO "Resetting PHY chip...\n");
		ksz8873_reset(phy_device);
		printk(KERN_INFO "Reset done.\n");
		break;
	case KSZ8873_IOCTL_PHY_REINIT:
	        if (phy_device->drv->config_init) {
			printk(KERN_INFO "Re-initializing PHY chip...\n");
                	/* Call generic PHY fixup function */
                	err = phy_scan_fixups(phy_device);
                	if (err < 0)
                        	return err;
                	err = phy_device->drv->config_init(phy_device);
                	if (err < 0)
                        	return err;
			printk(KERN_INFO "Initialization done.\n");
        	}
		break;
	case KSZ8873_IOCTL_PHY_WRITE_SMI_REGISTER:
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		printk(KERN_INFO "KSZ8873: Writing value: %X to register: %X\n", ((phyID >> 16)&0xFFFF), phyID & 0xFFFF);
		phy_write_smi(phy_device, (phyID & 0xFFFF), ((phyID >> 16) & 0xFFFF));
		break;
	case KSZ8873_IOCTL_PHY_READ_SMI_REGISTER:
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		printk(KERN_INFO "KSZ8873: Reading register: %X, value: %X\n", (phyID & 0xFFFF), phy_read_smi(phy_device, (phyID & 0xFFFF)));
		break;
	case KSZ8873_IOCTL_PHY_READ_MIIM_REGISTER:
                if (copy_from_user(&phyID, (int __user *)arg, sizeof(int))) {
                        return -EFAULT;
		}
		printk(KERN_INFO "KSZ8873: Reading reg/PHY: %X/%X, value: %X\n", (phyID & 0xFFFF), (phyID >> 16), phy_read_ext(phy_device, (phyID >> 16), (phyID & 0xFFFF)));
		break;
        case KSZ8873_IOCTL_SNAPSHOT:
                get_snapshot_entry(&snap);
                if(copy_to_user((struct KSZ8873_snapshot_entry __user *)arg, &snap, sizeof(struct KSZ8873_snapshot_entry)) != 0) {
                         return -EFAULT;
                }
                break;
	default:
		return -ENOTTY;
	}

	return 0;
}

static int __read_MAC_table_entry(struct phy_device *phydev, int address, struct _dynamic_MAC_table_entry *pentry) {
	unsigned char addr_low;		/* Low bits of an address */
	unsigned char addr_hi;		/* High bits of an address */
	unsigned char val8;
	unsigned int MAC;		/* Temp MAC var */

	/* Prepare address bits */
	addr_low = (unsigned char)address;
	addr_hi = (unsigned char)((address >> 8) & 0x03);

        phy_write_smi(phydev, 0x79, (0x18 | addr_hi));	/* Write sequence with address high bits */
        phy_write_smi(phydev, 0x7A, addr_low);		/* Address - low bits */
	/* Check if we are ready to read address 0 entry */
	do {
		val8 = phy_read_smi(phydev, 0x7B);
	}while(val8 & 0x80);
	/* Check for valid entries */
	if(val8 & 0x04) {
		printk(KERN_INFO "KSZ8873: There are no valid entries in the dynamic MAC table\n");
		return -1;
	}
	/* Read the size of the dynamic table. Read with every entry */
	/* Shift lower two bits of a table size in place */
	pentry->table_size = (val8 & 0x03);
	pentry->table_size <<= 8;
	/* Read number of entries and add to the size value */
	pentry->table_size |= ((unsigned short)phy_read_smi(phydev, 0x7C) & 0x00FF);
	pentry->table_size++;	// Always one more than reported value
	val8 = phy_read_smi(phydev, 0x7D);	/* Bits 48 - 55 */
	/* Populate aging - upper 2 bits to lower two bits of char */
	pentry->aging = (val8 >> 6) & 0x03;
	/* Filter ID - lower 4 bits */
	pentry->FID = (val8 & 0x0F);
	/* Source port: lower 2 bits of upper nibble - to lower 2 bits of source port field */
	pentry->source_port = ((val8 >> 4) & 0x03);
//	printk(KERN_INFO "MAC TABLE: info reg part: %X\n", val8);
	
	/* Get MAC address */
	/* Upper MAC - 2 bytes  */
	pentry->MAC1 = phy_read_smi(phydev, 0x7E);
	pentry->MAC1 <<= 8;
	pentry->MAC1 |= phy_read_smi(phydev, 0x7F);
	/* Lower MAC - 4 bytes */
	pentry->MAC0 = 0;
	MAC = phy_read_smi(phydev, 0x80);
	pentry->MAC0 |= ((MAC << 24) & 0xFF000000);
	MAC = phy_read_smi(phydev, 0x81);
	pentry->MAC0 |= ((MAC << 16) & 0x00FF0000);
	MAC = phy_read_smi(phydev, 0x82);
	pentry->MAC0 |= ((MAC << 8) & 0x0000FF00);
	MAC = phy_read_smi(phydev, 0x83);
	pentry->MAC0 |= (MAC & 0x000000FF);
	return 0;
}



static int __read_phy_MIB_counters(struct phy_device *phydev, struct KSZ8873_MIB_entry *mibentry) {
        unsigned int address;
        unsigned char addr_low;         /* Low bits of an address */
        unsigned char addr_hi;          /* High bits of an address */
        unsigned char val8;
        unsigned int  val32;            /* Temp 32bit var */
        unsigned long timeout;
        unsigned int *regPtr;           /* Registers pointer */
        unsigned int port_base;         /* Port offset */

        /* Align pointer to the beginning of the MIB entry structure */
        regPtr = (unsigned int *)mibentry;
        /* Find base offset to a per-port register set */
        port_base = (mibentry->portID - 1) * 0x20;
        regPtr++;       /* Adjust pointer to the first register */
        /* For all addresses, read all counters */
        for(address = port_base; address <= (port_base+0x1F); address++) {
                /* Prepare address bits */
                addr_low = (unsigned char)address;
                addr_hi = (unsigned char)((address >> 8) & 0x03);

                phy_write_smi(phydev, 0x79, (0x1C | addr_hi));  /* Write sequence with address high bits */
                phy_write_smi(phydev, 0x7A, addr_low);          /* Address - low bits */
                /* Check if we are ready to read address 0 entry */
                timeout = 100;
                val8 = phy_read_smi(phydev, 0x80);
                while(!(val8 & 0x40)) {
                        val8 = phy_read_smi(phydev, 0x80);
                        if(timeout > 0)
                                timeout--;
                        else
                                break;
                        msleep(1);
                }
                /* store upper bits */
                val32 = val8;
                *regPtr |= ((val32 << 24) & 0xFF000000);
                /* Read remaining register value */
                val32 = phy_read_smi(phydev, 0x81);
                *regPtr |= ((val32 << 16) & 0x00FF0000);
                val32 = phy_read_smi(phydev, 0x82);
                *regPtr |= ((val32 << 8) & 0x0000FF00);
                val32 = phy_read_smi(phydev, 0x83);
                *regPtr |= (val32 & 0x000000FF);
                regPtr++;       /* Next register */
        }
        return 0;
}


/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873  generate snapshot of all vital parameters in the Micrel PHY system
*/
static void get_snapshot_entry(struct KSZ8873_snapshot_entry *snap)
{
    int status, idx, ti;
    u8 regval;
    unsigned short *phy_ctrl = &snap->phy_control[0];
    unsigned short *def_tag = &snap->default_tag[0];
    /* Clear structure parameters */
    memset(snap, 0x00, sizeof(struct KSZ8873_snapshot_entry));
    /* Get global control stats */
	regval = phy_read_smi(phy_device, 0x02);
	snap->global_control |= ((regval & 0x08)?KSZ8873_SNAPSHOT_BIT_PASS_FLOW_CTRL:0);
	regval = phy_read_smi(phy_device, 0x03);
	snap->global_control |= ((regval & 0x80)?KSZ8873_SNAPSHOT_BIT_PASS_ALL_FRAMES:0);
	snap->global_control |= ((regval & 0x04)?KSZ8873_SNAPSHOT_BIT_AGING_ENBL:0);
	snap->global_control |= ((regval & 0x02)?KSZ8873_SNAPSHOT_BIT_FAST_AGING_ENBL:0);
	regval = phy_read_smi(phy_device, 0x06);
	snap->global_control |= ((regval & 0x10)?KSZ8873_SNAPSHOT_BIT_MBPS_MODE:0);
	snap->global_control |= ((regval & 0x20)?KSZ8873_SNAPSHOT_BIT_FULL_DUPLEX_FLOW_CTRL:0);
	snap->global_control |= ((regval & 0x40)?KSZ8873_SNAPSHOT_BIT_DUPLEX_MODE:0);
    /* Get port link status (PHY1 and PHY2 only) */
    for(idx = 0; idx < 2; idx++) {
	    status = __get_port_link_stat(idx+1);
	    if(!status) {
	    	*(phy_ctrl+idx) |= KSZ8873_SNAPSHOT_BIT_PHY_LINK_GOOD;
	    }
	}
	/* Get PHY specific control registers */
    for(idx = 0; idx < KSZ8873_PHY_IFACE_NUMBER; idx++) {
	 	regval = phy_read_smi(phy_device, (0x10+idx*0x10));
		*(phy_ctrl+idx) |= ((regval & 0x08)?KSZ8873_SNAPSHOT_BIT_STORM_PROTECTION:0);
	 	regval = phy_read_smi(phy_device, (0x12+idx*0x10));
		*(phy_ctrl+idx) |= ((regval & 0x10)?KSZ8873_SNAPSHOT_BIT_FORCE_FLOW_CTRL:0);
		*(phy_ctrl+idx) |= ((regval & 0x08)?KSZ8873_SNAPSHOT_BIT_BACK_PRESSURE_ENBL:0);
		*(phy_ctrl+idx) |= ((regval & 0x04)?KSZ8873_SNAPSHOT_BIT_TRANSMIT_ENBL:0);
		*(phy_ctrl+idx) |= ((regval & 0x02)?KSZ8873_SNAPSHOT_BIT_RECEIVE_ENBL:0);
		*(phy_ctrl+idx) |= ((regval & 0x01)?KSZ8873_SNAPSHOT_BIT_LEARNING_ENBL:0);
   }
	/* Get PHY specific default tag */
    for(idx = 0; idx < KSZ8873_PHY_IFACE_NUMBER; idx++) {
	 	regval = phy_read_smi(phy_device, (0x13+idx*0x10));
	 	*(def_tag+idx) = regval;
	 	*(def_tag+idx) <<= 8;
	 	regval = phy_read_smi(phy_device, (0x14+idx*0x10));
	 	*(def_tag+idx) |= regval;
	}
	 /* PHY specific control registers for PHY1 and PHY2 only */
	/* Get PHY specific control registers */
    for(idx = 0; idx < 2; idx++) {
	 	regval = phy_read_smi(phy_device, (0x1a+idx*0x10));
		snap->vct_result[idx] = regval;
	 	regval = phy_read_smi(phy_device, (0x1c+idx*0x10));
		*(phy_ctrl+idx) |= ((regval & 0x80)?KSZ8873_SNAPSHOT_BIT_AUTO_NEG_ENBL:0);
		*(phy_ctrl+idx) |= ((regval & 0x40)?KSZ8873_SNAPSHOT_BIT_FORCE_SPEED:0);
		*(phy_ctrl+idx) |= ((regval & 0x20)?KSZ8873_SNAPSHOT_BIT_FORCE_DUPLEX:0);
	 	regval = phy_read_smi(phy_device, (0x1d+idx*0x10));
		*(phy_ctrl+idx) |= ((regval & 0x08)?KSZ8873_SNAPSHOT_BIT_POWER_DOWN:0);
		ti = idx*0x10;
	 	regval = phy_read_smi(phy_device, (0x1e + ti));
		*(phy_ctrl+idx) |= ((regval & 0x40)?KSZ8873_SNAPSHOT_BIT_AUTO_NEG_DONE:0);
    }

    for(idx = 0; idx < KSZ8873_PHY_IFACE_NUMBER; idx++) {
	 	regval = phy_read_smi(phy_device, (0x1f+idx*0x10));
		*(phy_ctrl+idx) |= ((regval & 0x02)?KSZ8873_SNAPSHOT_BIT_OPERATION_DUPLEX:0);
		*(phy_ctrl+idx) |= ((regval & 0x04)?KSZ8873_SNAPSHOT_BIT_OPERATION_SPEED:0);
		*(phy_ctrl+idx) |= ((regval & 0x08)?KSZ8873_SNAPSHOT_BIT_RECEIVE_FLOW_CTRL_STATUS:0);
		*(phy_ctrl+idx) |= ((regval & 0x10)?KSZ8873_SNAPSHOT_BIT_TRANSMIT_FLOW_CTRL_STATUS:0);
	}	
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 read call. USER API
*/
static ssize_t ksz8873_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos) {
	/* Get private data entry */
	struct _dynamic_MAC_table_entry tentry;		/* The first entry to get size of the table */
	struct _dynamic_MAC_table_entry *pentry;	/* The first entry to get size of the table */
	struct _dynamic_MAC_table_entry *pmactbl;	/* Dynamic MAC table */
	int ret = 0;
	int tNum = 0;

	//printk(KERN_INFO "KSZ8873 Read\n");
	/* Read the first entry and get the size of the table */
	if(__read_MAC_table_entry(phy_device, 0, &tentry) == -1) {
		printk(KERN_INFO "KSZ8873: There are no valid entries in the dynamic MAC table\n");
		return 0;
	}
//	printk(KERN_INFO "KSZ8873: Number of valid entries in the dynamic MAC table: %d\n", tentry.table_size);
	if(tentry.table_size == 0) {
		return 0;
	}
	
	/* Allocate memory for the whole table and copy the first value there */
	pmactbl = (struct _dynamic_MAC_table_entry *)kmalloc(sizeof(struct _dynamic_MAC_table_entry)*tentry.table_size, GFP_KERNEL);
	if(pmactbl == NULL) {
		printk(KERN_ERR "KSZ8873: Unable to allocate memory the dynamic MAC table size: %d\n", tentry.table_size);
		return -ENOMEM;
	}
	pentry = pmactbl;
	/* Copy the first entry */
	memcpy(pentry, &tentry, sizeof(struct _dynamic_MAC_table_entry));
	pentry++;
	/* Get all of the entries */
	for(tNum = 1; tNum < tentry.table_size; tNum++) {
		__read_MAC_table_entry(phy_device, tNum, pentry);
		pentry++;
	}
	/* Get size of the table in bytes and adjust to the max amount user ascked */
	ret = tentry.table_size*sizeof(struct _dynamic_MAC_table_entry);
	if(ret > count) {
		ret = count;
	}
	/* Copy to user */
	if(copy_to_user(buf, pmactbl, ret) != 0) {
		ret = -EFAULT;
	}
	kfree(pmactbl);
	return ret;
}

/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 * KSZ8873 write call. USER API
*/
#if 0
static ssize_t ksz8873_write(struct file *file, char __user *buf,
				size_t count, loff_t *ppos) {
	printk(KERN_INFO "KSZ8873 Write\n");
	return -ENOSYS;
}
#endif

/* File Operations structure */
static const struct file_operations ksz8873_fops = {
	.owner		= THIS_MODULE,
	.read		= ksz8873_read,
	.unlocked_ioctl	= ksz8873_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= ksz8873_ioctl,
#endif
	.open		= ksz8873_open,
	.release	= ksz8873_release,
};


/**
 * ksz8873_probe - probe and init a Micrel port 3 device
 * @dev: device to probe and init
 *
 * Description: Take care of setting up the phy_device structure,
 *   set the state to READY (the driver's init function should
 *   set it to STARTING if needed).
 */
int ksz8873_probe(struct device *dev)
{
	struct phy_device *phydev;
	struct phy_driver *phydrv;
	struct device_driver *drv;
	int err = 0;

	printk(KERN_INFO "[PARALLAX]: KSZ8873 device probe\n");
	phydev = to_phy_device(dev);

	/* Make sure the driver is held. */
	drv = get_driver(phydev->dev.driver);
	phydrv = to_phy_driver(drv);
	phydev->drv = phydrv;

	/* Disable the interrupt if the PHY doesn't support it */
	if (!(phydrv->flags & PHY_HAS_INTERRUPT))
		phydev->irq = PHY_POLL;

	mutex_lock(&phydev->lock);

	/* Start out supporting everything. Eventually,
	 * a controller will attach, and may modify one
	 * or both of these values */
	phydev->supported = phydrv->features;
	phydev->advertising = phydrv->features;

	/* Set the state to READY by default */
	phydev->state = PHY_READY;

	if (phydev->drv->probe)
		err = phydev->drv->probe(phydev);

	mutex_unlock(&phydev->lock);
	/* -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-== */
	/* Now, we need to expose some functionality to user */
	/* Create character device driver for our PHY */
	ksz8873_cdev = cdev_alloc();
	if(ksz8873_cdev == NULL) {
		printk(KERN_ERR "KSZ8873: can't allocate memory for cdev\n");
		return -ENOMEM;
	}
	cdev_init(ksz8873_cdev, &ksz8873_fops);
	ksz8873_devno = MKDEV(KSZ8873_MAJOR, KSZ8873_MINOR);
	ksz8873_cdev->ops = &ksz8873_fops;
	ksz8873_cdev->owner = THIS_MODULE;

	err = cdev_add(ksz8873_cdev, ksz8873_devno, 1);
	if(err) {
		printk(KERN_ERR "KSZ8873: can't register camera device on major/minor = %d/%d\n",
		    KSZ8873_MAJOR, KSZ8873_MINOR);
		return -1;
	}
	/* Save phy device */
	phy_device = phydev;
	return err;

}
EXPORT_SYMBOL(ksz8873_probe);


/**
 * ksz8873_attach - attach a network device to a particular PHY device
 * @dev: network device to attach
 * @bus_id: PHY device to attach
 * @flags: PHY device's dev_flags
 * @interface: PHY device's interface
 *
 * Description: Called by drivers to attach to a particular PHY
 *     device. The phy_device is found, and properly hooked up
 *     to the phy_driver.  If no driver is attached, then the
 *     genphy_driver is used.  The phy_device is given a ptr to
 *     the attaching device, and given a callback for link status
 *     change.  The phy_device is returned to the attaching driver.
 */
struct phy_device *ksz8873_attach(struct net_device *dev,
		const char *bus_id, u32 flags, phy_interface_t interface)
{
	struct bus_type *bus = &mdio_bus_type;
	struct phy_device *phydev;
	struct device *d;
	int err;

	/* Search the list of PHY devices on the mdio bus for the
	 * PHY with the requested name */
//	d = bus_find_device_by_name(bus, NULL, bus_id);
	d = bus_find_device_by_name(bus, NULL, bus_id);
	if (d) {
		phydev = to_phy_device(d);
	} else {
		printk(KERN_ERR "%s not found\n", bus_id);
		return ERR_PTR(-ENODEV);
	}

	/* Assume that if there is no driver, that it doesn't
	 * exist, and we should use the genphy driver. */
	if (NULL == d->driver) {
		d->driver = &ksz8873_driver.driver;
//		err = ksz8873_probe(d);
		err = d->driver->probe(d);
		if (err >= 0)
			err = device_bind_driver(d);

		if (err)
			return ERR_PTR(err);
	}

	if (phydev->attached_dev) {
		printk(KERN_ERR "%s: %s already attached\n",
				dev->name, bus_id);
		return ERR_PTR(-EBUSY);
	}

	phydev->attached_dev = dev;

	phydev->dev_flags = flags;

	phydev->interface = interface;

	/* Do initial configuration here, now that
	 * we have certain key parameters
	 * (dev_flags and interface) */
	
	if (phydev->drv->config_init) {
		ksz8873_reset(phydev);
		/* Call generic PHY fixup function */
		err = phy_scan_fixups(phydev);

		if (err < 0) {
			printk(KERN_INFO "KSZ8873: Error PHY scan fixup\n");
			return ERR_PTR(err);
		}

		err = phydev->drv->config_init(phydev);

		if (err < 0) {
			printk(KERN_INFO "KSZ8873: Error Device driver config init\n");
			return ERR_PTR(err);
		}
	}

	return phydev;
}
EXPORT_SYMBOL(ksz8873_attach);

/**
 * ksz8873_detach - detach a PHY device from its network device
 * @phydev: target phy_device struct
 */
void ksz8873_detach(struct phy_device *phydev)
{
	phydev->attached_dev = NULL;

	/* If the device had no specific driver before (i.e. - it
	 * was using the generic driver), we unbind the device
	 * from the generic driver so that there's a chance a
	 * real driver could be loaded */
	if (phydev->dev.driver == &ksz8873_driver.driver)
		device_release_driver(&phydev->dev);
}
EXPORT_SYMBOL(ksz8873_detach);



/**
 * ksz8873_connect - connect an ethernet device to a PHY device
 * @dev: the network device to connect
 * @bus_id: the id string of the PHY device to connect
 * @handler: callback function for state change notifications
 * @flags: PHY device's dev_flags
 * @interface: PHY device's interface
 *
 * Description: Convenience function for connecting ethernet
 *   devices to PHY devices.  The default behavior is for
 *   the PHY infrastructure to handle everything, and only notify
 *   the connected driver when the link status changes.  If you
 *   don't want, or can't use the provided functionality, you may
 *   choose to call only the subset of functions which provide
 *   the desired functionality.
 */
struct phy_device *ksz8873_connect(struct net_device *dev, const char *bus_id,
		void (*handler)(struct net_device *), u32 flags,
		phy_interface_t interface)
{
	struct phy_device *phydev;
	/* 
	 * Attach our PHY interface to the calling device driver. 
	 * In our case Atmel's macb driver
	*/
	printk(KERN_INFO "KSZ8873: Device Attach\n");
	phydev = ksz8873_attach(dev, bus_id, flags, interface);

	if (IS_ERR(phydev)) {
		printk(KERN_INFO "KSZ8873: ERROR: Device Attach\n");
		return phydev;
	}

	printk(KERN_INFO "KSZ8873: Preparing link\n");
	ksz8873_prepare_link(phydev, handler);

	/* Call generic PHY function to start state machine for PHY interface */
	printk(KERN_INFO "KSZ8873: Start Machine\n");
	phy_start_machine(phydev, NULL);

	if (phydev->irq > 0) {
		printk(KERN_INFO "KSZ8873: Start IRQ: %d\n", phydev->irq);

		phy_start_interrupts(phydev);
	}
	printk(KERN_INFO "KSZ8873: Connect done\n");
	return phydev;
}
EXPORT_SYMBOL(ksz8873_connect);

/**
 * ksz8873_disconnect - disable interrupts, stop state machine, and detach a PHY device
 * @phydev: target phy_device struct
 */
void ksz8873_disconnect(struct phy_device *phydev)
{
	if (phydev->irq > 0)
		phy_stop_interrupts(phydev);

	phy_stop_machine(phydev);
	
	phydev->adjust_link = NULL;

	ksz8873_detach(phydev);
}
EXPORT_SYMBOL(ksz8873_disconnect);


/* KSZ8873RLL RMII PHY support and helper functions */

/*
* Configure advertisement of capabilities. Note that
* we can hardcode capability here for 100T and 10T
*/
int ksz8873_config_advert(struct phy_device *phydev)
{
	int changed = 0;

	/* 
	 * Only allow advertising what this PHY supports.
	*/
	phydev->advertising &= phydev->supported;

	return changed;
}
EXPORT_SYMBOL(ksz8873_config_advert);

/*
 * ksz8873_setup_forced - configures/forces speed/duplex from @phydev
 * @phydev: target phy_device struct
 *
 * Description: Configures SMI 8 bit registers to force speed/duplex
 *   to the values in phydev. Assumes that the values are valid.
 *   Please see phy_sanitize_settings().
 */
int ksz8873_setup_forced(struct phy_device *phydev)
{
	int err;
	u8 ctl = 0;
	u32 ctl32 = 0;

	phydev->pause = phydev->asym_pause = 0;
	/* Enable full duplex flow control */
//	ctl |= KSZ8873_CTRL4_FDFC;
	/* Set 10 base T speed to the global control register */
//	ctl |= KSZ8873_CTRL4_10BT;
	/* The rest of the bits are 0s for 10 BT and Full Duplex */
	err = phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL4, 0x0);
//	err = phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL4, 0x20);

	/* 
	 * OK, we've configured global control register which 
	 * applies to all three ports. Now we need to configure 
	 * two PHYs which connect to outside world. We need to set 
	 * 100 base T and advertise auto-negotiation, speed and duplex
	*/
        ctl32 |= (BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_ANENABLE);
	/* Write PHY function to BMCR register for both PHY IDs 1 and 2 */
        phy_write_ext(phydev, 1, MII_BMCR, ctl32);
       	phy_write_ext(phydev, 2, MII_BMCR, ctl32);

	return err;
}



/*
 * ksz8873_config_aneg - configure auto-negotiation
 * Description: If auto-negotiation is enabled, we configure the
 *   advertising, and then restart auto-negotiation.
 * If it is not enabled, we are setting registers direct with
 * predefined values.
 */
int ksz8873_config_aneg(struct phy_device *phydev)
{
	int result = 0;

	if (AUTONEG_ENABLE != phydev->autoneg)
		return ksz8873_setup_forced(phydev);

	return result;
}
EXPORT_SYMBOL(ksz8873_config_aneg);

/**
 * ksz8873_update_link - update link status in @phydev
 * @phydev: target phy_device struct
 *
 * Description: Update the value in phydev->link to reflect the
 *   current link value.  In order to do this, we need to read
 *   the status register twice, keeping the second value.
 */
int ksz8873_update_link(struct phy_device *phydev)
{
	/* Link is ALWAYS up for the Port 3 between this PHY and Atmel chip */
	phydev->link = 1;

	return 0;
}
EXPORT_SYMBOL(ksz8873_update_link);

/**
 * ksz8873_read_status - check the link status and update current link state
 * @phydev: target phy_device struct
 *
 * Do not check advertised configuration, we are hardcoded on the line from
 * Atmel to the Micrel chip. Set hardcoded values here
 */
int ksz8873_read_status(struct phy_device *phydev)
{
	int err;

	/* Once again, hardcode link and mode */
	err = ksz8873_update_link(phydev);
	if (err)
		return err;

//	phydev->speed = SPEED_10;
	phydev->duplex = DUPLEX_FULL;
//	100 BT generates frame errors so far. Lower speed to 10BT, full duplex
	phydev->speed = SPEED_100;
	phydev->pause = phydev->asym_pause = 0;

	return 0;
}
EXPORT_SYMBOL(ksz8873_read_status);



static int ksz8873_config_init(struct phy_device *phydev)
{
	u32 features;
	int adv = 0;
	u32 ctl = 0;

	printk("[PARALLAX]: KSZ8873 config init\n");
	features = SUPPORTED_MII;
	/* We do not support autoneg, hardcode couple of modes Micrel KSZ8873 supports */
	features |= SUPPORTED_100baseT_Full;
	features |= SUPPORTED_100baseT_Half;
	features |= SUPPORTED_10baseT_Full;
	features |= SUPPORTED_10baseT_Half;
	features |= SUPPORTED_Autoneg;
	features |= SUPPORTED_Pause;

	phydev->supported = features;
	phydev->advertising = features;

#if 0
	/* 
	* ATTN!!!
	* We are doing all configurations through the macb_smi_read and write funcitons 
	* we are no longer using our won bit-banging mechanism. Atmel PHY managmenet 
	* register works fine to transmit non standard SMI frames
	*/
	printk(KERN_INFO "<<Initializing KSZ8873 SMI interface>>\n");
	smi.parent = &phydev->dev;
	ksz8873_smi_init(&smi);

	ksz8873_smi_read_reg(&smi, 1, 0x01, &data);
	printk(KERN_INFO "++register 0: %X\n", data);
	ksz8873_smi_read_reg(&smi, 1, 0x02, &data);
	printk(KERN_INFO "+++register 1: %X\n", data);
	ksz8873_smi_read_reg(&smi, 1, 0x03, &data);
	printk(KERN_INFO "++register 2: %X\n", data);
#endif	
	/* Configure global interface and local port 3 RMII */
	/* Reset. None needed so far */
#if 0
        phy_write_smi(phydev, KSZ8873_SMI_RESET, 0x10);
        msleep(100);
        phy_write_smi(phydev, KSZ8873_SMI_RESET, 0x00);
#endif
	/* Disable LEDs for 2 PHY ports which apparently helps configure ports */
#if 0
        phy_write_smi(phydev, 0x1D, 0x80);
        msleep(1);
        phy_write_smi(phydev, 0x2D, 0x80);
        msleep(1);
        phy_write_smi(phydev, 0xC3, 0x30);
        msleep(1);
#endif
	/* Setup port 3 using hard-coded values */
	/* Global register 1 */
//        phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL1, 0x00);
	// FOR 100BT
        phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL1, 0x34);
        msleep(1);
//	phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL4, KSZ8873_CTRL4_10BT);
	/* Global register 4 - for 100/10 BT bit 5 */
	phy_write_smi(phydev, KSZ8873_SMI_GBL_CTRL4, 0x0);
        msleep(1);

	// KP: THese were enabled from Atmel processor ?????????/
//        phy_write_smi(phydev, 0x85, 0x12);
//        phy_write_smi(phydev, 0x87, 0x08);
//        phy_write_smi(phydev, 0x88, 0x08);

	// Changed back to internal clock
        phy_write_smi(phydev, 0xC6, 0x0B);

#if 0
	/* Configure other global registers if needed */
        phy_write_smi(phydev, KSZ8873_SMI_P3_CTRL2, 0x16);
        msleep(1);
        phy_write_smi(phydev, KSZ8873_SMI_P3_CTRL3, 0x00);
        msleep(1);
        phy_write_smi(phydev, KSZ8873_SMI_P3_CTRL5, 0x00);
        /* Read all registers and dump on the screen */
        for(i = 0; i <= 198; i++) {
                phy_read_smi(phydev, i);
        }
#endif
	/* 
	 * NOW, work with two PHY interfaces for outside world 
	 * using standard MII read of the PHY registers only.
	*/
        /* Setup standard advertisement */
        adv = phy_read(phydev, MII_ADVERTISE);

        if (adv < 0) {
		printk(KERN_INFO "KSZ8873: Error reading advertise register\n");
                return -1;
	}
	/* Advertise only what we need */
        adv |= ADVERTISE_10FULL;
        adv |= ADVERTISE_100FULL;
	adv |= ADVERTISE_PAUSE_CAP;
	/* Set advertise register for both PHYs */
        phy_write_ext(phydev, 1, MII_ADVERTISE, adv);
        phy_write_ext(phydev, 2, MII_ADVERTISE, adv);
	/* Set full speed and duplex for outside world */
        ctl |= (BMCR_FULLDPLX | BMCR_SPEED100 | BMCR_ANENABLE);
	/* Set PHY control register */
        phy_write_ext(phydev, 1, MII_BMCR, ctl);
        phy_write_ext(phydev, 2, MII_BMCR, ctl);

        /* Perform VCT on both PHYs */
        phy_write_smi(phydev, 0x1A, 0x10);
        phy_write_smi(phydev, 0x2A, 0x10);

	return 0;
}


int ksz8873_suspend(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);
	/* I don't know if I can access power down of port 3 phy through the SMI */
	/* TODO: suspend both PHY1 and 2 here */
	mutex_unlock(&phydev->lock);

	return 0;
}
EXPORT_SYMBOL(ksz8873_suspend);


int ksz8873_resume(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);

	/* I don't know if I can access power up of port 3 phy through the SMI */
	/* TODO: suspend both PHY1 and 2 here */
	mutex_unlock(&phydev->lock);

	return 0;
}
EXPORT_SYMBOL(ksz8873_resume);


static int ksz8873_remove(struct device *dev)
{
	struct phy_device *phydev;

	phydev = to_phy_device(dev);

	mutex_lock(&phydev->lock);
	phydev->state = PHY_DOWN;
	mutex_unlock(&phydev->lock);

	if (phydev->drv->remove)
		phydev->drv->remove(phydev);

	put_driver(dev->driver);
	phydev->drv = NULL;

	return 0;
}

/**
 * ksz8873_driver_register - register a phy_driver with the PHY layer
 * @new_driver: new phy_driver to register
 */
int ksz8873_driver_register(struct phy_driver *new_driver)
{
	int retval;

	/* Set driver parameters and functions */
	new_driver->driver.name = new_driver->name;
	new_driver->driver.bus = &mdio_bus_type;
	new_driver->driver.probe = ksz8873_probe;
	new_driver->driver.remove = ksz8873_remove;

	retval = driver_register(&new_driver->driver);

	if (retval) {
		printk(KERN_ERR "%s: Error %d in registering driver\n",
				new_driver->name, retval);

		return retval;
	}

	pr_debug("%s: Registered new driver\n", new_driver->name);
	mac_table_choice = KSZ8873_MAC_TABLE_DYNAMIC;
	return 0;
}
EXPORT_SYMBOL(ksz8873_driver_register);

void ksz8873_driver_unregister(struct phy_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL(ksz8873_driver_unregister);

/* Our device structure and control functions to execute */
static struct phy_driver ksz8873_driver = {
	.phy_id		= PHY_ID_KSZ8873,
	.phy_id_mask	= 0x00fffff0,
	.name		= "KSZ8873RLL",
	.config_init	= ksz8873_config_init,
	.features	= 0,
	.config_aneg	= ksz8873_config_aneg,
	.read_status	= ksz8873_read_status,
	.suspend	= ksz8873_suspend,
	.resume		= ksz8873_resume,
	.driver		= {.owner = THIS_MODULE, },
};

static int __init ksz8873_init(void)
{
	int rc;
	/* Call our register function */
	rc = ksz8873_driver_register(&ksz8873_driver);
	return rc;
}

static void __exit ksz8873_exit(void)
{
	ksz8873_driver_unregister(&ksz8873_driver);
}

module_init(ksz8873_init);
module_exit(ksz8873_exit);

MODULE_AUTHOR("Konstantyn Prokopenko <kp@parkassist.com>");
MODULE_DESCRIPTION("KSZ8873RLL driver");
MODULE_LICENSE("GPL");


