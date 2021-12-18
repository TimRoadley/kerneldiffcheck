/* KSZ8873 RMII PHY specific definitions for KSZ8873RLL chip. 
  * 
  * Copyright (C) 2010 ParkAssist Inc.  Konstantyn Prokopenko 
  *
  * This program is free software; you can redistribute it and/or modify it 
  * under the terms of the GNU General Public License version 2 as published 
  * by the Free Software Foundation. 
  */ 
  
#ifndef __KSZ8873_HEAD_H__
#define __KSZ8873_HEAD_H__
  
#include <linux/phy.h> 
#include <linux/ethtool.h>

#define KSZ8873_MAJOR 121
#define KSZ8873_MINOR 0 
#define PHY_ID_KSZ8873    0x00221430
#define DRV_NAME    "Micrel KSZ8873RLL"

#define KSZ8873_NUM_PORTS 3

/* SMI 8 bit registers */
/* Global ID registers */
#define KSZ8873_SMI_ID_REG0     0x00
#define KSZ8873_SMI_ID_REG1     0x01
/* Global control registers */
#define KSZ8873_SMI_GBL_CTRL0   0x02
#define KSZ8873_SMI_GBL_CTRL1   0x03
#define KSZ8873_SMI_GBL_CTRL2   0x04
#define KSZ8873_SMI_GBL_CTRL3   0x05
#define KSZ8873_SMI_GBL_CTRL4   0x06
#define KSZ8873_SMI_GBL_CTRL5   0x07
#define KSZ8873_SMI_GBL_CTRL6   0x08
#define KSZ8873_SMI_GBL_CTRL7   0x09
#define KSZ8873_SMI_GBL_CTRL8   0x0a
#define KSZ8873_SMI_GBL_CTRL9   0x0b
#define KSZ8873_SMI_GBL_CTRL10  0x0c
#define KSZ8873_SMI_GBL_CTRL11  0x0d
#define KSZ8873_SMI_GBL_CTRL12  0x0e
#define KSZ8873_SMI_GBL_CTRL13  0x0f

/* Global control register 4 bits */
#define KSZ8873_CTRL4_10BT      (1 << 4)        /* Switch is in 10 Base T */
#define KSZ8873_CTRL4_FDFC      (1 << 5)        /* Full duplex flow control */
#define KSZ8873_CTRL4_HD        (1 << 6)        /* Half duplex mode */

/* Port 3 control registers */
#define KSZ8873_SMI_P3_CTRL0    0x30
#define KSZ8873_SMI_P3_CTRL1    0x31
#define KSZ8873_SMI_P3_CTRL2    0x32
#define KSZ8873_SMI_P3_CTRL3    0x33
#define KSZ8873_SMI_P3_CTRL4    0x34
#define KSZ8873_SMI_P3_CTRL5    0x35
#define KSZ8873_SMI_P3_CTRL6    0x36
#define KSZ8873_SMI_P3_CTRL7    0x37
#define KSZ8873_SMI_P3_CTRL8    0x38
#define KSZ8873_SMI_P3_CTRL9    0x39

/* Port 3 status register */
#define KSZ8873_SMI_P3_STAT     0x3f

/* Reset register */
#define KSZ8873_SMI_RESET 0x43

#define DEFAULT_PORT_VID  0
#define KSZ8873_TAG_INSERTION 0x04
#define KSZ8873_START_SWITCH 0x01


/* MII Basic Control */
#define SOFT_RESET        0x8000
#define LOOPBACK          0x4000
#define FORCE_100         0x2000
#define AN_ENABLE         0x1000
#define POWER_DOWN        0x0800
#define ISOLATE           0x0400
#define RESTART_AN        0x0200
#define FORCE_FULL_DUPLEX 0x0100
#define COLLISION_TEST    0x0080
/* Bit Reserved */
#define HP_MDIX           0x0020
#define Force_MDI         0x0010
#define DISABLE_MDIX      0x0008
#define DIS_FAR_END_FAULT 0x0004
#define DISABLE_TRANSMIT  0x0002
#define DISABLE_LED       0x0001
  
/* MII Basic Status */
#define T4_CAPABLE        0x8000
#define FULL_100_CAPABLE  0x4000
#define HALF_100_CAPABLE  0x2000
#define FULL_10_CAPABLE   0x1000
#define HALF_10_CAPABLE   0x0800
/* 4 Bits Reserved */
#define PREAMBLE_SUPPRESS 0x0040
#define AN_COMPLETE       0x0020
#define FAR_END_FAULT     0x0010
#define AN_CAPABLE        0x0008
#define LINK_STATUS       0x0004
#define JABBER_TEST       0x0002
#define EXTENDED_CAPABLE  0x0001
  
/* Auto-Negotiation Advertisement Ability */
#define NEXT_PAGE         0x8000
/* Bit Reserved */
#define REMOTE_FAULT      0x2000
/* 2 Bits Reserved */
#define PAUSE             0x0400
/* Bit Reserved */
#define ADV_100_FULL      0x0100
#define ADV_100_HALF      0x0080
#define ADV_10_FULL       0x0040
#define ADV_10_HALF       0x0020
#define SELECTOR_FIELD    0x001F
  
/* Auto-Negotiation Link Partner Ability */
#define NEXT_PAGE         0x8000
#define LP_ACK            0x4000
#define REMOTE_FAULT      0x2000
/* 2 Bits Reserved */
#define PAUSE             0x0400
/* Bit Reserved */
#define ADV_100_FULL      0x0100
#define ADV_100_HALF      0x0080
#define ADV_10_FULL       0x0040
#define ADV_10_HALF       0x0020
/* 5 Bits Reserved */

/* LinkMD Control/Status */
#define VCT_ENABLE        0x8000
#define VCT_RESULT        0x6000
#define VCT_10M_SHORT     0x1000
/* 3 Bits Reserved */
#define VCT_FAULT_COUNT   0x01FF

/* PHY Special Control/Status */
/* 10 Bits Reserved */
#define POLRVS            0x0020
#define MDI_X_STATUS      0x0010
#define FORCE_LNK         0x0008
#define PWRSAVE           0x0004
#define REMOTE_LOOPBACK   0x0002



enum KSZ8873_PORT_CONTROL_REGS {
  KSZ8873_PORT_CONTROL0 = 0,
  KSZ8873_PORT_CONTROL1,
  KSZ8873_PORT_CONTROL2,
  KSZ8873_PORT_CONTROL3,
  KSZ8873_PORT_CONTROL4,
  KSZ8873_PORT_CONTROL5
};


enum KSZ8873_MAC_TABLE_TYPES {
	KSZ8873_MAC_TABLE_DYNAMIC,
	KSZ8873_MAC_TABLE_STATIC
};

struct ksz8873_hw_stat {
  char string[ETH_GSTRING_LEN];
  int sizeof_stat;
  int reg;
};


struct phy_device *ksz8873_get_phy_device(struct mii_bus *bus, int addr);
struct phy_device *ksz8873_attach(struct net_device *dev,
                const char *bus_id, u32 flags, phy_interface_t interface);
int ksz8873_probe(struct device *dev);
void ksz8873_detach(struct phy_device *phydev);
struct phy_device * ksz8873_connect(struct net_device *dev, const char *bus_id,
                void (*handler)(struct net_device *), u32 flags,
                phy_interface_t interface);
void ksz8873_disconnect(struct phy_device *phydev);
void ksz8873_detach(struct phy_device *phydev);

//int __init ksz8873_dsa_init(void);
//void __exit ksz8873_dsa_cleanup(void);

#endif 

