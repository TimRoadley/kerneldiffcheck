/* KSZ8873 PHY user header
  * 
  * Copyright (C) 2010 ParkAssist Inc.  Konstantyn Prokopenko 
  *
  * This program is free software; you can redistribute it and/or modify it 
  * under the terms of the GNU General Public License version 2 as published 
  * by the Free Software Foundation. 
  */ 
  
#ifndef __KSZ8873_PHY_H__
#define __KSZ8873_PHY_H__

/* IOCTL calls */
/* Enable or disable PHY. Accepts PHY ID: 1 or 2 */
#define KSZ8873_IOCTL_PHY_ENABLE	 			_IOW('K', 0x10, int)
#define KSZ8873_IOCTL_PHY_DISABLE 				_IOW('K', 0x11, int)
#define KSZ8873_IOCTL_TAILTAG_ENABLE 			_IO('K', 0x12)
#define KSZ8873_IOCTL_TAILTAG_DISABLE 			_IO('K', 0x13)
#define KSZ8873_IOCTL_SET_SNIFFING 				_IOW('K', 0x14, struct _port_sniffer_control)
#define KSZ8873_IOCTL_SET_STP 					_IOW('K', 0x15, struct _port_stp_control)
/* Set dynamic or static MAC table processing */
#define KSZ8873_IOCTL_CHOOSE_DYNMAC 			_IO('K', 0x16)
#define KSZ8873_IOCTL_CHOOSE_STATMAC 			_IO('K', 0x17)
/* Connection status */
#define KSZ8873_IOCTL_PHY_GET_LINK_STAT 		_IOW('K', 0x18, int)
/* SNMP MIB support */
#define KSZ8873_IOCTL_GET_PHY_MIB_COUNTERS 		_IOW('K', 0x19, struct KSZ8873_MIB_entry)
/* PHY reset/reinit calls */
#define KSZ8873_IOCTL_PHY_RESET 				_IO('K', 0x1A)
#define KSZ8873_IOCTL_PHY_REINIT 				_IO('K', 0x1B)
#define KSZ8873_IOCTL_PHY_WRITE_SMI_REGISTER	_IOW('K', 0x1C, int)
#define KSZ8873_IOCTL_PHY_READ_SMI_REGISTER		_IOW('K', 0x1D, int)
#define KSZ8873_IOCTL_PHY_READ_MIIM_REGISTER	_IOW('K', 0x1E, int)
#define KSZ8873_IOCTL_SNAPSHOT                  _IOW('K', 0x1F, struct KSZ8873_snapshot_entry)

#define KSZ8873_FWD_MASK_TO_1	0x01	/* Forwarding to port 1 */
#define KSZ8873_FWD_MASK_TO_2	0x02	/* Forwarding to port 2 */
#define KSZ8873_FWD_MASK_TO_3	0x04	/* Forwarding to port 3 */
#define KSZ8873_FWD_MASK_BCAST	0x07	/* Broadcasting port */

struct _dynamic_MAC_table_entry {
        unsigned int MAC0;		/* Lower 4 bytes of MAC */
	unsigned short MAC1;		/* Upper 2 bytes of MAC */
	unsigned short table_size;	/* Table size */
	unsigned char FID;		/* Filter ID */
	unsigned char source_port;	/* Port source: 0 - port 1,  1 - port 2, 2 - port 3*/
	unsigned char aging;		/* MAC address aging */
	unsigned char valid;		/* Valid MAC */
};

struct _static_MAC_table_entry {
        unsigned int MAC0;		/* Lower 4 bytes of MAC */
	unsigned short MAC1;		/* Upper 2 bytes of MAC */
	unsigned char FID;		/* Filter ID */
	unsigned char forward_mask;	/* Farwarding mask */
};


/* PHY port sniffing */
struct _port_sniffer_control {
	unsigned char enable;		/* Enable/disable sniffing  */
	unsigned char port_monitor;	/* Port ID to set as monitor*/
	unsigned char port_rx_fwd;	/* RX packets forwarding port */
	unsigned char port_tx_fwd;	/* TX packets forwarding port  */
};

/* Spanning Tree control */
struct _port_stp_control {
	unsigned char portID;
	unsigned char learning_enable;	/* Learning enable/disable */
	unsigned char rx_enable;	/* Receive enable */
	unsigned char tx_enable;	/* Transmit enable */
};


/* Complete MIB entry with all counters */
struct KSZ8873_MIB_entry {
	unsigned int portID;		/* Port ID of the counters */
	unsigned int rx_lo_prio_byte;	/* RX lo-priority (default) octet count including bad packets */
	unsigned int rx_hi_prio_byte;	/* RX hi-priority octet count including bad packets */
	unsigned int rx_undersize;	/* RX undersize packets w/ good CRC */
	unsigned int rx_fragments;	/* RX fragment packets w/ bad CRC, symbol or alignment errors */
	unsigned int rx_oversize;	/* RX oversize packets w/ good CRC */
	unsigned int rx_jabbers;	/* RX packets longer than 1522 bytes w/ either CRC errors, 
					   alignment errors or symbol errors */
	unsigned int rx_symbol_err;	/* RX packets w/ invalid data symbol and legal packet size */
	unsigned int rx_crc_err;	/* RX packets w/ integral number of bytes and a bad CRC */
	unsigned int rx_alignment_err;	/* RX w/  a non-integral number of bytes and a bad CRC */
	unsigned int rx_control_8808;	/* Number of MAC control frames received with 99-08h in EtherType field */
	unsigned int rx_pause;		/* Number of PAUSE frames received */
	unsigned int rx_broadcast;	/* RX good broadcast packets */
	unsigned int rx_multicast;	/* RX good multicast packets */
	unsigned int rx_unicast;	/* RX good unicast packets */
	unsigned int rx_64_octets;	/* Total RX packets that were 64 octets in length */
	unsigned int rx_65to127_octets; /* Total RX packets that were between 65 and 127 octets in length */
	unsigned int rx_128to255_octets;/* Total RX packets that were between 128 and 255 octets in length */
	unsigned int rx_256to511_octets;/* Total RX packets that were between 256 and 511 octets in length */
	unsigned int rx_512to1023_octets;/* Total RX packets that were between 512 and 1023 octets in length */
	unsigned int rx_1024to1522_octets;/* Total RX packets that were between 1024 and 1522 octets in length */
	unsigned int tx_lo_prio_byte;	/* TX lo-priority good octet count including PAUSE packets */
	unsigned int tx_hi_prio_byte;	/* TX hi-priority good octet count including PAUSE packets */
	unsigned int tx_late_collision;	/* Number of times a collision is detected later than 512 bit-times 
					   into the Txof a packet  */
	unsigned int tx_pause;		/* Number of PAUSE frames transmitted */
	unsigned int tx_broadcast;	/* broadcast packets */
	unsigned int tx_multicast;	/* multicast packets */
	unsigned int tx_unicast;	/* unicast packets */
	unsigned int tx_deferred;	/* Packets for a port for which the 1st attempt to TX is delayed 
					   due to busy medium  */
	unsigned int tx_total_collision;/* TX total collision for half duplex only */
	unsigned int tx_exces_collision;/* Number of frames failed due to excessive collision */
	unsigned int tx_single_collision; /* Successfully TX frames on a port for which TX is inhibited 
					   by exactly one collision */
	unsigned int tx_multi_collision; /* Successfully TX frames on a port for which TX is inhibited 
					   by more than one one collision */
};

#define KSZ8873_PHY_IFACE_NUMBER 3
/* Snapshot global bits */
#define KSZ8873_SNAPSHOT_BIT_PASS_FLOW_CTRL				0x01
#define KSZ8873_SNAPSHOT_BIT_PASS_ALL_FRAMES			0x02
#define KSZ8873_SNAPSHOT_BIT_AGING_ENBL					0x04
#define KSZ8873_SNAPSHOT_BIT_FAST_AGING_ENBL			0x08
#define KSZ8873_SNAPSHOT_BIT_MBPS_MODE 					0x10
#define KSZ8873_SNAPSHOT_BIT_FULL_DUPLEX_FLOW_CTRL		0x20
#define KSZ8873_SNAPSHOT_BIT_DUPLEX_MODE				0x40
/* Snapshot bits per PHY interface */
#define KSZ8873_SNAPSHOT_BIT_PHY_LINK_GOOD				0x0001
#define KSZ8873_SNAPSHOT_BIT_STORM_PROTECTION			0x0002
#define KSZ8873_SNAPSHOT_BIT_FORCE_FLOW_CTRL			0x0004
#define KSZ8873_SNAPSHOT_BIT_BACK_PRESSURE_ENBL			0x0008
#define KSZ8873_SNAPSHOT_BIT_TRANSMIT_ENBL				0x0010
#define KSZ8873_SNAPSHOT_BIT_RECEIVE_ENBL				0x0020
#define KSZ8873_SNAPSHOT_BIT_LEARNING_ENBL				0x0040
#define KSZ8873_SNAPSHOT_BIT_AUTO_NEG_ENBL				0x0080
#define KSZ8873_SNAPSHOT_BIT_FORCE_SPEED				0x0100
#define KSZ8873_SNAPSHOT_BIT_FORCE_DUPLEX				0x0200
#define KSZ8873_SNAPSHOT_BIT_POWER_DOWN					0x0400
#define KSZ8873_SNAPSHOT_BIT_AUTO_NEG_DONE				0x0800
#define KSZ8873_SNAPSHOT_BIT_OPERATION_DUPLEX			0x1000
#define KSZ8873_SNAPSHOT_BIT_OPERATION_SPEED			0x2000
#define KSZ8873_SNAPSHOT_BIT_RECEIVE_FLOW_CTRL_STATUS	0x4000
#define KSZ8873_SNAPSHOT_BIT_TRANSMIT_FLOW_CTRL_STATUS	0x8000

/* Snapshot entry */
struct KSZ8873_snapshot_entry {
	unsigned char global_control;
	unsigned short phy_control[KSZ8873_PHY_IFACE_NUMBER];
	unsigned short default_tag[KSZ8873_PHY_IFACE_NUMBER];
	unsigned char vct_result[2];
	unsigned short reserved;
}__attribute__((packed));


#endif

