/* 
 *pasrc. h
 *
 * ParkAssist SRC interface device user definitions
 *
 * Copyright 2016 (c) ParkAssist
 *
 * Konstantyn Prokopenko
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version. NO WARRANTY OF ANY KIND is expressed or implied.
 *
 */

#ifndef __LINUX_SRC_H
#define __LINUX_SRC_H

#ifdef __cplusplus
extern "C" 
{
#endif

#define PASRC_SCR 	0x00
#define PASRC_SBMR1 	0x04
#define PASRC_SRSR 	0x08
#define PASRC_SISR 	0x14
#define PASRC_SIMR 	0x18
#define PASRC_SBMR2 	0x1C
#define PASRC_GPR1 	0x20
#define PASRC_GPR2 	0x24
#define PASRC_GPR3 	0x28
#define PASRC_GPR4 	0x2C
#define PASRC_GPR5 	0x30
#define PASRC_GPR6 	0x34
#define PASRC_GPR7 	0x38
#define PASRC_GPR8 	0x3C
#define PASRC_GPR9 	0x40
#define PASRC_GPR10 	0x44

/* IOCTL calls */
#define PASRC_IOCTL_READ  _IOWR('S', 0x10, struct pasrc_control)
#define PASRC_IOCTL_WRITE _IOWR('S', 0x11, struct pasrc_control)


struct pasrc_control{
	unsigned int reg; /* SRC register */
	unsigned int val; /* SRC register value */
};


#ifdef __cplusplus
}
#endif
#endif


