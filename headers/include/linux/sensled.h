/* 
 * sensled.h
 *
 * ParkAssist LED interface device user definitions
 *
 * This driver controls LED interface for ParkAssist Sensor board and allows to open/close and ioctl the device. The ioctl 
 * call includes means of controlling LED parameters such as brightness, colors, etc...
 *
 * Copyright 2010 (c) ParkAssist
 *
 * Konstantyn Prokopenko
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version. NO WARRANTY OF ANY KIND is expressed or implied.
 *
 */

#ifndef __LINUX_SENSLED_H
#define __LINUX_SENSLED_H

#ifdef __cplusplus
extern "C" 
{
#endif

/* IOCTL calls */
#define SENSLED_IOCTL_ON _IO('L', 0x10)
#define SENSLED_IOCTL_OFF _IO('L', 0x11)
#define SENSLED_IOCTL_SET_BRIGHTNESS _IO('L', 0x12)
#define SENSLED_IOCTL_GET_BRIGHTNESS _IO('L', 0x13)
#define SENSLED_IOCTL_SET_COLOR _IOW('L', 0x14, struct sensled_color_config)
#define SENSLED_IOCTL_GET_COLOR _IOR('L', 0x15, struct sensled_color_config)

#define USER_MAX_BRIGHTNESS 100

struct sensled_color_config{
	int red;
	int green;
	int blue;
	int brightness;
};


#ifdef __cplusplus
}
#endif
#endif


