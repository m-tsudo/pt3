/*******************************************************************************
   earthsoft PT3 Linux driver

   This program is free software; you can redistribute it and/or modify it
   under the terms and conditions of the GNU General Public License,
   version 3, as published by the Free Software Foundation.

   This program is distributed in the hope it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   The full GNU General Public License is included in this distribution in
   the file called "COPYING".

 *******************************************************************************/

#ifndef __PT3_IOCTL_H
#define __PT3_IOCTL_H

#include <asm/types.h>

typedef	struct	_frequency{
	int frequencyno;
	int slot;
} FREQUENCY;

#define		SET_CHANNEL	_IOW(0x8d, 0x01, FREQUENCY)
#define		START_REC	_IO(0x8d, 0x02)
#define		STOP_REC	_IO(0x8d, 0x03)
#define		GET_SIGNAL_STRENGTH	_IOR(0x8d, 0x04, int *)
#define		LNB_ENABLE	_IOW(0x8d, 0x05, int)
#define		LNB_DISABLE	_IO(0x8d, 0x06)
#define		GET_STATUS _IOR(0x8d, 0x07, int *)
#define		SET_TEST_MODE_ON _IO(0x8d, 0x08)
#define		SET_TEST_MODE_OFF _IO(0x8d, 0x09)
#define		GET_TS_ERROR_PACKET_COUNT _IOR(0x8d, 0x0a, unsigned int *)

#endif
