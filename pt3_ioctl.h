#ifndef __PT3_IOCTL_H
#define __PT3_IOCTL_H

enum LAYER_INDEX {
	LAYER_INDEX_L = 0,
	LAYER_INDEX_H,

	LAYER_INDEX_A = 0,
	LAYER_INDEX_B,
	LAYER_INDEX_C
};

enum LAYER_COUNT {
	LAYER_COUNT_S = LAYER_INDEX_H + 1,
	LAYER_COUNT_T = LAYER_INDEX_C + 1,
};

typedef struct __TMCC_S {
	__u32 indicator;
	__u32 mode[4];
	__u32 slot[4];
	__u32 id[8];
	__u32 emergency;
	__u32 uplink;
	__u32 extflag;
	__u32 extdata[2];
} TMCC_S;

typedef struct _TMCC_T {
	__u32 system;
	__u32 indicator;
	__u32 emergency;
	__u32 partial;
	__u32 mode[LAYER_COUNT_T];
	__u32 rate[LAYER_COUNT_T];
	__u32 interleave[LAYER_COUNT_T];
	__u32 segment[LAYER_COUNT_T];
	__u32 phase;
	__u32 reserved;
} TMCC_T;

typedef	struct	_frequency{
	int channel;
}FREQUENCY;

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
#define		GET_TMCC_S _IOR(0x8d, 0x0b, TMCC_S *)
#define		GET_TMCC_T _IOR(0x8d, 0x0c, TMCC_T *)

#endif
