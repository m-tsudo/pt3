#ifndef __PT3_IOCTL_H
#define __PT3_IOCTL_H

typedef	struct	_frequency{
	int		frequencyno ;			// ¼þÇÈ¿ô¥Æ¡¼¥Ö¥ëÈÖ¹æ
	int		slot ;					// ¥¹¥í¥Ã¥ÈÈÖ¹æ¡¿²Ã»»¤¹¤ë¼þÇÈ¿ô
}FREQUENCY;

#define		SET_CHANNEL	_IOW(0x8d, 0x01, FREQUENCY)
#define		START_REC	_IO(0x8d, 0x02)
#define		STOP_REC	_IO(0x8d, 0x03)
#define		GET_SIGNAL_STRENGTH	_IOR(0x8d, 0x04, int *)
#define		LNB_ENABLE	_IOW(0x8d, 0x05, int)
#define		LNB_DISABLE	_IO(0x8d, 0x06)
#define		GET_STATUS _IOR(0x8d, 0x07, int *)
#define		SET_TEST_MODE _IO(0x8d, 0x08)

#endif
