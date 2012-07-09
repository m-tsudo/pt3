#ifndef		__PT3_COM_H__
#define		__PT3_COM_H__

#define BIT_SHIFT_MASK(value, shift, mask) (((value) >> (shift)) & ((1<<(mask))-1))

#define		MAX_TUNER			2		//チューナ数
#define		MAX_CHANNEL			4		// チャネル数
#define		FALSE		0
#define		TRUE		1

enum {
	PT3_ISDB_S,
	PT3_ISDB_T,
};

#endif
