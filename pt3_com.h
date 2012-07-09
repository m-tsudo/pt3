#ifndef		__PT3_COM_H__
#define		__PT3_COM_H__

#define BIT_SHIFT_MASK(value, shift, mask) (((value) >> (shift)) & ((1<<(mask))-1))

#define		MAX_TUNER			2		//チューナ数
#define		MAX_CHANNEL			4		// チャネル数
#define		FALSE		0
#define		TRUE		1

typedef struct _BAR {
	unsigned long	mmio_start ;
	__u32			mmio_len ;
	void __iomem		*regs;
} BAR;

enum {
	PT3_ISDB_S,
	PT3_ISDB_T,
};

enum {
	PT3_TS_PIN_MODE_NORMAL,
	PT3_TS_PIN_MODE_LOW,
	PT3_TS_PIN_MODE_HIGH,
};

typedef struct _TS_PINS_MODE {
	int clock_data;
	int byte;
	int valid;
} TS_PINS_MODE;

typedef struct _TS_PINS_LEVEL {
	int clock;
	int data;
	int byte;
	int valid;
} TS_PINS_LEVEL;

#endif
