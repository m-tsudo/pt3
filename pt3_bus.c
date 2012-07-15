#include "version.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "pt3_com.h"
#include "pt3_pci.h"
#include "pt3_bus.h"

enum {
	I_END,
	I_ADDRESS,
	I_CLOCK_L,
	I_CLOCK_H,
	I_DATA_L,
	I_DATA_H,
	I_RESET,
	I_SLEEP,	// Sleep 1ms
	I_DATA_L_NOP  = 0x08,
	I_DATA_H_NOP  = 0x0c,
	I_DATA_H_READ = 0x0d,
	I_DATA_H_ACK0 = 0x0e,
	I_DATA_H_ACK1 = 0x0f,

	// テスト用
	_I_DATA_L_READ = I_DATA_H_READ ^ 0x04
//	_I_DATA_L_ACK0 = I_DATA_H_ACK0 ^ 0x04,
//	_I_DATA_L_ACK1 = I_DATA_H_ACK1 ^ 0x04
};

static void
add_instruction(PT3_BUS *bus, __u32 instruction)
{
	if ((bus->inst_count % 2) == 0) {
		bus->tmp_inst = instruction;
	} else {
		bus->tmp_inst |= instruction << 4;
	}

	if (bus->inst_count % 2) {
#if 0
		printk(KERN_DEBUG "PT3 : add_instruction %d %p %x",
						bus->inst_count, priv->s, tmp_inst);
#endif
		bus->insts[bus->inst_pos] = bus->tmp_inst;
		bus->inst_pos++;
		if (bus->inst_pos >= sizeof(bus->insts)) {
			printk(KERN_ERR "PT3:bus instructions is over flow");
			bus->inst_pos = 0;
		}
	}
	bus->inst_count++;
}

static __u32
datan(PT3_BUS *bus, size_t index, __u32 n)
{
	__u32 i, data;

	if (bus->buf == NULL) {
		printk(KERN_ERR "PT3: buf is not ready.");
		return 0;
	}
	if (index + n < bus->buf_size) {
		printk(KERN_ERR "PT3: buf does not  have enough size.");
		return 0;
	}

	data = 0;
	for (i = 0; i < n; i++) {
		data <<= 8;
		data |= bus->buf[index + i];
	}

	return data;
}

void
pt3_bus_start(PT3_BUS *bus)
{
	add_instruction(bus, I_DATA_H);
	add_instruction(bus, I_CLOCK_H);
	add_instruction(bus, I_DATA_L);
	add_instruction(bus, I_CLOCK_L);
}

void
pt3_bus_stop(PT3_BUS *bus)
{
	//add_instruction(bus, I_CLOCK_L);
	add_instruction(bus, I_DATA_L);
	add_instruction(bus, I_CLOCK_H);
	add_instruction(bus, I_DATA_H);
}

void
pt3_bus_write(PT3_BUS *bus, const __u8 *data, __u32 size)
{
	__u32 i, j;
	__u8 byte;

	for (i = 0; i < size; i++) {
		byte = data[i];
		for (j = 0; j < 8; j++) {
			add_instruction(bus, BIT_SHIFT_MASK(byte, 7 - j, 1) ?
									I_DATA_H_NOP : I_DATA_L_NOP);
		}
		add_instruction(bus, I_DATA_H_ACK0);
	}
}

size_t
pt3_bus_read(PT3_BUS *bus, __u8 *data, __u32 size)
{
	__u32 i, j;
	size_t index;

	for (i = 0; i < size; i++) {
		for (j = 0; j < 8; j++) {
			add_instruction(bus, I_DATA_H_READ);
		}

		if (i == (size - 1))
			add_instruction(bus, I_DATA_H_NOP);
		else
			add_instruction(bus, I_DATA_L_NOP);
	}
	index = bus->read_addr;
	bus->read_addr += size;
	if (bus->buf == NULL) {
		bus->buf = data;
		bus->buf_pos = 0;
		bus->buf_size = size;
	} else
		printk(KERN_ERR "PT3: bus read buff is already exists.");

	return index;
}

void
pt3_bus_push_read_data(PT3_BUS *bus, __u8 data)
{
	if (bus->buf != NULL) {
		if (bus->buf_pos >= bus->buf_size) {
			printk(KERN_ERR "PT3: buffer over run. pos=%d", bus->buf_pos);
			bus->buf_pos = 0;
		}
		bus->buf[bus->buf_pos] = data;
		bus->buf_pos++;
	}
#if 1
	printk(KERN_DEBUG "bus read data=0x%02x", data);
#endif
}

__u8
pt3_bus_data1(PT3_BUS *bus, size_t index)
{
	return (__u8)datan(bus, index, 1);
}

void
pt3_bus_sleep(PT3_BUS *bus, __u32 ms)
{
	__u32 i;
	for (i = 0; i< ms; i++)
		add_instruction(bus, I_SLEEP);
}

void
pt3_bus_end(PT3_BUS *bus)
{
	add_instruction(bus, I_END);

	if (bus->inst_count % 2)
		add_instruction(bus, I_END);
}

void
pt3_bus_reset(PT3_BUS *bus)
{
	add_instruction(bus, I_RESET);
}

PT3_BUS *
create_pt3_bus(void)
{
	PT3_BUS *bus;

	bus = vzalloc(sizeof(PT3_BUS));
	if (bus == NULL)
		goto fail;

	bus->inst_addr = 0;
	bus->read_addr = 0;
	bus->inst_count = 0;
	bus->tmp_inst = 0;
	bus->inst_pos = 0;
	bus->buf = NULL;

	return bus;
fail:
	if (bus != NULL)
		free_pt3_bus(bus);
	return NULL;
}

void
free_pt3_bus(PT3_BUS *bus)
{
	vfree(bus);
}
