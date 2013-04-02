TARGET := pt3_drv.ko
VERBOSITY = 0
REL_VERSION = "0.0.1"
REL_DATE = "2012-07-07"
EXTRA_CFLAGS += -Wformat=2
KVER ?= `uname -r`

KBUILD = /lib/modules/$(KVER)/build
INSTALL_DIR = /lib/modules/$(KVER)/kernel/drivers/video

all: ${TARGET}

pt3_drv.ko: pt3_pci.c pt3_bus.c pt3_i2c.c pt3_tc.c pt3_qm.c pt3_mx.c pt3_dma.c version.h
	make -C $(KBUILD) M=`pwd` V=$(VERBOSITY) modules

clean:
	make -C $(KBUILD) M=`pwd` V=$(VERBOSITY) clean

obj-m := pt3_drv.o

pt3_drv-objs := pt3_pci.o pt3_bus.o pt3_i2c.o pt3_tc.o pt3_qm.o pt3_mx.o pt3_dma.o

clean-files := *.o *.ko *.mod.[co] *~ version.h

version.h:
	revh="#define DRV_VERSION \"rev.`git rev-list HEAD | wc -l 2> /dev/null`\"\n#define DRV_RELDATE \"`git show --date=short --format=%ad | sed -n '1p' 2> /dev/null`\""; \
	if [ -n "$$revh" ] ; then \
		/bin/echo -e "$$revh" > $@; \
	else \
		printf "#define DRV_VERSION \"$(REL_VERSION)\"\n#define DRV_RELDATE \"$(REL_DATE)\"\n" > $@; \
	fi

uninstall:
	rm -f $(INSTALL_DIR)/$(TARGET)*

install: $(TARGET) uninstall
	install -d $(INSTALL_DIR)
	install -m 644 $(TARGET) $(INSTALL_DIR)
	if [ -d /etc/udev/rules.d -a ! -f /etc/udev/rules.d/99-pt3.rules ] ; then \
		install -m 644 etc/99-pt3.rules /etc/udev/rules.d ; \
	fi
	depmod -a

install_compress: install
	. $(KBUILD)/.config ; \
	if [ $$CONFIG_DECOMPRESS_XZ = "y" ] ; then \
		xz   -9e $(INSTALL_DIR)/$(TARGET); \
	elif [ $$CONFIG_DECOMPRESS_BZIP2 = "y" ] ; then \
		bzip2 -9 $(INSTALL_DIR)/$(TARGET); \
	elif [ $$CONFIG_DECOMPRESS_GZIP = "y" ] ; then \
		gzip  -9 $(INSTALL_DIR)/$(TARGET); \
	fi
	depmod -a
