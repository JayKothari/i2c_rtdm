obj-m		:= i2c_rtdm_omap.o
KDIR		:= /root/embedded_linux/elabs_BBB/beagle-kernel/kernel
PWD		:= $(shell pwd)
EXTRA_CFLAGS    := -I/usr/xenomai/include -I/usr/include/

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean


