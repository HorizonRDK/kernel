ccflags-y := -DCONFIG_HOBOT_BIF_AP
ccflags-y += -DCONFIG_HOBOT_BIFSD

#ifneq ($(KERNELRELEASE),)

obj-m +=biftty.o
biftty-objs:=bif_tty.o debug.o ringbuf.o

#else
PWD  := $(shell pwd)

KVER = $(shell uname -r)
KDIR := /lib/modules/$(KVER)/build

build:kernel_modules

kernel_modules:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions *.order *.symvers

install:
	/sbin/insmod biftty.ko

remove:
	/sbin/rmmod biftty

mknodd:
	/bin/mknod /dev/ttyBif c 240 0

reinstall:
	 $(MAKE) -C $(KDIR) M=$(CURDIR) modules
	/sbin/rmmod biftty
	/sbin/insmod biftty.ko


test:
	gcc $(CFLAGS) test_bif_tty.c -o test_bif_tty

#endif
