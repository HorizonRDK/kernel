#
# Makefile marvell wifi driver
#

KERNELDIR	?= ../../..
ARCH		?= arm64
CROSS_COMPILE	?= aarch64-linux-gnu-
PACKDIR		=SD-8801-U16-MMC-W14.68.36.p139-C4X14635_B0-GPL
SRCDIR		=$(PACKDIR)/wlan_src
#echo "SD-8801-U16-MMC-W14.68.36.p139-C4X14635_B0-GPL"

.PHONY:clean all

all:
	mkdir -p ko
	@echo "ARCH=$(ARCH)"
	@echo "CROSS_COMPILE=$(CROSS_COMPILE)"
	@echo "KERNELDIR=$(KERNELDIR)"
	cd $(SRCDIR); make build; cd -;	
	cp $(PACKDIR)/bin_sd8801/*ko ko;
	${CROSS_COMPILE}strip -g ko/*.ko
clean:
	cd $(SRCDIR); make clean; cd -;
	rm -fr $(PACKDIR)/bin_sd8801
	rm -fr ko


