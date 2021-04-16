

EXTRA_CFLAGS += -Wall -I../pdu/ -I../core

elpclp800-objs := clp800.o clp800_debug.o clp800_core.o
obj-$(CONFIG_CRYPTO_DEV_HOBOT_TRNG) += elpclp800.o

ifeq ($(CONFIG_HOBOT_GCOV_BASE),y)
GCOV_PROFILE := y
endif
