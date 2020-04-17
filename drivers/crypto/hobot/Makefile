subdir-ccflags-y := -I$(src)/core/
subdir-ccflags-y += -I$(src)/pdu/
subdir-ccflags-y += -I$(src)/spacc/
subdir-ccflags-y += -I$(src)/pka/
subdir-ccflags-y += -I$(src)/trng/

obj-y += core/
obj-y += pdu/
obj-y += spacc/
obj-y += pka/
obj-y += trng/

ifeq ($(CONFIG_HOBOT_GCOV_BASE),y)
GCOV_PROFILE := y
endif
