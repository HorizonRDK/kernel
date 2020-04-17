
elpspacc-objs := spacc.o spacc_ctx_manager.o	spacc_job_manager.o \
				spacc_autodetect.o		spacc_common.o \

obj-$(CONFIG_CRYPTO_DEV_HOBOT) := elpspacc.o

ifeq ($(CONFIG_HOBOT_GCOV_BASE),y)
GCOV_PROFILE := y
endif
