# X2 Platform Support
ifeq ($(BOARD_ID), 105)
obj-y += hobot-cpudai.o
obj-y += hobot-dma.o
obj-y += hobot-snd-96.o
else
ifeq ($(BOARD_ID), 401)
obj-y += hobot-cpudai.o
obj-y += hobot-dma.o
obj-y += hobot-snd-s202.o
else
obj-$(CONFIG_HOBOT_AUDIO) += hobot-cpudai.o
obj-$(CONFIG_HOBOT_AUDIO) += hobot-dma.o
obj-$(CONFIG_HOBOT_AUDIO) += hobot-snd-96.o
obj-$(CONFIG_HOBOT_AUDIO) += hobot_codec.o
endif
endif

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
