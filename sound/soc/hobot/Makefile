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
obj-$(CONFIG_HOBOT_AUDIO) += hobot_codec.o

obj-$(CONFIG_HOBOT_1977)  += hobot-snd-1977.o
obj-$(CONFIG_HOBOT_102)   += hobot-snd-102.o
obj-$(CONFIG_HOBOT_DUMMY) += hobot-snd-dummy.o
obj-$(CONFIG_HOBOT_101) += hobot-snd-101.o
obj-$(CONFIG_HOBOT_96BOARD) += hobot-snd-96.o
endif
endif

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
