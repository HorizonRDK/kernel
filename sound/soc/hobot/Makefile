# X2 Platform Support
ifeq ($(BOARD_ID), 105)
obj-y += x2-cpudai.o
obj-y += x2-dma.o
obj-y += x2-snd-96.o
else
ifeq ($(BOARD_ID), 401)
obj-y += x2-cpudai.o
obj-y += x2-dma.o
obj-y += x2-snd-s202.o
else
obj-$(CONFIG_X2_AUDIO) += x2-cpudai.o
obj-$(CONFIG_X2_AUDIO) += x2-dma.o
obj-$(CONFIG_X2_AUDIO) += x2-snd-96.o
endif
endif
