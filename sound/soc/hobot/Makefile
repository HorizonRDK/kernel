# X2 Platform Support
ifneq ($(BOARDID), 105)
obj-y += x2-snd-96.o
obj-y += x2-cpudai.o
obj-y += x2-dma.o
else
obj-$(CONFIG_X2_AUDIO) += x2-cpudai.o
obj-$(CONFIG_X2_AUDIO) += x2-dma.o
obj-$(CONFIG_X2_AUDIO) += x2-snd.o
endif
