obj-$(CONFIG_BPU)		+= bpu_framework.o
bpu_framework-objs		:=  bpu.o bpu_ctrl.o \
							bpu_debug.o \
							bpu_sched.o bpu_stat.o

bpu_cores-y := bpu_core.o
bpu_cores-$(CONFIG_X2_BPU) += x2_bpu.o


obj-$(CONFIG_BPU_CORE)	+= bpu_cores.o
