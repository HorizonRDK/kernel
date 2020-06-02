obj-$(CONFIG_BPU)		+= bpu_framework.o
bpu_framework-objs		:=  bpu_drv.o bpu_ctrl.o \
							bpu_debug.o bpu_prio.o \
							bpu_sched.o bpu_stat.o
obj-$(CONFIG_BPU_ONDEMAND_GOVERNOR) += bpu_governor.o
bpu_cores-y := bpu_core.o hw_io.o
bpu_cores-$(CONFIG_X2_BPU) += hw_io_xj3.o
bpu_cores-$(CONFIG_X3_BPU) += hw_io_xj3.o
bpu_cores-$(CONFIG_J5_BPU) += hw_io_j5.o

obj-$(CONFIG_BPU_CORE)	+= bpu_cores.o
