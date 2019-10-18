
elpspacc-objs := spacc.o spacc_init.o 	spacc_open.o spacc_close.o  \
				spacc_get_context.o spacc_get_rc4_context.o \
				spacc_packet_dequeue.o	spacc_packet_enqueue_ddt.o \
				spacc_set_context.o 	spacc_set_operation.o spacc_set_rc4_context.o  \
				spacc_set_auxinfo.o		spacc_virtual_request_rc4.o \
				spacc_virtual_set_weight.o spacc_error_msg.o \
				spacc_ctx_manager.o		spacc_job_manager.o \
				spacc_enable_int.o		spacc_process_irq.o \
				spacc_load_skp.o spacc_dump_ctx.o spacc_hsm_semaphore.o \
				spacc_set_key_exp.o		spacc_set_wd_count.o \
				spacc_autodetect.o		spacc_isenabled.o \
				spacc_compute_xcbc_key.o spacc_process_jb.o

obj-$(CONFIG_CRYPTO_DEV_HOBOT) += elpspacc.o
