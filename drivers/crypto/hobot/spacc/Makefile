CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS= elpspacc.o spacc_close.o  spacc_ctx_manager.o spacc_disable_int.o spacc_dump_ctx.o spacc_enable_int.o spacc_error_msg.o spacc_get_context.o spacc_get_rc4_context.o spacc_get_version.o spacc_hsm_semaphore.o spacc_job_manager.o spacc_load_skp.o spacc_open.o spacc_packet_dequeue.o spacc_packet_enqueue_ddt.o spacc_process_irq.o spacc_set_auxinfo.o spacc_set_context.o spacc_set_key_exp.o spacc_set_operation.o spacc_set_rc4_context.o spacc_set_secure_mode.o spacc_set_wd_count.o spacc_virtual_request_rc4.o spacc_virtual_set_weight.o 

libspacc.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a

