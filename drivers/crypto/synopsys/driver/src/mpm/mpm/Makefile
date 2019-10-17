CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/ -I../../core/include/
OBJECTS=mpm_alloc_key.o mpm_alloc_pdu.o mpm_cleanup.o mpm_clear_chains.o mpm_deinit.o mpm_enqueue_chain.o mpm_free_key.o mpm_free_spacc_ctx.o mpm_get_status_word.o mpm_init.o mpm_insert_pdu.o mpm_req_spacc_ctx.o mpm_set_key.o mpm_stats.o mpm_process_int.o

libspacc_mpm.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a
