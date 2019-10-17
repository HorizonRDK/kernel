CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/ -I../../core/include/
OBJECTS=elpre.o re_error_msg.o re_fini.o re_finish_operation.o re_get_context.o re_init_context.o re_packet_dequeue.o re_print_diag.o re_release_context.o re_reset_sa.o re_retrieve_sa.o re_set_next_read.o re_set_next_read_iv.o re_set_next_read_key.o re_set_next_read_mackey.o re_set_next_read_params.o re_set_next_read_sequence_number.o re_set_next_write.o re_set_next_write_iv.o re_set_next_write_key.o re_set_next_write_mackey.o re_set_next_write_params.o re_set_next_write_sequence_number.o re_start_operation.o re_write_sa.o 

libspacc_re.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a
