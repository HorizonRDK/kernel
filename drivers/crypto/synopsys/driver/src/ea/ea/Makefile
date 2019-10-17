CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/ -I../../core/include/
OBJECTS=ea_build_sa.o ea_close.o ea_deinit.o ea_done.o ea_go.o ea_init.o ea_open.o ea_pop_packets.o ea_set_mode.o


libspacc_ea.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a
