CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS=eape_build_sa.o eape_close.o eape_deinit.o eape_done.o eape_go.o eape_init.o eape_open.o eape_pop_packets.o eape_rng.o

libeape.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a

