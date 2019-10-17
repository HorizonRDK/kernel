CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS=clp30_build_sa.o clp30_close.o clp30_deinit.o clp30_done.o clp30_go.o clp30_init.o clp30_open.o clp30_pop_packets.o 

libclp30.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a

