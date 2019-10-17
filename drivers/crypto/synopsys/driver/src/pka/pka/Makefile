CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS=elppka.o  firmware.o

libpka.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/



clean:
	rm -f *.o

