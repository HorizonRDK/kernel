CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS=clp800.o clp850.o clp890.o

libtrng.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a

