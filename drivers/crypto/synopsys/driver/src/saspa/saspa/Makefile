CFLAGS=-Wall -I../include -I../../pdu/${PDU_ARCH}/include/
OBJECTS= elpaes.o elpdes.o elphash.o elpsaspa.o

libsaspa.a: ${OBJECTS}
	ar rs $@ $^
	cp $@ ../../../bin/

clean:
	rm -f *.o .*.cmd *.a
