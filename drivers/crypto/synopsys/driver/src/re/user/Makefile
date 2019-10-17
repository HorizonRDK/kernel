CFLAGS += -I../include/ -g3

all: reuserdiag reuserspd kepuserdiag
	cp $^ ../../../bin

reuserspd: reuser.o reuserspd.o
	${CC} -g3 $^ -o $@ -lssl -lcrypto -lpthread

reuserdiag: reuser.o reuserdiag.o
	${CC} -g3 $^ -o $@

kepuserdiag: kepuser.o kepuserdiag.o
	${CC} -g3 $^ -o $@

clean:
	rm -f *.o reuserdiag reuserspd kepuserdiag


