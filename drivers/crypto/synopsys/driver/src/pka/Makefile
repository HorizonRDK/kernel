ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS} diag

lib:
	make -C pka libpka.a

module:
	make -C kernel
	cp kernel/*.ko ../../bin/

diag:
	make -C user pka-diag
	cp user/pka-diag ../../bin/cluediag

clean:
	make -C pka clean
	make -C kernel clean
	make -C user clean

