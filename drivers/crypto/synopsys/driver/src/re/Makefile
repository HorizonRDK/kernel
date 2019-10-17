ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS}

lib:
	make -C re

module:
	make -C kernel
	cp kernel/*.ko ../../bin/

clean:
	make -C re clean
	make -C kep clean
	make -C kernel clean
	make -C user clean

