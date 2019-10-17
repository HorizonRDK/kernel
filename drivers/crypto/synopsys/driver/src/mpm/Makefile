ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS}

lib:
	make -C mpm

module:
	make -C kernel
	cp kernel/*.ko ../../bin/

clean:
	make -C mpm clean
	make -C kernel clean

