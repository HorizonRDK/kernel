ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS}

lib:
	make -C clp30

module:
	make -C kernel
	cp kernel/*.ko ../../bin/

clean:
	make -C clp30 clean
	make -C kernel clean

