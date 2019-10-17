ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS}

lib:
	make -C eape

module:
	make -C kernel
	cp kernel/*.ko ../../bin/

clean:
	make -C eape clean
	make -C kernel clean

