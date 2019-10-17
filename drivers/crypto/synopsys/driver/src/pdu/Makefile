ifdef PDU_USE_KERNEL
PARTS=module
else
PARTS=lib
endif

all: ${PARTS}

lib:
	make -C ${PDU_ARCH}/pdu

module:
	make -C ${PDU_ARCH}/kernel
ifdef PDU_USE_KERNEL
	cp ${PDU_ARCH}/kernel/*.ko ../../bin/
endif

clean:
	make -C ${PDU_ARCH}/pdu clean
	make -C ${PDU_ARCH}/kernel clean

