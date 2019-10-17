ifdef PDU_USE_KERNEL
PARTS=module usr
else
PARTS=lib
endif

all: ${PARTS} 

lib:
	$(MAKE) -C spacc

module:
	$(MAKE) -C kernel
	cp kernel/*.ko ../../bin/
	cp kernel/rc4test.sh ../../bin/

usr:
	$(MAKE) -C user

clean:
	$(MAKE) -C spacc clean
	$(MAKE) -C kernel clean
	$(MAKE) -C user clean

