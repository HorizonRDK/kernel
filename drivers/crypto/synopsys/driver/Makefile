#Top level makefile for the EPN1613 Driver SDK bundle
#
#As shipped the makefile is DESIGNED to fail since it is a template makefile for all driver projects
#customers are required to customize the [CONFIG] blocks based on their specific hardware and platform.
#
#For support please contact support@elliptictech.com
#

#CONFIG_ME=1
ifdef CONFIG_ME
def:
	@echo You must configure the makefile before building and then remove the CONFIG_ME line to build, search for [CONFIG] in the Makefile
endif

#[CONFIG] Use PCI to find device, comment this line if you're not using PCI (or just specify a base address in your environment)
ifndef PDU_BASE_ADDR
ifndef PDU_BASE_IRQ
#PDU_USE_PCI=1
endif
endif

#[CONFIG] If we are not using PCI then we must configure the memory address/IRQ of the device
ifndef PDU_USE_PCI
ifndef PDU_BASE_ADDR
PDU_OPTIONS+= PDU_BASE_ADDR=0xb2100000
endif
ifndef PDU_BASE_IRQ
PDU_OPTIONS+= PDU_BASE_IRQ=118
endif
endif

#[CONFIG] Build Linux Kernel modules, comment these lines out if you are building a bare platform (or define a PDU_ARCH)
ifndef PDU_ARCH
PDU_ARCH=linux
PDU_USE_KERNEL=1
endif

#[CONFIG] Enable specific modules
#ENABLE_SASPA=1
ENABLE_SPACC=1
ENABLE_TRNG=1
ENABLE_PKA=1

ifdef ENABLE_SPACC
#Can only enable these three if the SPACC is enabled
#ENABLE_RE=1
#ENABLE_MPM=1
#ENABLE_EA=1
endif

ifndef ENABLE_EA
#Cannot current enable both EA and CLP30/CLP36
#ENABLE_CLP30=1
#Turn this on to enable CLP36 support (you need to turn on ENABLE_CLP30 as well...)
#ENABLE_CLP36=1
#Cannot current enable both EA and EAPE
#ENABLE_EAPE=1
endif

#[CONFIG] Enable single core configurations of EAPE/PKA/TRNG 
#ENABLE_SINGLE_CORE=1
#ENABLE_SINGLE_CLP800=1
#ENABLE_SINGLE_CLP850=1
#ENABLE_SINGLE_CLP890=1
#ENABLE_SINGLE_CLP30=1
#ENABLE_SINGLE_CLP36=1
#ENABLE_SINGLE_EAPE=1
#ENABLE_SINGLE_PKA=1

#PKA_OPTIONS += KERNELDIR=${HOME}/source/kernel_branch/kernel-4.14-proto-x2a/ CROSS_COMPILE=aarch64-linux-gnu- ARCH=arm64
PDU_OPTIONS += KERNELDIR=${PWD}/../../../kernel-4.14-proto-x2a CROSS_COMPILE=aarch64-linux-gnu- ARCH=arm64

# Generally you don't have to modify anything below this point unless you're in some really odd configuration
ifdef ENABLE_SINGLE_CORE
#Must turn on ENABLE_SINGLE_CORE for any of these...
PDU_OPTIONS += PDU_SINGLE_CORE=1

ifdef ENABLE_SINGLE_CLP800
PDU_OPTIONS += PDU_SINGLE_CLP800=1
endif
ifdef ENABLE_SINGLE_CLP850
PDU_OPTIONS += PDU_SINGLE_CLP850=1
endif
ifdef ENABLE_SINGLE_CLP890
PDU_OPTIONS += PDU_SINGLE_CLP890=1
endif
ifdef ENABLE_SINGLE_EAPE
PDU_OPTIONS += PDU_SINGLE_EAPE=1
endif
ifdef ENABLE_SINGLE_CLP30
PDU_OPTIONS += PDU_SINGLE_CLP30=1
endif
ifdef ENABLE_SINGLE_CLP36
PDU_OPTIONS += PDU_SINGLE_CLP36=1
endif
ifdef ENABLE_SINGLE_PKA
PDU_OPTIONS += PDU_SINGLE_PKA=1
endif

endif

ifndef PDU_USE_KERNEL

#use a "bare" PDU interface.  This will also not build kernel modules.  The SDK will
#build as a series of libraries that the developer can then link into their 
#application

PDU_OPTIONS += PDU_ARCH=${PDU_ARCH}

else

#these two must go together
PDU_OPTIONS += PDU_USE_KERNEL=${PDU_USE_KERNEL}
PDU_OPTIONS += PDU_ARCH=linux

#PDU_OPTIONS += PCI_INDIRECT=1
#PDU_OPTIONS += MAKEAVECTOR=1

#options for the platform module
ifdef ENABLE_SPACC
PDU_OPTIONS += PDU_USE_SPACC=1
endif
ifdef ENABLE_SASPA
PDU_OPTIONS += PDU_USE_SASPA=1
endif
ifdef ENABLE_PKA
PDU_OPTIONS += PDU_USE_PKA=1
endif
ifdef ENABLE_CLP36
PDU_OPTIONS += CLP36_ENABLED=1
endif

# use PCI to find device, comment out to use memory mapped sample platform kernel module
ifdef PDU_USE_PCI
PDU_OPTIONS+= PDU_USE_PCI=1
ifndef PDU_PCI_DID
PDU_OPTIONS+= PDU_PCI_DID=0x59AE
else
PDU_OPTIONS+= PDU_PCI_DID=${PDU_PCI_DID}
endif
else
#if we are not using PCI then the base address is given below
endif
endif

PDU_OPTIONS += PDU_LITTLE_ENDIAN=1
#PDU_OPTIONS += PDU_BIG_ENDIAN=1
#PDU_OPTIONS += MPM_TIMING_X86=1
#PDU_OPTIONS += MPM_PERF_MON=1

#assemble list of things to build
PARTS=pdu
ifdef ENABLE_SASPA
PARTS+= saspa
endif
ifdef ENABLE_SPACC
PARTS+= spacc
endif
ifdef ENABLE_RE
PARTS+= re
endif
ifdef ENABLE_TRNG
PARTS+= trng
endif
ifdef ENABLE_PKA
PARTS+= pka
endif
ifdef ENABLE_MPM
PARTS+= mpm
endif
ifdef ENABLE_EA
PARTS+= ea
endif
ifdef ENABLE_EAPE
PARTS+= eape
endif
ifdef ENABLE_CLP30
PARTS+= clp30
endif

ifndef ENABLE_SPACC
ifdef ENABLE_EA
no_ea:
	@echo you must define ENABLE_SPACC to use the EA
endif
ifdef ENABLE_RE
no_re:
	@echo you must define ENABLE_SPACC to use the RE
endif
ifdef ENABLE_KEP
no_kep:
	@echo you must define ENABLE_SPACC to use the KEP
endif
ifdef ENABLE_MPM
no_mpm:
	@echo you must define ENABLE_SPACC to use the MPM
endif
endif

all: ${PARTS}

pdu:
	${PDU_OPTIONS} ${MAKE} -C src/pdu
ifdef PDU_USE_KERNEL
	cp src/pdu/linux/kernel/Module.symvers src/core/kernel/
	cp src/pdu/linux/kernel/Module.symvers src/saspa/kernel/
	cp src/pdu/linux/kernel/Module.symvers src/pka/kernel/
	cp src/pdu/linux/kernel/Module.symvers src/trng/kernel/
	cp src/pdu/linux/kernel/Module.symvers src/eape/kernel/
	cp src/pdu/linux/kernel/Module.symvers src/clp30/kernel/
endif

trng: pdu
	${PDU_OPTIONS} ${TRNG_OPTIONS} ${MAKE} -C src/trng

pka: pdu
	${PDU_OPTIONS} ${PKA_OPTIONS} ${MAKE} -C src/pka

# things below this are dependent on the SPAcc being enabled
spacc: pdu
	${PDU_OPTIONS} ${SPACC_OPTIONS} ${MAKE} -C src/core
ifdef PDU_USE_KERNEL
	cp src/core/kernel/Module.symvers src/re/kernel/
	cp src/core/kernel/Module.symvers src/mpm/kernel/
	cp src/core/kernel/Module.symvers src/ea/kernel/
endif

saspa: pdu
	${PDU_OPTIONS} ${SPACC_OPTIONS} ${MAKE} -C src/saspa

re: spacc rekern reuserspace

rekern:
	${PDU_OPTIONS} ${SPACC_OPTIONS} ${RE_OPTIONS} ${MAKE} -C src/re
		
reuserspace:
	${MAKE} -C src/re/user

mpm: spacc
	${PDU_OPTIONS} ${PKA_OPTIONS} ${MAKE} -C src/mpm

ea: spacc
	${PDU_OPTIONS} ${PKA_OPTIONS} ${MAKE} -C src/ea

eape:
	${PDU_OPTIONS} ${MAKE} -C src/eape

clp30:
	${PDU_OPTIONS} ${MAKE} -C src/clp30

clean:
	${PDU_OPTIONS} ${MAKE} -C src clean
	${MAKE} -C bin clean
	rm -rf modules.* Module.* .tmp*


