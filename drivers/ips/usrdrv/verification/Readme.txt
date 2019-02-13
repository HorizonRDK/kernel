(1) goto <linux> forder:
./mk_kernel.sh -e

(2) goto <linux>/driver/ips/usrdrv/verification
make
make cp

(3)goto <linux> forder:
./mk_kernel.sh -r;./mk_kernel.sh -j;cp arch/arm64/boot/Image ../verify_platform/verify/TestSuite/suite_data/sys_suite_data/;ls -l ../verify_platform/verify/TestSuite/suite_data/sys_suite_data/Image


(4)goto <linux>/arch/boot/hotbot/hobot.dtsi
add following before ips section:
		usedrv: usrdrv@0xA700C000 {
			compatible = "hobot,usrdrv";
			reg = <0 0xA500C000 0 0x100>;
			interrupt-parent = <&gic>;
			interrupts = <0 42 4>, <0 41 4>, <0 40 4>;
		};

(5)./mk_kernel.sh -d;cp arch/arm64/boot/dts/hobot/hobot-x2-fpga.dtb ../verify_platform/verify/TestSuite/suite_data/sys_suite_data/;ls -l ../verify_platform/verify/TestSuite/suite_data/sys_suite_data/hobot-x2-fpga.dtb

(6)goto python:
physon3 test_main.py
#modify test item in test_main.py //pattern='test_boot_kernel.py'
#modify config.py
BROKER_ADDR = 'tcp://10.104.34.40:8888'
BROKER_XPUB_ADDR = 'tcp://10.104.34.40:7700'
