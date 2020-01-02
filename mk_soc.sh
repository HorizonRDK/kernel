LANG=ENG

./mk_kernel.sh -j


echo "======== Generate files for Palladium start================"
#sh img2ddrmem.sh
COMBINEDFILE=palladium/ddrmem.bin
dd conv=notrunc bs=1 if=palladium/zero_0x80000.bin of=$COMBINEDFILE seek=0 
dd conv=notrunc bs=1 if=arch/arm64/boot/Image of=$COMBINEDFILE seek=$((0x80000))
dd conv=notrunc bs=1 if=arch/arm64/boot/dts/hobot/hobot-xj3-soc-cv.dtb of=$COMBINEDFILE seek=$((0x4000000))
cd palladium
zip ddrmem.zip ddrmem.bin
cd ..

ls -lh palladium/ddrmem.bin
ls -lh palladium/ddrmem.zip

echo "NOTE: NEED TO disable drivers/tty/serial/x2_serial.c::CONFIG_HOBOT_TTY_DMA_MODE for palladium"
echo "NOTE: NEET TO disable CONFIG_HOBOT_TTY_DMA_MODE=y in .config for palladium"
echo "======== Generate files for Palladium end================"

echo "please make sure to use ./mk_kernel -M to create new config for soc"
echo "please make sure to use ./mk_kernel -C to create new config for soc-cv"
echo "======== check config start================"
grep CONFIG_HOBOT_XJ3 .config
grep CONFIG_HOBOT_FPGA_X2 .config
echo "*** 1. setting for CV test: HOBOT_XJ3_CV_ADDITION=y, CONFIG_HOBOT_VIO=N"
grep HOBOT_XJ3_CV_ADDITION .config
grep CONFIG_HOBOT_VIO .config
echo "*** 2. setting for palladium: CONFIG_MMC_DW=N, CONFIG_HOBOT_TTY_DMA_MODE=N"
grep CONFIG_HOBOT_TTY_DMA_MODE .config
grep CONFIG_MMC_DW .config
echo "*** 3. diff .config arch/arm64/configs/xj3_soc_cv_defconfig"
diff .config arch/arm64/configs/xj3_soc_cv_defconfig
echo "*** 4. dts list"
echo "arch/arm64/boot/dts/hobot/hobot-xj3-fpga-cv.dtb for CV FPGA"
echo "arch/arm64/boot/dts/hobot/hobot-xj3-soc-cv.dtb for CV SOC"
echo "======== check config end=================="