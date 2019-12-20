LANG=ENG
echo "======== check config start================"
echo "please make sure to use ./mk_kernel -M to create new config"
grep CONFIG_HOBOT_XJ3 .config
grep CONFIG_HOBOT_FPGA_X2 .config
echo "======== check config end=================="


./mk_kernel.sh -j


echo "======== Generate files for Palladium start================"
#sh img2ddrmem.sh
COMBINEDFILE=palladium/ddrmem.bin
dd conv=notrunc bs=1 if=palladium/zero_0x80000.bin of=$COMBINEDFILE seek=0 
dd conv=notrunc bs=1 if=arch/arm64/boot/Image of=$COMBINEDFILE seek=$((0x80000))
dd conv=notrunc bs=1 if=arch/arm64/boot/dts/hobot/hobot-x2aj2a-soc.dtb of=$COMBINEDFILE seek=$((0x4000000))
cd palladium
zip ddrmem.zip ddrmem.bin
cd ..

ls -lh palladium/ddrmem.bin
ls -lh palladium/ddrmem.zip

echo "NOTE: NEED TO disable drivers/tty/serial/x2_serial.c::CONFIG_X2_TTY_DMA_MODE for palladium"
echo "NOTE: NEET TO disable CONFIG_X2_TTY_DMA_MODE=y in .config for palladium"
echo "======== Generate files for Palladium end================"