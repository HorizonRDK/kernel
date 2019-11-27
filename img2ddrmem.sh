COMBINEDFILE=palladium/ddrmem.bin
dd conv=notrunc bs=1 if=palladium/zero_0x80000.bin of=$COMBINEDFILE seek=0 
dd conv=notrunc bs=1 if=arch/arm64/boot/Image of=$COMBINEDFILE seek=$((0x80000))
dd conv=notrunc bs=1 if=arch/arm64/boot/dts/hobot/hobot-x2aj2a-fpga.dtb of=$COMBINEDFILE seek=$((0x4000000))
cd palladium
zip ddrmem.zip ddrmem.bin
