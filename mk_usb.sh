sh mk_soc.sh
cp arch/arm64/boot/Image ../u-boot/pkgtool/images/Image
cp arch/arm64/boot/dts/hobot/hobot-xj3-soc-cv.dtb ../u-boot/pkgtool/images/hobot-x2aj2a-fpga.dtb
cd ../u-boot
echo "assume you did: sh mk_uboot.sh -a -m"
echo "assume you did: sh mk_uboot.sh -p u"
sh mk_uboot.sh -b 0 -j
sh mk_uboot.sh -p u
echo "please cp spl_out/non-secure/boot.pkg ~/  or ( cp spl_out/boot.pkg ~/)"
