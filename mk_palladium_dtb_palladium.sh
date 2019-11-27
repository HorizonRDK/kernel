cp arch/arm64/boot/dts/hobot/hobot.dtsi.cpu-t24mhz_per-t1mhz arch/arm64/boot/dts/hobot/hobot.dtsi
./mk_kernel.sh -d
./__mk_palladium.sh

echo "NOTE: NEED TO disable drivers/tty/serial/x2_serial.c::CONFIG_X2_TTY_DMA_MODE for palladium"
echo "NOTE: NEET TO disable CONFIG_X2_TTY_DMA_MODE=y in .config"
