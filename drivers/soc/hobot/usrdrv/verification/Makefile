CC = aarch64-linux-gnu-gcc

all: usrdrv_app.c usrdrv_lib.c
	echo "======================================================="
	$(CC) -o usrdrv_app.bin usrdrv_app.c usrdrv_lib.c -lpthread -I.

cp:
	sudo cp usrdrv_app.bin ../../../../rootfs/usr/
	ls -l ../../../../rootfs/usr/usrdrv_app.bin

clean:
	rm *.bin
