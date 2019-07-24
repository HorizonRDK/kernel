#!/bin/bash

path=$SRC_KERNEL_DIR/usr/prerootfs

rm -rf ubifs.img
rm -rf rootfs_ubifs.img

# create rootfs ubifs
mkfs.ubifs -r $path/ -m 1 -e 65408 -c 256 -o ubifs.img
ubinize -o rootfs_ubifs.img -m 1 -p 65536 -s 1 rootfs_ubinize.cfg

rm -rf ubifs.img
# create app ubifs
mkfs.ubifs  -m 1 -e 65408 -c 512 -o ubifs.img
ubinize -o app_ubifs.img -m 1 -p 65536 -s 1 app_ubinize.cfg
