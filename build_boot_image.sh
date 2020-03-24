#!/bin/bash

kernel_path=$SRC_KERNEL_DIR
system_id=""
config=$GPT_CONFIG

function get_system_partition_number()
{
    for line in $(cat $config)
    do
        arr=(${line//:/ })

        local needparted=${arr[0]}
        local _name=${arr[1]}
        local dir=${_name%/*}

	if [ x"$needparted" = x"1" ];then
		system_id=$(($system_id + 1))
	fi

	if [ x"$dir" = x"system" ];then
		return
	fi
    done
}

rm -rf out

# compile kernel image
echo "begin to compile kernel"
bash $kernel_path/build.sh || {
        echo "$SRC_BUILD_DIR/tools/mkbootimg/build_bootimg.sh failed"
        exit 1
}

# build boot.img
echo "begin to compile boot.img"
cd $SRC_BUILD_DIR/tools/mkbootimg
bash build_bootimg.sh || {
	echo "$SRC_BUILD_DIR/tools/mkbootimg/build_bootimg.sh failed"
	exit 1
}
cd -

# get system id
get_system_partition_number

# build vbmeta.img 
echo "begin to compile vbmeta.img"
cd $SRC_BUILD_DIR/tools/avbtools

echo "**************************"
echo "bash build_boot_vbmeta.sh boot $system_id"
bash build_boot_vbmeta.sh boot $system_id

echo "*************************"
echo "bash build_vbmeta.sh boot $system_id"
bash build_vbmeta.sh boot $system_id
cd -

mkdir -p out
cp -rf $SRC_BUILD_DIR/tools/avbtools/out/vbmeta.img out/
cp -rf $SRC_BUILD_DIR/tools/avbtools/images/boot.img out/

# build boot.zip

cd $SRC_BUILD_DIR/ota_tools/kernel_package_maker/
bash build_kernel_update_package.sh emmc
cd -

cp -rf $SRC_BUILD_DIR/ota_tools/kernel_package_maker/boot.zip out/
