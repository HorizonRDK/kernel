#!/bin/bash

function choose()
{
    local hascpio=$KERNEL_WITH_CPIO
    local conftmp=.config_tmp
    local rootfscpio="CONFIG_INITRAMFS_SOURCE=\"./usr/rootfs.cpio\""
    local rootfspre="CONFIG_INITRAMFS_SOURCE=\"./usr/prerootfs/\""
    local rootfsnone="CONFIG_INITRAMFS_SOURCE=\"\""
    local manifest=$KERNEL_INITRAMFS_MANIFEST
    cp .config $conftmp

    if ! $hascpio ;then
        sed -i "s#${rootfscpio}#${rootfsnone}#g" $conftmp
    else
        sed -i "s#${rootfscpio}#${rootfspre}#g" $conftmp
        if [ "$BOOT_MODE" != "ap" ];then
            rm -rf ${SRC_KERNEL_DIR}/usr/prerootfs/
            mkdir -p ${SRC_KERNEL_DIR}/usr/prerootfs/
            if [ "$BOOT_MODE" = "nor" ];then
                manifest=${KERNEL_DEBUG_INITRAMFS_MANIFEST}
            fi
            ${SRC_SCRIPTS_DIR}/build_root_manifest.sh $manifest \
                ${TARGET_PREROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} ${SRC_KERNEL_DIR}/usr/prerootfs/
            sed -i "/AMA0/d" ${SRC_KERNEL_DIR}/usr/prerootfs/etc/inittab
        fi
    fi

    cp $conftmp .config
}

function make_recovery_img()
{
    prefix=$TARGET_KERNEL_DIR
    config=$KERNEL_DEFCONFIG

    # real build
    ARCH=$ARCH_KERNEL
    make $config || {
        echo "make $config failed"
        exit 1
    }

    local conftmp=.config_tmp
    cp .config $conftmp

    sed -i "s#CONFIG_INITRAMFS_SOURCE=\"./usr/rootfs.cpio\"#CONFIG_INITRAMFS_SOURCE=\"./usr/prerootfs/\"#g" $conftmp
    rm -rf ${SRC_KERNEL_DIR}/usr/prerootfs/
    mkdir -p ${SRC_KERNEL_DIR}/usr/prerootfs/

    ${SRC_SCRIPTS_DIR}/build_root_manifest.sh ${KERNEL_DEBUG_INITRAMFS_MANIFEST} ${TARGET_PREROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} ${SRC_KERNEL_DIR}/usr/prerootfs/
    sed -i "/AMA0/d" ${SRC_KERNEL_DIR}/usr/prerootfs/etc/inittab

    cp $conftmp .config
    make -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/Image.gz"  "$prefix/"
    mv $prefix/Image.gz $prefix/recovery.gz
}

function build_dtbmapping()
{
    prefix=$TARGET_KERNEL_DIR
    path=$SRC_KERNEL_DIR/tools/dtbmapping

    cd $path

    # real build
    python2 makeimg.py || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "dtb-mapping.conf"  "$prefix/"
    cd $SRC_KERNEL_DIR
}

function set_kernel_config()
{
    echo "******************************"
    echo "Set Kernel Defconfig"
    if [ "$BOOT_MODE" = "nand"  ];then
        sed -i 's/# CONFIG_MTD_UBI_FASTMAP is not set/CONFIG_MTD_UBI_FASTMAP=y/g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/$KERNEL_DEFCONFIG
        sed -i 's/# CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT is not set/CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT=1/g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/$KERNEL_DEFCONFIG
    else
        sed -i 's/CONFIG_MTD_UBI_FASTMAP=y/# CONFIG_MTD_UBI_FASTMAP is not set/g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/$KERNEL_DEFCONFIG
        sed -i 's/CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT=1/# CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT is not set/g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/$KERNEL_DEFCONFIG
    fi

    if [ ! -z "$GCOV_CFLAGS" ];then
        echo "Kernel GCOV function is enabled!"
        sed -i 's:# CONFIG_GCOV_KERNEL is not set:CONFIG_GCOV_KERNEL=y:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:# CONFIG_GCOV_FORMAT_AUTODETECT is not set:CONFIG_GCOV_FORMAT_AUTODETECT=y:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:# CONFIG_HOBOT_GCOV_AVIO is not set:CONFIG_HOBOT_GCOV_AVIO=y:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:# CONFIG_HOBOT_GCOV_BASE is not set:CONFIG_HOBOT_GCOV_BASE=y:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
    else
        sed -i 's:CONFIG_GCOV_KERNEL=y:# CONFIG_GCOV_KERNEL is not set:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:CONFIG_GCOV_FORMAT_AUTODETECT=y:# CONFIG_GCOV_FORMAT_AUTODETECT is not set:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:CONFIG_HOBOT_GCOV_AVIO=y:# CONFIG_HOBOT_GCOV_AVIO is not set:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
        sed -i 's:CONFIG_HOBOT_GCOV_BASE=y:# CONFIG_HOBOT_GCOV_BASE is not set:g' $OUT_BUILD_KERNEL_DIR/arch/arm64/configs/xj3_debug_defconfig
    fi
    echo "******************************"
}

function change_dts_flash_config()
{
    local dts_file="arch/arm64/boot/dts/hobot/hobot-xj3-xvb.dtsi"
    local key_value="$1_flash {"

    declare -i nline

    getline()
    {
        cat -n $dts_file|grep "${key_value}"|awk '{print $1}'
    }

    getlinenum()
    {
        awk "BEGIN{a=`getline`;b="1";c=(a+b);print c}";
    }

    key_value="nor_flash {"
    local norline=`getlinenum`
    sed -i "${norline}s#okay#disabled#g" $dts_file
    key_value="nand_flash {"
    local nandline=`getlinenum`
    sed -i "${nandline}s#okay#disabled#g" $dts_file

    if [ x"$1" = x"nor" ] || [ x"$1" = x"nand" ];then
        key_value="$1_flash {"
        flashline=`getlinenum`
        sed -i "${flashline}s#disabled#okay#g" $dts_file
    fi
}

function get_system_partition_number()
{
    local config=$GPT_CONFIG

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

function build_boot_image()
{
    local path_avbtool="$SRC_BUILD_DIR/tools/avbtools"
    local path_otatool="$SRC_BUILD_DIR/ota_tools/kernel_package_maker"
    local path_mkbootimg="$SRC_BUILD_DIR/tools/mkbootimg"
    local kernel_path=$SRC_KERNEL_DIR

    # build boot.img
    echo "begin to compile boot.img"
    cd $path_mkbootimg
    bash build_bootimg.sh || {
	echo "$path_mkbootimg/build_bootimg.sh failed"
	exit 1
    }
    cd -

    # get system id
    get_system_partition_number

    # build vbmeta.img
    echo "begin to compile vbmeta.img"
    cd $path_avbtool

    echo "**************************"
    echo "bash build_boot_vbmeta.sh boot $system_id"
    bash build_boot_vbmeta.sh boot $system_id

    echo "*************************"
    echo "bash build_vbmeta.sh boot $system_id"
    bash build_vbmeta.sh boot $system_id
    cd -

    cpfiles "$path_avbtool/out/vbmeta.img" "$TARGET_DEPLOY_DIR"
    cpfiles "$path_avbtool/images/boot.img" "$TARGET_DEPLOY_DIR"
    [ -f "$TARGET_DEPLOY_DIR/kernel.img" ] && { runcmd "rm $TARGET_DEPLOY_DIR/kernel.img"; }
    # calculate vbmeta partition size
    line=`sed -n '/vbmeta/p' ${GPT_CONFIG}`
    arg=(${line//:/ })
    vbmeta_size=$(( ${arg[4]%?} - ${arg[3]%?} + 1 ))
    # create kernel.img
    dd if=$TARGET_DEPLOY_DIR/vbmeta.img of=$TARGET_DEPLOY_DIR/kernel.img bs=512 conv=sync,notrunc > /dev/null 2>&1
    dd if=$TARGET_DEPLOY_DIR/boot.img of=$TARGET_DEPLOY_DIR/kernel.img bs=512 seek=${vbmeta_size} conv=sync,notrunc > /dev/null 2>&1
    # build boot.zip
    cd $path_otatool
    bash build_kernel_update_package.sh emmc
    cd -

    cpfiles "$path_otatool/boot.zip" "$TARGET_DEPLOY_DIR/ota"
}


function all()
{
    export SRC_KERNEL_DIR=`pwd`
    change_dts_flash_config $if_flash
    set_kernel_config

    if [ "x$KERNEL_WITH_RECOVERY" = "xtrue" ];then
        cd $SRC_KERNEL_DIR

        # get recovery.gz
        make_recovery_img
    fi

    prefix=$TARGET_KERNEL_DIR
    config=$KERNEL_DEFCONFIG
    echo "kernel config: $config"

    # real build
    ARCH=$ARCH_KERNEL
    make $config || {
        echo "make $config failed"
        exit 1
    }
    choose
    make -j${N} || {
        echo "make failed"
        exit 1
    }

    [ ! -d _install/ ] && mkdir _install/

    if [ "$TARGET_MODE" = "debug" ];then
        [ -d ${TOPDIR}/ko/ko_debug/ ] && cp -raf ${TOPDIR}/ko/ko_debug/* _install/
    elif [ "$TARGET_MODE" = "release" ];then
        [ -d ${TOPDIR}/ko/ko_release/ ] && cp -raf ${TOPDIR}/ko/ko_release/* _install/
    elif [ "$TARGET_MODE" = "docker" ];then
        [ -d ${TOPDIR}/ko/ko_docker/ ] && cp -raf ${TOPDIR}/ko/ko_docker/* _install/
    else
        echo "TARGET_MODE:$TARGET_MODE has not support yet"
        exit 1
    fi

    # make modules_install to INSTALL_MOD_PATH for debug ko (default: /)
    make INSTALL_MOD_PATH=$SRC_KERNEL_DIR/_debug INSTALL_NO_SUBDIR=1 modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH for debug ko failed"
        exit 1
    }

    # make modules_install to INSTALL_MOD_PATH release ko (default: /)
    make INSTALL_MOD_PATH=$SRC_KERNEL_DIR/_install INSTALL_MOD_STRIP=1 INSTALL_NO_SUBDIR=1 modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH for release ko failed"
        exit 1
    }

    # put kernel image & dtb to dest directory
    cpfiles "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cd $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/
    cpfiles "$KERNEL_DTB_NAME" "$prefix/"

    # copy firmware
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/marvell/FwImage/sd8801_uapsta.bin" "$TARGET_TMPROOTFS_DIR/lib/firmware/mrvl/"
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8723bs/rtlwifi/rtl8723bs_nic.bin " "$TARGET_TMPROOTFS_DIR/lib/firmware/rtlwifi/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/pka/clp300.elf " "$TARGET_TMPROOTFS_DIR/lib/firmware/"

    # strip & copy kernel modules
    cpfiles "$SRC_KERNEL_DIR/_install/lib/modules/*" "$TARGET_TMPROOTFS_DIR/lib/modules/"
    [ -z "${KERNEL_VER}" ] && { KERNEL_VER=$(cat $SRC_KERNEL_DIR/include/config/kernel.release 2> /dev/null); }
    ${CROSS_COMPILE}strip -v -g $TARGET_TMPROOTFS_DIR/lib/modules/${KERNEL_VER}/*.ko

    #rm $SRC_KERNEL_DIR/_install/ -rf

    # build dtb-mapping.conf
    build_dtbmapping

    if [ x"$build_boot_img" = x"true" ];then
        build_boot_image
    fi
}

function all_32()
{
    CROSS_COMPILE=$CROSS_COMPILE_64
    all
}

function usage()
{
    echo "Usage: build.sh [-p]"
    echo "Options:"
    echo "  -p  compile kernel, get vbmeta.img, boot.img and boot.zip"
    echo "  -h  help info"
    echo "Command:"
    echo "  clean clean all the object files along with the executable"
}

build_boot_img="false"
system_id=""

while getopts "ph:" opt
do
    case $opt in
        p)
            build_boot_img="true"
            ;;
        h)
            usage
            exit 0
            ;;
        \?)
            usage
            exit 1
            ;;
    esac
done

shift $[ $OPTIND - 1 ]

cmd=$1

function clean()
{
    SRC_KERNEL_DIR=`pwd`
    make distclean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

# config dts
if_flash=$BOOT_MODE
if [[ ! -z "$FLASH_ENABLE" ]];then
    if [ "$FLASH_ENABLE" = "nor" -o  "$FLASH_ENABLE" = "nand" ];then
        echo "$FLASH_ENABLE flash is enabled!!"
        if_flash=$FLASH_ENABLE
    fi
fi

buildopt $cmd
