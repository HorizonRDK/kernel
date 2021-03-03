#!/bin/bash

function choose()
{
    local manifest=$KERNEL_INITRAMFS_MANIFEST

    if $KERNEL_WITH_CPIO ;then
        if [ "$BOOT_MODE" != "ap" ];then
            rm -rf ${BUILD_OUTPUT_PATH}/usr/prerootfs/
            mkdir -p ${BUILD_OUTPUT_PATH}/usr/prerootfs/
            if [ "$BOOT_MODE" = "nor" ];then
                manifest=${KERNEL_DEBUG_INITRAMFS_MANIFEST}
            fi
            ${SRC_SCRIPTS_DIR}/build_root_manifest.sh $manifest \
                ${TARGET_PREROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} ${BUILD_OUTPUT_PATH}/usr/prerootfs/
            sed -i "/AMA0/d" ${SRC_KERNEL_DIR}/usr/prerootfs/etc/inittab
        fi
        config=${KERNEL_PREROOTFS_DEFCONFIG}
    fi
}

function make_recovery_img()
{
    prefix=$TARGET_KERNEL_DIR
    config=${KERNEL_PREROOTFS_DEFCONFIG}
    mkdir -p ${BUILD_OUTPUT_PATH}/usr/prerootfs/
    # real build
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} $config || {
        echo "make $config failed"
        exit 1
    }

    ${SRC_SCRIPTS_DIR}/build_root_manifest.sh ${KERNEL_DEBUG_INITRAMFS_MANIFEST} ${TARGET_PREROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} ${BUILD_OUTPUT_PATH}/usr/prerootfs/
    sed -i "/AMA0/d" ${BUILD_OUTPUT_PATH}/usr/prerootfs/etc/inittab

    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/Image.gz"  "$prefix/"
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

    if [ ! -z "$GCOV_CFLAGS" ];then
        echo "Kernel GCOV function is enabled!"
        sed -i 's:# CONFIG_GCOV_KERNEL is not set:CONFIG_GCOV_KERNEL=y:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:# CONFIG_GCOV_FORMAT_AUTODETECT is not set:CONFIG_GCOV_FORMAT_AUTODETECT=y:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:# CONFIG_HOBOT_GCOV_AVIO is not set:CONFIG_HOBOT_GCOV_AVIO=y:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:# CONFIG_HOBOT_GCOV_BASE is not set:CONFIG_HOBOT_GCOV_BASE=y:g' ${BUILD_OUTPUT_PATH}/.config
    else
        sed -i 's:CONFIG_GCOV_KERNEL=y:# CONFIG_GCOV_KERNEL is not set:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:CONFIG_GCOV_FORMAT_AUTODETECT=y:# CONFIG_GCOV_FORMAT_AUTODETECT is not set:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:CONFIG_HOBOT_GCOV_AVIO=y:# CONFIG_HOBOT_GCOV_AVIO is not set:g' ${BUILD_OUTPUT_PATH}/.config
        sed -i 's:CONFIG_HOBOT_GCOV_BASE=y:# CONFIG_HOBOT_GCOV_BASE is not set:g' ${BUILD_OUTPUT_PATH}/.config
    fi
    echo "******************************"
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
    if [ "x$KERNEL_WITH_RECOVERY" = "xtrue" ];then
        # get recovery.gz
        # TODO: should be optimized with "choose"
        make_recovery_img
    fi

    prefix=$TARGET_KERNEL_DIR
    config=$KERNEL_DEFCONFIG

    # Configuring Image for AP Booting
    choose
    echo "kernel config: $config"
    # real build
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} $config || {
        echo "make $config failed"
        exit 1
    }

    # For GCOV only, will modify the resulting .config
    set_kernel_config

    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} -j${N} || {
        echo "make failed"
        exit 1
    }

    # Moving Close Source kos to target/tmprootfs
    [ ! -d ${TARGET_TMPROOTFS_DIR}/ -a -d ${TOPDIR}/ko ] && mkdir -p ${TARGET_TMPROOTFS_DIR}/
    if [ "$TARGET_MODE" = "debug" ];then
        [ -d ${TOPDIR}/ko/ko_debug/ ] && cp -raf ${TOPDIR}/ko/ko_debug/* ${TARGET_TMPROOTFS_DIR}/
    elif [ "$TARGET_MODE" = "release" ];then
        [ -d ${TOPDIR}/ko/ko_release/ ] && cp -raf ${TOPDIR}/ko/ko_release/* ${TARGET_TMPROOTFS_DIR}/
    elif [ "$TARGET_MODE" = "docker" ];then
        [ -d ${TOPDIR}/ko/ko_docker/ ] && cp -raf ${TOPDIR}/ko/ko_docker/* ${TARGET_TMPROOTFS_DIR}/
    else
        echo "TARGET_MODE:$TARGET_MODE has not support yet"
        exit 1
    fi

    # make modules_install to INSTALL_MOD_PATH for debug ko (default: /)
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} INSTALL_MOD_PATH=${BUILD_OUTPUT_PATH}/_debug INSTALL_NO_SUBDIR=1 modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH for debug ko failed"
        exit 1
    }

    # make modules_install to INSTALL_MOD_PATH release ko (default: /)
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} INSTALL_MOD_PATH=${TARGET_TMPROOTFS_DIR}/ INSTALL_MOD_STRIP=1 INSTALL_NO_SUBDIR=1 modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH for release ko failed"
        exit 1
    }

   # strip kernel modules
    [ -z "${KERNEL_VER}" ] && { KERNEL_VER=$(cat ${BUILD_OUTPUT_PATH}/include/config/kernel.release 2> /dev/null); }
    ${CROSS_COMPILE}strip -v -g $TARGET_TMPROOTFS_DIR/lib/modules/${KERNEL_VER}/*.ko

    # copy firmware
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/marvell/FwImage/sd8801_uapsta.bin" "$TARGET_TMPROOTFS_DIR/lib/firmware/mrvl/"
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8723bs/rtlwifi/rtl8723bs_nic.bin " "$TARGET_TMPROOTFS_DIR/lib/firmware/rtlwifi/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/pka/clp300.elf " "$TARGET_TMPROOTFS_DIR/lib/firmware/"

    # put kernel image & dtb to dest directory
    cpfiles "${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cpfiles "${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/dts/hobot/$KERNEL_DTB_NAME" "$prefix/"

    #calculate kernel crc
    runcmd "$SRC_KERNEL_DIR/crc.py ${BUILD_OUTPUT_PATH}/arch/arm64/boot/Image.gz ${BUILD_OUTPUT_PATH}/result_crc"
    cpfiles "${BUILD_OUTPUT_PATH}/result_crc " "$TARGET_TMPROOTFS_DIR/etc/diag/"
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
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} distclean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $cmd
