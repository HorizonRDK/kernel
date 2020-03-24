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
                manifest="$SRC_DEVICE_DIR/$TARGET_VENDOR/$TARGET_PROJECT/debug-kernel-rootfs.manifest"
            fi
            ${SRC_SCRIPTS_DIR}/build_root_manifest.sh $manifest \
                ${TARGET_PREROOTFS_DIR} ${SRC_KERNEL_DIR}/usr/prerootfs/
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

    local kernel_initram="$SRC_DEVICE_DIR/$TARGET_VENDOR/$TARGET_PROJECT/debug-kernel-rootfs.manifest"
    ${SRC_SCRIPTS_DIR}/build_root_manifest.sh ${kernel_initram} ${TARGET_PREROOTFS_DIR} ${SRC_KERNEL_DIR}/usr/prerootfs/
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
    python makeimg.py || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cpfiles "dtb-mapping.conf"  "$prefix/"
    cd $SRC_KERNEL_DIR
}

function set_kernel_config()
{
    if [ "$BOOT_MODE" = "nand"  ];then
        sed -i 's/# CONFIG_MTD_UBI_FASTMAP is not set/CONFIG_MTD_UBI_FASTMAP=y/g' $TOPDIR/kernel/arch/arm64/configs/$KERNEL_DEFCONFIG
        sed -i 's/# CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT is not set/CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT=1/g' $TOPDIR/kernel/arch/arm64/configs/$KERNEL_DEFCONFIG
    else
        sed -i 's/CONFIG_MTD_UBI_FASTMAP=y/# CONFIG_MTD_UBI_FASTMAP is not set/g' $TOPDIR/kernel/arch/arm64/configs/$KERNEL_DEFCONFIG
        sed -i 's/CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT=1/# CONFIG_MTD_UBI_FASTMAP_AUTOCONVERT is not set/g' $TOPDIR/kernel/arch/arm64/configs/$KERNEL_DEFCONFIG
    fi
}

function build_wifi()
{
    echo "begin build wifi...................."
    echo "SRC_KERNEL_DIR=$SRC_KERNEL_DIR"
    echo "TARGET_TMPROOTFS_DIR=$TARGET_TMPROOTFS_DIR"

    echo "build wifi marvell 8801............."
    cd $SRC_KERNEL_DIR/drivers/staging/marvell/
    #make clean
    make
    cd $SRC_KERNEL_DIR
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/marvell/bin_sd8801/*.ko" "$TARGET_TMPROOTFS_DIR/lib/modules/"
    rm -fr $SRC_KERNEL_DIR/drivers/staging/marvell/bin_sd8801
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/marvell/FwImage/sd8801_uapsta.bin" "$TARGET_TMPROOTFS_DIR/lib/firmware/mrvl/"
    
    echo "build wifi rtl8819fs.............."
    cd $SRC_KERNEL_DIR/drivers/staging/rtl8189fs/
    make
    cd $SRC_KERNEL_DIR
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8189fs/*.ko" "$TARGET_TMPROOTFS_DIR/lib/modules/"
    rm -fr $SRC_KERNEL_DIR/drivers/staging/rtl8189fs/*.ko 
    echo "end build wifi...................."
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


    if [ x"$BOOT_MODE" = x"nor" ] || [ x"$BOOT_MODE" = x"nand" ];then
        flashline=`getlinenum`
        sed -i "${flashline}s#disabled#okay#g" $dts_file
    else
        key_value="nor_flash {"
        local norline=`getlinenum`
        sed -i "${norline}s#okay#disabled#g" $dts_file
        key_value="nand_flash {"
        local nandline=`getlinenum`
        sed -i "${nandline}s#okay#disabled#g" $dts_file
    fi
}

function all()
{
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

    # make modules_install to INSTALL_MOD_PATH (default: /)
    make INSTALL_MOD_PATH=$SRC_KERNEL_DIR/_install INSTALL_MOD_STRIP=1  modules_install || {
        echo "make modules_install to INSTALL_MOD_PATH failed"
        exit 1
    }
    # x3 wifi
    build_wifi

    # put binaries to dest directory
    cpfiles "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cd $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/
    cpfiles "$KERNEL_DTB_NAME" "$prefix/"
    cpfiles "$SRC_KERNEL_DIR/net/mac80211/mac80211.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/net/wireless/cfg80211.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8723bs/r8723bs.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    ${CROSS_COMPILE}strip -g $TARGET_TMPROOTFS_DIR/lib/modules/*.ko
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8723bs/rtlwifi/rtl8723bs_nic.bin " "$TARGET_TMPROOTFS_DIR/lib/firmware/rtlwifi/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/pka/clp300.elf " "$TARGET_TMPROOTFS_DIR/lib/firmware/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/core/elpspacccrypto.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/pka/elprsa.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/core/spaccexample.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/drivers/soc/hobot/cnn_host/hobot_cnn_host_total.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/drivers/misc/x2_efuse.ko " "$TARGET_TMPROOTFS_DIR/lib/modules/"
    cpfiles "$SRC_KERNEL_DIR/_install/lib/modules/*" "$TARGET_TMPROOTFS_DIR/lib/modules/"

    rm $SRC_KERNEL_DIR/_install/ -rf

    # build dtb-mapping.conf
    build_dtbmapping
}

function all_32()
{
    CROSS_COMPILE=$CROSS_COMPILE_64
    all
}

function clean()
{
    make clean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)
# config dts
change_dts_flash_config $BOOT_MODE

set_kernel_config
buildopt $1
