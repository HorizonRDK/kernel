#!/bin/bash

function choose()
{
    local hascpio=$KERNEL_WITH_CPIO
    local conftmp=.config_tmp
    cp .config $conftmp

    if ! $hascpio ;then
        sed -i "/CONFIG_BLK_DEV_INITRD/d" $conftmp
        echo "CONFIG_BLK_DEV_INITRD=n" >> $conftmp
    else
        sed -i "s#CONFIG_INITRAMFS_SOURCE=\"./usr/rootfs.cpio\"#CONFIG_INITRAMFS_SOURCE=\"./usr/prerootfs/\"#g" $conftmp
        rm -rf ${SRC_KERNEL_DIR}/usr/prerootfs/
        mkdir -p ${SRC_KERNEL_DIR}/usr/prerootfs/
        if [ "$BOOT_MODE" = "nor" ];then
            export KERNEL_INITRAMFS_MANIFEST="$SRC_DEVICE_DIR/$TARGET_VENDOR/$TARGET_PROJECT/debug-kernel-rootfs.manifest"
        fi
        ${SRC_SCRIPTS_DIR}/build_root_manifest.sh ${KERNEL_INITRAMFS_MANIFEST} ${TARGET_PREROOTFS_DIR} ${SRC_KERNEL_DIR}/usr/prerootfs/
        if [ ! -f "${SRC_KERNEL_DIR}/usr/prerootfs/init" ];then
            echo "#!/bin/sh" > ${SRC_KERNEL_DIR}/usr/prerootfs/init
            echo "exec /sbin/init \"\$@\"" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
            chmod +x ${SRC_KERNEL_DIR}/usr/prerootfs/init
        fi
    fi

    cp $conftmp .config
}

function make_recovery_img()
{
    prefix=$TARGET_KERNEL_DIR
    if [ $TARGET_MODE == "debug" ];then
        config=$KERNEL_DEFCONFIG
    else
        config=$KERNEL_PERF_DEFCONFIG
    fi

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
    if [ ! -f "${SRC_KERNEL_DIR}/usr/prerootfs/init" ];then
        echo "#!/bin/sh" > ${SRC_KERNEL_DIR}/usr/prerootfs/init
        echo "/bin/mount -t devtmpfs devtmpfs /dev" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
        echo "exec 0</dev/console" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
        echo "exec 1>/dev/console" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
        echo "exec 2>/dev/console" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
        echo "exec /sbin/init \"\$@\"" >> ${SRC_KERNEL_DIR}/usr/prerootfs/init
        chmod +x ${SRC_KERNEL_DIR}/usr/prerootfs/init
    fi

    cp $conftmp .config
    make -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    cp $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/Image.gz  $prefix/recovery.gz
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

function all()
{
    prefix=$TARGET_KERNEL_DIR
    if [ $TARGET_MODE == "debug" ];then
        config=$KERNEL_DEFCONFIG
    else
        config=$KERNEL_PERF_DEFCONFIG
    fi
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

    # put binaries to dest directory
    cpfiles "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cd $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/
    cpfiles "$KERNEL_DTB_NAME" "$prefix/"

    # build dtb-mapping.conf
    build_dtbmapping

    if [ "x$KERNEL_WITH_RECOVERY" = "xtrue" ];then
        cd $SRC_KERNEL_DIR

        # get recovery.gz
        make_recovery_img
    fi
}

function clean()
{
    make clean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $1
