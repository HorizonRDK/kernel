#!/bin/bash

function all()
{
    prefix=$TARGET_KERNEL_DIR
    config=$KERNEL_DEFCONFIG
    echo "kernel config: $config"

    # real build
    ARCH=$ARCH_KERNEL
    make $config || {
        echo "make $config failed"
        exit 1
    }
    make -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    echo "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$SRC_KERNEL_IMAGE_NAME"
    echo "$prefix/$TARGET_KERNEL_IMAGE_NAME"
    if [ -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$SRC_KERNEL_IMAGE_NAME -a -n $prefix/$TARGET_KERNEL_IMAGE_NAME ];then
        mkdir -p $prefix
        echo "cp -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$SRC_KERNEL_IMAGE_NAME $prefix/$TARGET_KERNEL_IMAGE_NAME"
        cp -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$SRC_KERNEL_IMAGE_NAME $prefix/$TARGET_KERNEL_IMAGE_NAME
    fi
    if [ -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/$SRC_KERNEL_DTB_NAME -a -n $prefix/$TARGET_KERNEL_DTB_NAME ];then
        mkdir -p $prefix
        echo "cp -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/$SRC_KERNEL_DTB_NAME $prefix/$TARGET_KERNEL_DTB_NAME"
        cp -f $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/$SRC_KERNEL_DTB_NAME $prefix/$TARGET_KERNEL_DTB_NAME
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
