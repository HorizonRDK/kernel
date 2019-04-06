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
    cpfiles "$SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cd $SRC_KERNEL_DIR/arch/$ARCH_KERNEL/boot/dts/hobot/
    cpfiles "$KERNEL_DTB_NAME" "$prefix/"
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
