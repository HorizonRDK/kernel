#!/bin/bash

CURR_DIR=$(realpath $(pwd))

export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
export KERNELDIR=${CURR_DIR}/../../..

echo "ARCH=$ARCH"
echo "CURR_DIR=$CURR_DIR"
echo "CROSS_COMPILE=$CROSS_COMPILE"
echo "KERNELDIR=$KERNELDIR"
echo "-----------------------------------"



