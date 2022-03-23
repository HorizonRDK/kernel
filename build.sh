#!/bin/bash

function add_initramfs_src()
{
    config=$1
    local rootfspre="CONFIG_INITRAMFS_SOURCE=\"./usr/prerootfs/\""
    local rootfsnone="CONFIG_INITRAMFS_SOURCE=\"\""
    sed -i "/${rootfsnone}/a CONFIG_INITRAMFS_COMPRESSION=\".gz\"" ${config}
    sed -i "/${rootfsnone}/a CONFIG_INITRAMFS_ROOT_GID=0" ${config}
    sed -i "/${rootfsnone}/a CONFIG_INITRAMFS_ROOT_UID=0" ${config}
    sed -i "s#${rootfsnone}#${rootfspre}#g" ${config}
}

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
        add_initramfs_src ${BUILD_OUTPUT_PATH}/.config
    fi
}

function make_recovery_img()
{
    if [ -f "${BUILD_OUTPUT_PATH}/vmlinux" ];then
        mv "${BUILD_OUTPUT_PATH}/vmlinux" "${BUILD_OUTPUT_PATH}/vmlinux_ori"
    fi
    prefix=$TARGET_KERNEL_DIR
    mkdir -p ${BUILD_OUTPUT_PATH}/usr/prerootfs/

    add_initramfs_src ${BUILD_OUTPUT_PATH}/.config

    ${SRC_SCRIPTS_DIR}/build_root_manifest.sh ${KERNEL_DEBUG_INITRAMFS_MANIFEST} ${TARGET_PREROOTFS_DIR} ${TARGET_TMPROOTFS_DIR} ${BUILD_OUTPUT_PATH}/usr/prerootfs/
    sed -i "/AMA0/d" ${BUILD_OUTPUT_PATH}/usr/prerootfs/etc/inittab

    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} -j${N} || {
        echo "make failed"
        exit 1
    }

    # put binaries to dest directory
    runcmd "mkdir -p ${prefix}"
    runcmd "cp ${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME $prefix/$RECOVERY_IMAGE_NAME"
    if [ -f "${BUILD_OUTPUT_PATH}/vmlinux_ori" ];then
        mv "${BUILD_OUTPUT_PATH}/vmlinux" "${BUILD_OUTPUT_PATH}/vmlinux_recovery"
        mv "${BUILD_OUTPUT_PATH}/vmlinux_ori" "${BUILD_OUTPUT_PATH}/vmlinux"
    fi
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

    # clean old images
    echo "Cleaning old images!"
    runcmd "rm -f ${TARGET_DEPLOY_DIR}/kernel.img"
    runcmd "rm -f ${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}.img"
    runcmd "rm -f ${TARGET_DEPLOY_DIR}/${KERNEL_PARTITION_NAME}.img"
    runcmd "rm -f ${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}/${VBMETA_PARTITION_NAME}.img"
    runcmd "rm -f ${TARGET_DEPLOY_DIR}/${KERNEL_PARTITION_NAME}/${KERNEL_PARTITION_NAME}.img"

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

    blk_sz=512
    vbmeta_size=$(get_partition_size $blk_sz "vbmeta")
    if [ -f "${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}/${VBMETA_PARTITION_NAME}.img" ];then
        runcmd "dd if=/dev/zero of=${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}.img bs=${blk_sz} count=${vbmeta_size} conv=sync,notrunc status=none"
        runcmd "dd if=${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}/${VBMETA_PARTITION_NAME}.img of=${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}.img conv=sync,notrunc status=none"
    else
        echo "vbmeta not created! Skip rest of packaging!"
        exit 1
    fi
    cpfiles "${TARGET_DEPLOY_DIR}/${KERNEL_PARTITION_NAME}/${KERNEL_PARTITION_NAME}.img" "${TARGET_DEPLOY_DIR}"

    # create kernel.img
    output_file=${TARGET_DEPLOY_DIR}/kernel.img
    runcmd "dd if=${TARGET_DEPLOY_DIR}/${VBMETA_PARTITION_NAME}.img of=${output_file} bs=${blk_sz} conv=sync,notrunc status=none"
    runcmd "dd if=${TARGET_DEPLOY_DIR}/${KERNEL_PARTITION_NAME}.img of=${output_file} bs=${blk_sz} seek=${vbmeta_size} conv=sync,notrunc status=none"
    # build boot.zip
    bash ${path_otatool}/build_kernel_update_package.sh emmc
}

function pre_pkg_preinst() {
    # Get the signature algorithm used by the kernel.
    local module_sig_hash="$(grep -Po '(?<=CONFIG_MODULE_SIG_HASH=").*(?=")' "${BUILD_OUTPUT_PATH}/.config")"
    # Get the key file used by the kernel.
    local module_sig_key="$(grep -Po '(?<=CONFIG_MODULE_SIG_KEY=").*(?=")' "${BUILD_OUTPUT_PATH}/.config")"
    module_sig_key="${module_sig_key:-certs/hobot_fixed_signing_key.pem}"
    # Path to the key file or PKCS11 URI
    if [[ "${module_sig_key#pkcs11:}" == "${module_sig_key}" && "${module_sig_key#/}" == "${module_sig_key}" ]]; then
        local key_path="${BUILD_OUTPUT_PATH}/${module_sig_key}"
    else
        local key_path="${module_sig_key}"
    fi
    # Certificate path
    local cert_path="${BUILD_OUTPUT_PATH}/certs/signing_key.x509"
    # Sign all installed modules before merging.
    find $TARGET_TMPROOTFS_DIR/lib/modules/${KERNEL_VER}/ -name "*.ko" -exec "${BUILD_OUTPUT_PATH}/scripts/sign-file" "${module_sig_hash}" "${key_path}" "${cert_path}" '{}' \;
}

function all()
{
    prefix=$TARGET_KERNEL_DIR
    config=$KERNEL_DEFCONFIG

    echo "kernel config: $config"
    # real build
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} $config || {
        echo "make $config failed"
        exit 1
    }

    # Configuring Image for AP Booting
    choose

    # For GCOV only, will modify the resulting .config
    set_kernel_config

    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} -j${N} || {
        echo "make failed"
        exit 1
    }
    if [ "x$UBUNTU_ROOT" = "xtrue" ]; then
        rm -rf ${BUILD_OUTPUT_PATH}/../linux-*.deb
        rm -rf ${BUILD_OUTPUT_PATH}/../linux-*.changes
        make ARCH=${ARCH_KERNEL} -j${N} O=${BUILD_OUTPUT_PATH} bindeb-pkg || {
            echo "make failed"
            exit 1
        }
    fi


    # Moving Close Source kos to target/tmprootfs
    rel_ko_path="${SRC_PREBUILTS_DIR}/ko"
    [ ! -d ${TARGET_TMPROOTFS_DIR}/ -a -d ${rel_ko_path} ] && mkdir -p ${TARGET_TMPROOTFS_DIR}/
    if [ "$TARGET_MODE" = "debug" ];then
        [ -d ${rel_ko_path}/ko_debug/ ] && cp -raf ${rel_ko_path}/ko_debug/* ${TARGET_TMPROOTFS_DIR}/
    elif [ "$TARGET_MODE" = "release" ] || [ "$TARGET_MODE" = "quickboot" ];then
        [ -d ${rel_ko_path}/ko_release/ ] && cp -raf ${rel_ko_path}/ko_release/* ${TARGET_TMPROOTFS_DIR}/
    elif [ "$TARGET_MODE" = "docker" ];then
        [ -d ${rel_ko_path}/ko_docker/ ] && cp -raf ${rel_ko_path}/ko_docker/* ${TARGET_TMPROOTFS_DIR}/
    else
        echo "TARGET_MODE:$TARGET_MODE has not support yet"
        exit 1
    fi

    #release: module signature: use fixed public and private keys
    if [ "$TARGET_MODE" = "release" ] || [ "$TARGET_MODE" = "quickboot" ];then
        cp -rf ${SRC_KERNEL_DIR}/certs/hobot_fixed_signing_key.pem ${BUILD_OUTPUT_PATH}/certs/
        cp -rf ${SRC_KERNEL_DIR}/certs/signing_key.x509 ${BUILD_OUTPUT_PATH}/certs/
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

    if [ "$TARGET_MODE" = "release" ] || [ "$TARGET_MODE" = "quickboot" ];then
        pre_pkg_preinst
    fi

    # copy firmware
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/marvell/FwImage/sd8801_uapsta.bin" "$TARGET_TMPROOTFS_DIR/lib/firmware/mrvl/"
    cpfiles "$SRC_KERNEL_DIR/drivers/staging/rtl8723bs/rtlwifi/rtl8723bs_nic.bin " "$TARGET_TMPROOTFS_DIR/lib/firmware/rtlwifi/"
    cpfiles "$SRC_KERNEL_DIR/drivers/crypto/hobot/pka/clp300.elf " "$TARGET_TMPROOTFS_DIR/lib/firmware/"

    # put kernel image & dtb to dest directory
    cpfiles "${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/$KERNEL_IMAGE_NAME" "$prefix/"
    cpfiles "${BUILD_OUTPUT_PATH}/arch/$ARCH_KERNEL/boot/dts/hobot/$KERNEL_DTB_NAME" "$prefix/"

    #calculate kernel crc
    runcmd "$SRC_KERNEL_DIR/crc.py ${BUILD_OUTPUT_PATH}/arch/arm64/boot/$KERNEL_IMAGE_NAME ${BUILD_OUTPUT_PATH}/result_crc"
    cpfiles "${BUILD_OUTPUT_PATH}/result_crc " "$TARGET_TMPROOTFS_DIR/etc/diag/"
    #rm $SRC_KERNEL_DIR/_install/ -rf

    # build dtb-mapping.conf
    build_dtbmapping

    if [ "x$KERNEL_WITH_RECOVERY" = "xtrue" ];then
        # get recovery.gz or recovery.lz4
        # TODO: should be optimized with "choose"
        make_recovery_img
    fi

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
    make mrproper
    make ARCH=${ARCH_KERNEL} O=${BUILD_OUTPUT_PATH} distclean
}

# include
. $INCLUDE_FUNCS
# include end

cd $(dirname $0)

buildopt $cmd
