####################################
# Set you own environment here
####################################
export PATH=~/codes/testbench-master/hbtdisk/toolchain/aarch64-gcc-linaro-7.2.1-2017.11/bin/:$PATH
cross=aarch64-linux-gnu-
arch=arm64
cfg=x2aj2a_fpga_defconfig
fs_tmp="rootfs"
mod_path="mod_dir"

####################################
# function define
####################################
mk_cfg()
{
    #echo make menuconfig by using defconfig: $cfg
    make CROSS_COMPILE=$cross ARCH=$arch $cfg
    make CROSS_COMPILE=$cross ARCH=$arch menuconfig
}
mk_old_cfg()
{
    #echo make menuconfig by using .config
    make CROSS_COMPILE=$cross ARCH=$arch menuconfig
}
mk_all()
{
    #echo make all with $j
    make CROSS_COMPILE=$cross ARCH=$arch Image $j
    make CROSS_COMPILE=$cross ARCH=$arch dtbs
    make CROSS_COMPILE=$cross ARCH=$arch modules
}

mk_dtbs()
{
    make CROSS_COMPILE=$cross ARCH=$arch dtbs
}

mk_clean()
{
    #echo make clean
    make CROSS_COMPILE=$cross ARCH=$arch clean
}

extract_initramfs()
{
    #echo please be sure that you are in root folder of kernel!
    if [ -d "$fs_tmp" ]; then
	    sudo rm $fs_tmp -rf
    fi
    mkdir $fs_tmp
    cd $fs_tmp
    sudo cpio -ivmd < ../usr/rootfs.cpio
    cd -
}

mk_initramfs()
{
    if [ -d "$fs_tmp" ]; then
        echo this will make the rootfs.cpio to usr folder from ./rootfs
    else
        echo no existing $fs_tmp, please make sure you already execute ./mk_kernel.sh -e
        exit
    fi

    cd $fs_tmp
    sudo find . | sudo cpio -H newc -o> ../usr/rootfs.cpio
    cd -
}

mk_modules_install()
{
    make CROSS_COMPILE=$cross ARCH=$arch INSTALL_MOD_PATH=$mod_path modules_install

    if [ ! -d "$mod_path" ] || [ ! -d "$fs_tmp" ]; then
        echo no existing $mod_path or $fs_tmp, please make sure you already excute ./mk_kernel.sh -e
        exit
    fi

    sudo cp $mod_path/* $fs_tmp/ -rf
    sync
}

helper()
{
    echo
    echo ---------------------------------------------------------------------
    echo "Usage:  "
    echo "  sh mk_kernel.sh [option]"
    echo "    option:"
    echo "    -j: burst build (make -jxx)"
    echo "    -c: make clean command"
    echo "    -e: extract rootfs to kernel root folder"
    echo "    -r: pack rootfs from kernel root folder"
    echo "    -m: make menuconfig by specified defconfig (define as cfg above)"
    echo "    -o: make menuconfig without specified defconfig (define as cfg above)"
    echo "    -i: make modules_install (to INSTALL_MOD_PATH)"
    echo "    -h: helper prompt"
    echo
}

####################################
# main logic from here
####################################

#"uh" > "u:h:" if need args
while getopts "MCmjcheroid" opt; do
  case $opt in
       m)
			mk_cfg
			exit
			;;
       M)
			cfg=x2aj2a_soc_defconfig
			echo $cfg
			mk_cfg
			exit
			;;
       C)
			cfg=xj3_soc_cv_defconfig
			echo $cfg
			mk_cfg
			exit
			;;
       o)
			mk_old_cfg
			exit
			;;
       j)
           j="-j64"
			echo burst build !
           ;;
       c)
			mk_clean
			exit
			;;
       e)
			extract_initramfs
			exit
			;;
       r)
			mk_initramfs
			exit
			;;
       d)
			mk_dtbs
			exit
			;;
       i)
			mk_modules_install
			exit
			;;
       h)
           helper
			exit
           ;;
      \?)
      echo "Invalid option: -$OPTARG" >&2
      ;;
  esac
done

mk_all
