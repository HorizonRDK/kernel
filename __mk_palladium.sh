LANG=ENG
./mk_kernel.sh -j
sh img2ddrmem.sh
ls -lh palladium/ddrmem.bin
ls -lh palladium/ddrmem.zip