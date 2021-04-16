cd code/
make clean
cp Makefile_gdc0.mk Makefile
make
mv gdc0.ko ../ko/gdc0.ko

make clean
cp Makefile_gdc1.mk Makefile
make
mv gdc1.ko ../ko/gdc1.ko
cp Makefile_gdc0.mk Makefile
cd ..
