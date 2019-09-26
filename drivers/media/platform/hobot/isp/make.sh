rm -rf ko
rm -f v4l2_dev/iv009_isp.ko 
cd subdev
cd iq
make
cd ../lens
make
cd ../sensor
make
cd ../../v4l2_dev
make
cd ..
mkdir ko
cp `find -name "*.ko"` ko
echo "insmod iv009_isp_iq.ko" > ko/insert.sh
echo "insmod iv009_isp_sensor.ko" >> ko/insert.sh
echo "insmod iv009_isp_lens.ko" >> ko/insert.sh
echo "insmod iv009_isp.ko" >> ko/insert.sh

echo "rmmod iv009_isp.ko" > ko/rm.sh
echo "rmmod iv009_isp_iq.ko" >> ko/rm.sh
echo "rmmod iv009_isp_sensor.ko" >> ko/rm.sh
echo "rmmod iv009_isp_lens.ko" >> ko/rm.sh

echo "hexdump isp.reg | sed -nr 's/.* (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*)/\2\1 \4\3\n\6\5 \8\7/p'" > ko/hexdump2reg_dump.sh

