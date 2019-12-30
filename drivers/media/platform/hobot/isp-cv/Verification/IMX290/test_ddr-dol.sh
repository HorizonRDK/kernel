mkdir output
chmod 777 output
echo "=============run1==================="
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hn.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nn.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-low.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nl.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-low.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hl.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-low-low.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_ll.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-low-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_lh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-low-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_ln.yuv


./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hhh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hhn.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hnh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hnn.yuv

./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nhh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nhn.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nnh.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nnn.yuv
echo "=============run2==================="
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hn2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nn2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-normal-low.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_nl2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol2-high-low.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr_dol2_frm000_2048x1097_hl2.yuv

./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hhh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hhn2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hnh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-high-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_hnn2.yuv

./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-high-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nhh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-high-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nhn2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-normal-high.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nnh2.yuv
./x2a_vio test_isp ./../config/ddr_sif_isp_dol3-normal-normal-normal.json ./../config/sif_isp_empty_patch.json  d  1 1 >/tmp/log
mv *.yuv output/ddr-dol3_frm000_2048x1097_nnn2.yuv
echo "=============caculate md5==================="
cd output
md5sum *.yuv | sort > md5sum.txt