[0.环境配置]

该程序运行在python2.7.12

依赖库 numpy

安装方法：sudo apt-get install python-numpy

[1.执行]

可以直接运行 python makeimg.py

[附加功能]

在dtbmapping下面有3个配置文件，dtb_header.json  dtb_file_out.json dtb_mapping.json

dtb_header.json 用于指定DTB镜像头文件信息，可手动修改，默认根据配置文件生成；

dtb_file_out.json 用于指定打包文件以及输出文件;

dtb_mapping.json 用于指定dtb和board信息，添加新的board时，需要在本文件添加board id, gpio id, dtb name等信息;

产生文件：dtb-mapping.conf

bootinfo0 大小为1k，用于烧录uboot动态查询dtb文件
