English| [简体中文](./README_cn.md)

[0. Environment Configuration]

This program runs on Python 2.7.12

Dependency library: numpy

Installation method: sudo apt-get install python-numpy

[1. Execution]

You can directly run python makeimg.py

[Additional Features]

Under dtbmapping, there are 3 configuration files: dtb_header.json, dtb_file_out.json, dtb_mapping.json

dtb_header.json is used to specify information about the DTB image header file, which can be manually modified or generated based on the configuration file by default;

dtb_file_out.json is used to specify the packaging and output files;

dtb_mapping.json is used to specify dtb and board information. When adding a new board, you need to add board id, gpio id, dtb name, and other information to this file;

Generated file: dtb-mapping.conf

bootinfo0 has a size of 1k, used for burning uboot to dynamically query dtb files