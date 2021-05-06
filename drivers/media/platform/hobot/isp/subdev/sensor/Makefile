#
# SPDX-License-Identifier: GPL-2.0
#
# Copyright (C) 2011-2018 ARM or its affiliates
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; version 2.
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#

ifneq ($(KERNELRELEASE),)
sensor_dir := $(KBUILD_SRC)/$(src_subdev)/sensor
else
sensor_dir := $(src_subdev)/sensor
endif

srcfiles := $(subst $(sensor_dir)/,,$(wildcard $(sensor_dir)/src/*/*.c $(sensor_dir)/src/*/*/*.c $(sensor_dir)/src/*/*/*/camera_subdev.c $(sensor_dir)/app/*.c))
hobot_sns_bridge-objs += $(patsubst %.c,%.o,$(srcfiles))
obj-$(CONFIG_VIDEO_HOBOT_ISP) += hobot_sns_bridge.o

EXTRA_CFLAGS += $(addprefix -I,$(shell find $(sensor_dir) -type d))

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
