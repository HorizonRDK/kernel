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
dwe_dir := $(KBUILD_SRC)/$(src_subdev)/dwe
else
dwe_dir := $(src_subdev)/dwe
endif

srcfiles := $(subst $(dwe_dir)/,,$(wildcard $(dwe_dir)/src/*/*.c))
hobot_dwe-objs += $(patsubst %.c,%.o,$(srcfiles))
obj-$(CONFIG_VIDEO_HOBOT_ISP) += hobot_dwe.o

EXTRA_CFLAGS += $(addprefix -I,$(shell find $(dwe_dir) -type d))

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
