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
src_v4l2 := $(KBUILD_SRC)/$(src_v4l2)
endif

srcfiles := $(subst $(src_v4l2)/,,$(wildcard $(src_v4l2)/src/*/*.c $(src_v4l2)/src/*/*/*.c $(src_v4l2)/app/*.c $(src_v4l2)/app/*/*.c))
hobot_isp-objs += $(patsubst %.c,%.o,$(srcfiles))
obj-$(CONFIG_VIDEO_HOBOT_ISP) += hobot_isp.o

EXTRA_CFLAGS += $(addprefix -I,$(shell find $(src_v4l2) -type d))

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
