/*
 *
 *    Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

#ifndef __SOC_DWE_H__
#define __SOC_DWE_H__

// This name is used by both V4L2 ISP device and
// V4L2 DWE subdevice to match the subdevice in the
// V4L2 async subdevice list.
#define V4L2_SOC_DWE_NAME "SocDwe"

// This is used as the main communication structure between
// V4L2 ISP Device and V4L2 Dwe Subdevice
// Parameters are used differently depending on the actual API command ID.
struct soc_dwe_ioctl_args {
    uint32_t ctx_num;
    union {
        struct {
            uint32_t val_in;  // first input value for the API function (optional)
            uint32_t val_in2; // second input value for the API function (optional)
            uint32_t val_out; // output value returned by API function (optional)
        } general;
    } args;
};

// The enum declares the API commands ID which
// must be supported by V4L2 Dwe subdevice.
// This API ID will be used on each ioctl call from
// the V4L2 ISP Device.
enum SocDwe_ioctl {
    // set ldc context info 
    // input: val_in - the next context
    // output: none
    SOC_DWE_SET_LDC = 0,

    // get ldc context info
    // input: none
    // output: start or stop
    SOC_DWE_GET_LDC,

    // set dis context info 
    // input: val_in - the next context
    // output: none
    SOC_DWE_SET_DIS,

    // set dis context info
    // input: none
    // output: start or stop
    SOC_DWE_GET_DIS

};


#endif //__SOC_DWE_H__
