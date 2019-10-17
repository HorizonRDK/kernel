/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions, and the following disclaimer, without
 *    modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ELPPKA_DEVICE_H_
#define ELPPKA_DEVICE_H_

#include <stdarg.h>

/*
 * Open the PKA character device specified by name.  If name is NULL, attempt
 * to find a device automatically.  Returns the opened file descriptor, or -1
 * on error.
 */
int elppka_device_open(const char *name);

/*
 * Get/set PKA operands.  If func is the empty string or NULL, the name
 * parameter designates an absolute operand (such as A0 or D3).  Otherwise,
 * the parameter names are specific to the particular function.
 *
 * Parameter data is size bytes long, and stored most-significant byte first.
 */
int elppka_set_operand(int fd, const char *func, const char *name,
                               unsigned size, const void *data);
int elppka_get_operand(int fd, const char *func, const char *name,
                               unsigned size, void *data);

/*
 * Copy a PKA operand from one location to another.  Same effect as calling
 *
 *   elppka_get_operand(fd, src_func, src_name, size, tmp_buf);
 *   elppka_set_operand(fd, dst_func, dst_name, size, tmp_buf);
 *
 * except that no temporary buffer is required.  Returns 0 on success, or -1
 * on failure.
 */
int elppka_copy_operand(int fd, const char *dst_func, const char *dst_name,
                                const char *src_func, const char *src_name,
                                unsigned size);

/*
 * Tests a PKA flag.  Flags are essentially treated like boolean operands.
 * If func is the empty string or NULL, the name parameter designates a flag
 * directly (one of Z, M, B, C, or F0--F3).  Otherwise, the flag names are
 * specific to the particular function.
 *
 * Returns 0 if the flag is unset, a positive value if the flag is set, or -1
 * on error.
 */
int elppka_test_flag(int fd, const char *func, const char *name);

/*
 * Set a PKA flag for the next operation.
 *
 * Returns 0 if the flag was previously unset, a positive value if it was
 * previously set, or -1 on error.
 */
int elppka_set_flag(int fd, const char *func, const char *name);

/*
 * Run the PKA.  Takes the function name and operand size (in bytes).  Each
 * successive pair of arguments in the variadic part indicates the operands,
 * the first argument is a pointer to the parameter name and the second is
 * a pointer to the data (or, in the case of output parameters, a buffer in
 * which to store the data).  All operand buffers must be at least size
 * bytes long.  The parameter list is terminated by (char *)NULL.
 *
 * Input and output parameters may be specified in any order.  All input
 * parameters are processed prior to any output parameters, but otherwise
 * paramters are handled in the same order that they are specified.  Thus,
 * the same buffer may be used multiple times.
 *
 * Output parameters are indicated by prefixing their name by a single =, and
 * absolute parameter locations are indicated by prefixing the name by a
 * single %.  For absolute output parameters, the = must precede the %.
 * Other names are specific to the particular function being called.
 *
 * Example: call the function "foo" with a named input "bar" and an absolute
 * input "D0", with named outputs "baz" and absolute output "A1".
 *
 *   elppka_run(fd, "foo", 32,
 *              "bar",  &in_bar,
 *              "=baz", &out_baz,
 *              "%D0",  &in_d0,
 *              "=%A1", &out_a1,
 *              (char *)NULL);
 *
 * Returns -1 on failure, 0 on success, or a positive value indicating some
 * other error.  If a non-zero value is returned, none of the output parameters
 * are written.
 *
 * The elppka_vrun function is similar, except it takes a va_list instead.
 */
int elppka_vrun(int fd, const char *func, unsigned size, va_list ap);
int elppka_run(int fd, const char *func, unsigned size, ...);

/*
 * Close a previously opened PKA character device.
 */
int elppka_device_close(int fd);

#endif
