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
 * Copyright (c) 2013-2015 Synopsys, Inc. and/or its affiliates.
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

// ------------------------------------------------------------------------
//
//  Project:
//
//    Driver SDK
//
//  Description:
//
//    Application to control the /dev/spaccirq interface to reprogram
//    the IRQ handler
//
// ------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "elpspacc_irq.h"

void usage(void)
{
   printf("spaccirq: --irq_mode wd|step --epn 0x???? [--virt ??] [--wd nnnn] [--stat nnnn] [--cmd nnnn]\n");
   printf("\t--irq_mode\twd==watchdog, step==stepping IRQ\n");
   printf("\t--epn\t\tEPN of SPAcc to change (in hex, e.g. 0x0605)\n");
   printf("\t--virt\t\tVirtual SPAcc to change (default 0)\n");
   printf("\t--wd\t\tWatch dog counter (in cycles) good values are typically >15000\n");
   printf("\t--stat\t\tSTAT_CNT IRQ trigger, see the print out from loading elpspacc.ko to see the size of the STAT FIFO\n");
   printf("\t--cmd\t\tCMD_CNT IRQ trigger (for CMD0),  see the print out from loading elpspacc.ko to see the size of the CMD FIFO\n");
   exit(0);
}


int main(int argc, char **argv)
{
   elpspacc_irq_ioctl io;
   int x, y;

   memset(&io, 0, sizeof io);
   if (argc == 1) {
      usage();
   }
   for (x = 1; x < argc; x++) {
      if (!strcmp(argv[x], "--epn")) {
         io.spacc_epn = strtoul(argv[x+1], NULL, 16);
         ++x;
      } else if (!strcmp(argv[x], "--virt")) {
         io.spacc_virt = strtoul(argv[x+1], NULL, 10);
         ++x;
      } else if (!strcmp(argv[x], "--irq_mode")) {
         if (!strcmp(argv[x+1], "wd")) {
            io.irq_mode = SPACC_IRQ_MODE_WD;
         } else {
            io.irq_mode = SPACC_IRQ_MODE_STEP;
         }
         ++x;
      } else if (!strcmp(argv[x], "--wd")) {
         io.wd_value = strtoul(argv[x+1], NULL, 10);
         ++x;
      } else if (!strcmp(argv[x], "--stat")) {
         io.stat_value = strtoul(argv[x+1], NULL, 10);
         ++x;
      } else if (!strcmp(argv[x], "--cmd")) {
         io.cmd_value = strtoul(argv[x+1], NULL, 10);
         ++x;
      } else {
         usage();
      }
   }

   x = open("/dev/spaccirq", O_RDWR);
   if (!x) {
      perror("Cannot open SPAcc IRQ device");
      return -1;
   }
   y = ioctl(x, SPACC_IRQ_CMD_SET, &io);
   if (y) {
      perror("Cannot send IOCTL to SPAcc IRQ device");
   }
   close(x);
   return y;
}

