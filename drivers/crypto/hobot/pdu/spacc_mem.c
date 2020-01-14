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
 * Copyright (c) 2011-2016 Synopsys, Inc. and/or its affiliates.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io-mapping.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <linux/err.h>

#include "elppdu.h"

static unsigned long vex_baseaddr = PDU_BASE_ADDR;
module_param_named(baseaddr, vex_baseaddr, ulong, 0);
MODULE_PARM_DESC(baseaddr, "Hardware base address (default " __stringify(PDU_BASE_ADDR) ")");

// max of 16 devices
#define MAX_DEV 16

static struct platform_device *devices[MAX_DEV];
static int dev_id;

static int spdu_init(unsigned long baseaddr, pdu_info *info)
{
   void *pdu_mem;

   pr_debug("SPAcc-PDU base address: %.8lx\n", baseaddr);

   pdu_mem = ioremap_nocache(baseaddr, 0x1000);
   if (!pdu_mem)
      return -ENOMEM;

   /* Read the config to see what's up */
   pdu_get_version(pdu_mem, info);

   /* Enable SPAcc-PDU interrupts. */
   pdu_io_write32(pdu_mem, PDU_IRQ_EN_GLBL); // enable all ints!

   iounmap(pdu_mem);
   return 0;
}

static void register_device(const char *name, int id,
                            const struct resource *res, unsigned num,
                            pdu_info *info)
{
   char suffix[16] = "";
   struct platform_device_info pdevinfo = {
      .name = name,
      .id = id,
      .res = res,
      .num_res = num,
      .data = info,
      .size_data = sizeof *info,
      .dma_mask = 0xffffffff,
   };

   if (dev_id >= MAX_DEV) {
      pr_err("Too many devices; increase MAX_DEV.\n");
      return;
   }

   devices[dev_id] = platform_device_register_full(&pdevinfo);
  
   if (id != -1)
      arch_setup_dma_ops(&devices[id]->dev, 0, DMA_BIT_MASK(32), NULL, false);

   if (IS_ERR(devices[dev_id])) {
      if (id >= 0)
         snprintf(suffix, sizeof suffix, ".%d", id);
      pr_err("Failed to register %s%s\n", name, suffix);

      devices[dev_id] = NULL;
      return;
   }

   dev_id++;
}

static int __init get_irq_num(unsigned irq_num)
{
//   if (IS_ENABLED(CONFIG_ARCH_ZYNQ)) {
      struct of_phandle_args args = {0};

      /*
       * Since this driver is for non-DT use but Zynq uses DT to setup IRQs,
       * find the GIC by searching for its DT node then manually create the
       * IRQ mappings.
       */

      do {
         args.np = of_find_node_with_property(args.np, "interrupt-controller");
         if (!args.np) {
            pr_err("cannot find IRQ controller");
            return -ENODEV;
         }
      } while (!of_device_is_compatible(args.np, "arm,gic-400"));

    //  if (irq_num < 32 || irq_num >= 96) {
    //     pr_err("SPI interrupts must be in the range [32,96) on Zynq\n");
    //     return -EINVAL;
    //  }

      args.args_count = 3;
      args.args[0] = 0; /* SPI */
      args.args[1] = irq_num - 32;
      args.args[2] = 4; /* Active high, level-sensitive */

      irq_num = irq_create_of_mapping(&args);
      of_node_put(args.np);
      if (irq_num == 0)
         return -EINVAL;
//   }

   if (irq_num > INT_MAX)
      return -EINVAL;

   return irq_num;
}

static int __init pdu_vex_mod_init(void)
{
   int i, rc, irq_num = get_irq_num(PDU_BASE_IRQ);
   struct resource res[2];
   pdu_info info;
   void *pdu_mem;


   if (irq_num >= 0) {
      res[1] = (struct resource) {
         .start = irq_num,
         .end   = irq_num,
         .flags = IORESOURCE_IRQ,
      };
   } else {
      res[1] = (struct resource) { 0 };
      pr_err("IRQ setup failed (error %d), not using IRQs\n", irq_num);
   }

#ifndef PDU_SINGLE_CORE
   rc = spdu_init(vex_baseaddr, &info);
   if (rc < 0)
      return -1;

   pdu_mem = ioremap_nocache(vex_baseaddr, 0x1000);
   if (!pdu_mem)
      return -ENOMEM;

   pr_debug("irq_num             = %d\n", irq_num);
   pr_debug("info.spacc_config.num_ctx             = %d\n", info.spacc_config.num_ctx          );
   pr_debug("info.spacc_config.num_rc4_ctx         = %d\n", info.spacc_config.num_rc4_ctx      );
   pr_debug("info.spacc_config.num_vspacc          = %d\n", info.spacc_config.num_vspacc           );
   pr_debug("info.spacc_config.ciph_ctx_page_size  = %d\n", info.spacc_config.ciph_ctx_page_size   );
   pr_debug("info.spacc_config.hash_ctx_page_size  = %d\n", info.spacc_config.hash_ctx_page_size   );
   pr_debug("info.spacc_config.dma_type            = %d\n", info.spacc_config.dma_type         );
   pr_debug("info.spacc_config.cmd0_fifo_depth     = %d\n", info.spacc_config.cmd0_fifo_depth  );
   pr_debug("info.spacc_config.cmd1_fifo_depth     = %d\n", info.spacc_config.cmd1_fifo_depth  );
   pr_debug("info.spacc_config.cmd2_fifo_depth     = %d\n", info.spacc_config.cmd2_fifo_depth  );
   pr_debug("info.spacc_config.stat_fifo_depth     = %d\n", info.spacc_config.stat_fifo_depth  );

   pr_debug("info.pdu_config.minor   = %d\n",  info.pdu_config.minor );
   pr_debug("info.pdu_config.major   = %d\n",  info.pdu_config.major );
   pr_debug("info.pdu_config.is_rng  = %d\n",  info.pdu_config.is_rng);
   pr_debug("info.pdu_config.is_pka  = %d\n",  info.pdu_config.is_pka);
   pr_debug("info.pdu_config.is_re   = %d\n",  info.pdu_config.is_re );
   pr_debug("info.pdu_config.is_kep  = %d\n",  info.pdu_config.is_kep);
   pr_debug("info.pdu_config.is_ea   = %d\n",  info.pdu_config.is_ea );
   pr_debug("info.pdu_config.is_mpm  = %d\n",  info.pdu_config.is_mpm);


   pr_debug("info.spacc_version.minor = %d\n",    info.spacc_version.minor );
   pr_debug("info.spacc_version.major = %d\n",    info.spacc_version.major );
   pr_debug("info.spacc_version.version= %d\n",   info.spacc_version.version);
   pr_debug("info.spacc_version.qos   = %d\n",    info.spacc_version.qos);
   pr_debug("info.spacc_version.is_spacc = %d\n", info.spacc_version.is_spacc );
   pr_debug("info.spacc_version.is_pdu = %d\n",      info.spacc_version.is_pdu);
   pr_debug("info.spacc_version.is_hsm = %d\n",  info.spacc_version.is_hsm );
   pr_debug("info.spacc_version.idxidx = %d\n",  info.spacc_version.vspacc_idx);
   pr_debug("info.spacc_version.partial = %d\n",  info.spacc_version.partial);
   pr_debug("info.spacc_version.project = %d\n",  info.spacc_version.project);
   pr_debug("info.spacc_version.ivimport= %d\n", info.spacc_version.ivimport);


   for (i = 0; i < info.spacc_config.num_vspacc; i++) {
      unsigned long offset = i*0x40000;

      if (info.spacc_version.is_pdu)
         offset += 0x40000;

      res[0] = (struct resource) {
         .start = vex_baseaddr + offset,
         .end   = vex_baseaddr + offset + 0x40000-1,
         .flags = IORESOURCE_MEM,
      };

      if (info.spacc_version.is_pdu) {
         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_VSPACC(i));
      }
      register_device("spacc", i | (info.spacc_version.project << 16), res, 2, &info);
   }

   if (info.spacc_version.is_pdu) {
      if (info.pdu_config.is_re) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0x8000,
            .end   = vex_baseaddr + 0x8000 + 0x4000-1,
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_RE);
         register_device("spacc-re", -1, res, 2, &info);
      }

      if (info.pdu_config.is_kep) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0x10000,
            .end   = vex_baseaddr + 0x10000 + 0x4000-1,
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_KEP);
         register_device("spacc-kep", -1, res, 2, &info);
      }

      if (info.pdu_config.is_mpm) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0xC000,
            .end   = vex_baseaddr + 0xC000 +  (0xDFFF - 0xC000),
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_MPM);
         register_device("spacc-mpm", 0, res, 2, &info);
      }
#ifdef PDU_DUAL_MPM
      // this configures a second parallel MPM with id "1".  Only enable on platforms
      // that support dual-MPM.
      if (info.pdu_config.is_mpm) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0xE000,
            .end   = vex_baseaddr + 0xE000 +  (0xFFFF - 0xE000),
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_MPM1);
         register_device("spacc-mpm", 1, res, 2, &info);
      }
#endif

      if (info.pdu_config.is_ea) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0x14000,
            .end   = vex_baseaddr + 0x14000 + (0x7FFF - 0x4000),
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_EA);
         register_device("spacc-ea", -1, res, 2, &info);
      }

      if (info.pdu_config.is_rng) {
         res[0] = (struct resource) {
            .start = vex_baseaddr + 0x18000,
            .end   = vex_baseaddr + 0x18000 + 32768-1,
            .flags = IORESOURCE_MEM,
         };

         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_RNG);
         register_device("clp800", -1, res, 2, &info);
      }

      //if (info.pdu_config.is_pka) {
         res[0] = (struct resource) {
            .start = 0xA1018000,
            .end   = 0xA1018000 + 131072-1,
            .flags = IORESOURCE_MEM,
         };
         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_PKA);
         register_device("pka", -1, res, 2, &info);
      //}
   }

         res[0] = (struct resource) {
            .start = 0xA1018000,
            .end   = 0xA1018000 + 131072-1,
            .flags = IORESOURCE_MEM,
         };

         if (irq_num >= 0) {
            res[1] = (struct resource) {
               .start = get_irq_num(PDU_BASE_IRQ+1),
               .end   = get_irq_num(PDU_BASE_IRQ+1),
               .flags = IORESOURCE_IRQ,
            };
         } else {
            res[1] = (struct resource) { 0 };
            pr_err("IRQ setup failed (error %d), not using IRQs\n", irq_num);
         }
      
         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_PKA);
         register_device("pka", -1, res, 2, &info);

	 /////////////////////////////////////////////////////////////////////////////////////////

         res[0] = (struct resource) {
            .start = 0xA100E000,
            .end   = 0xA100E000 + 32768-1,
            .flags = IORESOURCE_MEM,
         };
         res[1] = (struct resource) {
            .start = get_irq_num(PDU_BASE_IRQ+5),
            .end   = get_irq_num(PDU_BASE_IRQ+5),
            .flags = IORESOURCE_IRQ,
	 };
         pdu_io_write32(pdu_mem, pdu_io_read32(pdu_mem) | PDU_IRQ_EN_RNG);
         register_device("clp800", -1, res, 2, &info);

	 /////////////////////////////////////////////////////////////////////////////////////////


   iounmap(pdu_mem);
#else

#ifdef PDU_SINGLE_EAPE
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x200-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("eape", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_CLP30
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x200-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("clp30", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_CLP36
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x200-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("clp36", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_CLP800
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x80-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("clp800", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_CLP850
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x100-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("clp850", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_CLP890
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 0x100-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("clp890", -1, res, 2, &info);
#endif

#ifdef PDU_SINGLE_PKA
   res[0] = (struct resource) {
      .start = vex_baseaddr,
      .end   = vex_baseaddr + 131072-1,
      .flags = IORESOURCE_MEM,
   };
   register_device("pka", -1, res, 2, &info);
#endif
#endif
   return 0;
}
module_init(pdu_vex_mod_init);

static void __exit pdu_vex_mod_exit(void)
{
   int i;

   for (i = 0; i < MAX_DEV; i++) {
      platform_device_unregister(devices[i]);
   }
}
module_exit(pdu_vex_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Synopsys, Inc.");
