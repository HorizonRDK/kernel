/*
 * Horizon Robotics
 *
 *  Author:	Liwei Zhang <liwei.zhang@hobot.cc>
 *  Copyright 	(C) 2021 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/arm-smccc.h>
#include <linux/ptrace.h>
#include <asm/stacktrace.h>
#include <soc/hobot/hobot_fiq_debugger.h>
#include <asm/current.h>
#include <linux/console.h>
#include <linux/kgdb.h>
#include <asm/system_misc.h>
#include <linux/sched/debug.h>

struct hobot_fiq_debugger {
    uint32_t mem_share_start;
    uint32_t mem_share_len;
    uint32_t __iomem *base_mem_share;
    struct mutex enable_mlock;
    int fiq_enabled;
    int kgdb_support;
    int lockup_fiq_support;
    int callback_sel; //1 means dump allself, 2 means dump by BUG
};

static struct hobot_fiq_debugger *s_pdebugger;

#define HOBOT_FIQDEBUG_NAME "hobot-fiqdebug"

#define SIP_SVC_IRQ_TRIGGER    0x8200ff10
#define SIP_SVC_FIQ_CALLBACK_REGISTER    0x8200ff11
#define SIP_SVC_FIQ_NO_REGISTER    0x8200ff12
#define SIP_SVC_NS_CONTEXT_GET    0x8200ff13
#define SIP_SVC_MEM_SHARE_REQUEST    0x8200ff14
#define SIP_SVC_FIQ_AFFINITY_SET    0x8200ff15
#define SIP_SVC_FIQ_NO_UNREGISTER    0x8200ff16

#define FIQ_NUM_BARK_IRQ   47
#define FIQ_NUM_APB_TIMEOUT 97
#define FIQ_NUM_PADC        86

#define MEM_SHARE_OFFSET_SYSREG     0
#define MEM_SHARE_OFFSET_PSTATE     34
#define MEM_SHARE_OFFSET_PC         35
#define MEM_SHARE_OFFSET_SP         36

static void mem_share_context_get(struct hobot_fiq_debugger* debugger, struct pt_regs* fiq_pt_regs);

static uint64_t invoke_fn_smc(unsigned long function_id,
             unsigned long arg0, unsigned long arg1,
             unsigned long arg2)
{
    struct arm_smccc_res res;

    arm_smccc_smc(function_id, arg0, arg1, arg2, 0, 0, 0, 0, &res);
    return res.a0;
}

static void fiq_debugger_cb(unsigned long sp_el1,
    unsigned long cpu, unsigned long offset)
{
    struct pt_regs el1_state;
    pr_info("bak to EL1 in_irq %lx %x preempt_count %x\n", in_interrupt(), in_atomic(), preempt_count());
    mem_share_context_get(s_pdebugger, &el1_state);
    show_regs(&el1_state);
#ifdef CONFIG_PREEMPT_RT_FULL
    /*
     * we can't print in these contexts with IRQs or preeption disabled after
     * PREEMPT_RT_FULL enabled, so we print manually
     */
    console_trylock();
    console_unlock();
#endif
    if (s_pdebugger->kgdb_support)
        kgdb_breakpoint();
}

static void panic_cb(unsigned long sp_el1,
    unsigned long cpu, unsigned long offset)
{
    struct pt_regs el1_state;
    pr_info("bak to EL1 in_irq %lx %x preempt_count %x\n", in_interrupt(), in_atomic(), preempt_count());
    mem_share_context_get(s_pdebugger, &el1_state);
    /*
     * BUG should trigger undefined instruction sync exception, this exception can save the callback 
     * enviorment, then die and panic runs. the stack is useful, however, regs are no longer useful.
     * If die(), argument regs should be setup in mem_share_context_get().
     * */
    //BUG();
    die("panic_cb", &el1_state, 0);
}

static void sip_fiq_callback_regitster(int fiq_no)
{
    if (fiq_no == FIQ_NUM_BARK_IRQ) {
        if (s_pdebugger->callback_sel == 1)
            invoke_fn_smc(SIP_SVC_FIQ_CALLBACK_REGISTER,
                (unsigned long)fiq_debugger_cb, 0, 0);
        else if (s_pdebugger->callback_sel == 2)
            invoke_fn_smc(SIP_SVC_FIQ_CALLBACK_REGISTER,
                (unsigned long)panic_cb, 0, 0);
        else
            pr_err("unsupported callback sel\n");
        pr_debug("fiq_no %d callback registered %lx\n", fiq_no, (unsigned long)fiq_debugger_cb);
    }
}

static uint64_t sip_fiq_no_register(int fiq_no)
{
    uint64_t ret = 0;
    if (fiq_no == FIQ_NUM_BARK_IRQ) {
        ret = invoke_fn_smc(SIP_SVC_FIQ_NO_REGISTER,
            (unsigned long)fiq_no, 0, 0);
        pr_debug("fiq_no %d registered\n", fiq_no);
    }
    return ret;
}

static uint64_t sip_fiq_no_unregister(int fiq_no)
{
    uint64_t ret = 0;
    if (fiq_no == FIQ_NUM_BARK_IRQ) {
        ret = invoke_fn_smc(SIP_SVC_FIQ_NO_UNREGISTER,
            (unsigned long)fiq_no, 0, 0);
        pr_debug("fiq_no %d unregistered\n", fiq_no);
    }
    return ret;
}


static void __maybe_unused sip_nosecure_context_get(void)
{
    invoke_fn_smc(SIP_SVC_NS_CONTEXT_GET,
        0, 0, 0);
}

static void sip_fiq_affinity_set(int fiq_no, int id)
{
    invoke_fn_smc(SIP_SVC_FIQ_AFFINITY_SET,
        fiq_no, id, 0);
    pr_debug("%s fiq_no %d\n", __func__, fiq_no);
}


static void sip_mem_share_request(struct hobot_fiq_debugger* debugger)
{
    uint64_t ret;

    ret = invoke_fn_smc(SIP_SVC_MEM_SHARE_REQUEST,
        debugger->mem_share_start, debugger->mem_share_len, 0);
    if (ret)
        pr_debug("mem request success\n");
}

static void mem_share_context_get(struct hobot_fiq_debugger* debugger, struct pt_regs* fiq_pt_regs)
{
    /*
     * 64-bit ATF + 64-bit kernel
     */
    /* copy cpu context: x0 ~ spsr_el3 */
    memcpy(fiq_pt_regs, debugger->base_mem_share, 8 * 31);

    /* copy pstate: spsr_el3 */
    memcpy(&fiq_pt_regs->pstate, (uint64_t*)debugger->base_mem_share + MEM_SHARE_OFFSET_PSTATE, 8);
    //fiq_pt_regs.sp = sp_el1;

    /* copy pc: elr_el3 */
    memcpy(&fiq_pt_regs->pc, (uint64_t*)debugger->base_mem_share + MEM_SHARE_OFFSET_PC, 8);
    memcpy(&fiq_pt_regs->sp, (uint64_t*)debugger->base_mem_share + MEM_SHARE_OFFSET_SP, 8);
}

static ssize_t callback_sel_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    mutex_lock(&debugger->enable_mlock);
    len += snprintf(buf, 16, "%d\n", debugger->callback_sel);
    mutex_unlock(&debugger->enable_mlock);
    return len;
}

static ssize_t callback_sel_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
    int ret = 0;
    int tmp;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    ret = sscanf(buf, "%d", &tmp);
    //if (ret != 1)
    //    return size;
    mutex_lock(&debugger->enable_mlock);
    debugger->callback_sel = tmp;
    mutex_unlock(&debugger->enable_mlock);
    sip_fiq_callback_regitster(FIQ_NUM_BARK_IRQ);
    return size;
}

static ssize_t kgdb_support_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    mutex_lock(&debugger->enable_mlock);
    len += snprintf(buf, 16, "%d\n", debugger->kgdb_support);
    mutex_unlock(&debugger->enable_mlock);
    return len;
}

static ssize_t kgdb_support_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
    int ret = 0;
    int tmp;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    ret = sscanf(buf, "%d", &tmp);
    //if (ret != 1)
    //    return size;
    mutex_lock(&debugger->enable_mlock);
    debugger->kgdb_support = tmp;
    mutex_unlock(&debugger->enable_mlock);
    return size;
}

static ssize_t lockup_trigger_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    mutex_lock(&debugger->enable_mlock);
    len += snprintf(buf, 16, "%d\n", debugger->lockup_fiq_support);
    mutex_unlock(&debugger->enable_mlock);
    return len;
}

static ssize_t lockup_trigger_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
    int ret = 0;
    int tmp;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    ret = sscanf(buf, "%d", &tmp);
    if (ret != 1)
        return size;
    mutex_lock(&debugger->enable_mlock);
    debugger->lockup_fiq_support = tmp;
    mutex_unlock(&debugger->enable_mlock);
    return size;
}

static ssize_t fiq_enable_show(struct device *dev,
				struct device_attribute *attr, char* buf)
{
    ssize_t len = 0;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    mutex_lock(&debugger->enable_mlock);
    len += snprintf(buf, 16, "%d\n", debugger->fiq_enabled);
    mutex_unlock(&debugger->enable_mlock);
    return len;
}

static ssize_t fiq_enable_store(struct device *dev,
				       struct device_attribute *devAttr,
				       const char *buf, size_t size)
{
    int ret = 0;
    uint32_t tmp = 0;
    struct hobot_fiq_debugger *debugger = dev_get_drvdata(dev);

    ret = sscanf(buf, "%du", &tmp);
    if (ret != 1)
        return size;
    if ((tmp != 0) && (tmp != 1))
        return size;
    if (tmp == debugger->fiq_enabled)
        return size;

    mutex_lock(&debugger->enable_mlock);
    if (tmp == debugger->fiq_enabled) {
        mutex_unlock(&debugger->enable_mlock);
        return size;
    } else {
        debugger->fiq_enabled = tmp;
    }
    mutex_unlock(&debugger->enable_mlock);

    if (debugger->fiq_enabled) {
        ret = sip_fiq_no_register(FIQ_NUM_BARK_IRQ);
        if (!ret) {
            pr_err("fiq no register failed\n");
            return size;
        }
        if (debugger->callback_sel)
            sip_fiq_callback_regitster(FIQ_NUM_BARK_IRQ);
    } else {
         ret = sip_fiq_no_unregister(FIQ_NUM_BARK_IRQ);
        if (!ret) {
            pr_err("fiq no register failed\n");
            return size;
        }
    }
    return size;
}
static DEVICE_ATTR(kgdb_support, 0644, kgdb_support_show, kgdb_support_store);
static DEVICE_ATTR(fiq_enable, 0644, fiq_enable_show, fiq_enable_store);
static DEVICE_ATTR(lockup_trigger, 0644, lockup_trigger_show, lockup_trigger_store);
static DEVICE_ATTR(callback_sel, 0644, callback_sel_show, callback_sel_store);

int hobot_hardlockup_fiq_support(void)
{
    if (s_pdebugger == NULL)
        return 0;
    return s_pdebugger->lockup_fiq_support;
}

void hobot_fiqdebug_affinity_set(int id)
{
    sip_fiq_affinity_set(FIQ_NUM_BARK_IRQ, id);
}
EXPORT_SYMBOL(hobot_fiqdebug_affinity_set);

static int hobot_fiqdebug_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *npm;
	struct resource r;
    int err;
    struct hobot_fiq_debugger *pdebugger;

    pdebugger = kzalloc(sizeof(struct hobot_fiq_debugger), GFP_KERNEL);
    if (!pdebugger) {
        dev_err(&pdev->dev, "kzalloc failed\n");
        return -ENOMEM;
    }

    npm = of_parse_phandle(np, "shared-region", 0);
	if (npm) {
        err = of_address_to_resource(npm, 0, &r);
		if (err == 0) {
            pdebugger->base_mem_share = ioremap_nocache(r.start,
					resource_size(&r));
            dev_info(&pdev->dev, "shared mem paddr=%p,vaddr=%p,size=0x%llx clean\n",
                (void *)r.start, pdebugger->base_mem_share, resource_size(&r));
        } else {
            dev_err(&pdev->dev, "shared-region map failed\n");
            goto err1;
        }
    }
    pdebugger->mem_share_start = r.start;
    pdebugger->mem_share_len = resource_size(&r);

    sip_mem_share_request(pdebugger);

/*
 * set default fiq enabled
    err = sip_fiq_no_register(FIQ_NUM_BARK_IRQ);
        if (!err) {
            dev_err(&pdev->dev, "fiq no register failed\n");
            goto err2;
    }
    sip_fiq_callback_regitster(FIQ_NUM_BARK_IRQ);
    pdebugger->fiq_enabled = 1;
    pdebugger->lockup_fiq_support = 1;
 */
    pdebugger->lockup_fiq_support = 0;
    pdebugger->fiq_enabled = 0;
    pdebugger->kgdb_support = 0;
    pdebugger->callback_sel = 1;

    s_pdebugger = pdebugger;

    mutex_init(&pdebugger->enable_mlock);
    platform_set_drvdata(pdev, pdebugger);

    err = device_create_file(&pdev->dev, &dev_attr_fiq_enable);
	if (err < 0) {
		dev_err(&pdev->dev, "create fiq enable failed %d\n", err);
		goto err2;
	}

    err = device_create_file(&pdev->dev, &dev_attr_lockup_trigger);
	if (err < 0) {
		dev_err(&pdev->dev, "create lockup trigger failed %d\n", err);
		goto err2;
	}

    err = device_create_file(&pdev->dev, &dev_attr_kgdb_support);
	if (err < 0) {
		dev_err(&pdev->dev, "create kgdb_support failed %d\n", err);
		goto err2;
	}

    err = device_create_file(&pdev->dev, &dev_attr_callback_sel);
	if (err < 0) {
		dev_err(&pdev->dev, "create callback_sel failed %d\n", err);
		goto err2;
	}

    return 0;
err2:
    iounmap(pdebugger->base_mem_share);
err1:
    kfree(pdebugger);
    pdebugger = NULL;
    dev_err(&pdev->dev, "err %d\n", err);
	return err;
}

static int hobot_fiqdebug_remove(struct platform_device *pdev)
{
    struct hobot_fiq_debugger *debugger = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_fiq_enable);
	device_remove_file(&pdev->dev, &dev_attr_lockup_trigger);
	device_remove_file(&pdev->dev, &dev_attr_kgdb_support);
	device_remove_file(&pdev->dev, &dev_attr_callback_sel);
    kfree(debugger);
    debugger = NULL;
    return 0;
}

static const struct of_device_id hobot_fiqdebug_of_match[] = {
	{ .compatible = "hobot,fiq-debugger", },
	{ /* end of table */ }
};
MODULE_DEVICE_TABLE(of, hobot_fiqdebug_of_match);


/* Driver Structure */
static struct platform_driver hobot_fiqdebug_driver = {
	.probe		= hobot_fiqdebug_probe,
	.remove 	= hobot_fiqdebug_remove,
	.driver 	= {
		.name	= HOBOT_FIQDEBUG_NAME,
		.of_match_table = hobot_fiqdebug_of_match,
	},
};

static int __init hobot_fiqdebug_init(void)
{
    int ret = platform_driver_register(&hobot_fiqdebug_driver);
    if (ret)
        pr_err("platform_driver_register failed: %d\n", ret);

    return ret;
}
static void __exit hobot_fiqdebug_exit(void)
{
    platform_driver_unregister(&hobot_fiqdebug_driver);
}


late_initcall(hobot_fiqdebug_init);

module_exit(hobot_fiqdebug_exit);


MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Liwei Zhang<liwei.zhang@hobot.cc>");
MODULE_DESCRIPTION("HOBOT FIQ DEBUGGER");
