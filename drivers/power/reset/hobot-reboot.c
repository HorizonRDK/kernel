/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/dma-mapping.h>
#include <linux/kdebug.h>
#include <asm/proc-fns.h>
#include <asm/system_misc.h>

#define HOBOT_REBOOT_OPT    0x00001001
static void __iomem *base;
static u32 reboot_offset;

#define HOBOT_SWINFO_SIZE_MAX   0x10
#define HOBOT_SWINFO_MAGIC_MEMI 0
#define HOBOT_SWINFO_MAGIC_CODE 0x57534248
static struct kobject *k_obj;
static struct mutex swinfo_lock;
static void __iomem *swreg_base;
static void __iomem *swmem_base;
static void __iomem *swinfo_base;
//static void __iomem *swfifo_base;
static u32 swinfo_size;
static u32 swinfo_ro;
//static u32 swfifo_size;
static u32 swi_boot[3];
static u32 swi_dump[3];
static u32 swinfo_ptype, swinfo_preg;
static u32 cpu_num = NR_CPUS;
static dma_addr_t hobot_cpu_data_paddr;

int hobot_swinfo_set(u32 sel, u32 index, u32 mask, u32 value)
{
	u32 rega, regv;
	void __iomem *sw_base;

	sw_base = (sel == 2) ? swmem_base :
		((sel == 1) ? swreg_base : swinfo_base);
	rega = index << 2;

	if (sw_base == NULL || swinfo_size == 0)
		return -ENODEV;
	if (rega >= swinfo_size)
		return -EINVAL;
	if (index < 32 && ((0x1 << index) & swinfo_ro))
		return -EACCES;

	mutex_lock(&swinfo_lock);
	regv = readl_relaxed(sw_base + rega);
	regv = (regv & ~mask) | (value & mask);

	writel_relaxed(regv, sw_base + rega);

	/* set magic code if mem */
	if (sw_base == swmem_base && index != HOBOT_SWINFO_MAGIC_MEMI)
		writel_relaxed(HOBOT_SWINFO_MAGIC_CODE,
				sw_base + (HOBOT_SWINFO_MAGIC_MEMI << 2));
	mutex_unlock(&swinfo_lock);

	return 0;
}
EXPORT_SYMBOL(hobot_swinfo_set);

int hobot_swinfo_get(u32 sel, u32 index, u32 mask, u32 *value)
{
	u32 rega, regv;
	void __iomem *sw_base;

	sw_base = (sel == 2) ? swmem_base :
		((sel == 1) ? swreg_base : swinfo_base);
	rega = index << 2;

	if (sw_base == NULL || swinfo_size == 0)
		return -ENODEV;
	if (rega >= swinfo_size || value == NULL)
		return -EINVAL;

	mutex_lock(&swinfo_lock);
	regv = readl_relaxed(sw_base + rega);
	*value = regv & mask;
	mutex_unlock(&swinfo_lock);

	return 0;
}
EXPORT_SYMBOL(hobot_swinfo_get);

u32 hobot_swinfo_sel(u32 sel)
{
	if (sel > 0) {
		mutex_lock(&swinfo_lock);
		if (sel == 2 && swmem_base)
			swinfo_base = swmem_base;
		else if (sel == 1 && swreg_base)
			swinfo_base = swreg_base;

		/* clean magic code if reg */
		if (swinfo_base == swreg_base && swmem_base)
			writel_relaxed(0x0, swmem_base +
				(HOBOT_SWINFO_MAGIC_MEMI << 2));
		mutex_unlock(&swinfo_lock);
	}
	return (swinfo_base == swreg_base) ? 1 : 2;
}
EXPORT_SYMBOL(hobot_swinfo_sel);

int hobot_swinfo_boot(u32 type)
{
	if (type > (swi_boot[1] >> swi_boot[2]))
		return -EINVAL;
	return hobot_swinfo_set(0, swi_boot[0], swi_boot[1], type << swi_boot[2]);
}
EXPORT_SYMBOL(hobot_swinfo_boot);

int hobot_swinfo_dump(u32 ip)
{
	if (ip > (swi_dump[1] >> swi_dump[2]))
		return -EINVAL;
	return hobot_swinfo_set(0, swi_dump[0], swi_dump[1], ip << swi_dump[2]);
}
EXPORT_SYMBOL(hobot_swinfo_dump);

int hobot_swinfo_panic(void)
{
	if (swinfo_ptype == 1) {
		pr_info("swinfo panic boot %d\n", swinfo_preg);
		return hobot_swinfo_boot(swinfo_preg);
	} else if (swinfo_ptype == 2) {
		pr_info("swinfo panic dump %x\n", swinfo_preg);
		return hobot_swinfo_dump(swinfo_preg);
	}
	return 0;
}
EXPORT_SYMBOL(hobot_swinfo_panic);

static ssize_t hobot_swinfo_sel_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	u32 sel;

	sel = hobot_swinfo_sel(0);
	s += sprintf(s, "%s: %d %s\n", attr->attr.name, sel,
			(sel == 1) ? "reg" : "mem");

	return (s - buf);
}

static ssize_t hobot_swinfo_sel_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, error = -EINVAL;
	u32 sel = -1;

	if (strncmp(buf, "reg", 3) == 0)
		sel = 1;
	else if (strncmp(buf, "mem", 3) == 0)
		sel = 2;
	else
		ret = kstrtouint(buf, 0, &sel);

	if (sel == 1 || sel == 2) {
		hobot_swinfo_sel(sel);
		error = 0;
	}

	return error ? error : count;
}

static u32 hobot_swinfo_attr_sel(struct kobj_attribute *attr)
{
	u32 i;
	const char * const swinfo_sel[] = {
		"swinfo", "swreg", "swmem"
	};

	for (i = 0; i < 3; i++) {
		if (strcmp(attr->attr.name, swinfo_sel[i]) == 0)
			return i;
	}
	return 0;
}

static ssize_t hobot_swinfo_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	u32 sel, i, regv;

	sel = hobot_swinfo_attr_sel(attr);
	sel = (sel) ? sel : hobot_swinfo_sel(0);
	for (i = 0; i < (swinfo_size >> 2); i++) {
		if (hobot_swinfo_get(sel, i, -1, &regv) == 0)
			s += sprintf(s, "%2d[%02X-%s]: 0x%08X%s\n",
			i, i << 2,
			(i < 32 && (swinfo_ro & (0x1 << i))) ? "ro" : "rw",
			regv,
			(sel == 2 && i == HOBOT_SWINFO_MAGIC_MEMI) ?
			" [MAGIC]" : "");
		else
			break;
		}
	if (i == 0)
		s += sprintf(s, "no %s\n", attr->attr.name);

	return (s - buf);
}

static ssize_t hobot_swinfo_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, error = -EINVAL;
	char *sv, *sm, *so;
	u32 sel, i = -1, regv = 0, mask = -1, offs = 0;

	sel = hobot_swinfo_attr_sel(attr);
	sv = memchr(buf, '=', count);
	if (sv) {
		*sv = '\0';
		ret = kstrtouint(buf, 0, &i);
		sv++;
		sm = memchr(sv, ',', count - (sv - buf));
		if (sm) {
			*sm = '\0';
			sm++;
			so = memchr(sm, ',', count - (sm - buf));
			if (so) {
				*so = '\0';
				so++;
				ret = kstrtouint(so, 0, &offs);
				if (offs >= 32)
					offs = 0;
			}
			ret = kstrtouint(sm, 0, &mask);
		}
		ret = kstrtouint(sv, 0, &regv);
	}
	error = hobot_swinfo_set(sel, i, mask << offs, regv << offs);

	return error ? error : count;
}



static const char * const hobot_swi_boot_desp[] = {
	"normal", "splonce", "ubootonce",
	"splwait", "ubootwait", "udumptf", "udumpemmc",
	"udumpusb", "udumpfastboot", "unknown"
};

static ssize_t hobot_swinfo_boot_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	u32 regv;
	u32 *swi = swi_boot;
	const char * const *srd = hobot_swi_boot_desp;
	int srd_n = ARRAY_SIZE(hobot_swi_boot_desp);

	if (hobot_swinfo_get(0, swi[0], -1, &regv) == 0) {
		regv = (regv & swi[1]) >> swi[2];
		srd_n = (regv < srd_n) ? regv : (srd_n - 1);
		s += sprintf(s, "%s: %d %s\n",
					 attr->attr.name, regv, srd[srd_n]);
	} else {
		s += sprintf(s, "%s: %d %x %d error\n",
			 attr->attr.name, swi[0], swi[1], swi[2]);
	}

	return (s - buf);
}

static ssize_t hobot_swinfo_boot_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, i, error = -EINVAL;
	u32 type = -1;
	const char * const *srd = hobot_swi_boot_desp;
	int srd_n = ARRAY_SIZE(hobot_swi_boot_desp);

	for (i = 0; i < (srd_n - 1); i++) {
		if (strncmp(buf, srd[i], 6) == 0)
			type = i;
	}
	if (type == -1)
		ret = kstrtouint(buf, 0, &type);

	error = hobot_swinfo_boot(type);

	return error ? error : count;
}

static ssize_t hobot_swinfo_dump_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	u32 regv;
	u32 *swi = swi_dump;

	if (hobot_swinfo_get(0, swi[0], -1, &regv) == 0) {
		regv = (regv & swi[1]) >> swi[2];
		s += sprintf(s, "%s: 0x%08X %d.%d.%d.%d\n",
				attr->attr.name, regv,
				(regv >> 24) & 0xff, (regv >> 16) & 0xff,
				(regv >> 8) & 0xff, regv & 0xff);
	} else {
		s += sprintf(s, "%s: %d %x %d error\n",
				attr->attr.name, swi[0], swi[1], swi[2]);
	}

	return (s - buf);
}

static ssize_t hobot_swinfo_dump_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret, error = -EINVAL;
	int ipx[4];
	u32 ip = 0;

	if (strchr(buf, '.') &&
		sscanf(buf, "%d.%d.%d.%d", &ipx[0], &ipx[1],
			&ipx[2], &ipx[3]) == 4) {
		ip = (ipx[0] << 24) | (ipx[1] << 16) | (ipx[2] << 8) | ipx[3];
		ret = 0;
	} else {
		ret = kstrtouint(buf, 0, &ip);
	}
	if (ret == 0)
		error = hobot_swinfo_dump(ip);

	return error ? error : count;
}

static ssize_t hobot_swinfo_panic_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	u32 regv;
	const char * const *srd = hobot_swi_boot_desp;
	int srd_n = ARRAY_SIZE(hobot_swi_boot_desp);

	regv = swinfo_preg;
	if (swinfo_ptype) {
		if (swinfo_ptype == 1) {
			srd_n = (regv < srd_n) ? regv : (srd_n - 1);
			s += sprintf(s, "%s: boot=%d %s\n",
					 attr->attr.name, regv, srd[srd_n]);
		} else if (swinfo_ptype == 2) {
			s += sprintf(s, "%s: dump=0x%08X %d.%d.%d.%d\n",
				attr->attr.name, regv,
				(regv >> 24) & 0xff, (regv >> 16) & 0xff,
				(regv >> 8) & 0xff, regv & 0xff);
		}
	} else {
		s += sprintf(s, "%s: none\n", attr->attr.name);
	}

	return (s - buf);
}

static ssize_t hobot_swinfo_panic_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int i, ret, error = -EINVAL;
	u32 *swi;
	const char * const *srd = hobot_swi_boot_desp;
	char *s;
	int srd_n = ARRAY_SIZE(hobot_swi_boot_desp);
	int ipx[4];
	int type = -1;
	u32 reg = -1;

	if (strncmp(buf, "0", 1) == 0) {
		swinfo_ptype = 0;
		swinfo_preg = 0;
		return count;
	}

	s = strchr(buf, '=');
	if (s == NULL)
		return error;
	if (strncmp(buf, "boot", 4) == 0) {
		type = 1;
		swi = swi_boot;
	} else if (strncmp(buf, "dump", 4) == 0) {
		type = 2;
		swi = swi_dump;
	} else {
		return error;
	}

	s++;
	if (type == 1) {
		for (i = 0; i < (srd_n - 1); i++) {
			if (strncmp(s, srd[i], 6) == 0)
				reg = i;
		}
	} else if (strchr(s, '.') &&
		sscanf(s, "%d.%d.%d.%d", &ipx[0], &ipx[1],
			&ipx[2], &ipx[3]) == 4) {
		reg = (ipx[0] << 24) | (ipx[1] << 16) | (ipx[2] << 8) | ipx[3];
	}
	if (reg == -1)
		ret = kstrtouint(s, 0, &reg);

	if (reg != -1 && reg <= (swi[1] >> swi[2])) {
		swinfo_ptype = type;
		swinfo_preg = reg;
		error = 0;
	}

	return error ? error : count;
}

static struct kobj_attribute hobot_swinfo_sel_attribute =
	__ATTR(sel, 0644, hobot_swinfo_sel_show, hobot_swinfo_sel_store);
static struct kobj_attribute hobot_swinfo_info_attribute =
	__ATTR(swinfo, 0644, hobot_swinfo_show, hobot_swinfo_store);
static struct kobj_attribute hobot_swinfo_reg_attribute =
	__ATTR(swreg, 0644, hobot_swinfo_show, hobot_swinfo_store);
static struct kobj_attribute hobot_swinfo_mem_attribute =
	__ATTR(swmem, 0644, hobot_swinfo_show, hobot_swinfo_store);
static struct kobj_attribute hobot_swinfo_boot_attribute =
	__ATTR(boot, 0644, hobot_swinfo_boot_show, hobot_swinfo_boot_store);
static struct kobj_attribute hobot_swinfo_dump_attribute =
	__ATTR(dump, 0644, hobot_swinfo_dump_show, hobot_swinfo_dump_store);
static struct kobj_attribute hobot_swinfo_panic_attribute =
	__ATTR(panic, 0644, hobot_swinfo_panic_show, hobot_swinfo_panic_store);

static struct attribute *hobot_swinfo_attributes[] = {
	&hobot_swinfo_sel_attribute.attr,
	&hobot_swinfo_info_attribute.attr,
	&hobot_swinfo_reg_attribute.attr,
	&hobot_swinfo_mem_attribute.attr,
	&hobot_swinfo_boot_attribute.attr,
	&hobot_swinfo_dump_attribute.attr,
	&hobot_swinfo_panic_attribute.attr,
	NULL
};

static const struct attribute_group hobot_swinfo_attr_group = {
	.attrs = hobot_swinfo_attributes,
};

static void hobot_swinfo_bit2mask(int bs, int be, u32 *mask, u32 *offset)
{
	int bt;

	if (bs > be) {
		bt = bs;
		bs = be;
		be = bt;
	}
	if (bs < 0 || bs > 31)
		bs = 0;
	if (be < 0 || be > 31)
		be = 31;
	*mask = (((0x1 << bs) - 1) ^ (((0x2 << be) - 1)));
	*offset = bs;
}

struct hobot_cpu_data_s {
	struct pt_regs pt_regs;
	bool is_online;
	struct task_struct *cur_task;
};

static struct hobot_cpu_data_s *hobot_cpu_data;


void print_cpu_data(int id)
{
    struct pt_regs *pt_regs = &hobot_cpu_data[id].pt_regs;
	int i;

	for (i = 0; i < 31; i++) {
		if (i < 10) /* Keep "=" align */
			pr_debug("x%d    = 0x%016llx\n",
				i, pt_regs->regs[i]);
		else
			pr_debug("x%d   = 0x%016llx\n",
				i, pt_regs->regs[i]);
	}
	pr_debug("sp    = 0x%016llx\n", pt_regs->sp);
	pr_debug("pc    = 0x%016llx\n", pt_regs->pc);
}

void hobot_swinfo_crash_save_cpu(unsigned int cpu, struct pt_regs *regs)
{
	pr_info("hobot_swinfo_crash_save_cpu: saving regs for cpu %d\n", cpu);

	memcpy(&hobot_cpu_data[cpu].pt_regs, regs, sizeof(struct pt_regs));
}

enum ipi_msg_type {
    IPI_RESCHEDULE,
    IPI_CALL_FUNC,
    IPI_CPU_STOP,
    IPI_CPU_CRASH_STOP,
    IPI_TIMER,
    IPI_IRQ_WORK,
    IPI_WAKEUP
};

extern void (*__smp_cross_call)(const struct cpumask *, unsigned int);
static int hobot_swinfo_die_handler(struct notifier_block *nb,
			unsigned long reason, void *data)
{
	int cpu;
	struct cpumask  mask;
	struct die_args *args = (struct die_args *)data;

	cpu = smp_processor_id();

	memcpy(&hobot_cpu_data[cpu].pt_regs, args->regs, sizeof(struct pt_regs));

	cpumask_copy(&mask, cpu_online_mask);
	cpumask_clear_cpu(cpu, &mask);

	pr_crit("swinfo: saving other cpus context and stopping them\n");
	__smp_cross_call(&mask, IPI_CPU_CRASH_STOP);

	return NOTIFY_OK;
}

static struct notifier_block hobot_swinfo_die_notifier = {
	.notifier_call = hobot_swinfo_die_handler,
	.priority      = INT_MAX-1,
};

static int hobot_swinfo_panic_handler(struct notifier_block *this,
                    unsigned long event, void *unused)
{
	int id;

	for (id = 0; id < cpu_num; id++)
		print_cpu_data(id);

	return NOTIFY_OK;
}
struct notifier_block hobot_swinfo_panic_notifier = {
    .notifier_call = hobot_swinfo_panic_handler,
    .priority      = INT_MAX-1, /* priority: INT_MAX >= x >= 0 */
};

static int hobot_restart_handler(struct notifier_block *this,
				unsigned long mode, void *cmd)
{
	writel_relaxed(HOBOT_REBOOT_OPT, base + reboot_offset);

	return NOTIFY_DONE;
}

static void hobot_pm_restart(enum reboot_mode mode, const char *cmd)
{
	writel_relaxed(HOBOT_REBOOT_OPT, base + reboot_offset);
}

static struct notifier_block hobot_restart_nb = {
	.notifier_call = hobot_restart_handler,
	.priority = 128,
};

static int hobot_reboot_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *npm;
	struct resource r;
	u32 swreg_offset, swmem_magic, swinfo_sel, b = 0, d = 0;
	u32 swi[3], i;
	int err;

	base = of_iomap(of_get_parent(np), 0);
	if (!base) {
		WARN(1, "failed to map base address");
		return -ENODEV;
	}

	if (of_property_read_u32(np, "reboot-offset", &reboot_offset) < 0) {
		pr_err("failed to find reboot-offset property\n");
		iounmap(base);
		return -EINVAL;
	}

	err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&pdev->dev,
			"dma_set_coherent_mask 32 fails with: %d\n", err);
		return err;
	}

	hobot_cpu_data = dma_alloc_coherent(&pdev->dev,
			sizeof(struct hobot_cpu_data_s) * cpu_num, &hobot_cpu_data_paddr, 0);

	if (hobot_cpu_data == NULL) {
		pr_err("%s: can not allocate cpu_data buffer\n", __func__);
		return -ENOMEM;
	}
	memset(hobot_cpu_data, 0, sizeof(struct hobot_cpu_data_s) * cpu_num);

	err = register_die_notifier(&hobot_swinfo_die_notifier);
	if (err != 0) {
		pr_err("%s: registering die notifier failed with err=%d\n",
				__func__, err);
		return err;
	}

    err = atomic_notifier_chain_register(&panic_notifier_list,
        &hobot_swinfo_panic_notifier);
    if (err != 0) {
        pr_err("%s: unable to register a panic notifier (err=%d)\n",
                __func__, err);
        return err;
    }

	err = register_restart_handler(&hobot_restart_nb);
	if (err) {
		dev_err(&pdev->dev, "cannot register restart handler (err=%d)\n",
			err);
		iounmap(base);
	}

	if (IS_ENABLED(CONFIG_HOBOT_XJ2))
		arm_pm_restart = hobot_pm_restart;

	mutex_init(&swinfo_lock);
	if (of_property_read_u32(np, "swinfo-size", &swinfo_size) < 0 ||
		swinfo_size > HOBOT_SWINFO_SIZE_MAX) {
		swinfo_size = HOBOT_SWINFO_SIZE_MAX;
	}
	if (of_property_read_u32(np, "swreg-offset", &swreg_offset) == 0) {
		swreg_base = base + swreg_offset;
		pr_debug("hobot swinfo reg vaddr=%p,size=0x%x\n",
			swreg_base, swinfo_size);
	}
	if (of_property_read_u32(np, "swinfo-ro", &swinfo_ro) < 0)
		swinfo_ro = 0;

	if (of_property_read_u32_array(np, "swi-boot", swi, 3) == 0 &&
			swi[0] < (swinfo_size >> 2)) {
		swi_boot[0] = swi[0];
		hobot_swinfo_bit2mask(swi[1], swi[2], &swi_boot[1], &swi_boot[2]);
	} else {
		swi_boot[0] = -1;
	}
	if (of_property_read_u32_array(np, "swi-dump", swi, 3) == 0 &&
			swi[0] < (swinfo_size >> 2)) {
		swi_dump[0] = swi[0];
		hobot_swinfo_bit2mask(swi[1], swi[2], &swi_dump[1], &swi_dump[2]);
	} else {
		swi_dump[0] = -1;
	}

	if (of_property_read_u32(np, "swinfo-sel", &swinfo_sel) < 0 ||
		swinfo_sel == 0)
		swinfo_sel = 2;

	npm = of_parse_phandle(np, "memory-region", 0);
	if (npm) {
		err = of_address_to_resource(npm, 0, &r);
		if (err == 0 && resource_size(&r) >= swinfo_size)
			swmem_base = ioremap_nocache(r.start,
					resource_size(&r));
	}

	if (swmem_base) {
		hobot_swinfo_get(2, HOBOT_SWINFO_MAGIC_MEMI, -1, &swmem_magic);
		if (swmem_magic == HOBOT_SWINFO_MAGIC_CODE) {
			swinfo_sel = 2;
			pr_debug("hobot swinfo mem paddr=%p,vaddr=%p,size=0x%llx magic\n",
				(void *)r.start, swmem_base, resource_size(&r));
		} else {
			pr_debug("hobot swinfo mem paddr=%p,vaddr=%p,size=0x%llx clean\n",
				(void *)r.start, swmem_base, resource_size(&r));
			for (i = 0; i < resource_size(&r); i += 4)
				*(u32 *)(swmem_base + i) = 0x0;
			hobot_swinfo_get(1, swi_boot[0], -1, &b);
			hobot_swinfo_get(1, swi_dump[0], -1, &d);
			if (b & swi_boot[1] || d & swi_dump[1])
				swinfo_sel = 1;
		}
	} else {
		swinfo_sel = 1;
	}
	hobot_swinfo_sel(swinfo_sel);

	pr_info("hobot swinfo sel %s ro=0x%x%s%s\n",
			(hobot_swinfo_sel(0) == 1) ? "reg" : "mem", swinfo_ro,
			(swi_boot[0] == -1) ? "" : " boot",
			(swi_dump[0] == -1) ? "" : " dump");

	k_obj = kobject_create_and_add("hobot-swinfo", kernel_kobj);
	if (k_obj) {
		if (sysfs_create_group(k_obj, &hobot_swinfo_attr_group)) {
			pr_warn("hobot-swinfo sys group create error\n");
			kobject_put(k_obj);
			k_obj = NULL;
		}
	} else {
		pr_warn("hobot-swinfo sys node create error\n");
	}

	return err;
}

static int hobot_reboot_remove(struct platform_device *pdev)
{
	if (k_obj) {
		sysfs_remove_group(k_obj, &hobot_swinfo_attr_group);
		kobject_put(k_obj);
	}

	dma_free_coherent(&pdev->dev,
			sizeof(struct hobot_cpu_data_s) * cpu_num,
			hobot_cpu_data, hobot_cpu_data_paddr);

	unregister_die_notifier(&hobot_swinfo_die_notifier);
    atomic_notifier_chain_unregister(&panic_notifier_list,
            &hobot_swinfo_panic_notifier);

	return 0;
}

static const struct of_device_id hobot_reboot_of_match[] = {
	{ .compatible = "hobot,hobot-power" },
	{}
};

static struct platform_driver hobot_reboot_driver = {
	.probe = hobot_reboot_probe,
	.remove = hobot_reboot_remove,
	.driver = {
		.name = "hobot-reboot",
		.of_match_table = hobot_reboot_of_match,
	},
};
module_platform_driver(hobot_reboot_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("Hobot Reboot driver");
MODULE_LICENSE("GPL v2");
