/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2020 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/string.h>


/* the offset of pwm registers */
#define LPWM_EN       0x00
#define LPWM0_CFG     0x04
#define LPWM1_CFG     0x08
#define LPWM2_CFG     0x0C
#define LPWM3_CFG     0x10
#define LPWM_SW_TRIG  0x14
#define LPWM_RST      0x20

#define LPWM_INT_EN   BIT(4)
#define LPWM_MODE_PPS_TRIG BIT(5)
#define LPWM_NAME     "hobot-lpwm"
#define LPWM_NPWM 4
#define LPWM_PPS  4
#define LPWM_NPIN 5

#define LPWM_CLASS_NAME "hobot_lpwm_dev"

struct lpwm_cdev_state {
	unsigned int lpwm_num;
	unsigned int period;
	unsigned int duty_cycle;
	bool enabled;
};

#define LPWM_CDEV_MAGIC 'L'
#define LPWM_CDEV_INIT		_IOW(LPWM_CDEV_MAGIC, 0x12, unsigned int)
#define LPWM_CDEV_DEINIT	_IOW(LPWM_CDEV_MAGIC, 0x13, unsigned int)
#define LPWM_CONF_SINGLE	_IOW(LPWM_CDEV_MAGIC, 0x14, struct lpwm_cdev_state)
#define LPWM_ENABLE_SINGLE	_IOW(LPWM_CDEV_MAGIC, 0x15, unsigned int)
#define LPWM_DISABLE_SINGLE _IOW(LPWM_CDEV_MAGIC, 0x16, unsigned int)
#define LPWM_SWTRIG			_IOW(LPWM_CDEV_MAGIC, 0x17, int)
#define LPWM_PPSTRIG		_IOW(LPWM_CDEV_MAGIC, 0x18, int)
#define LPWM_GET_STATE		_IOWR(LPWM_CDEV_MAGIC, 0x19, struct lpwm_cdev_state)
#define LPWM_OFFSET_CONF	_IOW(LPWM_CDEV_MAGIC, 0x20, int)
#define LPWM_OFFSET_STATE	_IOR(LPWM_CDEV_MAGIC, 0x21, int)

static unsigned int swtrig_period = 0;
module_param(swtrig_period, uint, 0644);

struct lpwm_chip_cdev {
	const char	*name;
	int major;
	int minor;
	struct cdev cdev;
	struct device *dev;
	dev_t dev_num;
	struct class *lpwm_cls;
	struct pwm_device *pwm[LPWM_NPWM];
	struct hobot_lpwm_chip *lpwm;
};

struct hobot_lpwm_chip {
	struct pwm_chip chip;
	int irq;
	char name[8];
	struct clk *clk;
	void __iomem *base;
	int offset[LPWM_NPWM];
	struct hrtimer swtrig_timer;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins[LPWM_NPIN];
	struct mutex lpwm_mutex;
};

#define to_hobot_lpwm_chip(_chip) \
	container_of(_chip, struct hobot_lpwm_chip, chip)


#define to_lpwm_chip_cdev(_cdev) \
	container_of(_cdev, struct lpwm_chip_cdev, cdev)

static struct lpwm_chip_cdev *lpwm_cdev = NULL;

//	Read / write function declaration
static inline u32 hobot_lpwm_rd(struct hobot_lpwm_chip*, u32);
static inline void hobot_lpwm_wr(struct hobot_lpwm_chip*, u32, u32);

static int lpwm_chip_open(struct inode *pinode, struct file *pfile)
{
	struct lpwm_chip_cdev *lpwm_cdev_p;
	lpwm_cdev_p = to_lpwm_chip_cdev(pinode->i_cdev);
	pfile->private_data = lpwm_cdev_p;
	return 0;
}

static int lpwm_chip_release(struct inode *pinode, struct file *pfile)
{
	pfile->private_data = NULL;
	return 0;
}

static long lpwm_chip_ioctl(struct file *pfile, unsigned int cmd,
							unsigned long args)
{
	int ret = 0;
	void	__user *arg = (void __user *)args;
	struct lpwm_chip_cdev *lpwm_cdev_pctl;

	lpwm_cdev_pctl = pfile->private_data;

	switch (cmd) {
	case LPWM_CDEV_INIT:
		{
			unsigned int num;
			if (copy_from_user(&num, arg, sizeof(unsigned int)))
				return -EFAULT;

			if (num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (!lpwm_cdev_pctl->pwm[num]) {
				lpwm_cdev_pctl->pwm[num] = pwm_request_from_chip(
						&lpwm_cdev_pctl->lpwm->chip, num, "sysfs");
				if (IS_ERR(lpwm_cdev_pctl->pwm[num])) {
					pr_err("lpwm%u device request failed\n", num);
					mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
					return PTR_ERR(lpwm_cdev_pctl->pwm[num]);
				}
				pr_debug("lpwm device request okay!!!\n");
			} else {
				pr_debug("lpwm device already exists!\n");
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_CDEV_DEINIT:
		{
			unsigned int num;
			if (copy_from_user(&num, arg, sizeof(unsigned int)))
				return -EFAULT;

			if (num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (lpwm_cdev_pctl->pwm[num]) {
				pwm_free(lpwm_cdev_pctl->pwm[num]);
				lpwm_cdev_pctl->pwm[num] = NULL;
				pr_debug("lpwm device free okay!!!\n");
			} else {
				pr_err("lpwm%u device not init!!!\n", num);
				mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
				return -ENODEV;
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_CONF_SINGLE:
		{
			struct lpwm_cdev_state cdev_state;
			if (copy_from_user(&cdev_state, arg, sizeof(cdev_state)))
				return -EFAULT;

			if (cdev_state.lpwm_num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (lpwm_cdev_pctl->pwm[cdev_state.lpwm_num]) {
				ret = pwm_config(lpwm_cdev_pctl->pwm[cdev_state.lpwm_num],
						cdev_state.duty_cycle, cdev_state.period);
				if (ret) {
					pr_err("\nError config lpwm%u!!!!\n", cdev_state.lpwm_num);
					mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
					return ret;
				}
				pr_debug("lpwm_cdev config is okay!!!\n");
			} else {
				pr_err("\nlpwm_cdev:lpwm%u should be init first!!!!\n",
						cdev_state.lpwm_num);
				mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
				return -ENODEV;
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_ENABLE_SINGLE:
		{
			unsigned int num;
			if (copy_from_user(&num, arg, sizeof(unsigned int)))
				return -EFAULT;

			if (num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (lpwm_cdev_pctl->pwm[num]) {
				ret = pwm_enable(lpwm_cdev_pctl->pwm[num]);
				if (ret) {
					pr_err("\nError enable pwm%u!!!!\n", num);
					mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
					return ret;
				}
				pr_debug("pwm enable is okay!!!\n");
			} else {
				pr_err("\nlpwm_cdev:lpwm%u should be init first!!!!\n", num);
				mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
				return -ENODEV;
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_DISABLE_SINGLE:
		{
			unsigned int num;
			if (copy_from_user(&num, arg, sizeof(unsigned int)))
				return -EFAULT;

			if (num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (lpwm_cdev_pctl->pwm[num]) {
				pwm_disable(lpwm_cdev_pctl->pwm[num]);
				pr_debug("pwm disable is okay!!!\n");
			} else {
				pr_err("\nlpwm_cdev:lpwm%u should be init first!!!!\n", num);
				mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
				return -ENODEV;
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_SWTRIG:
		{
			int val;
			if (copy_from_user(&val, arg, sizeof(int)))
				return -EFAULT;

			pr_info("trigger lpwms, val:%d\n", val);
			val = (val == 0) ? 1 : val;

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (swtrig_period) {
				hrtimer_start(&lpwm_cdev_pctl->lpwm->swtrig_timer,
						ms_to_ktime(swtrig_period), HRTIMER_MODE_REL);
			}
			hobot_lpwm_wr(lpwm_cdev_pctl->lpwm, LPWM_SW_TRIG, val);
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_PPSTRIG:
		{
			int val;
			if (copy_from_user(&val, arg, sizeof(int)))
				return -EFAULT;

			pr_info("pps trigger lpwms, val:%d\n", val);

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			if (val == 0) {
				val = hobot_lpwm_rd(lpwm_cdev_pctl->lpwm, LPWM_EN);
				val &= ~(LPWM_MODE_PPS_TRIG);
			} else {
				if (lpwm_cdev_pctl->lpwm->pinctrl != NULL &&
						lpwm_cdev_pctl->lpwm->pins[LPWM_PPS] != NULL)
					pinctrl_select_state(lpwm_cdev_pctl->lpwm->pinctrl,
							lpwm_cdev_pctl->lpwm->pins[LPWM_PPS]);
				udelay(100);
				val = hobot_lpwm_rd(lpwm_cdev_pctl->lpwm, LPWM_EN);
				val |= LPWM_MODE_PPS_TRIG;
			}
			hobot_lpwm_wr(lpwm_cdev_pctl->lpwm, LPWM_EN, val);
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_GET_STATE:
		{
			struct lpwm_cdev_state cdev_state;
			struct pwm_state prestate;
			if (copy_from_user(&cdev_state, arg, sizeof(cdev_state)))
				return -EFAULT;

			if (cdev_state.lpwm_num > 3) {
				pr_err("lpwm_num should be 0~3 \n");
				return -ERANGE;
			}

			if (lpwm_cdev_pctl->pwm[cdev_state.lpwm_num]) {
				pwm_get_state(lpwm_cdev_pctl->pwm[cdev_state.lpwm_num],
								&prestate);
				cdev_state.period = prestate.period;
				cdev_state.duty_cycle = prestate.duty_cycle;
				cdev_state.enabled = prestate.enabled;
				if (copy_to_user(arg, &cdev_state, sizeof(cdev_state)))
					return -EFAULT;

				pr_debug("lpwm_cdev get state okay!!!\n");
			} else {
				pr_err("\nlpwm_cdev:lpwm%u should be init first!!!!\n",
						cdev_state.lpwm_num);
				return -ENODEV;
			}
		}
		break;
	case LPWM_OFFSET_CONF:
		{
			int offset_us[4];
			int val;
			int i;
			if (copy_from_user(&offset_us, arg, sizeof(offset_us)))
				return -EFAULT;

			mutex_lock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
			for (i = 0; i < LPWM_NPWM; i++) {
				if (offset_us[i] < 10 || offset_us[i] > 40960) {
					pr_info("lpwm offset should be in [10, 40960] us\n");
					mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
					return -EINVAL;
				}
				lpwm_cdev_pctl->lpwm->offset[i] = offset_us[i];
				val = hobot_lpwm_rd(lpwm_cdev_pctl->lpwm, (i * 0x4)
									+ LPWM0_CFG);
				val &= 0xFFFFF000;
				val |= offset_us[i] / 10 - 1;
				hobot_lpwm_wr(lpwm_cdev_pctl->lpwm, (i * 0x4) + LPWM0_CFG, val);
			}
			mutex_unlock(&lpwm_cdev_pctl->lpwm->lpwm_mutex);
		}
		break;
	case LPWM_OFFSET_STATE:
		{
			int offset_us[4];
			int i;
			for (i = 0; i < LPWM_NPWM; i++)
				offset_us[i] = lpwm_cdev_pctl->lpwm->offset[i];

			if (copy_to_user(arg, &offset_us, sizeof(offset_us)))
				return -EFAULT;
		}
		break;
	default:
		break;
	}
	return ret;
}

static const struct file_operations lpwm_cdev_ops = {
	.owner		= THIS_MODULE,
	.open		= lpwm_chip_open,
	.release	= lpwm_chip_release,
	.unlocked_ioctl = lpwm_chip_ioctl,
	.compat_ioctl   = lpwm_chip_ioctl,
};

static int lpwm_chip_cdev_create(struct hobot_lpwm_chip *lpwm)
{
	int ret = 0;
	struct device *devf = NULL;

	lpwm_cdev = devm_kzalloc(lpwm->chip.dev,
			sizeof(struct lpwm_chip_cdev), GFP_KERNEL);
	if (!lpwm_cdev) {
		printk("no memory for lpwm_cdev\n");
		return -ENOMEM;
	}
	memset(lpwm_cdev, 0, sizeof(struct lpwm_chip_cdev));

	lpwm_cdev->lpwm = lpwm;
	lpwm_cdev->dev = lpwm->chip.dev;
	lpwm_cdev->name = "lpwm_cdev";

	ret = alloc_chrdev_region(&lpwm_cdev->dev_num, 0, 1, lpwm_cdev->name);
	if (!ret) {
		lpwm_cdev->major = MAJOR(lpwm_cdev->dev_num);
		lpwm_cdev->minor = MINOR(lpwm_cdev->dev_num);
	} else {
		pr_err("%s: error alloc chrdev region with return value %d!\n",
				__func__, ret);
		goto err;
	}

	cdev_init(&lpwm_cdev->cdev, &lpwm_cdev_ops);
	lpwm_cdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&lpwm_cdev->cdev, lpwm_cdev->dev_num, 1);
	if (ret) {
		unregister_chrdev_region(lpwm_cdev->dev_num, 1);
		pr_err("%s: error add cdev with return value %d\n", __func__, ret);
		goto err1;
	}

	lpwm_cdev->lpwm_cls = class_create(THIS_MODULE, LPWM_CLASS_NAME);
	if (IS_ERR(lpwm_cdev->lpwm_cls))
		return PTR_ERR(lpwm_cdev->lpwm_cls);

	devf = device_create(lpwm_cdev->lpwm_cls, NULL,
			lpwm_cdev->dev_num, NULL, lpwm_cdev->name);
	if (IS_ERR(devf)) {
		pr_err("%s: error create device!\n", __func__);
		ret = -1;
		goto err2;
	}

	mutex_init(&lpwm->lpwm_mutex);

	return 0;
err2:
	cdev_del(&lpwm_cdev->cdev);
err1:
	unregister_chrdev_region(lpwm_cdev->dev_num, 1);
err:
	devm_kfree(lpwm->chip.dev, lpwm_cdev);
	lpwm_cdev = NULL;
	return ret;
}

static int lpwm_chip_cdev_remove(struct hobot_lpwm_chip *lpwm)
{
	cdev_del(&lpwm_cdev->cdev);
	unregister_chrdev_region(lpwm_cdev->dev_num, 1);

	devm_kfree(lpwm->chip.dev, lpwm_cdev);
	lpwm_cdev = NULL;

	return 0;
}

/* IO accessors */
static inline u32 hobot_lpwm_rd(struct hobot_lpwm_chip *hobot_chip, u32 reg)
{
	return ioread32(hobot_chip->base + reg);
}

static inline void hobot_lpwm_wr(struct hobot_lpwm_chip *hobot_chip,
						u32 reg, u32 value)
{
	iowrite32(value, hobot_chip->base + reg);
}

static int hobot_lpwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
		int duty_ns, int period_ns)
{
	u32 cfg, reg;
	int period, high;
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);
	int ret;

	if (period_ns > 0 && (period_ns < 10000 || period_ns > 40960000)) {
		pr_err("lpwm only support period in [10000ns~40960000ns]\n");
		return -ERANGE;
	}

	if (duty_ns > 0 && (duty_ns < 10000 || duty_ns > 160000)) {
		pr_err("lpwm only support duty_cycle in [10000ns~160000ns]\n");
		return -ERANGE;
	}

	if (duty_ns >= period_ns)
		duty_ns = period_ns - 1;

	/* config pwm freq */
	period = div64_u64((uint64_t)period_ns, (uint64_t)10000) - 1;
	high = div64_u64((uint64_t)duty_ns, (uint64_t)10000) - 1;

	ret = clk_prepare_enable(lpwm->clk);
	if (ret) {
		pr_err("failed to enable lpwm_mclk clock\n");
		return ret;
	}

	pr_debug("set period_ns: %d, duty_ns: %d\n", period_ns, duty_ns);

	/**
	 * when userspace call ioctl pwm_config() callled which is
	 * already protected by mutex, so spin_lock is unnecessary.
	*/
	reg = (pwm->hwpwm * 0x4) + LPWM0_CFG;
	cfg = hobot_lpwm_rd(lpwm, reg);
	cfg &= 0xFFF;//keep offset values
	cfg = period << 12 | high << 24 | cfg;

	pr_debug("set pwm:%d period_ns:%d, duty_ns:%d, cfg:0x%08x to reg:0x%08x\n",
		pwm->hwpwm, period_ns, duty_ns, cfg, reg);

	hobot_lpwm_wr(lpwm, reg, cfg);

	return 0;
}

static enum hrtimer_restart swtig_timer_func(struct hrtimer *hrt)
{
	struct hobot_lpwm_chip *lpwm =
		container_of(hrt, struct hobot_lpwm_chip, swtrig_timer);

	hobot_lpwm_wr(lpwm, LPWM_SW_TRIG, 1);

	/* 0 meaningless, performance stability requirements */
	if (!swtrig_period)
		swtrig_period = 10U;

	hrtimer_forward_now(hrt, ms_to_ktime(swtrig_period));

	pr_debug("swtrig_period %d\n", swtrig_period);
	return HRTIMER_RESTART;
}

static int hobot_lpwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);
	u32 val;
	int ret;

	if (!__clk_is_enabled(lpwm->clk)) {
		ret = clk_prepare_enable(lpwm->clk);
		if (ret) {
			pr_err("failed to enable lpwm_mclk clock\n");
			return ret;
		}
	}

	if (lpwm->pinctrl != NULL && lpwm->pins[pwm->hwpwm] != NULL)
		pinctrl_select_state(lpwm->pinctrl, lpwm->pins[pwm->hwpwm]);

	/**
	 * when userspace call ioctl pwm_enable() callled which is
	 * already protected by mutex, so spin_lock is unnecessary.
	*/
	val = hobot_lpwm_rd(lpwm, LPWM_EN);
	val |= (1 << pwm->hwpwm);
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	pr_debug("enable lpwm%d, LPWM_EN: 0x%08x, LPWM_CFG: 0x%08x \n",
		pwm->hwpwm, hobot_lpwm_rd(lpwm, LPWM_EN),
		hobot_lpwm_rd(lpwm, (pwm->hwpwm * 4) + LPWM0_CFG));

	return 0;
}

static void hobot_lpwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 val;
	struct hobot_lpwm_chip *lpwm = to_hobot_lpwm_chip(chip);

	/**
	 * when userspace call ioctl pwm_disable() callled which is
	 * already protected by mutex, so spin_lock is unnecessary.
	*/
	val = hobot_lpwm_rd(lpwm, LPWM_EN);
	val &= (~(1 << pwm->hwpwm));

	if (hrtimer_active(&lpwm->swtrig_timer))
		hrtimer_cancel(&lpwm->swtrig_timer);
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	if (__clk_is_enabled(lpwm->clk))
		clk_disable_unprepare(lpwm->clk);

	return;
}

static const struct pwm_ops hobot_lpwm_ops = {
	.config  = hobot_lpwm_config,
	.enable  = hobot_lpwm_enable,
	.disable = hobot_lpwm_disable,
	.owner   = THIS_MODULE,
};

static ssize_t lpwm_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int i;
	size_t len = 0;

	for (i = 0; i < LPWM_NPWM; i++)
		len += snprintf(buf+len, 32, "%d:%dus\n", i, lpwm->offset[i]);

	return len;
}

static ssize_t lpwm_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int offset_us;
	int val;
	char *p = (char *)buf;
	char *token;
	int i;

	count = count > 64 ? 64 : count;

	for (i = 0; i < LPWM_NPWM; i++) {
		token = strsep(&p, " ");
		sscanf(token, "%d", &offset_us);
		if (offset_us < 10 || offset_us > 40960) {
			pr_info("lpwm offset should be in [10, 40960] microseconds\n");
			return -EINVAL;
		}

		lpwm->offset[i] = offset_us;
	}

	for (i = 0; i < LPWM_NPWM; i++) {
		val = hobot_lpwm_rd(lpwm, (i * 0x4) + LPWM0_CFG);
		val &= 0xFFFFF000;
		val |= lpwm->offset[i] / 10 - 1;
		hobot_lpwm_wr(lpwm, (i * 0x4) + LPWM0_CFG, val);
	}

	return count;
}
static DEVICE_ATTR_RW(lpwm_offset);


static ssize_t lpwm_swtrig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);
	pr_info("trigger lpwms, val:%d\n", val);
	val = (val == 0) ? 1 : val;

	if (swtrig_period) {
		hrtimer_start(&lpwm->swtrig_timer, ms_to_ktime(swtrig_period), HRTIMER_MODE_REL);
	}
	hobot_lpwm_wr(lpwm, LPWM_SW_TRIG, val);
	return count;
}
static DEVICE_ATTR_WO(lpwm_swtrig);

static ssize_t lpwm_ppstrig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct hobot_lpwm_chip *lpwm = dev_get_drvdata(dev);
	int val;

	sscanf(buf, "%d", &val);
	pr_info("pps trigger lpwms, val:%d\n", val);

	if (val == 0) {
		val = hobot_lpwm_rd(lpwm, LPWM_EN);
		val &= ~(LPWM_MODE_PPS_TRIG);
	} else {
		if (lpwm->pinctrl != NULL && lpwm->pins[LPWM_PPS] != NULL)
			pinctrl_select_state(lpwm->pinctrl, lpwm->pins[LPWM_PPS]);
		udelay(100);
		val = hobot_lpwm_rd(lpwm, LPWM_EN);
		val |= LPWM_MODE_PPS_TRIG;
	}
	hobot_lpwm_wr(lpwm, LPWM_EN, val);

	return count;
}
static DEVICE_ATTR_WO(lpwm_ppstrig);


static int hobot_lpwm_probe(struct platform_device *pdev)
{
	int ret;
	struct hobot_lpwm_chip *lpwm;
	struct resource *res;
	int i;
	char buf[16];

	lpwm = devm_kzalloc(&pdev->dev, sizeof(struct hobot_lpwm_chip), GFP_KERNEL);
	if (!lpwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	lpwm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(lpwm->base))
		return PTR_ERR(lpwm->base);

	snprintf(lpwm->name, sizeof(lpwm->name), "%s", LPWM_NAME);

	lpwm->clk = devm_clk_get(&pdev->dev, "lpwm_mclk");
	if (IS_ERR(lpwm->clk)) {
		pr_err("lpwm_mclk clock not found.\n");
		return PTR_ERR(lpwm->clk);
	}

	lpwm->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(lpwm->pinctrl)) {
		pr_err("Failed to get a pinctrl state holder, check dts.\n");
		return -ENODEV;
	}

	hrtimer_init(&lpwm->swtrig_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL | HRTIMER_MODE_PINNED);
	lpwm->swtrig_timer.function = swtig_timer_func;
	for (i = 0; i < LPWM_NPWM; i++) {
		memset(buf, 0, sizeof(buf));
		snprintf(buf, sizeof(buf), "lpwm%d", i);

		lpwm->pins[i] = pinctrl_lookup_state(lpwm->pinctrl, buf);
		if (lpwm->pins[i] == NULL) {
			pr_err("lpwm%d pinctrl is not found, check dts.\n", i);
			return -ENODEV;
		}
	}
	lpwm->pins[LPWM_PPS] = pinctrl_lookup_state(lpwm->pinctrl, "lpwm_pps");
	if (lpwm->pins[LPWM_PPS] == NULL) {
		pr_err("lpwm_pps pinctrl is not found, check dts.\n");
		return -ENODEV;
	}

	ret = clk_prepare_enable(lpwm->clk);
	if (ret) {
		pr_err("failed to enable lpwm_mclk clock\n");
		return ret;
	}

	lpwm->chip.dev  = &pdev->dev;
	lpwm->chip.ops  = &hobot_lpwm_ops;
	lpwm->chip.npwm = LPWM_NPWM;
	lpwm->chip.base = -1;

	ret = pwmchip_add(&lpwm->chip);
	if (ret < 0) {
		pr_err("failed to add LPWM chip, error %d\n", ret);
		return ret;
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_offset.attr)) {
		pr_err("lpwm_offset create failed\n");
		return -ENOMEM;
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_swtrig.attr)) {
		pr_err("lpwm_swtrig create failed\n");
		return -ENOMEM;
	}

	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_ppstrig.attr)) {
		pr_err("lpwm_ppstrig create failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, lpwm);


	/* reset all regs, use sw_trigger by default */
	hobot_lpwm_wr(lpwm, LPWM_RST, 1);

	clk_disable_unprepare(lpwm->clk);

	pr_info("%s registered\n", lpwm->name);

	lpwm_chip_cdev_create(lpwm);

	return 0;
}

static int hobot_lpwm_remove(struct platform_device *pdev)
{
	struct hobot_lpwm_chip *lpwm = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < lpwm->chip.npwm; i++)
		pwm_disable(&lpwm->chip.pwms[i]);

	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_offset.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_swtrig.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_ppstrig.attr);

	lpwm_chip_cdev_remove(lpwm);

	return pwmchip_remove(&lpwm->chip);
}

static const struct of_device_id hobot_lpwm_dt_ids[] = {
	{ .compatible = "hobot,hobot-lpwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hobot_lpwm_dt_ids);

static struct platform_driver hobot_lpwm_driver = {
	.driver = {
		.name = "hobot-lpwm",
		.of_match_table = hobot_lpwm_dt_ids,
	},
	.probe = hobot_lpwm_probe,
	.remove = hobot_lpwm_remove,
};
module_platform_driver(hobot_lpwm_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("HOBOT LPWM driver");
MODULE_LICENSE("GPL v2");
