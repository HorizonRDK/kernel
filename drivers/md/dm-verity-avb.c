/*
 * Copyright (C) 2019 Google.
 *
 * This file is released under the GPLv2.
 *
 * Based on drivers/md/dm-verity-chromeos.c
 */
#include <linux/cdev.h>
#include <linux/device-mapper.h>
#include <linux/fs.h>
#include<linux/kobject.h>
#include <linux/module.h>
#include <linux/mount.h>
#include<linux/sysfs.h>

#define DM_MSG_PREFIX "verity-avb"

/* Set via module parameters. */
static char avb_vbmeta_device[64];
static char avb_invalidate_on_error[4];

volatile bool avb_inval_on_err;
#ifdef CONFIG_DM_VERITY_AVB_SYSFS
#define BUFLEN 32
struct kobject *kobj_ref;

static ssize_t sys_inval_on_err_show(struct kobject *kobj,
                struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, BUFLEN, "Invalidate on error: %s\n",
							avb_inval_on_err ? "True" : "False");
	return ret;
}

static ssize_t sys_inval_on_err_store(struct kobject *kobj,
                struct kobj_attribute *attr, const char *buf, size_t count)
{
	int scan_in = 0;
	sscanf(buf, "%d", &scan_in);
	if (scan_in == 0)
		avb_inval_on_err = false;
	else if (scan_in == 1)
		avb_inval_on_err = true;
	else
		pr_err("Supports only 0/1, setting preserved!\n");
	return count;
}

struct kobj_attribute avb_inval_attr = __ATTR(invalidate_on_error, 0660,
				sys_inval_on_err_show, sys_inval_on_err_store);
#endif

static void invalidate_vbmeta_endio(struct bio *bio)
{
	if (bio->bi_status)
		DMERR("invalidate_vbmeta_endio: error %d", bio->bi_status);
	complete(bio->bi_private);
}

static int invalidate_vbmeta_submit(struct bio *bio,
				    struct block_device *bdev,
				    int op, int access_last_sector,
				    struct page *page)
{
	DECLARE_COMPLETION_ONSTACK(wait);

	bio->bi_private = &wait;
	bio->bi_end_io = invalidate_vbmeta_endio;
	bio_set_dev(bio, bdev);
	bio_set_op_attrs(bio, op, REQ_SYNC);

	bio->bi_iter.bi_sector = 0;
	if (access_last_sector) {
		sector_t last_sector;

		last_sector = (i_size_read(bdev->bd_inode) >> SECTOR_SHIFT) - 1;
		bio->bi_iter.bi_sector = last_sector;
	}
	if (!bio_add_page(bio, page, PAGE_SIZE, 0)) {
		DMERR("invalidate_vbmeta_submit: bio_add_page error");
		return -EIO;
	}

	submit_bio(bio);
	/* Wait up to 2 seconds for completion or fail. */
	if (!wait_for_completion_timeout(&wait, msecs_to_jiffies(2000)))
		return -EIO;
	return 0;
}

static int invalidate_vbmeta(dev_t vbmeta_devt)
{
	int ret = 0;
	struct block_device *bdev;
	struct bio *bio;
	struct page *page;
	fmode_t dev_mode;
	/* Ensure we do synchronous unblocked I/O. We may also need
	 * sync_bdev() on completion, but it really shouldn't.
	 */
	int access_last_sector = 0;

	DMINFO("invalidate_vbmeta: acting on device %d:%d",
	       MAJOR(vbmeta_devt), MINOR(vbmeta_devt));

	/* First we open the device for reading. */
	dev_mode = FMODE_READ | FMODE_EXCL;
	bdev = blkdev_get_by_dev(vbmeta_devt, dev_mode,
				 invalidate_vbmeta);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_kernel: could not open device for reading");
		dev_mode = 0;
		ret = -ENOENT;
		goto failed_to_read;
	}

	bio = bio_alloc(GFP_NOIO, 1);
	if (!bio) {
		ret = -ENOMEM;
		goto failed_bio_alloc;
	}

	page = alloc_page(GFP_NOIO);
	if (!page) {
		ret = -ENOMEM;
		goto failed_to_alloc_page;
	}

	access_last_sector = 0;
	ret = invalidate_vbmeta_submit(bio, bdev, REQ_OP_READ,
				       access_last_sector, page);
	if (ret) {
		DMERR("invalidate_vbmeta: error reading");
		goto failed_to_submit_read;
	}

	/* We have a page. Let's make sure it looks right. */
	if (memcmp("AVB0", page_address(page), 4) == 0) {
		/* Stamp it. */
		memcpy(page_address(page), "AVE0", 4);
		DMINFO("invalidate_vbmeta: found vbmeta partition");
	} else {
		/* Could be this is on a AVB footer, check. Also, since the
		 * AVB footer is in the last 64 bytes, adjust for the fact that
		 * we're dealing with 512-byte sectors.
		 */
		size_t offset = (1 << SECTOR_SHIFT) - 64;

		access_last_sector = 1;
		ret = invalidate_vbmeta_submit(bio, bdev, REQ_OP_READ,
					       access_last_sector, page);
		if (ret) {
			DMERR("invalidate_vbmeta: error reading");
			goto failed_to_submit_read;
		}
		if (memcmp("AVBf", page_address(page) + offset, 4) != 0) {
			DMERR("invalidate_vbmeta on non-vbmeta partition");
			ret = -EINVAL;
			goto invalid_header;
		}
		/* Stamp it. */
		memcpy(page_address(page) + offset, "AVE0", 4);
		DMINFO("invalidate_vbmeta: found vbmeta footer partition");
	}

	/* Now rewrite the changed page - the block dev was being
	 * changed on read. Let's reopen here.
	 */
	blkdev_put(bdev, dev_mode);
	dev_mode = FMODE_WRITE | FMODE_EXCL;
	bdev = blkdev_get_by_dev(vbmeta_devt, dev_mode,
				 invalidate_vbmeta);
	if (IS_ERR(bdev)) {
		DMERR("invalidate_vbmeta: could not open device for writing");
		dev_mode = 0;
		ret = -ENOENT;
		goto failed_to_write;
	}

	/* We re-use the same bio to do the write after the read. Need to reset
	 * it to initialize bio->bi_remaining.
	 */
	bio_reset(bio);

	ret = invalidate_vbmeta_submit(bio, bdev, REQ_OP_WRITE,
				       access_last_sector, page);
	if (ret) {
		DMERR("invalidate_vbmeta: error writing");
		goto failed_to_submit_write;
	}

	DMERR("invalidate_vbmeta: completed.");
	ret = 0;
failed_to_submit_write:
failed_to_write:
invalid_header:
	__free_page(page);
failed_to_submit_read:
	/* Technically, we'll leak a page with the pending bio, but
	 * we're about to reboot anyway.
	 */
failed_to_alloc_page:
	bio_put(bio);
failed_bio_alloc:
	if (dev_mode)
		blkdev_put(bdev, dev_mode);
failed_to_read:
	return ret;
}

void dm_verity_avb_error_handler(void)
{
	dev_t dev;

	DMINFO("AVB error handler called for %s", avb_vbmeta_device);

	if (strcmp(avb_invalidate_on_error, "yes") != 0 || !avb_inval_on_err) {
		DMINFO("Not configured to invalidate");
		return;
	}

	if (avb_vbmeta_device[0] == '\0') {
		DMERR("avb_vbmeta_device parameter not set");
		goto fail_no_dev;
	}

	dev = name_to_dev_t(avb_vbmeta_device);
	if (!dev) {
		DMERR("No matching partition for device: %s",
		      avb_vbmeta_device);
		goto fail_no_dev;
	}

	invalidate_vbmeta(dev);

fail_no_dev:
	return;
}

static int __init dm_verity_avb_init(void)
{
	DMINFO("AVB error handler initialized with vbmeta device: %s",
	       avb_vbmeta_device);

	avb_inval_on_err = true;
#ifdef CONFIG_DM_VERITY_AVB_SYSFS
	/*Creating a directory in /sys/kernel/ */
	kobj_ref = kobject_create_and_add("dm-verity-avb", kernel_kobj);

	/*Creating sysfs file for avb_inval*/
	if(sysfs_create_file(kobj_ref, &avb_inval_attr.attr))
			pr_info("Cannot create sysfs file......\n");
#endif
	return 0;
}

static void __exit dm_verity_avb_exit(void)
{
#ifdef CONFIG_DM_VERITY_AVB_SYSFS
	kobject_put(kobj_ref);
	sysfs_remove_file(kernel_kobj, &avb_inval_attr.attr);
#endif
}

module_init(dm_verity_avb_init);
module_exit(dm_verity_avb_exit);

MODULE_AUTHOR("David Zeuthen <zeuthen@google.com>");
MODULE_DESCRIPTION("AVB-specific error handler for dm-verity");
MODULE_LICENSE("GPL");

/* Declare parameter with no module prefix */
#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX	"androidboot.vbmeta."
module_param_string(device, avb_vbmeta_device, sizeof(avb_vbmeta_device), 0);
module_param_string(invalidate_on_error, avb_invalidate_on_error,
		    sizeof(avb_invalidate_on_error), 0);
