#include <linux/platform_device.h>
#include <sound/pcm_params.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/tsn.h>
#include "tsn_iec61883.h"

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;



struct avb_chip {
	struct snd_card *card;
	struct tsn_link *link;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *substream;
	struct platform_device *device;
	void *kbuffer;
	unsigned int buffer_pos; /* in frames */
	bool period_elapsed;
	struct hrtimer timer;
	atomic_t timer_running;
	unsigned int timer_period_ns;
	struct tasklet_struct tasklet;
	ktime_t base_time;
};



static const struct snd_pcm_hardware avb_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000,
	.rate_min = 32000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.period_bytes_min = 44,
	.period_bytes_max = 32768,
	.buffer_bytes_max = 32768,
	.periods_min = 2,
	.periods_max = 1024,
};



static int snd_avb_playback_open(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (!avb_chip || !avb_chip->link)
		return -EINVAL;
	if (!avb_chip->link->estype_talker)
		return -EINVAL;

	runtime->hw = avb_hw;
	tsn_lb_enable(avb_chip->link);
	pr_info("%s: buffersize=%zd\n", __func__, avb_chip->link->buffer_size);
	return 0;
}


static int snd_avb_playback_close(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);

	tsn_lb_disable(avb_chip->link);
	pr_info("%s\n", __func__);
	return 0;
}



static int snd_avb_playback_copy(struct snd_pcm_substream *substream,int channel, unsigned long hwoff,void __user *src, unsigned long bytes)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int period_pos = avb_chip->buffer_pos % runtime->period_size;
	unsigned long count = bytes_to_frames(runtime, bytes);
	int ret;

	if (copy_from_user(avb_chip->kbuffer, src, bytes))
		return -EFAULT;
	//if (!snd_pcm_running(substream))
	//	return 0;

	ret = tsn_buffer_write(avb_chip->link, avb_chip->kbuffer, bytes);
	if (ret != bytes) {
		pr_err("%s: tsn_buffer_write failed (%ld, got %d)\n", __func__, bytes, ret);
		return -EIO;
	}

	avb_chip->buffer_pos = (avb_chip->buffer_pos + count) % runtime->buffer_size;
	period_pos += count;
	if (period_pos >= runtime->period_size)
		avb_chip->period_elapsed = true;
		//snd_pcm_period_elapsed(substream);

	//pr_info("%s: period_pos=%d, count=%d, buffer_pos=%d, period_size=%d, avail=%d\n",
	//		__func__, period_pos, count, avb_chip->buffer_pos,
	//		runtime->period_size, snd_pcm_playback_avail(runtime));
	return 0;
}

static int snd_avb_capture_open(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;

	if (!avb_chip || !avb_chip->link)
		return -EINVAL;
	if (avb_chip->link->estype_talker)
		return -EINVAL;

	runtime->hw = avb_hw;
	tsn_lb_enable(avb_chip->link);
	pr_info("%s: buffersize=%zd\n", __func__, avb_chip->link->buffer_size);
	return 0;
}


static int snd_avb_capture_close(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);

	tsn_lb_disable(avb_chip->link);
	pr_info("%s\n", __func__);
	return 0;
}




static int snd_avb_capture_copy(struct snd_pcm_substream *substream,
				int channel, unsigned long hwoff,
				void __user *dest, unsigned long bytes)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int period_pos = avb_chip->buffer_pos % runtime->period_size;
	unsigned long count = bytes_to_frames(runtime, bytes);
	int ret;

	//if (!snd_pcm_running(substream))
	//	return 0;
	memset(avb_chip->kbuffer, 0, bytes);

	ret = tsn_buffer_read(avb_chip->link, avb_chip->kbuffer, bytes);
	if (ret != bytes) {
		pr_err("%s: tsn_buffer_read failed (%d, got %d)\n", __func__, bytes, ret);
		tsn_lb_disable(avb_chip->link);
		return -EIO;
	}

	if (copy_to_user(dest, avb_chip->kbuffer, bytes))
		return -EFAULT;

	period_pos += count;
	if (period_pos >= runtime->period_size)
		avb_chip->period_elapsed = true;
		//snd_pcm_period_elapsed(substream);

	//pr_info("%s: period_pos=%d, count=%d, buffer_pos=%d, period_size=%d\n",
	//		__func__, period_pos, count, avb_chip->buffer_pos,
	//		runtime->period_size);
	return 0;
}





static int snd_avb_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *hw_params)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	unsigned int pbytes = params_period_size(hw_params) * 2 * 2;
	unsigned int bsize = params_buffer_bytes(hw_params);
	int ret;

	avb_chip->substream = substream;

	ret = tsn_set_buffer_size(avb_chip->link, pbytes);
	if (ret < 0) {
		pr_err("%s: failed to set buffer size (%d)\n", __func__, pbytes);
		return ret;
	}

	avb_chip->kbuffer = kzalloc(bsize, GFP_KERNEL);
	if (!avb_chip->kbuffer)
		return -ENOMEM;
	return 0;
}




static void snd_avb_sync(struct avb_chip *avb_chip)
{
	hrtimer_cancel(&avb_chip->timer);
	tasklet_kill(&avb_chip->tasklet);
}




static int snd_avb_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);

	if (!avb_chip || !avb_chip->link)
		return -EINVAL;

	snd_avb_sync(avb_chip);
	avb_chip->substream = NULL;
	tsn_clear_buffer_size(avb_chip->link);
	kfree(avb_chip->kbuffer);
	pr_info("%s\n", __func__);
	return 0;
}



static int snd_avb_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int period, rate;
	unsigned long nsecs;
	long sec;

	snd_avb_sync(avb_chip);
	period = runtime->period_size;
	rate = runtime->rate;
	sec = period / rate;
	period %= rate;
	nsecs = div_u64((u64)period * 1000000000UL + rate - 1, rate);
	avb_chip->timer_period_ns = nsecs;

	return 0;
}




static int snd_avb_pcm_trigger(struct snd_pcm_substream *substream,
			       int cmd)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		//tsn_lb_enable(avb_chip->link);
		hrtimer_start(&avb_chip->timer, ns_to_ktime(avb_chip->timer_period_ns),
			HRTIMER_MODE_REL);
		atomic_set(&avb_chip->timer_running, 1);
		pr_info("%s: starting\n", __func__);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		//tsn_lb_disable(avb_chip->link);
		atomic_set(&avb_chip->timer_running, 0);
		hrtimer_cancel(&avb_chip->timer);
		pr_info("%s: stopping\n", __func__);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}





static snd_pcm_uframes_t snd_avb_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct avb_chip *avb_chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	u64 delta;
	u32 pos;

	delta = ktime_us_delta(hrtimer_cb_get_time(&avb_chip->timer),
			avb_chip->base_time);
	delta = div_u64(delta * runtime->rate + 999999, 1000000);
	div_u64_rem(delta, runtime->buffer_size, &pos);
	return pos;

	//pointer = avb_chip->buffer_pos;
	//if (pointer >= runtime->buffer_size)
	//	pointer = 0;

	//pr_info("%s: pointer=%d\n", __func__, pointer);
	//return pointer;
}





static int snd_avb_silence(struct snd_pcm_substream *substream, int channel,
		snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	/* TODO */
	return 0;
}



static struct snd_pcm_ops snd_avb_playback_ops = {
	.open		= snd_avb_playback_open,
	.close		= snd_avb_playback_close,
	.copy_user	= snd_avb_playback_copy,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_avb_pcm_hw_params,
	.hw_free	= snd_avb_pcm_hw_free,
	.prepare	= snd_avb_pcm_prepare,
	.trigger	= snd_avb_pcm_trigger,
	.pointer	= snd_avb_pcm_pointer,
	.fill_silence	= snd_avb_silence,
};



static struct snd_pcm_ops snd_avb_capture_ops = {
	.open		= snd_avb_capture_open,
	.close		= snd_avb_capture_close,
	.copy_user	= snd_avb_capture_copy,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= snd_avb_pcm_hw_params,
	.hw_free	= snd_avb_pcm_hw_free,
	.prepare	= snd_avb_pcm_prepare,
	.trigger	= snd_avb_pcm_trigger,
	.pointer	= snd_avb_pcm_pointer,
	.fill_silence	= snd_avb_silence,
};



static size_t snd_avb_refill(struct tsn_link *link)
{
	struct avb_chip *avb_chip = link->media_chip;

	if (avb_chip && avb_chip->substream)
		return 0;
	return -EINVAL;
}

static size_t snd_avb_drain(struct tsn_link *link)
{
	/*struct avb_chip *avb_chip = link->media_chip;
	struct snd_pcm_substream *substream = avb_chip->substream;

	if (substream && avb_chip->period_elapsed) {
		pr_info("%s: period elapsed!\n", __func__);
		snd_pcm_period_elapsed(substream);
		avb_chip->period_elapsed = false;
	}*/

	return 0;
}




static size_t snd_avb_hdr_size(struct tsn_link *link)
{
	/* return the size of the iec61883-6 audio header */
	return _iec61883_hdr_len();
}

static size_t snd_avb_copy_size(struct tsn_link *link)
{
	struct avb_chip *avb_chip = link->media_chip;
	struct snd_pcm_substream *substream = avb_chip->substream;

	if (substream && substream->runtime)
		return snd_pcm_lib_period_bytes(substream);
	return link->max_payload_size;
}



static void snd_avb_assemble_iidc(struct tsn_link *link,
				  struct avtpdu_header *header, size_t bytes)
{
	_iec61883_hdr_assemble(header, bytes);
}

static int snd_avb_validate_iidc(struct tsn_link *link,
				 struct avtpdu_header *header)
{
	return _iec61883_hdr_verify(header);
}



static void *snd_avb_get_payload_data(struct tsn_link *link,
				      struct avtpdu_header *header)
{
	return _iec61883_payload(header);
}

static void snd_avb_copy_done(struct tsn_link *link)
{
	/*struct avb_chip *avb_chip = link->media_chip;
	struct snd_pcm_substream *substream = avb_chip->substream;

	if (substream && avb_chip->period_elapsed) {
		pr_info("%s: period elapsed!\n", __func__);
		snd_pcm_period_elapsed(substream);
		avb_chip->period_elapsed = false;
	}*/
}



static int snd_avb_new_pcm(struct avb_chip *avb_chip, int device)
{
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(avb_chip->card, "AVB PCM", device, 1, 1, &pcm);
	if (err < 0)
		return err;
	pcm->private_data = avb_chip;
	strcpy(pcm->name, "AVB PCM");
	avb_chip->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_avb_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,  &snd_avb_capture_ops);
	return 0;
}


static void avb_pcm_elapsed(unsigned long priv)
{
	struct avb_chip *avb_chip = (struct avb_chip *)priv;
	struct snd_pcm_substream *substream = avb_chip->substream;
	struct snd_pcm_runtime *runtime;

	if (substream && atomic_read(&avb_chip->timer_running)) {
		runtime = substream->runtime;

		//pr_info("%s\n", __func__);
		avb_chip->buffer_pos = (avb_chip->buffer_pos + runtime->period_size) %
			runtime->buffer_size;
		snd_pcm_period_elapsed(substream);
	}
}






static enum hrtimer_restart avb_hrtimer_callback(struct hrtimer *hrt)
{
	struct avb_chip *avb_chip = container_of(hrt, struct avb_chip, timer);

	if (!atomic_read(&avb_chip->timer_running))
		return HRTIMER_NORESTART;

	tasklet_schedule(&avb_chip->tasklet);
	hrtimer_forward_now(hrt, ns_to_ktime(avb_chip->timer_period_ns));

	/*if (substream && snd_pcm_running(substream)) {
		runtime = substream->runtime;
		pr_info("%s: period elapsed!\n", __func__);
		avb_chip->buffer_pos = (avb_chip->buffer_pos +
				runtime->period_size) % runtime->buffer_size;
		snd_pcm_period_elapsed(substream);
		avb_chip->period_elapsed = false;
	}*/

	return HRTIMER_RESTART;
}




static int snd_avb_probe(struct platform_device *devptr)
{
	int err;
	struct snd_card *card;
	struct avb_chip *avb_chip;
	int dev = devptr->id;

	pr_info("%s: starting\n", __func__);

	err = snd_card_new(&devptr->dev, index[dev], id[dev], THIS_MODULE,
			   sizeof(struct avb_chip), &card);
	if (err < 0) {
		pr_err("%s: trouble creating new card -> %d\n",
			__func__, err);
		return err;
	}
	avb_chip = card->private_data;
	avb_chip->card = card;
	avb_chip->timer_period_ns = 42666667;

	/* create PCM device*/
	err = snd_avb_new_pcm(avb_chip, 0);
	if (err < 0) {
		pr_err("%s: could not create new PCM device\n", __func__);
		goto err_out;
	}

	/* register card */
	pr_info("%s: ready to register card\n", __func__);
	strcpy(card->driver, "Avb");
	strcpy(card->shortname, "Avb");
	sprintf(card->longname, "Avb %i", devptr->id + 1);
	err = snd_card_register(card);
	if (err < 0) {
		pr_err("%s: Could not register card -> %d\n",
			__func__, err);
		snd_card_free(card);
		return err;
	}

	hrtimer_init(&avb_chip->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	avb_chip->timer.function = avb_hrtimer_callback;
	atomic_set(&avb_chip->timer_running, 0);
	tasklet_init(&avb_chip->tasklet, avb_pcm_elapsed, (unsigned long)avb_chip);

	if (err == 0) {
		platform_set_drvdata(devptr, card);
		pr_info("%s: Successfully initialized %s\n",
			__func__, card->shortname);
		return 0;
	}
err_out:
	snd_card_free(card);
	return err;
}




static int snd_avb_remove(struct platform_device *devptr)
{
	struct snd_card *card     = platform_get_drvdata(devptr);
	struct avb_chip *avb_chip = card->private_data;

	/* Make sure link holds no ref to this now dead card */
	if (avb_chip && avb_chip->link) {
		atomic_set(&avb_chip->timer_running, 0);
		hrtimer_cancel(&avb_chip->timer);
		avb_chip->link->media_chip = NULL;
		avb_chip->link = NULL;
	}

	/* call into link->ops->media_close() ? */
	snd_card_free(card);
	return 0;
}




static struct platform_driver snd_avb_driver = {
	.probe  = snd_avb_probe,
	.remove = snd_avb_remove,
	.driver = {
		.name = "snd_avb",
		.pm   = NULL,	/* don't care about Power Management */
	},
};




static int snd_avb_close(struct tsn_link *link)
{
	struct avb_chip *avb_chip = link->media_chip;

	if (!link->media_chip)
		return 0;

	pr_info("%s: Removing device\n", __func__);

	platform_device_unregister(avb_chip->device);
	/* platform unregister will call into snd_avb_remove */
	platform_driver_unregister(&snd_avb_driver);

	/* update link to remove pointer to now invalid memory */
	link->media_chip = NULL;
	return 0;
}




static int snd_avb_new(struct tsn_link *link)
{
	struct avb_chip *avb_chip;
	struct snd_card *card;
	struct platform_device *device;
	int err;

	err = platform_driver_register(&snd_avb_driver);
	if (err < 0) {
		pr_info("%s: trouble registering driver %d, unreg. partial driver and abort.\n",
			__func__, err);
		return err;
	}

	/*
	 * We only register a single card for now, look to
	 * /sys/devices/platform/snd_avb.0 for content.
	 *
	 * Probe will be triggered if name is same as .name in platform_driver
	 */
	device = platform_device_register_simple("snd_avb", 0, NULL, 0);
	if (IS_ERR(device)) {
		pr_info("%s: ERROR registering simple platform-device\n",
			__func__);
		platform_driver_unregister(&snd_avb_driver);
		return -ENODEV;
	}

	/* store data in driver so we can access it in .probe */
	card = platform_get_drvdata(device);
	if (card == NULL) {
		pr_info("%s: Did not get anything from platform_get_drvdata()\n",
			__func__);
		platform_device_unregister(device);
		return -ENODEV;
	}
	avb_chip = card->private_data;
	avb_chip->device = device;
	avb_chip->link = link;

	link->media_chip = avb_chip;

	return 0;
}


static struct tsn_shim_ops shim_ops = {
	.shim_name	 = "alsa",
	.probe		 = snd_avb_new,
	.buffer_refill   = snd_avb_refill,
	.buffer_drain    = snd_avb_drain,
	.media_close     = snd_avb_close,
	.hdr_size        = snd_avb_hdr_size,
	.copy_size       = snd_avb_copy_size,
	.assemble_header = snd_avb_assemble_iidc,
	.validate_header = snd_avb_validate_iidc,
	.get_payload_data = snd_avb_get_payload_data,
	.copy_done = snd_avb_copy_done,
};



static int __init avb_alsa_init(void)
{
	if (tsn_shim_register_ops(&shim_ops)) {
		pr_err("Could not register ALSA-shim with TSN\n");
		return -EINVAL;
	}
	pr_info("AVB ALSA added OK\n");
	return 0;
}

static void __exit avb_alsa_exit(void)
{
	tsn_shim_deregister_ops(&shim_ops);
}

module_init(avb_alsa_init);
module_exit(avb_alsa_exit);
MODULE_AUTHOR("Henrik Austad");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TSN ALSA shim driver");
