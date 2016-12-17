/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * AM335x MCASP to ADSP-21262 SPORT driver.
 *
 * Based on sound/soc/davinci/davinci-mcasp.c
 *
 * Copyright (C) GeoTechnologies 2016.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_data/davinci_asp.h>
#include <linux/platform_data/edma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>

#include "davinci-mcasp.h"
#include "mcasp-setup.h"
#include "logging.h"

#ifndef CONFIG_OF
#error "This module requires CONFIG_OF to be set"
#endif

#define DEVICE_NAME "gtsport"

#define CHUNK_SIZE (128 * 1024) /* 32K words */
#define CHUNKS_COUNT 4
#define BUF_SIZE (CHUNK_SIZE * CHUNKS_COUNT)

static int gtsport_major = 0;
static struct class *gtsport_char_class = NULL;

struct circular_buffer {
	unsigned long tail;
	unsigned long head;
	unsigned long count;
	unsigned long overflow;
};

struct gtsport_dev {
	struct device *chardev;
	struct mutex dev_lock;
	spinlock_t rx_lock;
	wait_queue_head_t poll_wq;
	int users;

	struct mcasp_dev mcasp;

	void *dma_buf;
	dma_addr_t dmaphysbuf;
	int asp_link[CHUNKS_COUNT];

	unsigned int next_rx_chunk;
	unsigned long rx_total;

	struct circular_buffer rx_buf;

	int rx_dma_channel;
	u32 rx_dma_offset;
	u32 asp_chan_q;
} ;

static struct gtsport_dev *_gtsport_instance = NULL;

static void davinci_pcm_enqueue_dma(int asp_link, dma_addr_t fifo_dma_addr,
				    dma_addr_t dst_dma_addr, unsigned int len)
{
	unsigned int word_size = 4;
	unsigned int fifo_level = 1;
	unsigned int words = len / word_size;

	edma_set_src(asp_link, fifo_dma_addr, INCR, W8BIT);
	edma_set_dest(asp_link, dst_dma_addr, INCR, W8BIT);

	edma_set_src_index(asp_link, 0, 0);
	edma_set_dest_index(asp_link, word_size, word_size * fifo_level);

	edma_set_transfer_params(asp_link,
				 word_size, fifo_level, words,
				 fifo_level, ABSYNC);
}

static void gtsport_dma_callback(unsigned link, u16 ch_status, void *data)
{
	struct gtsport_dev *dev = (struct gtsport_dev *) data;
	unsigned long flags;
#ifdef TEST_COUNTER
	u32 *dma_buf = (u32 *) dev->dma_buf;
	int i;
	int bad_counter = 0;
	static u32 __counter;
#endif

	if (unlikely(ch_status != DMA_COMPLETE)) {
		WARNING("ch_status = %04x", ch_status);
		return;
	}

#ifdef TEST_COUNTER
	dma_buf = dev->dma_buf + dev->next_rx_chunk * CHUNK_SIZE;

	for (i = 0; i < (CHUNK_SIZE / 4); i++) {
		if (dma_buf[i] != __counter) {
			if (!bad_counter) {
				printk("bad counter...%08x\n", dma_buf[i]);
			}
			bad_counter++;
		}
		__counter = dma_buf[i] + 1;
		dev->rx_total++;
	}
#endif
	spin_lock_irqsave(&dev->rx_lock, flags);
	/* update buffer head */
	dev->next_rx_chunk = (dev->next_rx_chunk + 1) % CHUNKS_COUNT;
	dev->rx_buf.count += CHUNK_SIZE;
	dev->rx_buf.head = CHUNK_SIZE * dev->next_rx_chunk;

	if (dev->rx_buf.count >= BUF_SIZE) {
                /* chunk is lost */
		dev->rx_buf.count = BUF_SIZE - CHUNK_SIZE;
                dev->rx_buf.tail = (BUF_SIZE + dev->rx_buf.head -
                                    dev->rx_buf.count) % BUF_SIZE;
		dev->rx_buf.overflow += CHUNK_SIZE;
	}
	spin_unlock_irqrestore(&dev->rx_lock, flags);

	wake_up(&dev->poll_wq);
}

static void gtsport_mcasp_rx_dma_cleanup(struct gtsport_dev *dev)
{
	size_t i;

	edma_stop(dev->rx_dma_channel);
	edma_free_channel(dev->rx_dma_channel);

	for (i = 0; i < CHUNKS_COUNT; i++) {
		if (dev->asp_link[i] >= 0)
			edma_free_slot(dev->asp_link[i]);
	}

	if (dev->dma_buf) {
		dma_free_coherent(NULL, BUF_SIZE,
				  dev->dma_buf, dev->dmaphysbuf);
	}
}

static int gtsport_mcasp_rx_dma_setup(struct gtsport_dev *dev)
{
	struct edmacc_param param_set;
	int asp_channel;
	size_t i;

	dev->dma_buf = (unsigned char *)
		dma_alloc_coherent(NULL, BUF_SIZE, &dev->dmaphysbuf, 0);

	if (dev->dma_buf == NULL) {
		ERROR("dma_alloc_coherent(size=%d) failed", BUF_SIZE);
		return -ENOMEM;
	}

	DEBUG("got DMA buffer: %08lx", (unsigned long) dev->dma_buf);

	asp_channel = edma_alloc_channel(dev->rx_dma_channel,
					 gtsport_dma_callback, dev,
					 //dev->asp_chan_q
					 EVENTQ_0);
	if (asp_channel < 0) {
		ERROR("edma_alloc_channel() failed");
		gtsport_mcasp_rx_dma_cleanup(dev);
		return asp_channel;
	}

	DEBUG("asp_channel = %d, rx_dma_channel = %d, chan_q = %d",
	      asp_channel, dev->rx_dma_channel, dev->asp_chan_q);

	for (i = 0; i < CHUNKS_COUNT; i++) {
		dev->asp_link[i] = edma_alloc_slot(
						   EDMA_CTLR(dev->rx_dma_channel), EDMA_SLOT_ANY);

		davinci_pcm_enqueue_dma(dev->asp_link[i],
					dev->rx_dma_offset,
					dev->dmaphysbuf + CHUNK_SIZE * i,
					CHUNK_SIZE);

		edma_read_slot(dev->asp_link[i], &param_set);
		//param_set.opt |= ITCINTEN;
		param_set.opt |= TCINTEN;
		param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dev->rx_dma_channel));
		edma_write_slot(dev->asp_link[i], &param_set);
	}

	for (i = 0; i < CHUNKS_COUNT - 1; i++) {
		edma_link(dev->asp_link[i], dev->asp_link[i + 1]);
	}
	edma_link(dev->asp_link[CHUNKS_COUNT - 1], dev->asp_link[0]);

	/* initialize slot */
	edma_read_slot(dev->asp_link[0], &param_set);
	edma_write_slot(asp_channel, &param_set);

	/* reset DMA buffer */
	dev->next_rx_chunk = 0;
	memset(&dev->rx_buf, 0, sizeof(dev->rx_buf));

	edma_start(dev->rx_dma_channel);
	return 0;
}

static ssize_t show_rx_buf(struct device *dev,
			   struct device_attribute *attr,
			   char *buf)
{
	struct gtsport_dev *sport = dev_get_drvdata(dev->parent);
	return sprintf(buf, "head=%08lx tail=%08lx count=%08lx over=%08lx\n",
		       sport->rx_buf.head, sport->rx_buf.tail,
		       sport->rx_buf.count, sport->rx_buf.overflow);
}

static ssize_t show_rx_dma_buf(struct device *dev,
                               struct device_attribute *attr,
                               char *buf)
{
	struct gtsport_dev *sport = dev_get_drvdata(dev->parent);
	struct edmacc_param p;
	int slot = sport->rx_dma_channel;
	edma_read_slot(slot, &p);
	return sprintf(
		buf, "0x%x, opt=%x, src=%x, a_b_cnt=%x dst=%x\n"
		"src_dst_bidx=%x link_bcntrld=%x src_dst_cidx=%x ccnt=%x\n",
		slot, p.opt, p.src, p.a_b_cnt, p.dst,
		p.src_dst_bidx, p.link_bcntrld, p.src_dst_cidx, p.ccnt);
}

static ssize_t show_rx_overflow(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct gtsport_dev *sport = dev_get_drvdata(dev->parent);
	return sprintf(buf, "%lu\n", sport->rx_buf.overflow);
}

static ssize_t show_rx_total(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct gtsport_dev *sport = dev_get_drvdata(dev->parent);
	return sprintf(buf, "%lu\n", sport->rx_total);
}

static DEVICE_ATTR(rx_buf, S_IRUGO, show_rx_buf, NULL);
static DEVICE_ATTR(rx_dma_buf, S_IRUGO, show_rx_dma_buf, NULL);
static DEVICE_ATTR(rx_overflow, S_IRUGO, show_rx_overflow, NULL);
static DEVICE_ATTR(rx_total, S_IRUGO, show_rx_total, NULL);

static struct attribute *gtsport_sysfs_entries[] = {
	&dev_attr_rx_buf.attr,
	&dev_attr_rx_dma_buf.attr,
	&dev_attr_rx_overflow.attr,
	&dev_attr_rx_total.attr,
	NULL,
} ;

static struct attribute_group gtsport_attribute_group = {
	.name = NULL,
	.attrs = gtsport_sysfs_entries,
} ;

#if defined(CONFIG_OF)
static const struct of_device_id gtsport_ids[] = {
	{
		.compatible = "ti,gtsport",
		.data = (void *) 0,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, gtsport_ids);

static int gtsport_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *mem, *ioarea, *res;
	struct gtsport_dev *dev;
	struct pinctrl *pinctrl;
	int ret;

	if (!pdev->dev.platform_data && !pdev->dev.of_node) {
		dev_err(&pdev->dev, "No platform data supplied\n");
		return -EINVAL;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(struct gtsport_dev),
			   GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->dev_lock);
	spin_lock_init(&dev->rx_lock);
	init_waitqueue_head(&dev->poll_wq);

	platform_set_drvdata(pdev, dev);

	ret = of_property_read_u32(np, "asp-chan-q", &dev->asp_chan_q);
	if (ret < 0) {
		dev->asp_chan_q = EVENTQ_0;
	}

	ret = of_property_read_u32(np, "rx-dma-offset", &dev->rx_dma_offset);
	if (ret < 0) {
		dev_err(&pdev->dev, "invalid rx-dma-offset\n");
		return -EINVAL;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	ioarea = devm_request_mem_region(&pdev->dev, mem->start,
					 resource_size(mem), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "Audio region already claimed\n");
		return -EBUSY;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&pdev->dev,
			 "pins are not configured from the driver\n");
	}

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "pm_runtime_get_sync() failed\n");
		return ret;
	}

	dev->mcasp.fifo_base = DAVINCI_MCASP_V3_AFIFO_BASE;
	dev->mcasp.base = devm_ioremap(&pdev->dev, mem->start,
				       resource_size(mem));
	if (!dev->mcasp.base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_release_clk;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_DMA, "rx");
	if (!res) {
		dev_err(&pdev->dev, "Failed to get rx dma resource\n");
		ret = -ENODEV;
		goto err_release_clk;
	}
	dev->rx_dma_channel = res->start;

	/* only one device is now supported */
	dev->chardev =
		device_create(gtsport_char_class, &pdev->dev,
			      MKDEV(gtsport_major, 0), dev,
			      DEVICE_NAME);
	if (IS_ERR(dev->chardev)) {
		ret = PTR_ERR(dev->chardev);
		goto err_release_clk;
	}

        if (sysfs_create_group(&dev->chardev->kobj, &gtsport_attribute_group)) {
		dev_err(dev->chardev, "Failed to create sysfs group\n");
        }

	_gtsport_instance = dev;
	return 0;

 err_release_clk:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int gtsport_remove(struct platform_device *pdev)
{
	struct gtsport_dev *dev = platform_get_drvdata(pdev);

	DEBUG("removing driver");

	sysfs_remove_group(&dev->chardev->kobj, &gtsport_attribute_group);
	device_destroy(gtsport_char_class, dev->chardev->devt);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static struct platform_driver gtsport_driver = {
	.probe		= gtsport_probe,
	.remove		= gtsport_remove,
	.driver		= {
		.name	= "gtsport",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gtsport_ids),
	},
};
#endif

static int gtsport_char_open(struct inode *inode, struct file *filp)
{
	struct gtsport_dev *dev = _gtsport_instance;
	int ret = 0;

	if (!dev)
		return -ENODEV;

	DEBUG("");

        if (mutex_lock_interruptible(&dev->dev_lock)) {
		return -ERESTARTSYS;
	}

	if (dev->users) {
		ret = -EBUSY;
		goto error;
	}

	ret = gtsport_mcasp_rx_dma_setup(dev);
	if (ret < 0)
		goto error;

	gtsport_mcasp_setup(&dev->mcasp);
	gtsport_mcasp_rx_start(&dev->mcasp);

	dev->users++;
	filp->private_data = dev;

	mutex_unlock(&dev->dev_lock);
	return 0;
 error:
	mutex_unlock(&dev->dev_lock);
	return ret;
}

static int gtsport_char_release(struct inode *inode, struct file *filp)
{
	struct gtsport_dev *dev = filp->private_data;
	DEBUG("");

	mutex_lock(&dev->dev_lock);
	dev->users--;

	if (!dev->users) {
		DEBUG("stopping rx");
		gtsport_mcasp_rx_stop(&dev->mcasp);
		gtsport_mcasp_rx_dma_cleanup(dev);
	}
	mutex_unlock(&dev->dev_lock);

	return 0;
}

static inline bool gtsport_buffer_not_empty(struct gtsport_dev *dev)
{
	return dev->rx_buf.count > 0;
}

static ssize_t gtsport_char_read(struct file *filp, char *buf,
				 size_t count, loff_t *f_pos)
{
	struct gtsport_dev *dev = filp->private_data;
	ssize_t retval;
	struct circular_buffer rx_buf;
	size_t amount;
	unsigned long flags;

	// TODO: check DMA status between chunks
	spin_lock_irqsave(&dev->rx_lock, flags);
	while (!gtsport_buffer_not_empty(dev)) {
		spin_unlock_irqrestore(&dev->rx_lock, flags);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		retval = wait_event_interruptible(
		    dev->poll_wq,
		    gtsport_buffer_not_empty(dev));
		if (retval)
			return retval;
		spin_lock_irqsave(&dev->rx_lock, flags);
        }

	rx_buf = dev->rx_buf;
	spin_unlock_irqrestore(&dev->rx_lock, flags);

	retval = 0;

	while ((amount = min(min(rx_buf.count, BUF_SIZE - rx_buf.tail),
			     (unsigned long) count)) > 0) {
		if (copy_to_user(buf, dev->dma_buf + rx_buf.tail, amount)) {
			return -EFAULT;
		}

		rx_buf.tail = (rx_buf.tail + amount) % BUF_SIZE;
		rx_buf.count -= amount;

		retval += amount;
		buf += amount;
		count -= amount;
	}

	/* move tail */
	spin_lock_irqsave(&dev->rx_lock, flags);
	if (dev->rx_buf.count >= retval)
		dev->rx_buf.count -= retval;
	else
		dev->rx_buf.count = 0;
        dev->rx_buf.tail = (BUF_SIZE + dev->rx_buf.head -
			    dev->rx_buf.count) % BUF_SIZE;
	spin_unlock_irqrestore(&dev->rx_lock, flags);

	return retval;
}

static unsigned int gtsport_char_poll(struct file *filp, poll_table *wait)
{
	struct gtsport_dev *dev = filp->private_data;

	poll_wait(filp, &dev->poll_wq, wait);

	if (gtsport_buffer_not_empty(dev))
		return POLLIN | POLLRDNORM;

	return 0;
}

static struct file_operations gtsport_char_fops = {
	.owner   = THIS_MODULE,
	.open    = gtsport_char_open,
	.release = gtsport_char_release,
	.poll    = gtsport_char_poll,
	.read    = gtsport_char_read,
} ;

static int __init gtsport_init(void)
{
	gtsport_major = register_chrdev(0, DEVICE_NAME, &gtsport_char_fops);
	if (gtsport_major < 0) {
		return gtsport_major;
	}

	DEBUG("gtsport chardev registered");

	gtsport_char_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(gtsport_char_class)) {
		unregister_chrdev(gtsport_major, DEVICE_NAME);
		return PTR_ERR(gtsport_char_class);
	}

	DEBUG("gtsport device class registered");

#if defined(CONFIG_OF)
	/*
	 * If dtb is there, the devices will be created dynamically.
	 * Only register platfrom driver structure.
	 */
	if (of_have_populated_dt())
		return platform_driver_register(&gtsport_driver);
#endif
	return 0;
}

static void __exit gtsport_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&gtsport_driver);
	}
#endif
	class_unregister(gtsport_char_class);
	class_destroy(gtsport_char_class);
	unregister_chrdev(gtsport_major, DEVICE_NAME);
}

module_init(gtsport_init);
module_exit(gtsport_exit);

MODULE_AUTHOR("Victor Makarov");
MODULE_DESCRIPTION("McASP to SPORT driver");
MODULE_LICENSE("GPL");
