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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/platform_data/edma.h>
#include <linux/of_platform.h>
#include <linux/platform_data/davinci_asp.h>
#include <linux/pinctrl/consumer.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include "davinci-mcasp.h"


#define BUF_SIZE (1024 * 64)
#define CHUNK (BUF_SIZE / 2)


struct gtsport_dev {
	void __iomem *base;
	u32 fifo_base;

	void *dma_buf;
	dma_addr_t dmaphysbuf;
	int asp_link[2];

	u32 *__head;
	int __chunk;

	u32 __counter;
	unsigned int __total;
	unsigned int __bad;

	int rx_dma_channel;
	u32 rx_dma_offset;
	u32 asp_chan_q;
} ;

static inline void mcasp_set_bits(struct gtsport_dev *mcasp, u32 offset,
				  u32 val)
{
	void __iomem *reg = mcasp->base + offset;
	__raw_writel(__raw_readl(reg) | val, reg);
}

static inline void mcasp_clr_bits(struct gtsport_dev *mcasp, u32 offset,
				  u32 val)
{
	void __iomem *reg = mcasp->base + offset;
	__raw_writel((__raw_readl(reg) & ~(val)), reg);
}

static inline void mcasp_mod_bits(struct gtsport_dev *mcasp, u32 offset,
				  u32 val, u32 mask)
{
	void __iomem *reg = mcasp->base + offset;
	__raw_writel((__raw_readl(reg) & ~mask) | val, reg);
}

static inline void mcasp_set_reg(struct gtsport_dev *mcasp, u32 offset,
				 u32 val)
{
	__raw_writel(val, mcasp->base + offset);
}

static inline u32 mcasp_get_reg(struct gtsport_dev *mcasp, u32 offset)
{
	return (u32)__raw_readl(mcasp->base + offset);
}

static void mcasp_set_ctl_reg(struct gtsport_dev *mcasp, u32 ctl_reg, u32 val)
{
	int i = 0;

	mcasp_set_bits(mcasp, ctl_reg, val);

	/* programming GBLCTL needs to read back from GBLCTL and verfiy */
	/* loop count is to avoid the lock-up */
	for (i = 0; i < 1000; i++) {
		if ((mcasp_get_reg(mcasp, ctl_reg) & val) == val)
			break;
	}

	if (i == 1000 && ((mcasp_get_reg(mcasp, ctl_reg) & val) != val))
		printk(KERN_ERR "GBLCTL write error\n");
}

static bool mcasp_is_synchronous(struct gtsport_dev *mcasp)
{
	u32 rxfmctl = mcasp_get_reg(mcasp, DAVINCI_MCASP_RXFMCTL_REG);
	u32 aclkxctl = mcasp_get_reg(mcasp, DAVINCI_MCASP_ACLKXCTL_REG);

	return !(aclkxctl & TX_ASYNC) && rxfmctl & AFSRE;
}

static void mcasp_start_rx(struct gtsport_dev *mcasp)
{
	/* Start clocks */
	mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, RXHCLKRST);
	mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, RXCLKRST);
	/*
	 * When ASYNC == 0 the transmit and receive sections operate
	 * synchronously from the transmit clock and frame sync. We need to make
	 * sure that the TX signlas are enabled when starting reception.
	 */
	if (mcasp_is_synchronous(mcasp)) {
		mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, TXHCLKRST);
		mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, TXCLKRST);
	}

	/* Activate serializer(s) */
	mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, RXSERCLR);
	/* Release RX state machine */
	mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, RXSMRST);
	/* Release Frame Sync generator */
	mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, RXFSRST);
	if (mcasp_is_synchronous(mcasp))
		mcasp_set_ctl_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, TXFSRST);

	/* Disable IRQs */
	mcasp_set_bits(mcasp, DAVINCI_MCASP_EVTCTLR_REG, 0);
}

static void mcasp_stop_rx(struct gtsport_dev *mcasp)
{
	/* disable IRQ sources */
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_EVTCTLR_REG, 0);

	/*
	 * In synchronous mode stop the TX clocks if no other stream is
	 * running
	 */
	if (mcasp_is_synchronous(mcasp))
		mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, 0);

	mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, 0);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXSTAT_REG, 0xFFFFFFFF);
}

void mcasp_set_hw_params(struct gtsport_dev *mcasp)
{
	int i;
	u8 tx_ser = 0;
	u8 rx_ser = 0;
	int max_active_serializers = 1;

	const int serial_dir[16] = {
		RX_MODE, 0, TX_MODE, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0};
	const int num_serializer = 16;


	u32 reg = mcasp->fifo_base + MCASP_RFIFOCTL_OFFSET;


	mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, 0);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, 0);

	mcasp_clr_bits(mcasp, reg, FIFO_ENABLE);

#if 1
	mcasp_mod_bits(mcasp, reg, 1, NUMDMA_MASK);
	mcasp_mod_bits(mcasp, reg, NUMEVT(1), NUMEVT_MASK);
	mcasp_set_bits(mcasp, reg, FIFO_ENABLE);
#endif

	/* Default configuration */
	//if (mcasp->version < MCASP_VERSION_3)
	mcasp_set_bits(mcasp, DAVINCI_MCASP_PWREMUMGT_REG, MCASP_SOFT);

	/* All PINS as McASP */
	mcasp_set_reg(mcasp, DAVINCI_MCASP_PFUNC_REG, 0x00000000);

	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXSTAT_REG, 0xFFFFFFFF);
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_REVTCTL_REG, RXDATADMADIS);


	for (i = 0; i < num_serializer; i++) {
		mcasp_set_bits(mcasp, DAVINCI_MCASP_XRSRCTL_REG(i),
			       serial_dir[i]);
		if (serial_dir[i] == TX_MODE &&
		    tx_ser < max_active_serializers) {
			mcasp_set_bits(mcasp, DAVINCI_MCASP_PDIR_REG, AXR(i));
			mcasp_mod_bits(mcasp, DAVINCI_MCASP_XRSRCTL_REG(i),
				       DISMOD_LOW, DISMOD_MASK);
			tx_ser++;
		} else if (serial_dir[i] == RX_MODE &&
			   rx_ser < max_active_serializers) {
			mcasp_clr_bits(mcasp, DAVINCI_MCASP_PDIR_REG, AXR(i));
			rx_ser++;
		} else {
			mcasp_mod_bits(mcasp, DAVINCI_MCASP_XRSRCTL_REG(i),
				       SRMOD_INACTIVE, SRMOD_MASK);
		}
	}
}


static
void mcasp_set_dai_fmt(struct gtsport_dev *mcasp)
{
	u32 data_delay;
	bool fs_pol_rising;
	bool inv_fs = false;

	mcasp_clr_bits(mcasp, DAVINCI_MCASP_TXFMCTL_REG, FSXDUR);
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_RXFMCTL_REG, FSRDUR);
	/* 1st data bit occur one ACLK cycle after the frame sync */
	data_delay = 0;

	mcasp_mod_bits(mcasp, DAVINCI_MCASP_TXFMT_REG, FSXDLY(data_delay),
		       FSXDLY(3));
	mcasp_mod_bits(mcasp, DAVINCI_MCASP_RXFMT_REG, FSRDLY(data_delay),
		       FSRDLY(3));

	/* codec is clock and frame master */
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_ACLKXCTL_REG, ACLKXE);
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_TXFMCTL_REG, AFSXE);

	mcasp_clr_bits(mcasp, DAVINCI_MCASP_ACLKRCTL_REG, ACLKRE);
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_RXFMCTL_REG, AFSRE);

	mcasp_clr_bits(mcasp, DAVINCI_MCASP_PDIR_REG,
		       ACLKX | AHCLKX | AFSX | ACLKR | AHCLKR | AFSR);


	mcasp_set_bits(mcasp, DAVINCI_MCASP_PDIR_REG, AHCLKX);


	mcasp_set_bits(mcasp, DAVINCI_MCASP_ACLKXCTL_REG, ACLKXPOL);
	mcasp_clr_bits(mcasp, DAVINCI_MCASP_ACLKRCTL_REG, ACLKRPOL);//???
	fs_pol_rising = false;

	if (inv_fs)
		fs_pol_rising = !fs_pol_rising;

	if (fs_pol_rising) {
		mcasp_clr_bits(mcasp, DAVINCI_MCASP_TXFMCTL_REG, FSXPOL);
		mcasp_clr_bits(mcasp, DAVINCI_MCASP_RXFMCTL_REG, FSRPOL);
	} else {
		mcasp_set_bits(mcasp, DAVINCI_MCASP_TXFMCTL_REG, FSXPOL);
		mcasp_set_bits(mcasp, DAVINCI_MCASP_RXFMCTL_REG, FSRPOL);
	}
}


static
void mcasp_i2s_hw_param(struct gtsport_dev *mcasp)
{
	// 32 bits
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMCTL_REG, 0x00000100);
	// channel 1
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXTDM_REG, 3);
}

static
void set_channel_size(struct gtsport_dev *mcasp)
{
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXMASK_REG, 0xffffffff);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMT_REG, 0x000080f0);
}


static
int gtsport_mcasp_port_setup(struct gtsport_dev *dev)
{
	/* TODO: clean me up */
	mcasp_set_hw_params(dev);
	mcasp_i2s_hw_param(dev);
	set_channel_size(dev);
	mcasp_set_dai_fmt(dev);

	mcasp_set_reg(dev, DAVINCI_MCASP_ACLKRCTL_REG,0);
	mcasp_set_reg(dev, DAVINCI_MCASP_ACLKXCTL_REG, 0);
	mcasp_set_reg(dev, DAVINCI_MCASP_AHCLKRCTL_REG, 0x00008001);
	mcasp_set_reg(dev, DAVINCI_MCASP_AHCLKXCTL_REG, 0x00008001);
	mcasp_set_reg(dev, DAVINCI_MCASP_GBLCTLR_REG, 0x0000021f);
	mcasp_set_reg(dev, DAVINCI_MCASP_GBLCTLX_REG, 0x0000021f);

	//mcasp_set_reg(dev, DAVINCI_MCASP_RXFMCTL_REG, 0x00000100);
	//mcasp_set_reg(dev, DAVINCI_MCASP_RXFMT_REG, 0x000080f0);

	mcasp_set_reg(dev, DAVINCI_MCASP_XRSRCTL_REG(0), 0x22);
	return 0;
}


static void mcasp_dma_enqueue(int asp_link, dma_addr_t fifo_dma_addr,
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



static inline
void print_buf_info(int slot, char *name)
{
	struct edmacc_param p;
	if (slot < 0)
		return;
	edma_read_slot(slot, &p);
	printk("%s: 0x%x, opt=%x, src=%x, a_b_cnt=%x dst=%x\n",
			name, slot, p.opt, p.src, p.a_b_cnt, p.dst);
	printk("    src_dst_bidx=%x link_bcntrld=%x src_dst_cidx=%x ccnt=%x\n",
			p.src_dst_bidx, p.link_bcntrld, p.src_dst_cidx, p.ccnt);
}


static
void mcasp_dma_callback(unsigned link, u16 ch_status, void *data)
{
	struct gtsport_dev *dev = (struct gtsport_dev *) data;
	u32 *dma_buf = (u32 *) dev->dma_buf;
	int i, bad_counter = 0;

	if (unlikely(ch_status != DMA_COMPLETE)) {
		printk("%s: ch_status = %04x\n", __func__, ch_status);
		return;
	}

	dma_buf = dev->dma_buf + dev->__chunk * CHUNK;

	for (i = 0; i < (CHUNK / 4); i++) {
		if (dma_buf[i] != dev->__counter) {
			if (!bad_counter)
				printk("bad...%08x\n", dma_buf[i]);
			dev->__bad++;
			bad_counter++;
		}
		dev->__counter = dma_buf[i] + 1;
		dev->__total++;
	}

	dev->__chunk = (dev->__chunk + 1) % 2;
}


static
int gtsport_mcasp_dma_setup(struct gtsport_dev *dev)
{
	struct edmacc_param param_set;
	int i, ret;

	dev->dma_buf = (unsigned char *)
		dma_alloc_coherent(NULL, BUF_SIZE, &dev->dmaphysbuf, 0);

	if (dev->dma_buf == NULL)
		return -ENOMEM;

	printk("got buffer: %08lx\n", (unsigned long) dev->dma_buf);

	ret = edma_alloc_channel(dev->rx_dma_channel,
					 mcasp_dma_callback, dev,
					 //dev->asp_chan_q
					 EVENTQ_0);
	if (ret < 0) {
		printk("edma_alloc_channel() failed\n");
		return ret;
	}

	for (i = 0; i < 2; i++) {
		ret = dev->asp_link[i] = edma_alloc_slot(
			EDMA_CTLR(dev->rx_dma_channel), EDMA_SLOT_ANY);
		if (ret < 0)
			return ret;

		mcasp_dma_enqueue(dev->asp_link[i],
				  dev->rx_dma_offset,
				  dev->dmaphysbuf + CHUNK * i,
				  CHUNK);

		edma_read_slot(dev->asp_link[i], &param_set);
		param_set.opt |= TCINTEN;
		param_set.opt |= EDMA_TCC(EDMA_CHAN_SLOT(dev->rx_dma_channel));
		edma_write_slot(dev->asp_link[i], &param_set);
	}

	/* cycle */
	edma_link(dev->asp_link[0], dev->asp_link[1]);
	edma_link(dev->asp_link[1], dev->asp_link[0]);

	/* initialize slot */
	edma_read_slot(dev->asp_link[0], &param_set);
	edma_write_slot(dev->rx_dma_channel, &param_set);

	edma_start(dev->rx_dma_channel);
	return 0;
}

static
void start_test_tx(struct platform_device *pdev)
{
	struct gtsport_dev *dev = platform_get_drvdata(pdev);

	gtsport_mcasp_dma_setup(dev);
	gtsport_mcasp_port_setup(dev);

	mcasp_start_rx(dev);
}


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
		return	-ENOMEM;

	platform_set_drvdata(pdev, dev);

	ret = of_property_read_u32(np, "asp-chan-q", &dev->asp_chan_q);
	if (ret < 0) {
		dev->asp_chan_q = EVENTQ_0;
	}

	ret = of_property_read_u32(np, "rx-dma-offset",	&dev->rx_dma_offset);
	if (ret < 0) {
		printk("%s:%d: %s(): HERE?\n", __FILE__, __LINE__, __func__);
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
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			 "pins are not configured from the driver\n");

	pm_runtime_enable(&pdev->dev);

	ret = pm_runtime_get_sync(&pdev->dev);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "pm_runtime_get_sync() failed\n");
		return ret;
	}

	dev->fifo_base = DAVINCI_MCASP_V3_AFIFO_BASE;
	dev->base = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!dev->base) {
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

	start_test_tx(pdev);
	return 0;

 err_release_clk:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int gtsport_remove(struct platform_device *pdev)
{
	struct gtsport_dev *dev = platform_get_drvdata(pdev);
	int i;

	printk("%s\n", __func__);

	mcasp_stop_rx(dev);

	edma_stop(dev->rx_dma_channel);
	edma_free_channel(dev->rx_dma_channel);

	for (i = 0; i < 2; i++) {
		if (dev->asp_link[i] >= 0)
			edma_free_slot(dev->asp_link[i]);
	}

	if (dev->dma_buf) {
		dma_free_coherent(NULL, BUF_SIZE,
				  dev->dma_buf, dev->dmaphysbuf);
	}

	printk("%u errors of total %u words\n", dev->__bad, dev->__total);

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


static int __init gtsport_start(void)
{
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

static void __exit gtsport_end(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&gtsport_driver);
		return;
	}
#endif
}

module_init(gtsport_start);
module_exit(gtsport_end);

MODULE_AUTHOR("Victor Makarov");
MODULE_DESCRIPTION("MCASP to SPORT driver");
MODULE_LICENSE("GPL");
