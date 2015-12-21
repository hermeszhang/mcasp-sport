/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * AM335x MCASP to ADSP-21262 SPORT driver.
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
#include "mcasp-setup.h"

#include <linux/platform_device.h>
#include <linux/platform_data/davinci_asp.h>

#include "logging.h"

static void mcasp_set_hw_params(struct mcasp_dev *mcasp)
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

#define X_NUMEVT 1

#if 1
	mcasp_mod_bits(mcasp, reg, 1, NUMDMA_MASK);
	mcasp_mod_bits(mcasp, reg, NUMEVT(X_NUMEVT), NUMEVT_MASK);
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

#if 0
	{
		int active_serializers = rx_ser;
		int numevt = 1;
		u32 reg = mcasp->fifo_base + MCASP_RFIFOCTL_OFFSET;

		mcasp_mod_bits(mcasp, reg,
			       active_serializers,
			       NUMDMA_MASK);
		mcasp_mod_bits(mcasp, reg,
			       NUMEVT(numevt),
			       NUMEVT_MASK);
	}
#endif
}

static void mcasp_set_dai_fmt(struct mcasp_dev *mcasp)
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

#if 0
	//???
	mcasp_set_bits(mcasp, DAVINCI_MCASP_AHCLKXCTL_REG, AHCLKXE);
	mcasp_set_bits(mcasp, DAVINCI_MCASP_AHCLKRCTL_REG, AHCLKRE);
	mcasp_set_bits(mcasp, DAVINCI_MCASP_PDIR_REG, AHCLKX);
#endif

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


static void mcasp_i2s_hw_param(struct mcasp_dev *mcasp)
{
	// 32 bits
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMCTL_REG, 0x00000100);
	// channel 1
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXTDM_REG, 3);

#if 0
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXTDM_REG, mask);
	mcasp_set_bits(mcasp, DAVINCI_MCASP_RXFMT_REG, busel | RXORD);
	mcasp_mod_bits(mcasp, DAVINCI_MCASP_RXFMCTL_REG,
		       FSRMOD(total_slots), FSRMOD(0x1FF));
#endif
}

static void set_channel_size(struct mcasp_dev *mcasp)
{
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXMASK_REG, 0xffffffff);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMT_REG, 0x000080f0);

#if 0
	mcasp_mod_bits(mcasp, DAVINCI_MCASP_RXFMT_REG, RXSSZ(fmt),
		       RXSSZ(0x0F));
	mcasp_mod_bits(mcasp, DAVINCI_MCASP_RXFMT_REG, RXROT(rx_rotate),
		       RXROT(7));
	mcasp_set_reg(mcasp, DAVINCI_MCASP_RXMASK_REG, mask);
#endif
}


void gtsport_mcasp_setup(struct mcasp_dev *mcasp)
{
	mcasp_set_hw_params(mcasp);
	mcasp_i2s_hw_param(mcasp);
	set_channel_size(mcasp);
	mcasp_set_dai_fmt(mcasp);

#if 1
	mcasp_set_reg(mcasp, DAVINCI_MCASP_ACLKRCTL_REG,0);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_ACLKXCTL_REG, 0);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_AHCLKRCTL_REG, 0x00008001);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_AHCLKXCTL_REG, 0x00008001);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLR_REG, 0x0000021f);
	mcasp_set_reg(mcasp, DAVINCI_MCASP_GBLCTLX_REG, 0x0000021f);

	//mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMCTL_REG, 0x00000100);
	//mcasp_set_reg(mcasp, DAVINCI_MCASP_RXFMT_REG, 0x000080f0);

	mcasp_set_reg(mcasp, DAVINCI_MCASP_XRSRCTL_REG(0), 0x22);
#endif
}

void gtsport_mcasp_rx_start(struct mcasp_dev *mcasp)
{
	DEBUG("starting rx");
#if 0
	if (1) {	/* enable FIFO */
		u32 reg = mcasp->fifo_base + MCASP_RFIFOCTL_OFFSET;

		//mcasp_set_reg(mcasp, reg, 0);
		//mcasp_clr_bits(mcasp, reg, FIFO_ENABLE);
		//mcasp_set_bits(mcasp, reg, FIFO_ENABLE);
	}
#endif
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

	/* enable receive IRQs */
	//mcasp_set_bits(mcasp, DAVINCI_MCASP_EVTCTLR_REG, RECIEVE_IRQ_MASK);
	mcasp_set_bits(mcasp, DAVINCI_MCASP_EVTCTLR_REG, 0);
}

void gtsport_mcasp_rx_stop(struct mcasp_dev *mcasp)
{
	DEBUG("stopping rx");

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

#if 0
	if (1) {	/* disable FIFO */
		u32 reg = mcasp->fifo_base + MCASP_RFIFOCTL_OFFSET;
		mcasp_clr_bits(mcasp, reg, FIFO_ENABLE);
	}
#endif
}
