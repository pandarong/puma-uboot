// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2016-2017 Rockchip Inc.
 *
 * Adapted from coreboot.
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dt-structs.h>
#include <ram.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/sdram_common.h>
#include <asm/arch/sdram_rk3399.h>
#include <asm/arch/cru_rk3399.h>
#include <asm/arch/grf_rk3399.h>
#include <asm/arch/hardware.h>
#include <linux/err.h>
#include <time.h>
#include "dram_spec_timing.h"

struct chan_info {
	struct rk3399_ddr_pctl_regs *pctl;
	struct rk3399_ddr_pi_regs *pi;
	struct rk3399_ddr_publ_regs *publ;
	struct rk3399_msch_regs *msch;
};

struct dram_info {
#ifdef CONFIG_SPL_BUILD
	struct chan_info chan[2];
	struct clk ddr_clk;
	struct rk3399_cru *cru;
	struct rk3399_pmucru *pmucru;
	struct rk3399_pmusgrf_regs *pmusgrf;
	struct rk3399_ddr_cic_regs *cic;
#endif
	struct ram_info info;
	struct rk3399_pmugrf_regs *pmugrf;
};

#define PRESET_SGRF_HOLD(n)	((0x1 << (6 + 16)) | ((n) << 6))
#define PRESET_GPIO0_HOLD(n)	((0x1 << (7 + 16)) | ((n) << 7))
#define PRESET_GPIO1_HOLD(n)	((0x1 << (8 + 16)) | ((n) << 8))

#define PHY_DRV_ODT_Hi_Z	0x0
#define PHY_DRV_ODT_240		0x1
#define PHY_DRV_ODT_120		0x8
#define PHY_DRV_ODT_80		0x9
#define PHY_DRV_ODT_60		0xc
#define PHY_DRV_ODT_48		0xd
#define PHY_DRV_ODT_40		0xe
#define PHY_DRV_ODT_34_3	0xf

#define ENPER_CS_TRAINING_FREQ	(666)
#define TDFI_LAT_THRESHOLD_FREQ	(933) /* TODO: actually 928 */
#define PHY_DLL_BYPASS_FREQ	(260)

#define PI_REGS_DIMM_SUPPORT	(0)
#define PI_ADD_LATENCY	(0)
#define PI_DOUBLEFREEK	(1)

#define PI_PAD_DELAY_PS_VALUE	(1000)
#define PI_IE_ENABLE_VALUE	(3000)
#define PI_TSEL_ENABLE_VALUE	(700)

#ifdef CONFIG_SPL_BUILD

struct rockchip_dmc_plat {
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct dtd_rockchip_rk3399_dmc dtplat;
#else
	struct rk3399_sdram_params sdram_params;
#endif
	struct regmap *map;
};

static uint32_t wrdqs_delay_val[2][2][4];
static uint32_t rddqs_delay_ps;

static void copy_to_reg(u32 *dest, const u32 *src, u32 n)
{
	int i;

	for (i = 0; i < n / sizeof(u32); i++) {
		writel(*src, dest);
		src++;
		dest++;
	}
}

static void phy_dll_bypass_set(struct rk3399_ddr_publ_regs *ddr_publ_regs,
			       u32 freq, u32 ch, u32 index, u32 dram_type)
{
	u32 *denali_phy = ddr_publ_regs->denali_phy;
	u32 sw_master_mode = 0;
	u32 rddqs_gate_delay, rddqs_latency, total_delay;
	u32 i;

	debug("%s\n", __func__);

	if (dram_type == DDR3)
		total_delay = PI_PAD_DELAY_PS_VALUE;
	else if (dram_type == LPDDR3)
		total_delay = PI_PAD_DELAY_PS_VALUE + 2500;
	else
		total_delay = PI_PAD_DELAY_PS_VALUE + 1500;

	/* total_delay + 0.55tck */
	total_delay +=  (55 * 10000)/freq;
	rddqs_latency = total_delay * freq / 1000000;
	total_delay -= rddqs_latency * 1000000 / freq;
	rddqs_gate_delay = total_delay * 0x200 * freq / 1000000;

	if (freq <= PHY_DLL_BYPASS_FREQ) {
		sw_master_mode = 0xc;

		setbits_le32(&denali_phy[514], 1);
		setbits_le32(&denali_phy[642], 1);
		setbits_le32(&denali_phy[770], 1);

		/* setting bypass mode slave delay */
		for (i = 0; i < 4; i++) {
			/* wr dq delay = -180deg + (0x60 / 4) * 20ps */
			clrsetbits_le32(&denali_phy[1 + 128 * i],
					0x7ff << 8, 0x4a0 << 8);
			/* rd dqs/dq delay = (0x60 / 4) * 20ps */
			clrsetbits_le32(&denali_phy[11 + 128 * i],
					0x3ff, 0xa0);
			/* rd rddqs_gate delay */
			clrsetbits_le32(&denali_phy[2 + 128 * i],
					0x3ff, rddqs_gate_delay);
			clrsetbits_le32(&denali_phy[78 + 128 * i],
					0xf, rddqs_latency);
		}

		/* adr delay */
		for (i = 0; i < 3; i++)
			clrsetbits_le32(&denali_phy[513 + 128 * i],
					0x7ff << 16, 0x80 << 16);

		if ((readl(&denali_phy[86]) & 0xc00) == 0) {
			/*
			 * old status is normal mode,
			 * and saving the wrdqs slave delay
			 */
			for (i = 0; i < 4; i++) {
				u32 reg = readl(&denali_phy[63 + i * 128]);

				/* save and clear wr dqs slave delay */
				wrdqs_delay_val[ch][index][i] =
					        0x3ff &(reg >> 16);
				clrsetbits_le32(&denali_phy[63 + i * 128],
						0x3ff << 16, 0 << 16);
				/*
				 * in normal mode the cmd may delay 1cycle by
				 * wrlvl and in bypass mode making dqs also
				 * delay 1cycle.
				 */
				clrsetbits_le32(&denali_phy[78 + i * 128],
						0x7 << 8, 0x1 << 8);
			}
		}
	} else if (readl(&denali_phy[86]) & 0xc00) {
		/* old status is bypass mode and restore wrlvl resume */
		for (i = 0; i < 4; i++) {
			clrsetbits_le32(&denali_phy[63 + i * 128],
					0x03ff << 16,
					(wrdqs_delay_val[ch][index][i] &
					 0x3ff) << 16);
			/* resume phy_write_path_lat_add */
			clrbits_le32(&denali_phy[78 + i * 128], 0x7 << 8);
		}
	}

	/* phy_sw_master_mode_X PHY_86/214/342/470 4bits offset_8 */
	clrsetbits_le32(&denali_phy[86], 0xf << 8, sw_master_mode << 8);
	clrsetbits_le32(&denali_phy[214], 0xf << 8, sw_master_mode << 8);
	clrsetbits_le32(&denali_phy[342], 0xf << 8, sw_master_mode << 8);
	clrsetbits_le32(&denali_phy[470], 0xf << 8, sw_master_mode << 8);

	/* phy_adrctl_sw_master_mode PHY_547/675/803 4bits offset_16 */
	clrsetbits_le32(&denali_phy[547], 0xf << 16, sw_master_mode << 16);
	clrsetbits_le32(&denali_phy[675], 0xf << 16, sw_master_mode << 16);
	clrsetbits_le32(&denali_phy[803], 0xf << 16, sw_master_mode << 16);
}

static void set_memory_map(const struct chan_info *chan, u32 channel,
			   const struct rk3399_sdram_params *sdram_params)
{
	const struct rk3399_sdram_channel *sdram_ch =
		&sdram_params->ch[channel];
	u32 *denali_ctl = chan->pctl->denali_ctl;
	u32 *denali_pi = chan->pi->denali_pi;
	u32 cs_map;
	u32 reduc;
	u32 row;

	/* Get row number from ddrconfig setting */
	if (sdram_ch->ddrconfig < 2 || sdram_ch->ddrconfig == 4)
		row = 16;
	else if (sdram_ch->ddrconfig == 3)
		row = 14;
	else
		row = 15;

	cs_map = (sdram_ch->rank > 1) ? 3 : 1;
	reduc = (sdram_ch->bw == 2) ? 0 : 1;

	/* Set the dram configuration to ctrl */
	clrsetbits_le32(&denali_ctl[191], 0xF, (12 - sdram_ch->col));
	clrsetbits_le32(&denali_ctl[190], (0x3 << 16) | (0x7 << 24),
			((3 - sdram_ch->bk) << 16) |
			((16 - row) << 24));

	clrsetbits_le32(&denali_ctl[196], 0x3 | (1 << 16),
			cs_map | (reduc << 16));

	/* PI_199 PI_COL_DIFF:RW:0:4 */
	clrsetbits_le32(&denali_pi[199], 0xF, (12 - sdram_ch->col));

	/* PI_155 PI_ROW_DIFF:RW:24:3 PI_BANK_DIFF:RW:16:2 */
	clrsetbits_le32(&denali_pi[155], (0x3 << 16) | (0x7 << 24),
			((3 - sdram_ch->bk) << 16) |
			((16 - row) << 24));
	/* PI_41 PI_CS_MAP:RW:24:4 */
	clrsetbits_le32(&denali_pi[41], 0xf << 24, cs_map << 24);
	if ((sdram_ch->rank == 1) && (sdram_params->base.dramtype == DDR3))
		writel(0x2EC7FFFF, &denali_pi[34]);
}

static void set_ds_odt(const struct chan_info *chan,
		       const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_phy = chan->publ->denali_phy;

	u32 tsel_idle_en, tsel_wr_en, tsel_rd_en;
	u32 tsel_idle_select_p, tsel_wr_select_p, tsel_rd_select_p;
	u32 ca_tsel_wr_select_p, ca_tsel_wr_select_n;
	u32 tsel_idle_select_n, tsel_wr_select_n, tsel_rd_select_n;
	u32 reg_value;

	if (sdram_params->base.dramtype == LPDDR4) {
		tsel_rd_select_p = PHY_DRV_ODT_Hi_Z;
		tsel_wr_select_p = PHY_DRV_ODT_40;
		ca_tsel_wr_select_p = PHY_DRV_ODT_40;
		tsel_idle_select_p = PHY_DRV_ODT_Hi_Z;

		tsel_rd_select_n = PHY_DRV_ODT_240;
		tsel_wr_select_n = PHY_DRV_ODT_40;
		ca_tsel_wr_select_n = PHY_DRV_ODT_40;
		tsel_idle_select_n = PHY_DRV_ODT_240;
	} else if (sdram_params->base.dramtype == LPDDR3) {
		tsel_rd_select_p = PHY_DRV_ODT_240;
		tsel_wr_select_p = PHY_DRV_ODT_34_3;
		ca_tsel_wr_select_p = PHY_DRV_ODT_48;
		tsel_idle_select_p = PHY_DRV_ODT_240;

		tsel_rd_select_n = PHY_DRV_ODT_Hi_Z;
		tsel_wr_select_n = PHY_DRV_ODT_34_3;
		ca_tsel_wr_select_n = PHY_DRV_ODT_48;
		tsel_idle_select_n = PHY_DRV_ODT_Hi_Z;
	} else {
		tsel_rd_select_p = PHY_DRV_ODT_240;
		tsel_wr_select_p = PHY_DRV_ODT_34_3;
		ca_tsel_wr_select_p = PHY_DRV_ODT_34_3;
		tsel_idle_select_p = PHY_DRV_ODT_240;

		tsel_rd_select_n = PHY_DRV_ODT_240;
		tsel_wr_select_n = PHY_DRV_ODT_34_3;
		ca_tsel_wr_select_n = PHY_DRV_ODT_34_3;
		tsel_idle_select_n = PHY_DRV_ODT_240;
	}

	if (sdram_params->base.odt == 1)
		tsel_rd_en = 1;
	else
		tsel_rd_en = 0;

	tsel_wr_en = 0;
	tsel_idle_en = 0;

	/*
	 * phy_dq_tsel_select_X 24bits DENALI_PHY_6/134/262/390 offset_0
	 * sets termination values for read/idle cycles and drive strength
	 * for write cycles for DQ/DM
	 */
	reg_value = tsel_rd_select_n | (tsel_rd_select_p << 0x4) |
		    (tsel_wr_select_n << 8) | (tsel_wr_select_p << 12) |
		    (tsel_idle_select_n << 16) | (tsel_idle_select_p << 20);
	clrsetbits_le32(&denali_phy[6], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[134], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[262], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[390], 0xffffff, reg_value);

	/*
	 * phy_dqs_tsel_select_X 24bits DENALI_PHY_7/135/263/391 offset_0
	 * sets termination values for read/idle cycles and drive strength
	 * for write cycles for DQS
	 */
	clrsetbits_le32(&denali_phy[7], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[135], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[263], 0xffffff, reg_value);
	clrsetbits_le32(&denali_phy[391], 0xffffff, reg_value);

	/* phy_adr_tsel_select_ 8bits DENALI_PHY_544/672/800 offset_0 */
	reg_value = ca_tsel_wr_select_n | (ca_tsel_wr_select_p << 0x4);
	clrsetbits_le32(&denali_phy[544], 0xff, reg_value);
	clrsetbits_le32(&denali_phy[672], 0xff, reg_value);
	clrsetbits_le32(&denali_phy[800], 0xff, reg_value);

	/* phy_pad_addr_drive 8bits DENALI_PHY_928 offset_0 */
	clrsetbits_le32(&denali_phy[928], 0xff, reg_value);

	/* phy_pad_rst_drive 8bits DENALI_PHY_937 offset_0 */
	clrsetbits_le32(&denali_phy[937], 0xff, reg_value);

	/* phy_pad_cke_drive 8bits DENALI_PHY_935 offset_0 */
	clrsetbits_le32(&denali_phy[935], 0xff, reg_value);

	/* phy_pad_cs_drive 8bits DENALI_PHY_939 offset_0 */
	clrsetbits_le32(&denali_phy[939], 0xff, reg_value);

	/* phy_pad_clk_drive 8bits DENALI_PHY_929 offset_0 */
	clrsetbits_le32(&denali_phy[929], 0xff, reg_value);

	/* phy_pad_fdbk_drive 23bit DENALI_PHY_924/925 */
	clrsetbits_le32(&denali_phy[924], 0xff,
			tsel_wr_select_n | (tsel_wr_select_p << 4));
	clrsetbits_le32(&denali_phy[925], 0xff,
			tsel_rd_select_n | (tsel_rd_select_p << 4));

	/* phy_dq_tsel_enable_X 3bits DENALI_PHY_5/133/261/389 offset_16 */
	reg_value = (tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2))
		<< 16;
	clrsetbits_le32(&denali_phy[5], 0x7 << 16, reg_value);
	clrsetbits_le32(&denali_phy[133], 0x7 << 16, reg_value);
	clrsetbits_le32(&denali_phy[261], 0x7 << 16, reg_value);
	clrsetbits_le32(&denali_phy[389], 0x7 << 16, reg_value);

	/* phy_dqs_tsel_enable_X 3bits DENALI_PHY_6/134/262/390 offset_24 */
	reg_value = (tsel_rd_en | (tsel_wr_en << 1) | (tsel_idle_en << 2))
		<< 24;
	clrsetbits_le32(&denali_phy[6], 0x7 << 24, reg_value);
	clrsetbits_le32(&denali_phy[134], 0x7 << 24, reg_value);
	clrsetbits_le32(&denali_phy[262], 0x7 << 24, reg_value);
	clrsetbits_le32(&denali_phy[390], 0x7 << 24, reg_value);

	/* phy_adr_tsel_enable_ 1bit DENALI_PHY_518/646/774 offset_8 */
	reg_value = tsel_wr_en << 8;
	clrsetbits_le32(&denali_phy[518], 0x1 << 8, reg_value);
	clrsetbits_le32(&denali_phy[646], 0x1 << 8, reg_value);
	clrsetbits_le32(&denali_phy[774], 0x1 << 8, reg_value);

	/* phy_pad_addr_term tsel 1bit DENALI_PHY_933 offset_17 */
	reg_value = tsel_wr_en << 17;
	clrsetbits_le32(&denali_phy[933], 0x1 << 17, reg_value);
	/*
	 * pad_rst/cke/cs/clk_term tsel 1bits
	 * DENALI_PHY_938/936/940/934 offset_17
	 */
	clrsetbits_le32(&denali_phy[938], 0x1 << 17, reg_value);
	clrsetbits_le32(&denali_phy[936], 0x1 << 17, reg_value);
	clrsetbits_le32(&denali_phy[940], 0x1 << 17, reg_value);
	clrsetbits_le32(&denali_phy[934], 0x1 << 17, reg_value);

	/* phy_pad_fdbk_term 1bit DENALI_PHY_930 offset_17 */
	clrsetbits_le32(&denali_phy[930], 0x1 << 17, reg_value);
}

static int phy_io_config(const struct chan_info *chan,
			  const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_phy = chan->publ->denali_phy;
	u32 vref_mode_dq, vref_value_dq, vref_mode_ac, vref_value_ac;
	u32 mode_sel;
	u32 reg_value;
	u32 drv_value, odt_value;
	u32 speed;

	/* vref setting */
	if (sdram_params->base.dramtype == LPDDR4) {
		/* LPDDR4 */
		vref_mode_dq = 0x6;
		vref_value_dq = 0x1f;
		vref_mode_ac = 0x6;
		vref_value_ac = 0x1f;
	} else if (sdram_params->base.dramtype == LPDDR3) {
		if (sdram_params->base.odt == 1) {
			vref_mode_dq = 0x5;  /* LPDDR3 ODT */
			drv_value = (readl(&denali_phy[6]) >> 12) & 0xf;
			odt_value = (readl(&denali_phy[6]) >> 4) & 0xf;
			if (drv_value == PHY_DRV_ODT_48) {
				switch (odt_value) {
				case PHY_DRV_ODT_240:
					vref_value_dq = 0x16;
					break;
				case PHY_DRV_ODT_120:
					vref_value_dq = 0x26;
					break;
				case PHY_DRV_ODT_60:
					vref_value_dq = 0x36;
					break;
				default:
					debug("Invalid ODT value.\n");
					return -EINVAL;
				}
			} else if (drv_value == PHY_DRV_ODT_40) {
				switch (odt_value) {
				case PHY_DRV_ODT_240:
					vref_value_dq = 0x19;
					break;
				case PHY_DRV_ODT_120:
					vref_value_dq = 0x23;
					break;
				case PHY_DRV_ODT_60:
					vref_value_dq = 0x31;
					break;
				default:
					debug("Invalid ODT value.\n");
					return -EINVAL;
				}
			} else if (drv_value == PHY_DRV_ODT_34_3) {
				switch (odt_value) {
				case PHY_DRV_ODT_240:
					vref_value_dq = 0x17;
					break;
				case PHY_DRV_ODT_120:
					vref_value_dq = 0x20;
					break;
				case PHY_DRV_ODT_60:
					vref_value_dq = 0x2e;
					break;
				default:
					debug("Invalid ODT value.\n");
					return -EINVAL;
				}
			} else {
				debug("Invalid DRV value.\n");
				return -EINVAL;
			}
		} else {
			vref_mode_dq = 0x2;  /* LPDDR3 */
			vref_value_dq = 0x1f;
		}
		vref_mode_ac = 0x2;
		vref_value_ac = 0x1f;
	} else if (sdram_params->base.dramtype == DDR3) {
		/* DDR3L */
		vref_mode_dq = 0x1;
		vref_value_dq = 0x1f;
		vref_mode_ac = 0x1;
		vref_value_ac = 0x1f;
	} else {
		debug("Unknown DRAM type.\n");
		return -EINVAL;
	}

	reg_value = (vref_mode_dq << 9) | (0x1 << 8) | vref_value_dq;

	/* PHY_913 PHY_PAD_VREF_CTRL_DQ_0 12bits offset_8 */
	clrsetbits_le32(&denali_phy[913], 0xfff << 8, reg_value << 8);
	/* PHY_914 PHY_PAD_VREF_CTRL_DQ_1 12bits offset_0 */
	clrsetbits_le32(&denali_phy[914], 0xfff, reg_value);
	/* PHY_914 PHY_PAD_VREF_CTRL_DQ_2 12bits offset_16 */
	clrsetbits_le32(&denali_phy[914], 0xfff << 16, reg_value << 16);
	/* PHY_915 PHY_PAD_VREF_CTRL_DQ_3 12bits offset_0 */
	clrsetbits_le32(&denali_phy[915], 0xfff, reg_value);

	reg_value = (vref_mode_ac << 9) | (0x1 << 8) | vref_value_ac;

	/* PHY_915 PHY_PAD_VREF_CTRL_AC 12bits offset_16 */
	clrsetbits_le32(&denali_phy[915], 0xfff << 16, reg_value << 16);

	if (sdram_params->base.dramtype == LPDDR4)
		mode_sel = 0x6;
	else if (sdram_params->base.dramtype == LPDDR3)
		mode_sel = 0x0;
	else if (sdram_params->base.dramtype == DDR3)
		mode_sel = 0x1;
	else
		return -EINVAL;

	/* PHY_924 PHY_PAD_FDBK_DRIVE */
	clrsetbits_le32(&denali_phy[924], 0x7 << 15, mode_sel << 15);
	/* PHY_926 PHY_PAD_DATA_DRIVE */
	clrsetbits_le32(&denali_phy[926], 0x7 << 6, mode_sel << 6);
	/* PHY_927 PHY_PAD_DQS_DRIVE */
	clrsetbits_le32(&denali_phy[927], 0x7 << 6, mode_sel << 6);
	/* PHY_928 PHY_PAD_ADDR_DRIVE */
	clrsetbits_le32(&denali_phy[928], 0x7 << 14, mode_sel << 14);
	/* PHY_929 PHY_PAD_CLK_DRIVE */
	clrsetbits_le32(&denali_phy[929], 0x7 << 14, mode_sel << 14);
	/* PHY_935 PHY_PAD_CKE_DRIVE */
	clrsetbits_le32(&denali_phy[935], 0x7 << 14, mode_sel << 14);
	/* PHY_937 PHY_PAD_RST_DRIVE */
	clrsetbits_le32(&denali_phy[937], 0x7 << 14, mode_sel << 14);
	/* PHY_939 PHY_PAD_CS_DRIVE */
	clrsetbits_le32(&denali_phy[939], 0x7 << 14, mode_sel << 14);


	/* speed setting */
	if (sdram_params->base.ddr_freq < 400)
		speed = 0x0;
	else if (sdram_params->base.ddr_freq < 800)
		speed = 0x1;
	else if (sdram_params->base.ddr_freq < 1200)
		speed = 0x2;
	else
		speed = 0x3;

	/* PHY_924 PHY_PAD_FDBK_DRIVE */
	clrsetbits_le32(&denali_phy[924], 0x3 << 21, speed << 21);
	/* PHY_926 PHY_PAD_DATA_DRIVE */
	clrsetbits_le32(&denali_phy[926], 0x3 << 9, speed << 9);
	/* PHY_927 PHY_PAD_DQS_DRIVE */
	clrsetbits_le32(&denali_phy[927], 0x3 << 9, speed << 9);
	/* PHY_928 PHY_PAD_ADDR_DRIVE */
	clrsetbits_le32(&denali_phy[928], 0x3 << 17, speed << 17);
	/* PHY_929 PHY_PAD_CLK_DRIVE */
	clrsetbits_le32(&denali_phy[929], 0x3 << 17, speed << 17);
	/* PHY_935 PHY_PAD_CKE_DRIVE */
	clrsetbits_le32(&denali_phy[935], 0x3 << 17, speed << 17);
	/* PHY_937 PHY_PAD_RST_DRIVE */
	clrsetbits_le32(&denali_phy[937], 0x3 << 17, speed << 17);
	/* PHY_939 PHY_PAD_CS_DRIVE */
	clrsetbits_le32(&denali_phy[939], 0x3 << 17, speed << 17);

	return 0;
}

static uint32_t get_pi_rdlat_adj(struct dram_timing_t *pdram_timing)
{
	/*[DLLSUBTYPE2] == "STD_DENALI_HS" */
	uint32_t rdlat, delay_adder, ie_enable, hs_offset, tsel_adder,
	    extra_adder, tsel_enable;

	ie_enable = PI_IE_ENABLE_VALUE;
	tsel_enable = PI_TSEL_ENABLE_VALUE;

	rdlat = pdram_timing->cl + PI_ADD_LATENCY;
	delay_adder = ie_enable / (1000000 / pdram_timing->mhz);
	if ((ie_enable % (1000000 / pdram_timing->mhz)) != 0)
		delay_adder++;
	hs_offset = 0;
	tsel_adder = 0;
	extra_adder = 0;
	/* rdlat = rdlat - (PREAMBLE_SUPPORT & 0x1); */
	tsel_adder = tsel_enable / (1000000 / pdram_timing->mhz);
	if ((tsel_enable % (1000000 / pdram_timing->mhz)) != 0)
		tsel_adder++;
	delay_adder = delay_adder - 1;
	if (tsel_adder > delay_adder)
		extra_adder = tsel_adder - delay_adder;
	else
		extra_adder = 0;
	if (PI_REGS_DIMM_SUPPORT && PI_DOUBLEFREEK)
		hs_offset = 2;
	else
		hs_offset = 1;

	if (delay_adder > (rdlat - 1 - hs_offset)) {
		rdlat = rdlat - tsel_adder;
	} else {
		if ((rdlat - delay_adder) < 2)
			rdlat = 2;
		else
			rdlat = rdlat - delay_adder - extra_adder;
	}

	return rdlat;
}

static uint32_t get_pi_wrlat(struct dram_timing_t *pdram_timing,
			     struct timing_related_config *timing_config)
{
	uint32_t tmp;

	if (timing_config->dram_type == LPDDR3) {
		tmp = pdram_timing->cl;
		if (tmp >= 14)
			tmp = 8;
		else if (tmp >= 10)
			tmp = 6;
		else if (tmp == 9)
			tmp = 5;
		else if (tmp == 8)
			tmp = 4;
		else if (tmp == 6)
			tmp = 3;
		else
			tmp = 1;
	} else {
		tmp = 1;
	}

	return tmp;
}

static uint32_t get_pi_wrlat_adj(struct dram_timing_t *pdram_timing,
				 struct timing_related_config *timing_config)
{
	return get_pi_wrlat(pdram_timing, timing_config) + PI_ADD_LATENCY - 1;
}

static uint32_t get_pi_tdfi_phy_rdlat(struct dram_timing_t *pdram_timing,
			struct timing_related_config *timing_config)
{
	/* [DLLSUBTYPE2] == "STD_DENALI_HS" */
	uint32_t cas_lat, delay_adder, ie_enable, hs_offset, ie_delay_adder;
	uint32_t mem_delay_ps, round_trip_ps;
	uint32_t phy_internal_delay, lpddr_adder, dfi_adder, rdlat_delay;

	ie_enable = PI_IE_ENABLE_VALUE;

	delay_adder = ie_enable / (1000000 / pdram_timing->mhz);
	if ((ie_enable % (1000000 / pdram_timing->mhz)) != 0)
		delay_adder++;
	delay_adder = delay_adder - 1;
	if (PI_REGS_DIMM_SUPPORT && PI_DOUBLEFREEK)
		hs_offset = 2;
	else
		hs_offset = 1;

	cas_lat = pdram_timing->cl + PI_ADD_LATENCY;

	if (delay_adder > (cas_lat - 1 - hs_offset)) {
		ie_delay_adder = 0;
	} else {
		ie_delay_adder = ie_enable / (1000000 / pdram_timing->mhz);
		if ((ie_enable % (1000000 / pdram_timing->mhz)) != 0)
			ie_delay_adder++;
	}

	if (timing_config->dram_type == DDR3) {
		mem_delay_ps = 0;
	} else if (timing_config->dram_type == LPDDR4) {
		mem_delay_ps = 3600;
	} else if (timing_config->dram_type == LPDDR3) {
		mem_delay_ps = 5500;
	} else {
		printf("get_pi_tdfi_phy_rdlat:dramtype unsupport\n");
		return 0;
	}
	round_trip_ps = 1100 + 500 + mem_delay_ps + 500 + 600;
	delay_adder = round_trip_ps / (1000000 / pdram_timing->mhz);
	if ((round_trip_ps % (1000000 / pdram_timing->mhz)) != 0)
		delay_adder++;

	phy_internal_delay = 5 + 2 + 4;
	lpddr_adder = mem_delay_ps / (1000000 / pdram_timing->mhz);
	if ((mem_delay_ps % (1000000 / pdram_timing->mhz)) != 0)
		lpddr_adder++;
	dfi_adder = 0;
	phy_internal_delay = phy_internal_delay + 2;
	rdlat_delay = delay_adder + phy_internal_delay +
	    ie_delay_adder + lpddr_adder + dfi_adder;

	rdlat_delay = rdlat_delay + 2;
	return rdlat_delay;
}

static uint32_t get_pi_todtoff_min(struct dram_timing_t *pdram_timing,
				   struct timing_related_config *timing_config)
{
	uint32_t tmp, todtoff_min_ps;

	if (timing_config->dram_type == LPDDR3)
		todtoff_min_ps = 2500;
	else if (timing_config->dram_type == LPDDR4)
		todtoff_min_ps = 1500;
	else
		todtoff_min_ps = 0;
	/* todtoff_min */
	tmp = todtoff_min_ps / (1000000 / pdram_timing->mhz);
	if ((todtoff_min_ps % (1000000 / pdram_timing->mhz)) != 0)
		tmp++;
	return tmp;
}

static uint32_t get_pi_todtoff_max(struct dram_timing_t *pdram_timing,
				   struct timing_related_config *timing_config)
{
	uint32_t tmp, todtoff_max_ps;

	if ((timing_config->dram_type == LPDDR4)
	    || (timing_config->dram_type == LPDDR3))
		todtoff_max_ps = 3500;
	else
		todtoff_max_ps = 0;

	/* todtoff_max */
	tmp = todtoff_max_ps / (1000000 / pdram_timing->mhz);
	if ((todtoff_max_ps % (1000000 / pdram_timing->mhz)) != 0)
		tmp++;
	return tmp;
}

struct lat_adj_pair {
	uint32_t cl;
	uint32_t rdlat_adj;
	uint32_t cwl;
	uint32_t wrlat_adj;
};

const struct lat_adj_pair ddr3_lat_adj[] = {
	{6, 5, 5, 4},
	{8, 7, 6, 5},
	{10, 9, 7, 6},
	{11, 9, 8, 7},
	{13, 0xb, 9, 8},
	{14, 0xb, 0xa, 9}
};

static uint32_t get_rdlat_adj(uint32_t dram_type, uint32_t cl)
{
	const struct lat_adj_pair *p;
	uint32_t cnt;
	uint32_t i;

	if (dram_type == DDR3) {
		p = ddr3_lat_adj;
		cnt = ARRAY_SIZE(ddr3_lat_adj);
	}
#if 0 // TODO
	else if (dram_type == LPDDR3) {
		p = lpddr3_lat_adj;
		cnt = ARRAY_SIZE(lpddr3_lat_adj);
	} else {
		p = lpddr4_lat_adj;
		cnt = ARRAY_SIZE(lpddr4_lat_adj);
	}
#endif

	for (i = 0; i < cnt; i++) {
		if (cl == p[i].cl)
			return p[i].rdlat_adj;
	}
	/* fail */
	return 0xff;
}

static uint32_t get_wrlat_adj(uint32_t dram_type, uint32_t cwl)
{
	const struct lat_adj_pair *p;
	uint32_t cnt;
	uint32_t i;

	if (dram_type == DDR3) {
		p = ddr3_lat_adj;
		cnt = ARRAY_SIZE(ddr3_lat_adj);
	}
#if 0
	else if (dram_type == LPDDR3) {
		p = lpddr3_lat_adj;
		cnt = ARRAY_SIZE(lpddr3_lat_adj);
	} else {
		p = lpddr4_lat_adj;
		cnt = ARRAY_SIZE(lpddr4_lat_adj);
	}
#endif

	for (i = 0; i < cnt; i++) {
		if (cwl == p[i].cwl)
			return p[i].wrlat_adj;
	}
	/* fail */
	return 0xff;
}


static void gen_rk3399_ctl_params_f0(const struct chan_info *chan,
				     struct timing_related_config *timing_config,
				     struct dram_timing_t *pdram_timing)
{
	u32 *denali_ctl = chan->pctl->denali_ctl;
	//	u32 *denali_pi = chan->pi->denali_pi;
	//	u32 *denali_phy = chan->publ->denali_phy;
	uint32_t i;
	uint32_t tmp, tmp1;

	for (i = 0; i < timing_config->ch_cnt; i++) {
		if (timing_config->dram_type == DDR3) {
			tmp = ((700000 + 10) * timing_config->freq +
				999) / 1000;
			tmp += pdram_timing->txsnr + (pdram_timing->tmrd * 3) +
			    pdram_timing->tmod + pdram_timing->tzqinit;
			writel(tmp, &denali_ctl[5]);

			clrsetbits_le32(&denali_ctl[22], 0xffff, pdram_timing->tdllk);

			tmp = (pdram_timing->tmod << 8) | pdram_timing->tmrd;
			writel(tmp, &denali_ctl[32]);

			clrsetbits_le32(&denali_ctl[59], 0xffff << 16,
					(pdram_timing->txsr - pdram_timing->trcd) << 16);
#if 0
		} else if (timing_config->dram_type == LPDDR4) {
			mmio_write_32(CTL_REG(i, 5), pdram_timing->tinit1 +
						     pdram_timing->tinit3);
			mmio_write_32(CTL_REG(i, 32),
				      (pdram_timing->tmrd << 8) |
				      pdram_timing->tmrd);
			mmio_clrsetbits_32(CTL_REG(i, 59), 0xffff << 16,
					   pdram_timing->txsr << 16);
		} else {
			mmio_write_32(CTL_REG(i, 5), pdram_timing->tinit1);
			mmio_write_32(CTL_REG(i, 7), pdram_timing->tinit4);
			mmio_write_32(CTL_REG(i, 32),
				      (pdram_timing->tmrd << 8) |
				      pdram_timing->tmrd);
			mmio_clrsetbits_32(CTL_REG(i, 59), 0xffff << 16,
					   pdram_timing->txsr << 16);
#endif
		}
		writel(pdram_timing->tinit3, &denali_ctl[6]);
		writel(pdram_timing->tinit5, &denali_ctl[8]);
		clrsetbits_le32(&denali_ctl[23], (0x7f << 16),
				((pdram_timing->cl * 2) << 16));
		clrsetbits_le32(&denali_ctl[23], (0x1f << 24),
				(pdram_timing->cwl << 24));
		clrsetbits_le32(&denali_ctl[24], 0x3f, pdram_timing->al);
		clrsetbits_le32(&denali_ctl[26], 0xffff << 16,
				(pdram_timing->trc << 24) |
				(pdram_timing->trrd << 16));
		writel(((pdram_timing->tfaw << 24) |
			(pdram_timing->trppb << 16) |
			(pdram_timing->twtr << 8) |
			pdram_timing->tras_min),
		       &denali_ctl[27]);

		clrsetbits_le32(&denali_ctl[31], 0xff << 24,
				max(4, pdram_timing->trtp) << 24);
		writel(((pdram_timing->tcke << 24) |
			pdram_timing->tras_max),
		       &denali_ctl[33]);

		clrsetbits_le32(&denali_ctl[34], 0xff,
				   max(1, pdram_timing->tckesr));
		clrsetbits_le32(&denali_ctl[39],
				(0x3f << 16) | (0xff << 8),
				(pdram_timing->twr << 16) |
				(pdram_timing->trcd << 8));
		clrsetbits_le32(&denali_ctl[42], 0x1f << 16,
			      pdram_timing->tmrz << 16);
		tmp = pdram_timing->tdal ? pdram_timing->tdal :
		      (pdram_timing->twr + pdram_timing->trp);
		clrsetbits_le32(&denali_ctl[44], 0xff, tmp);
		clrsetbits_le32(&denali_ctl[45], 0xff, pdram_timing->trp);
		writel(((pdram_timing->trefi - 8) << 16) | pdram_timing->trfc,
		       &denali_ctl[48]);
		clrsetbits_le32(&denali_ctl[52], 0xffff, pdram_timing->txp);
		clrsetbits_le32(&denali_ctl[53], 0xffff << 16,
				pdram_timing->txpdll << 16);
		clrsetbits_le32(&denali_ctl[55], 0xf << 24,
				pdram_timing->tcscke << 24);
		clrsetbits_le32(&denali_ctl[55], 0xff,
				pdram_timing->tmrri);
		writel((pdram_timing->tzqcke << 24) |
		       (pdram_timing->tmrwckel << 16) |
		       (pdram_timing->tckehcs << 8) |
		       pdram_timing->tckelcs,
		       &denali_ctl[56]);
		clrsetbits_le32(&denali_ctl[60], 0xffff, pdram_timing->txsnr);
		clrsetbits_le32(&denali_ctl[62], 0xffff << 16,
				   (pdram_timing->tckehcmd << 24) |
				   (pdram_timing->tckelcmd << 16));
		writel((pdram_timing->tckelpd << 24) |
		       (pdram_timing->tescke << 16) |
		       (pdram_timing->tsr << 8) |
		       pdram_timing->tckckel,
		       &denali_ctl[63]);

		clrsetbits_le32(&denali_ctl[64], 0xfff,
				(pdram_timing->tcmdcke << 8) |
				pdram_timing->tcsckeh);
		clrsetbits_le32(&denali_ctl[92], 0xffff << 8,
				(pdram_timing->tcksrx << 16) |
				(pdram_timing->tcksre << 8));
		clrsetbits_le32(&denali_ctl[108], 0x1 << 24,
				(timing_config->dllbp << 24));
		clrsetbits_le32(&denali_ctl[122], 0x3ff << 16,
				(pdram_timing->tvrcg_enable << 16));
		writel((pdram_timing->tfc_long << 16) |
		       pdram_timing->tvrcg_disable,
		       &denali_ctl[123]);
		writel((pdram_timing->tvref_long << 16) |
		       (pdram_timing->tckfspx << 8) |
		       pdram_timing->tckfspe,
		       &denali_ctl[124]);

		writel((pdram_timing->mr[1] << 16) |
		       pdram_timing->mr[0], &denali_ctl[133]);
		clrsetbits_le32(&denali_ctl[134], 0xffff,
				pdram_timing->mr[2]);
		clrsetbits_le32(&denali_ctl[138], 0xffff,
				pdram_timing->mr[3]);
		clrsetbits_le32(&denali_ctl[139], 0xff << 24,
				pdram_timing->mr11 << 24);
		writel((pdram_timing->mr[1] << 16) |
		       pdram_timing->mr[0],
		       &denali_ctl[147]);
		clrsetbits_le32(&denali_ctl[148], 0xffff,
				pdram_timing->mr[2]);
		clrsetbits_le32(&denali_ctl[152], 0xffff,
				pdram_timing->mr[3]);
		clrsetbits_le32(&denali_ctl[153], 0xff << 24,
				pdram_timing->mr11 << 24);
#if 0  /* MARK */
		/*
		if (timing_config->dram_type == LPDDR4) {
			mmio_clrsetbits_32(CTL_REG(i, 140), 0xffff << 16,
					   pdram_timing->mr12 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 142), 0xffff << 16,
					   pdram_timing->mr14 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 145), 0xffff << 16,
					   pdram_timing->mr22 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 154), 0xffff << 16,
					   pdram_timing->mr12 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 156), 0xffff << 16,
					   pdram_timing->mr14 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 159), 0xffff << 16,
					   pdram_timing->mr22 << 16);
		}
		*/
		mmio_clrsetbits_32(CTL_REG(i, 179), 0xfff << 8,
				   pdram_timing->tzqinit << 8);
		mmio_write_32(CTL_REG(i, 180), (pdram_timing->tzqcs << 16) |
					       (pdram_timing->tzqinit / 2));
		mmio_write_32(CTL_REG(i, 181), (pdram_timing->tzqlat << 16) |
					       pdram_timing->tzqcal);
		mmio_clrsetbits_32(CTL_REG(i, 212), 0xff << 8,
				   pdram_timing->todton << 8);

		if (timing_config->odt) {
			mmio_setbits_32(CTL_REG(i, 213), 1 << 16);
			if (timing_config->freq < 400)
				tmp = 4 << 24;
			else
				tmp = 8 << 24;
		} else {
			mmio_clrbits_32(CTL_REG(i, 213), 1 << 16);
			tmp = 2 << 24;
		}

		mmio_clrsetbits_32(CTL_REG(i, 216), 0x1f << 24, tmp);
		mmio_clrsetbits_32(CTL_REG(i, 221), (0x3 << 16) | (0xf << 8),
				   (pdram_timing->tdqsck << 16) |
				   (pdram_timing->tdqsck_max << 8));
#endif


		tmp =
		    (get_wrlat_adj(timing_config->dram_type, pdram_timing->cwl)
		     << 8) | get_rdlat_adj(timing_config->dram_type,
					   pdram_timing->cl);

		clrsetbits_le32(&denali_ctl[284], 0xffff, tmp);

		clrsetbits_le32(&denali_ctl[82], 0xffff << 16,
				(4 * pdram_timing->trefi) << 16);

		clrsetbits_le32(&denali_ctl[83], 0xffff,
				(2 * pdram_timing->trefi) & 0xffff);

		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			tmp = get_pi_wrlat(pdram_timing, timing_config);
			tmp1 = get_pi_todtoff_max(pdram_timing, timing_config);
			tmp = (tmp > tmp1) ? (tmp - tmp1) : 0;
		} else {
			tmp = 0;
		}
		clrsetbits_le32(&denali_ctl[214], 0x3f << 16,
				   (tmp & 0x3f) << 16);

		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			/* min_rl_preamble = cl+TDQSCK_MIN -1 */
			tmp = pdram_timing->cl +
			    get_pi_todtoff_min(pdram_timing, timing_config) - 1;
			/* todtoff_max */
			tmp1 = get_pi_todtoff_max(pdram_timing, timing_config);
			tmp = (tmp > tmp1) ? (tmp - tmp1) : 0;
		} else {
			tmp = pdram_timing->cl - pdram_timing->cwl;
		}
		clrsetbits_le32(&denali_ctl[215], 0x3f << 8,
				   (tmp & 0x3f) << 8);

		clrsetbits_le32(&denali_ctl[275], 0xff << 16,
				   (get_pi_tdfi_phy_rdlat(pdram_timing,
							  timing_config) &
				    0xff) << 16);

		clrsetbits_le32(&denali_ctl[277], 0xffff,
				   (2 * pdram_timing->trefi) & 0xffff);

		clrsetbits_le32(&denali_ctl[282], 0xffff,
				   (2 * pdram_timing->trefi) & 0xffff);

		writel(20 * pdram_timing->trefi,
		       &denali_ctl[283]);

		/* CTL_308 TDFI_CALVL_CAPTURE_F0:RW:16:10 */
		tmp1 = 20000 / (1000000 / pdram_timing->mhz) + 1;
		if ((20000 % (1000000 / pdram_timing->mhz)) != 0)
			tmp1++;
		tmp = (tmp1 >> 1) + (tmp1 % 2) + 5;
		clrsetbits_le32(&denali_ctl[308], 0x3ff << 16, tmp << 16);

		/* CTL_308 TDFI_CALVL_CC_F0:RW:0:10 */
		tmp = tmp + 18;
		clrsetbits_le32(&denali_ctl[308], 0x3ff, tmp);

		/* CTL_314 TDFI_WRCSLAT_F0:RW:8:8 */
		tmp1 = get_pi_wrlat_adj(pdram_timing, timing_config);
		if (timing_config->freq <= TDFI_LAT_THRESHOLD_FREQ) {
			if (tmp1 == 0)
				tmp = 0;
			else if (tmp1 < 5)
				tmp = tmp1 - 1;
			else
				tmp = tmp1 - 5;
		} else {
			tmp = tmp1 - 2;
		}
		clrsetbits_le32(&denali_ctl[314], 0xff << 8, tmp << 8);

		/* CTL_314 TDFI_RDCSLAT_F0:RW:0:8 */
		if ((timing_config->freq <= TDFI_LAT_THRESHOLD_FREQ) &&
		    (pdram_timing->cl >= 5))
			tmp = pdram_timing->cl - 5;
		else
			tmp = pdram_timing->cl - 2;
		clrsetbits_le32(&denali_ctl[314], 0xff, tmp);
	}
}

static void gen_rk3399_ctl_params_f1(const struct chan_info *chan,
				     struct timing_related_config *timing_config,
				     struct dram_timing_t *pdram_timing)
{
	u32 *denali_ctl = chan->pctl->denali_ctl;
	//	u32 *denali_pi = chan->pi->denali_pi;
	//	u32 *denali_phy = chan->publ->denali_phy;
	uint32_t i;
	uint32_t tmp, tmp1;

	for (i = 0; i < timing_config->ch_cnt; i++) {
		if (timing_config->dram_type == DDR3) {
			tmp = ((700000 + 10) * timing_config->freq +
				999) / 1000;
			tmp += pdram_timing->txsnr + (pdram_timing->tmrd * 3) +
			    pdram_timing->tmod + pdram_timing->tzqinit;
			writel(tmp, &denali_ctl[9]);

			clrsetbits_le32(&denali_ctl[22], 0xffff << 16, pdram_timing->tdllk << 16);

			tmp = (pdram_timing->tmod << 24) |
			      (pdram_timing->tmrd << 16) |
			      (pdram_timing->trtp << 8);
			writel(tmp, &denali_ctl[34]);

			clrsetbits_le32(&denali_ctl[60], 0xffff << 16,
					(pdram_timing->txsr - pdram_timing->trcd) << 16);
#if 0
		} else if (timing_config->dram_type == LPDDR4) {
			mmio_write_32(CTL_REG(i, 5), pdram_timing->tinit1 +
						     pdram_timing->tinit3);
			mmio_write_32(CTL_REG(i, 32),
				      (pdram_timing->tmrd << 8) |
				      pdram_timing->tmrd);
			mmio_clrsetbits_32(CTL_REG(i, 59), 0xffff << 16,
					   pdram_timing->txsr << 16);
		} else {
			mmio_write_32(CTL_REG(i, 5), pdram_timing->tinit1);
			mmio_write_32(CTL_REG(i, 7), pdram_timing->tinit4);
			mmio_write_32(CTL_REG(i, 32),
				      (pdram_timing->tmrd << 8) |
				      pdram_timing->tmrd);
			mmio_clrsetbits_32(CTL_REG(i, 59), 0xffff << 16,
					   pdram_timing->txsr << 16);
#endif
		}
		writel(pdram_timing->tinit3, &denali_ctl[10]);
		writel(pdram_timing->tinit5, &denali_ctl[12]);
		clrsetbits_le32(&denali_ctl[24], (0x7f << 8),
				((pdram_timing->cl * 2) << 8));
		clrsetbits_le32(&denali_ctl[24], (0x1f << 16),
				(pdram_timing->cwl << 16));
		clrsetbits_le32(&denali_ctl[24], 0x3f << 24, pdram_timing->al << 24);
		clrsetbits_le32(&denali_ctl[28], 0xffffff << 8,
				(pdram_timing->tras_min << 24) |
				(pdram_timing->trc << 16) |
				(pdram_timing->trrd << 8));
		clrsetbits_le32(&denali_ctl[29], 0xffffff,
				(pdram_timing->tfaw << 16) |
				(pdram_timing->trppb << 8) |
				(pdram_timing->twtr));

		writel(pdram_timing->tras_max | (pdram_timing->tcke << 24),
		       &denali_ctl[35]);

		clrsetbits_le32(&denali_ctl[36], 0xff,
				   max(1, pdram_timing->tckesr));
		clrsetbits_le32(&denali_ctl[39],
				(0xff << 24),
				(pdram_timing->trcd << 24));
		clrsetbits_le32(&denali_ctl[40], 0x3f, pdram_timing->twr);
		clrsetbits_le32(&denali_ctl[42], 0x1f << 24,
			      pdram_timing->tmrz << 24);
		tmp = pdram_timing->tdal ? pdram_timing->tdal :
		      (pdram_timing->twr + pdram_timing->trp);
		clrsetbits_le32(&denali_ctl[44], 0xff << 8, tmp << 8);
		clrsetbits_le32(&denali_ctl[45], 0xff << 8, pdram_timing->trp << 8);
		writel(((pdram_timing->trefi - 8) << 16) | pdram_timing->trfc,
		       &denali_ctl[49]);
		clrsetbits_le32(&denali_ctl[52], 0xffff << 16, pdram_timing->txp << 16);
		clrsetbits_le32(&denali_ctl[54], 0xffff, pdram_timing->txpdll);
		//		clrsetbits_le32(&denali_ctl[55], 0xf << 24,
		//				pdram_timing->tcscke << 24);
		clrsetbits_le32(&denali_ctl[55], 0xff << 8,
				pdram_timing->tmrri << 8);
		writel((pdram_timing->tzqcke << 24) |
		       (pdram_timing->tmrwckel << 16) |
		       (pdram_timing->tckehcs << 8) |
		       pdram_timing->tckelcs,
		       &denali_ctl[57]);
		//
		clrsetbits_le32(&denali_ctl[61], 0xffff, pdram_timing->txsnr);
		clrsetbits_le32(&denali_ctl[64], 0xffff << 16,
				   (pdram_timing->tckehcmd << 24) |
				   (pdram_timing->tckelcmd << 16));
		writel((pdram_timing->tckelpd << 24) |
		       (pdram_timing->tescke << 16) |
		       (pdram_timing->tsr << 8) |
		       pdram_timing->tckckel,
		       &denali_ctl[65]);

		clrsetbits_le32(&denali_ctl[66], 0xfff,
				(pdram_timing->tcmdcke << 8) |
				pdram_timing->tcsckeh);
		clrsetbits_le32(&denali_ctl[92], 0xff << 24,
				(pdram_timing->tcksre << 24));
		clrsetbits_le32(&denali_ctl[93], 0xff, pdram_timing->tcksrx);
		clrsetbits_le32(&denali_ctl[108], 0x1 << 25,
				(timing_config->dllbp << 25));
		clrsetbits_le32(&denali_ctl[125], 0x3ff << 16,
				(pdram_timing->tvrcg_enable << 16));
		writel(pdram_timing->tfc_long |
		       (pdram_timing->tckfspx << 24) |
		       (pdram_timing->tckfspe << 16),
		       &denali_ctl[126]);
		clrsetbits_le32(&denali_ctl[127], 0xffff, pdram_timing->tvref_long);

		clrsetbits_le32(&denali_ctl[134], 0xffff << 16,
				pdram_timing->mr[0] << 16);
		writel((pdram_timing->mr[2] << 16) |
		       pdram_timing->mr[1], &denali_ctl[135]);
		clrsetbits_le32(&denali_ctl[138], 0xffff << 16,
				pdram_timing->mr[3] << 16);
		clrsetbits_le32(&denali_ctl[140], 0xff,
				pdram_timing->mr11);
		clrsetbits_le32(&denali_ctl[148], 0xffff << 16,
				pdram_timing->mr[0] << 16);
		writel((pdram_timing->mr[2] << 16) |
		       pdram_timing->mr[1],
		       &denali_ctl[149]);
		clrsetbits_le32(&denali_ctl[152], 0xffff << 16,
				pdram_timing->mr[3] << 16);
		clrsetbits_le32(&denali_ctl[154], 0xff,
				pdram_timing->mr11);
#if 0  /* MARK */
		/*
		if (timing_config->dram_type == LPDDR4) {
			mmio_clrsetbits_32(CTL_REG(i, 140), 0xffff << 16,
					   pdram_timing->mr12 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 142), 0xffff << 16,
					   pdram_timing->mr14 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 145), 0xffff << 16,
					   pdram_timing->mr22 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 154), 0xffff << 16,
					   pdram_timing->mr12 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 156), 0xffff << 16,
					   pdram_timing->mr14 << 16);
			mmio_clrsetbits_32(CTL_REG(i, 159), 0xffff << 16,
					   pdram_timing->mr22 << 16);
		}
		*/
		mmio_clrsetbits_32(CTL_REG(i, 179), 0xfff << 8,
				   pdram_timing->tzqinit << 8);
		mmio_write_32(CTL_REG(i, 180), (pdram_timing->tzqcs << 16) |
					       (pdram_timing->tzqinit / 2));
		mmio_write_32(CTL_REG(i, 181), (pdram_timing->tzqlat << 16) |
					       pdram_timing->tzqcal);
		mmio_clrsetbits_32(CTL_REG(i, 212), 0xff << 8,
				   pdram_timing->todton << 8);

		if (timing_config->odt) {
			mmio_setbits_32(CTL_REG(i, 213), 1 << 16);
			if (timing_config->freq < 400)
				tmp = 4 << 24;
			else
				tmp = 8 << 24;
		} else {
			mmio_clrbits_32(CTL_REG(i, 213), 1 << 16);
			tmp = 2 << 24;
		}

		mmio_clrsetbits_32(CTL_REG(i, 216), 0x1f << 24, tmp);
		mmio_clrsetbits_32(CTL_REG(i, 221), (0x3 << 16) | (0xf << 8),
				   (pdram_timing->tdqsck << 16) |
				   (pdram_timing->tdqsck_max << 8));
#endif


		tmp =
		    (get_wrlat_adj(timing_config->dram_type, pdram_timing->cwl)
		     << 8) | get_rdlat_adj(timing_config->dram_type,
					   pdram_timing->cl);

		clrsetbits_le32(&denali_ctl[291], 0xffff, tmp);

		clrsetbits_le32(&denali_ctl[84], 0xffff,
				(4 * pdram_timing->trefi));
		clrsetbits_le32(&denali_ctl[84], 0xffff << 16,
				((2 * pdram_timing->trefi) & 0xffff) << 16);

		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			tmp = get_pi_wrlat(pdram_timing, timing_config);
			tmp1 = get_pi_todtoff_max(pdram_timing, timing_config);
			tmp = (tmp > tmp1) ? (tmp - tmp1) : 0;
		} else {
			tmp = 0;
		}
		clrsetbits_le32(&denali_ctl[214], 0x3f << 24,
				   (tmp & 0x3f) << 24);

		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			/* min_rl_preamble = cl+TDQSCK_MIN -1 */
			tmp = pdram_timing->cl +
			    get_pi_todtoff_min(pdram_timing, timing_config) - 1;
			/* todtoff_max */
			tmp1 = get_pi_todtoff_max(pdram_timing, timing_config);
			tmp = (tmp > tmp1) ? (tmp - tmp1) : 0;
		} else {
			tmp = pdram_timing->cl - pdram_timing->cwl;
		}
		clrsetbits_le32(&denali_ctl[215], 0x3f << 24,
				   (tmp & 0x3f) << 24);

		clrsetbits_le32(&denali_ctl[275], 0xff << 24,
				   (get_pi_tdfi_phy_rdlat(pdram_timing,
							  timing_config) &
				    0xff) << 24);

		clrsetbits_le32(&denali_ctl[284], 0xffff << 16,
				((2 * pdram_timing->trefi) & 0xffff) << 16);

		clrsetbits_le32(&denali_ctl[289], 0xffff,
				   (2 * pdram_timing->trefi) & 0xffff);

		writel(20 * pdram_timing->trefi,
		       &denali_ctl[290]);

		/* CTL_308 TDFI_CALVL_CAPTURE_F0:RW:16:10 */
		tmp1 = 20000 / (1000000 / pdram_timing->mhz) + 1;
		if ((20000 % (1000000 / pdram_timing->mhz)) != 0)
			tmp1++;
		tmp = (tmp1 >> 1) + (tmp1 % 2) + 5;
		clrsetbits_le32(&denali_ctl[309], 0x3ff << 16, tmp << 16);

		/* CTL_308 TDFI_CALVL_CC_F1:RW:0:10 */
		tmp = tmp + 18;
		clrsetbits_le32(&denali_ctl[309], 0x3ff, tmp);

		/* CTL_314 TDFI_WRCSLAT_F1:RW:24:8 */
		tmp1 = get_pi_wrlat_adj(pdram_timing, timing_config);
		if (timing_config->freq <= TDFI_LAT_THRESHOLD_FREQ) {
			if (tmp1 == 0)
				tmp = 0;
			else if (tmp1 < 5)
				tmp = tmp1 - 1;
			else
				tmp = tmp1 - 5;
		} else {
			tmp = tmp1 - 2;
		}
		clrsetbits_le32(&denali_ctl[314], 0xff << 24, tmp << 24);

		/* CTL_314 TDFI_RDCSLAT_F1:RW:16:8 */
		if ((timing_config->freq <= TDFI_LAT_THRESHOLD_FREQ) &&
		    (pdram_timing->cl >= 5))
			tmp = pdram_timing->cl - 5;
		else
			tmp = pdram_timing->cl - 2;
		clrsetbits_le32(&denali_ctl[314], 0xff << 16, tmp << 16);
	}
}


static void gen_rk3399_phy_params(const struct chan_info *chan,
				  struct timing_related_config *timing_config,
				  /*				  struct drv_odt_lp_config *drv_config, */
				  struct dram_timing_t *pdram_timing,
  uint32_t fn)
{
	uint32_t tmp, i, div, j;
	uint32_t mem_delay_ps, pad_delay_ps, total_delay_ps, delay_frac_ps;
	uint32_t trpre_min_ps, gate_delay_ps, gate_delay_frac_ps;
	uint32_t ie_enable, tsel_enable, cas_lat, rddata_en_ie_dly, tsel_adder;
	uint32_t extra_adder, delta, hs_offset;
	//	u32 fn = 0;
	u32 *denali_phy = chan->publ->denali_phy;

	for (i = 0; i < timing_config->ch_cnt; i++) {

		pad_delay_ps = PI_PAD_DELAY_PS_VALUE;
		ie_enable = PI_IE_ENABLE_VALUE;
		tsel_enable = PI_TSEL_ENABLE_VALUE;

		clrsetbits_le32(&denali_phy[896], (0x3 << 8) | 1, fn << 8);

		/* PHY_LOW_FREQ_SEL */
		/* DENALI_PHY_913 1bit offset_0 */
		if (timing_config->freq > 400)
			clrbits_le32(&denali_phy[913], 1);
		else
			setbits_le32(&denali_phy[913], 1);

		/* PHY_RPTR_UPDATE_x */
		/* DENALI_PHY_87/215/343/471 4bit offset_16 */
		tmp = 2500 / (1000000 / pdram_timing->mhz) + 3;
		if ((2500 % (1000000 / pdram_timing->mhz)) != 0)
			tmp++;
		clrsetbits_le32(&denali_phy[87], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[215], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[343], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[471], 0xf << 16, tmp << 16);

		/* PHY_PLL_CTRL */
		/* DENALI_PHY_911 13bits offset_0 */
		/* PHY_LP4_BOOT_PLL_CTRL */
		/* DENALI_PHY_919 13bits offset_0 */
		tmp = (1 << 12) | (2 << 7) | (1 << 1);
		clrsetbits_le32(&denali_phy[911], 0x1fff, tmp);
		clrsetbits_le32(&denali_phy[919], 0x1fff, tmp);

		/* PHY_PLL_CTRL_CA */
		/* DENALI_PHY_911 13bits offset_16 */
		/* PHY_LP4_BOOT_PLL_CTRL_CA */
		/* DENALI_PHY_919 13bits offset_16 */
		tmp = (2 << 7) | (1 << 5) | (1 << 1);
		clrsetbits_le32(&denali_phy[911], 0x1fff << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[919], 0x1fff << 16, tmp << 16);

		/* PHY_TCKSRE_WAIT */
		/* DENALI_PHY_922 4bits offset_24 */
		if (pdram_timing->mhz <= 400)
			tmp = 1;
		else if (pdram_timing->mhz <= 800)
			tmp = 3;
		else if (pdram_timing->mhz <= 1000)
			tmp = 4;
		else
			tmp = 5;
		clrsetbits_le32(&denali_phy[922], 0xf << 24, tmp << 24);
		/* PHY_CAL_CLK_SELECT_0:RW8:3 */
		div = pdram_timing->mhz / (2 * 20);
		for (j = 2, tmp = 1; j <= 128; j <<= 1, tmp++) {
			if (div < j)
				break;
		}
		clrsetbits_le32(&denali_phy[947], 0x7 << 8, tmp << 8);

		if (timing_config->dram_type == DDR3) {
			mem_delay_ps = 0;
			trpre_min_ps = 1000;
		} else if (timing_config->dram_type == LPDDR4) {
			mem_delay_ps = 1500;
			trpre_min_ps = 900;
		} else if (timing_config->dram_type == LPDDR3) {
			mem_delay_ps = 2500;
			trpre_min_ps = 900;
		} else {
			printf("gen_rk3399_phy_params:dramtype unsupport\n");
			return;
		}
		total_delay_ps = mem_delay_ps + pad_delay_ps;
		delay_frac_ps = 1000 * total_delay_ps /
				(1000000 / pdram_timing->mhz);
		gate_delay_ps = delay_frac_ps + 1000 - (trpre_min_ps / 2);
		gate_delay_frac_ps = gate_delay_ps % 1000;
		tmp = gate_delay_frac_ps * 0x200 / 1000;
		/* PHY_RDDQS_GATE_SLAVE_DELAY */
		/* DENALI_PHY_77/205/333/461 10bits offset_16 */
		clrsetbits_le32(&denali_phy[77], 0x2ff << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[205], 0x2ff << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[333], 0x2ff << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[461], 0x2ff << 16, tmp << 16);

		tmp = gate_delay_ps / 1000;
		/* PHY_LP4_BOOT_RDDQS_LATENCY_ADJUST */
		/* DENALI_PHY_10/138/266/394 4bit offset_0 */
		clrsetbits_le32(&denali_phy[10], 0xf, tmp);
		clrsetbits_le32(&denali_phy[138], 0xf, tmp);
		clrsetbits_le32(&denali_phy[266], 0xf, tmp);
		clrsetbits_le32(&denali_phy[394], 0xf, tmp);
		/* PHY_GTLVL_LAT_ADJ_START */
		/* DENALI_PHY_80/208/336/464 4bits offset_16 */
		tmp = rddqs_delay_ps / (1000000 / pdram_timing->mhz) + 2;
		clrsetbits_le32(&denali_phy[80], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[208], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[336], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[464], 0xf << 16, tmp << 16);

		cas_lat = pdram_timing->cl + PI_ADD_LATENCY;
		rddata_en_ie_dly = ie_enable / (1000000 / pdram_timing->mhz);
		if ((ie_enable % (1000000 / pdram_timing->mhz)) != 0)
			rddata_en_ie_dly++;
		rddata_en_ie_dly = rddata_en_ie_dly - 1;
		tsel_adder = tsel_enable / (1000000 / pdram_timing->mhz);
		if ((tsel_enable % (1000000 / pdram_timing->mhz)) != 0)
			tsel_adder++;
		if (rddata_en_ie_dly > tsel_adder)
			extra_adder = rddata_en_ie_dly - tsel_adder;
		else
			extra_adder = 0;
		delta = cas_lat - rddata_en_ie_dly;
		if (PI_REGS_DIMM_SUPPORT && PI_DOUBLEFREEK)
			hs_offset = 2;
		else
			hs_offset = 1;
		if (rddata_en_ie_dly > (cas_lat - 1 - hs_offset))
			tmp = 0;
		else if ((delta == 2) || (delta == 1))
			tmp = rddata_en_ie_dly - 0 - extra_adder;
		else
			tmp = extra_adder;
		/* PHY_LP4_BOOT_RDDATA_EN_TSEL_DLY */
		/* DENALI_PHY_9/137/265/393 4bit offset_16 */
		clrsetbits_le32(&denali_phy[9], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[137], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[265], 0xf << 16, tmp << 16);
		clrsetbits_le32(&denali_phy[393], 0xf << 16, tmp << 16);
		/* PHY_RDDATA_EN_TSEL_DLY */
		/* DENALI_PHY_86/214/342/470 4bit offset_0 */
		clrsetbits_le32(&denali_phy[86], 0xf, tmp);
		clrsetbits_le32(&denali_phy[214], 0xf, tmp);
		clrsetbits_le32(&denali_phy[342], 0xf, tmp);
		clrsetbits_le32(&denali_phy[470], 0xf, tmp);

		if (tsel_adder > rddata_en_ie_dly)
			extra_adder = tsel_adder - rddata_en_ie_dly;
		else
			extra_adder = 0;
		if (rddata_en_ie_dly > (cas_lat - 1 - hs_offset))
			tmp = tsel_adder;
		else
			tmp = rddata_en_ie_dly - 0 + extra_adder;
		/* PHY_LP4_BOOT_RDDATA_EN_DLY */
		/* DENALI_PHY_9/137/265/393 4bit offset_8 */
		clrsetbits_le32(&denali_phy[9], 0xf << 8, tmp << 8);
		clrsetbits_le32(&denali_phy[137], 0xf << 8, tmp << 8);
		clrsetbits_le32(&denali_phy[265], 0xf << 8, tmp << 8);
		clrsetbits_le32(&denali_phy[393], 0xf << 8, tmp << 8);
		/* PHY_RDDATA_EN_DLY */
		/* DENALI_PHY_85/213/341/469 4bit offset_24 */
		clrsetbits_le32(&denali_phy[85], 0xf << 24, tmp << 24);
		clrsetbits_le32(&denali_phy[213], 0xf << 24, tmp << 24);
		clrsetbits_le32(&denali_phy[341], 0xf << 24, tmp << 24);
		clrsetbits_le32(&denali_phy[469], 0xf << 24, tmp << 24);

		if (pdram_timing->mhz <= ENPER_CS_TRAINING_FREQ) {
			/*
			 * Note:Per-CS Training is not compatible at speeds
			 * under 533 MHz. If the PHY is running at a speed
			 * less than 533MHz, all phy_per_cs_training_en_X
			 * parameters must be cleared to 0.
			 */

			/*DENALI_PHY_84/212/340/468 1bit offset_16 */
		        clrbits_le32(&denali_phy[84], 0x1 << 16);
			clrbits_le32(&denali_phy[212], 0x1 << 16);
			clrbits_le32(&denali_phy[340], 0x1 << 16);
			clrbits_le32(&denali_phy[468], 0x1 << 16);
		} else {
			setbits_le32(&denali_phy[84], 0x1 << 16);
			setbits_le32(&denali_phy[212], 0x1 << 16);
			setbits_le32(&denali_phy[340], 0x1 << 16);
			setbits_le32(&denali_phy[468], 0x1 << 16);
		}
		//		gen_rk3399_phy_dll_bypass(pdram_timing->mhz, i, fn,
		//					  timing_config->dram_type);
		phy_dll_bypass_set(chan->publ, pdram_timing->mhz, i, fn, timing_config->dram_type);
	}
}

static void gen_rk3399_enable_training(u32 *denali_ctl, uint32_t nmhz)
{
	uint32_t i, tmp;

	if (nmhz <= PHY_DLL_BYPASS_FREQ)
		tmp = 0;
	else
		tmp = 1;

	clrsetbits_le32(&denali_ctl[305], 1 << 16, tmp << 16);
	clrsetbits_le32(&denali_ctl[71], 1, tmp);
	clrsetbits_le32(&denali_ctl[70], 1 << 8, 1 << 8);
}

static int pctl_cfg(const struct chan_info *chan, u32 channel,
		    const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_ctl = chan->pctl->denali_ctl;
	u32 *denali_pi = chan->pi->denali_pi;
	u32 *denali_phy = chan->publ->denali_phy;
	const u32 *params_ctl = sdram_params->pctl_regs.denali_ctl;
	const u32 *params_phy = sdram_params->phy_regs.denali_phy;
	u32 tmp, tmp1, tmp2;
	u32 pwrup_srefresh_exit;
	int ret;
	const ulong timeout_ms = 200;
	struct dram_timing_t dram_timing;
	struct timing_related_config timing_config;

	timing_config.dram_type = DDR3;
	timing_config.dram_info[0].speed_rate = DDR3_1600K;
	timing_config.dram_info[0].cs_cnt = 1;
	timing_config.dram_info[1].speed_rate = DDR3_1600K;
	timing_config.dram_info[1].cs_cnt = 1;
	timing_config.ch_cnt = 1;
	timing_config.odt = 1;
	timing_config.freq = 800;
	timing_config.bl = 8;
	timing_config.ap = 0;
	/*
	if (mhz < 300)
		rk3399_dram_status.timing_config.dllbp = 1;
	else
		rk3399_dram_status.timing_config.dllbp = 0;
	 */
	timing_config.dllbp = 0;
	timing_config.dramds = 34;
	timing_config.dramodt = 120;
	/*	timing_config.caodt = 120;  */ /* NOT USED FOR DDR3 */
	dram_get_parameter(&timing_config, &dram_timing);

	/*
	 * work around controller bug:
	 * Do not program DRAM_CLASS until NO_PHY_IND_TRAIN_INT is programmed
	 */
	copy_to_reg(&denali_ctl[1], &params_ctl[1],
		    sizeof(struct rk3399_ddr_pctl_regs) - 4);
	writel(params_ctl[0], &denali_ctl[0]);
	copy_to_reg(denali_pi, &sdram_params->pi_regs.denali_pi[0],
		    sizeof(struct rk3399_ddr_pi_regs));
	/* rank count need to set for init */
	set_memory_map(chan, channel, sdram_params);

	gen_rk3399_ctl_params_f0(chan, &timing_config, &dram_timing);
	gen_rk3399_ctl_params_f1(chan, &timing_config, &dram_timing);
	{
		u32 tmp;
		struct dram_timing_t *pdram_timing = &dram_timing;

		/* PI_02 PI_TDFI_PHYMSTR_MAX_F0:RW:0:32 */
		tmp = 4 * pdram_timing->trefi;
		writel(tmp, &denali_pi[2]);
		/* PI_03 PI_TDFI_PHYMSTR_RESP_F0:RW:0:16 */
		tmp = 2 * pdram_timing->trefi;
		clrsetbits_le32(&denali_pi[2], 0xffff, tmp);
		/* PI_07 PI_TDFI_PHYUPD_RESP_F0:RW:16:16 */
		clrsetbits_le32(&denali_pi[7], 0xffff << 16, tmp << 16);

		tmp = 0;
		tmp = (dram_timing.bl / 2) + 4 +
			(get_pi_rdlat_adj(pdram_timing) - 2) + tmp +
			get_pi_tdfi_phy_rdlat(pdram_timing, &timing_config);
		clrsetbits_le32(&denali_pi[42], 0xff, tmp);
		clrsetbits_le32(&denali_pi[43], 0x7f << 16, (pdram_timing->cl * 2) << 16);

		/* PI_46 PI_TREF_F0:RW:16:16 */
		clrsetbits_le32(&denali_pi[46], 0xffff << 16,
				pdram_timing->trefi << 16);
		/* PI_46 PI_TRFC_F0:RW:0:10 */
		clrsetbits_le32(&denali_pi[46], 0x3ff, pdram_timing->trfc);

		/* PI_72 PI_WR_TO_ODTH_F0:RW:16:6 */
		/*		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			tmp1 = get_pi_wrlat(pdram_timing, timing_config);
			tmp2 = get_pi_todtoff_max(pdram_timing, timing_config);
			if (tmp1 > tmp2)
				tmp = tmp1 - tmp2;
			else
				tmp = 0;
		} else if (timing_config->dram_type == DDR3) {
			tmp = 0;
			}*/
		tmp = 0;
		clrsetbits_le32(&denali_pi[72], 0x3f << 16, tmp << 16);
		/* PI_73 PI_RD_TO_ODTH_F0:RW:8:6 */
		/*
		if ((timing_config->dram_type == LPDDR3) ||
		    (timing_config->dram_type == LPDDR4)) {
			/* min_rl_preamble = cl + TDQSCK_MIN - 1 * /
			tmp1 = pdram_timing->cl;
			tmp1 += get_pi_todtoff_min(pdram_timing, timing_config);
			tmp1--;
			/* todtoff_max * /
			tmp2 = get_pi_todtoff_max(pdram_timing, timing_config);
			if (tmp1 > tmp2)
				tmp = tmp1 - tmp2;
			else
				tmp = 0;
		} else if (timing_config->dram_type == DDR3) {
			tmp = pdram_timing->cl - pdram_timing->cwl;
		}
	*/
		tmp = pdram_timing->cl - pdram_timing->cwl;
		clrsetbits_le32(&denali_pi[73], 0x3f << 8, tmp << 8);
		/* PI_89 PI_RDLAT_ADJ_F0:RW:16:8 */
		tmp = get_pi_rdlat_adj(pdram_timing);
		clrsetbits_le32(&denali_pi[89], 0xff << 16, tmp << 16);
		/* PI_90 PI_WRLAT_ADJ_F0:RW:16:8 */
		tmp = get_pi_wrlat_adj(pdram_timing, &timing_config);
		clrsetbits_le32(&denali_pi[90], 0xff << 16, tmp << 16);
		/* PI_125 PI_MR1_DATA_F0_0:RW+:8:16 */
		clrsetbits_le32(&denali_pi[125], 0xffff << 8,
				pdram_timing->mr[1] << 8);
		/* PI_133 PI_MR1_DATA_F0_1:RW+:0:16 */
		clrsetbits_le32(&denali_pi[133], 0xffff, pdram_timing->mr[1]);
#if 0
		/* PI_140 PI_MR1_DATA_F0_2:RW+:16:16 */
		clrsetbits_le32(&denali_pi[140], 0xffff << 16,
				   pdram_timing->mr[1] << 16);
		/* PI_148 PI_MR1_DATA_F0_3:RW+:0:16 */
		clrsetbits_le32(&denali_pi[148], 0xffff, pdram_timing->mr[1]);
#endif
		/* PI_126 PI_MR2_DATA_F0_0:RW+:0:16 */
		clrsetbits_le32(&denali_pi[126], 0xffff, pdram_timing->mr[2]);
		/* PI_133 PI_MR2_DATA_F0_1:RW+:16:16 */
		clrsetbits_le32(&denali_pi[133], 0xffff << 16,
				   pdram_timing->mr[2] << 16);
#if 0
		/* PI_141 PI_MR2_DATA_F0_2:RW+:0:16 */
		clrsetbits_le32(&denali_pi[141], 0xffff, pdram_timing->mr[2]);
		/* PI_148 PI_MR2_DATA_F0_3:RW+:16:16 */
		clrsetbits_le32(&denali_pi[148], 0xffff << 16,
				   pdram_timing->mr[2] << 16);
#endif
		/* PI_156 PI_TFC_F0:RW:0:10 */
		clrsetbits_le32(&denali_pi[156], 0x3ff,
				   pdram_timing->tfc_long);
		/* PI_158 PI_TWR_F0:RW:24:6 */
		clrsetbits_le32(&denali_pi[158], 0x3f << 24,
				   pdram_timing->twr << 24);

		/* PI_158 PI_TWTR_F0:RW:16:6 */
		clrsetbits_le32(&denali_pi[158], 0x3f << 16,
				   pdram_timing->twtr << 16);
		/* PI_158 PI_TRCD_F0:RW:8:8 */
		clrsetbits_le32(&denali_pi[158], 0xff << 8,
				pdram_timing->trcd << 8);
		/* PI_158 PI_TRP_F0:RW:0:8 */
		clrsetbits_le32(&denali_pi[158], 0xff, pdram_timing->trp);
		/* PI_157 PI_TRTP_F0:RW:24:8 */
		clrsetbits_le32(&denali_pi[157], 0xff << 24,
				pdram_timing->trtp << 24);
		/* PI_159 PI_TRAS_MIN_F0:RW:24:8 */
		clrsetbits_le32(&denali_pi[159], 0xff << 24,
				pdram_timing->tras_min << 24);
		/* PI_159 PI_TRAS_MAX_F0:RW:0:17 */
		tmp = pdram_timing->tras_max * 99 / 100;
		clrsetbits_le32(&denali_pi[159], 0x1ffff, tmp);
		/* PI_160 PI_TMRD_F0:RW:16:6 */
		clrsetbits_le32(&denali_pi[160], 0x3f << 16,
				pdram_timing->tmrd << 16);
		/*PI_160 PI_TDQSCK_MAX_F0:RW:0:4 */
		clrsetbits_le32(&denali_pi[160], 0xf,
				pdram_timing->tdqsck_max);
		/* PI_187 PI_TDFI_CTRLUPD_MAX_F0:RW:8:16 */
		clrsetbits_le32(&denali_pi[187], 0xffff << 8,
				(2 * pdram_timing->trefi) << 8);
		/* PI_188 PI_TDFI_CTRLUPD_INTERVAL_F0:RW:0:32 */
		clrsetbits_le32(&denali_pi[188], 0xffffffff,
				20 * pdram_timing->trefi);
	}

	writel(sdram_params->phy_regs.denali_phy[910], &denali_phy[910]);
	writel(sdram_params->phy_regs.denali_phy[911], &denali_phy[911]);
	writel(sdram_params->phy_regs.denali_phy[912], &denali_phy[912]);

	pwrup_srefresh_exit = readl(&denali_ctl[68]) & PWRUP_SREFRESH_EXIT;
	clrbits_le32(&denali_ctl[68], PWRUP_SREFRESH_EXIT);

	/* PHY_DLL_RST_EN */
	clrsetbits_le32(&denali_phy[957], 0x3 << 24, 1 << 24);

	setbits_le32(&denali_pi[0], START);
	setbits_le32(&denali_ctl[0], START);

	/* Wating for phy DLL lock */
	while (1) {
		tmp = readl(&denali_phy[920]);
		tmp1 = readl(&denali_phy[921]);
		tmp2 = readl(&denali_phy[922]);
		if ((((tmp >> 16) & 0x1) == 0x1) &&
		    (((tmp1 >> 16) & 0x1) == 0x1) &&
		    (((tmp1 >> 0) & 0x1) == 0x1) &&
		    (((tmp2 >> 0) & 0x1) == 0x1))
			break;
	}

	copy_to_reg(&denali_phy[896], &params_phy[896], (958 - 895) * 4);
	copy_to_reg(&denali_phy[0], &params_phy[0], (90 - 0 + 1) * 4);
	copy_to_reg(&denali_phy[128], &params_phy[128], (218 - 128 + 1) * 4);
	copy_to_reg(&denali_phy[256], &params_phy[256], (346 - 256 + 1) * 4);
	copy_to_reg(&denali_phy[384], &params_phy[384], (474 - 384 + 1) * 4);
	copy_to_reg(&denali_phy[512], &params_phy[512], (549 - 512 + 1) * 4);
	copy_to_reg(&denali_phy[640], &params_phy[640], (677 - 640 + 1) * 4);
	copy_to_reg(&denali_phy[768], &params_phy[768], (805 - 768 + 1) * 4);
	set_ds_odt(chan, sdram_params);

#if 0
	{
	uint32_t tmp, i, div, j;
	uint32_t mem_delay_ps, pad_delay_ps, total_delay_ps, delay_frac_ps;
	uint32_t trpre_min_ps, gate_delay_ps, gate_delay_frac_ps;
	uint32_t ie_enable, tsel_enable, cas_lat, rddata_en_ie_dly, tsel_adder;
	uint32_t extra_adder, delta, hs_offset;
	int fn = 0;
		struct dram_timing_t *pdram_timing = &dram_timing;

	pad_delay_ps = PI_PAD_DELAY_PS_VALUE;
	ie_enable = PI_IE_ENABLE_VALUE;
	tsel_enable = PI_TSEL_ENABLE_VALUE;

	clrsetbits_le32(&denali_phy[896], (0x3 << 8) | 1, fn << 8);
	/* PHY_LOW_FREQ_SEL */
	/* DENALI_PHY_913 1bit offset_0 */
	if (timing_config.freq > 400)
		clrbits_le32(&denali_phy[913], 1);
	else
		setbits_le32(&denali_phy[913], 1);

	/* PHY_RPTR_UPDATE_x */
	/* DENALI_PHY_87/215/343/471 4bit offset_16 */
	tmp = 2500 / (1000000 / pdram_timing->mhz) + 3;
	if ((2500 % (1000000 / pdram_timing->mhz)) != 0)
		tmp++;
	clrsetbits_le32(&denali_phy[87], 0xf << 16, tmp << 16);
	clrsetbits_le32(&denali_phy[215], 0xf << 16, tmp << 16);
	clrsetbits_le32(&denali_phy[343], 0xf << 16, tmp << 16);
	clrsetbits_le32(&denali_phy[471], 0xf << 16, tmp << 16);

	}
#endif

#if 1
	/*
	 * phy_dqs_tsel_wr_timing_X 8bits DENALI_PHY_84/212/340/468 offset_8
	 * dqs_tsel_wr_end[7:4] add Half cycle
	 */
	tmp = (readl(&denali_phy[84]) >> 8) & 0xff;
	clrsetbits_le32(&denali_phy[84], 0xff << 8, (tmp + 0x10) << 8);
	tmp = (readl(&denali_phy[212]) >> 8) & 0xff;
	clrsetbits_le32(&denali_phy[212], 0xff << 8, (tmp + 0x10) << 8);
	tmp = (readl(&denali_phy[340]) >> 8) & 0xff;
	clrsetbits_le32(&denali_phy[340], 0xff << 8, (tmp + 0x10) << 8);
	tmp = (readl(&denali_phy[468]) >> 8) & 0xff;
	clrsetbits_le32(&denali_phy[468], 0xff << 8, (tmp + 0x10) << 8);

	/*
	 * phy_dqs_tsel_wr_timing_X 8bits DENALI_PHY_83/211/339/467 offset_8
	 * dq_tsel_wr_end[7:4] add Half cycle
	 */
	tmp = (readl(&denali_phy[83]) >> 16) & 0xff;
	clrsetbits_le32(&denali_phy[83], 0xff << 16, (tmp + 0x10) << 16);
	tmp = (readl(&denali_phy[211]) >> 16) & 0xff;
	clrsetbits_le32(&denali_phy[211], 0xff << 16, (tmp + 0x10) << 16);
	tmp = (readl(&denali_phy[339]) >> 16) & 0xff;
	clrsetbits_le32(&denali_phy[339], 0xff << 16, (tmp + 0x10) << 16);
	tmp = (readl(&denali_phy[467]) >> 16) & 0xff;
	clrsetbits_le32(&denali_phy[467], 0xff << 16, (tmp + 0x10) << 16);
#endif

	ret = phy_io_config(chan, sdram_params);

	gen_rk3399_phy_params(chan, &timing_config, &dram_timing, 0);
	gen_rk3399_phy_params(chan, &timing_config, &dram_timing, 1);

	if (ret)
		return ret;

#if defined(DEBUG)
	{
		u32 trefi0 = (readl(&denali_ctl[48]) >> 16) + 8;
		u32 trefi1 = (readl(&denali_ctl[49]) >> 16) + 8;

		printf("trefi0     %08x\n", trefi0);
		printf("trefi1     %08x\n", trefi1);
		printf("tinit1+2   %08x (txsnr, 3*mrd, tmod, tzqinit)) ... 700us\n",
		       readl(&denali_ctl[5]));
		printf("tinit3     %08x\n", readl(&denali_ctl[6]));
		printf("tinit5     %08x\n", readl(&denali_ctl[8]));
		printf("tCL * 2    %08x (%d)\n",
		       ((readl(&denali_ctl[23]) >> 16) & 0x7f),
		       ((readl(&denali_ctl[23]) >> 16) & 0x7f));
		printf("timing.CL %d CWL %d\n", dram_timing.cl, dram_timing.cwl);
		printf("tCWL       %d\n", ((readl(&denali_ctl[23]) >> 24) & 0x1f));
		printf("tAL        %d\n", (readl(&denali_ctl[24]) & 0x3f));
		printf("tRC        %d\n", (readl(&denali_ctl[26]) >> 24) & 0xff);
		printf("tRRD       %d\n", (readl(&denali_ctl[26]) >> 16) & 0xff);
		printf("tMOD       %d\n", (readl(&denali_ctl[32]) >> 8) & 0xff);
		printf("tMRD       %d\n", (readl(&denali_ctl[32]) >> 0) & 0xff);
		printf("MR[0]      %04x (tCL, tWR)\n",
		       (readl(&denali_ctl[133]) >> 0) & 0xffff);
		printf("MR[1]      %04x\n", (readl(&denali_ctl[133]) >> 16) & 0xffff);
		printf("MR[2]      %04x (tCWL)\n",
		       (readl(&denali_ctl[134]) >> 0) & 0xffff);
		printf("MR[3]      %04x\n", (readl(&denali_ctl[138]) >> 0) & 0xffff);
		//		printf("
		printf("CTL284: wrlat_adj %d rdlat_adj %d (depends on CWL, CL)\n",
		       (readl(&denali_ctl[284]) >> 8) & 0xff,
		       (readl(&denali_ctl[284]) >> 0) & 0xff);

	}
#endif

	/* PHY_DLL_RST_EN */
	clrsetbits_le32(&denali_phy[957], 0x3 << 24, 0x2 << 24);

	gen_rk3399_enable_training(denali_ctl, timing_config.freq);

	/* Wating for PHY and DRAM init complete */
	tmp = get_timer(0);
	do {
		if (get_timer(tmp) > timeout_ms) {
			pr_err("DRAM (%s): phy failed to lock within  %ld ms\n",
			      __func__, timeout_ms);
			return -ETIME;
		}
	} while (!(readl(&denali_ctl[203]) & (1 << 3)));
	debug("DRAM (%s): phy locked after %ld ms\n", __func__, get_timer(tmp));

	clrsetbits_le32(&denali_ctl[68], PWRUP_SREFRESH_EXIT,
			pwrup_srefresh_exit);

#if 0
	printf ("/* PCTL */\n");
	for (int i = 0; i < sizeof(struct rk3399_ddr_pctl_regs) / 4; ++i)
	  printf("\t\t0x%08x\n", readl(&denali_ctl[i]));

	printf ("/* PI */\n");
	for (int i = 0; i < sizeof(struct rk3399_ddr_pi_regs) / 4; ++i)
	  printf("\t\t0x%08x\n", readl(&denali_pi[i]));

	printf ("/* PHY */\n");
	for (int i = 0; i < 958; ++i)
	  printf("\t\t0x%08x\n", readl(&denali_phy[i]));
#endif

	return 0;
}

static void select_per_cs_training_index(const struct chan_info *chan,
					 u32 rank)
{
	u32 *denali_phy = chan->publ->denali_phy;

	/* PHY_84 PHY_PER_CS_TRAINING_EN_0 1bit offset_16 */
	if ((readl(&denali_phy[84])>>16) & 1) {
		/*
		 * PHY_8/136/264/392
		 * phy_per_cs_training_index_X 1bit offset_24
		 */
		clrsetbits_le32(&denali_phy[8], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[136], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[264], 0x1 << 24, rank << 24);
		clrsetbits_le32(&denali_phy[392], 0x1 << 24, rank << 24);
	}
}

static void override_write_leveling_value(const struct chan_info *chan)
{
	u32 *denali_ctl = chan->pctl->denali_ctl;
	u32 *denali_phy = chan->publ->denali_phy;
	u32 byte;

	/* PHY_896 PHY_FREQ_SEL_MULTICAST_EN 1bit offset_0 */
	setbits_le32(&denali_phy[896], 1);

	/*
	 * PHY_8/136/264/392
	 * phy_per_cs_training_multicast_en_X 1bit offset_16
	 */
	clrsetbits_le32(&denali_phy[8], 0x1 << 16, 1 << 16);
	clrsetbits_le32(&denali_phy[136], 0x1 << 16, 1 << 16);
	clrsetbits_le32(&denali_phy[264], 0x1 << 16, 1 << 16);
	clrsetbits_le32(&denali_phy[392], 0x1 << 16, 1 << 16);

	for (byte = 0; byte < 4; byte++)
		clrsetbits_le32(&denali_phy[63 + (128 * byte)], 0xffff << 16,
				0x200 << 16);

	/* PHY_896 PHY_FREQ_SEL_MULTICAST_EN 1bit offset_0 */
	clrbits_le32(&denali_phy[896], 1);

	/* CTL_200 ctrlupd_req 1bit offset_8 */
	clrsetbits_le32(&denali_ctl[200], 0x1 << 8, 0x1 << 8);
}

static int data_training_ca(const struct chan_info *chan, u32 channel,
			    const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = chan->pi->denali_pi;
	u32 *denali_phy = chan->publ->denali_phy;
	u32 i, tmp;
	u32 obs_0, obs_1, obs_2, obs_err = 0;
	u32 rank = sdram_params->ch[channel].rank;

	for (i = 0; i < rank; i++) {
		select_per_cs_training_index(chan, i);
		/* PI_100 PI_CALVL_EN:RW:8:2 */
		clrsetbits_le32(&denali_pi[100], 0x3 << 8, 0x2 << 8);
		/* PI_92 PI_CALVL_REQ:WR:16:1,PI_CALVL_CS:RW:24:2 */
		clrsetbits_le32(&denali_pi[92],
				(0x1 << 16) | (0x3 << 24),
				(0x1 << 16) | (i << 24));

		/* Waiting for training complete */
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:18 */
			tmp = readl(&denali_pi[174]) >> 8;
			/*
			 * check status obs
			 * PHY_532/660/789 phy_adr_calvl_obs1_:0:32
			 */
			obs_0 = readl(&denali_phy[532]);
			obs_1 = readl(&denali_phy[660]);
			obs_2 = readl(&denali_phy[788]);
			if (((obs_0 >> 30) & 0x3) ||
			    ((obs_1 >> 30) & 0x3) ||
			    ((obs_2 >> 30) & 0x3))
				obs_err = 1;
			if ((((tmp >> 11) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 5) & 0x1) == 0x0) &&
			    (obs_err == 0))
				break;
			else if ((((tmp >> 5) & 0x1) == 0x1) ||
				 (obs_err == 1))
				return -EIO;
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		writel(0x00003f7c, (&denali_pi[175]));
	}
	clrbits_le32(&denali_pi[100], 0x3 << 8);

	return 0;
}

static int data_training_wl(const struct chan_info *chan, u32 channel,
			    const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = chan->pi->denali_pi;
	u32 *denali_phy = chan->publ->denali_phy;
	u32 i, tmp;
	u32 obs_0, obs_1, obs_2, obs_3, obs_err = 0;
	u32 rank = sdram_params->ch[channel].rank;

	for (i = 0; i < rank; i++) {
		/* PI_60 PI_WRLVL_EN:RW:8:2 */
		clrsetbits_le32(&denali_pi[59], 0xff << 24, 0xff << 24);
		debug("denali_pi[59] %08x\n", readl(&denali_pi[59]));
		debug("denali_pi[60] %08x\n", readl(&denali_pi[60]));
		clrsetbits_le32(&denali_pi[60], 0x3 << 8, 0x1f << 8);
		/* PI_59 PI_WRLVL_REQ:WR:8:1,PI_WRLVL_CS:RW:16:2 */
		clrsetbits_le32(&denali_pi[59],
				(0x1 << 8) | (0x3 << 16),
				(0x1 << 8) | (i << 16));

		select_per_cs_training_index(chan, i);

		/* Waiting for training complete */
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:18 */
			tmp = readl(&denali_pi[174]) >> 8;

			/*
			 * check status obs, if error maybe can not
			 * get leveling done PHY_40/168/296/424
			 * phy_wrlvl_status_obs_X:0:13
			 */
			obs_0 = readl(&denali_phy[40]);
			obs_1 = readl(&denali_phy[168]);
			obs_2 = readl(&denali_phy[296]);
			obs_3 = readl(&denali_phy[424]);
			if (((obs_0 >> 12) & 0x1) ||
			    ((obs_1 >> 12) & 0x1) ||
			    ((obs_2 >> 12) & 0x1) ||
			    ((obs_3 >> 12) & 0x1))
				obs_err = 1;
			if ((((tmp >> 10) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 4) & 0x1) == 0x0) &&
			    (obs_err == 0))
				break;
			else if ((((tmp >> 4) & 0x1) == 0x1) ||
				 (obs_err == 1)) {
				printf("%s: -EIO\n", __func__);
				return -EIO;
			}
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		writel(0x00003f7c, (&denali_pi[175]));
	}

	override_write_leveling_value(chan);
	clrbits_le32(&denali_pi[60], 0x3 << 8);

	return 0;
}

static int data_training_rg(const struct chan_info *chan, u32 channel,
			    const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = chan->pi->denali_pi;
	u32 *denali_phy = chan->publ->denali_phy;
	u32 i, tmp;
	u32 obs_0, obs_1, obs_2, obs_3, obs_err = 0;
	u32 rank = sdram_params->ch[channel].rank;

	for (i = 0; i < rank; i++) {
		select_per_cs_training_index(chan, i);
		/* PI_80 PI_RDLVL_GATE_EN:RW:24:2 */
		clrsetbits_le32(&denali_pi[80], 0x3 << 24, 0x2 << 24);
		/*
		 * PI_74 PI_RDLVL_GATE_REQ:WR:16:1
		 * PI_RDLVL_CS:RW:24:2
		 */
		clrsetbits_le32(&denali_pi[74],
				(0x1 << 16) | (0x3 << 24),
				(0x1 << 16) | (i << 24));

		/* Waiting for training complete */
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:18 */
			tmp = readl(&denali_pi[174]) >> 8;

			/*
			 * check status obs
			 * PHY_43/171/299/427
			 *     PHY_GTLVL_STATUS_OBS_x:16:8
			 */
			obs_0 = readl(&denali_phy[43]);
			obs_1 = readl(&denali_phy[171]);
			obs_2 = readl(&denali_phy[299]);
			obs_3 = readl(&denali_phy[427]);
			if (((obs_0 >> (16 + 6)) & 0x3) ||
			    ((obs_1 >> (16 + 6)) & 0x3) ||
			    ((obs_2 >> (16 + 6)) & 0x3) ||
			    ((obs_3 >> (16 + 6)) & 0x3))
				obs_err = 1;
			if ((((tmp >> 9) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 3) & 0x1) == 0x0) &&
			    (obs_err == 0))
				break;
			else if ((((tmp >> 3) & 0x1) == 0x1) ||
				 (obs_err == 1))
				return -EIO;
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		writel(0x00003f7c, (&denali_pi[175]));
	}
	clrbits_le32(&denali_pi[80], 0x3 << 24);

	return 0;
}

static int data_training_rl(const struct chan_info *chan, u32 channel,
			    const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = chan->pi->denali_pi;
	u32 i, tmp;
	u32 rank = sdram_params->ch[channel].rank;

	for (i = 0; i < rank; i++) {
		select_per_cs_training_index(chan, i);
		/* PI_80 PI_RDLVL_EN:RW:16:2 */
		clrsetbits_le32(&denali_pi[80], 0x3 << 16, 0x2 << 16);
		/* PI_74 PI_RDLVL_REQ:WR:8:1,PI_RDLVL_CS:RW:24:2 */
		clrsetbits_le32(&denali_pi[74],
				(0x1 << 8) | (0x3 << 24),
				(0x1 << 8) | (i << 24));

		/* Waiting for training complete */
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:18 */
			tmp = readl(&denali_pi[174]) >> 8;

			/*
			 * make sure status obs not report error bit
			 * PHY_46/174/302/430
			 *     phy_rdlvl_status_obs_X:16:8
			 */
			if ((((tmp >> 8) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 2) & 0x1) == 0x0))
				break;
			else if (((tmp >> 2) & 0x1) == 0x1)
				return -EIO;
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		writel(0x00003f7c, (&denali_pi[175]));
	}
	clrbits_le32(&denali_pi[80], 0x3 << 16);

	return 0;
}

static int data_training_wdql(const struct chan_info *chan, u32 channel,
			      const struct rk3399_sdram_params *sdram_params)
{
	u32 *denali_pi = chan->pi->denali_pi;
	u32 i, tmp;
	u32 rank = sdram_params->ch[channel].rank;

	for (i = 0; i < rank; i++) {
		select_per_cs_training_index(chan, i);
		/*
		 * disable PI_WDQLVL_VREF_EN before wdq leveling?
		 * PI_181 PI_WDQLVL_VREF_EN:RW:8:1
		 */
		clrbits_le32(&denali_pi[181], 0x1 << 8);
		/* PI_124 PI_WDQLVL_EN:RW:16:2 */
		clrsetbits_le32(&denali_pi[124], 0x3 << 16, 0x2 << 16);
		/* PI_121 PI_WDQLVL_REQ:WR:8:1,PI_WDQLVL_CS:RW:16:2 */
		clrsetbits_le32(&denali_pi[121],
				(0x1 << 8) | (0x3 << 16),
				(0x1 << 8) | (i << 16));

		/* Waiting for training complete */
		while (1) {
			/* PI_174 PI_INT_STATUS:RD:8:18 */
			tmp = readl(&denali_pi[174]) >> 8;
			if ((((tmp >> 12) & 0x1) == 0x1) &&
			    (((tmp >> 13) & 0x1) == 0x1) &&
			    (((tmp >> 6) & 0x1) == 0x0))
				break;
			else if (((tmp >> 6) & 0x1) == 0x1)
				return -EIO;
		}
		/* clear interrupt,PI_175 PI_INT_ACK:WR:0:17 */
		writel(0x00003f7c, (&denali_pi[175]));
	}
	clrbits_le32(&denali_pi[124], 0x3 << 16);

	return 0;
}

static int data_training(const struct chan_info *chan, u32 channel,
			 const struct rk3399_sdram_params *sdram_params,
			 u32 training_flag)
{
	u32 *denali_phy = chan->publ->denali_phy;

	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	setbits_le32(&denali_phy[927], (1 << 22));

	if (training_flag == PI_FULL_TRAINING) {
		if (sdram_params->base.dramtype == LPDDR4) {
			training_flag = PI_CA_TRAINING | PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING | PI_WDQ_LEVELING;
		} else if (sdram_params->base.dramtype == LPDDR3) {
			training_flag = PI_CA_TRAINING | PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING;
		} else if (sdram_params->base.dramtype == DDR3) {
			training_flag = PI_WRITE_LEVELING |
					PI_READ_GATE_TRAINING |
					PI_READ_LEVELING;
		}
	}

	/* ca training(LPDDR4,LPDDR3 support) */
	if ((training_flag & PI_CA_TRAINING) == PI_CA_TRAINING)
		data_training_ca(chan, channel, sdram_params);

	/* write leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_WRITE_LEVELING) == PI_WRITE_LEVELING)
		data_training_wl(chan, channel, sdram_params);

	/* read gate training(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_GATE_TRAINING) == PI_READ_GATE_TRAINING)
		data_training_rg(chan, channel, sdram_params);

	/* read leveling(LPDDR4,LPDDR3,DDR3 support) */
	if ((training_flag & PI_READ_LEVELING) == PI_READ_LEVELING)
		data_training_rl(chan, channel, sdram_params);

	/* wdq leveling(LPDDR4 support) */
	if ((training_flag & PI_WDQ_LEVELING) == PI_WDQ_LEVELING)
		data_training_wdql(chan, channel, sdram_params);

	/* PHY_927 PHY_PAD_DQS_DRIVE  RPULL offset_22 */
	clrbits_le32(&denali_phy[927], (1 << 22));

	return 0;
}

static void set_ddrconfig(const struct chan_info *chan,
			  const struct rk3399_sdram_params *sdram_params,
			  unsigned char channel, u32 ddrconfig)
{
	/* only need to set ddrconfig */
	struct rk3399_msch_regs *ddr_msch_regs = chan->msch;
	unsigned int cs0_cap = 0;
	unsigned int cs1_cap = 0;

	cs0_cap = (1 << (sdram_params->ch[channel].cs0_row
			+ sdram_params->ch[channel].col
			+ sdram_params->ch[channel].bk
			+ sdram_params->ch[channel].bw - 20));
	if (sdram_params->ch[channel].rank > 1)
		cs1_cap = cs0_cap >> (sdram_params->ch[channel].cs0_row
				- sdram_params->ch[channel].cs1_row);
	if (sdram_params->ch[channel].row_3_4) {
		cs0_cap = cs0_cap * 3 / 4;
		cs1_cap = cs1_cap * 3 / 4;
	}

	writel(ddrconfig | (ddrconfig << 8), &ddr_msch_regs->ddrconf);
	writel(((cs0_cap / 32) & 0xff) | (((cs1_cap / 32) & 0xff) << 8),
	       &ddr_msch_regs->ddrsize);
}

static void dram_all_config(struct dram_info *dram,
			    const struct rk3399_sdram_params *sdram_params)
{
	u32 sys_reg = 0;
	unsigned int channel, idx;

	sys_reg |= sdram_params->base.dramtype << SYS_REG_DDRTYPE_SHIFT;
	sys_reg |= (sdram_params->base.num_channels - 1)
		    << SYS_REG_NUM_CH_SHIFT;
	for (channel = 0, idx = 0;
	     (idx < sdram_params->base.num_channels) && (channel < 2);
	     channel++) {
		const struct rk3399_sdram_channel *info =
			&sdram_params->ch[channel];
		struct rk3399_msch_regs *ddr_msch_regs;
		const struct rk3399_msch_timings *noc_timing;

		if (sdram_params->ch[channel].col == 0)
			continue;
		idx++;
		sys_reg |= info->row_3_4 << SYS_REG_ROW_3_4_SHIFT(channel);
		sys_reg |= 1 << SYS_REG_CHINFO_SHIFT(channel);
		sys_reg |= (info->rank - 1) << SYS_REG_RANK_SHIFT(channel);
		sys_reg |= (info->col - 9) << SYS_REG_COL_SHIFT(channel);
		sys_reg |= info->bk == 3 ? 0 : 1 << SYS_REG_BK_SHIFT(channel);
		sys_reg |= (info->cs0_row - 13) << SYS_REG_CS0_ROW_SHIFT(channel);
		sys_reg |= (info->cs1_row - 13) << SYS_REG_CS1_ROW_SHIFT(channel);
		sys_reg |= (2 >> info->bw) << SYS_REG_BW_SHIFT(channel);
		sys_reg |= (2 >> info->dbw) << SYS_REG_DBW_SHIFT(channel);

		ddr_msch_regs = dram->chan[channel].msch;
		noc_timing = &sdram_params->ch[channel].noc_timings;
		writel(noc_timing->ddrtiminga0,
		       &ddr_msch_regs->ddrtiminga0);
		writel(noc_timing->ddrtimingb0,
		       &ddr_msch_regs->ddrtimingb0);
		writel(noc_timing->ddrtimingc0,
		       &ddr_msch_regs->ddrtimingc0);
		writel(noc_timing->devtodev0,
		       &ddr_msch_regs->devtodev0);
		writel(noc_timing->ddrmode,
		       &ddr_msch_regs->ddrmode);

		/* rank 1 memory clock disable (dfi_dram_clk_disable = 1) */
		if (sdram_params->ch[channel].rank == 1)
			setbits_le32(&dram->chan[channel].pctl->denali_ctl[276],
				     1 << 17);
	}

	writel(sys_reg, &dram->pmugrf->os_reg2);
	rk_clrsetreg(&dram->pmusgrf->soc_con4, 0x1f << 10,
		     sdram_params->base.stride << 10);

	/* reboot hold register set */
	writel(PRESET_SGRF_HOLD(0) | PRESET_GPIO0_HOLD(1) |
		PRESET_GPIO1_HOLD(1),
		&dram->pmucru->pmucru_rstnhold_con[1]);
	clrsetbits_le32(&dram->cru->glb_rst_con, 0x3, 0x3);
}

static int switch_to_phy_index(struct dram_info *dram,
			       const struct rk3399_sdram_params *sdram_params,
			       u32 index)
{
	u32 channel;
	u32 *denali_phy;
	u32 ch_count = sdram_params->base.num_channels;
	int ret;
	int i = 0;

	writel(RK_CLRSETBITS(0x03 << 4 | 1 << 2 | 1,
			     (index & 0x3) << 4 | 1 << 2 | 1),
			&dram->cic->cic_ctrl0);
	while (!(readl(&dram->cic->cic_status0) & (1 << 2))) {
		mdelay(10);
		i++;
		if (i > 10) {
			debug("index1 frequency change overtime\n");
			return -ETIME;
		}
	}

	i = 0;
	writel(RK_CLRSETBITS(1 << 1, 1 << 1), &dram->cic->cic_ctrl0);
	while (!(readl(&dram->cic->cic_status0) & (1 << 0))) {
		mdelay(10);
		i++;
		if (i > 10) {
			debug("index1 frequency done overtime\n");
			return -ETIME;
		}
	}

	for (channel = 0; channel < ch_count; channel++) {
		denali_phy = dram->chan[channel].publ->denali_phy;
		clrsetbits_le32(&denali_phy[896], (0x3 << 8) | 1, (index & 3) << 8);
		debug("data training\n");
		ret = data_training(&dram->chan[channel], channel,
				    sdram_params, PI_FULL_TRAINING);
		if (ret) {
			debug("index1 training failed\n");
			return ret;
		}
	}

	return 0;
}

static int sdram_init(struct dram_info *dram,
		      const struct rk3399_sdram_params *sdram_params)
{
	unsigned char dramtype = sdram_params->base.dramtype;
	unsigned int ddr_freq = sdram_params->base.ddr_freq;
	int channel;

	debug("Starting SDRAM initialization...\n");

	if ((dramtype == DDR3 && ddr_freq > 933) ||
	    (dramtype == LPDDR3 && ddr_freq > 933) ||
	    (dramtype == LPDDR4 && ddr_freq > 800)) {
		debug("SDRAM frequency is to high!");
		return -E2BIG;
	}

	for (channel = 0; channel < 2; channel++) {
		const struct chan_info *chan = &dram->chan[channel];
		struct rk3399_ddr_publ_regs *publ = chan->publ;

		phy_dll_bypass_set(publ, ddr_freq, channel, 0, dramtype);
		phy_dll_bypass_set(publ, ddr_freq, channel, 1, dramtype);

		if (channel >= sdram_params->base.num_channels)
			continue;

		if (pctl_cfg(chan, channel, sdram_params) != 0) {
			printf("pctl_cfg fail, reset\n");
			return -EIO;
		}

		/* LPDDR2/LPDDR3 need to wait DAI complete, max 10us */
		if (dramtype == LPDDR3)
			udelay(10);

		if (data_training(chan, channel,
				  sdram_params, PI_FULL_TRAINING)) {
			printf("SDRAM initialization failed, reset\n");
			return -EIO;
		}

		set_ddrconfig(chan, sdram_params, channel,
			      sdram_params->ch[channel].ddrconfig);
	}
	dram_all_config(dram, sdram_params);
	switch_to_phy_index(dram, sdram_params, 1);

	debug("Finish SDRAM initialization...\n");
	return 0;
}

static int rk3399_dmc_ofdata_to_platdata(struct udevice *dev)
{
#if !CONFIG_IS_ENABLED(OF_PLATDATA)
	struct rockchip_dmc_plat *plat = dev_get_platdata(dev);
	int ret;

	ret = dev_read_u32_array(dev, "rockchip,sdram-params",
				 (u32 *)&plat->sdram_params,
				 sizeof(plat->sdram_params) / sizeof(u32));
	if (ret) {
		printf("%s: Cannot read rockchip,sdram-params %d\n",
		       __func__, ret);
		return ret;
	}
	ret = regmap_init_mem(dev_ofnode(dev), &plat->map);
	if (ret)
		printf("%s: regmap failed %d\n", __func__, ret);

#endif
	return 0;
}

#if CONFIG_IS_ENABLED(OF_PLATDATA)
static int conv_of_platdata(struct udevice *dev)
{
	struct rockchip_dmc_plat *plat = dev_get_platdata(dev);
	struct dtd_rockchip_rk3399_dmc *dtplat = &plat->dtplat;
	int ret;

	ret = regmap_init_mem_platdata(dev, dtplat->reg,
			ARRAY_SIZE(dtplat->reg) / 2,
			&plat->map);
	if (ret)
		return ret;

	return 0;
}
#endif

static int rk3399_dmc_init(struct udevice *dev)
{
	struct dram_info *priv = dev_get_priv(dev);
	struct rockchip_dmc_plat *plat = dev_get_platdata(dev);
	int ret;
#if !CONFIG_IS_ENABLED(OF_PLATDATA)
	struct rk3399_sdram_params *params = &plat->sdram_params;
#else
	struct dtd_rockchip_rk3399_dmc *dtplat = &plat->dtplat;
	struct rk3399_sdram_params *params =
					(void *)dtplat->rockchip_sdram_params;

	ret = conv_of_platdata(dev);
	if (ret)
		return ret;
#endif

	priv->cic = syscon_get_first_range(ROCKCHIP_SYSCON_CIC);
	priv->pmugrf = syscon_get_first_range(ROCKCHIP_SYSCON_PMUGRF);
	priv->pmusgrf = syscon_get_first_range(ROCKCHIP_SYSCON_PMUSGRF);
	priv->pmucru = rockchip_get_pmucru();
	priv->cru = rockchip_get_cru();
	priv->chan[0].pctl = regmap_get_range(plat->map, 0);
	priv->chan[0].pi = regmap_get_range(plat->map, 1);
	priv->chan[0].publ = regmap_get_range(plat->map, 2);
	priv->chan[0].msch = regmap_get_range(plat->map, 3);
	priv->chan[1].pctl = regmap_get_range(plat->map, 4);
	priv->chan[1].pi = regmap_get_range(plat->map, 5);
	priv->chan[1].publ = regmap_get_range(plat->map, 6);
	priv->chan[1].msch = regmap_get_range(plat->map, 7);

	debug("con reg %p %p %p %p %p %p %p %p\n",
	      priv->chan[0].pctl, priv->chan[0].pi,
	      priv->chan[0].publ, priv->chan[0].msch,
	      priv->chan[1].pctl, priv->chan[1].pi,
	      priv->chan[1].publ, priv->chan[1].msch);
	debug("cru %p, cic %p, grf %p, sgrf %p, pmucru %p\n", priv->cru,
	      priv->cic, priv->pmugrf, priv->pmusgrf, priv->pmucru);
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	ret = clk_get_by_index_platdata(dev, 0, dtplat->clocks, &priv->ddr_clk);
#else
	ret = clk_get_by_index(dev, 0, &priv->ddr_clk);
#endif
	if (ret) {
		printf("%s clk get failed %d\n", __func__, ret);
		return ret;
	}
	ret = clk_set_rate(&priv->ddr_clk, params->base.ddr_freq * MHz);
	if (ret < 0) {
		printf("%s clk set failed %d\n", __func__, ret);
		return ret;
	}
	ret = sdram_init(priv, params);
	if (ret < 0) {
		printf("%s DRAM init failed%d\n", __func__, ret);
		return ret;
	}

#if defined(DEBUG)
	{
		*(volatile u64*)0 = 0x0123456789abcdef;
		printf("%08x %08x\n", *(volatile u32*)0, *(volatile u32*)4);
	}
#endif

	return 0;
}
#endif

static int rk3399_dmc_probe(struct udevice *dev)
{
#ifdef CONFIG_SPL_BUILD
	if (rk3399_dmc_init(dev))
		do_reset(NULL, 0, 0, NULL);
#else
	struct dram_info *priv = dev_get_priv(dev);

	priv->pmugrf = syscon_get_first_range(ROCKCHIP_SYSCON_PMUGRF);
	debug("%s: pmugrf=%p\n", __func__, priv->pmugrf);
	priv->info.base = CONFIG_SYS_SDRAM_BASE;
	priv->info.size = rockchip_sdram_size(
			(phys_addr_t)&priv->pmugrf->os_reg2);
#endif
	return 0;
}

static int rk3399_dmc_get_info(struct udevice *dev, struct ram_info *info)
{
	struct dram_info *priv = dev_get_priv(dev);

	*info = priv->info;

	return 0;
}

static struct ram_ops rk3399_dmc_ops = {
	.get_info = rk3399_dmc_get_info,
};


static const struct udevice_id rk3399_dmc_ids[] = {
	{ .compatible = "rockchip,rk3399-dmc" },
	{ }
};

U_BOOT_DRIVER(dmc_rk3399) = {
	.name = "rockchip_rk3399_dmc",
	.id = UCLASS_RAM,
	.of_match = rk3399_dmc_ids,
	.ops = &rk3399_dmc_ops,
#ifdef CONFIG_SPL_BUILD
	.ofdata_to_platdata = rk3399_dmc_ofdata_to_platdata,
#endif
	.probe = rk3399_dmc_probe,
	.priv_auto_alloc_size = sizeof(struct dram_info),
#ifdef CONFIG_SPL_BUILD
	.platdata_auto_alloc_size = sizeof(struct rockchip_dmc_plat),
#endif
};
