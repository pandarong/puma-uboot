/*
 * Copyright (c) 2017 Theobroma Systems Design und Consulting GmbH
 * Copyright (c) 2015 Google, Inc
 * Copyright 2014 Rockchip Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#define DEBUG

#include <common.h>
#include <clk.h>
#include <display.h>
#include <dm.h>
#include <dw_hdmi.h>
#include <edid.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/hardware.h>
#include <asm/arch/grf_rk3288.h>
#include <asm/arch/grf_rk3399.h>
#include <power/regulator.h>

struct rk_hdmi_priv {
	struct dw_hdmi hdmi;
	void *grf;
};

struct rkhdmi_driverdata {
	/* configuration */
	u8 i2c_clk_high;
	u8 i2c_clk_low;
	const char * const *regulator_names;
	u32 regulator_names_cnt;
	/* setters/getters */
	int (*set_input_vop)(struct udevice *dev);
	int (*clk_config)(struct udevice *dev);
};

static const struct hdmi_phy_config rockchip_phy_config[] = {
	{
		.mpixelclock = 74250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0272,
	}, {
		.mpixelclock = 148500000,
		.sym_ctr = 0x802b, .term = 0x0004, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = 297000000,
		.sym_ctr = 0x8039, .term = 0x0005, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = ~0ul,
		.sym_ctr = 0x0000, .term = 0x0000, .vlev_ctr = 0x0000,
	}
};

static const struct hdmi_mpll_config rockchip_mpll_cfg[] = {
	{
		.mpixelclock = 40000000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0018,
	}, {
		.mpixelclock = 65000000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 66000000,
		.cpce = 0x013e, .gmp = 0x0003, .curr = 0x0038,
	}, {
		.mpixelclock = 83500000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 146250000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x0038,
	}, {
		.mpixelclock = 148500000,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = ~0ul,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}
};

static int rk3288_set_input_vop(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct display_plat *uc_plat = dev_get_uclass_platdata(dev);
	int vop_id = uc_plat->source_id;
	struct rk3288_grf *grf = priv->grf;

	/* hdmi source select hdmi controller */
	rk_setreg(&grf->soc_con6, 1 << 15);

	/* hdmi data from vop id */
	rk_clrsetreg(&grf->soc_con6, 1 << 4, (vop_id == 1) ? (1 << 4) : 0);

	return 0;
}

static int rk3399_set_input_vop(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct display_plat *uc_plat = dev_get_uclass_platdata(dev);
	int vop_id = uc_plat->source_id;
	struct rk3399_grf_regs *grf = priv->grf;

	/* hdmi data from vop id */
	rk_clrsetreg(&grf->soc_con20, GRF_RK3399_HDMI_VOP_SEL_MASK,
		     (vop_id == 1) ? GRF_RK3399_HDMI_VOP_SEL_L : 0);

	return 0;
}

static int rk3288_clk_config(struct udevice *dev)
{
	struct display_plat *uc_plat = dev_get_uclass_platdata(dev);
	struct clk clk;
	int ret;

	/*
	 * Configure the maximum clock to permit whatever resolution the
	 * monitor wants
	 */
	ret = clk_get_by_index(uc_plat->src_dev, 0, &clk);
	if (ret >= 0) {
		ret = clk_set_rate(&clk, 384000000);
		clk_free(&clk);
	}
	if (ret < 0) {
		debug("%s: Failed to set clock in source device '%s': ret=%d\n",
		      __func__, uc_plat->src_dev->name, ret);
		return ret;
	}

	return 0;
}

static int rk_hdmi_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_read_edid(&priv->hdmi, buf, buf_size);
}

static int rk_hdmi_enable(struct udevice *dev, int panel_bpp,
			  const struct display_timing *edid)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_enable(&priv->hdmi, edid);
}

static int rk_hdmi_ofdata_to_platdata(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct rkhdmi_driverdata *data =
		(struct rkhdmi_driverdata *)dev_get_driver_data(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;

	hdmi->ioaddr = (ulong)dev_get_addr(dev);
	hdmi->mpll_cfg = rockchip_mpll_cfg;
	hdmi->phy_cfg = rockchip_phy_config;
	hdmi->i2c_clk_high = data->i2c_clk_high;
	hdmi->i2c_clk_low = data->i2c_clk_low;

	hdmi->reg_io_width = 4;
	hdmi->phy_set = dw_hdmi_phy_cfg;

	priv->grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);

	return 0;
}

static int rk_hdmi_probe(struct udevice *dev)
{
	struct rkhdmi_driverdata *data =
		(struct rkhdmi_driverdata *)dev_get_driver_data(dev);
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;
	struct udevice *reg;
	struct clk clk;
	const char *regulator_name;
	int ret;
	int i;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret >= 0) {
		ret = clk_set_rate(&clk, 0);
		clk_free(&clk);
	}
	if (ret) {
		debug("%s: Failed to set hdmi clock: ret=%d\n", __func__, ret);
		return ret;
	}

	if (data->clk_config) {
		ret = data->clk_config(dev);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < data->regulator_names_cnt; ++i) {
		regulator_name = data->regulator_names[i];
		debug("%s: probing regulator '%s'\n", __func__, regulator_name);

		ret = regulator_autoset_by_name(regulator_name, &reg);
		if (!ret)
			ret = regulator_set_enable(reg, true);
	}

	if (!data->set_input_vop) {
		debug("%s: data->set_input_vop not set\n", __func__);
		return -1;
	}

	data->set_input_vop(dev);

	ret = dw_hdmi_phy_wait_for_hpd(hdmi);
	if (ret < 0) {
		debug("hdmi can not get hpd signal\n");
		return -1;
	}

	dw_hdmi_init(hdmi);
	dw_hdmi_phy_init(hdmi);

	return 0;
}

static const struct dm_display_ops rk_hdmi_ops = {
	.read_edid = rk_hdmi_read_edid,
	.enable = rk_hdmi_enable,
};

static const char * const rk3288_regulator_names[] = {
	"vcc50_hdmi",
};

static const struct rkhdmi_driverdata rk3288_driverdata = {
	/*
	 * TODO(sjg@chromium.org): The above values don't work - these ones
	 * work better, but generate lots of errors in the data.
	 */
	.i2c_clk_high = 0x0d,
	.i2c_clk_low = 0x0d,
	.regulator_names = rk3288_regulator_names,
	.regulator_names_cnt = ARRAY_SIZE(rk3288_regulator_names),
	.set_input_vop = rk3288_set_input_vop,
	.clk_config = rk3288_clk_config,
};

static const char* rk3399_regulator_names[] = {
	"vcc1v8_hdmi",
	"vcc0v9_hdmi"
};

static const struct rkhdmi_driverdata rk3399_driverdata = {
	.i2c_clk_high = 0x7a,
	.i2c_clk_low = 0x8d,
	.regulator_names = rk3399_regulator_names,
	.regulator_names_cnt = ARRAY_SIZE(rk3399_regulator_names),
	.set_input_vop = rk3399_set_input_vop,
};

static const struct udevice_id rk_hdmi_ids[] = {
	{ .compatible = "rockchip,rk3288-dw-hdmi",
	  .data = (ulong)&rk3288_driverdata },
	{ .compatible = "rockchip,rk3399-dw-hdmi",
	  .data = (ulong)&rk3399_driverdata },
	{ }
};

U_BOOT_DRIVER(hdmi_rockchip) = {
	.name	= "hdmi_rockchip",
	.id	= UCLASS_DISPLAY,
	.of_match = rk_hdmi_ids,
	.ops	= &rk_hdmi_ops,
	.ofdata_to_platdata	= rk_hdmi_ofdata_to_platdata,
	.probe	= rk_hdmi_probe,
	.priv_auto_alloc_size	 = sizeof(struct rk_hdmi_priv),
};
