/*
 * DesignWare MIPI DSI Host Controller v1.02 driver
 *
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * Author:
 *	<shizongxuan@huawei.com>
 *	<zhangxiubin@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/of_graph.h>
#include <linux/iopoll.h>
#include <video/mipi_display.h>
#include <linux/gpio/consumer.h>
#include <linux/of_address.h>

#include <drm/drm_of.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_panel.h>

#include "sprd_dsi_reg.h"
#include "kirin_dpe_reg.h"
#include "kirin_drm_dpe_utils.h"

#define DTS_COMP_DSI_NAME "hisilicon,hi3660-dsi"

#define ROUND(x, y)		((x) / (y) + \
				((x) % (y) * 10 / (y) >= 5 ? 1 : 0))
#define ROUND1(x, y)	((x) / (y) + ((x) % (y)  ? 1 : 0))
#define PHY_REF_CLK_RATE	19200000
#define PHY_REF_CLK_PERIOD_PS	(1000000000 / (PHY_REF_CLK_RATE / 1000))

#define encoder_to_dsi(encoder) \
	container_of(encoder, struct sprd_dsi, encoder)
#define host_to_dsi(host) \
	container_of(host, struct sprd_dsi, host)
#define connector_to_dsi(connector) \
	container_of(connector, struct sprd_dsi, connector)
#define DSS_REDUCE(x)	((x) > 0 ? ((x) - 1) : (x))

enum dsi_output_client {
	OUT_HDMI = 0,
	OUT_PANEL,
	OUT_MAX
};

struct mipi_phy_params {
	u64 lane_byte_clk;
	u32 clk_division;

	u32 clk_lane_lp2hs_time;
	u32 clk_lane_hs2lp_time;
	u32 data_lane_lp2hs_time;
	u32 data_lane_hs2lp_time;
	u32 clk2data_delay;
	u32 data2clk_delay;

	u32 clk_pre_delay;
	u32 clk_post_delay;
	u32 clk_t_lpx;
	u32 clk_t_hs_prepare;
	u32 clk_t_hs_zero;
	u32 clk_t_hs_trial;
	u32 clk_t_wakeup;
	u32 data_pre_delay;
	u32 data_post_delay;
	u32 data_t_lpx;
	u32 data_t_hs_prepare;
	u32 data_t_hs_zero;
	u32 data_t_hs_trial;
	u32 data_t_ta_go;
	u32 data_t_ta_get;
	u32 data_t_wakeup;

	u32 phy_stop_wait_time;

	u32 rg_vrefsel_vcm;
	u32 rg_hstx_ckg_sel;
	u32 rg_pll_fbd_div5f;
	u32 rg_pll_fbd_div1f;
	u32 rg_pll_fbd_2p;
	u32 rg_pll_enbwt;
	u32 rg_pll_fbd_p;
	u32 rg_pll_fbd_s;
	u32 rg_pll_pre_div1p;
	u32 rg_pll_pre_p;
	u32 rg_pll_vco_750m;
	u32 rg_pll_lpf_rs;
	u32 rg_pll_lpf_cs;
	u32 rg_pll_enswc;
	u32 rg_pll_chp;

	u32 pll_register_override;		/*0x1E[0]*/
	u32 pll_power_down;			/*0x1E[1]*/
	u32 rg_band_sel;				/*0x1E[2]*/
	u32 rg_phase_gen_en;		/*0x1E[3]*/
	u32 reload_sel;				/*0x1E[4]*/
	u32 rg_pll_cp_p;				/*0x1E[7:5]*/
	u32 rg_pll_refsel;				/*0x16[1:0]*/
	u32 rg_pll_cp;				/*0x16[7:5]*/
	u32 load_command;
};

struct dsi_hw_ctx {
	void __iomem *base;
	char __iomem *peri_crg_base;

	struct clk *dss_dphy0_ref_clk;
	struct clk *dss_dphy1_ref_clk;
	struct clk *dss_dphy0_cfg_clk;
	struct clk *dss_dphy1_cfg_clk;
	struct clk *dss_pclk_dsi0_clk;
	struct clk *dss_pclk_dsi1_clk;
};

struct sprd_dsi_client {
	u32 lanes;
	u32 phy_clock; /* in kHz */
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
};

struct mipi_panel_info {
	u8 dsi_version;
	u8 vc;
	u8 lane_nums;
	u8 lane_nums_select_support;
	u8 color_mode;
	u32 dsi_bit_clk; /* clock lane(p/n) */
	u32 burst_mode;
	u32 max_tx_esc_clk;
	u8 non_continue_en;

	u32 dsi_bit_clk_val1;
	u32 dsi_bit_clk_val2;
	u32 dsi_bit_clk_val3;
	u32 dsi_bit_clk_val4;
	u32 dsi_bit_clk_val5;
	u32 dsi_bit_clk_upt;
	/*uint32_t dsi_pclk_rate;*/

	u32 hs_wr_to_time;

	/* dphy config parameter adjust*/
	u32 clk_post_adjust;
	u32 clk_pre_adjust;
	u32 clk_pre_delay_adjust;
	u32 clk_t_hs_exit_adjust;
	u32 clk_t_hs_trial_adjust;
	u32 clk_t_hs_prepare_adjust;
	int clk_t_lpx_adjust;
	u32 clk_t_hs_zero_adjust;
	u32 data_post_delay_adjust;
	int data_t_lpx_adjust;
	u32 data_t_hs_prepare_adjust;
	u32 data_t_hs_zero_adjust;
	u32 data_t_hs_trial_adjust;
	u32 rg_vrefsel_vcm_adjust;

	/*only for Chicago<3660> use*/
	u32 rg_vrefsel_vcm_clk_adjust;
	u32 rg_vrefsel_vcm_data_adjust;
};

struct ldi_panel_info {
	u32 h_back_porch;
	u32 h_front_porch;
	u32 h_pulse_width;

	/*
	** note: vbp > 8 if used overlay compose,
	** also lcd vbp > 8 in lcd power on sequence
	*/
	u32 v_back_porch;
	u32 v_front_porch;
	u32 v_pulse_width;

	u8 hsync_plr;
	u8 vsync_plr;
	u8 pixelclk_plr;
	u8 data_en_plr;

	/* for cabc */
	u8 dpi0_overlap_size;
	u8 dpi1_overlap_size;
};

struct sprd_dsi {
	struct drm_encoder encoder;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct mipi_dsi_host host;
	struct drm_connector connector; /* connector for panel */
	struct drm_display_mode cur_mode;
	struct dsi_hw_ctx *ctx;
	struct mipi_phy_params phy;
	struct mipi_panel_info mipi;
	struct ldi_panel_info ldi;
	u32 lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	struct gpio_desc *gpio_mux;
	struct sprd_dsi_client client[OUT_MAX];
	enum dsi_output_client cur_client;
	bool enable;
};

struct dsi_data {
	struct sprd_dsi dsi;
	struct dsi_hw_ctx ctx;
};

struct dsi_phy_range {
	u32 min_range_kHz;
	u32 max_range_kHz;
	u32 pll_vco_750M;
	u32 hstx_ckg_sel;
};

static const struct dsi_phy_range dphy_range_info[] = {
	{   46875,    62500,   1,    7 },
	{   62500,    93750,   0,    7 },
	{   93750,   125000,   1,    6 },
	{  125000,   187500,   0,    6 },
	{  187500,   250000,   1,    5 },
	{  250000,   375000,   0,    5 },
	{  375000,   500000,   1,    4 },
	{  500000,   750000,   0,    4 },
	{  750000,  1000000,   1,    0 },
	{ 1000000,  1500000,   0,    0 }
};

void dsi_set_output_client(struct drm_device *dev)
{
	enum dsi_output_client client;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct sprd_dsi *dsi;

	mutex_lock(&dev->mode_config.mutex);

	/* find dsi encoder */
	drm_for_each_encoder(encoder, dev)
		if (encoder->encoder_type == DRM_MODE_ENCODER_DSI)
			break;
	dsi = encoder_to_dsi(encoder);

	/* find HDMI connector */
	drm_for_each_connector(connector, dev)
		if (connector->connector_type == DRM_MODE_CONNECTOR_HDMIA)
			break;

	/*
	 * set the proper dsi output client
	 */
	client = connector->status == connector_status_connected ?
		OUT_HDMI : OUT_PANEL;
	if (client != dsi->cur_client) {
		/* associate bridge and dsi encoder */
		if (client == OUT_HDMI)
			encoder->bridge = dsi->bridge;
		else
			encoder->bridge = NULL;

		gpiod_set_value_cansleep(dsi->gpio_mux, client);
		dsi->cur_client = client;
		/* let the userspace know panel connector status has changed */
		drm_sysfs_hotplug_event(dev);
		DRM_INFO("client change to %s\n", client == OUT_HDMI ?
				 "HDMI" : "panel");
	}

	mutex_unlock(&dev->mode_config.mutex);
}
EXPORT_SYMBOL(dsi_set_output_client);


static void dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct sprd_dsi *dsi = encoder_to_dsi(encoder);
	struct dsi_hw_ctx *ctx = dsi->ctx;
	void __iomem *base = ctx->base;

	if (!dsi->enable)
		return;

	/* turn off panel's backlight */
	if (dsi->panel && drm_panel_disable(dsi->panel))
		DRM_ERROR("failed to disable panel\n");

	/* turn off panel */
	if (dsi->panel && drm_panel_unprepare(dsi->panel))
		DRM_ERROR("failed to unprepare panel\n");

	dsi->enable = false;
}


static void dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct sprd_dsi *dsi = encoder_to_dsi(encoder);
	struct dsi_hw_ctx *ctx = dsi->ctx;
	int ret;

	if (dsi->enable)
		return;

	/* turn on panel */
	if (dsi->panel && drm_panel_prepare(dsi->panel))
		DRM_ERROR("failed to prepare panel\n");

	/*sprd_dsi_set_mode(dsi, DSI_VIDEO_MODE);*/

	/* turn on panel's back light */
	if (dsi->panel && drm_panel_enable(dsi->panel))
		DRM_ERROR("failed to enable panel\n");

	dsi->enable = true;
}

static void dsi_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adj_mode)
{
	struct sprd_dsi *dsi = encoder_to_dsi(encoder);

	drm_mode_copy(&dsi->cur_mode, adj_mode);
}

static int dsi_encoder_atomic_check(struct drm_encoder *encoder,
				    struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	/* do nothing */
	return 0;
}

static const struct drm_encoder_helper_funcs sprd_encoder_helper_funcs = {
	.atomic_check	= dsi_encoder_atomic_check,
	.mode_set	= dsi_encoder_mode_set,
	.enable		= dsi_encoder_enable,
	.disable	= dsi_encoder_disable
};

static const struct drm_encoder_funcs sprd_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int sprd_drm_encoder_init(struct device *dev,
			       struct drm_device *drm_dev,
			       struct drm_encoder *encoder)
{
	int ret;
	u32 crtc_mask = drm_of_find_possible_crtcs(drm_dev, dev->of_node);

	if (!crtc_mask) {
		DRM_ERROR("failed to find crtc mask\n");
		return -EINVAL;
	}

	encoder->possible_crtcs = crtc_mask;
	ret = drm_encoder_init(drm_dev, encoder, &sprd_encoder_funcs,
			       DRM_MODE_ENCODER_DSI);
	if (ret) {
		DRM_ERROR("failed to init dsi encoder\n");
		return ret;
	}

	drm_encoder_helper_add(encoder, &sprd_encoder_helper_funcs);

	return 0;
}

static int dsi_host_attach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *mdsi)
{
	struct sprd_dsi *dsi = host_to_dsi(host);
	u32 id = mdsi->channel >= 1 ? OUT_PANEL : OUT_HDMI;

	if (mdsi->lanes < 1 || mdsi->lanes > 4) {
		DRM_ERROR("dsi device params invalid\n");
		return -EINVAL;
	}

	dsi->client[id].lanes = mdsi->lanes;
	dsi->client[id].format = mdsi->format;
	dsi->client[id].mode_flags = mdsi->mode_flags;
	dsi->client[id].phy_clock = mdsi->phy_clock;

	DRM_INFO("host attach, client name=[%s], id=%d\n", mdsi->name, id);

	return 0;
}

static int dsi_host_detach(struct mipi_dsi_host *host,
			   struct mipi_dsi_device *mdsi)
{
	/* do nothing */
	return 0;
}

static int dsi_gen_pkt_hdr_write(void __iomem *base, u32 val)
{
	u32 status;
	int ret;

	return 0;
}

static int dsi_dcs_short_write(void __iomem *base,
			       const struct mipi_dsi_msg *msg)
{
	const u16 *tx_buf = msg->tx_buf;
	u32 val = GEN_HDATA(*tx_buf) | GEN_HTYPE(msg->type);

	return dsi_gen_pkt_hdr_write(base, val);
}

static int dsi_dcs_long_write(void __iomem *base,
			      const struct mipi_dsi_msg *msg)
{
	const u32 *tx_buf = msg->tx_buf;
	int len = msg->tx_len, pld_data_bytes = sizeof(*tx_buf), ret;
	u32 val = GEN_HDATA(msg->tx_len) | GEN_HTYPE(msg->type);
	u32 remainder = 0;
	u32 status;

	return dsi_gen_pkt_hdr_write(base, val);
}

static ssize_t dsi_host_transfer(struct mipi_dsi_host *host,
				    const struct mipi_dsi_msg *msg)
{
	struct sprd_dsi *dsi = host_to_dsi(host);
	struct dsi_hw_ctx *ctx = dsi->ctx;
	void __iomem *base = ctx->base;
	int ret;

	switch (msg->type) {
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
		ret = dsi_dcs_short_write(base, msg);
		break;
	case MIPI_DSI_DCS_LONG_WRITE:
		ret = dsi_dcs_long_write(base, msg);
		break;
	default:
		DRM_ERROR("unsupported message type\n");
		ret = -EINVAL;
	}

	return ret;
}

static const struct mipi_dsi_host_ops dsi_host_ops = {
	.attach = dsi_host_attach,
	.detach = dsi_host_detach,
	.transfer = dsi_host_transfer,
};

static int dsi_host_init(struct device *dev, struct sprd_dsi *dsi)
{
	struct mipi_dsi_host *host = &dsi->host;
	struct mipi_panel_info *mipi = &dsi->mipi;
	int ret;

	host->dev = dev;
	host->ops = &dsi_host_ops;

	ret = mipi_dsi_host_register(host);
	if (ret) {
		DRM_ERROR("failed to register dsi host\n");
		return ret;
	}

	return 0;
}

static int dsi_bridge_init(struct drm_device *dev, struct sprd_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_bridge *bridge = dsi->bridge;
	int ret;

	/* associate the bridge to dsi encoder */
	bridge->encoder = encoder;

	ret = drm_bridge_attach(dev, bridge);
	if (ret) {
		DRM_ERROR("failed to attach external bridge\n");
		return ret;
	}

	return 0;
}

static int dsi_connector_get_modes(struct drm_connector *connector)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	return drm_panel_get_modes(dsi->panel);
}

static enum drm_mode_status
dsi_connector_mode_valid(struct drm_connector *connector,
			 struct drm_display_mode *mode)
{
	enum drm_mode_status mode_status = MODE_OK;

	return mode_status;
}

static struct drm_encoder *
dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);

	return &dsi->encoder;
}

static struct drm_connector_helper_funcs dsi_connector_helper_funcs = {
	.get_modes = dsi_connector_get_modes,
	.mode_valid = dsi_connector_mode_valid,
	.best_encoder = dsi_connector_best_encoder,
};

static enum drm_connector_status
dsi_connector_detect(struct drm_connector *connector, bool force)
{
	struct sprd_dsi *dsi = connector_to_dsi(connector);
	enum drm_connector_status status;

	status = dsi->cur_client == OUT_PANEL ?	connector_status_connected :
		connector_status_disconnected;

	return status;
}

static void dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs dsi_atomic_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = dsi_connector_detect,
	.destroy = dsi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int dsi_connector_init(struct drm_device *dev, struct sprd_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_connector *connector = &dsi->connector;
	int ret;

	connector->polled = DRM_CONNECTOR_POLL_HPD;
	drm_connector_helper_add(connector,
				 &dsi_connector_helper_funcs);

	ret = drm_connector_init(dev, &dsi->connector,
				 &dsi_atomic_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret)
		return ret;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		return ret;

	ret = drm_panel_attach(dsi->panel, connector);
	if (ret)
		return ret;

	DRM_INFO("connector init\n");
	return 0;
}
static int dsi_bind(struct device *dev, struct device *master, void *data)
{
	struct dsi_data *ddata = dev_get_drvdata(dev);
	struct sprd_dsi *dsi = &ddata->dsi;
	struct drm_device *drm_dev = data;
	int ret;

	ret = sprd_drm_encoder_init(dev, drm_dev, &dsi->encoder);
	if (ret)
		return ret;

	if (dsi->bridge) {
		ret = dsi_bridge_init(drm_dev, dsi);
		if (ret)
			return ret;
	}

	if (dsi->panel) {
		ret = dsi_connector_init(drm_dev, dsi);
		if (ret)
			return ret;
	}

	return 0;
}

static void dsi_unbind(struct device *dev, struct device *master, void *data)
{
	/* do nothing */
}

static const struct component_ops dsi_ops = {
	.bind	= dsi_bind,
	.unbind	= dsi_unbind,
};

static int dsi_parse_bridge_endpoint(struct sprd_dsi *dsi,
				     struct device_node *endpoint)
{
	struct device_node *bridge_node;
	struct drm_bridge *bridge;

	bridge_node = of_graph_get_remote_port_parent(endpoint);
	if (!bridge_node) {
		DRM_ERROR("no valid bridge node\n");
		return -ENODEV;
	}
	of_node_put(bridge_node);

	bridge = of_drm_find_bridge(bridge_node);
	if (!bridge) {
		DRM_INFO("wait for external HDMI bridge driver.\n");
		return -EPROBE_DEFER;
	}
	dsi->bridge = bridge;

	return 0;
}

static int dsi_parse_panel_endpoint(struct sprd_dsi *dsi,
				    struct device_node *endpoint)
{
	struct device_node *panel_node;
	struct drm_panel *panel;

	panel_node = of_graph_get_remote_port_parent(endpoint);
	if (!panel_node) {
		DRM_ERROR("no valid panel node\n");
		return -ENODEV;
	}
	of_node_put(panel_node);

	panel = of_drm_find_panel(panel_node);
	if (!panel) {
		DRM_DEBUG_DRIVER("skip this panel endpoint.\n");
		return 0;
	}
	dsi->panel = panel;

	return 0;
}

static int dsi_parse_endpoint(struct sprd_dsi *dsi,
			      struct device_node *np,
			      enum dsi_output_client client)
{
	struct device_node *ep_node;
	struct of_endpoint ep;
	int ret = 0;

	if (client == OUT_MAX)
		return -EINVAL;

	for_each_endpoint_of_node(np, ep_node) {
		ret = of_graph_parse_endpoint(ep_node, &ep);
		if (ret) {
			of_node_put(ep_node);
			return ret;
		}

		/* skip dsi input port, port == 0 is input port */
		if (ep.port == 0)
			continue;

		/* parse bridge endpoint */
		if (client == OUT_HDMI) {
			if (ep.id == 0) {
				ret = dsi_parse_bridge_endpoint(dsi, ep_node);
				if (dsi->bridge)
					break;
			}
		} else { /* parse panel endpoint */
			if (ep.id > 0) {
				ret = dsi_parse_panel_endpoint(dsi, ep_node);
				if (dsi->panel)
					break;
			}
		}

		if (ret) {
			of_node_put(ep_node);
			return ret;
		}
	}

	if (!dsi->bridge && !dsi->panel) {
		DRM_ERROR("at least one bridge or panel node is required\n");
		return -ENODEV;
	}

	return 0;
}

static int dsi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct dsi_data *data;
	struct sprd_dsi *dsi;
	struct dsi_hw_ctx *ctx;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		DRM_ERROR("failed to allocate dsi data.\n");
		return -ENOMEM;
	}
	dsi = &data->dsi;
	ctx = &data->ctx;
	dsi->ctx = ctx;

	/* parse HDMI bridge endpoint */
	ret = dsi_parse_endpoint(dsi, np, OUT_HDMI);
	if (ret)
		return ret;

	ret = dsi_host_init(dev, dsi);
	if (ret)
		return ret;

	/* parse panel endpoint */
	ret = dsi_parse_endpoint(dsi, np, OUT_PANEL);
	if (ret)
		goto err_host_unregister;

	platform_set_drvdata(pdev, data);

	ret = component_add(dev, &dsi_ops);
	if (ret)
		goto err_host_unregister;

	return 0;

err_host_unregister:
	mipi_dsi_host_unregister(&dsi->host);
	return ret;
}

static int dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dsi_ops);

	return 0;
}

static const struct of_device_id dsi_of_match[] = {
	{.compatible = "sprd,dsi-controller"},
	{ }
};
MODULE_DEVICE_TABLE(of, dsi_of_match);

static struct platform_driver dsi_driver = {
	.probe = dsi_probe,
	.remove = dsi_remove,
	.driver = {
		.name = "sprd-dsi-driver",
		.of_match_table = dsi_of_match,
	},
};

module_platform_driver(dsi_driver);

MODULE_DESCRIPTION("DesignWare MIPI DSI Host Controller v1.02 driver");
MODULE_LICENSE("GPL v2");
