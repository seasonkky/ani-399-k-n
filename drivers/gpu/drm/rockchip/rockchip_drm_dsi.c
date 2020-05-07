/*
 * Copyright (c) 2017 Rockchip Electronics Co. Ltd.
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
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/irq.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/gpio/consumer.h>
#include <asm/unaligned.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drmP.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_vop.h"

#define IS_DSI0(dsi)	((dsi)->id == 0)
#define IS_DSI1(dsi)	((dsi)->id == 1)

#define UPDATE(v, h, l)		(((v) << (l)) & GENMASK((h), (l)))
#define HIWORD_UPDATE(v, h, l)	(((v) << (l)) | (GENMASK(h, l) << 16))
#define GRF_DESC(r, h, l)	((r << 16) | (h << 8) | (l))

/* DWC MIPI DSI Host Controller Register and Field Descriptions */
#define DSI_VERSION			0x000
#define DSI_PWR_UP			0x004
#define RESET				0
#define POWER_UP			BIT(0)
#define DSI_CLKMGR_CFG			0x008
#define TO_CLK_DIVIDSION_MASK		GENMASK(15, 8)
#define TO_CLK_DIVIDSION(v)		UPDATE(v, 15, 8)
#define TX_ESC_CLK_DIVIDSION_MASK	GENMASK(7, 0)
#define TX_ESC_CLK_DIVIDSION(v)		UPDATE(v, 7, 0)
#define DSI_DPI_VCID			0x00c
#define DPI_VCID(v)			UPDATE(v, 1, 0)
#define DSI_DPI_COLOR_CODING		0x010
#define LOOSELY18_EN(v)			UPDATE(v, 8, 8)
#define DPI_COLOR_CODING(v)		UPDATE(v, 3, 0)
#define DSI_DPI_CFG_POL			0x014
#define COLORM_ACTIVE_LOW		BIT(4)
#define SHUTD_ACTIVE_LOW		BIT(3)
#define HSYNC_ACTIVE_LOW		BIT(2)
#define VSYNC_ACTIVE_LOW		BIT(1)
#define DATAEN_ACTIVE_LOW		BIT(0)
#define DSI_DPI_LP_CMD_TIM		0x018
#define DSI_PCKHDL_CFG			0x02c
#define CRC_RX_EN			BIT(4)
#define ECC_RX_EN			BIT(3)
#define BTA_EN				BIT(2)
#define EOTP_RX_EN			BIT(1)
#define EOTP_TX_EN			BIT(0)
#define DSI_GEN_VCID			0x030
#define GEN_VCID_RX(v)			UPDATE(v, 1, 0)
#define DSI_MODE_CFG			0x034
#define COMMAND_MODE			BIT(0)
#define VIDEO_MODE			0
#define DSI_VID_MODE_CFG		0x038
#define VPG_ORIENTATION(v)		UPDATE(v, 24, 24)
#define VPG_MODE(v)			UPDATE(v, 20, 20)
#define VPG_EN				BIT(16)
#define LP_CMD_EN			BIT(15)
#define FRAME_BTA_ACK_EN		BIT(14)
#define LP_HFP_EN			BIT(13)
#define LP_HBP_EN			BIT(12)
#define LP_VACT_EN			BIT(11)
#define LP_VFP_EN			BIT(10)
#define LP_VBP_EN			BIT(9)
#define LP_VSA_EN			BIT(8)
#define VID_MODE_TYPE(v)		UPDATE(v, 1, 0)
#define DSI_VID_PKT_SIZE		0x03c
#define VID_PKT_SIZE(v)			UPDATE(v, 13, 0)
#define DSI_VID_NUM_CHUMKS		0x040
#define DSI_VID_NULL_PKT_SIZE		0x044
#define DSI_VID_HSA_TIME		0x048
#define VID_HSA_TIME(v)			UPDATE(v, 11, 0)
#define DSI_VID_HBP_TIME		0x04c
#define VID_HBP_TIME(v)			UPDATE(v, 11, 0)
#define DSI_VID_HLINE_TIME		0x050
#define VID_HLINE_TIME(v)		UPDATE(v, 14, 0)
#define DSI_VID_VSA_LINES		0x054
#define VSA_LINES(v)			UPDATE(v, 9, 0)
#define DSI_VID_VBP_LINES		0x058
#define VBP_LINES(v)			UPDATE(v, 9, 0)
#define DSI_VID_VFP_LINES		0x05c
#define VFP_LINES(v)			UPDATE(v, 9, 0)
#define DSI_VID_VACTIVE_LINES		0x060
#define V_ACTIVE_LINES(v)		UPDATE(v, 13, 0)
#define DSI_CMD_MODE_CFG		0x068
#define CMD_XFER_TYPE_LP		0x010f7f00
#define CMD_XFER_TYPE_HS		0
#define ACK_RQST_EN			BIT(1)
#define TEAR_FX_EN			BIT(0)
#define DSI_GEN_HDR			0x06c
#define DSI_GEN_PLD_DATA		0x070
#define DSI_CMD_PKT_STATUS		0x074
#define GEN_RD_CMD_BUSY			BIT(6)
#define GEN_PLD_R_FULL			BIT(5)
#define GEN_PLD_R_EMPTY			BIT(4)
#define GEN_PLD_W_FULL			BIT(3)
#define GEN_PLD_W_EMPTY			BIT(2)
#define GEN_CMD_FULL			BIT(1)
#define GEN_CMD_EMPTY			BIT(0)
#define DSI_TO_CNT_CFG			0x078
#define HSTX_TO_CNT(v)			UPDATE(v, 31, 16)
#define LPRX_TO_CNT(v)			UPDATE(v, 15, 0)
#define DSI_HS_RD_TO_CNT		0x07c
#define DSI_LP_RD_TO_CNT		0x080
#define DSI_HS_WR_TO_CNT		0x084
#define DSI_LP_WR_TO_CNT		0x088
#define DSI_BTA_TO_CNT			0x08c
#define DSI_LPCLK_CTRL			0x094
#define AUTO_CLKLANE_CTRL		BIT(1)
#define PHY_TXREQUESTCLKHS		BIT(0)
#define DSI_PHY_TMR_LPCLK_CFG		0x098
#define PHY_CLKHS2LP_TIME(v)		UPDATE(v, 25, 16)
#define PHY_CLKLP2HS_TIME(v)		UPDATE(v, 9, 0)
#define DSI_PHY_TMR_CFG			0x09c
#define PHY_HS2LP_TIME(v)		UPDATE(v, 31, 24)
#define PHY_LP2HS_TIME(v)		UPDATE(v, 23, 16)
#define MAX_RD_TIME(v)			UPDATE(v, 14, 0)
#define DSI_PHY_RSTZ			0x0a0
#define PHY_FORCEPLL			BIT(3)
#define PHY_ENABLECLK			BIT(2)
#define PHY_RSTZ			BIT(1)
#define PHY_SHUTDOWNZ			BIT(0)
#define DSI_PHY_IF_CFG			0x0a4
#define PHY_STOP_WAIT_TIME(v)		UPDATE(v, 15, 8)
#define N_LANES(v)			UPDATE(v, 1, 0)
#define PHY_ULPS_CTRL			0x0a8
#define PHY_TXEXITULPSLAN		BIT(3)
#define PHY_TXREQULPSLAN		BIT(2)
#define PHY_TXEXITULPSCLK		BIT(1)
#define PHY_TXREQULPSCLK		BIT(0)
#define PHY_TX_TRIGGERS			0x0ac
#define DSI_PHY_STATUS			0x0b0
#define PHY_ULPSACTIVENOT3LANE		BIT(12)
#define PHY_STOPSTATE3LANE		BIT(11)
#define PHY_ULPSACTIVENOT2LANE		BIT(10)
#define PHY_STOPSTATE2LANE		BIT(9)
#define PHY_ULPSACTIVENOT1LANE		BIT(8)
#define PHY_STOPSTATE1LANE		BIT(7)
#define PHY_ULPSACTIVENOT0LANE		BIT(5)
#define PHY_STOPSTATE0LANE		BIT(4)
#define PHY_ULPSACTIVENOTCLK		BIT(3)
#define PHY_STOPSTATECLKLANE		BIT(2)
#define PHY_DIRECTION			BIT(1)
#define PHY_LOCK			BIT(0)
#define PHY_STOPSTATELANE		(PHY_STOPSTATE0LANE | \
					 PHY_STOPSTATECLKLANE)
#define DSI_PHY_TST_CTRL0		0x0b4
#define PHY_TESTCLK			BIT(1)
#define PHY_TESTCLR			BIT(0)
#define DSI_PHY_TST_CTRL1		0x0b8
#define PHY_TESTEN			BIT(16)
#define PHY_TESTDOUT_SHIFT		8
#define PHY_TESTDIN_MASK		GENMASK(7, 0)
#define PHY_TESTDIN(v)			UPDATE(v, 7, 0)
#define DSI_INT_ST0			0x0bc
#define DSI_INT_ST1			0x0c0
#define DSI_INT_MSK0			0x0c4
#define DSI_INT_MSK1			0x0c8
#define DSI_MAX_REGISGER		DSI_INT_MSK1

/* PLL Input Divider Ratio */
#define INPUT_DIV(v)		UPDATE(v, 6, 0)
/* PLL Loop Divider Ratio */
#define LOW_PROGRAM_EN		0
#define HIGH_PROGRAM_EN		BIT(7)
#define LOOP_DIV_LOW(v)		UPDATE(v, 4, 0)
#define LOOP_DIV_HIGH(v)	UPDATE(v, 3, 0)
/* PLL Input and Loop Divider Ratios Control */
#define LOOP_DIV_PROGRAM_EN	BIT(5)
#define INPUT_DIV_PROGRAM_EN	BIT(4)
/* HS operating frequency range selection */
#define HSFREQRANGE(v)		UPDATE(v, 6, 1)

enum {
	NON_BURST_MODE_SYNC_PULSE,
	NON_BURST_MODE_SYNC_EVENT,
	BURST_MODE,
};

enum {
	PIXEL_COLOR_CODING_16BIT_1,
	PIXEL_COLOR_CODING_16BIT_2,
	PIXEL_COLOR_CODING_16BIT_3,
	PIXEL_COLOR_CODING_18BIT_1,
	PIXEL_COLOR_CODING_18BIT_2,
	PIXEL_COLOR_CODING_24BIT,
};

enum grf_index {
	DPIUPDATECFG,
	DPISHUTDN,
	DPICOLORM,
	VOPSEL,
	TURNREQUEST,
	TURNDISABLE,
	FORCETXSTOPMODE,
	FORCERXMODE,
	ENABLE_N,
	MASTERSLAVEZ,
	ENABLECLK,
	BASEDIR,
	NUM_GRF_DESC,
};

struct mipi_dphy {
	/* Non-SNPS PHY */
	struct phy *phy;
	struct clk *hs_clk;

	/* SNPS PHY */
	struct clk *ref_clk;
	struct clk *cfg_clk;
	struct regmap *regmap;
	u8 prediv;
	u16 fbdiv;
};

struct rockchip_dsi_soc_data {
	unsigned int min_bit_rate_per_lane;
	unsigned int max_bit_rate_per_lane;
	const u32 *dsi0_grf_desc;
	const u32 *dsi1_grf_desc;
};

struct rockchip_dsi {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct device_node *client;

	/* for dual-channel mode support */
	struct rockchip_dsi *master;
	struct rockchip_dsi *slave;

	struct device *dev;
	void __iomem *regs;
	struct regmap *regmap;
	struct regmap *grf;
	struct clk *pclk;
	struct reset_control *rst;
	int id;
	int irq;

	struct mipi_dsi_host host;
	struct mipi_dphy dphy;
	unsigned int lane_mbps;
	unsigned int channel;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
	unsigned long mode_flags;
	struct videomode vm;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	const u32 *grf_desc;
	const struct rockchip_dsi_soc_data *soc_data;
};

#define HSTT(_max_freq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)	\
{	\
	.max_freq = _max_freq,	\
	.clk_lane = {	\
		.lp2hs = _c_lp2hs,	\
		.hs2lp = _c_hs2lp,	\
	},	\
	.data_lane = {	\
		.lp2hs = _d_lp2hs,	\
		.hs2lp = _d_hs2lp,	\
	},	\
}

/* Table A-3 High-Speed Transition Times */
static const struct {
	unsigned long max_freq;
	struct {
		u16 lp2hs;
		u16 hs2lp;
	} clk_lane;
	struct {
		u8 lp2hs;
		u8 hs2lp;
	} data_lane;
} hstt_table[] = {
	HSTT(90,    32, 20,  26, 13), HSTT(100,   35, 23,  28, 14),
	HSTT(110,   32, 22,  26, 13), HSTT(130,   31, 20,  27, 13),
	HSTT(140,   33, 22,  26, 14), HSTT(150,   33, 21,  26, 14),
	HSTT(170,   32, 20,  27, 13), HSTT(180,   36, 23,  30, 15),
	HSTT(200,   40, 22,  33, 15), HSTT(220,   40, 22,  33, 15),
	HSTT(240,   44, 24,  36, 16), HSTT(250,   48, 24,  38, 17),
	HSTT(270,   48, 24,  38, 17), HSTT(300,   50, 27,  41, 18),
	HSTT(330,   56, 28,  45, 18), HSTT(360,   59, 28,  48, 19),
	HSTT(400,   61, 30,  50, 20), HSTT(450,   67, 31,  55, 21),
	HSTT(500,   73, 31,  59, 22), HSTT(550,   79, 36,  63, 24),
	HSTT(600,   83, 37,  68, 25), HSTT(650,   90, 38,  73, 27),
	HSTT(700,   95, 40,  77, 28), HSTT(750,  102, 40,  84, 28),
	HSTT(800,  106, 42,  87, 30), HSTT(850,  113, 44,  93, 31),
	HSTT(900,  118, 47,  98, 32), HSTT(950,  124, 47, 102, 34),
	HSTT(1000, 130, 49, 107, 35), HSTT(1050, 135, 51, 111, 37),
	HSTT(1100, 139, 51, 114, 38), HSTT(1150, 146, 54, 120, 40),
	HSTT(1200, 153, 57, 125, 41), HSTT(1250, 158, 58, 130, 42),
	HSTT(1300, 163, 58, 135, 44), HSTT(1350, 168, 60, 140, 45),
	HSTT(1400, 172, 64, 144, 47), HSTT(1450, 176, 65, 148, 48),
	HSTT(1500, 181, 66, 153, 50)
};

/* Table 5-1 Frequency Ranges */
static const struct {
	unsigned long max_freq;
	u8 hsfreqrange;
} hsfreqrange_table[] = {
	{  90, 0x00}, { 100, 0x10}, { 110, 0x20}, { 130, 0x01},
	{ 140, 0x11}, { 150, 0x21}, { 170, 0x02}, { 180, 0x12},
	{ 200, 0x22}, { 220, 0x03}, { 240, 0x13}, { 250, 0x23},
	{ 270, 0x04}, { 300, 0x14}, { 330, 0x05}, { 360, 0x15},
	{ 400, 0x25}, { 450, 0x06}, { 500, 0x16}, { 550, 0x07},
	{ 600, 0x17}, { 650, 0x08}, { 700, 0x18}, { 750, 0x09},
	{ 800, 0x19}, { 850, 0x29}, { 900, 0x39}, { 950, 0x0a},
	{1000, 0x1a}, {1050, 0x2a}, {1100, 0x3a}, {1150, 0x0b},
	{1200, 0x1b}, {1250, 0x2b}, {1300, 0x3b}, {1350, 0x0c},
	{1400, 0x1c}, {1450, 0x2c}, {1500, 0x3c}
};

static inline struct rockchip_dsi *host_to_dsi(struct mipi_dsi_host *h)
{
	return container_of(h, struct rockchip_dsi, host);
}

static inline struct rockchip_dsi *connector_to_dsi(struct drm_connector *c)
{
	return container_of(c, struct rockchip_dsi, connector);
}

static inline struct rockchip_dsi *encoder_to_dsi(struct drm_encoder *e)
{
	return container_of(e, struct rockchip_dsi, encoder);
}

static void grf_write(struct rockchip_dsi *dsi, enum grf_index index, u32 val)
{
	const u32 desc = dsi->grf_desc[index];
	u16 reg;
	u8 h, l;

	if (!desc)
		return;

	reg = (desc >> 16) & 0xffff;
	h = (desc >> 8) & 0xff;
	l = desc & 0xff;
	regmap_write(dsi->grf, reg, HIWORD_UPDATE(val, h, l));
}

static inline void ppi_txrequestclkhs_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_LPCLK_CTRL,
			   PHY_TXREQUESTCLKHS, PHY_TXREQUESTCLKHS);
	udelay(1);
}

static inline void ppi_txrequestclkhs_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_LPCLK_CTRL, PHY_TXREQUESTCLKHS, 0);
	udelay(1);
}

static int ppi_stopstatelane_asserted(struct rockchip_dsi *dsi)
{
	u32 v, m = PHY_STOPSTATELANE;

	return readl_poll_timeout_atomic(dsi->regs + DSI_PHY_STATUS,
					 v, (v & m) == m, 1, USEC_PER_MSEC);
}

static inline void ppi_turnrequest_assert(struct rockchip_dsi *dsi)
{
	grf_write(dsi, TURNREQUEST, 1);
}

static inline void ppi_turnrequest_deassert(struct rockchip_dsi *dsi)
{
	grf_write(dsi, TURNREQUEST, 0);
}

static inline void ppi_forcetxstopmode_assert(struct rockchip_dsi *dsi)
{
	grf_write(dsi, FORCETXSTOPMODE, 1);
}

static inline void ppi_forcetxstopmode_deassert(struct rockchip_dsi *dsi)
{
	grf_write(dsi, FORCETXSTOPMODE, 0);
}

static inline void genif_vcid_init(struct rockchip_dsi *dsi, u8 vcid)
{
	regmap_write(dsi->regmap, DSI_GEN_VCID, GEN_VCID_RX(vcid));
}

static int genif_wait_w_pld_fifo_not_full(struct rockchip_dsi *dsi)
{
	u32 v, m = GEN_PLD_W_FULL;

	return readl_poll_timeout_atomic(dsi->regs + DSI_CMD_PKT_STATUS,
					 v, !(v & m), 1, USEC_PER_MSEC);
}

static int genif_wait_cmd_fifo_not_full(struct rockchip_dsi *dsi)
{
	u32 v, m = GEN_CMD_FULL;

	return readl_poll_timeout_atomic(dsi->regs + DSI_CMD_PKT_STATUS,
					 v, !(v & m), 1, USEC_PER_MSEC);
}

static int genif_wait_w_fifo_is_empty(struct rockchip_dsi *dsi)
{
	u32 v, m = GEN_PLD_W_EMPTY | GEN_CMD_EMPTY;

	return readl_poll_timeout_atomic(dsi->regs + DSI_CMD_PKT_STATUS,
					 v, (v & m) == m, 1, USEC_PER_MSEC);
}

static int genif_wait_rd_cmd_not_busy(struct rockchip_dsi *dsi)
{
	u32 v, m = GEN_RD_CMD_BUSY;

	return readl_relaxed_poll_timeout(dsi->regs + DSI_CMD_PKT_STATUS,
					  v, !(v & m), 50, 5000);
}

static int genif_wait_r_pld_fifo_not_empty(struct rockchip_dsi *dsi)
{
	u32 v, m = GEN_PLD_R_EMPTY;

	return readl_poll_timeout_atomic(dsi->regs + DSI_CMD_PKT_STATUS,
					 v, !(v & m), 1, USEC_PER_MSEC);
}

static inline void testif_testclk_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL0,
			   PHY_TESTCLK, PHY_TESTCLK);
	udelay(1);
}

static inline void testif_testclk_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL0, PHY_TESTCLK, 0);
	udelay(1);
}

static inline void testif_testclr_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL0,
			   PHY_TESTCLR, PHY_TESTCLR);
	udelay(1);
}

static inline void testif_testclr_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL0, PHY_TESTCLR, 0);
	udelay(1);
}

static inline void testif_testen_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL1,
			   PHY_TESTEN, PHY_TESTEN);
	udelay(1);
}

static inline void testif_testen_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL1, PHY_TESTEN, 0);
	udelay(1);
}

static inline void testif_set_data(struct rockchip_dsi *dsi, u8 data)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_TST_CTRL1,
			   PHY_TESTDIN_MASK, PHY_TESTDIN(data));
	udelay(1);
}

static inline u8 testif_get_data(struct rockchip_dsi *dsi)
{
	u32 data = 0;

	regmap_read(dsi->regmap, DSI_PHY_TST_CTRL1, &data);

	return data >> PHY_TESTDOUT_SHIFT;
}

static void testif_test_code_write(struct rockchip_dsi *dsi, u8 test_code)
{
	testif_testclk_assert(dsi);
	testif_set_data(dsi, test_code);
	testif_testen_assert(dsi);
	testif_testclk_deassert(dsi);
	testif_testen_deassert(dsi);
}

static void testif_test_data_write(struct rockchip_dsi *dsi, u8 test_data)
{
	testif_testclk_deassert(dsi);
	testif_set_data(dsi, test_data);
	testif_testclk_assert(dsi);
}

static int testif_write(void *context, unsigned int reg, unsigned int value)
{
	struct rockchip_dsi *dsi = context;

	testif_test_code_write(dsi, reg);
	testif_test_data_write(dsi, value);

	dev_dbg(dsi->dev, "test_code=0x%02x, test_data=0x%02x, monitor_data=0x%02x\n",
		reg, value, testif_get_data(dsi));

	return 0;
}

static int testif_read(void *context, unsigned int reg, unsigned int *value)
{
	struct rockchip_dsi *dsi = context;

	testif_test_code_write(dsi, reg);
	*value = testif_get_data(dsi);
	testif_test_data_write(dsi, *value);

	return 0;
}

static inline void mipi_dphy_forcepll_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ,
			   PHY_FORCEPLL, PHY_FORCEPLL);
	udelay(1);
}

static inline void mipi_dphy_enableclk_assert(struct rockchip_dsi *dsi)
{
	grf_write(dsi, ENABLECLK, 1);
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ,
			   PHY_ENABLECLK, PHY_ENABLECLK);
	udelay(1);
}

static inline void mipi_dphy_enableclk_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ, PHY_ENABLECLK, 0);
	grf_write(dsi, ENABLECLK, 0);
	udelay(1);
}

static inline void mipi_dphy_shutdownz_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ, PHY_SHUTDOWNZ, 0);
	udelay(1);
}

static inline void mipi_dphy_shutdownz_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ,
			   PHY_SHUTDOWNZ, PHY_SHUTDOWNZ);
	udelay(1);
}

static inline void mipi_dphy_rstz_assert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ, PHY_RSTZ, 0);
	udelay(1);
}

static inline void mipi_dphy_rstz_deassert(struct rockchip_dsi *dsi)
{
	regmap_update_bits(dsi->regmap, DSI_PHY_RSTZ, PHY_RSTZ, PHY_RSTZ);
	udelay(1);
}

static void mipi_dphy_hstt_config(struct rockchip_dsi *dsi)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hstt_table); i++)
		if (dsi->lane_mbps < hstt_table[i].max_freq)
			break;

	if (i == ARRAY_SIZE(hstt_table))
		--i;

	regmap_write(dsi->regmap, DSI_PHY_TMR_CFG,
		     PHY_HS2LP_TIME(hstt_table[i].data_lane.hs2lp) |
		     PHY_LP2HS_TIME(hstt_table[i].data_lane.lp2hs) |
		     MAX_RD_TIME(0x7fff));
	regmap_write(dsi->regmap, DSI_PHY_TMR_LPCLK_CFG,
		     PHY_CLKHS2LP_TIME(hstt_table[i].clk_lane.hs2lp) |
		     PHY_CLKLP2HS_TIME(hstt_table[i].clk_lane.lp2hs));
}

static inline void mipi_dphy_if_config(struct rockchip_dsi *dsi)
{
	/* XXX: don't hardcode? */
	regmap_write(dsi->regmap, DSI_PHY_IF_CFG,
		     N_LANES(dsi->lanes - 1) | PHY_STOP_WAIT_TIME(0x20));
}

static inline void mipi_dphy_enable_n_assert(struct rockchip_dsi *dsi,
					     unsigned int lanes)
{
	u32 map[] = {0x1, 0x3, 0x7, 0xf};

	grf_write(dsi, ENABLE_N, map[lanes - 1]);
}

static void mipi_dphy_pll_configure(struct rockchip_dsi *dsi)
{
	struct mipi_dphy *dphy = &dsi->dphy;
	u8 n;
	u16 m;

	regmap_write(dphy->regmap, 0x19,
		     LOOP_DIV_PROGRAM_EN | INPUT_DIV_PROGRAM_EN);

	/* PLL Input Divider Ratio */
	n = dphy->prediv - 1;
	regmap_write(dphy->regmap, 0x17, INPUT_DIV(n));

	/* PLL Loop Divider Ratio */
	m = dphy->fbdiv - 1;
	regmap_write(dphy->regmap, 0x18,
		     LOW_PROGRAM_EN | LOOP_DIV_LOW(m));
	regmap_write(dphy->regmap, 0x18,
		     HIGH_PROGRAM_EN | LOOP_DIV_HIGH(m >> 5));
}

static void mipi_dphy_configure(struct rockchip_dsi *dsi)
{
	struct mipi_dphy *dphy = &dsi->dphy;
	u8 hsfreqrange;
	int index;

	for (index = 0; index < ARRAY_SIZE(hsfreqrange_table); index++)
		if (dsi->lane_mbps < hsfreqrange_table[index].max_freq)
			break;

	if (index == ARRAY_SIZE(hsfreqrange_table))
		--index;

	hsfreqrange = hsfreqrange_table[index].hsfreqrange;

	/* HS frequency range selection */
	regmap_write(dphy->regmap, 0x44, HSFREQRANGE(hsfreqrange));

	/* 1.5 Gbps analog circuitry support */
	if (dsi->lane_mbps > 1000)
		regmap_write(dphy->regmap, 0x22, 0x88);

	mipi_dphy_pll_configure(dsi);

	/* Placing the test interface in inactive mode */
	regmap_write(dphy->regmap, 0x00, 0x00);
}

static void mipi_dphy_power_off(struct rockchip_dsi *dsi)
{
	struct mipi_dphy *dphy = &dsi->dphy;

	phy_power_off(dphy->phy);
}

static int mipi_dphy_power_on(struct rockchip_dsi *dsi)
{
	struct mipi_dphy *dphy = &dsi->dphy;
	u32 status;
	int ret;

	mipi_dphy_shutdownz_deassert(dsi);
	mipi_dphy_rstz_deassert(dsi);
	mipi_dphy_forcepll_assert(dsi);
	usleep_range(1500, 2000);

	phy_power_on(dphy->phy);

	/* waits for the PLL to acquire lock */
	ret = readl_poll_timeout_atomic(dsi->regs + DSI_PHY_STATUS,
					status, status & PHY_LOCK,
					10, USEC_PER_MSEC);
	if (ret < 0) {
		dev_err(dsi->dev, "PLL is not locked\n");
		return ret;
	}

	usleep_range(100, 200);

	/* waits for lane go to the stop state */
	ret = ppi_stopstatelane_asserted(dsi);
	if (ret) {
		dev_err(dsi->dev, "lane module is not in stop state\n");
		return ret;
	}

	udelay(10);

	return 0;
}

static void mipi_dphy_init(struct rockchip_dsi *dsi)
{
	struct mipi_dphy *dphy = &dsi->dphy;

	mipi_dphy_shutdownz_assert(dsi);
	mipi_dphy_rstz_assert(dsi);
	testif_testclr_assert(dsi);

	/* Configures DPHY to work as a Master */
	grf_write(dsi, MASTERSLAVEZ, 1);

	/* Configures lane as TX */
	grf_write(dsi, BASEDIR, 0);

	/* Set all REQUEST inputs to zero */
	grf_write(dsi, TURNREQUEST, 0);
	grf_write(dsi, TURNDISABLE, 0);
	grf_write(dsi, FORCETXSTOPMODE, 0);
	grf_write(dsi, FORCERXMODE, 0);
	udelay(1);

	testif_testclr_deassert(dsi);

	if (!dphy->phy)
		mipi_dphy_configure(dsi);

	/* Enable Data Lane Module */
	mipi_dphy_enable_n_assert(dsi, dsi->lanes);

	/* Enable Clock Lane Module */
	mipi_dphy_enableclk_assert(dsi);
}

static unsigned long mipi_dphy_pll_round_rate(unsigned long fin,
					      unsigned long fout,
					      u8 *prediv, u16 *fbdiv)
{
	unsigned long best_freq = 0;
	unsigned long fvco_min, fvco_max;
	u8 min_prediv, max_prediv;
	u8 _prediv, uninitialized_var(best_prediv);
	u16 _fbdiv, uninitialized_var(best_fbdiv);
	u32 min_delta = UINT_MAX;

	fin /= USEC_PER_SEC;
	fout /= USEC_PER_SEC;

	/*
	 * PLL constraint:
	 * 5Mhz < Fref / N < 40MHz, 80MHz < Fvco < 1500Mhz
	 */
	min_prediv = DIV_ROUND_UP(fin, 40);
	max_prediv = fin / 5;
	fvco_min = 80;
	fvco_max = 1500;

	for (_prediv = min_prediv; _prediv <= max_prediv; _prediv++) {
		u32 delta, _fout;

		/* Fvco = Fref * M / N */
		_fbdiv = fout * _prediv / fin;

		/*
		 * Due to the use of a "by 2 pre-scaler," the range of the
		 * feedback multiplication value M is limited to even division
		 * numbers, and m must be greater than 12, less than 1000.
		 */
		if (_fbdiv <= 12 || _fbdiv >= 1000)
			continue;

		if (_fbdiv % 2)
			++_fbdiv;

		_fout = _fbdiv * fin / _prediv;
		if (_fout < fvco_min || _fout > fvco_max)
			continue;

		delta = abs(fout - _fout);
		if (delta < min_delta) {
			best_prediv = _prediv;
			best_fbdiv = _fbdiv;
			min_delta = delta;
			best_freq = _fout;
		}
	}

	if (best_freq) {
		*prediv = best_prediv;
		*fbdiv = best_fbdiv;
	}

	return best_freq * USEC_PER_SEC;
}

static unsigned long mipi_dphy_set_pll(struct rockchip_dsi *dsi,
				       unsigned long rate)
{
	struct mipi_dphy *dphy = &dsi->dphy;
	unsigned long fin, fout;
	u8 prediv;
	u16 fbdiv;

	fin = clk_get_rate(dphy->ref_clk);
	fout = mipi_dphy_pll_round_rate(fin, rate, &prediv, &fbdiv);

	dev_dbg(dsi->dev, "fin=%lu, prediv=%u, fbdiv=%u\n", fin, prediv, fbdiv);

	dphy->prediv = prediv;
	dphy->fbdiv = fbdiv;
	if (dsi->slave) {
		dsi->slave->dphy.prediv = dphy->prediv;
		dsi->slave->dphy.fbdiv = dphy->fbdiv;
	}

	return fout;
}

static const struct regmap_config mipi_dphy_regmap_config = {
	.name = "phy",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x97,
	.fast_io = true,
	.reg_write = testif_write,
	.reg_read = testif_read,
};

static int mipi_dphy_attach(struct rockchip_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct mipi_dphy *dphy = &dsi->dphy;
	int ret;

	dphy->phy = devm_phy_optional_get(dev, "mipi_dphy");
	if (IS_ERR(dphy->phy)) {
		ret = PTR_ERR(dphy->phy);
		dev_err(dev, "failed to get mipi dphy: %d\n", ret);
		return ret;
	}

	if (dphy->phy) {
		dev_dbg(dev, "Use Non-SNPS PHY\n");

		dphy->hs_clk = devm_clk_get(dev, "hs_clk");
		if (IS_ERR(dphy->hs_clk)) {
			dev_err(dev, "failed to get PHY high-speed clock\n");
			return PTR_ERR(dphy->hs_clk);
		}
	} else {
		dev_dbg(dev, "Use SNPS PHY\n");

		dphy->ref_clk = devm_clk_get(dev, "ref");
		if (IS_ERR(dphy->ref_clk)) {
			dev_err(dev, "failed to get PHY reference clock\n");
			return PTR_ERR(dphy->ref_clk);
		}

		/* Check if cfg_clk provided */
		dphy->cfg_clk = devm_clk_get(dev, "phy_cfg");
		if (IS_ERR(dphy->cfg_clk)) {
			if (PTR_ERR(dphy->cfg_clk) != -ENOENT) {
				dev_err(dev, "failed to get phy cfg clock\n");
				return PTR_ERR(dphy->cfg_clk);
			}

			/* Otherwise mark the cfg_clk pointer to NULL */
			dphy->cfg_clk = NULL;
		}

		dphy->regmap = devm_regmap_init(dev, NULL, dsi,
						&mipi_dphy_regmap_config);
		if (IS_ERR(dphy->regmap)) {
			dev_err(dev, "failed to create mipi dphy regmap\n");
			return PTR_ERR(dphy->regmap);
		}
	}

	return 0;
}

static inline void dsi_host_power_up(struct rockchip_dsi *dsi)
{
	regmap_write(dsi->regmap, DSI_PWR_UP, POWER_UP);
}

static inline void dsi_host_reset(struct rockchip_dsi *dsi)
{
	regmap_write(dsi->regmap, DSI_PWR_UP, RESET);
}

static inline void dsi_host_set_vid_mode(struct rockchip_dsi *dsi)
{
	regmap_write(dsi->regmap, DSI_MODE_CFG, VIDEO_MODE);
}

static inline void dsi_host_set_cmd_mode(struct rockchip_dsi *dsi)
{
	regmap_write(dsi->regmap, DSI_MODE_CFG, COMMAND_MODE);
}

static inline void dsi_host_err_to_timer_init(struct rockchip_dsi *dsi)
{
	/* XXX: don't hardcode? */
	regmap_update_bits(dsi->regmap, DSI_CLKMGR_CFG,
			   TO_CLK_DIVIDSION_MASK, TO_CLK_DIVIDSION(10));
	regmap_write(dsi->regmap, DSI_TO_CNT_CFG, HSTX_TO_CNT(0xffff) |
		     LPRX_TO_CNT(0xffff));
}

static inline void dsi_host_escclk_init(struct rockchip_dsi *dsi)
{
	u32 lanebyteclk = dsi->lane_mbps >> 3;
	u32 esc_clk_div;

	esc_clk_div = DIV_ROUND_UP(lanebyteclk, 20);

	regmap_update_bits(dsi->regmap, DSI_CLKMGR_CFG,
			   TX_ESC_CLK_DIVIDSION_MASK,
			   TX_ESC_CLK_DIVIDSION(esc_clk_div));
}

static inline void dsi_host_interrupt_init(struct rockchip_dsi *dsi)
{
	regmap_write(dsi->regmap, DSI_INT_MSK0, 0x1fffff);
	regmap_write(dsi->regmap, DSI_INT_MSK1, 0x1f7f);
}

static inline void dsi_host_presp_to_counter_init(struct rockchip_dsi *dsi)
{
	/*
	 * The values in these registers are measured in number of cycles of
	 * the lanebyteclk clock. Setting a given timeout to 0 disables going
	 * into LP-11 state and timeout for events of that category.
	 */
	regmap_write(dsi->regmap, DSI_BTA_TO_CNT, 0);
	regmap_write(dsi->regmap, DSI_LP_WR_TO_CNT, 0);
	regmap_write(dsi->regmap, DSI_HS_WR_TO_CNT, 0);
	regmap_write(dsi->regmap, DSI_LP_RD_TO_CNT, 0);
	regmap_write(dsi->regmap, DSI_HS_RD_TO_CNT, 0);
}

static inline void dsi_host_pkthdl_init(struct rockchip_dsi *dsi)
{
	/* A peripheral shall implement ECC, and may optionally implement checksum. */
	regmap_write(dsi->regmap, DSI_PCKHDL_CFG, ECC_RX_EN | BTA_EN);
}

static int dsi_host_read_from_fifo(struct rockchip_dsi *dsi,
				   const struct mipi_dsi_msg *msg)
{
	u8 *payload = msg->rx_buf;
	u16 length;
	u32 val;
	int ret;

	ret = genif_wait_rd_cmd_not_busy(dsi);
	if (ret) {
		dev_err(dsi->dev, "entire response is not stored in the FIFO\n");
		return ret;
	}

	/* Receive payload */
	for (length = msg->rx_len; length; length -= 4) {
		ret = genif_wait_r_pld_fifo_not_empty(dsi);
		if (ret) {
			dev_err(dsi->dev, "Read payload FIFO is empty\n");
			return ret;
		}

		regmap_read(dsi->regmap, DSI_GEN_PLD_DATA, &val);

		switch (length) {
		case 3:
			payload[2] = (val >> 16) & 0xff;
			/* Fall through */
		case 2:
			payload[1] = (val >> 8) & 0xff;
			/* Fall through */
		case 1:
			payload[0] = val & 0xff;
			return 0;
		}

		payload[0] = (val >>  0) & 0xff;
		payload[1] = (val >>  8) & 0xff;
		payload[2] = (val >> 16) & 0xff;
		payload[3] = (val >> 24) & 0xff;
		payload += 4;
	}

	return 0;
}

static void dsi_host_vid_mode_timing_config(struct rockchip_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;
	unsigned int lanebyteclk = dsi->lane_mbps >> 3;
	unsigned int dpipclk = vm->pixelclock / USEC_PER_SEC;
	u32 hsa_time, hbp_time, hline_time;
	u32 hsa, hbp, hact, hfp, hline;
	u32 vact, vfp, vbp, vsa;

	vsa = vm->vsync_len;
	vbp = vm->vback_porch;
	vact = vm->vactive;
	vfp = vm->vfront_porch;

	regmap_write(dsi->regmap, DSI_VID_VACTIVE_LINES, V_ACTIVE_LINES(vact));
	regmap_write(dsi->regmap, DSI_VID_VSA_LINES, VSA_LINES(vsa));
	regmap_write(dsi->regmap, DSI_VID_VFP_LINES, VFP_LINES(vfp));
	regmap_write(dsi->regmap, DSI_VID_VBP_LINES, VBP_LINES(vbp));

	hsa = vm->hsync_len;
	hbp = vm->hback_porch;
	hact = vm->hactive;
	hfp = vm->hfront_porch;
	hline = hsa + hbp + hact + hfp;

	hline_time = DIV_ROUND_CLOSEST_ULL(hline * lanebyteclk, dpipclk);
	regmap_write(dsi->regmap, DSI_VID_HLINE_TIME, VID_HLINE_TIME(hline_time));
	hsa_time = DIV_ROUND_CLOSEST_ULL(hsa * lanebyteclk, dpipclk);
	regmap_write(dsi->regmap, DSI_VID_HSA_TIME, VID_HSA_TIME(hsa_time));
	hbp_time = DIV_ROUND_CLOSEST_ULL(hbp * lanebyteclk, dpipclk);
	regmap_write(dsi->regmap, DSI_VID_HBP_TIME, VID_HBP_TIME(hbp_time));

	if (dsi->slave || dsi->master)
		regmap_write(dsi->regmap, DSI_VID_PKT_SIZE,
			     VID_PKT_SIZE(hact / 2));
	else
		regmap_write(dsi->regmap, DSI_VID_PKT_SIZE,
			     VID_PKT_SIZE(hact));
}

static void dsi_host_vid_mode_config(struct rockchip_dsi *dsi)
{
	struct device *dev = dsi->dev;
	u32 val, prop;

	dsi_host_vid_mode_timing_config(dsi);

	val = LP_VACT_EN | LP_VFP_EN | LP_VBP_EN | LP_VSA_EN;

	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO_HFP))
		val |= LP_HFP_EN;
	if (!(dsi->mode_flags & MIPI_DSI_MODE_VIDEO_HBP))
		val |= LP_HBP_EN;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		val |= VID_MODE_TYPE(BURST_MODE);
	else if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		val |= VID_MODE_TYPE(NON_BURST_MODE_SYNC_PULSE);
	else
		val |= VID_MODE_TYPE(NON_BURST_MODE_SYNC_EVENT);

	if (!of_property_read_u32(dev->of_node, "snps,vpg-orientation", &prop))
		val |= VPG_ORIENTATION(prop) | VPG_EN;

	if (!of_property_read_u32(dev->of_node, "snps,vpg-mode", &prop))
		val |= VPG_MODE(prop) | VPG_EN;

	regmap_write(dsi->regmap, DSI_VID_MODE_CFG, val);

	if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		regmap_update_bits(dsi->regmap, DSI_LPCLK_CTRL,
				   AUTO_CLKLANE_CTRL, AUTO_CLKLANE_CTRL);

	if (!(dsi->mode_flags & MIPI_DSI_MODE_EOT_PACKET))
		regmap_update_bits(dsi->regmap, DSI_PCKHDL_CFG,
				   EOTP_TX_EN, EOTP_TX_EN);
}

static void dsi_host_init(struct rockchip_dsi *dsi)
{
	dsi_host_presp_to_counter_init(dsi);
	dsi_host_err_to_timer_init(dsi);
	dsi_host_escclk_init(dsi);
	dsi_host_set_cmd_mode(dsi);
	dsi_host_pkthdl_init(dsi);
	genif_vcid_init(dsi, dsi->channel);
	dsi_host_interrupt_init(dsi);
}

static void dpi_color_coding_config(struct rockchip_dsi *dsi,
				    enum mipi_dsi_pixel_format format)
{
	u8 color_coding;
	bool loosely18_en = false;

	switch (format) {
	case MIPI_DSI_FMT_RGB666:
		color_coding = PIXEL_COLOR_CODING_18BIT_2;
		break;
	case MIPI_DSI_FMT_RGB666_PACKED:
		color_coding = PIXEL_COLOR_CODING_18BIT_1;
		loosely18_en = true;
		break;
	case MIPI_DSI_FMT_RGB565:
		color_coding = PIXEL_COLOR_CODING_16BIT_1;
		break;
	case MIPI_DSI_FMT_RGB888:
	default:
		color_coding = PIXEL_COLOR_CODING_24BIT;
		break;
	}

	regmap_write(dsi->regmap, DSI_DPI_COLOR_CODING,
		     LOOSELY18_EN(loosely18_en) |
		     DPI_COLOR_CODING(color_coding));
}

static inline void dpi_vcid_config(struct rockchip_dsi *dsi, u8 vcid)
{
	regmap_write(dsi->regmap, DSI_DPI_VCID, DPI_VCID(vcid));
}

static void dpi_pol_config(struct rockchip_dsi *dsi)
{
	u32 val = 0;

	if (dsi->vm.flags & DISPLAY_FLAGS_VSYNC_LOW)
		val |= VSYNC_ACTIVE_LOW;
	if (dsi->vm.flags & DISPLAY_FLAGS_HSYNC_LOW)
		val |= HSYNC_ACTIVE_LOW;
	if (dsi->vm.flags & DISPLAY_FLAGS_DE_LOW)
		val |= DATAEN_ACTIVE_LOW;

	regmap_write(dsi->regmap, DSI_DPI_CFG_POL, val);
}

static void dpi_config(struct rockchip_dsi *dsi)
{
	dpi_color_coding_config(dsi, dsi->format);
	dpi_pol_config(dsi);
	dpi_vcid_config(dsi, dsi->channel);
}

static unsigned long rockchip_dsi_calc_link_bandwidth(struct rockchip_dsi *dsi)
{
	unsigned long min_freq = dsi->soc_data->min_bit_rate_per_lane;
	unsigned long max_freq = dsi->soc_data->max_bit_rate_per_lane;
	unsigned long fpclk, req_freq, tmp;
	unsigned int lanes;
	int bpp;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (bpp < 0)
		bpp = 24;
	lanes = dsi->slave ? dsi->lanes * 2 : dsi->lanes;
	fpclk = dsi->vm.pixelclock / USEC_PER_SEC;

	/*
	 * For video Burst mode, The DSI output bandwidth should be higher than
	 * the DPI system interface input bandwidth in a relation that enables
	 * the link to go to low-power once per line.
	 */
	tmp = fpclk * bpp * 12 / 10 / lanes;
	if (tmp < min_freq || tmp > max_freq)
		req_freq = max_freq;
	else
		req_freq = tmp;

	return req_freq * USEC_PER_SEC;
}

static int rockchip_dsi_compute_transmission_timing(struct rockchip_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct mipi_dphy *dphy = &dsi->dphy;
	unsigned long rate;
	u32 bw;

	/* XXX: optional override of the desired bandwidth */
	if (of_property_read_u32(dev->of_node, "snps,bit-rate-per-lane", &bw))
		bw = rockchip_dsi_calc_link_bandwidth(dsi);

	if (dphy->phy) {
		rate = clk_round_rate(dphy->hs_clk, bw);
		clk_set_rate(dphy->hs_clk, rate);
	} else {
		rate = mipi_dphy_set_pll(dsi, bw);
	}

	dsi->lane_mbps = rate / USEC_PER_SEC;
	if (dsi->slave)
		dsi->slave->lane_mbps = dsi->lane_mbps;

	dev_info(dsi->dev, "final DSI-Link bandwidth: %u x %d Mbps\n",
		 dsi->lane_mbps, dsi->slave ? dsi->lanes * 2 : dsi->lanes);

	return 0;
}

static void rockchip_dsi_external_bridge_power_on(struct rockchip_dsi *dsi)
{
	if (dsi->enable_gpio) {
		gpiod_direction_output(dsi->enable_gpio, 1);
		usleep_range(1000, 2000);
	}

	if (dsi->reset_gpio) {
		gpiod_direction_output(dsi->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpiod_direction_output(dsi->reset_gpio, 1);
		usleep_range(1000, 2000);
		gpiod_direction_output(dsi->reset_gpio, 0);
		usleep_range(1000, 2000);
	}
}

static void rockchip_dsi_external_bridge_power_off(struct rockchip_dsi *dsi)
{
	if (dsi->reset_gpio)
		gpiod_direction_output(dsi->reset_gpio, 1);

	if (dsi->enable_gpio)
		gpiod_direction_output(dsi->enable_gpio, 0);
}

static void rockchip_dsi_pre_enable(struct rockchip_dsi *dsi)
{
	rockchip_dsi_external_bridge_power_on(dsi);
	pm_runtime_get_sync(dsi->dev);
	dsi_host_reset(dsi);
	dsi_host_init(dsi);
	mipi_dphy_init(dsi);
	mipi_dphy_hstt_config(dsi);
	mipi_dphy_if_config(dsi);
	ppi_txrequestclkhs_assert(dsi);
	mipi_dphy_power_on(dsi);
	dsi_host_power_up(dsi);

	if (dsi->slave)
		rockchip_dsi_pre_enable(dsi->slave);
}

static ssize_t rockchip_dsi_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct rockchip_dsi *dsi = host_to_dsi(host);
	struct mipi_dsi_packet packet;
	u32 val;
	int ret;

	if (msg->flags & MIPI_DSI_MSG_USE_LPM) {
		val = CMD_XFER_TYPE_LP;
	} else {
		val = CMD_XFER_TYPE_HS;
		ppi_txrequestclkhs_assert(dsi);
	}

	/* enables the acknowledge request after each packet transmission */
	if (msg->flags & MIPI_DSI_MSG_REQ_ACK ||
	    of_property_read_bool(dsi->dev->of_node, "snps,ack-request"))
		val |= ACK_RQST_EN;

	regmap_write(dsi->regmap, DSI_CMD_MODE_CFG, val);

	/* create a packet to the DSI protocol */
	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret) {
		dev_err(dsi->dev, "failed to create packet: %d\n", ret);
		return ret;
	}

	/* Send payload */
	while (packet.payload_length >= 4) {
		/*
		 * Alternatively, you can always keep the FIFO nearly full by
		 * monitoring the FIFO state until it is not full, and then
		 * write a single word of data. This solution is more resource
		 * consuming but it simultaneously avoids FIFO starvation,
		 * making it possible to use FIFO sizes smaller than the amount
		 * of data of the longest packet to be written.
		 */
		ret = genif_wait_w_pld_fifo_not_full(dsi);
		if (ret) {
			dev_err(dsi->dev, "Write payload FIFO is full\n");
			return ret;
		}

		val = get_unaligned_le32(packet.payload);
		regmap_write(dsi->regmap, DSI_GEN_PLD_DATA, val);
		packet.payload += 4;
		packet.payload_length -= 4;
	}

	val = 0;
	switch (packet.payload_length) {
	case 3:
		val |= packet.payload[2] << 16;
		/* Fall through */
	case 2:
		val |= packet.payload[1] << 8;
		/* Fall through */
	case 1:
		val |= packet.payload[0];
		regmap_write(dsi->regmap, DSI_GEN_PLD_DATA, val);
		break;
	}

	ret = genif_wait_cmd_fifo_not_full(dsi);
	if (ret) {
		dev_err(dsi->dev, "Command FIFO is full\n");
		return ret;
	}

	/* Send packet header */
	val = get_unaligned_le32(packet.header);
	regmap_write(dsi->regmap, DSI_GEN_HDR, val);

	ret = genif_wait_w_fifo_is_empty(dsi);
	if (ret) {
		dev_err(dsi->dev, "Write payload FIFO is not empty\n");
		return ret;
	}

	if (msg->rx_len) {
		/*
		 * Since this is a read command, BTA shall be asserted by the
		 * host processor following completion of the transmission
		 */
		ppi_turnrequest_assert(dsi);
		ret = dsi_host_read_from_fifo(dsi, msg);
		if (ret)
			return ret;
		ppi_turnrequest_deassert(dsi);
	}

	return 0;
}

static int rockchip_dsi_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct rockchip_dsi *dsi = host_to_dsi(host);

	if (device->lanes < 1 || device->lanes > 8)
		return -EINVAL;

	dsi->client = device->dev.of_node;
	dsi->lanes = device->lanes;
	dsi->channel = device->channel;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	return 0;
}

static int rockchip_dsi_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *device)
{
	struct rockchip_dsi *dsi = host_to_dsi(host);

	dsi->panel = NULL;

	if (dsi->connector.dev)
		drm_helper_hpd_irq_event(dsi->connector.dev);

	return 0;
}

static const struct mipi_dsi_host_ops rockchip_dsi_host_ops = {
	.attach	  = rockchip_dsi_host_attach,
	.detach	  = rockchip_dsi_host_detach,
	.transfer = rockchip_dsi_host_transfer,
};

static void rockchip_dsi_encoder_mode_set(struct drm_encoder *encoder,
					  struct drm_display_mode *mode,
					  struct drm_display_mode *adj)
{
	struct rockchip_dsi *dsi = encoder_to_dsi(encoder);

	drm_display_mode_to_videomode(adj, &dsi->vm);
	if (dsi->slave)
		drm_display_mode_to_videomode(adj, &dsi->slave->vm);
}

static void rockchip_dsi_vop_routing(struct rockchip_dsi *dsi)
{
	struct device_node *np = dsi->dev->of_node;
	int pipe;

	pipe = drm_of_encoder_active_endpoint_id(np, &dsi->encoder);
	grf_write(dsi, VOPSEL, pipe);

	if (dsi->slave)
		grf_write(dsi->slave, VOPSEL, pipe);
}

static void rockchip_dsi_enable(struct rockchip_dsi *dsi)
{
	dsi_host_reset(dsi);
	dpi_config(dsi);
	dsi_host_vid_mode_config(dsi);
	ppi_txrequestclkhs_assert(dsi);
	dsi_host_set_vid_mode(dsi);
	dsi_host_power_up(dsi);

	if (dsi->slave)
		rockchip_dsi_enable(dsi->slave);
}

static void rockchip_dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct rockchip_dsi *dsi = encoder_to_dsi(encoder);

	rockchip_dsi_compute_transmission_timing(dsi);
	rockchip_dsi_vop_routing(dsi);

	rockchip_dsi_pre_enable(dsi);

	if (dsi->panel)
		drm_panel_prepare(dsi->panel);

	rockchip_dsi_enable(dsi);

	if (dsi->panel)
		drm_panel_enable(dsi->panel);
}

static void rockchip_dsi_disable(struct rockchip_dsi *dsi)
{
	dsi_host_set_cmd_mode(dsi);
	ppi_txrequestclkhs_deassert(dsi);
	ppi_forcetxstopmode_assert(dsi);

	if (dsi->slave)
		rockchip_dsi_disable(dsi->slave);
}

static void rockchip_dsi_post_disable(struct rockchip_dsi *dsi)
{
	rockchip_dsi_external_bridge_power_off(dsi);
	dsi_host_reset(dsi);
	mipi_dphy_power_off(dsi);
	pm_runtime_put(dsi->dev);

	if (dsi->slave)
		rockchip_dsi_post_disable(dsi->slave);
}

static void rockchip_dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct rockchip_dsi *dsi = encoder_to_dsi(encoder);

	if (dsi->panel)
		drm_panel_disable(dsi->panel);

	rockchip_dsi_disable(dsi);

	if (dsi->panel)
		drm_panel_unprepare(dsi->panel);

	rockchip_dsi_post_disable(dsi);
}

static int
rockchip_dsi_encoder_atomic_check(struct drm_encoder *encoder,
				  struct drm_crtc_state *crtc_state,
				  struct drm_connector_state *conn_state)
{
	struct rockchip_crtc_state *s = to_rockchip_crtc_state(crtc_state);
	struct rockchip_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_connector *connector = conn_state->connector;
	struct drm_display_info *info = &connector->display_info;

	switch (dsi->format) {
	case MIPI_DSI_FMT_RGB888:
		s->output_mode = ROCKCHIP_OUT_MODE_P888;
		break;
	case MIPI_DSI_FMT_RGB666:
		s->output_mode = ROCKCHIP_OUT_MODE_P666;
		break;
	case MIPI_DSI_FMT_RGB565:
		s->output_mode = ROCKCHIP_OUT_MODE_P565;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	s->output_type = DRM_MODE_CONNECTOR_DSI;

	if (dsi->slave)
		s->output_flags |= ROCKCHIP_OUTPUT_DSI_DUAL_CHANNEL;

	if (IS_DSI1(dsi))
		s->output_flags |= ROCKCHIP_OUTPUT_DSI_DUAL_LINK;

	if (info->num_bus_formats)
		s->bus_format = info->bus_formats[0];

	return 0;
}

static const struct drm_encoder_helper_funcs
rockchip_dsi_encoder_helper_funcs = {
	.mode_set = rockchip_dsi_encoder_mode_set,
	.atomic_check = rockchip_dsi_encoder_atomic_check,
	.enable = rockchip_dsi_encoder_enable,
	.disable = rockchip_dsi_encoder_disable,
};

static const struct drm_encoder_funcs rockchip_dsi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int rockchip_dsi_connector_get_modes(struct drm_connector *connector)
{
	struct rockchip_dsi *dsi = connector_to_dsi(connector);

	return drm_panel_get_modes(dsi->panel);
}

static struct drm_encoder *
rockchip_dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct rockchip_dsi *dsi = connector_to_dsi(connector);

	return &dsi->encoder;
}

static int
rockchip_dsi_connector_loader_protect(struct drm_connector *connector, bool on)
{
	struct rockchip_dsi *dsi = connector_to_dsi(connector);

	if (dsi->panel)
		drm_panel_loader_protect(dsi->panel, on);

	if (on)
		pm_runtime_get_sync(dsi->dev);
	else
		pm_runtime_put(dsi->dev);

	return 0;
}

static const struct drm_connector_helper_funcs
rockchip_dsi_connector_helper_funcs = {
	.get_modes = rockchip_dsi_connector_get_modes,
	.best_encoder = rockchip_dsi_connector_best_encoder,
	.loader_protect = rockchip_dsi_connector_loader_protect,
};

static void rockchip_dsi_connector_destroy(struct drm_connector *connector)
{
	struct rockchip_dsi *dsi = connector_to_dsi(connector);

	if (dsi->panel)
		drm_panel_detach(dsi->panel);

	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status
rockchip_dsi_connector_detect(struct drm_connector *connector, bool force)
{
	struct rockchip_dsi *dsi = connector_to_dsi(connector);

	if (dsi->panel)
		return connector_status_connected;

	return connector_status_disconnected;
}

static const struct drm_connector_funcs rockchip_dsi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = rockchip_dsi_connector_detect,
	.destroy = rockchip_dsi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int rockchip_dsi_connector_init(struct drm_device *drm,
				       struct rockchip_dsi *dsi)
{
	struct drm_encoder *encoder = &dsi->encoder;
	struct drm_connector *connector = &dsi->connector;
	int ret;

	ret = drm_connector_init(drm, connector, &rockchip_dsi_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector,
				 &rockchip_dsi_connector_helper_funcs);
	connector->port = dsi->dev->of_node;
	drm_mode_connector_attach_encoder(connector, encoder);

	ret = drm_panel_attach(dsi->panel, connector);
	if (ret) {
		DRM_ERROR("Failed to attach panel to drm\n");
		return ret;
	}

	return 0;
}

static int rockchip_dsi_dual_channel_probe(struct rockchip_dsi *dsi)
{
	struct device_node *np;

	np = of_parse_phandle(dsi->dev->of_node, "rockchip,dual-channel", 0);
	if (np) {
		struct platform_device *secondary = of_find_device_by_node(np);

		dsi->slave = platform_get_drvdata(secondary);
		of_node_put(np);

		if (!dsi->slave)
			return -EPROBE_DEFER;

		dsi->slave->master = dsi;

		dsi->lanes /= 2;
		dsi->slave->lanes = dsi->lanes;
		dsi->slave->channel = dsi->channel;
		dsi->slave->format = dsi->format;
		dsi->slave->mode_flags = dsi->mode_flags;
	}

	return 0;
}

static int rockchip_dsi_bind(struct device *dev, struct device *master,
			     void *data)
{
	struct device_node *np = dev->of_node;
	struct drm_device *drm = data;
	struct rockchip_dsi *dsi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dsi->encoder;
	int ret;

	ret = rockchip_dsi_dual_channel_probe(dsi);
	if (ret)
		return ret;

	if (dsi->master)
		return 0;

	dsi->panel = of_drm_find_panel(dsi->client);
	if (!dsi->panel)
		dsi->bridge = of_drm_find_bridge(dsi->client);

	if (!dsi->panel && !dsi->bridge) {
		DRM_INFO("Waiting for panel/bridge driver\n");
		return -EPROBE_DEFER;
	}

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, np);

	drm_encoder_init(drm, encoder, &rockchip_dsi_encoder_funcs,
			 DRM_MODE_ENCODER_DSI, NULL);
	drm_encoder_helper_add(encoder, &rockchip_dsi_encoder_helper_funcs);

	if (dsi->bridge) {
		/*
		 * If there's a bridge, attach to it and
		 * let it create the connector
		 */
		dsi->bridge->driver_private = &dsi->host;
		dsi->bridge->encoder = encoder;
		ret = drm_bridge_attach(drm, dsi->bridge);
		if (ret) {
			DRM_ERROR("Failed to attach bridge to drm\n");
			return ret;
		}
		encoder->bridge = dsi->bridge;
	} else {
		/* Otherwise create our own connector and attach to a panel */
		ret = rockchip_dsi_connector_init(drm, dsi);
		if (ret)
			return ret;
	}

	pm_runtime_enable(dev);
	if (dsi->slave)
		pm_runtime_enable(dsi->slave->dev);

	enable_irq(dsi->irq);

	return 0;
}

static void rockchip_dsi_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct rockchip_dsi *dsi = dev_get_drvdata(dev);

	pm_runtime_disable(dev);
	if (dsi->slave)
		pm_runtime_disable(dsi->slave->dev);

	drm_encoder_cleanup(&dsi->encoder);

	/* Skip connector cleanup if creation was delegated to the bridge */
	if (dsi->connector.dev)
		drm_connector_cleanup(&dsi->connector);
}

static const struct component_ops rockchip_dsi_component_ops = {
	.bind	= rockchip_dsi_bind,
	.unbind	= rockchip_dsi_unbind,
};

static const char * const dphy_error[] = {
	"ErrEsc escape entry error from Lane 0",
	"ErrSyncEsc low-power data transmission synchronization error from Lane 0",
	"the ErrControl error from Lane 0",
	"LP0 contention error ErrContentionLP0 from Lane 0",
	"LP1 contention error ErrContentionLP1 from Lane 0",
};

static const char * const ack_with_err[] = {
	"the SoT error from the Acknowledge error report",
	"the SoT Sync error from the Acknowledge error report",
	"the EoT Sync error from the Acknowledge error report",
	"the Escape Mode Entry Command error from the Acknowledge error report",
	"the LP Transmit Sync error from the Acknowledge error report",
	"the Peripheral Timeout error from the Acknowledge Error report",
	"the False Control error from the Acknowledge error report",
	"the reserved (specific to device) from the Acknowledge error report",
	"the ECC error, single-bit (detected and corrected) from the Acknowledge error report",
	"the ECC error, multi-bit (detected, not corrected) from the Acknowledge error report",
	"the checksum error (long packet only) from the Acknowledge error report",
	"the not recognized DSI data type from the Acknowledge error report",
	"the DSI VC ID Invalid from the Acknowledge error report",
	"the invalid transmission length from the Acknowledge error report",
	"the reserved (specific to device) from the Acknowledge error report"
	"the DSI protocol violation from the Acknowledge error report",
};

static const char * const error_report[] = {
	"Host reports that the configured timeout counter for the high-speed transmission has expired",
	"Host reports that the configured timeout counter for the low-power reception has expired",
	"Host reports that a received packet contains a single bit error",
	"Host reports that a received packet contains multiple ECC errors",
	"Host reports that a received long packet has a CRC error in its payload",
	"Host receives a transmission that does not end in the expected by boundaries",
	"Host receives a transmission that does not end with an End of Transmission packet",
	"An overflow occurs in the DPI pixel payload FIFO",
	"An overflow occurs in the Generic command FIFO",
	"An overflow occurs in the Generic write payload FIFO",
	"An underflow occurs in the Generic write payload FIFO",
	"An underflow occurs in the Generic read FIFO",
	"An overflow occurs in the Generic read FIFO",
};

static irqreturn_t rockchip_dsi_irq_thread(int irq, void *dev_id)
{
	struct rockchip_dsi *dsi = dev_id;
	u32 int_st0, int_st1;
	int i;

	regmap_read(dsi->regmap, DSI_INT_ST0, &int_st0);
	regmap_read(dsi->regmap, DSI_INT_ST1, &int_st1);

	for (i = 0; i < ARRAY_SIZE(ack_with_err); i++)
		if (int_st0 & BIT(i))
			dev_dbg(dsi->dev, "%s\n", ack_with_err[i]);

	for (i = 0; i < ARRAY_SIZE(dphy_error); i++)
		if (int_st0 & BIT(16 + i))
			dev_dbg(dsi->dev, "%s\n", dphy_error[i]);

	for (i = 0; i < ARRAY_SIZE(error_report); i++)
		if (int_st1 & BIT(i))
			dev_dbg(dsi->dev, "%s\n", error_report[i]);

	return IRQ_HANDLED;
}

static int rockchip_dsi_parse_dt(struct rockchip_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *np = dev->of_node;
	struct device_node *endpoint, *remote = NULL;

	endpoint = of_graph_get_endpoint_by_regs(np, 1, -1);
	if (endpoint) {
		remote = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
		if (!remote) {
			dev_err(dev, "no panel/bridge connected\n");
			return -ENODEV;
		}
		of_node_put(remote);
	}

	dsi->client = remote;

	return 0;
}

static const struct regmap_config rockchip_dsi_regmap_config = {
	.name = "host",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = DSI_MAX_REGISGER,
};

static int rockchip_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rockchip_dsi *dsi;
	struct resource *res;
	int of_id;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	of_id = of_alias_get_id(np, "dsi");
	if (of_id < 0)
		of_id = 0;

	dsi->dev = dev;
	dsi->soc_data = of_device_get_match_data(dev);
	dsi->id = of_id;
	dsi->grf_desc = of_id ? dsi->soc_data->dsi1_grf_desc :
				dsi->soc_data->dsi0_grf_desc;
	platform_set_drvdata(pdev, dsi);

	ret = rockchip_dsi_parse_dt(dsi);
	if (ret) {
		dev_err(dev, "failed to parse DT\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(dsi->regs))
		return PTR_ERR(dsi->regs);

	dsi->regmap = devm_regmap_init_mmio_clk(dev, "pclk", dsi->regs,
						&rockchip_dsi_regmap_config);
	if (IS_ERR(dsi->regmap)) {
		dev_err(dev, "failed to init register map\n");
		return PTR_ERR(dsi->regmap);
	}

	dsi->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(dsi->grf)) {
		dev_err(dev, "failed to get grf regmap\n");
		return PTR_ERR(dsi->grf);
	}

	dsi->pclk = devm_clk_get(dev, "pclk");
	if (IS_ERR(dsi->pclk)) {
		dev_err(dev, "failed to get apb clock\n");
		return PTR_ERR(dsi->pclk);
	}

	dsi->rst = devm_reset_control_get(dev, "apb");
	if (IS_ERR(dsi->rst)) {
		dev_err(dev, "failed to get apb reset\n");
		return PTR_ERR(dsi->rst);
	}

	ret = mipi_dphy_attach(dsi);
	if (ret)
		return ret;

	dsi->irq = platform_get_irq(pdev, 0);
	if (dsi->irq < 0) {
		dev_err(dev, "failed to get IRQ resource\n");
		return dsi->irq;
	}

	irq_set_status_flags(dsi->irq, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(dev, dsi->irq, NULL,
					rockchip_dsi_irq_thread,
					IRQF_ONESHOT, dev_name(dev), dsi);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	dsi->enable_gpio = devm_gpiod_get_optional(dev, "enable", 0);
	if (IS_ERR(dsi->enable_gpio)) {
		ret = PTR_ERR(dsi->enable_gpio);
		dev_err(dev, "failed to request enable GPIO: %d\n", ret);
		return ret;
	}

	dsi->reset_gpio = devm_gpiod_get_optional(dev, "reset", 0);
	if (IS_ERR(dsi->reset_gpio)) {
		ret = PTR_ERR(dsi->reset_gpio);
		dev_err(dev, "failed to request reset GPIO: %d\n", ret);
		return ret;
	}

	dsi->host.ops = &rockchip_dsi_host_ops;
	dsi->host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->host);
	if (ret) {
		dev_err(dev, "failed to register DSI host: %d\n", ret);
		return ret;
	}

	return component_add(dev, &rockchip_dsi_component_ops);
}

static int rockchip_dsi_remove(struct platform_device *pdev)
{
	struct rockchip_dsi *dsi = platform_get_drvdata(pdev);

	component_del(&pdev->dev, &rockchip_dsi_component_ops);
	mipi_dsi_host_unregister(&dsi->host);

	return 0;
}

static int rockchip_dsi_suspend(struct device *dev)
{
	struct rockchip_dsi *dsi = dev_get_drvdata(dev);
	struct mipi_dphy *dphy = &dsi->dphy;

	if (dphy->phy) {
		clk_disable_unprepare(dphy->hs_clk);
	} else {
		clk_disable_unprepare(dphy->ref_clk);
		clk_disable_unprepare(dphy->cfg_clk);
	}

	reset_control_assert(dsi->rst);
	usleep_range(1000, 2000);
	clk_disable_unprepare(dsi->pclk);

	return 0;
}

static int rockchip_dsi_resume(struct device *dev)
{
	struct rockchip_dsi *dsi = dev_get_drvdata(dev);
	struct mipi_dphy *dphy = &dsi->dphy;

	clk_prepare_enable(dsi->pclk);
	usleep_range(1000, 2000);
	reset_control_deassert(dsi->rst);

	if (dphy->phy) {
		clk_prepare_enable(dphy->hs_clk);
	} else {
		clk_prepare_enable(dphy->ref_clk);
		clk_prepare_enable(dphy->cfg_clk);
	}

	return 0;
}

static const u32 rk3288_dsi0_grf_desc[NUM_GRF_DESC] = {
	[DPICOLORM]	  = GRF_DESC(0x025c,  8, 8),
	[DPISHUTDN]	  = GRF_DESC(0x025c,  7, 7),
	[VOPSEL]	  = GRF_DESC(0x025c,  6, 6),
	[FORCETXSTOPMODE] = GRF_DESC(0x0264, 11, 8),
	[FORCERXMODE]	  = GRF_DESC(0x0264,  7, 4),
	[TURNDISABLE]	  = GRF_DESC(0x0264,  3, 0),
	[TURNREQUEST]	  = GRF_DESC(0x03a4, 10, 8),
	[DPIUPDATECFG]	  = GRF_DESC(0x03a8,  0, 0),
};

static const u32 rk3288_dsi1_grf_desc[NUM_GRF_DESC] = {
	[DPICOLORM]	  = GRF_DESC(0x025c, 11, 11),
	[DPISHUTDN]	  = GRF_DESC(0x025c, 10, 10),
	[VOPSEL]	  = GRF_DESC(0x025c,  9,  9),
	[ENABLE_N]	  = GRF_DESC(0x0268, 15, 12),
	[FORCETXSTOPMODE] = GRF_DESC(0x0268, 11,  8),
	[FORCERXMODE]	  = GRF_DESC(0x0268,  7,  4),
	[TURNDISABLE]	  = GRF_DESC(0x0268,  3,  0),
	[BASEDIR]	  = GRF_DESC(0x027c, 15, 15),
	[MASTERSLAVEZ]	  = GRF_DESC(0x027c, 14, 14),
	[ENABLECLK]	  = GRF_DESC(0x027c, 12, 12),
	[TURNREQUEST]	  = GRF_DESC(0x03a4,  7,  4),
	[DPIUPDATECFG]	  = GRF_DESC(0x03a8,  1,  1),
};

static const struct rockchip_dsi_soc_data rk3288_dsi_soc_data = {
	.min_bit_rate_per_lane = 80,
	.max_bit_rate_per_lane = 1500,
	.dsi0_grf_desc = rk3288_dsi0_grf_desc,
	.dsi1_grf_desc = rk3288_dsi1_grf_desc,
};

static const u32 rk3366_dsi_grf_desc[NUM_GRF_DESC] = {
	[VOPSEL]	  = GRF_DESC(0x0400,  2, 2),
	[DPIUPDATECFG]	  = GRF_DESC(0x0410,  9, 9),
	[DPICOLORM]	  = GRF_DESC(0x0410,  3, 3),
	[DPISHUTDN]	  = GRF_DESC(0x0410,  2, 2),
	[FORCETXSTOPMODE] = GRF_DESC(0x0414, 10, 7),
	[FORCERXMODE]	  = GRF_DESC(0x0414,  6, 6),
	[TURNDISABLE]	  = GRF_DESC(0x0414,  5, 5),
};

static const struct rockchip_dsi_soc_data rk3366_dsi_soc_data = {
	.min_bit_rate_per_lane = 80,
	.max_bit_rate_per_lane = 1000,
	.dsi0_grf_desc = rk3366_dsi_grf_desc,
};

static const u32 rk3368_dsi_grf_desc[NUM_GRF_DESC] = {
	[DPIUPDATECFG]	  = GRF_DESC(0x0418,  7, 7),
	[DPICOLORM]	  = GRF_DESC(0x0418,  3, 3),
	[DPISHUTDN]	  = GRF_DESC(0x0418,  2, 2),
	[FORCETXSTOPMODE] = GRF_DESC(0x041c, 10, 7),
	[FORCERXMODE]	  = GRF_DESC(0x041c,  6, 6),
	[TURNDISABLE]	  = GRF_DESC(0x041c,  5, 5),
};

static const struct rockchip_dsi_soc_data rk3368_dsi_soc_data = {
	.min_bit_rate_per_lane = 80,
	.max_bit_rate_per_lane = 1000,
	.dsi0_grf_desc = rk3368_dsi_grf_desc,
};

static const u32 rk3399_dsi0_grf_desc[NUM_GRF_DESC] = {
	[DPIUPDATECFG]	  = GRF_DESC(0x6224, 15, 15),
	[DPISHUTDN]	  = GRF_DESC(0x6224, 14, 14),
	[DPICOLORM]	  = GRF_DESC(0x6224, 13, 13),
	[VOPSEL]	  = GRF_DESC(0x6250,  0,  0),
	[TURNREQUEST]	  = GRF_DESC(0x6258, 15, 12),
	[TURNDISABLE]	  = GRF_DESC(0x6258, 11,  8),
	[FORCETXSTOPMODE] = GRF_DESC(0x6258,  7,  4),
	[FORCERXMODE]	  = GRF_DESC(0x6258,  3,  0),
};

static const u32 rk3399_dsi1_grf_desc[NUM_GRF_DESC] = {
	[VOPSEL]	  = GRF_DESC(0x6250,  4,  4),
	[DPIUPDATECFG]	  = GRF_DESC(0x6250,  3,  3),
	[DPISHUTDN]	  = GRF_DESC(0x6250,  2,  2),
	[DPICOLORM]	  = GRF_DESC(0x6250,  1,  1),
	[TURNDISABLE]	  = GRF_DESC(0x625c, 15, 12),
	[FORCETXSTOPMODE] = GRF_DESC(0x625c, 11,  8),
	[FORCERXMODE]	  = GRF_DESC(0x625c,  7,  4),
	[ENABLE_N]	  = GRF_DESC(0x625c,  3,  0),
	[MASTERSLAVEZ]	  = GRF_DESC(0x6260,  7,  7),
	[ENABLECLK]	  = GRF_DESC(0x6260,  6,  6),
	[BASEDIR]	  = GRF_DESC(0x6260,  5,  5),
	[TURNREQUEST]	  = GRF_DESC(0x6260,  3,  0),
};

static const struct rockchip_dsi_soc_data rk3399_dsi_soc_data = {
	.min_bit_rate_per_lane = 80,
	.max_bit_rate_per_lane = 1500,
	.dsi0_grf_desc = rk3399_dsi0_grf_desc,
	.dsi1_grf_desc = rk3399_dsi1_grf_desc,
};

static const struct of_device_id rockchip_dsi_of_match[] = {
	{ .compatible = "rockchip,rk3288-dsi", .data = &rk3288_dsi_soc_data },
	{ .compatible = "rockchip,rk3366-dsi", .data = &rk3366_dsi_soc_data },
	{ .compatible = "rockchip,rk3368-dsi", .data = &rk3368_dsi_soc_data },
	{ .compatible = "rockchip,rk3399-dsi", .data = &rk3399_dsi_soc_data },
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_dsi_of_match);

static const struct dev_pm_ops rockchip_dsi_pm_ops = {
	SET_RUNTIME_PM_OPS(rockchip_dsi_suspend, rockchip_dsi_resume, NULL)
};

static struct platform_driver rockchip_dsi_driver = {
	.driver = {
		.name = "rockchip-dsi",
		.pm = &rockchip_dsi_pm_ops,
		.of_match_table = of_match_ptr(rockchip_dsi_of_match),
	},
	.probe	= rockchip_dsi_probe,
	.remove	= rockchip_dsi_remove,
};
module_platform_driver(rockchip_dsi_driver);

MODULE_AUTHOR("Wyon Bi <bivvy.bi@rock-chips.com>");
MODULE_DESCRIPTION("Synopsys DesignWare Cores MIPI-DSI driver");
MODULE_LICENSE("GPL v2");
