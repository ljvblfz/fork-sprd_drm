/*
 * Copyright (c) 2016 Linaro Limited.
 * Copyright (c) 2014-2016 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __KIRIN_DPE_REG_H__
#define __KIRIN_DPE_REG_H__

#include <linux/delay.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/bug.h>

#include <linux/sprd_ion.h>
//#include <linux/hisi/hisi_ion.h>

/*******************************************************************************
**
*/
enum dss_chn_idx {
	DSS_RCHN_NONE = -1,
	DSS_RCHN_D2 = 0,
	DSS_RCHN_D3,
	DSS_RCHN_V0,
	DSS_RCHN_G0,
	DSS_RCHN_V1,
	DSS_RCHN_G1,
	DSS_RCHN_D0,
	DSS_RCHN_D1,

	DSS_WCHN_W0,
	DSS_WCHN_W1,

	DSS_CHN_MAX,

	DSS_RCHN_V2 = DSS_CHN_MAX,  /*for copybit, only supported in chicago*/
	DSS_WCHN_W2,

	DSS_COPYBIT_MAX,
};

enum dss_channel {
	DSS_CH1 = 0,	/* channel 1 for primary plane */
	DSS_CH_NUM
};

#define PRIMARY_CH	DSS_CH1 /* primary plane */

typedef struct dss_rect {
	s32 x;
	s32 y;
	s32 w;
	s32 h;
} dss_rect_t;

typedef struct dss_rect_ltrb {
	s32 left;
	s32 top;
	s32 right;
	s32 bottom;
} dss_rect_ltrb_t;

enum {
	DSI_1_LANES = 0,
	DSI_2_LANES,
	DSI_3_LANES,
	DSI_4_LANES,
};

enum dss_ovl_idx {
	DSS_OVL0 = 0,
	DSS_OVL1,
	DSS_OVL2,
	DSS_OVL3,
	DSS_OVL_IDX_MAX,
};

#define DSS_WCH_MAX  (2)

typedef struct dss_img {
	uint32_t format;
	uint32_t width;
	uint32_t height;
	uint32_t bpp;		/* bytes per pixel */
	uint32_t buf_size;
	uint32_t stride;
	uint32_t stride_plane1;
	uint32_t stride_plane2;
	uint64_t phy_addr;
	uint64_t vir_addr;
	uint32_t offset_plane1;
	uint32_t offset_plane2;

	uint64_t afbc_header_addr;
	uint64_t afbc_payload_addr;
	uint32_t afbc_header_stride;
	uint32_t afbc_payload_stride;
	uint32_t afbc_scramble_mode;
	uint32_t mmbuf_base;
	uint32_t mmbuf_size;

	uint32_t mmu_enable;
	uint32_t csc_mode;
	uint32_t secure_mode;
	int32_t shared_fd;
	uint32_t reserved0;
} dss_img_t;

typedef struct drm_dss_layer {
	dss_img_t img;
	dss_rect_t src_rect;
	dss_rect_t src_rect_mask;
	dss_rect_t dst_rect;
	uint32_t transform;
	int32_t blending;
	uint32_t glb_alpha;
	uint32_t color;		/* background color or dim color */
	int32_t layer_idx;
	int32_t chn_idx;
	uint32_t need_cap;
	int32_t acquire_fence;
} drm_dss_layer_t;

enum lcd_orientation {
	LCD_LANDSCAPE = 0,
	LCD_PORTRAIT,
};

enum lcd_format {
	LCD_RGB888 = 0,
	LCD_RGB101010,
	LCD_RGB565,
};

enum lcd_rgb_order {
	LCD_RGB = 0,
	LCD_BGR,
};

enum dss_addr {
	DSS_ADDR_PLANE0 = 0,
	DSS_ADDR_PLANE1,
	DSS_ADDR_PLANE2,
};

enum dss_transform {
	DSS_TRANSFORM_NOP = 0x0,
	DSS_TRANSFORM_FLIP_H = 0x01,
	DSS_TRANSFORM_FLIP_V = 0x02,
	DSS_TRANSFORM_ROT = 0x04,
};

enum dss_dfc_format {
	DFC_PIXEL_FORMAT_RGB_565 = 0,
	DFC_PIXEL_FORMAT_XRGB_4444,
	DFC_PIXEL_FORMAT_ARGB_4444,
	DFC_PIXEL_FORMAT_XRGB_5551,
	DFC_PIXEL_FORMAT_ARGB_5551,
	DFC_PIXEL_FORMAT_XRGB_8888,
	DFC_PIXEL_FORMAT_ARGB_8888,
	DFC_PIXEL_FORMAT_BGR_565,
	DFC_PIXEL_FORMAT_XBGR_4444,
	DFC_PIXEL_FORMAT_ABGR_4444,
	DFC_PIXEL_FORMAT_XBGR_5551,
	DFC_PIXEL_FORMAT_ABGR_5551,
	DFC_PIXEL_FORMAT_XBGR_8888,
	DFC_PIXEL_FORMAT_ABGR_8888,

	DFC_PIXEL_FORMAT_YUV444,
	DFC_PIXEL_FORMAT_YVU444,
	DFC_PIXEL_FORMAT_YUYV422,
	DFC_PIXEL_FORMAT_YVYU422,
	DFC_PIXEL_FORMAT_VYUY422,
	DFC_PIXEL_FORMAT_UYVY422,
};

enum dss_dma_format {
	DMA_PIXEL_FORMAT_RGB_565 = 0,
	DMA_PIXEL_FORMAT_ARGB_4444,
	DMA_PIXEL_FORMAT_XRGB_4444,
	DMA_PIXEL_FORMAT_ARGB_5551,
	DMA_PIXEL_FORMAT_XRGB_5551,
	DMA_PIXEL_FORMAT_ARGB_8888,
	DMA_PIXEL_FORMAT_XRGB_8888,

	DMA_PIXEL_FORMAT_RESERVED0,

	DMA_PIXEL_FORMAT_YUYV_422_Pkg,
	DMA_PIXEL_FORMAT_YUV_420_SP_HP,
	DMA_PIXEL_FORMAT_YUV_420_P_HP,
	DMA_PIXEL_FORMAT_YUV_422_SP_HP,
	DMA_PIXEL_FORMAT_YUV_422_P_HP,
	DMA_PIXEL_FORMAT_AYUV_4444,
};

enum dss_buf_format {
	DSS_BUF_LINEAR = 0,
	DSS_BUF_TILE,
};

enum dss_blend_mode {
	DSS_BLEND_CLEAR = 0,
	DSS_BLEND_SRC,
	DSS_BLEND_DST,
	DSS_BLEND_SRC_OVER_DST,
	DSS_BLEND_DST_OVER_SRC,
	DSS_BLEND_SRC_IN_DST,
	DSS_BLEND_DST_IN_SRC,
	DSS_BLEND_SRC_OUT_DST,
	DSS_BLEND_DST_OUT_SRC,
	DSS_BLEND_SRC_ATOP_DST,
	DSS_BLEND_DST_ATOP_SRC,
	DSS_BLEND_SRC_XOR_DST,
	DSS_BLEND_SRC_ADD_DST,
	DSS_BLEND_FIX_OVER,
	DSS_BLEND_FIX_PER0,
	DSS_BLEND_FIX_PER1,
	DSS_BLEND_FIX_PER2,
	DSS_BLEND_FIX_PER3,
	DSS_BLEND_FIX_PER4,
	DSS_BLEND_FIX_PER5,
	DSS_BLEND_FIX_PER6,
	DSS_BLEND_FIX_PER7,
	DSS_BLEND_FIX_PER8,
	DSS_BLEND_FIX_PER9,
	DSS_BLEND_FIX_PER10,
	DSS_BLEND_FIX_PER11,
	DSS_BLEND_FIX_PER12,
	DSS_BLEND_FIX_PER13,
	DSS_BLEND_FIX_PER14,
	DSS_BLEND_FIX_PER15,
	DSS_BLEND_FIX_PER16,
	DSS_BLEND_FIX_PER17,

	DSS_BLEND_MAX,
};

enum dss_chn_module {
	MODULE_MIF_CHN,
	MODULE_AIF0_CHN,
	MODULE_AIF1_CHN,
	MODULE_MCTL_CHN_MUTEX,
	MODULE_MCTL_CHN_FLUSH_EN,
	MODULE_MCTL_CHN_OV_OEN,
	MODULE_MCTL_CHN_STARTY,
	MODULE_MCTL_CHN_MOD_DBG,
	MODULE_DMA,
	MODULE_DFC,
	MODULE_SCL,
	MODULE_SCL_LUT,
	MODULE_ARSR2P,
	MODULE_ARSR2P_LUT,
	MODULE_POST_CLIP,
	MODULE_PCSC,
	MODULE_CSC,
	MODULE_CHN_MAX,
};

enum dss_chn_cap {
	MODULE_CAP_ROT,
	MODULE_CAP_SCL,
	MODULE_CAP_CSC,
	MODULE_CAP_SHARPNESS_1D,
	MODULE_CAP_SHARPNESS_2D,
	MODULE_CAP_CE,
	MODULE_CAP_AFBCD,
	MODULE_CAP_AFBCE,
	MODULE_CAP_YUV_PLANAR,
	MODULE_CAP_YUV_SEMI_PLANAR,
	MODULE_CAP_YUV_PACKAGE,
	MODULE_CAP_MAX,
};

enum dss_ovl_module {
	MODULE_OVL_BASE,
	MODULE_MCTL_BASE,
	MODULE_OVL_MAX,
};

enum dss_axi_idx {
	AXI_CHN0 = 0,
	AXI_CHN1,
	AXI_CHN_MAX,
};

#define AXI0_MAX_DSS_CHN_THRESHOLD	(3)
#define AXI1_MAX_DSS_CHN_THRESHOLD	(3)

#define DEFAULT_AXI_CLK_RATE0	(120 * 1000000)
#define DEFAULT_AXI_CLK_RATE1	(240 * 1000000)
#define DEFAULT_AXI_CLK_RATE2	(360 * 1000000)
#define DEFAULT_AXI_CLK_RATE3	(480 * 1000000)
#define DEFAULT_AXI_CLK_RATE4	(667 * 1000000)
#define DEFAULT_AXI_CLK_RATE5	(800 * 1000000)

enum dss_rdma_idx {
	DSS_RDMA0 = 0,
	DSS_RDMA1,
	DSS_RDMA2,
	DSS_RDMA3,
	DSS_RDMA4,
	DSS_RDMA_MAX,
};

typedef struct dss_aif {
	u32 aif_ch_ctl;
	u32 aif_ch_ctl_add;
} dss_aif_t;

typedef struct dss_aif_bw {
	u64 bw;
	u8 chn_idx;
	s8 axi_sel;
	u8 is_used;
} dss_aif_bw_t;

#define LITTLE_LAYER_BUF_SIZE	(256 * 1024)
#define MIF_STRIDE_UNIT (4 * 1024)

typedef struct dss_mif {
	u32 mif_ctrl1;
	u32 mif_ctrl2;
	u32 mif_ctrl3;
	u32 mif_ctrl4;
	u32 mif_ctrl5;
} dss_mif_t;

typedef struct dss_dfc {
	u32 disp_size;
	u32 pix_in_num;
	u32 disp_fmt;
	u32 clip_ctl_hrz;
	u32 clip_ctl_vrz;
	u32 ctl_clip_en;
	u32 icg_module;
	u32 dither_enable;
	u32 padding_ctl;
} dss_dfc_t;

typedef struct dss_scl {
	u32 en_hscl_str;
	u32 en_vscl_str;
	u32 h_v_order;
	u32 input_width_height;
	u32 output_width_height;
	u32 en_hscl;
	u32 en_vscl;
	u32 acc_hscl;
	u32 inc_hscl;
	u32 inc_vscl;
	u32 en_mmp;
	u32 scf_ch_core_gt;
	u32 fmt;
} dss_scl_t;

enum scl_coef_lut_idx {
	SCL_COEF_NONE_IDX = -1,
	SCL_COEF_YUV_IDX = 0,
	SCL_COEF_RGB_IDX = 1,
	SCL_COEF_IDX_MAX = 2,
};

typedef struct dss_arsr2p_effect {
	u32 skin_thres_y;
	u32 skin_thres_u;
	u32 skin_thres_v;
	u32 skin_cfg0;
	u32 skin_cfg1;
	u32 skin_cfg2;
	u32 shoot_cfg1;
	u32 shoot_cfg2;
	u32 sharp_cfg1;
	u32 sharp_cfg2;
	u32 sharp_cfg3;
	u32 sharp_cfg4;
	u32 sharp_cfg5;
	u32 sharp_cfg6;
	u32 sharp_cfg7;
	u32 sharp_cfg8;
	u32 sharp_cfg9;
	u32 texturw_analysts;
	u32 intplshootctrl;
} dss_arsr2p_effect_t;

typedef struct dss_arsr2p {
	u32 arsr_input_width_height;
	u32 arsr_output_width_height;
	u32 ihleft;
	u32 ihright;
	u32 ivtop;
	u32 ivbottom;
	u32 ihinc;
	u32 ivinc;
	u32 offset;
	u32 mode;
	dss_arsr2p_effect_t arsr2p_effect;
	u32 ihleft1;
	u32 ihright1;
	u32 ivbottom1;
} dss_arsr2p_t;

typedef struct dss_post_clip {
	u32 disp_size;
	u32 clip_ctl_hrz;
	u32 clip_ctl_vrz;
	u32 ctl_clip_en;
} dss_post_clip_t;

typedef struct dss_pcsc {
	u32 pcsc_idc0;
} dss_pcsc_t;

typedef struct dss_csc {
	u32 idc0;
	u32 idc2;
	u32 odc0;
	u32 odc2;
	u32 p0;
	u32 p1;
	u32 p2;
	u32 p3;
	u32 p4;
	u32 icg_module;
	u32 mprec;
} dss_csc_t;

typedef struct dss_rdma {
	u32 oft_x0;
	u32 oft_y0;
	u32 oft_x1;
	u32 oft_y1;
	u32 mask0;
	u32 mask1;
	u32 stretch_size_vrt;
	u32 ctrl;
	u32 tile_scram;

	u32 data_addr0;
	u32 stride0;
	u32 stretch_stride0;
	u32 data_num0;

	u32 data_addr1;
	u32 stride1;
	u32 stretch_stride1;
	u32 data_num1;

	u32 data_addr2;
	u32 stride2;
	u32 stretch_stride2;
	u32 data_num2;

	u32 ch_rd_shadow;
	u32 ch_ctl;

	u32 dma_buf_ctrl;

	u32 vpp_ctrl;
	u32 vpp_mem_ctrl;

	u32 afbcd_hreg_hdr_ptr_lo;
	u32 afbcd_hreg_pic_width;
	u32 afbcd_hreg_pic_height;
	u32 afbcd_hreg_format;
	u32 afbcd_ctl;
	u32 afbcd_str;
	u32 afbcd_line_crop;
	u32 afbcd_input_header_stride;
	u32 afbcd_payload_stride;
	u32 afbcd_mm_base_0;

	u32 afbcd_afbcd_payload_pointer;
	u32 afbcd_height_bf_str;
	u32 afbcd_os_cfg;
	u32 afbcd_mem_ctrl;
	u32 afbcd_scramble_mode;
	u32 afbcd_header_pointer_offset;

	u8 vpp_used;
	u8 afbc_used;
} dss_rdma_t;

typedef struct dss_wdma {
	u32 oft_x0;
	u32 oft_y0;
	u32 oft_x1;
	u32 oft_y1;

	u32 mask0;
	u32 mask1;
	u32 stretch_size_vrt;
	u32 ctrl;
	u32 tile_scram;

	u32 sw_mask_en;
	u32 start_mask0;
	u32 end_mask0;
	u32 start_mask1;
	u32 end_mask1;

	u32 data_addr;
	u32 stride0;
	u32 data1_addr;
	u32 stride1;

	u32 stretch_stride;
	u32 data_num;

	u32 ch_rd_shadow;
	u32 ch_ctl;
	u32 ch_secu_en;
	u32 ch_sw_end_req;

	u32 dma_buf_ctrl;
	u32 dma_buf_size;

	u32 rot_size;

	u32 afbce_hreg_pic_blks;
	u32 afbce_hreg_format;
	u32 afbce_hreg_hdr_ptr_lo;
	u32 afbce_hreg_pld_ptr_lo;
	u32 afbce_picture_size;
	u32 afbce_ctl;
	u32 afbce_header_srtide;
	u32 afbce_payload_stride;
	u32 afbce_enc_os_cfg;
	u32 afbce_mem_ctrl;
	u32 afbce_qos_cfg;
	u32 afbce_threshold;
	u32 afbce_scramble_mode;
	u32 afbce_header_pointer_offset;

	u8 afbc_used;
	u8 rot_used;
} dss_wdma_t;

enum dss_mctl_idx {
	DSS_MCTL0 = 0,
	DSS_MCTL1,
	DSS_MCTL2,
	DSS_MCTL3,
	DSS_MCTL4,
	DSS_MCTL5,
	DSS_MCTL_IDX_MAX,
};

typedef struct dss_mctl {
	u32 ctl_mutex_itf;
	u32 ctl_mutex_dbuf;
	u32 ctl_mutex_scf;
	u32 ctl_mutex_ov;
} dss_mctl_t;

typedef struct dss_mctl_ch_base {
	char __iomem *chn_mutex_base;
	char __iomem *chn_flush_en_base;
	char __iomem *chn_ov_en_base;
	char __iomem *chn_starty_base;
	char __iomem *chn_mod_dbg_base;
} dss_mctl_ch_base_t;

typedef struct dss_mctl_ch {
	u32 chn_mutex;
	u32 chn_flush_en;
	u32 chn_ov_oen;
	u32 chn_starty;
	u32 chn_mod_dbg;
} dss_mctl_ch_t;

typedef struct dss_mctl_sys {
	u32 ov_flush_en[DSS_OVL_IDX_MAX];
	u32 chn_ov_sel[DSS_OVL_IDX_MAX];
	u32 wchn_ov_sel[DSS_WCH_MAX];
	u8 ov_flush_en_used[DSS_OVL_IDX_MAX];
	u8 chn_ov_sel_used[DSS_OVL_IDX_MAX];
	u8 wch_ov_sel_used[DSS_WCH_MAX];
} dss_mctl_sys_t;

typedef struct dss_ovl_layer {
	u32 layer_pos;
	u32 layer_size;
	u32 layer_pattern;
	u32 layer_alpha;
	u32 layer_cfg;

} dss_ovl_layer_t;

typedef struct dss_ovl_layer_pos {
	u32 layer_pspos;
	u32 layer_pepos;

} dss_ovl_layer_pos_t;

typedef struct dss_ovl_alpha {
	u32 src_amode;
	u32 src_gmode;
	u32 alpha_offsrc;
	u32 src_lmode;
	u32 src_pmode;

	u32 alpha_smode;

	u32 dst_amode;
	u32 dst_gmode;
	u32 alpha_offdst;
	u32 dst_pmode;

	u32 fix_mode;
} dss_ovl_alpha_t;

typedef struct dss_arsr1p {
	u32 ihleft;
	u32 ihright;
	u32 ihleft1;
	u32 ihright1;
	u32 ivtop;
	u32 ivbottom;
	u32 uv_offset;
	u32 ihinc;
	u32 ivinc;
	u32 mode;
	u32 format;

	u32 skin_thres_y;
	u32 skin_thres_u;
	u32 skin_thres_v;
	u32 skin_expected;
	u32 skin_cfg;
	u32 shoot_cfg1;
	u32 shoot_cfg2;
	u32 sharp_cfg1;
	u32 sharp_cfg2;
	u32 sharp_cfg3;
	u32 sharp_cfg4;
	u32 sharp_cfg5;
	u32 sharp_cfg6;
	u32 sharp_cfg7;
	u32 sharp_cfg8;
	u32 sharp_cfg9;
	u32 sharp_cfg10;
	u32 sharp_cfg11;
	u32 diff_ctrl;
	u32 lsc_cfg1;
	u32 lsc_cfg2;
	u32 lsc_cfg3;
	u32 force_clk_on_cfg;

	u32 dpp_img_hrz_bef_sr;
	u32 dpp_img_vrt_bef_sr;
	u32 dpp_img_hrz_aft_sr;
	u32 dpp_img_vrt_aft_sr;
} dss_arsr1p_t;

enum hisi_fb_pixel_format {
	HISI_FB_PIXEL_FORMAT_RGB_565 = 0,
	HISI_FB_PIXEL_FORMAT_RGBX_4444,
	HISI_FB_PIXEL_FORMAT_RGBA_4444,
	HISI_FB_PIXEL_FORMAT_RGBX_5551,
	HISI_FB_PIXEL_FORMAT_RGBA_5551,
	HISI_FB_PIXEL_FORMAT_RGBX_8888,
	HISI_FB_PIXEL_FORMAT_RGBA_8888,

	HISI_FB_PIXEL_FORMAT_BGR_565,
	HISI_FB_PIXEL_FORMAT_BGRX_4444,
	HISI_FB_PIXEL_FORMAT_BGRA_4444,
	HISI_FB_PIXEL_FORMAT_BGRX_5551,
	HISI_FB_PIXEL_FORMAT_BGRA_5551,
	HISI_FB_PIXEL_FORMAT_BGRX_8888,
	HISI_FB_PIXEL_FORMAT_BGRA_8888,

	HISI_FB_PIXEL_FORMAT_YUV_422_I,

	/* YUV Semi-planar */
	HISI_FB_PIXEL_FORMAT_YCbCr_422_SP,	/* NV16 */
	HISI_FB_PIXEL_FORMAT_YCrCb_422_SP,
	HISI_FB_PIXEL_FORMAT_YCbCr_420_SP,
	HISI_FB_PIXEL_FORMAT_YCrCb_420_SP,	/* NV21 */

	/* YUV Planar */
	HISI_FB_PIXEL_FORMAT_YCbCr_422_P,
	HISI_FB_PIXEL_FORMAT_YCrCb_422_P,
	HISI_FB_PIXEL_FORMAT_YCbCr_420_P,
	HISI_FB_PIXEL_FORMAT_YCrCb_420_P,	/* HISI_FB_PIXEL_FORMAT_YV12 */

	/* YUV Package */
	HISI_FB_PIXEL_FORMAT_YUYV_422_Pkg,
	HISI_FB_PIXEL_FORMAT_UYVY_422_Pkg,
	HISI_FB_PIXEL_FORMAT_YVYU_422_Pkg,
	HISI_FB_PIXEL_FORMAT_VYUY_422_Pkg,
	HISI_FB_PIXEL_FORMAT_MAX,

	HISI_FB_PIXEL_FORMAT_UNSUPPORT = 800
};

struct dss_hw_ctx {
	void __iomem *base;
	struct regmap *noc_regmap;
	struct reset_control *reset;

	void __iomem *noc_dss_base;
	void __iomem *peri_crg_base;
	void __iomem *pmc_base;
	void __iomem *sctrl_base;

	struct clk *dss_axi_clk;
	struct clk *dss_pclk_dss_clk;
	struct clk *dss_pri_clk;
	struct clk *dss_pxl0_clk;
	struct clk *dss_pxl1_clk;
	struct clk *dss_mmbuf_clk;
	struct clk *dss_pclk_mmbuf_clk;

	bool power_on;
	int irq;

	wait_queue_head_t vactive0_end_wq;
	u32 vactive0_end_flag;
	ktime_t vsync_timestamp;
	ktime_t vsync_timestamp_prev;

	struct iommu_domain *mmu_domain;
	struct ion_client *ion_client;
	struct ion_handle *ion_handle;
//	struct iommu_map_format iommu_format;
	char __iomem *screen_base;
	unsigned long smem_start;
	unsigned long screen_size;
};

struct dss_crtc {
	struct drm_crtc base;
	struct dss_hw_ctx *ctx;
	bool enable;
	u32 out_format;
	u32 bgr_fmt;
};

struct dss_plane {
	struct drm_plane base;
	/*void *ctx;*/
	void *acrtc;
	u8 ch; /* channel */
};

struct dss_data {
	struct dss_crtc acrtc;
	struct dss_plane aplane[DSS_CH_NUM];
	struct dss_hw_ctx ctx;
};

/* ade-format info: */
struct dss_format {
	u32 pixel_format;
	enum hisi_fb_pixel_format dss_format;
};

#define MIPI_DPHY_NUM	(2)

/* IFBC compress mode */
enum IFBC_TYPE {
	IFBC_TYPE_NONE = 0,
	IFBC_TYPE_ORISE2X,
	IFBC_TYPE_ORISE3X,
	IFBC_TYPE_HIMAX2X,
	IFBC_TYPE_RSP2X,
	IFBC_TYPE_RSP3X,
	IFBC_TYPE_VESA2X_SINGLE,
	IFBC_TYPE_VESA3X_SINGLE,
	IFBC_TYPE_VESA2X_DUAL,
	IFBC_TYPE_VESA3X_DUAL,
	IFBC_TYPE_VESA3_75X_DUAL,

	IFBC_TYPE_MAX
};

/* IFBC compress mode */
enum IFBC_COMP_MODE {
	IFBC_COMP_MODE_0 = 0,
	IFBC_COMP_MODE_1,
	IFBC_COMP_MODE_2,
	IFBC_COMP_MODE_3,
	IFBC_COMP_MODE_4,
	IFBC_COMP_MODE_5,
	IFBC_COMP_MODE_6,
};

/* xres_div */
enum XRES_DIV {
	XRES_DIV_1 = 1,
	XRES_DIV_2,
	XRES_DIV_3,
	XRES_DIV_4,
	XRES_DIV_5,
	XRES_DIV_6,
};

/* yres_div */
enum YRES_DIV {
	YRES_DIV_1 = 1,
	YRES_DIV_2,
	YRES_DIV_3,
	YRES_DIV_4,
	YRES_DIV_5,
	YRES_DIV_6,
};

/* pxl0_divxcfg */
enum PXL0_DIVCFG {
	PXL0_DIVCFG_0 = 0,
	PXL0_DIVCFG_1,
	PXL0_DIVCFG_2,
	PXL0_DIVCFG_3,
	PXL0_DIVCFG_4,
	PXL0_DIVCFG_5,
	PXL0_DIVCFG_6,
	PXL0_DIVCFG_7,
};

/* pxl0_div2_gt_en */
enum PXL0_DIV2_GT_EN {
	PXL0_DIV2_GT_EN_CLOSE = 0,
	PXL0_DIV2_GT_EN_OPEN,
};

/* pxl0_div4_gt_en */
enum PXL0_DIV4_GT_EN {
	PXL0_DIV4_GT_EN_CLOSE = 0,
	PXL0_DIV4_GT_EN_OPEN,
};

/* pxl0_dsi_gt_en */
enum PXL0_DSI_GT_EN {
	PXL0_DSI_GT_EN_0 = 0,
	PXL0_DSI_GT_EN_1,
	PXL0_DSI_GT_EN_2,
	PXL0_DSI_GT_EN_3,
};

typedef struct mipi_ifbc_division {
	u32 xres_div;
	u32 yres_div;
	u32 comp_mode;
	u32 pxl0_div2_gt_en;
	u32 pxl0_div4_gt_en;
	u32 pxl0_divxcfg;
	u32 pxl0_dsi_gt_en;
} mipi_ifbc_division_t;

#ifndef ALIGN_DOWN
#define ALIGN_DOWN(val, al)  ((val) & ~((al) - 1))
#endif
#ifndef ALIGN_UP
#define ALIGN_UP(val, al)    (((val) + ((al) - 1)) & ~((al) - 1))
#endif

#define to_dss_crtc(crtc) \
	container_of(crtc, struct dss_crtc, base)

#define to_dss_plane(plane) \
	container_of(plane, struct dss_plane, base)

#endif
