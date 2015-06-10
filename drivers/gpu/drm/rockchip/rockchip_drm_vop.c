/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author:Mark Yao <mark.yao@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drm.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_plane_helper.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/component.h>

#include <linux/reset.h>
#include <linux/delay.h>

#include "rockchip_drm_drv.h"
#include "rockchip_drm_gem.h"
#include "rockchip_drm_fb.h"
#include "rockchip_drm_vop.h"

#define VOP_REG(off, _mask, s) \
		{.offset = off, \
		 .mask = _mask, \
		 .shift = s,}

#define __REG_SET_RELAXED(x, off, mask, shift, v) \
		vop_mask_write_relaxed(x, off, (mask) << shift, (v) << shift)
#define __REG_SET_NORMAL(x, off, mask, shift, v) \
		vop_mask_write(x, off, (mask) << shift, (v) << shift)

#define REG_SET(x, base, reg, v, mode) \
		__REG_SET_##mode(x, base + reg.offset, reg.mask, reg.shift, v)

#define VOP_WIN_SET(x, win, name, v) \
		REG_SET(x, win->base, win->phy->name, v, RELAXED)
#define VOP_CTRL_SET(x, name, v) \
		REG_SET(x, 0, (x)->data->ctrl->name, v, NORMAL)

#define VOP_WIN_GET(x, win, name) \
		vop_read_reg(x, win->base, &win->phy->name)

#define VOP_WIN_GET_YRGBADDR(vop, win) \
		vop_readl(vop, win->base + win->phy->yrgb_mst.offset)
#define VOP_SCL_SET(x, win, name, v) \
		REG_SET(x, win->base, win->scl->name, v, RELAXED)

#define to_vop(x) container_of(x, struct vop, crtc)
#define to_vop_win(x) container_of(x, struct vop_win, base)

struct vop_win_state {
	struct list_head head;
	struct drm_framebuffer *fb;
	dma_addr_t yrgb_mst;
	struct drm_pending_vblank_event *event;
};

struct vop_win {
	struct drm_plane base;
	const struct vop_win_data *data;
	struct vop *vop;

	struct list_head pending;
	struct vop_win_state *active;
};

struct vop {
	struct drm_crtc crtc;
	struct device *dev;
	struct drm_device *drm_dev;
	bool is_enabled;

	int connector_type;
	int connector_out_mode;

	/* mutex vsync_ work */
	struct mutex vsync_mutex;
	bool vsync_work_pending;
	struct completion dsp_hold_completion;

	const struct vop_data *data;

	uint32_t *regsbak;
	void __iomem *regs;

	/* physical map length of vop register */
	uint32_t len;

	/* one time only one process allowed to config the register */
	spinlock_t reg_lock;
	/* lock vop irq reg */
	spinlock_t irq_lock;

	unsigned int irq;

	/* vop AHP clk */
	struct clk *hclk;
	/* vop dclk */
	struct clk *dclk;
	/* vop share memory frequency */
	struct clk *aclk;

	/* vop dclk reset */
	struct reset_control *dclk_rst;

	int pipe;

	struct vop_win win[];
};

enum vop_data_format {
	VOP_FMT_ARGB8888 = 0,
	VOP_FMT_RGB888,
	VOP_FMT_RGB565,
	VOP_FMT_YUV420SP = 4,
	VOP_FMT_YUV422SP,
	VOP_FMT_YUV444SP,
};

struct vop_reg_data {
	uint32_t offset;
	uint32_t value;
};

struct vop_reg {
	uint32_t offset;
	uint32_t shift;
	uint32_t mask;
};

struct vop_ctrl {
	struct vop_reg standby;
	struct vop_reg data_blank;
	struct vop_reg gate_en;
	struct vop_reg mmu_en;
	struct vop_reg rgb_en;
	struct vop_reg edp_en;
	struct vop_reg hdmi_en;
	struct vop_reg mipi_en;
	struct vop_reg out_mode;
	struct vop_reg dither_down;
	struct vop_reg dither_up;
	struct vop_reg pin_pol;

	struct vop_reg htotal_pw;
	struct vop_reg hact_st_end;
	struct vop_reg vtotal_pw;
	struct vop_reg vact_st_end;
	struct vop_reg hpost_st_end;
	struct vop_reg vpost_st_end;
};

struct vop_win_phy {
	const uint32_t *data_formats;
	uint32_t nformats;

	struct vop_reg enable;
	struct vop_reg format;
	struct vop_reg act_info;
	struct vop_reg dsp_info;
	struct vop_reg dsp_st;
	struct vop_reg yrgb_mst;
	struct vop_reg uv_mst;
	struct vop_reg yrgb_vir;
	struct vop_reg uv_vir;

	struct vop_reg dst_alpha_ctl;
	struct vop_reg src_alpha_ctl;
};

struct vop_scl_regs {
	struct vop_reg cbr_vsd_mode;
	struct vop_reg cbr_vsu_mode;
	struct vop_reg cbr_hsd_mode;
	struct vop_reg cbr_ver_scl_mode;
	struct vop_reg cbr_hor_scl_mode;
	struct vop_reg yrgb_vsd_mode;
	struct vop_reg yrgb_vsu_mode;
	struct vop_reg yrgb_hsd_mode;
	struct vop_reg yrgb_ver_scl_mode;
	struct vop_reg yrgb_hor_scl_mode;
	struct vop_reg line_load_mode;
	struct vop_reg cbr_axi_gather_num;
	struct vop_reg yrgb_axi_gather_num;
	struct vop_reg vsd_cbr_gt2;
	struct vop_reg vsd_cbr_gt4;
	struct vop_reg vsd_yrgb_gt2;
	struct vop_reg vsd_yrgb_gt4;
	struct vop_reg bic_coe_sel;
	struct vop_reg cbr_axi_gather_en;
	struct vop_reg yrgb_axi_gather_en;

	struct vop_reg lb_mode;
	struct vop_reg scale_yrgb_x;
	struct vop_reg scale_yrgb_y;
	struct vop_reg scale_cbcr_x;
	struct vop_reg scale_cbcr_y;
};

struct vop_win_data {
	uint32_t base;
	const struct vop_win_phy *phy;
	const struct vop_scl_regs *scl;
	enum drm_plane_type type;
};

struct vop_data {
	const struct vop_reg_data *init_table;
	unsigned int table_size;
	const struct vop_ctrl *ctrl;
	const struct vop_win_data *win;
	unsigned int win_size;
};

static const uint32_t formats_01[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV16,
	DRM_FORMAT_NV24,
};

static const uint32_t formats_234[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB888,
	DRM_FORMAT_RGB565,
};

static const struct vop_scl_regs win_full_scl = {
	.cbr_vsd_mode = VOP_REG(WIN0_CTRL1, 0x1, 31),
	.cbr_vsu_mode = VOP_REG(WIN0_CTRL1, 0x1, 30),
	.cbr_hsd_mode = VOP_REG(WIN0_CTRL1, 0x3, 28),
	.cbr_ver_scl_mode = VOP_REG(WIN0_CTRL1, 0x3, 26),
	.cbr_hor_scl_mode = VOP_REG(WIN0_CTRL1, 0x3, 24),
	.yrgb_vsd_mode = VOP_REG(WIN0_CTRL1, 0x1, 23),
	.yrgb_vsu_mode = VOP_REG(WIN0_CTRL1, 0x1, 22),
	.yrgb_hsd_mode = VOP_REG(WIN0_CTRL1, 0x3, 20),
	.yrgb_ver_scl_mode = VOP_REG(WIN0_CTRL1, 0x3, 18),
	.yrgb_hor_scl_mode = VOP_REG(WIN0_CTRL1, 0x3, 16),
	.line_load_mode = VOP_REG(WIN0_CTRL1, 0x1, 15),
	.cbr_axi_gather_num = VOP_REG(WIN0_CTRL1, 0x7, 12),
	.yrgb_axi_gather_num = VOP_REG(WIN0_CTRL1, 0xf, 8),
	.vsd_cbr_gt2 = VOP_REG(WIN0_CTRL1, 0x1, 7),
	.vsd_cbr_gt4 = VOP_REG(WIN0_CTRL1, 0x1, 6),
	.vsd_yrgb_gt2 = VOP_REG(WIN0_CTRL1, 0x1, 5),
	.vsd_yrgb_gt4 = VOP_REG(WIN0_CTRL1, 0x1, 4),
	.bic_coe_sel = VOP_REG(WIN0_CTRL1, 0x3, 2),
	.cbr_axi_gather_en = VOP_REG(WIN0_CTRL1, 0x1, 1),
	.yrgb_axi_gather_en = VOP_REG(WIN0_CTRL1, 0x1, 0),
	.lb_mode = VOP_REG(WIN0_CTRL0, 0x7, 5),
	.scale_yrgb_x = VOP_REG(WIN0_SCL_FACTOR_YRGB, 0xffff, 0x0),
	.scale_yrgb_y = VOP_REG(WIN0_SCL_FACTOR_YRGB, 0xffff, 16),
	.scale_cbcr_x = VOP_REG(WIN0_SCL_FACTOR_CBR, 0xffff, 0x0),
	.scale_cbcr_y = VOP_REG(WIN0_SCL_FACTOR_CBR, 0xffff, 16),
};

static const struct vop_win_phy win01_data = {
	.data_formats = formats_01,
	.nformats = ARRAY_SIZE(formats_01),
	.enable = VOP_REG(WIN0_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN0_CTRL0, 0x7, 1),
	.act_info = VOP_REG(WIN0_ACT_INFO, 0x1fff1fff, 0),
	.dsp_info = VOP_REG(WIN0_DSP_INFO, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN0_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN0_YRGB_MST, 0xffffffff, 0),
	.uv_mst = VOP_REG(WIN0_CBR_MST, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN0_VIR, 0x3fff, 0),
	.uv_vir = VOP_REG(WIN0_VIR, 0x3fff, 16),
	.src_alpha_ctl = VOP_REG(WIN0_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN0_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy win23_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(WIN2_CTRL0, 0x1, 0),
	.format = VOP_REG(WIN2_CTRL0, 0x7, 1),
	.dsp_info = VOP_REG(WIN2_DSP_INFO0, 0x0fff0fff, 0),
	.dsp_st = VOP_REG(WIN2_DSP_ST0, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(WIN2_MST0, 0xffffffff, 0),
	.yrgb_vir = VOP_REG(WIN2_VIR0_1, 0x1fff, 0),
	.src_alpha_ctl = VOP_REG(WIN2_SRC_ALPHA_CTRL, 0xff, 0),
	.dst_alpha_ctl = VOP_REG(WIN2_DST_ALPHA_CTRL, 0xff, 0),
};

static const struct vop_win_phy cursor_data = {
	.data_formats = formats_234,
	.nformats = ARRAY_SIZE(formats_234),
	.enable = VOP_REG(HWC_CTRL0, 0x1, 0),
	.format = VOP_REG(HWC_CTRL0, 0x7, 1),
	.dsp_st = VOP_REG(HWC_DSP_ST, 0x1fff1fff, 0),
	.yrgb_mst = VOP_REG(HWC_MST, 0xffffffff, 0),
};

static const struct vop_ctrl ctrl_data = {
	.standby = VOP_REG(SYS_CTRL, 0x1, 22),
	.gate_en = VOP_REG(SYS_CTRL, 0x1, 23),
	.mmu_en = VOP_REG(SYS_CTRL, 0x1, 20),
	.rgb_en = VOP_REG(SYS_CTRL, 0x1, 12),
	.hdmi_en = VOP_REG(SYS_CTRL, 0x1, 13),
	.edp_en = VOP_REG(SYS_CTRL, 0x1, 14),
	.mipi_en = VOP_REG(SYS_CTRL, 0x1, 15),
	.dither_down = VOP_REG(DSP_CTRL1, 0xf, 1),
	.dither_up = VOP_REG(DSP_CTRL1, 0x1, 6),
	.data_blank = VOP_REG(DSP_CTRL0, 0x1, 19),
	.out_mode = VOP_REG(DSP_CTRL0, 0xf, 0),
	.pin_pol = VOP_REG(DSP_CTRL0, 0xf, 4),
	.htotal_pw = VOP_REG(DSP_HTOTAL_HS_END, 0x1fff1fff, 0),
	.hact_st_end = VOP_REG(DSP_HACT_ST_END, 0x1fff1fff, 0),
	.vtotal_pw = VOP_REG(DSP_VTOTAL_VS_END, 0x1fff1fff, 0),
	.vact_st_end = VOP_REG(DSP_VACT_ST_END, 0x1fff1fff, 0),
	.hpost_st_end = VOP_REG(POST_DSP_HACT_INFO, 0x1fff1fff, 0),
	.vpost_st_end = VOP_REG(POST_DSP_VACT_INFO, 0x1fff1fff, 0),
};

static const struct vop_reg_data vop_init_reg_table[] = {
	{SYS_CTRL, 0x00c00000},
	{DSP_CTRL0, 0x00000000},
	{WIN0_CTRL0, 0x00000080},
	{WIN1_CTRL0, 0x00000080},
	/*
	 * Todo: win2/3 support area func, but now havn't found a suitable
	 * way to use it, so default enable area0 as a win display.
	 */
	{WIN2_CTRL0, 0x00000010},
	{WIN3_CTRL0, 0x00000010},
};

/*
 * Note: rk3288 has a dedicated 'cursor' window, however, that window requires
 * special support to get alpha blending working.  For now, just use overlay
 * window 1 for the drm cursor.
 */
static const struct vop_win_data rk3288_vop_win_data[] = {
	{ .base = 0x00, .phy = &win01_data, .scl = &win_full_scl, .type = DRM_PLANE_TYPE_PRIMARY },
	{ .base = 0x40, .phy = &win01_data, .scl = &win_full_scl, .type = DRM_PLANE_TYPE_OVERLAY },
	{ .base = 0x00, .phy = &win23_data, .type = DRM_PLANE_TYPE_OVERLAY },
	{ .base = 0x50, .phy = &win23_data, .type = DRM_PLANE_TYPE_CURSOR },
	{ .base = 0x00, .phy = &cursor_data, .type = DRM_PLANE_TYPE_OVERLAY },
};

static const struct vop_data rk3288_vop = {
	.init_table = vop_init_reg_table,
	.table_size = ARRAY_SIZE(vop_init_reg_table),
	.ctrl = &ctrl_data,
	.win = rk3288_vop_win_data,
	.win_size = ARRAY_SIZE(rk3288_vop_win_data),
};

static const struct of_device_id vop_driver_dt_match[] = {
	{ .compatible = "rockchip,rk3288-vop",
	  .data = &rk3288_vop },
	{},
};

static inline void vop_writel(struct vop *vop, uint32_t offset, uint32_t v)
{
	writel(v, vop->regs + offset);
	vop->regsbak[offset >> 2] = v;
}

static inline uint32_t vop_readl(struct vop *vop, uint32_t offset)
{
	return readl(vop->regs + offset);
}

static inline uint32_t vop_read_reg(struct vop *vop, uint32_t base,
				    const struct vop_reg *reg)
{
	return (vop_readl(vop, base + reg->offset) >> reg->shift) & reg->mask;
}

static inline void vop_cfg_done(struct vop *vop)
{
	writel(0x01, vop->regs + REG_CFG_DONE);
}

static inline void vop_mask_write(struct vop *vop, uint32_t offset,
				  uint32_t mask, uint32_t v)
{
	if (mask) {
		uint32_t cached_val = vop->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel(cached_val, vop->regs + offset);
		vop->regsbak[offset >> 2] = cached_val;
	}
}

static inline void vop_mask_write_relaxed(struct vop *vop, uint32_t offset,
					  uint32_t mask, uint32_t v)
{
	if (mask) {
		uint32_t cached_val = vop->regsbak[offset >> 2];

		cached_val = (cached_val & ~mask) | v;
		writel_relaxed(cached_val, vop->regs + offset);
		vop->regsbak[offset >> 2] = cached_val;
	}
}

static int __getHardWareVSkipLines(u32 srcH, u32 dstH)
{
    u32 vScaleDnMult;

    if(srcH >= (u32)(4*dstH*MIN_SCALE_FACTOR_AFTER_VSKIP))
    {
        vScaleDnMult = 4;
    }
    else if(srcH >= (u32)(2*dstH*MIN_SCALE_FACTOR_AFTER_VSKIP))
    {
        vScaleDnMult = 2;
    }
    else
    {
        vScaleDnMult = 1;
    }

    return vScaleDnMult;
}

static enum vop_data_format vop_convert_format(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		return VOP_FMT_ARGB8888;
	case DRM_FORMAT_RGB888:
		return VOP_FMT_RGB888;
	case DRM_FORMAT_RGB565:
		return VOP_FMT_RGB565;
	case DRM_FORMAT_NV12:
		return VOP_FMT_YUV420SP;
	case DRM_FORMAT_NV16:
		return VOP_FMT_YUV422SP;
	case DRM_FORMAT_NV24:
		return VOP_FMT_YUV444SP;
	default:
		DRM_ERROR("unsupport format[%08x]\n", format);
		return -EINVAL;
	}
}

static bool is_alpha_support(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_ARGB8888:
		return true;
	default:
		return false;
	}
}

static void vop_dsp_hold_valid_irq_enable(struct vop *vop)
{
	unsigned long flags;

	if (WARN_ON(!vop->is_enabled))
		return;

	spin_lock_irqsave(&vop->irq_lock, flags);

	vop_mask_write(vop, INTR_CTRL0, DSP_HOLD_VALID_INTR_MASK,
		       DSP_HOLD_VALID_INTR_EN(1));

	spin_unlock_irqrestore(&vop->irq_lock, flags);
}

static void vop_dsp_hold_valid_irq_disable(struct vop *vop)
{
	unsigned long flags;

	if (WARN_ON(!vop->is_enabled))
		return;

	spin_lock_irqsave(&vop->irq_lock, flags);

	vop_mask_write(vop, INTR_CTRL0, DSP_HOLD_VALID_INTR_MASK,
		       DSP_HOLD_VALID_INTR_EN(0));

	spin_unlock_irqrestore(&vop->irq_lock, flags);
}

static void vop_enable(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);
	int ret;

	if (vop->is_enabled)
		return;

	ret = pm_runtime_get_sync(vop->dev);
	if (ret < 0) {
		dev_err(vop->dev, "failed to get pm runtime: %d\n", ret);
		return;
	}

	ret = clk_enable(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable hclk - %d\n", ret);
		return;
	}

	ret = clk_enable(vop->dclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable dclk - %d\n", ret);
		goto err_disable_hclk;
	}

	ret = clk_enable(vop->aclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to enable aclk - %d\n", ret);
		goto err_disable_dclk;
	}

	/*
	 * Slave iommu shares power, irq and clock with vop.  It was associated
	 * automatically with this master device via common driver code.
	 * Now that we have enabled the clock we attach it to the shared drm
	 * mapping.
	 */
	ret = rockchip_drm_dma_attach_device(vop->drm_dev, vop->dev);
	if (ret) {
		dev_err(vop->dev, "failed to attach dma mapping, %d\n", ret);
		goto err_disable_aclk;
	}

	/*
	 * At here, vop clock & iommu is enable, R/W vop regs would be safe.
	 */
	vop->is_enabled = true;

	spin_lock(&vop->reg_lock);

	VOP_CTRL_SET(vop, standby, 0);

	spin_unlock(&vop->reg_lock);

	enable_irq(vop->irq);

	drm_vblank_on(vop->drm_dev, vop->pipe);

	return;

err_disable_aclk:
	clk_disable(vop->aclk);
err_disable_dclk:
	clk_disable(vop->dclk);
err_disable_hclk:
	clk_disable(vop->hclk);
}

static void vop_disable(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);

	if (!vop->is_enabled)
		return;

	drm_vblank_off(crtc->dev, vop->pipe);

	/*
	 * Vop standby will take effect at end of current frame,
	 * if dsp hold valid irq happen, it means standby complete.
	 *
	 * we must wait standby complete when we want to disable aclk,
	 * if not, memory bus maybe dead.
	 */
	reinit_completion(&vop->dsp_hold_completion);
	vop_dsp_hold_valid_irq_enable(vop);

	spin_lock(&vop->reg_lock);

	VOP_CTRL_SET(vop, standby, 1);

	spin_unlock(&vop->reg_lock);

	wait_for_completion(&vop->dsp_hold_completion);

	vop_dsp_hold_valid_irq_disable(vop);

	disable_irq(vop->irq);

	vop->is_enabled = false;

	/*
	 * vop standby complete, so iommu detach is safe.
	 */
	rockchip_drm_dma_detach_device(vop->drm_dev, vop->dev);

	clk_disable(vop->dclk);
	clk_disable(vop->aclk);
	clk_disable(vop->hclk);
	pm_runtime_put(vop->dev);
}

static int _vop_cal_scl_fac(struct vop *vop, const struct vop_win_data *win, u16 srcW, u16 srcH,
							u16 dstW, u16 dstH, int format)
{
	u16 yrgb_srcW;
	u16 yrgb_srcH;
	u16 yrgb_dstW;
	u16 yrgb_dstH;
	u32 yrgb_vScaleDnMult;
	u32 yrgb_xscl_factor;
	u32 yrgb_yscl_factor;
	u8  yrgb_vsd_bil_gt2=0;
	u8  yrgb_vsd_bil_gt4=0;
	
	u16 cbcr_srcW;
	u16 cbcr_srcH;
	u16 cbcr_dstW;
	u16 cbcr_dstH;	  
	u32 cbcr_vScaleDnMult;
	u32 cbcr_xscl_factor;
	u32 cbcr_yscl_factor;
	u8  cbcr_vsd_bil_gt2=0;
	u8  cbcr_vsd_bil_gt4=0;
	u8  yuv_fmt=0;

	u16 yrgb_hor_scl_mode;
	u16 yrgb_ver_scl_mode;
	u16 cbr_hor_scl_mode;
	u16 cbr_ver_scl_mode;
	u16 lb_mode;
	u16 yrgb_hsd_mode;
	u16 cbr_hsd_mode;
	u16 yrgb_vsd_mode;
	u16 cbr_vsd_mode;
	u16 yrgb_vsu_mode; 
	u16 cbr_vsu_mode; 
	u16 scale_yrgb_x;
	u16 scale_yrgb_y;
	u16 vsd_yrgb_gt4;
	u16 vsd_yrgb_gt2;
	u16 scale_cbcr_x;
	u16 scale_cbcr_y;
	u16 vsd_cbr_gt4;
	u16 vsd_cbr_gt2;

	/*yrgb scl mode*/
	yrgb_srcW = srcW;
	yrgb_srcH = srcH;
	yrgb_dstW = dstW;
	yrgb_dstH = dstH;
	if ((yrgb_dstW*8 <= yrgb_srcW) || (yrgb_dstH*8 <= yrgb_srcH)) {
		pr_err("ERROR: yrgb scale exceed 8,"
				"srcW=%d,srcH=%d,dstW=%d,dstH=%d\n",
				yrgb_srcW,yrgb_srcH,yrgb_dstW,yrgb_dstH);
	}
	if(yrgb_srcW < yrgb_dstW)
		yrgb_hor_scl_mode = SCALE_UP;
	else if(yrgb_srcW > yrgb_dstW)
		yrgb_hor_scl_mode = SCALE_DOWN;
	else
		yrgb_hor_scl_mode = SCALE_NONE;

	if(yrgb_srcH < yrgb_dstH)
		yrgb_ver_scl_mode = SCALE_UP;
	else if (yrgb_srcH  > yrgb_dstH)
		yrgb_ver_scl_mode = SCALE_DOWN;
	else
		yrgb_ver_scl_mode = SCALE_NONE;

	/*cbcr scl mode*/
	switch (format) {
		case VOP_FMT_YUV422SP:
			cbcr_srcW = srcW/2;
			cbcr_dstW = dstW;
			cbcr_srcH = srcH;
			cbcr_dstH = dstH;
			yuv_fmt = 1;
			break;
		case VOP_FMT_YUV420SP:
			/*
			 *tmp modify.
			 */
			cbcr_srcW = srcW/2;
			cbcr_srcH = srcH/2;
			//cbcr_srcH = srcH;
			//cbcr_srcW = srcW;
			cbcr_dstW = dstW;
			cbcr_dstH = dstH;
			yuv_fmt = 1;
			break;
		case VOP_FMT_YUV444SP:
			cbcr_srcW = srcW;
			cbcr_dstW = dstW;
			cbcr_srcH = srcH;
			cbcr_dstH = dstH;
			yuv_fmt = 1;
			break;
		default:
			cbcr_srcW = 0;
			cbcr_dstW = 0;
			cbcr_srcH = 0;
			cbcr_dstH = 0;
			yuv_fmt = 0;
			break;
	}

	if (yuv_fmt) {
		if ((cbcr_dstW*8 <= cbcr_srcW) || (cbcr_dstH*8 <= cbcr_srcH)) {
			pr_err("ERROR: cbcr scale exceed 8,"
					"srcW=%d,srcH=%d,dstW=%d,dstH=%d\n",
					cbcr_srcW,cbcr_srcH,cbcr_dstW,cbcr_dstH);
		}
	}

	if(cbcr_srcW < cbcr_dstW)
		cbr_hor_scl_mode = SCALE_UP;
	else if(cbcr_srcW > cbcr_dstW)
		cbr_hor_scl_mode = SCALE_DOWN;
	else
		cbr_hor_scl_mode = SCALE_NONE;

	if(cbcr_srcH < cbcr_dstH)
		cbr_ver_scl_mode = SCALE_UP;
	else if(cbcr_srcH > cbcr_dstH)
		cbr_ver_scl_mode = SCALE_DOWN;
	else
		cbr_ver_scl_mode = SCALE_NONE;
#if 0
	pr_err( "srcW:%d>>srcH:%d>>dstW:%d>>dstH:%d>>\n"
			"yrgb:src:W=%d>>H=%d,dst:W=%d>>H=%d,H_mode=%d,V_mode=%d\n"
			"cbcr:src:W=%d>>H=%d,dst:W=%d>>H=%d,H_mode=%d,V_mode=%d\n"
			,srcW,srcH,dstW,dstH,yrgb_srcW,yrgb_srcH,yrgb_dstW,
			yrgb_dstH,yrgb_hor_scl_mode,yrgb_ver_scl_mode,
			cbcr_srcW,cbcr_srcH,cbcr_dstW,cbcr_dstH,
			cbr_hor_scl_mode,cbr_ver_scl_mode);
#endif
	/*line buffer mode*/
	if ((format == VOP_FMT_YUV422SP) ||
			(format == VOP_FMT_YUV420SP)) {
		if (cbr_hor_scl_mode == SCALE_DOWN) {
			if ((cbcr_dstW > 3840) || (cbcr_dstW == 0)) {
				pr_err("ERROR cbcr_dstW = %d\n",cbcr_dstW);                
			} else if (cbcr_dstW > 2560) {
				lb_mode = LB_RGB_3840X2;
			} else if (cbcr_dstW > 1920) {
				if (yrgb_hor_scl_mode == SCALE_DOWN) {
					if(yrgb_dstW > 3840){
						pr_err("ERROR yrgb_dst_width exceeds 3840\n");
					}else if(yrgb_dstW > 2560){
						lb_mode = LB_RGB_3840X2;
					}else if(yrgb_dstW > 1920){
						lb_mode = LB_RGB_2560X4;
					}else{
						pr_err("ERROR never run here!yrgb_dstW<1920 ==> cbcr_dstW>1920\n");
					}
				}
			} else if (cbcr_dstW > 1280) {
				lb_mode = LB_YUV_3840X5;
			} else {
				lb_mode = LB_YUV_2560X8;
			}            
		} else { /*SCALE_UP or SCALE_NONE*/
			if ((cbcr_srcW > 3840) || (cbcr_srcW == 0)) {
				pr_err("ERROR cbcr_srcW = %d\n",cbcr_srcW);
			}else if(cbcr_srcW > 2560){                
				lb_mode = LB_RGB_3840X2;
			}else if(cbcr_srcW > 1920){
				if(yrgb_hor_scl_mode == SCALE_DOWN){
					if(yrgb_dstW > 3840){
						pr_err("ERROR yrgb_dst_width exceeds 3840\n");
					}else if(yrgb_dstW > 2560){
						lb_mode = LB_RGB_3840X2;
					}else if(yrgb_dstW > 1920){
						lb_mode = LB_RGB_2560X4;
					}else{
						pr_err("ERROR never run here!yrgb_dstW<1920 ==> cbcr_dstW>1920\n");
					}
				}  
			}else if(cbcr_srcW > 1280){
				lb_mode = LB_YUV_3840X5;
			}else{
				lb_mode = LB_YUV_2560X8;
			}            
		}
	}else {
		if(yrgb_hor_scl_mode == SCALE_DOWN){
			if ((yrgb_dstW > 3840) || (yrgb_dstW == 0)) {
				pr_err("ERROR yrgb_dstW = %d\n",yrgb_dstW);
			}else if(yrgb_dstW > 2560){
				lb_mode = LB_RGB_3840X2;
			}else if(yrgb_dstW > 1920){
				lb_mode = LB_RGB_2560X4;
			}else if(yrgb_dstW > 1280){
				lb_mode = LB_RGB_1920X5;
			}else{
				lb_mode = LB_RGB_1280X8;
			}            
		}else{ /*SCALE_UP or SCALE_NONE*/
			if ((yrgb_srcW > 3840) || (yrgb_srcW == 0)) {
				pr_err("ERROR yrgb_srcW = %d\n",yrgb_srcW);
			}else if(yrgb_srcW > 2560){
				lb_mode = LB_RGB_3840X2;
			}else if(yrgb_srcW > 1920){
				lb_mode = LB_RGB_2560X4;
			}else if(yrgb_srcW > 1280){
				lb_mode = LB_RGB_1920X5;
			}else{
				lb_mode = LB_RGB_1280X8;
			}            
		}
	}
	//pr_err("lb_mode = %d;\n",lb_mode);

	/*vsd/vsu scale ALGORITHM*/
	yrgb_hsd_mode = SCALE_DOWN_BIL;/*not to specify*/
	cbr_hsd_mode  = SCALE_DOWN_BIL;/*not to specify*/
	yrgb_vsd_mode = SCALE_DOWN_BIL;/*not to specify*/
	cbr_vsd_mode  = SCALE_DOWN_BIL;/*not to specify*/
	switch(lb_mode){
		case LB_YUV_3840X5:
		case LB_YUV_2560X8:
		case LB_RGB_1920X5:
		case LB_RGB_1280X8: 	
			yrgb_vsu_mode = SCALE_UP_BIC; 
			cbr_vsu_mode  = SCALE_UP_BIC; 
			break;
		case LB_RGB_3840X2:
			if(yrgb_ver_scl_mode != SCALE_NONE) {
				pr_err("ERROR : not allow yrgb ver scale\n");
			}
			if(cbr_ver_scl_mode != SCALE_NONE) {
				pr_err("ERROR : not allow cbcr ver scale\n");
			}	     	  
			break;
		case LB_RGB_2560X4:
			yrgb_vsu_mode = SCALE_UP_BIL; 
			cbr_vsu_mode  = SCALE_UP_BIL; 	    
			break;
		default:
			printk(KERN_WARNING "%s:un supported lb_mode:%d\n",
					__func__,lb_mode);	
			break;
	}
	//pr_err("yrgb:hsd=%d,vsd=%d,vsu=%d;cbcr:hsd=%d,vsd=%d,vsu=%d\n",
	//		yrgb_hsd_mode,yrgb_vsd_mode,yrgb_vsu_mode,
	//		cbr_hsd_mode,cbr_vsd_mode,cbr_vsu_mode);

	/*SCALE FACTOR*/

	/*(1.1)YRGB HOR SCALE FACTOR*/
	switch(yrgb_hor_scl_mode){
		case SCALE_NONE:
			yrgb_xscl_factor = (1<<SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
			break;
		case SCALE_UP:
			yrgb_xscl_factor = GET_SCALE_FACTOR_BIC(yrgb_srcW, yrgb_dstW);
			break;
		case SCALE_DOWN:
			switch(yrgb_hsd_mode)
			{
				case SCALE_DOWN_BIL:
					yrgb_xscl_factor = GET_SCALE_FACTOR_BILI_DN(yrgb_srcW, yrgb_dstW);
					break;
				case SCALE_DOWN_AVG:
					yrgb_xscl_factor = GET_SCALE_FACTOR_AVRG(yrgb_srcW, yrgb_dstW);
					break;
				default :
					printk(KERN_WARNING "%s:un supported yrgb_hsd_mode:%d\n",
							__func__,yrgb_hsd_mode);		
					break;
			} 
			break;
		default :
			printk(KERN_WARNING "%s:un supported yrgb_hor_scl_mode:%d\n",
					__func__,yrgb_hor_scl_mode);	
			break;
	} /*yrgb_hor_scl_mode*/

	/*(1.2)YRGB VER SCALE FACTOR*/
	switch(yrgb_ver_scl_mode)
	{
		case SCALE_NONE:
			yrgb_yscl_factor = (1<<SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
			break;
		case SCALE_UP  :
			switch(yrgb_vsu_mode)
			{
				case SCALE_UP_BIL:
					yrgb_yscl_factor = GET_SCALE_FACTOR_BILI_UP(yrgb_srcH, yrgb_dstH);
					break;
				case SCALE_UP_BIC:
					if(yrgb_srcH < 3){
						pr_err("yrgb_srcH should be greater than 3 !!!\n");
					}                    
					yrgb_yscl_factor = GET_SCALE_FACTOR_BIC(yrgb_srcH, yrgb_dstH);
					break;
				default :
					printk(KERN_WARNING "%s:un supported yrgb_vsu_mode:%d\n",
							__func__,yrgb_vsu_mode);			
					break;
			}
			break;
		case SCALE_DOWN:
			switch(yrgb_vsd_mode)
			{
				case SCALE_DOWN_BIL:
					yrgb_vScaleDnMult = __getHardWareVSkipLines(yrgb_srcH, yrgb_dstH);
					yrgb_yscl_factor  = GET_SCALE_FACTOR_BILI_DN_VSKIP(yrgb_srcH, yrgb_dstH, yrgb_vScaleDnMult);                                 
					if(yrgb_vScaleDnMult == 4){
						yrgb_vsd_bil_gt4 = 1;
						yrgb_vsd_bil_gt2 = 0;
					}else if(yrgb_vScaleDnMult == 2){
						yrgb_vsd_bil_gt4 = 0;
						yrgb_vsd_bil_gt2 = 1;
					}else{
						yrgb_vsd_bil_gt4 = 0;
						yrgb_vsd_bil_gt2 = 0;
					}
					break;
				case SCALE_DOWN_AVG:
					yrgb_yscl_factor = GET_SCALE_FACTOR_AVRG(yrgb_srcH, yrgb_dstH);
					break;
				default:
					printk(KERN_WARNING "%s:un supported yrgb_vsd_mode:%d\n",
							__func__,yrgb_vsd_mode);		
					break;
			} /*yrgb_vsd_mode*/
			break;
		default :
			printk(KERN_WARNING "%s:un supported yrgb_ver_scl_mode:%d\n",
					__func__,yrgb_ver_scl_mode);		
			break;
	}
	scale_yrgb_x = yrgb_xscl_factor;
	scale_yrgb_y = yrgb_yscl_factor;
	vsd_yrgb_gt4 = yrgb_vsd_bil_gt4;
	vsd_yrgb_gt2 = yrgb_vsd_bil_gt2;
//	pr_err("yrgb:h_fac=%d,v_fac=%d,gt4=%d,gt2=%d\n",yrgb_xscl_factor,
//			yrgb_yscl_factor,yrgb_vsd_bil_gt4,yrgb_vsd_bil_gt2);

	/*(2.1)CBCR HOR SCALE FACTOR*/
	switch(cbr_hor_scl_mode)
	{
		case SCALE_NONE:
			cbcr_xscl_factor = (1<<SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
			break;
		case SCALE_UP  :
			cbcr_xscl_factor = GET_SCALE_FACTOR_BIC(cbcr_srcW, cbcr_dstW);
			break;
		case SCALE_DOWN:
			switch(cbr_hsd_mode)
			{
				case SCALE_DOWN_BIL:
					cbcr_xscl_factor = GET_SCALE_FACTOR_BILI_DN(cbcr_srcW, cbcr_dstW);
					break;
				case SCALE_DOWN_AVG:
					cbcr_xscl_factor = GET_SCALE_FACTOR_AVRG(cbcr_srcW, cbcr_dstW);
					break;
				default :
					printk(KERN_WARNING "%s:un supported cbr_hsd_mode:%d\n",
							__func__,cbr_hsd_mode);	
					break;
			}
			break;
		default :
			printk(KERN_WARNING "%s:un supported cbr_hor_scl_mode:%d\n",
					__func__,cbr_hor_scl_mode);	
			break;
	} /*cbr_hor_scl_mode*/

	/*(2.2)CBCR VER SCALE FACTOR*/
	switch(cbr_ver_scl_mode)
	{
		case SCALE_NONE:
			cbcr_yscl_factor = (1<<SCALE_FACTOR_DEFAULT_FIXPOINT_SHIFT);
			break;
		case SCALE_UP  :
			switch(cbr_vsu_mode)
			{
				case SCALE_UP_BIL:
					cbcr_yscl_factor = GET_SCALE_FACTOR_BILI_UP(cbcr_srcH, cbcr_dstH);
					break;
				case SCALE_UP_BIC:
					if(cbcr_srcH < 3) {
						pr_err("cbcr_srcH should be greater than 3 !!!\n");
					}                    
					cbcr_yscl_factor = GET_SCALE_FACTOR_BIC(cbcr_srcH, cbcr_dstH);
					break;
				default :
					printk(KERN_WARNING "%s:un supported cbr_vsu_mode:%d\n",
							__func__,cbr_vsu_mode);		
					break;
			}
			break;
		case SCALE_DOWN:
			switch(cbr_vsd_mode)
			{
				case SCALE_DOWN_BIL:
					cbcr_vScaleDnMult = __getHardWareVSkipLines(cbcr_srcH, cbcr_dstH);
					cbcr_yscl_factor  = GET_SCALE_FACTOR_BILI_DN_VSKIP(cbcr_srcH, cbcr_dstH, cbcr_vScaleDnMult);                    
					if(cbcr_vScaleDnMult == 4){
						cbcr_vsd_bil_gt4 = 1;
						cbcr_vsd_bil_gt2 = 0;
					}else if(cbcr_vScaleDnMult == 2){
						cbcr_vsd_bil_gt4 = 0;
						cbcr_vsd_bil_gt2 = 1;
					}else{
						cbcr_vsd_bil_gt4 = 0;
						cbcr_vsd_bil_gt2 = 0;
					}
					break;
				case SCALE_DOWN_AVG:
					cbcr_yscl_factor = GET_SCALE_FACTOR_AVRG(cbcr_srcH, cbcr_dstH);
					break;
				default :
					printk(KERN_WARNING "%s:un supported cbr_vsd_mode:%d\n",
							__func__,cbr_vsd_mode);		
					break;
			}
			break;
		default :
			printk(KERN_WARNING "%s:un supported cbr_ver_scl_mode:%d\n",
					__func__,cbr_ver_scl_mode);			
			break;
	}
	scale_cbcr_x = cbcr_xscl_factor;
	scale_cbcr_y = cbcr_yscl_factor;
	vsd_cbr_gt4  = cbcr_vsd_bil_gt4;
	vsd_cbr_gt2  = cbcr_vsd_bil_gt2;	
	if (lb_mode == 5)
	       lb_mode = 4;	

	VOP_SCL_SET(vop, win, yrgb_hor_scl_mode, yrgb_hor_scl_mode);
	VOP_SCL_SET(vop, win, yrgb_ver_scl_mode, yrgb_ver_scl_mode);
	VOP_SCL_SET(vop, win, cbr_hor_scl_mode, cbr_hor_scl_mode);
	VOP_SCL_SET(vop, win, cbr_ver_scl_mode, cbr_ver_scl_mode);
	VOP_SCL_SET(vop, win, lb_mode, lb_mode);
	VOP_SCL_SET(vop, win, yrgb_hsd_mode, yrgb_hsd_mode);
	VOP_SCL_SET(vop, win, cbr_hsd_mode, cbr_hsd_mode);
	VOP_SCL_SET(vop, win, yrgb_vsd_mode, yrgb_vsd_mode);
	VOP_SCL_SET(vop, win, cbr_vsd_mode, cbr_vsd_mode);
	VOP_SCL_SET(vop, win, yrgb_vsu_mode, yrgb_vsu_mode); 
	VOP_SCL_SET(vop, win, cbr_vsu_mode, cbr_vsu_mode); 
	VOP_SCL_SET(vop, win, scale_yrgb_x, scale_yrgb_x);
	VOP_SCL_SET(vop, win, scale_yrgb_y, scale_yrgb_y);
	VOP_SCL_SET(vop, win, vsd_yrgb_gt4, vsd_yrgb_gt4);
	VOP_SCL_SET(vop, win, vsd_yrgb_gt2, vsd_yrgb_gt2);
	VOP_SCL_SET(vop, win, scale_cbcr_x, scale_cbcr_x);
	VOP_SCL_SET(vop, win, scale_cbcr_y, scale_cbcr_y);
	VOP_SCL_SET(vop, win, vsd_cbr_gt4, vsd_cbr_gt4);
	VOP_SCL_SET(vop, win, vsd_cbr_gt2, vsd_cbr_gt2);

//	pr_err("cbcr:h_fac=%d,v_fac=%d,gt4=%d,gt2=%d\n",cbcr_xscl_factor,
//		cbcr_yscl_factor,cbcr_vsd_bil_gt4,cbcr_vsd_bil_gt2);
	return 0;
}


/*
 * Caller must hold vsync_mutex.
 */
static struct drm_framebuffer *vop_win_last_pending_fb(struct vop_win *vop_win)
{
	struct vop_win_state *last;
	struct vop_win_state *active = vop_win->active;

	if (list_empty(&vop_win->pending))
		return active ? active->fb : NULL;

	last = list_last_entry(&vop_win->pending, struct vop_win_state, head);
	return last ? last->fb : NULL;
}

/*
 * Caller must hold vsync_mutex.
 */
static int vop_win_queue_fb(struct vop_win *vop_win,
			    struct drm_framebuffer *fb, dma_addr_t yrgb_mst,
			    struct drm_pending_vblank_event *event)
{
	struct vop_win_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->fb = fb;
	state->yrgb_mst = yrgb_mst;
	state->event = event;

	list_add_tail(&state->head, &vop_win->pending);

	return 0;
}

static int vop_update_plane_event(struct drm_plane *plane,
				  struct drm_crtc *crtc,
				  struct drm_framebuffer *fb, int crtc_x,
				  int crtc_y, unsigned int crtc_w,
				  unsigned int crtc_h, uint32_t src_x,
				  uint32_t src_y, uint32_t src_w,
				  uint32_t src_h,
				  struct drm_pending_vblank_event *event)
{
	struct vop_win *vop_win = to_vop_win(plane);
	const struct vop_win_data *win = vop_win->data;
	struct vop *vop = to_vop(crtc);
	struct drm_gem_object *obj;
	struct rockchip_gem_object *rk_obj;
	struct drm_gem_object *uv_obj;
	struct rockchip_gem_object *rk_uv_obj;
	struct rockchip_fb_obj *fb_obj;
	struct rockchip_fb_obj *uv_fb_obj;
	unsigned long offset;
	unsigned int actual_w;
	unsigned int actual_h;
	unsigned int dsp_stx;
	unsigned int dsp_sty;
	unsigned int y_vir_stride;
	unsigned int uv_vir_stride;
	dma_addr_t yrgb_mst;
	dma_addr_t uv_mst;
	enum vop_data_format format;
	uint32_t val;
	bool is_alpha;
	bool visible;
	int ret;
	struct drm_rect dest = {
		.x1 = crtc_x,
		.y1 = crtc_y,
		.x2 = crtc_x + crtc_w,
		.y2 = crtc_y + crtc_h,
	};
	struct drm_rect src = {
		/* 16.16 fixed point */
		.x1 = src_x,
		.y1 = src_y,
		.x2 = src_x + src_w,
		.y2 = src_y + src_h,
	};
	const struct drm_rect clip = {
		.x2 = crtc->mode.hdisplay,
		.y2 = crtc->mode.vdisplay,
	};
	bool can_position = plane->type != DRM_PLANE_TYPE_PRIMARY;

#if 0
	ret = drm_plane_helper_check_update(plane, crtc, fb,
					    &src, &dest, &clip,
					    0,
					    0xffff,
					    can_position, false, &visible);
	if (ret)
		return ret;

	if (!visible)
		return 0;
#endif
	is_alpha = is_alpha_support(fb->pixel_format);
	format = vop_convert_format(fb->pixel_format);
	if (format < 0)
		return format;

	fb_obj = rockchip_fb_get_gem_obj(fb, 0);
	if (!fb_obj || !fb_obj->obj) {
		DRM_ERROR("fail to get rockchip gem object from framebuffer\n");
		return -EINVAL;
	}

	rk_obj = to_rockchip_obj(fb_obj->obj);

	actual_w = (src.x2 - src.x1) >> 16;
	actual_h = (src.y2 - src.y1) >> 16;
	crtc_x = max(0, crtc_x);
	crtc_y = max(0, crtc_y);

	dsp_stx = crtc_x + crtc->mode.htotal - crtc->mode.hsync_start;
	dsp_sty = crtc_y + crtc->mode.vtotal - crtc->mode.vsync_start;
	offset = (src.x1 >> 16) * (fb->bits_per_pixel >> 3);
	offset += (src.y1 >> 16) * fb->pitches[0];

	if (fb->pixel_format == DRM_FORMAT_NV12) {
		uv_fb_obj = rockchip_fb_get_gem_obj(fb, 1);
		if (!uv_fb_obj || !uv_fb_obj->obj) {
			DRM_ERROR("fail to get rockchip gem object from framebuffer\n");
			return -EINVAL;
		}

		rk_uv_obj = to_rockchip_obj(uv_fb_obj->obj);

		y_vir_stride = fb->pitches[0] >> 2;
		uv_vir_stride = y_vir_stride;
		yrgb_mst = rk_obj->dma_addr + offset + fb_obj->offset;
		uv_mst = rk_uv_obj->dma_addr + uv_fb_obj->offset;
	} else {
		yrgb_mst = rk_obj->dma_addr + offset + fb_obj->offset;
		y_vir_stride = fb->pitches[0] / (fb->bits_per_pixel >> 3);
	}

	/*
	 * If this plane update changes the plane's framebuffer, (or more
	 * precisely, if this update has a different framebuffer than the last
	 * update), enqueue it so we can track when it completes.
	 *
	 * Only when we discover that this update has completed, can we
	 * unreference any previous framebuffers.
	 */
	mutex_lock(&vop->vsync_mutex);
	if (fb != vop_win_last_pending_fb(vop_win)) {
		ret = drm_vblank_get(plane->dev, vop->pipe);
		if (ret) {
			DRM_ERROR("failed to get vblank, %d\n", ret);
			mutex_unlock(&vop->vsync_mutex);
			return ret;
		}

		drm_framebuffer_reference(fb);

		ret = vop_win_queue_fb(vop_win, fb, yrgb_mst, event);
		if (ret) {
			drm_vblank_put(plane->dev, vop->pipe);
			mutex_unlock(&vop->vsync_mutex);
			return ret;
		}

		vop->vsync_work_pending = true;
	}
	mutex_unlock(&vop->vsync_mutex);

	spin_lock(&vop->reg_lock);

	VOP_WIN_SET(vop, win, format, format);
	VOP_WIN_SET(vop, win, yrgb_vir, y_vir_stride);
	VOP_WIN_SET(vop, win, yrgb_mst, yrgb_mst);
	VOP_WIN_SET(vop, win, uv_vir, uv_vir_stride);
	VOP_WIN_SET(vop, win, uv_mst, uv_mst);
	if (win->scl)
		_vop_cal_scl_fac(vop, win, actual_w, actual_h, crtc_w, crtc_h, format);

	val = (actual_h - 1) << 16;
	val |= (actual_w - 1) & 0xffff;
	VOP_WIN_SET(vop, win, act_info, val);
	val = (crtc_h - 1) << 16;
	val |= (crtc_w - 1) & 0xffff;
	VOP_WIN_SET(vop, win, dsp_info, val);
	val = (dsp_sty - 1) << 16;
	val |= (dsp_stx - 1) & 0xffff;
	VOP_WIN_SET(vop, win, dsp_st, val);

	if (is_alpha) {
		VOP_WIN_SET(vop, win, dst_alpha_ctl,
			    DST_FACTOR_M0(ALPHA_SRC_INVERSE));
		val = SRC_ALPHA_EN(1) | SRC_COLOR_M0(ALPHA_SRC_PRE_MUL) |
			SRC_ALPHA_M0(ALPHA_STRAIGHT) |
			SRC_BLEND_M0(ALPHA_PER_PIX) |
			SRC_ALPHA_CAL_M0(ALPHA_NO_SATURATION) |
			SRC_FACTOR_M0(ALPHA_ONE);
		VOP_WIN_SET(vop, win, src_alpha_ctl, val);
	} else {
		VOP_WIN_SET(vop, win, src_alpha_ctl, SRC_ALPHA_EN(0));
	}

	VOP_WIN_SET(vop, win, enable, 1);

	vop_cfg_done(vop);
	spin_unlock(&vop->reg_lock);

	return 0;
}

static int vop_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
			    struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			    unsigned int crtc_w, unsigned int crtc_h,
			    uint32_t src_x, uint32_t src_y, uint32_t src_w,
			    uint32_t src_h)
{
	return vop_update_plane_event(plane, crtc, fb, crtc_x, crtc_y, crtc_w,
				      crtc_h, src_x, src_y, src_w, src_h,
				      NULL);
}

static int vop_update_primary_plane(struct drm_crtc *crtc,
				    struct drm_pending_vblank_event *event)
{
	unsigned int crtc_w, crtc_h;

	crtc_w = crtc->primary->fb->width - crtc->x;
	crtc_h = crtc->primary->fb->height - crtc->y;

	return vop_update_plane_event(crtc->primary, crtc, crtc->primary->fb,
				      0, 0, crtc_w, crtc_h, crtc->x << 16,
				      crtc->y << 16, crtc_w << 16,
				      crtc_h << 16, event);
}

static int vop_disable_plane(struct drm_plane *plane)
{
	struct vop_win *vop_win = to_vop_win(plane);
	const struct vop_win_data *win = vop_win->data;
	struct vop *vop;
	int ret;

	if (!plane->crtc)
		return 0;

	vop = to_vop(plane->crtc);

	ret = drm_vblank_get(plane->dev, vop->pipe);
	if (ret) {
		DRM_ERROR("failed to get vblank, %d\n", ret);
		return ret;
	}

	mutex_lock(&vop->vsync_mutex);

	ret = vop_win_queue_fb(vop_win, NULL, 0, NULL);
	if (ret) {
		drm_vblank_put(plane->dev, vop->pipe);
		mutex_unlock(&vop->vsync_mutex);
		return ret;
	}

	vop->vsync_work_pending = true;
	mutex_unlock(&vop->vsync_mutex);

	spin_lock(&vop->reg_lock);
	VOP_WIN_SET(vop, win, enable, 0);
	vop_cfg_done(vop);
	spin_unlock(&vop->reg_lock);

	return 0;
}

static void vop_plane_destroy(struct drm_plane *plane)
{
	vop_disable_plane(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs vop_plane_funcs = {
	.update_plane = vop_update_plane,
	.disable_plane = vop_disable_plane,
	.destroy = vop_plane_destroy,
};

int rockchip_drm_crtc_mode_config(struct drm_crtc *crtc,
				  int connector_type,
				  int out_mode)
{
	struct vop *vop = to_vop(crtc);

	vop->connector_type = connector_type;
	vop->connector_out_mode = out_mode;

	return 0;
}
EXPORT_SYMBOL_GPL(rockchip_drm_crtc_mode_config);

static int vop_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);
	unsigned long flags;

	if (!vop->is_enabled)
		return -EPERM;

	spin_lock_irqsave(&vop->irq_lock, flags);

	vop_mask_write(vop, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(1));

	spin_unlock_irqrestore(&vop->irq_lock, flags);

	return 0;
}

static void vop_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct vop *vop = to_vop(crtc);
	unsigned long flags;

	if (!vop->is_enabled)
		return;

	spin_lock_irqsave(&vop->irq_lock, flags);
	vop_mask_write(vop, INTR_CTRL0, FS_INTR_MASK, FS_INTR_EN(0));
	spin_unlock_irqrestore(&vop->irq_lock, flags);
}

static const struct rockchip_crtc_funcs private_crtc_funcs = {
	.enable_vblank = vop_crtc_enable_vblank,
	.disable_vblank = vop_crtc_disable_vblank,
};

static void vop_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	DRM_DEBUG_KMS("crtc[%d] mode[%d]\n", crtc->base.id, mode);

	switch (mode) {
	case DRM_MODE_DPMS_ON:
		vop_enable(crtc);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		vop_disable(crtc);
		break;
	default:
		DRM_DEBUG_KMS("unspecified mode %d\n", mode);
		break;
	}
}

static void vop_crtc_prepare(struct drm_crtc *crtc)
{
	vop_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static bool vop_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	if (adjusted_mode->htotal == 0 || adjusted_mode->vtotal == 0)
		return false;

	return true;
}

static int vop_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				  struct drm_framebuffer *old_fb)
{
	int ret;

	crtc->x = x;
	crtc->y = y;

	ret = vop_update_primary_plane(crtc, NULL);
	if (ret < 0) {
		DRM_ERROR("fail to update plane\n");
		return ret;
	}

	return 0;
}

static int vop_crtc_mode_set(struct drm_crtc *crtc,
			     struct drm_display_mode *mode,
			     struct drm_display_mode *adjusted_mode,
			     int x, int y, struct drm_framebuffer *fb)
{
	struct vop *vop = to_vop(crtc);
	u16 hsync_len = adjusted_mode->hsync_end - adjusted_mode->hsync_start;
	u16 hdisplay = adjusted_mode->hdisplay;
	u16 htotal = adjusted_mode->htotal;
	u16 hact_st = adjusted_mode->htotal - adjusted_mode->hsync_start;
	u16 hact_end = hact_st + hdisplay;
	u16 vdisplay = adjusted_mode->vdisplay;
	u16 vtotal = adjusted_mode->vtotal;
	u16 vsync_len = adjusted_mode->vsync_end - adjusted_mode->vsync_start;
	u16 vact_st = adjusted_mode->vtotal - adjusted_mode->vsync_start;
	u16 vact_end = vact_st + vdisplay;
	int ret, ret_clk;
	uint32_t val;

	/*
	 * disable dclk to stop frame scan, so that we can safe config mode and
	 * enable iommu.
	 */
	clk_disable(vop->dclk);

	switch (vop->connector_type) {
	case DRM_MODE_CONNECTOR_eDP:
		VOP_CTRL_SET(vop, edp_en, 1);
		break;
	case DRM_MODE_CONNECTOR_HDMIA:
		VOP_CTRL_SET(vop, hdmi_en, 1);
		break;
	case DRM_MODE_CONNECTOR_LVDS:
	default:
		VOP_CTRL_SET(vop, rgb_en, 1);
		break;
	};
	VOP_CTRL_SET(vop, out_mode, vop->connector_out_mode);

	val = 0x8;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NHSYNC) ? 0 : 1;
	val |= (adjusted_mode->flags & DRM_MODE_FLAG_NVSYNC) ? 0 : (1 << 1);
	VOP_CTRL_SET(vop, pin_pol, val);

	VOP_CTRL_SET(vop, htotal_pw, (htotal << 16) | hsync_len);
	val = hact_st << 16;
	val |= hact_end;
	VOP_CTRL_SET(vop, hact_st_end, val);
	VOP_CTRL_SET(vop, hpost_st_end, val);

	VOP_CTRL_SET(vop, vtotal_pw, (vtotal << 16) | vsync_len);
	val = vact_st << 16;
	val |= vact_end;
	VOP_CTRL_SET(vop, vact_st_end, val);
	VOP_CTRL_SET(vop, vpost_st_end, val);

	ret = vop_crtc_mode_set_base(crtc, x, y, fb);
	if (ret)
		goto out;

	/*
	 * reset dclk, take all mode config affect, so the clk would run in
	 * correct frame.
	 */
	reset_control_assert(vop->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(vop->dclk_rst);

	clk_set_rate(vop->dclk, adjusted_mode->clock * 1000);
out:
	ret_clk = clk_enable(vop->dclk);
	if (ret_clk < 0) {
		dev_err(vop->dev, "failed to enable dclk - %d\n", ret_clk);
		return ret_clk;
	}

	return ret;
}

static void vop_crtc_commit(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs vop_crtc_helper_funcs = {
	.dpms = vop_crtc_dpms,
	.prepare = vop_crtc_prepare,
	.mode_fixup = vop_crtc_mode_fixup,
	.mode_set = vop_crtc_mode_set,
	.mode_set_base = vop_crtc_mode_set_base,
	.commit = vop_crtc_commit,
};

static int vop_crtc_page_flip(struct drm_crtc *crtc,
			      struct drm_framebuffer *fb,
			      struct drm_pending_vblank_event *event,
			      uint32_t page_flip_flags)
{
	struct vop *vop = to_vop(crtc);
	struct drm_framebuffer *old_fb = crtc->primary->fb;
	int ret;

	/* when the page flip is requested, crtc should be on */
	if (!vop->is_enabled) {
		DRM_DEBUG("page flip request rejected because crtc is off.\n");
		return 0;
	}

	crtc->primary->fb = fb;

	ret = vop_update_primary_plane(crtc, event);
	if (ret)
		crtc->primary->fb = old_fb;

	return ret;
}

static void vop_win_state_complete(struct vop_win *vop_win,
				   struct vop_win_state *state)
{
	struct vop *vop = vop_win->vop;
	struct drm_crtc *crtc = &vop->crtc;
	struct drm_device *drm = crtc->dev;
	unsigned long flags;

	if (state->event) {
		spin_lock_irqsave(&drm->event_lock, flags);
		drm_send_vblank_event(drm, -1, state->event);
		spin_unlock_irqrestore(&drm->event_lock, flags);
	}

	list_del(&state->head);
	drm_vblank_put(crtc->dev, vop->pipe);
}

static void vop_crtc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static const struct drm_crtc_funcs vop_crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = vop_crtc_page_flip,
	.destroy = vop_crtc_destroy,
};

static bool vop_win_state_is_active(struct vop_win *vop_win,
				    struct vop_win_state *state)
{
	bool active = false;

	if (state->fb) {
		dma_addr_t yrgb_mst;

		/* check yrgb_mst to tell if pending_fb is now front */
		yrgb_mst = VOP_WIN_GET_YRGBADDR(vop_win->vop, vop_win->data);

		active = (yrgb_mst == state->yrgb_mst);
	} else {
		bool enabled;

		/* if enable bit is clear, plane is now disabled */
		enabled = VOP_WIN_GET(vop_win->vop, vop_win->data, enable);

		active = (enabled == 0);
	}

	return active;
}

static void vop_win_state_destroy(struct vop_win_state *state)
{
	struct drm_framebuffer *fb = state->fb;

	if (fb)
		drm_framebuffer_unreference(fb);

	kfree(state);
}

static void vop_win_update_state(struct vop_win *vop_win)
{
	struct vop_win_state *state, *n, *new_active = NULL;

	/* Check if any pending states are now active */
	list_for_each_entry(state, &vop_win->pending, head)
		if (vop_win_state_is_active(vop_win, state)) {
			new_active = state;
			break;
		}

	if (!new_active)
		return;

	/*
	 * Destroy any 'skipped' pending states - states that were queued
	 * before the newly active state.
	 */
	list_for_each_entry_safe(state, n, &vop_win->pending, head) {
		if (state == new_active)
			break;
		vop_win_state_complete(vop_win, state);
		vop_win_state_destroy(state);
	}

	vop_win_state_complete(vop_win, new_active);

	if (vop_win->active)
		vop_win_state_destroy(vop_win->active);
	vop_win->active = new_active;
}

static bool vop_win_has_pending_state(struct vop_win *vop_win)
{
	return !list_empty(&vop_win->pending);
}

static irqreturn_t vop_isr_thread(int irq, void *data)
{
	struct vop *vop = data;
	const struct vop_data *vop_data = vop->data;
	unsigned int i;

	mutex_lock(&vop->vsync_mutex);

	if (!vop->vsync_work_pending)
		goto done;

	vop->vsync_work_pending = false;

	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];

		vop_win_update_state(vop_win);
		if (vop_win_has_pending_state(vop_win))
			vop->vsync_work_pending = true;
	}

done:
	mutex_unlock(&vop->vsync_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t vop_isr(int irq, void *data)
{
	struct vop *vop = data;
	uint32_t intr0_reg, active_irqs;
	unsigned long flags;
	int ret = IRQ_NONE;

	/*
	 * INTR_CTRL0 register has interrupt status, enable and clear bits, we
	 * must hold irq_lock to avoid a race with enable/disable_vblank().
	*/
	spin_lock_irqsave(&vop->irq_lock, flags);
	intr0_reg = vop_readl(vop, INTR_CTRL0);
	active_irqs = intr0_reg & INTR_MASK;
	/* Clear all active interrupt sources */
	if (active_irqs)
		vop_writel(vop, INTR_CTRL0,
			   intr0_reg | (active_irqs << INTR_CLR_SHIFT));
	spin_unlock_irqrestore(&vop->irq_lock, flags);

	/* This is expected for vop iommu irqs, since the irq is shared */
	if (!active_irqs)
		return IRQ_NONE;

	if (active_irqs & DSP_HOLD_VALID_INTR) {
		complete(&vop->dsp_hold_completion);
		active_irqs &= ~DSP_HOLD_VALID_INTR;
		ret = IRQ_HANDLED;
	}

	if (active_irqs & FS_INTR) {
		drm_handle_vblank(vop->drm_dev, vop->pipe);
		active_irqs &= ~FS_INTR;
		ret = (vop->vsync_work_pending) ? IRQ_WAKE_THREAD : IRQ_HANDLED;
	}

	/* Unhandled irqs are spurious. */
	if (active_irqs)
		DRM_ERROR("Unknown VOP IRQs: %#02x\n", active_irqs);

	return ret;
}

static int vop_create_crtc(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	struct device *dev = vop->dev;
	struct drm_device *drm_dev = vop->drm_dev;
	struct drm_plane *primary = NULL, *cursor = NULL, *plane;
	struct drm_crtc *crtc = &vop->crtc;
	struct device_node *port;
	int ret;
	int i;

	/*
	 * Create drm_plane for primary and cursor planes first, since we need
	 * to pass them to drm_crtc_init_with_planes, which sets the
	 * "possible_crtcs" to the newly initialized crtc.
	 */
	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = vop_win->data;

		if (win_data->type != DRM_PLANE_TYPE_PRIMARY &&
		    win_data->type != DRM_PLANE_TYPE_CURSOR)
			continue;

		ret = drm_universal_plane_init(vop->drm_dev, &vop_win->base,
					       0, &vop_plane_funcs,
					       win_data->phy->data_formats,
					       win_data->phy->nformats,
					       win_data->type);
		if (ret) {
			DRM_ERROR("failed to initialize plane\n");
			goto err_cleanup_planes;
		}

		plane = &vop_win->base;
		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			primary = plane;
		else if (plane->type == DRM_PLANE_TYPE_CURSOR)
			cursor = plane;
	}

	ret = drm_crtc_init_with_planes(drm_dev, crtc, primary, cursor,
					&vop_crtc_funcs);
	if (ret)
		return ret;

	drm_crtc_helper_add(crtc, &vop_crtc_helper_funcs);

	/*
	 * Create drm_planes for overlay windows with possible_crtcs restricted
	 * to the newly created crtc.
	 */
	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = vop_win->data;
		unsigned long possible_crtcs = 1 << drm_crtc_index(crtc);

		if (win_data->type != DRM_PLANE_TYPE_OVERLAY)
			continue;

		ret = drm_universal_plane_init(vop->drm_dev, &vop_win->base,
					       possible_crtcs,
					       &vop_plane_funcs,
					       win_data->phy->data_formats,
					       win_data->phy->nformats,
					       win_data->type);
		if (ret) {
			DRM_ERROR("failed to initialize overlay plane\n");
			goto err_cleanup_crtc;
		}
	}

	port = of_get_child_by_name(dev->of_node, "port");
	if (!port) {
		DRM_ERROR("no port node found in %s\n",
			  dev->of_node->full_name);
		goto err_cleanup_crtc;
	}

	init_completion(&vop->dsp_hold_completion);
	crtc->port = port;
	vop->pipe = drm_crtc_index(crtc);
	rockchip_register_crtc_funcs(drm_dev, &private_crtc_funcs, vop->pipe);

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(crtc);
err_cleanup_planes:
	list_for_each_entry(plane, &drm_dev->mode_config.plane_list, head)
		drm_plane_cleanup(plane);
	return ret;
}

static void vop_destroy_crtc(struct vop *vop)
{
	struct drm_crtc *crtc = &vop->crtc;

	rockchip_unregister_crtc_funcs(vop->drm_dev, vop->pipe);
	of_node_put(crtc->port);
	drm_crtc_cleanup(crtc);
}

static int vop_initial(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	const struct vop_reg_data *init_table = vop_data->init_table;
	struct reset_control *ahb_rst;
	int i, ret;

	vop->hclk = devm_clk_get(vop->dev, "hclk_vop");
	if (IS_ERR(vop->hclk)) {
		dev_err(vop->dev, "failed to get hclk source\n");
		return PTR_ERR(vop->hclk);
	}
	vop->aclk = devm_clk_get(vop->dev, "aclk_vop");
	if (IS_ERR(vop->aclk)) {
		dev_err(vop->dev, "failed to get aclk source\n");
		return PTR_ERR(vop->aclk);
	}
	vop->dclk = devm_clk_get(vop->dev, "dclk_vop");
	if (IS_ERR(vop->dclk)) {
		dev_err(vop->dev, "failed to get dclk source\n");
		return PTR_ERR(vop->dclk);
	}

	ret = clk_prepare(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare hclk\n");
		return ret;
	}

	ret = clk_prepare(vop->dclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare dclk\n");
		goto err_unprepare_hclk;
	}

	ret = clk_prepare(vop->aclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare aclk\n");
		goto err_unprepare_dclk;
	}

	/*
	 * enable hclk, so that we can config vop register.
	 */
	ret = clk_enable(vop->hclk);
	if (ret < 0) {
		dev_err(vop->dev, "failed to prepare aclk\n");
		goto err_unprepare_aclk;
	}
	/*
	 * do hclk_reset, reset all vop registers.
	 */
	ahb_rst = devm_reset_control_get(vop->dev, "ahb");
	if (IS_ERR(ahb_rst)) {
		dev_err(vop->dev, "failed to get ahb reset\n");
		ret = PTR_ERR(ahb_rst);
		goto err_disable_hclk;
	}
	reset_control_assert(ahb_rst);
	usleep_range(10, 20);
	reset_control_deassert(ahb_rst);

	memcpy(vop->regsbak, vop->regs, vop->len);

	for (i = 0; i < vop_data->table_size; i++)
		vop_writel(vop, init_table[i].offset, init_table[i].value);

	for (i = 0; i < vop_data->win_size; i++) {
		const struct vop_win_data *win = &vop_data->win[i];

		VOP_WIN_SET(vop, win, enable, 0);
	}

	vop_cfg_done(vop);

	/*
	 * do dclk_reset, let all config take affect.
	 */
	vop->dclk_rst = devm_reset_control_get(vop->dev, "dclk");
	if (IS_ERR(vop->dclk_rst)) {
		dev_err(vop->dev, "failed to get dclk reset\n");
		ret = PTR_ERR(vop->dclk_rst);
		goto err_unprepare_aclk;
	}
	reset_control_assert(vop->dclk_rst);
	usleep_range(10, 20);
	reset_control_deassert(vop->dclk_rst);

	clk_disable(vop->hclk);

	vop->is_enabled = false;

	return 0;

err_disable_hclk:
	clk_disable(vop->hclk);
err_unprepare_aclk:
	clk_unprepare(vop->aclk);
err_unprepare_dclk:
	clk_unprepare(vop->dclk);
err_unprepare_hclk:
	clk_unprepare(vop->hclk);
	return ret;
}

/*
 * Initialize the vop->win array elements.
 */
static void vop_win_init(struct vop *vop)
{
	const struct vop_data *vop_data = vop->data;
	unsigned int i;

	for (i = 0; i < vop_data->win_size; i++) {
		struct vop_win *vop_win = &vop->win[i];
		const struct vop_win_data *win_data = &vop_data->win[i];

		vop_win->data = win_data;
		vop_win->vop = vop;
		INIT_LIST_HEAD(&vop_win->pending);
	}
}

static int vop_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct of_device_id *of_id;
	const struct vop_data *vop_data;
	struct drm_device *drm_dev = data;
	struct vop *vop;
	struct resource *res;
	size_t alloc_size;
	int ret;

	of_id = of_match_device(vop_driver_dt_match, dev);
	vop_data = of_id->data;
	if (!vop_data)
		return -ENODEV;

	/* Allocate vop struct and its vop_win array */
	alloc_size = sizeof(*vop) + sizeof(*vop->win) * vop_data->win_size;
	vop = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
	if (!vop)
		return -ENOMEM;

	vop->dev = dev;
	vop->data = vop_data;
	vop->drm_dev = drm_dev;
	dev_set_drvdata(dev, vop);

	vop_win_init(vop);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vop->len = resource_size(res);
	vop->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(vop->regs))
		return PTR_ERR(vop->regs);

	vop->regsbak = devm_kzalloc(dev, vop->len, GFP_KERNEL);
	if (!vop->regsbak)
		return -ENOMEM;

	ret = vop_initial(vop);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot initial vop dev - err %d\n", ret);
		return ret;
	}

	vop->irq = platform_get_irq(pdev, 0);
	if (vop->irq < 0) {
		dev_err(dev, "cannot find irq for vop\n");
		return vop->irq;
	}

	spin_lock_init(&vop->reg_lock);
	spin_lock_init(&vop->irq_lock);

	mutex_init(&vop->vsync_mutex);

	ret = devm_request_threaded_irq(dev, vop->irq, vop_isr, vop_isr_thread,
					IRQF_SHARED, dev_name(dev), vop);
	if (ret)
		return ret;

	/* IRQ is initially disabled; it gets enabled in power_on */
	disable_irq(vop->irq);

	ret = vop_create_crtc(vop);
	if (ret)
		return ret;

	pm_runtime_enable(&pdev->dev);
	return 0;
}

static void vop_unbind(struct device *dev, struct device *master, void *data)
{
	struct vop *vop = dev_get_drvdata(dev);

	pm_runtime_disable(dev);
	vop_destroy_crtc(vop);
}

static const struct component_ops vop_component_ops = {
	.bind = vop_bind,
	.unbind = vop_unbind,
};

static int vop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->of_node) {
		dev_err(dev, "can't find vop devices\n");
		return -ENODEV;
	}

	return component_add(dev, &vop_component_ops);
}

static int vop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vop_component_ops);

	return 0;
}

struct platform_driver vop_platform_driver = {
	.probe = vop_probe,
	.remove = vop_remove,
	.driver = {
		.name = "rockchip-vop",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vop_driver_dt_match),
	},
};

module_platform_driver(vop_platform_driver);

MODULE_AUTHOR("Mark Yao <mark.yao@rock-chips.com>");
MODULE_DESCRIPTION("ROCKCHIP VOP Driver");
MODULE_LICENSE("GPL v2");
