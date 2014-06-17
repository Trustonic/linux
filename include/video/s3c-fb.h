/* linux/drivers/video/s3c-fb.c
 *
 * Copyright 2008 Openmoko Inc.
 * Copyright 2008-2010 Simtec Electronics
 *      Ben Dooks <ben@simtec.co.uk>
 *      http://armlinux.simtec.co.uk/
 *
 * Samsung SoC Framebuffer driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software FoundatIon.
*/

#ifndef __S3C_FB_H
#define __S3C_FB_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/kthread.h>

#if defined(CONFIG_FB_EXYNOS_FIMD_MC) || defined(CONFIG_FB_EXYNOS_FIMD_MC_WB)
#include <media/v4l2-subdev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/exynos_mc.h>
#include <plat/map-base.h>
#endif

#include <mach/exynos5_bus.h>
#include <mach/map.h>
#include <plat/regs-fb-v4.h>
#include <plat/fb.h>

#ifdef CONFIG_ION_EXYNOS
#include <linux/dma-buf.h>
#include <linux/exynos_ion.h>
#include <linux/ion.h>
#include <linux/highmem.h>
#include <linux/memblock.h>
#include <linux/sw_sync.h>
#include <plat/devs.h>
#include <plat/iovmm.h>
#include <plat/sysmmu.h>
#include <mach/sysmmu.h>
#endif

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#define FIMD_PADS_NUM               1

/* This driver will export a number of framebuffer interfaces depending
 * on the configuration passed in via the platform data. Each fb instance
 * maps to a hardware window. Currently there is no support for runtime
 * setting of the alpha-blending functions that each window has, so only
 * window 0 is actually useful.
 *
 * Window 0 is treated specially, it is used for the basis of the LCD
 * output timings and as the control for the output power-down state.
 */

/* note, the previous use of <mach/regs-fb.h> to get platform specific data
 * has been replaced by using the platform device name to pick the correct
 * configuration data for the system.
 */

/**
 * struct s3c_fb_variant - fb variant information
 * @is_2443: Set if S3C2443/S3C2416 style hardware.
 * @nr_windows: The number of windows.
 * @vidtcon: The base for the VIDTCONx registers
 * @wincon: The base for the WINxCON registers.
 * @winmap: The base for the WINxMAP registers.
 * @keycon: The abse for the WxKEYCON registers.
 * @buf_start: Offset of buffer start registers.
 * @buf_size: Offset of buffer size registers.
 * @buf_end: Offset of buffer end registers.
 * @osd: The base for the OSD registers.
 * @palette: Address of palette memory, or 0 if none.
 * @has_prtcon: Set if has PRTCON register.
 * @has_shadowcon: Set if has SHADOWCON register.
 * @has_blendcon: Set if has BLENDCON register.
 * @has_alphacon: Set if has VIDWALPHA register.
 * @has_clksel: Set if VIDCON0 register has CLKSEL bit.
 * @has_fixvclk: Set if VIDCON1 register has FIXVCLK bits.
 */
struct s3c_fb_variant {
  unsigned int    is_2443:1;
  unsigned short  nr_windows;
  unsigned int    vidtcon;
  unsigned short  wincon;
  unsigned short  winmap;
  unsigned short  keycon;
  unsigned short  buf_start;
  unsigned short  buf_end;
  unsigned short  buf_size;
  unsigned short  osd;
  unsigned short  osd_stride;
  unsigned short  palette[S3C_FB_MAX_WIN];

  unsigned int    has_prtcon:1;
  unsigned int    has_shadowcon:1;
  unsigned int    has_blendcon:1;
  unsigned int    has_alphacon:1;
  unsigned int    has_clksel:1;
  unsigned int    has_fixvclk:1;
};

/**
 * struct s3c_fb_win_variant
 * @has_osd_c: Set if has OSD C register.
 * @has_osd_d: Set if has OSD D register.
 * @has_osd_alpha: Set if can change alpha transparency for a window.
 * @palette_sz: Size of palette in entries.
 * @palette_16bpp: Set if palette is 16bits wide.
 * @osd_size_off: If != 0, supports setting up OSD for a window; the appropriate
 *                register is located at the given offset from OSD_BASE.
 * @valid_bpp: 1 bit per BPP setting to show valid bits-per-pixel.
 *
 * valid_bpp bit x is set if (x+1)BPP is supported.
 */
struct s3c_fb_win_variant {
  unsigned int    has_osd_c:1;
  unsigned int    has_osd_d:1;
  unsigned int    has_osd_alpha:1;
  unsigned int    palette_16bpp:1;
  unsigned short  osd_size_off;
  unsigned short  palette_sz;
  u32     valid_bpp;
};

/**
 * struct s3c_fb_driverdata - per-device type driver data for init time.
 * @variant: The variant information for this driver.
 * @win: The window information for each window.
 */
struct s3c_fb_driverdata {
  struct s3c_fb_variant   variant;
  struct s3c_fb_win_variant *win[S3C_FB_MAX_WIN];
};

/**
 * struct s3c_fb_palette - palette information
 * @r: Red bitfield.
 * @g: Green bitfield.
 * @b: Blue bitfield.
 * @a: Alpha bitfield.
 */
struct s3c_fb_palette {
  struct fb_bitfield  r;
  struct fb_bitfield  g;
  struct fb_bitfield  b;
  struct fb_bitfield  a;
};

#ifdef CONFIG_ION_EXYNOS
struct s3c_dma_buf_data {
  struct ion_handle *ion_handle;
  struct dma_buf *dma_buf;
  struct dma_buf_attachment *attachment;
  struct sg_table *sg_table;
  dma_addr_t dma_addr;
  struct sync_fence *fence;
};

struct s3c_reg_data {
  struct list_head    list;
  u32         shadowcon;
  u32         wincon[S3C_FB_MAX_WIN];
  u32         win_rgborder[S3C_FB_MAX_WIN];
  u32         winmap[S3C_FB_MAX_WIN];
  u32         vidosd_a[S3C_FB_MAX_WIN];
  u32         vidosd_b[S3C_FB_MAX_WIN];
  u32         vidosd_c[S3C_FB_MAX_WIN];
  u32         vidosd_d[S3C_FB_MAX_WIN];
  u32         vidw_alpha0[S3C_FB_MAX_WIN];
  u32         vidw_alpha1[S3C_FB_MAX_WIN];
  u32         blendeq[S3C_FB_MAX_WIN - 1];
  u32         vidw_buf_start[S3C_FB_MAX_WIN];
  u32         vidw_buf_end[S3C_FB_MAX_WIN];
  u32         vidw_buf_size[S3C_FB_MAX_WIN];
  struct s3c_dma_buf_data dma_buf_data[S3C_FB_MAX_WIN];
  unsigned int        bandwidth;
};
#endif

/**
 * struct s3c_fb_win - per window private data for each framebuffer.
 * @windata: The platform data supplied for the window configuration.
 * @parent: The hardware that this window is part of.
 * @fbinfo: Pointer pack to the framebuffer info for this window.
 * @varint: The variant information for this window.
 * @palette_buffer: Buffer/cache to hold palette entries.
 * @pseudo_palette: For use in TRUECOLOUR modes for entries 0..15/
 * @index: The window number of this window.
 * @palette: The bitfields for changing r/g/b into a hardware palette entry.
 */
struct s3c_fb_win {
  struct s3c_fb_pd_win    *windata;
  struct s3c_fb       *parent;
  struct fb_info      *fbinfo;
  struct s3c_fb_palette    palette;
  struct s3c_fb_win_variant variant;

  u32         *palette_buffer;
  u32          pseudo_palette[16];
  unsigned int         index;
#ifdef CONFIG_ION_EXYNOS
  struct s3c_dma_buf_data dma_buf_data;
  struct fb_var_screeninfo prev_var;
  struct fb_fix_screeninfo prev_fix;
#endif

  int         fps;

#ifdef CONFIG_FB_EXYNOS_FIMD_MC
  int use;        /* use of widnow subdev in fimd */
  int local;      /* use of local path gscaler to window in fimd */
  struct media_pad pads[FIMD_PADS_NUM];   /* window's pad : 1 sink */
  struct v4l2_subdev sd;      /* Take a window as a v4l2_subdevice */
  int end_stream;
#endif
};

/**
 * struct s3c_fb_vsync - vsync information
 * @wait:     a queue for processes waiting for vsync
 * @timestamp:        the time of the last vsync interrupt
 * @active:       whether userspace is requesting vsync notifications
 * @irq_refcount: reference count for the underlying irq
 * @irq_lock:     mutex protecting the irq refcount and register
 * @thread:       notification-generating thread
 */
struct s3c_fb_vsync {
  wait_queue_head_t   wait;
  ktime_t         timestamp;
  bool            active;
  int         irq_refcount;
  struct mutex        irq_lock;
  struct task_struct  *thread;
};

#ifdef CONFIG_DEBUG_FS
#define S3C_FB_DEBUG_FIFO_TIMESTAMPS 32
#define S3C_FB_DEBUG_REGS_SIZE 0x0280

struct s3c_fb_debug {
  ktime_t     fifo_timestamps[S3C_FB_DEBUG_FIFO_TIMESTAMPS];
  unsigned int    num_timestamps;
  unsigned int    first_timestamp;
  u8      regs_at_underflow[S3C_FB_DEBUG_REGS_SIZE];
};
#endif

/**
 * struct s3c_fb - overall hardware state of the hardware
 * @slock: The spinlock protection for this data sturcture.
 * @dev: The device that we bound to, for printing, etc.
 * @bus_clk: The clk (hclk) feeding our interface and possibly pixclk.
 * @lcd_clk: The clk (sclk) feeding pixclk.
 * @regs: The mapped hardware registers.
 * @variant: Variant information for this hardware.
 * @enabled: A bitmask of enabled hardware windows.
 * @output_on: Flag if the physical output is enabled.
 * @pdata: The platform configuration data passed with the device.
 * @windows: The hardware windows that have been claimed.
 * @irq_no: IRQ line number
 * @vsync_info: VSYNC-related information (count, queues...)
 */
struct s3c_fb {
  spinlock_t      slock;
  struct device       *dev;
  struct clk      *bus_clk;
  struct clk      *lcd_clk;
  void __iomem        *regs;
  struct s3c_fb_variant    variant;

  bool            output_on;
  struct mutex        output_lock;

  struct s3c_fb_platdata  *pdata;
  struct s3c_fb_win   *windows[S3C_FB_MAX_WIN];

  int          irq_no;
  struct s3c_fb_vsync  vsync_info;

#ifdef CONFIG_ION_EXYNOS
  struct ion_client   *fb_ion_client;

  struct list_head    update_regs_list;
  struct mutex        update_regs_list_lock;
  struct kthread_worker   update_regs_worker;
  struct task_struct  *update_regs_thread;
  struct kthread_work update_regs_work;

  struct sw_sync_timeline *timeline;
  int         timeline_max;
#endif

#ifdef CONFIG_FB_EXYNOS_FIMD_MC
  struct exynos_md *md;
  unsigned int win_index;
#endif
#ifdef CONFIG_FB_EXYNOS_FIMD_MC_WB
  struct exynos_md *md_wb;
  int use_wb; /* use of fimd subdev for writeback */
  int local_wb;   /* use of writeback path to gscaler in fimd */
  struct media_pad pads_wb;   /* FIMD1's pad */
  struct v4l2_subdev sd_wb;   /* Take a FIMD1 as a v4l2_subdevice */
#endif

#ifdef CONFIG_DEBUG_FS
  struct dentry       *debug_dentry;
  struct s3c_fb_debug debug_data;
#endif
  struct exynos5_bus_mif_handle *fb_mif_handle;
  struct exynos5_bus_int_handle *fb_int_handle;

};


void s3c_fb_deactivate_vsync(struct s3c_fb *sfb);
void s3c_fb_activate_vsync(struct s3c_fb *sfb);

#endif /* __S3C_FB_H */
