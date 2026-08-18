// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC VGA framebuffer driver
 *
 * Supports the FPGA VGA/DMA controller on the Loongson Education SoC.
 * Registers (physical 0x1fe90000):
 *   0x00 FB_ADDR   framebuffer base address (physical)
 *   0x04 FB_STRIDE bytes per line
 *   0x08 CTRL      bit0: enable framebuffer/DMA
 *   0x0C STATUS    read-only status
 */
#include <linux/aperture.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>

#define VGA_FB_ADDR		0x00
#define VGA_FB_STRIDE		0x04
#define VGA_CTRL		0x08
#define VGA_STATUS		0x0C

#define VGA_CTRL_ENABLE		BIT(0)

#define VGA_XRES		640
#define VGA_YRES		480
#define VGA_BPP			16
#define VGA_STRIDE		(VGA_XRES * VGA_BPP / 8)

#define PSEUDO_PALETTE_SIZE	16

struct loongson_soc_vga_par {
	void __iomem *regs;
	unsigned long fb_phys;
	unsigned long fb_size;
	struct resource *mem;
	u32 palette[PSEUDO_PALETTE_SIZE];
};

static int loongson_soc_vga_setcolreg(u_int regno, u_int red, u_int green,
				      u_int blue, u_int transp,
				      struct fb_info *info)
{
	u32 *pal = info->pseudo_palette;
	u32 cr = red >> (16 - info->var.red.length);
	u32 cg = green >> (16 - info->var.green.length);
	u32 cb = blue >> (16 - info->var.blue.length);
	u32 value;

	if (regno >= PSEUDO_PALETTE_SIZE)
		return -EINVAL;

	value = (cr << info->var.red.offset) |
		(cg << info->var.green.offset) |
		(cb << info->var.blue.offset);
	if (info->var.transp.length > 0) {
		u32 mask = (1 << info->var.transp.length) - 1;

		mask <<= info->var.transp.offset;
		value |= mask;
	}
	pal[regno] = value;

	return 0;
}

static void loongson_soc_vga_destroy(struct fb_info *info)
{
	struct loongson_soc_vga_par *par = info->par;
	struct resource *mem = par->mem;

	if (info->screen_base)
		iounmap(info->screen_base);

	framebuffer_release(info);

	if (mem)
		release_mem_region(mem->start, resource_size(mem));
}

static const struct fb_ops loongson_soc_vga_ops = {
	.owner		= THIS_MODULE,
	FB_DEFAULT_IOMEM_OPS,
	.fb_destroy	= loongson_soc_vga_destroy,
	.fb_setcolreg	= loongson_soc_vga_setcolreg,
};

static int loongson_soc_vga_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_soc_vga_par *par;
	struct fb_info *info;
	struct resource fb_res;
	struct resource *mem;
	struct resource *regs_res;
	void __iomem *regs;
	int ret;

	regs_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_res) {
		dev_err(dev, "no register resource\n");
		return -ENODEV;
	}

	regs = devm_ioremap_resource(dev, regs_res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &fb_res);
	if (ret) {
		dev_err(dev, "no framebuffer memory-region: %d\n", ret);
		return ret;
	}

	mem = request_mem_region(fb_res.start, resource_size(&fb_res),
				 "loongson-soc-vga-fb");
	if (!mem) {
		dev_warn(dev, "cannot reserve framebuffer at %pR\n", &fb_res);
		mem = &fb_res;
	}

	info = framebuffer_alloc(sizeof(*par), dev);
	if (!info) {
		ret = -ENOMEM;
		goto err_release_mem;
	}
	platform_set_drvdata(pdev, info);

	par = info->par;
	par->regs = regs;
	par->fb_phys = mem->start;
	par->fb_size = resource_size(mem);
	if (mem != &fb_res)
		par->mem = mem;

	info->fix = (struct fb_fix_screeninfo) {
		.id		= "lsoc_vga",
		.type		= FB_TYPE_PACKED_PIXELS,
		.visual		= FB_VISUAL_TRUECOLOR,
		.accel		= FB_ACCEL_NONE,
		.smem_start	= mem->start,
		.smem_len	= resource_size(mem),
		.line_length	= VGA_STRIDE,
	};

	info->var = (struct fb_var_screeninfo) {
		.xres		= VGA_XRES,
		.yres		= VGA_YRES,
		.xres_virtual	= VGA_XRES,
		.yres_virtual	= VGA_YRES,
		.bits_per_pixel	= VGA_BPP,
		.red		= { .offset = 11, .length = 5 },
		.green		= { .offset = 5, .length = 6 },
		.blue		= { .offset = 0, .length = 5 },
		.activate	= FB_ACTIVATE_NOW,
		.height		= -1,
		.width		= -1,
		.vmode		= FB_VMODE_NONINTERLACED,
	};

	info->screen_base = ioremap_wc(mem->start, resource_size(mem));
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto err_fb_release;
	}
	info->pseudo_palette = par->palette;
	info->fbops = &loongson_soc_vga_ops;

	writel(lower_32_bits(mem->start), regs + VGA_FB_ADDR);
	writel(VGA_STRIDE, regs + VGA_FB_STRIDE);
	writel(VGA_CTRL_ENABLE, regs + VGA_CTRL);

	ret = devm_aperture_acquire_for_platform_device(pdev, mem->start,
							resource_size(mem));
	if (ret) {
		dev_err(dev, "cannot acquire aperture: %d\n", ret);
		goto err_unmap;
	}

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(dev, "failed to register framebuffer: %d\n", ret);
		goto err_unmap;
	}

	dev_info(dev, "fb%d: Loongson SoC VGA framebuffer at 0x%lx, %lu bytes\n",
		 info->node, par->fb_phys, par->fb_size);
	dev_info(dev, "mode %dx%d@%d, line length %d\n",
		 info->var.xres, info->var.yres, info->var.bits_per_pixel,
		 info->fix.line_length);

	return 0;

err_unmap:
	iounmap(info->screen_base);
err_fb_release:
	framebuffer_release(info);
err_release_mem:
	if (mem != &fb_res)
		release_mem_region(mem->start, resource_size(mem));
	return ret;
}

static void loongson_soc_vga_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);

	unregister_framebuffer(info);
}

static const struct of_device_id loongson_soc_vga_of_match[] = {
	{ .compatible = "loongson-edu,vga" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_soc_vga_of_match);

static struct platform_driver loongson_soc_vga_driver = {
	.probe		= loongson_soc_vga_probe,
	.remove		= loongson_soc_vga_remove,
	.driver		= {
		.name		= "loongson-soc-vga",
		.of_match_table	= loongson_soc_vga_of_match,
	},
};
module_platform_driver(loongson_soc_vga_driver);

MODULE_AUTHOR("Loongson Education FPGA Lab");
MODULE_DESCRIPTION("Loongson SoC VGA framebuffer driver");
MODULE_LICENSE("GPL v2");
