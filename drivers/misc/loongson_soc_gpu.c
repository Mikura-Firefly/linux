// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC Simple GPU driver
 *
 * The FPGA simple GPU exposes a command FIFO through three MMIO registers:
 *   0x00 CMD    write one 32-bit command word (stalls while FIFO full)
 *   0x04 STATUS bit0 = FIFO full, bit1 = FIFO empty, bit2 = interrupt pending
 *   0x08 INT_CLR write any value to clear the interrupt flag
 *
 * RLE/DECODE_FRAME data lives in a large physically-contiguous reserved-memory
 * region described by the "memory-region" DT property.  The driver ioremaps it
 * and exposes it to userspace through /dev/gpu mmap(); userspace writes RLE
 * data there and passes the physical address directly to DECODE_FRAME.
 *
 * When CONFIG_FB is enabled the driver also registers /dev/fb0 as an 8-bit
 * indexed framebuffer.  fbcon/logo draw into a shadow buffer and a workqueue
 * uploads the changed screen to the GPU through DECODE_FRAME/FLIP.
 */
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <asm/processor.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#endif

#define GPU_REG_CMD		0x00
#define GPU_REG_STAT		0x04
#define GPU_REG_INT_CLR		0x08

#define GPU_STATUS_FULL		BIT(0)
#define GPU_STATUS_EMPTY	BIT(1)
#define GPU_STATUS_INT		BIT(2)

#define GPU_IOC_MAGIC		0x47	/* 'G' */

#define GPU_IOC_GET_STATUS	_IOR(GPU_IOC_MAGIC, 1, u32)
#define GPU_IOC_WAIT_FENCE	_IOW(GPU_IOC_MAGIC, 2, u32)
#define GPU_IOC_CLEAR_IRQ	_IO(GPU_IOC_MAGIC, 3)
#define GPU_IOC_GET_BUF_PHYS	_IOR(GPU_IOC_MAGIC, 4, u32)
#define GPU_IOC_GET_BUF_SIZE	_IOR(GPU_IOC_MAGIC, 5, u32)

#define GPU_CMD_SET_DISPLAY	0x01
#define GPU_CMD_LOAD_PALETTE	0x02
#define GPU_CMD_DECODE_FRAME	0x03
#define GPU_CMD_FILL_RECT	0x04
#define GPU_CMD_COPY_RECT	0x05
#define GPU_CMD_FLIP		0x06
#define GPU_CMD_FENCE		0x07
#define GPU_CMD_COPY_FRAME	0x08

#ifdef CONFIG_FB
#define GPU_FB_W		640
#define GPU_FB_H		480
#define GPU_FB_BPP		8
#define GPU_FB_LINE		(GPU_FB_W * GPU_FB_BPP / 8)
#define GPU_FB_SIZE		(GPU_FB_W * GPU_FB_H)
#define GPU_TILE_W		160
#define GPU_TILE_H		240
#define GPU_TILE_COLS		4
#define GPU_TILE_ROWS		2
#define GPU_RLE_BUF_SIZE	SZ_1M
#endif

struct loongson_soc_gpu {
	void __iomem *regs;
	void __iomem *buf_cpu;
	struct device *dev;
	struct miscdevice misc;

	phys_addr_t buf_phys;
	size_t buf_size;

#ifdef CONFIG_FB
	struct fb_info *fb;
	u8 *fb_shadow;
	u32 *fb_rle;
	u32 fb_palette[256];
	bool fb_dirty;
	bool fb_palette_dirty;
	struct work_struct fb_work;
#endif
};

static void gpu_cmd_write(struct loongson_soc_gpu *gpu, u32 word)
{
	while (readl(gpu->regs + GPU_REG_STAT) & GPU_STATUS_FULL) {
		if (need_resched())
			cond_resched();
	}
	writel(word, gpu->regs + GPU_REG_CMD);
}

static void gpu_cmd_write_n(struct loongson_soc_gpu *gpu, const u32 *cmds, int n)
{
	int i;

	for (i = 0; i < n; i++)
		gpu_cmd_write(gpu, cmds[i]);
}

static int gpu_wait_fence(struct loongson_soc_gpu *gpu, int timeout_ms)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	while (!(readl(gpu->regs + GPU_REG_STAT) & GPU_STATUS_INT)) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}
	writel(1, gpu->regs + GPU_REG_INT_CLR);
	return 0;
}

static int loongson_soc_gpu_open(struct inode *inode, struct file *file)
{
	struct loongson_soc_gpu *gpu =
		container_of(file->private_data, struct loongson_soc_gpu, misc);

	file->private_data = gpu;
	return 0;
}

static ssize_t loongson_soc_gpu_write(struct file *file, const char __user *buf,
				      size_t count, loff_t *ppos)
{
	struct loongson_soc_gpu *gpu = file->private_data;
	u32 tmp[64];
	size_t done = 0;

	if (count & 3)
		return -EINVAL;

	while (done < count) {
		size_t chunk = min_t(size_t, count - done, sizeof(tmp));
		int i;

		if (copy_from_user(tmp, buf + done, chunk))
			return -EFAULT;

		for (i = 0; i < chunk / 4; i++)
			gpu_cmd_write(gpu, tmp[i]);

		done += chunk;
	}

	return done;
}

static long loongson_soc_gpu_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	struct loongson_soc_gpu *gpu = file->private_data;
	u32 val;
	u32 timeout_ms;

	switch (cmd) {
	case GPU_IOC_GET_STATUS:
		val = readl(gpu->regs + GPU_REG_STAT);
		if (copy_to_user((void __user *)arg, &val, sizeof(val)))
			return -EFAULT;
		return 0;

	case GPU_IOC_WAIT_FENCE:
		timeout_ms = 1000;
		if (copy_from_user(&timeout_ms, (void __user *)arg,
				   sizeof(timeout_ms)))
			return -EFAULT;
		return gpu_wait_fence(gpu, timeout_ms);

	case GPU_IOC_CLEAR_IRQ:
		writel(1, gpu->regs + GPU_REG_INT_CLR);
		return 0;

	case GPU_IOC_GET_BUF_PHYS:
		val = (u32)gpu->buf_phys;
		if (copy_to_user((void __user *)arg, &val, sizeof(val)))
			return -EFAULT;
		return 0;

	case GPU_IOC_GET_BUF_SIZE:
		val = (u32)gpu->buf_size;
		if (copy_to_user((void __user *)arg, &val, sizeof(val)))
			return -EFAULT;
		return 0;

	default:
		return -ENOTTY;
	}
}

static int loongson_soc_gpu_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct loongson_soc_gpu *gpu = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;

	if (size > gpu->buf_size)
		return -EINVAL;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, gpu->buf_phys >> PAGE_SHIFT,
			    size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static __poll_t loongson_soc_gpu_poll(struct file *file, poll_table *wait)
{
	struct loongson_soc_gpu *gpu = file->private_data;
	__poll_t mask = 0;

	if (readl(gpu->regs + GPU_REG_STAT) & GPU_STATUS_INT)
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static const struct file_operations loongson_soc_gpu_fops = {
	.owner		= THIS_MODULE,
	.open		= loongson_soc_gpu_open,
	.write		= loongson_soc_gpu_write,
	.unlocked_ioctl	= loongson_soc_gpu_ioctl,
	.mmap		= loongson_soc_gpu_mmap,
	.poll		= loongson_soc_gpu_poll,
};

#ifdef CONFIG_FB

static void gpu_fb_send_set_display(struct loongson_soc_gpu *gpu)
{
	u32 c[5] = {
		(GPU_CMD_SET_DISPLAY << 24) | (4 << 16),
		80, 0, 0, 0,
	};

	gpu_cmd_write_n(gpu, c, 5);
}

static void gpu_fb_send_palette(struct loongson_soc_gpu *gpu)
{
	u32 c[129];
	int i;

	c[0] = (GPU_CMD_LOAD_PALETTE << 24) | (128 << 16);
	for (i = 0; i < 128; i++)
		c[1 + i] = (gpu->fb_palette[i * 2 + 1] << 16) |
			   gpu->fb_palette[i * 2];
	gpu_cmd_write_n(gpu, c, 129);
	gpu->fb_palette_dirty = false;
}

/* Encode one 160x240 tile as packed 16-bit RLE tokens into out.
 * Returns number of 32-bit words used.
 */
static u32 gpu_fb_encode_tile(u32 *out, const u8 *shadow, int tile_row,
			      int tile_col)
{
	u32 words = 0;
	u16 pending = 0;
	bool have_pending = false;
	int ly, lx;

	for (ly = 0; ly < GPU_TILE_H; ly++) {
		lx = 0;
		while (lx < GPU_TILE_W) {
			int x = tile_col * GPU_TILE_W + lx;
			int y = tile_row * GPU_TILE_H + ly;
			u8 idx = shadow[y * GPU_FB_W + x];
			int run = 1;

			while (lx + run < GPU_TILE_W) {
				int nx = tile_col * GPU_TILE_W + lx + run;
				if (shadow[y * GPU_FB_W + nx] != idx)
					break;
				run++;
			}
			if (run > 255)
				run = 255;

			if (have_pending) {
				out[words++] = pending | ((u16)((run << 8) | idx) << 16);
				have_pending = false;
			} else {
				pending = (u16)((run << 8) | idx);
				have_pending = true;
			}
			lx += run;
		}
	}

	if (have_pending)
		out[words++] = pending;
	return words;
}

static void gpu_fb_upload(struct work_struct *work)
{
	struct loongson_soc_gpu *gpu =
		container_of(work, struct loongson_soc_gpu, fb_work);
	u32 tile_off[8], tile_words[8];
	u32 total_words = 0;
	u32 cmds[17];
	int i;

	if (!gpu->fb_dirty && !gpu->fb_palette_dirty)
		return;

	/* Encode all eight tiles into the kernel RLE buffer. */
	for (i = 0; i < 8; i++) {
		int tr = i / GPU_TILE_COLS;
		int tc = i % GPU_TILE_COLS;
		tile_off[i] = total_words;
		tile_words[i] = gpu_fb_encode_tile(gpu->fb_rle + total_words,
						   gpu->fb_shadow, tr, tc);
		total_words += tile_words[i];
	}

	memcpy_toio(gpu->buf_cpu, gpu->fb_rle, total_words * 4);

	if (gpu->fb_palette_dirty)
		gpu_fb_send_palette(gpu);

	cmds[0] = (GPU_CMD_DECODE_FRAME << 24) | (16 << 16);
	for (i = 0; i < 8; i++) {
		cmds[1 + i * 2] = (u32)(gpu->buf_phys + tile_off[i] * 4);
		cmds[1 + i * 2 + 1] = tile_words[i] * 4;
	}
	gpu_cmd_write_n(gpu, cmds, 17);

	{
		u32 flip[3] = {
			(GPU_CMD_FLIP << 24),
			(GPU_CMD_FENCE << 24) | (1 << 16),
			0xAA,
		};
		gpu_cmd_write_n(gpu, flip, 3);
	}
	gpu_wait_fence(gpu, 1000);

	gpu->fb_dirty = false;
}

static void gpu_fb_mark_dirty(struct loongson_soc_gpu *gpu)
{
	gpu->fb_dirty = true;
	schedule_work(&gpu->fb_work);
}

static int gpu_fb_setcolreg(unsigned int regno, unsigned int red,
			    unsigned int green, unsigned int blue,
			    unsigned int transp, struct fb_info *info)
{
	struct loongson_soc_gpu *gpu = info->par;
	u32 r, g, b;

	if (regno >= 256)
		return -EINVAL;

	r = red >> (16 - 5);
	g = green >> (16 - 6);
	b = blue >> (16 - 5);
	gpu->fb_palette[regno] = (r << 11) | (g << 5) | b;
	gpu->fb_palette_dirty = true;
	gpu_fb_mark_dirty(gpu);

	return 0;
}

static void gpu_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct loongson_soc_gpu *gpu = info->par;

	sys_fillrect(info, rect);
	gpu_fb_mark_dirty(gpu);
}

static void gpu_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct loongson_soc_gpu *gpu = info->par;

	sys_copyarea(info, area);
	gpu_fb_mark_dirty(gpu);
}

static void gpu_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct loongson_soc_gpu *gpu = info->par;

	sys_imageblit(info, image);
	gpu_fb_mark_dirty(gpu);
}

static ssize_t gpu_fb_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct loongson_soc_gpu *gpu = info->par;
	ssize_t ret;

	ret = fb_sys_write(info, buf, count, ppos);
	if (ret > 0)
		gpu_fb_mark_dirty(gpu);
	return ret;
}

static int gpu_fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	return 0;
}

static int gpu_fb_sync(struct fb_info *info)
{
	struct loongson_soc_gpu *gpu = info->par;

	flush_work(&gpu->fb_work);
	return 0;
}

static void gpu_fb_destroy(struct fb_info *info)
{
	struct loongson_soc_gpu *gpu = info->par;

	if (gpu->fb) {
		unregister_framebuffer(gpu->fb);
		if (gpu->fb->cmap.len)
			fb_dealloc_cmap(&gpu->fb->cmap);
		framebuffer_release(gpu->fb);
		gpu->fb = NULL;
	}
}

static const struct fb_ops gpu_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= gpu_fb_write,
	.fb_fillrect	= gpu_fb_fillrect,
	.fb_copyarea	= gpu_fb_copyarea,
	.fb_imageblit	= gpu_fb_imageblit,
	.fb_setcolreg	= gpu_fb_setcolreg,
	.fb_pan_display	= gpu_fb_pan_display,
	.fb_sync	= gpu_fb_sync,
	.fb_destroy	= gpu_fb_destroy,
};

static int gpu_fb_init(struct loongson_soc_gpu *gpu)
{
	struct fb_info *info;
	int ret;

	gpu->fb_shadow = devm_kzalloc(gpu->dev, GPU_FB_SIZE, GFP_KERNEL);
	if (!gpu->fb_shadow)
		return -ENOMEM;

	gpu->fb_rle = devm_kzalloc(gpu->dev, GPU_RLE_BUF_SIZE, GFP_KERNEL);
	if (!gpu->fb_rle)
		return -ENOMEM;

	INIT_WORK(&gpu->fb_work, gpu_fb_upload);

	info = framebuffer_alloc(0, gpu->dev);
	if (!info)
		return -ENOMEM;
	info->par = gpu;
	info->screen_base = gpu->fb_shadow;
	info->fbops = &gpu_fb_ops;

	strcpy(info->fix.id, "loongson-gpu");
	info->fix.smem_start = virt_to_phys(gpu->fb_shadow);
	info->fix.smem_len = GPU_FB_SIZE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	info->fix.line_length = GPU_FB_LINE;
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;

	info->var.xres = GPU_FB_W;
	info->var.yres = GPU_FB_H;
	info->var.xres_virtual = GPU_FB_W;
	info->var.yres_virtual = GPU_FB_H;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.bits_per_pixel = GPU_FB_BPP;
	info->var.grayscale = 0;
	info->var.nonstd = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = -1;
	info->var.width = -1;
	info->var.accel_flags = 0;
	info->var.pixclock = 0;
	info->var.left_margin = 0;
	info->var.right_margin = 0;
	info->var.upper_margin = 0;
	info->var.lower_margin = 0;
	info->var.hsync_len = 0;
	info->var.vsync_len = 0;
	info->var.sync = 0;
	info->var.vmode = FB_VMODE_NONINTERLACED;
	info->var.red.offset = 0;
	info->var.red.length = 8;
	info->var.green.offset = 0;
	info->var.green.length = 8;
	info->var.blue.offset = 0;
	info->var.blue.length = 8;
	info->var.transp.offset = 0;
	info->var.transp.length = 0;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret) {
		framebuffer_release(info);
		return ret;
	}

	gpu->fb = info;
	gpu_fb_send_set_display(gpu);
	gpu->fb_palette_dirty = true;
	gpu->fb_dirty = true;
	schedule_work(&gpu->fb_work);

	ret = register_framebuffer(info);
	if (ret) {
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
		gpu->fb = NULL;
		return ret;
	}

	dev_info(gpu->dev, "registered /dev/fb0 (%dx%d, 8bpp indexed)\n",
		 GPU_FB_W, GPU_FB_H);
	return 0;
}
#endif /* CONFIG_FB */

static int loongson_soc_gpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct resource res;
	struct loongson_soc_gpu *gpu;
	int ret;

	gpu = devm_kzalloc(dev, sizeof(*gpu), GFP_KERNEL);
	if (!gpu)
		return -ENOMEM;

	gpu->dev = dev;
	gpu->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gpu->regs))
		return PTR_ERR(gpu->regs);

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "missing memory-region phandle\n");
		return -ENODEV;
	}
	ret = of_address_to_resource(np, 0, &res);
	of_node_put(np);
	if (ret) {
		dev_err(dev, "failed to parse memory-region\n");
		return ret;
	}

	gpu->buf_phys = res.start;
	gpu->buf_size = resource_size(&res);
	gpu->buf_cpu = ioremap(gpu->buf_phys, gpu->buf_size);
	if (!gpu->buf_cpu) {
		dev_err(dev, "failed to ioremap GPU shared region\n");
		return -ENOMEM;
	}

	gpu->misc.minor = MISC_DYNAMIC_MINOR;
	gpu->misc.name = "gpu";
	gpu->misc.fops = &loongson_soc_gpu_fops;
	gpu->misc.parent = dev;

	ret = misc_register(&gpu->misc);
	if (ret) {
		iounmap(gpu->buf_cpu);
		return ret;
	}

#ifdef CONFIG_FB
	ret = gpu_fb_init(gpu);
	if (ret) {
		dev_err(dev, "failed to init GPU framebuffer: %d\n", ret);
		misc_deregister(&gpu->misc);
		iounmap(gpu->buf_cpu);
		return ret;
	}
#endif

	platform_set_drvdata(pdev, gpu);
	dev_info(dev, "Loongson SoC GPU ready, shared phys=%pa size=%zu\n",
		 &gpu->buf_phys, gpu->buf_size);

	return 0;
}

static void loongson_soc_gpu_remove(struct platform_device *pdev)
{
	struct loongson_soc_gpu *gpu = platform_get_drvdata(pdev);

#ifdef CONFIG_FB
	if (gpu->fb)
		gpu_fb_destroy(gpu->fb);
#endif
	misc_deregister(&gpu->misc);
	iounmap(gpu->buf_cpu);
}

static const struct of_device_id loongson_soc_gpu_of_match[] = {
	{ .compatible = "loongson-edu,gpu" },
	{}
};
MODULE_DEVICE_TABLE(of, loongson_soc_gpu_of_match);

static struct platform_driver loongson_soc_gpu_driver = {
	.probe	= loongson_soc_gpu_probe,
	.remove	= loongson_soc_gpu_remove,
	.driver	= {
		.name		= "loongson-soc-gpu",
		.of_match_table	= loongson_soc_gpu_of_match,
	},
};
module_platform_driver(loongson_soc_gpu_driver);

MODULE_DESCRIPTION("Loongson SoC Simple GPU driver");
MODULE_LICENSE("GPL");
