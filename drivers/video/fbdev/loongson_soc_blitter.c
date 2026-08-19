// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson SoC 2D Blitter driver
 *
 * Registers (physical 0x1fea0000):
 *   0x00 SRC_ADDR / 0x04 DST_ADDR / 0x08 SRC_STRIDE / 0x0C DST_STRIDE
 *   0x10 WIDTH    / 0x14 HEIGHT  / 0x18 COLOR    / 0x1C CTRL
 *   0x20 STATUS
 *
 * The configuration registers 0x00..0x1C form one 32-byte cache line with
 * CTRL at the highest address.  They are mapped cached and written back with
 * CACOP so the hardware receives parameters first and START last.  STATUS is
 * mapped uncached.
 */
#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

#include "loongson_soc_blitter.h"

#define BLT_DRV_NAME		"loongson-soc-blitter"

#define BLT_SRC_ADDR		0x00
#define BLT_DST_ADDR		0x04
#define BLT_SRC_STRIDE		0x08
#define BLT_DST_STRIDE		0x0C
#define BLT_WIDTH		0x10
#define BLT_HEIGHT		0x14
#define BLT_COLOR		0x18
#define BLT_CTRL		0x1C
#define BLT_STATUS		0x20

#define BLT_CTRL_START		BIT(0)
#define BLT_CTRL_OP_COPY	BIT(1)
#define BLT_CTRL_IRQ_EN		BIT(2)

#define BLT_STATUS_BUSY		BIT(0)
#define BLT_STATUS_DONE		BIT(1)
#define BLT_STATUS_ERR		BIT(2)

#define BLT_CFG_SIZE		0x20
#define BLT_STATUS_SIZE		0x04
#define BLT_TIMEOUT_MS		1000

#define BLITTER_IOCTL_MAGIC	0xB1

struct blitter_op {
	__u32 op;
	__u32 src_addr;
	__u32 dst_addr;
	__u32 src_stride;
	__u32 dst_stride;
	__u32 width;
	__u32 height;
	__u32 color;
};

#define BLITTER_IOCTL_FILL	_IOW(BLITTER_IOCTL_MAGIC, 1, struct blitter_op)
#define BLITTER_IOCTL_COPY	_IOW(BLITTER_IOCTL_MAGIC, 2, struct blitter_op)

struct loongson_soc_blitter {
	void __iomem *cfg;	/* ioremap_cache, 0x00..0x1C */
	void __iomem *status;	/* ioremap, 0x20 */
	struct device *dev;
	struct miscdevice misc;
	int irq;
	struct completion done;
	spinlock_t lock;	/* protects the config window */

	void __iomem *fb_va;	/* cached mapping for cache maintenance */
	phys_addr_t fb_phys;
	size_t fb_size;
};

static struct loongson_soc_blitter *g_blitter;

static void loongson_soc_blitter_flush_line(void __iomem *addr)
{
	unsigned long a = (unsigned long)addr;

	cache_op(Hit_Writeback_Inv_LEAF1, a);
	asm volatile("dbar 0" ::: "memory");
}

static void loongson_soc_blitter_sync_range(struct loongson_soc_blitter *blt,
					    phys_addr_t phys, size_t size)
{
	unsigned long addr;
	unsigned long end;

	if (phys < blt->fb_phys || phys + size > blt->fb_phys + blt->fb_size)
		return;

	addr = (unsigned long)blt->fb_va + (phys - blt->fb_phys);
	end = addr + size;
	addr &= ~0xful;
	for (; addr < end; addr += 16)
		cache_op(Hit_Writeback_Inv_LEAF1, addr);

	asm volatile("dbar 0" ::: "memory");
}

static irqreturn_t loongson_soc_blitter_irq(int irq, void *data)
{
	struct loongson_soc_blitter *blt = data;
	u32 status = readl(blt->status);

	if (status & (BLT_STATUS_DONE | BLT_STATUS_ERR)) {
		writel(status & (BLT_STATUS_DONE | BLT_STATUS_ERR), blt->status);
		complete(&blt->done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int loongson_soc_blitter_exec(struct loongson_soc_blitter *blt,
					     const struct blitter_op *op)
{
	u32 __iomem *cfg = blt->cfg;
	unsigned long flags;
	u32 ctrl;
	u32 status;
	int ret = 0;

	if (op->width == 0 || op->height == 0)
		return -EINVAL;
	if (op->width & 1)
		return -EINVAL;
	if ((op->dst_addr & 3) || (op->op == 1 && (op->src_addr & 3)))
		return -EINVAL;
	if (op->dst_stride < op->width * 2)
		return -EINVAL;
	if (op->op == 1 && op->src_stride < op->width * 2)
		return -EINVAL;

	/* Make CPU writes visible to the Blitter master before it starts. */
	if (op->op == 1)
		loongson_soc_blitter_sync_range(blt, op->src_addr,
						op->height * op->src_stride);
	loongson_soc_blitter_sync_range(blt, op->dst_addr,
					op->height * op->dst_stride);

	/*
	 * Config window: mask all local IRQs and disable preemption so no
	 * interrupt can evict/write back a partially written config line.
	 * This is a short, non-sleeping critical section.
	 */
	spin_lock_irqsave(&blt->lock, flags);

	/* Clear stale DONE/ERR and arm the completion before START. */
	writel(BLT_STATUS_DONE | BLT_STATUS_ERR, blt->status);
	reinit_completion(&blt->done);

	ctrl = BLT_CTRL_START | BLT_CTRL_IRQ_EN;
	if (op->op == 1)
		ctrl |= BLT_CTRL_OP_COPY;

	writel(op->src_addr,   cfg + 0);	/* BLT_SRC_ADDR   */
	writel(op->dst_addr,   cfg + 1);	/* BLT_DST_ADDR   */
	writel(op->src_stride, cfg + 2);	/* BLT_SRC_STRIDE */
	writel(op->dst_stride, cfg + 3);	/* BLT_DST_STRIDE */
	writel(op->width,      cfg + 4);	/* BLT_WIDTH      */
	writel(op->height,     cfg + 5);	/* BLT_HEIGHT     */
	writel(op->color,      cfg + 6);	/* BLT_COLOR      */
	writel(ctrl,           cfg + 7);	/* BLT_CTRL, last: parameters then START */

	loongson_soc_blitter_flush_line(cfg);

	spin_unlock_irqrestore(&blt->lock, flags);

	if (!wait_for_completion_timeout(&blt->done,
					 msecs_to_jiffies(BLT_TIMEOUT_MS))) {
		/* IRQ not wired/failed: fall back to polling STATUS. */
		ret = readl_poll_timeout(blt->status, status,
					status & (BLT_STATUS_DONE | BLT_STATUS_ERR),
					100, BLT_TIMEOUT_MS * 1000);
		if (ret)
			return -ETIMEDOUT;
	}

	status = readl(blt->status);
	if (status & BLT_STATUS_ERR)
		return -EIO;

	return 0;
}

static int loongson_soc_blitter_open(struct inode *inode, struct file *file)
{
	struct loongson_soc_blitter *blt =
		container_of(file->private_data, struct loongson_soc_blitter, misc);

	file->private_data = blt;
	return 0;
}

static long loongson_soc_blitter_ioctl(struct file *file, unsigned int cmd,
				       unsigned long arg)
{
	struct loongson_soc_blitter *blt = file->private_data;
	struct blitter_op op;
	int ret;

	if (copy_from_user(&op, (void __user *)arg, sizeof(op)))
		return -EFAULT;

	switch (cmd) {
	case BLITTER_IOCTL_FILL:
		op.op = 0;
		break;
	case BLITTER_IOCTL_COPY:
		op.op = 1;
		break;
	default:
		return -ENOTTY;
	}

	ret = loongson_soc_blitter_exec(blt, &op);
	return ret;
}

static const struct file_operations loongson_soc_blitter_fops = {
	.owner		= THIS_MODULE,
	.open		= loongson_soc_blitter_open,
	.unlocked_ioctl	= loongson_soc_blitter_ioctl,
};

bool loongson_soc_blitter_available(void)
{
	return g_blitter != NULL;
}
EXPORT_SYMBOL_GPL(loongson_soc_blitter_available);

int loongson_soc_blitter_fill(u32 dst_addr, u32 dst_stride,
			      u32 width, u32 height, u32 color)
{
	struct blitter_op op = {
		.op		= 0,
		.dst_addr	= dst_addr,
		.dst_stride	= dst_stride,
		.width		= width,
		.height		= height,
		.color		= color,
	};

	if (!g_blitter)
		return -ENODEV;

	return loongson_soc_blitter_exec(g_blitter, &op);
}
EXPORT_SYMBOL_GPL(loongson_soc_blitter_fill);

int loongson_soc_blitter_copy(u32 src_addr, u32 src_stride,
			      u32 dst_addr, u32 dst_stride,
			      u32 width, u32 height)
{
	struct blitter_op op = {
		.op		= 1,
		.src_addr	= src_addr,
		.src_stride	= src_stride,
		.dst_addr	= dst_addr,
		.dst_stride	= dst_stride,
		.width		= width,
		.height		= height,
	};

	if (!g_blitter)
		return -ENODEV;

	return loongson_soc_blitter_exec(g_blitter, &op);
}
EXPORT_SYMBOL_GPL(loongson_soc_blitter_copy);

static int loongson_soc_blitter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct loongson_soc_blitter *blt;
	struct resource fb_res;
	int ret;

	blt = devm_kzalloc(dev, sizeof(*blt), GFP_KERNEL);
	if (!blt)
		return -ENOMEM;

	blt->dev = dev;
	spin_lock_init(&blt->lock);
	init_completion(&blt->done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	blt->cfg = ioremap_cache(res->start, BLT_CFG_SIZE);
	if (!blt->cfg)
		return -ENOMEM;

	blt->status = devm_ioremap(dev, res->start + BLT_STATUS, BLT_STATUS_SIZE);
	if (!blt->status) {
		ret = -ENOMEM;
		goto err_iounmap_cfg;
	}

	blt->irq = platform_get_irq(pdev, 0);
	if (blt->irq < 0) {
		ret = blt->irq;
		goto err_iounmap_cfg;
	}

	ret = devm_request_irq(dev, blt->irq, loongson_soc_blitter_irq, 0,
			       BLT_DRV_NAME, blt);
	if (ret)
		goto err_iounmap_cfg;

	ret = of_reserved_mem_region_to_resource(dev->of_node, 0, &fb_res);
	if (ret)
		goto err_iounmap_cfg;

	blt->fb_phys = fb_res.start;
	blt->fb_size = resource_size(&fb_res);
	blt->fb_va = ioremap_cache(blt->fb_phys, blt->fb_size);
	if (!blt->fb_va) {
		ret = -ENOMEM;
		goto err_iounmap_cfg;
	}

	blt->misc.minor = MISC_DYNAMIC_MINOR;
	blt->misc.name = "blitter";
	blt->misc.fops = &loongson_soc_blitter_fops;
	blt->misc.parent = dev;

	ret = misc_register(&blt->misc);
	if (ret)
		goto err_iounmap_fb;

	platform_set_drvdata(pdev, blt);
	g_blitter = blt;

	dev_info(dev, "Loongson SoC blitter at 0x%llx, irq %d\n",
		 (u64)res->start, blt->irq);
	return 0;

err_iounmap_fb:
	iounmap(blt->fb_va);
err_iounmap_cfg:
	iounmap(blt->cfg);
	return ret;
}

static void loongson_soc_blitter_remove(struct platform_device *pdev)
{
	struct loongson_soc_blitter *blt = platform_get_drvdata(pdev);

	misc_deregister(&blt->misc);
	if (g_blitter == blt)
		g_blitter = NULL;
	iounmap(blt->fb_va);
	iounmap(blt->cfg);
}

static const struct of_device_id loongson_soc_blitter_of_match[] = {
	{ .compatible = "loongson-edu,blitter" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_soc_blitter_of_match);

static struct platform_driver loongson_soc_blitter_driver = {
	.probe	= loongson_soc_blitter_probe,
	.remove	= loongson_soc_blitter_remove,
	.driver	= {
		.name		= BLT_DRV_NAME,
		.of_match_table	= loongson_soc_blitter_of_match,
	},
};
module_platform_driver(loongson_soc_blitter_driver);

MODULE_AUTHOR("Loongson Education FPGA Lab");
MODULE_DESCRIPTION("Loongson SoC 2D Blitter driver");
MODULE_LICENSE("GPL v2");
