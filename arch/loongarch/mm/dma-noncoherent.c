// SPDX-License-Identifier: GPL-2.0
/*
 * Non-coherent DMA support for LA32R systems.
 *
 * LA32R uses DMW0 for uncached accesses and DMW1 for cached accesses.  The
 * Chiplab platform DMA masters are not coherent with the CPU data cache.
 */

#include <linux/dma-map-ops.h>
#include <linux/mm.h>

#include <asm/cacheflush.h>
#include <asm/io.h>

#define LA32R_DCACHE_LINE_SIZE	16

static void la32r_dma_cache_wback_inv(unsigned long addr, size_t size)
{
	unsigned long end;

	if (!size)
		return;

	end = ALIGN(addr + size, LA32R_DCACHE_LINE_SIZE);
	addr &= ~(LA32R_DCACHE_LINE_SIZE - 1);
	for (; addr < end; addr += LA32R_DCACHE_LINE_SIZE)
		cache_op(Hit_Writeback_Inv_LEAF1, addr);

	asm volatile("dbar 0" ::: "memory");
}

void arch_dma_prep_coherent(struct page *page, size_t size)
{
	la32r_dma_cache_wback_inv((unsigned long)page_address(page), size);
}

void *arch_dma_set_uncached(void *addr, size_t size)
{
	return (void *)TO_UNCACHE(__pa(addr));
}

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
			      enum dma_data_direction dir)
{
	la32r_dma_cache_wback_inv((unsigned long)__va(paddr), size);
}

void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
			   enum dma_data_direction dir)
{
	if (dir != DMA_TO_DEVICE)
		la32r_dma_cache_wback_inv((unsigned long)__va(paddr), size);
}

void arch_setup_dma_ops(struct device *dev, bool coherent)
{
	dev->dma_coherent = coherent;
}
