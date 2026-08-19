/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LOONGSON_SOC_BLITTER_H_
#define _LOONGSON_SOC_BLITTER_H_

#include <linux/types.h>

bool loongson_soc_blitter_available(void);
int loongson_soc_blitter_fill(u32 dst_addr, u32 dst_stride,
			      u32 width, u32 height, u32 color);
int loongson_soc_blitter_copy(u32 src_addr, u32 src_stride,
			      u32 dst_addr, u32 dst_stride,
			      u32 width, u32 height);

#endif /* _LOONGSON_SOC_BLITTER_H_ */
