/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 */
#include <linux/init.h>
#include <linux/threads.h>

#include <asm/addrspace.h>
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/bug.h>
#include <asm/regdef.h>
#include <asm/loongarch.h>
#include <asm/stackframe.h>

#ifdef CONFIG_EFI_STUB

#include "efi-header.S"

	__HEAD

_head:
	.word	IMAGE_DOS_SIGNATURE	/* "MZ", MS-DOS header */
	.org	0x8
	.dword	_kernel_entry		/* Kernel entry point (physical address) */
	.dword	_kernel_asize		/* Kernel image effective size */
	.quad	PHYS_LINK_KADDR		/* Kernel image load offset from start of RAM */
	.org	0x38			/* 0x20 ~ 0x37 reserved */
	.long	LINUX_PE_MAGIC
	.long	pe_header - _head	/* Offset to the PE header */

pe_header:
	__EFI_PE_HEADER

SYM_DATA(kernel_asize, .long _kernel_asize);
SYM_DATA(kernel_fsize, .long _kernel_fsize);

#endif

	__REF

	.align 12

/*
 * Chiplab uses the same uncached 0x9fe00000 UART mapping as U-Boot.  Do not
 * inherit its UART state: program the 33 MHz NS16550 for 115200 8N1 before
 * emitting the early handoff markers.  The divisor is 33 MHz / (16 * 115200),
 * rounded to 18.
 */
	.macro LA32R_EARLY_UART_INIT
	li.w	t0, 0x9fe00000
	li.w	t1, 0x03		/* 8 data bits, 1 stop bit, no parity */
	st.b	t1, t0, 3		/* LCR */
	st.b	zero, t0, 1		/* IER: disable interrupts */
	li.w	t1, 0x07
	st.b	t1, t0, 2		/* FCR: enable and clear both FIFOs */
	li.w	t1, 0x03
	st.b	t1, t0, 4		/* MCR: DTR and RTS */
	li.w	t1, 0x83
	st.b	t1, t0, 3		/* LCR: select divisor latch */
	li.w	t1, 18
	st.b	t1, t0, 0		/* DLL */
	st.b	zero, t0, 1		/* DLM */
	li.w	t1, 0x03
	st.b	t1, t0, 3		/* LCR: 8N1, divisor latch off */
	.endm

	.macro LA32R_EARLY_MARK value
	li.w	t0, 0x9fe00000
1:
	ld.b	t1, t0, 5
	andi	t1, t1, 0x20
	beq	t1, zero, 1b
	li.w	t1, \value
	st.b	t1, t0, 0
	.endm

	.macro LA32R_EARLY_PUTC reg
	li.w	t0, 0x9fe00000
.Learly_putc_wait\@:
	ld.b	t1, t0, 5
	andi	t1, t1, 0x20
	beq	t1, zero, .Learly_putc_wait\@
	st.b	\reg, t0, 0
	.endm

	.macro LA32R_EARLY_HEX reg
	li.w	t2, 28
.Learly_hex_loop\@:
	srl.w	t3, \reg, t2
	andi	t3, t3, 0xf
	sltui	t4, t3, 10
	addi.w	t5, t3, 0x30
	bne	t4, zero, .Learly_hex_emit\@
	addi.w	t5, t3, 0x57
.Learly_hex_emit\@:
	LA32R_EARLY_PUTC t5
	addi.w	t2, t2, -4
	bge	t2, zero, .Learly_hex_loop\@
	.endm

SYM_CODE_START(kernel_entry)			# kernel entry point
	UNWIND_HINT_END_OF_STACK

	/* Replace U-Boot's exception vectors before touching any kernel state. */
	la.pcrel	t0, la32r_early_exception
	csrwr	t0, LOONGARCH_CSR_EENTRY
	csrwr	t0, LOONGARCH_CSR_TLBRENTRY
	csrwr	zero, LOONGARCH_CSR_ECFG

	LA32R_EARLY_UART_INIT
	LA32R_EARLY_MARK	0x41		# 'A': entered kernel
	SETUP_TWINS
	SETUP_MODES	t0
	JUMP_VIRT_ADDR	t0, t1
	SETUP_DMWINS	t0
	LA32R_EARLY_MARK	0x42		# 'B': DMWs are installed

	la.pcrel	t0, __bss_start		# clear .bss
	LONG_S		zero, t0, 0
	la.pcrel	t1, __bss_stop - LONGSIZE
1:
	PTR_ADDI	t0, t0, LONGSIZE
	LONG_S		zero, t0, 0
	bne		t0, t1, 1b

	la.pcrel	t0, fw_arg0
	PTR_S		a0, t0, 0		# firmware arguments
	la.pcrel	t0, fw_arg1
	PTR_S		a1, t0, 0
	la.pcrel	t0, fw_arg2
	PTR_S		a2, t0, 0

#if defined(CONFIG_PAGE_SIZE_4KB) && !defined(CONFIG_32BIT_REDUCED)
	LONG_LI		t0, 0
	LONG_LI		t1, CSR_STFILL
	csrxchg		t0, t1, LOONGARCH_CSR_IMPCTL1
#endif
	/* KSave3 used for percpu base, initialized as 0 */
	csrwr		zero, PERCPU_BASE_KS
	/* GPR21 used for percpu base (runtime), initialized as 0 */
	move		u0, zero

	la.pcrel	tp, init_thread_union
	/* Set the SP after an empty pt_regs.  */
	PTR_LI		sp, (_THREAD_SIZE - PT_SIZE)
	PTR_ADD		sp, sp, tp
	set_saved_sp	sp, t0, t1

#ifdef CONFIG_RELOCATABLE

	bl		relocate_kernel

#ifdef CONFIG_RANDOMIZE_BASE
	/* Repoint the sp into the new kernel */
	PTR_LI		sp, (_THREAD_SIZE - PT_SIZE)
	PTR_ADD		sp, sp, tp
	set_saved_sp	sp, t0, t1

	/* Jump to the new kernel: new_pc = current_pc + random_offset */
	pcaddi		t0, 0
	PTR_ADD		t0, t0, a0
	jirl		zero, t0, 0xc
#endif /* CONFIG_RANDOMIZE_BASE */

#endif /* CONFIG_RELOCATABLE */

#ifdef CONFIG_KASAN
	bl		kasan_early_init
#endif

	bl		start_kernel
	ASM_BUG()

SYM_CODE_END(kernel_entry)

	/*
	 * Bring-up-only trap sink.  It deliberately uses no stack or memory and
	 * reports ESTAT, ERA, TLBRERA and BADV through the already known UART.
	 */
	.align 12
SYM_CODE_START(la32r_early_exception)
	csrrd	s0, LOONGARCH_CSR_ESTAT
	csrrd	s1, LOONGARCH_CSR_ERA
	csrrd	s2, LOONGARCH_CSR_TLBRERA
	csrrd	s3, LOONGARCH_CSR_BADV

	li.w	t5, 0x0d
	LA32R_EARLY_PUTC t5
	li.w	t5, 0x0a
	LA32R_EARLY_PUTC t5
	li.w	t5, 0x45		# 'E'
	LA32R_EARLY_PUTC t5
	LA32R_EARLY_HEX s0
	li.w	t5, 0x3a		# ':'
	LA32R_EARLY_PUTC t5
	LA32R_EARLY_HEX s1
	li.w	t5, 0x3a
	LA32R_EARLY_PUTC t5
	LA32R_EARLY_HEX s2
	li.w	t5, 0x3a
	LA32R_EARLY_PUTC t5
	LA32R_EARLY_HEX s3
	li.w	t5, 0x0d
	LA32R_EARLY_PUTC t5
	li.w	t5, 0x0a
	LA32R_EARLY_PUTC t5
1:
	b	1b
SYM_CODE_END(la32r_early_exception)

#ifdef CONFIG_SMP

/*
 * SMP slave cpus entry point.	Board specific code for bootstrap calls this
 * function after setting up the stack and tp registers.
 */
SYM_CODE_START(smpboot_entry)
	UNWIND_HINT_END_OF_STACK

	SETUP_TWINS
	SETUP_MODES	t0
	JUMP_VIRT_ADDR	t0, t1
	SETUP_DMWINS	t0

#if defined(CONFIG_PAGE_SIZE_4KB) && !defined(CONFIG_32BIT_REDUCED)
	LONG_LI		t0, 0
	LONG_LI		t1, CSR_STFILL
	csrxchg		t0, t1, LOONGARCH_CSR_IMPCTL1
#endif
	la.pcrel	t0, cpuboot_data
	ld.d		sp, t0, CPU_BOOT_STACK
	ld.d		tp, t0, CPU_BOOT_TINFO

	bl		start_secondary
	ASM_BUG()

SYM_CODE_END(smpboot_entry)

#endif /* CONFIG_SMP */
