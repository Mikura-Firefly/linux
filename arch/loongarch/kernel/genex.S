/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 *
 * Derived from MIPS:
 * Copyright (C) 1994 - 2000, 2001, 2003 Ralf Baechle
 * Copyright (C) 1999, 2000 Silicon Graphics, Inc.
 * Copyright (C) 2002, 2007  Maciej W. Rozycki
 * Copyright (C) 2001, 2012 MIPS Technologies, Inc.  All rights reserved.
 */
#include <asm/asm.h>
#include <asm/asmmacro.h>
#include <asm/loongarch.h>
#include <asm/regdef.h>
#include <asm/fpregdef.h>
#include <asm/stackframe.h>
#include <asm/thread_info.h>

	.section .cpuidle.text, "ax"
	.align	5
SYM_FUNC_START(__arch_cpu_idle)
	/* start of idle interrupt region */
	ori	t0, zero, CSR_CRMD_IE
	/* idle instruction needs irq enabled */
	csrxchg	t0, t0, LOONGARCH_CSR_CRMD
	/*
	 * If an interrupt lands here; between enabling interrupts above and
	 * going idle on the next instruction, we must *NOT* go idle since the
	 * interrupt could have set TIF_NEED_RESCHED or caused an timer to need
	 * reprogramming. Fall through -- see handle_vint() below -- and have
	 * the idle loop take care of things.
	 */
	idle	0
	/* end of idle interrupt region */
idle_exit:
	jr	ra
SYM_FUNC_END(__arch_cpu_idle)
	.previous

#ifdef CONFIG_32BIT_REDUCED
/*
 * LA32R sends every ordinary exception and interrupt to CSR.EENTRY.  It has
 * no ECODE-indexed vectoring, so preserve the temporaries used for dispatch
 * and branch to the existing software handlers explicitly.
 */
	.align	6
SYM_CODE_START(la32r_exception_entry)
	csrwr	t0, EXCEPTION_KS0
	csrwr	t1, EXCEPTION_KS1
	csrwr	t2, EXCEPTION_KS2
	csrrd	t0, LOONGARCH_CSR_ESTAT
	srli.w	t1, t0, 16
	andi	t1, t1, 0x3f
	beq	t1, zero, 1f
	la_abs	t0, exception_handlers
	slli.w	t1, t1, 7		/* VECSIZE is 128 bytes */
	add.w	t0, t0, t1
	jirl	zero, t0, 0
1:
	la_abs	t0, handle_vint
	jirl	zero, t0, 0
SYM_CODE_END(la32r_exception_entry)
#endif

SYM_CODE_START(handle_vint)
	UNWIND_HINT_UNDEFINED
	BACKUP_T0T1
	SAVE_ALL
	la_abs	t1, idle_exit
	LONG_L	t0, sp, PT_ERA
	/* 3 instructions idle interrupt region */
	ori	t0, t0, 0b1100
	bne	t0, t1, 1f
	LONG_S	t0, sp, PT_ERA
1:	move	a0, sp
	move	a1, sp
	la_abs	t0, do_vint
	jirl	ra, t0, 0
	RESTORE_ALL_AND_RET
SYM_CODE_END(handle_vint)

SYM_CODE_START(except_vec_cex)
	UNWIND_HINT_UNDEFINED
	b	cache_parity_error
SYM_CODE_END(except_vec_cex)

	.macro	build_prep_badv
	csrrd	t0, LOONGARCH_CSR_BADV
	PTR_S	t0, sp, PT_BVADDR
	.endm

	.macro	build_prep_fcsr
	movfcsr2gr	a1, fcsr0
	.endm

	.macro	build_prep_none
	.endm

	.macro	BUILD_HANDLER exception handler prep
	.align	5
	SYM_CODE_START(handle_\exception)
	UNWIND_HINT_UNDEFINED
	666:
	BACKUP_T0T1
	SAVE_ALL
	build_prep_\prep
	move	a0, sp
	la_abs	t0, do_\handler
	jirl	ra, t0, 0
	668:
	RESTORE_ALL_AND_RET
	SYM_CODE_END(handle_\exception)
	.pushsection	".data", "aw", %progbits
	SYM_DATA(unwind_hint_\exception, .word 668b - 666b)
	.popsection
	.endm

	BUILD_HANDLER ade ade badv
	BUILD_HANDLER ale ale badv
	BUILD_HANDLER bce bce none
	BUILD_HANDLER bp bp none
	BUILD_HANDLER fpe fpe fcsr
	BUILD_HANDLER fpu fpu none
	BUILD_HANDLER lsx lsx none
	BUILD_HANDLER lasx lasx none
	BUILD_HANDLER lbt lbt none
	BUILD_HANDLER ri ri none
	BUILD_HANDLER watch watch none
	BUILD_HANDLER reserved reserved none	/* others */

SYM_CODE_START(handle_sys)
	UNWIND_HINT_UNDEFINED
	la_abs	t0, handle_syscall
	jr	t0
SYM_CODE_END(handle_sys)
