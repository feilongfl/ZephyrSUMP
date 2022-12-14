/*
 * Copyright (c) 2018 Synopsys.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <offsets_short.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/syscall.h>
#include <swap_macros.h>
#include <v2/irq.h>

.macro clear_scratch_regs
	mov_s r1, 0
	mov_s r2, 0
	mov_s r3, 0
	mov_s r4, 0
	mov_s r5, 0
	mov_s r6, 0
	mov_s r7, 0
	mov_s r8, 0
	mov_s r9, 0
	mov_s r10, 0
	mov_s r11, 0
	mov_s r12, 0
.endm

.macro clear_callee_regs
	mov_s r25, 0
	mov_s r24, 0
	mov_s r23, 0
	mov_s r22, 0
	mov_s r21, 0
	mov_s r20, 0
	mov_s r19, 0
	mov_s r18, 0
	mov_s r17, 0
	mov_s r16, 0

	mov_s r15, 0
	mov_s r14, 0
	mov_s r13, 0
.endm

GTEXT(z_arc_userspace_enter)
GTEXT(_arc_do_syscall)
GTEXT(z_user_thread_entry_wrapper)
GTEXT(arch_user_string_nlen)
GTEXT(z_arc_user_string_nlen_fault_start)
GTEXT(z_arc_user_string_nlen_fault_end)
GTEXT(z_arc_user_string_nlen_fixup)

/**
 * @brief Wrapper for z_thread_entry in the case of user thread
 *
 * The init parameters are in privileged stack
 */
SECTION_FUNC(TEXT, z_user_thread_entry_wrapper)
	seti _ARC_V2_INIT_IRQ_LOCK_KEY
	pop_s r3
	pop_s r2
	pop_s r1
	pop_s r0
/* the start of user sp is in r5 */
	pop r5
/* start of privilege stack in blink */
	mov_s blink, sp

	st.aw r0, [r5, -4]
	st.aw r1, [r5, -4]
	st.aw r2, [r5, -4]
	st.aw r3, [r5, -4]

/*
 * when CONFIG_INIT_STACKS is enable, stack will be initialized
 * in z_new_thread_init.
 */
	j _arc_go_to_user_space

/**
 *
 * User space entry function
 *
 * This function is the entry point to user mode from privileged execution.
 * The conversion is one way, and threads which transition to user mode do
 * not transition back later, unless they are doing system calls.
 *
 */
SECTION_FUNC(TEXT, z_arc_userspace_enter)
	/*
	 * In ARCv2, the U bit can only be set through exception return
	 */
	/* disable stack checking as the stack should be initialized */
	_disable_stack_checking blink

	/* the end of user stack in r5 */
	add r5, r4, r5
	/* get start of privilege stack, r6 points to current thread */
	ld blink, [r6, _thread_offset_to_priv_stack_start]
	add blink, blink, CONFIG_PRIVILEGED_STACK_SIZE

	mov_s sp, r5

	push_s r0
	push_s r1
	push_s r2
	push_s r3

	mov r5, sp /* skip r0, r1, r2, r3 */

/* to avoid the leakage of kernel info, the thread stack needs to be
 * re-initialized
 */
#ifdef CONFIG_INIT_STACKS
	mov_s r0, 0xaaaaaaaa
#else
	mov_s r0, 0x0
#endif
_clear_user_stack:
	st.ab r0, [r4, 4]
	cmp r4, r5
	jlt _clear_user_stack

/* reload the stack checking regs as the original kernel stack
 * becomes user stack
 */
#ifdef CONFIG_ARC_STACK_CHECKING
/* current thread in r6, SMP case is also considered */
	mov r2, r6

	_load_stack_check_regs

	_enable_stack_checking r0
#endif

/* the following codes are used to switch from kernel mode
 * to user mode by fake exception, because U bit can only be set
 * by exception
 */
_arc_go_to_user_space:
	lr r0, [_ARC_V2_STATUS32]
	bset r0, r0, _ARC_V2_STATUS32_U_BIT

	mov_s r1, z_thread_entry_wrapper1

	sr r0, [_ARC_V2_ERSTATUS]
	sr r1, [_ARC_V2_ERET]

	/* fake exception return */
	lr r0, [_ARC_V2_STATUS32]
	bset r0, r0, _ARC_V2_STATUS32_AE_BIT
	kflag r0

/* when exception returns from kernel to user, sp and _ARC_V2_USER_SP
 * /_ARC_V2_SECU_SP will be switched
 */
#if defined(CONFIG_ARC_HAS_SECURE) && defined(CONFIG_ARC_SECURE_FIRMWARE)
	lr r0, [_ARC_V2_SEC_STAT]
	/* the mode returns from exception return is secure mode */
	bset r0, r0, 31
	sr r0, [_ARC_V2_ERSEC_STAT]
	sr r5, [_ARC_V2_SEC_U_SP]
#else
	sr r5, [_ARC_V2_USER_SP]
#endif
	mov_s sp, blink

	mov_s r0, 0

	clear_callee_regs

	clear_scratch_regs

	mov fp, 0
	mov r29, 0
	mov r30, 0
	mov blink, 0

	rtie

/**
 *
 * Userspace system call function
 *
 * This function is used to do system calls from unprivileged code.  This
 * function is responsible for the following:
 * 1) Dispatching the system call
 * 2) Restoring stack and calling back to the caller of the system call
 *
 */
SECTION_FUNC(TEXT, _arc_do_syscall)
	/*
	 * r0-r5: arg1-arg6, r6 is call id which is already checked in
	 * trap_s handler, r7 is the system call stack frame pointer
	 * need to recover r0, r1, r2 because they will be modified in
	 * _create_irq_stack_frame. If a specific syscall frame (different
	 * with irq stack frame) is defined, the cover of r0, r1, r2 can be
	 * optimized.
	 */
	ld_s r0, [sp, ___isf_t_r0_OFFSET]
	ld_s r1, [sp, ___isf_t_r1_OFFSET]
	ld_s r2, [sp, ___isf_t_r2_OFFSET]

	mov r7, sp

	mov_s blink, _k_syscall_table
	ld.as r6, [blink, r6]

	jl [r6]

	/* save return value */
	st_s r0, [sp, ___isf_t_r0_OFFSET]

	mov r29, 0
	mov r30, 0

	/* through fake exception return, go back to the caller */
	lr r0, [_ARC_V2_STATUS32]
	bset r0, r0, _ARC_V2_STATUS32_AE_BIT
	kflag r0


#ifdef CONFIG_ARC_SECURE_FIRMWARE
	ld_s r0, [sp, ___isf_t_sec_stat_OFFSET]
	sr r0,[_ARC_V2_ERSEC_STAT]
#endif
	ld_s r0, [sp, ___isf_t_status32_OFFSET]
	sr r0,[_ARC_V2_ERSTATUS]

	ld_s r0, [sp, ___isf_t_pc_OFFSET] /* eret into pc */
	sr r0,[_ARC_V2_ERET]

	_pop_irq_stack_frame

	rtie

/*
 * size_t arch_user_string_nlen(const char *s, size_t maxsize, int *err_arg)
 */
SECTION_FUNC(TEXT, arch_user_string_nlen)
	/* int err; */
	sub_s sp,sp,0x4

	/* Initial error value (-1 failure), store at [sp,0] */
	mov_s r3, -1
	st_s r3, [sp, 0]

	/* Loop setup.
	 * r12 (position locator) = s - 1
	 * r0 (length counter return value)) = 0
	 * lp_count = maxsize + 1
	 * */
	sub r12, r0, 0x1
	mov_s r0, 0
	add_s r1, r1, 1
	mov lp_count, r1

strlen_loop:
z_arc_user_string_nlen_fault_start:
	/* is the byte at ++r12 a NULL? if so, we're done. Might fault! */
	ldb.aw r1, [r12, 1]

z_arc_user_string_nlen_fault_end:
	brne_s r1, 0, not_null

strlen_done:
	/* Success, set err to 0 */
	mov_s r1, 0
	st_s r1, [sp, 0]

z_arc_user_string_nlen_fixup:
	/* *err_arg = err; Pop stack and return */
	ld_s r1, [sp, 0]
	add_s sp, sp, 4
	j_s.d [blink]
	st_s r1, [r2, 0]

not_null:
	/* check if we've hit the maximum, if so we're done. */
	brne.d.nt lp_count, 0x1, inc_len
	sub lp_count, lp_count, 0x1
	b_s strlen_done

inc_len:
	/* increment length measurement, loop again */
	add_s r0, r0, 1
	b_s strlen_loop
