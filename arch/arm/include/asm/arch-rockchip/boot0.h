
/*
 * Copyright 2017 Theobroma Systems Design und Consulting GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * Execution starts on the instruction following this 4-byte header
 * (containing the magic 'RK30', 'RK31', 'RK32' or 'RK33').  This
 * magic constant will be written into the final image by the rkimage
 * tool, but we need to reserve space for it here.
 *
 * To make life easier for everyone, we build the SPL binary with
 * space for this 4-byte header already included in the binary.
 */
#ifdef CONFIG_SPL_BUILD
	/*
	 * We need to add 4 bytes of space for the 'RK33' at the
	 * beginning of the executable.	 However, as we want to keep
	 * this generic and make it applicable to builds that are like
	 * the RK3368 (TPL needs this, SPL doesn't) or the RK3399 (no
	 * TPL, but extra space needed in the SPL), we simply insert
	 * a branch-to-next-instruction-word with the expectation that
	 * the first one may be overwritten, if this is the first stage
	 * contained in the final image created with mkimage)...
	 */
	b 1f	 /* if overwritten, entry-address is at the next word */
1:
#endif
#if defined(CONFIG_TPL_BUILD) && \
      (defined(CONFIG_ROCKCHIP_RK3066) || defined(CONFIG_ROCKCHIP_RK3188))
	adr     r3, entry_counter
	ldr	r0, [r3]
	cmp	r0, #1
	movne	r0, #1
	strne	r0, [r3]
	beq	out_of_bootrom
	bx	lr
entry_counter:
	.word   0
out_of_bootrom:
	mov	r0, #0
	str	r0, [r3]
#endif
	b reset
#if !defined(CONFIG_ARM64)
	/*
	 * For armv7, the addr '_start' will used as vector start address
	 * and write to VBAR register, which needs to aligned to 0x20.
	 */
	.align(5)
_start:
        ARM_VECTORS
#endif

#if defined(CONFIG_ROCKCHIP_RK3399) && defined(CONFIG_SPL_BUILD)
	.space CONFIG_ROCKCHIP_SPL_RESERVE_IRAM	/* space for the ATF data */
#endif
