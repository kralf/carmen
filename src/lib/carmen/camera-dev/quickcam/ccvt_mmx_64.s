/*  CCVT: ColourConVerT: simple library for converting colourspaces
    Copyright (C) 2002 Nemosoft Unv.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    For questions, remarks, patches, etc. for this program, the author can be
    reached at nemosoft@smcc.demon.nl.
*/


/* The ccvt_* functions always have 4 paramaters:
   width    8(%rbp)
   height  12(%rbp)
   src     16(%rbp)
   dst     20(%rbp)
 */

#define __ASSEMBLY__
#include <linux/linkage.h>

#define Width   8(%rbp)
#define Height 12(%rbp)

/* 2 parameters, 1 in, 1 out */
#define Src    16(%rbp)
#define Dst    20(%rbp)


/* The buffer space is in the MMX registers :-)
   We only need the U and V pointers on the stack
 */

#define Uptr        -4(%rbp)
#define Vptr        -8(%rbp)



	.data
/* Some constants used during processing */
mm_0:	 .byte    0,   0,   0,   0,   0,   0,   0,   0
mm_128:	 .byte  128, 128, 128, 128, 128, 128, 128, 128
mm_mask: .byte  255, 255, 255,   0, 255, 255, 255,   0
mm_low:	 .byte  255, 255, 255, 255,   0,   0,   0,   0
mm_high: .byte    0,   0,   0,   0, 255, 255, 255, 255

/* Multiplication factors for Vb, Vg, Ug, Ur */
mm_mul_bgr:	 .word  454,  -88, -183,  359
mm_mul_rgb:	 .word  359, -183,  -88,  454

	.text
/* This function will load the src and destination pointers, and test the
   width/height parameters.
   - %rsi will be set to Src
   - %rdi will be set to Dst
   the carry flag will be set if any of these tests fail.
   It assumes %rbp has been set.
 */
test_params:
	mov Src, %rsi
	mov Dst, %rdi

	cmp $0, %rsi		# NULL pointers?
	je param_fail
	cmp $0, %rdi
	je param_fail

test_width_height:
	cmpl $0, Width
	jbe param_fail
	testl $1, Width		# Odd no. of columns?
	jnz param_fail		# Aye

	cmp $0, Height
	jbe param_fail
	testl $1, Height	# Odd no. of lines?
	jnz param_fail		# Aye
	/* fall through */

/* exit points */
param_ok:
	clc			# Success: clear carry
	ret

param_fail:
	stc			# Fail: set carry
	ret


# Our output is YUV; UV-pointers are relative to edi
param_yuv_dst:
	mov Width, %rax		# add width * height to Y ptr, set in U
	mull Height
	mov %rdi, Uptr		# U = Y
	mov %rdi, Vptr		# V = Y
	add %rax, Uptr		# U = Y + w*h
	add %rax, Vptr		# V = Y + w*h
	shr $2, %rax
	add %rax, Vptr		# V = Y + w*h + (w*h)/4
	ret

# Our input is YUV; UV-pointers are relative to esi
param_yuv_src:
	mov Width, %rax		# add width * height to Y ptr, set in U
	mull Height
	mov %rsi, Uptr		# U = Y
	mov %rsi, Vptr		# V = Y
	add %rax, Uptr		# U = Y + w*h
	add %rax, Vptr		# V = Y + w*h
	shr $2, %rax
	add %rax, Vptr		# V = Y + w*h + (w*h)/4
	ret


/*************************************/

.macro ENTER_FUNC
	enter $8, $0		# 8 bytes for UV pointers
	push %rbx
	push %rsi
	push %rdi

	call test_params
	jc 9f
.endm


.macro START_LOOP_YUV order
	call param_yuv_src	# our input is YUVp
	mov Width, %rbx		# Use in offset calculation for Y2 / Dst2
	shrl $1, Height		# Only half the lines
	shrl $1, Width		# Only half the columns
	movq mm_128, %mm6	# load constants
	movq mm_mask, %mm5
	movq mm_mul_\order, %mm4

0:	mov Width, %rcx		# number of loops
.endm

.macro LOAD_MUL_UV is_rgb=0
1:	push %rbx
// Section A1: load and prepare UV values
        mov Uptr, %rbx
        movzbl (%rbx), %eax     # load U byte
        inc %rbx
        mov %al, %ah            # duplicate byte
        mov %rbx, Uptr		# Faster than "incl Uptr"

        mov Vptr, %rbx
        movzbl (%rbx), %edx	# load U byte
        inc %rbx
        mov %dl, %dh            # duplicate byte
        mov %rbx, Vptr

.if \is_rgb
        shl $16, %rax
        or %rdx, %rax		# move to lower 16 bits of eax
        movd %rax, %mm2         # 00 00 00 00 UU UU VV VV
.else
        shl $16, %rdx
        or %rax, %rdx		# move to lower 16 bits of edx
        movd %rdx, %mm2         # 00 00 00 00 VV VV UU UU
.endif

// Section A2: multiply UV values and shuffle
// Note: byte orders shown in the MMX registers are for BGR order
        psubb %mm6, %mm2	# -128
        punpcklbw mm_0, %mm2    # 00 VV 00 VV 00 UU 00 UU
        psllw $8, %mm2          # VV 00 VV 00 UU 00 UU 00
        pmulhw %mm4, %mm2       # multiply with factors, signed

        movq %mm2, %mm7		# vr vr vg vg ug ug ub ub
	pand mm_low, %mm7	# 00 00 00 00 ug ug ub ub
	pand mm_high, %mm2	# vr vr vg vg 00 00 00 00
	psrlq $16, %mm2		# 00 00 vr vr vg vg 00 00
	paddw %mm2, %mm7	# 00 00 vr vr *g *g ub ub
	packsswb %mm7, %mm7	# pack signed saturated, and duplicate values (!)
				/* 00 vr *g ub 00 vr *g ub
				   NB! This introduces saturation before the
				   end result is calculated. In strongly
				   saturated areas with high or low luminance
				   this is visible as a darkening resp.
				   brightening.
				   I doubt this is a real problem... The
				   only real solution is to keep these values
				   as 16 bits, and subtract at the end, which
				   unfortunately introduces extra cycles.
				 */
	pop %rbx
.endm


// load 2 Y values and add UV values
// Unfortunately, there is no  'duplicate byte into 4 mmx bytes' instruction
// In: %rax
.macro _DO_2Y_UV reg
	mov %rax, %rdx		# dup AX register
	shl $8, %rax		#  00 Yx1 Yx0  00
	ror $8, %rdx		# Yx0  00  00 Yx1
	bswap %rdx		# Yx1  00  00 Yx0
	or %rdx, %rax		# Yx1 Yx1 Yx0 Yx0
	movd %rax, %mm3		# Load into MMX register
	movq %mm3, %mm\reg	# Double, and....
	punpcklbw %mm3, %mm\reg	# Poof! Yx1 Yx1 Yx1 Yx1 Yx0 Yx0 Yx0 Yx0
	pand %mm5, %mm\reg	# This isnt strictly necessary, but keeps the alpha byte at 0
	psubb %mm6, %mm\reg	# Turn into signed
	paddsb %mm7, %mm\reg	# add UV part (8 bytes!)
	paddb %mm6, %mm\reg	# Make unsigned again
.endm

.macro LOAD_4Y_ADD_UV
	movzwl (%rsi, %rbx), %eax	# load Y10 & Y11, bits 0..15
	_DO_2Y_UV 1			# stuff in MM1
	xor %rax, %rax			# clear
	lodsw				# load Y00 & Y01
	_DO_2Y_UV 0			# stuff in MM0
.endm


.macro STORE_MMX32
// Section B2
	movq %mm0, (%rdi)		# store 2 pixels at once at [dst]
	movq %mm1, (%rdi, %rbx, 4)	# [dst + 4 * width] At moments like this,
					#   you must admire the Intel engineers :)
	add $8, %rdi
.endm


.macro _PUSH_EAX24 reg
	stosw
	shr $16, %rax
	stosb
.endm

.macro _PUSH_MMX24 reg
	movd %mm\reg, %rax			# eax = x0
	_PUSH_EAX24
	psrlq $32, %mm\reg			# pixel x1
	movd %mm\reg, %rax
	_PUSH_EAX24
.endm

.macro STORE_MMX24
	# Blegh; this is more work.
	push %rdi
	mov %rbx, %rdx
	shl $1, %rdx
	add %rbx, %rdx			# edx = 3 * ebx
	add %rdx, %rdi			# edi = edi + 3 * width
	_PUSH_MMX24 1
	pop %rdi			# restore edi
	_PUSH_MMX24 0
.endm

.macro END_LOOP_32
	# end of calculations
	dec %rcx
	jnz 1b			# perform column loop

	add %rbx, %rsi		# Done; go to next line
	add %rbx, %rdi
	add %rbx, %rdi
	add %rbx, %rdi
	add %rbx, %rdi
	decl Height		# decrement line counter
	jnz 0b
.endm

.macro END_LOOP_24
	# end of calculations
	dec %rcx
	jnz 1b			# perform column loop

	add %rbx, %rsi		# Done; go to next line
	add %rbx, %rdi
	add %rbx, %rdi
	add %rbx, %rdi
	decl Height		# decrement line counter
	jnz 0b
.endm



.macro LEAVE_FUNC
	emms			# Clear MMX state

9:	pop %rdi
	pop %rsi
	pop %rbx
	leave
	ret
.endm

/* Functions to go from YUV interlaced formats to RGB. Note that these
   functions are build entirely from macros
 */

ccvt_420p_bgr32:
	ENTER_FUNC
	START_LOOP_YUV bgr
	LOAD_MUL_UV 0
	LOAD_4Y_ADD_UV
	STORE_MMX32
	END_LOOP_32
	LEAVE_FUNC

ccvt_420p_bgr24:
	ENTER_FUNC
	START_LOOP_YUV bgr
	LOAD_MUL_UV 0
	LOAD_4Y_ADD_UV
	STORE_MMX24
	END_LOOP_24
	LEAVE_FUNC

ccvt_420p_rgb32:
	ENTER_FUNC
	START_LOOP_YUV rgb
	LOAD_MUL_UV 1
	LOAD_4Y_ADD_UV
	STORE_MMX32
	END_LOOP_32
	LEAVE_FUNC

ccvt_420p_rgb24:
	ENTER_FUNC
	START_LOOP_YUV rgb
	LOAD_MUL_UV 1
	LOAD_4Y_ADD_UV
	STORE_MMX24
	END_LOOP_24
	LEAVE_FUNC

