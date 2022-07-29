;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Copyright(c) 2011-2015 Intel Corporation All rights reserved.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;    * Redistributions of source code must retain the above copyright
;      notice, this list of conditions and the following disclaimer.
;    * Redistributions in binary form must reproduce the above copyright
;      notice, this list of conditions and the following disclaimer in
;      the documentation and/or other materials provided with the
;      distribution.
;    * Neither the name of Intel Corporation nor the names of its
;      contributors may be used to endorse or promote products derived
;      from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;
;;; gf_2vect_mad_avx512_aio(len, vec, new_src, mul_array, src, dest);
;;;

%include "reg_sizes.asm"

%ifdef HAVE_AS_KNOWS_AVX512

%ifidn __OUTPUT_FORMAT__, elf64  ; for linux
 %define arg0   rdi
 %define arg1   rsi
 %define arg2   rdx
 %define arg3   rcx
 %define arg4   r8
 %define arg5   r9
 %define tmp    r11
 %define tmp2   r10
 %define return rax
 %define func(x) x: endbranch
 %define FUNC_SAVE
 %define FUNC_RESTORE
%endif


%define PS    8
%define len   arg0
%define len.w arg0.w
%define vec   arg1
%define new_src arg2
;;; %define vec_i arg2        ; index of vec
%define mul_array arg3    ; g_tbls
%define	src   arg4        ; data
%define dest1 arg5        ; parity on PM
%define pos   return
%define pos.w return.w
%define dest2 tmp2

%ifndef EC_ALIGNED_ADDR
;;; Use Un-aligned load/store
 %define XLDR vmovdqu8
 %define XSTR vmovdqu8
%else
;;; Use Non-temporal load/stor
 %ifdef NO_NT_LDST
  %define XLDR vmovdqa
  %define XSTR vmovdqa
 %else
  %define XLDR vmovntdqa
  %define XSTR vmovntdq
 %endif
%endif

default rel
[bits 64]
section .text

%define x0        zmm0
%define xtmpa     zmm1
%define xtmph1    zmm2
%define xtmpl1    zmm3
%define xtmph2    zmm4
%define xtmpl2    zmm5
%define xd1       zmm6
%define xd2       zmm7
%define xtmpd1    zmm8
%define xtmpd2    zmm9
%define xgft1_hi  zmm10
%define xgft1_lo  zmm11
%define xgft1_loy ymm11
%define xgft2_hi  zmm12
%define xgft2_lo  zmm13
%define xgft2_loy ymm13
%define xmask0f   zmm14

align 16
mk_global gf_2vect_mad_avx512_aio, function
func(gf_2vect_mad_avx512_aio)
	FUNC_SAVE
	sub	len, 64
	jl	.return_fail
	xor	pos, pos
	mov	tmp, 0x0f
	vpbroadcastb xmask0f, tmp	;Construct mask 0x0f0f0f...
	;;; sal	vec_i, 5	;Multiply by 32 (rm by passing in mul_array_modded)
	sal	vec, 5
	lea	tmp, [mul_array]  ; corresponding gf table ; mul_array is modded
	vmovdqu	xgft1_loy, [tmp]	;Load array Ax{00}..{0f}, Ax{00}..{f0}
	vmovdqu	xgft2_loy, [tmp+vec]	;Load array Bx{00}..{0f}, Bx{00}..{f0}
	vshufi64x2 xgft1_hi, xgft1_lo, xgft1_lo, 0x55
	vshufi64x2 xgft1_lo, xgft1_lo, xgft1_lo, 0x00
	vshufi64x2 xgft2_hi, xgft2_lo, xgft2_lo, 0x55
	vshufi64x2 xgft2_lo, xgft2_lo, xgft2_lo, 0x00

	mov	dest2, [dest1+PS]	; reuse mul_array
	mov	dest1, [dest1]  ; here the "destN" indicate different parities
	
	mov	tmp, -1
	kmovq	k1, tmp

.loop64:
	XLDR	x0, [src+pos]		;Get next source vector (load data)
	XLDR	xd1, [new_src+pos]	;Get next source vector (load new data)
	vpxorq	x0, x0, xd1		; x0 = x0 ^ xd1   ;Get delta data

	XLDR	xd1, [dest1+pos]	;Get next dest vector (load parity1)
	XLDR	xd2, [dest2+pos]	;Get next dest vector (load parity2)

	vpandq	xtmpa, x0, xmask0f	;Mask low src nibble in bits 4-0
	vpsraw	x0, x0, 4		;Shift to put high nibble into bits 4-0
	vpandq	x0, x0, xmask0f		;Mask high src nibble in bits 4-0

	;;; xtmph is delta parity
	vpshufb	xtmph1 {k1}{z}, xgft1_hi, x0	;Lookup mul table of high nibble
	vpshufb	xtmpl1 {k1}{z}, xgft1_lo, xtmpa	;Lookup mul table of low nibble
	vpxorq	xtmph1, xtmph1, xtmpl1		;GF add high and low partials
	vpxorq	xd1, xd1, xtmph1		;xd1 += partial

	vpshufb	xtmph2 {k1}{z}, xgft2_hi, x0	;Lookup mul table of high nibble
	vpshufb	xtmpl2 {k1}{z}, xgft2_lo, xtmpa	;Lookup mul table of low nibble
	vpxorq	xtmph2, xtmph2, xtmpl2		;GF add high and low partials
	vpxorq	xd2, xd2, xtmph2		;xd2 += partial

	;;; get and store new parity
	vmovdqa32	[dest1+pos], xd1
	vmovdqa32	[dest2+pos], xd2

	add	pos, 64			;Loop on 64 bytes at a time
	cmp	pos, len
	jle	.loop64

	lea	tmp, [len + 64]
	cmp	pos, tmp
	je	.return_pass

	;; Tail len
	mov	pos, (1 << 63)
	lea	tmp, [len + 64 - 1]
	and	tmp, 63
	sarx	pos, pos, tmp
	kmovq	k1, pos
	mov	pos, len	;Overlapped offset length-64
	jmp	.loop64		;Do one more overlap pass

.return_pass:
	mov	return, 0
	FUNC_RESTORE
	ret

.return_fail:
	mov	return, 1
	FUNC_RESTORE
	ret

endproc_frame

%else
%ifidn __OUTPUT_FORMAT__, win64
global no_gf_2vect_mad_avx512_aio
no_gf_2vect_mad_avx512_aio:
%endif
%endif  ; ifdef HAVE_AS_KNOWS_AVX512
