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
;;; gf_vect_mad_avx512_aio(len, new_src, vec_i, mul_array, src, dest);
;;;

%include "reg_sizes.asm"

%ifdef HAVE_AS_KNOWS_AVX512

%ifidn __OUTPUT_FORMAT__, elf64
 %define arg0  rdi
 %define arg1  rsi
 %define arg2  rdx
 %define arg3  rcx
 %define arg4  r8
 %define arg5  r9
 %define tmp   r11
 %define return rax
 %define func(x) x: endbranch
 %define FUNC_SAVE
 %define FUNC_RESTORE
%endif

;;; gf_vect_mad_avx512_aio(len, new_src, vec_i, mul_array, src, dest);
%define len   arg0        ; len
;;; %define vec   arg1    ; k is useless when p==1
%define new_src arg1      ; new data
%define vec_i    arg2     ; index of vec 
%define mul_array arg3    ; g_tbls
%define	src   arg4        ; data 
%define dest  arg5        ; parity
%define pos   return        

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

%define x0       zmm0
%define xtmpa    zmm1
%define xtmph    zmm2
%define xtmpl    zmm3
%define xd       zmm4
%define xtmpd    zmm5
%define xgft_hi  zmm6
%define xgft_lo  zmm7
%define xgft_loy ymm7
%define xmask0f  zmm8

align 16
mk_global gf_vect_mad_avx512_aio, function
func(gf_vect_mad_avx512_aio)
	FUNC_SAVE
	sub	len, 64
	jl	.return_fail
	xor	pos, pos
	mov	tmp, 0x0f
	vpbroadcastb xmask0f, tmp	;Construct mask 0x0f0f0f...
	sal	vec_i, 5		;Multiply by 32
	vmovdqu8 xgft_loy, [vec_i+mul_array]	;Load array Cx{00}..{0f}, Cx{00}..{f0}
	vshufi64x2 xgft_hi, xgft_lo, xgft_lo, 0x55
	vshufi64x2 xgft_lo, xgft_lo, xgft_lo, 0x00
	mov	tmp, -1
	kmovq	k1, tmp

.loop64:

	;;; load data -> xor data&new_data, and use delta data instead
	XLDR	x0, [src+pos]		;Get next source vector (load old data)
	XLDR	xd, [new_src+pos]	;Get next source vector (load new data)
	vpxorq	x0, x0, xd		; x0 = x0 ^ xd, get delta data

	XLDR	xd, [dest+pos]		;Get next dest vector (load parity)

	vpandq	xtmpa, x0, xmask0f	;Mask low src nibble in bits 4-0
	vpsraw	x0, x0, 4		;Shift to put high nibble into bits 4-0
	vpandq	x0, x0, xmask0f		;Mask high src nibble in bits 4-0

	;;; xtmph is delta parity
	vpshufb	xtmph {k1}{z}, xgft_hi, x0	;Lookup mul table of high nibble
	vpshufb	xtmpl {k1}{z}, xgft_lo, xtmpa	;Lookup mul table of low nibble
	vpxorq	xtmph, xtmph, xtmpl	;GF add high and low partials
	vpxorq	xd, xd, xtmph		;xd += partial

	;;; store parity -> xor delta_parity&parity, and store new_parity instead
	vmovdqa32	[dest+pos], xd ; write new parity to PM

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
global no_gf_vect_mad_avx512_aio
no_gf_vect_mad_avx512_aio:
%endif
%endif  ; ifdef HAVE_AS_KNOWS_AVX512
