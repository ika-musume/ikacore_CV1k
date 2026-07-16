	.file	"diag_mini.c"
	.text
	.text
	.align 1
	.align 2
	.global	memset
	.type	memset, @function
memset:
	tst	r6,r6
	mov	r5,r0
	bt.s	.L6
	mov	r4,r1
	.align 2
.L3:
	add	#1,r1
	mov	r1,r2
	add	#-16,r2
	dt	r6
	bf.s	.L3
	mov.b	r0,@(15,r2)
.L6:
	rts	
	mov	r4,r0
	.size	memset, .-memset
	.align 1
	.align 2
	.global	memcpy
	.type	memcpy, @function
memcpy:
	tst	r6,r6
	bt	.L9
	mov	r4,r1
	.align 2
.L10:
	add	#1,r5
	mov	r5,r0
	add	#-16,r0
	mov.b	@(15,r0),r0
	dt	r6
	mov.b	r0,@r1
	bf.s	.L10
	add	#1,r1
.L9:
	rts	
	mov	r4,r0
	.size	memcpy, .-memcpy
	.section	.text.startup,"ax",@progbits
	.align 1
	.align 2
	.global	main
	.type	main, @function
main:
	mov.l	r8,@-r15
	mov	#-1,r7
	mov.l	r9,@-r15
	mov.l	r10,@-r15
	mov.l	r11,@-r15
	mov.l	r12,@-r15
	mov.l	r13,@-r15
	mov.l	.L80,r1
	mov.l	.L33,r2
	mov.l	r14,@-r15
	mov.l	r2,@r1
	mov	#32,r1
	mov.l	.L34,r2
	mov.w	.L35,r3
	mov.l	r1,@r2
	mov.l	r1,@(4,r2)
	mov.l	r1,@(44,r2)
	mov.l	r1,@(48,r2)
	mov.l	.L61,r1
	mov.l	.L37,r2
	mov.l	r2,@r1
	mov	#0,r2
	mov.l	r2,@(4,r1)
	mov.w	.L66,r2
	mov.l	r2,@(8,r1)
	mov.l	.L68,r2
	mov.l	r2,@(12,r1)
	mov.l	.L40,r2
	.align 2
.L16:
	mov.w	r7,@r2
	dt	r3
	bf.s	.L16
	add	#2,r2
	mov.w	.L63,r1
	mov.l	.L62,r3
	mov.l	r1,@(4,r2)
	mov.l	.L64,r1
	mov.w	.L66,r5
	mov.l	r1,@(8,r2)
	mov.l	.L65,r1
	mov.l	.L68,r6
	mov.l	r1,@(12,r2)
	mov	#0,r1
	mov.l	r1,@(16,r2)
	mov.l	.L67,r1
	mov.l	.L69,r7
	mov.l	r1,@(28,r2)
	mov.l	.L70,r1
	mov.l	.L71,r4
	mov.l	r3,@r2
	mov.l	r3,@(20,r2)
	mov.l	r5,@(24,r2)
	mov.l	r3,@(40,r2)
	mov.l	r1,@(48,r2)
	mov	r2,r1
	mov.l	r3,@(60,r2)
	add	#64,r1
	mov.l	r6,@(32,r2)
	mov.l	r7,@(36,r2)
	mov.l	r5,@(44,r2)
	mov.l	r6,@(52,r2)
	mov.l	r7,@(56,r2)
	add	#96,r2
	mov.l	r4,@(4,r1)
	mov.l	r3,@(16,r1)
	mov.l	r3,@(36,r1)
	mov.l	.L73,r4
	mov.l	.L52,r3
	mov.l	r5,@(0,r1)
	mov.l	r6,@(8,r1)
	mov.l	r7,@(12,r1)
	mov.l	r5,@(20,r1)
	mov.l	r4,@(24,r1)
	mov.l	r6,@(28,r1)
	mov.l	r7,@(32,r1)
	mov.l	r5,@(40,r1)
	mov.l	r3,@(44,r1)
	mov.l	r6,@(48,r1)
	mov.l	r7,@(52,r1)
	mov.w	.L76,r0
	mov.l	.L77,r1
	mov.w	r0,@(24,r2)
.L17:
	mov.l	@r1,r0
	tst	#16,r0
	bt	.L17
	mov.l	.L78,r1
	mov.l	.L79,r2
	add	#-4,r1
	mov.l	r2,@(4,r1)
	mov	#1,r2
	mov.l	r2,@r1
	add	#12,r1
.L18:
	mov.l	@r1,r0
	tst	#16,r0
	bt	.L18
	mov.l	.L80,r1
	mov	#0,r4
	mov.l	.L58,r2
	mov.l	.L59,r3
	mov.l	r2,@r1
	mov.l	.L60,r12
	mov.l	.L61,r1
	mov.l	.L62,r7
	mov.w	.L63,r11
	mov.l	.L64,r10
	mov.l	.L65,r9
	mov.w	.L66,r5
	mov.l	.L67,r8
	mov.l	.L68,r6
	.align 2
.L19:
	mov.b	@r3,r0
	tst	#4,r0
	bt.s	.L19
	mov	#-5,r2
	mov.b	r2,@r3
	mov	#1,r2
	mov.l	r2,@r12
	mov	#0,r2
	mov.l	.L69,r13
	mov.l	r2,@(16,r1)
	add	#1,r4
	mov.l	.L70,r2
	mov.l	.L71,r0
	mov.l	r2,@(48,r1)
	mov.l	.L72,r2
	mov.l	r7,@r1
	mov.l	r11,@(4,r1)
	mov.l	r10,@(8,r1)
	mov.l	r9,@(12,r1)
	mov.l	r7,@(20,r1)
	mov.l	r5,@(24,r1)
	mov.l	r8,@(28,r1)
	mov.l	r6,@(32,r1)
	mov.l	r13,@(36,r1)
	mov.l	r7,@(40,r1)
	mov.l	r5,@(44,r1)
	mov.l	r6,@(52,r1)
	mov.l	r13,@(56,r1)
	mov.l	r7,@(60,r1)
	mov.l	r0,@(4,r2)
	mov.l	.L73,r0
	mov.l	.L74,r14
	mov.l	r0,@(24,r2)
	mov	r4,r0
	shll2	r0
	extu.b	r0,r0
	mov.l	r5,@(0,r2)
	mov.l	r6,@(8,r2)
	add	#40,r0
	mov.l	r13,@(12,r2)
	mov.l	r7,@(16,r2)
	mov.l	r5,@(20,r2)
	mov.l	r6,@(28,r2)
	mov.l	r13,@(32,r2)
	mov.l	r7,@(36,r2)
	mov.l	r5,@(40,r2)
	mov.w	r0,@(12,r14)
	mov.w	.L75,r0
	mov.w	r0,@(14,r14)
	mov.l	r6,@(48,r2)
	mov.l	r13,@(52,r2)
	mov.w	.L76,r0
	mov.l	.L77,r2
	mov.w	r0,@(24,r14)
	.align 2
.L20:
	mov.l	@r2,r0
	tst	#16,r0
	bt	.L20
	mov.l	.L78,r2
	mov.l	.L79,r0
	add	#-4,r2
	mov.l	r0,@(4,r2)
	mov	#1,r0
	mov.l	r0,@r2
	mov.l	.L80,r2
	mov.l	.L81,r0
	mov.l	r0,@r2
	mov.l	r4,@(4,r2)
	bra	.L19
	nop
	.align 1
.L35:
	.short	1024
.L66:
	.short	3968
.L63:
	.short	3776
.L76:
	.short	-4096
.L75:
	.short	136
.L82:
	.align 2
.L80:
	.long	-1400963072
.L33:
	.long	-777650175
.L34:
	.long	-1207959532
.L61:
	.long	-1408237568
.L37:
	.long	536870912
.L68:
	.long	2031647
.L40:
	.long	-1408237552
.L62:
	.long	268500991
.L64:
	.long	2097184
.L65:
	.long	20906223
.L67:
	.long	2621480
.L69:
	.long	8421504
.L70:
	.long	20447272
.L71:
	.long	2621672
.L73:
	.long	20447464
.L52:
	.long	2621576
.L77:
	.long	-1207959536
.L78:
	.long	-1207959544
.L79:
	.long	202375168
.L58:
	.long	-777650174
.L59:
	.long	-1543503868
.L60:
	.long	-1207959516
.L72:
	.long	-1408237504
.L74:
	.long	-1408237472
.L81:
	.long	-777650173
	.size	main, .-main
	.ident	"GCC: (Ubuntu 15.2.0-16ubuntu1) 15.2.0"
	.section	.note.GNU-stack,"",@progbits
