	.file	"diag_1k.c"
	.text
	.text
	.align 1
	.align 2
	.type	draw_waves, @function
draw_waves:
	mov.l	r8,@-r15
	shlr	r5
	mov.l	r9,@-r15
	mov	r5,r8
	mov.l	r10,@-r15
	mov	#32,r6
	mov.w	.L14,r2
	mov	#30,r9
	mov.l	r11,@-r15
	mov	r4,r11
	mov.l	r12,@-r15
	add	r4,r2
	mov.l	.L9,r12
	mov.w	.L10,r10
	mov.w	.L11,r4
	mov.w	.L12,r5
	.align 2
.L3:
	mov.w	.L13,r1
	mov	r8,r0
	and	#31,r0
	add	r2,r1
	mov.b	@(r0,r12),r2
	mov	#11,r3
	extu.b	r2,r2
	mov	r2,r7
	add	#80,r7
	or	r10,r7
	add	#16,r2
	.align 2
.L2:
	mov	#-1,r0
	mov.w	r4,@r1
	mov.w	r0,@(2,r1)
	mov	#32,r0
	mov.w	r0,@(4,r1)
	mov	r5,r0
	mov.w	r0,@(6,r1)
	mov	r2,r0
	mov.w	r0,@(8,r1)
	mov	r6,r0
	mov.w	r0,@(10,r1)
	mov	#31,r0
	mov.w	r0,@(12,r1)
	mov	#7,r0
	mov.w	r0,@(14,r1)
	mov	#24,r0
	mov.w	r0,@(16,r1)
	dt	r3
	mov	r7,r0
	mov.w	r0,@(18,r1)
	add	#32,r2
	bf.s	.L2
	add	#20,r1
	mov.w	.L14,r2
	dt	r9
	add	#1,r8
	add	r1,r2
	bf.s	.L3
	add	#8,r6
	mov.w	.L15,r0
	mov.l	@r15+,r12
	add	r11,r0
	mov.l	@r15+,r11
	mov.l	@r15+,r10
	mov.l	@r15+,r9
	rts	
	mov.l	@r15+,r8
	.align 1
.L14:
	.short	220
.L10:
	.short	10240
.L11:
	.short	4096
.L12:
	.short	3968
.L13:
	.short	-220
.L15:
	.short	6600
.L16:
	.align 2
.L9:
	.long	wave
	.size	draw_waves, .-draw_waves
	.align 1
	.align 2
	.type	draw_circles, @function
draw_circles:
	mov.l	r8,@-r15
	mov	r4,r2
	mov.l	r9,@-r15
	mov.l	r10,@-r15
	mov.l	r11,@-r15
	mov.l	r12,@-r15
	mov.l	r13,@-r15
	mov.l	r14,@-r15
	mov.l	.L26,r1
	add	#-8,r15
	mov.l	.L27,r14
	mov.l	.L28,r8
	mov.l	.L29,r4
	mov.l	.L30,r13
	mov.l	.L31,r10
	mov.l	.L32,r9
	mov.w	.L33,r5
	mov.l	r2,@(4,r15)
	mov.l	r1,@r15
	mov	r2,r1
	mov.l	@r15,r7
	.align 2
.L41:
	mov.w	.L34,r11
	mov.w	@r14+,r0
	mov.w	@r7+,r6
	mov.w	@r8+,r3
	mov.w	@r4+,r2
	mov.l	r7,@r15
	mov	r8,r12
	mov.w	@r13+,r7
	add	#-32,r12
	mov.w	r0,@(2,r1)
	mov	#0,r0
	mov.w	r0,@(4,r1)
	mov.w	.L35,r0
	mov.w	r11,@r1
	mov	r4,r11
	mov.w	r0,@(6,r1)
	add	#-32,r11
	mov	r3,r0
	add	#32,r0
	mov.w	r0,@(8,r1)
	mov	r2,r0
	add	#32,r0
	mov.w	r0,@(10,r1)
	mov	#7,r0
	mov.w	r0,@(12,r1)
	mov.w	r0,@(14,r1)
	mov	r6,r0
	mov.w	r0,@(16,r1)
	mov	r7,r0
	mov.w	r0,@(18,r1)
	add	#20,r1
	mov.w	@r10+,r6
	mov.w	@r9+,r7
	extu.w	r6,r6
	add	r6,r3
	extu.w	r7,r7
	exts.w	r3,r3
	add	r7,r2
	mov	r3,r0
	exts.w	r2,r2
	mov.w	r0,@(30,r12)
	cmp/pz	r3
	mov	r2,r0
	mov.w	r0,@(30,r11)
	bf.s	.L24
	mov	#0,r0
	mov.w	.L36,r0
	cmp/gt	r0,r3
	bf.s	.L40
	cmp/pz	r2
.L24:
	mov.w	r0,@(30,r12)
	mov	r10,r3
	neg	r6,r0
	add	#-32,r3
	mov.w	r0,@(30,r3)
	cmp/pz	r2
.L40:
	bf.s	.L25
	mov	#0,r0
	mov.w	.L37,r3
	cmp/gt	r3,r2
	bf.s	.L21
	mov	r3,r0
.L25:
	mov.w	r0,@(30,r11)
	mov	r9,r2
	neg	r7,r0
	add	#-32,r2
	mov.w	r0,@(30,r2)
.L21:
	dt	r5
	bf.s	.L41
	mov.l	@r15,r7
	mov.l	@(4,r15),r2
	mov.w	.L38,r0
	add	r2,r0
	add	#8,r15
	mov.l	@r15+,r14
	mov.l	@r15+,r13
	mov.l	@r15+,r12
	mov.l	@r15+,r11
	mov.l	@r15+,r10
	mov.l	@r15+,r9
	rts	
	mov.l	@r15+,r8
	.align 1
.L33:
	.short	1024
.L34:
	.short	4864
.L35:
	.short	3968
.L36:
	.short	312
.L37:
	.short	232
.L38:
	.short	20480
.L39:
	.align 2
.L26:
	.long	tint_rw
.L27:
	.long	alpha_w
.L28:
	.long	pos_x
.L29:
	.long	pos_y
.L30:
	.long	tint_gb
.L31:
	.long	vel_x
.L32:
	.long	vel_y
	.size	draw_circles, .-draw_circles
	.align 1
	.align 2
	.global	memset
	.type	memset, @function
memset:
	tst	r6,r6
	mov	r5,r0
	bt.s	.L47
	mov	r4,r1
	.align 2
.L44:
	add	#1,r1
	mov	r1,r2
	add	#-16,r2
	dt	r6
	bf.s	.L44
	mov.b	r0,@(15,r2)
.L47:
	rts	
	mov	r4,r0
	.size	memset, .-memset
	.align 1
	.align 2
	.global	memcpy
	.type	memcpy, @function
memcpy:
	tst	r6,r6
	bt	.L49
	mov	r4,r1
	.align 2
.L50:
	add	#1,r5
	mov	r5,r0
	add	#-16,r0
	mov.b	@(15,r0),r0
	dt	r6
	mov.b	r0,@r1
	bf.s	.L50
	add	#1,r1
.L49:
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
	mov	#0,r0
	mov.l	r9,@-r15
	mov	#15,r3
	mov.l	r10,@-r15
	mov.l	r11,@-r15
	mov.l	r12,@-r15
	mov.l	r13,@-r15
	mov.l	.L102,r1
	mov.l	.L103,r2
	mov.l	r14,@-r15
	sts.l	pr,@-r15
	mov.l	r2,@r1
	mov	#62,r2
	mov.l	.L104,r1
	.align 2
.L59:
	cmp/hi	r3,r0
	bt	.L56
.L101:
	mov	r0,r7
	add	#1,r0
	add	r7,r7
	cmp/hi	r3,r0
	mov.b	r7,@r1
	add	#-2,r2
	bf.s	.L101
	add	#1,r1
.L56:
	add	#1,r0
	cmp/eq	#32,r0
	bt.s	.L58
	mov.b	r2,@r1
	add	#1,r1
	bra	.L59
	add	#-2,r2
.L105:
	.align 2
.L102:
	.long	-1400963072
.L103:
	.long	-777646079
.L104:
	.long	wave
	.align 1
.L58:
	mov.l	.L107,r10
	mov.l	.L108,r9
	mov.l	.L109,r3
	mov.l	.L110,r2
	mov.l	.L111,r4
	mov.l	.L112,r5
	mov.l	.L113,r6
	mov.l	.L114,r8
	mov.w	.L115,r7
	.align 2
.L62:
	mov	#13,r0
	mov	r8,r1
	shld	r0,r1
	xor	r8,r1
	mov	r1,r0
	shlr16	r0
	shlr	r0
	xor	r1,r0
	mov	#5,r8
	mov	r0,r1
	shld	r8,r1
	xor	r0,r1
	extu.b	r1,r0
	add	#28,r0
	mov.w	r0,@r10
	mov	r1,r0
	shlr8	r0
	and	#231,r0
	mov.w	r0,@r9
	mov	#3,r11
	mov	r1,r0
	shlr16	r0
	and	r0,r11
	mov	r11,r0
	add	#1,r0
	mov.w	r0,@r3
	mov	r1,r0
	shlr16	r0
	shlr2	r0
	mov	#3,r8
	and	r0,r8
	mov	r8,r0
	add	#1,r0
	mov.w	r0,@r2
	add	#2,r10
	mov.l	.L116,r0
	add	#2,r9
	add	#2,r3
	tst	r0,r1
	bt.s	.L60
	add	#2,r2
	not	r11,r0
	mov	r3,r11
	add	#-32,r11
	mov.w	r0,@(30,r11)
.L60:
	mov.l	.L117,r0
	tst	r0,r1
	bt.s	.L61
	not	r8,r0
	mov	r2,r8
	add	#-32,r8
	mov.w	r0,@(30,r8)
.L61:
	mov	#13,r0
	mov	r1,r8
	shld	r0,r8
	xor	r1,r8
	mov	r8,r1
	shlr16	r1
	shlr	r1
	xor	r1,r8
	mov	#5,r0
	mov	r8,r1
	shld	r0,r1
	xor	r1,r8
	mov	r8,r0
	and	#127,r0
	add	#48,r0
	mov.w	r0,@r4
	mov	r8,r0
	shlr8	r0
	and	#127,r0
	mov	r0,r1
	mov	r8,r0
	shlr16	r0
	add	#48,r1
	and	#127,r0
	shll8	r1
	add	#48,r0
	or	r1,r0
	mov.w	r0,@r5
	mov	r8,r0
	shlr16	r0
	shlr8	r0
	and	#127,r0
	mov	#-11,r11
	mov	r0,r1
	mov	r8,r0
	shld	r11,r0
	mov.w	.L118,r11
	add	#96,r1
	and	#127,r0
	shll8	r1
	add	r0,r11
	or	r1,r11
	mov.w	r11,@r6
	dt	r7
	add	#2,r4
	add	#2,r5
	bf.s	.L62
	add	#2,r6
	mov.l	.L119,r1
	mov	#-7,r10
	mov.l	.L120,r2
	mov	#49,r5
	mov.l	r8,@r1
	mov	#32,r1
	mov.l	r1,@r2
	mov	#31,r9
	mov.l	r1,@(4,r2)
	mov	#8,r11
	mov.l	r1,@(44,r2)
	mov.l	r1,@(48,r2)
	mov.l	.L121,r1
	mov.l	.L129,r2
	mov.l	.L123,r13
	mov.l	r2,@r1
	mov	#0,r2
	mov.l	r2,@(4,r1)
	mov.w	.L124,r2
	mov.l	.L125,r12
	mov.l	r2,@(8,r1)
	mov.l	.L126,r4
	mov.l	.L127,r2
	mov.w	.L132,r0
	mov.l	r2,@(12,r1)
.L66:
	mul.l	r10,r10
	mov	r12,r3
	sub	r4,r3
	add	#-2,r3
	mov	r13,r7
	sts	macl,r6
	shlr	r3
	mov	r4,r2
	add	#-7,r7
	add	#1,r3
	.align 2
.L65:
	mov	r2,r1
	add	r7,r1
	mul.l	r1,r1
	sts	macl,r1
	add	r6,r1
	cmp/gt	r5,r1
	bt.s	.L63
	mov	#0,r14
	neg	r1,r1
	add	#49,r1
	shlr	r1
	add	#12,r1
	cmp/hi	r9,r1
	bf	.L64
	mov	#31,r1
.L64:
	mov	r1,r14
	shll2	r14
	add	r14,r14
	shll2	r14
	add	r1,r14
	shll2	r14
	add	r14,r14
	shll2	r14
	add	r1,r14
	or	r0,r14
.L63:
	mov.w	r14,@r2
	dt	r3
	bf.s	.L65
	add	#2,r2
	dt	r11
	add	#16,r4
	add	#16,r12
	add	#2,r10
	bf.s	.L66
	add	#-16,r13
	mov.l	.L129,r1
	mov.l	.L130,r7
	mov.l	r1,@r4
	mov	#0,r1
	mov.l	r1,@(4,r4)
	mov.l	.L131,r1
	mov.w	.L132,r5
	mov.l	r1,@(8,r4)
	mov.l	.L133,r1
	mov.l	.L134,r6
	mov.l	r1,@(12,r4)
	mov.l	.L135,r1
.L68:
	mov.b	@r7+,r2
	add	#-64,r1
	extu.b	r2,r0
	mov	r0,r3
	shll2	r3
	shll8	r2
	add	r3,r3
	shll2	r2
	shll2	r3
	or	r3,r2
	or	r2,r0
	or	r5,r0
	mov	#32,r2
	.align 2
.L67:
	add	#2,r1
	mov	r1,r3
	add	#-32,r3
	dt	r2
	bf.s	.L67
	mov.w	r0,@(30,r3)
	add	#64,r1
	cmp/eq	r6,r1
	bf	.L68
	mov.l	.L136,r9
	mov.l	.L137,r4
	mov.l	.L138,r10
	jsr	@r9
	mov	#0,r5
	jsr	@r10
	mov	r0,r4
	mov.w	.L139,r1
	mov.w	r1,@r0
	mov.l	.L140,r1
.L69:
	mov.l	@r1,r0
	tst	#16,r0
	bt	.L69
	mov.l	.L141,r1
	mov.l	.L142,r2
	add	#-4,r1
	mov.l	r2,@(4,r1)
	bra	.L106
	mov	#1,r2
	.align 1
.L115:
	.short	1024
.L118:
	.short	128
.L124:
	.short	3968
.L132:
	.short	-32768
.L139:
	.short	-4096
.L143:
	.align 2
.L107:
	.long	pos_x
.L108:
	.long	pos_y
.L109:
	.long	vel_x
.L110:
	.long	vel_y
.L111:
	.long	tint_rw
.L112:
	.long	tint_gb
.L113:
	.long	alpha_w
.L114:
	.long	-1056969215
.L116:
	.long	1048576
.L117:
	.long	2097152
.L119:
	.long	-1400963064
.L120:
	.long	-1207959532
.L121:
	.long	-1408237568
.L129:
	.long	536870912
.L123:
	.long	1408237552
.L125:
	.long	-1408237536
.L126:
	.long	-1408237552
.L127:
	.long	458759
.L130:
	.long	band.0
.L131:
	.long	2101120
.L133:
	.long	2031623
.L134:
	.long	-1408236832
.L135:
	.long	-1408237344
.L136:
	.long	draw_waves
.L137:
	.long	-1408236896
.L138:
	.long	draw_circles
.L140:
	.long	-1207959536
.L141:
	.long	-1207959544
.L142:
	.long	202375168
	.align 1
.L106:
	mov.l	r2,@r1
	add	#12,r1
.L70:
	mov.l	@r1,r0
	tst	#16,r0
	bt	.L70
	mov.l	.L154,r1
	mov	#0,r12
	mov.l	.L145,r2
	mov.l	.L146,r14
	mov.l	.L147,r13
	mov.l	.L148,r11
	mov.l	r2,@r1
	.align 2
.L71:
	mov.b	@r14,r0
	tst	#4,r0
	bt	.L71
	mov	#-5,r2
	mov.b	r2,@r14
	mov	#1,r2
	mov.l	r2,@r13
	add	#1,r12
	mov	r12,r0
	tst	#1,r0
	bt.s	.L72
	mov	r12,r5
	mov.l	.L149,r4
	jsr	@r9
	nop
	jsr	@r10
	mov	r0,r4
	mov.w	.L157,r1
	mov.w	r1,@r0
	.align 2
.L73:
	mov.l	@r11,r0
	tst	#16,r0
	bt	.L73
	mov.l	.L151,r2
.L100:
	mov.l	.L152,r1
	mov	#1,r3
	mov.l	r2,@r1
	mov.l	.L153,r2
	mov.l	r3,@r2
	mov.l	.L154,r2
	mov.l	.L155,r3
	mov.l	r3,@r2
	mov.l	r12,@(4,r2)
	mov.l	r8,@(8,r2)
	bra	.L71
	nop
	.align 1
.L72:
	mov.l	.L156,r4
	jsr	@r9
	nop
	jsr	@r10
	mov	r0,r4
	mov.w	.L157,r1
	mov.w	r1,@r0
	.align 2
.L75:
	mov.l	@r11,r0
	tst	#16,r0
	bt	.L75
	mov.l	.L158,r2
	bra	.L100
	nop
	.align 1
.L157:
	.short	-4096
.L159:
	.align 2
.L154:
	.long	-1400963072
.L145:
	.long	-777646078
.L146:
	.long	-1543503868
.L147:
	.long	-1207959516
.L148:
	.long	-1207959536
.L149:
	.long	-1407713280
.L151:
	.long	202899456
.L152:
	.long	-1207959544
.L153:
	.long	-1207959548
.L155:
	.long	-777646077
.L156:
	.long	-1408237568
.L158:
	.long	202375168
	.size	main, .-main
	.section	.rodata
	.align 2
	.type	band.0, @object
	.size	band.0, 8
band.0:
	.base64	"BgoPFRoVDwo="
	.local	wave
	.comm	wave,32,4
	.local	alpha_w
	.comm	alpha_w,2048,2
	.local	tint_gb
	.comm	tint_gb,2048,2
	.local	tint_rw
	.comm	tint_rw,2048,2
	.local	vel_y
	.comm	vel_y,2048,2
	.local	vel_x
	.comm	vel_x,2048,2
	.local	pos_y
	.comm	pos_y,2048,2
	.local	pos_x
	.comm	pos_x,2048,2
	.ident	"GCC: (Ubuntu 15.2.0-16ubuntu1) 15.2.0"
	.section	.note.GNU-stack,"",@progbits
