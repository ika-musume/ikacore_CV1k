! ===========================================================================
! crt0.s - CV1k diagnostic ROM startup  [H7b.D]
!
! Replicates the real ibara U4 reset block VERBATIM (disasm/u4.txt,
! workram:0c000000..0c00007e): WDT stop keys, FRQCR=0x0112, ICR1=0x8000
! (IRQ-pin mode, falling edge - IRR0 latches vblank for the polled pacing),
! the byte-identical BSC block (BCR/WCR/MCR/RTC*), the SDMR3 mode poke
! (address 0xFFFFE880 encodes mode 0x220 = CL2/BL1), and the PFC setup
! (ports C/D/L = inputs, PTH = "other function" = IRQ pins).  Then the
! boot's own NOR->SDRAM copy-loop pattern (0c000082..) moves the whole
! image to work RAM and jumps to C.
!
! Runs position-independent from the NOR at 0xA0000000 (PC-relative
! literals only); everything is LINKED at 0xAC000000 (P2, uncached SDRAM)
! so op-list stores need no cache management.  Stack top = 0xAC800000
! (the boot's 0x0C800000, P2 view).
! ===========================================================================
	.section .text.crt0, "ax"
	.global	_start
_start:
	mov.l	Lsp, r15		! SP = top of work RAM
	mov.l	Lsr, r0
	ldc	r0, sr			! SR = 0x700000F0 (BL=1, IMASK=F: polled, never vectored)
	mov.l	Lvbr, r0
	ldc	r0, vbr			! VBR = 0x8C000000 (as the boot; unused - we poll IRR0)
	mov.l	Lgbr, r0
	ldc	r0, gbr			! GBR = 0xFFFFFF00 (CPG/BSC register window)

	! --- WDT / clock (A5/5A write keys) ---
	mov.l	La500, r0
	mov.w	r0, @(0x86, gbr)	! WTCSR <- 00 (stop)
	mov.l	La507, r0
	mov.w	r0, @(0x86, gbr)	! WTCSR <- 07
	mov.l	L5a00, r0
	mov.w	r0, @(0x84, gbr)	! WTCNT <- 00
	mov.l	Lfrq, r0
	mov.w	r0, @(0x80, gbr)	! FRQCR <- 0x0112 (Iphi/CKIO/Pphi = 102.4/51.2/25.6)

	! --- INTC: IRQ-pin mode, falling edge on all (vblank = IRQ2/PTH2) ---
	mov.l	Lintc, r1		! 0xA4000000
	mov.l	Licr1, r0
	mov.w	r0, @(16, r1)		! ICR1 <- 0x8000

	! --- BSC block (byte-identical across B/D boards) ---
	mov.l	Lbcr1, r0
	mov.w	r0, @(0x60, gbr)	! BCR1  <- 0xC008
	mov.l	Lbcr2, r0
	mov.w	r0, @(0x62, gbr)	! BCR2  <- 0x39F0
	mov.l	Lwcr1, r0
	mov.w	r0, @(0x64, gbr)	! WCR1  <- 0x9551
	mov.l	Lwcr2, r0
	mov.w	r0, @(0x66, gbr)	! WCR2  <- 0xFDD7
	mov.l	Lmcr, r0
	mov.w	r0, @(0x68, gbr)	! MCR   <- 0x543C (RASD=0: auto-precharge)
	mov.l	Lrtcor, r0
	mov.w	r0, @(0x72, gbr)	! RTCOR <- 0x60
	mov.l	La500, r0
	mov.w	r0, @(0x70, gbr)	! RTCNT <- 00
	mov.l	Lrtcsr, r0
	mov.w	r0, @(0x6e, gbr)	! RTCSR <- 0x10
	mov.l	Lsdmr, r1		! SDMR3: the ADDRESS encodes mode 0x220 (CL2/BL1)
	xor	r0, r0
	mov.w	r0, @r1

	! --- PFC: C/D/L inputs (JAMMA), E/F/J per boot, G/H/SCP = other function ---
	mov.l	Lpfc, r1		! 0xA4000100
	mov.l	Laaaa, r0
	mov.w	r0, @(4, r1)		! PCCR  <- 0xAAAA (system inputs)
	mov.w	r0, @(6, r1)		! PDCR  <- 0xAAAA (player 1)
	mov.w	r0, @(20, r1)		! PLCR  <- 0xAAAA (player 2)
	mov.l	Lpjcr, r0
	mov.w	r0, @(16, r1)		! PJCR  <- 0xA544
	mov.l	Lpecr, r0
	mov.w	r0, @(8, r1)		! PECR  <- 0x1944 (PE5 = NAND R/B input)
	mov.l	Lpfcr, r0
	mov.w	r0, @(10, r1)		! PFCR  <- 0x0009
	xor	r0, r0
	mov.w	r0, @(12, r1)		! PGCR  <- 0
	mov.w	r0, @(14, r1)		! PHCR  <- 0 (PTH2 = IRQ2 pin)
	mov.w	r0, @(22, r1)		! SCPCR <- 0
	mov.l	Lpedr, r1
	mov.b	@r1, r0
	or	#0x10, r0
	mov.b	r0, @r1			! PEDR |= 0x10 (as the boot)

	! --- NOR -> SDRAM self-copy (the boot's 0c000092 loop pattern) ---
	mov.l	Lsrc, r1		! 0xA0000000 (this image, via CS0)
	mov.l	Ldst, r2		! 0xAC000000 (work RAM, uncached)
	mov.l	Lwords, r3		! link-time image length in longwords
1:	mov.l	@r1+, r0
	dt	r3
	mov.l	r0, @r2
	bf.s	1b
	add	#4, r2

	! --- clear .bss ---
	mov.l	Lbssw, r3
	tst	r3, r3
	bt	3f
	mov.l	Lbss, r2
	xor	r0, r0
2:	dt	r3
	mov.l	r0, @r2
	bf.s	2b
	add	#4, r2
3:
	! --- enter C, now executing from SDRAM ---
	mov.l	Lmain, r0
	jmp	@r0
	nop

	.align	2
Lsp:	.long	0xAC800000
Lsr:	.long	0x700000F0
Lvbr:	.long	0x8C000000
Lgbr:	.long	0xFFFFFF00
La500:	.long	0x0000A500
La507:	.long	0x0000A507
L5a00:	.long	0x00005A00
Lfrq:	.long	0x00000112
Lintc:	.long	0xA4000000
Licr1:	.long	0x00008000
Lbcr1:	.long	0x0000C008
Lbcr2:	.long	0x000039F0
Lwcr1:	.long	0x00009551
Lwcr2:	.long	0x0000FDD7
Lmcr:	.long	0x0000543C
Lrtcor:	.long	0x0000A560
Lrtcsr:	.long	0x0000A510
Lsdmr:	.long	0xFFFFE880
Lpfc:	.long	0xA4000100
Laaaa:	.long	0x0000AAAA
Lpjcr:	.long	0x0000A544
Lpecr:	.long	0x00001944
Lpfcr:	.long	0x00000009
Lpedr:	.long	0xA4000128
Lsrc:	.long	0xA0000000
Ldst:	.long	0xAC000000
Lwords:	.long	__image_words
Lbss:	.long	__bss_start
Lbssw:	.long	__bss_words
Lmain:	.long	main
