.include "macros.inc"

.section .sbss
.balign 8
.global InterruptHandlerTable
InterruptHandlerTable:
	.skip 0x4
.global __OSLastInterruptSrr0
__OSLastInterruptSrr0:
	.skip 0x4
.global __OSLastInterrupt
__OSLastInterrupt:
	.skip 0x2
	.skip 6
.global __OSLastInterruptTime
__OSLastInterruptTime:
	.skip 0x8

.section .data
.balign 8
.global InterruptPrioTable
InterruptPrioTable:
	.4byte 0x00000100
	.4byte 0x00000040
	.4byte 0xF8000000
	.4byte 0x00000200
	.4byte 0x00000080
	.4byte 0x00003000
	.4byte 0x00000020
	.4byte 0x03FF8C00
	.4byte 0x04000000
	.4byte 0x00004000
	.4byte 0xFFFFFFFF
	.4byte 0x00000000

.section .text, "ax"

.global OSDisableInterrupts
OSDisableInterrupts:
.global __RAS_OSDisableInterrupts_begin
__RAS_OSDisableInterrupts_begin:
/* 80381660 0037E5C0  7C 60 00 A6 */	mfmsr r3
/* 80381664 0037E5C4  54 64 04 5E */	rlwinm r4, r3, 0, 0x11, 0xf
/* 80381668 0037E5C8  7C 80 01 24 */	mtmsr r4
.global __RAS_OSDisableInterrupts_end
__RAS_OSDisableInterrupts_end:
/* 8038166C 0037E5CC  54 63 8F FE */	rlwinm r3, r3, 0x11, 0x1f, 0x1f
/* 80381670 0037E5D0  4E 80 00 20 */	blr

.global OSEnableInterrupts
OSEnableInterrupts:
/* 80381674 0037E5D4  7C 60 00 A6 */	mfmsr r3
/* 80381678 0037E5D8  60 64 80 00 */	ori r4, r3, 0x8000
/* 8038167C 0037E5DC  7C 80 01 24 */	mtmsr r4
/* 80381680 0037E5E0  54 63 8F FE */	rlwinm r3, r3, 0x11, 0x1f, 0x1f
/* 80381684 0037E5E4  4E 80 00 20 */	blr

.global OSRestoreInterrupts
OSRestoreInterrupts:
/* 80381688 0037E5E8  2C 03 00 00 */	cmpwi r3, 0
/* 8038168C 0037E5EC  7C 80 00 A6 */	mfmsr r4
/* 80381690 0037E5F0  41 82 00 0C */	beq lbl_8038169C
/* 80381694 0037E5F4  60 85 80 00 */	ori r5, r4, 0x8000
/* 80381698 0037E5F8  48 00 00 08 */	b lbl_803816A0
lbl_8038169C:
/* 8038169C 0037E5FC  54 85 04 5E */	rlwinm r5, r4, 0, 0x11, 0xf
lbl_803816A0:
/* 803816A0 0037E600  7C A0 01 24 */	mtmsr r5
/* 803816A4 0037E604  54 83 8F FE */	rlwinm r3, r4, 0x11, 0x1f, 0x1f
/* 803816A8 0037E608  4E 80 00 20 */	blr

.global __OSSetInterruptHandler
__OSSetInterruptHandler:
/* 803816AC 0037E60C  7C 60 07 34 */	extsh r0, r3
/* 803816B0 0037E610  80 6D AD 10 */	lwz r3, InterruptHandlerTable@sda21(r13)
/* 803816B4 0037E614  54 00 10 3A */	slwi r0, r0, 2
/* 803816B8 0037E618  7C A3 02 14 */	add r5, r3, r0
/* 803816BC 0037E61C  80 65 00 00 */	lwz r3, 0(r5)
/* 803816C0 0037E620  90 85 00 00 */	stw r4, 0(r5)
/* 803816C4 0037E624  4E 80 00 20 */	blr

.global __OSGetInterruptHandler
__OSGetInterruptHandler:
/* 803816C8 0037E628  7C 60 07 34 */	extsh r0, r3
/* 803816CC 0037E62C  80 6D AD 10 */	lwz r3, InterruptHandlerTable@sda21(r13)
/* 803816D0 0037E630  54 00 10 3A */	slwi r0, r0, 2
/* 803816D4 0037E634  7C 63 00 2E */	lwzx r3, r3, r0
/* 803816D8 0037E638  4E 80 00 20 */	blr

.global __OSInterruptInit
__OSInterruptInit:
/* 803816DC 0037E63C  7C 08 02 A6 */	mflr r0
/* 803816E0 0037E640  90 01 00 04 */	stw r0, 4(r1)
/* 803816E4 0037E644  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 803816E8 0037E648  93 E1 00 0C */	stw r31, 0xc(r1)
/* 803816EC 0037E64C  3F E0 80 00 */	lis r31, 0x80003040@ha
/* 803816F0 0037E650  38 1F 30 40 */	addi r0, r31, 0x80003040@l
/* 803816F4 0037E654  90 0D AD 10 */	stw r0, InterruptHandlerTable@sda21(r13)
/* 803816F8 0037E658  38 80 00 00 */	li r4, 0
/* 803816FC 0037E65C  38 A0 00 80 */	li r5, 0x80
/* 80381700 0037E660  80 6D AD 10 */	lwz r3, InterruptHandlerTable@sda21(r13)
/* 80381704 0037E664  4B C8 1C A5 */	bl memset
/* 80381708 0037E668  38 00 00 00 */	li r0, 0
/* 8038170C 0037E66C  90 1F 00 C4 */	stw r0, 0xc4(r31)
/* 80381710 0037E670  3C 60 CC 00 */	lis r3, 0xCC003000@ha
/* 80381714 0037E674  38 83 30 00 */	addi r4, r3, 0xCC003000@l
/* 80381718 0037E678  90 1F 00 C8 */	stw r0, 0xc8(r31)
/* 8038171C 0037E67C  38 00 00 F0 */	li r0, 0xf0
/* 80381720 0037E680  38 60 FF E0 */	li r3, -32
/* 80381724 0037E684  90 04 00 04 */	stw r0, 4(r4)
/* 80381728 0037E688  48 00 03 01 */	bl __OSMaskInterrupts
/* 8038172C 0037E68C  3C 60 80 38 */	lis r3, ExternalInterruptHandler@ha
/* 80381730 0037E690  38 83 1E 7C */	addi r4, r3, ExternalInterruptHandler@l
/* 80381734 0037E694  38 60 00 04 */	li r3, 4
/* 80381738 0037E698  4B FF C7 91 */	bl __OSSetExceptionHandler
/* 8038173C 0037E69C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80381740 0037E6A0  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80381744 0037E6A4  38 21 00 10 */	addi r1, r1, 0x10
/* 80381748 0037E6A8  7C 08 03 A6 */	mtlr r0
/* 8038174C 0037E6AC  4E 80 00 20 */	blr

.global SetInterruptMask
SetInterruptMask:
/* 80381750 0037E6B0  7C 60 00 34 */	cntlzw r0, r3
/* 80381754 0037E6B4  2C 00 00 0C */	cmpwi r0, 0xc
/* 80381758 0037E6B8  40 80 00 24 */	bge lbl_8038177C
/* 8038175C 0037E6BC  2C 00 00 08 */	cmpwi r0, 8
/* 80381760 0037E6C0  41 82 00 FC */	beq lbl_8038185C
/* 80381764 0037E6C4  40 80 01 28 */	bge lbl_8038188C
/* 80381768 0037E6C8  2C 00 00 05 */	cmpwi r0, 5
/* 8038176C 0037E6CC  40 80 00 9C */	bge lbl_80381808
/* 80381770 0037E6D0  2C 00 00 00 */	cmpwi r0, 0
/* 80381774 0037E6D4  40 80 00 28 */	bge lbl_8038179C
/* 80381778 0037E6D8  48 00 02 AC */	b lbl_80381A24
lbl_8038177C:
/* 8038177C 0037E6DC  2C 00 00 11 */	cmpwi r0, 0x11
/* 80381780 0037E6E0  40 80 00 10 */	bge lbl_80381790
/* 80381784 0037E6E4  2C 00 00 0F */	cmpwi r0, 0xf
/* 80381788 0037E6E8  40 80 01 A8 */	bge lbl_80381930
/* 8038178C 0037E6EC  48 00 01 50 */	b lbl_803818DC
lbl_80381790:
/* 80381790 0037E6F0  2C 00 00 1B */	cmpwi r0, 0x1b
/* 80381794 0037E6F4  40 80 02 90 */	bge lbl_80381A24
/* 80381798 0037E6F8  48 00 01 D8 */	b lbl_80381970
lbl_8038179C:
/* 8038179C 0037E6FC  54 80 00 00 */	rlwinm r0, r4, 0, 0, 0
/* 803817A0 0037E700  28 00 00 00 */	cmplwi r0, 0
/* 803817A4 0037E704  38 A0 00 00 */	li r5, 0
/* 803817A8 0037E708  40 82 00 08 */	bne lbl_803817B0
/* 803817AC 0037E70C  60 A5 00 01 */	ori r5, r5, 1
lbl_803817B0:
/* 803817B0 0037E710  54 80 00 42 */	rlwinm r0, r4, 0, 1, 1
/* 803817B4 0037E714  28 00 00 00 */	cmplwi r0, 0
/* 803817B8 0037E718  40 82 00 08 */	bne lbl_803817C0
/* 803817BC 0037E71C  60 A5 00 02 */	ori r5, r5, 2
lbl_803817C0:
/* 803817C0 0037E720  54 80 00 84 */	rlwinm r0, r4, 0, 2, 2
/* 803817C4 0037E724  28 00 00 00 */	cmplwi r0, 0
/* 803817C8 0037E728  40 82 00 08 */	bne lbl_803817D0
/* 803817CC 0037E72C  60 A5 00 04 */	ori r5, r5, 4
lbl_803817D0:
/* 803817D0 0037E730  54 80 00 C6 */	rlwinm r0, r4, 0, 3, 3
/* 803817D4 0037E734  28 00 00 00 */	cmplwi r0, 0
/* 803817D8 0037E738  40 82 00 08 */	bne lbl_803817E0
/* 803817DC 0037E73C  60 A5 00 08 */	ori r5, r5, 8
lbl_803817E0:
/* 803817E0 0037E740  54 80 01 08 */	rlwinm r0, r4, 0, 4, 4
/* 803817E4 0037E744  28 00 00 00 */	cmplwi r0, 0
/* 803817E8 0037E748  40 82 00 08 */	bne lbl_803817F0
/* 803817EC 0037E74C  60 A5 00 10 */	ori r5, r5, 0x10
lbl_803817F0:
/* 803817F0 0037E750  3C 80 CC 00 */	lis r4, 0xCC004000@ha
/* 803817F4 0037E754  54 A0 04 3E */	clrlwi r0, r5, 0x10
/* 803817F8 0037E758  38 84 40 00 */	addi r4, r4, 0xCC004000@l
/* 803817FC 0037E75C  B0 04 00 1C */	sth r0, 0x1c(r4)
/* 80381800 0037E760  54 63 01 7E */	clrlwi r3, r3, 5
/* 80381804 0037E764  48 00 02 20 */	b lbl_80381A24
lbl_80381808:
/* 80381808 0037E768  3C A0 CC 00 */	lis r5, 0xCC005000@ha
/* 8038180C 0037E76C  38 A5 50 00 */	addi r5, r5, 0xCC005000@l
/* 80381810 0037E770  38 A5 00 0A */	addi r5, r5, 0xa
/* 80381814 0037E774  54 80 01 4A */	rlwinm r0, r4, 0, 5, 5
/* 80381818 0037E778  A0 C5 00 00 */	lhz r6, 0(r5)
/* 8038181C 0037E77C  28 00 00 00 */	cmplwi r0, 0
/* 80381820 0037E780  54 C6 07 6C */	rlwinm r6, r6, 0, 0x1d, 0x16
/* 80381824 0037E784  40 82 00 08 */	bne lbl_8038182C
/* 80381828 0037E788  60 C6 00 10 */	ori r6, r6, 0x10
lbl_8038182C:
/* 8038182C 0037E78C  54 80 01 8C */	rlwinm r0, r4, 0, 6, 6
/* 80381830 0037E790  28 00 00 00 */	cmplwi r0, 0
/* 80381834 0037E794  40 82 00 08 */	bne lbl_8038183C
/* 80381838 0037E798  60 C6 00 40 */	ori r6, r6, 0x40
lbl_8038183C:
/* 8038183C 0037E79C  54 80 01 CE */	rlwinm r0, r4, 0, 7, 7
/* 80381840 0037E7A0  28 00 00 00 */	cmplwi r0, 0
/* 80381844 0037E7A4  40 82 00 08 */	bne lbl_8038184C
/* 80381848 0037E7A8  60 C6 01 00 */	ori r6, r6, 0x100
lbl_8038184C:
/* 8038184C 0037E7AC  54 C0 04 3E */	clrlwi r0, r6, 0x10
/* 80381850 0037E7B0  B0 05 00 00 */	sth r0, 0(r5)
/* 80381854 0037E7B4  54 63 02 08 */	rlwinm r3, r3, 0, 8, 4
/* 80381858 0037E7B8  48 00 01 CC */	b lbl_80381A24
lbl_8038185C:
/* 8038185C 0037E7BC  54 80 02 10 */	rlwinm r0, r4, 0, 8, 8
/* 80381860 0037E7C0  3C 80 CC 00 */	lis r4, 0xCC006C00@ha
/* 80381864 0037E7C4  28 00 00 00 */	cmplwi r0, 0
/* 80381868 0037E7C8  80 A4 6C 00 */	lwz r5, 0xCC006C00@l(r4)
/* 8038186C 0037E7CC  38 00 FF D3 */	li r0, -45
/* 80381870 0037E7D0  7C A5 00 38 */	and r5, r5, r0
/* 80381874 0037E7D4  40 82 00 08 */	bne lbl_8038187C
/* 80381878 0037E7D8  60 A5 00 04 */	ori r5, r5, 4
lbl_8038187C:
/* 8038187C 0037E7DC  3C 80 CC 00 */	lis r4, 0xCC006C00@ha
/* 80381880 0037E7E0  90 A4 6C 00 */	stw r5, 0xCC006C00@l(r4)
/* 80381884 0037E7E4  54 63 02 4E */	rlwinm r3, r3, 0, 9, 7
/* 80381888 0037E7E8  48 00 01 9C */	b lbl_80381A24
lbl_8038188C:
/* 8038188C 0037E7EC  54 80 02 52 */	rlwinm r0, r4, 0, 9, 9
/* 80381890 0037E7F0  3C A0 CC 00 */	lis r5, 0xCC006800@ha
/* 80381894 0037E7F4  28 00 00 00 */	cmplwi r0, 0
/* 80381898 0037E7F8  80 A5 68 00 */	lwz r5, 0xCC006800@l(r5)
/* 8038189C 0037E7FC  38 00 D3 F0 */	li r0, -11280
/* 803818A0 0037E800  7C A5 00 38 */	and r5, r5, r0
/* 803818A4 0037E804  40 82 00 08 */	bne lbl_803818AC
/* 803818A8 0037E808  60 A5 00 01 */	ori r5, r5, 1
lbl_803818AC:
/* 803818AC 0037E80C  54 80 02 94 */	rlwinm r0, r4, 0, 0xa, 0xa
/* 803818B0 0037E810  28 00 00 00 */	cmplwi r0, 0
/* 803818B4 0037E814  40 82 00 08 */	bne lbl_803818BC
/* 803818B8 0037E818  60 A5 00 04 */	ori r5, r5, 4
lbl_803818BC:
/* 803818BC 0037E81C  54 80 02 D6 */	rlwinm r0, r4, 0, 0xb, 0xb
/* 803818C0 0037E820  28 00 00 00 */	cmplwi r0, 0
/* 803818C4 0037E824  40 82 00 08 */	bne lbl_803818CC
/* 803818C8 0037E828  60 A5 04 00 */	ori r5, r5, 0x400
lbl_803818CC:
/* 803818CC 0037E82C  3C 80 CC 00 */	lis r4, 0xCC006800@ha
/* 803818D0 0037E830  90 A4 68 00 */	stw r5, 0xCC006800@l(r4)
/* 803818D4 0037E834  54 63 03 10 */	rlwinm r3, r3, 0, 0xc, 8
/* 803818D8 0037E838  48 00 01 4C */	b lbl_80381A24
lbl_803818DC:
/* 803818DC 0037E83C  3C A0 CC 00 */	lis r5, 0xCC006800@ha
/* 803818E0 0037E840  38 C5 68 00 */	addi r6, r5, 0xCC006800@l
/* 803818E4 0037E844  38 C6 00 14 */	addi r6, r6, 0x14
/* 803818E8 0037E848  54 80 03 18 */	rlwinm r0, r4, 0, 0xc, 0xc
/* 803818EC 0037E84C  80 E6 00 00 */	lwz r7, 0(r6)
/* 803818F0 0037E850  38 A0 F3 F0 */	li r5, -3088
/* 803818F4 0037E854  28 00 00 00 */	cmplwi r0, 0
/* 803818F8 0037E858  7C E7 28 38 */	and r7, r7, r5
/* 803818FC 0037E85C  40 82 00 08 */	bne lbl_80381904
/* 80381900 0037E860  60 E7 00 01 */	ori r7, r7, 1
lbl_80381904:
/* 80381904 0037E864  54 80 03 5A */	rlwinm r0, r4, 0, 0xd, 0xd
/* 80381908 0037E868  28 00 00 00 */	cmplwi r0, 0
/* 8038190C 0037E86C  40 82 00 08 */	bne lbl_80381914
/* 80381910 0037E870  60 E7 00 04 */	ori r7, r7, 4
lbl_80381914:
/* 80381914 0037E874  54 80 03 9C */	rlwinm r0, r4, 0, 0xe, 0xe
/* 80381918 0037E878  28 00 00 00 */	cmplwi r0, 0
/* 8038191C 0037E87C  40 82 00 08 */	bne lbl_80381924
/* 80381920 0037E880  60 E7 04 00 */	ori r7, r7, 0x400
lbl_80381924:
/* 80381924 0037E884  90 E6 00 00 */	stw r7, 0(r6)
/* 80381928 0037E888  54 63 03 D6 */	rlwinm r3, r3, 0, 0xf, 0xb
/* 8038192C 0037E88C  48 00 00 F8 */	b lbl_80381A24
lbl_80381930:
/* 80381930 0037E890  3C A0 CC 00 */	lis r5, 0xCC006800@ha
/* 80381934 0037E894  38 A5 68 00 */	addi r5, r5, 0xCC006800@l
/* 80381938 0037E898  38 A5 00 28 */	addi r5, r5, 0x28
/* 8038193C 0037E89C  54 80 03 DE */	rlwinm r0, r4, 0, 0xf, 0xf
/* 80381940 0037E8A0  80 C5 00 00 */	lwz r6, 0(r5)
/* 80381944 0037E8A4  28 00 00 00 */	cmplwi r0, 0
/* 80381948 0037E8A8  54 C6 00 36 */	rlwinm r6, r6, 0, 0, 0x1b
/* 8038194C 0037E8AC  40 82 00 08 */	bne lbl_80381954
/* 80381950 0037E8B0  60 C6 00 01 */	ori r6, r6, 1
lbl_80381954:
/* 80381954 0037E8B4  54 80 04 20 */	rlwinm r0, r4, 0, 0x10, 0x10
/* 80381958 0037E8B8  28 00 00 00 */	cmplwi r0, 0
/* 8038195C 0037E8BC  40 82 00 08 */	bne lbl_80381964
/* 80381960 0037E8C0  60 C6 00 04 */	ori r6, r6, 4
lbl_80381964:
/* 80381964 0037E8C4  90 C5 00 00 */	stw r6, 0(r5)
/* 80381968 0037E8C8  54 63 04 5C */	rlwinm r3, r3, 0, 0x11, 0xe
/* 8038196C 0037E8CC  48 00 00 B8 */	b lbl_80381A24
lbl_80381970:
/* 80381970 0037E8D0  54 80 04 62 */	rlwinm r0, r4, 0, 0x11, 0x11
/* 80381974 0037E8D4  28 00 00 00 */	cmplwi r0, 0
/* 80381978 0037E8D8  38 A0 00 F0 */	li r5, 0xf0
/* 8038197C 0037E8DC  40 82 00 08 */	bne lbl_80381984
/* 80381980 0037E8E0  60 A5 08 00 */	ori r5, r5, 0x800
lbl_80381984:
/* 80381984 0037E8E4  54 80 05 28 */	rlwinm r0, r4, 0, 0x14, 0x14
/* 80381988 0037E8E8  28 00 00 00 */	cmplwi r0, 0
/* 8038198C 0037E8EC  40 82 00 08 */	bne lbl_80381994
/* 80381990 0037E8F0  60 A5 00 08 */	ori r5, r5, 8
lbl_80381994:
/* 80381994 0037E8F4  54 80 05 6A */	rlwinm r0, r4, 0, 0x15, 0x15
/* 80381998 0037E8F8  28 00 00 00 */	cmplwi r0, 0
/* 8038199C 0037E8FC  40 82 00 08 */	bne lbl_803819A4
/* 803819A0 0037E900  60 A5 00 04 */	ori r5, r5, 4
lbl_803819A4:
/* 803819A4 0037E904  54 80 05 AC */	rlwinm r0, r4, 0, 0x16, 0x16
/* 803819A8 0037E908  28 00 00 00 */	cmplwi r0, 0
/* 803819AC 0037E90C  40 82 00 08 */	bne lbl_803819B4
/* 803819B0 0037E910  60 A5 00 02 */	ori r5, r5, 2
lbl_803819B4:
/* 803819B4 0037E914  54 80 05 EE */	rlwinm r0, r4, 0, 0x17, 0x17
/* 803819B8 0037E918  28 00 00 00 */	cmplwi r0, 0
/* 803819BC 0037E91C  40 82 00 08 */	bne lbl_803819C4
/* 803819C0 0037E920  60 A5 00 01 */	ori r5, r5, 1
lbl_803819C4:
/* 803819C4 0037E924  54 80 06 30 */	rlwinm r0, r4, 0, 0x18, 0x18
/* 803819C8 0037E928  28 00 00 00 */	cmplwi r0, 0
/* 803819CC 0037E92C  40 82 00 08 */	bne lbl_803819D4
/* 803819D0 0037E930  60 A5 01 00 */	ori r5, r5, 0x100
lbl_803819D4:
/* 803819D4 0037E934  54 80 06 72 */	rlwinm r0, r4, 0, 0x19, 0x19
/* 803819D8 0037E938  28 00 00 00 */	cmplwi r0, 0
/* 803819DC 0037E93C  40 82 00 08 */	bne lbl_803819E4
/* 803819E0 0037E940  60 A5 10 00 */	ori r5, r5, 0x1000
lbl_803819E4:
/* 803819E4 0037E944  54 80 04 A4 */	rlwinm r0, r4, 0, 0x12, 0x12
/* 803819E8 0037E948  28 00 00 00 */	cmplwi r0, 0
/* 803819EC 0037E94C  40 82 00 08 */	bne lbl_803819F4
/* 803819F0 0037E950  60 A5 02 00 */	ori r5, r5, 0x200
lbl_803819F4:
/* 803819F4 0037E954  54 80 04 E6 */	rlwinm r0, r4, 0, 0x13, 0x13
/* 803819F8 0037E958  28 00 00 00 */	cmplwi r0, 0
/* 803819FC 0037E95C  40 82 00 08 */	bne lbl_80381A04
/* 80381A00 0037E960  60 A5 04 00 */	ori r5, r5, 0x400
lbl_80381A04:
/* 80381A04 0037E964  54 80 06 B4 */	rlwinm r0, r4, 0, 0x1a, 0x1a
/* 80381A08 0037E968  28 00 00 00 */	cmplwi r0, 0
/* 80381A0C 0037E96C  40 82 00 08 */	bne lbl_80381A14
/* 80381A10 0037E970  60 A5 20 00 */	ori r5, r5, 0x2000
lbl_80381A14:
/* 80381A14 0037E974  3C 80 CC 00 */	lis r4, 0xCC003000@ha
/* 80381A18 0037E978  38 84 30 00 */	addi r4, r4, 0xCC003000@l
/* 80381A1C 0037E97C  90 A4 00 04 */	stw r5, 4(r4)
/* 80381A20 0037E980  54 63 06 E0 */	rlwinm r3, r3, 0, 0x1b, 0x10
lbl_80381A24:
/* 80381A24 0037E984  4E 80 00 20 */	blr

.global __OSMaskInterrupts
__OSMaskInterrupts:
/* 80381A28 0037E988  7C 08 02 A6 */	mflr r0
/* 80381A2C 0037E98C  90 01 00 04 */	stw r0, 4(r1)
/* 80381A30 0037E990  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80381A34 0037E994  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80381A38 0037E998  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80381A3C 0037E99C  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80381A40 0037E9A0  7C 7F 1B 78 */	mr r31, r3
/* 80381A44 0037E9A4  4B FF FC 1D */	bl OSDisableInterrupts
/* 80381A48 0037E9A8  3C 80 80 00 */	lis r4, 0x800000C4@ha
/* 80381A4C 0037E9AC  83 A4 00 C4 */	lwz r29, 0x800000C4@l(r4)
/* 80381A50 0037E9B0  7C 7E 1B 78 */	mr r30, r3
/* 80381A54 0037E9B4  80 A4 00 C8 */	lwz r5, 0xc8(r4)
/* 80381A58 0037E9B8  7F A0 2B 78 */	or r0, r29, r5
/* 80381A5C 0037E9BC  7F E3 00 78 */	andc r3, r31, r0
/* 80381A60 0037E9C0  7F FF EB 78 */	or r31, r31, r29
/* 80381A64 0037E9C4  93 E4 00 C4 */	stw r31, 0xc4(r4)
/* 80381A68 0037E9C8  7F FF 2B 78 */	or r31, r31, r5
/* 80381A6C 0037E9CC  48 00 00 04 */	b lbl_80381A70
lbl_80381A70:
/* 80381A70 0037E9D0  48 00 00 04 */	b lbl_80381A74
lbl_80381A74:
/* 80381A74 0037E9D4  48 00 00 0C */	b lbl_80381A80
lbl_80381A78:
/* 80381A78 0037E9D8  7F E4 FB 78 */	mr r4, r31
/* 80381A7C 0037E9DC  4B FF FC D5 */	bl SetInterruptMask
lbl_80381A80:
/* 80381A80 0037E9E0  28 03 00 00 */	cmplwi r3, 0
/* 80381A84 0037E9E4  40 82 FF F4 */	bne lbl_80381A78
/* 80381A88 0037E9E8  7F C3 F3 78 */	mr r3, r30
/* 80381A8C 0037E9EC  4B FF FB FD */	bl OSRestoreInterrupts
/* 80381A90 0037E9F0  7F A3 EB 78 */	mr r3, r29
/* 80381A94 0037E9F4  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80381A98 0037E9F8  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80381A9C 0037E9FC  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 80381AA0 0037EA00  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80381AA4 0037EA04  38 21 00 20 */	addi r1, r1, 0x20
/* 80381AA8 0037EA08  7C 08 03 A6 */	mtlr r0
/* 80381AAC 0037EA0C  4E 80 00 20 */	blr

.global __OSUnmaskInterrupts
__OSUnmaskInterrupts:
/* 80381AB0 0037EA10  7C 08 02 A6 */	mflr r0
/* 80381AB4 0037EA14  90 01 00 04 */	stw r0, 4(r1)
/* 80381AB8 0037EA18  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80381ABC 0037EA1C  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80381AC0 0037EA20  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80381AC4 0037EA24  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80381AC8 0037EA28  7C 7F 1B 78 */	mr r31, r3
/* 80381ACC 0037EA2C  4B FF FB 95 */	bl OSDisableInterrupts
/* 80381AD0 0037EA30  3C 80 80 00 */	lis r4, 0x800000C4@ha
/* 80381AD4 0037EA34  83 A4 00 C4 */	lwz r29, 0x800000C4@l(r4)
/* 80381AD8 0037EA38  7C 7E 1B 78 */	mr r30, r3
/* 80381ADC 0037EA3C  80 A4 00 C8 */	lwz r5, 0xc8(r4)
/* 80381AE0 0037EA40  7F A0 2B 78 */	or r0, r29, r5
/* 80381AE4 0037EA44  7F E3 00 38 */	and r3, r31, r0
/* 80381AE8 0037EA48  7F BF F8 78 */	andc r31, r29, r31
/* 80381AEC 0037EA4C  93 E4 00 C4 */	stw r31, 0xc4(r4)
/* 80381AF0 0037EA50  7F FF 2B 78 */	or r31, r31, r5
/* 80381AF4 0037EA54  48 00 00 04 */	b lbl_80381AF8
lbl_80381AF8:
/* 80381AF8 0037EA58  48 00 00 04 */	b lbl_80381AFC
lbl_80381AFC:
/* 80381AFC 0037EA5C  48 00 00 0C */	b lbl_80381B08
lbl_80381B00:
/* 80381B00 0037EA60  7F E4 FB 78 */	mr r4, r31
/* 80381B04 0037EA64  4B FF FC 4D */	bl SetInterruptMask
lbl_80381B08:
/* 80381B08 0037EA68  28 03 00 00 */	cmplwi r3, 0
/* 80381B0C 0037EA6C  40 82 FF F4 */	bne lbl_80381B00
/* 80381B10 0037EA70  7F C3 F3 78 */	mr r3, r30
/* 80381B14 0037EA74  4B FF FB 75 */	bl OSRestoreInterrupts
/* 80381B18 0037EA78  7F A3 EB 78 */	mr r3, r29
/* 80381B1C 0037EA7C  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80381B20 0037EA80  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80381B24 0037EA84  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 80381B28 0037EA88  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80381B2C 0037EA8C  38 21 00 20 */	addi r1, r1, 0x20
/* 80381B30 0037EA90  7C 08 03 A6 */	mtlr r0
/* 80381B34 0037EA94  4E 80 00 20 */	blr

.global __OSDispatchInterrupt
__OSDispatchInterrupt:
/* 80381B38 0037EA98  7C 08 02 A6 */	mflr r0
/* 80381B3C 0037EA9C  90 01 00 04 */	stw r0, 4(r1)
/* 80381B40 0037EAA0  94 21 FF D8 */	stwu r1, -0x28(r1)
/* 80381B44 0037EAA4  93 E1 00 24 */	stw r31, 0x24(r1)
/* 80381B48 0037EAA8  93 C1 00 20 */	stw r30, 0x20(r1)
/* 80381B4C 0037EAAC  93 A1 00 1C */	stw r29, 0x1c(r1)
/* 80381B50 0037EAB0  7C 9E 23 78 */	mr r30, r4
/* 80381B54 0037EAB4  3C 60 CC 00 */	lis r3, 0xCC003000@ha
/* 80381B58 0037EAB8  83 E3 30 00 */	lwz r31, 0xCC003000@l(r3)
/* 80381B5C 0037EABC  57 FF 04 1C */	rlwinm r31, r31, 0, 0x10, 0xe
/* 80381B60 0037EAC0  28 1F 00 00 */	cmplwi r31, 0
/* 80381B64 0037EAC4  41 82 00 18 */	beq lbl_80381B7C
/* 80381B68 0037EAC8  38 63 30 00 */	addi r3, r3, 0x3000
/* 80381B6C 0037EACC  80 03 00 04 */	lwz r0, 4(r3)
/* 80381B70 0037EAD0  7F E0 00 38 */	and r0, r31, r0
/* 80381B74 0037EAD4  28 00 00 00 */	cmplwi r0, 0
/* 80381B78 0037EAD8  40 82 00 0C */	bne lbl_80381B84
lbl_80381B7C:
/* 80381B7C 0037EADC  7F C3 F3 78 */	mr r3, r30
/* 80381B80 0037EAE0  4B FF D9 85 */	bl OSLoadContext
lbl_80381B84:
/* 80381B84 0037EAE4  57 E0 06 30 */	rlwinm r0, r31, 0, 0x18, 0x18
/* 80381B88 0037EAE8  28 00 00 00 */	cmplwi r0, 0
/* 80381B8C 0037EAEC  38 00 00 00 */	li r0, 0
/* 80381B90 0037EAF0  41 82 00 60 */	beq lbl_80381BF0
/* 80381B94 0037EAF4  3C 60 CC 00 */	lis r3, 0xCC004000@ha
/* 80381B98 0037EAF8  38 63 40 00 */	addi r3, r3, 0xCC004000@l
/* 80381B9C 0037EAFC  A0 83 00 1E */	lhz r4, 0x1e(r3)
/* 80381BA0 0037EB00  54 83 07 FE */	clrlwi r3, r4, 0x1f
/* 80381BA4 0037EB04  28 03 00 00 */	cmplwi r3, 0
/* 80381BA8 0037EB08  41 82 00 08 */	beq lbl_80381BB0
/* 80381BAC 0037EB0C  64 00 80 00 */	oris r0, r0, 0x8000
lbl_80381BB0:
/* 80381BB0 0037EB10  54 83 07 BC */	rlwinm r3, r4, 0, 0x1e, 0x1e
/* 80381BB4 0037EB14  28 03 00 00 */	cmplwi r3, 0
/* 80381BB8 0037EB18  41 82 00 08 */	beq lbl_80381BC0
/* 80381BBC 0037EB1C  64 00 40 00 */	oris r0, r0, 0x4000
lbl_80381BC0:
/* 80381BC0 0037EB20  54 83 07 7A */	rlwinm r3, r4, 0, 0x1d, 0x1d
/* 80381BC4 0037EB24  28 03 00 00 */	cmplwi r3, 0
/* 80381BC8 0037EB28  41 82 00 08 */	beq lbl_80381BD0
/* 80381BCC 0037EB2C  64 00 20 00 */	oris r0, r0, 0x2000
lbl_80381BD0:
/* 80381BD0 0037EB30  54 83 07 38 */	rlwinm r3, r4, 0, 0x1c, 0x1c
/* 80381BD4 0037EB34  28 03 00 00 */	cmplwi r3, 0
/* 80381BD8 0037EB38  41 82 00 08 */	beq lbl_80381BE0
/* 80381BDC 0037EB3C  64 00 10 00 */	oris r0, r0, 0x1000
lbl_80381BE0:
/* 80381BE0 0037EB40  54 83 06 F6 */	rlwinm r3, r4, 0, 0x1b, 0x1b
/* 80381BE4 0037EB44  28 03 00 00 */	cmplwi r3, 0
/* 80381BE8 0037EB48  41 82 00 08 */	beq lbl_80381BF0
/* 80381BEC 0037EB4C  64 00 08 00 */	oris r0, r0, 0x800
lbl_80381BF0:
/* 80381BF0 0037EB50  57 E3 06 72 */	rlwinm r3, r31, 0, 0x19, 0x19
/* 80381BF4 0037EB54  28 03 00 00 */	cmplwi r3, 0
/* 80381BF8 0037EB58  41 82 00 40 */	beq lbl_80381C38
/* 80381BFC 0037EB5C  3C 60 CC 00 */	lis r3, 0xCC005000@ha
/* 80381C00 0037EB60  38 63 50 00 */	addi r3, r3, 0xCC005000@l
/* 80381C04 0037EB64  A0 83 00 0A */	lhz r4, 0xa(r3)
/* 80381C08 0037EB68  54 83 07 38 */	rlwinm r3, r4, 0, 0x1c, 0x1c
/* 80381C0C 0037EB6C  28 03 00 00 */	cmplwi r3, 0
/* 80381C10 0037EB70  41 82 00 08 */	beq lbl_80381C18
/* 80381C14 0037EB74  64 00 04 00 */	oris r0, r0, 0x400
lbl_80381C18:
/* 80381C18 0037EB78  54 83 06 B4 */	rlwinm r3, r4, 0, 0x1a, 0x1a
/* 80381C1C 0037EB7C  28 03 00 00 */	cmplwi r3, 0
/* 80381C20 0037EB80  41 82 00 08 */	beq lbl_80381C28
/* 80381C24 0037EB84  64 00 02 00 */	oris r0, r0, 0x200
lbl_80381C28:
/* 80381C28 0037EB88  54 83 06 30 */	rlwinm r3, r4, 0, 0x18, 0x18
/* 80381C2C 0037EB8C  28 03 00 00 */	cmplwi r3, 0
/* 80381C30 0037EB90  41 82 00 08 */	beq lbl_80381C38
/* 80381C34 0037EB94  64 00 01 00 */	oris r0, r0, 0x100
lbl_80381C38:
/* 80381C38 0037EB98  57 E3 06 B4 */	rlwinm r3, r31, 0, 0x1a, 0x1a
/* 80381C3C 0037EB9C  28 03 00 00 */	cmplwi r3, 0
/* 80381C40 0037EBA0  41 82 00 1C */	beq lbl_80381C5C
/* 80381C44 0037EBA4  3C 60 CC 00 */	lis r3, 0xCC006C00@ha
/* 80381C48 0037EBA8  80 63 6C 00 */	lwz r3, 0xCC006C00@l(r3)
/* 80381C4C 0037EBAC  54 63 07 38 */	rlwinm r3, r3, 0, 0x1c, 0x1c
/* 80381C50 0037EBB0  28 03 00 00 */	cmplwi r3, 0
/* 80381C54 0037EBB4  41 82 00 08 */	beq lbl_80381C5C
/* 80381C58 0037EBB8  64 00 00 80 */	oris r0, r0, 0x80
lbl_80381C5C:
/* 80381C5C 0037EBBC  57 E3 06 F6 */	rlwinm r3, r31, 0, 0x1b, 0x1b
/* 80381C60 0037EBC0  28 03 00 00 */	cmplwi r3, 0
/* 80381C64 0037EBC4  41 82 00 A4 */	beq lbl_80381D08
/* 80381C68 0037EBC8  3C 60 CC 00 */	lis r3, 0xCC006800@ha
/* 80381C6C 0037EBCC  80 83 68 00 */	lwz r4, 0xCC006800@l(r3)
/* 80381C70 0037EBD0  54 83 07 BC */	rlwinm r3, r4, 0, 0x1e, 0x1e
/* 80381C74 0037EBD4  28 03 00 00 */	cmplwi r3, 0
/* 80381C78 0037EBD8  41 82 00 08 */	beq lbl_80381C80
/* 80381C7C 0037EBDC  64 00 00 40 */	oris r0, r0, 0x40
lbl_80381C80:
/* 80381C80 0037EBE0  54 83 07 38 */	rlwinm r3, r4, 0, 0x1c, 0x1c
/* 80381C84 0037EBE4  28 03 00 00 */	cmplwi r3, 0
/* 80381C88 0037EBE8  41 82 00 08 */	beq lbl_80381C90
/* 80381C8C 0037EBEC  64 00 00 20 */	oris r0, r0, 0x20
lbl_80381C90:
/* 80381C90 0037EBF0  54 83 05 28 */	rlwinm r3, r4, 0, 0x14, 0x14
/* 80381C94 0037EBF4  28 03 00 00 */	cmplwi r3, 0
/* 80381C98 0037EBF8  41 82 00 08 */	beq lbl_80381CA0
/* 80381C9C 0037EBFC  64 00 00 10 */	oris r0, r0, 0x10
lbl_80381CA0:
/* 80381CA0 0037EC00  3C 60 CC 00 */	lis r3, 0xCC006800@ha
/* 80381CA4 0037EC04  38 63 68 00 */	addi r3, r3, 0xCC006800@l
/* 80381CA8 0037EC08  80 83 00 14 */	lwz r4, 0x14(r3)
/* 80381CAC 0037EC0C  54 83 07 BC */	rlwinm r3, r4, 0, 0x1e, 0x1e
/* 80381CB0 0037EC10  28 03 00 00 */	cmplwi r3, 0
/* 80381CB4 0037EC14  41 82 00 08 */	beq lbl_80381CBC
/* 80381CB8 0037EC18  64 00 00 08 */	oris r0, r0, 8
lbl_80381CBC:
/* 80381CBC 0037EC1C  54 83 07 38 */	rlwinm r3, r4, 0, 0x1c, 0x1c
/* 80381CC0 0037EC20  28 03 00 00 */	cmplwi r3, 0
/* 80381CC4 0037EC24  41 82 00 08 */	beq lbl_80381CCC
/* 80381CC8 0037EC28  64 00 00 04 */	oris r0, r0, 4
lbl_80381CCC:
/* 80381CCC 0037EC2C  54 83 05 28 */	rlwinm r3, r4, 0, 0x14, 0x14
/* 80381CD0 0037EC30  28 03 00 00 */	cmplwi r3, 0
/* 80381CD4 0037EC34  41 82 00 08 */	beq lbl_80381CDC
/* 80381CD8 0037EC38  64 00 00 02 */	oris r0, r0, 2
lbl_80381CDC:
/* 80381CDC 0037EC3C  3C 60 CC 00 */	lis r3, 0xCC006800@ha
/* 80381CE0 0037EC40  38 63 68 00 */	addi r3, r3, 0xCC006800@l
/* 80381CE4 0037EC44  80 83 00 28 */	lwz r4, 0x28(r3)
/* 80381CE8 0037EC48  54 83 07 BC */	rlwinm r3, r4, 0, 0x1e, 0x1e
/* 80381CEC 0037EC4C  28 03 00 00 */	cmplwi r3, 0
/* 80381CF0 0037EC50  41 82 00 08 */	beq lbl_80381CF8
/* 80381CF4 0037EC54  64 00 00 01 */	oris r0, r0, 1
lbl_80381CF8:
/* 80381CF8 0037EC58  54 83 07 38 */	rlwinm r3, r4, 0, 0x1c, 0x1c
/* 80381CFC 0037EC5C  28 03 00 00 */	cmplwi r3, 0
/* 80381D00 0037EC60  41 82 00 08 */	beq lbl_80381D08
/* 80381D04 0037EC64  60 00 80 00 */	ori r0, r0, 0x8000
lbl_80381D08:
/* 80381D08 0037EC68  57 E3 04 A4 */	rlwinm r3, r31, 0, 0x12, 0x12
/* 80381D0C 0037EC6C  28 03 00 00 */	cmplwi r3, 0
/* 80381D10 0037EC70  41 82 00 08 */	beq lbl_80381D18
/* 80381D14 0037EC74  60 00 00 20 */	ori r0, r0, 0x20
lbl_80381D18:
/* 80381D18 0037EC78  57 E3 04 E6 */	rlwinm r3, r31, 0, 0x13, 0x13
/* 80381D1C 0037EC7C  28 03 00 00 */	cmplwi r3, 0
/* 80381D20 0037EC80  41 82 00 08 */	beq lbl_80381D28
/* 80381D24 0037EC84  60 00 00 40 */	ori r0, r0, 0x40
lbl_80381D28:
/* 80381D28 0037EC88  57 E3 05 6A */	rlwinm r3, r31, 0, 0x15, 0x15
/* 80381D2C 0037EC8C  28 03 00 00 */	cmplwi r3, 0
/* 80381D30 0037EC90  41 82 00 08 */	beq lbl_80381D38
/* 80381D34 0037EC94  60 00 10 00 */	ori r0, r0, 0x1000
lbl_80381D38:
/* 80381D38 0037EC98  57 E3 05 AC */	rlwinm r3, r31, 0, 0x16, 0x16
/* 80381D3C 0037EC9C  28 03 00 00 */	cmplwi r3, 0
/* 80381D40 0037ECA0  41 82 00 08 */	beq lbl_80381D48
/* 80381D44 0037ECA4  60 00 20 00 */	ori r0, r0, 0x2000
lbl_80381D48:
/* 80381D48 0037ECA8  57 E3 05 EE */	rlwinm r3, r31, 0, 0x17, 0x17
/* 80381D4C 0037ECAC  28 03 00 00 */	cmplwi r3, 0
/* 80381D50 0037ECB0  41 82 00 08 */	beq lbl_80381D58
/* 80381D54 0037ECB4  60 00 00 80 */	ori r0, r0, 0x80
lbl_80381D58:
/* 80381D58 0037ECB8  57 E3 07 38 */	rlwinm r3, r31, 0, 0x1c, 0x1c
/* 80381D5C 0037ECBC  28 03 00 00 */	cmplwi r3, 0
/* 80381D60 0037ECC0  41 82 00 08 */	beq lbl_80381D68
/* 80381D64 0037ECC4  60 00 08 00 */	ori r0, r0, 0x800
lbl_80381D68:
/* 80381D68 0037ECC8  57 E3 07 7A */	rlwinm r3, r31, 0, 0x1d, 0x1d
/* 80381D6C 0037ECCC  28 03 00 00 */	cmplwi r3, 0
/* 80381D70 0037ECD0  41 82 00 08 */	beq lbl_80381D78
/* 80381D74 0037ECD4  60 00 04 00 */	ori r0, r0, 0x400
lbl_80381D78:
/* 80381D78 0037ECD8  57 E3 07 BC */	rlwinm r3, r31, 0, 0x1e, 0x1e
/* 80381D7C 0037ECDC  28 03 00 00 */	cmplwi r3, 0
/* 80381D80 0037ECE0  41 82 00 08 */	beq lbl_80381D88
/* 80381D84 0037ECE4  60 00 02 00 */	ori r0, r0, 0x200
lbl_80381D88:
/* 80381D88 0037ECE8  57 E3 05 28 */	rlwinm r3, r31, 0, 0x14, 0x14
/* 80381D8C 0037ECEC  28 03 00 00 */	cmplwi r3, 0
/* 80381D90 0037ECF0  41 82 00 08 */	beq lbl_80381D98
/* 80381D94 0037ECF4  60 00 40 00 */	ori r0, r0, 0x4000
lbl_80381D98:
/* 80381D98 0037ECF8  57 E3 07 FE */	clrlwi r3, r31, 0x1f
/* 80381D9C 0037ECFC  28 03 00 00 */	cmplwi r3, 0
/* 80381DA0 0037ED00  41 82 00 08 */	beq lbl_80381DA8
/* 80381DA4 0037ED04  60 00 01 00 */	ori r0, r0, 0x100
lbl_80381DA8:
/* 80381DA8 0037ED08  3C 60 80 00 */	lis r3, 0x800000C4@ha
/* 80381DAC 0037ED0C  80 83 00 C4 */	lwz r4, 0x800000C4@l(r3)
/* 80381DB0 0037ED10  80 63 00 C8 */	lwz r3, 0xc8(r3)
/* 80381DB4 0037ED14  7C 83 1B 78 */	or r3, r4, r3
/* 80381DB8 0037ED18  7C 04 18 78 */	andc r4, r0, r3
/* 80381DBC 0037ED1C  28 04 00 00 */	cmplwi r4, 0
/* 80381DC0 0037ED20  41 82 00 98 */	beq lbl_80381E58
/* 80381DC4 0037ED24  3C 60 80 3F */	lis r3, InterruptPrioTable@ha
/* 80381DC8 0037ED28  38 03 22 60 */	addi r0, r3, InterruptPrioTable@l
/* 80381DCC 0037ED2C  7C 03 03 78 */	mr r3, r0
/* 80381DD0 0037ED30  48 00 00 04 */	b lbl_80381DD4
lbl_80381DD4:
/* 80381DD4 0037ED34  48 00 00 04 */	b lbl_80381DD8
lbl_80381DD8:
/* 80381DD8 0037ED38  80 03 00 00 */	lwz r0, 0(r3)
/* 80381DDC 0037ED3C  7C 80 00 38 */	and r0, r4, r0
/* 80381DE0 0037ED40  28 00 00 00 */	cmplwi r0, 0
/* 80381DE4 0037ED44  41 82 00 10 */	beq lbl_80381DF4
/* 80381DE8 0037ED48  7C 00 00 34 */	cntlzw r0, r0
/* 80381DEC 0037ED4C  7C 1D 07 34 */	extsh r29, r0
/* 80381DF0 0037ED50  48 00 00 0C */	b lbl_80381DFC
lbl_80381DF4:
/* 80381DF4 0037ED54  38 63 00 04 */	addi r3, r3, 4
/* 80381DF8 0037ED58  4B FF FF E0 */	b lbl_80381DD8
lbl_80381DFC:
/* 80381DFC 0037ED5C  80 6D AD 10 */	lwz r3, InterruptHandlerTable@sda21(r13)
/* 80381E00 0037ED60  57 A0 10 3A */	slwi r0, r29, 2
/* 80381E04 0037ED64  7F E3 00 2E */	lwzx r31, r3, r0
/* 80381E08 0037ED68  28 1F 00 00 */	cmplwi r31, 0
/* 80381E0C 0037ED6C  41 82 00 4C */	beq lbl_80381E58
/* 80381E10 0037ED70  2C 1D 00 04 */	cmpwi r29, 4
/* 80381E14 0037ED74  40 81 00 1C */	ble lbl_80381E30
/* 80381E18 0037ED78  B3 AD AD 18 */	sth r29, __OSLastInterrupt@sda21(r13)
/* 80381E1C 0037ED7C  48 00 35 8D */	bl OSGetTime
/* 80381E20 0037ED80  90 8D AD 24 */	stw r4, __OSLastInterruptTime+4@sda21(r13)
/* 80381E24 0037ED84  90 6D AD 20 */	stw r3, __OSLastInterruptTime@sda21(r13)
/* 80381E28 0037ED88  80 1E 01 98 */	lwz r0, 0x198(r30)
/* 80381E2C 0037ED8C  90 0D AD 14 */	stw r0, __OSLastInterruptSrr0@sda21(r13)
lbl_80381E30:
/* 80381E30 0037ED90  48 00 27 B1 */	bl OSDisableScheduler
/* 80381E34 0037ED94  7F A3 EB 78 */	mr r3, r29
/* 80381E38 0037ED98  7F C4 F3 78 */	mr r4, r30
/* 80381E3C 0037ED9C  7F EC FB 78 */	mr r12, r31
/* 80381E40 0037EDA0  7D 88 03 A6 */	mtlr r12
/* 80381E44 0037EDA4  4E 80 00 21 */	blrl
/* 80381E48 0037EDA8  48 00 27 D9 */	bl OSEnableScheduler
/* 80381E4C 0037EDAC  48 00 2C A1 */	bl __OSReschedule
/* 80381E50 0037EDB0  7F C3 F3 78 */	mr r3, r30
/* 80381E54 0037EDB4  4B FF D6 B1 */	bl OSLoadContext
lbl_80381E58:
/* 80381E58 0037EDB8  7F C3 F3 78 */	mr r3, r30
/* 80381E5C 0037EDBC  4B FF D6 A9 */	bl OSLoadContext
/* 80381E60 0037EDC0  80 01 00 2C */	lwz r0, 0x2c(r1)
/* 80381E64 0037EDC4  83 E1 00 24 */	lwz r31, 0x24(r1)
/* 80381E68 0037EDC8  83 C1 00 20 */	lwz r30, 0x20(r1)
/* 80381E6C 0037EDCC  83 A1 00 1C */	lwz r29, 0x1c(r1)
/* 80381E70 0037EDD0  38 21 00 28 */	addi r1, r1, 0x28
/* 80381E74 0037EDD4  7C 08 03 A6 */	mtlr r0
/* 80381E78 0037EDD8  4E 80 00 20 */	blr

.global ExternalInterruptHandler
ExternalInterruptHandler:
/* 80381E7C 0037EDDC  90 04 00 00 */	stw r0, 0(r4)
/* 80381E80 0037EDE0  90 24 00 04 */	stw r1, 4(r4)
/* 80381E84 0037EDE4  90 44 00 08 */	stw r2, 8(r4)
/* 80381E88 0037EDE8  BC C4 00 18 */	stmw r6, 0x18(r4)
/* 80381E8C 0037EDEC  7C 11 E2 A6 */	mfspr r0, 0x391
/* 80381E90 0037EDF0  90 04 01 A8 */	stw r0, 0x1a8(r4)
/* 80381E94 0037EDF4  7C 12 E2 A6 */	mfspr r0, 0x392
/* 80381E98 0037EDF8  90 04 01 AC */	stw r0, 0x1ac(r4)
/* 80381E9C 0037EDFC  7C 13 E2 A6 */	mfspr r0, 0x393
/* 80381EA0 0037EE00  90 04 01 B0 */	stw r0, 0x1b0(r4)
/* 80381EA4 0037EE04  7C 14 E2 A6 */	mfspr r0, 0x394
/* 80381EA8 0037EE08  90 04 01 B4 */	stw r0, 0x1b4(r4)
/* 80381EAC 0037EE0C  7C 15 E2 A6 */	mfspr r0, 0x395
/* 80381EB0 0037EE10  90 04 01 B8 */	stw r0, 0x1b8(r4)
/* 80381EB4 0037EE14  7C 16 E2 A6 */	mfspr r0, 0x396
/* 80381EB8 0037EE18  90 04 01 BC */	stw r0, 0x1bc(r4)
/* 80381EBC 0037EE1C  7C 17 E2 A6 */	mfspr r0, 0x397
/* 80381EC0 0037EE20  90 04 01 C0 */	stw r0, 0x1c0(r4)
/* 80381EC4 0037EE24  94 21 FF F8 */	stwu r1, -8(r1)
/* 80381EC8 0037EE28  4B FF FC 70 */	b __OSDispatchInterrupt
