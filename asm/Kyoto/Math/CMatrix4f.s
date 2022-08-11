.include "macros.inc"

.comm lbl_805A6620, 0x40, 4

.section .ctors, "wa"
lbl_ctor:
.4byte __sinit_CMatrix4f_cpp

.section .text, "ax"

.global MultiplyGetW__9CMatrix4fCFRC9CVector3f
MultiplyGetW__9CMatrix4fCFRC9CVector3f:
/* 80310720 0030D680  C0 24 00 04 */	lfs f1, 4(r4)
/* 80310724 0030D684  C0 03 00 34 */	lfs f0, 0x34(r3)
/* 80310728 0030D688  C0 44 00 00 */	lfs f2, 0(r4)
/* 8031072C 0030D68C  EC 01 00 32 */	fmuls f0, f1, f0
/* 80310730 0030D690  C0 23 00 30 */	lfs f1, 0x30(r3)
/* 80310734 0030D694  C0 84 00 08 */	lfs f4, 8(r4)
/* 80310738 0030D698  C0 63 00 38 */	lfs f3, 0x38(r3)
/* 8031073C 0030D69C  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 80310740 0030D6A0  C0 23 00 3C */	lfs f1, 0x3c(r3)
/* 80310744 0030D6A4  EC 04 00 FA */	fmadds f0, f4, f3, f0
/* 80310748 0030D6A8  EC 21 00 2A */	fadds f1, f1, f0
/* 8031074C 0030D6AC  4E 80 00 20 */	blr

.global MultiplyOneOverW__9CMatrix4fCFRC9CVector3f
MultiplyOneOverW__9CMatrix4fCFRC9CVector3f:
/* 80310750 0030D6B0  C0 E5 00 04 */	lfs f7, 4(r5)
/* 80310754 0030D6B4  C0 04 00 34 */	lfs f0, 0x34(r4)
/* 80310758 0030D6B8  C0 44 00 04 */	lfs f2, 4(r4)
/* 8031075C 0030D6BC  EC 07 00 32 */	fmuls f0, f7, f0
/* 80310760 0030D6C0  C1 65 00 00 */	lfs f11, 0(r5)
/* 80310764 0030D6C4  C0 24 00 30 */	lfs f1, 0x30(r4)
/* 80310768 0030D6C8  EC A7 00 B2 */	fmuls f5, f7, f2
/* 8031076C 0030D6CC  C0 44 00 14 */	lfs f2, 0x14(r4)
/* 80310770 0030D6D0  EC 0B 00 7A */	fmadds f0, f11, f1, f0
/* 80310774 0030D6D4  C1 85 00 08 */	lfs f12, 8(r5)
/* 80310778 0030D6D8  EC 87 00 B2 */	fmuls f4, f7, f2
/* 8031077C 0030D6DC  C0 24 00 38 */	lfs f1, 0x38(r4)
/* 80310780 0030D6E0  C0 44 00 3C */	lfs f2, 0x3c(r4)
/* 80310784 0030D6E4  EC 0C 00 7A */	fmadds f0, f12, f1, f0
/* 80310788 0030D6E8  C0 C4 00 00 */	lfs f6, 0(r4)
/* 8031078C 0030D6EC  C0 64 00 24 */	lfs f3, 0x24(r4)
/* 80310790 0030D6F0  C0 22 C8 58 */	lfs f1, lbl_805AE578@sda21(r2)
/* 80310794 0030D6F4  ED 0B 29 BA */	fmadds f8, f11, f6, f5
/* 80310798 0030D6F8  EC 02 00 2A */	fadds f0, f2, f0
/* 8031079C 0030D6FC  C0 A4 00 10 */	lfs f5, 0x10(r4)
/* 803107A0 0030D700  EC 47 00 F2 */	fmuls f2, f7, f3
/* 803107A4 0030D704  C1 24 00 08 */	lfs f9, 8(r4)
/* 803107A8 0030D708  ED A1 00 24 */	fdivs f13, f1, f0
/* 803107AC 0030D70C  C0 04 00 20 */	lfs f0, 0x20(r4)
/* 803107B0 0030D710  C0 C4 00 18 */	lfs f6, 0x18(r4)
/* 803107B4 0030D714  C1 44 00 0C */	lfs f10, 0xc(r4)
/* 803107B8 0030D718  C0 24 00 28 */	lfs f1, 0x28(r4)
/* 803107BC 0030D71C  C0 E4 00 1C */	lfs f7, 0x1c(r4)
/* 803107C0 0030D720  EC 8B 21 7A */	fmadds f4, f11, f5, f4
/* 803107C4 0030D724  C0 64 00 2C */	lfs f3, 0x2c(r4)
/* 803107C8 0030D728  EC AC 42 7A */	fmadds f5, f12, f9, f8
/* 803107CC 0030D72C  EC 0B 10 3A */	fmadds f0, f11, f0, f2
/* 803107D0 0030D730  EC 4C 21 BA */	fmadds f2, f12, f6, f4
/* 803107D4 0030D734  EC 8A 28 2A */	fadds f4, f10, f5
/* 803107D8 0030D738  EC 0C 00 7A */	fmadds f0, f12, f1, f0
/* 803107DC 0030D73C  EC 27 10 2A */	fadds f1, f7, f2
/* 803107E0 0030D740  EC 84 03 72 */	fmuls f4, f4, f13
/* 803107E4 0030D744  EC 03 00 2A */	fadds f0, f3, f0
/* 803107E8 0030D748  EC 21 03 72 */	fmuls f1, f1, f13
/* 803107EC 0030D74C  D0 83 00 00 */	stfs f4, 0(r3)
/* 803107F0 0030D750  EC 00 03 72 */	fmuls f0, f0, f13
/* 803107F4 0030D754  D0 23 00 04 */	stfs f1, 4(r3)
/* 803107F8 0030D758  D0 03 00 08 */	stfs f0, 8(r3)
/* 803107FC 0030D75C  4E 80 00 20 */	blr

.global __ml__9CMatrix4fCFRC9CVector3f
__ml__9CMatrix4fCFRC9CVector3f:
/* 80310800 0030D760  C0 65 00 04 */	lfs f3, 4(r5)
/* 80310804 0030D764  C0 04 00 04 */	lfs f0, 4(r4)
/* 80310808 0030D768  C0 44 00 14 */	lfs f2, 0x14(r4)
/* 8031080C 0030D76C  EC 03 00 32 */	fmuls f0, f3, f0
/* 80310810 0030D770  C0 C5 00 00 */	lfs f6, 0(r5)
/* 80310814 0030D774  C0 24 00 00 */	lfs f1, 0(r4)
/* 80310818 0030D778  EC 83 00 B2 */	fmuls f4, f3, f2
/* 8031081C 0030D77C  C0 44 00 24 */	lfs f2, 0x24(r4)
/* 80310820 0030D780  C0 A4 00 10 */	lfs f5, 0x10(r4)
/* 80310824 0030D784  EC 06 00 7A */	fmadds f0, f6, f1, f0
/* 80310828 0030D788  C0 E5 00 08 */	lfs f7, 8(r5)
/* 8031082C 0030D78C  C0 24 00 08 */	lfs f1, 8(r4)
/* 80310830 0030D790  EC 43 00 B2 */	fmuls f2, f3, f2
/* 80310834 0030D794  C0 64 00 20 */	lfs f3, 0x20(r4)
/* 80310838 0030D798  EC 86 21 7A */	fmadds f4, f6, f5, f4
/* 8031083C 0030D79C  C0 A4 00 18 */	lfs f5, 0x18(r4)
/* 80310840 0030D7A0  EC 07 00 7A */	fmadds f0, f7, f1, f0
/* 80310844 0030D7A4  C0 24 00 0C */	lfs f1, 0xc(r4)
/* 80310848 0030D7A8  EC 46 10 FA */	fmadds f2, f6, f3, f2
/* 8031084C 0030D7AC  C0 64 00 28 */	lfs f3, 0x28(r4)
/* 80310850 0030D7B0  EC A7 21 7A */	fmadds f5, f7, f5, f4
/* 80310854 0030D7B4  C0 C4 00 1C */	lfs f6, 0x1c(r4)
/* 80310858 0030D7B8  EC 01 00 2A */	fadds f0, f1, f0
/* 8031085C 0030D7BC  C0 84 00 2C */	lfs f4, 0x2c(r4)
/* 80310860 0030D7C0  EC 27 10 FA */	fmadds f1, f7, f3, f2
/* 80310864 0030D7C4  EC 46 28 2A */	fadds f2, f6, f5
/* 80310868 0030D7C8  D0 03 00 00 */	stfs f0, 0(r3)
/* 8031086C 0030D7CC  EC 04 08 2A */	fadds f0, f4, f1
/* 80310870 0030D7D0  D0 43 00 04 */	stfs f2, 4(r3)
/* 80310874 0030D7D4  D0 03 00 08 */	stfs f0, 8(r3)
/* 80310878 0030D7D8  4E 80 00 20 */	blr

.global __ct__9CMatrix4fFffffffffffffffff
__ct__9CMatrix4fFffffffffffffffff:
/* 8031087C 0030D7DC  D0 23 00 00 */	stfs f1, 0(r3)
/* 80310880 0030D7E0  C1 61 00 08 */	lfs f11, 8(r1)
/* 80310884 0030D7E4  D0 43 00 04 */	stfs f2, 4(r3)
/* 80310888 0030D7E8  C1 41 00 0C */	lfs f10, 0xc(r1)
/* 8031088C 0030D7EC  D0 63 00 08 */	stfs f3, 8(r3)
/* 80310890 0030D7F0  C1 21 00 10 */	lfs f9, 0x10(r1)
/* 80310894 0030D7F4  D0 83 00 0C */	stfs f4, 0xc(r3)
/* 80310898 0030D7F8  C0 81 00 14 */	lfs f4, 0x14(r1)
/* 8031089C 0030D7FC  D0 A3 00 10 */	stfs f5, 0x10(r3)
/* 803108A0 0030D800  C0 61 00 18 */	lfs f3, 0x18(r1)
/* 803108A4 0030D804  D0 C3 00 14 */	stfs f6, 0x14(r3)
/* 803108A8 0030D808  C0 41 00 1C */	lfs f2, 0x1c(r1)
/* 803108AC 0030D80C  D0 E3 00 18 */	stfs f7, 0x18(r3)
/* 803108B0 0030D810  C0 21 00 20 */	lfs f1, 0x20(r1)
/* 803108B4 0030D814  D1 03 00 1C */	stfs f8, 0x1c(r3)
/* 803108B8 0030D818  C0 01 00 24 */	lfs f0, 0x24(r1)
/* 803108BC 0030D81C  D1 63 00 20 */	stfs f11, 0x20(r3)
/* 803108C0 0030D820  D1 43 00 24 */	stfs f10, 0x24(r3)
/* 803108C4 0030D824  D1 23 00 28 */	stfs f9, 0x28(r3)
/* 803108C8 0030D828  D0 83 00 2C */	stfs f4, 0x2c(r3)
/* 803108CC 0030D82C  D0 63 00 30 */	stfs f3, 0x30(r3)
/* 803108D0 0030D830  D0 43 00 34 */	stfs f2, 0x34(r3)
/* 803108D4 0030D834  D0 23 00 38 */	stfs f1, 0x38(r3)
/* 803108D8 0030D838  D0 03 00 3C */	stfs f0, 0x3c(r3)
/* 803108DC 0030D83C  4E 80 00 20 */	blr

.global __sinit_CMatrix4f_cpp
__sinit_CMatrix4f_cpp:
/* 803108E0 0030D840  94 21 FF 70 */	stwu r1, -0x90(r1)
/* 803108E4 0030D844  7C 08 02 A6 */	mflr r0
/* 803108E8 0030D848  90 01 00 94 */	stw r0, 0x94(r1)
/* 803108EC 0030D84C  DB E1 00 80 */	stfd f31, 0x80(r1)
/* 803108F0 0030D850  F3 E1 00 88 */	psq_st f31, 136(r1), 0, qr0
/* 803108F4 0030D854  DB C1 00 70 */	stfd f30, 0x70(r1)
/* 803108F8 0030D858  F3 C1 00 78 */	psq_st f30, 120(r1), 0, qr0
/* 803108FC 0030D85C  C0 42 C8 5C */	lfs f2, lbl_805AE57C@sda21(r2)
/* 80310900 0030D860  38 61 00 28 */	addi r3, r1, 0x28
/* 80310904 0030D864  C0 22 C8 58 */	lfs f1, lbl_805AE578@sda21(r2)
/* 80310908 0030D868  D0 41 00 08 */	stfs f2, 8(r1)
/* 8031090C 0030D86C  FC 60 10 90 */	fmr f3, f2
/* 80310910 0030D870  FC 80 10 90 */	fmr f4, f2
/* 80310914 0030D874  D0 41 00 0C */	stfs f2, 0xc(r1)
/* 80310918 0030D878  FC A0 10 90 */	fmr f5, f2
/* 8031091C 0030D87C  FC C0 08 90 */	fmr f6, f1
/* 80310920 0030D880  D0 21 00 10 */	stfs f1, 0x10(r1)
/* 80310924 0030D884  FC E0 10 90 */	fmr f7, f2
/* 80310928 0030D888  FD 00 10 90 */	fmr f8, f2
/* 8031092C 0030D88C  D0 41 00 14 */	stfs f2, 0x14(r1)
/* 80310930 0030D890  D0 41 00 18 */	stfs f2, 0x18(r1)
/* 80310934 0030D894  D0 41 00 1C */	stfs f2, 0x1c(r1)
/* 80310938 0030D898  D0 41 00 20 */	stfs f2, 0x20(r1)
/* 8031093C 0030D89C  D0 21 00 24 */	stfs f1, 0x24(r1)
/* 80310940 0030D8A0  4B FF FF 3D */	bl __ct__9CMatrix4fFffffffffffffffff
/* 80310944 0030D8A4  C3 C1 00 28 */	lfs f30, 0x28(r1)
/* 80310948 0030D8A8  3C 60 80 5A */	lis r3, lbl_805A6620@ha
/* 8031094C 0030D8AC  C3 E1 00 2C */	lfs f31, 0x2c(r1)
/* 80310950 0030D8B0  D7 C3 66 20 */	stfsu f30, lbl_805A6620@l(r3)
/* 80310954 0030D8B4  C1 A1 00 30 */	lfs f13, 0x30(r1)
/* 80310958 0030D8B8  C1 81 00 34 */	lfs f12, 0x34(r1)
/* 8031095C 0030D8BC  C1 61 00 38 */	lfs f11, 0x38(r1)
/* 80310960 0030D8C0  C1 41 00 3C */	lfs f10, 0x3c(r1)
/* 80310964 0030D8C4  C1 21 00 40 */	lfs f9, 0x40(r1)
/* 80310968 0030D8C8  C1 01 00 44 */	lfs f8, 0x44(r1)
/* 8031096C 0030D8CC  C0 E1 00 48 */	lfs f7, 0x48(r1)
/* 80310970 0030D8D0  C0 C1 00 4C */	lfs f6, 0x4c(r1)
/* 80310974 0030D8D4  C0 A1 00 50 */	lfs f5, 0x50(r1)
/* 80310978 0030D8D8  C0 81 00 54 */	lfs f4, 0x54(r1)
/* 8031097C 0030D8DC  C0 61 00 58 */	lfs f3, 0x58(r1)
/* 80310980 0030D8E0  C0 41 00 5C */	lfs f2, 0x5c(r1)
/* 80310984 0030D8E4  C0 21 00 60 */	lfs f1, 0x60(r1)
/* 80310988 0030D8E8  C0 01 00 64 */	lfs f0, 0x64(r1)
/* 8031098C 0030D8EC  D3 E3 00 04 */	stfs f31, 4(r3)
/* 80310990 0030D8F0  D1 A3 00 08 */	stfs f13, 8(r3)
/* 80310994 0030D8F4  D1 83 00 0C */	stfs f12, 0xc(r3)
/* 80310998 0030D8F8  D1 63 00 10 */	stfs f11, 0x10(r3)
/* 8031099C 0030D8FC  D1 43 00 14 */	stfs f10, 0x14(r3)
/* 803109A0 0030D900  D1 23 00 18 */	stfs f9, 0x18(r3)
/* 803109A4 0030D904  D1 03 00 1C */	stfs f8, 0x1c(r3)
/* 803109A8 0030D908  D0 E3 00 20 */	stfs f7, 0x20(r3)
/* 803109AC 0030D90C  D0 C3 00 24 */	stfs f6, 0x24(r3)
/* 803109B0 0030D910  D0 A3 00 28 */	stfs f5, 0x28(r3)
/* 803109B4 0030D914  D0 83 00 2C */	stfs f4, 0x2c(r3)
/* 803109B8 0030D918  D0 63 00 30 */	stfs f3, 0x30(r3)
/* 803109BC 0030D91C  D0 43 00 34 */	stfs f2, 0x34(r3)
/* 803109C0 0030D920  D0 23 00 38 */	stfs f1, 0x38(r3)
/* 803109C4 0030D924  D0 03 00 3C */	stfs f0, 0x3c(r3)
/* 803109C8 0030D928  E3 E1 00 88 */	psq_l f31, 136(r1), 0, qr0
/* 803109CC 0030D92C  CB E1 00 80 */	lfd f31, 0x80(r1)
/* 803109D0 0030D930  E3 C1 00 78 */	psq_l f30, 120(r1), 0, qr0
/* 803109D4 0030D934  80 01 00 94 */	lwz r0, 0x94(r1)
/* 803109D8 0030D938  CB C1 00 70 */	lfd f30, 0x70(r1)
/* 803109DC 0030D93C  7C 08 03 A6 */	mtlr r0
/* 803109E0 0030D940  38 21 00 90 */	addi r1, r1, 0x90
/* 803109E4 0030D944  4E 80 00 20 */	blr

.section .sdata2, "a"
.balign 8
.global lbl_805AE578
lbl_805AE578:
	# ROM: 0x3FAE18
	.float 1.0

.global lbl_805AE57C
lbl_805AE57C:
	# ROM: 0x3FAE1C
	.4byte 0

