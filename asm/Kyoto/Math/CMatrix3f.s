.include "macros.inc"

.comm lbl_805A65FC, 0x24, 4

.section .ctors, "wa"
lbl_ctor:
.4byte __sinit_CMatrix3f_cpp

.section .text, "ax"

.global __as__9CMatrix3fFRC9CMatrix3f
__as__9CMatrix3fFRC9CMatrix3f:
/* 8030FFF8 0030CF58  C8 04 00 00 */	lfd f0, 0(r4)
/* 8030FFFC 0030CF5C  C8 24 00 08 */	lfd f1, 8(r4)
/* 80310000 0030CF60  C8 44 00 10 */	lfd f2, 0x10(r4)
/* 80310004 0030CF64  D8 03 00 00 */	stfd f0, 0(r3)
/* 80310008 0030CF68  D8 23 00 08 */	stfd f1, 8(r3)
/* 8031000C 0030CF6C  D8 43 00 10 */	stfd f2, 0x10(r3)
/* 80310010 0030CF70  C8 04 00 18 */	lfd f0, 0x18(r4)
/* 80310014 0030CF74  C0 24 00 20 */	lfs f1, 0x20(r4)
/* 80310018 0030CF78  D8 03 00 18 */	stfd f0, 0x18(r3)
/* 8031001C 0030CF7C  D0 23 00 20 */	stfs f1, 0x20(r3)
/* 80310020 0030CF80  4E 80 00 20 */	blr

.global __ct__9CMatrix3fFRC9CMatrix3f
__ct__9CMatrix3fFRC9CMatrix3f:
/* 80310024 0030CF84  C8 04 00 00 */	lfd f0, 0(r4)
/* 80310028 0030CF88  C8 24 00 08 */	lfd f1, 8(r4)
/* 8031002C 0030CF8C  C8 44 00 10 */	lfd f2, 0x10(r4)
/* 80310030 0030CF90  D8 03 00 00 */	stfd f0, 0(r3)
/* 80310034 0030CF94  D8 23 00 08 */	stfd f1, 8(r3)
/* 80310038 0030CF98  D8 43 00 10 */	stfd f2, 0x10(r3)
/* 8031003C 0030CF9C  C8 04 00 18 */	lfd f0, 0x18(r4)
/* 80310040 0030CFA0  C0 24 00 20 */	lfs f1, 0x20(r4)
/* 80310044 0030CFA4  D8 03 00 18 */	stfd f0, 0x18(r3)
/* 80310048 0030CFA8  D0 23 00 20 */	stfs f1, 0x20(r3)
/* 8031004C 0030CFAC  4E 80 00 20 */	blr

.global AddScaledMatrix__9CMatrix3fFRC9CMatrix3ff
AddScaledMatrix__9CMatrix3fFRC9CMatrix3ff:
/* 80310050 0030CFB0  C0 44 00 00 */	lfs f2, 0(r4)
/* 80310054 0030CFB4  C0 03 00 00 */	lfs f0, 0(r3)
/* 80310058 0030CFB8  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 8031005C 0030CFBC  D0 03 00 00 */	stfs f0, 0(r3)
/* 80310060 0030CFC0  C0 44 00 04 */	lfs f2, 4(r4)
/* 80310064 0030CFC4  C0 03 00 04 */	lfs f0, 4(r3)
/* 80310068 0030CFC8  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 8031006C 0030CFCC  D0 03 00 04 */	stfs f0, 4(r3)
/* 80310070 0030CFD0  C0 44 00 08 */	lfs f2, 8(r4)
/* 80310074 0030CFD4  C0 03 00 08 */	lfs f0, 8(r3)
/* 80310078 0030CFD8  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 8031007C 0030CFDC  D0 03 00 08 */	stfs f0, 8(r3)
/* 80310080 0030CFE0  C0 44 00 0C */	lfs f2, 0xc(r4)
/* 80310084 0030CFE4  C0 03 00 0C */	lfs f0, 0xc(r3)
/* 80310088 0030CFE8  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 8031008C 0030CFEC  D0 03 00 0C */	stfs f0, 0xc(r3)
/* 80310090 0030CFF0  C0 44 00 10 */	lfs f2, 0x10(r4)
/* 80310094 0030CFF4  C0 03 00 10 */	lfs f0, 0x10(r3)
/* 80310098 0030CFF8  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 8031009C 0030CFFC  D0 03 00 10 */	stfs f0, 0x10(r3)
/* 803100A0 0030D000  C0 44 00 14 */	lfs f2, 0x14(r4)
/* 803100A4 0030D004  C0 03 00 14 */	lfs f0, 0x14(r3)
/* 803100A8 0030D008  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 803100AC 0030D00C  D0 03 00 14 */	stfs f0, 0x14(r3)
/* 803100B0 0030D010  C0 44 00 18 */	lfs f2, 0x18(r4)
/* 803100B4 0030D014  C0 03 00 18 */	lfs f0, 0x18(r3)
/* 803100B8 0030D018  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 803100BC 0030D01C  D0 03 00 18 */	stfs f0, 0x18(r3)
/* 803100C0 0030D020  C0 44 00 1C */	lfs f2, 0x1c(r4)
/* 803100C4 0030D024  C0 03 00 1C */	lfs f0, 0x1c(r3)
/* 803100C8 0030D028  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 803100CC 0030D02C  D0 03 00 1C */	stfs f0, 0x1c(r3)
/* 803100D0 0030D030  C0 44 00 20 */	lfs f2, 0x20(r4)
/* 803100D4 0030D034  C0 03 00 20 */	lfs f0, 0x20(r3)
/* 803100D8 0030D038  EC 02 00 7A */	fmadds f0, f2, f1, f0
/* 803100DC 0030D03C  D0 03 00 20 */	stfs f0, 0x20(r3)
/* 803100E0 0030D040  4E 80 00 20 */	blr

.global Determinant__9CMatrix3fCFv
Determinant__9CMatrix3fCFv:
/* 803100E4 0030D044  C1 23 00 0C */	lfs f9, 0xc(r3)
/* 803100E8 0030D048  C0 C3 00 20 */	lfs f6, 0x20(r3)
/* 803100EC 0030D04C  C0 83 00 18 */	lfs f4, 0x18(r3)
/* 803100F0 0030D050  EC 09 01 B2 */	fmuls f0, f9, f6
/* 803100F4 0030D054  C0 63 00 14 */	lfs f3, 0x14(r3)
/* 803100F8 0030D058  C1 03 00 1C */	lfs f8, 0x1c(r3)
/* 803100FC 0030D05C  C0 E3 00 10 */	lfs f7, 0x10(r3)
/* 80310100 0030D060  EC 43 02 32 */	fmuls f2, f3, f8
/* 80310104 0030D064  C0 23 00 04 */	lfs f1, 4(r3)
/* 80310108 0030D068  EC 03 01 38 */	fmsubs f0, f3, f4, f0
/* 8031010C 0030D06C  C0 63 00 00 */	lfs f3, 0(r3)
/* 80310110 0030D070  EC 87 01 32 */	fmuls f4, f7, f4
/* 80310114 0030D074  C0 A3 00 08 */	lfs f5, 8(r3)
/* 80310118 0030D078  EC 47 11 B8 */	fmsubs f2, f7, f6, f2
/* 8031011C 0030D07C  EC 01 00 32 */	fmuls f0, f1, f0
/* 80310120 0030D080  EC 29 22 38 */	fmsubs f1, f9, f8, f4
/* 80310124 0030D084  EC 03 00 BA */	fmadds f0, f3, f2, f0
/* 80310128 0030D088  EC 25 00 7A */	fmadds f1, f5, f1, f0
/* 8031012C 0030D08C  4E 80 00 20 */	blr

.global __ml__9CMatrix3fCFRC9CMatrix3f
__ml__9CMatrix3fCFRC9CMatrix3f:
/* 80310130 0030D090  94 21 FF 70 */	stwu r1, -0x90(r1)
/* 80310134 0030D094  DB E1 00 80 */	stfd f31, 0x80(r1)
/* 80310138 0030D098  F3 E1 00 88 */	psq_st f31, 136(r1), 0, qr0
/* 8031013C 0030D09C  DB C1 00 70 */	stfd f30, 0x70(r1)
/* 80310140 0030D0A0  F3 C1 00 78 */	psq_st f30, 120(r1), 0, qr0
/* 80310144 0030D0A4  DB A1 00 60 */	stfd f29, 0x60(r1)
/* 80310148 0030D0A8  F3 A1 00 68 */	psq_st f29, 104(r1), 0, qr0
/* 8031014C 0030D0AC  DB 81 00 50 */	stfd f28, 0x50(r1)
/* 80310150 0030D0B0  F3 81 00 58 */	psq_st f28, 88(r1), 0, qr0
/* 80310154 0030D0B4  DB 61 00 40 */	stfd f27, 0x40(r1)
/* 80310158 0030D0B8  F3 61 00 48 */	psq_st f27, 72(r1), 0, qr0
/* 8031015C 0030D0BC  DB 41 00 30 */	stfd f26, 0x30(r1)
/* 80310160 0030D0C0  F3 41 00 38 */	psq_st f26, 56(r1), 0, qr0
/* 80310164 0030D0C4  DB 21 00 20 */	stfd f25, 0x20(r1)
/* 80310168 0030D0C8  F3 21 00 28 */	psq_st f25, 40(r1), 0, qr0
/* 8031016C 0030D0CC  DB 01 00 10 */	stfd f24, 0x10(r1)
/* 80310170 0030D0D0  F3 01 00 18 */	psq_st f24, 24(r1), 0, qr0
/* 80310174 0030D0D4  C3 05 00 0C */	lfs f24, 0xc(r5)
/* 80310178 0030D0D8  C0 C4 00 04 */	lfs f6, 4(r4)
/* 8031017C 0030D0DC  C0 85 00 10 */	lfs f4, 0x10(r5)
/* 80310180 0030D0E0  EC 66 06 32 */	fmuls f3, f6, f24
/* 80310184 0030D0E4  C0 25 00 00 */	lfs f1, 0(r5)
/* 80310188 0030D0E8  C3 44 00 00 */	lfs f26, 0(r4)
/* 8031018C 0030D0EC  EC 06 01 32 */	fmuls f0, f6, f4
/* 80310190 0030D0F0  C0 A5 00 14 */	lfs f5, 0x14(r5)
/* 80310194 0030D0F4  C1 04 00 10 */	lfs f8, 0x10(r4)
/* 80310198 0030D0F8  C0 45 00 04 */	lfs f2, 4(r5)
/* 8031019C 0030D0FC  ED 7A 18 7A */	fmadds f11, f26, f1, f3
/* 803101A0 0030D100  ED A6 01 72 */	fmuls f13, f6, f5
/* 803101A4 0030D104  C0 65 00 08 */	lfs f3, 8(r5)
/* 803101A8 0030D108  C3 A4 00 1C */	lfs f29, 0x1c(r4)
/* 803101AC 0030D10C  ED 9A 00 BA */	fmadds f12, f26, f2, f0
/* 803101B0 0030D110  C0 C5 00 18 */	lfs f6, 0x18(r5)
/* 803101B4 0030D114  C3 24 00 08 */	lfs f25, 8(r4)
/* 803101B8 0030D118  C0 E5 00 1C */	lfs f7, 0x1c(r5)
/* 803101BC 0030D11C  ED 48 06 32 */	fmuls f10, f8, f24
/* 803101C0 0030D120  C3 64 00 0C */	lfs f27, 0xc(r4)
/* 803101C4 0030D124  ED 28 01 32 */	fmuls f9, f8, f4
/* 803101C8 0030D128  EF C8 01 72 */	fmuls f30, f8, f5
/* 803101CC 0030D12C  C1 05 00 20 */	lfs f8, 0x20(r5)
/* 803101D0 0030D130  EF FA 68 FA */	fmadds f31, f26, f3, f13
/* 803101D4 0030D134  ED B9 59 BA */	fmadds f13, f25, f6, f11
/* 803101D8 0030D138  C0 04 00 20 */	lfs f0, 0x20(r4)
/* 803101DC 0030D13C  C3 84 00 18 */	lfs f28, 0x18(r4)
/* 803101E0 0030D140  C3 44 00 14 */	lfs f26, 0x14(r4)
/* 803101E4 0030D144  ED 7B 50 7A */	fmadds f11, f27, f1, f10
/* 803101E8 0030D148  ED 5B 48 BA */	fmadds f10, f27, f2, f9
/* 803101EC 0030D14C  ED 3D 06 32 */	fmuls f9, f29, f24
/* 803101F0 0030D150  D1 A3 00 00 */	stfs f13, 0(r3)
/* 803101F4 0030D154  ED 99 61 FA */	fmadds f12, f25, f7, f12
/* 803101F8 0030D158  ED BB F0 FA */	fmadds f13, f27, f3, f30
/* 803101FC 0030D15C  EC 9D 01 32 */	fmuls f4, f29, f4
/* 80310200 0030D160  ED 3C 48 7A */	fmadds f9, f28, f1, f9
/* 80310204 0030D164  D1 83 00 04 */	stfs f12, 4(r3)
/* 80310208 0030D168  ED 99 FA 3A */	fmadds f12, f25, f8, f31
/* 8031020C 0030D16C  EC BD 01 72 */	fmuls f5, f29, f5
/* 80310210 0030D170  EC 3C 20 BA */	fmadds f1, f28, f2, f4
/* 80310214 0030D174  EC 5A 59 BA */	fmadds f2, f26, f6, f11
/* 80310218 0030D178  D1 83 00 08 */	stfs f12, 8(r3)
/* 8031021C 0030D17C  EC 9C 28 FA */	fmadds f4, f28, f3, f5
/* 80310220 0030D180  EC 7A 51 FA */	fmadds f3, f26, f7, f10
/* 80310224 0030D184  D0 43 00 0C */	stfs f2, 0xc(r3)
/* 80310228 0030D188  EC BA 6A 3A */	fmadds f5, f26, f8, f13
/* 8031022C 0030D18C  EC 40 49 BA */	fmadds f2, f0, f6, f9
/* 80310230 0030D190  D0 63 00 10 */	stfs f3, 0x10(r3)
/* 80310234 0030D194  EC 20 09 FA */	fmadds f1, f0, f7, f1
/* 80310238 0030D198  EC 00 22 3A */	fmadds f0, f0, f8, f4
/* 8031023C 0030D19C  D0 A3 00 14 */	stfs f5, 0x14(r3)
/* 80310240 0030D1A0  D0 43 00 18 */	stfs f2, 0x18(r3)
/* 80310244 0030D1A4  D0 23 00 1C */	stfs f1, 0x1c(r3)
/* 80310248 0030D1A8  D0 03 00 20 */	stfs f0, 0x20(r3)
/* 8031024C 0030D1AC  E3 E1 00 88 */	psq_l f31, 136(r1), 0, qr0
/* 80310250 0030D1B0  CB E1 00 80 */	lfd f31, 0x80(r1)
/* 80310254 0030D1B4  E3 C1 00 78 */	psq_l f30, 120(r1), 0, qr0
/* 80310258 0030D1B8  CB C1 00 70 */	lfd f30, 0x70(r1)
/* 8031025C 0030D1BC  E3 A1 00 68 */	psq_l f29, 104(r1), 0, qr0
/* 80310260 0030D1C0  CB A1 00 60 */	lfd f29, 0x60(r1)
/* 80310264 0030D1C4  E3 81 00 58 */	psq_l f28, 88(r1), 0, qr0
/* 80310268 0030D1C8  CB 81 00 50 */	lfd f28, 0x50(r1)
/* 8031026C 0030D1CC  E3 61 00 48 */	psq_l f27, 72(r1), 0, qr0
/* 80310270 0030D1D0  CB 61 00 40 */	lfd f27, 0x40(r1)
/* 80310274 0030D1D4  E3 41 00 38 */	psq_l f26, 56(r1), 0, qr0
/* 80310278 0030D1D8  CB 41 00 30 */	lfd f26, 0x30(r1)
/* 8031027C 0030D1DC  E3 21 00 28 */	psq_l f25, 40(r1), 0, qr0
/* 80310280 0030D1E0  CB 21 00 20 */	lfd f25, 0x20(r1)
/* 80310284 0030D1E4  E3 01 00 18 */	psq_l f24, 24(r1), 0, qr0
/* 80310288 0030D1E8  CB 01 00 10 */	lfd f24, 0x10(r1)
/* 8031028C 0030D1EC  38 21 00 90 */	addi r1, r1, 0x90
/* 80310290 0030D1F0  4E 80 00 20 */	blr

.global __ml__9CMatrix3fCFRC9CVector3f
__ml__9CMatrix3fCFRC9CVector3f:
/* 80310294 0030D1F4  C0 65 00 04 */	lfs f3, 4(r5)
/* 80310298 0030D1F8  C0 04 00 04 */	lfs f0, 4(r4)
/* 8031029C 0030D1FC  C0 44 00 10 */	lfs f2, 0x10(r4)
/* 803102A0 0030D200  EC 03 00 32 */	fmuls f0, f3, f0
/* 803102A4 0030D204  C0 E5 00 00 */	lfs f7, 0(r5)
/* 803102A8 0030D208  C0 24 00 00 */	lfs f1, 0(r4)
/* 803102AC 0030D20C  EC 83 00 B2 */	fmuls f4, f3, f2
/* 803102B0 0030D210  C0 44 00 1C */	lfs f2, 0x1c(r4)
/* 803102B4 0030D214  C0 A4 00 0C */	lfs f5, 0xc(r4)
/* 803102B8 0030D218  EC 07 00 7A */	fmadds f0, f7, f1, f0
/* 803102BC 0030D21C  C1 05 00 08 */	lfs f8, 8(r5)
/* 803102C0 0030D220  C0 24 00 08 */	lfs f1, 8(r4)
/* 803102C4 0030D224  EC 43 00 B2 */	fmuls f2, f3, f2
/* 803102C8 0030D228  C0 64 00 18 */	lfs f3, 0x18(r4)
/* 803102CC 0030D22C  EC A7 21 7A */	fmadds f5, f7, f5, f4
/* 803102D0 0030D230  C0 C4 00 14 */	lfs f6, 0x14(r4)
/* 803102D4 0030D234  EC 08 00 7A */	fmadds f0, f8, f1, f0
/* 803102D8 0030D238  C0 84 00 20 */	lfs f4, 0x20(r4)
/* 803102DC 0030D23C  EC 27 10 FA */	fmadds f1, f7, f3, f2
/* 803102E0 0030D240  EC 48 29 BA */	fmadds f2, f8, f6, f5
/* 803102E4 0030D244  D0 03 00 00 */	stfs f0, 0(r3)
/* 803102E8 0030D248  EC 08 09 3A */	fmadds f0, f8, f4, f1
/* 803102EC 0030D24C  D0 43 00 04 */	stfs f2, 4(r3)
/* 803102F0 0030D250  D0 03 00 08 */	stfs f0, 8(r3)
/* 803102F4 0030D254  4E 80 00 20 */	blr

.global Orthonormalized__9CMatrix3fCFv
Orthonormalized__9CMatrix3fCFv:
/* 803102F8 0030D258  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 803102FC 0030D25C  7C 08 02 A6 */	mflr r0
/* 80310300 0030D260  90 01 00 44 */	stw r0, 0x44(r1)
/* 80310304 0030D264  93 E1 00 3C */	stw r31, 0x3c(r1)
/* 80310308 0030D268  7C 9F 23 78 */	mr r31, r4
/* 8031030C 0030D26C  93 C1 00 38 */	stw r30, 0x38(r1)
/* 80310310 0030D270  7C 7E 1B 78 */	mr r30, r3
/* 80310314 0030D274  38 61 00 20 */	addi r3, r1, 0x20
/* 80310318 0030D278  C0 44 00 18 */	lfs f2, 0x18(r4)
/* 8031031C 0030D27C  C0 24 00 0C */	lfs f1, 0xc(r4)
/* 80310320 0030D280  C0 04 00 00 */	lfs f0, 0(r4)
/* 80310324 0030D284  D0 21 00 24 */	stfs f1, 0x24(r1)
/* 80310328 0030D288  D0 01 00 20 */	stfs f0, 0x20(r1)
/* 8031032C 0030D28C  D0 41 00 28 */	stfs f2, 0x28(r1)
/* 80310330 0030D290  48 00 45 C9 */	bl Normalize__9CVector3fFv
/* 80310334 0030D294  C0 5F 00 1C */	lfs f2, 0x1c(r31)
/* 80310338 0030D298  38 61 00 14 */	addi r3, r1, 0x14
/* 8031033C 0030D29C  C0 3F 00 10 */	lfs f1, 0x10(r31)
/* 80310340 0030D2A0  C0 1F 00 04 */	lfs f0, 4(r31)
/* 80310344 0030D2A4  D0 21 00 18 */	stfs f1, 0x18(r1)
/* 80310348 0030D2A8  D0 01 00 14 */	stfs f0, 0x14(r1)
/* 8031034C 0030D2AC  D0 41 00 1C */	stfs f2, 0x1c(r1)
/* 80310350 0030D2B0  48 00 45 A9 */	bl Normalize__9CVector3fFv
/* 80310354 0030D2B4  C0 61 00 24 */	lfs f3, 0x24(r1)
/* 80310358 0030D2B8  38 61 00 08 */	addi r3, r1, 8
/* 8031035C 0030D2BC  C0 E1 00 14 */	lfs f7, 0x14(r1)
/* 80310360 0030D2C0  C0 81 00 28 */	lfs f4, 0x28(r1)
/* 80310364 0030D2C4  C0 41 00 18 */	lfs f2, 0x18(r1)
/* 80310368 0030D2C8  EC 07 00 F2 */	fmuls f0, f7, f3
/* 8031036C 0030D2CC  C0 C1 00 20 */	lfs f6, 0x20(r1)
/* 80310370 0030D2D0  C0 A1 00 1C */	lfs f5, 0x1c(r1)
/* 80310374 0030D2D4  EC 22 01 32 */	fmuls f1, f2, f4
/* 80310378 0030D2D8  EC 46 00 B8 */	fmsubs f2, f6, f2, f0
/* 8031037C 0030D2DC  EC 05 01 B2 */	fmuls f0, f5, f6
/* 80310380 0030D2E0  EC 23 09 78 */	fmsubs f1, f3, f5, f1
/* 80310384 0030D2E4  D0 41 00 10 */	stfs f2, 0x10(r1)
/* 80310388 0030D2E8  EC 04 01 F8 */	fmsubs f0, f4, f7, f0
/* 8031038C 0030D2EC  D0 21 00 08 */	stfs f1, 8(r1)
/* 80310390 0030D2F0  D0 01 00 0C */	stfs f0, 0xc(r1)
/* 80310394 0030D2F4  48 00 45 65 */	bl Normalize__9CVector3fFv
/* 80310398 0030D2F8  C0 61 00 0C */	lfs f3, 0xc(r1)
/* 8031039C 0030D2FC  38 61 00 14 */	addi r3, r1, 0x14
/* 803103A0 0030D300  C0 E1 00 20 */	lfs f7, 0x20(r1)
/* 803103A4 0030D304  C0 81 00 10 */	lfs f4, 0x10(r1)
/* 803103A8 0030D308  C0 41 00 24 */	lfs f2, 0x24(r1)
/* 803103AC 0030D30C  EC 07 00 F2 */	fmuls f0, f7, f3
/* 803103B0 0030D310  C0 C1 00 08 */	lfs f6, 8(r1)
/* 803103B4 0030D314  C0 A1 00 28 */	lfs f5, 0x28(r1)
/* 803103B8 0030D318  EC 22 01 32 */	fmuls f1, f2, f4
/* 803103BC 0030D31C  EC 46 00 B8 */	fmsubs f2, f6, f2, f0
/* 803103C0 0030D320  EC 05 01 B2 */	fmuls f0, f5, f6
/* 803103C4 0030D324  EC 23 09 78 */	fmsubs f1, f3, f5, f1
/* 803103C8 0030D328  D0 41 00 1C */	stfs f2, 0x1c(r1)
/* 803103CC 0030D32C  EC 04 01 F8 */	fmsubs f0, f4, f7, f0
/* 803103D0 0030D330  D0 21 00 14 */	stfs f1, 0x14(r1)
/* 803103D4 0030D334  D0 01 00 18 */	stfs f0, 0x18(r1)
/* 803103D8 0030D338  48 00 45 21 */	bl Normalize__9CVector3fFv
/* 803103DC 0030D33C  7F C3 F3 78 */	mr r3, r30
/* 803103E0 0030D340  38 81 00 20 */	addi r4, r1, 0x20
/* 803103E4 0030D344  38 A1 00 14 */	addi r5, r1, 0x14
/* 803103E8 0030D348  38 C1 00 08 */	addi r6, r1, 8
/* 803103EC 0030D34C  48 00 02 91 */	bl __ct__9CMatrix3fFRC9CVector3fRC9CVector3fRC9CVector3f
/* 803103F0 0030D350  80 01 00 44 */	lwz r0, 0x44(r1)
/* 803103F4 0030D354  83 E1 00 3C */	lwz r31, 0x3c(r1)
/* 803103F8 0030D358  83 C1 00 38 */	lwz r30, 0x38(r1)
/* 803103FC 0030D35C  7C 08 03 A6 */	mtlr r0
/* 80310400 0030D360  38 21 00 40 */	addi r1, r1, 0x40
/* 80310404 0030D364  4E 80 00 20 */	blr

.global RotateZ__9CMatrix3fFRC9CRelAngle
RotateZ__9CMatrix3fFRC9CRelAngle:
/* 80310408 0030D368  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8031040C 0030D36C  7C 08 02 A6 */	mflr r0
/* 80310410 0030D370  90 01 00 24 */	stw r0, 0x24(r1)
/* 80310414 0030D374  DB E1 00 10 */	stfd f31, 0x10(r1)
/* 80310418 0030D378  F3 E1 00 18 */	psq_st f31, 24(r1), 0, qr0
/* 8031041C 0030D37C  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80310420 0030D380  93 C1 00 08 */	stw r30, 8(r1)
/* 80310424 0030D384  7C 9F 23 78 */	mr r31, r4
/* 80310428 0030D388  7C 7E 1B 78 */	mr r30, r3
/* 8031042C 0030D38C  C0 24 00 00 */	lfs f1, 0(r4)
/* 80310430 0030D390  48 08 46 AD */	bl sin
/* 80310434 0030D394  FF E0 08 18 */	frsp f31, f1
/* 80310438 0030D398  C0 3F 00 00 */	lfs f1, 0(r31)
/* 8031043C 0030D39C  48 08 3F B5 */	bl cos
/* 80310440 0030D3A0  FC 60 08 18 */	frsp f3, f1
/* 80310444 0030D3A4  C0 22 C8 50 */	lfs f1, lbl_805AE570@sda21(r2)
/* 80310448 0030D3A8  FC 40 F8 50 */	fneg f2, f31
/* 8031044C 0030D3AC  C0 02 C8 54 */	lfs f0, lbl_805AE574@sda21(r2)
/* 80310450 0030D3B0  D0 7E 00 00 */	stfs f3, 0(r30)
/* 80310454 0030D3B4  D0 5E 00 04 */	stfs f2, 4(r30)
/* 80310458 0030D3B8  D0 3E 00 08 */	stfs f1, 8(r30)
/* 8031045C 0030D3BC  D3 FE 00 0C */	stfs f31, 0xc(r30)
/* 80310460 0030D3C0  D0 7E 00 10 */	stfs f3, 0x10(r30)
/* 80310464 0030D3C4  D0 3E 00 14 */	stfs f1, 0x14(r30)
/* 80310468 0030D3C8  D0 3E 00 18 */	stfs f1, 0x18(r30)
/* 8031046C 0030D3CC  D0 3E 00 1C */	stfs f1, 0x1c(r30)
/* 80310470 0030D3D0  D0 1E 00 20 */	stfs f0, 0x20(r30)
/* 80310474 0030D3D4  E3 E1 00 18 */	psq_l f31, 24(r1), 0, qr0
/* 80310478 0030D3D8  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8031047C 0030D3DC  CB E1 00 10 */	lfd f31, 0x10(r1)
/* 80310480 0030D3E0  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80310484 0030D3E4  83 C1 00 08 */	lwz r30, 8(r1)
/* 80310488 0030D3E8  7C 08 03 A6 */	mtlr r0
/* 8031048C 0030D3EC  38 21 00 20 */	addi r1, r1, 0x20
/* 80310490 0030D3F0  4E 80 00 20 */	blr

.global RotateY__9CMatrix3fFRC9CRelAngle
RotateY__9CMatrix3fFRC9CRelAngle:
/* 80310494 0030D3F4  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80310498 0030D3F8  7C 08 02 A6 */	mflr r0
/* 8031049C 0030D3FC  90 01 00 24 */	stw r0, 0x24(r1)
/* 803104A0 0030D400  DB E1 00 10 */	stfd f31, 0x10(r1)
/* 803104A4 0030D404  F3 E1 00 18 */	psq_st f31, 24(r1), 0, qr0
/* 803104A8 0030D408  93 E1 00 0C */	stw r31, 0xc(r1)
/* 803104AC 0030D40C  93 C1 00 08 */	stw r30, 8(r1)
/* 803104B0 0030D410  7C 9F 23 78 */	mr r31, r4
/* 803104B4 0030D414  7C 7E 1B 78 */	mr r30, r3
/* 803104B8 0030D418  C0 24 00 00 */	lfs f1, 0(r4)
/* 803104BC 0030D41C  48 08 46 21 */	bl sin
/* 803104C0 0030D420  FF E0 08 18 */	frsp f31, f1
/* 803104C4 0030D424  C0 3F 00 00 */	lfs f1, 0(r31)
/* 803104C8 0030D428  48 08 3F 29 */	bl cos
/* 803104CC 0030D42C  FC 60 08 18 */	frsp f3, f1
/* 803104D0 0030D430  C0 42 C8 50 */	lfs f2, lbl_805AE570@sda21(r2)
/* 803104D4 0030D434  C0 22 C8 54 */	lfs f1, lbl_805AE574@sda21(r2)
/* 803104D8 0030D438  FC 00 F8 50 */	fneg f0, f31
/* 803104DC 0030D43C  D0 7E 00 00 */	stfs f3, 0(r30)
/* 803104E0 0030D440  D0 5E 00 04 */	stfs f2, 4(r30)
/* 803104E4 0030D444  D3 FE 00 08 */	stfs f31, 8(r30)
/* 803104E8 0030D448  D0 5E 00 0C */	stfs f2, 0xc(r30)
/* 803104EC 0030D44C  D0 3E 00 10 */	stfs f1, 0x10(r30)
/* 803104F0 0030D450  D0 5E 00 14 */	stfs f2, 0x14(r30)
/* 803104F4 0030D454  D0 1E 00 18 */	stfs f0, 0x18(r30)
/* 803104F8 0030D458  D0 5E 00 1C */	stfs f2, 0x1c(r30)
/* 803104FC 0030D45C  D0 7E 00 20 */	stfs f3, 0x20(r30)
/* 80310500 0030D460  E3 E1 00 18 */	psq_l f31, 24(r1), 0, qr0
/* 80310504 0030D464  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80310508 0030D468  CB E1 00 10 */	lfd f31, 0x10(r1)
/* 8031050C 0030D46C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80310510 0030D470  83 C1 00 08 */	lwz r30, 8(r1)
/* 80310514 0030D474  7C 08 03 A6 */	mtlr r0
/* 80310518 0030D478  38 21 00 20 */	addi r1, r1, 0x20
/* 8031051C 0030D47C  4E 80 00 20 */	blr

.global __ct__9CMatrix3fFR12CInputStream
__ct__9CMatrix3fFR12CInputStream:
/* 80310520 0030D480  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80310524 0030D484  7C 08 02 A6 */	mflr r0
/* 80310528 0030D488  90 01 00 14 */	stw r0, 0x14(r1)
/* 8031052C 0030D48C  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80310530 0030D490  7C 9F 23 78 */	mr r31, r4
/* 80310534 0030D494  93 C1 00 08 */	stw r30, 8(r1)
/* 80310538 0030D498  7C 7E 1B 78 */	mr r30, r3
/* 8031053C 0030D49C  7F E3 FB 78 */	mr r3, r31
/* 80310540 0030D4A0  48 02 E6 E9 */	bl ReadFloat__12CInputStreamFv
/* 80310544 0030D4A4  D0 3E 00 00 */	stfs f1, 0(r30)
/* 80310548 0030D4A8  7F E3 FB 78 */	mr r3, r31
/* 8031054C 0030D4AC  48 02 E6 DD */	bl ReadFloat__12CInputStreamFv
/* 80310550 0030D4B0  D0 3E 00 04 */	stfs f1, 4(r30)
/* 80310554 0030D4B4  7F E3 FB 78 */	mr r3, r31
/* 80310558 0030D4B8  48 02 E6 D1 */	bl ReadFloat__12CInputStreamFv
/* 8031055C 0030D4BC  D0 3E 00 08 */	stfs f1, 8(r30)
/* 80310560 0030D4C0  7F E3 FB 78 */	mr r3, r31
/* 80310564 0030D4C4  48 02 E6 C5 */	bl ReadFloat__12CInputStreamFv
/* 80310568 0030D4C8  D0 3E 00 0C */	stfs f1, 0xc(r30)
/* 8031056C 0030D4CC  7F E3 FB 78 */	mr r3, r31
/* 80310570 0030D4D0  48 02 E6 B9 */	bl ReadFloat__12CInputStreamFv
/* 80310574 0030D4D4  D0 3E 00 10 */	stfs f1, 0x10(r30)
/* 80310578 0030D4D8  7F E3 FB 78 */	mr r3, r31
/* 8031057C 0030D4DC  48 02 E6 AD */	bl ReadFloat__12CInputStreamFv
/* 80310580 0030D4E0  D0 3E 00 14 */	stfs f1, 0x14(r30)
/* 80310584 0030D4E4  7F E3 FB 78 */	mr r3, r31
/* 80310588 0030D4E8  48 02 E6 A1 */	bl ReadFloat__12CInputStreamFv
/* 8031058C 0030D4EC  D0 3E 00 18 */	stfs f1, 0x18(r30)
/* 80310590 0030D4F0  7F E3 FB 78 */	mr r3, r31
/* 80310594 0030D4F4  48 02 E6 95 */	bl ReadFloat__12CInputStreamFv
/* 80310598 0030D4F8  D0 3E 00 1C */	stfs f1, 0x1c(r30)
/* 8031059C 0030D4FC  7F E3 FB 78 */	mr r3, r31
/* 803105A0 0030D500  48 02 E6 89 */	bl ReadFloat__12CInputStreamFv
/* 803105A4 0030D504  D0 3E 00 20 */	stfs f1, 0x20(r30)
/* 803105A8 0030D508  7F C3 F3 78 */	mr r3, r30
/* 803105AC 0030D50C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 803105B0 0030D510  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 803105B4 0030D514  83 C1 00 08 */	lwz r30, 8(r1)
/* 803105B8 0030D518  7C 08 03 A6 */	mtlr r0
/* 803105BC 0030D51C  38 21 00 10 */	addi r1, r1, 0x10
/* 803105C0 0030D520  4E 80 00 20 */	blr

.global __ct__9CMatrix3fFRC9CMatrix3ffRC9CMatrix3ff
__ct__9CMatrix3fFRC9CMatrix3ffRC9CMatrix3ff:
/* 803105C4 0030D524  C0 05 00 00 */	lfs f0, 0(r5)
/* 803105C8 0030D528  C0 64 00 00 */	lfs f3, 0(r4)
/* 803105CC 0030D52C  EC 00 00 B2 */	fmuls f0, f0, f2
/* 803105D0 0030D530  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 803105D4 0030D534  D0 03 00 00 */	stfs f0, 0(r3)
/* 803105D8 0030D538  C0 05 00 04 */	lfs f0, 4(r5)
/* 803105DC 0030D53C  C0 64 00 04 */	lfs f3, 4(r4)
/* 803105E0 0030D540  EC 00 00 B2 */	fmuls f0, f0, f2
/* 803105E4 0030D544  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 803105E8 0030D548  D0 03 00 04 */	stfs f0, 4(r3)
/* 803105EC 0030D54C  C0 05 00 08 */	lfs f0, 8(r5)
/* 803105F0 0030D550  C0 64 00 08 */	lfs f3, 8(r4)
/* 803105F4 0030D554  EC 00 00 B2 */	fmuls f0, f0, f2
/* 803105F8 0030D558  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 803105FC 0030D55C  D0 03 00 08 */	stfs f0, 8(r3)
/* 80310600 0030D560  C0 05 00 0C */	lfs f0, 0xc(r5)
/* 80310604 0030D564  C0 64 00 0C */	lfs f3, 0xc(r4)
/* 80310608 0030D568  EC 00 00 B2 */	fmuls f0, f0, f2
/* 8031060C 0030D56C  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 80310610 0030D570  D0 03 00 0C */	stfs f0, 0xc(r3)
/* 80310614 0030D574  C0 05 00 10 */	lfs f0, 0x10(r5)
/* 80310618 0030D578  C0 64 00 10 */	lfs f3, 0x10(r4)
/* 8031061C 0030D57C  EC 00 00 B2 */	fmuls f0, f0, f2
/* 80310620 0030D580  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 80310624 0030D584  D0 03 00 10 */	stfs f0, 0x10(r3)
/* 80310628 0030D588  C0 05 00 14 */	lfs f0, 0x14(r5)
/* 8031062C 0030D58C  C0 64 00 14 */	lfs f3, 0x14(r4)
/* 80310630 0030D590  EC 00 00 B2 */	fmuls f0, f0, f2
/* 80310634 0030D594  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 80310638 0030D598  D0 03 00 14 */	stfs f0, 0x14(r3)
/* 8031063C 0030D59C  C0 05 00 18 */	lfs f0, 0x18(r5)
/* 80310640 0030D5A0  C0 64 00 18 */	lfs f3, 0x18(r4)
/* 80310644 0030D5A4  EC 00 00 B2 */	fmuls f0, f0, f2
/* 80310648 0030D5A8  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 8031064C 0030D5AC  D0 03 00 18 */	stfs f0, 0x18(r3)
/* 80310650 0030D5B0  C0 05 00 1C */	lfs f0, 0x1c(r5)
/* 80310654 0030D5B4  C0 64 00 1C */	lfs f3, 0x1c(r4)
/* 80310658 0030D5B8  EC 00 00 B2 */	fmuls f0, f0, f2
/* 8031065C 0030D5BC  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 80310660 0030D5C0  D0 03 00 1C */	stfs f0, 0x1c(r3)
/* 80310664 0030D5C4  C0 05 00 20 */	lfs f0, 0x20(r5)
/* 80310668 0030D5C8  C0 64 00 20 */	lfs f3, 0x20(r4)
/* 8031066C 0030D5CC  EC 00 00 B2 */	fmuls f0, f0, f2
/* 80310670 0030D5D0  EC 03 00 7A */	fmadds f0, f3, f1, f0
/* 80310674 0030D5D4  D0 03 00 20 */	stfs f0, 0x20(r3)
/* 80310678 0030D5D8  4E 80 00 20 */	blr

.global __ct__9CMatrix3fFRC9CVector3fRC9CVector3fRC9CVector3f
__ct__9CMatrix3fFRC9CVector3fRC9CVector3fRC9CVector3f:
/* 8031067C 0030D5DC  C0 04 00 00 */	lfs f0, 0(r4)
/* 80310680 0030D5E0  D0 03 00 00 */	stfs f0, 0(r3)
/* 80310684 0030D5E4  C0 04 00 04 */	lfs f0, 4(r4)
/* 80310688 0030D5E8  D0 03 00 04 */	stfs f0, 4(r3)
/* 8031068C 0030D5EC  C0 04 00 08 */	lfs f0, 8(r4)
/* 80310690 0030D5F0  D0 03 00 08 */	stfs f0, 8(r3)
/* 80310694 0030D5F4  C0 05 00 00 */	lfs f0, 0(r5)
/* 80310698 0030D5F8  D0 03 00 0C */	stfs f0, 0xc(r3)
/* 8031069C 0030D5FC  C0 05 00 04 */	lfs f0, 4(r5)
/* 803106A0 0030D600  D0 03 00 10 */	stfs f0, 0x10(r3)
/* 803106A4 0030D604  C0 05 00 08 */	lfs f0, 8(r5)
/* 803106A8 0030D608  D0 03 00 14 */	stfs f0, 0x14(r3)
/* 803106AC 0030D60C  C0 06 00 00 */	lfs f0, 0(r6)
/* 803106B0 0030D610  D0 03 00 18 */	stfs f0, 0x18(r3)
/* 803106B4 0030D614  C0 06 00 04 */	lfs f0, 4(r6)
/* 803106B8 0030D618  D0 03 00 1C */	stfs f0, 0x1c(r3)
/* 803106BC 0030D61C  C0 06 00 08 */	lfs f0, 8(r6)
/* 803106C0 0030D620  D0 03 00 20 */	stfs f0, 0x20(r3)
/* 803106C4 0030D624  4E 80 00 20 */	blr

.global __sinit_CMatrix3f_cpp
__sinit_CMatrix3f_cpp:
/* 803106C8 0030D628  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 803106CC 0030D62C  7C 08 02 A6 */	mflr r0
/* 803106D0 0030D630  3C 60 80 5A */	lis r3, lbl_805A65FC@ha
/* 803106D4 0030D634  C0 02 C8 50 */	lfs f0, lbl_805AE570@sda21(r2)
/* 803106D8 0030D638  90 01 00 34 */	stw r0, 0x34(r1)
/* 803106DC 0030D63C  38 63 65 FC */	addi r3, r3, lbl_805A65FC@l
/* 803106E0 0030D640  C0 22 C8 54 */	lfs f1, lbl_805AE574@sda21(r2)
/* 803106E4 0030D644  38 81 00 08 */	addi r4, r1, 8
/* 803106E8 0030D648  D0 01 00 0C */	stfs f0, 0xc(r1)
/* 803106EC 0030D64C  D0 21 00 08 */	stfs f1, 8(r1)
/* 803106F0 0030D650  D0 01 00 10 */	stfs f0, 0x10(r1)
/* 803106F4 0030D654  D0 01 00 14 */	stfs f0, 0x14(r1)
/* 803106F8 0030D658  D0 21 00 18 */	stfs f1, 0x18(r1)
/* 803106FC 0030D65C  D0 01 00 1C */	stfs f0, 0x1c(r1)
/* 80310700 0030D660  D0 01 00 20 */	stfs f0, 0x20(r1)
/* 80310704 0030D664  D0 01 00 24 */	stfs f0, 0x24(r1)
/* 80310708 0030D668  D0 21 00 28 */	stfs f1, 0x28(r1)
/* 8031070C 0030D66C  4B FF F9 19 */	bl __ct__9CMatrix3fFRC9CMatrix3f
/* 80310710 0030D670  80 01 00 34 */	lwz r0, 0x34(r1)
/* 80310714 0030D674  7C 08 03 A6 */	mtlr r0
/* 80310718 0030D678  38 21 00 30 */	addi r1, r1, 0x30
/* 8031071C 0030D67C  4E 80 00 20 */	blr

.section .sdata2, "a"
.balign 8
.global lbl_805AE570
lbl_805AE570:
	# ROM: 0x3FAE10
	.4byte 0

.global lbl_805AE574
lbl_805AE574:
	# ROM: 0x3FAE14
	.float 1.0

