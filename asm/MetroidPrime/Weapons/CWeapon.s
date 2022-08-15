.include "macros.inc"

.section .data
.balign 8

.global lbl_803DFEF0
lbl_803DFEF0:
	# ROM: 0x3DCEF0
	.4byte 0
	.4byte 0
	.4byte __dt__7CWeaponFv
	.4byte 0
	.4byte PreThink__7CEntityFfR13CStateManager
	.4byte Think__7CWeaponFfR13CStateManager
	.4byte AcceptScriptMsg__6CActorF20EScriptObjectMessage9TUniqueIdR13CStateManager
	.4byte SetActive__6CActorFb
	.4byte PreRender__6CActorFR13CStateManagerRC14CFrustumPlanes
	.4byte AddToRenderer__6CActorCFRC14CFrustumPlanesRC13CStateManager
	.4byte Render__7CWeaponCFRC13CStateManager
	.4byte CanRenderUnsorted__6CActorCFRC13CStateManager
	.4byte CalculateRenderBounds__6CActorFv
	.4byte HealthInfo__6CActorFR13CStateManager
	.4byte GetDamageVulnerability__6CActorCFv
	.4byte GetDamageVulnerability__6CActorCFRC9CVector3fRC9CVector3fRC11CDamageInfo
	.4byte GetTouchBounds__6CActorCFv
	.4byte Touch__6CActorFR6CActorR13CStateManager
	.4byte GetOrbitPosition__6CActorCFRC13CStateManager
	.4byte GetAimPosition__6CActorCFRC13CStateManagerf
	.4byte GetHomingPosition__6CActorCFRC13CStateManagerf
	.4byte GetScanObjectIndicatorPosition__6CActorCFRC13CStateManager
	.4byte GetCollisionResponseType__7CWeaponCFRC9CVector3fRC9CVector3fRC11CWeaponModei
	.4byte FluidFXThink__7CWeaponFQ26CActor11EFluidStateR12CScriptWaterR13CStateManager
	.4byte OnScanStateChange__6CActorFQ26CActor10EScanStateR13CStateManager
	.4byte GetSortingBounds__6CActorCFRC12CTransform4f
	.4byte DoUserAnimEvent__6CActorFR13CStateManagerRC13CInt32POINode14EUserEventTypef

.global lbl_803DFF5C
lbl_803DFF5C:
	# ROM: 0x3DCF5C
	.4byte lbl_80101F60
	.4byte lbl_80101F68
	.4byte lbl_80101F70
	.4byte lbl_80101F78
	.4byte lbl_80101F8C
	.4byte lbl_80101F8C
	.4byte lbl_80101F7C
	.4byte lbl_80101F8C
	.4byte lbl_80101F84

.section .sdata
.balign 8

.global lbl_805A7558
lbl_805A7558:
	# ROM: 0x3F4EF8
	.4byte 0
	.4byte 0

.section .sdata2, "a"
.balign 8

.global lbl_805AB108
lbl_805AB108:
	# ROM: 0x3F79A8
	.4byte 0

.global lbl_805AB10C
lbl_805AB10C:
	# ROM: 0x3F79AC
	.float 0.1

.global lbl_805AB110
lbl_805AB110:
	# ROM: 0x3F79B0
	.float 0.3

.global lbl_805AB114
lbl_805AB114:
	# ROM: 0x3F79B4
	.float 0.5

.global lbl_805AB118
lbl_805AB118:
	# ROM: 0x3F79B8
	.float 0.25

.global lbl_805AB11C
lbl_805AB11C:
	# ROM: 0x3F79BC
	.float 1.0

.section .text, "ax"

.global GetCollisionResponseType__7CWeaponCFRC9CVector3fRC9CVector3fRC11CWeaponModei
GetCollisionResponseType__7CWeaponCFRC9CVector3fRC9CVector3fRC11CWeaponModei:
/* 80101F00 000FEE60  38 60 00 0C */	li r3, 0xc
/* 80101F04 000FEE64  4E 80 00 20 */	blr

.global Render__7CWeaponCFRC13CStateManager
Render__7CWeaponCFRC13CStateManager:
/* 80101F08 000FEE68  4E 80 00 20 */	blr

.global FluidFXThink__7CWeaponFQ26CActor11EFluidStateR12CScriptWaterR13CStateManager
FluidFXThink__7CWeaponFQ26CActor11EFluidStateR12CScriptWaterR13CStateManager:
/* 80101F0C 000FEE6C  94 21 FF 90 */	stwu r1, -0x70(r1)
/* 80101F10 000FEE70  7C 08 02 A6 */	mflr r0
/* 80101F14 000FEE74  90 01 00 74 */	stw r0, 0x74(r1)
/* 80101F18 000FEE78  DB E1 00 60 */	stfd f31, 0x60(r1)
/* 80101F1C 000FEE7C  F3 E1 00 68 */	psq_st f31, 104(r1), 0, qr0
/* 80101F20 000FEE80  BF 61 00 4C */	stmw r27, 0x4c(r1)
/* 80101F24 000FEE84  7C 7C 1B 78 */	mr r28, r3
/* 80101F28 000FEE88  C3 E2 93 E8 */	lfs f31, lbl_805AB108@sda21(r2)
/* 80101F2C 000FEE8C  80 03 00 F0 */	lwz r0, 0xf0(r3)
/* 80101F30 000FEE90  7C 9D 23 78 */	mr r29, r4
/* 80101F34 000FEE94  7C BE 2B 78 */	mr r30, r5
/* 80101F38 000FEE98  7C DF 33 78 */	mr r31, r6
/* 80101F3C 000FEE9C  28 00 00 08 */	cmplwi r0, 8
/* 80101F40 000FEEA0  3B 60 00 01 */	li r27, 1
/* 80101F44 000FEEA4  41 81 00 48 */	bgt lbl_80101F8C
/* 80101F48 000FEEA8  3C 60 80 3E */	lis r3, lbl_803DFF5C@ha
/* 80101F4C 000FEEAC  54 00 10 3A */	slwi r0, r0, 2
/* 80101F50 000FEEB0  38 63 FF 5C */	addi r3, r3, lbl_803DFF5C@l
/* 80101F54 000FEEB4  7C 03 00 2E */	lwzx r0, r3, r0
/* 80101F58 000FEEB8  7C 09 03 A6 */	mtctr r0
/* 80101F5C 000FEEBC  4E 80 04 20 */	bctr
.global lbl_80101F60
lbl_80101F60:
/* 80101F60 000FEEC0  C3 E2 93 EC */	lfs f31, lbl_805AB10C@sda21(r2)
/* 80101F64 000FEEC4  48 00 00 2C */	b lbl_80101F90
.global lbl_80101F68
lbl_80101F68:
/* 80101F68 000FEEC8  C3 E2 93 F0 */	lfs f31, lbl_805AB110@sda21(r2)
/* 80101F6C 000FEECC  48 00 00 24 */	b lbl_80101F90
.global lbl_80101F70
lbl_80101F70:
/* 80101F70 000FEED0  C3 E2 93 EC */	lfs f31, lbl_805AB10C@sda21(r2)
/* 80101F74 000FEED4  48 00 00 1C */	b lbl_80101F90
.global lbl_80101F78
lbl_80101F78:
/* 80101F78 000FEED8  48 00 00 18 */	b lbl_80101F90
.global lbl_80101F7C
lbl_80101F7C:
/* 80101F7C 000FEEDC  C3 E2 93 F4 */	lfs f31, lbl_805AB114@sda21(r2)
/* 80101F80 000FEEE0  48 00 00 10 */	b lbl_80101F90
.global lbl_80101F84
lbl_80101F84:
/* 80101F84 000FEEE4  C3 E2 93 EC */	lfs f31, lbl_805AB10C@sda21(r2)
/* 80101F88 000FEEE8  48 00 00 08 */	b lbl_80101F90
.global lbl_80101F8C
lbl_80101F8C:
/* 80101F8C 000FEEEC  3B 60 00 00 */	li r27, 0
lbl_80101F90:
/* 80101F90 000FEEF0  80 7C 00 E8 */	lwz r3, 0xe8(r28)
/* 80101F94 000FEEF4  54 60 06 31 */	rlwinm. r0, r3, 0, 0x18, 0x18
/* 80101F98 000FEEF8  41 82 00 14 */	beq lbl_80101FAC
/* 80101F9C 000FEEFC  2C 1D 00 01 */	cmpwi r29, 1
/* 80101FA0 000FEF00  41 82 00 0C */	beq lbl_80101FAC
/* 80101FA4 000FEF04  C0 02 93 F4 */	lfs f0, lbl_805AB114@sda21(r2)
/* 80101FA8 000FEF08  EF FF 00 2A */	fadds f31, f31, f0
lbl_80101FAC:
/* 80101FAC 000FEF0C  54 60 07 7B */	rlwinm. r0, r3, 0, 0x1d, 0x1d
/* 80101FB0 000FEF10  41 82 00 0C */	beq lbl_80101FBC
/* 80101FB4 000FEF14  C0 02 93 F8 */	lfs f0, lbl_805AB118@sda21(r2)
/* 80101FB8 000FEF18  EF FF 00 2A */	fadds f31, f31, f0
lbl_80101FBC:
/* 80101FBC 000FEF1C  C0 02 93 FC */	lfs f0, lbl_805AB11C@sda21(r2)
/* 80101FC0 000FEF20  FC 1F 00 40 */	fcmpo cr0, f31, f0
/* 80101FC4 000FEF24  40 81 00 08 */	ble lbl_80101FCC
/* 80101FC8 000FEF28  FF E0 00 90 */	fmr f31, f0
lbl_80101FCC:
/* 80101FCC 000FEF2C  57 60 06 3F */	clrlwi. r0, r27, 0x18
/* 80101FD0 000FEF30  41 82 00 EC */	beq lbl_801020BC
/* 80101FD4 000FEF34  7F C4 F3 78 */	mr r4, r30
/* 80101FD8 000FEF38  38 61 00 30 */	addi r3, r1, 0x30
/* 80101FDC 000FEF3C  4B F7 46 F1 */	bl GetTriggerBoundsWR__14CScriptTriggerCFv
/* 80101FE0 000FEF40  C0 3C 00 60 */	lfs f1, 0x60(r28)
/* 80101FE4 000FEF44  C0 1C 00 50 */	lfs f0, 0x50(r28)
/* 80101FE8 000FEF48  C0 7C 00 40 */	lfs f3, 0x40(r28)
/* 80101FEC 000FEF4C  C0 41 00 44 */	lfs f2, 0x44(r1)
/* 80101FF0 000FEF50  D0 61 00 24 */	stfs f3, 0x24(r1)
/* 80101FF4 000FEF54  D0 01 00 28 */	stfs f0, 0x28(r1)
/* 80101FF8 000FEF58  D0 41 00 2C */	stfs f2, 0x2c(r1)
/* 80101FFC 000FEF5C  80 1C 00 E8 */	lwz r0, 0xe8(r28)
/* 80102000 000FEF60  D0 61 00 18 */	stfs f3, 0x18(r1)
/* 80102004 000FEF64  54 00 06 31 */	rlwinm. r0, r0, 0, 0x18, 0x18
/* 80102008 000FEF68  D0 01 00 1C */	stfs f0, 0x1c(r1)
/* 8010200C 000FEF6C  D0 21 00 20 */	stfs f1, 0x20(r1)
/* 80102010 000FEF70  41 82 00 20 */	beq lbl_80102030
/* 80102014 000FEF74  7F C3 F3 78 */	mr r3, r30
/* 80102018 000FEF78  38 81 00 24 */	addi r4, r1, 0x24
/* 8010201C 000FEF7C  4B FF CC 95 */	bl CanRippleAtPoint__12CScriptWaterCFRC9CVector3f
/* 80102020 000FEF80  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 80102024 000FEF84  40 82 00 18 */	bne lbl_8010203C
/* 80102028 000FEF88  3B 60 00 00 */	li r27, 0
/* 8010202C 000FEF8C  48 00 00 10 */	b lbl_8010203C
lbl_80102030:
/* 80102030 000FEF90  2C 1D 00 01 */	cmpwi r29, 1
/* 80102034 000FEF94  40 82 00 08 */	bne lbl_8010203C
/* 80102038 000FEF98  3B 60 00 00 */	li r27, 0
lbl_8010203C:
/* 8010203C 000FEF9C  57 60 06 3F */	clrlwi. r0, r27, 0x18
/* 80102040 000FEFA0  41 82 00 7C */	beq lbl_801020BC
/* 80102044 000FEFA4  2C 1D 00 00 */	cmpwi r29, 0
/* 80102048 000FEFA8  3B 60 00 00 */	li r27, 0
/* 8010204C 000FEFAC  41 82 00 0C */	beq lbl_80102058
/* 80102050 000FEFB0  2C 1D 00 02 */	cmpwi r29, 2
/* 80102054 000FEFB4  40 82 00 08 */	bne lbl_8010205C
lbl_80102058:
/* 80102058 000FEFB8  3B 60 00 01 */	li r27, 1
lbl_8010205C:
/* 8010205C 000FEFBC  A0 1C 00 08 */	lhz r0, 8(r28)
/* 80102060 000FEFC0  FC 20 F8 90 */	fmr f1, f31
/* 80102064 000FEFC4  7F C6 F3 78 */	mr r6, r30
/* 80102068 000FEFC8  7F E7 FB 78 */	mr r7, r31
/* 8010206C 000FEFCC  B0 01 00 14 */	sth r0, 0x14(r1)
/* 80102070 000FEFD0  38 81 00 14 */	addi r4, r1, 0x14
/* 80102074 000FEFD4  38 A1 00 24 */	addi r5, r1, 0x24
/* 80102078 000FEFD8  80 7E 01 B4 */	lwz r3, 0x1b4(r30)
/* 8010207C 000FEFDC  B0 01 00 10 */	sth r0, 0x10(r1)
/* 80102080 000FEFE0  81 83 00 00 */	lwz r12, 0(r3)
/* 80102084 000FEFE4  81 8C 00 0C */	lwz r12, 0xc(r12)
/* 80102088 000FEFE8  7D 89 03 A6 */	mtctr r12
/* 8010208C 000FEFEC  4E 80 04 21 */	bctrl
/* 80102090 000FEFF0  A0 1C 00 08 */	lhz r0, 8(r28)
/* 80102094 000FEFF4  FC 20 F8 90 */	fmr f1, f31
/* 80102098 000FEFF8  7F E5 FB 78 */	mr r5, r31
/* 8010209C 000FEFFC  7F C6 F3 78 */	mr r6, r30
/* 801020A0 000FF000  B0 01 00 0C */	sth r0, 0xc(r1)
/* 801020A4 000FF004  7F 68 DB 78 */	mr r8, r27
/* 801020A8 000FF008  38 81 00 0C */	addi r4, r1, 0xc
/* 801020AC 000FF00C  B0 01 00 08 */	sth r0, 8(r1)
/* 801020B0 000FF010  38 E1 00 24 */	addi r7, r1, 0x24
/* 801020B4 000FF014  80 7F 08 7C */	lwz r3, 0x87c(r31)
/* 801020B8 000FF018  48 02 D2 01 */	bl CreateSplash__18CFluidPlaneManagerF9TUniqueIdR13CStateManagerRC12CScriptWaterRC9CVector3ffb
lbl_801020BC:
/* 801020BC 000FF01C  E3 E1 00 68 */	psq_l f31, 104(r1), 0, qr0
/* 801020C0 000FF020  CB E1 00 60 */	lfd f31, 0x60(r1)
/* 801020C4 000FF024  BB 61 00 4C */	lmw r27, 0x4c(r1)
/* 801020C8 000FF028  80 01 00 74 */	lwz r0, 0x74(r1)
/* 801020CC 000FF02C  7C 08 03 A6 */	mtlr r0
/* 801020D0 000FF030  38 21 00 70 */	addi r1, r1, 0x70
/* 801020D4 000FF034  4E 80 00 20 */	blr

.global Think__7CWeaponFfR13CStateManager
Think__7CWeaponFfR13CStateManager:
/* 801020D8 000FF038  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 801020DC 000FF03C  7C 08 02 A6 */	mflr r0
/* 801020E0 000FF040  90 01 00 44 */	stw r0, 0x44(r1)
/* 801020E4 000FF044  DB E1 00 30 */	stfd f31, 0x30(r1)
/* 801020E8 000FF048  F3 E1 00 38 */	psq_st f31, 56(r1), 0, qr0
/* 801020EC 000FF04C  93 E1 00 2C */	stw r31, 0x2c(r1)
/* 801020F0 000FF050  93 C1 00 28 */	stw r30, 0x28(r1)
/* 801020F4 000FF054  7C 7E 1B 78 */	mr r30, r3
/* 801020F8 000FF058  FF E0 08 90 */	fmr f31, f1
/* 801020FC 000FF05C  C0 03 01 48 */	lfs f0, 0x148(r3)
/* 80102100 000FF060  7C 9F 23 78 */	mr r31, r4
/* 80102104 000FF064  EC 00 F8 2A */	fadds f0, f0, f31
/* 80102108 000FF068  D0 03 01 48 */	stfs f0, 0x148(r3)
/* 8010210C 000FF06C  80 03 00 E8 */	lwz r0, 0xe8(r3)
/* 80102110 000FF070  54 00 04 A4 */	rlwinm r0, r0, 0, 0x12, 0x12
/* 80102114 000FF074  2C 00 20 00 */	cmpwi r0, 0x2000
/* 80102118 000FF078  40 82 00 98 */	bne lbl_801021B0
/* 8010211C 000FF07C  C0 5E 01 48 */	lfs f2, 0x148(r30)
/* 80102120 000FF080  38 81 00 08 */	addi r4, r1, 8
/* 80102124 000FF084  C0 3E 01 4C */	lfs f1, 0x14c(r30)
/* 80102128 000FF088  38 6D 89 98 */	addi r3, r13, lbl_805A7558@sda21
/* 8010212C 000FF08C  C0 02 93 FC */	lfs f0, lbl_805AB11C@sda21(r2)
/* 80102130 000FF090  EC 02 00 7C */	fnmsubs f0, f2, f1, f0
/* 80102134 000FF094  D0 01 00 08 */	stfs f0, 8(r1)
/* 80102138 000FF098  4B F4 D5 F9 */	bl "Max<f>__5CMathFRCfRCf"
/* 8010213C 000FF09C  88 1E 01 14 */	lbz r0, 0x114(r30)
/* 80102140 000FF0A0  38 80 00 00 */	li r4, 0
/* 80102144 000FF0A4  C0 83 00 00 */	lfs f4, 0(r3)
/* 80102148 000FF0A8  C0 1E 01 18 */	lfs f0, 0x118(r30)
/* 8010214C 000FF0AC  80 BE 01 10 */	lwz r5, 0x110(r30)
/* 80102150 000FF0B0  EC 64 00 32 */	fmuls f3, f4, f0
/* 80102154 000FF0B4  98 01 00 10 */	stb r0, 0x10(r1)
/* 80102158 000FF0B8  C0 3E 01 20 */	lfs f1, 0x120(r30)
/* 8010215C 000FF0BC  C0 1E 01 24 */	lfs f0, 0x124(r30)
/* 80102160 000FF0C0  EC 44 00 72 */	fmuls f2, f4, f1
/* 80102164 000FF0C4  80 01 00 10 */	lwz r0, 0x10(r1)
/* 80102168 000FF0C8  EC 24 00 32 */	fmuls f1, f4, f0
/* 8010216C 000FF0CC  90 BE 01 2C */	stw r5, 0x12c(r30)
/* 80102170 000FF0D0  FC 00 18 18 */	frsp f0, f3
/* 80102174 000FF0D4  90 1E 01 30 */	stw r0, 0x130(r30)
/* 80102178 000FF0D8  88 01 00 24 */	lbz r0, 0x24(r1)
/* 8010217C 000FF0DC  50 80 3E 30 */	rlwimi r0, r4, 7, 0x18, 0x18
/* 80102180 000FF0E0  D0 7E 01 34 */	stfs f3, 0x134(r30)
/* 80102184 000FF0E4  D0 1E 01 38 */	stfs f0, 0x138(r30)
/* 80102188 000FF0E8  D0 5E 01 3C */	stfs f2, 0x13c(r30)
/* 8010218C 000FF0EC  D0 3E 01 40 */	stfs f1, 0x140(r30)
/* 80102190 000FF0F0  90 A1 00 0C */	stw r5, 0xc(r1)
/* 80102194 000FF0F4  D0 61 00 14 */	stfs f3, 0x14(r1)
/* 80102198 000FF0F8  D0 61 00 18 */	stfs f3, 0x18(r1)
/* 8010219C 000FF0FC  D0 41 00 1C */	stfs f2, 0x1c(r1)
/* 801021A0 000FF100  D0 21 00 20 */	stfs f1, 0x20(r1)
/* 801021A4 000FF104  98 01 00 24 */	stb r0, 0x24(r1)
/* 801021A8 000FF108  98 1E 01 44 */	stb r0, 0x144(r30)
/* 801021AC 000FF10C  48 00 00 3C */	b lbl_801021E8
lbl_801021B0:
/* 801021B0 000FF110  80 7E 01 10 */	lwz r3, 0x110(r30)
/* 801021B4 000FF114  80 1E 01 14 */	lwz r0, 0x114(r30)
/* 801021B8 000FF118  90 7E 01 2C */	stw r3, 0x12c(r30)
/* 801021BC 000FF11C  90 1E 01 30 */	stw r0, 0x130(r30)
/* 801021C0 000FF120  C0 1E 01 18 */	lfs f0, 0x118(r30)
/* 801021C4 000FF124  D0 1E 01 34 */	stfs f0, 0x134(r30)
/* 801021C8 000FF128  C0 1E 01 1C */	lfs f0, 0x11c(r30)
/* 801021CC 000FF12C  D0 1E 01 38 */	stfs f0, 0x138(r30)
/* 801021D0 000FF130  C0 1E 01 20 */	lfs f0, 0x120(r30)
/* 801021D4 000FF134  D0 1E 01 3C */	stfs f0, 0x13c(r30)
/* 801021D8 000FF138  C0 1E 01 24 */	lfs f0, 0x124(r30)
/* 801021DC 000FF13C  D0 1E 01 40 */	stfs f0, 0x140(r30)
/* 801021E0 000FF140  88 1E 01 28 */	lbz r0, 0x128(r30)
/* 801021E4 000FF144  98 1E 01 44 */	stb r0, 0x144(r30)
lbl_801021E8:
/* 801021E8 000FF148  FC 20 F8 90 */	fmr f1, f31
/* 801021EC 000FF14C  7F C3 F3 78 */	mr r3, r30
/* 801021F0 000FF150  7F E4 FB 78 */	mr r4, r31
/* 801021F4 000FF154  4B F4 EE C5 */	bl Think__7CEntityFfR13CStateManager
/* 801021F8 000FF158  E3 E1 00 38 */	psq_l f31, 56(r1), 0, qr0
/* 801021FC 000FF15C  80 01 00 44 */	lwz r0, 0x44(r1)
/* 80102200 000FF160  CB E1 00 30 */	lfd f31, 0x30(r1)
/* 80102204 000FF164  83 E1 00 2C */	lwz r31, 0x2c(r1)
/* 80102208 000FF168  83 C1 00 28 */	lwz r30, 0x28(r1)
/* 8010220C 000FF16C  7C 08 03 A6 */	mtlr r0
/* 80102210 000FF170  38 21 00 40 */	addi r1, r1, 0x40
/* 80102214 000FF174  4E 80 00 20 */	blr

.global sub_80102218
sub_80102218:
/* 80102218 000FF178  C0 02 93 E8 */	lfs f0, lbl_805AB108@sda21(r2)
/* 8010221C 000FF17C  FC 01 00 40 */	fcmpo cr0, f1, f0
/* 80102220 000FF180  4C 81 00 20 */	blelr
/* 80102224 000FF184  C0 02 93 FC */	lfs f0, lbl_805AB11C@sda21(r2)
/* 80102228 000FF188  EC 00 08 24 */	fdivs f0, f0, f1
/* 8010222C 000FF18C  D0 03 01 4C */	stfs f0, 0x14c(r3)
/* 80102230 000FF190  4E 80 00 20 */	blr

.global __dt__7CWeaponFv
__dt__7CWeaponFv:
/* 80102234 000FF194  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80102238 000FF198  7C 08 02 A6 */	mflr r0
/* 8010223C 000FF19C  90 01 00 14 */	stw r0, 0x14(r1)
/* 80102240 000FF1A0  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80102244 000FF1A4  7C 9F 23 78 */	mr r31, r4
/* 80102248 000FF1A8  93 C1 00 08 */	stw r30, 8(r1)
/* 8010224C 000FF1AC  7C 7E 1B 79 */	or. r30, r3, r3
/* 80102250 000FF1B0  41 82 00 28 */	beq lbl_80102278
/* 80102254 000FF1B4  3C A0 80 3E */	lis r5, lbl_803DFEF0@ha
/* 80102258 000FF1B8  38 80 00 00 */	li r4, 0
/* 8010225C 000FF1BC  38 05 FE F0 */	addi r0, r5, lbl_803DFEF0@l
/* 80102260 000FF1C0  90 1E 00 00 */	stw r0, 0(r30)
/* 80102264 000FF1C4  4B F5 34 8D */	bl __dt__6CActorFv
/* 80102268 000FF1C8  7F E0 07 35 */	extsh. r0, r31
/* 8010226C 000FF1CC  40 81 00 0C */	ble lbl_80102278
/* 80102270 000FF1D0  7F C3 F3 78 */	mr r3, r30
/* 80102274 000FF1D4  48 21 36 BD */	bl Free__7CMemoryFPCv
lbl_80102278:
/* 80102278 000FF1D8  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8010227C 000FF1DC  7F C3 F3 78 */	mr r3, r30
/* 80102280 000FF1E0  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80102284 000FF1E4  83 C1 00 08 */	lwz r30, 8(r1)
/* 80102288 000FF1E8  7C 08 03 A6 */	mtlr r0
/* 8010228C 000FF1EC  38 21 00 10 */	addi r1, r1, 0x10
/* 80102290 000FF1F0  4E 80 00 20 */	blr

.global "__ct__7CWeaponF9TUniqueIdb9TUniqueId11EWeaponTypeRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC12CTransform4fRC15CMaterialFilterRC13CMaterialListRC11CDamageInfoiRC10CModelData"
"__ct__7CWeaponF9TUniqueIdb9TUniqueId11EWeaponTypeRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC12CTransform4fRC15CMaterialFilterRC13CMaterialListRC11CDamageInfoiRC10CModelData":
/* 80102294 000FF1F4  94 21 FE B0 */	stwu r1, -0x150(r1)
/* 80102298 000FF1F8  7C 08 02 A6 */	mflr r0
/* 8010229C 000FF1FC  3D 60 80 57 */	lis r11, NullConnectionList__7CEntity@ha
/* 801022A0 000FF200  90 01 01 54 */	stw r0, 0x154(r1)
/* 801022A4 000FF204  BE 81 01 20 */	stmw r20, 0x120(r1)
/* 801022A8 000FF208  7C 79 1B 78 */	mr r25, r3
/* 801022AC 000FF20C  7C 94 23 78 */	mr r20, r4
/* 801022B0 000FF210  7C DA 33 78 */	mr r26, r6
/* 801022B4 000FF214  83 A1 01 58 */	lwz r29, 0x158(r1)
/* 801022B8 000FF218  7C FB 3B 78 */	mr r27, r7
/* 801022BC 000FF21C  82 E1 01 5C */	lwz r23, 0x15c(r1)
/* 801022C0 000FF220  7D 1C 43 78 */	mr r28, r8
/* 801022C4 000FF224  83 C1 01 60 */	lwz r30, 0x160(r1)
/* 801022C8 000FF228  7D 35 4B 78 */	mr r21, r9
/* 801022CC 000FF22C  83 E1 01 64 */	lwz r31, 0x164(r1)
/* 801022D0 000FF230  7D 56 53 78 */	mr r22, r10
/* 801022D4 000FF234  83 01 01 68 */	lwz r24, 0x168(r1)
/* 801022D8 000FF238  38 61 00 30 */	addi r3, r1, 0x30
/* 801022DC 000FF23C  38 81 00 2C */	addi r4, r1, 0x2c
/* 801022E0 000FF240  38 C1 00 28 */	addi r6, r1, 0x28
/* 801022E4 000FF244  80 05 00 00 */	lwz r0, 0(r5)
/* 801022E8 000FF248  38 AB D4 10 */	addi r5, r11, NullConnectionList__7CEntity@l
/* 801022EC 000FF24C  81 8D A3 88 */	lwz r12, kInvalidEditorId@sda21(r13)
/* 801022F0 000FF250  90 01 00 2C */	stw r0, 0x2c(r1)
/* 801022F4 000FF254  91 81 00 28 */	stw r12, 0x28(r1)
/* 801022F8 000FF258  4B F4 F1 8D */	bl "__ct__11CEntityInfoF7TAreaIdRCQ24rstl48vector<11SConnection,Q24rstl17rmemory_allocator>9TEditorId"
/* 801022FC 000FF25C  38 61 00 B0 */	addi r3, r1, 0xb0
/* 80102300 000FF260  4B F1 8B 39 */	bl None__16CActorParametersFv
/* 80102304 000FF264  38 61 00 48 */	addi r3, r1, 0x48
/* 80102308 000FF268  38 81 00 B0 */	addi r4, r1, 0xb0
/* 8010230C 000FF26C  38 A0 00 01 */	li r5, 1
/* 80102310 000FF270  48 00 44 FD */	bl sub_8010680c
/* 80102314 000FF274  A0 6D A3 8C */	lhz r3, kInvalidUniqueId@sda21(r13)
/* 80102318 000FF278  38 81 00 48 */	addi r4, r1, 0x48
/* 8010231C 000FF27C  A0 F4 00 00 */	lhz r7, 0(r20)
/* 80102320 000FF280  38 01 00 10 */	addi r0, r1, 0x10
/* 80102324 000FF284  B0 61 00 10 */	sth r3, 0x10(r1)
/* 80102328 000FF288  7F 23 CB 78 */	mr r3, r25
/* 8010232C 000FF28C  7F 45 D3 78 */	mr r5, r26
/* 80102330 000FF290  7E A6 AB 78 */	mr r6, r21
/* 80102334 000FF294  B0 E1 00 14 */	sth r7, 0x14(r1)
/* 80102338 000FF298  7E C8 B3 78 */	mr r8, r22
/* 8010233C 000FF29C  7F 09 C3 78 */	mr r9, r24
/* 80102340 000FF2A0  7E EA BB 78 */	mr r10, r23
/* 80102344 000FF2A4  90 81 00 08 */	stw r4, 8(r1)
/* 80102348 000FF2A8  38 81 00 14 */	addi r4, r1, 0x14
/* 8010234C 000FF2AC  38 E1 00 30 */	addi r7, r1, 0x30
/* 80102350 000FF2B0  90 01 00 0C */	stw r0, 0xc(r1)
/* 80102354 000FF2B4  4B F5 34 CD */	bl "__ct__6CActorF9TUniqueIdbRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC11CEntityInfoRC12CTransform4fRC10CModelDataRC13CMaterialListRC16CActorParameters9TUniqueId"
/* 80102358 000FF2B8  38 61 00 48 */	addi r3, r1, 0x48
/* 8010235C 000FF2BC  38 80 FF FF */	li r4, -1
/* 80102360 000FF2C0  48 00 42 A9 */	bl __dt__16CActorParametersFv
/* 80102364 000FF2C4  38 61 00 B0 */	addi r3, r1, 0xb0
/* 80102368 000FF2C8  38 80 FF FF */	li r4, -1
/* 8010236C 000FF2CC  48 00 42 9D */	bl __dt__16CActorParametersFv
/* 80102370 000FF2D0  80 01 00 38 */	lwz r0, 0x38(r1)
/* 80102374 000FF2D4  80 61 00 40 */	lwz r3, 0x40(r1)
/* 80102378 000FF2D8  1C 00 00 0C */	mulli r0, r0, 0xc
/* 8010237C 000FF2DC  90 61 00 1C */	stw r3, 0x1c(r1)
/* 80102380 000FF2E0  7C 64 1B 78 */	mr r4, r3
/* 80102384 000FF2E4  7C 03 02 14 */	add r0, r3, r0
/* 80102388 000FF2E8  90 61 00 18 */	stw r3, 0x18(r1)
/* 8010238C 000FF2EC  90 01 00 24 */	stw r0, 0x24(r1)
/* 80102390 000FF2F0  90 01 00 20 */	stw r0, 0x20(r1)
/* 80102394 000FF2F4  48 00 00 08 */	b lbl_8010239C
lbl_80102398:
/* 80102398 000FF2F8  38 84 00 0C */	addi r4, r4, 0xc
lbl_8010239C:
/* 8010239C 000FF2FC  7C 04 00 40 */	cmplw r4, r0
/* 801023A0 000FF300  40 82 FF F8 */	bne lbl_80102398
/* 801023A4 000FF304  28 03 00 00 */	cmplwi r3, 0
/* 801023A8 000FF308  41 82 00 08 */	beq lbl_801023B0
/* 801023AC 000FF30C  48 21 35 85 */	bl Free__7CMemoryFPCv
lbl_801023B0:
/* 801023B0 000FF310  3C 60 80 3E */	lis r3, lbl_803DFEF0@ha
/* 801023B4 000FF314  C0 02 93 E8 */	lfs f0, lbl_805AB108@sda21(r2)
/* 801023B8 000FF318  38 03 FE F0 */	addi r0, r3, lbl_803DFEF0@l
/* 801023BC 000FF31C  7F 23 CB 78 */	mr r3, r25
/* 801023C0 000FF320  90 19 00 00 */	stw r0, 0(r25)
/* 801023C4 000FF324  93 F9 00 E8 */	stw r31, 0xe8(r25)
/* 801023C8 000FF328  A0 1B 00 00 */	lhz r0, 0(r27)
/* 801023CC 000FF32C  B0 19 00 EC */	sth r0, 0xec(r25)
/* 801023D0 000FF330  93 99 00 F0 */	stw r28, 0xf0(r25)
/* 801023D4 000FF334  80 1D 00 00 */	lwz r0, 0(r29)
/* 801023D8 000FF338  80 9D 00 04 */	lwz r4, 4(r29)
/* 801023DC 000FF33C  90 99 00 FC */	stw r4, 0xfc(r25)
/* 801023E0 000FF340  90 19 00 F8 */	stw r0, 0xf8(r25)
/* 801023E4 000FF344  80 1D 00 08 */	lwz r0, 8(r29)
/* 801023E8 000FF348  80 9D 00 0C */	lwz r4, 0xc(r29)
/* 801023EC 000FF34C  90 99 01 04 */	stw r4, 0x104(r25)
/* 801023F0 000FF350  90 19 01 00 */	stw r0, 0x100(r25)
/* 801023F4 000FF354  80 1D 00 10 */	lwz r0, 0x10(r29)
/* 801023F8 000FF358  90 19 01 08 */	stw r0, 0x108(r25)
/* 801023FC 000FF35C  80 1E 00 00 */	lwz r0, 0(r30)
/* 80102400 000FF360  90 19 01 10 */	stw r0, 0x110(r25)
/* 80102404 000FF364  88 1E 00 04 */	lbz r0, 4(r30)
/* 80102408 000FF368  98 19 01 14 */	stb r0, 0x114(r25)
/* 8010240C 000FF36C  C0 3E 00 08 */	lfs f1, 8(r30)
/* 80102410 000FF370  D0 39 01 18 */	stfs f1, 0x118(r25)
/* 80102414 000FF374  C0 3E 00 0C */	lfs f1, 0xc(r30)
/* 80102418 000FF378  D0 39 01 1C */	stfs f1, 0x11c(r25)
/* 8010241C 000FF37C  C0 3E 00 10 */	lfs f1, 0x10(r30)
/* 80102420 000FF380  D0 39 01 20 */	stfs f1, 0x120(r25)
/* 80102424 000FF384  C0 3E 00 14 */	lfs f1, 0x14(r30)
/* 80102428 000FF388  D0 39 01 24 */	stfs f1, 0x124(r25)
/* 8010242C 000FF38C  88 1E 00 18 */	lbz r0, 0x18(r30)
/* 80102430 000FF390  98 19 01 28 */	stb r0, 0x128(r25)
/* 80102434 000FF394  80 1E 00 00 */	lwz r0, 0(r30)
/* 80102438 000FF398  90 19 01 2C */	stw r0, 0x12c(r25)
/* 8010243C 000FF39C  88 1E 00 04 */	lbz r0, 4(r30)
/* 80102440 000FF3A0  98 19 01 30 */	stb r0, 0x130(r25)
/* 80102444 000FF3A4  C0 3E 00 08 */	lfs f1, 8(r30)
/* 80102448 000FF3A8  D0 39 01 34 */	stfs f1, 0x134(r25)
/* 8010244C 000FF3AC  C0 3E 00 0C */	lfs f1, 0xc(r30)
/* 80102450 000FF3B0  D0 39 01 38 */	stfs f1, 0x138(r25)
/* 80102454 000FF3B4  C0 3E 00 10 */	lfs f1, 0x10(r30)
/* 80102458 000FF3B8  D0 39 01 3C */	stfs f1, 0x13c(r25)
/* 8010245C 000FF3BC  C0 3E 00 14 */	lfs f1, 0x14(r30)
/* 80102460 000FF3C0  D0 39 01 40 */	stfs f1, 0x140(r25)
/* 80102464 000FF3C4  88 1E 00 18 */	lbz r0, 0x18(r30)
/* 80102468 000FF3C8  98 19 01 44 */	stb r0, 0x144(r25)
/* 8010246C 000FF3CC  D0 19 01 48 */	stfs f0, 0x148(r25)
/* 80102470 000FF3D0  D0 19 01 4C */	stfs f0, 0x14c(r25)
/* 80102474 000FF3D4  D0 19 01 50 */	stfs f0, 0x150(r25)
/* 80102478 000FF3D8  D0 19 01 54 */	stfs f0, 0x154(r25)
/* 8010247C 000FF3DC  BA 81 01 20 */	lmw r20, 0x120(r1)
/* 80102480 000FF3E0  80 01 01 54 */	lwz r0, 0x154(r1)
/* 80102484 000FF3E4  7C 08 03 A6 */	mtlr r0
/* 80102488 000FF3E8  38 21 01 50 */	addi r1, r1, 0x150
/* 8010248C 000FF3EC  4E 80 00 20 */	blr
