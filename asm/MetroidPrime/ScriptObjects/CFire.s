.include "macros.inc"

.section .data
.balign 8

.global lbl_803E9990
lbl_803E9990:
	# ROM: 0x3E6990
	.4byte 0
	.4byte 0
	.4byte __dt__5CFireFv
	.4byte Accept__5CFireFR8IVisitor
	.4byte PreThink__7CEntityFfR13CStateManager
	.4byte Think__5CFireFfR13CStateManager
	.4byte AcceptScriptMsg__5CFireF20EScriptObjectMessage9TUniqueIdR13CStateManager
	.4byte SetActive__6CActorFUc
	.4byte PreRender__6CActorFR13CStateManagerRC14CFrustumPlanes
	.4byte Render__5CFireCFRC13CStateManager
	.4byte Render__6CActorCFRC13CStateManager
	.4byte CanRenderUnsorted__6CActorCFRC13CStateManager
	.4byte CalculateRenderBounds__6CActorFv
	.4byte HealthInfo__6CActorFR13CStateManager
	.4byte GetDamageVulnerability__6CActorCFv
	.4byte GetDamageVulnerability__6CActorCFRC9CVector3fRC9CVector3fRC11CDamageInfo
	.4byte GetTouchBounds__5CFireCFv
	.4byte Touch__5CFireFR6CActorR13CStateManager
	.4byte GetOrbitPosition__6CActorCFRC13CStateManager
	.4byte GetAimPosition__6CActorCFRC13CStateManagerf
	.4byte GetHomingPosition__6CActorCFRC13CStateManagerf
	.4byte GetScanObjectIndicatorPosition__6CActorCFRC13CStateManager
	.4byte GetCollisionResponseType__6CActorCFRC9CVector3fRC9CVector3fRC11CWeaponModei
	.4byte FluidFXThink__6CActorFQ26CActor11EFluidStateR12CScriptWaterR13CStateManager
	.4byte OnScanStateChange__6CActorFQ26CActor10EScanStateR13CStateManager
	.4byte GetSortingBounds__6CActorCFRC13CStateManager
	.4byte DoUserAnimEvent__6CActorFR13CStateManagerRC13CInt32POINode14EUserEventTypef
	.4byte 0

.section .sdata
.balign 8

.global lbl_805A8390
lbl_805A8390:
	# ROM: 0x3F5D30
	.4byte 0x00000013

.global lbl_805A8394
lbl_805A8394:
	# ROM: 0x3F5D34
	.4byte 0x00000023

.section .text, "ax"

.global AcceptScriptMsg__5CFireF20EScriptObjectMessage9TUniqueIdR13CStateManager
AcceptScriptMsg__5CFireF20EScriptObjectMessage9TUniqueIdR13CStateManager:
/* 8025CC24 00259B84  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8025CC28 00259B88  7C 08 02 A6 */	mflr r0
/* 8025CC2C 00259B8C  90 01 00 24 */	stw r0, 0x24(r1)
/* 8025CC30 00259B90  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8025CC34 00259B94  7C 9F 23 78 */	mr r31, r4
/* 8025CC38 00259B98  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8025CC3C 00259B9C  7C 7E 1B 78 */	mr r30, r3
/* 8025CC40 00259BA0  A0 05 00 00 */	lhz r0, 0(r5)
/* 8025CC44 00259BA4  38 A1 00 08 */	addi r5, r1, 8
/* 8025CC48 00259BA8  B0 01 00 08 */	sth r0, 8(r1)
/* 8025CC4C 00259BAC  4B DF 69 C9 */	bl AcceptScriptMsg__6CActorF20EScriptObjectMessage9TUniqueIdR13CStateManager
/* 8025CC50 00259BB0  2C 1F 00 21 */	cmpwi r31, 0x21
/* 8025CC54 00259BB4  41 82 00 08 */	beq lbl_8025CC5C
/* 8025CC58 00259BB8  48 00 00 34 */	b lbl_8025CC8C
lbl_8025CC5C:
/* 8025CC5C 00259BBC  80 7E 00 E8 */	lwz r3, 0xe8(r30)
/* 8025CC60 00259BC0  38 80 00 01 */	li r4, 1
/* 8025CC64 00259BC4  81 83 00 00 */	lwz r12, 0(r3)
/* 8025CC68 00259BC8  81 8C 00 2C */	lwz r12, 0x2c(r12)
/* 8025CC6C 00259BCC  7D 89 03 A6 */	mtctr r12
/* 8025CC70 00259BD0  4E 80 04 21 */	bctrl
/* 8025CC74 00259BD4  7F C3 F3 78 */	mr r3, r30
/* 8025CC78 00259BD8  38 80 00 01 */	li r4, 1
/* 8025CC7C 00259BDC  81 9E 00 00 */	lwz r12, 0(r30)
/* 8025CC80 00259BE0  81 8C 00 1C */	lwz r12, 0x1c(r12)
/* 8025CC84 00259BE4  7D 89 03 A6 */	mtctr r12
/* 8025CC88 00259BE8  4E 80 04 21 */	bctrl
lbl_8025CC8C:
/* 8025CC8C 00259BEC  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8025CC90 00259BF0  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8025CC94 00259BF4  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8025CC98 00259BF8  7C 08 03 A6 */	mtlr r0
/* 8025CC9C 00259BFC  38 21 00 20 */	addi r1, r1, 0x20
/* 8025CCA0 00259C00  4E 80 00 20 */	blr

.global Think__5CFireFfR13CStateManager
Think__5CFireFfR13CStateManager:
/* 8025CCA4 00259C04  94 21 FF 50 */	stwu r1, -0xb0(r1)
/* 8025CCA8 00259C08  7C 08 02 A6 */	mflr r0
/* 8025CCAC 00259C0C  90 01 00 B4 */	stw r0, 0xb4(r1)
/* 8025CCB0 00259C10  DB E1 00 A0 */	stfd f31, 0xa0(r1)
/* 8025CCB4 00259C14  F3 E1 00 A8 */	psq_st f31, 168(r1), 0, qr0
/* 8025CCB8 00259C18  DB C1 00 90 */	stfd f30, 0x90(r1)
/* 8025CCBC 00259C1C  F3 C1 00 98 */	psq_st f30, 152(r1), 0, qr0
/* 8025CCC0 00259C20  93 E1 00 8C */	stw r31, 0x8c(r1)
/* 8025CCC4 00259C24  93 C1 00 88 */	stw r30, 0x88(r1)
/* 8025CCC8 00259C28  93 A1 00 84 */	stw r29, 0x84(r1)
/* 8025CCCC 00259C2C  7C 7E 1B 78 */	mr r30, r3
/* 8025CCD0 00259C30  FF C0 08 90 */	fmr f30, f1
/* 8025CCD4 00259C34  83 A3 00 E8 */	lwz r29, 0xe8(r3)
/* 8025CCD8 00259C38  7C 9F 23 78 */	mr r31, r4
/* 8025CCDC 00259C3C  7F A3 EB 78 */	mr r3, r29
/* 8025CCE0 00259C40  81 9D 00 00 */	lwz r12, 0(r29)
/* 8025CCE4 00259C44  81 8C 00 60 */	lwz r12, 0x60(r12)
/* 8025CCE8 00259C48  7D 89 03 A6 */	mtctr r12
/* 8025CCEC 00259C4C  4E 80 04 21 */	bctrl
/* 8025CCF0 00259C50  3C A0 43 30 */	lis r5, 0x4330
/* 8025CCF4 00259C54  6C 66 80 00 */	xoris r6, r3, 0x8000
/* 8025CCF8 00259C58  80 9D 00 90 */	lwz r4, 0x90(r29)
/* 8025CCFC 00259C5C  90 C1 00 6C */	stw r6, 0x6c(r1)
/* 8025CD00 00259C60  6C 83 80 00 */	xoris r3, r4, 0x8000
/* 8025CD04 00259C64  C8 42 BA 08 */	lfd f2, lbl_805AD728@sda21(r2)
/* 8025CD08 00259C68  90 A1 00 68 */	stw r5, 0x68(r1)
/* 8025CD0C 00259C6C  88 1E 00 30 */	lbz r0, 0x30(r30)
/* 8025CD10 00259C70  C8 01 00 68 */	lfd f0, 0x68(r1)
/* 8025CD14 00259C74  90 61 00 74 */	stw r3, 0x74(r1)
/* 8025CD18 00259C78  54 00 CF FF */	rlwinm. r0, r0, 0x19, 0x1f, 0x1f
/* 8025CD1C 00259C7C  EC 20 10 28 */	fsubs f1, f0, f2
/* 8025CD20 00259C80  90 A1 00 70 */	stw r5, 0x70(r1)
/* 8025CD24 00259C84  C8 01 00 70 */	lfd f0, 0x70(r1)
/* 8025CD28 00259C88  EC 00 10 28 */	fsubs f0, f0, f2
/* 8025CD2C 00259C8C  EF E1 00 24 */	fdivs f31, f1, f0
/* 8025CD30 00259C90  41 82 00 80 */	beq lbl_8025CDB0
/* 8025CD34 00259C94  80 7E 00 E8 */	lwz r3, 0xe8(r30)
/* 8025CD38 00259C98  C0 1E 01 44 */	lfs f0, 0x144(r30)
/* 8025CD3C 00259C9C  81 83 00 00 */	lwz r12, 0(r3)
/* 8025CD40 00259CA0  EC 3E 00 32 */	fmuls f1, f30, f0
/* 8025CD44 00259CA4  81 8C 00 0C */	lwz r12, 0xc(r12)
/* 8025CD48 00259CA8  7D 89 03 A6 */	mtctr r12
/* 8025CD4C 00259CAC  4E 80 04 21 */	bctrl
/* 8025CD50 00259CB0  C0 02 B9 FC */	lfs f0, lbl_805AD71C@sda21(r2)
/* 8025CD54 00259CB4  38 61 00 48 */	addi r3, r1, 0x48
/* 8025CD58 00259CB8  38 9E 00 F0 */	addi r4, r30, 0xf0
/* 8025CD5C 00259CBC  FC 1F 00 40 */	fcmpo cr0, f31, f0
/* 8025CD60 00259CC0  40 81 00 0C */	ble lbl_8025CD6C
/* 8025CD64 00259CC4  FC 00 F8 90 */	fmr f0, f31
/* 8025CD68 00259CC8  48 00 00 08 */	b lbl_8025CD70
lbl_8025CD6C:
/* 8025CD6C 00259CCC  C0 02 B9 F8 */	lfs f0, lbl_805AD718@sda21(r2)
lbl_8025CD70:
/* 8025CD70 00259CD0  EC 3E 00 32 */	fmuls f1, f30, f0
/* 8025CD74 00259CD4  4B E6 66 D5 */	bl __ct__11CDamageInfoFRC11CDamageInfof
/* 8025CD78 00259CD8  80 61 00 48 */	lwz r3, 0x48(r1)
/* 8025CD7C 00259CDC  80 01 00 4C */	lwz r0, 0x4c(r1)
/* 8025CD80 00259CE0  90 7E 01 0C */	stw r3, 0x10c(r30)
/* 8025CD84 00259CE4  90 1E 01 10 */	stw r0, 0x110(r30)
/* 8025CD88 00259CE8  C0 01 00 50 */	lfs f0, 0x50(r1)
/* 8025CD8C 00259CEC  D0 1E 01 14 */	stfs f0, 0x114(r30)
/* 8025CD90 00259CF0  C0 01 00 54 */	lfs f0, 0x54(r1)
/* 8025CD94 00259CF4  D0 1E 01 18 */	stfs f0, 0x118(r30)
/* 8025CD98 00259CF8  C0 01 00 58 */	lfs f0, 0x58(r1)
/* 8025CD9C 00259CFC  D0 1E 01 1C */	stfs f0, 0x11c(r30)
/* 8025CDA0 00259D00  C0 01 00 5C */	lfs f0, 0x5c(r1)
/* 8025CDA4 00259D04  D0 1E 01 20 */	stfs f0, 0x120(r30)
/* 8025CDA8 00259D08  88 01 00 60 */	lbz r0, 0x60(r1)
/* 8025CDAC 00259D0C  98 1E 01 24 */	stb r0, 0x124(r30)
lbl_8025CDB0:
/* 8025CDB0 00259D10  80 7E 00 E8 */	lwz r3, 0xe8(r30)
/* 8025CDB4 00259D14  3B A0 00 00 */	li r29, 0
/* 8025CDB8 00259D18  81 83 00 00 */	lwz r12, 0(r3)
/* 8025CDBC 00259D1C  81 8C 00 58 */	lwz r12, 0x58(r12)
/* 8025CDC0 00259D20  7D 89 03 A6 */	mtctr r12
/* 8025CDC4 00259D24  4E 80 04 21 */	bctrl
/* 8025CDC8 00259D28  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 8025CDCC 00259D2C  41 82 00 08 */	beq lbl_8025CDD4
/* 8025CDD0 00259D30  3B A0 00 01 */	li r29, 1
lbl_8025CDD4:
/* 8025CDD4 00259D34  88 1E 01 48 */	lbz r0, 0x148(r30)
/* 8025CDD8 00259D38  54 00 F7 FF */	rlwinm. r0, r0, 0x1e, 0x1f, 0x1f
/* 8025CDDC 00259D3C  41 82 00 A8 */	beq lbl_8025CE84
/* 8025CDE0 00259D40  80 9F 08 4C */	lwz r4, 0x84c(r31)
/* 8025CDE4 00259D44  38 61 00 2C */	addi r3, r1, 0x2c
/* 8025CDE8 00259D48  81 84 00 00 */	lwz r12, 0(r4)
/* 8025CDEC 00259D4C  81 8C 00 40 */	lwz r12, 0x40(r12)
/* 8025CDF0 00259D50  7D 89 03 A6 */	mtctr r12
/* 8025CDF4 00259D54  4E 80 04 21 */	bctrl
/* 8025CDF8 00259D58  7F C4 F3 78 */	mr r4, r30
/* 8025CDFC 00259D5C  38 61 00 10 */	addi r3, r1, 0x10
/* 8025CE00 00259D60  81 9E 00 00 */	lwz r12, 0(r30)
/* 8025CE04 00259D64  81 8C 00 40 */	lwz r12, 0x40(r12)
/* 8025CE08 00259D68  7D 89 03 A6 */	mtctr r12
/* 8025CE0C 00259D6C  4E 80 04 21 */	bctrl
/* 8025CE10 00259D70  38 61 00 2C */	addi r3, r1, 0x2c
/* 8025CE14 00259D74  38 81 00 10 */	addi r4, r1, 0x10
/* 8025CE18 00259D78  48 0D AD 6D */	bl DoBoundsOverlap__6CAABoxCFRC6CAABox
/* 8025CE1C 00259D7C  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 8025CE20 00259D80  38 60 00 00 */	li r3, 0
/* 8025CE24 00259D84  98 61 00 28 */	stb r3, 0x28(r1)
/* 8025CE28 00259D88  98 61 00 44 */	stb r3, 0x44(r1)
/* 8025CE2C 00259D8C  41 82 00 3C */	beq lbl_8025CE68
/* 8025CE30 00259D90  57 A0 06 3F */	clrlwi. r0, r29, 0x18
/* 8025CE34 00259D94  40 82 00 34 */	bne lbl_8025CE68
/* 8025CE38 00259D98  C0 02 B9 FC */	lfs f0, lbl_805AD71C@sda21(r2)
/* 8025CE3C 00259D9C  FC 1F 00 40 */	fcmpo cr0, f31, f0
/* 8025CE40 00259DA0  40 81 00 28 */	ble lbl_8025CE68
/* 8025CE44 00259DA4  C0 1E 01 4C */	lfs f0, 0x14c(r30)
/* 8025CE48 00259DA8  38 A0 00 01 */	li r5, 1
/* 8025CE4C 00259DAC  80 7F 08 4C */	lwz r3, 0x84c(r31)
/* 8025CE50 00259DB0  EC 3F 00 32 */	fmuls f1, f31, f0
/* 8025CE54 00259DB4  C0 5E 01 54 */	lfs f2, 0x154(r30)
/* 8025CE58 00259DB8  C0 7E 01 58 */	lfs f3, 0x158(r30)
/* 8025CE5C 00259DBC  80 9E 01 50 */	lwz r4, 0x150(r30)
/* 8025CE60 00259DC0  4B DB AC 55 */	bl SetVisorSteam__7CPlayerFfffUib
/* 8025CE64 00259DC4  48 00 00 20 */	b lbl_8025CE84
lbl_8025CE68:
/* 8025CE68 00259DC8  C0 42 BA 00 */	lfs f2, lbl_805AD720@sda21(r2)
/* 8025CE6C 00259DCC  38 80 FF FF */	li r4, -1
/* 8025CE70 00259DD0  80 7F 08 4C */	lwz r3, 0x84c(r31)
/* 8025CE74 00259DD4  38 A0 00 01 */	li r5, 1
/* 8025CE78 00259DD8  FC 60 10 90 */	fmr f3, f2
/* 8025CE7C 00259DDC  C0 22 B9 F8 */	lfs f1, lbl_805AD718@sda21(r2)
/* 8025CE80 00259DE0  4B DB AC 35 */	bl SetVisorSteam__7CPlayerFfffUib
lbl_8025CE84:
/* 8025CE84 00259DE4  C0 3E 01 5C */	lfs f1, 0x15c(r30)
/* 8025CE88 00259DE8  C0 02 BA 04 */	lfs f0, lbl_805AD724@sda21(r2)
/* 8025CE8C 00259DEC  EC 21 F0 2A */	fadds f1, f1, f30
/* 8025CE90 00259DF0  D0 3E 01 5C */	stfs f1, 0x15c(r30)
/* 8025CE94 00259DF4  C0 3E 01 5C */	lfs f1, 0x15c(r30)
/* 8025CE98 00259DF8  FC 01 00 40 */	fcmpo cr0, f1, f0
/* 8025CE9C 00259DFC  40 81 00 08 */	ble lbl_8025CEA4
/* 8025CEA0 00259E00  3B A0 00 01 */	li r29, 1
lbl_8025CEA4:
/* 8025CEA4 00259E04  57 A0 06 3F */	clrlwi. r0, r29, 0x18
/* 8025CEA8 00259E08  41 82 00 1C */	beq lbl_8025CEC4
/* 8025CEAC 00259E0C  A0 1E 00 08 */	lhz r0, 8(r30)
/* 8025CEB0 00259E10  7F E3 FB 78 */	mr r3, r31
/* 8025CEB4 00259E14  38 81 00 0C */	addi r4, r1, 0xc
/* 8025CEB8 00259E18  B0 01 00 08 */	sth r0, 8(r1)
/* 8025CEBC 00259E1C  B0 01 00 0C */	sth r0, 0xc(r1)
/* 8025CEC0 00259E20  4B DE F3 AD */	bl FreeScriptObject__13CStateManagerF9TUniqueId
lbl_8025CEC4:
/* 8025CEC4 00259E24  E3 E1 00 A8 */	psq_l f31, 168(r1), 0, qr0
/* 8025CEC8 00259E28  CB E1 00 A0 */	lfd f31, 0xa0(r1)
/* 8025CECC 00259E2C  E3 C1 00 98 */	psq_l f30, 152(r1), 0, qr0
/* 8025CED0 00259E30  CB C1 00 90 */	lfd f30, 0x90(r1)
/* 8025CED4 00259E34  83 E1 00 8C */	lwz r31, 0x8c(r1)
/* 8025CED8 00259E38  83 C1 00 88 */	lwz r30, 0x88(r1)
/* 8025CEDC 00259E3C  80 01 00 B4 */	lwz r0, 0xb4(r1)
/* 8025CEE0 00259E40  83 A1 00 84 */	lwz r29, 0x84(r1)
/* 8025CEE4 00259E44  7C 08 03 A6 */	mtlr r0
/* 8025CEE8 00259E48  38 21 00 B0 */	addi r1, r1, 0xb0
/* 8025CEEC 00259E4C  4E 80 00 20 */	blr

.global Accept__5CFireFR8IVisitor
Accept__5CFireFR8IVisitor:
/* 8025CEF0 00259E50  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8025CEF4 00259E54  7C 08 02 A6 */	mflr r0
/* 8025CEF8 00259E58  90 01 00 14 */	stw r0, 0x14(r1)
/* 8025CEFC 00259E5C  7C 60 1B 78 */	mr r0, r3
/* 8025CF00 00259E60  7C 83 23 78 */	mr r3, r4
/* 8025CF04 00259E64  81 84 00 00 */	lwz r12, 0(r4)
/* 8025CF08 00259E68  7C 04 03 78 */	mr r4, r0
/* 8025CF0C 00259E6C  81 8C 00 08 */	lwz r12, 8(r12)
/* 8025CF10 00259E70  7D 89 03 A6 */	mtctr r12
/* 8025CF14 00259E74  4E 80 04 21 */	bctrl
/* 8025CF18 00259E78  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8025CF1C 00259E7C  7C 08 03 A6 */	mtlr r0
/* 8025CF20 00259E80  38 21 00 10 */	addi r1, r1, 0x10
/* 8025CF24 00259E84  4E 80 00 20 */	blr

.global Render__5CFireCFRC13CStateManager
Render__5CFireCFRC13CStateManager:
/* 8025CF28 00259E88  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8025CF2C 00259E8C  7C 08 02 A6 */	mflr r0
/* 8025CF30 00259E90  90 01 00 24 */	stw r0, 0x24(r1)
/* 8025CF34 00259E94  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8025CF38 00259E98  3B E0 00 01 */	li r31, 1
/* 8025CF3C 00259E9C  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8025CF40 00259EA0  7C BE 2B 78 */	mr r30, r5
/* 8025CF44 00259EA4  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8025CF48 00259EA8  7C 9D 23 78 */	mr r29, r4
/* 8025CF4C 00259EAC  93 81 00 10 */	stw r28, 0x10(r1)
/* 8025CF50 00259EB0  7C 7C 1B 78 */	mr r28, r3
/* 8025CF54 00259EB4  88 03 01 48 */	lbz r0, 0x148(r3)
/* 8025CF58 00259EB8  54 00 E7 FF */	rlwinm. r0, r0, 0x1c, 0x1f, 0x1f
/* 8025CF5C 00259EBC  40 82 00 68 */	bne lbl_8025CFC4
/* 8025CF60 00259EC0  80 7E 08 B8 */	lwz r3, 0x8b8(r30)
/* 8025CF64 00259EC4  7F C4 F3 78 */	mr r4, r30
/* 8025CF68 00259EC8  80 63 00 00 */	lwz r3, 0(r3)
/* 8025CF6C 00259ECC  4B E3 45 ED */	bl GetActiveVisor__12CPlayerStateCFRC13CStateManager
/* 8025CF70 00259ED0  2C 03 00 02 */	cmpwi r3, 2
/* 8025CF74 00259ED4  41 82 00 24 */	beq lbl_8025CF98
/* 8025CF78 00259ED8  40 80 00 14 */	bge lbl_8025CF8C
/* 8025CF7C 00259EDC  2C 03 00 00 */	cmpwi r3, 0
/* 8025CF80 00259EE0  41 82 00 18 */	beq lbl_8025CF98
/* 8025CF84 00259EE4  40 80 00 24 */	bge lbl_8025CFA8
/* 8025CF88 00259EE8  48 00 00 3C */	b lbl_8025CFC4
lbl_8025CF8C:
/* 8025CF8C 00259EEC  2C 03 00 04 */	cmpwi r3, 4
/* 8025CF90 00259EF0  40 80 00 34 */	bge lbl_8025CFC4
/* 8025CF94 00259EF4  48 00 00 24 */	b lbl_8025CFB8
lbl_8025CF98:
/* 8025CF98 00259EF8  88 1C 01 48 */	lbz r0, 0x148(r28)
/* 8025CF9C 00259EFC  54 00 CF FE */	rlwinm r0, r0, 0x19, 0x1f, 0x1f
/* 8025CFA0 00259F00  7C 1F 03 78 */	mr r31, r0
/* 8025CFA4 00259F04  48 00 00 20 */	b lbl_8025CFC4
lbl_8025CFA8:
/* 8025CFA8 00259F08  88 1C 01 48 */	lbz r0, 0x148(r28)
/* 8025CFAC 00259F0C  54 00 DF FE */	rlwinm r0, r0, 0x1b, 0x1f, 0x1f
/* 8025CFB0 00259F10  7C 1F 03 78 */	mr r31, r0
/* 8025CFB4 00259F14  48 00 00 10 */	b lbl_8025CFC4
lbl_8025CFB8:
/* 8025CFB8 00259F18  88 1C 01 48 */	lbz r0, 0x148(r28)
/* 8025CFBC 00259F1C  54 00 D7 FE */	rlwinm r0, r0, 0x1a, 0x1f, 0x1f
/* 8025CFC0 00259F20  7C 1F 03 78 */	mr r31, r0
lbl_8025CFC4:
/* 8025CFC4 00259F24  57 E0 06 3F */	clrlwi. r0, r31, 0x18
/* 8025CFC8 00259F28  41 82 00 1C */	beq lbl_8025CFE4
/* 8025CFCC 00259F2C  80 6D A0 68 */	lwz r3, gpRender@sda21(r13)
/* 8025CFD0 00259F30  80 9C 00 E8 */	lwz r4, 0xe8(r28)
/* 8025CFD4 00259F34  81 83 00 00 */	lwz r12, 0(r3)
/* 8025CFD8 00259F38  81 8C 00 34 */	lwz r12, 0x34(r12)
/* 8025CFDC 00259F3C  7D 89 03 A6 */	mtctr r12
/* 8025CFE0 00259F40  4E 80 04 21 */	bctrl
lbl_8025CFE4:
/* 8025CFE4 00259F44  7F 83 E3 78 */	mr r3, r28
/* 8025CFE8 00259F48  7F A4 EB 78 */	mr r4, r29
/* 8025CFEC 00259F4C  7F C5 F3 78 */	mr r5, r30
/* 8025CFF0 00259F50  4B DF 7B 49 */	bl AddToRenderer__6CActorCFRC14CFrustumPlanesRC13CStateManager
/* 8025CFF4 00259F54  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8025CFF8 00259F58  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8025CFFC 00259F5C  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8025D000 00259F60  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8025D004 00259F64  83 81 00 10 */	lwz r28, 0x10(r1)
/* 8025D008 00259F68  7C 08 03 A6 */	mtlr r0
/* 8025D00C 00259F6C  38 21 00 20 */	addi r1, r1, 0x20
/* 8025D010 00259F70  4E 80 00 20 */	blr

.global GetTouchBounds__5CFireCFv
GetTouchBounds__5CFireCFv:
/* 8025D014 00259F74  88 04 00 30 */	lbz r0, 0x30(r4)
/* 8025D018 00259F78  54 00 CF FF */	rlwinm. r0, r0, 0x19, 0x1f, 0x1f
/* 8025D01C 00259F7C  41 82 00 54 */	beq lbl_8025D070
/* 8025D020 00259F80  88 04 01 40 */	lbz r0, 0x140(r4)
/* 8025D024 00259F84  98 03 00 18 */	stb r0, 0x18(r3)
/* 8025D028 00259F88  88 04 01 40 */	lbz r0, 0x140(r4)
/* 8025D02C 00259F8C  28 00 00 00 */	cmplwi r0, 0
/* 8025D030 00259F90  4D 82 00 20 */	beqlr
/* 8025D034 00259F94  28 03 00 00 */	cmplwi r3, 0
/* 8025D038 00259F98  4D 82 00 20 */	beqlr
/* 8025D03C 00259F9C  C0 04 01 28 */	lfs f0, 0x128(r4)
/* 8025D040 00259FA0  D0 03 00 00 */	stfs f0, 0(r3)
/* 8025D044 00259FA4  C0 04 01 2C */	lfs f0, 0x12c(r4)
/* 8025D048 00259FA8  D0 03 00 04 */	stfs f0, 4(r3)
/* 8025D04C 00259FAC  C0 04 01 30 */	lfs f0, 0x130(r4)
/* 8025D050 00259FB0  D0 03 00 08 */	stfs f0, 8(r3)
/* 8025D054 00259FB4  C0 04 01 34 */	lfs f0, 0x134(r4)
/* 8025D058 00259FB8  D0 03 00 0C */	stfs f0, 0xc(r3)
/* 8025D05C 00259FBC  C0 04 01 38 */	lfs f0, 0x138(r4)
/* 8025D060 00259FC0  D0 03 00 10 */	stfs f0, 0x10(r3)
/* 8025D064 00259FC4  C0 04 01 3C */	lfs f0, 0x13c(r4)
/* 8025D068 00259FC8  D0 03 00 14 */	stfs f0, 0x14(r3)
/* 8025D06C 00259FCC  4E 80 00 20 */	blr
lbl_8025D070:
/* 8025D070 00259FD0  38 00 00 00 */	li r0, 0
/* 8025D074 00259FD4  98 03 00 18 */	stb r0, 0x18(r3)
/* 8025D078 00259FD8  4E 80 00 20 */	blr

.global Touch__5CFireFR6CActorR13CStateManager
Touch__5CFireFR6CActorR13CStateManager:
/* 8025D07C 00259FDC  94 21 FF B0 */	stwu r1, -0x50(r1)
/* 8025D080 00259FE0  7C 08 02 A6 */	mflr r0
/* 8025D084 00259FE4  90 01 00 54 */	stw r0, 0x54(r1)
/* 8025D088 00259FE8  93 E1 00 4C */	stw r31, 0x4c(r1)
/* 8025D08C 00259FEC  7C BF 2B 78 */	mr r31, r5
/* 8025D090 00259FF0  93 C1 00 48 */	stw r30, 0x48(r1)
/* 8025D094 00259FF4  7C 9E 23 78 */	mr r30, r4
/* 8025D098 00259FF8  93 A1 00 44 */	stw r29, 0x44(r1)
/* 8025D09C 00259FFC  7C 7D 1B 78 */	mr r29, r3
/* 8025D0A0 0025A000  A0 64 00 08 */	lhz r3, 8(r4)
/* 8025D0A4 0025A004  A0 1D 00 EC */	lhz r0, 0xec(r29)
/* 8025D0A8 0025A008  7C 03 00 40 */	cmplw r3, r0
/* 8025D0AC 0025A00C  41 82 00 78 */	beq lbl_8025D124
/* 8025D0B0 0025A010  80 AD 97 D0 */	lwz r5, lbl_805A8390@sda21(r13)
/* 8025D0B4 0025A014  38 60 00 00 */	li r3, 0
/* 8025D0B8 0025A018  38 80 00 01 */	li r4, 1
/* 8025D0BC 0025A01C  48 12 CE 39 */	bl __shl2i
/* 8025D0C0 0025A020  39 40 00 00 */	li r10, 0
/* 8025D0C4 0025A024  38 00 00 03 */	li r0, 3
/* 8025D0C8 0025A028  90 81 00 24 */	stw r4, 0x24(r1)
/* 8025D0CC 0025A02C  3C 80 80 5A */	lis r4, sZeroVector__9CVector3f@ha
/* 8025D0D0 0025A030  39 24 66 A0 */	addi r9, r4, sZeroVector__9CVector3f@l
/* 8025D0D4 0025A034  38 A1 00 14 */	addi r5, r1, 0x14
/* 8025D0D8 0025A038  90 61 00 20 */	stw r3, 0x20(r1)
/* 8025D0DC 0025A03C  38 81 00 1C */	addi r4, r1, 0x1c
/* 8025D0E0 0025A040  7F E3 FB 78 */	mr r3, r31
/* 8025D0E4 0025A044  38 C1 00 0C */	addi r6, r1, 0xc
/* 8025D0E8 0025A048  91 41 00 2C */	stw r10, 0x2c(r1)
/* 8025D0EC 0025A04C  38 FD 01 0C */	addi r7, r29, 0x10c
/* 8025D0F0 0025A050  39 01 00 20 */	addi r8, r1, 0x20
/* 8025D0F4 0025A054  91 41 00 28 */	stw r10, 0x28(r1)
/* 8025D0F8 0025A058  90 01 00 30 */	stw r0, 0x30(r1)
/* 8025D0FC 0025A05C  A0 1D 00 08 */	lhz r0, 8(r29)
/* 8025D100 0025A060  B0 01 00 0C */	sth r0, 0xc(r1)
/* 8025D104 0025A064  A1 5E 00 08 */	lhz r10, 8(r30)
/* 8025D108 0025A068  B0 01 00 08 */	sth r0, 8(r1)
/* 8025D10C 0025A06C  B1 41 00 14 */	sth r10, 0x14(r1)
/* 8025D110 0025A070  A0 1D 00 08 */	lhz r0, 8(r29)
/* 8025D114 0025A074  B1 41 00 10 */	sth r10, 0x10(r1)
/* 8025D118 0025A078  B0 01 00 18 */	sth r0, 0x18(r1)
/* 8025D11C 0025A07C  B0 01 00 1C */	sth r0, 0x1c(r1)
/* 8025D120 0025A080  4B DE CF 81 */	bl ApplyDamage__13CStateManagerF9TUniqueId9TUniqueId9TUniqueIdRC11CDamageInfoRC15CMaterialFilterRC9CVector3f
lbl_8025D124:
/* 8025D124 0025A084  80 01 00 54 */	lwz r0, 0x54(r1)
/* 8025D128 0025A088  83 E1 00 4C */	lwz r31, 0x4c(r1)
/* 8025D12C 0025A08C  83 C1 00 48 */	lwz r30, 0x48(r1)
/* 8025D130 0025A090  83 A1 00 44 */	lwz r29, 0x44(r1)
/* 8025D134 0025A094  7C 08 03 A6 */	mtlr r0
/* 8025D138 0025A098  38 21 00 50 */	addi r1, r1, 0x50
/* 8025D13C 0025A09C  4E 80 00 20 */	blr

.global __dt__5CFireFv
__dt__5CFireFv:
/* 8025D140 0025A0A0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8025D144 0025A0A4  7C 08 02 A6 */	mflr r0
/* 8025D148 0025A0A8  90 01 00 14 */	stw r0, 0x14(r1)
/* 8025D14C 0025A0AC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8025D150 0025A0B0  7C 9F 23 78 */	mr r31, r4
/* 8025D154 0025A0B4  93 C1 00 08 */	stw r30, 8(r1)
/* 8025D158 0025A0B8  7C 7E 1B 79 */	or. r30, r3, r3
/* 8025D15C 0025A0BC  41 82 00 64 */	beq lbl_8025D1C0
/* 8025D160 0025A0C0  3C 60 80 3F */	lis r3, lbl_803E9990@ha
/* 8025D164 0025A0C4  34 1E 01 28 */	addic. r0, r30, 0x128
/* 8025D168 0025A0C8  38 03 99 90 */	addi r0, r3, lbl_803E9990@l
/* 8025D16C 0025A0CC  90 1E 00 00 */	stw r0, 0(r30)
/* 8025D170 0025A0D0  41 82 00 0C */	beq lbl_8025D17C
/* 8025D174 0025A0D4  38 00 00 00 */	li r0, 0
/* 8025D178 0025A0D8  98 1E 01 40 */	stb r0, 0x140(r30)
lbl_8025D17C:
/* 8025D17C 0025A0DC  34 1E 00 E8 */	addic. r0, r30, 0xe8
/* 8025D180 0025A0E0  41 82 00 24 */	beq lbl_8025D1A4
/* 8025D184 0025A0E4  80 7E 00 E8 */	lwz r3, 0xe8(r30)
/* 8025D188 0025A0E8  28 03 00 00 */	cmplwi r3, 0
/* 8025D18C 0025A0EC  41 82 00 18 */	beq lbl_8025D1A4
/* 8025D190 0025A0F0  81 83 00 00 */	lwz r12, 0(r3)
/* 8025D194 0025A0F4  38 80 00 01 */	li r4, 1
/* 8025D198 0025A0F8  81 8C 00 08 */	lwz r12, 8(r12)
/* 8025D19C 0025A0FC  7D 89 03 A6 */	mtctr r12
/* 8025D1A0 0025A100  4E 80 04 21 */	bctrl
lbl_8025D1A4:
/* 8025D1A4 0025A104  7F C3 F3 78 */	mr r3, r30
/* 8025D1A8 0025A108  38 80 00 00 */	li r4, 0
/* 8025D1AC 0025A10C  4B DF 85 45 */	bl __dt__6CActorFv
/* 8025D1B0 0025A110  7F E0 07 35 */	extsh. r0, r31
/* 8025D1B4 0025A114  40 81 00 0C */	ble lbl_8025D1C0
/* 8025D1B8 0025A118  7F C3 F3 78 */	mr r3, r30
/* 8025D1BC 0025A11C  48 0B 87 75 */	bl Free__7CMemoryFPCv
lbl_8025D1C0:
/* 8025D1C0 0025A120  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8025D1C4 0025A124  7F C3 F3 78 */	mr r3, r30
/* 8025D1C8 0025A128  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8025D1CC 0025A12C  83 C1 00 08 */	lwz r30, 8(r1)
/* 8025D1D0 0025A130  7C 08 03 A6 */	mtlr r0
/* 8025D1D4 0025A134  38 21 00 10 */	addi r1, r1, 0x10
/* 8025D1D8 0025A138  4E 80 00 20 */	blr

.global "__ct__5CFireF25TToken<15CGenDescription>9TUniqueId7TAreaIdb9TUniqueIdRC12CTransform4fRC11CDamageInfoRC6CAABoxRC9CVector3fbUibbbffff"
"__ct__5CFireF25TToken<15CGenDescription>9TUniqueId7TAreaIdb9TUniqueIdRC12CTransform4fRC11CDamageInfoRC6CAABoxRC9CVector3fbUibbbffff":
/* 8025D1DC 0025A13C  94 21 FE 50 */	stwu r1, -0x1b0(r1)
/* 8025D1E0 0025A140  7C 08 02 A6 */	mflr r0
/* 8025D1E4 0025A144  90 01 01 B4 */	stw r0, 0x1b4(r1)
/* 8025D1E8 0025A148  DB E1 01 A0 */	stfd f31, 0x1a0(r1)
/* 8025D1EC 0025A14C  F3 E1 01 A8 */	psq_st f31, 424(r1), 0, qr0
/* 8025D1F0 0025A150  DB C1 01 90 */	stfd f30, 0x190(r1)
/* 8025D1F4 0025A154  F3 C1 01 98 */	psq_st f30, 408(r1), 0, qr0
/* 8025D1F8 0025A158  DB A1 01 80 */	stfd f29, 0x180(r1)
/* 8025D1FC 0025A15C  F3 A1 01 88 */	psq_st f29, 392(r1), 0, qr0
/* 8025D200 0025A160  DB 81 01 70 */	stfd f28, 0x170(r1)
/* 8025D204 0025A164  F3 81 01 78 */	psq_st f28, 376(r1), 0, qr0
/* 8025D208 0025A168  BE 21 01 34 */	stmw r17, 0x134(r1)
/* 8025D20C 0025A16C  3D 60 80 3D */	lis r11, lbl_803D5230@ha
/* 8025D210 0025A170  FF 80 08 90 */	fmr f28, f1
/* 8025D214 0025A174  FF A0 10 90 */	fmr f29, f2
/* 8025D218 0025A178  38 0B 52 30 */	addi r0, r11, lbl_803D5230@l
/* 8025D21C 0025A17C  7C 75 1B 78 */	mr r21, r3
/* 8025D220 0025A180  7C 96 23 78 */	mr r22, r4
/* 8025D224 0025A184  7C B3 2B 78 */	mr r19, r5
/* 8025D228 0025A188  7C D1 33 78 */	mr r17, r6
/* 8025D22C 0025A18C  FF C0 18 90 */	fmr f30, f3
/* 8025D230 0025A190  83 41 01 B8 */	lwz r26, 0x1b8(r1)
/* 8025D234 0025A194  FF E0 20 90 */	fmr f31, f4
/* 8025D238 0025A198  83 61 01 BC */	lwz r27, 0x1bc(r1)
/* 8025D23C 0025A19C  8B 81 01 C3 */	lbz r28, 0x1c3(r1)
/* 8025D240 0025A1A0  7C F2 3B 78 */	mr r18, r7
/* 8025D244 0025A1A4  83 A1 01 C4 */	lwz r29, 0x1c4(r1)
/* 8025D248 0025A1A8  7D 17 43 78 */	mr r23, r8
/* 8025D24C 0025A1AC  8B C1 01 CB */	lbz r30, 0x1cb(r1)
/* 8025D250 0025A1B0  7D 38 4B 78 */	mr r24, r9
/* 8025D254 0025A1B4  8B E1 01 CF */	lbz r31, 0x1cf(r1)
/* 8025D258 0025A1B8  7D 59 53 78 */	mr r25, r10
/* 8025D25C 0025A1BC  8A 81 01 D3 */	lbz r20, 0x1d3(r1)
/* 8025D260 0025A1C0  7C 04 03 78 */	mr r4, r0
/* 8025D264 0025A1C4  38 61 00 54 */	addi r3, r1, 0x54
/* 8025D268 0025A1C8  38 C1 00 10 */	addi r6, r1, 0x10
/* 8025D26C 0025A1CC  38 A0 FF FF */	li r5, -1
/* 8025D270 0025A1D0  48 0E 0F 25 */	bl "__ct__Q24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>FPCciRCQ24rstl17rmemory_allocator"
/* 8025D274 0025A1D4  80 8D A3 88 */	lwz r4, kInvalidEditorId@sda21(r13)
/* 8025D278 0025A1D8  3C 60 80 57 */	lis r3, NullConnectionList__7CEntity@ha
/* 8025D27C 0025A1DC  80 11 00 00 */	lwz r0, 0(r17)
/* 8025D280 0025A1E0  38 A3 D4 10 */	addi r5, r3, NullConnectionList__7CEntity@l
/* 8025D284 0025A1E4  90 81 00 2C */	stw r4, 0x2c(r1)
/* 8025D288 0025A1E8  38 61 00 64 */	addi r3, r1, 0x64
/* 8025D28C 0025A1EC  38 81 00 30 */	addi r4, r1, 0x30
/* 8025D290 0025A1F0  38 C1 00 2C */	addi r6, r1, 0x2c
/* 8025D294 0025A1F4  90 01 00 30 */	stw r0, 0x30(r1)
/* 8025D298 0025A1F8  4B DF 41 ED */	bl "__ct__11CEntityInfoF7TAreaIdRCQ24rstl48vector<11SConnection,Q24rstl17rmemory_allocator>9TEditorId"
/* 8025D29C 0025A1FC  38 61 00 E4 */	addi r3, r1, 0xe4
/* 8025D2A0 0025A200  4B DD D5 05 */	bl CModelDataNull__10CModelDataFv
/* 8025D2A4 0025A204  38 61 00 7C */	addi r3, r1, 0x7c
/* 8025D2A8 0025A208  4B DB DB 91 */	bl None__16CActorParametersFv
/* 8025D2AC 0025A20C  A0 8D A3 8C */	lhz r4, kInvalidUniqueId@sda21(r13)
/* 8025D2B0 0025A210  38 00 00 00 */	li r0, 0
/* 8025D2B4 0025A214  80 AD 97 D4 */	lwz r5, lbl_805A8394@sda21(r13)
/* 8025D2B8 0025A218  38 60 00 00 */	li r3, 0
/* 8025D2BC 0025A21C  B0 81 00 14 */	sth r4, 0x14(r1)
/* 8025D2C0 0025A220  38 80 00 01 */	li r4, 1
/* 8025D2C4 0025A224  90 01 00 44 */	stw r0, 0x44(r1)
/* 8025D2C8 0025A228  90 01 00 40 */	stw r0, 0x40(r1)
/* 8025D2CC 0025A22C  48 12 CC 29 */	bl __shl2i
/* 8025D2D0 0025A230  80 A1 00 40 */	lwz r5, 0x40(r1)
/* 8025D2D4 0025A234  39 61 00 7C */	addi r11, r1, 0x7c
/* 8025D2D8 0025A238  80 C1 00 44 */	lwz r6, 0x44(r1)
/* 8025D2DC 0025A23C  38 01 00 14 */	addi r0, r1, 0x14
/* 8025D2E0 0025A240  7C A3 1B 78 */	or r3, r5, r3
/* 8025D2E4 0025A244  A0 F3 00 00 */	lhz r7, 0(r19)
/* 8025D2E8 0025A248  7C C4 23 78 */	or r4, r6, r4
/* 8025D2EC 0025A24C  90 61 00 40 */	stw r3, 0x40(r1)
/* 8025D2F0 0025A250  7E A3 AB 78 */	mr r3, r21
/* 8025D2F4 0025A254  7E 45 93 78 */	mr r5, r18
/* 8025D2F8 0025A258  90 81 00 44 */	stw r4, 0x44(r1)
/* 8025D2FC 0025A25C  7F 08 C3 78 */	mr r8, r24
/* 8025D300 0025A260  38 81 00 18 */	addi r4, r1, 0x18
/* 8025D304 0025A264  38 C1 00 54 */	addi r6, r1, 0x54
/* 8025D308 0025A268  B0 E1 00 18 */	sth r7, 0x18(r1)
/* 8025D30C 0025A26C  38 E1 00 64 */	addi r7, r1, 0x64
/* 8025D310 0025A270  39 21 00 E4 */	addi r9, r1, 0xe4
/* 8025D314 0025A274  39 41 00 40 */	addi r10, r1, 0x40
/* 8025D318 0025A278  91 61 00 08 */	stw r11, 8(r1)
/* 8025D31C 0025A27C  90 01 00 0C */	stw r0, 0xc(r1)
/* 8025D320 0025A280  4B DF 85 01 */	bl "__ct__6CActorF9TUniqueIdbRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC11CEntityInfoRC12CTransform4fRC10CModelDataRC13CMaterialListRC16CActorParameters9TUniqueId"
/* 8025D324 0025A284  38 61 00 7C */	addi r3, r1, 0x7c
/* 8025D328 0025A288  38 80 FF FF */	li r4, -1
/* 8025D32C 0025A28C  4B EA 92 DD */	bl __dt__16CLightParametersFv
/* 8025D330 0025A290  38 61 00 E4 */	addi r3, r1, 0xe4
/* 8025D334 0025A294  38 80 FF FF */	li r4, -1
/* 8025D338 0025A298  4B EB 97 15 */	bl __dt__10CModelDataFv
/* 8025D33C 0025A29C  80 01 00 6C */	lwz r0, 0x6c(r1)
/* 8025D340 0025A2A0  80 61 00 74 */	lwz r3, 0x74(r1)
/* 8025D344 0025A2A4  1C 00 00 0C */	mulli r0, r0, 0xc
/* 8025D348 0025A2A8  90 61 00 20 */	stw r3, 0x20(r1)
/* 8025D34C 0025A2AC  7C 64 1B 78 */	mr r4, r3
/* 8025D350 0025A2B0  7C 03 02 14 */	add r0, r3, r0
/* 8025D354 0025A2B4  90 61 00 1C */	stw r3, 0x1c(r1)
/* 8025D358 0025A2B8  90 01 00 28 */	stw r0, 0x28(r1)
/* 8025D35C 0025A2BC  90 01 00 24 */	stw r0, 0x24(r1)
/* 8025D360 0025A2C0  48 00 00 08 */	b lbl_8025D368
lbl_8025D364:
/* 8025D364 0025A2C4  38 84 00 0C */	addi r4, r4, 0xc
lbl_8025D368:
/* 8025D368 0025A2C8  7C 04 00 40 */	cmplw r4, r0
/* 8025D36C 0025A2CC  40 82 FF F8 */	bne lbl_8025D364
/* 8025D370 0025A2D0  28 03 00 00 */	cmplwi r3, 0
/* 8025D374 0025A2D4  41 82 00 08 */	beq lbl_8025D37C
/* 8025D378 0025A2D8  48 0B 85 B9 */	bl Free__7CMemoryFPCv
lbl_8025D37C:
/* 8025D37C 0025A2DC  38 61 00 54 */	addi r3, r1, 0x54
/* 8025D380 0025A2E0  48 0E 07 61 */	bl "internal_dereference__Q24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>Fv"
/* 8025D384 0025A2E4  3C 80 80 3F */	lis r4, lbl_803E9990@ha
/* 8025D388 0025A2E8  3C 60 80 3D */	lis r3, lbl_803D5230@ha
/* 8025D38C 0025A2EC  38 04 99 90 */	addi r0, r4, lbl_803E9990@l
/* 8025D390 0025A2F0  3A 20 00 00 */	li r17, 0
/* 8025D394 0025A2F4  38 63 52 30 */	addi r3, r3, lbl_803D5230@l
/* 8025D398 0025A2F8  90 15 00 00 */	stw r0, 0(r21)
/* 8025D39C 0025A2FC  38 83 00 05 */	addi r4, r3, 5
/* 8025D3A0 0025A300  38 60 03 40 */	li r3, 0x340
/* 8025D3A4 0025A304  38 A0 00 00 */	li r5, 0
/* 8025D3A8 0025A308  48 0B 84 C5 */	bl __nw__FUlPCcPCc
/* 8025D3AC 0025A30C  7C 72 1B 79 */	or. r18, r3, r3
/* 8025D3B0 0025A310  41 82 00 2C */	beq lbl_8025D3DC
/* 8025D3B4 0025A314  7E C4 B3 78 */	mr r4, r22
/* 8025D3B8 0025A318  38 61 00 34 */	addi r3, r1, 0x34
/* 8025D3BC 0025A31C  48 0E 3A ED */	bl __ct__6CTokenFRC6CToken
/* 8025D3C0 0025A320  7E 43 93 78 */	mr r3, r18
/* 8025D3C4 0025A324  38 81 00 34 */	addi r4, r1, 0x34
/* 8025D3C8 0025A328  3A 20 00 01 */	li r17, 1
/* 8025D3CC 0025A32C  38 A0 00 00 */	li r5, 0
/* 8025D3D0 0025A330  38 C0 00 01 */	li r6, 1
/* 8025D3D4 0025A334  48 0C 20 0D */	bl "__ct__11CElementGenF25TToken<15CGenDescription>Q211CElementGen21EModelOrientationTypeQ211CElementGen20EOptionalSystemFlags"
/* 8025D3D8 0025A338  7C 72 1B 78 */	mr r18, r3
lbl_8025D3DC:
/* 8025D3DC 0025A33C  7E 20 07 75 */	extsb. r0, r17
/* 8025D3E0 0025A340  92 55 00 E8 */	stw r18, 0xe8(r21)
/* 8025D3E4 0025A344  41 82 00 10 */	beq lbl_8025D3F4
/* 8025D3E8 0025A348  38 61 00 34 */	addi r3, r1, 0x34
/* 8025D3EC 0025A34C  38 80 00 00 */	li r4, 0
/* 8025D3F0 0025A350  48 0E 3A 51 */	bl __dt__6CTokenFv
lbl_8025D3F4:
/* 8025D3F4 0025A354  A0 77 00 00 */	lhz r3, 0(r23)
/* 8025D3F8 0025A358  34 95 01 28 */	addic. r4, r21, 0x128
/* 8025D3FC 0025A35C  38 00 00 01 */	li r0, 1
/* 8025D400 0025A360  B0 75 00 EC */	sth r3, 0xec(r21)
/* 8025D404 0025A364  80 79 00 00 */	lwz r3, 0(r25)
/* 8025D408 0025A368  90 75 00 F0 */	stw r3, 0xf0(r21)
/* 8025D40C 0025A36C  88 79 00 04 */	lbz r3, 4(r25)
/* 8025D410 0025A370  98 75 00 F4 */	stb r3, 0xf4(r21)
/* 8025D414 0025A374  C0 19 00 08 */	lfs f0, 8(r25)
/* 8025D418 0025A378  D0 15 00 F8 */	stfs f0, 0xf8(r21)
/* 8025D41C 0025A37C  C0 19 00 0C */	lfs f0, 0xc(r25)
/* 8025D420 0025A380  D0 15 00 FC */	stfs f0, 0xfc(r21)
/* 8025D424 0025A384  C0 19 00 10 */	lfs f0, 0x10(r25)
/* 8025D428 0025A388  D0 15 01 00 */	stfs f0, 0x100(r21)
/* 8025D42C 0025A38C  C0 19 00 14 */	lfs f0, 0x14(r25)
/* 8025D430 0025A390  D0 15 01 04 */	stfs f0, 0x104(r21)
/* 8025D434 0025A394  88 79 00 18 */	lbz r3, 0x18(r25)
/* 8025D438 0025A398  98 75 01 08 */	stb r3, 0x108(r21)
/* 8025D43C 0025A39C  80 79 00 00 */	lwz r3, 0(r25)
/* 8025D440 0025A3A0  90 75 01 0C */	stw r3, 0x10c(r21)
/* 8025D444 0025A3A4  88 79 00 04 */	lbz r3, 4(r25)
/* 8025D448 0025A3A8  98 75 01 10 */	stb r3, 0x110(r21)
/* 8025D44C 0025A3AC  C0 19 00 08 */	lfs f0, 8(r25)
/* 8025D450 0025A3B0  D0 15 01 14 */	stfs f0, 0x114(r21)
/* 8025D454 0025A3B4  C0 19 00 0C */	lfs f0, 0xc(r25)
/* 8025D458 0025A3B8  D0 15 01 18 */	stfs f0, 0x118(r21)
/* 8025D45C 0025A3BC  C0 19 00 10 */	lfs f0, 0x10(r25)
/* 8025D460 0025A3C0  D0 15 01 1C */	stfs f0, 0x11c(r21)
/* 8025D464 0025A3C4  C0 19 00 14 */	lfs f0, 0x14(r25)
/* 8025D468 0025A3C8  D0 15 01 20 */	stfs f0, 0x120(r21)
/* 8025D46C 0025A3CC  88 79 00 18 */	lbz r3, 0x18(r25)
/* 8025D470 0025A3D0  98 75 01 24 */	stb r3, 0x124(r21)
/* 8025D474 0025A3D4  98 15 01 40 */	stb r0, 0x140(r21)
/* 8025D478 0025A3D8  41 82 00 34 */	beq lbl_8025D4AC
/* 8025D47C 0025A3DC  C0 1A 00 00 */	lfs f0, 0(r26)
/* 8025D480 0025A3E0  D0 04 00 00 */	stfs f0, 0(r4)
/* 8025D484 0025A3E4  C0 1A 00 04 */	lfs f0, 4(r26)
/* 8025D488 0025A3E8  D0 04 00 04 */	stfs f0, 4(r4)
/* 8025D48C 0025A3EC  C0 1A 00 08 */	lfs f0, 8(r26)
/* 8025D490 0025A3F0  D0 04 00 08 */	stfs f0, 8(r4)
/* 8025D494 0025A3F4  C0 1A 00 0C */	lfs f0, 0xc(r26)
/* 8025D498 0025A3F8  D0 04 00 0C */	stfs f0, 0xc(r4)
/* 8025D49C 0025A3FC  C0 1A 00 10 */	lfs f0, 0x10(r26)
/* 8025D4A0 0025A400  D0 04 00 10 */	stfs f0, 0x10(r4)
/* 8025D4A4 0025A404  C0 1A 00 14 */	lfs f0, 0x14(r26)
/* 8025D4A8 0025A408  D0 04 00 14 */	stfs f0, 0x14(r4)
lbl_8025D4AC:
/* 8025D4AC 0025A40C  D3 95 01 44 */	stfs f28, 0x144(r21)
/* 8025D4B0 0025A410  28 14 00 00 */	cmplwi r20, 0
/* 8025D4B4 0025A414  38 60 00 00 */	li r3, 0
/* 8025D4B8 0025A418  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D4BC 0025A41C  53 C0 3E 30 */	rlwimi r0, r30, 7, 0x18, 0x18
/* 8025D4C0 0025A420  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D4C4 0025A424  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D4C8 0025A428  53 E0 36 72 */	rlwimi r0, r31, 6, 0x19, 0x19
/* 8025D4CC 0025A42C  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D4D0 0025A430  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D4D4 0025A434  52 80 2E B4 */	rlwimi r0, r20, 5, 0x1a, 0x1a
/* 8025D4D8 0025A438  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D4DC 0025A43C  41 82 00 18 */	beq lbl_8025D4F4
/* 8025D4E0 0025A440  28 1F 00 00 */	cmplwi r31, 0
/* 8025D4E4 0025A444  41 82 00 10 */	beq lbl_8025D4F4
/* 8025D4E8 0025A448  28 1E 00 00 */	cmplwi r30, 0
/* 8025D4EC 0025A44C  41 82 00 08 */	beq lbl_8025D4F4
/* 8025D4F0 0025A450  38 60 00 01 */	li r3, 1
lbl_8025D4F4:
/* 8025D4F4 0025A454  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D4F8 0025A458  50 60 26 F6 */	rlwimi r0, r3, 4, 0x1b, 0x1b
/* 8025D4FC 0025A45C  38 60 00 00 */	li r3, 0
/* 8025D500 0025A460  C0 02 B9 F8 */	lfs f0, lbl_805AD718@sda21(r2)
/* 8025D504 0025A464  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D508 0025A468  7F 64 DB 78 */	mr r4, r27
/* 8025D50C 0025A46C  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D510 0025A470  50 60 1F 38 */	rlwimi r0, r3, 3, 0x1c, 0x1c
/* 8025D514 0025A474  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D518 0025A478  88 15 01 48 */	lbz r0, 0x148(r21)
/* 8025D51C 0025A47C  53 80 17 7A */	rlwimi r0, r28, 2, 0x1d, 0x1d
/* 8025D520 0025A480  98 15 01 48 */	stb r0, 0x148(r21)
/* 8025D524 0025A484  D3 B5 01 4C */	stfs f29, 0x14c(r21)
/* 8025D528 0025A488  93 B5 01 50 */	stw r29, 0x150(r21)
/* 8025D52C 0025A48C  D3 D5 01 54 */	stfs f30, 0x154(r21)
/* 8025D530 0025A490  D3 F5 01 58 */	stfs f31, 0x158(r21)
/* 8025D534 0025A494  D0 15 01 5C */	stfs f0, 0x15c(r21)
/* 8025D538 0025A498  80 75 00 E8 */	lwz r3, 0xe8(r21)
/* 8025D53C 0025A49C  81 83 00 00 */	lwz r12, 0(r3)
/* 8025D540 0025A4A0  81 8C 00 24 */	lwz r12, 0x24(r12)
/* 8025D544 0025A4A4  7D 89 03 A6 */	mtctr r12
/* 8025D548 0025A4A8  4E 80 04 21 */	bctrl
/* 8025D54C 0025A4AC  C0 58 00 2C */	lfs f2, 0x2c(r24)
/* 8025D550 0025A4B0  38 81 00 48 */	addi r4, r1, 0x48
/* 8025D554 0025A4B4  C0 38 00 1C */	lfs f1, 0x1c(r24)
/* 8025D558 0025A4B8  C0 18 00 0C */	lfs f0, 0xc(r24)
/* 8025D55C 0025A4BC  D0 21 00 4C */	stfs f1, 0x4c(r1)
/* 8025D560 0025A4C0  D0 01 00 48 */	stfs f0, 0x48(r1)
/* 8025D564 0025A4C4  D0 41 00 50 */	stfs f2, 0x50(r1)
/* 8025D568 0025A4C8  80 75 00 E8 */	lwz r3, 0xe8(r21)
/* 8025D56C 0025A4CC  81 83 00 00 */	lwz r12, 0(r3)
/* 8025D570 0025A4D0  81 8C 00 18 */	lwz r12, 0x18(r12)
/* 8025D574 0025A4D4  7D 89 03 A6 */	mtctr r12
/* 8025D578 0025A4D8  4E 80 04 21 */	bctrl
/* 8025D57C 0025A4DC  7E A3 AB 78 */	mr r3, r21
/* 8025D580 0025A4E0  E3 E1 01 A8 */	psq_l f31, 424(r1), 0, qr0
/* 8025D584 0025A4E4  CB E1 01 A0 */	lfd f31, 0x1a0(r1)
/* 8025D588 0025A4E8  E3 C1 01 98 */	psq_l f30, 408(r1), 0, qr0
/* 8025D58C 0025A4EC  CB C1 01 90 */	lfd f30, 0x190(r1)
/* 8025D590 0025A4F0  E3 A1 01 88 */	psq_l f29, 392(r1), 0, qr0
/* 8025D594 0025A4F4  CB A1 01 80 */	lfd f29, 0x180(r1)
/* 8025D598 0025A4F8  E3 81 01 78 */	psq_l f28, 376(r1), 0, qr0
/* 8025D59C 0025A4FC  CB 81 01 70 */	lfd f28, 0x170(r1)
/* 8025D5A0 0025A500  BA 21 01 34 */	lmw r17, 0x134(r1)
/* 8025D5A4 0025A504  80 01 01 B4 */	lwz r0, 0x1b4(r1)
/* 8025D5A8 0025A508  7C 08 03 A6 */	mtlr r0
/* 8025D5AC 0025A50C  38 21 01 B0 */	addi r1, r1, 0x1b0
/* 8025D5B0 0025A510  4E 80 00 20 */	blr

.section .sdata2, "a"
.balign 8
.global lbl_805AD718
lbl_805AD718:
	# ROM: 0x3F9FB8
	.4byte 0

.global lbl_805AD71C
lbl_805AD71C:
	# ROM: 0x3F9FBC
	.float 0.5

.global lbl_805AD720
lbl_805AD720:
	# ROM: 0x3F9FC0
	.float 1.0

.global lbl_805AD724
lbl_805AD724:
	# ROM: 0x3F9FC4
	.4byte 0x42340000

.global lbl_805AD728
lbl_805AD728:
	# ROM: 0x3F9FC8
	.double 4.503601774854144E15


.section .rodata
.balign 8
.global lbl_803D5230
lbl_803D5230:
	# ROM: 0x3D2230
	.asciz "Fire"
	.byte 0x3F, 0x3F, 0x28
	.4byte 0x3F3F2900
	.4byte 0
