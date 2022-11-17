.include "macros.inc"

.section .sdata
.balign 8

.global lbl_805A7578
lbl_805A7578:
	# ROM: 0x3F4F18
	.4byte 0x0000001A

.global lbl_805A757C
lbl_805A757C:
	# ROM: 0x3F4F1C
	.4byte 0x00000031

.section .sdata2, "a"
.balign 8

.global lbl_805AB170
lbl_805AB170:
	# ROM: 0x3F7A10
	.float 255.0

.global lbl_805AB174
lbl_805AB174:
	# ROM: 0x3F7A14
	.float 0.0

.global lbl_805AB178
lbl_805AB178:
	# ROM: 0x3F7A18
	.float 1.0

.global lbl_805AB17C
lbl_805AB17C:
	# ROM: 0x3F7A1C
	.float 0.5

.global lbl_805AB180
lbl_805AB180:
	# ROM: 0x3F7A20
	.float -1.0

.global lbl_805AB184
lbl_805AB184:
	# ROM: 0x3F7A24
	.float 0.1

.global lbl_805AB188
lbl_805AB188:
	# ROM: 0x3F7A28
	.double 0.5

.global lbl_805AB190
lbl_805AB190:
	# ROM: 0x3F7A30
	.double 3.0

.section .text, "ax"

.global Valid__13CSimpleShadowCFv
Valid__13CSimpleShadowCFv:
/* 80105B00 00102A60  88 03 00 48 */	lbz r0, 0x48(r3)
/* 80105B04 00102A64  54 03 CF FE */	rlwinm r3, r0, 0x19, 0x1f, 0x1f
/* 80105B08 00102A68  4E 80 00 20 */	blr

.global GetMaxShadowBox__13CSimpleShadowCFRC6CAABox
GetMaxShadowBox__13CSimpleShadowCFRC6CAABox:
/* 80105B0C 00102A6C  94 21 FF 60 */	stwu r1, -0xa0(r1)
/* 80105B10 00102A70  7C 08 02 A6 */	mflr r0
/* 80105B14 00102A74  90 01 00 A4 */	stw r0, 0xa4(r1)
/* 80105B18 00102A78  DB E1 00 90 */	stfd f31, 0x90(r1)
/* 80105B1C 00102A7C  F3 E1 00 98 */	psq_st f31, 152(r1), 0, qr0
/* 80105B20 00102A80  DB C1 00 80 */	stfd f30, 0x80(r1)
/* 80105B24 00102A84  F3 C1 00 88 */	psq_st f30, 136(r1), 0, qr0
/* 80105B28 00102A88  DB A1 00 70 */	stfd f29, 0x70(r1)
/* 80105B2C 00102A8C  F3 A1 00 78 */	psq_st f29, 120(r1), 0, qr0
/* 80105B30 00102A90  DB 81 00 60 */	stfd f28, 0x60(r1)
/* 80105B34 00102A94  F3 81 00 68 */	psq_st f28, 104(r1), 0, qr0
/* 80105B38 00102A98  93 E1 00 5C */	stw r31, 0x5c(r1)
/* 80105B3C 00102A9C  93 C1 00 58 */	stw r30, 0x58(r1)
/* 80105B40 00102AA0  93 A1 00 54 */	stw r29, 0x54(r1)
/* 80105B44 00102AA4  7C 9E 23 78 */	mr r30, r4
/* 80105B48 00102AA8  7C BF 2B 78 */	mr r31, r5
/* 80105B4C 00102AAC  C0 24 00 34 */	lfs f1, 0x34(r4)
/* 80105B50 00102AB0  7C 7D 1B 78 */	mr r29, r3
/* 80105B54 00102AB4  C0 04 00 30 */	lfs f0, 0x30(r4)
/* 80105B58 00102AB8  7F E4 FB 78 */	mr r4, r31
/* 80105B5C 00102ABC  38 61 00 20 */	addi r3, r1, 0x20
/* 80105B60 00102AC0  EF 81 00 32 */	fmuls f28, f1, f0
/* 80105B64 00102AC4  48 23 1A A9 */	bl GetCenterPoint__6CAABoxCFv
/* 80105B68 00102AC8  C0 BF 00 00 */	lfs f5, 0(r31)
/* 80105B6C 00102ACC  7F C3 F3 78 */	mr r3, r30
/* 80105B70 00102AD0  C0 9F 00 04 */	lfs f4, 4(r31)
/* 80105B74 00102AD4  C0 7F 00 08 */	lfs f3, 8(r31)
/* 80105B78 00102AD8  C0 5F 00 0C */	lfs f2, 0xc(r31)
/* 80105B7C 00102ADC  C0 3F 00 10 */	lfs f1, 0x10(r31)
/* 80105B80 00102AE0  C0 1F 00 14 */	lfs f0, 0x14(r31)
/* 80105B84 00102AE4  D0 A1 00 2C */	stfs f5, 0x2c(r1)
/* 80105B88 00102AE8  C3 E1 00 20 */	lfs f31, 0x20(r1)
/* 80105B8C 00102AEC  D0 81 00 30 */	stfs f4, 0x30(r1)
/* 80105B90 00102AF0  C3 C1 00 24 */	lfs f30, 0x24(r1)
/* 80105B94 00102AF4  D0 61 00 34 */	stfs f3, 0x34(r1)
/* 80105B98 00102AF8  C3 A1 00 28 */	lfs f29, 0x28(r1)
/* 80105B9C 00102AFC  D0 41 00 38 */	stfs f2, 0x38(r1)
/* 80105BA0 00102B00  D0 21 00 3C */	stfs f1, 0x3c(r1)
/* 80105BA4 00102B04  D0 01 00 40 */	stfs f0, 0x40(r1)
/* 80105BA8 00102B08  48 00 01 59 */	bl GetMaxObjectHeight__13CSimpleShadowCFv
/* 80105BAC 00102B0C  FC 00 08 50 */	fneg f0, f1
/* 80105BB0 00102B10  38 61 00 2C */	addi r3, r1, 0x2c
/* 80105BB4 00102B14  EC 5F E0 2A */	fadds f2, f31, f28
/* 80105BB8 00102B18  38 81 00 14 */	addi r4, r1, 0x14
/* 80105BBC 00102B1C  EC 3E E0 2A */	fadds f1, f30, f28
/* 80105BC0 00102B20  EC 1D 00 2A */	fadds f0, f29, f0
/* 80105BC4 00102B24  D0 41 00 14 */	stfs f2, 0x14(r1)
/* 80105BC8 00102B28  D0 21 00 18 */	stfs f1, 0x18(r1)
/* 80105BCC 00102B2C  D0 01 00 1C */	stfs f0, 0x1c(r1)
/* 80105BD0 00102B30  48 23 1F 39 */	bl AccumulateBounds__6CAABoxFRC9CVector3f
/* 80105BD4 00102B34  7F C3 F3 78 */	mr r3, r30
/* 80105BD8 00102B38  48 00 01 29 */	bl GetMaxObjectHeight__13CSimpleShadowCFv
/* 80105BDC 00102B3C  FC 60 E0 50 */	fneg f3, f28
/* 80105BE0 00102B40  38 61 00 2C */	addi r3, r1, 0x2c
/* 80105BE4 00102B44  FC 00 08 50 */	fneg f0, f1
/* 80105BE8 00102B48  38 81 00 08 */	addi r4, r1, 8
/* 80105BEC 00102B4C  EC 5F 18 2A */	fadds f2, f31, f3
/* 80105BF0 00102B50  EC 3E 18 2A */	fadds f1, f30, f3
/* 80105BF4 00102B54  EC 1D 00 2A */	fadds f0, f29, f0
/* 80105BF8 00102B58  D0 41 00 08 */	stfs f2, 8(r1)
/* 80105BFC 00102B5C  D0 21 00 0C */	stfs f1, 0xc(r1)
/* 80105C00 00102B60  D0 01 00 10 */	stfs f0, 0x10(r1)
/* 80105C04 00102B64  48 23 1F 05 */	bl AccumulateBounds__6CAABoxFRC9CVector3f
/* 80105C08 00102B68  C0 01 00 2C */	lfs f0, 0x2c(r1)
/* 80105C0C 00102B6C  D0 1D 00 00 */	stfs f0, 0(r29)
/* 80105C10 00102B70  C0 01 00 30 */	lfs f0, 0x30(r1)
/* 80105C14 00102B74  D0 1D 00 04 */	stfs f0, 4(r29)
/* 80105C18 00102B78  C0 01 00 34 */	lfs f0, 0x34(r1)
/* 80105C1C 00102B7C  D0 1D 00 08 */	stfs f0, 8(r29)
/* 80105C20 00102B80  C0 01 00 38 */	lfs f0, 0x38(r1)
/* 80105C24 00102B84  D0 1D 00 0C */	stfs f0, 0xc(r29)
/* 80105C28 00102B88  C0 01 00 3C */	lfs f0, 0x3c(r1)
/* 80105C2C 00102B8C  D0 1D 00 10 */	stfs f0, 0x10(r29)
/* 80105C30 00102B90  C0 01 00 40 */	lfs f0, 0x40(r1)
/* 80105C34 00102B94  D0 1D 00 14 */	stfs f0, 0x14(r29)
/* 80105C38 00102B98  E3 E1 00 98 */	psq_l f31, 152(r1), 0, qr0
/* 80105C3C 00102B9C  CB E1 00 90 */	lfd f31, 0x90(r1)
/* 80105C40 00102BA0  E3 C1 00 88 */	psq_l f30, 136(r1), 0, qr0
/* 80105C44 00102BA4  CB C1 00 80 */	lfd f30, 0x80(r1)
/* 80105C48 00102BA8  E3 A1 00 78 */	psq_l f29, 120(r1), 0, qr0
/* 80105C4C 00102BAC  CB A1 00 70 */	lfd f29, 0x70(r1)
/* 80105C50 00102BB0  E3 81 00 68 */	psq_l f28, 104(r1), 0, qr0
/* 80105C54 00102BB4  CB 81 00 60 */	lfd f28, 0x60(r1)
/* 80105C58 00102BB8  83 E1 00 5C */	lwz r31, 0x5c(r1)
/* 80105C5C 00102BBC  83 C1 00 58 */	lwz r30, 0x58(r1)
/* 80105C60 00102BC0  80 01 00 A4 */	lwz r0, 0xa4(r1)
/* 80105C64 00102BC4  83 A1 00 54 */	lwz r29, 0x54(r1)
/* 80105C68 00102BC8  7C 08 03 A6 */	mtlr r0
/* 80105C6C 00102BCC  38 21 00 A0 */	addi r1, r1, 0xa0
/* 80105C70 00102BD0  4E 80 00 20 */	blr

.global GetBounds__13CSimpleShadowCFv
GetBounds__13CSimpleShadowCFv:
/* 80105C74 00102BD4  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 80105C78 00102BD8  7C 08 02 A6 */	mflr r0
/* 80105C7C 00102BDC  90 01 00 34 */	stw r0, 0x34(r1)
/* 80105C80 00102BE0  38 A1 00 08 */	addi r5, r1, 8
/* 80105C84 00102BE4  C0 24 00 34 */	lfs f1, 0x34(r4)
/* 80105C88 00102BE8  C0 04 00 30 */	lfs f0, 0x30(r4)
/* 80105C8C 00102BEC  C0 C4 00 2C */	lfs f6, 0x2c(r4)
/* 80105C90 00102BF0  EC E1 00 32 */	fmuls f7, f1, f0
/* 80105C94 00102BF4  C0 04 00 0C */	lfs f0, 0xc(r4)
/* 80105C98 00102BF8  C0 24 00 1C */	lfs f1, 0x1c(r4)
/* 80105C9C 00102BFC  38 81 00 14 */	addi r4, r1, 0x14
/* 80105CA0 00102C00  D0 01 00 20 */	stfs f0, 0x20(r1)
/* 80105CA4 00102C04  EC A1 38 2A */	fadds f5, f1, f7
/* 80105CA8 00102C08  D0 21 00 24 */	stfs f1, 0x24(r1)
/* 80105CAC 00102C0C  EC 86 38 2A */	fadds f4, f6, f7
/* 80105CB0 00102C10  EC 60 38 2A */	fadds f3, f0, f7
/* 80105CB4 00102C14  EC 40 38 28 */	fsubs f2, f0, f7
/* 80105CB8 00102C18  D0 C1 00 28 */	stfs f6, 0x28(r1)
/* 80105CBC 00102C1C  EC 21 38 28 */	fsubs f1, f1, f7
/* 80105CC0 00102C20  EC 06 38 28 */	fsubs f0, f6, f7
/* 80105CC4 00102C24  D0 61 00 08 */	stfs f3, 8(r1)
/* 80105CC8 00102C28  D0 A1 00 0C */	stfs f5, 0xc(r1)
/* 80105CCC 00102C2C  D0 81 00 10 */	stfs f4, 0x10(r1)
/* 80105CD0 00102C30  D0 41 00 14 */	stfs f2, 0x14(r1)
/* 80105CD4 00102C34  D0 21 00 18 */	stfs f1, 0x18(r1)
/* 80105CD8 00102C38  D0 01 00 1C */	stfs f0, 0x1c(r1)
/* 80105CDC 00102C3C  48 23 28 2D */	bl __ct__6CAABoxFRC9CVector3fRC9CVector3f
/* 80105CE0 00102C40  80 01 00 34 */	lwz r0, 0x34(r1)
/* 80105CE4 00102C44  7C 08 03 A6 */	mtlr r0
/* 80105CE8 00102C48  38 21 00 30 */	addi r1, r1, 0x30
/* 80105CEC 00102C4C  4E 80 00 20 */	blr

.global SetAlwaysCalculateRadius__13CSimpleShadowFb
SetAlwaysCalculateRadius__13CSimpleShadowFb:
/* 80105CF0 00102C50  88 03 00 48 */	lbz r0, 0x48(r3)
/* 80105CF4 00102C54  50 80 36 72 */	rlwimi r0, r4, 6, 0x19, 0x19
/* 80105CF8 00102C58  98 03 00 48 */	stb r0, 0x48(r3)
/* 80105CFC 00102C5C  4E 80 00 20 */	blr

.global GetMaxObjectHeight__13CSimpleShadowCFv
GetMaxObjectHeight__13CSimpleShadowCFv:
/* 80105D00 00102C60  C0 23 00 40 */	lfs f1, 0x40(r3)
/* 80105D04 00102C64  4E 80 00 20 */	blr

.global SetUserAlpha__13CSimpleShadowFf
SetUserAlpha__13CSimpleShadowFf:
/* 80105D08 00102C68  D0 23 00 38 */	stfs f1, 0x38(r3)
/* 80105D0C 00102C6C  4E 80 00 20 */	blr

.global GetTransform__13CSimpleShadowCFv
GetTransform__13CSimpleShadowCFv:
/* 80105D10 00102C70  4E 80 00 20 */	blr

.global Render__13CSimpleShadowCFPC8CTexture
Render__13CSimpleShadowCFPC8CTexture:
/* 80105D14 00102C74  94 21 FF 90 */	stwu r1, -0x70(r1)
/* 80105D18 00102C78  7C 08 02 A6 */	mflr r0
/* 80105D1C 00102C7C  90 01 00 74 */	stw r0, 0x74(r1)
/* 80105D20 00102C80  DB E1 00 60 */	stfd f31, 0x60(r1)
/* 80105D24 00102C84  F3 E1 00 68 */	psq_st f31, 104(r1), 0, qr0
/* 80105D28 00102C88  DB C1 00 50 */	stfd f30, 0x50(r1)
/* 80105D2C 00102C8C  F3 C1 00 58 */	psq_st f30, 88(r1), 0, qr0
/* 80105D30 00102C90  93 E1 00 4C */	stw r31, 0x4c(r1)
/* 80105D34 00102C94  93 C1 00 48 */	stw r30, 0x48(r1)
/* 80105D38 00102C98  88 03 00 48 */	lbz r0, 0x48(r3)
/* 80105D3C 00102C9C  7C 7E 1B 78 */	mr r30, r3
/* 80105D40 00102CA0  7C 9F 23 78 */	mr r31, r4
/* 80105D44 00102CA4  54 00 CF FF */	rlwinm. r0, r0, 0x19, 0x1f, 0x1f
/* 80105D48 00102CA8  41 82 01 58 */	beq lbl_80105EA0
/* 80105D4C 00102CAC  48 20 6C 31 */	bl DisableAllLights__9CGraphicsFv
/* 80105D50 00102CB0  80 6D A0 68 */	lwz r3, gpRender@sda21(r13)
/* 80105D54 00102CB4  7F C4 F3 78 */	mr r4, r30
/* 80105D58 00102CB8  81 83 00 00 */	lwz r12, 0(r3)
/* 80105D5C 00102CBC  81 8C 00 30 */	lwz r12, 0x30(r12)
/* 80105D60 00102CC0  7D 89 03 A6 */	mtctr r12
/* 80105D64 00102CC4  4E 80 04 21 */	bctrl
/* 80105D68 00102CC8  7F E3 FB 78 */	mr r3, r31
/* 80105D6C 00102CCC  38 80 00 00 */	li r4, 0
/* 80105D70 00102CD0  38 A0 00 01 */	li r5, 1
/* 80105D74 00102CD4  48 20 99 C9 */	bl Load__8CTextureCF11_GXTexMapIDQ28CTexture10EClampMode
/* 80105D78 00102CD8  3C 80 80 5A */	lis r4, kEnvModulate__9CGraphics@ha
/* 80105D7C 00102CDC  38 60 00 00 */	li r3, 0
/* 80105D80 00102CE0  38 84 5E BC */	addi r4, r4, kEnvModulate__9CGraphics@l
/* 80105D84 00102CE4  48 20 45 35 */	bl SetTevOp__9CGraphicsF12ERglTevStageRCQ213CTevCombiners8CTevPass
/* 80105D88 00102CE8  80 8D 9C 68 */	lwz r4, kEnvPassthru__9CGraphics@sda21(r13)
/* 80105D8C 00102CEC  38 60 00 01 */	li r3, 1
/* 80105D90 00102CF0  48 20 45 29 */	bl SetTevOp__9CGraphicsF12ERglTevStageRCQ213CTevCombiners8CTevPass
/* 80105D94 00102CF4  38 60 00 07 */	li r3, 7
/* 80105D98 00102CF8  38 80 00 00 */	li r4, 0
/* 80105D9C 00102CFC  38 A0 00 00 */	li r5, 0
/* 80105DA0 00102D00  38 C0 00 07 */	li r6, 7
/* 80105DA4 00102D04  38 E0 00 00 */	li r7, 0
/* 80105DA8 00102D08  48 20 5C 81 */	bl SetAlphaCompare__9CGraphicsF13ERglAlphaFuncUc11ERglAlphaOp13ERglAlphaFuncUc
/* 80105DAC 00102D0C  38 60 00 01 */	li r3, 1
/* 80105DB0 00102D10  38 80 00 03 */	li r4, 3
/* 80105DB4 00102D14  38 A0 00 00 */	li r5, 0
/* 80105DB8 00102D18  48 20 5C DD */	bl SetDepthWriteMode__9CGraphicsFb8ERglEnumb
/* 80105DBC 00102D1C  38 60 00 01 */	li r3, 1
/* 80105DC0 00102D20  38 80 00 04 */	li r4, 4
/* 80105DC4 00102D24  38 A0 00 05 */	li r5, 5
/* 80105DC8 00102D28  38 C0 00 00 */	li r6, 0
/* 80105DCC 00102D2C  48 20 5C 85 */	bl SetBlendMode__9CGraphicsF13ERglBlendMode15ERglBlendFactor15ERglBlendFactor11ERglLogicOp
/* 80105DD0 00102D30  C0 3E 00 34 */	lfs f1, 0x34(r30)
/* 80105DD4 00102D34  38 60 00 80 */	li r3, 0x80
/* 80105DD8 00102D38  C0 1E 00 30 */	lfs f0, 0x30(r30)
/* 80105DDC 00102D3C  EF C1 00 32 */	fmuls f30, f1, f0
/* 80105DE0 00102D40  48 20 58 89 */	bl StreamBegin__9CGraphicsF13ERglPrimitive
/* 80105DE4 00102D44  C0 3E 00 3C */	lfs f1, 0x3c(r30)
/* 80105DE8 00102D48  C0 1E 00 38 */	lfs f0, 0x38(r30)
/* 80105DEC 00102D4C  C0 42 94 50 */	lfs f2, lbl_805AB170@sda21(r2)
/* 80105DF0 00102D50  EC 01 00 32 */	fmuls f0, f1, f0
/* 80105DF4 00102D54  EF E2 00 32 */	fmuls f31, f2, f0
/* 80105DF8 00102D58  F3 E1 A0 08 */	psq_st f31, 8(r1), 1, qr2
/* 80105DFC 00102D5C  88 61 00 08 */	lbz r3, 8(r1)
/* 80105E00 00102D60  38 63 FF 00 */	addi r3, r3, -256
/* 80105E04 00102D64  48 20 57 69 */	bl StreamColor__9CGraphicsFUi
/* 80105E08 00102D68  C0 22 94 54 */	lfs f1, lbl_805AB174@sda21(r2)
/* 80105E0C 00102D6C  FC 40 08 90 */	fmr f2, f1
/* 80105E10 00102D70  48 20 56 55 */	bl StreamTexcoord__9CGraphicsFff
/* 80105E14 00102D74  FF E0 F0 50 */	fneg f31, f30
/* 80105E18 00102D78  C0 02 94 54 */	lfs f0, lbl_805AB174@sda21(r2)
/* 80105E1C 00102D7C  38 61 00 30 */	addi r3, r1, 0x30
/* 80105E20 00102D80  D0 01 00 34 */	stfs f0, 0x34(r1)
/* 80105E24 00102D84  D3 E1 00 30 */	stfs f31, 0x30(r1)
/* 80105E28 00102D88  D3 E1 00 38 */	stfs f31, 0x38(r1)
/* 80105E2C 00102D8C  48 20 57 95 */	bl StreamVertex__9CGraphicsFRC9CVector3f
/* 80105E30 00102D90  C0 22 94 54 */	lfs f1, lbl_805AB174@sda21(r2)
/* 80105E34 00102D94  C0 42 94 58 */	lfs f2, lbl_805AB178@sda21(r2)
/* 80105E38 00102D98  48 20 56 2D */	bl StreamTexcoord__9CGraphicsFff
/* 80105E3C 00102D9C  C0 02 94 54 */	lfs f0, lbl_805AB174@sda21(r2)
/* 80105E40 00102DA0  38 61 00 24 */	addi r3, r1, 0x24
/* 80105E44 00102DA4  D3 C1 00 24 */	stfs f30, 0x24(r1)
/* 80105E48 00102DA8  D0 01 00 28 */	stfs f0, 0x28(r1)
/* 80105E4C 00102DAC  D3 E1 00 2C */	stfs f31, 0x2c(r1)
/* 80105E50 00102DB0  48 20 57 71 */	bl StreamVertex__9CGraphicsFRC9CVector3f
/* 80105E54 00102DB4  C0 22 94 58 */	lfs f1, lbl_805AB178@sda21(r2)
/* 80105E58 00102DB8  FC 40 08 90 */	fmr f2, f1
/* 80105E5C 00102DBC  48 20 56 09 */	bl StreamTexcoord__9CGraphicsFff
/* 80105E60 00102DC0  C0 02 94 54 */	lfs f0, lbl_805AB174@sda21(r2)
/* 80105E64 00102DC4  38 61 00 18 */	addi r3, r1, 0x18
/* 80105E68 00102DC8  D3 C1 00 18 */	stfs f30, 0x18(r1)
/* 80105E6C 00102DCC  D0 01 00 1C */	stfs f0, 0x1c(r1)
/* 80105E70 00102DD0  D3 C1 00 20 */	stfs f30, 0x20(r1)
/* 80105E74 00102DD4  48 20 57 4D */	bl StreamVertex__9CGraphicsFRC9CVector3f
/* 80105E78 00102DD8  C0 22 94 58 */	lfs f1, lbl_805AB178@sda21(r2)
/* 80105E7C 00102DDC  C0 42 94 54 */	lfs f2, lbl_805AB174@sda21(r2)
/* 80105E80 00102DE0  48 20 55 E5 */	bl StreamTexcoord__9CGraphicsFff
/* 80105E84 00102DE4  C0 02 94 54 */	lfs f0, lbl_805AB174@sda21(r2)
/* 80105E88 00102DE8  38 61 00 0C */	addi r3, r1, 0xc
/* 80105E8C 00102DEC  D3 E1 00 0C */	stfs f31, 0xc(r1)
/* 80105E90 00102DF0  D0 01 00 10 */	stfs f0, 0x10(r1)
/* 80105E94 00102DF4  D3 C1 00 14 */	stfs f30, 0x14(r1)
/* 80105E98 00102DF8  48 20 57 29 */	bl StreamVertex__9CGraphicsFRC9CVector3f
/* 80105E9C 00102DFC  48 20 55 75 */	bl StreamEnd__9CGraphicsFv
lbl_80105EA0:
/* 80105EA0 00102E00  E3 E1 00 68 */	psq_l f31, 104(r1), 0, qr0
/* 80105EA4 00102E04  CB E1 00 60 */	lfd f31, 0x60(r1)
/* 80105EA8 00102E08  E3 C1 00 58 */	psq_l f30, 88(r1), 0, qr0
/* 80105EAC 00102E0C  CB C1 00 50 */	lfd f30, 0x50(r1)
/* 80105EB0 00102E10  83 E1 00 4C */	lwz r31, 0x4c(r1)
/* 80105EB4 00102E14  80 01 00 74 */	lwz r0, 0x74(r1)
/* 80105EB8 00102E18  83 C1 00 48 */	lwz r30, 0x48(r1)
/* 80105EBC 00102E1C  7C 08 03 A6 */	mtlr r0
/* 80105EC0 00102E20  38 21 00 70 */	addi r1, r1, 0x70
/* 80105EC4 00102E24  4E 80 00 20 */	blr

.global Calculate__13CSimpleShadowFRC6CAABoxRC12CTransform4fRC13CStateManager
Calculate__13CSimpleShadowFRC6CAABoxRC12CTransform4fRC13CStateManager:
/* 80105EC8 00102E28  94 21 F6 50 */	stwu r1, -0x9b0(r1)
/* 80105ECC 00102E2C  7C 08 02 A6 */	mflr r0
/* 80105ED0 00102E30  90 01 09 B4 */	stw r0, 0x9b4(r1)
/* 80105ED4 00102E34  DB E1 09 A0 */	stfd f31, 0x9a0(r1)
/* 80105ED8 00102E38  F3 E1 09 A8 */	psq_st f31, -1624(r1), 0, qr0
/* 80105EDC 00102E3C  DB C1 09 90 */	stfd f30, 0x990(r1)
/* 80105EE0 00102E40  F3 C1 09 98 */	psq_st f30, -1640(r1), 0, qr0
/* 80105EE4 00102E44  DB A1 09 80 */	stfd f29, 0x980(r1)
/* 80105EE8 00102E48  F3 A1 09 88 */	psq_st f29, -1656(r1), 0, qr0
/* 80105EEC 00102E4C  DB 81 09 70 */	stfd f28, 0x970(r1)
/* 80105EF0 00102E50  F3 81 09 78 */	psq_st f28, -1672(r1), 0, qr0
/* 80105EF4 00102E54  93 E1 09 6C */	stw r31, 0x96c(r1)
/* 80105EF8 00102E58  93 C1 09 68 */	stw r30, 0x968(r1)
/* 80105EFC 00102E5C  7C 7E 1B 78 */	mr r30, r3
/* 80105F00 00102E60  38 60 00 00 */	li r3, 0
/* 80105F04 00102E64  88 1E 00 48 */	lbz r0, 0x48(r30)
/* 80105F08 00102E68  50 60 3E 30 */	rlwimi r0, r3, 7, 0x18, 0x18
/* 80105F0C 00102E6C  C0 82 94 5C */	lfs f4, lbl_805AB17C@sda21(r2)
/* 80105F10 00102E70  7C DF 33 78 */	mr r31, r6
/* 80105F14 00102E74  98 1E 00 48 */	stb r0, 0x48(r30)
/* 80105F18 00102E78  38 60 00 00 */	li r3, 0
/* 80105F1C 00102E7C  C0 62 94 54 */	lfs f3, lbl_805AB174@sda21(r2)
/* 80105F20 00102E80  C0 44 00 14 */	lfs f2, 0x14(r4)
/* 80105F24 00102E84  C0 24 00 08 */	lfs f1, 8(r4)
/* 80105F28 00102E88  C0 05 00 1C */	lfs f0, 0x1c(r5)
/* 80105F2C 00102E8C  EC A2 08 28 */	fsubs f5, f2, f1
/* 80105F30 00102E90  C0 25 00 0C */	lfs f1, 0xc(r5)
/* 80105F34 00102E94  ED 00 18 2A */	fadds f8, f0, f3
/* 80105F38 00102E98  C0 45 00 2C */	lfs f2, 0x2c(r5)
/* 80105F3C 00102E9C  EC 21 18 2A */	fadds f1, f1, f3
/* 80105F40 00102EA0  C0 02 94 60 */	lfs f0, lbl_805AB180@sda21(r2)
/* 80105F44 00102EA4  EF A5 01 32 */	fmuls f29, f5, f4
/* 80105F48 00102EA8  C0 C4 00 0C */	lfs f6, 0xc(r4)
/* 80105F4C 00102EAC  C0 A4 00 00 */	lfs f5, 0(r4)
/* 80105F50 00102EB0  C0 84 00 10 */	lfs f4, 0x10(r4)
/* 80105F54 00102EB4  EC E2 E8 2A */	fadds f7, f2, f29
/* 80105F58 00102EB8  C0 44 00 04 */	lfs f2, 4(r4)
/* 80105F5C 00102EBC  EF C6 28 28 */	fsubs f30, f6, f5
/* 80105F60 00102EC0  80 AD 89 B8 */	lwz r5, lbl_805A7578@sda21(r13)
/* 80105F64 00102EC4  EF E4 10 28 */	fsubs f31, f4, f2
/* 80105F68 00102EC8  D0 21 00 28 */	stfs f1, 0x28(r1)
/* 80105F6C 00102ECC  D1 01 00 2C */	stfs f8, 0x2c(r1)
/* 80105F70 00102ED0  38 80 00 01 */	li r4, 1
/* 80105F74 00102ED4  D0 E1 00 30 */	stfs f7, 0x30(r1)
/* 80105F78 00102ED8  D0 61 00 1C */	stfs f3, 0x1c(r1)
/* 80105F7C 00102EDC  D0 61 00 20 */	stfs f3, 0x20(r1)
/* 80105F80 00102EE0  D0 01 00 24 */	stfs f0, 0x24(r1)
/* 80105F84 00102EE4  48 28 3F 71 */	bl __shl2i
/* 80105F88 00102EE8  39 20 FF FF */	li r9, -1
/* 80105F8C 00102EEC  39 00 00 00 */	li r8, 0
/* 80105F90 00102EF0  38 00 00 02 */	li r0, 2
/* 80105F94 00102EF4  90 81 00 5C */	stw r4, 0x5c(r1)
/* 80105F98 00102EF8  7F E4 FB 78 */	mr r4, r31
/* 80105F9C 00102EFC  38 A1 00 28 */	addi r5, r1, 0x28
/* 80105FA0 00102F00  90 61 00 58 */	stw r3, 0x58(r1)
/* 80105FA4 00102F04  38 61 00 C8 */	addi r3, r1, 0xc8
/* 80105FA8 00102F08  38 C1 00 1C */	addi r6, r1, 0x1c
/* 80105FAC 00102F0C  38 E1 00 50 */	addi r7, r1, 0x50
/* 80105FB0 00102F10  91 21 00 54 */	stw r9, 0x54(r1)
/* 80105FB4 00102F14  91 01 00 50 */	stw r8, 0x50(r1)
/* 80105FB8 00102F18  90 01 00 60 */	stw r0, 0x60(r1)
/* 80105FBC 00102F1C  C0 3E 00 40 */	lfs f1, 0x40(r30)
/* 80105FC0 00102F20  4B F4 6F D9 */	bl RayStaticIntersection__13CStateManagerCFRC9CVector3fRC9CVector3ffRC15CMaterialFilter
/* 80105FC4 00102F24  88 81 00 E8 */	lbz r4, 0xe8(r1)
/* 80105FC8 00102F28  C0 21 00 C8 */	lfs f1, 0xc8(r1)
/* 80105FCC 00102F2C  28 04 00 00 */	cmplwi r4, 0
/* 80105FD0 00102F30  C0 41 00 CC */	lfs f2, 0xcc(r1)
/* 80105FD4 00102F34  C0 61 00 D0 */	lfs f3, 0xd0(r1)
/* 80105FD8 00102F38  C0 81 00 D4 */	lfs f4, 0xd4(r1)
/* 80105FDC 00102F3C  C0 A1 00 D8 */	lfs f5, 0xd8(r1)
/* 80105FE0 00102F40  C0 C1 00 DC */	lfs f6, 0xdc(r1)
/* 80105FE4 00102F44  C0 E1 00 E0 */	lfs f7, 0xe0(r1)
/* 80105FE8 00102F48  C1 01 00 E4 */	lfs f8, 0xe4(r1)
/* 80105FEC 00102F4C  80 A1 00 F0 */	lwz r5, 0xf0(r1)
/* 80105FF0 00102F50  80 C1 00 F4 */	lwz r6, 0xf4(r1)
/* 80105FF4 00102F54  C3 9E 00 40 */	lfs f28, 0x40(r30)
/* 80105FF8 00102F58  41 82 00 18 */	beq lbl_80106010
/* 80105FFC 00102F5C  88 1E 00 48 */	lbz r0, 0x48(r30)
/* 80106000 00102F60  38 60 00 01 */	li r3, 1
/* 80106004 00102F64  50 60 3E 30 */	rlwimi r0, r3, 7, 0x18, 0x18
/* 80106008 00102F68  FF 80 08 90 */	fmr f28, f1
/* 8010600C 00102F6C  98 1E 00 48 */	stb r0, 0x48(r30)
lbl_80106010:
/* 80106010 00102F70  C0 02 94 64 */	lfs f0, lbl_805AB184@sda21(r2)
/* 80106014 00102F74  D0 21 01 28 */	stfs f1, 0x128(r1)
/* 80106018 00102F78  EC 00 E8 2A */	fadds f0, f0, f29
/* 8010601C 00102F7C  D0 41 01 2C */	stfs f2, 0x12c(r1)
/* 80106020 00102F80  FC 1C 00 40 */	fcmpo cr0, f28, f0
/* 80106024 00102F84  D0 61 01 30 */	stfs f3, 0x130(r1)
/* 80106028 00102F88  D0 81 01 34 */	stfs f4, 0x134(r1)
/* 8010602C 00102F8C  D0 A1 01 38 */	stfs f5, 0x138(r1)
/* 80106030 00102F90  D0 C1 01 3C */	stfs f6, 0x13c(r1)
/* 80106034 00102F94  D0 E1 01 40 */	stfs f7, 0x140(r1)
/* 80106038 00102F98  D1 01 01 44 */	stfs f8, 0x144(r1)
/* 8010603C 00102F9C  98 81 01 48 */	stb r4, 0x148(r1)
/* 80106040 00102FA0  90 C1 01 54 */	stw r6, 0x154(r1)
/* 80106044 00102FA4  90 A1 01 50 */	stw r5, 0x150(r1)
/* 80106048 00102FA8  40 81 01 90 */	ble lbl_801061D8
/* 8010604C 00102FAC  38 00 00 00 */	li r0, 0
/* 80106050 00102FB0  80 AD 89 BC */	lwz r5, lbl_805A757C@sda21(r13)
/* 80106054 00102FB4  90 01 01 58 */	stw r0, 0x158(r1)
/* 80106058 00102FB8  38 60 00 00 */	li r3, 0
/* 8010605C 00102FBC  38 80 00 01 */	li r4, 1
/* 80106060 00102FC0  48 28 3E 95 */	bl __shl2i
/* 80106064 00102FC4  39 20 00 00 */	li r9, 0
/* 80106068 00102FC8  38 00 00 01 */	li r0, 1
/* 8010606C 00102FCC  90 81 00 3C */	stw r4, 0x3c(r1)
/* 80106070 00102FD0  38 81 01 58 */	addi r4, r1, 0x158
/* 80106074 00102FD4  38 A1 00 28 */	addi r5, r1, 0x28
/* 80106078 00102FD8  38 C1 00 1C */	addi r6, r1, 0x1c
/* 8010607C 00102FDC  90 61 00 38 */	stw r3, 0x38(r1)
/* 80106080 00102FE0  7F E3 FB 78 */	mr r3, r31
/* 80106084 00102FE4  38 E1 00 38 */	addi r7, r1, 0x38
/* 80106088 00102FE8  39 00 00 00 */	li r8, 0
/* 8010608C 00102FEC  91 21 00 44 */	stw r9, 0x44(r1)
/* 80106090 00102FF0  91 21 00 40 */	stw r9, 0x40(r1)
/* 80106094 00102FF4  90 01 00 48 */	stw r0, 0x48(r1)
/* 80106098 00102FF8  C0 3E 00 40 */	lfs f1, 0x40(r30)
/* 8010609C 00102FFC  4B F4 66 3D */	bl "BuildNearList__13CStateManagerCFRQ24rstl32reserved_vector<9TUniqueId,1024>RC9CVector3fRC9CVector3ffRC15CMaterialFilterPC6CActor"
/* 801060A0 00103000  A0 0D A3 8C */	lhz r0, kInvalidUniqueId@sda21(r13)
/* 801060A4 00103004  3C 60 80 5A */	lis r3, skPassEverything__15CMaterialFilter@ha
/* 801060A8 00103008  39 03 FD 18 */	addi r8, r3, skPassEverything__15CMaterialFilter@l
/* 801060AC 0010300C  7F E4 FB 78 */	mr r4, r31
/* 801060B0 00103010  B0 01 00 08 */	sth r0, 8(r1)
/* 801060B4 00103014  38 61 00 98 */	addi r3, r1, 0x98
/* 801060B8 00103018  38 A1 00 08 */	addi r5, r1, 8
/* 801060BC 0010301C  38 C1 00 28 */	addi r6, r1, 0x28
/* 801060C0 00103020  C0 3E 00 40 */	lfs f1, 0x40(r30)
/* 801060C4 00103024  38 E1 00 1C */	addi r7, r1, 0x1c
/* 801060C8 00103028  39 21 01 58 */	addi r9, r1, 0x158
/* 801060CC 0010302C  48 07 ED A1 */	bl "RayDynamicIntersection__14CGameCollisionFRC13CStateManagerR9TUniqueIdRC9CVector3fRC9CVector3ffRC15CMaterialFilterRCQ24rstl32reserved_vector<9TUniqueId,1024>"
/* 801060D0 00103030  88 E1 00 B8 */	lbz r7, 0xb8(r1)
/* 801060D4 00103034  C0 E1 00 98 */	lfs f7, 0x98(r1)
/* 801060D8 00103038  54 E0 06 3F */	clrlwi. r0, r7, 0x18
/* 801060DC 0010303C  C0 C1 00 9C */	lfs f6, 0x9c(r1)
/* 801060E0 00103040  C0 A1 00 A0 */	lfs f5, 0xa0(r1)
/* 801060E4 00103044  C0 01 00 A4 */	lfs f0, 0xa4(r1)
/* 801060E8 00103048  C0 81 00 A8 */	lfs f4, 0xa8(r1)
/* 801060EC 0010304C  C0 61 00 AC */	lfs f3, 0xac(r1)
/* 801060F0 00103050  C0 41 00 B0 */	lfs f2, 0xb0(r1)
/* 801060F4 00103054  C0 21 00 B4 */	lfs f1, 0xb4(r1)
/* 801060F8 00103058  80 01 00 C0 */	lwz r0, 0xc0(r1)
/* 801060FC 0010305C  80 61 00 C4 */	lwz r3, 0xc4(r1)
/* 80106100 00103060  D0 E1 00 F8 */	stfs f7, 0xf8(r1)
/* 80106104 00103064  D0 C1 00 FC */	stfs f6, 0xfc(r1)
/* 80106108 00103068  D0 A1 01 00 */	stfs f5, 0x100(r1)
/* 8010610C 0010306C  D0 01 01 04 */	stfs f0, 0x104(r1)
/* 80106110 00103070  D0 81 01 08 */	stfs f4, 0x108(r1)
/* 80106114 00103074  D0 61 01 0C */	stfs f3, 0x10c(r1)
/* 80106118 00103078  D0 41 01 10 */	stfs f2, 0x110(r1)
/* 8010611C 0010307C  D0 21 01 14 */	stfs f1, 0x114(r1)
/* 80106120 00103080  98 E1 01 18 */	stb r7, 0x118(r1)
/* 80106124 00103084  90 61 01 24 */	stw r3, 0x124(r1)
/* 80106128 00103088  90 01 01 20 */	stw r0, 0x120(r1)
/* 8010612C 0010308C  41 82 00 58 */	beq lbl_80106184
/* 80106130 00103090  FC 07 E0 40 */	fcmpo cr0, f7, f28
/* 80106134 00103094  40 80 00 50 */	bge lbl_80106184
/* 80106138 00103098  80 C1 00 FC */	lwz r6, 0xfc(r1)
/* 8010613C 0010309C  38 60 00 01 */	li r3, 1
/* 80106140 001030A0  80 A1 01 00 */	lwz r5, 0x100(r1)
/* 80106144 001030A4  FF 80 38 90 */	fmr f28, f7
/* 80106148 001030A8  80 81 01 04 */	lwz r4, 0x104(r1)
/* 8010614C 001030AC  C8 01 01 20 */	lfd f0, 0x120(r1)
/* 80106150 001030B0  88 1E 00 48 */	lbz r0, 0x48(r30)
/* 80106154 001030B4  50 60 3E 30 */	rlwimi r0, r3, 7, 0x18, 0x18
/* 80106158 001030B8  D0 E1 01 28 */	stfs f7, 0x128(r1)
/* 8010615C 001030BC  90 C1 01 2C */	stw r6, 0x12c(r1)
/* 80106160 001030C0  90 A1 01 30 */	stw r5, 0x130(r1)
/* 80106164 001030C4  90 81 01 34 */	stw r4, 0x134(r1)
/* 80106168 001030C8  D0 81 01 38 */	stfs f4, 0x138(r1)
/* 8010616C 001030CC  D0 61 01 3C */	stfs f3, 0x13c(r1)
/* 80106170 001030D0  D0 41 01 40 */	stfs f2, 0x140(r1)
/* 80106174 001030D4  D0 21 01 44 */	stfs f1, 0x144(r1)
/* 80106178 001030D8  98 E1 01 48 */	stb r7, 0x148(r1)
/* 8010617C 001030DC  D8 01 01 50 */	stfd f0, 0x150(r1)
/* 80106180 001030E0  98 1E 00 48 */	stb r0, 0x48(r30)
lbl_80106184:
/* 80106184 001030E4  80 A1 01 58 */	lwz r5, 0x158(r1)
/* 80106188 001030E8  38 60 00 00 */	li r3, 0
/* 8010618C 001030EC  2C 05 00 00 */	cmpwi r5, 0
/* 80106190 001030F0  40 81 00 40 */	ble lbl_801061D0
/* 80106194 001030F4  2C 05 00 08 */	cmpwi r5, 8
/* 80106198 001030F8  38 85 FF F8 */	addi r4, r5, -8
/* 8010619C 001030FC  40 81 00 20 */	ble lbl_801061BC
/* 801061A0 00103100  38 04 00 07 */	addi r0, r4, 7
/* 801061A4 00103104  54 00 E8 FE */	srwi r0, r0, 3
/* 801061A8 00103108  7C 09 03 A6 */	mtctr r0
/* 801061AC 0010310C  2C 04 00 00 */	cmpwi r4, 0
/* 801061B0 00103110  40 81 00 0C */	ble lbl_801061BC
lbl_801061B4:
/* 801061B4 00103114  38 63 00 08 */	addi r3, r3, 8
/* 801061B8 00103118  42 00 FF FC */	bdnz lbl_801061B4
lbl_801061BC:
/* 801061BC 0010311C  7C 03 28 50 */	subf r0, r3, r5
/* 801061C0 00103120  7C 09 03 A6 */	mtctr r0
/* 801061C4 00103124  7C 03 28 00 */	cmpw r3, r5
/* 801061C8 00103128  40 80 00 08 */	bge lbl_801061D0
lbl_801061CC:
/* 801061CC 0010312C  42 00 00 00 */	bdnz lbl_801061CC
lbl_801061D0:
/* 801061D0 00103130  38 00 00 00 */	li r0, 0
/* 801061D4 00103134  90 01 01 58 */	stw r0, 0x158(r1)
lbl_801061D8:
/* 801061D8 00103138  88 1E 00 48 */	lbz r0, 0x48(r30)
/* 801061DC 0010313C  54 00 CF FF */	rlwinm. r0, r0, 0x19, 0x1f, 0x1f
/* 801061E0 00103140  41 82 01 28 */	beq lbl_80106308
/* 801061E4 00103144  C0 1E 00 40 */	lfs f0, 0x40(r30)
/* 801061E8 00103148  3C 80 80 5A */	lis r4, sZeroVector__9CVector3f@ha
/* 801061EC 0010314C  3C 60 80 5A */	lis r3, sUpVector__9CVector3f@ha
/* 801061F0 00103150  C0 62 94 58 */	lfs f3, lbl_805AB178@sda21(r2)
/* 801061F4 00103154  EC 1C 00 24 */	fdivs f0, f28, f0
/* 801061F8 00103158  38 A4 66 A0 */	addi r5, r4, sZeroVector__9CVector3f@l
/* 801061FC 0010315C  38 C3 66 F4 */	addi r6, r3, sUpVector__9CVector3f@l
/* 80106200 00103160  C0 41 01 38 */	lfs f2, 0x138(r1)
/* 80106204 00103164  C0 21 01 3C */	lfs f1, 0x13c(r1)
/* 80106208 00103168  38 61 00 68 */	addi r3, r1, 0x68
/* 8010620C 0010316C  EC 63 00 28 */	fsubs f3, f3, f0
/* 80106210 00103170  C0 01 01 40 */	lfs f0, 0x140(r1)
/* 80106214 00103174  38 81 00 10 */	addi r4, r1, 0x10
/* 80106218 00103178  D0 7E 00 3C */	stfs f3, 0x3c(r30)
/* 8010621C 0010317C  D0 41 00 10 */	stfs f2, 0x10(r1)
/* 80106220 00103180  D0 21 00 14 */	stfs f1, 0x14(r1)
/* 80106224 00103184  D0 01 00 18 */	stfs f0, 0x18(r1)
/* 80106228 00103188  48 20 D8 E9 */	bl LookAt__12CTransform4fFRC9CVector3fRC9CVector3fRC9CVector3f
/* 8010622C 0010318C  7F C3 F3 78 */	mr r3, r30
/* 80106230 00103190  38 81 00 68 */	addi r4, r1, 0x68
/* 80106234 00103194  48 20 C9 0D */	bl __as__12CTransform4fFRC12CTransform4f
/* 80106238 00103198  C0 BE 00 44 */	lfs f5, 0x44(r30)
/* 8010623C 0010319C  C0 01 00 10 */	lfs f0, 0x10(r1)
/* 80106240 001031A0  C0 61 00 14 */	lfs f3, 0x14(r1)
/* 80106244 001031A4  EC 05 00 32 */	fmuls f0, f5, f0
/* 80106248 001031A8  C0 21 01 2C */	lfs f1, 0x12c(r1)
/* 8010624C 001031AC  C0 41 00 18 */	lfs f2, 0x18(r1)
/* 80106250 001031B0  EC 65 00 F2 */	fmuls f3, f5, f3
/* 80106254 001031B4  C0 81 01 30 */	lfs f4, 0x130(r1)
/* 80106258 001031B8  EC 01 00 2A */	fadds f0, f1, f0
/* 8010625C 001031BC  EC 25 00 B2 */	fmuls f1, f5, f2
/* 80106260 001031C0  C0 41 01 34 */	lfs f2, 0x134(r1)
/* 80106264 001031C4  EC 64 18 2A */	fadds f3, f4, f3
/* 80106268 001031C8  D0 1E 00 0C */	stfs f0, 0xc(r30)
/* 8010626C 001031CC  EC 02 08 2A */	fadds f0, f2, f1
/* 80106270 001031D0  D0 7E 00 1C */	stfs f3, 0x1c(r30)
/* 80106274 001031D4  D0 1E 00 2C */	stfs f0, 0x2c(r30)
/* 80106278 001031D8  88 7E 00 48 */	lbz r3, 0x48(r30)
/* 8010627C 001031DC  54 60 D7 FF */	rlwinm. r0, r3, 0x1a, 0x1f, 0x1f
/* 80106280 001031E0  40 82 00 0C */	bne lbl_8010628C
/* 80106284 001031E4  54 60 DF FF */	rlwinm. r0, r3, 0x1b, 0x1f, 0x1f
/* 80106288 001031E8  40 82 00 80 */	bne lbl_80106308
lbl_8010628C:
/* 8010628C 001031EC  EC 3F 07 F2 */	fmuls f1, f31, f31
/* 80106290 001031F0  C0 02 94 54 */	lfs f0, lbl_805AB174@sda21(r2)
/* 80106294 001031F4  EC 9E 0F BA */	fmadds f4, f30, f30, f1
/* 80106298 001031F8  FC 04 00 40 */	fcmpo cr0, f4, f0
/* 8010629C 001031FC  40 81 00 50 */	ble lbl_801062EC
/* 801062A0 00103200  FC 20 20 34 */	frsqrte f1, f4
/* 801062A4 00103204  C8 62 94 68 */	lfd f3, lbl_805AB188@sda21(r2)
/* 801062A8 00103208  C8 42 94 70 */	lfd f2, lbl_805AB190@sda21(r2)
/* 801062AC 0010320C  FC 01 00 72 */	fmul f0, f1, f1
/* 801062B0 00103210  FC 23 00 72 */	fmul f1, f3, f1
/* 801062B4 00103214  FC 04 10 3C */	fnmsub f0, f4, f0, f2
/* 801062B8 00103218  FC 21 00 32 */	fmul f1, f1, f0
/* 801062BC 0010321C  FC 01 00 72 */	fmul f0, f1, f1
/* 801062C0 00103220  FC 23 00 72 */	fmul f1, f3, f1
/* 801062C4 00103224  FC 04 10 3C */	fnmsub f0, f4, f0, f2
/* 801062C8 00103228  FC 21 00 32 */	fmul f1, f1, f0
/* 801062CC 0010322C  FC 01 00 72 */	fmul f0, f1, f1
/* 801062D0 00103230  FC 23 00 72 */	fmul f1, f3, f1
/* 801062D4 00103234  FC 04 10 3C */	fnmsub f0, f4, f0, f2
/* 801062D8 00103238  FC 01 00 32 */	fmul f0, f1, f0
/* 801062DC 0010323C  FC 04 00 32 */	fmul f0, f4, f0
/* 801062E0 00103240  FC 00 00 18 */	frsp f0, f0
/* 801062E4 00103244  D0 01 00 0C */	stfs f0, 0xc(r1)
/* 801062E8 00103248  C0 81 00 0C */	lfs f4, 0xc(r1)
lbl_801062EC:
/* 801062EC 0010324C  C0 02 94 5C */	lfs f0, lbl_805AB17C@sda21(r2)
/* 801062F0 00103250  38 60 00 01 */	li r3, 1
/* 801062F4 00103254  EC 04 00 32 */	fmuls f0, f4, f0
/* 801062F8 00103258  D0 1E 00 34 */	stfs f0, 0x34(r30)
/* 801062FC 0010325C  88 1E 00 48 */	lbz r0, 0x48(r30)
/* 80106300 00103260  50 60 2E B4 */	rlwimi r0, r3, 5, 0x1a, 0x1a
/* 80106304 00103264  98 1E 00 48 */	stb r0, 0x48(r30)
lbl_80106308:
/* 80106308 00103268  E3 E1 09 A8 */	psq_l f31, -1624(r1), 0, qr0
/* 8010630C 0010326C  CB E1 09 A0 */	lfd f31, 0x9a0(r1)
/* 80106310 00103270  E3 C1 09 98 */	psq_l f30, -1640(r1), 0, qr0
/* 80106314 00103274  CB C1 09 90 */	lfd f30, 0x990(r1)
/* 80106318 00103278  E3 A1 09 88 */	psq_l f29, -1656(r1), 0, qr0
/* 8010631C 0010327C  CB A1 09 80 */	lfd f29, 0x980(r1)
/* 80106320 00103280  E3 81 09 78 */	psq_l f28, -1672(r1), 0, qr0
/* 80106324 00103284  CB 81 09 70 */	lfd f28, 0x970(r1)
/* 80106328 00103288  83 E1 09 6C */	lwz r31, 0x96c(r1)
/* 8010632C 0010328C  80 01 09 B4 */	lwz r0, 0x9b4(r1)
/* 80106330 00103290  83 C1 09 68 */	lwz r30, 0x968(r1)
/* 80106334 00103294  7C 08 03 A6 */	mtlr r0
/* 80106338 00103298  38 21 09 B0 */	addi r1, r1, 0x9b0
/* 8010633C 0010329C  4E 80 00 20 */	blr

.global __ct__13CSimpleShadowFffff
__ct__13CSimpleShadowFffff:
/* 80106340 001032A0  94 21 FF B0 */	stwu r1, -0x50(r1)
/* 80106344 001032A4  7C 08 02 A6 */	mflr r0
/* 80106348 001032A8  90 01 00 54 */	stw r0, 0x54(r1)
/* 8010634C 001032AC  DB E1 00 40 */	stfd f31, 0x40(r1)
/* 80106350 001032B0  F3 E1 00 48 */	psq_st f31, 72(r1), 0, qr0
/* 80106354 001032B4  DB C1 00 30 */	stfd f30, 0x30(r1)
/* 80106358 001032B8  F3 C1 00 38 */	psq_st f30, 56(r1), 0, qr0
/* 8010635C 001032BC  DB A1 00 20 */	stfd f29, 0x20(r1)
/* 80106360 001032C0  F3 A1 00 28 */	psq_st f29, 40(r1), 0, qr0
/* 80106364 001032C4  DB 81 00 10 */	stfd f28, 0x10(r1)
/* 80106368 001032C8  F3 81 00 18 */	psq_st f28, 24(r1), 0, qr0
/* 8010636C 001032CC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80106370 001032D0  FF 80 08 90 */	fmr f28, f1
/* 80106374 001032D4  3C 80 80 5A */	lis r4, sIdentity__12CTransform4f@ha
/* 80106378 001032D8  FF A0 10 90 */	fmr f29, f2
/* 8010637C 001032DC  38 84 66 70 */	addi r4, r4, sIdentity__12CTransform4f@l
/* 80106380 001032E0  FF C0 18 90 */	fmr f30, f3
/* 80106384 001032E4  7C 7F 1B 78 */	mr r31, r3
/* 80106388 001032E8  FF E0 20 90 */	fmr f31, f4
/* 8010638C 001032EC  48 20 C7 E9 */	bl __ct__12CTransform4fFRC12CTransform4f
/* 80106390 001032F0  D3 9F 00 30 */	stfs f28, 0x30(r31)
/* 80106394 001032F4  38 A0 00 00 */	li r5, 0
/* 80106398 001032F8  C0 02 94 58 */	lfs f0, lbl_805AB178@sda21(r2)
/* 8010639C 001032FC  38 80 00 01 */	li r4, 1
/* 801063A0 00103300  7F E3 FB 78 */	mr r3, r31
/* 801063A4 00103304  D0 1F 00 34 */	stfs f0, 0x34(r31)
/* 801063A8 00103308  D3 BF 00 38 */	stfs f29, 0x38(r31)
/* 801063AC 0010330C  D0 1F 00 3C */	stfs f0, 0x3c(r31)
/* 801063B0 00103310  D3 DF 00 40 */	stfs f30, 0x40(r31)
/* 801063B4 00103314  D3 FF 00 44 */	stfs f31, 0x44(r31)
/* 801063B8 00103318  88 1F 00 48 */	lbz r0, 0x48(r31)
/* 801063BC 0010331C  50 A0 3E 30 */	rlwimi r0, r5, 7, 0x18, 0x18
/* 801063C0 00103320  98 1F 00 48 */	stb r0, 0x48(r31)
/* 801063C4 00103324  88 1F 00 48 */	lbz r0, 0x48(r31)
/* 801063C8 00103328  50 80 36 72 */	rlwimi r0, r4, 6, 0x19, 0x19
/* 801063CC 0010332C  98 1F 00 48 */	stb r0, 0x48(r31)
/* 801063D0 00103330  88 1F 00 48 */	lbz r0, 0x48(r31)
/* 801063D4 00103334  50 A0 2E B4 */	rlwimi r0, r5, 5, 0x1a, 0x1a
/* 801063D8 00103338  98 1F 00 48 */	stb r0, 0x48(r31)
/* 801063DC 0010333C  E3 E1 00 48 */	psq_l f31, 72(r1), 0, qr0
/* 801063E0 00103340  CB E1 00 40 */	lfd f31, 0x40(r1)
/* 801063E4 00103344  E3 C1 00 38 */	psq_l f30, 56(r1), 0, qr0
/* 801063E8 00103348  CB C1 00 30 */	lfd f30, 0x30(r1)
/* 801063EC 0010334C  E3 A1 00 28 */	psq_l f29, 40(r1), 0, qr0
/* 801063F0 00103350  CB A1 00 20 */	lfd f29, 0x20(r1)
/* 801063F4 00103354  E3 81 00 18 */	psq_l f28, 24(r1), 0, qr0
/* 801063F8 00103358  CB 81 00 10 */	lfd f28, 0x10(r1)
/* 801063FC 0010335C  80 01 00 54 */	lwz r0, 0x54(r1)
/* 80106400 00103360  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80106404 00103364  7C 08 03 A6 */	mtlr r0
/* 80106408 00103368  38 21 00 50 */	addi r1, r1, 0x50
/* 8010640C 0010336C  4E 80 00 20 */	blr
