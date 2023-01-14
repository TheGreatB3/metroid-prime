.include "macros.inc"

.section .data
.balign 8

.global lbl_803EC888
lbl_803EC888:
	# ROM: 0x3E9888
	.4byte 0
	.4byte 0
	.4byte __dt__9CGuiModelFv
	.4byte Update__10CGuiWidgetFf
	.4byte Draw__9CGuiModelCFRC19CGuiWidgetDrawParms
	.4byte Initialize__10CGuiWidgetFv
	.4byte ProcessUserInput__10CGuiWidgetFRC11CFinalInput
	.4byte Touch__9CGuiModelCFv
	.4byte GetIsVisible__10CGuiWidgetCFv
	.4byte GetIsActive__10CGuiWidgetCFv
	.4byte GetWidgetTypeID__9CGuiModelCFv
	.4byte AddWorkerWidget__10CGuiWidgetFP10CGuiWidget
	.4byte GetIsFinishedLoadingWidgetSpecific__9CGuiModelCFv
	.4byte OnVisible__10CGuiWidgetFv
	.4byte OnActivate__10CGuiWidgetFv
	.4byte GetModelAssets__9CGuiModelCFv

.section .text, "ax"

.global GetWidgetTypeID__9CGuiModelCFv
GetWidgetTypeID__9CGuiModelCFv:
/* 802C3F64 002C0EC4  3C 60 4D 4F */	lis r3, 0x4D4F444C@ha
/* 802C3F68 002C0EC8  38 63 44 4C */	addi r3, r3, 0x4D4F444C@l
/* 802C3F6C 002C0ECC  4E 80 00 20 */	blr

.global GetModelAssets__9CGuiModelCFv
GetModelAssets__9CGuiModelCFv:
/* 802C3F70 002C0ED0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 802C3F74 002C0ED4  7C 08 02 A6 */	mflr r0
/* 802C3F78 002C0ED8  90 01 00 14 */	stw r0, 0x14(r1)
/* 802C3F7C 002C0EDC  38 00 00 01 */	li r0, 1
/* 802C3F80 002C0EE0  93 E1 00 0C */	stw r31, 0xc(r1)
/* 802C3F84 002C0EE4  7C 9F 23 78 */	mr r31, r4
/* 802C3F88 002C0EE8  93 C1 00 08 */	stw r30, 8(r1)
/* 802C3F8C 002C0EEC  7C 7E 1B 78 */	mr r30, r3
/* 802C3F90 002C0EF0  90 03 00 04 */	stw r0, 4(r3)
/* 802C3F94 002C0EF4  90 03 00 08 */	stw r0, 8(r3)
/* 802C3F98 002C0EF8  80 03 00 04 */	lwz r0, 4(r3)
/* 802C3F9C 002C0EFC  54 03 10 3B */	rlwinm. r3, r0, 2, 0, 0x1d
/* 802C3FA0 002C0F00  40 82 00 10 */	bne lbl_802C3FB0
/* 802C3FA4 002C0F04  38 00 00 00 */	li r0, 0
/* 802C3FA8 002C0F08  90 1E 00 0C */	stw r0, 0xc(r30)
/* 802C3FAC 002C0F0C  48 00 00 18 */	b lbl_802C3FC4
lbl_802C3FB0:
/* 802C3FB0 002C0F10  3C 80 80 3D */	lis r4, lbl_803D68B0@ha
/* 802C3FB4 002C0F14  38 A0 00 00 */	li r5, 0
/* 802C3FB8 002C0F18  38 84 68 B0 */	addi r4, r4, lbl_803D68B0@l
/* 802C3FBC 002C0F1C  48 05 18 5D */	bl __nwa__FUlPCcPCc
/* 802C3FC0 002C0F20  90 7E 00 0C */	stw r3, 0xc(r30)
lbl_802C3FC4:
/* 802C3FC4 002C0F24  80 7E 00 0C */	lwz r3, 0xc(r30)
/* 802C3FC8 002C0F28  28 03 00 00 */	cmplwi r3, 0
/* 802C3FCC 002C0F2C  41 82 00 0C */	beq lbl_802C3FD8
/* 802C3FD0 002C0F30  80 1F 00 C8 */	lwz r0, 0xc8(r31)
/* 802C3FD4 002C0F34  90 03 00 00 */	stw r0, 0(r3)
lbl_802C3FD8:
/* 802C3FD8 002C0F38  80 01 00 14 */	lwz r0, 0x14(r1)
/* 802C3FDC 002C0F3C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 802C3FE0 002C0F40  83 C1 00 08 */	lwz r30, 8(r1)
/* 802C3FE4 002C0F44  7C 08 03 A6 */	mtlr r0
/* 802C3FE8 002C0F48  38 21 00 10 */	addi r1, r1, 0x10
/* 802C3FEC 002C0F4C  4E 80 00 20 */	blr

.global GetIsFinishedLoadingWidgetSpecific__9CGuiModelCFv
GetIsFinishedLoadingWidgetSpecific__9CGuiModelCFv:
/* 802C3FF0 002C0F50  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 802C3FF4 002C0F54  7C 08 02 A6 */	mflr r0
/* 802C3FF8 002C0F58  90 01 00 14 */	stw r0, 0x14(r1)
/* 802C3FFC 002C0F5C  93 E1 00 0C */	stw r31, 0xc(r1)
/* 802C4000 002C0F60  93 C1 00 08 */	stw r30, 8(r1)
/* 802C4004 002C0F64  7C 7E 1B 78 */	mr r30, r3
/* 802C4008 002C0F68  88 03 00 C4 */	lbz r0, 0xc4(r3)
/* 802C400C 002C0F6C  28 00 00 00 */	cmplwi r0, 0
/* 802C4010 002C0F70  40 82 00 0C */	bne lbl_802C401C
/* 802C4014 002C0F74  38 60 00 01 */	li r3, 1
/* 802C4018 002C0F78  48 00 00 6C */	b lbl_802C4084
lbl_802C401C:
/* 802C401C 002C0F7C  3B FE 00 B8 */	addi r31, r30, 0xb8
/* 802C4020 002C0F80  80 1E 00 C0 */	lwz r0, 0xc0(r30)
/* 802C4024 002C0F84  28 00 00 00 */	cmplwi r0, 0
/* 802C4028 002C0F88  40 82 00 30 */	bne lbl_802C4058
/* 802C402C 002C0F8C  88 1F 00 04 */	lbz r0, 4(r31)
/* 802C4030 002C0F90  28 00 00 00 */	cmplwi r0, 0
/* 802C4034 002C0F94  41 82 00 24 */	beq lbl_802C4058
/* 802C4038 002C0F98  80 7F 00 00 */	lwz r3, 0(r31)
/* 802C403C 002C0F9C  80 03 00 10 */	lwz r0, 0x10(r3)
/* 802C4040 002C0FA0  28 00 00 00 */	cmplwi r0, 0
/* 802C4044 002C0FA4  41 82 00 14 */	beq lbl_802C4058
/* 802C4048 002C0FA8  7F E3 FB 78 */	mr r3, r31
/* 802C404C 002C0FAC  48 07 CD C1 */	bl GetObj__6CTokenFv
/* 802C4050 002C0FB0  80 03 00 04 */	lwz r0, 4(r3)
/* 802C4054 002C0FB4  90 1F 00 08 */	stw r0, 8(r31)
lbl_802C4058:
/* 802C4058 002C0FB8  83 FE 00 C0 */	lwz r31, 0xc0(r30)
/* 802C405C 002C0FBC  28 1F 00 00 */	cmplwi r31, 0
/* 802C4060 002C0FC0  41 82 00 20 */	beq lbl_802C4080
/* 802C4064 002C0FC4  7F E3 FB 78 */	mr r3, r31
/* 802C4068 002C0FC8  38 80 00 00 */	li r4, 0
/* 802C406C 002C0FCC  48 09 15 C9 */	bl Touch__6CModelCFi
/* 802C4070 002C0FD0  7F E3 FB 78 */	mr r3, r31
/* 802C4074 002C0FD4  38 80 00 00 */	li r4, 0
/* 802C4078 002C0FD8  48 09 15 31 */	bl IsLoaded__6CModelCFi
/* 802C407C 002C0FDC  48 00 00 08 */	b lbl_802C4084
lbl_802C4080:
/* 802C4080 002C0FE0  38 60 00 00 */	li r3, 0
lbl_802C4084:
/* 802C4084 002C0FE4  80 01 00 14 */	lwz r0, 0x14(r1)
/* 802C4088 002C0FE8  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 802C408C 002C0FEC  83 C1 00 08 */	lwz r30, 8(r1)
/* 802C4090 002C0FF0  7C 08 03 A6 */	mtlr r0
/* 802C4094 002C0FF4  38 21 00 10 */	addi r1, r1, 0x10
/* 802C4098 002C0FF8  4E 80 00 20 */	blr

.global Touch__9CGuiModelCFv
Touch__9CGuiModelCFv:
/* 802C409C 002C0FFC  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 802C40A0 002C1000  7C 08 02 A6 */	mflr r0
/* 802C40A4 002C1004  90 01 00 14 */	stw r0, 0x14(r1)
/* 802C40A8 002C1008  88 03 00 C4 */	lbz r0, 0xc4(r3)
/* 802C40AC 002C100C  28 00 00 00 */	cmplwi r0, 0
/* 802C40B0 002C1010  41 82 00 18 */	beq lbl_802C40C8
/* 802C40B4 002C1014  80 63 00 C0 */	lwz r3, 0xc0(r3)
/* 802C40B8 002C1018  28 03 00 00 */	cmplwi r3, 0
/* 802C40BC 002C101C  41 82 00 0C */	beq lbl_802C40C8
/* 802C40C0 002C1020  38 80 00 00 */	li r4, 0
/* 802C40C4 002C1024  48 09 15 71 */	bl Touch__6CModelCFi
lbl_802C40C8:
/* 802C40C8 002C1028  80 01 00 14 */	lwz r0, 0x14(r1)
/* 802C40CC 002C102C  7C 08 03 A6 */	mtlr r0
/* 802C40D0 002C1030  38 21 00 10 */	addi r1, r1, 0x10
/* 802C40D4 002C1034  4E 80 00 20 */	blr

.global Draw__9CGuiModelCFRC19CGuiWidgetDrawParms
Draw__9CGuiModelCFRC19CGuiWidgetDrawParms:
/* 802C40D8 002C1038  94 21 FF 80 */	stwu r1, -0x80(r1)
/* 802C40DC 002C103C  7C 08 02 A6 */	mflr r0
/* 802C40E0 002C1040  90 01 00 84 */	stw r0, 0x84(r1)
/* 802C40E4 002C1044  DB E1 00 70 */	stfd f31, 0x70(r1)
/* 802C40E8 002C1048  F3 E1 00 78 */	psq_st f31, 120(r1), 0, qr0
/* 802C40EC 002C104C  BF 41 00 58 */	stmw r26, 0x58(r1)
/* 802C40F0 002C1050  7C 7A 1B 78 */	mr r26, r3
/* 802C40F4 002C1054  7C 9B 23 78 */	mr r27, r4
/* 802C40F8 002C1058  38 7A 00 34 */	addi r3, r26, 0x34
/* 802C40FC 002C105C  48 04 84 E5 */	bl SetModelMatrix__9CGraphicsFRC12CTransform4f
/* 802C4100 002C1060  88 1A 00 C4 */	lbz r0, 0xc4(r26)
/* 802C4104 002C1064  28 00 00 00 */	cmplwi r0, 0
/* 802C4108 002C1068  41 82 03 44 */	beq lbl_802C444C
/* 802C410C 002C106C  7F 43 D3 78 */	mr r3, r26
/* 802C4110 002C1070  48 00 68 29 */	bl GetIsFinishedLoading__10CGuiWidgetCFv
/* 802C4114 002C1074  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 802C4118 002C1078  41 82 03 34 */	beq lbl_802C444C
/* 802C411C 002C107C  83 BA 00 C0 */	lwz r29, 0xc0(r26)
/* 802C4120 002C1080  28 1D 00 00 */	cmplwi r29, 0
/* 802C4124 002C1084  41 82 03 28 */	beq lbl_802C444C
/* 802C4128 002C1088  7F 43 D3 78 */	mr r3, r26
/* 802C412C 002C108C  81 9A 00 00 */	lwz r12, 0(r26)
/* 802C4130 002C1090  81 8C 00 20 */	lwz r12, 0x20(r12)
/* 802C4134 002C1094  7D 89 03 A6 */	mtctr r12
/* 802C4138 002C1098  4E 80 04 21 */	bctrl
/* 802C413C 002C109C  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 802C4140 002C10A0  41 82 03 00 */	beq lbl_802C4440
/* 802C4144 002C10A4  88 7A 00 AB */	lbz r3, 0xab(r26)
/* 802C4148 002C10A8  3C 00 43 30 */	lis r0, 0x4330
/* 802C414C 002C10AC  90 01 00 48 */	stw r0, 0x48(r1)
/* 802C4150 002C10B0  C8 22 C3 90 */	lfd f1, lbl_805AE0B0@sda21(r2)
/* 802C4154 002C10B4  90 61 00 4C */	stw r3, 0x4c(r1)
/* 802C4158 002C10B8  C0 5B 00 00 */	lfs f2, 0(r27)
/* 802C415C 002C10BC  C8 01 00 48 */	lfd f0, 0x48(r1)
/* 802C4160 002C10C0  EC 00 08 28 */	fsubs f0, f0, f1
/* 802C4164 002C10C4  EF E2 00 32 */	fmuls f31, f2, f0
/* 802C4168 002C10C8  F3 E1 A0 08 */	psq_st f31, 8(r1), 1, qr2
/* 802C416C 002C10CC  80 1A 00 A8 */	lwz r0, 0xa8(r26)
/* 802C4170 002C10D0  8B C1 00 08 */	lbz r30, 8(r1)
/* 802C4174 002C10D4  80 7A 00 B0 */	lwz r3, 0xb0(r26)
/* 802C4178 002C10D8  50 1E 00 2E */	rlwimi r30, r0, 0, 0, 0x17
/* 802C417C 002C10DC  80 9A 00 CC */	lwz r4, 0xcc(r26)
/* 802C4180 002C10E0  4B FF E2 59 */	bl EnableLights__9CGuiFrameCFUi
/* 802C4184 002C10E4  88 1A 00 B6 */	lbz r0, 0xb6(r26)
/* 802C4188 002C10E8  54 1F F7 FF */	rlwinm. r31, r0, 0x1e, 0x1f, 0x1f
/* 802C418C 002C10EC  41 82 00 0C */	beq lbl_802C4198
/* 802C4190 002C10F0  38 60 00 01 */	li r3, 1
/* 802C4194 002C10F4  48 04 78 DD */	bl SetCullMode__9CGraphicsF12ERglCullMode
lbl_802C4198:
/* 802C4198 002C10F8  C0 22 C3 88 */	lfs f1, lbl_805AE0A8@sda21(r2)
/* 802C419C 002C10FC  38 80 00 00 */	li r4, 0
/* 802C41A0 002C1100  38 00 00 03 */	li r0, 3
/* 802C41A4 002C1104  98 81 00 34 */	stb r4, 0x34(r1)
/* 802C41A8 002C1108  FC 40 08 90 */	fmr f2, f1
/* 802C41AC 002C110C  38 61 00 38 */	addi r3, r1, 0x38
/* 802C41B0 002C1110  FC 60 08 90 */	fmr f3, f1
/* 802C41B4 002C1114  98 81 00 35 */	stb r4, 0x35(r1)
/* 802C41B8 002C1118  FC 80 08 90 */	fmr f4, f1
/* 802C41BC 002C111C  B0 01 00 36 */	sth r0, 0x36(r1)
/* 802C41C0 002C1120  48 09 F2 2D */	bl __ct__6CColorFffff
/* 802C41C4 002C1124  80 1A 00 AC */	lwz r0, 0xac(r26)
/* 802C41C8 002C1128  3B 80 00 01 */	li r28, 1
/* 802C41CC 002C112C  88 C1 00 34 */	lbz r6, 0x34(r1)
/* 802C41D0 002C1130  88 A1 00 35 */	lbz r5, 0x35(r1)
/* 802C41D4 002C1134  2C 00 00 02 */	cmpwi r0, 2
/* 802C41D8 002C1138  A0 81 00 36 */	lhz r4, 0x36(r1)
/* 802C41DC 002C113C  80 61 00 38 */	lwz r3, 0x38(r1)
/* 802C41E0 002C1140  98 C1 00 3C */	stb r6, 0x3c(r1)
/* 802C41E4 002C1144  98 A1 00 3D */	stb r5, 0x3d(r1)
/* 802C41E8 002C1148  B0 81 00 3E */	sth r4, 0x3e(r1)
/* 802C41EC 002C114C  90 61 00 40 */	stw r3, 0x40(r1)
/* 802C41F0 002C1150  41 82 00 98 */	beq lbl_802C4288
/* 802C41F4 002C1154  40 80 00 14 */	bge lbl_802C4208
/* 802C41F8 002C1158  2C 00 00 00 */	cmpwi r0, 0
/* 802C41FC 002C115C  41 82 00 1C */	beq lbl_802C4218
/* 802C4200 002C1160  40 80 00 68 */	bge lbl_802C4268
/* 802C4204 002C1164  48 00 01 E4 */	b lbl_802C43E8
lbl_802C4208:
/* 802C4208 002C1168  2C 00 00 04 */	cmpwi r0, 4
/* 802C420C 002C116C  41 82 01 04 */	beq lbl_802C4310
/* 802C4210 002C1170  40 80 01 D8 */	bge lbl_802C43E8
/* 802C4214 002C1174  48 00 00 B8 */	b lbl_802C42CC
lbl_802C4218:
/* 802C4218 002C1178  C0 22 C3 88 */	lfs f1, lbl_805AE0A8@sda21(r2)
/* 802C421C 002C117C  38 80 00 00 */	li r4, 0
/* 802C4220 002C1180  38 00 00 03 */	li r0, 3
/* 802C4224 002C1184  98 81 00 2C */	stb r4, 0x2c(r1)
/* 802C4228 002C1188  FC 40 08 90 */	fmr f2, f1
/* 802C422C 002C118C  38 61 00 30 */	addi r3, r1, 0x30
/* 802C4230 002C1190  FC 60 08 90 */	fmr f3, f1
/* 802C4234 002C1194  98 81 00 2D */	stb r4, 0x2d(r1)
/* 802C4238 002C1198  FC 80 08 90 */	fmr f4, f1
/* 802C423C 002C119C  B0 01 00 2E */	sth r0, 0x2e(r1)
/* 802C4240 002C11A0  48 09 F1 AD */	bl __ct__6CColorFffff
/* 802C4244 002C11A4  88 A1 00 2C */	lbz r5, 0x2c(r1)
/* 802C4248 002C11A8  88 81 00 2D */	lbz r4, 0x2d(r1)
/* 802C424C 002C11AC  A0 61 00 2E */	lhz r3, 0x2e(r1)
/* 802C4250 002C11B0  80 01 00 30 */	lwz r0, 0x30(r1)
/* 802C4254 002C11B4  98 A1 00 3C */	stb r5, 0x3c(r1)
/* 802C4258 002C11B8  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C425C 002C11BC  B0 61 00 3E */	sth r3, 0x3e(r1)
/* 802C4260 002C11C0  90 01 00 40 */	stw r0, 0x40(r1)
/* 802C4264 002C11C4  48 00 01 88 */	b lbl_802C43EC
lbl_802C4268:
/* 802C4268 002C11C8  38 80 00 01 */	li r4, 1
/* 802C426C 002C11CC  38 60 00 00 */	li r3, 0
/* 802C4270 002C11D0  38 00 00 03 */	li r0, 3
/* 802C4274 002C11D4  98 81 00 3C */	stb r4, 0x3c(r1)
/* 802C4278 002C11D8  98 61 00 3D */	stb r3, 0x3d(r1)
/* 802C427C 002C11DC  B0 01 00 3E */	sth r0, 0x3e(r1)
/* 802C4280 002C11E0  93 C1 00 40 */	stw r30, 0x40(r1)
/* 802C4284 002C11E4  48 00 01 68 */	b lbl_802C43EC
lbl_802C4288:
/* 802C4288 002C11E8  88 1A 00 B7 */	lbz r0, 0xb7(r26)
/* 802C428C 002C11EC  38 A0 00 05 */	li r5, 5
/* 802C4290 002C11F0  38 60 00 03 */	li r3, 3
/* 802C4294 002C11F4  88 DA 00 B6 */	lbz r6, 0xb6(r26)
/* 802C4298 002C11F8  38 80 00 00 */	li r4, 0
/* 802C429C 002C11FC  54 00 D7 BC */	rlwinm r0, r0, 0x1a, 0x1e, 0x1e
/* 802C42A0 002C1200  50 C0 07 FE */	rlwimi r0, r6, 0, 0x1f, 0x1f
/* 802C42A4 002C1204  98 A1 00 24 */	stb r5, 0x24(r1)
/* 802C42A8 002C1208  50 60 04 3A */	rlwimi r0, r3, 0, 0x10, 0x1d
/* 802C42AC 002C120C  98 81 00 25 */	stb r4, 0x25(r1)
/* 802C42B0 002C1210  B0 61 00 26 */	sth r3, 0x26(r1)
/* 802C42B4 002C1214  93 C1 00 28 */	stw r30, 0x28(r1)
/* 802C42B8 002C1218  98 A1 00 3C */	stb r5, 0x3c(r1)
/* 802C42BC 002C121C  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C42C0 002C1220  B0 01 00 3E */	sth r0, 0x3e(r1)
/* 802C42C4 002C1224  93 C1 00 40 */	stw r30, 0x40(r1)
/* 802C42C8 002C1228  48 00 01 24 */	b lbl_802C43EC
lbl_802C42CC:
/* 802C42CC 002C122C  88 1A 00 B7 */	lbz r0, 0xb7(r26)
/* 802C42D0 002C1230  38 A0 00 07 */	li r5, 7
/* 802C42D4 002C1234  38 60 00 03 */	li r3, 3
/* 802C42D8 002C1238  88 DA 00 B6 */	lbz r6, 0xb6(r26)
/* 802C42DC 002C123C  38 80 00 00 */	li r4, 0
/* 802C42E0 002C1240  54 00 D7 BC */	rlwinm r0, r0, 0x1a, 0x1e, 0x1e
/* 802C42E4 002C1244  50 C0 07 FE */	rlwimi r0, r6, 0, 0x1f, 0x1f
/* 802C42E8 002C1248  98 A1 00 1C */	stb r5, 0x1c(r1)
/* 802C42EC 002C124C  50 60 04 3A */	rlwimi r0, r3, 0, 0x10, 0x1d
/* 802C42F0 002C1250  98 81 00 1D */	stb r4, 0x1d(r1)
/* 802C42F4 002C1254  B0 61 00 1E */	sth r3, 0x1e(r1)
/* 802C42F8 002C1258  93 C1 00 20 */	stw r30, 0x20(r1)
/* 802C42FC 002C125C  98 A1 00 3C */	stb r5, 0x3c(r1)
/* 802C4300 002C1260  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C4304 002C1264  B0 01 00 3E */	sth r0, 0x3e(r1)
/* 802C4308 002C1268  93 C1 00 40 */	stw r30, 0x40(r1)
/* 802C430C 002C126C  48 00 00 E0 */	b lbl_802C43EC
lbl_802C4310:
/* 802C4310 002C1270  88 1A 00 B6 */	lbz r0, 0xb6(r26)
/* 802C4314 002C1274  38 A0 00 05 */	li r5, 5
/* 802C4318 002C1278  38 60 00 03 */	li r3, 3
/* 802C431C 002C127C  38 80 00 00 */	li r4, 0
/* 802C4320 002C1280  54 06 07 FE */	clrlwi r6, r0, 0x1f
/* 802C4324 002C1284  54 00 FF FF */	rlwinm. r0, r0, 0x1f, 0x1f, 0x1f
/* 802C4328 002C1288  50 66 04 3A */	rlwimi r6, r3, 0, 0x10, 0x1d
/* 802C432C 002C128C  98 A1 00 14 */	stb r5, 0x14(r1)
/* 802C4330 002C1290  54 C0 04 3E */	clrlwi r0, r6, 0x10
/* 802C4334 002C1294  98 81 00 15 */	stb r4, 0x15(r1)
/* 802C4338 002C1298  B0 61 00 16 */	sth r3, 0x16(r1)
/* 802C433C 002C129C  93 C1 00 18 */	stw r30, 0x18(r1)
/* 802C4340 002C12A0  98 A1 00 3C */	stb r5, 0x3c(r1)
/* 802C4344 002C12A4  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C4348 002C12A8  B0 C1 00 3E */	sth r6, 0x3e(r1)
/* 802C434C 002C12AC  93 C1 00 40 */	stw r30, 0x40(r1)
/* 802C4350 002C12B0  41 82 00 18 */	beq lbl_802C4368
/* 802C4354 002C12B4  60 00 00 08 */	ori r0, r0, 8
/* 802C4358 002C12B8  98 A1 00 3C */	stb r5, 0x3c(r1)
/* 802C435C 002C12BC  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C4360 002C12C0  B0 01 00 3E */	sth r0, 0x3e(r1)
/* 802C4364 002C12C4  93 C1 00 40 */	stw r30, 0x40(r1)
lbl_802C4368:
/* 802C4368 002C12C8  7F A3 EB 78 */	mr r3, r29
/* 802C436C 002C12CC  38 81 00 3C */	addi r4, r1, 0x3c
/* 802C4370 002C12D0  48 09 18 65 */	bl Draw__6CModelCFRC11CModelFlags
/* 802C4374 002C12D4  88 1A 00 B7 */	lbz r0, 0xb7(r26)
/* 802C4378 002C12D8  38 C0 00 08 */	li r6, 8
/* 802C437C 002C12DC  38 80 00 03 */	li r4, 3
/* 802C4380 002C12E0  88 FA 00 B6 */	lbz r7, 0xb6(r26)
/* 802C4384 002C12E4  54 03 D7 BC */	rlwinm r3, r0, 0x1a, 0x1e, 0x1e
/* 802C4388 002C12E8  38 A0 00 00 */	li r5, 0
/* 802C438C 002C12EC  50 E3 07 FE */	rlwimi r3, r7, 0, 0x1f, 0x1f
/* 802C4390 002C12F0  54 E0 FF FF */	rlwinm. r0, r7, 0x1f, 0x1f, 0x1f
/* 802C4394 002C12F4  50 83 04 3A */	rlwimi r3, r4, 0, 0x10, 0x1d
/* 802C4398 002C12F8  98 C1 00 0C */	stb r6, 0xc(r1)
/* 802C439C 002C12FC  54 60 04 3E */	clrlwi r0, r3, 0x10
/* 802C43A0 002C1300  98 A1 00 0D */	stb r5, 0xd(r1)
/* 802C43A4 002C1304  B0 81 00 0E */	sth r4, 0xe(r1)
/* 802C43A8 002C1308  93 C1 00 10 */	stw r30, 0x10(r1)
/* 802C43AC 002C130C  98 C1 00 3C */	stb r6, 0x3c(r1)
/* 802C43B0 002C1310  98 A1 00 3D */	stb r5, 0x3d(r1)
/* 802C43B4 002C1314  B0 61 00 3E */	sth r3, 0x3e(r1)
/* 802C43B8 002C1318  93 C1 00 40 */	stw r30, 0x40(r1)
/* 802C43BC 002C131C  41 82 00 18 */	beq lbl_802C43D4
/* 802C43C0 002C1320  60 00 00 08 */	ori r0, r0, 8
/* 802C43C4 002C1324  98 C1 00 3C */	stb r6, 0x3c(r1)
/* 802C43C8 002C1328  98 A1 00 3D */	stb r5, 0x3d(r1)
/* 802C43CC 002C132C  B0 01 00 3E */	sth r0, 0x3e(r1)
/* 802C43D0 002C1330  93 C1 00 40 */	stw r30, 0x40(r1)
lbl_802C43D4:
/* 802C43D4 002C1334  7F A3 EB 78 */	mr r3, r29
/* 802C43D8 002C1338  38 81 00 3C */	addi r4, r1, 0x3c
/* 802C43DC 002C133C  48 09 17 F9 */	bl Draw__6CModelCFRC11CModelFlags
/* 802C43E0 002C1340  3B 80 00 00 */	li r28, 0
/* 802C43E4 002C1344  48 00 00 08 */	b lbl_802C43EC
lbl_802C43E8:
/* 802C43E8 002C1348  3B 80 00 00 */	li r28, 0
lbl_802C43EC:
/* 802C43EC 002C134C  57 80 06 3F */	clrlwi. r0, r28, 0x18
/* 802C43F0 002C1350  41 82 00 38 */	beq lbl_802C4428
/* 802C43F4 002C1354  88 1A 00 B6 */	lbz r0, 0xb6(r26)
/* 802C43F8 002C1358  54 00 FF FF */	rlwinm. r0, r0, 0x1f, 0x1f, 0x1f
/* 802C43FC 002C135C  41 82 00 20 */	beq lbl_802C441C
/* 802C4400 002C1360  A0 61 00 3E */	lhz r3, 0x3e(r1)
/* 802C4404 002C1364  88 81 00 3D */	lbz r4, 0x3d(r1)
/* 802C4408 002C1368  80 A1 00 40 */	lwz r5, 0x40(r1)
/* 802C440C 002C136C  60 63 00 08 */	ori r3, r3, 8
/* 802C4410 002C1370  98 81 00 3D */	stb r4, 0x3d(r1)
/* 802C4414 002C1374  B0 61 00 3E */	sth r3, 0x3e(r1)
/* 802C4418 002C1378  90 A1 00 40 */	stw r5, 0x40(r1)
lbl_802C441C:
/* 802C441C 002C137C  7F A3 EB 78 */	mr r3, r29
/* 802C4420 002C1380  38 81 00 3C */	addi r4, r1, 0x3c
/* 802C4424 002C1384  48 09 17 B1 */	bl Draw__6CModelCFRC11CModelFlags
lbl_802C4428:
/* 802C4428 002C1388  28 1F 00 00 */	cmplwi r31, 0
/* 802C442C 002C138C  41 82 00 0C */	beq lbl_802C4438
/* 802C4430 002C1390  38 60 00 00 */	li r3, 0
/* 802C4434 002C1394  48 04 76 3D */	bl SetCullMode__9CGraphicsF12ERglCullMode
lbl_802C4438:
/* 802C4438 002C1398  80 7A 00 B0 */	lwz r3, 0xb0(r26)
/* 802C443C 002C139C  4B FF E0 BD */	bl DisableLights__9CGuiFrameCFv
lbl_802C4440:
/* 802C4440 002C13A0  7F 43 D3 78 */	mr r3, r26
/* 802C4444 002C13A4  7F 64 DB 78 */	mr r4, r27
/* 802C4448 002C13A8  48 00 66 19 */	bl Draw__10CGuiWidgetCFRC19CGuiWidgetDrawParms
lbl_802C444C:
/* 802C444C 002C13AC  E3 E1 00 78 */	psq_l f31, 120(r1), 0, qr0
/* 802C4450 002C13B0  CB E1 00 70 */	lfd f31, 0x70(r1)
/* 802C4454 002C13B4  BB 41 00 58 */	lmw r26, 0x58(r1)
/* 802C4458 002C13B8  80 01 00 84 */	lwz r0, 0x84(r1)
/* 802C445C 002C13BC  7C 08 03 A6 */	mtlr r0
/* 802C4460 002C13C0  38 21 00 80 */	addi r1, r1, 0x80
/* 802C4464 002C13C4  4E 80 00 20 */	blr

.global __dt__9CGuiModelFv
__dt__9CGuiModelFv:
/* 802C4468 002C13C8  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 802C446C 002C13CC  7C 08 02 A6 */	mflr r0
/* 802C4470 002C13D0  90 01 00 14 */	stw r0, 0x14(r1)
/* 802C4474 002C13D4  93 E1 00 0C */	stw r31, 0xc(r1)
/* 802C4478 002C13D8  7C 9F 23 78 */	mr r31, r4
/* 802C447C 002C13DC  93 C1 00 08 */	stw r30, 8(r1)
/* 802C4480 002C13E0  7C 7E 1B 79 */	or. r30, r3, r3
/* 802C4484 002C13E4  41 82 00 60 */	beq lbl_802C44E4
/* 802C4488 002C13E8  3C 60 80 3F */	lis r3, lbl_803EC888@ha
/* 802C448C 002C13EC  34 1E 00 B8 */	addic. r0, r30, 0xb8
/* 802C4490 002C13F0  38 03 C8 88 */	addi r0, r3, lbl_803EC888@l
/* 802C4494 002C13F4  90 1E 00 00 */	stw r0, 0(r30)
/* 802C4498 002C13F8  41 82 00 30 */	beq lbl_802C44C8
/* 802C449C 002C13FC  88 1E 00 C4 */	lbz r0, 0xc4(r30)
/* 802C44A0 002C1400  28 00 00 00 */	cmplwi r0, 0
/* 802C44A4 002C1404  41 82 00 1C */	beq lbl_802C44C0
/* 802C44A8 002C1408  34 7E 00 B8 */	addic. r3, r30, 0xb8
/* 802C44AC 002C140C  41 82 00 14 */	beq lbl_802C44C0
/* 802C44B0 002C1410  28 03 00 00 */	cmplwi r3, 0
/* 802C44B4 002C1414  41 82 00 0C */	beq lbl_802C44C0
/* 802C44B8 002C1418  38 80 00 00 */	li r4, 0
/* 802C44BC 002C141C  48 07 C9 85 */	bl __dt__6CTokenFv
lbl_802C44C0:
/* 802C44C0 002C1420  38 00 00 00 */	li r0, 0
/* 802C44C4 002C1424  98 1E 00 C4 */	stb r0, 0xc4(r30)
lbl_802C44C8:
/* 802C44C8 002C1428  7F C3 F3 78 */	mr r3, r30
/* 802C44CC 002C142C  38 80 00 00 */	li r4, 0
/* 802C44D0 002C1430  48 00 66 E5 */	bl __dt__10CGuiWidgetFv
/* 802C44D4 002C1434  7F E0 07 35 */	extsh. r0, r31
/* 802C44D8 002C1438  40 81 00 0C */	ble lbl_802C44E4
/* 802C44DC 002C143C  7F C3 F3 78 */	mr r3, r30
/* 802C44E0 002C1440  48 05 14 51 */	bl Free__7CMemoryFPCv
lbl_802C44E4:
/* 802C44E4 002C1444  80 01 00 14 */	lwz r0, 0x14(r1)
/* 802C44E8 002C1448  7F C3 F3 78 */	mr r3, r30
/* 802C44EC 002C144C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 802C44F0 002C1450  83 C1 00 08 */	lwz r30, 8(r1)
/* 802C44F4 002C1454  7C 08 03 A6 */	mtlr r0
/* 802C44F8 002C1458  38 21 00 10 */	addi r1, r1, 0x10
/* 802C44FC 002C145C  4E 80 00 20 */	blr

.global __ct__9CGuiModelFRCQ210CGuiWidget15CGuiWidgetParmsUiUib
__ct__9CGuiModelFRCQ210CGuiWidget15CGuiWidgetParmsUiUib:
/* 802C4500 002C1460  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 802C4504 002C1464  7C 08 02 A6 */	mflr r0
/* 802C4508 002C1468  90 01 00 44 */	stw r0, 0x44(r1)
/* 802C450C 002C146C  BF 61 00 2C */	stmw r27, 0x2c(r1)
/* 802C4510 002C1470  7C 7F 1B 78 */	mr r31, r3
/* 802C4514 002C1474  7C BB 2B 78 */	mr r27, r5
/* 802C4518 002C1478  7C DC 33 78 */	mr r28, r6
/* 802C451C 002C147C  7C FD 3B 78 */	mr r29, r7
/* 802C4520 002C1480  7D 1E 43 78 */	mr r30, r8
/* 802C4524 002C1484  48 00 67 01 */	bl __ct__10CGuiWidgetFRCQ210CGuiWidget15CGuiWidgetParms
/* 802C4528 002C1488  3C 60 80 3F */	lis r3, lbl_803EC888@ha
/* 802C452C 002C148C  57 C0 06 3F */	clrlwi. r0, r30, 0x18
/* 802C4530 002C1490  38 83 C8 88 */	addi r4, r3, lbl_803EC888@l
/* 802C4534 002C1494  90 9F 00 00 */	stw r4, 0(r31)
/* 802C4538 002C1498  38 60 00 00 */	li r3, 0
/* 802C453C 002C149C  98 7F 00 C4 */	stb r3, 0xc4(r31)
/* 802C4540 002C14A0  93 9F 00 C8 */	stw r28, 0xc8(r31)
/* 802C4544 002C14A4  93 BF 00 CC */	stw r29, 0xcc(r31)
/* 802C4548 002C14A8  80 8D A6 40 */	lwz r4, spGuiSys__7CGuiSys@sda21(r13)
/* 802C454C 002C14AC  41 82 00 CC */	beq lbl_802C4618
/* 802C4550 002C14B0  80 7F 00 C8 */	lwz r3, 0xc8(r31)
/* 802C4554 002C14B4  3C 03 00 01 */	addis r0, r3, 1
/* 802C4558 002C14B8  28 00 FF FF */	cmplwi r0, 0xffff
/* 802C455C 002C14BC  41 82 00 BC */	beq lbl_802C4618
/* 802C4560 002C14C0  80 04 00 08 */	lwz r0, 8(r4)
/* 802C4564 002C14C4  2C 00 00 02 */	cmpwi r0, 2
/* 802C4568 002C14C8  41 82 00 B0 */	beq lbl_802C4618
/* 802C456C 002C14CC  3C 60 43 4D */	lis r3, 0x434D444C@ha
/* 802C4570 002C14D0  93 81 00 14 */	stw r28, 0x14(r1)
/* 802C4574 002C14D4  38 03 44 4C */	addi r0, r3, 0x434D444C@l
/* 802C4578 002C14D8  7F 64 DB 78 */	mr r4, r27
/* 802C457C 002C14DC  90 01 00 10 */	stw r0, 0x10(r1)
/* 802C4580 002C14E0  38 61 00 08 */	addi r3, r1, 8
/* 802C4584 002C14E4  38 A1 00 10 */	addi r5, r1, 0x10
/* 802C4588 002C14E8  81 9B 00 00 */	lwz r12, 0(r27)
/* 802C458C 002C14EC  81 8C 00 0C */	lwz r12, 0xc(r12)
/* 802C4590 002C14F0  7D 89 03 A6 */	mtctr r12
/* 802C4594 002C14F4  4E 80 04 21 */	bctrl
/* 802C4598 002C14F8  38 61 00 18 */	addi r3, r1, 0x18
/* 802C459C 002C14FC  38 81 00 08 */	addi r4, r1, 8
/* 802C45A0 002C1500  48 07 C9 09 */	bl __ct__6CTokenFRC6CToken
/* 802C45A4 002C1504  38 00 00 00 */	li r0, 0
/* 802C45A8 002C1508  90 01 00 20 */	stw r0, 0x20(r1)
/* 802C45AC 002C150C  88 1F 00 C4 */	lbz r0, 0xc4(r31)
/* 802C45B0 002C1510  28 00 00 00 */	cmplwi r0, 0
/* 802C45B4 002C1514  40 82 00 2C */	bne lbl_802C45E0
/* 802C45B8 002C1518  37 DF 00 B8 */	addic. r30, r31, 0xb8
/* 802C45BC 002C151C  41 82 00 18 */	beq lbl_802C45D4
/* 802C45C0 002C1520  7F C3 F3 78 */	mr r3, r30
/* 802C45C4 002C1524  38 81 00 18 */	addi r4, r1, 0x18
/* 802C45C8 002C1528  48 07 C8 E1 */	bl __ct__6CTokenFRC6CToken
/* 802C45CC 002C152C  80 01 00 20 */	lwz r0, 0x20(r1)
/* 802C45D0 002C1530  90 1E 00 08 */	stw r0, 8(r30)
lbl_802C45D4:
/* 802C45D4 002C1534  38 00 00 01 */	li r0, 1
/* 802C45D8 002C1538  98 1F 00 C4 */	stb r0, 0xc4(r31)
/* 802C45DC 002C153C  48 00 00 1C */	b lbl_802C45F8
lbl_802C45E0:
/* 802C45E0 002C1540  3B DF 00 B8 */	addi r30, r31, 0xb8
/* 802C45E4 002C1544  38 81 00 18 */	addi r4, r1, 0x18
/* 802C45E8 002C1548  7F C3 F3 78 */	mr r3, r30
/* 802C45EC 002C154C  48 07 C6 D9 */	bl __as__6CTokenFRC6CToken
/* 802C45F0 002C1550  80 01 00 20 */	lwz r0, 0x20(r1)
/* 802C45F4 002C1554  90 1E 00 08 */	stw r0, 8(r30)
lbl_802C45F8:
/* 802C45F8 002C1558  38 61 00 18 */	addi r3, r1, 0x18
/* 802C45FC 002C155C  38 80 00 00 */	li r4, 0
/* 802C4600 002C1560  48 07 C8 41 */	bl __dt__6CTokenFv
/* 802C4604 002C1564  38 61 00 08 */	addi r3, r1, 8
/* 802C4608 002C1568  38 80 FF FF */	li r4, -1
/* 802C460C 002C156C  48 07 C8 35 */	bl __dt__6CTokenFv
/* 802C4610 002C1570  38 7F 00 B8 */	addi r3, r31, 0xb8
/* 802C4614 002C1574  48 07 C7 71 */	bl Lock__6CTokenFv
lbl_802C4618:
/* 802C4618 002C1578  7F E3 FB 78 */	mr r3, r31
/* 802C461C 002C157C  BB 61 00 2C */	lmw r27, 0x2c(r1)
/* 802C4620 002C1580  80 01 00 44 */	lwz r0, 0x44(r1)
/* 802C4624 002C1584  7C 08 03 A6 */	mtlr r0
/* 802C4628 002C1588  38 21 00 40 */	addi r1, r1, 0x40
/* 802C462C 002C158C  4E 80 00 20 */	blr

.global Create__9CGuiModelFP9CGuiFrameR12CInputStreamb
Create__9CGuiModelFP9CGuiFrameR12CInputStreamb:
/* 802C4630 002C1590  94 21 FF B0 */	stwu r1, -0x50(r1)
/* 802C4634 002C1594  7C 08 02 A6 */	mflr r0
/* 802C4638 002C1598  90 01 00 54 */	stw r0, 0x54(r1)
/* 802C463C 002C159C  BF 41 00 38 */	stmw r26, 0x38(r1)
/* 802C4640 002C15A0  7C 7D 1B 78 */	mr r29, r3
/* 802C4644 002C15A4  7C 9E 23 78 */	mr r30, r4
/* 802C4648 002C15A8  7C BF 2B 78 */	mr r31, r5
/* 802C464C 002C15AC  38 61 00 08 */	addi r3, r1, 8
/* 802C4650 002C15B0  7F A4 EB 78 */	mr r4, r29
/* 802C4654 002C15B4  7F C5 F3 78 */	mr r5, r30
/* 802C4658 002C15B8  48 00 66 F9 */	bl ReadWidgetHeader__10CGuiWidgetFP9CGuiFrameR12CInputStream
/* 802C465C 002C15BC  83 81 00 08 */	lwz r28, 8(r1)
/* 802C4660 002C15C0  7F C3 F3 78 */	mr r3, r30
/* 802C4664 002C15C4  89 81 00 0C */	lbz r12, 0xc(r1)
/* 802C4668 002C15C8  A9 61 00 0E */	lha r11, 0xe(r1)
/* 802C466C 002C15CC  A9 41 00 10 */	lha r10, 0x10(r1)
/* 802C4670 002C15D0  89 21 00 12 */	lbz r9, 0x12(r1)
/* 802C4674 002C15D4  89 01 00 13 */	lbz r8, 0x13(r1)
/* 802C4678 002C15D8  88 E1 00 14 */	lbz r7, 0x14(r1)
/* 802C467C 002C15DC  88 C1 00 15 */	lbz r6, 0x15(r1)
/* 802C4680 002C15E0  88 A1 00 16 */	lbz r5, 0x16(r1)
/* 802C4684 002C15E4  80 81 00 18 */	lwz r4, 0x18(r1)
/* 802C4688 002C15E8  80 01 00 1C */	lwz r0, 0x1c(r1)
/* 802C468C 002C15EC  93 81 00 20 */	stw r28, 0x20(r1)
/* 802C4690 002C15F0  99 81 00 24 */	stb r12, 0x24(r1)
/* 802C4694 002C15F4  B1 61 00 26 */	sth r11, 0x26(r1)
/* 802C4698 002C15F8  B1 41 00 28 */	sth r10, 0x28(r1)
/* 802C469C 002C15FC  99 21 00 2A */	stb r9, 0x2a(r1)
/* 802C46A0 002C1600  99 01 00 2B */	stb r8, 0x2b(r1)
/* 802C46A4 002C1604  98 E1 00 2C */	stb r7, 0x2c(r1)
/* 802C46A8 002C1608  98 C1 00 2D */	stb r6, 0x2d(r1)
/* 802C46AC 002C160C  98 A1 00 2E */	stb r5, 0x2e(r1)
/* 802C46B0 002C1610  90 81 00 30 */	stw r4, 0x30(r1)
/* 802C46B4 002C1614  90 01 00 34 */	stw r0, 0x34(r1)
/* 802C46B8 002C1618  48 07 A5 CD */	bl ReadLong__12CInputStreamFv
/* 802C46BC 002C161C  7C 7C 1B 78 */	mr r28, r3
/* 802C46C0 002C1620  7F C3 F3 78 */	mr r3, r30
/* 802C46C4 002C1624  48 07 A5 C1 */	bl ReadLong__12CInputStreamFv
/* 802C46C8 002C1628  7F C3 F3 78 */	mr r3, r30
/* 802C46CC 002C162C  48 07 A5 B9 */	bl ReadLong__12CInputStreamFv
/* 802C46D0 002C1630  3C 80 80 3D */	lis r4, lbl_803D68B0@ha
/* 802C46D4 002C1634  7C 7B 1B 78 */	mr r27, r3
/* 802C46D8 002C1638  38 84 68 B0 */	addi r4, r4, lbl_803D68B0@l
/* 802C46DC 002C163C  38 60 00 D0 */	li r3, 0xd0
/* 802C46E0 002C1640  38 A0 00 00 */	li r5, 0
/* 802C46E4 002C1644  48 05 11 89 */	bl __nw__FUlPCcPCc
/* 802C46E8 002C1648  7C 7A 1B 79 */	or. r26, r3, r3
/* 802C46EC 002C164C  41 82 00 20 */	beq lbl_802C470C
/* 802C46F0 002C1650  7F E5 FB 78 */	mr r5, r31
/* 802C46F4 002C1654  7F 86 E3 78 */	mr r6, r28
/* 802C46F8 002C1658  7F 67 DB 78 */	mr r7, r27
/* 802C46FC 002C165C  38 81 00 20 */	addi r4, r1, 0x20
/* 802C4700 002C1660  39 00 00 01 */	li r8, 1
/* 802C4704 002C1664  4B FF FD FD */	bl __ct__9CGuiModelFRCQ210CGuiWidget15CGuiWidgetParmsUiUib
/* 802C4708 002C1668  7C 7A 1B 78 */	mr r26, r3
lbl_802C470C:
/* 802C470C 002C166C  7F 43 D3 78 */	mr r3, r26
/* 802C4710 002C1670  7F A4 EB 78 */	mr r4, r29
/* 802C4714 002C1674  7F C5 F3 78 */	mr r5, r30
/* 802C4718 002C1678  38 C1 00 20 */	addi r6, r1, 0x20
/* 802C471C 002C167C  48 00 63 6D */	bl ParseBaseInfo__10CGuiWidgetFP9CGuiFrameR12CInputStreamRCQ210CGuiWidget15CGuiWidgetParms
/* 802C4720 002C1680  7F 43 D3 78 */	mr r3, r26
/* 802C4724 002C1684  BB 41 00 38 */	lmw r26, 0x38(r1)
/* 802C4728 002C1688  80 01 00 54 */	lwz r0, 0x54(r1)
/* 802C472C 002C168C  7C 08 03 A6 */	mtlr r0
/* 802C4730 002C1690  38 21 00 50 */	addi r1, r1, 0x50
/* 802C4734 002C1694  4E 80 00 20 */	blr

.section .sdata2, "a"
.balign 8
.global lbl_805AE0A8
lbl_805AE0A8:
	# ROM: 0x3FA948
	.float 1.0
	.4byte 0

.global lbl_805AE0B0
lbl_805AE0B0:
	# ROM: 0x3FA950
	.4byte 0x43300000
	.4byte 0


.section .rodata
.balign 8
.global lbl_803D68B0
lbl_803D68B0:
	# ROM: 0x3D38B0
	.asciz "??(??)"
	.balign 4
