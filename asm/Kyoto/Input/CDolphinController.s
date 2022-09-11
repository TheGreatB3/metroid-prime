.include "macros.inc"

.section .data
.balign 8

.global lbl_803EF6A0
lbl_803EF6A0:
	# ROM: 0x3EC6A0
	.4byte 0x01000200
	.4byte 0x04000800
	.4byte 0x10000010
	.4byte 0x00080002
	.4byte 0x00040001
	.4byte 0x00400020

.global lbl_803EF6B8
lbl_803EF6B8:
	# ROM: 0x3EC6B8
	.4byte 0
	.4byte 0
	.4byte __dt__18CDolphinControllerFv
	.4byte Poll__18CDolphinControllerFv
	.4byte GetDeviceCount__18CDolphinControllerCFv
	.4byte GetGamepadData__18CDolphinControllerFi
	.4byte GetControllerType__18CDolphinControllerFi
	.4byte SetMotorState__18CDolphinControllerF7EIOPort11EMotorState

.section .sbss, "wa"
.balign 8

.global lbl_805A95C8
lbl_805A95C8:
	.skip 0x1
.global lbl_805A95C9
lbl_805A95C9:
	.skip 0x7

.section .text, "ax"

.global GetAnalogStickMaxValue__18CDolphinControllerCF8EJoyAxis
GetAnalogStickMaxValue__18CDolphinControllerCF8EJoyAxis:
/* 8034F060 0034BFC0  2C 04 00 02 */	cmpwi r4, 2
/* 8034F064 0034BFC4  40 80 00 10 */	bge lbl_8034F074
/* 8034F068 0034BFC8  2C 04 00 00 */	cmpwi r4, 0
/* 8034F06C 0034BFCC  40 80 00 14 */	bge lbl_8034F080
/* 8034F070 0034BFD0  48 00 00 20 */	b lbl_8034F090
lbl_8034F074:
/* 8034F074 0034BFD4  2C 04 00 04 */	cmpwi r4, 4
/* 8034F078 0034BFD8  40 80 00 18 */	bge lbl_8034F090
/* 8034F07C 0034BFDC  48 00 00 0C */	b lbl_8034F088
lbl_8034F080:
/* 8034F080 0034BFE0  C0 22 CC 70 */	lfs f1, lbl_805AE990@sda21(r2)
/* 8034F084 0034BFE4  4E 80 00 20 */	blr
lbl_8034F088:
/* 8034F088 0034BFE8  C0 22 CC 74 */	lfs f1, lbl_805AE994@sda21(r2)
/* 8034F08C 0034BFEC  4E 80 00 20 */	blr
lbl_8034F090:
/* 8034F090 0034BFF0  C0 22 CC 78 */	lfs f1, lbl_805AE998@sda21(r2)
/* 8034F094 0034BFF4  4E 80 00 20 */	blr

.global SetMotorState__18CDolphinControllerF7EIOPort11EMotorState
SetMotorState__18CDolphinControllerF7EIOPort11EMotorState:
/* 8034F098 0034BFF8  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F09C 0034BFFC  7C 08 02 A6 */	mflr r0
/* 8034F0A0 0034C000  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F0A4 0034C004  54 80 10 3A */	slwi r0, r4, 2
/* 8034F0A8 0034C008  7C 83 02 14 */	add r4, r3, r0
/* 8034F0AC 0034C00C  38 63 01 94 */	addi r3, r3, 0x194
/* 8034F0B0 0034C010  90 A4 01 94 */	stw r5, 0x194(r4)
/* 8034F0B4 0034C014  48 03 7A 71 */	bl PADControlAllMotors
/* 8034F0B8 0034C018  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F0BC 0034C01C  7C 08 03 A6 */	mtlr r0
/* 8034F0C0 0034C020  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F0C4 0034C024  4E 80 00 20 */	blr

.global GetControllerType__18CDolphinControllerFi
GetControllerType__18CDolphinControllerFi:
/* 8034F0C8 0034C028  54 80 10 3A */	slwi r0, r4, 2
/* 8034F0CC 0034C02C  7C 63 02 14 */	add r3, r3, r0
/* 8034F0D0 0034C030  80 63 01 A4 */	lwz r3, 0x1a4(r3)
/* 8034F0D4 0034C034  4E 80 00 20 */	blr

.global GetGamepadData__18CDolphinControllerFi
GetGamepadData__18CDolphinControllerFi:
/* 8034F0D8 0034C038  1C 84 00 58 */	mulli r4, r4, 0x58
/* 8034F0DC 0034C03C  7C 60 1B 78 */	mr r0, r3
/* 8034F0E0 0034C040  38 64 00 34 */	addi r3, r4, 0x34
/* 8034F0E4 0034C044  7C 60 1A 14 */	add r3, r0, r3
/* 8034F0E8 0034C048  4E 80 00 20 */	blr

.global GetDeviceCount__18CDolphinControllerCFv
GetDeviceCount__18CDolphinControllerCFv:
/* 8034F0EC 0034C04C  38 60 00 04 */	li r3, 4
/* 8034F0F0 0034C050  4E 80 00 20 */	blr

.global ProcessAnalogButton__18CDolphinControllerFfR15CControllerAxis
ProcessAnalogButton__18CDolphinControllerFfR15CControllerAxis:
/* 8034F0F4 0034C054  C0 02 CC 7C */	lfs f0, lbl_805AE99C@sda21(r2)
/* 8034F0F8 0034C058  C0 42 CC 54 */	lfs f2, kAbsoluteMaximum__11IController@sda21(r2)
/* 8034F0FC 0034C05C  EC 61 00 32 */	fmuls f3, f1, f0
/* 8034F100 0034C060  FC 03 10 40 */	fcmpo cr0, f3, f2
/* 8034F104 0034C064  40 81 00 08 */	ble lbl_8034F10C
/* 8034F108 0034C068  FC 60 10 90 */	fmr f3, f2
lbl_8034F10C:
/* 8034F10C 0034C06C  C0 04 00 04 */	lfs f0, 4(r4)
/* 8034F110 0034C070  C0 22 CC 5C */	lfs f1, kRelativeMaximum__11IController@sda21(r2)
/* 8034F114 0034C074  EC 03 00 28 */	fsubs f0, f3, f0
/* 8034F118 0034C078  FC 00 08 40 */	fcmpo cr0, f0, f1
/* 8034F11C 0034C07C  40 81 00 08 */	ble lbl_8034F124
/* 8034F120 0034C080  FC 00 08 90 */	fmr f0, f1
lbl_8034F124:
/* 8034F124 0034C084  D0 04 00 00 */	stfs f0, 0(r4)
/* 8034F128 0034C088  D0 64 00 04 */	stfs f3, 4(r4)
/* 8034F12C 0034C08C  4E 80 00 20 */	blr

.global ProcessDigitalButton__18CDolphinControllerFiR17CControllerButtonUs
ProcessDigitalButton__18CDolphinControllerFiR17CControllerButtonUs:
/* 8034F130 0034C090  1C E4 00 0C */	mulli r7, r4, 0xc
/* 8034F134 0034C094  54 C4 04 3E */	clrlwi r4, r6, 0x10
/* 8034F138 0034C098  88 05 00 00 */	lbz r0, 0(r5)
/* 8034F13C 0034C09C  7C 63 3A 14 */	add r3, r3, r7
/* 8034F140 0034C0A0  A0 63 00 04 */	lhz r3, 4(r3)
/* 8034F144 0034C0A4  7C 64 20 38 */	and r4, r3, r4
/* 8034F148 0034C0A8  7C 64 00 D0 */	neg r3, r4
/* 8034F14C 0034C0AC  7C 63 23 78 */	or r3, r3, r4
/* 8034F150 0034C0B0  54 63 0F FE */	srwi r3, r3, 0x1f
/* 8034F154 0034C0B4  7C 60 02 78 */	xor r0, r3, r0
/* 8034F158 0034C0B8  7C 60 00 38 */	and r0, r3, r0
/* 8034F15C 0034C0BC  7C 00 00 34 */	cntlzw r0, r0
/* 8034F160 0034C0C0  54 00 DE 3E */	rlwinm r0, r0, 0x1b, 0x18, 0x1f
/* 8034F164 0034C0C4  7C 00 00 34 */	cntlzw r0, r0
/* 8034F168 0034C0C8  54 00 DE 3E */	rlwinm r0, r0, 0x1b, 0x18, 0x1f
/* 8034F16C 0034C0CC  98 05 00 01 */	stb r0, 1(r5)
/* 8034F170 0034C0D0  88 85 00 00 */	lbz r4, 0(r5)
/* 8034F174 0034C0D4  7C 60 22 78 */	xor r0, r3, r4
/* 8034F178 0034C0D8  7C 80 00 38 */	and r0, r4, r0
/* 8034F17C 0034C0DC  7C 00 00 34 */	cntlzw r0, r0
/* 8034F180 0034C0E0  54 00 DE 3E */	rlwinm r0, r0, 0x1b, 0x18, 0x1f
/* 8034F184 0034C0E4  7C 00 00 34 */	cntlzw r0, r0
/* 8034F188 0034C0E8  54 00 DE 3E */	rlwinm r0, r0, 0x1b, 0x18, 0x1f
/* 8034F18C 0034C0EC  98 05 00 02 */	stb r0, 2(r5)
/* 8034F190 0034C0F0  98 65 00 00 */	stb r3, 0(r5)
/* 8034F194 0034C0F4  4E 80 00 20 */	blr

.global ProcessButtons__18CDolphinControllerFi
ProcessButtons__18CDolphinControllerFi:
/* 8034F198 0034C0F8  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 8034F19C 0034C0FC  7C 08 02 A6 */	mflr r0
/* 8034F1A0 0034C100  3C A0 80 3F */	lis r5, lbl_803EF6A0@ha
/* 8034F1A4 0034C104  90 01 00 34 */	stw r0, 0x34(r1)
/* 8034F1A8 0034C108  BF 41 00 18 */	stmw r26, 0x18(r1)
/* 8034F1AC 0034C10C  7C 9B 23 78 */	mr r27, r4
/* 8034F1B0 0034C110  1C 9B 00 58 */	mulli r4, r27, 0x58
/* 8034F1B4 0034C114  7C 7A 1B 78 */	mr r26, r3
/* 8034F1B8 0034C118  3B C5 F6 A0 */	addi r30, r5, lbl_803EF6A0@l
/* 8034F1BC 0034C11C  3B 80 00 00 */	li r28, 0
/* 8034F1C0 0034C120  3B A4 00 34 */	addi r29, r4, 0x34
/* 8034F1C4 0034C124  3B E0 00 00 */	li r31, 0
/* 8034F1C8 0034C128  7F BA EA 14 */	add r29, r26, r29
lbl_8034F1CC:
/* 8034F1CC 0034C12C  38 BF 00 34 */	addi r5, r31, 0x34
/* 8034F1D0 0034C130  A0 DE 00 00 */	lhz r6, 0(r30)
/* 8034F1D4 0034C134  7F 43 D3 78 */	mr r3, r26
/* 8034F1D8 0034C138  7F 64 DB 78 */	mr r4, r27
/* 8034F1DC 0034C13C  7C BD 2A 14 */	add r5, r29, r5
/* 8034F1E0 0034C140  4B FF FF 51 */	bl ProcessDigitalButton__18CDolphinControllerFiR17CControllerButtonUs
/* 8034F1E4 0034C144  3B 9C 00 01 */	addi r28, r28, 1
/* 8034F1E8 0034C148  3B FF 00 03 */	addi r31, r31, 3
/* 8034F1EC 0034C14C  2C 1C 00 0C */	cmpwi r28, 0xc
/* 8034F1F0 0034C150  3B DE 00 02 */	addi r30, r30, 2
/* 8034F1F4 0034C154  41 80 FF D8 */	blt lbl_8034F1CC
/* 8034F1F8 0034C158  1F DB 00 0C */	mulli r30, r27, 0xc
/* 8034F1FC 0034C15C  3C 00 43 30 */	lis r0, 0x4330
/* 8034F200 0034C160  90 01 00 08 */	stw r0, 8(r1)
/* 8034F204 0034C164  7F 43 D3 78 */	mr r3, r26
/* 8034F208 0034C168  C8 22 CC 80 */	lfd f1, lbl_805AE9A0@sda21(r2)
/* 8034F20C 0034C16C  38 9D 00 24 */	addi r4, r29, 0x24
/* 8034F210 0034C170  7C BA F2 14 */	add r5, r26, r30
/* 8034F214 0034C174  88 05 00 0A */	lbz r0, 0xa(r5)
/* 8034F218 0034C178  90 01 00 0C */	stw r0, 0xc(r1)
/* 8034F21C 0034C17C  C8 01 00 08 */	lfd f0, 8(r1)
/* 8034F220 0034C180  EC 20 08 28 */	fsubs f1, f0, f1
/* 8034F224 0034C184  4B FF FE D1 */	bl ProcessAnalogButton__18CDolphinControllerFfR15CControllerAxis
/* 8034F228 0034C188  7C 7A F2 14 */	add r3, r26, r30
/* 8034F22C 0034C18C  3C 00 43 30 */	lis r0, 0x4330
/* 8034F230 0034C190  88 A3 00 0B */	lbz r5, 0xb(r3)
/* 8034F234 0034C194  7F 43 D3 78 */	mr r3, r26
/* 8034F238 0034C198  90 01 00 10 */	stw r0, 0x10(r1)
/* 8034F23C 0034C19C  38 9D 00 2C */	addi r4, r29, 0x2c
/* 8034F240 0034C1A0  C8 22 CC 80 */	lfd f1, lbl_805AE9A0@sda21(r2)
/* 8034F244 0034C1A4  90 A1 00 14 */	stw r5, 0x14(r1)
/* 8034F248 0034C1A8  C8 01 00 10 */	lfd f0, 0x10(r1)
/* 8034F24C 0034C1AC  EC 20 08 28 */	fsubs f1, f0, f1
/* 8034F250 0034C1B0  4B FF FE A5 */	bl ProcessAnalogButton__18CDolphinControllerFfR15CControllerAxis
/* 8034F254 0034C1B4  BB 41 00 18 */	lmw r26, 0x18(r1)
/* 8034F258 0034C1B8  80 01 00 34 */	lwz r0, 0x34(r1)
/* 8034F25C 0034C1BC  7C 08 03 A6 */	mtlr r0
/* 8034F260 0034C1C0  38 21 00 30 */	addi r1, r1, 0x30
/* 8034F264 0034C1C4  4E 80 00 20 */	blr

.global ProcessAxis__18CDolphinControllerFi8EJoyAxis
ProcessAxis__18CDolphinControllerFi8EJoyAxis:
/* 8034F268 0034C1C8  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8034F26C 0034C1CC  7C 08 02 A6 */	mflr r0
/* 8034F270 0034C1D0  90 01 00 24 */	stw r0, 0x24(r1)
/* 8034F274 0034C1D4  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8034F278 0034C1D8  7C BF 2B 78 */	mr r31, r5
/* 8034F27C 0034C1DC  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8034F280 0034C1E0  7C 9E 23 78 */	mr r30, r4
/* 8034F284 0034C1E4  7F E4 FB 78 */	mr r4, r31
/* 8034F288 0034C1E8  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8034F28C 0034C1EC  7C 7D 1B 78 */	mr r29, r3
/* 8034F290 0034C1F0  4B FF FD D1 */	bl GetAnalogStickMaxValue__18CDolphinControllerCF8EJoyAxis
/* 8034F294 0034C1F4  C0 02 CC 88 */	lfs f0, lbl_805AE9A8@sda21(r2)
/* 8034F298 0034C1F8  1C 7E 00 58 */	mulli r3, r30, 0x58
/* 8034F29C 0034C1FC  57 E0 18 38 */	slwi r0, r31, 3
/* 8034F2A0 0034C200  C0 62 CC 78 */	lfs f3, lbl_805AE998@sda21(r2)
/* 8034F2A4 0034C204  EC 40 08 24 */	fdivs f2, f0, f1
/* 8034F2A8 0034C208  2C 1F 00 02 */	cmpwi r31, 2
/* 8034F2AC 0034C20C  38 63 00 34 */	addi r3, r3, 0x34
/* 8034F2B0 0034C210  7C 63 02 14 */	add r3, r3, r0
/* 8034F2B4 0034C214  38 83 00 04 */	addi r4, r3, 4
/* 8034F2B8 0034C218  7C 9D 22 14 */	add r4, r29, r4
/* 8034F2BC 0034C21C  41 82 00 84 */	beq lbl_8034F340
/* 8034F2C0 0034C220  40 80 00 14 */	bge lbl_8034F2D4
/* 8034F2C4 0034C224  2C 1F 00 00 */	cmpwi r31, 0
/* 8034F2C8 0034C228  41 82 00 18 */	beq lbl_8034F2E0
/* 8034F2CC 0034C22C  40 80 00 44 */	bge lbl_8034F310
/* 8034F2D0 0034C230  48 00 00 CC */	b lbl_8034F39C
lbl_8034F2D4:
/* 8034F2D4 0034C234  2C 1F 00 04 */	cmpwi r31, 4
/* 8034F2D8 0034C238  40 80 00 C4 */	bge lbl_8034F39C
/* 8034F2DC 0034C23C  48 00 00 94 */	b lbl_8034F370
lbl_8034F2E0:
/* 8034F2E0 0034C240  1C 7E 00 0C */	mulli r3, r30, 0xc
/* 8034F2E4 0034C244  3C 00 43 30 */	lis r0, 0x4330
/* 8034F2E8 0034C248  90 01 00 08 */	stw r0, 8(r1)
/* 8034F2EC 0034C24C  C8 22 CC 90 */	lfd f1, lbl_805AE9B0@sda21(r2)
/* 8034F2F0 0034C250  7C 7D 1A 14 */	add r3, r29, r3
/* 8034F2F4 0034C254  88 03 00 06 */	lbz r0, 6(r3)
/* 8034F2F8 0034C258  7C 00 07 74 */	extsb r0, r0
/* 8034F2FC 0034C25C  6C 00 80 00 */	xoris r0, r0, 0x8000
/* 8034F300 0034C260  90 01 00 0C */	stw r0, 0xc(r1)
/* 8034F304 0034C264  C8 01 00 08 */	lfd f0, 8(r1)
/* 8034F308 0034C268  EC 60 08 28 */	fsubs f3, f0, f1
/* 8034F30C 0034C26C  48 00 00 90 */	b lbl_8034F39C
lbl_8034F310:
/* 8034F310 0034C270  1C 7E 00 0C */	mulli r3, r30, 0xc
/* 8034F314 0034C274  3C 00 43 30 */	lis r0, 0x4330
/* 8034F318 0034C278  90 01 00 08 */	stw r0, 8(r1)
/* 8034F31C 0034C27C  C8 22 CC 90 */	lfd f1, lbl_805AE9B0@sda21(r2)
/* 8034F320 0034C280  7C 7D 1A 14 */	add r3, r29, r3
/* 8034F324 0034C284  88 03 00 07 */	lbz r0, 7(r3)
/* 8034F328 0034C288  7C 00 07 74 */	extsb r0, r0
/* 8034F32C 0034C28C  6C 00 80 00 */	xoris r0, r0, 0x8000
/* 8034F330 0034C290  90 01 00 0C */	stw r0, 0xc(r1)
/* 8034F334 0034C294  C8 01 00 08 */	lfd f0, 8(r1)
/* 8034F338 0034C298  EC 60 08 28 */	fsubs f3, f0, f1
/* 8034F33C 0034C29C  48 00 00 60 */	b lbl_8034F39C
lbl_8034F340:
/* 8034F340 0034C2A0  1C 7E 00 0C */	mulli r3, r30, 0xc
/* 8034F344 0034C2A4  3C 00 43 30 */	lis r0, 0x4330
/* 8034F348 0034C2A8  90 01 00 08 */	stw r0, 8(r1)
/* 8034F34C 0034C2AC  C8 22 CC 90 */	lfd f1, lbl_805AE9B0@sda21(r2)
/* 8034F350 0034C2B0  7C 7D 1A 14 */	add r3, r29, r3
/* 8034F354 0034C2B4  88 03 00 08 */	lbz r0, 8(r3)
/* 8034F358 0034C2B8  7C 00 07 74 */	extsb r0, r0
/* 8034F35C 0034C2BC  6C 00 80 00 */	xoris r0, r0, 0x8000
/* 8034F360 0034C2C0  90 01 00 0C */	stw r0, 0xc(r1)
/* 8034F364 0034C2C4  C8 01 00 08 */	lfd f0, 8(r1)
/* 8034F368 0034C2C8  EC 60 08 28 */	fsubs f3, f0, f1
/* 8034F36C 0034C2CC  48 00 00 30 */	b lbl_8034F39C
lbl_8034F370:
/* 8034F370 0034C2D0  1C 7E 00 0C */	mulli r3, r30, 0xc
/* 8034F374 0034C2D4  3C 00 43 30 */	lis r0, 0x4330
/* 8034F378 0034C2D8  90 01 00 08 */	stw r0, 8(r1)
/* 8034F37C 0034C2DC  C8 22 CC 90 */	lfd f1, lbl_805AE9B0@sda21(r2)
/* 8034F380 0034C2E0  7C 7D 1A 14 */	add r3, r29, r3
/* 8034F384 0034C2E4  88 03 00 09 */	lbz r0, 9(r3)
/* 8034F388 0034C2E8  7C 00 07 74 */	extsb r0, r0
/* 8034F38C 0034C2EC  6C 00 80 00 */	xoris r0, r0, 0x8000
/* 8034F390 0034C2F0  90 01 00 0C */	stw r0, 0xc(r1)
/* 8034F394 0034C2F4  C8 01 00 08 */	lfd f0, 8(r1)
/* 8034F398 0034C2F8  EC 60 08 28 */	fsubs f3, f0, f1
lbl_8034F39C:
/* 8034F39C 0034C2FC  EC 43 00 B2 */	fmuls f2, f3, f2
/* 8034F3A0 0034C300  C0 02 CC 50 */	lfs f0, kAbsoluteMinimum__11IController@sda21(r2)
/* 8034F3A4 0034C304  FC 02 00 40 */	fcmpo cr0, f2, f0
/* 8034F3A8 0034C308  40 80 00 0C */	bge lbl_8034F3B4
/* 8034F3AC 0034C30C  FC 40 00 90 */	fmr f2, f0
/* 8034F3B0 0034C310  48 00 00 14 */	b lbl_8034F3C4
lbl_8034F3B4:
/* 8034F3B4 0034C314  C0 02 CC 54 */	lfs f0, kAbsoluteMaximum__11IController@sda21(r2)
/* 8034F3B8 0034C318  FC 02 00 40 */	fcmpo cr0, f2, f0
/* 8034F3BC 0034C31C  40 81 00 08 */	ble lbl_8034F3C4
/* 8034F3C0 0034C320  FC 40 00 90 */	fmr f2, f0
lbl_8034F3C4:
/* 8034F3C4 0034C324  C0 04 00 04 */	lfs f0, 4(r4)
/* 8034F3C8 0034C328  C0 22 CC 58 */	lfs f1, kRelativeMinimum__11IController@sda21(r2)
/* 8034F3CC 0034C32C  EC 62 00 28 */	fsubs f3, f2, f0
/* 8034F3D0 0034C330  FC 03 08 40 */	fcmpo cr0, f3, f1
/* 8034F3D4 0034C334  40 80 00 0C */	bge lbl_8034F3E0
/* 8034F3D8 0034C338  FC 60 08 90 */	fmr f3, f1
/* 8034F3DC 0034C33C  48 00 00 14 */	b lbl_8034F3F0
lbl_8034F3E0:
/* 8034F3E0 0034C340  C0 02 CC 5C */	lfs f0, kRelativeMaximum__11IController@sda21(r2)
/* 8034F3E4 0034C344  FC 03 00 40 */	fcmpo cr0, f3, f0
/* 8034F3E8 0034C348  40 81 00 08 */	ble lbl_8034F3F0
/* 8034F3EC 0034C34C  FC 60 00 90 */	fmr f3, f0
lbl_8034F3F0:
/* 8034F3F0 0034C350  D0 64 00 00 */	stfs f3, 0(r4)
/* 8034F3F4 0034C354  D0 44 00 04 */	stfs f2, 4(r4)
/* 8034F3F8 0034C358  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8034F3FC 0034C35C  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8034F400 0034C360  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8034F404 0034C364  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8034F408 0034C368  7C 08 03 A6 */	mtlr r0
/* 8034F40C 0034C36C  38 21 00 20 */	addi r1, r1, 0x20
/* 8034F410 0034C370  4E 80 00 20 */	blr

.global ProcessInputData__18CDolphinControllerFv
ProcessInputData__18CDolphinControllerFv:
/* 8034F414 0034C374  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8034F418 0034C378  7C 08 02 A6 */	mflr r0
/* 8034F41C 0034C37C  90 01 00 24 */	stw r0, 0x24(r1)
/* 8034F420 0034C380  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8034F424 0034C384  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8034F428 0034C388  3B C0 00 00 */	li r30, 0
/* 8034F42C 0034C38C  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8034F430 0034C390  7C 7D 1B 78 */	mr r29, r3
/* 8034F434 0034C394  7F BF EB 78 */	mr r31, r29
lbl_8034F438:
/* 8034F438 0034C398  88 1F 00 34 */	lbz r0, 0x34(r31)
/* 8034F43C 0034C39C  28 00 00 00 */	cmplwi r0, 0
/* 8034F440 0034C3A0  41 82 00 50 */	beq lbl_8034F490
/* 8034F444 0034C3A4  7F A3 EB 78 */	mr r3, r29
/* 8034F448 0034C3A8  7F C4 F3 78 */	mr r4, r30
/* 8034F44C 0034C3AC  38 A0 00 00 */	li r5, 0
/* 8034F450 0034C3B0  4B FF FE 19 */	bl ProcessAxis__18CDolphinControllerFi8EJoyAxis
/* 8034F454 0034C3B4  7F A3 EB 78 */	mr r3, r29
/* 8034F458 0034C3B8  7F C4 F3 78 */	mr r4, r30
/* 8034F45C 0034C3BC  38 A0 00 01 */	li r5, 1
/* 8034F460 0034C3C0  4B FF FE 09 */	bl ProcessAxis__18CDolphinControllerFi8EJoyAxis
/* 8034F464 0034C3C4  7F A3 EB 78 */	mr r3, r29
/* 8034F468 0034C3C8  7F C4 F3 78 */	mr r4, r30
/* 8034F46C 0034C3CC  38 A0 00 02 */	li r5, 2
/* 8034F470 0034C3D0  4B FF FD F9 */	bl ProcessAxis__18CDolphinControllerFi8EJoyAxis
/* 8034F474 0034C3D4  7F A3 EB 78 */	mr r3, r29
/* 8034F478 0034C3D8  7F C4 F3 78 */	mr r4, r30
/* 8034F47C 0034C3DC  38 A0 00 03 */	li r5, 3
/* 8034F480 0034C3E0  4B FF FD E9 */	bl ProcessAxis__18CDolphinControllerFi8EJoyAxis
/* 8034F484 0034C3E4  7F A3 EB 78 */	mr r3, r29
/* 8034F488 0034C3E8  7F C4 F3 78 */	mr r4, r30
/* 8034F48C 0034C3EC  4B FF FD 0D */	bl ProcessButtons__18CDolphinControllerFi
lbl_8034F490:
/* 8034F490 0034C3F0  3B DE 00 01 */	addi r30, r30, 1
/* 8034F494 0034C3F4  3B FF 00 58 */	addi r31, r31, 0x58
/* 8034F498 0034C3F8  2C 1E 00 04 */	cmpwi r30, 4
/* 8034F49C 0034C3FC  41 80 FF 9C */	blt lbl_8034F438
/* 8034F4A0 0034C400  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8034F4A4 0034C404  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8034F4A8 0034C408  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8034F4AC 0034C40C  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8034F4B0 0034C410  7C 08 03 A6 */	mtlr r0
/* 8034F4B4 0034C414  38 21 00 20 */	addi r1, r1, 0x20
/* 8034F4B8 0034C418  4E 80 00 20 */	blr

.global ReadDevices__18CDolphinControllerFv
ReadDevices__18CDolphinControllerFv:
/* 8034F4BC 0034C41C  94 21 FF B0 */	stwu r1, -0x50(r1)
/* 8034F4C0 0034C420  7C 08 02 A6 */	mflr r0
/* 8034F4C4 0034C424  90 01 00 54 */	stw r0, 0x54(r1)
/* 8034F4C8 0034C428  BF 41 00 38 */	stmw r26, 0x38(r1)
/* 8034F4CC 0034C42C  7C 7A 1B 78 */	mr r26, r3
/* 8034F4D0 0034C430  38 61 00 08 */	addi r3, r1, 8
/* 8034F4D4 0034C434  48 03 72 A5 */	bl PADRead
/* 8034F4D8 0034C438  88 61 00 12 */	lbz r3, 0x12(r1)
/* 8034F4DC 0034C43C  7C 60 07 75 */	extsb. r0, r3
/* 8034F4E0 0034C440  40 82 00 20 */	bne lbl_8034F500
/* 8034F4E4 0034C444  38 61 00 08 */	addi r3, r1, 8
/* 8034F4E8 0034C448  48 03 65 01 */	bl PADClamp
/* 8034F4EC 0034C44C  38 7A 00 04 */	addi r3, r26, 4
/* 8034F4F0 0034C450  38 81 00 08 */	addi r4, r1, 8
/* 8034F4F4 0034C454  38 A0 00 30 */	li r5, 0x30
/* 8034F4F8 0034C458  4B CB 3F 99 */	bl memcpy
/* 8034F4FC 0034C45C  48 00 00 20 */	b lbl_8034F51C
lbl_8034F500:
/* 8034F500 0034C460  98 7A 00 0E */	stb r3, 0xe(r26)
/* 8034F504 0034C464  88 01 00 1E */	lbz r0, 0x1e(r1)
/* 8034F508 0034C468  98 1A 00 1A */	stb r0, 0x1a(r26)
/* 8034F50C 0034C46C  88 01 00 2A */	lbz r0, 0x2a(r1)
/* 8034F510 0034C470  98 1A 00 26 */	stb r0, 0x26(r26)
/* 8034F514 0034C474  88 01 00 36 */	lbz r0, 0x36(r1)
/* 8034F518 0034C478  98 1A 00 32 */	stb r0, 0x32(r26)
lbl_8034F51C:
/* 8034F51C 0034C47C  7F 5E D3 78 */	mr r30, r26
/* 8034F520 0034C480  7F 5D D3 78 */	mr r29, r26
/* 8034F524 0034C484  7F 5C D3 78 */	mr r28, r26
/* 8034F528 0034C488  3B 60 00 00 */	li r27, 0
/* 8034F52C 0034C48C  3F E0 80 00 */	lis r31, 0x8000
lbl_8034F530:
/* 8034F530 0034C490  88 1E 00 0E */	lbz r0, 0xe(r30)
/* 8034F534 0034C494  7F E4 DC 30 */	srw r4, r31, r27
/* 8034F538 0034C498  7C 03 07 74 */	extsb r3, r0
/* 8034F53C 0034C49C  2C 03 FF FE */	cmpwi r3, -2
/* 8034F540 0034C4A0  41 82 00 34 */	beq lbl_8034F574
/* 8034F544 0034C4A4  7C 00 07 75 */	extsb. r0, r0
/* 8034F548 0034C4A8  40 82 00 10 */	bne lbl_8034F558
/* 8034F54C 0034C4AC  38 00 00 01 */	li r0, 1
/* 8034F550 0034C4B0  98 1D 00 34 */	stb r0, 0x34(r29)
/* 8034F554 0034C4B4  48 00 00 20 */	b lbl_8034F574
lbl_8034F558:
/* 8034F558 0034C4B8  2C 03 FF FF */	cmpwi r3, -1
/* 8034F55C 0034C4BC  40 82 00 18 */	bne lbl_8034F574
/* 8034F560 0034C4C0  80 7A 01 C8 */	lwz r3, 0x1c8(r26)
/* 8034F564 0034C4C4  38 00 00 00 */	li r0, 0
/* 8034F568 0034C4C8  7C 63 23 78 */	or r3, r3, r4
/* 8034F56C 0034C4CC  90 7A 01 C8 */	stw r3, 0x1c8(r26)
/* 8034F570 0034C4D0  98 1D 00 34 */	stb r0, 0x34(r29)
lbl_8034F574:
/* 8034F574 0034C4D4  80 7C 01 B4 */	lwz r3, 0x1b4(r28)
/* 8034F578 0034C4D8  28 03 00 00 */	cmplwi r3, 0
/* 8034F57C 0034C4DC  41 82 00 10 */	beq lbl_8034F58C
/* 8034F580 0034C4E0  38 03 FF FF */	addi r0, r3, -1
/* 8034F584 0034C4E4  90 1C 01 B4 */	stw r0, 0x1b4(r28)
/* 8034F588 0034C4E8  48 00 00 78 */	b lbl_8034F600
lbl_8034F58C:
/* 8034F58C 0034C4EC  7F 63 DB 78 */	mr r3, r27
/* 8034F590 0034C4F0  48 07 09 59 */	bl SIProbe
/* 8034F594 0034C4F4  70 60 00 C8 */	andi. r0, r3, 0xc8
/* 8034F598 0034C4F8  41 82 00 1C */	beq lbl_8034F5B4
/* 8034F59C 0034C4FC  80 1C 01 B4 */	lwz r0, 0x1b4(r28)
/* 8034F5A0 0034C500  28 00 00 00 */	cmplwi r0, 0
/* 8034F5A4 0034C504  40 82 00 5C */	bne lbl_8034F600
/* 8034F5A8 0034C508  80 02 CC 60 */	lwz r0, lbl_805AE980@sda21(r2)
/* 8034F5AC 0034C50C  90 1C 01 A4 */	stw r0, 0x1a4(r28)
/* 8034F5B0 0034C510  48 00 00 50 */	b lbl_8034F600
lbl_8034F5B4:
/* 8034F5B4 0034C514  3C 03 74 F0 */	addis r0, r3, 0x74f0
/* 8034F5B8 0034C518  38 80 00 3C */	li r4, 0x3c
/* 8034F5BC 0034C51C  28 00 00 00 */	cmplwi r0, 0
/* 8034F5C0 0034C520  90 9C 01 B4 */	stw r4, 0x1b4(r28)
/* 8034F5C4 0034C524  40 82 00 10 */	bne lbl_8034F5D4
/* 8034F5C8 0034C528  80 02 CC 6C */	lwz r0, lbl_805AE98C@sda21(r2)
/* 8034F5CC 0034C52C  90 1C 01 A4 */	stw r0, 0x1a4(r28)
/* 8034F5D0 0034C530  48 00 00 30 */	b lbl_8034F600
lbl_8034F5D4:
/* 8034F5D4 0034C534  3C 03 FF FC */	addis r0, r3, 0xfffc
/* 8034F5D8 0034C538  28 00 00 00 */	cmplwi r0, 0
/* 8034F5DC 0034C53C  40 82 00 10 */	bne lbl_8034F5EC
/* 8034F5E0 0034C540  80 02 CC 68 */	lwz r0, lbl_805AE988@sda21(r2)
/* 8034F5E4 0034C544  90 1C 01 A4 */	stw r0, 0x1a4(r28)
/* 8034F5E8 0034C548  48 00 00 18 */	b lbl_8034F600
lbl_8034F5EC:
/* 8034F5EC 0034C54C  3C 03 F7 00 */	addis r0, r3, 0xf700
/* 8034F5F0 0034C550  28 00 00 00 */	cmplwi r0, 0
/* 8034F5F4 0034C554  40 82 00 0C */	bne lbl_8034F600
/* 8034F5F8 0034C558  80 02 CC 64 */	lwz r0, lbl_805AE984@sda21(r2)
/* 8034F5FC 0034C55C  90 1C 01 A4 */	stw r0, 0x1a4(r28)
lbl_8034F600:
/* 8034F600 0034C560  3B 7B 00 01 */	addi r27, r27, 1
/* 8034F604 0034C564  3B BD 00 58 */	addi r29, r29, 0x58
/* 8034F608 0034C568  2C 1B 00 04 */	cmpwi r27, 4
/* 8034F60C 0034C56C  3B 9C 00 04 */	addi r28, r28, 4
/* 8034F610 0034C570  3B DE 00 0C */	addi r30, r30, 0xc
/* 8034F614 0034C574  41 80 FF 1C */	blt lbl_8034F530
/* 8034F618 0034C578  80 7A 01 C8 */	lwz r3, 0x1c8(r26)
/* 8034F61C 0034C57C  28 03 00 00 */	cmplwi r3, 0
/* 8034F620 0034C580  41 82 00 18 */	beq lbl_8034F638
/* 8034F624 0034C584  48 03 6D 39 */	bl PADReset
/* 8034F628 0034C588  2C 03 00 00 */	cmpwi r3, 0
/* 8034F62C 0034C58C  41 82 00 0C */	beq lbl_8034F638
/* 8034F630 0034C590  38 00 00 00 */	li r0, 0
/* 8034F634 0034C594  90 1A 01 C8 */	stw r0, 0x1c8(r26)
lbl_8034F638:
/* 8034F638 0034C598  BB 41 00 38 */	lmw r26, 0x38(r1)
/* 8034F63C 0034C59C  80 01 00 54 */	lwz r0, 0x54(r1)
/* 8034F640 0034C5A0  7C 08 03 A6 */	mtlr r0
/* 8034F644 0034C5A4  38 21 00 50 */	addi r1, r1, 0x50
/* 8034F648 0034C5A8  4E 80 00 20 */	blr

.global Poll__18CDolphinControllerFv
Poll__18CDolphinControllerFv:
/* 8034F64C 0034C5AC  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F650 0034C5B0  7C 08 02 A6 */	mflr r0
/* 8034F654 0034C5B4  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F658 0034C5B8  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8034F65C 0034C5BC  7C 7F 1B 78 */	mr r31, r3
/* 8034F660 0034C5C0  4B FF FE 5D */	bl ReadDevices__18CDolphinControllerFv
/* 8034F664 0034C5C4  7F E3 FB 78 */	mr r3, r31
/* 8034F668 0034C5C8  4B FF FD AD */	bl ProcessInputData__18CDolphinControllerFv
/* 8034F66C 0034C5CC  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F670 0034C5D0  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8034F674 0034C5D4  7C 08 03 A6 */	mtlr r0
/* 8034F678 0034C5D8  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F67C 0034C5DC  4E 80 00 20 */	blr

.global Initialize__18CDolphinControllerFv
Initialize__18CDolphinControllerFv:
/* 8034F680 0034C5E0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F684 0034C5E4  7C 08 02 A6 */	mflr r0
/* 8034F688 0034C5E8  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F68C 0034C5EC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8034F690 0034C5F0  7C 7F 1B 78 */	mr r31, r3
/* 8034F694 0034C5F4  48 07 9F B9 */	bl GBAInit
/* 8034F698 0034C5F8  38 7F 00 04 */	addi r3, r31, 4
/* 8034F69C 0034C5FC  38 80 00 00 */	li r4, 0
/* 8034F6A0 0034C600  38 A0 00 30 */	li r5, 0x30
/* 8034F6A4 0034C604  4B CB 3D 05 */	bl memset
/* 8034F6A8 0034C608  38 A0 00 00 */	li r5, 0
/* 8034F6AC 0034C60C  38 80 00 02 */	li r4, 2
/* 8034F6B0 0034C610  98 BF 00 34 */	stb r5, 0x34(r31)
/* 8034F6B4 0034C614  38 7F 01 94 */	addi r3, r31, 0x194
/* 8034F6B8 0034C618  80 02 CC 60 */	lwz r0, lbl_805AE980@sda21(r2)
/* 8034F6BC 0034C61C  90 9F 01 94 */	stw r4, 0x194(r31)
/* 8034F6C0 0034C620  90 BF 01 B4 */	stw r5, 0x1b4(r31)
/* 8034F6C4 0034C624  90 1F 01 A4 */	stw r0, 0x1a4(r31)
/* 8034F6C8 0034C628  98 BF 00 8C */	stb r5, 0x8c(r31)
/* 8034F6CC 0034C62C  90 9F 01 98 */	stw r4, 0x198(r31)
/* 8034F6D0 0034C630  90 BF 01 B8 */	stw r5, 0x1b8(r31)
/* 8034F6D4 0034C634  90 1F 01 A8 */	stw r0, 0x1a8(r31)
/* 8034F6D8 0034C638  98 BF 00 E4 */	stb r5, 0xe4(r31)
/* 8034F6DC 0034C63C  90 9F 01 9C */	stw r4, 0x19c(r31)
/* 8034F6E0 0034C640  90 BF 01 BC */	stw r5, 0x1bc(r31)
/* 8034F6E4 0034C644  90 1F 01 AC */	stw r0, 0x1ac(r31)
/* 8034F6E8 0034C648  98 BF 01 3C */	stb r5, 0x13c(r31)
/* 8034F6EC 0034C64C  90 9F 01 A0 */	stw r4, 0x1a0(r31)
/* 8034F6F0 0034C650  90 BF 01 C0 */	stw r5, 0x1c0(r31)
/* 8034F6F4 0034C654  90 1F 01 B0 */	stw r0, 0x1b0(r31)
/* 8034F6F8 0034C658  48 03 74 2D */	bl PADControlAllMotors
/* 8034F6FC 0034C65C  7F E3 FB 78 */	mr r3, r31
/* 8034F700 0034C660  81 9F 00 00 */	lwz r12, 0(r31)
/* 8034F704 0034C664  81 8C 00 0C */	lwz r12, 0xc(r12)
/* 8034F708 0034C668  7D 89 03 A6 */	mtctr r12
/* 8034F70C 0034C66C  4E 80 04 21 */	bctrl
/* 8034F710 0034C670  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F714 0034C674  38 60 00 01 */	li r3, 1
/* 8034F718 0034C678  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8034F71C 0034C67C  7C 08 03 A6 */	mtlr r0
/* 8034F720 0034C680  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F724 0034C684  4E 80 00 20 */	blr

.global __dt__18CDolphinControllerFv
__dt__18CDolphinControllerFv:
/* 8034F728 0034C688  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F72C 0034C68C  7C 08 02 A6 */	mflr r0
/* 8034F730 0034C690  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F734 0034C694  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8034F738 0034C698  7C 9F 23 78 */	mr r31, r4
/* 8034F73C 0034C69C  93 C1 00 08 */	stw r30, 8(r1)
/* 8034F740 0034C6A0  7C 7E 1B 79 */	or. r30, r3, r3
/* 8034F744 0034C6A4  41 82 00 28 */	beq lbl_8034F76C
/* 8034F748 0034C6A8  3C A0 80 3F */	lis r5, lbl_803EF6B8@ha
/* 8034F74C 0034C6AC  38 80 00 00 */	li r4, 0
/* 8034F750 0034C6B0  38 05 F6 B8 */	addi r0, r5, lbl_803EF6B8@l
/* 8034F754 0034C6B4  90 1E 00 00 */	stw r0, 0(r30)
/* 8034F758 0034C6B8  4B FF F8 B1 */	bl __dt__11IControllerFv
/* 8034F75C 0034C6BC  7F E0 07 35 */	extsh. r0, r31
/* 8034F760 0034C6C0  40 81 00 0C */	ble lbl_8034F76C
/* 8034F764 0034C6C4  7F C3 F3 78 */	mr r3, r30
/* 8034F768 0034C6C8  4B FC 61 C9 */	bl Free__7CMemoryFPCv
lbl_8034F76C:
/* 8034F76C 0034C6CC  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F770 0034C6D0  7F C3 F3 78 */	mr r3, r30
/* 8034F774 0034C6D4  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8034F778 0034C6D8  83 C1 00 08 */	lwz r30, 8(r1)
/* 8034F77C 0034C6DC  7C 08 03 A6 */	mtlr r0
/* 8034F780 0034C6E0  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F784 0034C6E4  4E 80 00 20 */	blr

.global __ct__18CDolphinControllerFv
__ct__18CDolphinControllerFv:
/* 8034F788 0034C6E8  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F78C 0034C6EC  7C 08 02 A6 */	mflr r0
/* 8034F790 0034C6F0  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F794 0034C6F4  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8034F798 0034C6F8  7C 7F 1B 78 */	mr r31, r3
/* 8034F79C 0034C6FC  4B FF F8 B5 */	bl __ct__11IControllerFv
/* 8034F7A0 0034C700  3C 60 80 3F */	lis r3, lbl_803EF6B8@ha
/* 8034F7A4 0034C704  3C 80 80 35 */	lis r4, __ct__22CControllerGamepadDataFv@ha
/* 8034F7A8 0034C708  38 03 F6 B8 */	addi r0, r3, lbl_803EF6B8@l
/* 8034F7AC 0034C70C  38 A0 00 00 */	li r5, 0
/* 8034F7B0 0034C710  90 1F 00 00 */	stw r0, 0(r31)
/* 8034F7B4 0034C714  38 7F 00 34 */	addi r3, r31, 0x34
/* 8034F7B8 0034C718  38 84 F8 2C */	addi r4, r4, __ct__22CControllerGamepadDataFv@l
/* 8034F7BC 0034C71C  38 C0 00 58 */	li r6, 0x58
/* 8034F7C0 0034C720  38 E0 00 04 */	li r7, 4
/* 8034F7C4 0034C724  48 03 9F 89 */	bl __construct_array
/* 8034F7C8 0034C728  3C 00 F0 00 */	lis r0, 0xf000
/* 8034F7CC 0034C72C  38 60 00 00 */	li r3, 0
/* 8034F7D0 0034C730  90 1F 01 C4 */	stw r0, 0x1c4(r31)
/* 8034F7D4 0034C734  90 7F 01 C8 */	stw r3, 0x1c8(r31)
/* 8034F7D8 0034C738  90 7F 01 CC */	stw r3, 0x1cc(r31)
/* 8034F7DC 0034C73C  88 0D AA 09 */	lbz r0, lbl_805A95C9@sda21(r13)
/* 8034F7E0 0034C740  7C 00 07 75 */	extsb. r0, r0
/* 8034F7E4 0034C744  40 82 00 10 */	bne lbl_8034F7F4
/* 8034F7E8 0034C748  38 00 00 01 */	li r0, 1
/* 8034F7EC 0034C74C  98 6D AA 08 */	stb r3, lbl_805A95C8@sda21(r13)
/* 8034F7F0 0034C750  98 0D AA 09 */	stb r0, lbl_805A95C9@sda21(r13)
lbl_8034F7F4:
/* 8034F7F4 0034C754  88 0D AA 08 */	lbz r0, lbl_805A95C8@sda21(r13)
/* 8034F7F8 0034C758  28 00 00 00 */	cmplwi r0, 0
/* 8034F7FC 0034C75C  40 82 00 18 */	bne lbl_8034F814
/* 8034F800 0034C760  38 60 00 05 */	li r3, 5
/* 8034F804 0034C764  48 03 74 7D */	bl PADSetSpec
/* 8034F808 0034C768  48 03 6D 59 */	bl PADInit
/* 8034F80C 0034C76C  38 00 00 01 */	li r0, 1
/* 8034F810 0034C770  98 0D AA 08 */	stb r0, lbl_805A95C8@sda21(r13)
lbl_8034F814:
/* 8034F814 0034C774  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F818 0034C778  7F E3 FB 78 */	mr r3, r31
/* 8034F81C 0034C77C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8034F820 0034C780  7C 08 03 A6 */	mtlr r0
/* 8034F824 0034C784  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F828 0034C788  4E 80 00 20 */	blr

.global __ct__22CControllerGamepadDataFv
__ct__22CControllerGamepadDataFv:
/* 8034F82C 0034C78C  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8034F830 0034C790  7C 08 02 A6 */	mflr r0
/* 8034F834 0034C794  3C 80 80 35 */	lis r4, __ct__15CControllerAxisFv@ha
/* 8034F838 0034C798  38 A0 00 00 */	li r5, 0
/* 8034F83C 0034C79C  90 01 00 14 */	stw r0, 0x14(r1)
/* 8034F840 0034C7A0  38 84 F8 C0 */	addi r4, r4, __ct__15CControllerAxisFv@l
/* 8034F844 0034C7A4  38 C0 00 08 */	li r6, 8
/* 8034F848 0034C7A8  38 E0 00 04 */	li r7, 4
/* 8034F84C 0034C7AC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8034F850 0034C7B0  7C 7F 1B 78 */	mr r31, r3
/* 8034F854 0034C7B4  38 7F 00 04 */	addi r3, r31, 4
/* 8034F858 0034C7B8  48 03 9E F5 */	bl __construct_array
/* 8034F85C 0034C7BC  3C 80 80 35 */	lis r4, __ct__15CControllerAxisFv@ha
/* 8034F860 0034C7C0  38 7F 00 24 */	addi r3, r31, 0x24
/* 8034F864 0034C7C4  38 84 F8 C0 */	addi r4, r4, __ct__15CControllerAxisFv@l
/* 8034F868 0034C7C8  38 A0 00 00 */	li r5, 0
/* 8034F86C 0034C7CC  38 C0 00 08 */	li r6, 8
/* 8034F870 0034C7D0  38 E0 00 02 */	li r7, 2
/* 8034F874 0034C7D4  48 03 9E D9 */	bl __construct_array
/* 8034F878 0034C7D8  3C 80 80 35 */	lis r4, __ct__17CControllerButtonFv@ha
/* 8034F87C 0034C7DC  38 7F 00 34 */	addi r3, r31, 0x34
/* 8034F880 0034C7E0  38 84 F8 AC */	addi r4, r4, __ct__17CControllerButtonFv@l
/* 8034F884 0034C7E4  38 A0 00 00 */	li r5, 0
/* 8034F888 0034C7E8  38 C0 00 03 */	li r6, 3
/* 8034F88C 0034C7EC  38 E0 00 0C */	li r7, 0xc
/* 8034F890 0034C7F0  48 03 9E BD */	bl __construct_array
/* 8034F894 0034C7F4  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8034F898 0034C7F8  7F E3 FB 78 */	mr r3, r31
/* 8034F89C 0034C7FC  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8034F8A0 0034C800  7C 08 03 A6 */	mtlr r0
/* 8034F8A4 0034C804  38 21 00 10 */	addi r1, r1, 0x10
/* 8034F8A8 0034C808  4E 80 00 20 */	blr

.global __ct__17CControllerButtonFv
__ct__17CControllerButtonFv:
/* 8034F8AC 0034C80C  38 00 00 00 */	li r0, 0
/* 8034F8B0 0034C810  98 03 00 00 */	stb r0, 0(r3)
/* 8034F8B4 0034C814  98 03 00 01 */	stb r0, 1(r3)
/* 8034F8B8 0034C818  98 03 00 02 */	stb r0, 2(r3)
/* 8034F8BC 0034C81C  4E 80 00 20 */	blr

.global __ct__15CControllerAxisFv
__ct__15CControllerAxisFv:
/* 8034F8C0 0034C820  C0 02 CC 78 */	lfs f0, lbl_805AE998@sda21(r2)
/* 8034F8C4 0034C824  D0 03 00 00 */	stfs f0, 0(r3)
/* 8034F8C8 0034C828  D0 03 00 04 */	stfs f0, 4(r3)
/* 8034F8CC 0034C82C  4E 80 00 20 */	blr

.section .sdata2, "a"
.balign 8

.global lbl_805AE980
lbl_805AE980:
	# ROM: 0x3FB220
	.4byte 0x554E4B4E

.global lbl_805AE984
lbl_805AE984:
	# ROM: 0x3FB224
	.4byte 0x53544E44

.global lbl_805AE988
lbl_805AE988:
	# ROM: 0x3FB228
	.4byte 0x4742415F

.global lbl_805AE98C
lbl_805AE98C:
	# ROM: 0x3FB22C
	.4byte 0x57415645

.global lbl_805AE990
lbl_805AE990:
	# ROM: 0x3FB230
	.4byte 0x42900000

.global lbl_805AE994
lbl_805AE994:
	# ROM: 0x3FB234
	.4byte 0x426C0000

.global lbl_805AE998
lbl_805AE998:
	# ROM: 0x3FB238
	.4byte 0

.global lbl_805AE99C
lbl_805AE99C:
	# ROM: 0x3FB23C
	.4byte 0x3BDA740E

.global lbl_805AE9A0
lbl_805AE9A0:
	# ROM: 0x3FB240
	.4byte 0x43300000
	.4byte 0

.global lbl_805AE9A8
lbl_805AE9A8:
	# ROM: 0x3FB248
	.float 1.0
	.4byte 0

.global lbl_805AE9B0
lbl_805AE9B0:
	# ROM: 0x3FB250
	.double 4.503601774854144E15

