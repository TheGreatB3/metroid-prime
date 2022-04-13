.include "macros.inc"

.section .text, "ax"  # 0x80003640 - 0x803CB1C0

.global AcceptScriptMsg__18CScriptDistanceFogF20EScriptObjectMessage9TUniqueIdR13CStateManager
AcceptScriptMsg__18CScriptDistanceFogF20EScriptObjectMessage9TUniqueIdR13CStateManager:
/* 8014CFFC 00149F5C  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 8014D000 00149F60  7C 08 02 A6 */	mflr r0
/* 8014D004 00149F64  90 01 00 44 */	stw r0, 0x44(r1)
/* 8014D008 00149F68  93 E1 00 3C */	stw r31, 0x3c(r1)
/* 8014D00C 00149F6C  7C DF 33 78 */	mr r31, r6
/* 8014D010 00149F70  93 C1 00 38 */	stw r30, 0x38(r1)
/* 8014D014 00149F74  7C 7E 1B 78 */	mr r30, r3
/* 8014D018 00149F78  93 A1 00 34 */	stw r29, 0x34(r1)
/* 8014D01C 00149F7C  7C 9D 23 78 */	mr r29, r4
/* 8014D020 00149F80  A0 05 00 00 */	lhz r0, 0(r5)
/* 8014D024 00149F84  38 A1 00 08 */	addi r5, r1, 8
/* 8014D028 00149F88  B0 01 00 08 */	sth r0, 8(r1)
/* 8014D02C 00149F8C  4B F0 41 31 */	bl AcceptScriptMsg__7CEntityF20EScriptObjectMessage9TUniqueIdR13CStateManager
/* 8014D030 00149F90  80 DE 00 04 */	lwz r6, 4(r30)
/* 8014D034 00149F94  80 0D A3 90 */	lwz r0, lbl_805A8F50@sda21(r13)
/* 8014D038 00149F98  7C 06 00 00 */	cmpw r6, r0
/* 8014D03C 00149F9C  41 82 01 68 */	beq lbl_8014D1A4
/* 8014D040 00149FA0  88 1E 00 30 */	lbz r0, 0x30(r30)
/* 8014D044 00149FA4  54 00 CF FF */	rlwinm. r0, r0, 0x19, 0x1f, 0x1f
/* 8014D048 00149FA8  41 82 01 5C */	beq lbl_8014D1A4
/* 8014D04C 00149FAC  2C 1D 00 23 */	cmpwi r29, 0x23
/* 8014D050 00149FB0  41 82 00 14 */	beq lbl_8014D064
/* 8014D054 00149FB4  40 80 01 50 */	bge lbl_8014D1A4
/* 8014D058 00149FB8  2C 1D 00 13 */	cmpwi r29, 0x13
/* 8014D05C 00149FBC  41 82 00 5C */	beq lbl_8014D0B8
/* 8014D060 00149FC0  48 00 01 44 */	b lbl_8014D1A4
lbl_8014D064:
/* 8014D064 00149FC4  88 1E 00 60 */	lbz r0, 0x60(r30)
/* 8014D068 00149FC8  28 00 00 00 */	cmplwi r0, 0
/* 8014D06C 00149FCC  41 82 01 38 */	beq lbl_8014D1A4
/* 8014D070 00149FD0  80 BF 08 50 */	lwz r5, 0x850(r31)
/* 8014D074 00149FD4  54 C3 18 38 */	slwi r3, r6, 3
/* 8014D078 00149FD8  80 9E 00 34 */	lwz r4, 0x34(r30)
/* 8014D07C 00149FDC  38 03 00 04 */	addi r0, r3, 4
/* 8014D080 00149FE0  80 65 00 20 */	lwz r3, 0x20(r5)
/* 8014D084 00149FE4  2C 04 00 00 */	cmpwi r4, 0
/* 8014D088 00149FE8  90 C1 00 28 */	stw r6, 0x28(r1)
/* 8014D08C 00149FEC  7C 63 00 2E */	lwzx r3, r3, r0
/* 8014D090 00149FF0  90 C1 00 18 */	stw r6, 0x18(r1)
/* 8014D094 00149FF4  80 63 01 2C */	lwz r3, 0x12c(r3)
/* 8014D098 00149FF8  80 63 10 C4 */	lwz r3, 0x10c4(r3)
/* 8014D09C 00149FFC  40 82 00 0C */	bne lbl_8014D0A8
/* 8014D0A0 0014A000  4B F1 20 F9 */	bl DisableFog__Q29CGameArea8CAreaFogFv
/* 8014D0A4 0014A004  48 00 01 00 */	b lbl_8014D1A4
lbl_8014D0A8:
/* 8014D0A8 0014A008  38 BE 00 38 */	addi r5, r30, 0x38
/* 8014D0AC 0014A00C  38 DE 00 3C */	addi r6, r30, 0x3c
/* 8014D0B0 0014A010  4B F1 20 71 */	bl SetFogExplicit__Q29CGameArea8CAreaFogF11ERglFogModeRC6CColorRC9CVector2f
/* 8014D0B4 0014A014  48 00 00 F0 */	b lbl_8014D1A4
lbl_8014D0B8:
/* 8014D0B8 0014A018  88 1E 00 61 */	lbz r0, 0x61(r30)
/* 8014D0BC 0014A01C  28 00 00 00 */	cmplwi r0, 0
/* 8014D0C0 0014A020  41 82 00 5C */	beq lbl_8014D11C
/* 8014D0C4 0014A024  80 BF 08 50 */	lwz r5, 0x850(r31)
/* 8014D0C8 0014A028  54 C3 18 38 */	slwi r3, r6, 3
/* 8014D0CC 0014A02C  80 9E 00 34 */	lwz r4, 0x34(r30)
/* 8014D0D0 0014A030  38 03 00 04 */	addi r0, r3, 4
/* 8014D0D4 0014A034  80 65 00 20 */	lwz r3, 0x20(r5)
/* 8014D0D8 0014A038  2C 04 00 00 */	cmpwi r4, 0
/* 8014D0DC 0014A03C  90 C1 00 24 */	stw r6, 0x24(r1)
/* 8014D0E0 0014A040  7C 63 00 2E */	lwzx r3, r3, r0
/* 8014D0E4 0014A044  90 C1 00 14 */	stw r6, 0x14(r1)
/* 8014D0E8 0014A048  80 63 01 2C */	lwz r3, 0x12c(r3)
/* 8014D0EC 0014A04C  80 63 10 C4 */	lwz r3, 0x10c4(r3)
/* 8014D0F0 0014A050  41 82 00 1C */	beq lbl_8014D10C
/* 8014D0F4 0014A054  C0 3E 00 44 */	lfs f1, 0x44(r30)
/* 8014D0F8 0014A058  38 BE 00 38 */	addi r5, r30, 0x38
/* 8014D0FC 0014A05C  38 DE 00 3C */	addi r6, r30, 0x3c
/* 8014D100 0014A060  38 FE 00 48 */	addi r7, r30, 0x48
/* 8014D104 0014A064  4B F1 1E F5 */	bl FadeFog__Q29CGameArea8CAreaFogF11ERglFogModeRC6CColorRC9CVector2ffRC9CVector2f
/* 8014D108 0014A068  48 00 00 14 */	b lbl_8014D11C
lbl_8014D10C:
/* 8014D10C 0014A06C  C0 3E 00 48 */	lfs f1, 0x48(r30)
/* 8014D110 0014A070  38 9E 00 38 */	addi r4, r30, 0x38
/* 8014D114 0014A074  C0 5E 00 44 */	lfs f2, 0x44(r30)
/* 8014D118 0014A078  4B F1 1E 2D */	bl RollFogOut__Q29CGameArea8CAreaFogFffRC6CColor
lbl_8014D11C:
/* 8014D11C 0014A07C  C0 3E 00 54 */	lfs f1, 0x54(r30)
/* 8014D120 0014A080  C0 42 9D 20 */	lfs f2, lbl_805ABA40@sda21(r2)
/* 8014D124 0014A084  C0 02 9D 24 */	lfs f0, lbl_805ABA44@sda21(r2)
/* 8014D128 0014A088  EC 41 10 28 */	fsubs f2, f1, f2
/* 8014D12C 0014A08C  FC 40 12 10 */	fabs f2, f2
/* 8014D130 0014A090  FC 02 00 40 */	fcmpo cr0, f2, f0
/* 8014D134 0014A094  41 80 00 2C */	blt lbl_8014D160
/* 8014D138 0014A098  80 BE 00 04 */	lwz r5, 4(r30)
/* 8014D13C 0014A09C  80 9F 08 50 */	lwz r4, 0x850(r31)
/* 8014D140 0014A0A0  54 A3 18 38 */	slwi r3, r5, 3
/* 8014D144 0014A0A4  90 A1 00 20 */	stw r5, 0x20(r1)
/* 8014D148 0014A0A8  80 84 00 20 */	lwz r4, 0x20(r4)
/* 8014D14C 0014A0AC  38 03 00 04 */	addi r0, r3, 4
/* 8014D150 0014A0B0  90 A1 00 10 */	stw r5, 0x10(r1)
/* 8014D154 0014A0B4  7C 64 00 2E */	lwzx r3, r4, r0
/* 8014D158 0014A0B8  C0 5E 00 50 */	lfs f2, 0x50(r30)
/* 8014D15C 0014A0BC  4B F1 10 A1 */	bl sub_8005e1fc
lbl_8014D160:
/* 8014D160 0014A0C0  C0 3E 00 5C */	lfs f1, 0x5c(r30)
/* 8014D164 0014A0C4  C0 42 9D 20 */	lfs f2, lbl_805ABA40@sda21(r2)
/* 8014D168 0014A0C8  C0 02 9D 24 */	lfs f0, lbl_805ABA44@sda21(r2)
/* 8014D16C 0014A0CC  EC 41 10 28 */	fsubs f2, f1, f2
/* 8014D170 0014A0D0  FC 40 12 10 */	fabs f2, f2
/* 8014D174 0014A0D4  FC 02 00 40 */	fcmpo cr0, f2, f0
/* 8014D178 0014A0D8  41 80 00 2C */	blt lbl_8014D1A4
/* 8014D17C 0014A0DC  80 BE 00 04 */	lwz r5, 4(r30)
/* 8014D180 0014A0E0  80 9F 08 50 */	lwz r4, 0x850(r31)
/* 8014D184 0014A0E4  54 A3 18 38 */	slwi r3, r5, 3
/* 8014D188 0014A0E8  90 A1 00 1C */	stw r5, 0x1c(r1)
/* 8014D18C 0014A0EC  80 84 00 20 */	lwz r4, 0x20(r4)
/* 8014D190 0014A0F0  38 03 00 04 */	addi r0, r3, 4
/* 8014D194 0014A0F4  90 A1 00 0C */	stw r5, 0xc(r1)
/* 8014D198 0014A0F8  7C 64 00 2E */	lwzx r3, r4, r0
/* 8014D19C 0014A0FC  C0 5E 00 58 */	lfs f2, 0x58(r30)
/* 8014D1A0 0014A100  4B F1 10 49 */	bl sub_8005e1e8
lbl_8014D1A4:
/* 8014D1A4 0014A104  80 01 00 44 */	lwz r0, 0x44(r1)
/* 8014D1A8 0014A108  83 E1 00 3C */	lwz r31, 0x3c(r1)
/* 8014D1AC 0014A10C  83 C1 00 38 */	lwz r30, 0x38(r1)
/* 8014D1B0 0014A110  83 A1 00 34 */	lwz r29, 0x34(r1)
/* 8014D1B4 0014A114  7C 08 03 A6 */	mtlr r0
/* 8014D1B8 0014A118  38 21 00 40 */	addi r1, r1, 0x40
/* 8014D1BC 0014A11C  4E 80 00 20 */	blr 

.global Accept__18CScriptDistanceFogFR8IVisitor
Accept__18CScriptDistanceFogFR8IVisitor:
/* 8014D1C0 0014A120  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8014D1C4 0014A124  7C 08 02 A6 */	mflr r0
/* 8014D1C8 0014A128  90 01 00 14 */	stw r0, 0x14(r1)
/* 8014D1CC 0014A12C  7C 60 1B 78 */	mr r0, r3
/* 8014D1D0 0014A130  7C 83 23 78 */	mr r3, r4
/* 8014D1D4 0014A134  81 84 00 00 */	lwz r12, 0(r4)
/* 8014D1D8 0014A138  7C 04 03 78 */	mr r4, r0
/* 8014D1DC 0014A13C  81 8C 00 7C */	lwz r12, 0x7c(r12)
/* 8014D1E0 0014A140  7D 89 03 A6 */	mtctr r12
/* 8014D1E4 0014A144  4E 80 04 21 */	bctrl 
/* 8014D1E8 0014A148  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8014D1EC 0014A14C  7C 08 03 A6 */	mtlr r0
/* 8014D1F0 0014A150  38 21 00 10 */	addi r1, r1, 0x10
/* 8014D1F4 0014A154  4E 80 00 20 */	blr 

.global __dt__18CScriptDistanceFogFv
__dt__18CScriptDistanceFogFv:
/* 8014D1F8 0014A158  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8014D1FC 0014A15C  7C 08 02 A6 */	mflr r0
/* 8014D200 0014A160  90 01 00 14 */	stw r0, 0x14(r1)
/* 8014D204 0014A164  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8014D208 0014A168  7C 9F 23 78 */	mr r31, r4
/* 8014D20C 0014A16C  93 C1 00 08 */	stw r30, 8(r1)
/* 8014D210 0014A170  7C 7E 1B 79 */	or. r30, r3, r3
/* 8014D214 0014A174  41 82 00 28 */	beq lbl_8014D23C
/* 8014D218 0014A178  3C A0 80 3E */	lis r5, lbl_803E27B0@ha
/* 8014D21C 0014A17C  38 80 00 00 */	li r4, 0
/* 8014D220 0014A180  38 05 27 B0 */	addi r0, r5, lbl_803E27B0@l
/* 8014D224 0014A184  90 1E 00 00 */	stw r0, 0(r30)
/* 8014D228 0014A188  4B F0 40 4D */	bl __dt__7CEntityFv
/* 8014D22C 0014A18C  7F E0 07 35 */	extsh. r0, r31
/* 8014D230 0014A190  40 81 00 0C */	ble lbl_8014D23C
/* 8014D234 0014A194  7F C3 F3 78 */	mr r3, r30
/* 8014D238 0014A198  48 1C 86 F9 */	bl Free__7CMemoryFPCv
lbl_8014D23C:
/* 8014D23C 0014A19C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8014D240 0014A1A0  7F C3 F3 78 */	mr r3, r30
/* 8014D244 0014A1A4  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8014D248 0014A1A8  83 C1 00 08 */	lwz r30, 8(r1)
/* 8014D24C 0014A1AC  7C 08 03 A6 */	mtlr r0
/* 8014D250 0014A1B0  38 21 00 10 */	addi r1, r1, 0x10
/* 8014D254 0014A1B4  4E 80 00 20 */	blr 

.global "__ct__18CScriptDistanceFogF9TUniqueIdRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC11CEntityInfoRC11ERglFogModeRC6CColorRC9CVector2ff9CVector2fbbffff"
"__ct__18CScriptDistanceFogF9TUniqueIdRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>RC11CEntityInfoRC11ERglFogModeRC6CColorRC9CVector2ff9CVector2fbbffff":
/* 8014D258 0014A1B8  94 21 FF 70 */	stwu r1, -0x90(r1)
/* 8014D25C 0014A1BC  7C 08 02 A6 */	mflr r0
/* 8014D260 0014A1C0  90 01 00 94 */	stw r0, 0x94(r1)
/* 8014D264 0014A1C4  DB E1 00 80 */	stfd f31, 0x80(r1)
/* 8014D268 0014A1C8  F3 E1 00 88 */	psq_st f31, 136(r1), 0, qr0
/* 8014D26C 0014A1CC  DB C1 00 70 */	stfd f30, 0x70(r1)
/* 8014D270 0014A1D0  F3 C1 00 78 */	psq_st f30, 120(r1), 0, qr0
/* 8014D274 0014A1D4  DB A1 00 60 */	stfd f29, 0x60(r1)
/* 8014D278 0014A1D8  F3 A1 00 68 */	psq_st f29, 104(r1), 0, qr0
/* 8014D27C 0014A1DC  DB 81 00 50 */	stfd f28, 0x50(r1)
/* 8014D280 0014A1E0  F3 81 00 58 */	psq_st f28, 88(r1), 0, qr0
/* 8014D284 0014A1E4  DB 61 00 40 */	stfd f27, 0x40(r1)
/* 8014D288 0014A1E8  F3 61 00 48 */	psq_st f27, 72(r1), 0, qr0
/* 8014D28C 0014A1EC  BF 21 00 24 */	stmw r25, 0x24(r1)
/* 8014D290 0014A1F0  A0 04 00 00 */	lhz r0, 0(r4)
/* 8014D294 0014A1F4  7C AB 2B 78 */	mr r11, r5
/* 8014D298 0014A1F8  FF 60 08 90 */	fmr f27, f1
/* 8014D29C 0014A1FC  88 81 00 9F */	lbz r4, 0x9f(r1)
/* 8014D2A0 0014A200  FF 80 10 90 */	fmr f28, f2
/* 8014D2A4 0014A204  B0 01 00 08 */	sth r0, 8(r1)
/* 8014D2A8 0014A208  7C FA 3B 78 */	mr r26, r7
/* 8014D2AC 0014A20C  7C C5 33 78 */	mr r5, r6
/* 8014D2B0 0014A210  7C 86 23 78 */	mr r6, r4
/* 8014D2B4 0014A214  FF A0 18 90 */	fmr f29, f3
/* 8014D2B8 0014A218  FF C0 20 90 */	fmr f30, f4
/* 8014D2BC 0014A21C  8B C1 00 9B */	lbz r30, 0x9b(r1)
/* 8014D2C0 0014A220  FF E0 28 90 */	fmr f31, f5
/* 8014D2C4 0014A224  7C 79 1B 78 */	mr r25, r3
/* 8014D2C8 0014A228  7D 1B 43 78 */	mr r27, r8
/* 8014D2CC 0014A22C  7D 3C 4B 78 */	mr r28, r9
/* 8014D2D0 0014A230  7D 5D 53 78 */	mr r29, r10
/* 8014D2D4 0014A234  7D 67 5B 78 */	mr r7, r11
/* 8014D2D8 0014A238  38 81 00 08 */	addi r4, r1, 8
/* 8014D2DC 0014A23C  4B F0 40 49 */	bl "__ct__7CEntityF9TUniqueIdRC11CEntityInfobRCQ24rstl66basic_string<c,Q24rstl14char_traits<c>,Q24rstl17rmemory_allocator>"
/* 8014D2E0 0014A240  3C 60 80 3E */	lis r3, lbl_803E27B0@ha
/* 8014D2E4 0014A244  C0 22 9D 20 */	lfs f1, lbl_805ABA40@sda21(r2)
/* 8014D2E8 0014A248  38 03 27 B0 */	addi r0, r3, lbl_803E27B0@l
/* 8014D2EC 0014A24C  38 61 00 0C */	addi r3, r1, 0xc
/* 8014D2F0 0014A250  90 19 00 00 */	stw r0, 0(r25)
/* 8014D2F4 0014A254  FC 40 08 90 */	fmr f2, f1
/* 8014D2F8 0014A258  3B E0 00 01 */	li r31, 1
/* 8014D2FC 0014A25C  93 59 00 34 */	stw r26, 0x34(r25)
/* 8014D300 0014A260  80 1B 00 00 */	lwz r0, 0(r27)
/* 8014D304 0014A264  90 19 00 38 */	stw r0, 0x38(r25)
/* 8014D308 0014A268  C0 1C 00 00 */	lfs f0, 0(r28)
/* 8014D30C 0014A26C  D0 19 00 3C */	stfs f0, 0x3c(r25)
/* 8014D310 0014A270  C0 1C 00 04 */	lfs f0, 4(r28)
/* 8014D314 0014A274  D0 19 00 40 */	stfs f0, 0x40(r25)
/* 8014D318 0014A278  D3 79 00 44 */	stfs f27, 0x44(r25)
/* 8014D31C 0014A27C  C0 1D 00 00 */	lfs f0, 0(r29)
/* 8014D320 0014A280  D0 19 00 48 */	stfs f0, 0x48(r25)
/* 8014D324 0014A284  C0 1D 00 04 */	lfs f0, 4(r29)
/* 8014D328 0014A288  D0 19 00 4C */	stfs f0, 0x4c(r25)
/* 8014D32C 0014A28C  D3 99 00 50 */	stfs f28, 0x50(r25)
/* 8014D330 0014A290  D3 B9 00 54 */	stfs f29, 0x54(r25)
/* 8014D334 0014A294  D3 D9 00 58 */	stfs f30, 0x58(r25)
/* 8014D338 0014A298  D3 F9 00 5C */	stfs f31, 0x5c(r25)
/* 8014D33C 0014A29C  9B D9 00 60 */	stb r30, 0x60(r25)
/* 8014D340 0014A2A0  48 1C 6E C1 */	bl __ct__9CVector2fFff
/* 8014D344 0014A2A4  C0 22 9D 28 */	lfs f1, lbl_805ABA48@sda21(r2)
/* 8014D348 0014A2A8  7C 64 1B 78 */	mr r4, r3
/* 8014D34C 0014A2AC  7F A3 EB 78 */	mr r3, r29
/* 8014D350 0014A2B0  48 1C 2B D5 */	bl sub_8030ff24
/* 8014D354 0014A2B4  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 8014D358 0014A2B8  41 82 00 28 */	beq lbl_8014D380
/* 8014D35C 0014A2BC  C0 22 9D 20 */	lfs f1, lbl_805ABA40@sda21(r2)
/* 8014D360 0014A2C0  C8 02 9D 30 */	lfd f0, lbl_805ABA50@sda21(r2)
/* 8014D364 0014A2C4  EC 3B 08 28 */	fsubs f1, f27, f1
/* 8014D368 0014A2C8  FC 20 0A 10 */	fabs f1, f1
/* 8014D36C 0014A2CC  FC 01 00 40 */	fcmpo cr0, f1, f0
/* 8014D370 0014A2D0  7C 00 00 26 */	mfcr r0
/* 8014D374 0014A2D4  54 00 0F FF */	rlwinm. r0, r0, 1, 0x1f, 0x1f
/* 8014D378 0014A2D8  41 82 00 08 */	beq lbl_8014D380
/* 8014D37C 0014A2DC  3B E0 00 00 */	li r31, 0
lbl_8014D380:
/* 8014D380 0014A2E0  9B F9 00 61 */	stb r31, 0x61(r25)
/* 8014D384 0014A2E4  7F 23 CB 78 */	mr r3, r25
/* 8014D388 0014A2E8  E3 E1 00 88 */	psq_l f31, 136(r1), 0, qr0
/* 8014D38C 0014A2EC  CB E1 00 80 */	lfd f31, 0x80(r1)
/* 8014D390 0014A2F0  E3 C1 00 78 */	psq_l f30, 120(r1), 0, qr0
/* 8014D394 0014A2F4  CB C1 00 70 */	lfd f30, 0x70(r1)
/* 8014D398 0014A2F8  E3 A1 00 68 */	psq_l f29, 104(r1), 0, qr0
/* 8014D39C 0014A2FC  CB A1 00 60 */	lfd f29, 0x60(r1)
/* 8014D3A0 0014A300  E3 81 00 58 */	psq_l f28, 88(r1), 0, qr0
/* 8014D3A4 0014A304  CB 81 00 50 */	lfd f28, 0x50(r1)
/* 8014D3A8 0014A308  E3 61 00 48 */	psq_l f27, 72(r1), 0, qr0
/* 8014D3AC 0014A30C  CB 61 00 40 */	lfd f27, 0x40(r1)
/* 8014D3B0 0014A310  BB 21 00 24 */	lmw r25, 0x24(r1)
/* 8014D3B4 0014A314  80 01 00 94 */	lwz r0, 0x94(r1)
/* 8014D3B8 0014A318  7C 08 03 A6 */	mtlr r0
/* 8014D3BC 0014A31C  38 21 00 90 */	addi r1, r1, 0x90
/* 8014D3C0 0014A320  4E 80 00 20 */	blr
