.include "macros.inc"

.section .sdata2
.global lbl_805AF2F0
lbl_805AF2F0:
	.incbin "baserom.dol", 0x3FBB90, 0x4
.global lbl_805AF2F4
lbl_805AF2F4:
	.incbin "baserom.dol", 0x3FBB94, 0x4
.global lbl_805AF2F8
lbl_805AF2F8:
	.incbin "baserom.dol", 0x3FBB98, 0x8
.global lbl_805AF300
lbl_805AF300:
	.incbin "baserom.dol", 0x3FBBA0, 0x8
.global lbl_805AF308
lbl_805AF308:
	.incbin "baserom.dol", 0x3FBBA8, 0x8
.global lbl_805AF310
lbl_805AF310:
	.incbin "baserom.dol", 0x3FBBB0, 0x8
.global lbl_805AF318
lbl_805AF318:
	.incbin "baserom.dol", 0x3FBBB8, 0x4
.global lbl_805AF31C
lbl_805AF31C:
	.incbin "baserom.dol", 0x3FBBBC, 0x4
.global lbl_805AF320
lbl_805AF320:
	.incbin "baserom.dol", 0x3FBBC0, 0x8
	
.section .data, "wa"

.global lbl_803F3FC0
lbl_803F3FC0:
	# ROM: 0x3F0FC0
	.4byte 0
	.4byte 0x380000FD
	.4byte 0x3920015F
	.4byte 0x39D001AD
	.4byte 0x3A380175
	.4byte 0x3A9400FE
	.4byte 0x3AD801B4
	.4byte 0x3B12011F
	.4byte 0x3B40018D
	.4byte 0x3B7401F0
	.4byte 0x3B970123
	.4byte 0x3BB70174
	.4byte 0x3BDA01BF
	.4byte 0x3C008104
	.4byte 0x3C158126
	.4byte 0x3C2C017B
	.4byte 0x3C4401B8
	.4byte 0x3C5E01A8
	.4byte 0x3C7901DF
	.4byte 0x3C8B4111
	.4byte 0x3C9A8120
	.4byte 0x3CAAC13F
	.4byte 0x3CBBC177
	.4byte 0x3CCDC18A
	.4byte 0x3CE081B5
	.4byte 0x3CF441F1
	.4byte 0x3D046108
	.4byte 0x3D0F2120
	.4byte 0x3D1A412A
	.4byte 0x3D25C140
	.4byte 0x3D31E159
	.4byte 0x3D3E617F
	.4byte 0x3D4B4197
	.4byte 0x3D58A1B7
	.4byte 0x3D6681C5
	.4byte 0x3D74E1F4
	.4byte 0x3D81D0FE
	.4byte 0x3D897113
	.4byte 0x3D91511E
	.4byte 0x3D99712D
	.4byte 0x3DA1C143
	.4byte 0x3DAA514F
	.4byte 0x3DB32160
	.4byte 0x3DBC417F
	.4byte 0x3DC5918A
	.4byte 0x3DCF2171
	.4byte 0x3DD8F1D4
	.4byte 0x3DE301A8
	.4byte 0x3DED51F8
	.4byte 0x3DF7E1B9
	.4byte 0x3E01611C
	.4byte 0x3E06E8F3
	.4byte 0x3E0C9928
	.4byte 0x3E126916
	.4byte 0x3E185943
	.4byte 0x3E1E7147
	.4byte 0x3E24A16A
	.4byte 0x3E2B0143
	.4byte 0x3E31797D
	.4byte 0x3E38198F
	.4byte 0x3E3EE17A
	.4byte 0x3E45C183
	.4byte 0x3E4CD185
	.4byte 0x3E53F9A5
	.4byte 0x3E5B51BD
	.4byte 0x3E62C9D1
	.4byte 0x3E6A69BE
	.4byte 0x3E7229E9
	.4byte 0x3E7A11ED
	.4byte 0x3E811106
	.4byte 0x3E852913
	.4byte 0x3E89591D
	.4byte 0x3E8D9924
	.4byte 0x3E91ED18
	.4byte 0x3E965929
	.4byte 0x3E9AD539
	.4byte 0x3E9F6534
	.4byte 0x3EA4093E
	.4byte 0x3EA8C155
	.4byte 0x3EAD9168
	.4byte 0x3EB27158
	.4byte 0x3EB76966
	.4byte 0x3EBC7582
	.4byte 0x3EC19589
	.4byte 0x3EC6CD8E
	.4byte 0x3ECC19A0
	.4byte 0x3ED1799E
	.4byte 0x3ED6F1BB
	.4byte 0x3EDC7DC3
	.4byte 0x3EE221C9
	.4byte 0x3EE7DDCA
	.4byte 0x3EEDADDA
	.4byte 0x3EF395E6
	.4byte 0x3EF991FF
	.4byte 0x3EFFA5F4
	.4byte 0x3F02EB0B
	.4byte 0x3F060B0B
	.4byte 0x3F093911
	.4byte 0x3F0C7315
	.4byte 0x3F0FB928
	.4byte 0x3F130B28
	.4byte 0x3F166927
	.4byte 0x3F19D335
	.4byte 0x3F1D4941
	.4byte 0x3F20CD42
	.4byte 0x3F245B4B
	.4byte 0x3F27F951
	.4byte 0x3F2BA15E
	.4byte 0x3F2F5760
	.4byte 0x3F331B69
	.4byte 0x3F36EB70
	.4byte 0x3F3AC97E
	.4byte 0x3F3EB378
	.4byte 0x3F42AB8A
	.4byte 0x3F46B192
	.4byte 0x3F4AC58F
	.4byte 0x3F4EE59B
	.4byte 0x3F5315A5
	.4byte 0x3F5751AD
	.4byte 0x3F5B9BBB
	.4byte 0x3F5FF5C7
	.4byte 0x3F645DC8
	.4byte 0x3F68D1D9
	.4byte 0x3F6D57DE
	.4byte 0x3F71EBEA
	.4byte 0x3F768BF4
	.4byte 0x3F7B3DF3
	.4byte 0x3F800000
	.4byte 0x3F800000
	.4byte 0
	.4byte 0x3F3538EF
	.4byte 0x3F800000
	.4byte 0x3F800000
	.4byte 0x3F133333
	.4byte 0x3F3538EF
	.4byte 0x3F800000
	.4byte 0x3F800000
	.4byte 0


.section .text, "ax"

.global salCalcVolume
salCalcVolume:
/* 803AE110 003AB070  94 21 FE F0 */	stwu r1, -0x110(r1)
/* 803AE114 003AB074  7C 08 02 A6 */	mflr r0
/* 803AE118 003AB078  90 01 01 14 */	stw r0, 0x114(r1)
/* 803AE11C 003AB07C  DB E1 01 00 */	stfd f31, 0x100(r1)
/* 803AE120 003AB080  F3 E1 01 08 */	psq_st f31, 264(r1), 0, qr0
/* 803AE124 003AB084  DB C1 00 F0 */	stfd f30, 0xf0(r1)
/* 803AE128 003AB088  F3 C1 00 F8 */	psq_st f30, 248(r1), 0, qr0
/* 803AE12C 003AB08C  DB A1 00 E0 */	stfd f29, 0xe0(r1)
/* 803AE130 003AB090  F3 A1 00 E8 */	psq_st f29, 232(r1), 0, qr0
/* 803AE134 003AB094  DB 81 00 D0 */	stfd f28, 0xd0(r1)
/* 803AE138 003AB098  F3 81 00 D8 */	psq_st f28, 216(r1), 0, qr0
/* 803AE13C 003AB09C  DB 61 00 C0 */	stfd f27, 0xc0(r1)
/* 803AE140 003AB0A0  F3 61 00 C8 */	psq_st f27, 200(r1), 0, qr0
/* 803AE144 003AB0A4  DB 41 00 B0 */	stfd f26, 0xb0(r1)
/* 803AE148 003AB0A8  F3 41 00 B8 */	psq_st f26, 184(r1), 0, qr0
/* 803AE14C 003AB0AC  DB 21 00 A0 */	stfd f25, 0xa0(r1)
/* 803AE150 003AB0B0  F3 21 00 A8 */	psq_st f25, 168(r1), 0, qr0
/* 803AE154 003AB0B4  DB 01 00 90 */	stfd f24, 0x90(r1)
/* 803AE158 003AB0B8  F3 01 00 98 */	psq_st f24, 152(r1), 0, qr0
/* 803AE15C 003AB0BC  DA E1 00 80 */	stfd f23, 0x80(r1)
/* 803AE160 003AB0C0  F2 E1 00 88 */	psq_st f23, 136(r1), 0, qr0
/* 803AE164 003AB0C4  DA C1 00 70 */	stfd f22, 0x70(r1)
/* 803AE168 003AB0C8  F2 C1 00 78 */	psq_st f22, 120(r1), 0, qr0
/* 803AE16C 003AB0CC  DA A1 00 60 */	stfd f21, 0x60(r1)
/* 803AE170 003AB0D0  F2 A1 00 68 */	psq_st f21, 104(r1), 0, qr0
/* 803AE174 003AB0D4  DA 81 00 50 */	stfd f20, 0x50(r1)
/* 803AE178 003AB0D8  F2 81 00 58 */	psq_st f20, 88(r1), 0, qr0
/* 803AE17C 003AB0DC  39 61 00 50 */	addi r11, r1, 0x50
/* 803AE180 003AB0E0  4B FD B8 E5 */	bl func_80389A64
/* 803AE184 003AB0E4  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 803AE188 003AB0E8  3C 60 80 3F */	lis r3, lbl_803F3DA4@ha
/* 803AE18C 003AB0EC  3B E3 3D A4 */	addi r31, r3, lbl_803F3DA4@l
/* 803AE190 003AB0F0  FE C0 08 90 */	fmr f22, f1
/* 803AE194 003AB0F4  3C 60 80 3F */	lis r3, lbl_803F3FC0@ha
/* 803AE198 003AB0F8  FF E0 10 90 */	fmr f31, f2
/* 803AE19C 003AB0FC  FE A0 18 90 */	fmr f21, f3
/* 803AE1A0 003AB100  7C 99 23 78 */	mr r25, r4
/* 803AE1A4 003AB104  7C FA 3B 78 */	mr r26, r7
/* 803AE1A8 003AB108  7D 1B 43 78 */	mr r27, r8
/* 803AE1AC 003AB10C  3B C3 3F C0 */	addi r30, r3, lbl_803F3FC0@l
/* 803AE1B0 003AB110  40 82 00 08 */	bne lbl_803AE1B8
/* 803AE1B4 003AB114  3B FE 00 00 */	addi r31, r30, 0
lbl_803AE1B8:
/* 803AE1B8 003AB118  3C 05 FF 80 */	addis r0, r5, 0xff80
/* 803AE1BC 003AB11C  28 00 00 00 */	cmplwi r0, 0
/* 803AE1C0 003AB120  40 82 00 0C */	bne lbl_803AE1CC
/* 803AE1C4 003AB124  38 A0 00 00 */	li r5, 0
/* 803AE1C8 003AB128  3C C0 00 7F */	lis r6, 0x7f
lbl_803AE1CC:
/* 803AE1CC 003AB12C  3C 80 00 01 */	lis r4, 1
/* 803AE1D0 003AB130  3C 00 43 30 */	lis r0, 0x4330
/* 803AE1D4 003AB134  7C 65 20 10 */	subfc r3, r5, r4
/* 803AE1D8 003AB138  3C E5 FF FF */	addis r7, r5, 0xffff
/* 803AE1DC 003AB13C  7D 05 01 94 */	addze r8, r5
/* 803AE1E0 003AB140  3C 66 FF FF */	addis r3, r6, 0xffff
/* 803AE1E4 003AB144  7C 86 20 10 */	subfc r4, r6, r4
/* 803AE1E8 003AB148  90 01 00 08 */	stw r0, 8(r1)
/* 803AE1EC 003AB14C  7C 86 01 94 */	addze r4, r6
/* 803AE1F0 003AB150  7C A8 28 50 */	subf r5, r8, r5
/* 803AE1F4 003AB154  7C 84 30 50 */	subf r4, r4, r6
/* 803AE1F8 003AB158  90 01 00 10 */	stw r0, 0x10(r1)
/* 803AE1FC 003AB15C  7C E0 28 78 */	andc r0, r7, r5
/* 803AE200 003AB160  C8 42 D5 E0 */	lfd f2, lbl_805AF300@sda21(r2)
/* 803AE204 003AB164  7C 63 20 78 */	andc r3, r3, r4
/* 803AE208 003AB168  90 01 00 0C */	stw r0, 0xc(r1)
/* 803AE20C 003AB16C  C0 62 D5 E8 */	lfs f3, lbl_805AF308@sda21(r2)
/* 803AE210 003AB170  28 1B 00 00 */	cmplwi r27, 0
/* 803AE214 003AB174  90 61 00 14 */	stw r3, 0x14(r1)
/* 803AE218 003AB178  C8 21 00 08 */	lfd f1, 8(r1)
/* 803AE21C 003AB17C  C8 01 00 10 */	lfd f0, 0x10(r1)
/* 803AE220 003AB180  EC 21 10 28 */	fsubs f1, f1, f2
/* 803AE224 003AB184  EC 00 10 28 */	fsubs f0, f0, f2
/* 803AE228 003AB188  EF 03 00 72 */	fmuls f24, f3, f1
/* 803AE22C 003AB18C  EE 83 00 32 */	fmuls f20, f3, f0
/* 803AE230 003AB190  41 82 00 44 */	beq lbl_803AE274
/* 803AE234 003AB194  FC 20 C0 90 */	fmr f1, f24
/* 803AE238 003AB198  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE23C 003AB19C  4B FE 6A 71 */	bl fmod
/* 803AE240 003AB1A0  FF 40 08 18 */	frsp f26, f1
/* 803AE244 003AB1A4  FC 20 C0 90 */	fmr f1, f24
/* 803AE248 003AB1A8  4B FD B7 0D */	bl __cvt_fp2unsigned
/* 803AE24C 003AB1AC  C0 02 D5 F8 */	lfs f0, lbl_805AF318@sda21(r2)
/* 803AE250 003AB1B0  7C 7D 1B 78 */	mr r29, r3
/* 803AE254 003AB1B4  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE258 003AB1B8  EE E0 C0 28 */	fsubs f23, f0, f24
/* 803AE25C 003AB1BC  FC 20 B8 90 */	fmr f1, f23
/* 803AE260 003AB1C0  4B FE 6A 4D */	bl fmod
/* 803AE264 003AB1C4  FF 20 08 18 */	frsp f25, f1
/* 803AE268 003AB1C8  FC 20 B8 90 */	fmr f1, f23
/* 803AE26C 003AB1CC  4B FD B6 E9 */	bl __cvt_fp2unsigned
/* 803AE270 003AB1D0  7C 7C 1B 78 */	mr r28, r3
lbl_803AE274:
/* 803AE274 003AB1D4  28 1A 00 00 */	cmplwi r26, 0
/* 803AE278 003AB1D8  41 82 00 18 */	beq lbl_803AE290
/* 803AE27C 003AB1DC  C0 42 D5 D4 */	lfs f2, lbl_805AF2F4@sda21(r2)
/* 803AE280 003AB1E0  C0 22 D5 FC */	lfs f1, lbl_805AF31C@sda21(r2)
/* 803AE284 003AB1E4  EC 18 10 28 */	fsubs f0, f24, f2
/* 803AE288 003AB1E8  EC 01 00 32 */	fmuls f0, f1, f0
/* 803AE28C 003AB1EC  EF 02 00 2A */	fadds f24, f2, f0
lbl_803AE290:
/* 803AE290 003AB1F0  FC 20 C0 90 */	fmr f1, f24
/* 803AE294 003AB1F4  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE298 003AB1F8  4B FE 6A 15 */	bl fmod
/* 803AE29C 003AB1FC  FF C0 08 18 */	frsp f30, f1
/* 803AE2A0 003AB200  FC 20 C0 90 */	fmr f1, f24
/* 803AE2A4 003AB204  4B FD B6 B1 */	bl __cvt_fp2unsigned
/* 803AE2A8 003AB208  FC 20 A0 90 */	fmr f1, f20
/* 803AE2AC 003AB20C  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE2B0 003AB210  7C 76 1B 78 */	mr r22, r3
/* 803AE2B4 003AB214  4B FE 69 F9 */	bl fmod
/* 803AE2B8 003AB218  FF A0 08 18 */	frsp f29, f1
/* 803AE2BC 003AB21C  FC 20 A0 90 */	fmr f1, f20
/* 803AE2C0 003AB220  4B FD B6 95 */	bl __cvt_fp2unsigned
/* 803AE2C4 003AB224  C0 02 D5 F8 */	lfs f0, lbl_805AF318@sda21(r2)
/* 803AE2C8 003AB228  7C 77 1B 78 */	mr r23, r3
/* 803AE2CC 003AB22C  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE2D0 003AB230  EE E0 C0 28 */	fsubs f23, f0, f24
/* 803AE2D4 003AB234  EE 80 A0 28 */	fsubs f20, f0, f20
/* 803AE2D8 003AB238  FC 20 B8 90 */	fmr f1, f23
/* 803AE2DC 003AB23C  4B FE 69 D1 */	bl fmod
/* 803AE2E0 003AB240  FF 80 08 18 */	frsp f28, f1
/* 803AE2E4 003AB244  FC 20 B8 90 */	fmr f1, f23
/* 803AE2E8 003AB248  4B FD B6 6D */	bl __cvt_fp2unsigned
/* 803AE2EC 003AB24C  FC 20 A0 90 */	fmr f1, f20
/* 803AE2F0 003AB250  C8 42 D5 F0 */	lfd f2, lbl_805AF310@sda21(r2)
/* 803AE2F4 003AB254  7C 78 1B 78 */	mr r24, r3
/* 803AE2F8 003AB258  4B FE 69 B5 */	bl fmod
/* 803AE2FC 003AB25C  FF 60 08 18 */	frsp f27, f1
/* 803AE300 003AB260  FC 20 A0 90 */	fmr f1, f20
/* 803AE304 003AB264  4B FD B6 51 */	bl __cvt_fp2unsigned
/* 803AE308 003AB268  28 1B 00 00 */	cmplwi r27, 0
/* 803AE30C 003AB26C  7C 7A 1B 78 */	mr r26, r3
/* 803AE310 003AB270  40 82 02 7C */	bne lbl_803AE58C
/* 803AE314 003AB274  C0 02 D5 D0 */	lfs f0, lbl_805AF2F0@sda21(r2)
/* 803AE318 003AB278  EE 80 05 B2 */	fmuls f20, f0, f22
/* 803AE31C 003AB27C  FC 20 A0 90 */	fmr f1, f20
/* 803AE320 003AB280  4B FD B6 35 */	bl __cvt_fp2unsigned
/* 803AE324 003AB284  3C 00 43 30 */	lis r0, 0x4330
/* 803AE328 003AB288  C0 E2 D5 D4 */	lfs f7, lbl_805AF2F4@sda21(r2)
/* 803AE32C 003AB28C  90 61 00 14 */	stw r3, 0x14(r1)
/* 803AE330 003AB290  54 64 10 3A */	slwi r4, r3, 2
/* 803AE334 003AB294  3B 9E 02 04 */	addi r28, r30, 0x204
/* 803AE338 003AB298  56 F7 10 3A */	slwi r23, r23, 2
/* 803AE33C 003AB29C  90 01 00 10 */	stw r0, 0x10(r1)
/* 803AE340 003AB2A0  7C 7F 22 14 */	add r3, r31, r4
/* 803AE344 003AB2A4  C8 22 D5 E0 */	lfd f1, lbl_805AF300@sda21(r2)
/* 803AE348 003AB2A8  3B 7C 00 04 */	addi r27, r28, 4
/* 803AE34C 003AB2AC  C8 01 00 10 */	lfd f0, 0x10(r1)
/* 803AE350 003AB2B0  EF 47 E8 28 */	fsubs f26, f7, f29
/* 803AE354 003AB2B4  7C 5C BC 2E */	lfsx f2, r28, r23
/* 803AE358 003AB2B8  57 5A 10 3A */	slwi r26, r26, 2
/* 803AE35C 003AB2BC  EC 60 08 28 */	fsubs f3, f0, f1
/* 803AE360 003AB2C0  7C 3B BC 2E */	lfsx f1, r27, r23
/* 803AE364 003AB2C4  C0 02 D5 D0 */	lfs f0, lbl_805AF2F0@sda21(r2)
/* 803AE368 003AB2C8  7C BF 24 2E */	lfsx f5, r31, r4
/* 803AE36C 003AB2CC  EC 5A 00 B2 */	fmuls f2, f26, f2
/* 803AE370 003AB2D0  ED 14 18 28 */	fsubs f8, f20, f3
/* 803AE374 003AB2D4  C0 83 00 04 */	lfs f4, 4(r3)
/* 803AE378 003AB2D8  EC 3D 00 72 */	fmuls f1, f29, f1
/* 803AE37C 003AB2DC  EF 20 07 F2 */	fmuls f25, f0, f31
/* 803AE380 003AB2E0  C0 62 D5 D8 */	lfs f3, lbl_805AF2F8@sda21(r2)
/* 803AE384 003AB2E4  EC C7 40 28 */	fsubs f6, f7, f8
/* 803AE388 003AB2E8  EC 08 01 32 */	fmuls f0, f8, f4
/* 803AE38C 003AB2EC  56 D6 10 3A */	slwi r22, r22, 2
/* 803AE390 003AB2F0  EC 42 08 2A */	fadds f2, f2, f1
/* 803AE394 003AB2F4  EC 26 01 72 */	fmuls f1, f6, f5
/* 803AE398 003AB2F8  57 18 10 3A */	slwi r24, r24, 2
/* 803AE39C 003AB2FC  EF E7 D8 28 */	fsubs f31, f7, f27
/* 803AE3A0 003AB300  EF 07 F0 28 */	fsubs f24, f7, f30
/* 803AE3A4 003AB304  EC 01 00 2A */	fadds f0, f1, f0
/* 803AE3A8 003AB308  EE E7 E0 28 */	fsubs f23, f7, f28
/* 803AE3AC 003AB30C  FC 20 C8 90 */	fmr f1, f25
/* 803AE3B0 003AB310  EC 40 00 B2 */	fmuls f2, f0, f2
/* 803AE3B4 003AB314  EC 43 00 B2 */	fmuls f2, f3, f2
/* 803AE3B8 003AB318  D0 59 00 08 */	stfs f2, 8(r25)
/* 803AE3BC 003AB31C  7C 9C D4 2E */	lfsx f4, r28, r26
/* 803AE3C0 003AB320  7C 5B D4 2E */	lfsx f2, r27, r26
/* 803AE3C4 003AB324  7C 7C B4 2E */	lfsx f3, r28, r22
/* 803AE3C8 003AB328  EC BF 01 32 */	fmuls f5, f31, f4
/* 803AE3CC 003AB32C  EC 9B 00 B2 */	fmuls f4, f27, f2
/* 803AE3D0 003AB330  7C 5B B4 2E */	lfsx f2, r27, r22
/* 803AE3D4 003AB334  EC 78 00 F2 */	fmuls f3, f24, f3
/* 803AE3D8 003AB338  EC 5E 00 B2 */	fmuls f2, f30, f2
/* 803AE3DC 003AB33C  EC 85 20 2A */	fadds f4, f5, f4
/* 803AE3E0 003AB340  EC 43 10 2A */	fadds f2, f3, f2
/* 803AE3E4 003AB344  EC 00 01 32 */	fmuls f0, f0, f4
/* 803AE3E8 003AB348  EC 40 00 B2 */	fmuls f2, f0, f2
/* 803AE3EC 003AB34C  D0 59 00 04 */	stfs f2, 4(r25)
/* 803AE3F0 003AB350  7C 7C C4 2E */	lfsx f3, r28, r24
/* 803AE3F4 003AB354  7C 5B C4 2E */	lfsx f2, r27, r24
/* 803AE3F8 003AB358  EC 77 00 F2 */	fmuls f3, f23, f3
/* 803AE3FC 003AB35C  EC 5C 00 B2 */	fmuls f2, f28, f2
/* 803AE400 003AB360  EC 43 10 2A */	fadds f2, f3, f2
/* 803AE404 003AB364  EC 00 00 B2 */	fmuls f0, f0, f2
/* 803AE408 003AB368  D0 19 00 00 */	stfs f0, 0(r25)
/* 803AE40C 003AB36C  4B FD B5 49 */	bl __cvt_fp2unsigned
/* 803AE410 003AB370  3C 00 43 30 */	lis r0, 0x4330
/* 803AE414 003AB374  54 64 10 3A */	slwi r4, r3, 2
/* 803AE418 003AB378  90 61 00 0C */	stw r3, 0xc(r1)
/* 803AE41C 003AB37C  7C 7F 22 14 */	add r3, r31, r4
/* 803AE420 003AB380  7C 1C BC 2E */	lfsx f0, r28, r23
/* 803AE424 003AB384  90 01 00 08 */	stw r0, 8(r1)
/* 803AE428 003AB388  C8 62 D5 E0 */	lfd f3, lbl_805AF300@sda21(r2)
/* 803AE42C 003AB38C  EC 5A 00 32 */	fmuls f2, f26, f0
/* 803AE430 003AB390  C8 01 00 08 */	lfd f0, 8(r1)
/* 803AE434 003AB394  7C 3B BC 2E */	lfsx f1, r27, r23
/* 803AE438 003AB398  EC 60 18 28 */	fsubs f3, f0, f3
/* 803AE43C 003AB39C  C0 02 D5 D0 */	lfs f0, lbl_805AF2F0@sda21(r2)
/* 803AE440 003AB3A0  EC 3D 00 72 */	fmuls f1, f29, f1
/* 803AE444 003AB3A4  C0 A2 D5 D4 */	lfs f5, lbl_805AF2F4@sda21(r2)
/* 803AE448 003AB3A8  EE C0 05 72 */	fmuls f22, f0, f21
/* 803AE44C 003AB3AC  7C 9F 24 2E */	lfsx f4, r31, r4
/* 803AE450 003AB3B0  EC D9 18 28 */	fsubs f6, f25, f3
/* 803AE454 003AB3B4  C0 63 00 04 */	lfs f3, 4(r3)
/* 803AE458 003AB3B8  EC 02 08 2A */	fadds f0, f2, f1
/* 803AE45C 003AB3BC  C0 42 D5 D8 */	lfs f2, lbl_805AF2F8@sda21(r2)
/* 803AE460 003AB3C0  FC 20 B0 90 */	fmr f1, f22
/* 803AE464 003AB3C4  EC A5 30 28 */	fsubs f5, f5, f6
/* 803AE468 003AB3C8  EC 66 00 F2 */	fmuls f3, f6, f3
/* 803AE46C 003AB3CC  EC 85 01 32 */	fmuls f4, f5, f4
/* 803AE470 003AB3D0  EC A4 18 2A */	fadds f5, f4, f3
/* 803AE474 003AB3D4  EC 05 00 32 */	fmuls f0, f5, f0
/* 803AE478 003AB3D8  EC 02 00 32 */	fmuls f0, f2, f0
/* 803AE47C 003AB3DC  D0 19 00 14 */	stfs f0, 0x14(r25)
/* 803AE480 003AB3E0  7C 7C D4 2E */	lfsx f3, r28, r26
/* 803AE484 003AB3E4  7C 1B D4 2E */	lfsx f0, r27, r26
/* 803AE488 003AB3E8  7C 5C B4 2E */	lfsx f2, r28, r22
/* 803AE48C 003AB3EC  EC 9F 00 F2 */	fmuls f4, f31, f3
/* 803AE490 003AB3F0  EC 7B 00 32 */	fmuls f3, f27, f0
/* 803AE494 003AB3F4  7C 1B B4 2E */	lfsx f0, r27, r22
/* 803AE498 003AB3F8  EC 58 00 B2 */	fmuls f2, f24, f2
/* 803AE49C 003AB3FC  EC 1E 00 32 */	fmuls f0, f30, f0
/* 803AE4A0 003AB400  EC 64 18 2A */	fadds f3, f4, f3
/* 803AE4A4 003AB404  EC 02 00 2A */	fadds f0, f2, f0
/* 803AE4A8 003AB408  EC A5 00 F2 */	fmuls f5, f5, f3
/* 803AE4AC 003AB40C  EC 05 00 32 */	fmuls f0, f5, f0
/* 803AE4B0 003AB410  D0 19 00 10 */	stfs f0, 0x10(r25)
/* 803AE4B4 003AB414  7C 5C C4 2E */	lfsx f2, r28, r24
/* 803AE4B8 003AB418  7C 1B C4 2E */	lfsx f0, r27, r24
/* 803AE4BC 003AB41C  EC 57 00 B2 */	fmuls f2, f23, f2
/* 803AE4C0 003AB420  EC 1C 00 32 */	fmuls f0, f28, f0
/* 803AE4C4 003AB424  EC 02 00 2A */	fadds f0, f2, f0
/* 803AE4C8 003AB428  EC 05 00 32 */	fmuls f0, f5, f0
/* 803AE4CC 003AB42C  D0 19 00 0C */	stfs f0, 0xc(r25)
/* 803AE4D0 003AB430  4B FD B4 85 */	bl __cvt_fp2unsigned
/* 803AE4D4 003AB434  3C 00 43 30 */	lis r0, 0x4330
/* 803AE4D8 003AB438  54 64 10 3A */	slwi r4, r3, 2
/* 803AE4DC 003AB43C  90 61 00 1C */	stw r3, 0x1c(r1)
/* 803AE4E0 003AB440  7C 7F 22 14 */	add r3, r31, r4
/* 803AE4E4 003AB444  7C 1C BC 2E */	lfsx f0, r28, r23
/* 803AE4E8 003AB448  90 01 00 18 */	stw r0, 0x18(r1)
/* 803AE4EC 003AB44C  C8 62 D5 E0 */	lfd f3, lbl_805AF300@sda21(r2)
/* 803AE4F0 003AB450  EC 3A 00 32 */	fmuls f1, f26, f0
/* 803AE4F4 003AB454  C8 41 00 18 */	lfd f2, 0x18(r1)
/* 803AE4F8 003AB458  7C 1B BC 2E */	lfsx f0, r27, r23
/* 803AE4FC 003AB45C  EC A2 18 28 */	fsubs f5, f2, f3
/* 803AE500 003AB460  C0 82 D5 D4 */	lfs f4, lbl_805AF2F4@sda21(r2)
/* 803AE504 003AB464  EC 1D 00 32 */	fmuls f0, f29, f0
/* 803AE508 003AB468  7C 7F 24 2E */	lfsx f3, r31, r4
/* 803AE50C 003AB46C  C0 43 00 04 */	lfs f2, 4(r3)
/* 803AE510 003AB470  EC B6 28 28 */	fsubs f5, f22, f5
/* 803AE514 003AB474  EC 01 00 2A */	fadds f0, f1, f0
/* 803AE518 003AB478  C0 22 D5 D8 */	lfs f1, lbl_805AF2F8@sda21(r2)
/* 803AE51C 003AB47C  EC 84 28 28 */	fsubs f4, f4, f5
/* 803AE520 003AB480  EC 45 00 B2 */	fmuls f2, f5, f2
/* 803AE524 003AB484  EC 64 00 F2 */	fmuls f3, f4, f3
/* 803AE528 003AB488  EC 83 10 2A */	fadds f4, f3, f2
/* 803AE52C 003AB48C  EC 04 00 32 */	fmuls f0, f4, f0
/* 803AE530 003AB490  EC 01 00 32 */	fmuls f0, f1, f0
/* 803AE534 003AB494  D0 19 00 20 */	stfs f0, 0x20(r25)
/* 803AE538 003AB498  7C 5C D4 2E */	lfsx f2, r28, r26
/* 803AE53C 003AB49C  7C 1B D4 2E */	lfsx f0, r27, r26
/* 803AE540 003AB4A0  7C 3C B4 2E */	lfsx f1, r28, r22
/* 803AE544 003AB4A4  EC 7F 00 B2 */	fmuls f3, f31, f2
/* 803AE548 003AB4A8  EC 5B 00 32 */	fmuls f2, f27, f0
/* 803AE54C 003AB4AC  7C 1B B4 2E */	lfsx f0, r27, r22
/* 803AE550 003AB4B0  EC 38 00 72 */	fmuls f1, f24, f1
/* 803AE554 003AB4B4  EC 1E 00 32 */	fmuls f0, f30, f0
/* 803AE558 003AB4B8  EC 43 10 2A */	fadds f2, f3, f2
/* 803AE55C 003AB4BC  EC 01 00 2A */	fadds f0, f1, f0
/* 803AE560 003AB4C0  EC 84 00 B2 */	fmuls f4, f4, f2
/* 803AE564 003AB4C4  EC 04 00 32 */	fmuls f0, f4, f0
/* 803AE568 003AB4C8  D0 19 00 1C */	stfs f0, 0x1c(r25)
/* 803AE56C 003AB4CC  7C 3C C4 2E */	lfsx f1, r28, r24
/* 803AE570 003AB4D0  7C 1B C4 2E */	lfsx f0, r27, r24
/* 803AE574 003AB4D4  EC 37 00 72 */	fmuls f1, f23, f1
/* 803AE578 003AB4D8  EC 1C 00 32 */	fmuls f0, f28, f0
/* 803AE57C 003AB4DC  EC 01 00 2A */	fadds f0, f1, f0
/* 803AE580 003AB4E0  EC 04 00 32 */	fmuls f0, f4, f0
/* 803AE584 003AB4E4  D0 19 00 18 */	stfs f0, 0x18(r25)
/* 803AE588 003AB4E8  48 00 02 00 */	b lbl_803AE788
lbl_803AE58C:
/* 803AE58C 003AB4EC  C0 02 D5 D0 */	lfs f0, lbl_805AF2F0@sda21(r2)
/* 803AE590 003AB4F0  EE 80 05 B2 */	fmuls f20, f0, f22
/* 803AE594 003AB4F4  FC 20 A0 90 */	fmr f1, f20
/* 803AE598 003AB4F8  4B FD B3 BD */	bl __cvt_fp2unsigned
/* 803AE59C 003AB4FC  3C 00 43 30 */	lis r0, 0x4330
/* 803AE5A0 003AB500  C0 02 D5 D4 */	lfs f0, lbl_805AF2F4@sda21(r2)
/* 803AE5A4 003AB504  90 61 00 1C */	stw r3, 0x1c(r1)
/* 803AE5A8 003AB508  54 64 10 3A */	slwi r4, r3, 2
/* 803AE5AC 003AB50C  3B 7E 02 04 */	addi r27, r30, 0x204
/* 803AE5B0 003AB510  56 F7 10 3A */	slwi r23, r23, 2
/* 803AE5B4 003AB514  90 01 00 18 */	stw r0, 0x18(r1)
/* 803AE5B8 003AB518  7C 7F 22 14 */	add r3, r31, r4
/* 803AE5BC 003AB51C  C8 62 D5 E0 */	lfd f3, lbl_805AF300@sda21(r2)
/* 803AE5C0 003AB520  3A BB 00 04 */	addi r21, r27, 4
/* 803AE5C4 003AB524  C8 41 00 18 */	lfd f2, 0x18(r1)
/* 803AE5C8 003AB528  57 5A 10 3A */	slwi r26, r26, 2
/* 803AE5CC 003AB52C  56 D6 10 3A */	slwi r22, r22, 2
/* 803AE5D0 003AB530  7C 35 BC 2E */	lfsx f1, r21, r23
/* 803AE5D4 003AB534  EC A2 18 28 */	fsubs f5, f2, f3
/* 803AE5D8 003AB538  7C 75 D4 2E */	lfsx f3, r21, r26
/* 803AE5DC 003AB53C  7C 55 B4 2E */	lfsx f2, r21, r22
/* 803AE5E0 003AB540  EE E0 E8 28 */	fsubs f23, f0, f29
/* 803AE5E4 003AB544  7C 9B BC 2E */	lfsx f4, r27, r23
/* 803AE5E8 003AB548  EF 00 D8 28 */	fsubs f24, f0, f27
/* 803AE5EC 003AB54C  ED 34 28 28 */	fsubs f9, f20, f5
/* 803AE5F0 003AB550  C0 C3 00 04 */	lfs f6, 4(r3)
/* 803AE5F4 003AB554  7C BB D4 2E */	lfsx f5, r27, r26
/* 803AE5F8 003AB558  ED 17 01 32 */	fmuls f8, f23, f4
/* 803AE5FC 003AB55C  7D 5F 24 2E */	lfsx f10, r31, r4
/* 803AE600 003AB560  EC FD 00 72 */	fmuls f7, f29, f1
/* 803AE604 003AB564  ED 60 48 28 */	fsubs f11, f0, f9
/* 803AE608 003AB568  7C 9B B4 2E */	lfsx f4, r27, r22
/* 803AE60C 003AB56C  EE A0 F0 28 */	fsubs f21, f0, f30
/* 803AE610 003AB570  C0 22 D5 D0 */	lfs f1, lbl_805AF2F0@sda21(r2)
/* 803AE614 003AB574  ED 29 01 B2 */	fmuls f9, f9, f6
/* 803AE618 003AB578  57 18 10 3A */	slwi r24, r24, 2
/* 803AE61C 003AB57C  ED 4B 02 B2 */	fmuls f10, f11, f10
/* 803AE620 003AB580  57 A0 10 3A */	slwi r0, r29, 2
/* 803AE624 003AB584  EC D8 01 72 */	fmuls f6, f24, f5
/* 803AE628 003AB588  38 7E 02 14 */	addi r3, r30, 0x214
/* 803AE62C 003AB58C  EC BB 00 F2 */	fmuls f5, f27, f3
/* 803AE630 003AB590  57 84 10 3A */	slwi r4, r28, 2
/* 803AE634 003AB594  EC 6A 48 2A */	fadds f3, f10, f9
/* 803AE638 003AB598  EC E8 38 2A */	fadds f7, f8, f7
/* 803AE63C 003AB59C  EC C6 28 2A */	fadds f6, f6, f5
/* 803AE640 003AB5A0  EC B5 01 32 */	fmuls f5, f21, f4
/* 803AE644 003AB5A4  EC 9E 00 B2 */	fmuls f4, f30, f2
/* 803AE648 003AB5A8  EC 43 01 F2 */	fmuls f2, f3, f7
/* 803AE64C 003AB5AC  EC 63 01 B2 */	fmuls f3, f3, f6
/* 803AE650 003AB5B0  EC 85 20 2A */	fadds f4, f5, f4
/* 803AE654 003AB5B4  EE 80 E0 28 */	fsubs f20, f0, f28
/* 803AE658 003AB5B8  EE C1 07 F2 */	fmuls f22, f1, f31
/* 803AE65C 003AB5BC  EC 23 01 32 */	fmuls f1, f3, f4
/* 803AE660 003AB5C0  EC A0 D0 28 */	fsubs f5, f0, f26
/* 803AE664 003AB5C4  EC 80 C8 28 */	fsubs f4, f0, f25
/* 803AE668 003AB5C8  D0 39 00 04 */	stfs f1, 4(r25)
/* 803AE66C 003AB5CC  FC 20 B0 90 */	fmr f1, f22
/* 803AE670 003AB5D0  7C DB C4 2E */	lfsx f6, r27, r24
/* 803AE674 003AB5D4  7C 15 C4 2E */	lfsx f0, r21, r24
/* 803AE678 003AB5D8  EC D4 01 B2 */	fmuls f6, f20, f6
/* 803AE67C 003AB5DC  EC 1C 00 32 */	fmuls f0, f28, f0
/* 803AE680 003AB5E0  EC 06 00 2A */	fadds f0, f6, f0
/* 803AE684 003AB5E4  EC 03 00 32 */	fmuls f0, f3, f0
/* 803AE688 003AB5E8  D0 19 00 00 */	stfs f0, 0(r25)
/* 803AE68C 003AB5EC  7C 63 04 2E */	lfsx f3, r3, r0
/* 803AE690 003AB5F0  7C 15 04 2E */	lfsx f0, r21, r0
/* 803AE694 003AB5F4  EC 65 00 F2 */	fmuls f3, f5, f3
/* 803AE698 003AB5F8  EC 1A 00 32 */	fmuls f0, f26, f0
/* 803AE69C 003AB5FC  EC 03 00 2A */	fadds f0, f3, f0
/* 803AE6A0 003AB600  EC 02 00 32 */	fmuls f0, f2, f0
/* 803AE6A4 003AB604  D0 19 00 1C */	stfs f0, 0x1c(r25)
/* 803AE6A8 003AB608  7C 63 24 2E */	lfsx f3, r3, r4
/* 803AE6AC 003AB60C  7C 15 24 2E */	lfsx f0, r21, r4
/* 803AE6B0 003AB610  EC 64 00 F2 */	fmuls f3, f4, f3
/* 803AE6B4 003AB614  EC 19 00 32 */	fmuls f0, f25, f0
/* 803AE6B8 003AB618  EC 03 00 2A */	fadds f0, f3, f0
/* 803AE6BC 003AB61C  EC 02 00 32 */	fmuls f0, f2, f0
/* 803AE6C0 003AB620  D0 19 00 18 */	stfs f0, 0x18(r25)
/* 803AE6C4 003AB624  4B FD B2 91 */	bl __cvt_fp2unsigned
/* 803AE6C8 003AB628  3C 00 43 30 */	lis r0, 0x4330
/* 803AE6CC 003AB62C  54 64 10 3A */	slwi r4, r3, 2
/* 803AE6D0 003AB630  90 61 00 14 */	stw r3, 0x14(r1)
/* 803AE6D4 003AB634  7C 7F 22 14 */	add r3, r31, r4
/* 803AE6D8 003AB638  7C 1B BC 2E */	lfsx f0, r27, r23
/* 803AE6DC 003AB63C  90 01 00 10 */	stw r0, 0x10(r1)
/* 803AE6E0 003AB640  C8 62 D5 E0 */	lfd f3, lbl_805AF300@sda21(r2)
/* 803AE6E4 003AB644  EC 37 00 32 */	fmuls f1, f23, f0
/* 803AE6E8 003AB648  C8 41 00 10 */	lfd f2, 0x10(r1)
/* 803AE6EC 003AB64C  7C 15 BC 2E */	lfsx f0, r21, r23
/* 803AE6F0 003AB650  EC 42 18 28 */	fsubs f2, f2, f3
/* 803AE6F4 003AB654  C0 A2 D5 D4 */	lfs f5, lbl_805AF2F4@sda21(r2)
/* 803AE6F8 003AB658  EC 1D 00 32 */	fmuls f0, f29, f0
/* 803AE6FC 003AB65C  7C 9F 24 2E */	lfsx f4, r31, r4
/* 803AE700 003AB660  C0 63 00 04 */	lfs f3, 4(r3)
/* 803AE704 003AB664  EC D6 10 28 */	fsubs f6, f22, f2
/* 803AE708 003AB668  EC 21 00 2A */	fadds f1, f1, f0
/* 803AE70C 003AB66C  C0 42 D5 D8 */	lfs f2, lbl_805AF2F8@sda21(r2)
/* 803AE710 003AB670  C0 02 D6 00 */	lfs f0, lbl_805AF320@sda21(r2)
/* 803AE714 003AB674  EC A5 30 28 */	fsubs f5, f5, f6
/* 803AE718 003AB678  EC 66 00 F2 */	fmuls f3, f6, f3
/* 803AE71C 003AB67C  EC 85 01 32 */	fmuls f4, f5, f4
/* 803AE720 003AB680  EC A4 18 2A */	fadds f5, f4, f3
/* 803AE724 003AB684  EC 25 00 72 */	fmuls f1, f5, f1
/* 803AE728 003AB688  EC 22 00 72 */	fmuls f1, f2, f1
/* 803AE72C 003AB68C  D0 39 00 14 */	stfs f1, 0x14(r25)
/* 803AE730 003AB690  7C 7B D4 2E */	lfsx f3, r27, r26
/* 803AE734 003AB694  7C 35 D4 2E */	lfsx f1, r21, r26
/* 803AE738 003AB698  7C 5B B4 2E */	lfsx f2, r27, r22
/* 803AE73C 003AB69C  EC 98 00 F2 */	fmuls f4, f24, f3
/* 803AE740 003AB6A0  EC 7B 00 72 */	fmuls f3, f27, f1
/* 803AE744 003AB6A4  7C 35 B4 2E */	lfsx f1, r21, r22
/* 803AE748 003AB6A8  EC 55 00 B2 */	fmuls f2, f21, f2
/* 803AE74C 003AB6AC  EC 3E 00 72 */	fmuls f1, f30, f1
/* 803AE750 003AB6B0  EC 64 18 2A */	fadds f3, f4, f3
/* 803AE754 003AB6B4  EC 22 08 2A */	fadds f1, f2, f1
/* 803AE758 003AB6B8  EC A5 00 F2 */	fmuls f5, f5, f3
/* 803AE75C 003AB6BC  EC 25 00 72 */	fmuls f1, f5, f1
/* 803AE760 003AB6C0  D0 39 00 10 */	stfs f1, 0x10(r25)
/* 803AE764 003AB6C4  7C 5B C4 2E */	lfsx f2, r27, r24
/* 803AE768 003AB6C8  7C 35 C4 2E */	lfsx f1, r21, r24
/* 803AE76C 003AB6CC  EC 54 00 B2 */	fmuls f2, f20, f2
/* 803AE770 003AB6D0  EC 3C 00 72 */	fmuls f1, f28, f1
/* 803AE774 003AB6D4  EC 22 08 2A */	fadds f1, f2, f1
/* 803AE778 003AB6D8  EC 25 00 72 */	fmuls f1, f5, f1
/* 803AE77C 003AB6DC  D0 39 00 0C */	stfs f1, 0xc(r25)
/* 803AE780 003AB6E0  D0 19 00 08 */	stfs f0, 8(r25)
/* 803AE784 003AB6E4  D0 19 00 20 */	stfs f0, 0x20(r25)
lbl_803AE788:
/* 803AE788 003AB6E8  E3 E1 01 08 */	psq_l f31, 264(r1), 0, qr0
/* 803AE78C 003AB6EC  CB E1 01 00 */	lfd f31, 0x100(r1)
/* 803AE790 003AB6F0  E3 C1 00 F8 */	psq_l f30, 248(r1), 0, qr0
/* 803AE794 003AB6F4  CB C1 00 F0 */	lfd f30, 0xf0(r1)
/* 803AE798 003AB6F8  E3 A1 00 E8 */	psq_l f29, 232(r1), 0, qr0
/* 803AE79C 003AB6FC  CB A1 00 E0 */	lfd f29, 0xe0(r1)
/* 803AE7A0 003AB700  E3 81 00 D8 */	psq_l f28, 216(r1), 0, qr0
/* 803AE7A4 003AB704  CB 81 00 D0 */	lfd f28, 0xd0(r1)
/* 803AE7A8 003AB708  E3 61 00 C8 */	psq_l f27, 200(r1), 0, qr0
/* 803AE7AC 003AB70C  CB 61 00 C0 */	lfd f27, 0xc0(r1)
/* 803AE7B0 003AB710  E3 41 00 B8 */	psq_l f26, 184(r1), 0, qr0
/* 803AE7B4 003AB714  CB 41 00 B0 */	lfd f26, 0xb0(r1)
/* 803AE7B8 003AB718  E3 21 00 A8 */	psq_l f25, 168(r1), 0, qr0
/* 803AE7BC 003AB71C  CB 21 00 A0 */	lfd f25, 0xa0(r1)
/* 803AE7C0 003AB720  E3 01 00 98 */	psq_l f24, 152(r1), 0, qr0
/* 803AE7C4 003AB724  CB 01 00 90 */	lfd f24, 0x90(r1)
/* 803AE7C8 003AB728  E2 E1 00 88 */	psq_l f23, 136(r1), 0, qr0
/* 803AE7CC 003AB72C  CA E1 00 80 */	lfd f23, 0x80(r1)
/* 803AE7D0 003AB730  E2 C1 00 78 */	psq_l f22, 120(r1), 0, qr0
/* 803AE7D4 003AB734  CA C1 00 70 */	lfd f22, 0x70(r1)
/* 803AE7D8 003AB738  E2 A1 00 68 */	psq_l f21, 104(r1), 0, qr0
/* 803AE7DC 003AB73C  CA A1 00 60 */	lfd f21, 0x60(r1)
/* 803AE7E0 003AB740  E2 81 00 58 */	psq_l f20, 88(r1), 0, qr0
/* 803AE7E4 003AB744  39 61 00 50 */	addi r11, r1, 0x50
/* 803AE7E8 003AB748  CA 81 00 50 */	lfd f20, 0x50(r1)
/* 803AE7EC 003AB74C  4B FD B2 C5 */	bl _restgpr_21
/* 803AE7F0 003AB750  80 01 01 14 */	lwz r0, 0x114(r1)
/* 803AE7F4 003AB754  7C 08 03 A6 */	mtlr r0
/* 803AE7F8 003AB758  38 21 01 10 */	addi r1, r1, 0x110
/* 803AE7FC 003AB75C  4E 80 00 20 */	blr
