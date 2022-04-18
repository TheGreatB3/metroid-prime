.include "macros.inc"

.section .data

.global lbl_803EF840
lbl_803EF840:
	# ROM: 0x3EC840
	.4byte 0x00010002
	.4byte 0x00040008
	.4byte 0x00100020
	.4byte 0x00400080
	.4byte 0x01000200
	.4byte 0x04000800
	.4byte 0x10002000
	.4byte 0x40000000
	.4byte 0xF8000004
	.4byte 0x008700D5
	.4byte 0x01110143
	.4byte 0x017501A9
	.4byte 0x01A90175
	.4byte 0x01430111
	.4byte 0x00D50087
	.4byte 0x0004F800
	.4byte 0xFFF40012
	.4byte 0x00290040
	.4byte 0x007000C6
	.4byte 0x01630462
	.4byte 0x04620163
	.4byte 0x00C60070
	.4byte 0x00400029
	.4byte 0x0012FFF4
	.4byte 0
	.4byte 0x00000200
	.4byte 0x02000200
	.4byte 0x06000E00
	.4byte 0x0E000600
	.4byte 0x02000200
	.4byte 0x02000000
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0

.section .text, "ax"  # 0x80003640 - 0x803CB1C0

.global g721_decoder__FiP10g72x_state
g721_decoder__FiP10g72x_state:
/* 8036B2B8 00368218  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 8036B2BC 0036821C  7C 08 02 A6 */	mflr r0
/* 8036B2C0 00368220  3C A0 80 3F */	lis r5, lbl_803EF840@ha
/* 8036B2C4 00368224  90 01 00 34 */	stw r0, 0x34(r1)
/* 8036B2C8 00368228  BF 21 00 14 */	stmw r25, 0x14(r1)
/* 8036B2CC 0036822C  7C 9F 23 78 */	mr r31, r4
/* 8036B2D0 00368230  7C 7E 1B 78 */	mr r30, r3
/* 8036B2D4 00368234  3B A5 F8 40 */	addi r29, r5, lbl_803EF840@l
/* 8036B2D8 00368238  7F E3 FB 78 */	mr r3, r31
/* 8036B2DC 0036823C  48 00 07 55 */	bl predictor_zero__FP10g72x_state
/* 8036B2E0 00368240  7C 7B 07 34 */	extsh r27, r3
/* 8036B2E4 00368244  7F E3 FB 78 */	mr r3, r31
/* 8036B2E8 00368248  7F 60 0E 70 */	srawi r0, r27, 1
/* 8036B2EC 0036824C  7C 1A 07 34 */	extsh r26, r0
/* 8036B2F0 00368250  48 00 06 E9 */	bl predictor_pole__FP10g72x_state
/* 8036B2F4 00368254  7C 1B 1A 14 */	add r0, r27, r3
/* 8036B2F8 00368258  7F E3 FB 78 */	mr r3, r31
/* 8036B2FC 0036825C  7C 00 07 34 */	extsh r0, r0
/* 8036B300 00368260  7C 00 0E 70 */	srawi r0, r0, 1
/* 8036B304 00368264  7C 19 07 34 */	extsh r25, r0
/* 8036B308 00368268  48 00 06 7D */	bl step_size__FP10g72x_state
/* 8036B30C 0036826C  57 DC 0E FC */	rlwinm r28, r30, 1, 0x1b, 0x1e
/* 8036B310 00368270  38 9D 00 20 */	addi r4, r29, 0x20
/* 8036B314 00368274  7C 65 07 34 */	extsh r5, r3
/* 8036B318 00368278  7C 84 E2 AE */	lhax r4, r4, r28
/* 8036B31C 0036827C  7C BB 2B 78 */	mr r27, r5
/* 8036B320 00368280  57 C3 07 38 */	rlwinm r3, r30, 0, 0x1c, 0x1c
/* 8036B324 00368284  48 00 06 05 */	bl reconstruct__Fiii
/* 8036B328 00368288  7C 67 07 35 */	extsh. r7, r3
/* 8036B32C 0036828C  7C 19 3A 14 */	add r0, r25, r7
/* 8036B330 00368290  40 80 00 0C */	bge lbl_8036B33C
/* 8036B334 00368294  54 E0 04 BE */	clrlwi r0, r7, 0x12
/* 8036B338 00368298  7C 00 C8 50 */	subf r0, r0, r25
lbl_8036B33C:
/* 8036B33C 0036829C  7C 1E 07 34 */	extsh r30, r0
/* 8036B340 003682A0  38 9D 00 40 */	addi r4, r29, 0x40
/* 8036B344 003682A4  7C A4 E2 AE */	lhax r5, r4, r28
/* 8036B348 003682A8  7C 19 F0 50 */	subf r0, r25, r30
/* 8036B34C 003682AC  38 7D 00 60 */	addi r3, r29, 0x60
/* 8036B350 003682B0  7F 64 DB 78 */	mr r4, r27
/* 8036B354 003682B4  7C 1A 02 14 */	add r0, r26, r0
/* 8036B358 003682B8  7C C3 E2 AE */	lhax r6, r3, r28
/* 8036B35C 003682BC  7C 09 07 34 */	extsh r9, r0
/* 8036B360 003682C0  7F C8 F3 78 */	mr r8, r30
/* 8036B364 003682C4  7F EA FB 78 */	mr r10, r31
/* 8036B368 003682C8  54 A5 28 34 */	slwi r5, r5, 5
/* 8036B36C 003682CC  38 60 00 04 */	li r3, 4
/* 8036B370 003682D0  48 00 00 1D */	bl update__FiiiiiiiP10g72x_state
/* 8036B374 003682D4  57 C3 10 3A */	slwi r3, r30, 2
/* 8036B378 003682D8  BB 21 00 14 */	lmw r25, 0x14(r1)
/* 8036B37C 003682DC  80 01 00 34 */	lwz r0, 0x34(r1)
/* 8036B380 003682E0  7C 08 03 A6 */	mtlr r0
/* 8036B384 003682E4  38 21 00 30 */	addi r1, r1, 0x30
/* 8036B388 003682E8  4E 80 00 20 */	blr 

.global update__FiiiiiiiP10g72x_state
update__FiiiiiiiP10g72x_state:
/* 8036B38C 003682EC  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 8036B390 003682F0  7C 08 02 A6 */	mflr r0
/* 8036B394 003682F4  90 01 00 34 */	stw r0, 0x34(r1)
/* 8036B398 003682F8  BE E1 00 0C */	stmw r23, 0xc(r1)
/* 8036B39C 003682FC  7C FA 3B 78 */	mr r26, r7
/* 8036B3A0 00368300  55 27 0F FE */	srwi r7, r9, 0x1f
/* 8036B3A4 00368304  7C 98 23 78 */	mr r24, r4
/* 8036B3A8 00368308  7C D9 33 78 */	mr r25, r6
/* 8036B3AC 0036830C  7D 5C 53 78 */	mr r28, r10
/* 8036B3B0 00368310  7D 1B 43 78 */	mr r27, r8
/* 8036B3B4 00368314  7C FD 07 34 */	extsh r29, r7
/* 8036B3B8 00368318  80 0A 00 00 */	lwz r0, 0(r10)
/* 8036B3BC 0036831C  7C 04 7E 70 */	srawi r4, r0, 0xf
/* 8036B3C0 00368320  54 00 B6 FE */	rlwinm r0, r0, 0x16, 0x1b, 0x1f
/* 8036B3C4 00368324  7C 86 07 34 */	extsh r6, r4
/* 8036B3C8 00368328  7C 04 07 34 */	extsh r4, r0
/* 8036B3CC 0036832C  38 04 00 20 */	addi r0, r4, 0x20
/* 8036B3D0 00368330  2C 06 00 09 */	cmpwi r6, 9
/* 8036B3D4 00368334  7C 04 30 30 */	slw r4, r0, r6
/* 8036B3D8 00368338  7C 86 07 34 */	extsh r6, r4
/* 8036B3DC 0036833C  57 40 04 7E */	clrlwi r0, r26, 0x11
/* 8036B3E0 00368340  38 80 7C 00 */	li r4, 0x7c00
/* 8036B3E4 00368344  41 81 00 08 */	bgt lbl_8036B3EC
/* 8036B3E8 00368348  7C C4 33 78 */	mr r4, r6
lbl_8036B3EC:
/* 8036B3EC 0036834C  7C 87 07 34 */	extsh r7, r4
/* 8036B3F0 00368350  88 9C 00 30 */	lbz r4, 0x30(r28)
/* 8036B3F4 00368354  7C E6 0E 70 */	srawi r6, r7, 1
/* 8036B3F8 00368358  7C C7 32 14 */	add r6, r7, r6
/* 8036B3FC 0036835C  7C 84 07 75 */	extsb. r4, r4
/* 8036B400 00368360  7C C4 0E 70 */	srawi r4, r6, 1
/* 8036B404 00368364  7C 86 07 34 */	extsh r6, r4
/* 8036B408 00368368  40 82 00 0C */	bne lbl_8036B414
/* 8036B40C 0036836C  3B C0 00 00 */	li r30, 0
/* 8036B410 00368370  48 00 00 1C */	b lbl_8036B42C
lbl_8036B414:
/* 8036B414 00368374  7C 04 07 34 */	extsh r4, r0
/* 8036B418 00368378  7C 04 30 00 */	cmpw r4, r6
/* 8036B41C 0036837C  41 81 00 0C */	bgt lbl_8036B428
/* 8036B420 00368380  3B C0 00 00 */	li r30, 0
/* 8036B424 00368384  48 00 00 08 */	b lbl_8036B42C
lbl_8036B428:
/* 8036B428 00368388  3B C0 00 01 */	li r30, 1
lbl_8036B42C:
/* 8036B42C 0036838C  7C 98 28 50 */	subf r4, r24, r5
/* 8036B430 00368390  7C 84 2E 70 */	srawi r4, r4, 5
/* 8036B434 00368394  7C 98 22 14 */	add r4, r24, r4
/* 8036B438 00368398  B0 9C 00 04 */	sth r4, 4(r28)
/* 8036B43C 0036839C  A8 9C 00 04 */	lha r4, 4(r28)
/* 8036B440 003683A0  2C 04 02 20 */	cmpwi r4, 0x220
/* 8036B444 003683A4  40 80 00 10 */	bge lbl_8036B454
/* 8036B448 003683A8  38 80 02 20 */	li r4, 0x220
/* 8036B44C 003683AC  B0 9C 00 04 */	sth r4, 4(r28)
/* 8036B450 003683B0  48 00 00 14 */	b lbl_8036B464
lbl_8036B454:
/* 8036B454 003683B4  2C 04 14 00 */	cmpwi r4, 0x1400
/* 8036B458 003683B8  40 81 00 0C */	ble lbl_8036B464
/* 8036B45C 003683BC  38 80 14 00 */	li r4, 0x1400
/* 8036B460 003683C0  B0 9C 00 04 */	sth r4, 4(r28)
lbl_8036B464:
/* 8036B464 003683C4  80 DC 00 00 */	lwz r6, 0(r28)
/* 8036B468 003683C8  7F C4 07 74 */	extsb r4, r30
/* 8036B46C 003683CC  A8 BC 00 04 */	lha r5, 4(r28)
/* 8036B470 003683D0  2C 04 00 01 */	cmpwi r4, 1
/* 8036B474 003683D4  7C 86 00 D0 */	neg r4, r6
/* 8036B478 003683D8  7C 84 36 70 */	srawi r4, r4, 6
/* 8036B47C 003683DC  7C 84 32 14 */	add r4, r4, r6
/* 8036B480 003683E0  7C 85 22 14 */	add r4, r5, r4
/* 8036B484 003683E4  90 9C 00 00 */	stw r4, 0(r28)
/* 8036B488 003683E8  40 82 00 2C */	bne lbl_8036B4B4
/* 8036B48C 003683EC  38 60 00 00 */	li r3, 0
/* 8036B490 003683F0  B0 7C 00 0C */	sth r3, 0xc(r28)
/* 8036B494 003683F4  B0 7C 00 0E */	sth r3, 0xe(r28)
/* 8036B498 003683F8  B0 7C 00 10 */	sth r3, 0x10(r28)
/* 8036B49C 003683FC  B0 7C 00 12 */	sth r3, 0x12(r28)
/* 8036B4A0 00368400  B0 7C 00 14 */	sth r3, 0x14(r28)
/* 8036B4A4 00368404  B0 7C 00 16 */	sth r3, 0x16(r28)
/* 8036B4A8 00368408  B0 7C 00 18 */	sth r3, 0x18(r28)
/* 8036B4AC 0036840C  B0 7C 00 1A */	sth r3, 0x1a(r28)
/* 8036B4B0 00368410  48 00 02 10 */	b lbl_8036B6C0
lbl_8036B4B4:
/* 8036B4B4 00368414  A8 DC 00 0E */	lha r6, 0xe(r28)
/* 8036B4B8 00368418  2C 09 00 00 */	cmpwi r9, 0
/* 8036B4BC 0036841C  A8 BC 00 1C */	lha r5, 0x1c(r28)
/* 8036B4C0 00368420  7C C4 3E 70 */	srawi r4, r6, 7
/* 8036B4C4 00368424  7F A5 2A 78 */	xor r5, r29, r5
/* 8036B4C8 00368428  7C 84 30 50 */	subf r4, r4, r6
/* 8036B4CC 0036842C  7C A6 07 34 */	extsh r6, r5
/* 8036B4D0 00368430  7C 9F 07 34 */	extsh r31, r4
/* 8036B4D4 00368434  41 82 00 AC */	beq lbl_8036B580
/* 8036B4D8 00368438  7C C4 07 35 */	extsh. r4, r6
/* 8036B4DC 0036843C  41 82 00 0C */	beq lbl_8036B4E8
/* 8036B4E0 00368440  A8 9C 00 0C */	lha r4, 0xc(r28)
/* 8036B4E4 00368444  48 00 00 0C */	b lbl_8036B4F0
lbl_8036B4E8:
/* 8036B4E8 00368448  A8 9C 00 0C */	lha r4, 0xc(r28)
/* 8036B4EC 0036844C  7C 84 00 D0 */	neg r4, r4
lbl_8036B4F0:
/* 8036B4F0 00368450  7C 84 07 34 */	extsh r4, r4
/* 8036B4F4 00368454  2C 04 E0 01 */	cmpwi r4, -8191
/* 8036B4F8 00368458  40 80 00 0C */	bge lbl_8036B504
/* 8036B4FC 0036845C  3B FF FF 00 */	addi r31, r31, -256
/* 8036B500 00368460  48 00 00 20 */	b lbl_8036B520
lbl_8036B504:
/* 8036B504 00368464  2C 04 1F FF */	cmpwi r4, 0x1fff
/* 8036B508 00368468  40 81 00 0C */	ble lbl_8036B514
/* 8036B50C 0036846C  3B FF 00 FF */	addi r31, r31, 0xff
/* 8036B510 00368470  48 00 00 10 */	b lbl_8036B520
lbl_8036B514:
/* 8036B514 00368474  7C 84 2E 70 */	srawi r4, r4, 5
/* 8036B518 00368478  7C 9F 22 14 */	add r4, r31, r4
/* 8036B51C 0036847C  7C 9F 07 34 */	extsh r31, r4
lbl_8036B520:
/* 8036B520 00368480  A8 9C 00 1E */	lha r4, 0x1e(r28)
/* 8036B524 00368484  7F A4 22 79 */	xor. r4, r29, r4
/* 8036B528 00368488  41 82 00 30 */	beq lbl_8036B558
/* 8036B52C 0036848C  7F E4 07 34 */	extsh r4, r31
/* 8036B530 00368490  2C 04 D0 80 */	cmpwi r4, -12160
/* 8036B534 00368494  41 81 00 0C */	bgt lbl_8036B540
/* 8036B538 00368498  3B E0 D0 00 */	li r31, -12288
/* 8036B53C 0036849C  48 00 00 44 */	b lbl_8036B580
lbl_8036B540:
/* 8036B540 003684A0  2C 04 30 80 */	cmpwi r4, 0x3080
/* 8036B544 003684A4  41 80 00 0C */	blt lbl_8036B550
/* 8036B548 003684A8  3B E0 30 00 */	li r31, 0x3000
/* 8036B54C 003684AC  48 00 00 34 */	b lbl_8036B580
lbl_8036B550:
/* 8036B550 003684B0  3B FF FF 80 */	addi r31, r31, -128
/* 8036B554 003684B4  48 00 00 2C */	b lbl_8036B580
lbl_8036B558:
/* 8036B558 003684B8  7F E4 07 34 */	extsh r4, r31
/* 8036B55C 003684BC  2C 04 CF 80 */	cmpwi r4, -12416
/* 8036B560 003684C0  41 81 00 0C */	bgt lbl_8036B56C
/* 8036B564 003684C4  3B E0 D0 00 */	li r31, -12288
/* 8036B568 003684C8  48 00 00 18 */	b lbl_8036B580
lbl_8036B56C:
/* 8036B56C 003684CC  2C 04 2F 80 */	cmpwi r4, 0x2f80
/* 8036B570 003684D0  41 80 00 0C */	blt lbl_8036B57C
/* 8036B574 003684D4  3B E0 30 00 */	li r31, 0x3000
/* 8036B578 003684D8  48 00 00 08 */	b lbl_8036B580
lbl_8036B57C:
/* 8036B57C 003684DC  3B FF 00 80 */	addi r31, r31, 0x80
lbl_8036B580:
/* 8036B580 003684E0  B3 FC 00 0E */	sth r31, 0xe(r28)
/* 8036B584 003684E4  2C 09 00 00 */	cmpwi r9, 0
/* 8036B588 003684E8  A8 BC 00 0C */	lha r5, 0xc(r28)
/* 8036B58C 003684EC  7C A4 46 70 */	srawi r4, r5, 8
/* 8036B590 003684F0  7C 84 28 50 */	subf r4, r4, r5
/* 8036B594 003684F4  B0 9C 00 0C */	sth r4, 0xc(r28)
/* 8036B598 003684F8  41 82 00 28 */	beq lbl_8036B5C0
/* 8036B59C 003684FC  7C C4 07 35 */	extsh. r4, r6
/* 8036B5A0 00368500  40 82 00 14 */	bne lbl_8036B5B4
/* 8036B5A4 00368504  A8 9C 00 0C */	lha r4, 0xc(r28)
/* 8036B5A8 00368508  38 84 00 C0 */	addi r4, r4, 0xc0
/* 8036B5AC 0036850C  B0 9C 00 0C */	sth r4, 0xc(r28)
/* 8036B5B0 00368510  48 00 00 10 */	b lbl_8036B5C0
lbl_8036B5B4:
/* 8036B5B4 00368514  A8 9C 00 0C */	lha r4, 0xc(r28)
/* 8036B5B8 00368518  38 84 FF 40 */	addi r4, r4, -192
/* 8036B5BC 0036851C  B0 9C 00 0C */	sth r4, 0xc(r28)
lbl_8036B5C0:
/* 8036B5C0 00368520  20 9F 3C 00 */	subfic r4, r31, 0x3c00
/* 8036B5C4 00368524  A8 BC 00 0C */	lha r5, 0xc(r28)
/* 8036B5C8 00368528  7C 86 07 34 */	extsh r6, r4
/* 8036B5CC 0036852C  7C 86 00 D0 */	neg r4, r6
/* 8036B5D0 00368530  7C 05 20 00 */	cmpw r5, r4
/* 8036B5D4 00368534  40 80 00 0C */	bge lbl_8036B5E0
/* 8036B5D8 00368538  B0 9C 00 0C */	sth r4, 0xc(r28)
/* 8036B5DC 0036853C  48 00 00 10 */	b lbl_8036B5EC
lbl_8036B5E0:
/* 8036B5E0 00368540  7C 05 30 00 */	cmpw r5, r6
/* 8036B5E4 00368544  40 81 00 08 */	ble lbl_8036B5EC
/* 8036B5E8 00368548  B0 DC 00 0C */	sth r6, 0xc(r28)
lbl_8036B5EC:
/* 8036B5EC 0036854C  38 80 00 03 */	li r4, 3
/* 8036B5F0 00368550  7F 86 E3 78 */	mr r6, r28
/* 8036B5F4 00368554  38 E0 00 00 */	li r7, 0
/* 8036B5F8 00368558  7C 89 03 A6 */	mtctr r4
lbl_8036B5FC:
/* 8036B5FC 0036855C  2C 03 00 05 */	cmpwi r3, 5
/* 8036B600 00368560  40 82 00 18 */	bne lbl_8036B618
/* 8036B604 00368564  A8 A6 00 10 */	lha r5, 0x10(r6)
/* 8036B608 00368568  7C A4 4E 70 */	srawi r4, r5, 9
/* 8036B60C 0036856C  7C 84 28 50 */	subf r4, r4, r5
/* 8036B610 00368570  B0 86 00 10 */	sth r4, 0x10(r6)
/* 8036B614 00368574  48 00 00 14 */	b lbl_8036B628
lbl_8036B618:
/* 8036B618 00368578  A8 A6 00 10 */	lha r5, 0x10(r6)
/* 8036B61C 0036857C  7C A4 46 70 */	srawi r4, r5, 8
/* 8036B620 00368580  7C 84 28 50 */	subf r4, r4, r5
/* 8036B624 00368584  B0 86 00 10 */	sth r4, 0x10(r6)
lbl_8036B628:
/* 8036B628 00368588  2C 00 00 00 */	cmpwi r0, 0
/* 8036B62C 0036858C  41 82 00 2C */	beq lbl_8036B658
/* 8036B630 00368590  A8 86 00 20 */	lha r4, 0x20(r6)
/* 8036B634 00368594  7F 44 22 79 */	xor. r4, r26, r4
/* 8036B638 00368598  41 80 00 14 */	blt lbl_8036B64C
/* 8036B63C 0036859C  A8 86 00 10 */	lha r4, 0x10(r6)
/* 8036B640 003685A0  38 84 00 80 */	addi r4, r4, 0x80
/* 8036B644 003685A4  B0 86 00 10 */	sth r4, 0x10(r6)
/* 8036B648 003685A8  48 00 00 10 */	b lbl_8036B658
lbl_8036B64C:
/* 8036B64C 003685AC  A8 86 00 10 */	lha r4, 0x10(r6)
/* 8036B650 003685B0  38 84 FF 80 */	addi r4, r4, -128
/* 8036B654 003685B4  B0 86 00 10 */	sth r4, 0x10(r6)
lbl_8036B658:
/* 8036B658 003685B8  2C 03 00 05 */	cmpwi r3, 5
/* 8036B65C 003685BC  40 82 00 18 */	bne lbl_8036B674
/* 8036B660 003685C0  A8 A6 00 12 */	lha r5, 0x12(r6)
/* 8036B664 003685C4  7C A4 4E 70 */	srawi r4, r5, 9
/* 8036B668 003685C8  7C 84 28 50 */	subf r4, r4, r5
/* 8036B66C 003685CC  B0 86 00 12 */	sth r4, 0x12(r6)
/* 8036B670 003685D0  48 00 00 14 */	b lbl_8036B684
lbl_8036B674:
/* 8036B674 003685D4  A8 A6 00 12 */	lha r5, 0x12(r6)
/* 8036B678 003685D8  7C A4 46 70 */	srawi r4, r5, 8
/* 8036B67C 003685DC  7C 84 28 50 */	subf r4, r4, r5
/* 8036B680 003685E0  B0 86 00 12 */	sth r4, 0x12(r6)
lbl_8036B684:
/* 8036B684 003685E4  2C 00 00 00 */	cmpwi r0, 0
/* 8036B688 003685E8  41 82 00 2C */	beq lbl_8036B6B4
/* 8036B68C 003685EC  A8 86 00 22 */	lha r4, 0x22(r6)
/* 8036B690 003685F0  7F 44 22 79 */	xor. r4, r26, r4
/* 8036B694 003685F4  41 80 00 14 */	blt lbl_8036B6A8
/* 8036B698 003685F8  A8 86 00 12 */	lha r4, 0x12(r6)
/* 8036B69C 003685FC  38 84 00 80 */	addi r4, r4, 0x80
/* 8036B6A0 00368600  B0 86 00 12 */	sth r4, 0x12(r6)
/* 8036B6A4 00368604  48 00 00 10 */	b lbl_8036B6B4
lbl_8036B6A8:
/* 8036B6A8 00368608  A8 86 00 12 */	lha r4, 0x12(r6)
/* 8036B6AC 0036860C  38 84 FF 80 */	addi r4, r4, -128
/* 8036B6B0 00368610  B0 86 00 12 */	sth r4, 0x12(r6)
lbl_8036B6B4:
/* 8036B6B4 00368614  38 C6 00 04 */	addi r6, r6, 4
/* 8036B6B8 00368618  38 E7 00 01 */	addi r7, r7, 1
/* 8036B6BC 0036861C  42 00 FF 40 */	bdnz lbl_8036B5FC
lbl_8036B6C0:
/* 8036B6C0 00368620  A8 9C 00 28 */	lha r4, 0x28(r28)
/* 8036B6C4 00368624  7C 03 07 35 */	extsh. r3, r0
/* 8036B6C8 00368628  B0 9C 00 2A */	sth r4, 0x2a(r28)
/* 8036B6CC 0036862C  A8 9C 00 26 */	lha r4, 0x26(r28)
/* 8036B6D0 00368630  B0 9C 00 28 */	sth r4, 0x28(r28)
/* 8036B6D4 00368634  A8 9C 00 24 */	lha r4, 0x24(r28)
/* 8036B6D8 00368638  B0 9C 00 26 */	sth r4, 0x26(r28)
/* 8036B6DC 0036863C  A8 9C 00 22 */	lha r4, 0x22(r28)
/* 8036B6E0 00368640  B0 9C 00 24 */	sth r4, 0x24(r28)
/* 8036B6E4 00368644  A8 9C 00 20 */	lha r4, 0x20(r28)
/* 8036B6E8 00368648  B0 9C 00 22 */	sth r4, 0x22(r28)
/* 8036B6EC 0036864C  40 82 00 20 */	bne lbl_8036B70C
/* 8036B6F0 00368650  2C 1A 00 00 */	cmpwi r26, 0
/* 8036B6F4 00368654  3C 60 00 01 */	lis r3, 0x0000FC20@ha
/* 8036B6F8 00368658  38 03 FC 20 */	addi r0, r3, 0x0000FC20@l
/* 8036B6FC 0036865C  41 80 00 08 */	blt lbl_8036B704
/* 8036B700 00368660  38 00 00 20 */	li r0, 0x20
lbl_8036B704:
/* 8036B704 00368664  B0 1C 00 20 */	sth r0, 0x20(r28)
/* 8036B708 00368668  48 00 00 54 */	b lbl_8036B75C
lbl_8036B70C:
/* 8036B70C 0036866C  3C 60 80 3F */	lis r3, lbl_803EF840@ha
/* 8036B710 00368670  7C 17 07 34 */	extsh r23, r0
/* 8036B714 00368674  38 83 F8 40 */	addi r4, r3, lbl_803EF840@l
/* 8036B718 00368678  38 A0 00 0F */	li r5, 0xf
/* 8036B71C 0036867C  7E E3 BB 78 */	mr r3, r23
/* 8036B720 00368680  48 00 04 E5 */	bl quan__FiPsi
/* 8036B724 00368684  2C 1A 00 00 */	cmpwi r26, 0
/* 8036B728 00368688  7C 65 07 34 */	extsh r5, r3
/* 8036B72C 0036868C  41 80 00 18 */	blt lbl_8036B744
/* 8036B730 00368690  56 E0 30 32 */	slwi r0, r23, 6
/* 8036B734 00368694  54 A3 30 32 */	slwi r3, r5, 6
/* 8036B738 00368698  7C 00 2E 30 */	sraw r0, r0, r5
/* 8036B73C 0036869C  7C 03 02 14 */	add r0, r3, r0
/* 8036B740 003686A0  48 00 00 18 */	b lbl_8036B758
lbl_8036B744:
/* 8036B744 003686A4  56 E0 30 32 */	slwi r0, r23, 6
/* 8036B748 003686A8  54 A4 30 32 */	slwi r4, r5, 6
/* 8036B74C 003686AC  7C 03 2E 30 */	sraw r3, r0, r5
/* 8036B750 003686B0  38 03 FC 00 */	addi r0, r3, -1024
/* 8036B754 003686B4  7C 04 02 14 */	add r0, r4, r0
lbl_8036B758:
/* 8036B758 003686B8  B0 1C 00 20 */	sth r0, 0x20(r28)
lbl_8036B75C:
/* 8036B75C 003686BC  A8 1C 00 2C */	lha r0, 0x2c(r28)
/* 8036B760 003686C0  2C 1B 00 00 */	cmpwi r27, 0
/* 8036B764 003686C4  B0 1C 00 2E */	sth r0, 0x2e(r28)
/* 8036B768 003686C8  40 82 00 10 */	bne lbl_8036B778
/* 8036B76C 003686CC  38 00 00 20 */	li r0, 0x20
/* 8036B770 003686D0  B0 1C 00 2C */	sth r0, 0x2c(r28)
/* 8036B774 003686D4  48 00 00 84 */	b lbl_8036B7F8
lbl_8036B778:
/* 8036B778 003686D8  40 81 00 34 */	ble lbl_8036B7AC
/* 8036B77C 003686DC  3C 80 80 3F */	lis r4, lbl_803EF840@ha
/* 8036B780 003686E0  7F 63 DB 78 */	mr r3, r27
/* 8036B784 003686E4  38 84 F8 40 */	addi r4, r4, lbl_803EF840@l
/* 8036B788 003686E8  38 A0 00 0F */	li r5, 0xf
/* 8036B78C 003686EC  48 00 04 79 */	bl quan__FiPsi
/* 8036B790 003686F0  7C 64 07 34 */	extsh r4, r3
/* 8036B794 003686F4  57 60 30 32 */	slwi r0, r27, 6
/* 8036B798 003686F8  54 83 30 32 */	slwi r3, r4, 6
/* 8036B79C 003686FC  7C 00 26 30 */	sraw r0, r0, r4
/* 8036B7A0 00368700  7C 03 02 14 */	add r0, r3, r0
/* 8036B7A4 00368704  B0 1C 00 2C */	sth r0, 0x2c(r28)
/* 8036B7A8 00368708  48 00 00 50 */	b lbl_8036B7F8
lbl_8036B7AC:
/* 8036B7AC 0036870C  2C 1B 80 00 */	cmpwi r27, -32768
/* 8036B7B0 00368710  40 81 00 40 */	ble lbl_8036B7F0
/* 8036B7B4 00368714  7C 1B 00 D0 */	neg r0, r27
/* 8036B7B8 00368718  3C 60 80 3F */	lis r3, lbl_803EF840@ha
/* 8036B7BC 0036871C  7C 1A 07 34 */	extsh r26, r0
/* 8036B7C0 00368720  38 A0 00 0F */	li r5, 0xf
/* 8036B7C4 00368724  38 83 F8 40 */	addi r4, r3, lbl_803EF840@l
/* 8036B7C8 00368728  7F 43 D3 78 */	mr r3, r26
/* 8036B7CC 0036872C  48 00 04 39 */	bl quan__FiPsi
/* 8036B7D0 00368730  7C 64 07 34 */	extsh r4, r3
/* 8036B7D4 00368734  57 40 30 32 */	slwi r0, r26, 6
/* 8036B7D8 00368738  7C 03 26 30 */	sraw r3, r0, r4
/* 8036B7DC 0036873C  54 84 30 32 */	slwi r4, r4, 6
/* 8036B7E0 00368740  38 03 FC 00 */	addi r0, r3, -1024
/* 8036B7E4 00368744  7C 04 02 14 */	add r0, r4, r0
/* 8036B7E8 00368748  B0 1C 00 2C */	sth r0, 0x2c(r28)
/* 8036B7EC 0036874C  48 00 00 0C */	b lbl_8036B7F8
lbl_8036B7F0:
/* 8036B7F0 00368750  38 00 FC 20 */	li r0, -992
/* 8036B7F4 00368754  B0 1C 00 2C */	sth r0, 0x2c(r28)
lbl_8036B7F8:
/* 8036B7F8 00368758  A8 7C 00 1C */	lha r3, 0x1c(r28)
/* 8036B7FC 0036875C  7F C0 07 74 */	extsb r0, r30
/* 8036B800 00368760  2C 00 00 01 */	cmpwi r0, 1
/* 8036B804 00368764  B0 7C 00 1E */	sth r3, 0x1e(r28)
/* 8036B808 00368768  B3 BC 00 1C */	sth r29, 0x1c(r28)
/* 8036B80C 0036876C  40 82 00 10 */	bne lbl_8036B81C
/* 8036B810 00368770  38 00 00 00 */	li r0, 0
/* 8036B814 00368774  98 1C 00 30 */	stb r0, 0x30(r28)
/* 8036B818 00368778  48 00 00 24 */	b lbl_8036B83C
lbl_8036B81C:
/* 8036B81C 0036877C  7F E0 07 34 */	extsh r0, r31
/* 8036B820 00368780  2C 00 D2 00 */	cmpwi r0, -11776
/* 8036B824 00368784  40 80 00 10 */	bge lbl_8036B834
/* 8036B828 00368788  38 00 00 01 */	li r0, 1
/* 8036B82C 0036878C  98 1C 00 30 */	stb r0, 0x30(r28)
/* 8036B830 00368790  48 00 00 0C */	b lbl_8036B83C
lbl_8036B834:
/* 8036B834 00368794  38 00 00 00 */	li r0, 0
/* 8036B838 00368798  98 1C 00 30 */	stb r0, 0x30(r28)
lbl_8036B83C:
/* 8036B83C 0036879C  A8 BC 00 06 */	lha r5, 6(r28)
/* 8036B840 003687A0  7F C0 07 74 */	extsb r0, r30
/* 8036B844 003687A4  2C 00 00 01 */	cmpwi r0, 1
/* 8036B848 003687A8  57 23 10 3A */	slwi r3, r25, 2
/* 8036B84C 003687AC  7C 85 C8 50 */	subf r4, r5, r25
/* 8036B850 003687B0  7C 80 2E 70 */	srawi r0, r4, 5
/* 8036B854 003687B4  7C 05 02 14 */	add r0, r5, r0
/* 8036B858 003687B8  B0 1C 00 06 */	sth r0, 6(r28)
/* 8036B85C 003687BC  A8 9C 00 08 */	lha r4, 8(r28)
/* 8036B860 003687C0  7C 04 18 50 */	subf r0, r4, r3
/* 8036B864 003687C4  7C 00 3E 70 */	srawi r0, r0, 7
/* 8036B868 003687C8  7C 04 02 14 */	add r0, r4, r0
/* 8036B86C 003687CC  B0 1C 00 08 */	sth r0, 8(r28)
/* 8036B870 003687D0  40 82 00 10 */	bne lbl_8036B880
/* 8036B874 003687D4  38 00 01 00 */	li r0, 0x100
/* 8036B878 003687D8  B0 1C 00 0A */	sth r0, 0xa(r28)
/* 8036B87C 003687DC  48 00 00 98 */	b lbl_8036B914
lbl_8036B880:
/* 8036B880 003687E0  2C 18 06 00 */	cmpwi r24, 0x600
/* 8036B884 003687E4  40 80 00 1C */	bge lbl_8036B8A0
/* 8036B888 003687E8  A8 7C 00 0A */	lha r3, 0xa(r28)
/* 8036B88C 003687EC  20 03 02 00 */	subfic r0, r3, 0x200
/* 8036B890 003687F0  7C 00 26 70 */	srawi r0, r0, 4
/* 8036B894 003687F4  7C 03 02 14 */	add r0, r3, r0
/* 8036B898 003687F8  B0 1C 00 0A */	sth r0, 0xa(r28)
/* 8036B89C 003687FC  48 00 00 78 */	b lbl_8036B914
lbl_8036B8A0:
/* 8036B8A0 00368800  88 1C 00 30 */	lbz r0, 0x30(r28)
/* 8036B8A4 00368804  2C 00 00 01 */	cmpwi r0, 1
/* 8036B8A8 00368808  40 82 00 1C */	bne lbl_8036B8C4
/* 8036B8AC 0036880C  A8 7C 00 0A */	lha r3, 0xa(r28)
/* 8036B8B0 00368810  20 03 02 00 */	subfic r0, r3, 0x200
/* 8036B8B4 00368814  7C 00 26 70 */	srawi r0, r0, 4
/* 8036B8B8 00368818  7C 03 02 14 */	add r0, r3, r0
/* 8036B8BC 0036881C  B0 1C 00 0A */	sth r0, 0xa(r28)
/* 8036B8C0 00368820  48 00 00 54 */	b lbl_8036B914
lbl_8036B8C4:
/* 8036B8C4 00368824  A8 1C 00 06 */	lha r0, 6(r28)
/* 8036B8C8 00368828  A8 7C 00 08 */	lha r3, 8(r28)
/* 8036B8CC 0036882C  54 00 10 3A */	slwi r0, r0, 2
/* 8036B8D0 00368830  7C 63 00 50 */	subf r3, r3, r0
/* 8036B8D4 00368834  48 02 15 49 */	bl abs
/* 8036B8D8 00368838  A8 1C 00 08 */	lha r0, 8(r28)
/* 8036B8DC 0036883C  7C 00 1E 70 */	srawi r0, r0, 3
/* 8036B8E0 00368840  7C 03 00 00 */	cmpw r3, r0
/* 8036B8E4 00368844  41 80 00 1C */	blt lbl_8036B900
/* 8036B8E8 00368848  A8 7C 00 0A */	lha r3, 0xa(r28)
/* 8036B8EC 0036884C  20 03 02 00 */	subfic r0, r3, 0x200
/* 8036B8F0 00368850  7C 00 26 70 */	srawi r0, r0, 4
/* 8036B8F4 00368854  7C 03 02 14 */	add r0, r3, r0
/* 8036B8F8 00368858  B0 1C 00 0A */	sth r0, 0xa(r28)
/* 8036B8FC 0036885C  48 00 00 18 */	b lbl_8036B914
lbl_8036B900:
/* 8036B900 00368860  A8 7C 00 0A */	lha r3, 0xa(r28)
/* 8036B904 00368864  7C 03 00 D0 */	neg r0, r3
/* 8036B908 00368868  7C 00 26 70 */	srawi r0, r0, 4
/* 8036B90C 0036886C  7C 03 02 14 */	add r0, r3, r0
/* 8036B910 00368870  B0 1C 00 0A */	sth r0, 0xa(r28)
lbl_8036B914:
/* 8036B914 00368874  BA E1 00 0C */	lmw r23, 0xc(r1)
/* 8036B918 00368878  80 01 00 34 */	lwz r0, 0x34(r1)
/* 8036B91C 0036887C  7C 08 03 A6 */	mtlr r0
/* 8036B920 00368880  38 21 00 30 */	addi r1, r1, 0x30
/* 8036B924 00368884  4E 80 00 20 */	blr 

.global reconstruct__Fiii
reconstruct__Fiii:
/* 8036B928 00368888  7C A0 16 70 */	srawi r0, r5, 2
/* 8036B92C 0036888C  7C 04 02 14 */	add r0, r4, r0
/* 8036B930 00368890  7C 00 07 35 */	extsh. r0, r0
/* 8036B934 00368894  40 80 00 1C */	bge lbl_8036B950
/* 8036B938 00368898  7C 83 00 D0 */	neg r4, r3
/* 8036B93C 0036889C  38 00 80 00 */	li r0, -32768
/* 8036B940 003688A0  7C 83 1B 78 */	or r3, r4, r3
/* 8036B944 003688A4  7C 63 FE 70 */	srawi r3, r3, 0x1f
/* 8036B948 003688A8  7C 03 18 38 */	and r3, r0, r3
/* 8036B94C 003688AC  4E 80 00 20 */	blr 
lbl_8036B950:
/* 8036B950 003688B0  54 04 06 7E */	clrlwi r4, r0, 0x19
/* 8036B954 003688B4  54 00 CF 3E */	rlwinm r0, r0, 0x19, 0x1c, 0x1f
/* 8036B958 003688B8  38 84 00 80 */	addi r4, r4, 0x80
/* 8036B95C 003688BC  2C 03 00 00 */	cmpwi r3, 0
/* 8036B960 003688C0  7C 83 07 34 */	extsh r3, r4
/* 8036B964 003688C4  7C 00 07 34 */	extsh r0, r0
/* 8036B968 003688C8  54 63 38 30 */	slwi r3, r3, 7
/* 8036B96C 003688CC  20 00 00 0E */	subfic r0, r0, 0xe
/* 8036B970 003688D0  7C 60 06 30 */	sraw r0, r3, r0
/* 8036B974 003688D4  7C 03 07 34 */	extsh r3, r0
/* 8036B978 003688D8  4D 82 00 20 */	beqlr 
/* 8036B97C 003688DC  38 63 80 00 */	addi r3, r3, -32768
/* 8036B980 003688E0  4E 80 00 20 */	blr 

.global step_size__FP10g72x_state
step_size__FP10g72x_state:
/* 8036B984 003688E4  A8 A3 00 0A */	lha r5, 0xa(r3)
/* 8036B988 003688E8  2C 05 01 00 */	cmpwi r5, 0x100
/* 8036B98C 003688EC  41 80 00 0C */	blt lbl_8036B998
/* 8036B990 003688F0  A8 63 00 04 */	lha r3, 4(r3)
/* 8036B994 003688F4  4E 80 00 20 */	blr 
lbl_8036B998:
/* 8036B998 003688F8  80 83 00 00 */	lwz r4, 0(r3)
/* 8036B99C 003688FC  A8 03 00 04 */	lha r0, 4(r3)
/* 8036B9A0 00368900  7C 83 36 70 */	srawi r3, r4, 6
/* 8036B9A4 00368904  7C 03 00 51 */	subf. r0, r3, r0
/* 8036B9A8 00368908  7C A4 16 70 */	srawi r4, r5, 2
/* 8036B9AC 0036890C  40 81 00 14 */	ble lbl_8036B9C0
/* 8036B9B0 00368910  7C 00 21 D6 */	mullw r0, r0, r4
/* 8036B9B4 00368914  7C 00 36 70 */	srawi r0, r0, 6
/* 8036B9B8 00368918  7C 63 02 14 */	add r3, r3, r0
/* 8036B9BC 0036891C  4E 80 00 20 */	blr 
lbl_8036B9C0:
/* 8036B9C0 00368920  4C 80 00 20 */	bgelr 
/* 8036B9C4 00368924  7C 80 21 D6 */	mullw r4, r0, r4
/* 8036B9C8 00368928  38 04 00 3F */	addi r0, r4, 0x3f
/* 8036B9CC 0036892C  7C 00 36 70 */	srawi r0, r0, 6
/* 8036B9D0 00368930  7C 63 02 14 */	add r3, r3, r0
/* 8036B9D4 00368934  4E 80 00 20 */	blr 

.global predictor_pole__FP10g72x_state
predictor_pole__FP10g72x_state:
/* 8036B9D8 00368938  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8036B9DC 0036893C  7C 08 02 A6 */	mflr r0
/* 8036B9E0 00368940  90 01 00 14 */	stw r0, 0x14(r1)
/* 8036B9E4 00368944  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8036B9E8 00368948  93 C1 00 08 */	stw r30, 8(r1)
/* 8036B9EC 0036894C  7C 7E 1B 78 */	mr r30, r3
/* 8036B9F0 00368950  A8 03 00 0C */	lha r0, 0xc(r3)
/* 8036B9F4 00368954  A8 83 00 2C */	lha r4, 0x2c(r3)
/* 8036B9F8 00368958  7C 03 16 70 */	srawi r3, r0, 2
/* 8036B9FC 0036895C  48 00 01 29 */	bl fmult__Fii
/* 8036BA00 00368960  A8 1E 00 0E */	lha r0, 0xe(r30)
/* 8036BA04 00368964  7C 7F 1B 78 */	mr r31, r3
/* 8036BA08 00368968  A8 9E 00 2E */	lha r4, 0x2e(r30)
/* 8036BA0C 0036896C  7C 03 16 70 */	srawi r3, r0, 2
/* 8036BA10 00368970  48 00 01 15 */	bl fmult__Fii
/* 8036BA14 00368974  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8036BA18 00368978  7C 63 FA 14 */	add r3, r3, r31
/* 8036BA1C 0036897C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8036BA20 00368980  83 C1 00 08 */	lwz r30, 8(r1)
/* 8036BA24 00368984  7C 08 03 A6 */	mtlr r0
/* 8036BA28 00368988  38 21 00 10 */	addi r1, r1, 0x10
/* 8036BA2C 0036898C  4E 80 00 20 */	blr 

.global predictor_zero__FP10g72x_state
predictor_zero__FP10g72x_state:
/* 8036BA30 00368990  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8036BA34 00368994  7C 08 02 A6 */	mflr r0
/* 8036BA38 00368998  90 01 00 24 */	stw r0, 0x24(r1)
/* 8036BA3C 0036899C  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8036BA40 003689A0  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8036BA44 003689A4  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8036BA48 003689A8  7C 7D 1B 78 */	mr r29, r3
/* 8036BA4C 003689AC  A8 03 00 10 */	lha r0, 0x10(r3)
/* 8036BA50 003689B0  A8 83 00 20 */	lha r4, 0x20(r3)
/* 8036BA54 003689B4  7C 03 16 70 */	srawi r3, r0, 2
/* 8036BA58 003689B8  48 00 00 CD */	bl fmult__Fii
/* 8036BA5C 003689BC  3B FD 00 02 */	addi r31, r29, 2
/* 8036BA60 003689C0  7C 7D 1B 78 */	mr r29, r3
/* 8036BA64 003689C4  3B C0 00 01 */	li r30, 1
lbl_8036BA68:
/* 8036BA68 003689C8  A8 1F 00 10 */	lha r0, 0x10(r31)
/* 8036BA6C 003689CC  A8 9F 00 20 */	lha r4, 0x20(r31)
/* 8036BA70 003689D0  7C 03 16 70 */	srawi r3, r0, 2
/* 8036BA74 003689D4  48 00 00 B1 */	bl fmult__Fii
/* 8036BA78 003689D8  3B DE 00 01 */	addi r30, r30, 1
/* 8036BA7C 003689DC  7F BD 1A 14 */	add r29, r29, r3
/* 8036BA80 003689E0  2C 1E 00 06 */	cmpwi r30, 6
/* 8036BA84 003689E4  3B FF 00 02 */	addi r31, r31, 2
/* 8036BA88 003689E8  41 80 FF E0 */	blt lbl_8036BA68
/* 8036BA8C 003689EC  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8036BA90 003689F0  7F A3 EB 78 */	mr r3, r29
/* 8036BA94 003689F4  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8036BA98 003689F8  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8036BA9C 003689FC  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8036BAA0 00368A00  7C 08 03 A6 */	mtlr r0
/* 8036BAA4 00368A04  38 21 00 20 */	addi r1, r1, 0x20
/* 8036BAA8 00368A08  4E 80 00 20 */	blr 

.global g72x_init_state__FP10g72x_state
g72x_init_state__FP10g72x_state:
/* 8036BAAC 00368A0C  3C 80 00 01 */	lis r4, 0x00008800@ha
/* 8036BAB0 00368A10  38 A0 02 20 */	li r5, 0x220
/* 8036BAB4 00368A14  38 04 88 00 */	addi r0, r4, 0x00008800@l
/* 8036BAB8 00368A18  38 80 00 00 */	li r4, 0
/* 8036BABC 00368A1C  90 03 00 00 */	stw r0, 0(r3)
/* 8036BAC0 00368A20  38 00 00 20 */	li r0, 0x20
/* 8036BAC4 00368A24  B0 A3 00 04 */	sth r5, 4(r3)
/* 8036BAC8 00368A28  B0 83 00 06 */	sth r4, 6(r3)
/* 8036BACC 00368A2C  B0 83 00 08 */	sth r4, 8(r3)
/* 8036BAD0 00368A30  B0 83 00 0A */	sth r4, 0xa(r3)
/* 8036BAD4 00368A34  B0 83 00 0C */	sth r4, 0xc(r3)
/* 8036BAD8 00368A38  B0 83 00 1C */	sth r4, 0x1c(r3)
/* 8036BADC 00368A3C  B0 03 00 2C */	sth r0, 0x2c(r3)
/* 8036BAE0 00368A40  B0 83 00 0E */	sth r4, 0xe(r3)
/* 8036BAE4 00368A44  B0 83 00 1E */	sth r4, 0x1e(r3)
/* 8036BAE8 00368A48  B0 03 00 2E */	sth r0, 0x2e(r3)
/* 8036BAEC 00368A4C  B0 83 00 10 */	sth r4, 0x10(r3)
/* 8036BAF0 00368A50  B0 03 00 20 */	sth r0, 0x20(r3)
/* 8036BAF4 00368A54  B0 83 00 12 */	sth r4, 0x12(r3)
/* 8036BAF8 00368A58  B0 03 00 22 */	sth r0, 0x22(r3)
/* 8036BAFC 00368A5C  B0 83 00 14 */	sth r4, 0x14(r3)
/* 8036BB00 00368A60  B0 03 00 24 */	sth r0, 0x24(r3)
/* 8036BB04 00368A64  B0 83 00 16 */	sth r4, 0x16(r3)
/* 8036BB08 00368A68  B0 03 00 26 */	sth r0, 0x26(r3)
/* 8036BB0C 00368A6C  B0 83 00 18 */	sth r4, 0x18(r3)
/* 8036BB10 00368A70  B0 03 00 28 */	sth r0, 0x28(r3)
/* 8036BB14 00368A74  B0 83 00 1A */	sth r4, 0x1a(r3)
/* 8036BB18 00368A78  B0 03 00 2A */	sth r0, 0x2a(r3)
/* 8036BB1C 00368A7C  98 83 00 30 */	stb r4, 0x30(r3)
/* 8036BB20 00368A80  4E 80 00 20 */	blr 

.global fmult__Fii
fmult__Fii:
/* 8036BB24 00368A84  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8036BB28 00368A88  7C 08 02 A6 */	mflr r0
/* 8036BB2C 00368A8C  90 01 00 24 */	stw r0, 0x24(r1)
/* 8036BB30 00368A90  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8036BB34 00368A94  7C 9F 23 78 */	mr r31, r4
/* 8036BB38 00368A98  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8036BB3C 00368A9C  7C 7E 1B 79 */	or. r30, r3, r3
/* 8036BB40 00368AA0  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8036BB44 00368AA4  7C 1E 00 D0 */	neg r0, r30
/* 8036BB48 00368AA8  54 00 04 FE */	clrlwi r0, r0, 0x13
/* 8036BB4C 00368AAC  40 81 00 08 */	ble lbl_8036BB54
/* 8036BB50 00368AB0  7F C0 F3 78 */	mr r0, r30
lbl_8036BB54:
/* 8036BB54 00368AB4  3C 60 80 3F */	lis r3, lbl_803EF840@ha
/* 8036BB58 00368AB8  7C 1D 07 34 */	extsh r29, r0
/* 8036BB5C 00368ABC  38 83 F8 40 */	addi r4, r3, lbl_803EF840@l
/* 8036BB60 00368AC0  38 A0 00 0F */	li r5, 0xf
/* 8036BB64 00368AC4  7F A3 EB 78 */	mr r3, r29
/* 8036BB68 00368AC8  48 00 00 9D */	bl quan__FiPsi
/* 8036BB6C 00368ACC  7F A0 07 35 */	extsh. r0, r29
/* 8036BB70 00368AD0  38 03 FF FA */	addi r0, r3, -6
/* 8036BB74 00368AD4  7C 05 07 34 */	extsh r5, r0
/* 8036BB78 00368AD8  40 82 00 0C */	bne lbl_8036BB84
/* 8036BB7C 00368ADC  38 00 00 20 */	li r0, 0x20
/* 8036BB80 00368AE0  48 00 00 18 */	b lbl_8036BB98
lbl_8036BB84:
/* 8036BB84 00368AE4  7C A0 07 35 */	extsh. r0, r5
/* 8036BB88 00368AE8  7C 05 00 D0 */	neg r0, r5
/* 8036BB8C 00368AEC  7F A0 00 30 */	slw r0, r29, r0
/* 8036BB90 00368AF0  41 80 00 08 */	blt lbl_8036BB98
/* 8036BB94 00368AF4  7F A0 2E 30 */	sraw r0, r29, r5
lbl_8036BB98:
/* 8036BB98 00368AF8  7C 03 07 34 */	extsh r3, r0
/* 8036BB9C 00368AFC  57 E0 06 BE */	clrlwi r0, r31, 0x1a
/* 8036BBA0 00368B00  7C 60 19 D6 */	mullw r3, r0, r3
/* 8036BBA4 00368B04  57 E4 D7 3E */	rlwinm r4, r31, 0x1a, 0x1c, 0x1f
/* 8036BBA8 00368B08  38 04 FF F3 */	addi r0, r4, -13
/* 8036BBAC 00368B0C  7C 05 02 14 */	add r0, r5, r0
/* 8036BBB0 00368B10  7C 04 07 35 */	extsh. r4, r0
/* 8036BBB4 00368B14  38 03 00 30 */	addi r0, r3, 0x30
/* 8036BBB8 00368B18  7C 00 26 70 */	srawi r0, r0, 4
/* 8036BBBC 00368B1C  7C 05 07 34 */	extsh r5, r0
/* 8036BBC0 00368B20  7C 04 00 D0 */	neg r0, r4
/* 8036BBC4 00368B24  7C A3 06 30 */	sraw r3, r5, r0
/* 8036BBC8 00368B28  41 80 00 0C */	blt lbl_8036BBD4
/* 8036BBCC 00368B2C  7C A0 20 30 */	slw r0, r5, r4
/* 8036BBD0 00368B30  54 03 04 7E */	clrlwi r3, r0, 0x11
lbl_8036BBD4:
/* 8036BBD4 00368B34  7F C0 FA 79 */	xor. r0, r30, r31
/* 8036BBD8 00368B38  7C 60 07 34 */	extsh r0, r3
/* 8036BBDC 00368B3C  7C 03 03 78 */	mr r3, r0
/* 8036BBE0 00368B40  40 80 00 08 */	bge lbl_8036BBE8
/* 8036BBE4 00368B44  7C 60 00 D0 */	neg r3, r0
lbl_8036BBE8:
/* 8036BBE8 00368B48  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8036BBEC 00368B4C  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8036BBF0 00368B50  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8036BBF4 00368B54  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8036BBF8 00368B58  7C 08 03 A6 */	mtlr r0
/* 8036BBFC 00368B5C  38 21 00 20 */	addi r1, r1, 0x20
/* 8036BC00 00368B60  4E 80 00 20 */	blr 

.global quan__FiPsi
quan__FiPsi:
/* 8036BC04 00368B64  38 C0 00 00 */	li r6, 0
/* 8036BC08 00368B68  7C A9 03 A6 */	mtctr r5
/* 8036BC0C 00368B6C  2C 05 00 00 */	cmpwi r5, 0
/* 8036BC10 00368B70  40 81 00 1C */	ble lbl_8036BC2C
lbl_8036BC14:
/* 8036BC14 00368B74  A8 04 00 00 */	lha r0, 0(r4)
/* 8036BC18 00368B78  38 84 00 02 */	addi r4, r4, 2
/* 8036BC1C 00368B7C  7C 03 00 00 */	cmpw r3, r0
/* 8036BC20 00368B80  41 80 00 0C */	blt lbl_8036BC2C
/* 8036BC24 00368B84  38 C6 00 01 */	addi r6, r6, 1
/* 8036BC28 00368B88  42 00 FF EC */	bdnz lbl_8036BC14
lbl_8036BC2C:
/* 8036BC2C 00368B8C  7C C3 33 78 */	mr r3, r6
/* 8036BC30 00368B90  4E 80 00 20 */	blr
