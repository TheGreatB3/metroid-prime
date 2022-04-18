.include "macros.inc"

.section .data

.global lbl_803EE490
lbl_803EE490:
	# ROM: 0x3EB490
	.4byte lbl_803433E4
	.4byte lbl_803434A8
	.4byte lbl_80343560
	.4byte lbl_803435B4
	.4byte lbl_80343610
	.4byte lbl_8034366C
	.4byte lbl_803436D4
	.4byte lbl_80343700
	.4byte lbl_80343790
	.4byte lbl_803437E4
	.4byte lbl_80343840
	.4byte lbl_8034389C
	.4byte lbl_8034392C
	.4byte lbl_80343934

.section .text, "ax"  # 0x80003640 - 0x803CB1C0

.global inflate
inflate:
/* 8034335C 003402BC  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80343360 003402C0  7C 08 02 A6 */	mflr r0
/* 80343364 003402C4  90 01 00 24 */	stw r0, 0x24(r1)
/* 80343368 003402C8  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8034336C 003402CC  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80343370 003402D0  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80343374 003402D4  93 81 00 10 */	stw r28, 0x10(r1)
/* 80343378 003402D8  7C 7C 1B 79 */	or. r28, r3, r3
/* 8034337C 003402DC  41 82 00 1C */	beq lbl_80343398
/* 80343380 003402E0  80 1C 00 1C */	lwz r0, 0x1c(r28)
/* 80343384 003402E4  28 00 00 00 */	cmplwi r0, 0
/* 80343388 003402E8  41 82 00 10 */	beq lbl_80343398
/* 8034338C 003402EC  80 1C 00 00 */	lwz r0, 0(r28)
/* 80343390 003402F0  28 00 00 00 */	cmplwi r0, 0
/* 80343394 003402F4  40 82 00 0C */	bne lbl_803433A0
lbl_80343398:
/* 80343398 003402F8  38 60 FF FE */	li r3, -2
/* 8034339C 003402FC  48 00 05 A4 */	b lbl_80343940
lbl_803433A0:
/* 803433A0 00340300  38 64 FF FC */	addi r3, r4, -4
/* 803433A4 00340304  20 04 00 04 */	subfic r0, r4, 4
/* 803433A8 00340308  7C 60 00 F8 */	nor r0, r3, r0
/* 803433AC 0034030C  3C 60 80 3F */	lis r3, lbl_803EE490@ha
/* 803433B0 00340310  7C 04 FE 70 */	srawi r4, r0, 0x1f
/* 803433B4 00340314  38 00 FF FB */	li r0, -5
/* 803433B8 00340318  7C 1E 20 38 */	and r30, r0, r4
/* 803433BC 0034031C  3B E3 E4 90 */	addi r31, r3, lbl_803EE490@l
/* 803433C0 00340320  3B A0 FF FB */	li r29, -5
lbl_803433C4:
/* 803433C4 00340324  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803433C8 00340328  80 04 00 00 */	lwz r0, 0(r4)
/* 803433CC 0034032C  28 00 00 0D */	cmplwi r0, 0xd
/* 803433D0 00340330  41 81 05 6C */	bgt lbl_8034393C
/* 803433D4 00340334  54 00 10 3A */	slwi r0, r0, 2
/* 803433D8 00340338  7C 1F 00 2E */	lwzx r0, r31, r0
/* 803433DC 0034033C  7C 09 03 A6 */	mtctr r0
/* 803433E0 00340340  4E 80 04 20 */	bctr 
.global lbl_803433E4
lbl_803433E4:
/* 803433E4 00340344  80 7C 00 04 */	lwz r3, 4(r28)
/* 803433E8 00340348  28 03 00 00 */	cmplwi r3, 0
/* 803433EC 0034034C  40 82 00 0C */	bne lbl_803433F8
/* 803433F0 00340350  7F A3 EB 78 */	mr r3, r29
/* 803433F4 00340354  48 00 05 4C */	b lbl_80343940
lbl_803433F8:
/* 803433F8 00340358  38 03 FF FF */	addi r0, r3, -1
/* 803433FC 0034035C  7F DD F3 78 */	mr r29, r30
/* 80343400 00340360  90 1C 00 04 */	stw r0, 4(r28)
/* 80343404 00340364  80 7C 00 08 */	lwz r3, 8(r28)
/* 80343408 00340368  38 03 00 01 */	addi r0, r3, 1
/* 8034340C 0034036C  90 1C 00 08 */	stw r0, 8(r28)
/* 80343410 00340370  80 7C 00 00 */	lwz r3, 0(r28)
/* 80343414 00340374  38 03 00 01 */	addi r0, r3, 1
/* 80343418 00340378  90 1C 00 00 */	stw r0, 0(r28)
/* 8034341C 0034037C  88 83 00 00 */	lbz r4, 0(r3)
/* 80343420 00340380  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343424 00340384  54 80 07 3E */	clrlwi r0, r4, 0x1c
/* 80343428 00340388  28 00 00 08 */	cmplwi r0, 8
/* 8034342C 0034038C  90 83 00 04 */	stw r4, 4(r3)
/* 80343430 00340390  41 82 00 2C */	beq lbl_8034345C
/* 80343434 00340394  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 80343438 00340398  38 A0 00 0D */	li r5, 0xd
/* 8034343C 0034039C  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 80343440 003403A0  38 00 00 05 */	li r0, 5
/* 80343444 003403A4  90 A4 00 00 */	stw r5, 0(r4)
/* 80343448 003403A8  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 8034344C 003403AC  90 7C 00 18 */	stw r3, 0x18(r28)
/* 80343450 003403B0  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343454 003403B4  90 03 00 04 */	stw r0, 4(r3)
/* 80343458 003403B8  4B FF FF 6C */	b lbl_803433C4
lbl_8034345C:
/* 8034345C 003403BC  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 80343460 003403C0  80 64 00 04 */	lwz r3, 4(r4)
/* 80343464 003403C4  80 04 00 10 */	lwz r0, 0x10(r4)
/* 80343468 003403C8  54 63 E1 3E */	srwi r3, r3, 4
/* 8034346C 003403CC  38 63 00 08 */	addi r3, r3, 8
/* 80343470 003403D0  7C 03 00 40 */	cmplw r3, r0
/* 80343474 003403D4  40 81 00 2C */	ble lbl_803434A0
/* 80343478 003403D8  38 00 00 0D */	li r0, 0xd
/* 8034347C 003403DC  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 80343480 003403E0  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 80343484 003403E4  90 04 00 00 */	stw r0, 0(r4)
/* 80343488 003403E8  38 63 00 1B */	addi r3, r3, 0x1b
/* 8034348C 003403EC  38 00 00 05 */	li r0, 5
/* 80343490 003403F0  90 7C 00 18 */	stw r3, 0x18(r28)
/* 80343494 003403F4  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343498 003403F8  90 03 00 04 */	stw r0, 4(r3)
/* 8034349C 003403FC  4B FF FF 28 */	b lbl_803433C4
lbl_803434A0:
/* 803434A0 00340400  38 00 00 01 */	li r0, 1
/* 803434A4 00340404  90 04 00 00 */	stw r0, 0(r4)
.global lbl_803434A8
lbl_803434A8:
/* 803434A8 00340408  80 7C 00 04 */	lwz r3, 4(r28)
/* 803434AC 0034040C  28 03 00 00 */	cmplwi r3, 0
/* 803434B0 00340410  40 82 00 0C */	bne lbl_803434BC
/* 803434B4 00340414  7F A3 EB 78 */	mr r3, r29
/* 803434B8 00340418  48 00 04 88 */	b lbl_80343940
lbl_803434BC:
/* 803434BC 0034041C  38 03 FF FF */	addi r0, r3, -1
/* 803434C0 00340420  3C 60 08 42 */	lis r3, 0x08421085@ha
/* 803434C4 00340424  90 1C 00 04 */	stw r0, 4(r28)
/* 803434C8 00340428  38 03 10 85 */	addi r0, r3, 0x08421085@l
/* 803434CC 0034042C  7F DD F3 78 */	mr r29, r30
/* 803434D0 00340430  80 7C 00 08 */	lwz r3, 8(r28)
/* 803434D4 00340434  38 63 00 01 */	addi r3, r3, 1
/* 803434D8 00340438  90 7C 00 08 */	stw r3, 8(r28)
/* 803434DC 0034043C  80 9C 00 00 */	lwz r4, 0(r28)
/* 803434E0 00340440  38 64 00 01 */	addi r3, r4, 1
/* 803434E4 00340444  90 7C 00 00 */	stw r3, 0(r28)
/* 803434E8 00340448  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 803434EC 0034044C  88 C4 00 00 */	lbz r6, 0(r4)
/* 803434F0 00340450  80 65 00 04 */	lwz r3, 4(r5)
/* 803434F4 00340454  54 63 40 2E */	slwi r3, r3, 8
/* 803434F8 00340458  7C 83 32 14 */	add r4, r3, r6
/* 803434FC 0034045C  7C 60 20 16 */	mulhwu r3, r0, r4
/* 80343500 00340460  7C 03 20 50 */	subf r0, r3, r4
/* 80343504 00340464  54 00 F8 7E */	srwi r0, r0, 1
/* 80343508 00340468  7C 00 1A 14 */	add r0, r0, r3
/* 8034350C 0034046C  54 00 E1 3E */	srwi r0, r0, 4
/* 80343510 00340470  1C 00 00 1F */	mulli r0, r0, 0x1f
/* 80343514 00340474  7C 00 20 51 */	subf. r0, r0, r4
/* 80343518 00340478  41 82 00 2C */	beq lbl_80343544
/* 8034351C 0034047C  38 00 00 0D */	li r0, 0xd
/* 80343520 00340480  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 80343524 00340484  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 80343528 00340488  90 05 00 00 */	stw r0, 0(r5)
/* 8034352C 0034048C  38 63 00 2F */	addi r3, r3, 0x2f
/* 80343530 00340490  38 00 00 05 */	li r0, 5
/* 80343534 00340494  90 7C 00 18 */	stw r3, 0x18(r28)
/* 80343538 00340498  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 8034353C 0034049C  90 03 00 04 */	stw r0, 4(r3)
/* 80343540 003404A0  4B FF FE 84 */	b lbl_803433C4
lbl_80343544:
/* 80343544 003404A4  54 C0 06 B5 */	rlwinm. r0, r6, 0, 0x1a, 0x1a
/* 80343548 003404A8  40 82 00 10 */	bne lbl_80343558
/* 8034354C 003404AC  38 00 00 07 */	li r0, 7
/* 80343550 003404B0  90 05 00 00 */	stw r0, 0(r5)
/* 80343554 003404B4  4B FF FE 70 */	b lbl_803433C4
lbl_80343558:
/* 80343558 003404B8  38 00 00 02 */	li r0, 2
/* 8034355C 003404BC  90 05 00 00 */	stw r0, 0(r5)
.global lbl_80343560
lbl_80343560:
/* 80343560 003404C0  80 7C 00 04 */	lwz r3, 4(r28)
/* 80343564 003404C4  28 03 00 00 */	cmplwi r3, 0
/* 80343568 003404C8  40 82 00 0C */	bne lbl_80343574
/* 8034356C 003404CC  7F A3 EB 78 */	mr r3, r29
/* 80343570 003404D0  48 00 03 D0 */	b lbl_80343940
lbl_80343574:
/* 80343574 003404D4  38 63 FF FF */	addi r3, r3, -1
/* 80343578 003404D8  38 00 00 03 */	li r0, 3
/* 8034357C 003404DC  90 7C 00 04 */	stw r3, 4(r28)
/* 80343580 003404E0  7F DD F3 78 */	mr r29, r30
/* 80343584 003404E4  80 7C 00 08 */	lwz r3, 8(r28)
/* 80343588 003404E8  38 63 00 01 */	addi r3, r3, 1
/* 8034358C 003404EC  90 7C 00 08 */	stw r3, 8(r28)
/* 80343590 003404F0  80 9C 00 00 */	lwz r4, 0(r28)
/* 80343594 003404F4  38 64 00 01 */	addi r3, r4, 1
/* 80343598 003404F8  90 7C 00 00 */	stw r3, 0(r28)
/* 8034359C 003404FC  88 84 00 00 */	lbz r4, 0(r4)
/* 803435A0 00340500  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 803435A4 00340504  54 84 C0 0E */	slwi r4, r4, 0x18
/* 803435A8 00340508  90 83 00 08 */	stw r4, 8(r3)
/* 803435AC 0034050C  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 803435B0 00340510  90 03 00 00 */	stw r0, 0(r3)
.global lbl_803435B4
lbl_803435B4:
/* 803435B4 00340514  80 7C 00 04 */	lwz r3, 4(r28)
/* 803435B8 00340518  28 03 00 00 */	cmplwi r3, 0
/* 803435BC 0034051C  40 82 00 0C */	bne lbl_803435C8
/* 803435C0 00340520  7F A3 EB 78 */	mr r3, r29
/* 803435C4 00340524  48 00 03 7C */	b lbl_80343940
lbl_803435C8:
/* 803435C8 00340528  38 63 FF FF */	addi r3, r3, -1
/* 803435CC 0034052C  38 00 00 04 */	li r0, 4
/* 803435D0 00340530  90 7C 00 04 */	stw r3, 4(r28)
/* 803435D4 00340534  7F DD F3 78 */	mr r29, r30
/* 803435D8 00340538  80 7C 00 08 */	lwz r3, 8(r28)
/* 803435DC 0034053C  38 63 00 01 */	addi r3, r3, 1
/* 803435E0 00340540  90 7C 00 08 */	stw r3, 8(r28)
/* 803435E4 00340544  80 9C 00 00 */	lwz r4, 0(r28)
/* 803435E8 00340548  38 64 00 01 */	addi r3, r4, 1
/* 803435EC 0034054C  90 7C 00 00 */	stw r3, 0(r28)
/* 803435F0 00340550  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 803435F4 00340554  88 64 00 00 */	lbz r3, 0(r4)
/* 803435F8 00340558  80 85 00 08 */	lwz r4, 8(r5)
/* 803435FC 0034055C  54 63 80 1E */	slwi r3, r3, 0x10
/* 80343600 00340560  7C 64 1A 14 */	add r3, r4, r3
/* 80343604 00340564  90 65 00 08 */	stw r3, 8(r5)
/* 80343608 00340568  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 8034360C 0034056C  90 03 00 00 */	stw r0, 0(r3)
.global lbl_80343610
lbl_80343610:
/* 80343610 00340570  80 7C 00 04 */	lwz r3, 4(r28)
/* 80343614 00340574  28 03 00 00 */	cmplwi r3, 0
/* 80343618 00340578  40 82 00 0C */	bne lbl_80343624
/* 8034361C 0034057C  7F A3 EB 78 */	mr r3, r29
/* 80343620 00340580  48 00 03 20 */	b lbl_80343940
lbl_80343624:
/* 80343624 00340584  38 63 FF FF */	addi r3, r3, -1
/* 80343628 00340588  38 00 00 05 */	li r0, 5
/* 8034362C 0034058C  90 7C 00 04 */	stw r3, 4(r28)
/* 80343630 00340590  7F DD F3 78 */	mr r29, r30
/* 80343634 00340594  80 7C 00 08 */	lwz r3, 8(r28)
/* 80343638 00340598  38 63 00 01 */	addi r3, r3, 1
/* 8034363C 0034059C  90 7C 00 08 */	stw r3, 8(r28)
/* 80343640 003405A0  80 9C 00 00 */	lwz r4, 0(r28)
/* 80343644 003405A4  38 64 00 01 */	addi r3, r4, 1
/* 80343648 003405A8  90 7C 00 00 */	stw r3, 0(r28)
/* 8034364C 003405AC  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 80343650 003405B0  88 64 00 00 */	lbz r3, 0(r4)
/* 80343654 003405B4  80 85 00 08 */	lwz r4, 8(r5)
/* 80343658 003405B8  54 63 40 2E */	slwi r3, r3, 8
/* 8034365C 003405BC  7C 64 1A 14 */	add r3, r4, r3
/* 80343660 003405C0  90 65 00 08 */	stw r3, 8(r5)
/* 80343664 003405C4  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343668 003405C8  90 03 00 00 */	stw r0, 0(r3)
.global lbl_8034366C
lbl_8034366C:
/* 8034366C 003405CC  80 7C 00 04 */	lwz r3, 4(r28)
/* 80343670 003405D0  28 03 00 00 */	cmplwi r3, 0
/* 80343674 003405D4  40 82 00 0C */	bne lbl_80343680
/* 80343678 003405D8  7F A3 EB 78 */	mr r3, r29
/* 8034367C 003405DC  48 00 02 C4 */	b lbl_80343940
lbl_80343680:
/* 80343680 003405E0  38 63 FF FF */	addi r3, r3, -1
/* 80343684 003405E4  38 00 00 06 */	li r0, 6
/* 80343688 003405E8  90 7C 00 04 */	stw r3, 4(r28)
/* 8034368C 003405EC  38 60 00 02 */	li r3, 2
/* 80343690 003405F0  80 9C 00 08 */	lwz r4, 8(r28)
/* 80343694 003405F4  38 84 00 01 */	addi r4, r4, 1
/* 80343698 003405F8  90 9C 00 08 */	stw r4, 8(r28)
/* 8034369C 003405FC  80 BC 00 00 */	lwz r5, 0(r28)
/* 803436A0 00340600  38 85 00 01 */	addi r4, r5, 1
/* 803436A4 00340604  90 9C 00 00 */	stw r4, 0(r28)
/* 803436A8 00340608  80 DC 00 1C */	lwz r6, 0x1c(r28)
/* 803436AC 0034060C  88 85 00 00 */	lbz r4, 0(r5)
/* 803436B0 00340610  80 A6 00 08 */	lwz r5, 8(r6)
/* 803436B4 00340614  7C 85 22 14 */	add r4, r5, r4
/* 803436B8 00340618  90 86 00 08 */	stw r4, 8(r6)
/* 803436BC 0034061C  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803436C0 00340620  80 84 00 08 */	lwz r4, 8(r4)
/* 803436C4 00340624  90 9C 00 30 */	stw r4, 0x30(r28)
/* 803436C8 00340628  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803436CC 0034062C  90 04 00 00 */	stw r0, 0(r4)
/* 803436D0 00340630  48 00 02 70 */	b lbl_80343940
.global lbl_803436D4
lbl_803436D4:
/* 803436D4 00340634  38 00 00 0D */	li r0, 0xd
/* 803436D8 00340638  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 803436DC 0034063C  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 803436E0 00340640  90 04 00 00 */	stw r0, 0(r4)
/* 803436E4 00340644  38 63 00 46 */	addi r3, r3, 0x46
/* 803436E8 00340648  38 00 00 00 */	li r0, 0
/* 803436EC 0034064C  90 7C 00 18 */	stw r3, 0x18(r28)
/* 803436F0 00340650  38 60 FF FE */	li r3, -2
/* 803436F4 00340654  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803436F8 00340658  90 04 00 04 */	stw r0, 4(r4)
/* 803436FC 0034065C  48 00 02 44 */	b lbl_80343940
.global lbl_80343700
lbl_80343700:
/* 80343700 00340660  80 64 00 14 */	lwz r3, 0x14(r4)
/* 80343704 00340664  7F 84 E3 78 */	mr r4, r28
/* 80343708 00340668  7F A5 EB 78 */	mr r5, r29
/* 8034370C 0034066C  4B FF DC 91 */	bl inflate_blocks
/* 80343710 00340670  7C 7D 1B 78 */	mr r29, r3
/* 80343714 00340674  2C 1D FF FD */	cmpwi r29, -3
/* 80343718 00340678  40 82 00 20 */	bne lbl_80343738
/* 8034371C 0034067C  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343720 00340680  38 80 00 0D */	li r4, 0xd
/* 80343724 00340684  38 00 00 00 */	li r0, 0
/* 80343728 00340688  90 83 00 00 */	stw r4, 0(r3)
/* 8034372C 0034068C  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343730 00340690  90 03 00 04 */	stw r0, 4(r3)
/* 80343734 00340694  4B FF FC 90 */	b lbl_803433C4
lbl_80343738:
/* 80343738 00340698  2C 1D 00 00 */	cmpwi r29, 0
/* 8034373C 0034069C  40 82 00 08 */	bne lbl_80343744
/* 80343740 003406A0  7F DD F3 78 */	mr r29, r30
lbl_80343744:
/* 80343744 003406A4  2C 1D 00 01 */	cmpwi r29, 1
/* 80343748 003406A8  41 82 00 0C */	beq lbl_80343754
/* 8034374C 003406AC  7F A3 EB 78 */	mr r3, r29
/* 80343750 003406B0  48 00 01 F0 */	b lbl_80343940
lbl_80343754:
/* 80343754 003406B4  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 80343758 003406B8  7F DD F3 78 */	mr r29, r30
/* 8034375C 003406BC  7F 84 E3 78 */	mr r4, r28
/* 80343760 003406C0  80 65 00 14 */	lwz r3, 0x14(r5)
/* 80343764 003406C4  38 A5 00 04 */	addi r5, r5, 4
/* 80343768 003406C8  4B FF EB A5 */	bl inflate_blocks_reset
/* 8034376C 003406CC  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343770 003406D0  80 03 00 0C */	lwz r0, 0xc(r3)
/* 80343774 003406D4  2C 00 00 00 */	cmpwi r0, 0
/* 80343778 003406D8  41 82 00 10 */	beq lbl_80343788
/* 8034377C 003406DC  38 00 00 0C */	li r0, 0xc
/* 80343780 003406E0  90 03 00 00 */	stw r0, 0(r3)
/* 80343784 003406E4  4B FF FC 40 */	b lbl_803433C4
lbl_80343788:
/* 80343788 003406E8  38 00 00 08 */	li r0, 8
/* 8034378C 003406EC  90 03 00 00 */	stw r0, 0(r3)
.global lbl_80343790
lbl_80343790:
/* 80343790 003406F0  80 7C 00 04 */	lwz r3, 4(r28)
/* 80343794 003406F4  28 03 00 00 */	cmplwi r3, 0
/* 80343798 003406F8  40 82 00 0C */	bne lbl_803437A4
/* 8034379C 003406FC  7F A3 EB 78 */	mr r3, r29
/* 803437A0 00340700  48 00 01 A0 */	b lbl_80343940
lbl_803437A4:
/* 803437A4 00340704  38 63 FF FF */	addi r3, r3, -1
/* 803437A8 00340708  38 00 00 09 */	li r0, 9
/* 803437AC 0034070C  90 7C 00 04 */	stw r3, 4(r28)
/* 803437B0 00340710  7F DD F3 78 */	mr r29, r30
/* 803437B4 00340714  80 7C 00 08 */	lwz r3, 8(r28)
/* 803437B8 00340718  38 63 00 01 */	addi r3, r3, 1
/* 803437BC 0034071C  90 7C 00 08 */	stw r3, 8(r28)
/* 803437C0 00340720  80 9C 00 00 */	lwz r4, 0(r28)
/* 803437C4 00340724  38 64 00 01 */	addi r3, r4, 1
/* 803437C8 00340728  90 7C 00 00 */	stw r3, 0(r28)
/* 803437CC 0034072C  88 84 00 00 */	lbz r4, 0(r4)
/* 803437D0 00340730  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 803437D4 00340734  54 84 C0 0E */	slwi r4, r4, 0x18
/* 803437D8 00340738  90 83 00 08 */	stw r4, 8(r3)
/* 803437DC 0034073C  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 803437E0 00340740  90 03 00 00 */	stw r0, 0(r3)
.global lbl_803437E4
lbl_803437E4:
/* 803437E4 00340744  80 7C 00 04 */	lwz r3, 4(r28)
/* 803437E8 00340748  28 03 00 00 */	cmplwi r3, 0
/* 803437EC 0034074C  40 82 00 0C */	bne lbl_803437F8
/* 803437F0 00340750  7F A3 EB 78 */	mr r3, r29
/* 803437F4 00340754  48 00 01 4C */	b lbl_80343940
lbl_803437F8:
/* 803437F8 00340758  38 63 FF FF */	addi r3, r3, -1
/* 803437FC 0034075C  38 00 00 0A */	li r0, 0xa
/* 80343800 00340760  90 7C 00 04 */	stw r3, 4(r28)
/* 80343804 00340764  7F DD F3 78 */	mr r29, r30
/* 80343808 00340768  80 7C 00 08 */	lwz r3, 8(r28)
/* 8034380C 0034076C  38 63 00 01 */	addi r3, r3, 1
/* 80343810 00340770  90 7C 00 08 */	stw r3, 8(r28)
/* 80343814 00340774  80 9C 00 00 */	lwz r4, 0(r28)
/* 80343818 00340778  38 64 00 01 */	addi r3, r4, 1
/* 8034381C 0034077C  90 7C 00 00 */	stw r3, 0(r28)
/* 80343820 00340780  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 80343824 00340784  88 64 00 00 */	lbz r3, 0(r4)
/* 80343828 00340788  80 85 00 08 */	lwz r4, 8(r5)
/* 8034382C 0034078C  54 63 80 1E */	slwi r3, r3, 0x10
/* 80343830 00340790  7C 64 1A 14 */	add r3, r4, r3
/* 80343834 00340794  90 65 00 08 */	stw r3, 8(r5)
/* 80343838 00340798  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 8034383C 0034079C  90 03 00 00 */	stw r0, 0(r3)
.global lbl_80343840
lbl_80343840:
/* 80343840 003407A0  80 7C 00 04 */	lwz r3, 4(r28)
/* 80343844 003407A4  28 03 00 00 */	cmplwi r3, 0
/* 80343848 003407A8  40 82 00 0C */	bne lbl_80343854
/* 8034384C 003407AC  7F A3 EB 78 */	mr r3, r29
/* 80343850 003407B0  48 00 00 F0 */	b lbl_80343940
lbl_80343854:
/* 80343854 003407B4  38 63 FF FF */	addi r3, r3, -1
/* 80343858 003407B8  38 00 00 0B */	li r0, 0xb
/* 8034385C 003407BC  90 7C 00 04 */	stw r3, 4(r28)
/* 80343860 003407C0  7F DD F3 78 */	mr r29, r30
/* 80343864 003407C4  80 7C 00 08 */	lwz r3, 8(r28)
/* 80343868 003407C8  38 63 00 01 */	addi r3, r3, 1
/* 8034386C 003407CC  90 7C 00 08 */	stw r3, 8(r28)
/* 80343870 003407D0  80 9C 00 00 */	lwz r4, 0(r28)
/* 80343874 003407D4  38 64 00 01 */	addi r3, r4, 1
/* 80343878 003407D8  90 7C 00 00 */	stw r3, 0(r28)
/* 8034387C 003407DC  80 BC 00 1C */	lwz r5, 0x1c(r28)
/* 80343880 003407E0  88 64 00 00 */	lbz r3, 0(r4)
/* 80343884 003407E4  80 85 00 08 */	lwz r4, 8(r5)
/* 80343888 003407E8  54 63 40 2E */	slwi r3, r3, 8
/* 8034388C 003407EC  7C 64 1A 14 */	add r3, r4, r3
/* 80343890 003407F0  90 65 00 08 */	stw r3, 8(r5)
/* 80343894 003407F4  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 80343898 003407F8  90 03 00 00 */	stw r0, 0(r3)
.global lbl_8034389C
lbl_8034389C:
/* 8034389C 003407FC  80 7C 00 04 */	lwz r3, 4(r28)
/* 803438A0 00340800  28 03 00 00 */	cmplwi r3, 0
/* 803438A4 00340804  40 82 00 0C */	bne lbl_803438B0
/* 803438A8 00340808  7F A3 EB 78 */	mr r3, r29
/* 803438AC 0034080C  48 00 00 94 */	b lbl_80343940
lbl_803438B0:
/* 803438B0 00340810  38 03 FF FF */	addi r0, r3, -1
/* 803438B4 00340814  7F DD F3 78 */	mr r29, r30
/* 803438B8 00340818  90 1C 00 04 */	stw r0, 4(r28)
/* 803438BC 0034081C  80 7C 00 08 */	lwz r3, 8(r28)
/* 803438C0 00340820  38 03 00 01 */	addi r0, r3, 1
/* 803438C4 00340824  90 1C 00 08 */	stw r0, 8(r28)
/* 803438C8 00340828  80 7C 00 00 */	lwz r3, 0(r28)
/* 803438CC 0034082C  38 03 00 01 */	addi r0, r3, 1
/* 803438D0 00340830  90 1C 00 00 */	stw r0, 0(r28)
/* 803438D4 00340834  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803438D8 00340838  88 03 00 00 */	lbz r0, 0(r3)
/* 803438DC 0034083C  80 64 00 08 */	lwz r3, 8(r4)
/* 803438E0 00340840  7C 03 02 14 */	add r0, r3, r0
/* 803438E4 00340844  90 04 00 08 */	stw r0, 8(r4)
/* 803438E8 00340848  80 9C 00 1C */	lwz r4, 0x1c(r28)
/* 803438EC 0034084C  80 64 00 04 */	lwz r3, 4(r4)
/* 803438F0 00340850  80 04 00 08 */	lwz r0, 8(r4)
/* 803438F4 00340854  7C 03 00 40 */	cmplw r3, r0
/* 803438F8 00340858  41 82 00 2C */	beq lbl_80343924
/* 803438FC 0034085C  38 00 00 0D */	li r0, 0xd
/* 80343900 00340860  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 80343904 00340864  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 80343908 00340868  90 04 00 00 */	stw r0, 0(r4)
/* 8034390C 0034086C  38 63 00 56 */	addi r3, r3, 0x56
/* 80343910 00340870  38 00 00 05 */	li r0, 5
/* 80343914 00340874  90 7C 00 18 */	stw r3, 0x18(r28)
/* 80343918 00340878  80 7C 00 1C */	lwz r3, 0x1c(r28)
/* 8034391C 0034087C  90 03 00 04 */	stw r0, 4(r3)
/* 80343920 00340880  4B FF FA A4 */	b lbl_803433C4
lbl_80343924:
/* 80343924 00340884  38 00 00 0C */	li r0, 0xc
/* 80343928 00340888  90 04 00 00 */	stw r0, 0(r4)
.global lbl_8034392C
lbl_8034392C:
/* 8034392C 0034088C  38 60 00 01 */	li r3, 1
/* 80343930 00340890  48 00 00 10 */	b lbl_80343940
.global lbl_80343934
lbl_80343934:
/* 80343934 00340894  38 60 FF FD */	li r3, -3
/* 80343938 00340898  48 00 00 08 */	b lbl_80343940
lbl_8034393C:
/* 8034393C 0034089C  38 60 FF FE */	li r3, -2
lbl_80343940:
/* 80343940 003408A0  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80343944 003408A4  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80343948 003408A8  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8034394C 003408AC  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80343950 003408B0  83 81 00 10 */	lwz r28, 0x10(r1)
/* 80343954 003408B4  7C 08 03 A6 */	mtlr r0
/* 80343958 003408B8  38 21 00 20 */	addi r1, r1, 0x20
/* 8034395C 003408BC  4E 80 00 20 */	blr 

.global inflateInit2_
inflateInit2_:
/* 80343960 003408C0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80343964 003408C4  7C 08 02 A6 */	mflr r0
/* 80343968 003408C8  28 04 00 00 */	cmplwi r4, 0
/* 8034396C 003408CC  90 01 00 14 */	stw r0, 0x14(r1)
/* 80343970 003408D0  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80343974 003408D4  7C 7F 1B 78 */	mr r31, r3
/* 80343978 003408D8  41 82 00 2C */	beq lbl_803439A4
/* 8034397C 003408DC  3C 60 80 3D */	lis r3, lbl_803D7BD8@ha
/* 80343980 003408E0  88 84 00 00 */	lbz r4, 0(r4)
/* 80343984 003408E4  38 63 7B D8 */	addi r3, r3, lbl_803D7BD8@l
/* 80343988 003408E8  88 03 00 6B */	lbz r0, 0x6b(r3)
/* 8034398C 003408EC  7C 83 07 74 */	extsb r3, r4
/* 80343990 003408F0  7C 00 07 74 */	extsb r0, r0
/* 80343994 003408F4  7C 03 00 00 */	cmpw r3, r0
/* 80343998 003408F8  40 82 00 0C */	bne lbl_803439A4
/* 8034399C 003408FC  28 05 00 38 */	cmplwi r5, 0x38
/* 803439A0 00340900  41 82 00 0C */	beq lbl_803439AC
lbl_803439A4:
/* 803439A4 00340904  38 60 FF FA */	li r3, -6
/* 803439A8 00340908  48 00 01 84 */	b lbl_80343B2C
lbl_803439AC:
/* 803439AC 0034090C  28 1F 00 00 */	cmplwi r31, 0
/* 803439B0 00340910  40 82 00 0C */	bne lbl_803439BC
/* 803439B4 00340914  38 60 FF FE */	li r3, -2
/* 803439B8 00340918  48 00 01 74 */	b lbl_80343B2C
lbl_803439BC:
/* 803439BC 0034091C  38 80 00 00 */	li r4, 0
/* 803439C0 00340920  90 9F 00 18 */	stw r4, 0x18(r31)
/* 803439C4 00340924  80 1F 00 20 */	lwz r0, 0x20(r31)
/* 803439C8 00340928  28 00 00 00 */	cmplwi r0, 0
/* 803439CC 0034092C  40 82 00 14 */	bne lbl_803439E0
/* 803439D0 00340930  3C 60 80 34 */	lis r3, zcalloc@ha
/* 803439D4 00340934  38 03 45 C0 */	addi r0, r3, zcalloc@l
/* 803439D8 00340938  90 1F 00 20 */	stw r0, 0x20(r31)
/* 803439DC 0034093C  90 9F 00 28 */	stw r4, 0x28(r31)
lbl_803439E0:
/* 803439E0 00340940  80 1F 00 24 */	lwz r0, 0x24(r31)
/* 803439E4 00340944  28 00 00 00 */	cmplwi r0, 0
/* 803439E8 00340948  40 82 00 10 */	bne lbl_803439F8
/* 803439EC 0034094C  3C 60 80 34 */	lis r3, zcfree@ha
/* 803439F0 00340950  38 03 45 8C */	addi r0, r3, zcfree@l
/* 803439F4 00340954  90 1F 00 24 */	stw r0, 0x24(r31)
lbl_803439F8:
/* 803439F8 00340958  81 9F 00 20 */	lwz r12, 0x20(r31)
/* 803439FC 0034095C  38 80 00 01 */	li r4, 1
/* 80343A00 00340960  80 7F 00 28 */	lwz r3, 0x28(r31)
/* 80343A04 00340964  38 A0 00 18 */	li r5, 0x18
/* 80343A08 00340968  7D 89 03 A6 */	mtctr r12
/* 80343A0C 0034096C  4E 80 04 21 */	bctrl 
/* 80343A10 00340970  28 03 00 00 */	cmplwi r3, 0
/* 80343A14 00340974  90 7F 00 1C */	stw r3, 0x1c(r31)
/* 80343A18 00340978  40 82 00 0C */	bne lbl_80343A24
/* 80343A1C 0034097C  38 60 FF FC */	li r3, -4
/* 80343A20 00340980  48 00 01 0C */	b lbl_80343B2C
lbl_80343A24:
/* 80343A24 00340984  80 9F 00 1C */	lwz r4, 0x1c(r31)
/* 80343A28 00340988  38 C0 00 00 */	li r6, 0
/* 80343A2C 0034098C  38 00 00 0F */	li r0, 0xf
/* 80343A30 00340990  3C 60 80 34 */	lis r3, adler32@ha
/* 80343A34 00340994  90 C4 00 14 */	stw r6, 0x14(r4)
/* 80343A38 00340998  38 83 11 20 */	addi r4, r3, adler32@l
/* 80343A3C 0034099C  7F E3 FB 78 */	mr r3, r31
/* 80343A40 003409A0  80 BF 00 1C */	lwz r5, 0x1c(r31)
/* 80343A44 003409A4  90 C5 00 0C */	stw r6, 0xc(r5)
/* 80343A48 003409A8  80 BF 00 1C */	lwz r5, 0x1c(r31)
/* 80343A4C 003409AC  90 05 00 10 */	stw r0, 0x10(r5)
/* 80343A50 003409B0  80 BF 00 1C */	lwz r5, 0x1c(r31)
/* 80343A54 003409B4  80 05 00 0C */	lwz r0, 0xc(r5)
/* 80343A58 003409B8  2C 00 00 00 */	cmpwi r0, 0
/* 80343A5C 003409BC  41 82 00 08 */	beq lbl_80343A64
/* 80343A60 003409C0  7C C4 33 78 */	mr r4, r6
lbl_80343A64:
/* 80343A64 003409C4  3C A0 00 01 */	lis r5, 0x00008000@ha
/* 80343A68 003409C8  38 A5 80 00 */	addi r5, r5, 0x00008000@l
/* 80343A6C 003409CC  4B FF E7 01 */	bl inflate_blocks_new
/* 80343A70 003409D0  80 9F 00 1C */	lwz r4, 0x1c(r31)
/* 80343A74 003409D4  28 03 00 00 */	cmplwi r3, 0
/* 80343A78 003409D8  90 64 00 14 */	stw r3, 0x14(r4)
/* 80343A7C 003409DC  40 82 00 5C */	bne lbl_80343AD8
/* 80343A80 003409E0  28 1F 00 00 */	cmplwi r31, 0
/* 80343A84 003409E4  41 82 00 4C */	beq lbl_80343AD0
/* 80343A88 003409E8  80 7F 00 1C */	lwz r3, 0x1c(r31)
/* 80343A8C 003409EC  28 03 00 00 */	cmplwi r3, 0
/* 80343A90 003409F0  41 82 00 40 */	beq lbl_80343AD0
/* 80343A94 003409F4  80 1F 00 24 */	lwz r0, 0x24(r31)
/* 80343A98 003409F8  28 00 00 00 */	cmplwi r0, 0
/* 80343A9C 003409FC  41 82 00 34 */	beq lbl_80343AD0
/* 80343AA0 00340A00  80 63 00 14 */	lwz r3, 0x14(r3)
/* 80343AA4 00340A04  28 03 00 00 */	cmplwi r3, 0
/* 80343AA8 00340A08  41 82 00 0C */	beq lbl_80343AB4
/* 80343AAC 00340A0C  7F E4 FB 78 */	mr r4, r31
/* 80343AB0 00340A10  4B FF D7 F5 */	bl inflate_blocks_free
lbl_80343AB4:
/* 80343AB4 00340A14  81 9F 00 24 */	lwz r12, 0x24(r31)
/* 80343AB8 00340A18  80 7F 00 28 */	lwz r3, 0x28(r31)
/* 80343ABC 00340A1C  80 9F 00 1C */	lwz r4, 0x1c(r31)
/* 80343AC0 00340A20  7D 89 03 A6 */	mtctr r12
/* 80343AC4 00340A24  4E 80 04 21 */	bctrl 
/* 80343AC8 00340A28  38 00 00 00 */	li r0, 0
/* 80343ACC 00340A2C  90 1F 00 1C */	stw r0, 0x1c(r31)
lbl_80343AD0:
/* 80343AD0 00340A30  38 60 FF FC */	li r3, -4
/* 80343AD4 00340A34  48 00 00 58 */	b lbl_80343B2C
lbl_80343AD8:
/* 80343AD8 00340A38  28 1F 00 00 */	cmplwi r31, 0
/* 80343ADC 00340A3C  41 82 00 4C */	beq lbl_80343B28
/* 80343AE0 00340A40  80 1F 00 1C */	lwz r0, 0x1c(r31)
/* 80343AE4 00340A44  28 00 00 00 */	cmplwi r0, 0
/* 80343AE8 00340A48  41 82 00 40 */	beq lbl_80343B28
/* 80343AEC 00340A4C  38 60 00 00 */	li r3, 0
/* 80343AF0 00340A50  90 7F 00 14 */	stw r3, 0x14(r31)
/* 80343AF4 00340A54  90 7F 00 08 */	stw r3, 8(r31)
/* 80343AF8 00340A58  90 7F 00 18 */	stw r3, 0x18(r31)
/* 80343AFC 00340A5C  80 9F 00 1C */	lwz r4, 0x1c(r31)
/* 80343B00 00340A60  80 04 00 0C */	lwz r0, 0xc(r4)
/* 80343B04 00340A64  2C 00 00 00 */	cmpwi r0, 0
/* 80343B08 00340A68  41 82 00 08 */	beq lbl_80343B10
/* 80343B0C 00340A6C  38 60 00 07 */	li r3, 7
lbl_80343B10:
/* 80343B10 00340A70  90 64 00 00 */	stw r3, 0(r4)
/* 80343B14 00340A74  7F E4 FB 78 */	mr r4, r31
/* 80343B18 00340A78  38 A0 00 00 */	li r5, 0
/* 80343B1C 00340A7C  80 7F 00 1C */	lwz r3, 0x1c(r31)
/* 80343B20 00340A80  80 63 00 14 */	lwz r3, 0x14(r3)
/* 80343B24 00340A84  4B FF E7 E9 */	bl inflate_blocks_reset
lbl_80343B28:
/* 80343B28 00340A88  38 60 00 00 */	li r3, 0
lbl_80343B2C:
/* 80343B2C 00340A8C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80343B30 00340A90  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80343B34 00340A94  7C 08 03 A6 */	mtlr r0
/* 80343B38 00340A98  38 21 00 10 */	addi r1, r1, 0x10
/* 80343B3C 00340A9C  4E 80 00 20 */	blr 

.global inflateEnd
inflateEnd:
/* 80343B40 00340AA0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80343B44 00340AA4  7C 08 02 A6 */	mflr r0
/* 80343B48 00340AA8  90 01 00 14 */	stw r0, 0x14(r1)
/* 80343B4C 00340AAC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80343B50 00340AB0  7C 7F 1B 79 */	or. r31, r3, r3
/* 80343B54 00340AB4  41 82 00 1C */	beq lbl_80343B70
/* 80343B58 00340AB8  80 7F 00 1C */	lwz r3, 0x1c(r31)
/* 80343B5C 00340ABC  28 03 00 00 */	cmplwi r3, 0
/* 80343B60 00340AC0  41 82 00 10 */	beq lbl_80343B70
/* 80343B64 00340AC4  80 1F 00 24 */	lwz r0, 0x24(r31)
/* 80343B68 00340AC8  28 00 00 00 */	cmplwi r0, 0
/* 80343B6C 00340ACC  40 82 00 0C */	bne lbl_80343B78
lbl_80343B70:
/* 80343B70 00340AD0  38 60 FF FE */	li r3, -2
/* 80343B74 00340AD4  48 00 00 38 */	b lbl_80343BAC
lbl_80343B78:
/* 80343B78 00340AD8  80 63 00 14 */	lwz r3, 0x14(r3)
/* 80343B7C 00340ADC  28 03 00 00 */	cmplwi r3, 0
/* 80343B80 00340AE0  41 82 00 0C */	beq lbl_80343B8C
/* 80343B84 00340AE4  7F E4 FB 78 */	mr r4, r31
/* 80343B88 00340AE8  4B FF D7 1D */	bl inflate_blocks_free
lbl_80343B8C:
/* 80343B8C 00340AEC  81 9F 00 24 */	lwz r12, 0x24(r31)
/* 80343B90 00340AF0  80 7F 00 28 */	lwz r3, 0x28(r31)
/* 80343B94 00340AF4  80 9F 00 1C */	lwz r4, 0x1c(r31)
/* 80343B98 00340AF8  7D 89 03 A6 */	mtctr r12
/* 80343B9C 00340AFC  4E 80 04 21 */	bctrl 
/* 80343BA0 00340B00  38 00 00 00 */	li r0, 0
/* 80343BA4 00340B04  38 60 00 00 */	li r3, 0
/* 80343BA8 00340B08  90 1F 00 1C */	stw r0, 0x1c(r31)
lbl_80343BAC:
/* 80343BAC 00340B0C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80343BB0 00340B10  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80343BB4 00340B14  7C 08 03 A6 */	mtlr r0
/* 80343BB8 00340B18  38 21 00 10 */	addi r1, r1, 0x10
/* 80343BBC 00340B1C  4E 80 00 20 */	blr 
