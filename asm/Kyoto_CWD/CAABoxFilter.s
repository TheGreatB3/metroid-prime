.include "macros.inc"

.section .text, "ax"  # 0x80003640 - 0x803CB1C0

.global Filter__12CAABoxFilterCFRC18CCollisionInfoListR18CCollisionInfoList
Filter__12CAABoxFilterCFRC18CCollisionInfoListR18CCollisionInfoList:
/* 80185850 001827B0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80185854 001827B4  7C 08 02 A6 */	mflr r0
/* 80185858 001827B8  7C 83 23 78 */	mr r3, r4
/* 8018585C 001827BC  7C A4 2B 78 */	mr r4, r5
/* 80185860 001827C0  90 01 00 14 */	stw r0, 0x14(r1)
/* 80185864 001827C4  48 00 00 15 */	bl FilterBoxFloorCollisions__12CAABoxFilterFRC18CCollisionInfoListR18CCollisionInfoList
/* 80185868 001827C8  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8018586C 001827CC  7C 08 03 A6 */	mtlr r0
/* 80185870 001827D0  38 21 00 10 */	addi r1, r1, 0x10
/* 80185874 001827D4  4E 80 00 20 */	blr 

.global FilterBoxFloorCollisions__12CAABoxFilterFRC18CCollisionInfoListR18CCollisionInfoList
FilterBoxFloorCollisions__12CAABoxFilterFRC18CCollisionInfoListR18CCollisionInfoList:
/* 80185878 001827D8  94 21 F3 D0 */	stwu r1, -0xc30(r1)
/* 8018587C 001827DC  7C 08 02 A6 */	mflr r0
/* 80185880 001827E0  90 01 0C 34 */	stw r0, 0xc34(r1)
/* 80185884 001827E4  DB E1 0C 20 */	stfd f31, 0xc20(r1)
/* 80185888 001827E8  F3 E1 0C 28 */	psq_st f31, -984(r1), 0, qr0
/* 8018588C 001827EC  93 E1 0C 1C */	stw r31, 0xc1c(r1)
/* 80185890 001827F0  93 C1 0C 18 */	stw r30, 0xc18(r1)
/* 80185894 001827F4  93 A1 0C 14 */	stw r29, 0xc14(r1)
/* 80185898 001827F8  93 81 0C 10 */	stw r28, 0xc10(r1)
/* 8018589C 001827FC  7C 7E 1B 78 */	mr r30, r3
/* 801858A0 00182800  C3 E2 A3 08 */	lfs f31, lbl_805AC028@sda21(r2)
/* 801858A4 00182804  80 03 00 00 */	lwz r0, 0(r3)
/* 801858A8 00182808  7C 9F 23 78 */	mr r31, r4
/* 801858AC 0018280C  38 FE 00 04 */	addi r7, r30, 4
/* 801858B0 00182810  3C A0 40 00 */	lis r5, 0x4000
/* 801858B4 00182814  1C 00 00 60 */	mulli r0, r0, 0x60
/* 801858B8 00182818  38 80 00 00 */	li r4, 0
/* 801858BC 0018281C  7C DE 02 14 */	add r6, r30, r0
/* 801858C0 00182820  38 C6 00 04 */	addi r6, r6, 4
/* 801858C4 00182824  48 00 00 38 */	b lbl_801858FC
lbl_801858C8:
/* 801858C8 00182828  80 07 00 38 */	lwz r0, 0x38(r7)
/* 801858CC 0018282C  80 67 00 3C */	lwz r3, 0x3c(r7)
/* 801858D0 00182830  7C 00 20 38 */	and r0, r0, r4
/* 801858D4 00182834  7C 63 28 38 */	and r3, r3, r5
/* 801858D8 00182838  7C 63 22 78 */	xor r3, r3, r4
/* 801858DC 0018283C  7C 00 22 78 */	xor r0, r0, r4
/* 801858E0 00182840  7C 60 03 79 */	or. r0, r3, r0
/* 801858E4 00182844  41 82 00 14 */	beq lbl_801858F8
/* 801858E8 00182848  C0 07 00 08 */	lfs f0, 8(r7)
/* 801858EC 0018284C  FC 00 F8 40 */	fcmpo cr0, f0, f31
/* 801858F0 00182850  40 80 00 08 */	bge lbl_801858F8
/* 801858F4 00182854  FF E0 00 90 */	fmr f31, f0
lbl_801858F8:
/* 801858F8 00182858  38 E7 00 60 */	addi r7, r7, 0x60
lbl_801858FC:
/* 801858FC 0018285C  7C 07 30 40 */	cmplw r7, r6
/* 80185900 00182860  40 82 FF C8 */	bne lbl_801858C8
/* 80185904 00182864  38 00 00 00 */	li r0, 0
/* 80185908 00182868  3B 9E 00 04 */	addi r28, r30, 4
/* 8018590C 0018286C  90 01 00 08 */	stw r0, 8(r1)
/* 80185910 00182870  3F A0 80 00 */	lis r29, 0x8000
/* 80185914 00182874  48 00 00 5C */	b lbl_80185970
lbl_80185918:
/* 80185918 00182878  80 7C 00 3C */	lwz r3, 0x3c(r28)
/* 8018591C 0018287C  38 80 00 00 */	li r4, 0
/* 80185920 00182880  80 1C 00 38 */	lwz r0, 0x38(r28)
/* 80185924 00182884  7C 63 E8 38 */	and r3, r3, r29
/* 80185928 00182888  7C 00 20 38 */	and r0, r0, r4
/* 8018592C 0018288C  7C 63 22 78 */	xor r3, r3, r4
/* 80185930 00182890  7C 00 22 78 */	xor r0, r0, r4
/* 80185934 00182894  7C 60 03 79 */	or. r0, r3, r0
/* 80185938 00182898  41 82 00 24 */	beq lbl_8018595C
/* 8018593C 0018289C  C0 1C 00 08 */	lfs f0, 8(r28)
/* 80185940 001828A0  FC 00 F8 40 */	fcmpo cr0, f0, f31
/* 80185944 001828A4  40 80 00 28 */	bge lbl_8018596C
/* 80185948 001828A8  7F 84 E3 78 */	mr r4, r28
/* 8018594C 001828AC  38 61 00 08 */	addi r3, r1, 8
/* 80185950 001828B0  38 A0 00 00 */	li r5, 0
/* 80185954 001828B4  4B FF FC CD */	bl Add__18CCollisionInfoListFRC14CCollisionInfob
/* 80185958 001828B8  48 00 00 14 */	b lbl_8018596C
lbl_8018595C:
/* 8018595C 001828BC  7F 84 E3 78 */	mr r4, r28
/* 80185960 001828C0  38 61 00 08 */	addi r3, r1, 8
/* 80185964 001828C4  38 A0 00 00 */	li r5, 0
/* 80185968 001828C8  4B FF FC B9 */	bl Add__18CCollisionInfoListFRC14CCollisionInfob
lbl_8018596C:
/* 8018596C 001828CC  3B 9C 00 60 */	addi r28, r28, 0x60
lbl_80185970:
/* 80185970 001828D0  80 1E 00 00 */	lwz r0, 0(r30)
/* 80185974 001828D4  1C 00 00 60 */	mulli r0, r0, 0x60
/* 80185978 001828D8  7C 7E 02 14 */	add r3, r30, r0
/* 8018597C 001828DC  38 03 00 04 */	addi r0, r3, 4
/* 80185980 001828E0  7C 1C 00 40 */	cmplw r28, r0
/* 80185984 001828E4  40 82 FF 94 */	bne lbl_80185918
/* 80185988 001828E8  7F E4 FB 78 */	mr r4, r31
/* 8018598C 001828EC  38 61 00 08 */	addi r3, r1, 8
/* 80185990 001828F0  48 14 C7 25 */	bl AddAverageToFront__13CollisionUtilFRC18CCollisionInfoListR18CCollisionInfoList
/* 80185994 001828F4  80 A1 00 08 */	lwz r5, 8(r1)
/* 80185998 001828F8  38 60 00 00 */	li r3, 0
/* 8018599C 001828FC  2C 05 00 00 */	cmpwi r5, 0
/* 801859A0 00182900  40 81 00 40 */	ble lbl_801859E0
/* 801859A4 00182904  2C 05 00 08 */	cmpwi r5, 8
/* 801859A8 00182908  38 85 FF F8 */	addi r4, r5, -8
/* 801859AC 0018290C  40 81 00 20 */	ble lbl_801859CC
/* 801859B0 00182910  38 04 00 07 */	addi r0, r4, 7
/* 801859B4 00182914  54 00 E8 FE */	srwi r0, r0, 3
/* 801859B8 00182918  7C 09 03 A6 */	mtctr r0
/* 801859BC 0018291C  2C 04 00 00 */	cmpwi r4, 0
/* 801859C0 00182920  40 81 00 0C */	ble lbl_801859CC
lbl_801859C4:
/* 801859C4 00182924  38 63 00 08 */	addi r3, r3, 8
/* 801859C8 00182928  42 00 FF FC */	bdnz lbl_801859C4
lbl_801859CC:
/* 801859CC 0018292C  7C 03 28 50 */	subf r0, r3, r5
/* 801859D0 00182930  7C 09 03 A6 */	mtctr r0
/* 801859D4 00182934  7C 03 28 00 */	cmpw r3, r5
/* 801859D8 00182938  40 80 00 08 */	bge lbl_801859E0
lbl_801859DC:
/* 801859DC 0018293C  42 00 00 00 */	bdnz lbl_801859DC
lbl_801859E0:
/* 801859E0 00182940  38 00 00 00 */	li r0, 0
/* 801859E4 00182944  90 01 00 08 */	stw r0, 8(r1)
/* 801859E8 00182948  E3 E1 0C 28 */	psq_l f31, -984(r1), 0, qr0
/* 801859EC 0018294C  80 01 0C 34 */	lwz r0, 0xc34(r1)
/* 801859F0 00182950  CB E1 0C 20 */	lfd f31, 0xc20(r1)
/* 801859F4 00182954  83 E1 0C 1C */	lwz r31, 0xc1c(r1)
/* 801859F8 00182958  83 C1 0C 18 */	lwz r30, 0xc18(r1)
/* 801859FC 0018295C  83 A1 0C 14 */	lwz r29, 0xc14(r1)
/* 80185A00 00182960  83 81 0C 10 */	lwz r28, 0xc10(r1)
/* 80185A04 00182964  7C 08 03 A6 */	mtlr r0
/* 80185A08 00182968  38 21 0C 30 */	addi r1, r1, 0xc30
/* 80185A0C 0018296C  4E 80 00 20 */	blr 
