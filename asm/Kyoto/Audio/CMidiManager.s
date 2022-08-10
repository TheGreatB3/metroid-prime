.include "macros.inc"

.section .ctors, "wa"
lbl_ctor:
.4byte __sinit_CMidiManager_cpp

.section .data

.global lbl_803EF788
lbl_803EF788:
	# ROM: 0x3EC788
	.4byte 0
	.4byte 0
	.4byte sub_80358ca4
	.4byte 0

.section .text, "ax"

.global FMidiDataFactory__FRC10SObjectTagR12CInputStreamRC15CVParamTransfer
FMidiDataFactory__FRC10SObjectTagR12CInputStreamRC15CVParamTransfer:
/* 80358B64 00355AC4  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80358B68 00355AC8  7C 08 02 A6 */	mflr r0
/* 80358B6C 00355ACC  3C 80 80 3E */	lis r4, lbl_803D8340@ha
/* 80358B70 00355AD0  90 01 00 14 */	stw r0, 0x14(r1)
/* 80358B74 00355AD4  38 84 83 40 */	addi r4, r4, lbl_803D8340@l
/* 80358B78 00355AD8  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80358B7C 00355ADC  7C BF 2B 78 */	mr r31, r5
/* 80358B80 00355AE0  38 A0 00 00 */	li r5, 0
/* 80358B84 00355AE4  93 C1 00 08 */	stw r30, 8(r1)
/* 80358B88 00355AE8  7C 7E 1B 78 */	mr r30, r3
/* 80358B8C 00355AEC  38 60 00 10 */	li r3, 0x10
/* 80358B90 00355AF0  4B FB CC DD */	bl __nw__FUlPCcPCc
/* 80358B94 00355AF4  7C 64 1B 79 */	or. r4, r3, r3
/* 80358B98 00355AF8  41 82 00 10 */	beq lbl_80358BA8
/* 80358B9C 00355AFC  7F E4 FB 78 */	mr r4, r31
/* 80358BA0 00355B00  48 00 02 85 */	bl __ct__Q212CMidiManager9CMidiDataFR12CInputStream
/* 80358BA4 00355B04  7C 64 1B 78 */	mr r4, r3
lbl_80358BA8:
/* 80358BA8 00355B08  7F C3 F3 78 */	mr r3, r30
/* 80358BAC 00355B0C  48 00 00 1D */	bl "__ct<Q212CMidiManager9CMidiData>__16CFactoryFnReturnFPQ212CMidiManager9CMidiData"
/* 80358BB0 00355B10  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80358BB4 00355B14  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80358BB8 00355B18  83 C1 00 08 */	lwz r30, 8(r1)
/* 80358BBC 00355B1C  7C 08 03 A6 */	mtlr r0
/* 80358BC0 00355B20  38 21 00 10 */	addi r1, r1, 0x10
/* 80358BC4 00355B24  4E 80 00 20 */	blr

.global "__ct<Q212CMidiManager9CMidiData>__16CFactoryFnReturnFPQ212CMidiManager9CMidiData"
"__ct<Q212CMidiManager9CMidiData>__16CFactoryFnReturnFPQ212CMidiManager9CMidiData":
/* 80358BC8 00355B28  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80358BCC 00355B2C  7C 08 02 A6 */	mflr r0
/* 80358BD0 00355B30  90 01 00 24 */	stw r0, 0x24(r1)
/* 80358BD4 00355B34  7C 04 00 D0 */	neg r0, r4
/* 80358BD8 00355B38  7C 00 23 78 */	or r0, r0, r4
/* 80358BDC 00355B3C  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80358BE0 00355B40  54 00 0F FE */	srwi r0, r0, 0x1f
/* 80358BE4 00355B44  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80358BE8 00355B48  7C 7E 1B 78 */	mr r30, r3
/* 80358BEC 00355B4C  38 61 00 08 */	addi r3, r1, 8
/* 80358BF0 00355B50  90 81 00 14 */	stw r4, 0x14(r1)
/* 80358BF4 00355B54  38 81 00 10 */	addi r4, r1, 0x10
/* 80358BF8 00355B58  98 01 00 10 */	stb r0, 0x10(r1)
/* 80358BFC 00355B5C  48 00 01 61 */	bl "GetIObjObjectFor__34TToken<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>"
/* 80358C00 00355B60  80 61 00 0C */	lwz r3, 0xc(r1)
/* 80358C04 00355B64  38 00 00 00 */	li r0, 0
/* 80358C08 00355B68  98 01 00 08 */	stb r0, 8(r1)
/* 80358C0C 00355B6C  7C 03 00 D0 */	neg r0, r3
/* 80358C10 00355B70  7C 00 1B 78 */	or r0, r0, r3
/* 80358C14 00355B74  54 00 0F FE */	srwi r0, r0, 0x1f
/* 80358C18 00355B78  98 1E 00 00 */	stb r0, 0(r30)
/* 80358C1C 00355B7C  90 7E 00 04 */	stw r3, 4(r30)
/* 80358C20 00355B80  88 01 00 08 */	lbz r0, 8(r1)
/* 80358C24 00355B84  28 00 00 00 */	cmplwi r0, 0
/* 80358C28 00355B88  41 82 00 24 */	beq lbl_80358C4C
/* 80358C2C 00355B8C  80 61 00 0C */	lwz r3, 0xc(r1)
/* 80358C30 00355B90  28 03 00 00 */	cmplwi r3, 0
/* 80358C34 00355B94  41 82 00 18 */	beq lbl_80358C4C
/* 80358C38 00355B98  81 83 00 00 */	lwz r12, 0(r3)
/* 80358C3C 00355B9C  38 80 00 01 */	li r4, 1
/* 80358C40 00355BA0  81 8C 00 08 */	lwz r12, 8(r12)
/* 80358C44 00355BA4  7D 89 03 A6 */	mtctr r12
/* 80358C48 00355BA8  4E 80 04 21 */	bctrl
lbl_80358C4C:
/* 80358C4C 00355BAC  88 01 00 10 */	lbz r0, 0x10(r1)
/* 80358C50 00355BB0  28 00 00 00 */	cmplwi r0, 0
/* 80358C54 00355BB4  41 82 00 34 */	beq lbl_80358C88
/* 80358C58 00355BB8  83 E1 00 14 */	lwz r31, 0x14(r1)
/* 80358C5C 00355BBC  28 1F 00 00 */	cmplwi r31, 0
/* 80358C60 00355BC0  41 82 00 28 */	beq lbl_80358C88
/* 80358C64 00355BC4  34 1F 00 08 */	addic. r0, r31, 8
/* 80358C68 00355BC8  41 82 00 18 */	beq lbl_80358C80
/* 80358C6C 00355BCC  88 1F 00 08 */	lbz r0, 8(r31)
/* 80358C70 00355BD0  28 00 00 00 */	cmplwi r0, 0
/* 80358C74 00355BD4  41 82 00 0C */	beq lbl_80358C80
/* 80358C78 00355BD8  80 7F 00 0C */	lwz r3, 0xc(r31)
/* 80358C7C 00355BDC  4B FB CC B5 */	bl Free__7CMemoryFPCv
lbl_80358C80:
/* 80358C80 00355BE0  7F E3 FB 78 */	mr r3, r31
/* 80358C84 00355BE4  4B FB CC AD */	bl Free__7CMemoryFPCv
lbl_80358C88:
/* 80358C88 00355BE8  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80358C8C 00355BEC  7F C3 F3 78 */	mr r3, r30
/* 80358C90 00355BF0  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80358C94 00355BF4  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 80358C98 00355BF8  7C 08 03 A6 */	mtlr r0
/* 80358C9C 00355BFC  38 21 00 20 */	addi r1, r1, 0x20
/* 80358CA0 00355C00  4E 80 00 20 */	blr

.global sub_80358ca4
sub_80358ca4:
/* 80358CA4 00355C04  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80358CA8 00355C08  7C 08 02 A6 */	mflr r0
/* 80358CAC 00355C0C  90 01 00 24 */	stw r0, 0x24(r1)
/* 80358CB0 00355C10  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80358CB4 00355C14  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80358CB8 00355C18  7C 9E 23 78 */	mr r30, r4
/* 80358CBC 00355C1C  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80358CC0 00355C20  7C 7D 1B 79 */	or. r29, r3, r3
/* 80358CC4 00355C24  41 82 00 78 */	beq lbl_80358D3C
/* 80358CC8 00355C28  3C 60 80 3F */	lis r3, lbl_803EF788@ha
/* 80358CCC 00355C2C  38 03 F7 88 */	addi r0, r3, lbl_803EF788@l
/* 80358CD0 00355C30  90 1D 00 00 */	stw r0, 0(r29)
/* 80358CD4 00355C34  83 FD 00 04 */	lwz r31, 4(r29)
/* 80358CD8 00355C38  28 1F 00 00 */	cmplwi r31, 0
/* 80358CDC 00355C3C  41 82 00 2C */	beq lbl_80358D08
/* 80358CE0 00355C40  41 82 00 28 */	beq lbl_80358D08
/* 80358CE4 00355C44  34 1F 00 08 */	addic. r0, r31, 8
/* 80358CE8 00355C48  41 82 00 18 */	beq lbl_80358D00
/* 80358CEC 00355C4C  88 1F 00 08 */	lbz r0, 8(r31)
/* 80358CF0 00355C50  28 00 00 00 */	cmplwi r0, 0
/* 80358CF4 00355C54  41 82 00 0C */	beq lbl_80358D00
/* 80358CF8 00355C58  80 7F 00 0C */	lwz r3, 0xc(r31)
/* 80358CFC 00355C5C  4B FB CC 35 */	bl Free__7CMemoryFPCv
lbl_80358D00:
/* 80358D00 00355C60  7F E3 FB 78 */	mr r3, r31
/* 80358D04 00355C64  4B FB CC 2D */	bl Free__7CMemoryFPCv
lbl_80358D08:
/* 80358D08 00355C68  28 1D 00 00 */	cmplwi r29, 0
/* 80358D0C 00355C6C  41 82 00 20 */	beq lbl_80358D2C
/* 80358D10 00355C70  3C 60 80 3E */	lis r3, __vt__31TObjOwnerDerivedFromIObjUntyped@ha
/* 80358D14 00355C74  38 03 8D 78 */	addi r0, r3, __vt__31TObjOwnerDerivedFromIObjUntyped@l
/* 80358D18 00355C78  90 1D 00 00 */	stw r0, 0(r29)
/* 80358D1C 00355C7C  41 82 00 10 */	beq lbl_80358D2C
/* 80358D20 00355C80  3C 60 80 3E */	lis r3, __vt__4IObj@ha
/* 80358D24 00355C84  38 03 8D 6C */	addi r0, r3, __vt__4IObj@l
/* 80358D28 00355C88  90 1D 00 00 */	stw r0, 0(r29)
lbl_80358D2C:
/* 80358D2C 00355C8C  7F C0 07 35 */	extsh. r0, r30
/* 80358D30 00355C90  40 81 00 0C */	ble lbl_80358D3C
/* 80358D34 00355C94  7F A3 EB 78 */	mr r3, r29
/* 80358D38 00355C98  4B FB CB F9 */	bl Free__7CMemoryFPCv
lbl_80358D3C:
/* 80358D3C 00355C9C  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80358D40 00355CA0  7F A3 EB 78 */	mr r3, r29
/* 80358D44 00355CA4  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80358D48 00355CA8  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 80358D4C 00355CAC  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80358D50 00355CB0  7C 08 03 A6 */	mtlr r0
/* 80358D54 00355CB4  38 21 00 20 */	addi r1, r1, 0x20
/* 80358D58 00355CB8  4E 80 00 20 */	blr

.global "GetIObjObjectFor__34TToken<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>"
"GetIObjObjectFor__34TToken<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>":
/* 80358D5C 00355CBC  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80358D60 00355CC0  7C 08 02 A6 */	mflr r0
/* 80358D64 00355CC4  90 01 00 14 */	stw r0, 0x14(r1)
/* 80358D68 00355CC8  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80358D6C 00355CCC  7C 7F 1B 78 */	mr r31, r3
/* 80358D70 00355CD0  48 00 00 19 */	bl "GetNewDerivedObject__52TObjOwnerDerivedFromIObj<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>"
/* 80358D74 00355CD4  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80358D78 00355CD8  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80358D7C 00355CDC  7C 08 03 A6 */	mtlr r0
/* 80358D80 00355CE0  38 21 00 10 */	addi r1, r1, 0x10
/* 80358D84 00355CE4  4E 80 00 20 */	blr

.global "GetNewDerivedObject__52TObjOwnerDerivedFromIObj<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>"
"GetNewDerivedObject__52TObjOwnerDerivedFromIObj<Q212CMidiManager9CMidiData>FRCQ24rstl36auto_ptr<Q212CMidiManager9CMidiData>":
/* 80358D88 00355CE8  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80358D8C 00355CEC  7C 08 02 A6 */	mflr r0
/* 80358D90 00355CF0  3C A0 80 3E */	lis r5, lbl_803D8340@ha
/* 80358D94 00355CF4  90 01 00 14 */	stw r0, 0x14(r1)
/* 80358D98 00355CF8  38 05 83 40 */	addi r0, r5, lbl_803D8340@l
/* 80358D9C 00355CFC  38 A0 00 00 */	li r5, 0
/* 80358DA0 00355D00  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80358DA4 00355D04  7C 9F 23 78 */	mr r31, r4
/* 80358DA8 00355D08  7C 04 03 78 */	mr r4, r0
/* 80358DAC 00355D0C  93 C1 00 08 */	stw r30, 8(r1)
/* 80358DB0 00355D10  7C 7E 1B 78 */	mr r30, r3
/* 80358DB4 00355D14  38 60 00 08 */	li r3, 8
/* 80358DB8 00355D18  4B FB CA B5 */	bl __nw__FUlPCcPCc
/* 80358DBC 00355D1C  28 03 00 00 */	cmplwi r3, 0
/* 80358DC0 00355D20  41 82 00 38 */	beq lbl_80358DF8
/* 80358DC4 00355D24  3C 80 80 3E */	lis r4, __vt__4IObj@ha
/* 80358DC8 00355D28  3C A0 80 3E */	lis r5, __vt__31TObjOwnerDerivedFromIObjUntyped@ha
/* 80358DCC 00355D2C  38 04 8D 6C */	addi r0, r4, __vt__4IObj@l
/* 80358DD0 00355D30  3C 80 80 3F */	lis r4, lbl_803EF788@ha
/* 80358DD4 00355D34  90 03 00 00 */	stw r0, 0(r3)
/* 80358DD8 00355D38  38 C5 8D 78 */	addi r6, r5, __vt__31TObjOwnerDerivedFromIObjUntyped@l
/* 80358DDC 00355D3C  38 A0 00 00 */	li r5, 0
/* 80358DE0 00355D40  38 04 F7 88 */	addi r0, r4, lbl_803EF788@l
/* 80358DE4 00355D44  90 C3 00 00 */	stw r6, 0(r3)
/* 80358DE8 00355D48  98 BF 00 00 */	stb r5, 0(r31)
/* 80358DEC 00355D4C  80 9F 00 04 */	lwz r4, 4(r31)
/* 80358DF0 00355D50  90 83 00 04 */	stw r4, 4(r3)
/* 80358DF4 00355D54  90 03 00 00 */	stw r0, 0(r3)
lbl_80358DF8:
/* 80358DF8 00355D58  7C 03 00 D0 */	neg r0, r3
/* 80358DFC 00355D5C  7C 00 1B 78 */	or r0, r0, r3
/* 80358E00 00355D60  54 00 0F FE */	srwi r0, r0, 0x1f
/* 80358E04 00355D64  98 1E 00 00 */	stb r0, 0(r30)
/* 80358E08 00355D68  90 7E 00 04 */	stw r3, 4(r30)
/* 80358E0C 00355D6C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80358E10 00355D70  83 C1 00 08 */	lwz r30, 8(r1)
/* 80358E14 00355D74  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80358E18 00355D78  7C 08 03 A6 */	mtlr r0
/* 80358E1C 00355D7C  38 21 00 10 */	addi r1, r1, 0x10
/* 80358E20 00355D80  4E 80 00 20 */	blr

.global __ct__Q212CMidiManager9CMidiDataFR12CInputStream
__ct__Q212CMidiManager9CMidiDataFR12CInputStream:
/* 80358E24 00355D84  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80358E28 00355D88  7C 08 02 A6 */	mflr r0
/* 80358E2C 00355D8C  38 A0 FF FF */	li r5, -1
/* 80358E30 00355D90  90 01 00 24 */	stw r0, 0x24(r1)
/* 80358E34 00355D94  38 00 00 00 */	li r0, 0
/* 80358E38 00355D98  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80358E3C 00355D9C  7C 9F 23 78 */	mr r31, r4
/* 80358E40 00355DA0  93 C1 00 18 */	stw r30, 0x18(r1)
/* 80358E44 00355DA4  7C 7E 1B 78 */	mr r30, r3
/* 80358E48 00355DA8  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80358E4C 00355DAC  B0 A3 00 00 */	sth r5, 0(r3)
/* 80358E50 00355DB0  B0 A3 00 02 */	sth r5, 2(r3)
/* 80358E54 00355DB4  7F E3 FB 78 */	mr r3, r31
/* 80358E58 00355DB8  90 BE 00 04 */	stw r5, 4(r30)
/* 80358E5C 00355DBC  98 1E 00 08 */	stb r0, 8(r30)
/* 80358E60 00355DC0  90 1E 00 0C */	stw r0, 0xc(r30)
/* 80358E64 00355DC4  4B FE 5E 21 */	bl ReadLong__12CInputStreamFv
/* 80358E68 00355DC8  7F E3 FB 78 */	mr r3, r31
/* 80358E6C 00355DCC  4B FE 5E 19 */	bl ReadLong__12CInputStreamFv
/* 80358E70 00355DD0  B0 7E 00 00 */	sth r3, 0(r30)
/* 80358E74 00355DD4  7F E3 FB 78 */	mr r3, r31
/* 80358E78 00355DD8  4B FE 5E 0D */	bl ReadLong__12CInputStreamFv
/* 80358E7C 00355DDC  B0 7E 00 02 */	sth r3, 2(r30)
/* 80358E80 00355DE0  7F E3 FB 78 */	mr r3, r31
/* 80358E84 00355DE4  4B FE 5E 01 */	bl ReadLong__12CInputStreamFv
/* 80358E88 00355DE8  90 7E 00 04 */	stw r3, 4(r30)
/* 80358E8C 00355DEC  7F E3 FB 78 */	mr r3, r31
/* 80358E90 00355DF0  4B FE 5D F5 */	bl ReadLong__12CInputStreamFv
/* 80358E94 00355DF4  3C 80 80 3E */	lis r4, lbl_803D8340@ha
/* 80358E98 00355DF8  7C 7D 1B 78 */	mr r29, r3
/* 80358E9C 00355DFC  38 A0 00 00 */	li r5, 0
/* 80358EA0 00355E00  38 84 83 40 */	addi r4, r4, lbl_803D8340@l
/* 80358EA4 00355E04  4B FB C9 75 */	bl __nwa__FUlPCcPCc
/* 80358EA8 00355E08  7C 83 00 D0 */	neg r4, r3
/* 80358EAC 00355E0C  38 01 00 08 */	addi r0, r1, 8
/* 80358EB0 00355E10  7C 84 1B 78 */	or r4, r4, r3
/* 80358EB4 00355E14  38 BE 00 08 */	addi r5, r30, 8
/* 80358EB8 00355E18  54 84 0F FE */	srwi r4, r4, 0x1f
/* 80358EBC 00355E1C  90 61 00 0C */	stw r3, 0xc(r1)
/* 80358EC0 00355E20  7C 00 28 40 */	cmplw r0, r5
/* 80358EC4 00355E24  98 81 00 08 */	stb r4, 8(r1)
/* 80358EC8 00355E28  41 82 00 30 */	beq lbl_80358EF8
/* 80358ECC 00355E2C  88 05 00 00 */	lbz r0, 0(r5)
/* 80358ED0 00355E30  28 00 00 00 */	cmplwi r0, 0
/* 80358ED4 00355E34  41 82 00 0C */	beq lbl_80358EE0
/* 80358ED8 00355E38  80 7E 00 0C */	lwz r3, 0xc(r30)
/* 80358EDC 00355E3C  4B FB CA 55 */	bl Free__7CMemoryFPCv
lbl_80358EE0:
/* 80358EE0 00355E40  88 61 00 08 */	lbz r3, 8(r1)
/* 80358EE4 00355E44  38 00 00 00 */	li r0, 0
/* 80358EE8 00355E48  98 7E 00 08 */	stb r3, 8(r30)
/* 80358EEC 00355E4C  80 61 00 0C */	lwz r3, 0xc(r1)
/* 80358EF0 00355E50  90 7E 00 0C */	stw r3, 0xc(r30)
/* 80358EF4 00355E54  98 01 00 08 */	stb r0, 8(r1)
lbl_80358EF8:
/* 80358EF8 00355E58  88 01 00 08 */	lbz r0, 8(r1)
/* 80358EFC 00355E5C  28 00 00 00 */	cmplwi r0, 0
/* 80358F00 00355E60  41 82 00 0C */	beq lbl_80358F0C
/* 80358F04 00355E64  80 61 00 0C */	lwz r3, 0xc(r1)
/* 80358F08 00355E68  4B FB CA 29 */	bl Free__7CMemoryFPCv
lbl_80358F0C:
/* 80358F0C 00355E6C  80 9E 00 0C */	lwz r4, 0xc(r30)
/* 80358F10 00355E70  7F E3 FB 78 */	mr r3, r31
/* 80358F14 00355E74  7F A5 EB 78 */	mr r5, r29
/* 80358F18 00355E78  4B FE 60 3D */	bl Get__12CInputStreamFPvUl
/* 80358F1C 00355E7C  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80358F20 00355E80  7F C3 F3 78 */	mr r3, r30
/* 80358F24 00355E84  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80358F28 00355E88  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 80358F2C 00355E8C  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80358F30 00355E90  7C 08 03 A6 */	mtlr r0
/* 80358F34 00355E94  38 21 00 20 */	addi r1, r1, 0x20
/* 80358F38 00355E98  4E 80 00 20 */	blr

.global LocateHandle__12CMidiManagerFv
LocateHandle__12CMidiManagerFv:
/* 80358F3C 00355E9C  94 21 FF D0 */	stwu r1, -0x30(r1)
/* 80358F40 00355EA0  7C 08 02 A6 */	mflr r0
/* 80358F44 00355EA4  3C 80 80 5A */	lis r4, lbl_805A6828@ha
/* 80358F48 00355EA8  90 01 00 34 */	stw r0, 0x34(r1)
/* 80358F4C 00355EAC  BF 61 00 1C */	stmw r27, 0x1c(r1)
/* 80358F50 00355EB0  3B C4 68 28 */	addi r30, r4, lbl_805A6828@l
/* 80358F54 00355EB4  3B 9E 00 04 */	addi r28, r30, 4
/* 80358F58 00355EB8  7C 7F 1B 78 */	mr r31, r3
/* 80358F5C 00355EBC  3B 60 00 00 */	li r27, 0
/* 80358F60 00355EC0  7F 9D E3 78 */	mr r29, r28
/* 80358F64 00355EC4  48 00 00 2C */	b lbl_80358F90
lbl_80358F68:
/* 80358F68 00355EC8  7F A3 EB 78 */	mr r3, r29
/* 80358F6C 00355ECC  48 00 03 D1 */	bl IsAvailable__Q212CMidiManager12CMidiWrapperCFv
/* 80358F70 00355ED0  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 80358F74 00355ED4  41 82 00 14 */	beq lbl_80358F88
/* 80358F78 00355ED8  7F E3 FB 78 */	mr r3, r31
/* 80358F7C 00355EDC  7F 64 DB 78 */	mr r4, r27
/* 80358F80 00355EE0  4B F8 E8 ED */	bl __ct__10CSfxHandleFUi
/* 80358F84 00355EE4  48 00 00 88 */	b lbl_8035900C
lbl_80358F88:
/* 80358F88 00355EE8  3B BD 00 0C */	addi r29, r29, 0xc
/* 80358F8C 00355EEC  3B 7B 00 01 */	addi r27, r27, 1
lbl_80358F90:
/* 80358F90 00355EF0  80 1E 00 00 */	lwz r0, 0(r30)
/* 80358F94 00355EF4  7C 1B 00 00 */	cmpw r27, r0
/* 80358F98 00355EF8  41 80 FF D0 */	blt lbl_80358F68
/* 80358F9C 00355EFC  2C 00 00 03 */	cmpwi r0, 3
/* 80358FA0 00355F00  40 82 00 10 */	bne lbl_80358FB0
/* 80358FA4 00355F04  38 00 00 00 */	li r0, 0
/* 80358FA8 00355F08  90 1F 00 00 */	stw r0, 0(r31)
/* 80358FAC 00355F0C  48 00 00 60 */	b lbl_8035900C
lbl_80358FB0:
/* 80358FB0 00355F10  38 61 00 08 */	addi r3, r1, 8
/* 80358FB4 00355F14  48 00 03 A1 */	bl __ct__Q212CMidiManager12CMidiWrapperFv
/* 80358FB8 00355F18  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 80358FBC 00355F1C  80 03 68 28 */	lwz r0, lbl_805A6828@l(r3)
/* 80358FC0 00355F20  1C 00 00 0C */	mulli r0, r0, 0xc
/* 80358FC4 00355F24  7C 9C 02 15 */	add. r4, r28, r0
/* 80358FC8 00355F28  41 82 00 24 */	beq lbl_80358FEC
/* 80358FCC 00355F2C  80 61 00 08 */	lwz r3, 8(r1)
/* 80358FD0 00355F30  80 01 00 0C */	lwz r0, 0xc(r1)
/* 80358FD4 00355F34  90 64 00 00 */	stw r3, 0(r4)
/* 80358FD8 00355F38  A8 61 00 10 */	lha r3, 0x10(r1)
/* 80358FDC 00355F3C  90 04 00 04 */	stw r0, 4(r4)
/* 80358FE0 00355F40  88 01 00 12 */	lbz r0, 0x12(r1)
/* 80358FE4 00355F44  B0 64 00 08 */	sth r3, 8(r4)
/* 80358FE8 00355F48  98 04 00 0A */	stb r0, 0xa(r4)
lbl_80358FEC:
/* 80358FEC 00355F4C  3C 80 80 5A */	lis r4, lbl_805A6828@ha
/* 80358FF0 00355F50  7F E3 FB 78 */	mr r3, r31
/* 80358FF4 00355F54  38 A4 68 28 */	addi r5, r4, lbl_805A6828@l
/* 80358FF8 00355F58  80 85 00 00 */	lwz r4, 0(r5)
/* 80358FFC 00355F5C  38 84 00 01 */	addi r4, r4, 1
/* 80359000 00355F60  90 85 00 00 */	stw r4, 0(r5)
/* 80359004 00355F64  38 84 FF FF */	addi r4, r4, -1
/* 80359008 00355F68  4B F8 E8 65 */	bl __ct__10CSfxHandleFUi
lbl_8035900C:
/* 8035900C 00355F6C  BB 61 00 1C */	lmw r27, 0x1c(r1)
/* 80359010 00355F70  80 01 00 34 */	lwz r0, 0x34(r1)
/* 80359014 00355F74  7C 08 03 A6 */	mtlr r0
/* 80359018 00355F78  38 21 00 30 */	addi r1, r1, 0x30
/* 8035901C 00355F7C  4E 80 00 20 */	blr

.global StopAll__12CMidiManagerFv
StopAll__12CMidiManagerFv:
/* 80359020 00355F80  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 80359024 00355F84  7C 08 02 A6 */	mflr r0
/* 80359028 00355F88  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 8035902C 00355F8C  90 01 00 24 */	stw r0, 0x24(r1)
/* 80359030 00355F90  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 80359034 00355F94  3B E3 68 28 */	addi r31, r3, lbl_805A6828@l
/* 80359038 00355F98  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8035903C 00355F9C  3B DF 00 04 */	addi r30, r31, 4
/* 80359040 00355FA0  93 A1 00 14 */	stw r29, 0x14(r1)
/* 80359044 00355FA4  3B A0 00 00 */	li r29, 0
/* 80359048 00355FA8  48 00 00 2C */	b lbl_80359074
lbl_8035904C:
/* 8035904C 00355FAC  7F C3 F3 78 */	mr r3, r30
/* 80359050 00355FB0  48 00 02 ED */	bl IsAvailable__Q212CMidiManager12CMidiWrapperCFv
/* 80359054 00355FB4  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 80359058 00355FB8  40 82 00 14 */	bne lbl_8035906C
/* 8035905C 00355FBC  7F C3 F3 78 */	mr r3, r30
/* 80359060 00355FC0  48 00 02 ED */	bl GetManagerHandle__Q212CMidiManager12CMidiWrapperCFv
/* 80359064 00355FC4  38 80 00 00 */	li r4, 0
/* 80359068 00355FC8  48 00 00 35 */	bl Stop__12CMidiManagerFRC10CSfxHandleUs
lbl_8035906C:
/* 8035906C 00355FCC  3B DE 00 0C */	addi r30, r30, 0xc
/* 80359070 00355FD0  3B BD 00 01 */	addi r29, r29, 1
lbl_80359074:
/* 80359074 00355FD4  80 1F 00 00 */	lwz r0, 0(r31)
/* 80359078 00355FD8  7C 1D 00 00 */	cmpw r29, r0
/* 8035907C 00355FDC  41 80 FF D0 */	blt lbl_8035904C
/* 80359080 00355FE0  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80359084 00355FE4  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80359088 00355FE8  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8035908C 00355FEC  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80359090 00355FF0  7C 08 03 A6 */	mtlr r0
/* 80359094 00355FF4  38 21 00 20 */	addi r1, r1, 0x20
/* 80359098 00355FF8  4E 80 00 20 */	blr

.global Stop__12CMidiManagerFRC10CSfxHandleUs
Stop__12CMidiManagerFRC10CSfxHandleUs:
/* 8035909C 00355FFC  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 803590A0 00356000  7C 08 02 A6 */	mflr r0
/* 803590A4 00356004  90 01 00 24 */	stw r0, 0x24(r1)
/* 803590A8 00356008  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 803590AC 0035600C  93 C1 00 18 */	stw r30, 0x18(r1)
/* 803590B0 00356010  7C 9E 23 78 */	mr r30, r4
/* 803590B4 00356014  93 A1 00 14 */	stw r29, 0x14(r1)
/* 803590B8 00356018  7C 7D 1B 78 */	mr r29, r3
/* 803590BC 0035601C  80 03 00 00 */	lwz r0, 0(r3)
/* 803590C0 00356020  28 00 00 00 */	cmplwi r0, 0
/* 803590C4 00356024  41 82 00 7C */	beq lbl_80359140
/* 803590C8 00356028  54 00 05 3E */	clrlwi r0, r0, 0x14
/* 803590CC 0035602C  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 803590D0 00356030  1C 00 00 0C */	mulli r0, r0, 0xc
/* 803590D4 00356034  38 63 68 28 */	addi r3, r3, lbl_805A6828@l
/* 803590D8 00356038  3B E3 00 04 */	addi r31, r3, 4
/* 803590DC 0035603C  7C 7F 02 14 */	add r3, r31, r0
/* 803590E0 00356040  48 00 02 6D */	bl GetManagerHandle__Q212CMidiManager12CMidiWrapperCFv
/* 803590E4 00356044  80 9D 00 00 */	lwz r4, 0(r29)
/* 803590E8 00356048  80 03 00 00 */	lwz r0, 0(r3)
/* 803590EC 0035604C  7C 04 00 40 */	cmplw r4, r0
/* 803590F0 00356050  40 82 00 50 */	bne lbl_80359140
/* 803590F4 00356054  54 80 05 3E */	clrlwi r0, r4, 0x14
/* 803590F8 00356058  1C 00 00 0C */	mulli r0, r0, 0xc
/* 803590FC 0035605C  7C 7F 02 14 */	add r3, r31, r0
/* 80359100 00356060  48 00 02 45 */	bl GetAudioSysHandle__Q212CMidiManager12CMidiWrapperCFv
/* 80359104 00356064  57 C0 04 3F */	clrlwi. r0, r30, 0x10
/* 80359108 00356068  7C 65 1B 78 */	mr r5, r3
/* 8035910C 0035606C  40 82 00 0C */	bne lbl_80359118
/* 80359110 00356070  4B FF 17 F9 */	bl SeqStop__9CAudioSysFUl
/* 80359114 00356074  48 00 00 14 */	b lbl_80359128
lbl_80359118:
/* 80359118 00356078  7F C4 F3 78 */	mr r4, r30
/* 8035911C 0035607C  38 60 00 00 */	li r3, 0
/* 80359120 00356080  38 C0 00 01 */	li r6, 1
/* 80359124 00356084  4B FF 17 B9 */	bl SeqVolume__9CAudioSysFUcUsUlUc
lbl_80359128:
/* 80359128 00356088  80 1D 00 00 */	lwz r0, 0(r29)
/* 8035912C 0035608C  38 80 00 01 */	li r4, 1
/* 80359130 00356090  54 00 05 3E */	clrlwi r0, r0, 0x14
/* 80359134 00356094  1C 00 00 0C */	mulli r0, r0, 0xc
/* 80359138 00356098  7C 7F 02 14 */	add r3, r31, r0
/* 8035913C 0035609C  48 00 01 F1 */	bl SetAvailable__Q212CMidiManager12CMidiWrapperFb
lbl_80359140:
/* 80359140 003560A0  80 01 00 24 */	lwz r0, 0x24(r1)
/* 80359144 003560A4  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 80359148 003560A8  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8035914C 003560AC  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 80359150 003560B0  7C 08 03 A6 */	mtlr r0
/* 80359154 003560B4  38 21 00 20 */	addi r1, r1, 0x20
/* 80359158 003560B8  4E 80 00 20 */	blr

.global Play__12CMidiManagerFRCQ212CMidiManager9CMidiDataUsbs
Play__12CMidiManagerFRCQ212CMidiManager9CMidiDataUsbs:
/* 8035915C 003560BC  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 80359160 003560C0  7C 08 02 A6 */	mflr r0
/* 80359164 003560C4  90 01 00 44 */	stw r0, 0x44(r1)
/* 80359168 003560C8  BE A1 00 14 */	stmw r21, 0x14(r1)
/* 8035916C 003560CC  7C 79 1B 78 */	mr r25, r3
/* 80359170 003560D0  7C 9A 23 78 */	mr r26, r4
/* 80359174 003560D4  7C BB 2B 78 */	mr r27, r5
/* 80359178 003560D8  7C D5 33 78 */	mr r21, r6
/* 8035917C 003560DC  7C FC 3B 78 */	mr r28, r7
/* 80359180 003560E0  38 61 00 08 */	addi r3, r1, 8
/* 80359184 003560E4  3B C0 00 00 */	li r30, 0
/* 80359188 003560E8  3B A0 00 00 */	li r29, 0
/* 8035918C 003560EC  4B FF FD B1 */	bl LocateHandle__12CMidiManagerFv
/* 80359190 003560F0  80 01 00 08 */	lwz r0, 8(r1)
/* 80359194 003560F4  28 00 00 00 */	cmplwi r0, 0
/* 80359198 003560F8  90 01 00 0C */	stw r0, 0xc(r1)
/* 8035919C 003560FC  40 82 00 10 */	bne lbl_803591AC
/* 803591A0 00356100  38 00 00 00 */	li r0, 0
/* 803591A4 00356104  90 19 00 00 */	stw r0, 0(r25)
/* 803591A8 00356108  48 00 01 54 */	b lbl_803592FC
lbl_803591AC:
/* 803591AC 0035610C  54 00 05 3E */	clrlwi r0, r0, 0x14
/* 803591B0 00356110  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 803591B4 00356114  1C 00 00 0C */	mulli r0, r0, 0xc
/* 803591B8 00356118  38 80 00 00 */	li r4, 0
/* 803591BC 0035611C  38 63 68 28 */	addi r3, r3, lbl_805A6828@l
/* 803591C0 00356120  3A C3 00 04 */	addi r22, r3, 4
/* 803591C4 00356124  7F F6 02 14 */	add r31, r22, r0
/* 803591C8 00356128  7F E3 FB 78 */	mr r3, r31
/* 803591CC 0035612C  48 00 01 61 */	bl SetAvailable__Q212CMidiManager12CMidiWrapperFb
/* 803591D0 00356130  7F E3 FB 78 */	mr r3, r31
/* 803591D4 00356134  38 81 00 0C */	addi r4, r1, 0xc
/* 803591D8 00356138  48 00 01 41 */	bl SetMidiHandle__Q212CMidiManager12CMidiWrapperFRC10CSfxHandle
/* 803591DC 0035613C  56 A0 06 3F */	clrlwi. r0, r21, 0x18
/* 803591E0 00356140  41 82 00 84 */	beq lbl_80359264
/* 803591E4 00356144  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 803591E8 00356148  3A A0 00 00 */	li r21, 0
/* 803591EC 0035614C  3B 03 68 28 */	addi r24, r3, lbl_805A6828@l
/* 803591F0 00356150  48 00 00 68 */	b lbl_80359258
lbl_803591F4:
/* 803591F4 00356154  7E C3 B3 78 */	mr r3, r22
/* 803591F8 00356158  48 00 01 45 */	bl IsAvailable__Q212CMidiManager12CMidiWrapperCFv
/* 803591FC 0035615C  54 60 06 3F */	clrlwi. r0, r3, 0x18
/* 80359200 00356160  40 82 00 50 */	bne lbl_80359250
/* 80359204 00356164  AA FA 00 00 */	lha r23, 0(r26)
/* 80359208 00356168  7E C3 B3 78 */	mr r3, r22
/* 8035920C 0035616C  48 00 01 29 */	bl GetSongId__Q212CMidiManager12CMidiWrapperCFv
/* 80359210 00356170  7C 60 07 34 */	extsh r0, r3
/* 80359214 00356174  7C 17 00 00 */	cmpw r23, r0
/* 80359218 00356178  40 82 00 28 */	bne lbl_80359240
/* 8035921C 0035617C  7E C3 B3 78 */	mr r3, r22
/* 80359220 00356180  3B C0 00 01 */	li r30, 1
/* 80359224 00356184  48 00 01 21 */	bl GetAudioSysHandle__Q212CMidiManager12CMidiWrapperCFv
/* 80359228 00356188  7C 60 1B 78 */	mr r0, r3
/* 8035922C 0035618C  7E C3 B3 78 */	mr r3, r22
/* 80359230 00356190  7C 1D 03 78 */	mr r29, r0
/* 80359234 00356194  38 80 00 01 */	li r4, 1
/* 80359238 00356198  48 00 00 F5 */	bl SetAvailable__Q212CMidiManager12CMidiWrapperFb
/* 8035923C 0035619C  48 00 00 14 */	b lbl_80359250
lbl_80359240:
/* 80359240 003561A0  7E C3 B3 78 */	mr r3, r22
/* 80359244 003561A4  48 00 01 09 */	bl GetManagerHandle__Q212CMidiManager12CMidiWrapperCFv
/* 80359248 003561A8  7F 64 DB 78 */	mr r4, r27
/* 8035924C 003561AC  4B FF FE 51 */	bl Stop__12CMidiManagerFRC10CSfxHandleUs
lbl_80359250:
/* 80359250 003561B0  3A D6 00 0C */	addi r22, r22, 0xc
/* 80359254 003561B4  3A B5 00 01 */	addi r21, r21, 1
lbl_80359258:
/* 80359258 003561B8  80 18 00 00 */	lwz r0, 0(r24)
/* 8035925C 003561BC  7C 15 00 00 */	cmpw r21, r0
/* 80359260 003561C0  41 80 FF 94 */	blt lbl_803591F4
lbl_80359264:
/* 80359264 003561C4  57 C0 06 3F */	clrlwi. r0, r30, 0x18
/* 80359268 003561C8  41 82 00 20 */	beq lbl_80359288
/* 8035926C 003561CC  7F E3 FB 78 */	mr r3, r31
/* 80359270 003561D0  7F A4 EB 78 */	mr r4, r29
/* 80359274 003561D4  48 00 00 B1 */	bl SetAudioSysHandle__Q212CMidiManager12CMidiWrapperFUl
/* 80359278 003561D8  A8 9A 00 00 */	lha r4, 0(r26)
/* 8035927C 003561DC  7F E3 FB 78 */	mr r3, r31
/* 80359280 003561E0  48 00 00 91 */	bl SetSongId__Q212CMidiManager12CMidiWrapperFs
/* 80359284 003561E4  48 00 00 70 */	b lbl_803592F4
lbl_80359288:
/* 80359288 003561E8  A8 7A 00 02 */	lha r3, 2(r26)
/* 8035928C 003561EC  38 C0 00 00 */	li r6, 0
/* 80359290 003561F0  A8 1A 00 00 */	lha r0, 0(r26)
/* 80359294 003561F4  38 E0 00 00 */	li r7, 0
/* 80359298 003561F8  80 BA 00 0C */	lwz r5, 0xc(r26)
/* 8035929C 003561FC  54 63 04 3E */	clrlwi r3, r3, 0x10
/* 803592A0 00356200  54 04 04 3E */	clrlwi r4, r0, 0x10
/* 803592A4 00356204  4B FF 16 85 */	bl SeqPlayEx__9CAudioSysFUsUsPvP12SND_PLAYPARAUc
/* 803592A8 00356208  57 60 04 3F */	clrlwi. r0, r27, 0x10
/* 803592AC 0035620C  7C 75 1B 78 */	mr r21, r3
/* 803592B0 00356210  41 82 00 18 */	beq lbl_803592C8
/* 803592B4 00356214  7E A5 AB 78 */	mr r5, r21
/* 803592B8 00356218  38 60 00 00 */	li r3, 0
/* 803592BC 0035621C  38 80 00 00 */	li r4, 0
/* 803592C0 00356220  38 C0 00 00 */	li r6, 0
/* 803592C4 00356224  4B FF 16 19 */	bl SeqVolume__9CAudioSysFUcUsUlUc
lbl_803592C8:
/* 803592C8 00356228  7F 64 DB 78 */	mr r4, r27
/* 803592CC 0035622C  7E A5 AB 78 */	mr r5, r21
/* 803592D0 00356230  57 83 06 3E */	clrlwi r3, r28, 0x18
/* 803592D4 00356234  38 C0 00 00 */	li r6, 0
/* 803592D8 00356238  4B FF 16 05 */	bl SeqVolume__9CAudioSysFUcUsUlUc
/* 803592DC 0035623C  7F E3 FB 78 */	mr r3, r31
/* 803592E0 00356240  7E A4 AB 78 */	mr r4, r21
/* 803592E4 00356244  48 00 00 41 */	bl SetAudioSysHandle__Q212CMidiManager12CMidiWrapperFUl
/* 803592E8 00356248  A8 9A 00 00 */	lha r4, 0(r26)
/* 803592EC 0035624C  7F E3 FB 78 */	mr r3, r31
/* 803592F0 00356250  48 00 00 21 */	bl SetSongId__Q212CMidiManager12CMidiWrapperFs
lbl_803592F4:
/* 803592F4 00356254  80 01 00 0C */	lwz r0, 0xc(r1)
/* 803592F8 00356258  90 19 00 00 */	stw r0, 0(r25)
lbl_803592FC:
/* 803592FC 0035625C  BA A1 00 14 */	lmw r21, 0x14(r1)
/* 80359300 00356260  80 01 00 44 */	lwz r0, 0x44(r1)
/* 80359304 00356264  7C 08 03 A6 */	mtlr r0
/* 80359308 00356268  38 21 00 40 */	addi r1, r1, 0x40
/* 8035930C 0035626C  4E 80 00 20 */	blr

.global SetSongId__Q212CMidiManager12CMidiWrapperFs
SetSongId__Q212CMidiManager12CMidiWrapperFs:
/* 80359310 00356270  B0 83 00 08 */	sth r4, 8(r3)
/* 80359314 00356274  4E 80 00 20 */	blr

.global SetMidiHandle__Q212CMidiManager12CMidiWrapperFRC10CSfxHandle
SetMidiHandle__Q212CMidiManager12CMidiWrapperFRC10CSfxHandle:
/* 80359318 00356278  80 04 00 00 */	lwz r0, 0(r4)
/* 8035931C 0035627C  90 03 00 04 */	stw r0, 4(r3)
/* 80359320 00356280  4E 80 00 20 */	blr

.global SetAudioSysHandle__Q212CMidiManager12CMidiWrapperFUl
SetAudioSysHandle__Q212CMidiManager12CMidiWrapperFUl:
/* 80359324 00356284  90 83 00 00 */	stw r4, 0(r3)
/* 80359328 00356288  4E 80 00 20 */	blr

.global SetAvailable__Q212CMidiManager12CMidiWrapperFb
SetAvailable__Q212CMidiManager12CMidiWrapperFb:
/* 8035932C 0035628C  98 83 00 0A */	stb r4, 0xa(r3)
/* 80359330 00356290  4E 80 00 20 */	blr

.global GetSongId__Q212CMidiManager12CMidiWrapperCFv
GetSongId__Q212CMidiManager12CMidiWrapperCFv:
/* 80359334 00356294  A8 63 00 08 */	lha r3, 8(r3)
/* 80359338 00356298  4E 80 00 20 */	blr

.global IsAvailable__Q212CMidiManager12CMidiWrapperCFv
IsAvailable__Q212CMidiManager12CMidiWrapperCFv:
/* 8035933C 0035629C  88 63 00 0A */	lbz r3, 0xa(r3)
/* 80359340 003562A0  4E 80 00 20 */	blr

.global GetAudioSysHandle__Q212CMidiManager12CMidiWrapperCFv
GetAudioSysHandle__Q212CMidiManager12CMidiWrapperCFv:
/* 80359344 003562A4  80 63 00 00 */	lwz r3, 0(r3)
/* 80359348 003562A8  4E 80 00 20 */	blr

.global GetManagerHandle__Q212CMidiManager12CMidiWrapperCFv
GetManagerHandle__Q212CMidiManager12CMidiWrapperCFv:
/* 8035934C 003562AC  38 63 00 04 */	addi r3, r3, 4
/* 80359350 003562B0  4E 80 00 20 */	blr

.global __ct__Q212CMidiManager12CMidiWrapperFv
__ct__Q212CMidiManager12CMidiWrapperFv:
/* 80359354 003562B4  38 80 00 00 */	li r4, 0
/* 80359358 003562B8  38 00 00 01 */	li r0, 1
/* 8035935C 003562BC  90 83 00 00 */	stw r4, 0(r3)
/* 80359360 003562C0  90 83 00 04 */	stw r4, 4(r3)
/* 80359364 003562C4  98 03 00 0A */	stb r0, 0xa(r3)
/* 80359368 003562C8  4E 80 00 20 */	blr

.global __sinit_CMidiManager_cpp
__sinit_CMidiManager_cpp:
/* 8035936C 003562CC  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80359370 003562D0  7C 08 02 A6 */	mflr r0
/* 80359374 003562D4  3C 80 80 36 */	lis r4, "__dt__Q24rstl49reserved_vector<Q212CMidiManager12CMidiWrapper,3>Fv"@ha
/* 80359378 003562D8  3C A0 80 54 */	lis r5, lbl_80540010@ha
/* 8035937C 003562DC  90 01 00 14 */	stw r0, 0x14(r1)
/* 80359380 003562E0  38 00 00 00 */	li r0, 0
/* 80359384 003562E4  3C 60 80 5A */	lis r3, lbl_805A6828@ha
/* 80359388 003562E8  38 84 93 A8 */	addi r4, r4, "__dt__Q24rstl49reserved_vector<Q212CMidiManager12CMidiWrapper,3>Fv"@l
/* 8035938C 003562EC  94 03 68 28 */	stwu r0, lbl_805A6828@l(r3)
/* 80359390 003562F0  38 A5 00 10 */	addi r5, r5, lbl_80540010@l
/* 80359394 003562F4  48 03 02 F9 */	bl __register_global_object
/* 80359398 003562F8  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8035939C 003562FC  7C 08 03 A6 */	mtlr r0
/* 803593A0 00356300  38 21 00 10 */	addi r1, r1, 0x10
/* 803593A4 00356304  4E 80 00 20 */	blr

.global "__dt__Q24rstl49reserved_vector<Q212CMidiManager12CMidiWrapper,3>Fv"
"__dt__Q24rstl49reserved_vector<Q212CMidiManager12CMidiWrapper,3>Fv":
/* 803593A8 00356308  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 803593AC 0035630C  7C 08 02 A6 */	mflr r0
/* 803593B0 00356310  90 01 00 14 */	stw r0, 0x14(r1)
/* 803593B4 00356314  93 E1 00 0C */	stw r31, 0xc(r1)
/* 803593B8 00356318  7C 7F 1B 79 */	or. r31, r3, r3
/* 803593BC 0035631C  41 82 00 68 */	beq lbl_80359424
/* 803593C0 00356320  80 DF 00 00 */	lwz r6, 0(r31)
/* 803593C4 00356324  38 60 00 00 */	li r3, 0
/* 803593C8 00356328  2C 06 00 00 */	cmpwi r6, 0
/* 803593CC 0035632C  40 81 00 40 */	ble lbl_8035940C
/* 803593D0 00356330  2C 06 00 08 */	cmpwi r6, 8
/* 803593D4 00356334  38 A6 FF F8 */	addi r5, r6, -8
/* 803593D8 00356338  40 81 00 20 */	ble lbl_803593F8
/* 803593DC 0035633C  38 05 00 07 */	addi r0, r5, 7
/* 803593E0 00356340  54 00 E8 FE */	srwi r0, r0, 3
/* 803593E4 00356344  7C 09 03 A6 */	mtctr r0
/* 803593E8 00356348  2C 05 00 00 */	cmpwi r5, 0
/* 803593EC 0035634C  40 81 00 0C */	ble lbl_803593F8
lbl_803593F0:
/* 803593F0 00356350  38 63 00 08 */	addi r3, r3, 8
/* 803593F4 00356354  42 00 FF FC */	bdnz lbl_803593F0
lbl_803593F8:
/* 803593F8 00356358  7C 03 30 50 */	subf r0, r3, r6
/* 803593FC 0035635C  7C 09 03 A6 */	mtctr r0
/* 80359400 00356360  7C 03 30 00 */	cmpw r3, r6
/* 80359404 00356364  40 80 00 08 */	bge lbl_8035940C
lbl_80359408:
/* 80359408 00356368  42 00 00 00 */	bdnz lbl_80359408
lbl_8035940C:
/* 8035940C 0035636C  38 60 00 00 */	li r3, 0
/* 80359410 00356370  7C 80 07 35 */	extsh. r0, r4
/* 80359414 00356374  90 7F 00 00 */	stw r3, 0(r31)
/* 80359418 00356378  40 81 00 0C */	ble lbl_80359424
/* 8035941C 0035637C  7F E3 FB 78 */	mr r3, r31
/* 80359420 00356380  4B FB C5 11 */	bl Free__7CMemoryFPCv
lbl_80359424:
/* 80359424 00356384  80 01 00 14 */	lwz r0, 0x14(r1)
/* 80359428 00356388  7F E3 FB 78 */	mr r3, r31
/* 8035942C 0035638C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 80359430 00356390  7C 08 03 A6 */	mtlr r0
/* 80359434 00356394  38 21 00 10 */	addi r1, r1, 0x10
/* 80359438 00356398  4E 80 00 20 */	blr

.section .rodata
.global lbl_803D8340
lbl_803D8340:
	# ROM: 0x3D5340
	.asciz "??(??)"
	.balign 4

