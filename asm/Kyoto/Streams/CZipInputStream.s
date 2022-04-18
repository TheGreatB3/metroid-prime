.include "macros.inc"

.section .data

.global lbl_803EE3C8
lbl_803EE3C8:
	# ROM: 0x3EB3C8
	.4byte 0
	.4byte 0
	.4byte __dt__15CZipInputStreamFv
	.4byte Read__15CZipInputStreamFPvUl

.section .text, "ax"  # 0x80003640 - 0x803CB1C0

.global Read__15CZipInputStreamFPvUl
Read__15CZipInputStreamFPvUl:
/* 8033F7AC 0033C70C  94 21 FF E0 */	stwu r1, -0x20(r1)
/* 8033F7B0 0033C710  7C 08 02 A6 */	mflr r0
/* 8033F7B4 0033C714  90 01 00 24 */	stw r0, 0x24(r1)
/* 8033F7B8 0033C718  93 E1 00 1C */	stw r31, 0x1c(r1)
/* 8033F7BC 0033C71C  93 C1 00 18 */	stw r30, 0x18(r1)
/* 8033F7C0 0033C720  7C BE 2B 78 */	mr r30, r5
/* 8033F7C4 0033C724  93 A1 00 14 */	stw r29, 0x14(r1)
/* 8033F7C8 0033C728  7C 7D 1B 78 */	mr r29, r3
/* 8033F7CC 0033C72C  80 63 00 30 */	lwz r3, 0x30(r3)
/* 8033F7D0 0033C730  90 83 00 0C */	stw r4, 0xc(r3)
/* 8033F7D4 0033C734  80 7D 00 30 */	lwz r3, 0x30(r29)
/* 8033F7D8 0033C738  93 C3 00 10 */	stw r30, 0x10(r3)
/* 8033F7DC 0033C73C  83 FD 00 30 */	lwz r31, 0x30(r29)
/* 8033F7E0 0033C740  80 1F 00 04 */	lwz r0, 4(r31)
/* 8033F7E4 0033C744  28 00 00 00 */	cmplwi r0, 0
/* 8033F7E8 0033C748  40 82 00 24 */	bne lbl_8033F80C
/* 8033F7EC 0033C74C  80 7D 00 2C */	lwz r3, 0x2c(r29)
/* 8033F7F0 0033C750  38 A0 10 00 */	li r5, 0x1000
/* 8033F7F4 0033C754  80 9D 00 24 */	lwz r4, 0x24(r29)
/* 8033F7F8 0033C758  4B FF F6 89 */	bl ReadBytes__12CInputStreamFPvUl
/* 8033F7FC 0033C75C  90 7F 00 04 */	stw r3, 4(r31)
/* 8033F800 0033C760  80 1D 00 24 */	lwz r0, 0x24(r29)
/* 8033F804 0033C764  80 7D 00 30 */	lwz r3, 0x30(r29)
/* 8033F808 0033C768  90 03 00 00 */	stw r0, 0(r3)
lbl_8033F80C:
/* 8033F80C 0033C76C  80 7D 00 30 */	lwz r3, 0x30(r29)
/* 8033F810 0033C770  38 80 00 00 */	li r4, 0
/* 8033F814 0033C774  48 00 3B 49 */	bl inflate
/* 8033F818 0033C778  80 7D 00 30 */	lwz r3, 0x30(r29)
/* 8033F81C 0033C77C  80 03 00 10 */	lwz r0, 0x10(r3)
/* 8033F820 0033C780  7C 60 F0 50 */	subf r3, r0, r30
/* 8033F824 0033C784  83 E1 00 1C */	lwz r31, 0x1c(r1)
/* 8033F828 0033C788  83 C1 00 18 */	lwz r30, 0x18(r1)
/* 8033F82C 0033C78C  83 A1 00 14 */	lwz r29, 0x14(r1)
/* 8033F830 0033C790  80 01 00 24 */	lwz r0, 0x24(r1)
/* 8033F834 0033C794  7C 08 03 A6 */	mtlr r0
/* 8033F838 0033C798  38 21 00 20 */	addi r1, r1, 0x20
/* 8033F83C 0033C79C  4E 80 00 20 */	blr 

.global __dt__15CZipInputStreamFv
__dt__15CZipInputStreamFv:
/* 8033F840 0033C7A0  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8033F844 0033C7A4  7C 08 02 A6 */	mflr r0
/* 8033F848 0033C7A8  90 01 00 14 */	stw r0, 0x14(r1)
/* 8033F84C 0033C7AC  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8033F850 0033C7B0  7C 9F 23 78 */	mr r31, r4
/* 8033F854 0033C7B4  93 C1 00 08 */	stw r30, 8(r1)
/* 8033F858 0033C7B8  7C 7E 1B 79 */	or. r30, r3, r3
/* 8033F85C 0033C7BC  41 82 00 88 */	beq lbl_8033F8E4
/* 8033F860 0033C7C0  3C 60 80 3F */	lis r3, lbl_803EE3C8@ha
/* 8033F864 0033C7C4  38 03 E3 C8 */	addi r0, r3, lbl_803EE3C8@l
/* 8033F868 0033C7C8  90 1E 00 00 */	stw r0, 0(r30)
/* 8033F86C 0033C7CC  80 7E 00 30 */	lwz r3, 0x30(r30)
/* 8033F870 0033C7D0  48 00 42 D1 */	bl inflateEnd
/* 8033F874 0033C7D4  34 1E 00 30 */	addic. r0, r30, 0x30
/* 8033F878 0033C7D8  41 82 00 0C */	beq lbl_8033F884
/* 8033F87C 0033C7DC  80 7E 00 30 */	lwz r3, 0x30(r30)
/* 8033F880 0033C7E0  4B FD 60 B1 */	bl Free__7CMemoryFPCv
lbl_8033F884:
/* 8033F884 0033C7E4  34 1E 00 28 */	addic. r0, r30, 0x28
/* 8033F888 0033C7E8  41 82 00 30 */	beq lbl_8033F8B8
/* 8033F88C 0033C7EC  88 1E 00 28 */	lbz r0, 0x28(r30)
/* 8033F890 0033C7F0  28 00 00 00 */	cmplwi r0, 0
/* 8033F894 0033C7F4  41 82 00 24 */	beq lbl_8033F8B8
/* 8033F898 0033C7F8  80 7E 00 2C */	lwz r3, 0x2c(r30)
/* 8033F89C 0033C7FC  28 03 00 00 */	cmplwi r3, 0
/* 8033F8A0 0033C800  41 82 00 18 */	beq lbl_8033F8B8
/* 8033F8A4 0033C804  81 83 00 00 */	lwz r12, 0(r3)
/* 8033F8A8 0033C808  38 80 00 01 */	li r4, 1
/* 8033F8AC 0033C80C  81 8C 00 08 */	lwz r12, 8(r12)
/* 8033F8B0 0033C810  7D 89 03 A6 */	mtctr r12
/* 8033F8B4 0033C814  4E 80 04 21 */	bctrl 
lbl_8033F8B8:
/* 8033F8B8 0033C818  34 1E 00 24 */	addic. r0, r30, 0x24
/* 8033F8BC 0033C81C  41 82 00 0C */	beq lbl_8033F8C8
/* 8033F8C0 0033C820  80 7E 00 24 */	lwz r3, 0x24(r30)
/* 8033F8C4 0033C824  4B FD 60 6D */	bl Free__7CMemoryFPCv
lbl_8033F8C8:
/* 8033F8C8 0033C828  7F C3 F3 78 */	mr r3, r30
/* 8033F8CC 0033C82C  38 80 00 00 */	li r4, 0
/* 8033F8D0 0033C830  4B FF F7 DD */	bl __dt__12CInputStreamFv
/* 8033F8D4 0033C834  7F E0 07 35 */	extsh. r0, r31
/* 8033F8D8 0033C838  40 81 00 0C */	ble lbl_8033F8E4
/* 8033F8DC 0033C83C  7F C3 F3 78 */	mr r3, r30
/* 8033F8E0 0033C840  4B FD 60 51 */	bl Free__7CMemoryFPCv
lbl_8033F8E4:
/* 8033F8E4 0033C844  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8033F8E8 0033C848  7F C3 F3 78 */	mr r3, r30
/* 8033F8EC 0033C84C  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8033F8F0 0033C850  83 C1 00 08 */	lwz r30, 8(r1)
/* 8033F8F4 0033C854  7C 08 03 A6 */	mtlr r0
/* 8033F8F8 0033C858  38 21 00 10 */	addi r1, r1, 0x10
/* 8033F8FC 0033C85C  4E 80 00 20 */	blr 

.global "__ct__15CZipInputStreamFQ24rstl24auto_ptr<12CInputStream>"
"__ct__15CZipInputStreamFQ24rstl24auto_ptr<12CInputStream>":
/* 8033F900 0033C860  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 8033F904 0033C864  7C 08 02 A6 */	mflr r0
/* 8033F908 0033C868  90 01 00 14 */	stw r0, 0x14(r1)
/* 8033F90C 0033C86C  93 E1 00 0C */	stw r31, 0xc(r1)
/* 8033F910 0033C870  7C 7F 1B 78 */	mr r31, r3
/* 8033F914 0033C874  93 C1 00 08 */	stw r30, 8(r1)
/* 8033F918 0033C878  7C 9E 23 78 */	mr r30, r4
/* 8033F91C 0033C87C  38 80 10 00 */	li r4, 0x1000
/* 8033F920 0033C880  4B FF F8 2D */	bl __ct__12CInputStreamFi
/* 8033F924 0033C884  3C 60 80 3F */	lis r3, lbl_803EE3C8@ha
/* 8033F928 0033C888  3C 80 80 3D */	lis r4, lbl_803D7A78@ha
/* 8033F92C 0033C88C  38 03 E3 C8 */	addi r0, r3, lbl_803EE3C8@l
/* 8033F930 0033C890  38 A0 00 00 */	li r5, 0
/* 8033F934 0033C894  90 1F 00 00 */	stw r0, 0(r31)
/* 8033F938 0033C898  38 60 10 00 */	li r3, 0x1000
/* 8033F93C 0033C89C  38 84 7A 78 */	addi r4, r4, lbl_803D7A78@l
/* 8033F940 0033C8A0  4B FD 5E D9 */	bl __nwa__FUlPCcPCc
/* 8033F944 0033C8A4  90 7F 00 24 */	stw r3, 0x24(r31)
/* 8033F948 0033C8A8  3C 60 80 3D */	lis r3, lbl_803D7A78@ha
/* 8033F94C 0033C8AC  38 83 7A 78 */	addi r4, r3, lbl_803D7A78@l
/* 8033F950 0033C8B0  38 00 00 00 */	li r0, 0
/* 8033F954 0033C8B4  88 DE 00 00 */	lbz r6, 0(r30)
/* 8033F958 0033C8B8  38 60 00 38 */	li r3, 0x38
/* 8033F95C 0033C8BC  38 A0 00 00 */	li r5, 0
/* 8033F960 0033C8C0  98 DF 00 28 */	stb r6, 0x28(r31)
/* 8033F964 0033C8C4  80 DE 00 04 */	lwz r6, 4(r30)
/* 8033F968 0033C8C8  90 DF 00 2C */	stw r6, 0x2c(r31)
/* 8033F96C 0033C8CC  98 1E 00 00 */	stb r0, 0(r30)
/* 8033F970 0033C8D0  4B FD 5E FD */	bl __nw__FUlPCcPCc
/* 8033F974 0033C8D4  90 7F 00 30 */	stw r3, 0x30(r31)
/* 8033F978 0033C8D8  3C A0 80 34 */	lis r5, Alloc__11CZipSupportFPvUiUi@ha
/* 8033F97C 0033C8DC  3C 80 80 34 */	lis r4, Free__11CZipSupportFPvPv@ha
/* 8033F980 0033C8E0  3C 60 80 3D */	lis r3, lbl_803D7A78@ha
/* 8033F984 0033C8E4  81 3F 00 30 */	lwz r9, 0x30(r31)
/* 8033F988 0033C8E8  38 63 7A 78 */	addi r3, r3, lbl_803D7A78@l
/* 8033F98C 0033C8EC  81 1F 00 24 */	lwz r8, 0x24(r31)
/* 8033F990 0033C8F0  38 E0 00 00 */	li r7, 0
/* 8033F994 0033C8F4  38 C5 FA 18 */	addi r6, r5, Alloc__11CZipSupportFPvUiUi@l
/* 8033F998 0033C8F8  38 04 F9 EC */	addi r0, r4, Free__11CZipSupportFPvPv@l
/* 8033F99C 0033C8FC  91 09 00 00 */	stw r8, 0(r9)
/* 8033F9A0 0033C900  38 83 00 07 */	addi r4, r3, 7
/* 8033F9A4 0033C904  38 A0 00 38 */	li r5, 0x38
/* 8033F9A8 0033C908  80 7F 00 30 */	lwz r3, 0x30(r31)
/* 8033F9AC 0033C90C  90 E3 00 04 */	stw r7, 4(r3)
/* 8033F9B0 0033C910  80 7F 00 30 */	lwz r3, 0x30(r31)
/* 8033F9B4 0033C914  90 C3 00 20 */	stw r6, 0x20(r3)
/* 8033F9B8 0033C918  80 7F 00 30 */	lwz r3, 0x30(r31)
/* 8033F9BC 0033C91C  90 03 00 24 */	stw r0, 0x24(r3)
/* 8033F9C0 0033C920  80 7F 00 30 */	lwz r3, 0x30(r31)
/* 8033F9C4 0033C924  90 E3 00 28 */	stw r7, 0x28(r3)
/* 8033F9C8 0033C928  80 7F 00 30 */	lwz r3, 0x30(r31)
/* 8033F9CC 0033C92C  48 00 3F 95 */	bl inflateInit2_
/* 8033F9D0 0033C930  80 01 00 14 */	lwz r0, 0x14(r1)
/* 8033F9D4 0033C934  7F E3 FB 78 */	mr r3, r31
/* 8033F9D8 0033C938  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 8033F9DC 0033C93C  83 C1 00 08 */	lwz r30, 8(r1)
/* 8033F9E0 0033C940  7C 08 03 A6 */	mtlr r0
/* 8033F9E4 0033C944  38 21 00 10 */	addi r1, r1, 0x10
/* 8033F9E8 0033C948  4E 80 00 20 */	blr
