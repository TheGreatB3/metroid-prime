.include "macros.inc"

.section .data
.balign 8
lbl_803EFA50:
	.asciz "DSP is booting task: 0x%08X\n"
.balign 4
	.asciz "__DSP_boot_task()  : IRAM MMEM ADDR: 0x%08X\n"
.balign 4
	.asciz "__DSP_boot_task()  : IRAM DSP ADDR : 0x%08X\n"
.balign 4
	.asciz "__DSP_boot_task()  : IRAM LENGTH   : 0x%08X\n"
.balign 4
	.asciz "__DSP_boot_task()  : DRAM MMEM ADDR: 0x%08X\n"
.balign 4
	.asciz "__DSP_boot_task()  : Start Vector  : 0x%08X\n"
.balign 4
	.asciz "__DSP_add_task() : Added task    : 0x%08X\n"
	
.section .text, "ax"

.global __DSPHandler
__DSPHandler:
/* 8036FC20 0036CB80  7C 08 02 A6 */	mflr r0
/* 8036FC24 0036CB84  3C 60 CC 00 */	lis r3, 0xCC005000@ha
/* 8036FC28 0036CB88  90 01 00 04 */	stw r0, 4(r1)
/* 8036FC2C 0036CB8C  38 63 50 00 */	addi r3, r3, 0xCC005000@l
/* 8036FC30 0036CB90  38 00 FF D7 */	li r0, -41
/* 8036FC34 0036CB94  94 21 FD 18 */	stwu r1, -0x2e8(r1)
/* 8036FC38 0036CB98  93 E1 02 E4 */	stw r31, 0x2e4(r1)
/* 8036FC3C 0036CB9C  3B E4 00 00 */	addi r31, r4, 0
/* 8036FC40 0036CBA0  A0 A3 00 0A */	lhz r5, 0xa(r3)
/* 8036FC44 0036CBA4  7C A0 00 38 */	and r0, r5, r0
/* 8036FC48 0036CBA8  60 00 00 80 */	ori r0, r0, 0x80
/* 8036FC4C 0036CBAC  B0 03 00 0A */	sth r0, 0xa(r3)
/* 8036FC50 0036CBB0  38 61 00 10 */	addi r3, r1, 0x10
/* 8036FC54 0036CBB4  48 00 F9 C1 */	bl OSClearContext
/* 8036FC58 0036CBB8  38 61 00 10 */	addi r3, r1, 0x10
/* 8036FC5C 0036CBBC  48 00 F7 C1 */	bl OSSetCurrentContext
lbl_8036FC60:
/* 8036FC60 0036CBC0  4B FF FD 69 */	bl DSPCheckMailFromDSP
/* 8036FC64 0036CBC4  28 03 00 00 */	cmplwi r3, 0
/* 8036FC68 0036CBC8  41 82 FF F8 */	beq lbl_8036FC60
/* 8036FC6C 0036CBCC  4B FF FD 6D */	bl DSPReadMailFromDSP
/* 8036FC70 0036CBD0  80 AD AB A4 */	lwz r5, lbl_805A9764@sda21(r13)
/* 8036FC74 0036CBD4  80 05 00 08 */	lwz r0, 8(r5)
/* 8036FC78 0036CBD8  54 00 07 BD */	rlwinm. r0, r0, 0, 0x1e, 0x1e
/* 8036FC7C 0036CBDC  41 82 00 18 */	beq lbl_8036FC94
/* 8036FC80 0036CBE0  3C 03 23 2F */	addis r0, r3, 0x232f
/* 8036FC84 0036CBE4  28 00 00 02 */	cmplwi r0, 2
/* 8036FC88 0036CBE8  40 82 00 0C */	bne lbl_8036FC94
/* 8036FC8C 0036CBEC  3C 60 DC D1 */	lis r3, 0xDCD10003@ha
/* 8036FC90 0036CBF0  38 63 00 03 */	addi r3, r3, 0xDCD10003@l
lbl_8036FC94:
/* 8036FC94 0036CBF4  3C 80 DC D1 */	lis r4, 0xDCD10002@ha
/* 8036FC98 0036CBF8  38 04 00 02 */	addi r0, r4, 0xDCD10002@l
/* 8036FC9C 0036CBFC  7C 03 00 00 */	cmpw r3, r0
/* 8036FCA0 0036CC00  41 82 00 74 */	beq lbl_8036FD14
/* 8036FCA4 0036CC04  40 80 00 14 */	bge lbl_8036FCB8
/* 8036FCA8 0036CC08  7C 03 20 00 */	cmpw r3, r4
/* 8036FCAC 0036CC0C  41 82 00 20 */	beq lbl_8036FCCC
/* 8036FCB0 0036CC10  40 80 00 40 */	bge lbl_8036FCF0
/* 8036FCB4 0036CC14  48 00 03 6C */	b lbl_80370020
lbl_8036FCB8:
/* 8036FCB8 0036CC18  38 04 00 04 */	addi r0, r4, 4
/* 8036FCBC 0036CC1C  7C 03 00 00 */	cmpw r3, r0
/* 8036FCC0 0036CC20  41 82 03 48 */	beq lbl_80370008
/* 8036FCC4 0036CC24  40 80 03 5C */	bge lbl_80370020
/* 8036FCC8 0036CC28  48 00 01 B4 */	b lbl_8036FE7C
lbl_8036FCCC:
/* 8036FCCC 0036CC2C  38 00 00 01 */	li r0, 1
/* 8036FCD0 0036CC30  90 05 00 00 */	stw r0, 0(r5)
/* 8036FCD4 0036CC34  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FCD8 0036CC38  81 83 00 28 */	lwz r12, 0x28(r3)
/* 8036FCDC 0036CC3C  28 0C 00 00 */	cmplwi r12, 0
/* 8036FCE0 0036CC40  41 82 03 40 */	beq lbl_80370020
/* 8036FCE4 0036CC44  7D 88 03 A6 */	mtlr r12
/* 8036FCE8 0036CC48  4E 80 00 21 */	blrl 
/* 8036FCEC 0036CC4C  48 00 03 34 */	b lbl_80370020
lbl_8036FCF0:
/* 8036FCF0 0036CC50  38 00 00 01 */	li r0, 1
/* 8036FCF4 0036CC54  90 05 00 00 */	stw r0, 0(r5)
/* 8036FCF8 0036CC58  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FCFC 0036CC5C  81 83 00 2C */	lwz r12, 0x2c(r3)
/* 8036FD00 0036CC60  28 0C 00 00 */	cmplwi r12, 0
/* 8036FD04 0036CC64  41 82 03 1C */	beq lbl_80370020
/* 8036FD08 0036CC68  7D 88 03 A6 */	mtlr r12
/* 8036FD0C 0036CC6C  4E 80 00 21 */	blrl 
/* 8036FD10 0036CC70  48 00 03 10 */	b lbl_80370020
lbl_8036FD14:
/* 8036FD14 0036CC74  80 0D AB 90 */	lwz r0, lbl_805A9750@sda21(r13)
/* 8036FD18 0036CC78  2C 00 00 00 */	cmpwi r0, 0
/* 8036FD1C 0036CC7C  41 82 00 98 */	beq lbl_8036FDB4
/* 8036FD20 0036CC80  80 0D AB 94 */	lwz r0, lbl_805A9754@sda21(r13)
/* 8036FD24 0036CC84  7C 05 00 40 */	cmplw r5, r0
/* 8036FD28 0036CC88  40 82 00 44 */	bne lbl_8036FD6C
/* 8036FD2C 0036CC8C  3C 60 CD D1 */	lis r3, 0xCDD10003@ha
/* 8036FD30 0036CC90  38 63 00 03 */	addi r3, r3, 0xCDD10003@l
/* 8036FD34 0036CC94  4B FF FC BD */	bl DSPSendMailToDSP
lbl_8036FD38:
/* 8036FD38 0036CC98  4B FF FC 81 */	bl DSPCheckMailToDSP
/* 8036FD3C 0036CC9C  28 03 00 00 */	cmplwi r3, 0
/* 8036FD40 0036CCA0  40 82 FF F8 */	bne lbl_8036FD38
/* 8036FD44 0036CCA4  38 00 00 00 */	li r0, 0
/* 8036FD48 0036CCA8  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FD4C 0036CCAC  90 0D AB 94 */	stw r0, lbl_805A9754@sda21(r13)
/* 8036FD50 0036CCB0  90 0D AB 90 */	stw r0, lbl_805A9750@sda21(r13)
/* 8036FD54 0036CCB4  81 83 00 2C */	lwz r12, 0x2c(r3)
/* 8036FD58 0036CCB8  28 0C 00 00 */	cmplwi r12, 0
/* 8036FD5C 0036CCBC  41 82 02 C4 */	beq lbl_80370020
/* 8036FD60 0036CCC0  7D 88 03 A6 */	mtlr r12
/* 8036FD64 0036CCC4  4E 80 00 21 */	blrl 
/* 8036FD68 0036CCC8  48 00 02 B8 */	b lbl_80370020
lbl_8036FD6C:
/* 8036FD6C 0036CCCC  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FD70 0036CCD0  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FD74 0036CCD4  4B FF FC 7D */	bl DSPSendMailToDSP
lbl_8036FD78:
/* 8036FD78 0036CCD8  4B FF FC 41 */	bl DSPCheckMailToDSP
/* 8036FD7C 0036CCDC  28 03 00 00 */	cmplwi r3, 0
/* 8036FD80 0036CCE0  40 82 FF F8 */	bne lbl_8036FD78
/* 8036FD84 0036CCE4  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FD88 0036CCE8  80 8D AB 94 */	lwz r4, lbl_805A9754@sda21(r13)
/* 8036FD8C 0036CCEC  48 00 02 B9 */	bl __DSP_exec_task
/* 8036FD90 0036CCF0  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FD94 0036CCF4  38 80 00 02 */	li r4, 2
/* 8036FD98 0036CCF8  38 00 00 00 */	li r0, 0
/* 8036FD9C 0036CCFC  90 83 00 00 */	stw r4, 0(r3)
/* 8036FDA0 0036CD00  80 6D AB 94 */	lwz r3, lbl_805A9754@sda21(r13)
/* 8036FDA4 0036CD04  90 0D AB 90 */	stw r0, lbl_805A9750@sda21(r13)
/* 8036FDA8 0036CD08  90 6D AB A4 */	stw r3, lbl_805A9764@sda21(r13)
/* 8036FDAC 0036CD0C  90 0D AB 94 */	stw r0, lbl_805A9754@sda21(r13)
/* 8036FDB0 0036CD10  48 00 02 70 */	b lbl_80370020
lbl_8036FDB4:
/* 8036FDB4 0036CD14  80 05 00 38 */	lwz r0, 0x38(r5)
/* 8036FDB8 0036CD18  28 00 00 00 */	cmplwi r0, 0
/* 8036FDBC 0036CD1C  40 82 00 80 */	bne lbl_8036FE3C
/* 8036FDC0 0036CD20  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 8036FDC4 0036CD24  7C 05 00 40 */	cmplw r5, r0
/* 8036FDC8 0036CD28  40 82 00 38 */	bne lbl_8036FE00
/* 8036FDCC 0036CD2C  3C 60 CD D1 */	lis r3, 0xCDD10003@ha
/* 8036FDD0 0036CD30  38 63 00 03 */	addi r3, r3, 0xCDD10003@l
/* 8036FDD4 0036CD34  4B FF FC 1D */	bl DSPSendMailToDSP
lbl_8036FDD8:
/* 8036FDD8 0036CD38  4B FF FB E1 */	bl DSPCheckMailToDSP
/* 8036FDDC 0036CD3C  28 03 00 00 */	cmplwi r3, 0
/* 8036FDE0 0036CD40  40 82 FF F8 */	bne lbl_8036FDD8
/* 8036FDE4 0036CD44  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FDE8 0036CD48  81 83 00 2C */	lwz r12, 0x2c(r3)
/* 8036FDEC 0036CD4C  28 0C 00 00 */	cmplwi r12, 0
/* 8036FDF0 0036CD50  41 82 02 30 */	beq lbl_80370020
/* 8036FDF4 0036CD54  7D 88 03 A6 */	mtlr r12
/* 8036FDF8 0036CD58  4E 80 00 21 */	blrl 
/* 8036FDFC 0036CD5C  48 00 02 24 */	b lbl_80370020
lbl_8036FE00:
/* 8036FE00 0036CD60  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FE04 0036CD64  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FE08 0036CD68  4B FF FB E9 */	bl DSPSendMailToDSP
lbl_8036FE0C:
/* 8036FE0C 0036CD6C  4B FF FB AD */	bl DSPCheckMailToDSP
/* 8036FE10 0036CD70  28 03 00 00 */	cmplwi r3, 0
/* 8036FE14 0036CD74  40 82 FF F8 */	bne lbl_8036FE0C
/* 8036FE18 0036CD78  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FE1C 0036CD7C  80 8D AB A0 */	lwz r4, lbl_805A9760@sda21(r13)
/* 8036FE20 0036CD80  48 00 02 25 */	bl __DSP_exec_task
/* 8036FE24 0036CD84  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FE28 0036CD88  38 00 00 02 */	li r0, 2
/* 8036FE2C 0036CD8C  90 03 00 00 */	stw r0, 0(r3)
/* 8036FE30 0036CD90  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 8036FE34 0036CD94  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 8036FE38 0036CD98  48 00 01 E8 */	b lbl_80370020
lbl_8036FE3C:
/* 8036FE3C 0036CD9C  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FE40 0036CDA0  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FE44 0036CDA4  4B FF FB AD */	bl DSPSendMailToDSP
lbl_8036FE48:
/* 8036FE48 0036CDA8  4B FF FB 71 */	bl DSPCheckMailToDSP
/* 8036FE4C 0036CDAC  28 03 00 00 */	cmplwi r3, 0
/* 8036FE50 0036CDB0  40 82 FF F8 */	bne lbl_8036FE48
/* 8036FE54 0036CDB4  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FE58 0036CDB8  80 83 00 38 */	lwz r4, 0x38(r3)
/* 8036FE5C 0036CDBC  48 00 01 E9 */	bl __DSP_exec_task
/* 8036FE60 0036CDC0  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FE64 0036CDC4  38 00 00 02 */	li r0, 2
/* 8036FE68 0036CDC8  90 03 00 00 */	stw r0, 0(r3)
/* 8036FE6C 0036CDCC  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FE70 0036CDD0  80 03 00 38 */	lwz r0, 0x38(r3)
/* 8036FE74 0036CDD4  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 8036FE78 0036CDD8  48 00 01 A8 */	b lbl_80370020
lbl_8036FE7C:
/* 8036FE7C 0036CDDC  80 0D AB 90 */	lwz r0, lbl_805A9750@sda21(r13)
/* 8036FE80 0036CDE0  2C 00 00 00 */	cmpwi r0, 0
/* 8036FE84 0036CDE4  41 82 00 60 */	beq lbl_8036FEE4
/* 8036FE88 0036CDE8  81 85 00 30 */	lwz r12, 0x30(r5)
/* 8036FE8C 0036CDEC  28 0C 00 00 */	cmplwi r12, 0
/* 8036FE90 0036CDF0  41 82 00 10 */	beq lbl_8036FEA0
/* 8036FE94 0036CDF4  7D 88 03 A6 */	mtlr r12
/* 8036FE98 0036CDF8  38 65 00 00 */	addi r3, r5, 0
/* 8036FE9C 0036CDFC  4E 80 00 21 */	blrl 
lbl_8036FEA0:
/* 8036FEA0 0036CE00  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FEA4 0036CE04  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FEA8 0036CE08  4B FF FB 49 */	bl DSPSendMailToDSP
lbl_8036FEAC:
/* 8036FEAC 0036CE0C  4B FF FB 0D */	bl DSPCheckMailToDSP
/* 8036FEB0 0036CE10  28 03 00 00 */	cmplwi r3, 0
/* 8036FEB4 0036CE14  40 82 FF F8 */	bne lbl_8036FEAC
/* 8036FEB8 0036CE18  38 60 00 00 */	li r3, 0
/* 8036FEBC 0036CE1C  80 8D AB 94 */	lwz r4, lbl_805A9754@sda21(r13)
/* 8036FEC0 0036CE20  48 00 01 85 */	bl __DSP_exec_task
/* 8036FEC4 0036CE24  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FEC8 0036CE28  48 00 05 49 */	bl __DSP_remove_task
/* 8036FECC 0036CE2C  80 6D AB 94 */	lwz r3, lbl_805A9754@sda21(r13)
/* 8036FED0 0036CE30  38 00 00 00 */	li r0, 0
/* 8036FED4 0036CE34  90 0D AB 90 */	stw r0, lbl_805A9750@sda21(r13)
/* 8036FED8 0036CE38  90 6D AB A4 */	stw r3, lbl_805A9764@sda21(r13)
/* 8036FEDC 0036CE3C  90 0D AB 94 */	stw r0, lbl_805A9754@sda21(r13)
/* 8036FEE0 0036CE40  48 00 01 40 */	b lbl_80370020
lbl_8036FEE4:
/* 8036FEE4 0036CE44  80 05 00 38 */	lwz r0, 0x38(r5)
/* 8036FEE8 0036CE48  28 00 00 00 */	cmplwi r0, 0
/* 8036FEEC 0036CE4C  40 82 00 B4 */	bne lbl_8036FFA0
/* 8036FEF0 0036CE50  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 8036FEF4 0036CE54  7C 05 00 40 */	cmplw r5, r0
/* 8036FEF8 0036CE58  40 82 00 4C */	bne lbl_8036FF44
/* 8036FEFC 0036CE5C  81 85 00 30 */	lwz r12, 0x30(r5)
/* 8036FF00 0036CE60  28 0C 00 00 */	cmplwi r12, 0
/* 8036FF04 0036CE64  41 82 00 10 */	beq lbl_8036FF14
/* 8036FF08 0036CE68  7D 88 03 A6 */	mtlr r12
/* 8036FF0C 0036CE6C  38 65 00 00 */	addi r3, r5, 0
/* 8036FF10 0036CE70  4E 80 00 21 */	blrl 
lbl_8036FF14:
/* 8036FF14 0036CE74  3C 60 CD D1 */	lis r3, 0xCDD10002@ha
/* 8036FF18 0036CE78  38 63 00 02 */	addi r3, r3, 0xCDD10002@l
/* 8036FF1C 0036CE7C  4B FF FA D5 */	bl DSPSendMailToDSP
lbl_8036FF20:
/* 8036FF20 0036CE80  4B FF FA 99 */	bl DSPCheckMailToDSP
/* 8036FF24 0036CE84  28 03 00 00 */	cmplwi r3, 0
/* 8036FF28 0036CE88  40 82 FF F8 */	bne lbl_8036FF20
/* 8036FF2C 0036CE8C  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FF30 0036CE90  38 00 00 03 */	li r0, 3
/* 8036FF34 0036CE94  90 03 00 00 */	stw r0, 0(r3)
/* 8036FF38 0036CE98  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FF3C 0036CE9C  48 00 04 D5 */	bl __DSP_remove_task
/* 8036FF40 0036CEA0  48 00 00 E0 */	b lbl_80370020
lbl_8036FF44:
/* 8036FF44 0036CEA4  81 85 00 30 */	lwz r12, 0x30(r5)
/* 8036FF48 0036CEA8  28 0C 00 00 */	cmplwi r12, 0
/* 8036FF4C 0036CEAC  41 82 00 10 */	beq lbl_8036FF5C
/* 8036FF50 0036CEB0  7D 88 03 A6 */	mtlr r12
/* 8036FF54 0036CEB4  38 65 00 00 */	addi r3, r5, 0
/* 8036FF58 0036CEB8  4E 80 00 21 */	blrl 
lbl_8036FF5C:
/* 8036FF5C 0036CEBC  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FF60 0036CEC0  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FF64 0036CEC4  4B FF FA 8D */	bl DSPSendMailToDSP
lbl_8036FF68:
/* 8036FF68 0036CEC8  4B FF FA 51 */	bl DSPCheckMailToDSP
/* 8036FF6C 0036CECC  28 03 00 00 */	cmplwi r3, 0
/* 8036FF70 0036CED0  40 82 FF F8 */	bne lbl_8036FF68
/* 8036FF74 0036CED4  80 8D AB A4 */	lwz r4, lbl_805A9764@sda21(r13)
/* 8036FF78 0036CED8  38 00 00 03 */	li r0, 3
/* 8036FF7C 0036CEDC  38 60 00 00 */	li r3, 0
/* 8036FF80 0036CEE0  90 04 00 00 */	stw r0, 0(r4)
/* 8036FF84 0036CEE4  80 8D AB A0 */	lwz r4, lbl_805A9760@sda21(r13)
/* 8036FF88 0036CEE8  48 00 00 BD */	bl __DSP_exec_task
/* 8036FF8C 0036CEEC  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 8036FF90 0036CEF0  80 6D AB 9C */	lwz r3, lbl_805A975C@sda21(r13)
/* 8036FF94 0036CEF4  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 8036FF98 0036CEF8  48 00 04 79 */	bl __DSP_remove_task
/* 8036FF9C 0036CEFC  48 00 00 84 */	b lbl_80370020
lbl_8036FFA0:
/* 8036FFA0 0036CF00  81 85 00 30 */	lwz r12, 0x30(r5)
/* 8036FFA4 0036CF04  28 0C 00 00 */	cmplwi r12, 0
/* 8036FFA8 0036CF08  41 82 00 10 */	beq lbl_8036FFB8
/* 8036FFAC 0036CF0C  7D 88 03 A6 */	mtlr r12
/* 8036FFB0 0036CF10  38 65 00 00 */	addi r3, r5, 0
/* 8036FFB4 0036CF14  4E 80 00 21 */	blrl 
lbl_8036FFB8:
/* 8036FFB8 0036CF18  3C 60 CD D1 */	lis r3, 0xCDD10001@ha
/* 8036FFBC 0036CF1C  38 63 00 01 */	addi r3, r3, 0xCDD10001@l
/* 8036FFC0 0036CF20  4B FF FA 31 */	bl DSPSendMailToDSP
lbl_8036FFC4:
/* 8036FFC4 0036CF24  4B FF F9 F5 */	bl DSPCheckMailToDSP
/* 8036FFC8 0036CF28  28 03 00 00 */	cmplwi r3, 0
/* 8036FFCC 0036CF2C  40 82 FF F8 */	bne lbl_8036FFC4
/* 8036FFD0 0036CF30  80 8D AB A4 */	lwz r4, lbl_805A9764@sda21(r13)
/* 8036FFD4 0036CF34  38 00 00 03 */	li r0, 3
/* 8036FFD8 0036CF38  38 60 00 00 */	li r3, 0
/* 8036FFDC 0036CF3C  90 04 00 00 */	stw r0, 0(r4)
/* 8036FFE0 0036CF40  80 8D AB A4 */	lwz r4, lbl_805A9764@sda21(r13)
/* 8036FFE4 0036CF44  80 84 00 38 */	lwz r4, 0x38(r4)
/* 8036FFE8 0036CF48  48 00 00 5D */	bl __DSP_exec_task
/* 8036FFEC 0036CF4C  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FFF0 0036CF50  80 03 00 38 */	lwz r0, 0x38(r3)
/* 8036FFF4 0036CF54  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 8036FFF8 0036CF58  80 6D AB A4 */	lwz r3, lbl_805A9764@sda21(r13)
/* 8036FFFC 0036CF5C  80 63 00 3C */	lwz r3, 0x3c(r3)
/* 80370000 0036CF60  48 00 04 11 */	bl __DSP_remove_task
/* 80370004 0036CF64  48 00 00 1C */	b lbl_80370020
lbl_80370008:
/* 80370008 0036CF68  81 85 00 34 */	lwz r12, 0x34(r5)
/* 8037000C 0036CF6C  28 0C 00 00 */	cmplwi r12, 0
/* 80370010 0036CF70  41 82 00 10 */	beq lbl_80370020
/* 80370014 0036CF74  7D 88 03 A6 */	mtlr r12
/* 80370018 0036CF78  38 65 00 00 */	addi r3, r5, 0
/* 8037001C 0036CF7C  4E 80 00 21 */	blrl 
lbl_80370020:
/* 80370020 0036CF80  38 61 00 10 */	addi r3, r1, 0x10
/* 80370024 0036CF84  48 00 F5 F1 */	bl OSClearContext
/* 80370028 0036CF88  7F E3 FB 78 */	mr r3, r31
/* 8037002C 0036CF8C  48 00 F3 F1 */	bl OSSetCurrentContext
/* 80370030 0036CF90  80 01 02 EC */	lwz r0, 0x2ec(r1)
/* 80370034 0036CF94  83 E1 02 E4 */	lwz r31, 0x2e4(r1)
/* 80370038 0036CF98  38 21 02 E8 */	addi r1, r1, 0x2e8
/* 8037003C 0036CF9C  7C 08 03 A6 */	mtlr r0
/* 80370040 0036CFA0  4E 80 00 20 */	blr 

.global __DSP_exec_task
__DSP_exec_task:
/* 80370044 0036CFA4  7C 08 02 A6 */	mflr r0
/* 80370048 0036CFA8  90 01 00 04 */	stw r0, 4(r1)
/* 8037004C 0036CFAC  94 21 FF E8 */	stwu r1, -0x18(r1)
/* 80370050 0036CFB0  93 E1 00 14 */	stw r31, 0x14(r1)
/* 80370054 0036CFB4  3B E4 00 00 */	addi r31, r4, 0
/* 80370058 0036CFB8  93 C1 00 10 */	stw r30, 0x10(r1)
/* 8037005C 0036CFBC  7C 7E 1B 79 */	or. r30, r3, r3
/* 80370060 0036CFC0  41 82 00 44 */	beq lbl_803700A4
/* 80370064 0036CFC4  80 7E 00 18 */	lwz r3, 0x18(r30)
/* 80370068 0036CFC8  4B FF F9 89 */	bl DSPSendMailToDSP
lbl_8037006C:
/* 8037006C 0036CFCC  4B FF F9 4D */	bl DSPCheckMailToDSP
/* 80370070 0036CFD0  28 03 00 00 */	cmplwi r3, 0
/* 80370074 0036CFD4  40 82 FF F8 */	bne lbl_8037006C
/* 80370078 0036CFD8  80 7E 00 1C */	lwz r3, 0x1c(r30)
/* 8037007C 0036CFDC  4B FF F9 75 */	bl DSPSendMailToDSP
lbl_80370080:
/* 80370080 0036CFE0  4B FF F9 39 */	bl DSPCheckMailToDSP
/* 80370084 0036CFE4  28 03 00 00 */	cmplwi r3, 0
/* 80370088 0036CFE8  40 82 FF F8 */	bne lbl_80370080
/* 8037008C 0036CFEC  80 7E 00 20 */	lwz r3, 0x20(r30)
/* 80370090 0036CFF0  4B FF F9 61 */	bl DSPSendMailToDSP
lbl_80370094:
/* 80370094 0036CFF4  4B FF F9 25 */	bl DSPCheckMailToDSP
/* 80370098 0036CFF8  28 03 00 00 */	cmplwi r3, 0
/* 8037009C 0036CFFC  40 82 FF F8 */	bne lbl_80370094
/* 803700A0 0036D000  48 00 00 40 */	b lbl_803700E0
lbl_803700A4:
/* 803700A4 0036D004  38 60 00 00 */	li r3, 0
/* 803700A8 0036D008  4B FF F9 49 */	bl DSPSendMailToDSP
lbl_803700AC:
/* 803700AC 0036D00C  4B FF F9 0D */	bl DSPCheckMailToDSP
/* 803700B0 0036D010  28 03 00 00 */	cmplwi r3, 0
/* 803700B4 0036D014  40 82 FF F8 */	bne lbl_803700AC
/* 803700B8 0036D018  38 60 00 00 */	li r3, 0
/* 803700BC 0036D01C  4B FF F9 35 */	bl DSPSendMailToDSP
lbl_803700C0:
/* 803700C0 0036D020  4B FF F8 F9 */	bl DSPCheckMailToDSP
/* 803700C4 0036D024  28 03 00 00 */	cmplwi r3, 0
/* 803700C8 0036D028  40 82 FF F8 */	bne lbl_803700C0
/* 803700CC 0036D02C  38 60 00 00 */	li r3, 0
/* 803700D0 0036D030  4B FF F9 21 */	bl DSPSendMailToDSP
lbl_803700D4:
/* 803700D4 0036D034  4B FF F8 E5 */	bl DSPCheckMailToDSP
/* 803700D8 0036D038  28 03 00 00 */	cmplwi r3, 0
/* 803700DC 0036D03C  40 82 FF F8 */	bne lbl_803700D4
lbl_803700E0:
/* 803700E0 0036D040  80 7F 00 0C */	lwz r3, 0xc(r31)
/* 803700E4 0036D044  4B FF F9 0D */	bl DSPSendMailToDSP
lbl_803700E8:
/* 803700E8 0036D048  4B FF F8 D1 */	bl DSPCheckMailToDSP
/* 803700EC 0036D04C  28 03 00 00 */	cmplwi r3, 0
/* 803700F0 0036D050  40 82 FF F8 */	bne lbl_803700E8
/* 803700F4 0036D054  80 7F 00 10 */	lwz r3, 0x10(r31)
/* 803700F8 0036D058  4B FF F8 F9 */	bl DSPSendMailToDSP
lbl_803700FC:
/* 803700FC 0036D05C  4B FF F8 BD */	bl DSPCheckMailToDSP
/* 80370100 0036D060  28 03 00 00 */	cmplwi r3, 0
/* 80370104 0036D064  40 82 FF F8 */	bne lbl_803700FC
/* 80370108 0036D068  80 7F 00 14 */	lwz r3, 0x14(r31)
/* 8037010C 0036D06C  4B FF F8 E5 */	bl DSPSendMailToDSP
lbl_80370110:
/* 80370110 0036D070  4B FF F8 A9 */	bl DSPCheckMailToDSP
/* 80370114 0036D074  28 03 00 00 */	cmplwi r3, 0
/* 80370118 0036D078  40 82 FF F8 */	bne lbl_80370110
/* 8037011C 0036D07C  80 1F 00 00 */	lwz r0, 0(r31)
/* 80370120 0036D080  28 00 00 00 */	cmplwi r0, 0
/* 80370124 0036D084  40 82 00 58 */	bne lbl_8037017C
/* 80370128 0036D088  A0 7F 00 24 */	lhz r3, 0x24(r31)
/* 8037012C 0036D08C  4B FF F8 C5 */	bl DSPSendMailToDSP
lbl_80370130:
/* 80370130 0036D090  4B FF F8 89 */	bl DSPCheckMailToDSP
/* 80370134 0036D094  28 03 00 00 */	cmplwi r3, 0
/* 80370138 0036D098  40 82 FF F8 */	bne lbl_80370130
/* 8037013C 0036D09C  38 60 00 00 */	li r3, 0
/* 80370140 0036D0A0  4B FF F8 B1 */	bl DSPSendMailToDSP
lbl_80370144:
/* 80370144 0036D0A4  4B FF F8 75 */	bl DSPCheckMailToDSP
/* 80370148 0036D0A8  28 03 00 00 */	cmplwi r3, 0
/* 8037014C 0036D0AC  40 82 FF F8 */	bne lbl_80370144
/* 80370150 0036D0B0  38 60 00 00 */	li r3, 0
/* 80370154 0036D0B4  4B FF F8 9D */	bl DSPSendMailToDSP
lbl_80370158:
/* 80370158 0036D0B8  4B FF F8 61 */	bl DSPCheckMailToDSP
/* 8037015C 0036D0BC  28 03 00 00 */	cmplwi r3, 0
/* 80370160 0036D0C0  40 82 FF F8 */	bne lbl_80370158
/* 80370164 0036D0C4  38 60 00 00 */	li r3, 0
/* 80370168 0036D0C8  4B FF F8 89 */	bl DSPSendMailToDSP
lbl_8037016C:
/* 8037016C 0036D0CC  4B FF F8 4D */	bl DSPCheckMailToDSP
/* 80370170 0036D0D0  28 03 00 00 */	cmplwi r3, 0
/* 80370174 0036D0D4  40 82 FF F8 */	bne lbl_8037016C
/* 80370178 0036D0D8  48 00 00 54 */	b lbl_803701CC
lbl_8037017C:
/* 8037017C 0036D0DC  A0 7F 00 26 */	lhz r3, 0x26(r31)
/* 80370180 0036D0E0  4B FF F8 71 */	bl DSPSendMailToDSP
lbl_80370184:
/* 80370184 0036D0E4  4B FF F8 35 */	bl DSPCheckMailToDSP
/* 80370188 0036D0E8  28 03 00 00 */	cmplwi r3, 0
/* 8037018C 0036D0EC  40 82 FF F8 */	bne lbl_80370184
/* 80370190 0036D0F0  80 7F 00 18 */	lwz r3, 0x18(r31)
/* 80370194 0036D0F4  4B FF F8 5D */	bl DSPSendMailToDSP
lbl_80370198:
/* 80370198 0036D0F8  4B FF F8 21 */	bl DSPCheckMailToDSP
/* 8037019C 0036D0FC  28 03 00 00 */	cmplwi r3, 0
/* 803701A0 0036D100  40 82 FF F8 */	bne lbl_80370198
/* 803701A4 0036D104  80 7F 00 1C */	lwz r3, 0x1c(r31)
/* 803701A8 0036D108  4B FF F8 49 */	bl DSPSendMailToDSP
lbl_803701AC:
/* 803701AC 0036D10C  4B FF F8 0D */	bl DSPCheckMailToDSP
/* 803701B0 0036D110  28 03 00 00 */	cmplwi r3, 0
/* 803701B4 0036D114  40 82 FF F8 */	bne lbl_803701AC
/* 803701B8 0036D118  80 7F 00 20 */	lwz r3, 0x20(r31)
/* 803701BC 0036D11C  4B FF F8 35 */	bl DSPSendMailToDSP
lbl_803701C0:
/* 803701C0 0036D120  4B FF F7 F9 */	bl DSPCheckMailToDSP
/* 803701C4 0036D124  28 03 00 00 */	cmplwi r3, 0
/* 803701C8 0036D128  40 82 FF F8 */	bne lbl_803701C0
lbl_803701CC:
/* 803701CC 0036D12C  80 01 00 1C */	lwz r0, 0x1c(r1)
/* 803701D0 0036D130  83 E1 00 14 */	lwz r31, 0x14(r1)
/* 803701D4 0036D134  83 C1 00 10 */	lwz r30, 0x10(r1)
/* 803701D8 0036D138  38 21 00 18 */	addi r1, r1, 0x18
/* 803701DC 0036D13C  7C 08 03 A6 */	mtlr r0
/* 803701E0 0036D140  4E 80 00 20 */	blr 

.global __DSP_boot_task
__DSP_boot_task:
/* 803701E4 0036D144  7C 08 02 A6 */	mflr r0
/* 803701E8 0036D148  3C 80 80 3F */	lis r4, lbl_803EFA50@ha
/* 803701EC 0036D14C  90 01 00 04 */	stw r0, 4(r1)
/* 803701F0 0036D150  94 21 FF E8 */	stwu r1, -0x18(r1)
/* 803701F4 0036D154  93 E1 00 14 */	stw r31, 0x14(r1)
/* 803701F8 0036D158  3B E4 FA 50 */	addi r31, r4, lbl_803EFA50@l
/* 803701FC 0036D15C  93 C1 00 10 */	stw r30, 0x10(r1)
/* 80370200 0036D160  3B C3 00 00 */	addi r30, r3, 0
lbl_80370204:
/* 80370204 0036D164  4B FF F7 C5 */	bl DSPCheckMailFromDSP
/* 80370208 0036D168  28 03 00 00 */	cmplwi r3, 0
/* 8037020C 0036D16C  41 82 FF F8 */	beq lbl_80370204
/* 80370210 0036D170  4B FF F7 C9 */	bl DSPReadMailFromDSP
/* 80370214 0036D174  90 61 00 0C */	stw r3, 0xc(r1)
/* 80370218 0036D178  3C 60 80 F4 */	lis r3, 0x80F3A001@ha
/* 8037021C 0036D17C  38 63 A0 01 */	addi r3, r3, 0x80F3A001@l
/* 80370220 0036D180  4B FF F7 D1 */	bl DSPSendMailToDSP
lbl_80370224:
/* 80370224 0036D184  4B FF F7 95 */	bl DSPCheckMailToDSP
/* 80370228 0036D188  28 03 00 00 */	cmplwi r3, 0
/* 8037022C 0036D18C  40 82 FF F8 */	bne lbl_80370224
/* 80370230 0036D190  80 7E 00 0C */	lwz r3, 0xc(r30)
/* 80370234 0036D194  4B FF F7 BD */	bl DSPSendMailToDSP
lbl_80370238:
/* 80370238 0036D198  4B FF F7 81 */	bl DSPCheckMailToDSP
/* 8037023C 0036D19C  28 03 00 00 */	cmplwi r3, 0
/* 80370240 0036D1A0  40 82 FF F8 */	bne lbl_80370238
/* 80370244 0036D1A4  3C 60 80 F4 */	lis r3, 0x80F3C002@ha
/* 80370248 0036D1A8  38 63 C0 02 */	addi r3, r3, 0x80F3C002@l
/* 8037024C 0036D1AC  4B FF F7 A5 */	bl DSPSendMailToDSP
lbl_80370250:
/* 80370250 0036D1B0  4B FF F7 69 */	bl DSPCheckMailToDSP
/* 80370254 0036D1B4  28 03 00 00 */	cmplwi r3, 0
/* 80370258 0036D1B8  40 82 FF F8 */	bne lbl_80370250
/* 8037025C 0036D1BC  80 1E 00 14 */	lwz r0, 0x14(r30)
/* 80370260 0036D1C0  54 03 04 3E */	clrlwi r3, r0, 0x10
/* 80370264 0036D1C4  4B FF F7 8D */	bl DSPSendMailToDSP
lbl_80370268:
/* 80370268 0036D1C8  4B FF F7 51 */	bl DSPCheckMailToDSP
/* 8037026C 0036D1CC  28 03 00 00 */	cmplwi r3, 0
/* 80370270 0036D1D0  40 82 FF F8 */	bne lbl_80370268
/* 80370274 0036D1D4  3C 60 80 F4 */	lis r3, 0x80F3A002@ha
/* 80370278 0036D1D8  38 63 A0 02 */	addi r3, r3, 0x80F3A002@l
/* 8037027C 0036D1DC  4B FF F7 75 */	bl DSPSendMailToDSP
lbl_80370280:
/* 80370280 0036D1E0  4B FF F7 39 */	bl DSPCheckMailToDSP
/* 80370284 0036D1E4  28 03 00 00 */	cmplwi r3, 0
/* 80370288 0036D1E8  40 82 FF F8 */	bne lbl_80370280
/* 8037028C 0036D1EC  80 7E 00 10 */	lwz r3, 0x10(r30)
/* 80370290 0036D1F0  4B FF F7 61 */	bl DSPSendMailToDSP
lbl_80370294:
/* 80370294 0036D1F4  4B FF F7 25 */	bl DSPCheckMailToDSP
/* 80370298 0036D1F8  28 03 00 00 */	cmplwi r3, 0
/* 8037029C 0036D1FC  40 82 FF F8 */	bne lbl_80370294
/* 803702A0 0036D200  3C 60 80 F4 */	lis r3, 0x80F3B002@ha
/* 803702A4 0036D204  38 63 B0 02 */	addi r3, r3, 0x80F3B002@l
/* 803702A8 0036D208  4B FF F7 49 */	bl DSPSendMailToDSP
lbl_803702AC:
/* 803702AC 0036D20C  4B FF F7 0D */	bl DSPCheckMailToDSP
/* 803702B0 0036D210  28 03 00 00 */	cmplwi r3, 0
/* 803702B4 0036D214  40 82 FF F8 */	bne lbl_803702AC
/* 803702B8 0036D218  38 60 00 00 */	li r3, 0
/* 803702BC 0036D21C  4B FF F7 35 */	bl DSPSendMailToDSP
lbl_803702C0:
/* 803702C0 0036D220  4B FF F6 F9 */	bl DSPCheckMailToDSP
/* 803702C4 0036D224  28 03 00 00 */	cmplwi r3, 0
/* 803702C8 0036D228  40 82 FF F8 */	bne lbl_803702C0
/* 803702CC 0036D22C  3C 60 80 F4 */	lis r3, 0x80F3D001@ha
/* 803702D0 0036D230  38 63 D0 01 */	addi r3, r3, 0x80F3D001@l
/* 803702D4 0036D234  4B FF F7 1D */	bl DSPSendMailToDSP
lbl_803702D8:
/* 803702D8 0036D238  4B FF F6 E1 */	bl DSPCheckMailToDSP
/* 803702DC 0036D23C  28 03 00 00 */	cmplwi r3, 0
/* 803702E0 0036D240  40 82 FF F8 */	bne lbl_803702D8
/* 803702E4 0036D244  A0 7E 00 24 */	lhz r3, 0x24(r30)
/* 803702E8 0036D248  4B FF F7 09 */	bl DSPSendMailToDSP
lbl_803702EC:
/* 803702EC 0036D24C  4B FF F6 CD */	bl DSPCheckMailToDSP
/* 803702F0 0036D250  28 03 00 00 */	cmplwi r3, 0
/* 803702F4 0036D254  40 82 FF F8 */	bne lbl_803702EC
/* 803702F8 0036D258  38 7F 00 00 */	addi r3, r31, 0
/* 803702FC 0036D25C  4C C6 31 82 */	crclr 6
/* 80370300 0036D260  38 9E 00 00 */	addi r4, r30, 0
/* 80370304 0036D264  4B FF F8 CD */	bl __DSP_debug_printf
/* 80370308 0036D268  80 9E 00 0C */	lwz r4, 0xc(r30)
/* 8037030C 0036D26C  38 7F 00 20 */	addi r3, r31, 0x20
/* 80370310 0036D270  4C C6 31 82 */	crclr 6
/* 80370314 0036D274  4B FF F8 BD */	bl __DSP_debug_printf
/* 80370318 0036D278  80 9E 00 14 */	lwz r4, 0x14(r30)
/* 8037031C 0036D27C  38 7F 00 50 */	addi r3, r31, 0x50
/* 80370320 0036D280  4C C6 31 82 */	crclr 6
/* 80370324 0036D284  4B FF F8 AD */	bl __DSP_debug_printf
/* 80370328 0036D288  80 9E 00 10 */	lwz r4, 0x10(r30)
/* 8037032C 0036D28C  38 7F 00 80 */	addi r3, r31, 0x80
/* 80370330 0036D290  4C C6 31 82 */	crclr 6
/* 80370334 0036D294  4B FF F8 9D */	bl __DSP_debug_printf
/* 80370338 0036D298  80 9E 00 1C */	lwz r4, 0x1c(r30)
/* 8037033C 0036D29C  38 7F 00 B0 */	addi r3, r31, 0xb0
/* 80370340 0036D2A0  4C C6 31 82 */	crclr 6
/* 80370344 0036D2A4  4B FF F8 8D */	bl __DSP_debug_printf
/* 80370348 0036D2A8  A0 9E 00 24 */	lhz r4, 0x24(r30)
/* 8037034C 0036D2AC  38 7F 00 E0 */	addi r3, r31, 0xe0
/* 80370350 0036D2B0  4C C6 31 82 */	crclr 6
/* 80370354 0036D2B4  4B FF F8 7D */	bl __DSP_debug_printf
/* 80370358 0036D2B8  80 01 00 1C */	lwz r0, 0x1c(r1)
/* 8037035C 0036D2BC  83 E1 00 14 */	lwz r31, 0x14(r1)
/* 80370360 0036D2C0  83 C1 00 10 */	lwz r30, 0x10(r1)
/* 80370364 0036D2C4  38 21 00 18 */	addi r1, r1, 0x18
/* 80370368 0036D2C8  7C 08 03 A6 */	mtlr r0
/* 8037036C 0036D2CC  4E 80 00 20 */	blr 

.global __DSP_insert_task
__DSP_insert_task:
/* 80370370 0036D2D0  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 80370374 0036D2D4  28 00 00 00 */	cmplwi r0, 0
/* 80370378 0036D2D8  40 82 00 20 */	bne lbl_80370398
/* 8037037C 0036D2DC  90 6D AB A4 */	stw r3, lbl_805A9764@sda21(r13)
/* 80370380 0036D2E0  38 00 00 00 */	li r0, 0
/* 80370384 0036D2E4  90 6D AB 9C */	stw r3, lbl_805A975C@sda21(r13)
/* 80370388 0036D2E8  90 6D AB A0 */	stw r3, lbl_805A9760@sda21(r13)
/* 8037038C 0036D2EC  90 03 00 3C */	stw r0, 0x3c(r3)
/* 80370390 0036D2F0  90 03 00 38 */	stw r0, 0x38(r3)
/* 80370394 0036D2F4  4E 80 00 20 */	blr 
lbl_80370398:
/* 80370398 0036D2F8  7C 05 03 78 */	mr r5, r0
/* 8037039C 0036D2FC  48 00 00 44 */	b lbl_803703E0
lbl_803703A0:
/* 803703A0 0036D300  80 83 00 04 */	lwz r4, 4(r3)
/* 803703A4 0036D304  80 05 00 04 */	lwz r0, 4(r5)
/* 803703A8 0036D308  7C 04 00 40 */	cmplw r4, r0
/* 803703AC 0036D30C  40 80 00 30 */	bge lbl_803703DC
/* 803703B0 0036D310  80 05 00 3C */	lwz r0, 0x3c(r5)
/* 803703B4 0036D314  90 03 00 3C */	stw r0, 0x3c(r3)
/* 803703B8 0036D318  90 65 00 3C */	stw r3, 0x3c(r5)
/* 803703BC 0036D31C  90 A3 00 38 */	stw r5, 0x38(r3)
/* 803703C0 0036D320  80 83 00 3C */	lwz r4, 0x3c(r3)
/* 803703C4 0036D324  28 04 00 00 */	cmplwi r4, 0
/* 803703C8 0036D328  40 82 00 0C */	bne lbl_803703D4
/* 803703CC 0036D32C  90 6D AB A0 */	stw r3, lbl_805A9760@sda21(r13)
/* 803703D0 0036D330  48 00 00 18 */	b lbl_803703E8
lbl_803703D4:
/* 803703D4 0036D334  90 64 00 38 */	stw r3, 0x38(r4)
/* 803703D8 0036D338  48 00 00 10 */	b lbl_803703E8
lbl_803703DC:
/* 803703DC 0036D33C  80 A5 00 38 */	lwz r5, 0x38(r5)
lbl_803703E0:
/* 803703E0 0036D340  28 05 00 00 */	cmplwi r5, 0
/* 803703E4 0036D344  40 82 FF BC */	bne lbl_803703A0
lbl_803703E8:
/* 803703E8 0036D348  28 05 00 00 */	cmplwi r5, 0
/* 803703EC 0036D34C  4C 82 00 20 */	bnelr 
/* 803703F0 0036D350  80 8D AB 9C */	lwz r4, lbl_805A975C@sda21(r13)
/* 803703F4 0036D354  38 00 00 00 */	li r0, 0
/* 803703F8 0036D358  90 64 00 38 */	stw r3, 0x38(r4)
/* 803703FC 0036D35C  90 03 00 38 */	stw r0, 0x38(r3)
/* 80370400 0036D360  80 0D AB 9C */	lwz r0, lbl_805A975C@sda21(r13)
/* 80370404 0036D364  90 03 00 3C */	stw r0, 0x3c(r3)
/* 80370408 0036D368  90 6D AB 9C */	stw r3, lbl_805A975C@sda21(r13)
/* 8037040C 0036D36C  4E 80 00 20 */	blr 

.global __DSP_remove_task
__DSP_remove_task:
/* 80370410 0036D370  38 80 00 00 */	li r4, 0
/* 80370414 0036D374  90 83 00 08 */	stw r4, 8(r3)
/* 80370418 0036D378  38 00 00 03 */	li r0, 3
/* 8037041C 0036D37C  90 03 00 00 */	stw r0, 0(r3)
/* 80370420 0036D380  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 80370424 0036D384  7C 00 18 40 */	cmplw r0, r3
/* 80370428 0036D388  40 82 00 30 */	bne lbl_80370458
/* 8037042C 0036D38C  80 03 00 38 */	lwz r0, 0x38(r3)
/* 80370430 0036D390  28 00 00 00 */	cmplwi r0, 0
/* 80370434 0036D394  41 82 00 14 */	beq lbl_80370448
/* 80370438 0036D398  90 0D AB A0 */	stw r0, lbl_805A9760@sda21(r13)
/* 8037043C 0036D39C  80 63 00 38 */	lwz r3, 0x38(r3)
/* 80370440 0036D3A0  90 83 00 3C */	stw r4, 0x3c(r3)
/* 80370444 0036D3A4  4E 80 00 20 */	blr 
lbl_80370448:
/* 80370448 0036D3A8  90 8D AB A4 */	stw r4, lbl_805A9764@sda21(r13)
/* 8037044C 0036D3AC  90 8D AB 9C */	stw r4, lbl_805A975C@sda21(r13)
/* 80370450 0036D3B0  90 8D AB A0 */	stw r4, lbl_805A9760@sda21(r13)
/* 80370454 0036D3B4  4E 80 00 20 */	blr 
lbl_80370458:
/* 80370458 0036D3B8  80 0D AB 9C */	lwz r0, lbl_805A975C@sda21(r13)
/* 8037045C 0036D3BC  7C 00 18 40 */	cmplw r0, r3
/* 80370460 0036D3C0  40 82 00 20 */	bne lbl_80370480
/* 80370464 0036D3C4  80 03 00 3C */	lwz r0, 0x3c(r3)
/* 80370468 0036D3C8  90 0D AB 9C */	stw r0, lbl_805A975C@sda21(r13)
/* 8037046C 0036D3CC  80 63 00 3C */	lwz r3, 0x3c(r3)
/* 80370470 0036D3D0  90 83 00 38 */	stw r4, 0x38(r3)
/* 80370474 0036D3D4  80 0D AB A0 */	lwz r0, lbl_805A9760@sda21(r13)
/* 80370478 0036D3D8  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 8037047C 0036D3DC  4E 80 00 20 */	blr 
lbl_80370480:
/* 80370480 0036D3E0  80 03 00 38 */	lwz r0, 0x38(r3)
/* 80370484 0036D3E4  90 0D AB A4 */	stw r0, lbl_805A9764@sda21(r13)
/* 80370488 0036D3E8  80 03 00 38 */	lwz r0, 0x38(r3)
/* 8037048C 0036D3EC  80 83 00 3C */	lwz r4, 0x3c(r3)
/* 80370490 0036D3F0  90 04 00 38 */	stw r0, 0x38(r4)
/* 80370494 0036D3F4  80 03 00 3C */	lwz r0, 0x3c(r3)
/* 80370498 0036D3F8  80 63 00 38 */	lwz r3, 0x38(r3)
/* 8037049C 0036D3FC  90 03 00 3C */	stw r0, 0x3c(r3)
/* 803704A0 0036D400  4E 80 00 20 */	blr 

