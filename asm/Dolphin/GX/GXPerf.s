.include "macros.inc"

.section .data
lbl_803F0CB0:
	.incbin "baserom.dol", 0x3EDCB0, 0x5C
lbl_803F0D0C:
	.incbin "baserom.dol", 0x3EDD0C, 0x94
	
.section .text, "ax"

.global GXSetGPMetric
GXSetGPMetric:
/* 8037C7E8 00379748  80 A2 CE 08 */	lwz r5, lbl_805AEB28@sda21(r2)
/* 8037C7EC 0037974C  80 05 04 E4 */	lwz r0, 0x4e4(r5)
/* 8037C7F0 00379750  2C 00 00 22 */	cmpwi r0, 0x22
/* 8037C7F4 00379754  41 82 00 2C */	beq lbl_8037C820
/* 8037C7F8 00379758  40 80 00 74 */	bge lbl_8037C86C
/* 8037C7FC 0037975C  2C 00 00 0B */	cmpwi r0, 0xb
/* 8037C800 00379760  40 80 00 10 */	bge lbl_8037C810
/* 8037C804 00379764  2C 00 00 00 */	cmpwi r0, 0
/* 8037C808 00379768  40 80 00 18 */	bge lbl_8037C820
/* 8037C80C 0037976C  48 00 00 60 */	b lbl_8037C86C
lbl_8037C810:
/* 8037C810 00379770  2C 00 00 1B */	cmpwi r0, 0x1b
/* 8037C814 00379774  40 80 00 44 */	bge lbl_8037C858
/* 8037C818 00379778  48 00 00 28 */	b lbl_8037C840
/* 8037C81C 0037977C  48 00 00 50 */	b lbl_8037C86C
lbl_8037C820:
/* 8037C820 00379780  38 00 00 10 */	li r0, 0x10
/* 8037C824 00379784  3C C0 CC 01 */	lis r6, 0xCC008000@ha
/* 8037C828 00379788  98 06 80 00 */	stb r0, 0xCC008000@l(r6)
/* 8037C82C 0037978C  38 A0 10 06 */	li r5, 0x1006
/* 8037C830 00379790  38 00 00 00 */	li r0, 0
/* 8037C834 00379794  90 A6 80 00 */	stw r5, 0xCC008000@l(r6)
/* 8037C838 00379798  90 06 80 00 */	stw r0, 0xCC008000@l(r6)
/* 8037C83C 0037979C  48 00 00 30 */	b lbl_8037C86C
lbl_8037C840:
/* 8037C840 003797A0  38 00 00 61 */	li r0, 0x61
/* 8037C844 003797A4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C848 003797A8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C84C 003797AC  3C 00 23 00 */	lis r0, 0x2300
/* 8037C850 003797B0  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C854 003797B4  48 00 00 18 */	b lbl_8037C86C
lbl_8037C858:
/* 8037C858 003797B8  38 00 00 61 */	li r0, 0x61
/* 8037C85C 003797BC  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C860 003797C0  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C864 003797C4  3C 00 24 00 */	lis r0, 0x2400
/* 8037C868 003797C8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
lbl_8037C86C:
/* 8037C86C 003797CC  80 E2 CE 08 */	lwz r7, lbl_805AEB28@sda21(r2)
/* 8037C870 003797D0  80 07 04 E8 */	lwz r0, 0x4e8(r7)
/* 8037C874 003797D4  2C 00 00 15 */	cmpwi r0, 0x15
/* 8037C878 003797D8  41 82 00 2C */	beq lbl_8037C8A4
/* 8037C87C 003797DC  40 80 00 78 */	bge lbl_8037C8F4
/* 8037C880 003797E0  2C 00 00 09 */	cmpwi r0, 9
/* 8037C884 003797E4  40 80 00 10 */	bge lbl_8037C894
/* 8037C888 003797E8  2C 00 00 00 */	cmpwi r0, 0
/* 8037C88C 003797EC  40 80 00 18 */	bge lbl_8037C8A4
/* 8037C890 003797F0  48 00 00 64 */	b lbl_8037C8F4
lbl_8037C894:
/* 8037C894 003797F4  2C 00 00 11 */	cmpwi r0, 0x11
/* 8037C898 003797F8  40 80 00 50 */	bge lbl_8037C8E8
/* 8037C89C 003797FC  48 00 00 20 */	b lbl_8037C8BC
/* 8037C8A0 00379800  48 00 00 54 */	b lbl_8037C8F4
lbl_8037C8A4:
/* 8037C8A4 00379804  38 00 00 61 */	li r0, 0x61
/* 8037C8A8 00379808  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C8AC 0037980C  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C8B0 00379810  3C 00 67 00 */	lis r0, 0x6700
/* 8037C8B4 00379814  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C8B8 00379818  48 00 00 3C */	b lbl_8037C8F4
lbl_8037C8BC:
/* 8037C8BC 0037981C  80 07 04 EC */	lwz r0, 0x4ec(r7)
/* 8037C8C0 00379820  38 C0 00 08 */	li r6, 8
/* 8037C8C4 00379824  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C8C8 00379828  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037C8CC 0037982C  90 07 04 EC */	stw r0, 0x4ec(r7)
/* 8037C8D0 00379830  38 00 00 20 */	li r0, 0x20
/* 8037C8D4 00379834  98 C5 80 00 */	stb r6, 0xCC008000@l(r5)
/* 8037C8D8 00379838  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C8DC 0037983C  80 07 04 EC */	lwz r0, 0x4ec(r7)
/* 8037C8E0 00379840  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C8E4 00379844  48 00 00 10 */	b lbl_8037C8F4
lbl_8037C8E8:
/* 8037C8E8 00379848  80 AD AC 7C */	lwz r5, lbl_805A983C@sda21(r13)
/* 8037C8EC 0037984C  38 00 00 00 */	li r0, 0
/* 8037C8F0 00379850  B0 05 00 06 */	sth r0, 6(r5)
lbl_8037C8F4:
/* 8037C8F4 00379854  80 A2 CE 08 */	lwz r5, lbl_805AEB28@sda21(r2)
/* 8037C8F8 00379858  90 65 04 E4 */	stw r3, 0x4e4(r5)
/* 8037C8FC 0037985C  80 05 04 E4 */	lwz r0, 0x4e4(r5)
/* 8037C900 00379860  28 00 00 23 */	cmplwi r0, 0x23
/* 8037C904 00379864  41 81 04 1C */	bgt lbl_8037CD20
/* 8037C908 00379868  3C 60 80 3F */	lis r3, lbl_803F0D0C@ha
/* 8037C90C 0037986C  38 63 0D 0C */	addi r3, r3, lbl_803F0D0C@l
/* 8037C910 00379870  54 00 10 3A */	slwi r0, r0, 2
/* 8037C914 00379874  7C 03 00 2E */	lwzx r0, r3, r0
/* 8037C918 00379878  7C 09 03 A6 */	mtctr r0
/* 8037C91C 0037987C  4E 80 04 20 */	bctr 
/* 8037C920 00379880  38 00 00 10 */	li r0, 0x10
/* 8037C924 00379884  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C928 00379888  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C92C 0037988C  38 60 10 06 */	li r3, 0x1006
/* 8037C930 00379890  38 00 02 73 */	li r0, 0x273
/* 8037C934 00379894  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C938 00379898  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C93C 0037989C  48 00 03 E4 */	b lbl_8037CD20
/* 8037C940 003798A0  38 00 00 10 */	li r0, 0x10
/* 8037C944 003798A4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C948 003798A8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C94C 003798AC  38 60 10 06 */	li r3, 0x1006
/* 8037C950 003798B0  38 00 01 4A */	li r0, 0x14a
/* 8037C954 003798B4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C958 003798B8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C95C 003798BC  48 00 03 C4 */	b lbl_8037CD20
/* 8037C960 003798C0  38 00 00 10 */	li r0, 0x10
/* 8037C964 003798C4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C968 003798C8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C96C 003798CC  38 60 10 06 */	li r3, 0x1006
/* 8037C970 003798D0  38 00 01 6B */	li r0, 0x16b
/* 8037C974 003798D4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C978 003798D8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C97C 003798DC  48 00 03 A4 */	b lbl_8037CD20
/* 8037C980 003798E0  38 00 00 10 */	li r0, 0x10
/* 8037C984 003798E4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C988 003798E8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C98C 003798EC  38 60 10 06 */	li r3, 0x1006
/* 8037C990 003798F0  38 00 00 84 */	li r0, 0x84
/* 8037C994 003798F4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C998 003798F8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C99C 003798FC  48 00 03 84 */	b lbl_8037CD20
/* 8037C9A0 00379900  38 00 00 10 */	li r0, 0x10
/* 8037C9A4 00379904  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C9A8 00379908  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C9AC 0037990C  38 60 10 06 */	li r3, 0x1006
/* 8037C9B0 00379910  38 00 00 C6 */	li r0, 0xc6
/* 8037C9B4 00379914  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C9B8 00379918  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C9BC 0037991C  48 00 03 64 */	b lbl_8037CD20
/* 8037C9C0 00379920  38 00 00 10 */	li r0, 0x10
/* 8037C9C4 00379924  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C9C8 00379928  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C9CC 0037992C  38 60 10 06 */	li r3, 0x1006
/* 8037C9D0 00379930  38 00 02 10 */	li r0, 0x210
/* 8037C9D4 00379934  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C9D8 00379938  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C9DC 0037993C  48 00 03 44 */	b lbl_8037CD20
/* 8037C9E0 00379940  38 00 00 10 */	li r0, 0x10
/* 8037C9E4 00379944  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037C9E8 00379948  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037C9EC 0037994C  38 60 10 06 */	li r3, 0x1006
/* 8037C9F0 00379950  38 00 02 52 */	li r0, 0x252
/* 8037C9F4 00379954  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037C9F8 00379958  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037C9FC 0037995C  48 00 03 24 */	b lbl_8037CD20
/* 8037CA00 00379960  38 00 00 10 */	li r0, 0x10
/* 8037CA04 00379964  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CA08 00379968  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CA0C 0037996C  38 60 10 06 */	li r3, 0x1006
/* 8037CA10 00379970  38 00 02 31 */	li r0, 0x231
/* 8037CA14 00379974  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037CA18 00379978  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CA1C 0037997C  48 00 03 04 */	b lbl_8037CD20
/* 8037CA20 00379980  38 00 00 10 */	li r0, 0x10
/* 8037CA24 00379984  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CA28 00379988  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CA2C 0037998C  38 60 10 06 */	li r3, 0x1006
/* 8037CA30 00379990  38 00 01 AD */	li r0, 0x1ad
/* 8037CA34 00379994  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037CA38 00379998  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CA3C 0037999C  48 00 02 E4 */	b lbl_8037CD20
/* 8037CA40 003799A0  38 00 00 10 */	li r0, 0x10
/* 8037CA44 003799A4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CA48 003799A8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CA4C 003799AC  38 60 10 06 */	li r3, 0x1006
/* 8037CA50 003799B0  38 00 01 CE */	li r0, 0x1ce
/* 8037CA54 003799B4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037CA58 003799B8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CA5C 003799BC  48 00 02 C4 */	b lbl_8037CD20
/* 8037CA60 003799C0  38 00 00 10 */	li r0, 0x10
/* 8037CA64 003799C4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CA68 003799C8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CA6C 003799CC  38 60 10 06 */	li r3, 0x1006
/* 8037CA70 003799D0  38 00 00 21 */	li r0, 0x21
/* 8037CA74 003799D4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037CA78 003799D8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CA7C 003799DC  48 00 02 A4 */	b lbl_8037CD20
/* 8037CA80 003799E0  38 00 00 10 */	li r0, 0x10
/* 8037CA84 003799E4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CA88 003799E8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CA8C 003799EC  38 60 10 06 */	li r3, 0x1006
/* 8037CA90 003799F0  38 00 01 53 */	li r0, 0x153
/* 8037CA94 003799F4  90 65 80 00 */	stw r3, 0xCC008000@l(r5)
/* 8037CA98 003799F8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CA9C 003799FC  48 00 02 84 */	b lbl_8037CD20
/* 8037CAA0 00379A00  38 00 00 61 */	li r0, 0x61
/* 8037CAA4 00379A04  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CAA8 00379A08  3C 60 23 01 */	lis r3, 0x2300AE7F@ha
/* 8037CAAC 00379A0C  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CAB0 00379A10  38 03 AE 7F */	addi r0, r3, 0x2300AE7F@l
/* 8037CAB4 00379A14  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CAB8 00379A18  48 00 02 68 */	b lbl_8037CD20
/* 8037CABC 00379A1C  38 00 00 61 */	li r0, 0x61
/* 8037CAC0 00379A20  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CAC4 00379A24  3C 60 23 01 */	lis r3, 0x23008E7F@ha
/* 8037CAC8 00379A28  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CACC 00379A2C  38 03 8E 7F */	addi r0, r3, 0x23008E7F@l
/* 8037CAD0 00379A30  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CAD4 00379A34  48 00 02 4C */	b lbl_8037CD20
/* 8037CAD8 00379A38  38 00 00 61 */	li r0, 0x61
/* 8037CADC 00379A3C  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CAE0 00379A40  3C 60 23 01 */	lis r3, 0x23009E7F@ha
/* 8037CAE4 00379A44  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CAE8 00379A48  38 03 9E 7F */	addi r0, r3, 0x23009E7F@l
/* 8037CAEC 00379A4C  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CAF0 00379A50  48 00 02 30 */	b lbl_8037CD20
/* 8037CAF4 00379A54  38 00 00 61 */	li r0, 0x61
/* 8037CAF8 00379A58  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CAFC 00379A5C  3C 60 23 00 */	lis r3, 0x23001E7F@ha
/* 8037CB00 00379A60  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB04 00379A64  38 03 1E 7F */	addi r0, r3, 0x23001E7F@l
/* 8037CB08 00379A68  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB0C 00379A6C  48 00 02 14 */	b lbl_8037CD20
/* 8037CB10 00379A70  38 00 00 61 */	li r0, 0x61
/* 8037CB14 00379A74  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CB18 00379A78  3C 60 23 01 */	lis r3, 0x2300AC3F@ha
/* 8037CB1C 00379A7C  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB20 00379A80  38 03 AC 3F */	addi r0, r3, 0x2300AC3F@l
/* 8037CB24 00379A84  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB28 00379A88  48 00 01 F8 */	b lbl_8037CD20
/* 8037CB2C 00379A8C  38 00 00 61 */	li r0, 0x61
/* 8037CB30 00379A90  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CB34 00379A94  3C 60 23 01 */	lis r3, 0x2300AC7F@ha
/* 8037CB38 00379A98  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB3C 00379A9C  38 03 AC 7F */	addi r0, r3, 0x2300AC7F@l
/* 8037CB40 00379AA0  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB44 00379AA4  48 00 01 DC */	b lbl_8037CD20
/* 8037CB48 00379AA8  38 00 00 61 */	li r0, 0x61
/* 8037CB4C 00379AAC  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CB50 00379AB0  3C 60 23 01 */	lis r3, 0x2300ACBF@ha
/* 8037CB54 00379AB4  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB58 00379AB8  38 03 AC BF */	addi r0, r3, 0x2300ACBF@l
/* 8037CB5C 00379ABC  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB60 00379AC0  48 00 01 C0 */	b lbl_8037CD20
/* 8037CB64 00379AC4  38 00 00 61 */	li r0, 0x61
/* 8037CB68 00379AC8  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CB6C 00379ACC  3C 60 23 01 */	lis r3, 0x2300ACFF@ha
/* 8037CB70 00379AD0  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB74 00379AD4  38 03 AC FF */	addi r0, r3, 0x2300ACFF@l
/* 8037CB78 00379AD8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB7C 00379ADC  48 00 01 A4 */	b lbl_8037CD20
/* 8037CB80 00379AE0  38 00 00 61 */	li r0, 0x61
/* 8037CB84 00379AE4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CB88 00379AE8  3C 60 23 01 */	lis r3, 0x2300AD3F@ha
/* 8037CB8C 00379AEC  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CB90 00379AF0  38 03 AD 3F */	addi r0, r3, 0x2300AD3F@l
/* 8037CB94 00379AF4  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CB98 00379AF8  48 00 01 88 */	b lbl_8037CD20
/* 8037CB9C 00379AFC  38 00 00 61 */	li r0, 0x61
/* 8037CBA0 00379B00  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CBA4 00379B04  3C 60 23 01 */	lis r3, 0x2300AD7F@ha
/* 8037CBA8 00379B08  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CBAC 00379B0C  38 03 AD 7F */	addi r0, r3, 0x2300AD7F@l
/* 8037CBB0 00379B10  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CBB4 00379B14  48 00 01 6C */	b lbl_8037CD20
/* 8037CBB8 00379B18  38 00 00 61 */	li r0, 0x61
/* 8037CBBC 00379B1C  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CBC0 00379B20  3C 60 23 01 */	lis r3, 0x2300ADBF@ha
/* 8037CBC4 00379B24  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CBC8 00379B28  38 03 AD BF */	addi r0, r3, 0x2300ADBF@l
/* 8037CBCC 00379B2C  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CBD0 00379B30  48 00 01 50 */	b lbl_8037CD20
/* 8037CBD4 00379B34  38 00 00 61 */	li r0, 0x61
/* 8037CBD8 00379B38  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CBDC 00379B3C  3C 60 23 01 */	lis r3, 0x2300ADFF@ha
/* 8037CBE0 00379B40  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CBE4 00379B44  38 03 AD FF */	addi r0, r3, 0x2300ADFF@l
/* 8037CBE8 00379B48  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CBEC 00379B4C  48 00 01 34 */	b lbl_8037CD20
/* 8037CBF0 00379B50  38 00 00 61 */	li r0, 0x61
/* 8037CBF4 00379B54  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CBF8 00379B58  3C 60 23 01 */	lis r3, 0x2300AE3F@ha
/* 8037CBFC 00379B5C  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC00 00379B60  38 03 AE 3F */	addi r0, r3, 0x2300AE3F@l
/* 8037CC04 00379B64  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC08 00379B68  48 00 01 18 */	b lbl_8037CD20
/* 8037CC0C 00379B6C  38 00 00 61 */	li r0, 0x61
/* 8037CC10 00379B70  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CC14 00379B74  3C 60 23 01 */	lis r3, 0x2300A27F@ha
/* 8037CC18 00379B78  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC1C 00379B7C  38 03 A2 7F */	addi r0, r3, 0x2300A27F@l
/* 8037CC20 00379B80  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC24 00379B84  48 00 00 FC */	b lbl_8037CD20
/* 8037CC28 00379B88  38 00 00 61 */	li r0, 0x61
/* 8037CC2C 00379B8C  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CC30 00379B90  3C 60 23 01 */	lis r3, 0x2300A67F@ha
/* 8037CC34 00379B94  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC38 00379B98  38 03 A6 7F */	addi r0, r3, 0x2300A67F@l
/* 8037CC3C 00379B9C  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC40 00379BA0  48 00 00 E0 */	b lbl_8037CD20
/* 8037CC44 00379BA4  38 00 00 61 */	li r0, 0x61
/* 8037CC48 00379BA8  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CC4C 00379BAC  3C 60 23 01 */	lis r3, 0x2300AA7F@ha
/* 8037CC50 00379BB0  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC54 00379BB4  38 03 AA 7F */	addi r0, r3, 0x2300AA7F@l
/* 8037CC58 00379BB8  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC5C 00379BBC  48 00 00 C4 */	b lbl_8037CD20
/* 8037CC60 00379BC0  38 00 00 61 */	li r0, 0x61
/* 8037CC64 00379BC4  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CC68 00379BC8  3C 60 24 03 */	lis r3, 0x2402C0C6@ha
/* 8037CC6C 00379BCC  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC70 00379BD0  38 03 C0 C6 */	addi r0, r3, 0x2402C0C6@l
/* 8037CC74 00379BD4  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC78 00379BD8  48 00 00 A8 */	b lbl_8037CD20
/* 8037CC7C 00379BDC  38 00 00 61 */	li r0, 0x61
/* 8037CC80 00379BE0  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CC84 00379BE4  3C 60 24 03 */	lis r3, 0x2402C16B@ha
/* 8037CC88 00379BE8  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CC8C 00379BEC  38 03 C1 6B */	addi r0, r3, 0x2402C16B@l
/* 8037CC90 00379BF0  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CC94 00379BF4  48 00 00 8C */	b lbl_8037CD20
/* 8037CC98 00379BF8  38 00 00 61 */	li r0, 0x61
/* 8037CC9C 00379BFC  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CCA0 00379C00  3C 60 24 03 */	lis r3, 0x2402C0E7@ha
/* 8037CCA4 00379C04  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CCA8 00379C08  38 03 C0 E7 */	addi r0, r3, 0x2402C0E7@l
/* 8037CCAC 00379C0C  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CCB0 00379C10  48 00 00 70 */	b lbl_8037CD20
/* 8037CCB4 00379C14  38 00 00 61 */	li r0, 0x61
/* 8037CCB8 00379C18  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CCBC 00379C1C  3C 60 24 03 */	lis r3, 0x2402C108@ha
/* 8037CCC0 00379C20  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CCC4 00379C24  38 03 C1 08 */	addi r0, r3, 0x2402C108@l
/* 8037CCC8 00379C28  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CCCC 00379C2C  48 00 00 54 */	b lbl_8037CD20
/* 8037CCD0 00379C30  38 00 00 61 */	li r0, 0x61
/* 8037CCD4 00379C34  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CCD8 00379C38  3C 60 24 03 */	lis r3, 0x2402C129@ha
/* 8037CCDC 00379C3C  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CCE0 00379C40  38 03 C1 29 */	addi r0, r3, 0x2402C129@l
/* 8037CCE4 00379C44  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CCE8 00379C48  48 00 00 38 */	b lbl_8037CD20
/* 8037CCEC 00379C4C  38 00 00 61 */	li r0, 0x61
/* 8037CCF0 00379C50  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CCF4 00379C54  3C 60 24 03 */	lis r3, 0x2402C14A@ha
/* 8037CCF8 00379C58  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CCFC 00379C5C  38 03 C1 4A */	addi r0, r3, 0x2402C14A@l
/* 8037CD00 00379C60  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
/* 8037CD04 00379C64  48 00 00 1C */	b lbl_8037CD20
/* 8037CD08 00379C68  38 00 00 61 */	li r0, 0x61
/* 8037CD0C 00379C6C  3C A0 CC 01 */	lis r5, 0xCC008000@ha
/* 8037CD10 00379C70  3C 60 24 03 */	lis r3, 0x2402C1AD@ha
/* 8037CD14 00379C74  98 05 80 00 */	stb r0, 0xCC008000@l(r5)
/* 8037CD18 00379C78  38 03 C1 AD */	addi r0, r3, 0x2402C1AD@l
/* 8037CD1C 00379C7C  90 05 80 00 */	stw r0, 0xCC008000@l(r5)
lbl_8037CD20:
/* 8037CD20 00379C80  80 62 CE 08 */	lwz r3, lbl_805AEB28@sda21(r2)
/* 8037CD24 00379C84  90 83 04 E8 */	stw r4, 0x4e8(r3)
/* 8037CD28 00379C88  80 03 04 E8 */	lwz r0, 0x4e8(r3)
/* 8037CD2C 00379C8C  28 00 00 16 */	cmplwi r0, 0x16
/* 8037CD30 00379C90  41 81 02 F0 */	bgt lbl_8037D020
/* 8037CD34 00379C94  3C 80 80 3F */	lis r4, lbl_803F0CB0@ha
/* 8037CD38 00379C98  38 84 0C B0 */	addi r4, r4, lbl_803F0CB0@l
/* 8037CD3C 00379C9C  54 00 10 3A */	slwi r0, r0, 2
/* 8037CD40 00379CA0  7C 04 00 2E */	lwzx r0, r4, r0
/* 8037CD44 00379CA4  7C 09 03 A6 */	mtctr r0
/* 8037CD48 00379CA8  4E 80 04 20 */	bctr 
/* 8037CD4C 00379CAC  38 00 00 61 */	li r0, 0x61
/* 8037CD50 00379CB0  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CD54 00379CB4  3C 60 67 00 */	lis r3, 0x67000042@ha
/* 8037CD58 00379CB8  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CD5C 00379CBC  38 03 00 42 */	addi r0, r3, 0x67000042@l
/* 8037CD60 00379CC0  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CD64 00379CC4  48 00 02 BC */	b lbl_8037D020
/* 8037CD68 00379CC8  38 00 00 61 */	li r0, 0x61
/* 8037CD6C 00379CCC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CD70 00379CD0  3C 60 67 00 */	lis r3, 0x67000084@ha
/* 8037CD74 00379CD4  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CD78 00379CD8  38 03 00 84 */	addi r0, r3, 0x67000084@l
/* 8037CD7C 00379CDC  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CD80 00379CE0  48 00 02 A0 */	b lbl_8037D020
/* 8037CD84 00379CE4  38 00 00 61 */	li r0, 0x61
/* 8037CD88 00379CE8  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CD8C 00379CEC  3C 60 67 00 */	lis r3, 0x67000063@ha
/* 8037CD90 00379CF0  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CD94 00379CF4  38 03 00 63 */	addi r0, r3, 0x67000063@l
/* 8037CD98 00379CF8  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CD9C 00379CFC  48 00 02 84 */	b lbl_8037D020
/* 8037CDA0 00379D00  38 00 00 61 */	li r0, 0x61
/* 8037CDA4 00379D04  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CDA8 00379D08  3C 60 67 00 */	lis r3, 0x67000129@ha
/* 8037CDAC 00379D0C  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CDB0 00379D10  38 03 01 29 */	addi r0, r3, 0x67000129@l
/* 8037CDB4 00379D14  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CDB8 00379D18  48 00 02 68 */	b lbl_8037D020
/* 8037CDBC 00379D1C  38 00 00 61 */	li r0, 0x61
/* 8037CDC0 00379D20  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CDC4 00379D24  3C 60 67 00 */	lis r3, 0x67000252@ha
/* 8037CDC8 00379D28  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CDCC 00379D2C  38 03 02 52 */	addi r0, r3, 0x67000252@l
/* 8037CDD0 00379D30  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CDD4 00379D34  48 00 02 4C */	b lbl_8037D020
/* 8037CDD8 00379D38  38 00 00 61 */	li r0, 0x61
/* 8037CDDC 00379D3C  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CDE0 00379D40  3C 60 67 00 */	lis r3, 0x67000021@ha
/* 8037CDE4 00379D44  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CDE8 00379D48  38 03 00 21 */	addi r0, r3, 0x67000021@l
/* 8037CDEC 00379D4C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CDF0 00379D50  48 00 02 30 */	b lbl_8037D020
/* 8037CDF4 00379D54  38 00 00 61 */	li r0, 0x61
/* 8037CDF8 00379D58  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CDFC 00379D5C  3C 60 67 00 */	lis r3, 0x6700014B@ha
/* 8037CE00 00379D60  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CE04 00379D64  38 03 01 4B */	addi r0, r3, 0x6700014B@l
/* 8037CE08 00379D68  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CE0C 00379D6C  48 00 02 14 */	b lbl_8037D020
/* 8037CE10 00379D70  38 00 00 61 */	li r0, 0x61
/* 8037CE14 00379D74  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CE18 00379D78  3C 60 67 00 */	lis r3, 0x6700018D@ha
/* 8037CE1C 00379D7C  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CE20 00379D80  38 03 01 8D */	addi r0, r3, 0x6700018D@l
/* 8037CE24 00379D84  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CE28 00379D88  48 00 01 F8 */	b lbl_8037D020
/* 8037CE2C 00379D8C  38 00 00 61 */	li r0, 0x61
/* 8037CE30 00379D90  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CE34 00379D94  3C 60 67 00 */	lis r3, 0x670001CF@ha
/* 8037CE38 00379D98  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CE3C 00379D9C  38 03 01 CF */	addi r0, r3, 0x670001CF@l
/* 8037CE40 00379DA0  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CE44 00379DA4  48 00 01 DC */	b lbl_8037D020
/* 8037CE48 00379DA8  38 00 00 61 */	li r0, 0x61
/* 8037CE4C 00379DAC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CE50 00379DB0  3C 60 67 00 */	lis r3, 0x67000211@ha
/* 8037CE54 00379DB4  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CE58 00379DB8  38 03 02 11 */	addi r0, r3, 0x67000211@l
/* 8037CE5C 00379DBC  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CE60 00379DC0  48 00 01 C0 */	b lbl_8037D020
/* 8037CE64 00379DC4  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CE68 00379DC8  38 A0 00 08 */	li r5, 8
/* 8037CE6C 00379DCC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CE70 00379DD0  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CE74 00379DD4  60 00 00 20 */	ori r0, r0, 0x20
/* 8037CE78 00379DD8  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CE7C 00379DDC  38 00 00 20 */	li r0, 0x20
/* 8037CE80 00379DE0  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CE84 00379DE4  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CE88 00379DE8  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CE8C 00379DEC  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CE90 00379DF0  48 00 01 90 */	b lbl_8037D020
/* 8037CE94 00379DF4  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CE98 00379DF8  38 A0 00 08 */	li r5, 8
/* 8037CE9C 00379DFC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CEA0 00379E00  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CEA4 00379E04  60 00 00 30 */	ori r0, r0, 0x30
/* 8037CEA8 00379E08  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CEAC 00379E0C  38 00 00 20 */	li r0, 0x20
/* 8037CEB0 00379E10  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CEB4 00379E14  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CEB8 00379E18  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CEBC 00379E1C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CEC0 00379E20  48 00 01 60 */	b lbl_8037D020
/* 8037CEC4 00379E24  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CEC8 00379E28  38 A0 00 08 */	li r5, 8
/* 8037CECC 00379E2C  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CED0 00379E30  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CED4 00379E34  60 00 00 40 */	ori r0, r0, 0x40
/* 8037CED8 00379E38  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CEDC 00379E3C  38 00 00 20 */	li r0, 0x20
/* 8037CEE0 00379E40  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CEE4 00379E44  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CEE8 00379E48  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CEEC 00379E4C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CEF0 00379E50  48 00 01 30 */	b lbl_8037D020
/* 8037CEF4 00379E54  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CEF8 00379E58  38 A0 00 08 */	li r5, 8
/* 8037CEFC 00379E5C  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CF00 00379E60  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CF04 00379E64  60 00 00 50 */	ori r0, r0, 0x50
/* 8037CF08 00379E68  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CF0C 00379E6C  38 00 00 20 */	li r0, 0x20
/* 8037CF10 00379E70  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CF14 00379E74  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CF18 00379E78  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF1C 00379E7C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CF20 00379E80  48 00 01 00 */	b lbl_8037D020
/* 8037CF24 00379E84  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF28 00379E88  38 A0 00 08 */	li r5, 8
/* 8037CF2C 00379E8C  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CF30 00379E90  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CF34 00379E94  60 00 00 60 */	ori r0, r0, 0x60
/* 8037CF38 00379E98  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CF3C 00379E9C  38 00 00 20 */	li r0, 0x20
/* 8037CF40 00379EA0  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CF44 00379EA4  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CF48 00379EA8  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF4C 00379EAC  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CF50 00379EB0  48 00 00 D0 */	b lbl_8037D020
/* 8037CF54 00379EB4  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF58 00379EB8  38 A0 00 08 */	li r5, 8
/* 8037CF5C 00379EBC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CF60 00379EC0  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CF64 00379EC4  60 00 00 70 */	ori r0, r0, 0x70
/* 8037CF68 00379EC8  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CF6C 00379ECC  38 00 00 20 */	li r0, 0x20
/* 8037CF70 00379ED0  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CF74 00379ED4  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CF78 00379ED8  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF7C 00379EDC  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CF80 00379EE0  48 00 00 A0 */	b lbl_8037D020
/* 8037CF84 00379EE4  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CF88 00379EE8  38 A0 00 08 */	li r5, 8
/* 8037CF8C 00379EEC  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CF90 00379EF0  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CF94 00379EF4  60 00 00 90 */	ori r0, r0, 0x90
/* 8037CF98 00379EF8  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CF9C 00379EFC  38 00 00 20 */	li r0, 0x20
/* 8037CFA0 00379F00  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CFA4 00379F04  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CFA8 00379F08  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CFAC 00379F0C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CFB0 00379F10  48 00 00 70 */	b lbl_8037D020
/* 8037CFB4 00379F14  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CFB8 00379F18  38 A0 00 08 */	li r5, 8
/* 8037CFBC 00379F1C  3C 80 CC 01 */	lis r4, 0xCC008000@ha
/* 8037CFC0 00379F20  54 00 07 2E */	rlwinm r0, r0, 0, 0x1c, 0x17
/* 8037CFC4 00379F24  60 00 00 80 */	ori r0, r0, 0x80
/* 8037CFC8 00379F28  90 03 04 EC */	stw r0, 0x4ec(r3)
/* 8037CFCC 00379F2C  38 00 00 20 */	li r0, 0x20
/* 8037CFD0 00379F30  98 A4 80 00 */	stb r5, 0xCC008000@l(r4)
/* 8037CFD4 00379F34  98 04 80 00 */	stb r0, 0xCC008000@l(r4)
/* 8037CFD8 00379F38  80 03 04 EC */	lwz r0, 0x4ec(r3)
/* 8037CFDC 00379F3C  90 04 80 00 */	stw r0, 0xCC008000@l(r4)
/* 8037CFE0 00379F40  48 00 00 40 */	b lbl_8037D020
/* 8037CFE4 00379F44  80 6D AC 7C */	lwz r3, lbl_805A983C@sda21(r13)
/* 8037CFE8 00379F48  38 00 00 02 */	li r0, 2
/* 8037CFEC 00379F4C  B0 03 00 06 */	sth r0, 6(r3)
/* 8037CFF0 00379F50  48 00 00 30 */	b lbl_8037D020
/* 8037CFF4 00379F54  80 6D AC 7C */	lwz r3, lbl_805A983C@sda21(r13)
/* 8037CFF8 00379F58  38 00 00 03 */	li r0, 3
/* 8037CFFC 00379F5C  B0 03 00 06 */	sth r0, 6(r3)
/* 8037D000 00379F60  48 00 00 20 */	b lbl_8037D020
/* 8037D004 00379F64  80 6D AC 7C */	lwz r3, lbl_805A983C@sda21(r13)
/* 8037D008 00379F68  38 00 00 04 */	li r0, 4
/* 8037D00C 00379F6C  B0 03 00 06 */	sth r0, 6(r3)
/* 8037D010 00379F70  48 00 00 10 */	b lbl_8037D020
/* 8037D014 00379F74  80 6D AC 7C */	lwz r3, lbl_805A983C@sda21(r13)
/* 8037D018 00379F78  38 00 00 05 */	li r0, 5
/* 8037D01C 00379F7C  B0 03 00 06 */	sth r0, 6(r3)
lbl_8037D020:
/* 8037D020 00379F80  80 62 CE 08 */	lwz r3, lbl_805AEB28@sda21(r2)
/* 8037D024 00379F84  38 00 00 00 */	li r0, 0
/* 8037D028 00379F88  B0 03 00 02 */	sth r0, 2(r3)
/* 8037D02C 00379F8C  4E 80 00 20 */	blr 

.global GXClearGPMetric
GXClearGPMetric:
/* 8037D030 00379F90  80 6D AC 7C */	lwz r3, lbl_805A983C@sda21(r13)
/* 8037D034 00379F94  38 00 00 04 */	li r0, 4
/* 8037D038 00379F98  B0 03 00 04 */	sth r0, 4(r3)
/* 8037D03C 00379F9C  4E 80 00 20 */	blr 
