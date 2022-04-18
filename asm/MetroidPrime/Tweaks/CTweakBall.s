.include "macros.inc"

.section .data

.global lbl_803DAD30
lbl_803DAD30:
	# ROM: 0x3D7D30
	.4byte 0
	.4byte 0
	.4byte __dt__10CTweakBallFv
	.4byte 0

.section .text, "ax"

.global __ct__10CTweakBallFR12CInputStream
__ct__10CTweakBallFR12CInputStream:
/* 80090B54 0008DAB4  94 21 FF C0 */	stwu r1, -0x40(r1)
/* 80090B58 0008DAB8  7C 08 02 A6 */	mflr r0
/* 80090B5C 0008DABC  3C C0 80 3E */	lis r6, lbl_803D9CC4@ha
/* 80090B60 0008DAC0  3C A0 80 3E */	lis r5, lbl_803DAD30@ha
/* 80090B64 0008DAC4  90 01 00 44 */	stw r0, 0x44(r1)
/* 80090B68 0008DAC8  38 C6 9C C4 */	addi r6, r6, lbl_803D9CC4@l
/* 80090B6C 0008DACC  38 05 AD 30 */	addi r0, r5, lbl_803DAD30@l
/* 80090B70 0008DAD0  C0 22 8A F0 */	lfs f1, lbl_805AA810@sda21(r2)
/* 80090B74 0008DAD4  93 E1 00 3C */	stw r31, 0x3c(r1)
/* 80090B78 0008DAD8  7C 7F 1B 78 */	mr r31, r3
/* 80090B7C 0008DADC  3C 60 80 5A */	lis r3, skZero3f@ha
/* 80090B80 0008DAE0  93 C1 00 38 */	stw r30, 0x38(r1)
/* 80090B84 0008DAE4  3B C0 00 00 */	li r30, 0
/* 80090B88 0008DAE8  93 A1 00 34 */	stw r29, 0x34(r1)
/* 80090B8C 0008DAEC  3B A0 00 00 */	li r29, 0
/* 80090B90 0008DAF0  93 81 00 30 */	stw r28, 0x30(r1)
/* 80090B94 0008DAF4  7C 9C 23 78 */	mr r28, r4
/* 80090B98 0008DAF8  90 DF 00 00 */	stw r6, 0(r31)
/* 80090B9C 0008DAFC  90 1F 00 00 */	stw r0, 0(r31)
/* 80090BA0 0008DB00  C4 03 66 A0 */	lfsu f0, skZero3f@l(r3)
/* 80090BA4 0008DB04  D0 1F 00 78 */	stfs f0, 0x78(r31)
/* 80090BA8 0008DB08  C0 03 00 04 */	lfs f0, 4(r3)
/* 80090BAC 0008DB0C  D0 1F 00 7C */	stfs f0, 0x7c(r31)
/* 80090BB0 0008DB10  C0 03 00 08 */	lfs f0, 8(r3)
/* 80090BB4 0008DB14  D0 1F 00 80 */	stfs f0, 0x80(r31)
/* 80090BB8 0008DB18  D0 3F 01 58 */	stfs f1, 0x158(r31)
/* 80090BBC 0008DB1C  C0 03 00 00 */	lfs f0, 0(r3)
/* 80090BC0 0008DB20  D0 1F 01 8C */	stfs f0, 0x18c(r31)
/* 80090BC4 0008DB24  C0 03 00 04 */	lfs f0, 4(r3)
/* 80090BC8 0008DB28  D0 1F 01 90 */	stfs f0, 0x190(r31)
/* 80090BCC 0008DB2C  C0 03 00 08 */	lfs f0, 8(r3)
/* 80090BD0 0008DB30  D0 1F 01 94 */	stfs f0, 0x194(r31)
/* 80090BD4 0008DB34  C0 03 00 00 */	lfs f0, 0(r3)
/* 80090BD8 0008DB38  D0 1F 01 B8 */	stfs f0, 0x1b8(r31)
/* 80090BDC 0008DB3C  C0 03 00 04 */	lfs f0, 4(r3)
/* 80090BE0 0008DB40  D0 1F 01 BC */	stfs f0, 0x1bc(r31)
/* 80090BE4 0008DB44  C0 03 00 08 */	lfs f0, 8(r3)
/* 80090BE8 0008DB48  D0 1F 01 C0 */	stfs f0, 0x1c0(r31)
/* 80090BEC 0008DB4C  D0 3F 01 EC */	stfs f1, 0x1ec(r31)
lbl_80090BF0:
/* 80090BF0 0008DB50  7F 83 E3 78 */	mr r3, r28
/* 80090BF4 0008DB54  48 2A E0 35 */	bl ReadFloat__12CInputStreamFv
/* 80090BF8 0008DB58  3B BD 00 01 */	addi r29, r29, 1
/* 80090BFC 0008DB5C  38 1E 00 04 */	addi r0, r30, 4
/* 80090C00 0008DB60  2C 1D 00 08 */	cmpwi r29, 8
/* 80090C04 0008DB64  7C 3F 05 2E */	stfsx f1, r31, r0
/* 80090C08 0008DB68  3B DE 00 04 */	addi r30, r30, 4
/* 80090C0C 0008DB6C  41 80 FF E4 */	blt lbl_80090BF0
/* 80090C10 0008DB70  3B A0 00 00 */	li r29, 0
/* 80090C14 0008DB74  7F BE EB 78 */	mr r30, r29
lbl_80090C18:
/* 80090C18 0008DB78  7F 83 E3 78 */	mr r3, r28
/* 80090C1C 0008DB7C  48 2A E0 0D */	bl ReadFloat__12CInputStreamFv
/* 80090C20 0008DB80  3B BD 00 01 */	addi r29, r29, 1
/* 80090C24 0008DB84  38 1E 00 24 */	addi r0, r30, 0x24
/* 80090C28 0008DB88  2C 1D 00 08 */	cmpwi r29, 8
/* 80090C2C 0008DB8C  7C 3F 05 2E */	stfsx f1, r31, r0
/* 80090C30 0008DB90  3B DE 00 04 */	addi r30, r30, 4
/* 80090C34 0008DB94  41 80 FF E4 */	blt lbl_80090C18
/* 80090C38 0008DB98  3B A0 00 00 */	li r29, 0
/* 80090C3C 0008DB9C  7F BE EB 78 */	mr r30, r29
lbl_80090C40:
/* 80090C40 0008DBA0  7F 83 E3 78 */	mr r3, r28
/* 80090C44 0008DBA4  48 2A DF E5 */	bl ReadFloat__12CInputStreamFv
/* 80090C48 0008DBA8  3B BD 00 01 */	addi r29, r29, 1
/* 80090C4C 0008DBAC  38 1E 00 44 */	addi r0, r30, 0x44
/* 80090C50 0008DBB0  2C 1D 00 08 */	cmpwi r29, 8
/* 80090C54 0008DBB4  7C 3F 05 2E */	stfsx f1, r31, r0
/* 80090C58 0008DBB8  3B DE 00 04 */	addi r30, r30, 4
/* 80090C5C 0008DBBC  41 80 FF E4 */	blt lbl_80090C40
/* 80090C60 0008DBC0  7F 83 E3 78 */	mr r3, r28
/* 80090C64 0008DBC4  48 2A DF C5 */	bl ReadFloat__12CInputStreamFv
/* 80090C68 0008DBC8  D0 3F 00 64 */	stfs f1, 0x64(r31)
/* 80090C6C 0008DBCC  7F 83 E3 78 */	mr r3, r28
/* 80090C70 0008DBD0  48 2A DF B9 */	bl ReadFloat__12CInputStreamFv
/* 80090C74 0008DBD4  D0 3F 00 68 */	stfs f1, 0x68(r31)
/* 80090C78 0008DBD8  7F 83 E3 78 */	mr r3, r28
/* 80090C7C 0008DBDC  48 2A DF AD */	bl ReadFloat__12CInputStreamFv
/* 80090C80 0008DBE0  FC 00 08 50 */	fneg f0, f1
/* 80090C84 0008DBE4  7F 83 E3 78 */	mr r3, r28
/* 80090C88 0008DBE8  D0 1F 00 6C */	stfs f0, 0x6c(r31)
/* 80090C8C 0008DBEC  48 2A DF 9D */	bl ReadFloat__12CInputStreamFv
/* 80090C90 0008DBF0  FC 00 08 50 */	fneg f0, f1
/* 80090C94 0008DBF4  3B A0 00 00 */	li r29, 0
/* 80090C98 0008DBF8  7F BE EB 78 */	mr r30, r29
/* 80090C9C 0008DBFC  D0 1F 00 70 */	stfs f0, 0x70(r31)
lbl_80090CA0:
/* 80090CA0 0008DC00  7F 83 E3 78 */	mr r3, r28
/* 80090CA4 0008DC04  48 2A DF 85 */	bl ReadFloat__12CInputStreamFv
/* 80090CA8 0008DC08  3B BD 00 01 */	addi r29, r29, 1
/* 80090CAC 0008DC0C  38 1E 00 C4 */	addi r0, r30, 0xc4
/* 80090CB0 0008DC10  2C 1D 00 08 */	cmpwi r29, 8
/* 80090CB4 0008DC14  7C 3F 05 2E */	stfsx f1, r31, r0
/* 80090CB8 0008DC18  3B DE 00 04 */	addi r30, r30, 4
/* 80090CBC 0008DC1C  41 80 FF E4 */	blt lbl_80090CA0
/* 80090CC0 0008DC20  7F 83 E3 78 */	mr r3, r28
/* 80090CC4 0008DC24  48 2A DF 65 */	bl ReadFloat__12CInputStreamFv
/* 80090CC8 0008DC28  FC 00 08 50 */	fneg f0, f1
/* 80090CCC 0008DC2C  7F 83 E3 78 */	mr r3, r28
/* 80090CD0 0008DC30  D0 1F 00 E4 */	stfs f0, 0xe4(r31)
/* 80090CD4 0008DC34  48 2A DF 55 */	bl ReadFloat__12CInputStreamFv
/* 80090CD8 0008DC38  FC 00 08 50 */	fneg f0, f1
/* 80090CDC 0008DC3C  7F 83 E3 78 */	mr r3, r28
/* 80090CE0 0008DC40  D0 1F 00 E8 */	stfs f0, 0xe8(r31)
/* 80090CE4 0008DC44  48 2A DF 45 */	bl ReadFloat__12CInputStreamFv
/* 80090CE8 0008DC48  D0 3F 01 4C */	stfs f1, 0x14c(r31)
/* 80090CEC 0008DC4C  7F 83 E3 78 */	mr r3, r28
/* 80090CF0 0008DC50  48 2A DF 39 */	bl ReadFloat__12CInputStreamFv
/* 80090CF4 0008DC54  D0 3F 01 50 */	stfs f1, 0x150(r31)
/* 80090CF8 0008DC58  7F 83 E3 78 */	mr r3, r28
/* 80090CFC 0008DC5C  48 2A DF 2D */	bl ReadFloat__12CInputStreamFv
/* 80090D00 0008DC60  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090D04 0008DC64  7F 83 E3 78 */	mr r3, r28
/* 80090D08 0008DC68  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090D0C 0008DC6C  D0 1F 01 58 */	stfs f0, 0x158(r31)
/* 80090D10 0008DC70  48 2A DF 19 */	bl ReadFloat__12CInputStreamFv
/* 80090D14 0008DC74  D0 3F 01 DC */	stfs f1, 0x1dc(r31)
/* 80090D18 0008DC78  7F 83 E3 78 */	mr r3, r28
/* 80090D1C 0008DC7C  48 2A DF 0D */	bl ReadFloat__12CInputStreamFv
/* 80090D20 0008DC80  D0 3F 01 E0 */	stfs f1, 0x1e0(r31)
/* 80090D24 0008DC84  7F 83 E3 78 */	mr r3, r28
/* 80090D28 0008DC88  48 2A DF 01 */	bl ReadFloat__12CInputStreamFv
/* 80090D2C 0008DC8C  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090D30 0008DC90  7F 83 E3 78 */	mr r3, r28
/* 80090D34 0008DC94  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090D38 0008DC98  D0 1F 01 EC */	stfs f0, 0x1ec(r31)
/* 80090D3C 0008DC9C  48 2A DE ED */	bl ReadFloat__12CInputStreamFv
/* 80090D40 0008DCA0  D0 3F 01 F0 */	stfs f1, 0x1f0(r31)
/* 80090D44 0008DCA4  7F 83 E3 78 */	mr r3, r28
/* 80090D48 0008DCA8  48 2A DE E1 */	bl ReadFloat__12CInputStreamFv
/* 80090D4C 0008DCAC  D0 3F 01 F4 */	stfs f1, 0x1f4(r31)
/* 80090D50 0008DCB0  7F 83 E3 78 */	mr r3, r28
/* 80090D54 0008DCB4  48 2A DE D5 */	bl ReadFloat__12CInputStreamFv
/* 80090D58 0008DCB8  D0 3F 01 F8 */	stfs f1, 0x1f8(r31)
/* 80090D5C 0008DCBC  7F 83 E3 78 */	mr r3, r28
/* 80090D60 0008DCC0  48 2A DE C9 */	bl ReadFloat__12CInputStreamFv
/* 80090D64 0008DCC4  D0 3F 01 FC */	stfs f1, 0x1fc(r31)
/* 80090D68 0008DCC8  7F 83 E3 78 */	mr r3, r28
/* 80090D6C 0008DCCC  C0 62 8A F8 */	lfs f3, lbl_805AA818@sda21(r2)
/* 80090D70 0008DCD0  C0 42 8A FC */	lfs f2, lbl_805AA81C@sda21(r2)
/* 80090D74 0008DCD4  D0 7F 01 2C */	stfs f3, 0x12c(r31)
/* 80090D78 0008DCD8  C0 22 8B 00 */	lfs f1, lbl_805AA820@sda21(r2)
/* 80090D7C 0008DCDC  D0 7F 01 30 */	stfs f3, 0x130(r31)
/* 80090D80 0008DCE0  C0 02 8B 04 */	lfs f0, lbl_805AA824@sda21(r2)
/* 80090D84 0008DCE4  D0 5F 01 34 */	stfs f2, 0x134(r31)
/* 80090D88 0008DCE8  D0 7F 01 38 */	stfs f3, 0x138(r31)
/* 80090D8C 0008DCEC  D0 3F 01 3C */	stfs f1, 0x13c(r31)
/* 80090D90 0008DCF0  D0 3F 01 40 */	stfs f1, 0x140(r31)
/* 80090D94 0008DCF4  D0 3F 01 44 */	stfs f1, 0x144(r31)
/* 80090D98 0008DCF8  D0 3F 01 48 */	stfs f1, 0x148(r31)
/* 80090D9C 0008DCFC  D0 7F 00 EC */	stfs f3, 0xec(r31)
/* 80090DA0 0008DD00  D0 5F 00 F0 */	stfs f2, 0xf0(r31)
/* 80090DA4 0008DD04  D0 1F 00 F4 */	stfs f0, 0xf4(r31)
/* 80090DA8 0008DD08  D0 1F 00 F8 */	stfs f0, 0xf8(r31)
/* 80090DAC 0008DD0C  D0 1F 00 FC */	stfs f0, 0xfc(r31)
/* 80090DB0 0008DD10  D0 1F 01 00 */	stfs f0, 0x100(r31)
/* 80090DB4 0008DD14  D0 1F 01 04 */	stfs f0, 0x104(r31)
/* 80090DB8 0008DD18  D0 1F 01 08 */	stfs f0, 0x108(r31)
/* 80090DBC 0008DD1C  D0 7F 01 0C */	stfs f3, 0x10c(r31)
/* 80090DC0 0008DD20  D0 5F 01 10 */	stfs f2, 0x110(r31)
/* 80090DC4 0008DD24  D0 1F 01 14 */	stfs f0, 0x114(r31)
/* 80090DC8 0008DD28  D0 1F 01 18 */	stfs f0, 0x118(r31)
/* 80090DCC 0008DD2C  D0 1F 01 1C */	stfs f0, 0x11c(r31)
/* 80090DD0 0008DD30  D0 1F 01 20 */	stfs f0, 0x120(r31)
/* 80090DD4 0008DD34  D0 1F 01 24 */	stfs f0, 0x124(r31)
/* 80090DD8 0008DD38  D0 1F 01 28 */	stfs f0, 0x128(r31)
/* 80090DDC 0008DD3C  48 2A DE 4D */	bl ReadFloat__12CInputStreamFv
/* 80090DE0 0008DD40  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090DE4 0008DD44  7F 84 E3 78 */	mr r4, r28
/* 80090DE8 0008DD48  38 61 00 20 */	addi r3, r1, 0x20
/* 80090DEC 0008DD4C  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090DF0 0008DD50  D0 1F 00 74 */	stfs f0, 0x74(r31)
/* 80090DF4 0008DD54  48 28 3D 99 */	bl __ct__9CVector3fFR12CInputStream
/* 80090DF8 0008DD58  C0 01 00 20 */	lfs f0, 0x20(r1)
/* 80090DFC 0008DD5C  7F 83 E3 78 */	mr r3, r28
/* 80090E00 0008DD60  D0 1F 00 78 */	stfs f0, 0x78(r31)
/* 80090E04 0008DD64  C0 01 00 24 */	lfs f0, 0x24(r1)
/* 80090E08 0008DD68  D0 1F 00 7C */	stfs f0, 0x7c(r31)
/* 80090E0C 0008DD6C  C0 01 00 28 */	lfs f0, 0x28(r1)
/* 80090E10 0008DD70  D0 1F 00 80 */	stfs f0, 0x80(r31)
/* 80090E14 0008DD74  48 2A DE 15 */	bl ReadFloat__12CInputStreamFv
/* 80090E18 0008DD78  D0 3F 00 84 */	stfs f1, 0x84(r31)
/* 80090E1C 0008DD7C  7F 83 E3 78 */	mr r3, r28
/* 80090E20 0008DD80  48 2A DE 09 */	bl ReadFloat__12CInputStreamFv
/* 80090E24 0008DD84  D0 3F 00 88 */	stfs f1, 0x88(r31)
/* 80090E28 0008DD88  7F 83 E3 78 */	mr r3, r28
/* 80090E2C 0008DD8C  48 2A DD FD */	bl ReadFloat__12CInputStreamFv
/* 80090E30 0008DD90  D0 3F 00 8C */	stfs f1, 0x8c(r31)
/* 80090E34 0008DD94  7F 83 E3 78 */	mr r3, r28
/* 80090E38 0008DD98  48 2A DD F1 */	bl ReadFloat__12CInputStreamFv
/* 80090E3C 0008DD9C  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090E40 0008DDA0  7F 83 E3 78 */	mr r3, r28
/* 80090E44 0008DDA4  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090E48 0008DDA8  D0 1F 00 90 */	stfs f0, 0x90(r31)
/* 80090E4C 0008DDAC  48 2A DD DD */	bl ReadFloat__12CInputStreamFv
/* 80090E50 0008DDB0  D0 3F 00 94 */	stfs f1, 0x94(r31)
/* 80090E54 0008DDB4  7F 83 E3 78 */	mr r3, r28
/* 80090E58 0008DDB8  48 2A DD D1 */	bl ReadFloat__12CInputStreamFv
/* 80090E5C 0008DDBC  D0 3F 00 98 */	stfs f1, 0x98(r31)
/* 80090E60 0008DDC0  7F 83 E3 78 */	mr r3, r28
/* 80090E64 0008DDC4  48 2A DD C5 */	bl ReadFloat__12CInputStreamFv
/* 80090E68 0008DDC8  D0 3F 00 9C */	stfs f1, 0x9c(r31)
/* 80090E6C 0008DDCC  7F 83 E3 78 */	mr r3, r28
/* 80090E70 0008DDD0  48 2A DD B9 */	bl ReadFloat__12CInputStreamFv
/* 80090E74 0008DDD4  D0 3F 00 A0 */	stfs f1, 0xa0(r31)
/* 80090E78 0008DDD8  7F 83 E3 78 */	mr r3, r28
/* 80090E7C 0008DDDC  48 2A DD AD */	bl ReadFloat__12CInputStreamFv
/* 80090E80 0008DDE0  D0 3F 00 A4 */	stfs f1, 0xa4(r31)
/* 80090E84 0008DDE4  7F 83 E3 78 */	mr r3, r28
/* 80090E88 0008DDE8  48 2A DD A1 */	bl ReadFloat__12CInputStreamFv
/* 80090E8C 0008DDEC  D0 3F 00 A8 */	stfs f1, 0xa8(r31)
/* 80090E90 0008DDF0  7F 83 E3 78 */	mr r3, r28
/* 80090E94 0008DDF4  48 2A DD 95 */	bl ReadFloat__12CInputStreamFv
/* 80090E98 0008DDF8  D0 3F 00 AC */	stfs f1, 0xac(r31)
/* 80090E9C 0008DDFC  7F 83 E3 78 */	mr r3, r28
/* 80090EA0 0008DE00  48 2A DD 89 */	bl ReadFloat__12CInputStreamFv
/* 80090EA4 0008DE04  D0 3F 00 B0 */	stfs f1, 0xb0(r31)
/* 80090EA8 0008DE08  7F 83 E3 78 */	mr r3, r28
/* 80090EAC 0008DE0C  48 2A DD 7D */	bl ReadFloat__12CInputStreamFv
/* 80090EB0 0008DE10  D0 3F 00 B4 */	stfs f1, 0xb4(r31)
/* 80090EB4 0008DE14  7F 83 E3 78 */	mr r3, r28
/* 80090EB8 0008DE18  48 2A DD 71 */	bl ReadFloat__12CInputStreamFv
/* 80090EBC 0008DE1C  D0 3F 00 B8 */	stfs f1, 0xb8(r31)
/* 80090EC0 0008DE20  7F 83 E3 78 */	mr r3, r28
/* 80090EC4 0008DE24  48 2A DD 65 */	bl ReadFloat__12CInputStreamFv
/* 80090EC8 0008DE28  D0 3F 00 BC */	stfs f1, 0xbc(r31)
/* 80090ECC 0008DE2C  7F 83 E3 78 */	mr r3, r28
/* 80090ED0 0008DE30  48 2A DD 59 */	bl ReadFloat__12CInputStreamFv
/* 80090ED4 0008DE34  D0 3F 00 C0 */	stfs f1, 0xc0(r31)
/* 80090ED8 0008DE38  7F 83 E3 78 */	mr r3, r28
/* 80090EDC 0008DE3C  48 2A DD 4D */	bl ReadFloat__12CInputStreamFv
/* 80090EE0 0008DE40  D0 3F 01 54 */	stfs f1, 0x154(r31)
/* 80090EE4 0008DE44  7F 83 E3 78 */	mr r3, r28
/* 80090EE8 0008DE48  48 2A DD 41 */	bl ReadFloat__12CInputStreamFv
/* 80090EEC 0008DE4C  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090EF0 0008DE50  7F 83 E3 78 */	mr r3, r28
/* 80090EF4 0008DE54  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090EF8 0008DE58  D0 1F 01 5C */	stfs f0, 0x15c(r31)
/* 80090EFC 0008DE5C  48 2A DD 2D */	bl ReadFloat__12CInputStreamFv
/* 80090F00 0008DE60  D0 3F 01 60 */	stfs f1, 0x160(r31)
/* 80090F04 0008DE64  7F 83 E3 78 */	mr r3, r28
/* 80090F08 0008DE68  48 2A DD 21 */	bl ReadFloat__12CInputStreamFv
/* 80090F0C 0008DE6C  D0 3F 01 64 */	stfs f1, 0x164(r31)
/* 80090F10 0008DE70  7F 83 E3 78 */	mr r3, r28
/* 80090F14 0008DE74  48 2A DD 15 */	bl ReadFloat__12CInputStreamFv
/* 80090F18 0008DE78  D0 3F 01 68 */	stfs f1, 0x168(r31)
/* 80090F1C 0008DE7C  7F 83 E3 78 */	mr r3, r28
/* 80090F20 0008DE80  48 2A DD 09 */	bl ReadFloat__12CInputStreamFv
/* 80090F24 0008DE84  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090F28 0008DE88  7F 83 E3 78 */	mr r3, r28
/* 80090F2C 0008DE8C  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090F30 0008DE90  D0 1F 01 6C */	stfs f0, 0x16c(r31)
/* 80090F34 0008DE94  48 2A DC F5 */	bl ReadFloat__12CInputStreamFv
/* 80090F38 0008DE98  D0 3F 01 70 */	stfs f1, 0x170(r31)
/* 80090F3C 0008DE9C  7F 83 E3 78 */	mr r3, r28
/* 80090F40 0008DEA0  48 2A DC E9 */	bl ReadFloat__12CInputStreamFv
/* 80090F44 0008DEA4  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090F48 0008DEA8  7F 83 E3 78 */	mr r3, r28
/* 80090F4C 0008DEAC  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090F50 0008DEB0  D0 1F 01 74 */	stfs f0, 0x174(r31)
/* 80090F54 0008DEB4  48 2A DC D5 */	bl ReadFloat__12CInputStreamFv
/* 80090F58 0008DEB8  D0 3F 01 78 */	stfs f1, 0x178(r31)
/* 80090F5C 0008DEBC  7F 83 E3 78 */	mr r3, r28
/* 80090F60 0008DEC0  48 2A DC C9 */	bl ReadFloat__12CInputStreamFv
/* 80090F64 0008DEC4  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090F68 0008DEC8  7F 83 E3 78 */	mr r3, r28
/* 80090F6C 0008DECC  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090F70 0008DED0  D0 1F 01 7C */	stfs f0, 0x17c(r31)
/* 80090F74 0008DED4  48 2A DC B5 */	bl ReadFloat__12CInputStreamFv
/* 80090F78 0008DED8  D0 3F 01 80 */	stfs f1, 0x180(r31)
/* 80090F7C 0008DEDC  7F 83 E3 78 */	mr r3, r28
/* 80090F80 0008DEE0  48 2A DC A9 */	bl ReadFloat__12CInputStreamFv
/* 80090F84 0008DEE4  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090F88 0008DEE8  7F 83 E3 78 */	mr r3, r28
/* 80090F8C 0008DEEC  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090F90 0008DEF0  D0 1F 01 84 */	stfs f0, 0x184(r31)
/* 80090F94 0008DEF4  48 2A DC 95 */	bl ReadFloat__12CInputStreamFv
/* 80090F98 0008DEF8  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80090F9C 0008DEFC  7F 84 E3 78 */	mr r4, r28
/* 80090FA0 0008DF00  38 61 00 14 */	addi r3, r1, 0x14
/* 80090FA4 0008DF04  EC 00 00 72 */	fmuls f0, f0, f1
/* 80090FA8 0008DF08  D0 1F 01 88 */	stfs f0, 0x188(r31)
/* 80090FAC 0008DF0C  48 28 3B E1 */	bl __ct__9CVector3fFR12CInputStream
/* 80090FB0 0008DF10  C0 01 00 14 */	lfs f0, 0x14(r1)
/* 80090FB4 0008DF14  7F 83 E3 78 */	mr r3, r28
/* 80090FB8 0008DF18  D0 1F 01 8C */	stfs f0, 0x18c(r31)
/* 80090FBC 0008DF1C  C0 01 00 18 */	lfs f0, 0x18(r1)
/* 80090FC0 0008DF20  D0 1F 01 90 */	stfs f0, 0x190(r31)
/* 80090FC4 0008DF24  C0 01 00 1C */	lfs f0, 0x1c(r1)
/* 80090FC8 0008DF28  D0 1F 01 94 */	stfs f0, 0x194(r31)
/* 80090FCC 0008DF2C  48 2A DC 5D */	bl ReadFloat__12CInputStreamFv
/* 80090FD0 0008DF30  D0 3F 01 98 */	stfs f1, 0x198(r31)
/* 80090FD4 0008DF34  7F 83 E3 78 */	mr r3, r28
/* 80090FD8 0008DF38  48 2A DC 51 */	bl ReadFloat__12CInputStreamFv
/* 80090FDC 0008DF3C  D0 3F 01 9C */	stfs f1, 0x19c(r31)
/* 80090FE0 0008DF40  7F 83 E3 78 */	mr r3, r28
/* 80090FE4 0008DF44  48 2A DC 45 */	bl ReadFloat__12CInputStreamFv
/* 80090FE8 0008DF48  D0 3F 01 A0 */	stfs f1, 0x1a0(r31)
/* 80090FEC 0008DF4C  7F 83 E3 78 */	mr r3, r28
/* 80090FF0 0008DF50  48 2A DC 39 */	bl ReadFloat__12CInputStreamFv
/* 80090FF4 0008DF54  D0 3F 01 A4 */	stfs f1, 0x1a4(r31)
/* 80090FF8 0008DF58  7F 83 E3 78 */	mr r3, r28
/* 80090FFC 0008DF5C  48 2A DC 2D */	bl ReadFloat__12CInputStreamFv
/* 80091000 0008DF60  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80091004 0008DF64  7F 83 E3 78 */	mr r3, r28
/* 80091008 0008DF68  EC 00 00 72 */	fmuls f0, f0, f1
/* 8009100C 0008DF6C  D0 1F 01 A8 */	stfs f0, 0x1a8(r31)
/* 80091010 0008DF70  48 2A DC 19 */	bl ReadFloat__12CInputStreamFv
/* 80091014 0008DF74  D0 3F 01 AC */	stfs f1, 0x1ac(r31)
/* 80091018 0008DF78  7F 83 E3 78 */	mr r3, r28
/* 8009101C 0008DF7C  48 2A DC 0D */	bl ReadFloat__12CInputStreamFv
/* 80091020 0008DF80  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80091024 0008DF84  7F 83 E3 78 */	mr r3, r28
/* 80091028 0008DF88  EC 00 00 72 */	fmuls f0, f0, f1
/* 8009102C 0008DF8C  D0 1F 01 B0 */	stfs f0, 0x1b0(r31)
/* 80091030 0008DF90  48 2A DB F9 */	bl ReadFloat__12CInputStreamFv
/* 80091034 0008DF94  C0 02 8A F4 */	lfs f0, lbl_805AA814@sda21(r2)
/* 80091038 0008DF98  7F 84 E3 78 */	mr r4, r28
/* 8009103C 0008DF9C  38 61 00 08 */	addi r3, r1, 8
/* 80091040 0008DFA0  EC 00 00 72 */	fmuls f0, f0, f1
/* 80091044 0008DFA4  D0 1F 01 B4 */	stfs f0, 0x1b4(r31)
/* 80091048 0008DFA8  48 28 3B 45 */	bl __ct__9CVector3fFR12CInputStream
/* 8009104C 0008DFAC  C0 01 00 08 */	lfs f0, 8(r1)
/* 80091050 0008DFB0  7F 83 E3 78 */	mr r3, r28
/* 80091054 0008DFB4  D0 1F 01 B8 */	stfs f0, 0x1b8(r31)
/* 80091058 0008DFB8  C0 01 00 0C */	lfs f0, 0xc(r1)
/* 8009105C 0008DFBC  D0 1F 01 BC */	stfs f0, 0x1bc(r31)
/* 80091060 0008DFC0  C0 01 00 10 */	lfs f0, 0x10(r1)
/* 80091064 0008DFC4  D0 1F 01 C0 */	stfs f0, 0x1c0(r31)
/* 80091068 0008DFC8  48 2A DB C1 */	bl ReadFloat__12CInputStreamFv
/* 8009106C 0008DFCC  D0 3F 01 C4 */	stfs f1, 0x1c4(r31)
/* 80091070 0008DFD0  7F 83 E3 78 */	mr r3, r28
/* 80091074 0008DFD4  48 2A DB B5 */	bl ReadFloat__12CInputStreamFv
/* 80091078 0008DFD8  D0 3F 01 C8 */	stfs f1, 0x1c8(r31)
/* 8009107C 0008DFDC  7F 83 E3 78 */	mr r3, r28
/* 80091080 0008DFE0  48 2A DB A9 */	bl ReadFloat__12CInputStreamFv
/* 80091084 0008DFE4  D0 3F 01 CC */	stfs f1, 0x1cc(r31)
/* 80091088 0008DFE8  7F 83 E3 78 */	mr r3, r28
/* 8009108C 0008DFEC  48 2A DB 9D */	bl ReadFloat__12CInputStreamFv
/* 80091090 0008DFF0  D0 3F 01 D0 */	stfs f1, 0x1d0(r31)
/* 80091094 0008DFF4  7F 83 E3 78 */	mr r3, r28
/* 80091098 0008DFF8  48 2A DB 91 */	bl ReadFloat__12CInputStreamFv
/* 8009109C 0008DFFC  D0 3F 01 D4 */	stfs f1, 0x1d4(r31)
/* 800910A0 0008E000  7F 83 E3 78 */	mr r3, r28
/* 800910A4 0008E004  48 2A DB 85 */	bl ReadFloat__12CInputStreamFv
/* 800910A8 0008E008  D0 3F 01 D8 */	stfs f1, 0x1d8(r31)
/* 800910AC 0008E00C  7F 83 E3 78 */	mr r3, r28
/* 800910B0 0008E010  48 2A DB 79 */	bl ReadFloat__12CInputStreamFv
/* 800910B4 0008E014  D0 3F 01 E4 */	stfs f1, 0x1e4(r31)
/* 800910B8 0008E018  7F 83 E3 78 */	mr r3, r28
/* 800910BC 0008E01C  48 2A DB 6D */	bl ReadFloat__12CInputStreamFv
/* 800910C0 0008E020  D0 3F 01 E8 */	stfs f1, 0x1e8(r31)
/* 800910C4 0008E024  7F 83 E3 78 */	mr r3, r28
/* 800910C8 0008E028  48 2A DB 61 */	bl ReadFloat__12CInputStreamFv
/* 800910CC 0008E02C  D0 3F 02 00 */	stfs f1, 0x200(r31)
/* 800910D0 0008E030  7F 83 E3 78 */	mr r3, r28
/* 800910D4 0008E034  48 2A DB 55 */	bl ReadFloat__12CInputStreamFv
/* 800910D8 0008E038  D0 3F 02 04 */	stfs f1, 0x204(r31)
/* 800910DC 0008E03C  7F 83 E3 78 */	mr r3, r28
/* 800910E0 0008E040  48 2A DB 49 */	bl ReadFloat__12CInputStreamFv
/* 800910E4 0008E044  D0 3F 02 0C */	stfs f1, 0x20c(r31)
/* 800910E8 0008E048  7F 83 E3 78 */	mr r3, r28
/* 800910EC 0008E04C  48 2A DB 3D */	bl ReadFloat__12CInputStreamFv
/* 800910F0 0008E050  D0 3F 02 18 */	stfs f1, 0x218(r31)
/* 800910F4 0008E054  7F 83 E3 78 */	mr r3, r28
/* 800910F8 0008E058  48 2A DB 31 */	bl ReadFloat__12CInputStreamFv
/* 800910FC 0008E05C  D0 3F 02 1C */	stfs f1, 0x21c(r31)
/* 80091100 0008E060  7F 83 E3 78 */	mr r3, r28
/* 80091104 0008E064  48 2A DB 25 */	bl ReadFloat__12CInputStreamFv
/* 80091108 0008E068  D0 3F 02 20 */	stfs f1, 0x220(r31)
/* 8009110C 0008E06C  7F 83 E3 78 */	mr r3, r28
/* 80091110 0008E070  48 2A DB 19 */	bl ReadFloat__12CInputStreamFv
/* 80091114 0008E074  D0 3F 02 24 */	stfs f1, 0x224(r31)
/* 80091118 0008E078  7F 83 E3 78 */	mr r3, r28
/* 8009111C 0008E07C  48 2A DB 0D */	bl ReadFloat__12CInputStreamFv
/* 80091120 0008E080  D0 3F 02 10 */	stfs f1, 0x210(r31)
/* 80091124 0008E084  7F 83 E3 78 */	mr r3, r28
/* 80091128 0008E088  C0 1F 02 10 */	lfs f0, 0x210(r31)
/* 8009112C 0008E08C  D0 1F 02 28 */	stfs f0, 0x228(r31)
/* 80091130 0008E090  48 2A DA F9 */	bl ReadFloat__12CInputStreamFv
/* 80091134 0008E094  D0 3F 02 2C */	stfs f1, 0x22c(r31)
/* 80091138 0008E098  7F 83 E3 78 */	mr r3, r28
/* 8009113C 0008E09C  48 2A DA ED */	bl ReadFloat__12CInputStreamFv
/* 80091140 0008E0A0  D0 3F 02 30 */	stfs f1, 0x230(r31)
/* 80091144 0008E0A4  7F 83 E3 78 */	mr r3, r28
/* 80091148 0008E0A8  48 2A DA E1 */	bl ReadFloat__12CInputStreamFv
/* 8009114C 0008E0AC  D0 3F 02 34 */	stfs f1, 0x234(r31)
/* 80091150 0008E0B0  7F E3 FB 78 */	mr r3, r31
/* 80091154 0008E0B4  80 01 00 44 */	lwz r0, 0x44(r1)
/* 80091158 0008E0B8  83 E1 00 3C */	lwz r31, 0x3c(r1)
/* 8009115C 0008E0BC  83 C1 00 38 */	lwz r30, 0x38(r1)
/* 80091160 0008E0C0  83 A1 00 34 */	lwz r29, 0x34(r1)
/* 80091164 0008E0C4  83 81 00 30 */	lwz r28, 0x30(r1)
/* 80091168 0008E0C8  7C 08 03 A6 */	mtlr r0
/* 8009116C 0008E0CC  38 21 00 40 */	addi r1, r1, 0x40
/* 80091170 0008E0D0  4E 80 00 20 */	blr 

.global __dt__10CTweakBallFv
__dt__10CTweakBallFv:
/* 80091174 0008E0D4  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 80091178 0008E0D8  7C 08 02 A6 */	mflr r0
/* 8009117C 0008E0DC  90 01 00 14 */	stw r0, 0x14(r1)
/* 80091180 0008E0E0  93 E1 00 0C */	stw r31, 0xc(r1)
/* 80091184 0008E0E4  7C 7F 1B 79 */	or. r31, r3, r3
/* 80091188 0008E0E8  41 82 00 30 */	beq lbl_800911B8
/* 8009118C 0008E0EC  3C 60 80 3E */	lis r3, lbl_803DAD30@ha
/* 80091190 0008E0F0  38 03 AD 30 */	addi r0, r3, lbl_803DAD30@l
/* 80091194 0008E0F4  90 1F 00 00 */	stw r0, 0(r31)
/* 80091198 0008E0F8  41 82 00 10 */	beq lbl_800911A8
/* 8009119C 0008E0FC  3C 60 80 3E */	lis r3, lbl_803D9CC4@ha
/* 800911A0 0008E100  38 03 9C C4 */	addi r0, r3, lbl_803D9CC4@l
/* 800911A4 0008E104  90 1F 00 00 */	stw r0, 0(r31)
lbl_800911A8:
/* 800911A8 0008E108  7C 80 07 35 */	extsh. r0, r4
/* 800911AC 0008E10C  40 81 00 0C */	ble lbl_800911B8
/* 800911B0 0008E110  7F E3 FB 78 */	mr r3, r31
/* 800911B4 0008E114  48 00 00 1D */	bl "__dl__24TOneStatic<10CTweakBall>FPv"
lbl_800911B8:
/* 800911B8 0008E118  80 01 00 14 */	lwz r0, 0x14(r1)
/* 800911BC 0008E11C  7F E3 FB 78 */	mr r3, r31
/* 800911C0 0008E120  83 E1 00 0C */	lwz r31, 0xc(r1)
/* 800911C4 0008E124  7C 08 03 A6 */	mtlr r0
/* 800911C8 0008E128  38 21 00 10 */	addi r1, r1, 0x10
/* 800911CC 0008E12C  4E 80 00 20 */	blr 

.global "__dl__24TOneStatic<10CTweakBall>FPv"
"__dl__24TOneStatic<10CTweakBall>FPv":
/* 800911D0 0008E130  94 21 FF F0 */	stwu r1, -0x10(r1)
/* 800911D4 0008E134  7C 08 02 A6 */	mflr r0
/* 800911D8 0008E138  90 01 00 14 */	stw r0, 0x14(r1)
/* 800911DC 0008E13C  4B FA 5F A9 */	bl "ReferenceCount__24TOneStatic<10CTweakBall>Fv"
/* 800911E0 0008E140  80 83 00 00 */	lwz r4, 0(r3)
/* 800911E4 0008E144  38 04 FF FF */	addi r0, r4, -1
/* 800911E8 0008E148  90 03 00 00 */	stw r0, 0(r3)
/* 800911EC 0008E14C  80 01 00 14 */	lwz r0, 0x14(r1)
/* 800911F0 0008E150  7C 08 03 A6 */	mtlr r0
/* 800911F4 0008E154  38 21 00 10 */	addi r1, r1, 0x10
/* 800911F8 0008E158  4E 80 00 20 */	blr 
