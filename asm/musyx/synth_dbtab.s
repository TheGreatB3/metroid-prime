.include "macros.inc"

.section .data, "wa"

.global dspAttenuationTab
dspAttenuationTab:
	# ROM: 0x3F0820
	.4byte 0x7FFF78D6
	.4byte 0x72136BB1
	.4byte 0x65AB5FFB
	.4byte 0x5A9D558B
	.4byte 0x50C24C3E
	.4byte 0x47FA43F3
	.4byte 0x40263C8F
	.4byte 0x392C35F9
	.4byte 0x32F4301B
	.4byte 0x2D6A2ADF
	.4byte 0x28792636
	.4byte 0x2412220E
	.4byte 0x20261E5A
	.4byte 0x1CA71B0D
	.4byte 0x1989181C
	.4byte 0x16C2157C
	.4byte 0x14491326
	.4byte 0x12141111
	.4byte 0x101D0F36
	.4byte 0x0E5C0D8E
	.4byte 0x0CCC0C15
	.4byte 0x0B680AC5
	.4byte 0x0A2A0999
	.4byte 0x090F088D
	.4byte 0x0813079F
	.4byte 0x073206CB
	.4byte 0x066A060E
	.4byte 0x05B70565
	.4byte 0x051804CF
	.4byte 0x048A0449
	.4byte 0x040C03D2
	.4byte 0x039B0367
	.4byte 0x03370309
	.4byte 0x02DD02B4
	.4byte 0x028D0269
	.4byte 0x02460226
	.4byte 0x020701EA
	.4byte 0x01CE01B4
	.4byte 0x019C0185
	.4byte 0x016F015B
	.4byte 0x01470135
	.4byte 0x01240113
	.4byte 0x010400F5
	.4byte 0x00E700DA
	.4byte 0x00CE00C3
	.4byte 0x00B800AD
	.4byte 0x00A4009B
	.4byte 0x0092008A
	.4byte 0x0082007B
	.4byte 0x0074006D
	.4byte 0x00670061
	.4byte 0x005C0057
	.4byte 0x0052004D
	.4byte 0x00490045
	.4byte 0x0041003D
	.4byte 0x003A0037
	.4byte 0x00330031
	.4byte 0x002E002B
	.4byte 0x00290026
	.4byte 0x00240022
	.4byte 0x0020001E
	.4byte 0x001D001B
	.4byte 0x001A0018
	.4byte 0x00170015
	.4byte 0x00140013
	.4byte 0x00120011
	.4byte 0x0010000F
	.4byte 0x000E000D
	.4byte 0x000D000C
	.4byte 0x000B000A
	.4byte 0x000A0009
	.4byte 0x00090008
	.4byte 0x00080007
	.4byte 0x00070006
	.4byte 0x00060006
	.4byte 0x00050005
	.4byte 0x00050004
	.4byte 0x00040004
	.4byte 0x00040003
	.4byte 0x00030003
	.4byte 0x00030003
	.4byte 0x00020002
	.4byte 0x00020002
	.4byte 0x00020002
	.4byte 0x00020001
	.4byte 0x00010001
	.4byte 0x00010001
	.4byte 0x00010001
	.4byte 0x00010001
	.4byte 0x00010001
	.4byte 0x00010000
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0

.global dspScale2IndexTab
dspScale2IndexTab:
	# ROM: 0x3F09A4
	.4byte 0xC1786C65
	.4byte 0x605C5957
	.4byte 0x5452504F
	.4byte 0x4D4C4B49
	.4byte 0x48474645
	.4byte 0x44444342
	.4byte 0x4140403F
	.4byte 0x3F3E3D3D
	.4byte 0x3C3C3B3B
	.4byte 0x3A3A3939
	.4byte 0x38383737
	.4byte 0x37363636
	.4byte 0x35353434
	.4byte 0x34333333
	.4byte 0x32323232
	.4byte 0x31313130
	.4byte 0x3030302F
	.4byte 0x2F2F2F2E
	.4byte 0x2E2E2E2D
	.4byte 0x2D2D2D2C
	.4byte 0x2C2C2C2C
	.4byte 0x2B2B2B2B
	.4byte 0x2B2A2A2A
	.4byte 0x2A2A2929
	.4byte 0x29292929
	.4byte 0x28282828
	.4byte 0x28282727
	.4byte 0x27272727
	.4byte 0x26262626
	.4byte 0x26262625
	.4byte 0x25252525
	.4byte 0x25252424
	.4byte 0x24242424
	.4byte 0x24232323
	.4byte 0x23232323
	.4byte 0x23222222
	.4byte 0x22222222
	.4byte 0x22212121
	.4byte 0x21212121
	.4byte 0x21212020
	.4byte 0x20202020
	.4byte 0x2020201F
	.4byte 0x1F1F1F1F
	.4byte 0x1F1F1F1F
	.4byte 0x1F1E1E1E
	.4byte 0x1E1E1E1E
	.4byte 0x1E1E1E1E
	.4byte 0x1D1D1D1D
	.4byte 0x1D1D1D1D
	.4byte 0x1D1D1D1C
	.4byte 0x1C1C1C1C
	.4byte 0x1C1C1C1C
	.4byte 0x1C1C1C1B
	.4byte 0x1B1B1B1B
	.4byte 0x1B1B1B1B
	.4byte 0x1B1B1B1A
	.4byte 0x1A1A1A1A
	.4byte 0x1A1A1A1A
	.4byte 0x1A1A1A1A
	.4byte 0x19191919
	.4byte 0x19191919
	.4byte 0x19191919
	.4byte 0x19191818
	.4byte 0x18181818
	.4byte 0x18181818
	.4byte 0x18181818
	.4byte 0x18171717
	.4byte 0x17171717
	.4byte 0x17171717
	.4byte 0x17171717
	.4byte 0x17161616
	.4byte 0x16161616
	.4byte 0x16161616
	.4byte 0x16161616
	.4byte 0x16151515
	.4byte 0x15151515
	.4byte 0x15151515
	.4byte 0x15151515
	.4byte 0x15151514
	.4byte 0x14141414
	.4byte 0x14141414
	.4byte 0x14141414
	.4byte 0x14141414
	.4byte 0x14141313
	.4byte 0x13131313
	.4byte 0x13131313
	.4byte 0x13131313
	.4byte 0x13131313
	.4byte 0x13121212
	.4byte 0x12121212
	.4byte 0x12121212
	.4byte 0x12121212
	.4byte 0x12121212
	.4byte 0x12121111
	.4byte 0x11111111
	.4byte 0x11111111
	.4byte 0x11111111
	.4byte 0x11111111
	.4byte 0x11111111
	.4byte 0x10101010
	.4byte 0x10101010
	.4byte 0x10101010
	.4byte 0x10101010
	.4byte 0x10101010
	.4byte 0x10101010
	.4byte 0x0F0F0F0F
	.4byte 0x0F0F0F0F
	.4byte 0x0F0F0F0F
	.4byte 0x0F0F0F0F
	.4byte 0x0F0F0F0F
	.4byte 0x0F0F0F0F
	.4byte 0x0F0E0E0E
	.4byte 0x0E0E0E0E
	.4byte 0x0E0E0E0E
	.4byte 0x0E0E0E0E
	.4byte 0x0E0E0E0E
	.4byte 0x0E0E0E0E
	.4byte 0x0E0E0E0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0D
	.4byte 0x0D0D0D0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0C0C0C0C
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0B0B0B0B
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A0A0A0A
	.4byte 0x0A090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x09090909
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08080808
	.4byte 0x08070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x07070707
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060606
	.4byte 0x06060505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050505
	.4byte 0x05050404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04040404
	.4byte 0x04030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030303
	.4byte 0x03030302
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020202
	.4byte 0x02020201
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010101
	.4byte 0x01010100
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0
	.4byte 0

.global dspDLSVolTab
dspDLSVolTab:
	# ROM: 0x3F0DA4
	.4byte 0
	.4byte 0x388205FF
	.4byte 0x398205FF
	.4byte 0x3A1246BF
	.4byte 0x3A8205FF
	.4byte 0x3ACB295F
	.4byte 0x3B1246BF
	.4byte 0x3B47192F
	.4byte 0x3B8205FF
	.4byte 0x3BA48F97
	.4byte 0x3BCB295F
	.4byte 0x3BF5D356
	.4byte 0x3C1246BF
	.4byte 0x3C2BABEB
	.4byte 0x3C47192F
	.4byte 0x3C648E8A
	.4byte 0x3C8205FF
	.4byte 0x3C92C8C5
	.4byte 0x3CA48F97
	.4byte 0x3CB75A75
	.4byte 0x3CCB295F
	.4byte 0x3CDFFC54
	.4byte 0x3CF5D356
	.4byte 0x3D065732
	.4byte 0x3D1246BF
	.4byte 0x3D1EB852
	.4byte 0x3D2BABEB
	.4byte 0x3D39218A
	.4byte 0x3D47192F
	.4byte 0x3D5592DA
	.4byte 0x3D648E8A
	.4byte 0x3D740C41
	.4byte 0x3D8205FF
	.4byte 0x3D8A46E1
	.4byte 0x3D92C8C5
	.4byte 0x3D9B8BAC
	.4byte 0x3DA48F97
	.4byte 0x3DADD484
	.4byte 0x3DB75A75
	.4byte 0x3DC12168
	.4byte 0x3DCB295F
	.4byte 0x3DD57258
	.4byte 0x3DDFFC54
	.4byte 0x3DEAC754
	.4byte 0x3DF5D356
	.4byte 0x3E00902E
	.4byte 0x3E065732
	.4byte 0x3E0C3EB8
	.4byte 0x3E1246BF
	.4byte 0x3E186F48
	.4byte 0x3E1EB852
	.4byte 0x3E2521DE
	.4byte 0x3E2BABEB
	.4byte 0x3E32567A
	.4byte 0x3E39218A
	.4byte 0x3E400D1B
	.4byte 0x3E47192F
	.4byte 0x3E4E45C3
	.4byte 0x3E5592DA
	.4byte 0x3E5D0071
	.4byte 0x3E648E8A
	.4byte 0x3E6C3D25
	.4byte 0x3E740C41
	.4byte 0x3E7BFBDF
	.4byte 0x3E820621
	.4byte 0x3E861E71
	.4byte 0x3E8A4702
	.4byte 0x3E8E7FD4
	.4byte 0x3E92C8E7
	.4byte 0x3E97223A
	.4byte 0x3E9B8BCE
	.4byte 0x3EA005A3
	.4byte 0x3EA48FB8
	.4byte 0x3EA92A0F
	.4byte 0x3EADD4A6
	.4byte 0x3EB28F7E
	.4byte 0x3EB75A96
	.4byte 0x3EBC35F0
	.4byte 0x3EC1218A
	.4byte 0x3EC61D65
	.4byte 0x3ECB2980
	.4byte 0x3ED045DD
	.4byte 0x3ED5727A
	.4byte 0x3EDAAF57
	.4byte 0x3EDFFC76
	.4byte 0x3EE559D5
	.4byte 0x3EEAC775
	.4byte 0x3EF04556
	.4byte 0x3EF5D378
	.4byte 0x3EFB71DA
	.4byte 0x3F00903F
	.4byte 0x3F036FB1
	.4byte 0x3F065743
	.4byte 0x3F0946F6
	.4byte 0x3F0C3EC9
	.4byte 0x3F0F3EBC
	.4byte 0x3F1246D0
	.4byte 0x3F155704
	.4byte 0x3F186F58
	.4byte 0x3F1B8FCD
	.4byte 0x3F1EB863
	.4byte 0x3F21E918
	.4byte 0x3F2521EE
	.4byte 0x3F2862E5
	.4byte 0x3F2BABFC
	.4byte 0x3F2EFD33
	.4byte 0x3F32568A
	.4byte 0x3F35B802
	.4byte 0x3F39219B
	.4byte 0x3F3C9353
	.4byte 0x3F400D3D
	.4byte 0x3F438F36
	.4byte 0x3F471950
	.4byte 0x3F4AAB8A
	.4byte 0x3F4E45E5
	.4byte 0x3F51E860
	.4byte 0x3F5592FB
	.4byte 0x3F5945B7
	.4byte 0x3F5D0093
	.4byte 0x3F60C38F
	.4byte 0x3F648EAC
	.4byte 0x3F6861E9
	.4byte 0x3F6C3D47
	.4byte 0x3F7020C5
	.4byte 0x3F740C63
	.4byte 0x3F780022
	.4byte 0x3F7BFC01
	.4byte 0x3F800000
	.4byte 0x3F800000
