/*
TM16XXFonts.h - Font definition for TM16XX.

Copyright (C) 2011 Ricardo Batista (rjbatista <at> gmail <dot> com)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


The bits are displayed by mapping bellow
 -- 0 --
|       |
5       1
 -- 6 --
4       2
|       |
 -- 3 --  .7


Segment shift for WEP

 -- 7 --
|       |
6       5
 -- 0 --
4       1
|       |
 -- 3 --  .2


*/
#ifndef TM16XXFonts_h
#define TM16XXFonts_h


#define WEP_SEG_SHIFT

#ifdef WEP_SEG_SHIFT
// Segment shift for WEP, change this if you have different segment order
#define SEG_SHIFT(b) (((b&1)<<7)|(((b>>1)&1)<<5)|(((b>>2)&1)<<1)|(b&(1<<3))|(b&(1<<4))|(((b>>5)&1)<<6)|((b>>6)&1)|(((b>>7)&1)<<2))
#else
// No shift
#define SEG_SHIFT(b) (b)
#endif

// definition for standard hexadecimal numbers
const byte NUMBER_FONT[] = {
  SEG_SHIFT(0b00111111), // 0 0x3F
  SEG_SHIFT(0b00000110), // 1 0x06
  SEG_SHIFT(0b01011011), // 2 0x5B
  SEG_SHIFT(0b01001111), // 3 0x4F
  SEG_SHIFT(0b01100110), // 4 0x66
  SEG_SHIFT(0b01101101), // 5 0x4D
  SEG_SHIFT(0b01111101), // 6 0x7D
  SEG_SHIFT(0b00000111), // 7 0x07
  SEG_SHIFT(0b01111111), // 8 0x7F
  SEG_SHIFT(0b01101111), // 9 0x6F
  SEG_SHIFT(0b01110111), // A 0x77
  SEG_SHIFT(0b01111100), // B 0x7C
  SEG_SHIFT(0b00111001), // C 0x39
  SEG_SHIFT(0b01011110), // D 0x5E
  SEG_SHIFT(0b01111001), // E 0x79
  SEG_SHIFT(0b01110001)  // F 0x71
};

const byte MINUS = SEG_SHIFT(0b01000000);
const byte DOT = SEG_SHIFT(0b10000000);

// definition for error
const byte ERROR_DATA[] = {
  SEG_SHIFT(0b01111001), // E
  SEG_SHIFT(0b01010000), // r
  SEG_SHIFT(0b01010000), // r
  SEG_SHIFT(0b01011100), // o
  SEG_SHIFT(0b01010000), // r
  0,
  0,
  0
};

// definition for the displayable ASCII chars
const byte FONT_DEFAULT[] = {
  SEG_SHIFT(0b00000000), // (32)  <space>
  SEG_SHIFT(0b10000110), // (33)	!
  SEG_SHIFT(0b00100010), // (34)	"
  SEG_SHIFT(0b01111110), // (35)	#
  SEG_SHIFT(0b01101101), // (36)	$
  SEG_SHIFT(0b00000000), // (37)	%
  SEG_SHIFT(0b00000000), // (38)	&
  SEG_SHIFT(0b00000010), // (39)	'
  SEG_SHIFT(0b00110000), // (40)	(
  SEG_SHIFT(0b00000110), // (41)	)
  SEG_SHIFT(0b01100011), // (42)	*
  SEG_SHIFT(0b00000000), // (43)	+
  SEG_SHIFT(0b00000100), // (44)	,
  SEG_SHIFT(0b01000000), // (45)	-
  SEG_SHIFT(0b00001000), // (46)	.
  SEG_SHIFT(0b01010010), // (47)	/
  SEG_SHIFT(0b00111111), // (48)	0
  SEG_SHIFT(0b00000110), // (49)	1
  SEG_SHIFT(0b01011011), // (50)	2
  SEG_SHIFT(0b01001111), // (51)	3
  SEG_SHIFT(0b01100110), // (52)	4
  SEG_SHIFT(0b01101101), // (53)	5
  SEG_SHIFT(0b01111101), // (54)	6
  SEG_SHIFT(0b00100111), // (55)	7
  SEG_SHIFT(0b01111111), // (56)	8
  SEG_SHIFT(0b01101111), // (57)	9
  SEG_SHIFT(0b00001001), // (58)	:
  SEG_SHIFT(0b00000101), // (59)	;
  SEG_SHIFT(0b00000000), // (60)	<
  SEG_SHIFT(0b01001000), // (61)	=
  SEG_SHIFT(0b00000000), // (62)	>
  SEG_SHIFT(0b01010011), // (63)	?
  SEG_SHIFT(0b01011111), // (64)	@
  SEG_SHIFT(0b01110111), // (65)	A
  SEG_SHIFT(0b01111111), // (66)	B
  SEG_SHIFT(0b00111001), // (67)	C
  SEG_SHIFT(0b00111111), // (68)	D
  SEG_SHIFT(0b01111001), // (69)	E
  SEG_SHIFT(0b01110001), // (70)	F
  SEG_SHIFT(0b00111101), // (71)	G
  SEG_SHIFT(0b01110110), // (72)	H
  SEG_SHIFT(0b00000110), // (73)	I
  SEG_SHIFT(0b00011111), // (74)	J
  SEG_SHIFT(0b01101001), // (75)	K
  SEG_SHIFT(0b00111000), // (76)	L
  SEG_SHIFT(0b00010101), // (77)	M
  SEG_SHIFT(0b00110111), // (78)	N
  SEG_SHIFT(0b00111111), // (79)	O
  SEG_SHIFT(0b01110011), // (80)	P
  SEG_SHIFT(0b01100111), // (81)	Q
  SEG_SHIFT(0b00110001), // (82)	R
  SEG_SHIFT(0b01101101), // (83)	S
  SEG_SHIFT(0b01111000), // (84)	T
  SEG_SHIFT(0b00111110), // (85)	U
  SEG_SHIFT(0b00101010), // (86)	V
  SEG_SHIFT(0b00011101), // (87)	W
  SEG_SHIFT(0b01110110), // (88)	X
  SEG_SHIFT(0b01101110), // (89)	Y
  SEG_SHIFT(0b01011011), // (90)	Z
  SEG_SHIFT(0b00111001), // (91)	[
  SEG_SHIFT(0b01100100), // (92)	\ (this can't be the last char on a line, even in comment or it'll concat)
  SEG_SHIFT(0b00001111), // (93)	]
  SEG_SHIFT(0b00000000), // (94)	^
  SEG_SHIFT(0b00001000), // (95)	_
  SEG_SHIFT(0b00100000), // (96)	`
  SEG_SHIFT(0b01011111), // (97)	a
  SEG_SHIFT(0b01111100), // (98)	b
  SEG_SHIFT(0b01011000), // (99)	c
  SEG_SHIFT(0b01011110), // (100)	d
  SEG_SHIFT(0b01111011), // (101)	e
  SEG_SHIFT(0b00110001), // (102)	f
  SEG_SHIFT(0b01101111), // (103)	g
  SEG_SHIFT(0b01110100), // (104)	h
  SEG_SHIFT(0b00000100), // (105)	i
  SEG_SHIFT(0b00001110), // (106)	j
  SEG_SHIFT(0b01110101), // (107)	k
  SEG_SHIFT(0b00110000), // (108)	l
  SEG_SHIFT(0b01010101), // (109)	m
  SEG_SHIFT(0b01010100), // (110)	n
  SEG_SHIFT(0b01011100), // (111)	o
  SEG_SHIFT(0b01110011), // (112)	p
  SEG_SHIFT(0b01100111), // (113)	q
  SEG_SHIFT(0b01010000), // (114)	r
  SEG_SHIFT(0b01101101), // (115)	s
  SEG_SHIFT(0b01111000), // (116)	t
  SEG_SHIFT(0b00011100), // (117)	u
  SEG_SHIFT(0b00101010), // (118)	v
  SEG_SHIFT(0b00011101), // (119)	w
  SEG_SHIFT(0b01110110), // (120)	x
  SEG_SHIFT(0b01101110), // (121)	y
  SEG_SHIFT(0b01000111), // (122)	z
  SEG_SHIFT(0b01000110), // (123)	{
  SEG_SHIFT(0b00000110), // (124)	|
  SEG_SHIFT(0b01110000), // (125)	}
  SEG_SHIFT(0b00000001), // (126)	~
};

#endif

