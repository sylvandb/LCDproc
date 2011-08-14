/** \file server/drivers/glcd_font5x8.h
 * Font definition for use with graphical displays.
 */

/*-
 * Copyright (c) 2011 Markus Dolze <bsdfan@nurfuerspam.de>
 *
 * This file is released under the GNU General Public License. Refer to the
 * COPYING file distributed with this package.
 */

#ifndef GLCD_FONT5X8_H
#define GLCD_FONT5X8_H

/* Bit patterns */
#define b______ 0x00
#define b_____O 0x01
#define b____O_ 0x02
#define b____OO 0x03
#define b___O__ 0x04
#define b___O_O 0x05
#define b___OO_ 0x06
#define b___OOO 0x07
#define b__O___ 0x08
#define b__O__O 0x09
#define b__O_O_ 0x0a
#define b__O_OO 0x0b
#define b__OO__ 0x0c
#define b__OO_O 0x0d
#define b__OOO_ 0x0e
#define b__OOOO 0x0f
#define b_O____ 0x10
#define b_O___O 0x11
#define b_O__O_ 0x12
#define b_O__OO 0x13
#define b_O_O__ 0x14
#define b_O_O_O 0x15
#define b_O_OO_ 0x16
#define b_O_OOO 0x17
#define b_OO___ 0x18
#define b_OO__O 0x19
#define b_OO_O_ 0x1a
#define b_OO_OO 0x1b
#define b_OOO__ 0x1c
#define b_OOO_O 0x1d
#define b_OOOO_ 0x1e
#define b_OOOOO 0x1f
#define b_OOOOO 0x1f

/* Font properties */
#define GLCD_FONT_WIDTH  5
#define GLCD_FONT_HEIGHT 8

/**
 * Font definition for use with graphical displays. Character patterns are
 * based on HD44780 ones. It contains ISO 8859-1 characters with these
 * exceptions / extensions:
 * \li Euro symbol at A6h
 * \li ISO 8859-1 characters not implemented: A8h, ACh, ADh, AFh, B8h
 * \li LCDproc icons at 80h-8Fh and ACh
 * \li bar graph symbols at 90h-9Eh
 */
unsigned char glcd_iso8859_1[256][8] = {
    [0x21] {b______,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b______,
	    b______,
	    b___O__},
    [0x22] {b______,
	    b__O_O_,
	    b__O_O_,
	    b__O_O_,
	    b______,
	    b______,
	    b______,
	    b______},
    [0x23] {b______,
	    b__O_O_,
	    b__O_O_,
	    b_OOOOO,
	    b__O_O_,
	    b_OOOOO,
	    b__O_O_,
	    b__O_O_},
    [0x24] {b______,
	    b___O__,
	    b__OOOO,
	    b_O_O__,
	    b__OOO_,
	    b___O_O,
	    b_OOOO_,
	    b___O__},
    [0x25] {b______,
	    b_OO___,
	    b_OO__O,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_O__OO,
	    b____OO},
    [0x26] {b______,
	    b__OO__,
	    b_O__O_,
	    b_O_O__,
	    b__O___,
	    b_O_O_O,
	    b_O__O_,
	    b__OO_O},
    [0x27] {b______,
	    b__OO__,
	    b___O__,
	    b__O___,
	    b______,
	    b______,
	    b______,
	    b______},
    [0x28] {b______,
	    b____O_,
	    b___O__,
	    b__O___,
	    b__O___,
	    b__O___,
	    b___O__,
	    b____O_},
    [0x29] {b______,
	    b__O___,
	    b___O__,
	    b____O_,
	    b____O_,
	    b____O_,
	    b___O__,
	    b__O___},
    [0x2a] {b______,
	    b______,
	    b___O__,
	    b_O_O_O,
	    b__OOO_,
	    b_O_O_O,
	    b___O__,
	    b______},
    [0x2b] {b______,
	    b______,
	    b___O__,
	    b___O__,
	    b_OOOOO,
	    b___O__,
	    b___O__,
	    b______},
    [0x2c] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b__OO__,
	    b___O__,
	    b__O___},
    [0x2d] {b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b______,
	    b______,
	    b______},
    [0x2e] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b__OO__,
	    b__OO__},
    [0x2f] {b______,
	    b______,
	    b_____O,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_O____,
	    b______},
    [0x30] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O__OO,
	    b_O_O_O,
	    b_OO__O,
	    b_O___O,
	    b__OOO_},
    [0x31] {b______,
	    b___O__,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0x32] {b______,
	    b__OOO_,
	    b_O___O,
	    b_____O,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_OOOOO},
    [0x33] {b______,
	    b_OOOOO,
	    b____O_,
	    b___O__,
	    b____O_,
	    b_____O,
	    b_O___O,
	    b__OOO_},
    [0x34] {b______,
	    b____O_,
	    b___OO_,
	    b__O_O_,
	    b_O__O_,
	    b_OOOOO,
	    b____O_,
	    b____O_},
    [0x35] {b______,
	    b_OOOOO,
	    b_O____,
	    b_O____,
	    b_OOOO_,
	    b_____O,
	    b_O___O,
	    b__OOO_},
    [0x36] {b______,
	    b___OO_,
	    b__O___,
	    b_O____,
	    b_OOOO_,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0x37] {b______,
	    b_OOOOO,
	    b_O___O,
	    b____O_,
	    b___O__,
	    b__O___,
	    b__O___,
	    b__O___},
    [0x38] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0x39] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b__OOOO,
	    b_____O,
	    b____O_,
	    b__OO__},
    [0x3a] {b______,
	    b______,
	    b__OO__,
	    b__OO__,
	    b______,
	    b__OO__,
	    b__OO__,
	    b______},
    [0x3b] {b______,
	    b______,
	    b__OO__,
	    b__OO__,
	    b______,
	    b__OO__,
	    b___O__,
	    b__O___},
    [0x3c] {b______,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_O____,
	    b__O___,
	    b___O__,
	    b____O_},
    [0x3d] {b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b______,
	    b_OOOOO,
	    b______,
	    b______},
    [0x3e] {b______,
	    b_O____,
	    b__O___,
	    b___O__,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_O____},
    [0x3f] {b______,
	    b__OOO_,
	    b_O___O,
	    b_____O,
	    b____O_,
	    b___O__,
	    b______,
	    b___O__},
    [0x40] {b______,
	    b__OOO_,
	    b_O___O,
	    b_____O,
	    b__OO_O,
	    b_O_O_O,
	    b_O_O_O,
	    b__OOO_},
    [0x41] {b______,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0x42] {b______,
	    b_OOOO_,
	    b_O___O,
	    b_O___O,
	    b_OOOO_,
	    b_O___O,
	    b_O___O,
	    b_OOOO_},
    [0x43] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O___O,
	    b__OOO_},
    [0x44] {b______,
	    b_OOO__,
	    b_O__O_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__O_,
	    b_OOO__},
    [0x45] {b______,
	    b_OOOOO,
	    b_O____,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_O____,
	    b_OOOOO},
    [0x46] {b______,
	    b_OOOOO,
	    b_O____,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_O____,
	    b_O____},
    [0x47] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O____,
	    b_O_OOO,
	    b_O___O,
	    b_O___O,
	    b__OOOO},
    [0x48] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O,
	    b_O___O},
    [0x49] {b______,
	    b__OOO_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0x4a] {b______,
	    b___OOO,
	    b____O_,
	    b____O_,
	    b____O_,
	    b____O_,
	    b_O__O_,
	    b__OO__},
    [0x4b] {b______,
	    b_O___O,
	    b_O__O_,
	    b_O_O__,
	    b_OO___,
	    b_O_O__,
	    b_O__O_,
	    b_O___O},
    [0x4c] {b______,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_OOOOO},
    [0x4d] {b______,
	    b_O___O,
	    b_OO_OO,
	    b_O_O_O,
	    b_O_O_O,
	    b_O___O,
	    b_O___O,
	    b_O___O},
    [0x4e] {b______,
	    b_O___O,
	    b_O___O,
	    b_OO__O,
	    b_O_O_O,
	    b_O__OO,
	    b_O___O,
	    b_O___O},
    [0x4f] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0x50] {b______,
	    b_OOOO_,
	    b_O___O,
	    b_O___O,
	    b_OOOO_,
	    b_O____,
	    b_O____,
	    b_O____},
    [0x51] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O_O_O,
	    b_O__O_,
	    b__OO_O},
    [0x52] {b______,
	    b_OOOO_,
	    b_O___O,
	    b_O___O,
	    b_OOOO_,
	    b_O_O__,
	    b_O__O_,
	    b_O___O},
    [0x53] {b______,
	    b__OOOO,
	    b_O____,
	    b_O____,
	    b__OOO_,
	    b_____O,
	    b_____O,
	    b_OOOO_},
    [0x54] {b______,
	    b_OOOOO,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__},
    [0x55] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0x56] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__O_O_,
	    b___O__},
    [0x57] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O_O_O,
	    b_O_O_O,
	    b_O_O_O,
	    b__O_O_},
    [0x58] {b______,
	    b_O___O,
	    b_O___O,
	    b__O_O_,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b_O___O},
    [0x59] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__O_O_,
	    b___O__,
	    b___O__,
	    b___O__},
    [0x5a] {b______,
	    b_OOOOO,
	    b_____O,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_O____,
	    b_OOOOO},
    [0x5b] {b______,
	    b__OOO_,
	    b__O___,
	    b__O___,
	    b__O___,
	    b__O___,
	    b__O___,
	    b__OOO_},
    [0x5c] {b______,
	    b______,
	    b_O____,
	    b__O___,
	    b___O__,
	    b____O_,
	    b_____O,
	    b______},
    [0x5d] {b______,
	    b__OOO_,
	    b____O_,
	    b____O_,
	    b____O_,
	    b____O_,
	    b____O_,
	    b__OOO_},
    [0x5e] {b______,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b______,
	    b______,
	    b______,
	    b______},
    [0x5f] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO},
    [0x60] {b______,
	    b__O___,
	    b___O__,
	    b____O_,
	    b______,
	    b______,
	    b______,
	    b______},
    [0x61] {b______,
	    b______,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0x62] {b______,
	    b_O____,
	    b_O____,
	    b_O_OO_,
	    b_OO__O,
	    b_O___O,
	    b_O___O,
	    b_OOOO_},
    [0x63] {b______,
	    b______,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O____,
	    b_O___O,
	    b__OOO_},
    [0x64] {b______,
	    b_____O,
	    b_____O,
	    b__OO_O,
	    b_O__OO,
	    b_O___O,
	    b_O___O,
	    b__OOOO},
    [0x65] {b______,
	    b______,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O____,
	    b__OOO_},
    [0x66] {b______,
	    b___OO_,
	    b__O__O,
	    b__O___,
	    b_OOO__,
	    b__O___,
	    b__O___,
	    b__O___},
    [0x67] {b______,
	    b______,
	    b______,
	    b__OOOO,
	    b_O___O,
	    b__OOOO,
	    b_____O,
	    b__OOO_},
    [0x68] {b______,
	    b_O____,
	    b_O____,
	    b_O_OO_,
	    b_OO__O,
	    b_O___O,
	    b_O___O,
	    b_O___O},
    [0x69] {b______,
	    b___O__,
	    b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0x6a] {b______,
	    b____O_,
	    b______,
	    b___OO_,
	    b____O_,
	    b____O_,
	    b_O__O_,
	    b__OO__},
    [0x6b] {b______,
	    b__O___,
	    b__O___,
	    b__O__O,
	    b__O_O_,
	    b__OO__,
	    b__O_O_,
	    b__O__O},
    [0x6c] {b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0x6d] {b______,
	    b______,
	    b______,
	    b_OO_O_,
	    b_O_O_O,
	    b_O_O_O,
	    b_O___O,
	    b_O___O},
    [0x6e] {b______,
	    b______,
	    b______,
	    b_O_OO_,
	    b_OO__O,
	    b_O___O,
	    b_O___O,
	    b_O___O},
    [0x6f] {b______,
	    b______,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0x70] {b______,
	    b______,
	    b______,
	    b_OOOO_,
	    b_O___O,
	    b_OOOO_,
	    b_O____,
	    b_O____},
    [0x71] {b______,
	    b______,
	    b______,
	    b__OO_O,
	    b_O__OO,
	    b__OOOO,
	    b_____O,
	    b_____O},
    [0x72] {b______,
	    b______,
	    b______,
	    b_O_OO_,
	    b_OO__O,
	    b_O____,
	    b_O____,
	    b_O____},
    [0x73] {b______,
	    b______,
	    b______,
	    b__OOO_,
	    b_O____,
	    b__OOO_,
	    b_____O,
	    b_OOOO_},
    [0x74] {b______,
	    b__O___,
	    b_OOO__,
	    b__O___,
	    b__O___,
	    b__O___,
	    b__O__O,
	    b___OO_},
    [0x75] {b______,
	    b______,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b__OO_O},
    [0x76] {b______,
	    b______,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__O_O_,
	    b___O__},
    [0x77] {b______,
	    b______,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O_O_O,
	    b__O_O_},
    [0x78] {b______,
	    b______,
	    b______,
	    b_O___O,
	    b__O_O_,
	    b___O__,
	    b__O_O_,
	    b_O___O},
    [0x79] {b______,
	    b______,
	    b______,
	    b_O___O,
	    b_O___O,
	    b__OOOO,
	    b_____O,
	    b__OOO_},
    [0x7a] {b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b____O_,
	    b___O__,
	    b__O___,
	    b_OOOOO},
    [0x7b] {b______,
	    b____O_,
	    b___O__,
	    b___O__,
	    b__O___,
	    b___O__,
	    b___O__,
	    b____O_},
    [0x7c] {b______,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__},
    [0x7d] {b______,
	    b__O___,
	    b___O__,
	    b___O__,
	    b____O_,
	    b___O__,
	    b___O__,
	    b__O___},
    [0x7e] {b______,
	    b__OO_O,
	    b_O__O_,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______},
    [0x80] {b______,
	    b__O_O_,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b__OOO_,
	    b___O__,
	    b______},
    [0x81] {b______,
	    b______,
	    b__O_O_,
	    b__OOO_,
	    b__OOO_,
	    b___O__,
	    b______,
	    b______},
    [0x82] {b______,
	    b___O__,
	    b__OOO_,
	    b_O_O_O,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__},
    [0x83] {b______,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b_O_O_O,
	    b__OOO_,
	    b___O__},
    [0x84] {b______,
	    b______,
	    b___O__,
	    b__O___,
	    b_OOOOO,
	    b__O___,
	    b___O__,
	    b______},
    [0x85] {b______,
	    b______,
	    b___O__,
	    b____O_,
	    b_OOOOO,
	    b____O_,
	    b___O__,
	    b______},
    [0x86] {b______,
	    b______,
	    b_OOOOO,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_OOOOO,
	    b______},
    [0x87] {b___O__,
	    b___O__,
	    b_OOO_O,
	    b_O_OO_,
	    b_O_O_O,
	    b_O___O,
	    b_OOOOO,
	    b______},
    [0x88] {b______,
	    b______,
	    b_OOOOO,
	    b_O_O_O,
	    b_OO_OO,
	    b_O_O_O,
	    b_OOOOO,
	    b______},
    [0x89] {b______,
	    b__O___,
	    b__OO__,
	    b__OOO_,
	    b__OOOO,
	    b__OOO_,
	    b__OO__,
	    b__O___},
    [0x8a] {b______,
	    b____O_,
	    b___OO_,
	    b__OOO_,
	    b_OOOO_,
	    b__OOO_,
	    b___OO_,
	    b____O_},
    [0x8b] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b_O_O_O},
    [0x8c] {b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b______},
    [0x8d] {b______,
	    b_OO_OO,
	    b_OO_OO,
	    b_OO_OO,
	    b_OO_OO,
	    b_OO_OO,
	    b_OO_OO,
	    b_OO_OO},
    [0x8e] {b______,
	    b__O___,
	    b___O__,
	    b____O_,
	    b___O__,
	    b__O___,
	    b______,
	    b_OOOOO},
    [0x8f] {b______,
	    b____O_,
	    b___O__,
	    b__O___,
	    b___O__,
	    b____O_,
	    b______,
	    b_OOOOO},
    [0x91] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO},
    [0x92] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO},
    [0x93] {b______,
	    b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x94] {b______,
	    b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x95] {b______,
	    b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x96] {b______,
	    b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x97] {b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x98] {b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0x9a] {b______,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____,
	    b_O____},
    [0x9b] {b______,
	    b_OO___,
	    b_OO___,
	    b_OO___,
	    b_OO___,
	    b_OO___,
	    b_OO___,
	    b_OO___},
    [0x9c] {b______,
	    b_OOO__,
	    b_OOO__,
	    b_OOO__,
	    b_OOO__,
	    b_OOO__,
	    b_OOO__,
	    b_OOO__},
    [0x9d] {b______,
	    b_OOOO_,
	    b_OOOO_,
	    b_OOOO_,
	    b_OOOO_,
	    b_OOOO_,
	    b_OOOO_,
	    b_OOOO_},
    [0x9e] {b______,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO},
    [0xa1] {b______,
	    b___O__,
	    b______,
	    b______,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__},
    [0xa2] {b______,
	    b___O__,
	    b__OOO_,
	    b_O_O__,
	    b_O_O__,
	    b_O_O_O,
	    b__OOO_,
	    b___O__},
    [0xa3] {b______,
	    b___OO_,
	    b__O___,
	    b__O___,
	    b_OOO__,
	    b__O___,
	    b__O__O,
	    b_O_OO_},
    [0xa4] {b______,
	    b______,
	    b_O___O,
	    b__OOO_,
	    b__O_O_,
	    b__OOO_,
	    b_O___O,
	    b______},
    [0xa5] {b______,
	    b_O___O,
	    b__O_O_,
	    b_OOOOO,
	    b___O__,
	    b_OOOOO,
	    b___O__,
	    b___O__},
    [0xa6] {b______,
	    b___OOO,
	    b__O___,
	    b_OOOO_,
	    b__O___,
	    b_OOOO_,
	    b__O___,
	    b___OOO},
    [0xa7] {b______,
	    b___OO_,
	    b__O__O,
	    b___O__,
	    b__O_O_,
	    b___O__,
	    b_O__O_,
	    b__OO__},
    [0xa9] {b______,
	    b_OOOOO,
	    b_O___O,
	    b_O_O_O,
	    b_O_OOO,
	    b_O_O_O,
	    b_O___O,
	    b_OOOOO},
    [0xaa] {b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO,
	    b______,
	    b_OOOOO},
    [0xab] {b______,
	    b______,
	    b___O_O,
	    b__O_O_,
	    b_O_O__,
	    b__O_O_,
	    b___O_O,
	    b______},
    [0xac] {b______,
	    b______,
	    b__OOO_,
	    b_OOOOO,
	    b_OOOOO,
	    b_OOOOO,
	    b__OOO_,
	    b______},
    [0xae] {b______,
	    b_OOOOO,
	    b_O___O,
	    b_O_O_O,
	    b_O___O,
	    b_O__OO,
	    b_O_O_O,
	    b_OOOOO},
    [0xb0] {b__OO__,
	    b_O__O_,
	    b_O__O_,
	    b__OO__,
	    b______,
	    b______,
	    b______,
	    b______},
    [0xb1] {b______,
	    b___O__,
	    b___O__,
	    b_OOOOO,
	    b___O__,
	    b___O__,
	    b______,
	    b_OOOOO},
    [0xb2] {b__OO__,
	    b_O__O_,
	    b___O__,
	    b__O___,
	    b_OOOO_,
	    b______,
	    b______,
	    b______},
    [0xb3] {b_OOO__,
	    b____O_,
	    b__OO__,
	    b____O_,
	    b_OOO__,
	    b______,
	    b______,
	    b______},
    [0xb4] {b____O_,
	    b___O__,
	    b__O___,
	    b______,
	    b______,
	    b______,
	    b______,
	    b______},
    [0xb5] {b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b_OOO_O,
	    b_O____,
	    b_O____},
    [0xb6] {b______,
	    b__OOOO,
	    b_O_O_O,
	    b_O_O_O,
	    b__OO_O,
	    b___O_O,
	    b___O_O,
	    b___O_O},
    [0xb7] {b______,
	    b______,
	    b______,
	    b__OO__,
	    b__OO__,
	    b______,
	    b______,
	    b______},
    [0xb9] {b__O___,
	    b_OO___,
	    b__O___,
	    b__O___,
	    b_OOO__,
	    b______,
	    b______,
	    b______},
    [0xba] {b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_,
	    b______,
	    b_OOOOO},
    [0xbb] {b______,
	    b______,
	    b_O_O__,
	    b__O_O_,
	    b___O_O,
	    b__O_O_,
	    b_O_O__,
	    b______},
    [0xbc] {b_O___O,
	    b_O__O_,
	    b_O_O__,
	    b__O_O_,
	    b_O_OO_,
	    b__O_O_,
	    b__OOOO,
	    b____O_},
    [0xbd] {b_O___O,
	    b_O__O_,
	    b_O_O__,
	    b__O_O_,
	    b_O_O_O,
	    b_____O,
	    b____O_,
	    b___OOO},
    [0xbe] {b_OO___,
	    b__O___,
	    b_OO___,
	    b__O__O,
	    b_OO_OO,
	    b___O_O,
	    b___OOO,
	    b_____O},
    [0xbf] {b______,
	    b___O__,
	    b______,
	    b___O__,
	    b__O___,
	    b_O____,
	    b_O___O,
	    b__OOO_},
    [0xc0] {b__O___,
	    b___O__,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc1] {b____O_,
	    b___O__,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc2] {b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc3] {b__OO_O,
	    b_O__O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc4] {b__O_O_,
	    b______,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc5] {b___O__,
	    b__O_O_,
	    b___O__,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O___O,
	    b_O___O},
    [0xc6] {b______,
	    b___OOO,
	    b__OO__,
	    b_O_O__,
	    b_O_OOO,
	    b_OOO__,
	    b_O_O__,
	    b_O_OOO},
    [0xc7] {b__OOO_,
	    b_O___O,
	    b_O____,
	    b_O____,
	    b_O___O,
	    b__OOO_,
	    b____O_,
	    b___OO_},
    [0xc8] {b__O___,
	    b___O__,
	    b______,
	    b_OOOOO,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_OOOOO},
    [0xc9] {b___O__,
	    b__O___,
	    b______,
	    b_OOOOO,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_OOOOO},
    [0xca] {b___O__,
	    b__O_O_,
	    b______,
	    b_OOOOO,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_OOOOO},
    [0xcb] {b______,
	    b__O_O_,
	    b______,
	    b_OOOOO,
	    b_O____,
	    b_OOOO_,
	    b_O____,
	    b_OOOOO},
    [0xcc] {b__O___,
	    b___O__,
	    b______,
	    b__OOO_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xcd] {b____O_,
	    b___O__,
	    b______,
	    b__OOO_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xce] {b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xcf] {b______,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xd0] {b______,
	    b__OOO_,
	    b__O__O,
	    b__O__O,
	    b_OOO_O,
	    b__O__O,
	    b__O__O,
	    b__OOO_},
    [0xd1] {b__OO_O,
	    b_O__O_,
	    b______,
	    b_O___O,
	    b_OO__O,
	    b_O_O_O,
	    b_O__OO,
	    b_O___O},
    [0xd2] {b__O___,
	    b___O__,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xd3] {b____O_,
	    b___O__,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xd4] {b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xd5] {b__OO_O,
	    b_O__O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xd6] {b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xd7] {b______,
	    b______,
	    b_O___O,
	    b__O_O_,
	    b___O__,
	    b__O_O_,
	    b_O___O,
	    b______},
    [0xd8] {b______,
	    b__OOO_,
	    b_O__OO,
	    b_O_O_O,
	    b_O_O_O,
	    b_O_O_O,
	    b_OO__O,
	    b__OOO_},
    [0xd9] {b__O___,
	    b___O__,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xda] {b____O_,
	    b___O__,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xdb] {b___O__,
	    b__O_O_,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xdc] {b__O_O_,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xdd] {b____O_,
	    b___O__,
	    b_O___O,
	    b__O_O_,
	    b___O__,
	    b___O__,
	    b___O__,
	    b___O__},
    [0xde] {b_OO___,
	    b__O___,
	    b__OOO_,
	    b__O__O,
	    b__O__O,
	    b__OOO_,
	    b__O___,
	    b_OOO__},
    [0xdf] {b______,
	    b___OO_,
	    b__O__O,
	    b__O__O,
	    b__OOO_,
	    b__O__O,
	    b__O__O,
	    b_O_OO_},
    [0xe0] {b__O___,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe1] {b____O_,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe2] {b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe3] {b__OO_O,
	    b_O__O_,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe4] {b______,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe5] {b___O__,
	    b__O_O_,
	    b___O__,
	    b__OOO_,
	    b_____O,
	    b__OOOO,
	    b_O___O,
	    b__OOOO},
    [0xe6] {b______,
	    b______,
	    b_OO_O_,
	    b___O_O,
	    b__OOOO,
	    b_O_O__,
	    b_O_O_O,
	    b__O_O_},
    [0xe7] {b______,
	    b______,
	    b__OOO_,
	    b_O____,
	    b_O___O,
	    b__OOO_,
	    b___O__,
	    b__OO__},
    [0xe8] {b__O___,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O____,
	    b__OOO_},
    [0xe9] {b____O_,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O____,
	    b__OOO_},
    [0xea] {b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O____,
	    b__OOO_},
    [0xeb] {b______,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_OOOOO,
	    b_O____,
	    b__OOO_},
    [0xec] {b__O___,
	    b___O__,
	    b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xed] {b____O_,
	    b___O__,
	    b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xee] {b___O__,
	    b__O_O_,
	    b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xef] {b______,
	    b__O_O_,
	    b______,
	    b__OO__,
	    b___O__,
	    b___O__,
	    b___O__,
	    b__OOO_},
    [0xf0] {b______,
	    b_O_O__,
	    b__O___,
	    b_O_O__,
	    b____O_,
	    b__OOOO,
	    b_O___O,
	    b__OOO_},
    [0xf1] {b__OO_O,
	    b_O__O_,
	    b______,
	    b_O_OO_,
	    b_OO__O,
	    b_O___O,
	    b_O___O,
	    b_O___O},
    [0xf2] {b__O___,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xf3] {b____O_,
	    b___O__,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xf4] {b______,
	    b___O__,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xf5] {b______,
	    b__OO_O,
	    b_O__O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xf6] {b______,
	    b__O_O_,
	    b______,
	    b__OOO_,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b__OOO_},
    [0xf7] {b______,
	    b______,
	    b___O__,
	    b______,
	    b_OOOOO,
	    b______,
	    b___O__,
	    b______},
    [0xf8] {b______,
	    b_____O,
	    b__OOO_,
	    b_O__OO,
	    b_O_O_O,
	    b_OO__O,
	    b__OOO_,
	    b_O____},
    [0xf9] {b__O___,
	    b___O__,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b__OO_O},
    [0xfa] {b____O_,
	    b___O__,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b__OO_O},
    [0xfb] {b___O__,
	    b__O_O_,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b__OO_O},
    [0xfc] {b______,
	    b__O_O_,
	    b______,
	    b_O___O,
	    b_O___O,
	    b_O___O,
	    b_O__OO,
	    b__OO_O},
    [0xfd] {b______,
	    b____O_,
	    b___O__,
	    b_O___O,
	    b_O___O,
	    b__OOOO,
	    b_____O,
	    b__OOO_},
    [0xfe] {b_OO___,
	    b__O___,
	    b__O___,
	    b__OO__,
	    b__O_O_,
	    b__OO__,
	    b__O___,
	    b_OOO__},
    [0xff] {b______,
	    b__O_O_,
	    b______,
	    b_O___O,
	    b_O___O,
	    b__OOOO,
	    b_____O,
	    b__OOO_},
};

static int
glcd_icon5x8(Driver *drvthis, int x, int y, int icon)
{
	switch (icon) {
	    case ICON_BLOCK_FILLED:
		drvthis->chr(drvthis, x, y, 0x98);
		break;
	    case ICON_HEART_FILLED:
		drvthis->chr(drvthis, x, y, 0x80);
		break;
	    case ICON_HEART_OPEN:
		drvthis->chr(drvthis, x, y, 0x81);
		break;
	    case ICON_ARROW_UP:
		drvthis->chr(drvthis, x, y, 0x82);
		break;
	    case ICON_ARROW_DOWN:
		drvthis->chr(drvthis, x, y, 0x83);
		break;
	    case ICON_ARROW_LEFT:
		drvthis->chr(drvthis, x, y, 0x84);
		break;
	    case ICON_ARROW_RIGHT:
		drvthis->chr(drvthis, x, y, 0x85);
		break;
	    case ICON_CHECKBOX_OFF:
		drvthis->chr(drvthis, x, y, 0x86);
		break;
	    case ICON_CHECKBOX_ON:
		drvthis->chr(drvthis, x, y, 0x87);
		break;
	    case ICON_CHECKBOX_GRAY:
		drvthis->chr(drvthis, x, y, 0x88);
		break;
	    case ICON_SELECTOR_AT_LEFT:
		drvthis->chr(drvthis, x, y, 0x89);
		break;
	    case ICON_SELECTOR_AT_RIGHT:
		drvthis->chr(drvthis, x, y, 0x8a);
		break;
	    case ICON_ELLIPSIS:
		drvthis->chr(drvthis, x, y, 0x8b);
		break;
	    case ICON_STOP:
		drvthis->chr(drvthis, x, y, 0x8c);
		break;
	    case ICON_PAUSE:
		drvthis->chr(drvthis, x, y, 0x8d);
		break;
	    case ICON_PLAY:
		drvthis->chr(drvthis, x, y, 0x89);
		break;
	    case ICON_PLAYR:
		drvthis->chr(drvthis, x, y, 0x8a);
		break;
	    case ICON_FF:
		drvthis->chr(drvthis, x, y, 0xab);
		break;
	    case ICON_FR:
		drvthis->chr(drvthis, x, y, 0xbb);
		break;
	    case ICON_NEXT:
		drvthis->chr(drvthis, x, y, 0x8e);
		break;
	    case ICON_PREV:
		drvthis->chr(drvthis, x, y, 0x8f);
		break;
	    case ICON_REC:
		drvthis->chr(drvthis, x, y, 0xac);
		break;
	    default:
		return -1;
	}
	return 0;
}

#endif
