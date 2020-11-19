/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  CRC.c
           Definitions for CRC

   Copyright (C) 2016 TRINAMIC Motion Control GmbH & Co KG
                      Waterloohain 5
                      D - 22769 Hamburg, Germany
                      http://www.trinamic.com/

   This program is free software; you can redistribute it and/or modify it
   freely.

   This program is distributed "as is" in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

/**
  \file CRC.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief Mini TMCL interpreter

  This file all functions necessary to implement a CRC.
*/

#include <stdlib.h>
#include "main.h"
#include "Types.h"

// 8-bit CRC using the polynomial x^8+x^6+x^3+x^2+1, 0x14D.
// Chosen based on Koopman, et al. (0xA6 in his notation = 0x14D >> 1):
// http://www.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//
// This implementation is reflected, processing the least-significant bit of the
// input first, has an initial CRC register value of 0xff, and exclusive-or's
// the final register value with 0xff. As a result the CRC of an empty string,
// and therefore the initial CRC value, is zero.
//
// The standard description of this CRC is:
// width=8 poly=0x4d init=0xff refin=true refout=true xorout=0xff check=0xd8
// name="CRC-8/KOOP"

UCHAR crc8(UCHAR const* data, size_t len)
{
  LL_CRC_ResetCRCCalculationUnit(CRC);
  UCHAR const* end = data + len;
  while (data < end)
  {
    LL_CRC_FeedData8(CRC, *data++);
  }
  return ~LL_CRC_ReadData8(CRC);
}