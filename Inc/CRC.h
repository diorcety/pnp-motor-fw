/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  CRC.h
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
  \file CRC.h
  \author Trinamic Motion Control GmbH & Co KG

  \brief CRC functions

  This file contains the definitions of the CRC functions.
*/

#ifndef __CRC_H
#define __CRC_H

#include <stdlib.h>
#include "Types.h"

UCHAR crc8(UCHAR crc, UCHAR const* data, size_t len);

#endif //__CRC_H