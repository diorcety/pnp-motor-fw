/**
  \file CRC.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring

  This file contains the definitions of the CRC functions.
*/

#ifndef __CRC_H
#define __CRC_H

#include <stdlib.h>
#include "Types.h"

UCHAR crc8(UCHAR crc, UCHAR const* data, size_t len);

#endif //__CRC_H