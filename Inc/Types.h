/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  bits.h
           Bit definitions

   Copyright (C) 2016 TRINAMIC Motion Control GmbH & Co KG
                      Sternstrae 67
                      D - 20357 Hamburg, Germany
                      http://www.trinamic.com/

   This program is free software; you can redistribute it and/or modify it
   freely.

   This program is distributed "as is" in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE.
*******************************************************************************/

/**
  \file bits.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Bit definitions.

  This file just contains some useful bit definitions.
*/
#ifndef __BITS_H
#define __BITS_H

typedef unsigned char UCHAR;                   //!< 8 bits unsigned
typedef unsigned short USHORT;                 //!< 16 bits unsigned
typedef unsigned int UINT;                     //!< 32 bits unsigned
typedef int INT;                               //!< 32 bits signed

#define TRUE 1
#define FALSE 0

#define   BIT0        0x00000001
#define   BIT1        0x00000002
#define   BIT2        0x00000004
#define   BIT3        0x00000008
#define   BIT4        0x00000010
#define   BIT5        0x00000020
#define   BIT6        0x00000040
#define   BIT7        0x00000080
#define   BIT8        0x00000100
#define   BIT9        0x00000200
#define   BIT10       0x00000400
#define   BIT11       0x00000800
#define   BIT12       0x00001000
#define   BIT13       0x00002000
#define   BIT14       0x00004000
#define   BIT15       0x00008000
#define   BIT16       0x00010000
#define   BIT17       0x00020000
#define   BIT18       0x00040000
#define   BIT19       0x00080000
#define   BIT20       0x00100000
#define   BIT21       0x00200000
#define   BIT22       0x00400000
#define   BIT23       0x00800000
#define   BIT24       0x01000000
#define   BIT25       0x02000000
#define   BIT26       0x04000000
#define   BIT27       0x08000000
#define   BIT28       0x10000000
#define   BIT29       0x20000000
#define   BIT30       0x40000000
#define   BIT31       0x80000000

#endif //__BITS_H