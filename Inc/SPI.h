/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SPI.h
           Definitions of SPI access functions

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
  \file SPI.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief SPI functions

  This file contains the definitions of the SPI access functions.
*/

#ifndef __RS485_H
#define __RS485_H

void InitSPI();
void WriteSPIData(USHORT DeviceNumber, UCHAR* Data, UCHAR Size);
void ReadSPIData(USHORT DeviceNumber, UCHAR* Data, UCHAR Size);

#endif //__RS485_H


