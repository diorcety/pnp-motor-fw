/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Encoder.c
           Processing of TMCL commands

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
  \file Encoder.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 1.00

  \brief Encoder functions

  This file contains functions for using the encoder interface of the TMC5160.
*/

#ifndef __ENCODER_H
#define __ENCODER_H

void InitEncoder(void);
void CalculateEncoderParameters(UCHAR Axis);
int GetEncoderPosition(UCHAR Axis);
void SetEncoderPosition(UCHAR Axis, int Value);

#endif //__ENCODER_H

