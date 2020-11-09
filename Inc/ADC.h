/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  ADC.h
           Definitions for ADC

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
  \file ADC.h
  \author Trinamic Motion Control GmbH & Co KG

  \brief ADC functions

  This file all functions necessary to implement a ADC.
*/

#ifndef __ADC_H
#define __ADC_H

void InitADC();
void InterruptADC();
float GetGaugePressure();

#endif //__CRC_H