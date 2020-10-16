/**
  \file ADC.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Mini TMCL interpreter

  This file all functions necessary to implement a ADC.
*/

#ifndef __ADC_H
#define __ADC_H

void InitADC();
void InterruptADC();
float GetGaugePressure();

#endif //__CRC_H