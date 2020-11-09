/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  Encoder.c
           Definitions for encoder

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
  \file Encoder.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief Encoder functions

  This file contains functions for using the encoder interface of the TMC5072.
*/

#include <stdlib.h>
#include <math.h>
#include "Types.h"
#include "PnpSpindle.h"
#include "TMC5072.h"
#include "Globals.h"


/**********************************************************//**
  \fn CalculateEncoderParameters(UCHAR Axis)
  \brief Calculate the encoder multiplier
  \param Axis   Axis number (always 0 with stealthRocker)

  This calculates the encoder multiplier of the TMC5072 in
  such a way that the encoder resolution matches the motor
  microstep resolution.
**************************************************************/
void CalculateEncoderParameters(UCHAR Axis)
{
  int MotorSteps;
  double EncFactor;
  double EncFactorFp;
  double EncFactorIp;
  int EncPrescaler;

  if (MotorConfig[Axis].EncoderResolution == 0 || MotorConfig[Axis].MotorFullStepResolution == 0)
  {
    //Select 1:1 if encoder resolution or motor resolution parameter should be zero.
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis)), ReadTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis))) & ~TMC5072_ENC_SEL_DECIMAL_MASK);
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENC_CONST(MOTOR_TO_IC_MOTOR(Axis)), 65536);
    return;
  }

  MotorSteps = (1 << (8 - ((ReadTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(Axis))) & TMC5072_MRES_MASK) >> TMC5072_MRES_SHIFT))) * MotorConfig[Axis].MotorFullStepResolution;
  EncFactor = (double)MotorSteps / (double)abs(MotorConfig[Axis].EncoderResolution);

  if (modf(EncFactor * 65536.0, &EncFactorIp) == 0.0)
  {
    EncPrescaler = (int)(EncFactor * 65536.0);
    if (MotorConfig[Axis].EncoderResolution < 0) EncPrescaler = -EncPrescaler;
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis)), ReadTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis))) & ~TMC5072_ENC_SEL_DECIMAL_MASK);
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENC_CONST(MOTOR_TO_IC_MOTOR(Axis)), EncPrescaler);
  }
  else
  {
    EncFactorFp = modf(EncFactor, &EncFactorIp);
    if (MotorConfig[Axis].EncoderResolution > 0)
    {
      EncPrescaler = ((int)EncFactorIp) << 16;
      EncPrescaler |= (int)(EncFactorFp * 10000.0);
    }
    else
    {
      EncPrescaler = (-((int)EncFactorIp) - 1) << 16;
      EncPrescaler |= 10000 - ((int)(EncFactorFp * 10000.0));
    }
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis)), ReadTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(Axis))) | TMC5072_ENC_SEL_DECIMAL_MASK);
    WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_ENC_CONST(MOTOR_TO_IC_MOTOR(Axis)), EncPrescaler);
  }
}

/*********************************************************//**
  \fn GetEncoderPosition(UCHAR Axis)
  \brief Read encoder position
  \param Axis  Motor number (always 0 with stealthRocker)
  \return Encoder position

  This function returns the value of the encoder
  position register in the TMC5072.
*************************************************************/
int GetEncoderPosition(UCHAR Axis)
{
  return ReadTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_XENC(MOTOR_TO_IC_MOTOR(Axis)));
}

/*********************************************************//**
  \fn SetEncoderPosition(UCHAR Axis, int Value)
  \brief Change encoder position
  \param Axis  Motor number (always 0 with stealthRocker)
  \param Value  New encoder position value

  This function position register in the TMC5072 to a new
  value.
*************************************************************/
void SetEncoderPosition(UCHAR Axis, int Value)
{
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Axis), TMC5072_XENC(MOTOR_TO_IC_MOTOR(Axis)), Value);
}
