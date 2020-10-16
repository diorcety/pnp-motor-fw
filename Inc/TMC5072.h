/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker TMCM-1316)

  Module:  TMC5072.h
           TMC5072 library

   Copyright (C) 2018 TRINAMIC Motion Control GmbH & Co KG
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
  \file TMC5072.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 1.00

  \brief TMC5072 Motor driver functions

  This file provides all functions needed for easy
  access to the TMC5072 stepper motor driver IC.
*/


#ifndef __TMC5072_H
#define __TMC5072_H

#include "stm32l0xx_hal.h"
#include <tmc/ic/TMC5072/TMC5072.h>

#define MOTOR_BY_IC 2
#define MOTOR_TO_IC_SPI(x) (x / MOTOR_BY_IC)
#define MOTOR_TO_IC_MOTOR(x) (x % MOTOR_BY_IC)

#define TMC5072_FREQ 16000000
#define TMC5072_VEL_FACTOR 0.9536743164         //fClk/2 / 2^23   (fClk=16MHz)
#define TMC5072_ACC_FACTOR 116.415321827        //fClk^2 / (512*256) / 2^24   (fClk=16MHz)
#define TMC5072_TPOWERDOWN_FACTOR 0.0032        //512 / fClk * 100 (fClk=16MHz)

// SPI
void WriteTMC5072Int(UCHAR WhichSPI, UCHAR Address, int Value);
int ReadTMC5072Int(UCHAR WhichSPI, UCHAR Address);

// Functions
void InitMotorDrivers(void);
void HardStop(UINT Motor);
void SoftStop(UINT Motor);
void Hold(UINT Motor);
void Update(UINT Motor);
int MotorStopped(UINT Motor);

// Conversions
int ConvertVelocityUserToInternal(int UserVelocity);
int ConvertAccelerationUserToInternal(int UserAcceleration);
int ConvertVelocityInternalToUser(int InternalVelocity);
int ConvertAccelerationInternalToUser(int InternalAcceleration);
int ConvertInternalToInternal(int Internal);

extern UCHAR TMC5072ReverseModuleRegisterMap[TMC5072_REGISTER_COUNT];
extern UCHAR TMC5072ReverseMotorRegisterMap[MOTOR_BY_IC][TMC5072_REGISTER_COUNT];

#define TMC5072_INVALID_REGISTER 0xFF

#endif
