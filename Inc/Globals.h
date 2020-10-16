/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Modul:   Globals.h
           Global variables and data structures.

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
  \file Globals.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains the definitions for importing the variables
  defined in Globals.c.
*/

#ifndef __GLOBALS_H
#define __GLOBALS_H

#include "Types.h"
#include "PnpSpindle.h"

#define EEPROM_MAGIC 0xE4

//! Global module settings
typedef struct
{
  UCHAR SerialBitrate;         //!< RS485 baud rate (0..7, 0=9600bps)
  UCHAR SerialModuleAddress;   //!< RS485 TMCL module address
  UCHAR SerialHostAddress;     //!< RS485 TMCL reply address
  float GaugePressureOffset;   //!< Gauge pressure offset
  UCHAR EEPROMMagic;           //!< EEPROM magic
} TModuleConfig;

//! Motor configuration data
typedef struct
{
  UINT StallVMin;                 //!< Minimum velocity for stallGuard
  UINT MaxDeviation;              //!< Maximum deviation
  INT EncoderResolution;          //!< Encoder resolution
  USHORT MotorFullStepResolution; //!< Motor full step resolution
  UCHAR UnitMode;                 //!< Unit conversion mode
  UCHAR EEPROMMagic;              //!< EEPROM magic
} TMotorConfig;

extern TModuleConfig ModuleConfig;
extern TMotorConfig MotorConfig[N_O_MOTORS];

extern UCHAR MotorFlags[N_O_MOTORS];
extern UCHAR VMaxModified[N_O_MOTORS];
extern int AMax[N_O_MOTORS];
extern int VMax[N_O_MOTORS];
extern UCHAR AMaxModified[N_O_MOTORS];

extern UCHAR ExitTMCLFlag;

#endif //__GLOBALS_H