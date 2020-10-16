/*******************************************************************************
  Project: stealthRocker Mini-TMCL (for stealthRocker)

  Module:  Globals.c
           Global variables and data structures

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
  \file Globals.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Global variables

  This file contains all globally used variables.
*/

#include "Globals.h"

//! Global parameters (here only used for the RS232 interface)
TModuleConfig ModuleConfig =
{
  0,              //!< RS485 bitrate (0=9600)
  1,              //!< RS485 address
  2,              //!< RS485 reply address
  0.0f,           //!< Gauge pressure offset
  EEPROM_MAGIC,   //!< EEPROM magic
};

//! Motor configuration data
TMotorConfig MotorConfig[N_O_MOTORS] =
{
  {
    0,             //!< StallVMin
    0,             //!< MaxDeviation
    4096,          //!< EncoderResolution
    200,           //!< MotorFullStepResolution
    1,             //!< UnitMode
    EEPROM_MAGIC,  //!< EEPROM magic
  },
  {
    0,             //!< StallVMin
    0,             //!< MaxDeviation
    4096,          //!< EncoderResolution
    200,           //!< MotorFullStepResolution
    1,             //!< UnitMode
    EEPROM_MAGIC,  //!< EEPROM magic
  }
};

UCHAR MotorFlags[N_O_MOTORS];       //!< Motor flag states
UCHAR VMaxModified[N_O_MOTORS];     //!< Maximum positioning speed has been modified
UCHAR AMaxModified[N_O_MOTORS];     //!< Maximum acceleration has been modified
int AMax[N_O_MOTORS];               //!< Maximum acceleration (axis parameter #5)
int VMax[N_O_MOTORS];               //!< Maximum positioning speed

UCHAR ExitTMCLFlag;   //!< This will be set to TRUE for exiting TMCL and branching to the boot loader
