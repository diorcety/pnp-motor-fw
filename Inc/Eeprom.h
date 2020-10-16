/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  Eeprom.h
           Definitions of EEPROM access functions

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
  \file Eeprom.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief EEPROM access functions

  This file contains the EEPROM access function defintions.
*/

#ifndef __EEPROM_H
#define __EEPROM_H

#include "stm32l0xx_hal.h"
#include "Globals.h"

#define DATA_EEPROM_SIZE (DATA_EEPROM_END - DATA_EEPROM_BASE)

#define MODULE_EEPROM_REGISTER_SIZE 2
#define MOTOR_EEPROM_REGISTER_SIZE 22

typedef struct
{
  TModuleConfig config;
  int registers[MODULE_EEPROM_REGISTER_SIZE];
} TModuleEeprom;

typedef struct
{
  TMotorConfig config;
  int registers[MOTOR_EEPROM_REGISTER_SIZE];
} TMotorEeprom;

typedef struct
{
  UCHAR crc;
  UCHAR version_low;
  UCHAR version_high;
  UCHAR pad;
} EepromHeader;

typedef struct
{
  EepromHeader header;
  TModuleEeprom content;
} EepromModuleContent;

typedef struct
{
  EepromHeader header;
  TMotorEeprom content;
} EepromMotorContent;

_Static_assert(((sizeof(EepromMotorContent)* N_O_MOTORS) + sizeof(EepromModuleContent)) <= DATA_EEPROM_SIZE, "EEPROM too small");

void InitEeprom();
void UpdateEeprom();
TModuleEeprom* GetModuleEeprom(UCHAR* valid);
TMotorEeprom* GetMotorEeprom(UCHAR WhichMotor, UCHAR* valid);

#endif //__EEPROM_H