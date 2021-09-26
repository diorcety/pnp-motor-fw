/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  TMC5072.c
           TMC5072 library

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
  \file TMC5072.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief TMC5072 Motor driver functions

  This file provides all functions needed for easy
  access to the TMC5072 stepper motor driver IC.
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Types.h"
#include "main.h"
#include "PnpSpindle.h"
#include "Globals.h"
#include "SPI.h"
#include "SysTick.h"
#include "TMC5072.h"
#include "Eeprom.h"

static TMC5072TypeDef TMC5072[(N_O_MOTORS - 1) / MOTOR_BY_IC + 1];
static ConfigurationTypeDef Configuration[(N_O_MOTORS - 1) / MOTOR_BY_IC + 1];

UCHAR TMC5072ReverseModuleRegisterMap[TMC5072_REGISTER_COUNT];
UCHAR TMC5072ReverseMotorRegisterMap[MOTOR_BY_IC][TMC5072_REGISTER_COUNT];

const UCHAR TMC5072ModuleRegisterMap[MODULE_EEPROM_REGISTER_SIZE] =
{
  TMC5072_GCONF(MOTOR_TO_IC_MOTOR(0)),
  TMC5072_INVALID_REGISTER
};

const UCHAR TMC5072MotorRegisterMap[MOTOR_BY_IC][MOTOR_EEPROM_REGISTER_SIZE + 1] =
{
  {
    TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_XENC(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VSTART(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_A1(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_V1(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_AMAX(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VMAX(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_DMAX(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_D1(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VSTOP(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VCOOLTHRS(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VHIGH(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_VDCMIN(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(0)),
    TMC5072_INVALID_REGISTER,
  },
  {
    TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_XENC(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VSTART(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_A1(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_V1(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_AMAX(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VMAX(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_DMAX(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_D1(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VSTOP(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VCOOLTHRS(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VHIGH(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_VDCMIN(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(1)),
    TMC5072_INVALID_REGISTER
  }
};

/*
 * SPI
 */
void tmc5072_readWriteArray(uint8_t channel, uint8_t* data, size_t length)
{
  WriteSPIData(channel, data, length);
  ReadSPIData(channel, data, length);
}

void WriteTMC5072Int(UCHAR WhichSPI, UCHAR Address, int Value)
{
  tmc5072_writeInt(&TMC5072[WhichSPI], Address, Value);
}

int ReadTMC5072Int(UCHAR WhichSPI, UCHAR Address)
{
  return tmc5072_readInt(&TMC5072[WhichSPI], Address);
}

/***************************************************************//**
   \fn ConvertVelocityUserToInternal(int UserVelocity)
   \brief Convert from pps to internal unit
   \param UserVelocity: Velocity as pps value
   \return Internal velocity value

  This function converts a velocity value given in pps for use
  with TMC5072 velocity registers.
********************************************************************/
int ConvertVelocityUserToInternal(int UserVelocity)
{
  if (UserVelocity >= 0)
    return (int)floor((double)UserVelocity / TMC5072_VEL_FACTOR);
  else
    return (int)ceil((double)UserVelocity / TMC5072_VEL_FACTOR);
}


/***************************************************************//**
   \fn ConvertAccelerationUserToInternal(int UserAcceleration)
   \brief Convert from pps/s to internal unit
   \param UserAcceleration: Acceleration/Deceleration as pps/s value
   \return Internal acceleration/deceleration value

  This function converts an acceleration value or a deceleration value
  given in pps/s for use with TMC5072 acceleration/deceleration
  registers.
********************************************************************/
int ConvertAccelerationUserToInternal(int UserAcceleration)
{
  return (int)floor((double)UserAcceleration / TMC5072_ACC_FACTOR);
}


/***************************************************************//**
   \fn ConvertVelocityInternalToUser(int InternalVelocity)
   \brief Convert from internal unit to pps
   \param InternalVelocity: Velocity as internal value
   \return PPS velocity value

  This function converts a velocity value given in internal units
  of the TMC5072 back into pps.
********************************************************************/
int ConvertVelocityInternalToUser(int InternalVelocity)
{
  if (InternalVelocity >= 0)
    return (int)ceil((double)InternalVelocity * TMC5072_VEL_FACTOR);
  else
    return (int)floor((double)InternalVelocity * TMC5072_VEL_FACTOR);
}


/***************************************************************//**
   \fn ConvertAccelerationInternalToUser(int InternalAcceleration)
   \brief Convert from internal unit to pps/s
   \param InternalAcceleration: Accleration/Deceleration as internal value
   \return PPS/S acceleration/deceleration value

  This function converts an acceleration/deceleration value given
  in internal units of the TMC5072 back into pps/s.
********************************************************************/
int ConvertAccelerationInternalToUser(int InternalAcceleration)
{
  return (int)ceil((float)InternalAcceleration * TMC5072_ACC_FACTOR);
}


/***************************************************************//**
   \fn ConvertInternalToInternal(int Internal)
   \brief Dummy function used when unit conversion is switched off
   \param Internal: Input value
   \return Unchanged value

   This is a dummy function which is used when unit conversion is
   switched off.
********************************************************************/
int ConvertInternalToInternal(int Internal)
{
  return Internal;
}


static void configCallback(TMC5072TypeDef* tmc5072, ConfigState state)
{
  if (state == CONFIG_RESET)
  {
    // Fill missing shadow registers (hardware preset registers)
    tmc5072_fillShadowRegisters(tmc5072);
  }
}


/***************************************************************//**
   \fn InitMotorDrivers(void)
   \brief Initialise all motor drivers

   This function initalizes the software copies of all TMC5072
   registers and sends this basic initialization data to all
   TMC5072 ICs.
********************************************************************/
int32_t resetState[TMC5072_REGISTER_COUNT];
void InitMotorDrivers(void)
{
  // Fill reverse register map
  for (unsigned int j = 0; j < MOTOR_BY_IC; ++j)
  {
    for (unsigned int i = 0; i < TMC5072_REGISTER_COUNT; i++)
      TMC5072ReverseMotorRegisterMap[j][i] = TMC5072_INVALID_REGISTER;
    for (unsigned int i = 0; i < MOTOR_EEPROM_REGISTER_SIZE && TMC5072MotorRegisterMap[j][i] != TMC5072_INVALID_REGISTER; i++)
      TMC5072ReverseMotorRegisterMap[j][TMC5072MotorRegisterMap[j][i]] = i;
  }
  for (unsigned int i = 0; i < TMC5072_REGISTER_COUNT; i++)
    TMC5072ReverseModuleRegisterMap[i] = TMC5072_INVALID_REGISTER;
  for (unsigned int i = 0; i < MODULE_EEPROM_REGISTER_SIZE && TMC5072ModuleRegisterMap[i] != TMC5072_INVALID_REGISTER; i++)
    TMC5072ReverseModuleRegisterMap[TMC5072ModuleRegisterMap[i]] = i;

  UCHAR valid;
  for (unsigned int i = 0; i < N_O_MOTORS; i++)
  {
    if ((i % N_O_MOTORS) == 0)
    {
      memcpy(&resetState[0], &tmc5072_defaultRegisterResetState[0], sizeof(tmc5072_defaultRegisterResetState));
      TModuleEeprom* eeprom = GetModuleEeprom(&valid);
      if (valid)
      {
        for (unsigned int j = 0; j < MODULE_EEPROM_REGISTER_SIZE; ++j)
        {
          if (TMC5072ModuleRegisterMap[j] == TMC5072_INVALID_REGISTER)
          {
            break;
          }
          resetState[TMC5072ModuleRegisterMap[j]] = eeprom->registers[j];
        }
      }
      else
      {
        resetState[TMC5072_GCONF(MOTOR_TO_IC_MOTOR(i))] = TMC5072_SHAFT1_MASK | TMC5072_SHAFT2_MASK;

        for (unsigned int j = 0; j < MODULE_EEPROM_REGISTER_SIZE; ++j)
        {
          if (TMC5072ModuleRegisterMap[j] == TMC5072_INVALID_REGISTER)
          {
            break;
          }
          eeprom->registers[j] = resetState[TMC5072ModuleRegisterMap[j]];
        }
      }
    }

    TMotorEeprom* eeprom = GetMotorEeprom(i, &valid);
    if (valid)
    {
      //Take data from EEPROM
      memcpy(&MotorConfig[i], &eeprom->config, sizeof(TMotorConfig));
      for (unsigned int j = 0; j < MOTOR_EEPROM_REGISTER_SIZE; ++j)
      {
        if (TMC5072MotorRegisterMap[MOTOR_TO_IC_MOTOR(i)][j] == TMC5072_INVALID_REGISTER)
        {
          break;
        }
        resetState[TMC5072MotorRegisterMap[MOTOR_TO_IC_MOTOR(i)][j]] = eeprom->registers[j];
      }
    }
    else
    {
      //Default values
      resetState[TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(i))] = 0x000100C5; // CHOPCONF: TOFF=5, HSTRT=4, HEND=1, TBL=2, CHM=0 (spreadCycle)
      resetState[TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(i))] = 0x00071004; // IHOLD_IRUN: IHOLD=4, IRUN=16, IHOLDDELAY=7
      resetState[TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(i))] = 0x000401C8; // PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amplitude limit=200, Grad=1
      resetState[TMC5072_VHIGH(MOTOR_TO_IC_MOTOR(i))] = 0x00061A80; //  VHIGH=400 000: Set VHIGH to a high value to allow stealthChop
      resetState[TMC5072_VCOOLTHRS(MOTOR_TO_IC_MOTOR(i))] = 0x00007530; // VCOOLTHRS=30000: Set upper limit for stealthChop to about 30RPM
      resetState[TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(i))] = 0x00002710; // TZEROWAIT=10000

      resetState[TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(i))] = 0;
      resetState[TMC5072_VSTART(MOTOR_TO_IC_MOTOR(i))] = 1;
      resetState[TMC5072_A1(MOTOR_TO_IC_MOTOR(i))] = 220;
      resetState[TMC5072_V1(MOTOR_TO_IC_MOTOR(i))] = 26843;
      resetState[TMC5072_AMAX(MOTOR_TO_IC_MOTOR(i))] = 439;
      resetState[TMC5072_VMAX(MOTOR_TO_IC_MOTOR(i))] = 53687;
      resetState[TMC5072_DMAX(MOTOR_TO_IC_MOTOR(i))] = 439;
      resetState[TMC5072_D1(MOTOR_TO_IC_MOTOR(i))] = 220;
      resetState[TMC5072_VSTOP(MOTOR_TO_IC_MOTOR(i))] = 10;

      //Save default data in EEPROM
      memcpy(&eeprom->config, &MotorConfig[i], sizeof(TMotorConfig));
      for (unsigned int j = 0; j < MOTOR_EEPROM_REGISTER_SIZE; ++j)
      {
        if (TMC5072MotorRegisterMap[MOTOR_TO_IC_MOTOR(i)][j] == TMC5072_INVALID_REGISTER)
        {
          break;
        }
        eeprom->registers[j] = resetState[TMC5072MotorRegisterMap[MOTOR_TO_IC_MOTOR(i)][j]];
      }
    }

    // Always start in standstill state
    resetState[TMC5072_RAMPMODE(MOTOR_TO_IC_MOTOR(i))] = TMC5072_MODE_HOLD;
    resetState[TMC5072_XENC(MOTOR_TO_IC_MOTOR(i))] = resetState[TMC5072_XTARGET(MOTOR_TO_IC_MOTOR(i))] = resetState[TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(i))];

    if (((i + 1) % N_O_MOTORS) == 0 || i == (N_O_MOTORS - 1))
    {
      tmc5072_init(&TMC5072[MOTOR_TO_IC_SPI(i)], MOTOR_TO_IC_SPI(i), &Configuration[MOTOR_TO_IC_SPI(i)], &resetState[0]);
      tmc5072_setCallback(&TMC5072[MOTOR_TO_IC_SPI(i)], configCallback);
      tmc5072_reset(&TMC5072[MOTOR_TO_IC_SPI(i)]);
    }
  }

  // Initialize the ICs
  for (unsigned int i = 0; i < (MOTOR_TO_IC_SPI(N_O_MOTORS - 1) + 1); ++i)
  {
    {
      while (Configuration[MOTOR_TO_IC_SPI(i)].state != CONFIG_READY)
      {
        tmc5072_periodicJob(&TMC5072[MOTOR_TO_IC_SPI(i)], 0);

#if !defined(DEBUG)
        LL_IWDG_ReloadCounter(IWDG);
#endif
      }
    }
  }
}

/***************************************************************//**
   \fn HardStop(UINT Motor)
   \brief Stop motor immediately
   \param Motor  Motornummer

   Stop a motor immediately, without deceleration ramp.
********************************************************************/
void HardStop(UINT Motor)
{
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(Motor)), 0);
  VMaxModified[Motor] = TRUE;
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(Motor)), TMC5072_AMAX_MASK);
  AMaxModified[Motor] = TRUE;
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_RAMPMODE(MOTOR_TO_IC_MOTOR(Motor)), (TMC5072_MODE_VELPOS << TMC5072_RAMPMODE_SHIFT) & TMC5072_RAMPMODE_MASK);
}


/***************************************************************//**
   \fn SoftStop(UINT Motor)
   \brief Stop motor
   \param Motor  Motornummer

   Stop a motor, with deceleration ramp.
********************************************************************/
void SoftStop(UINT Motor)
{
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(Motor)), 0);
  VMaxModified[Motor] = TRUE;
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_RAMPMODE(MOTOR_TO_IC_MOTOR(Motor)), (TMC5072_MODE_VELPOS << TMC5072_RAMPMODE_SHIFT) & TMC5072_RAMPMODE_MASK);
}


/***************************************************************//**
   \fn Hold(UINT Motor)
   \brief Hold motor with current velocity
   \param Motor  Motornummer

   Hold motor with current velocity
********************************************************************/
void Hold(UINT Motor)
{
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_RAMPMODE(MOTOR_TO_IC_MOTOR(Motor)), (TMC5072_MODE_HOLD << TMC5072_RAMPMODE_SHIFT) & TMC5072_RAMPMODE_MASK);
}

/***************************************************************//**
   \fn Update(UINT Motor)
   \brief Update motor internal settings
   \param Motor  Motornummer

   Update motor internal settings
********************************************************************/
void Update(UINT Motor)
{
  tmc5072_periodicJob(&TMC5072[MOTOR_TO_IC_SPI(Motor)], 0);
}

/***************************************************************//**
   \fn MotorStopped(UINT Motor)
   \brief Check if the motor is stopped
   \param Motor  Motornummer

   Return 1 if the motor is stopped
********************************************************************/
int MotorStopped(UINT Motor)
{
  return (ReadTMC5072Int(MOTOR_TO_IC_SPI(Motor), TMC5072_DRVSTATUS(MOTOR_TO_IC_MOTOR(Motor))) & TMC5072_STST_MASK) ? 1 : 0;
}