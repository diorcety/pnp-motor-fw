/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  SysControl.c
           Motor monitoring (automatic current switching etc.)

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
  \file SysControl.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief Motor monitoring

  This file contains the SystemControl function which does all necessary motor
  monitoring tasks.
*/

#include <stdlib.h>
#include "SysControl.h"
#include "PnpSpindle.h"
#include "Globals.h"
#include "SysTick.h"
#include "TMC5072.h"

static UCHAR ActualAxis;                      //!< monitored axis
static int TOld[N_O_MOTORS];                  //!< last time (for speed measuring)
static int XOld[N_O_MOTORS];                  //!< last position (for speed measuring)
static int MeasuredSpeed[N_O_MOTORS];         //!< measured speed
static UCHAR StopOnStallState[N_O_MOTORS];    //!< state of stop on stall function

/***************************************************************//**
   \fn GetMeasuredSpeed(UCHAR Axis)
   \brief Read measured speed
   \param Axis  Axis number (with stepRocker always 0)
   \return Measured speed

   This function returns the speed measured by the TMC5072.
********************************************************************/
int GetMeasuredSpeed(UCHAR Axis)
{
  return MeasuredSpeed[Axis];
}

/***************************************************************//**
   \fn SystemControl(void)
   \brief Motor monitoring

   This function must be called periodically from the main loop and
   does some monitoring tasks, e.g. lowering the current after the
   motor has not been moving for some time.
********************************************************************/
void SystemControl(void)
{
  int XActual;
  int t2;
  UINT RampStat;

  //speed measuring (mainly for dcStep)
  if (abs(GetSysTimer() - TOld[ActualAxis]) > 10)
  {
    t2 = GetSysTimer();
    XActual = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(ActualAxis)));

    MeasuredSpeed[ActualAxis] = ((float)(XActual - XOld[ActualAxis]) / (float)(t2 - TOld[ActualAxis])) * 1048.576;
    TOld[ActualAxis] = t2;
    XOld[ActualAxis] = XActual;
  }

  //status of the ramp generator
  RampStat = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_RAMPSTAT(MOTOR_TO_IC_MOTOR(ActualAxis)));

  //stop on stall
  if (RampStat & TMC5072_EVENT_STOP_SG_MASK)
  {
    HardStop(ActualAxis);
    MotorFlags[ActualAxis] |= MOTOR_FLAG_STALL;
  }

  //stop on left switch
  if ((RampStat & TMC5072_EVENT_STOP_L_MASK) && MotorStopped(ActualAxis))
  {
    Hold(ActualAxis);
    MotorFlags[ActualAxis] |= MOTOR_FLAG_LSWITH;
  }

  //stop on right switch
  if ((RampStat & TMC5072_EVENT_STOP_R_MASK) && MotorStopped(ActualAxis))
  {
    Hold(ActualAxis);
    MotorFlags[ActualAxis] |= MOTOR_FLAG_RSWITH;
  }

  //switch stop on stall off or on depending on the speed
  if (MotorConfig[ActualAxis].StallVMin > 0 && ((UINT)abs(ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_VACTUAL(MOTOR_TO_IC_MOTOR(ActualAxis))))) > MotorConfig[ActualAxis].StallVMin)
  {
    if (!StopOnStallState[ActualAxis])
    {
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualAxis)), ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualAxis))) | TMC5072_SG_STOP_MASK);
      StopOnStallState[ActualAxis] = TRUE;
    }
  }
  else
  {
    if (StopOnStallState[ActualAxis])
    {
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualAxis)), ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualAxis))) & ~TMC5072_SG_STOP_MASK);
      StopOnStallState[ActualAxis] = FALSE;
    }
  }

  //Reset flags
  WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_RAMPSTAT(MOTOR_TO_IC_MOTOR(ActualAxis)), TMC5072_EVENT_POS_REACHED_MASK | TMC5072_EVENT_STOP_SG_MASK);

  //Stop on deviation
  if (MotorConfig[ActualAxis].MaxDeviation > 0 && !(RampStat & TMC5072_VZERO_MASK))
  {
    if (((UINT)abs(ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_XENC(MOTOR_TO_IC_MOTOR(ActualAxis))) - ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(ActualAxis))))) > MotorConfig[ActualAxis].MaxDeviation)
    {
      HardStop(ActualAxis);
      MotorFlags[ActualAxis] |= MOTOR_FLAG_DEVIATION;
    }
  }

  //Reset events after they have been processed,
  //but only if they already had been set when entering this procedure.
  //Otherwise they could be reset without having been processed.
  if (RampStat & TMC5072_EVENT_POS_REACHED_MASK)
    WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_RAMPSTAT(MOTOR_TO_IC_MOTOR(ActualAxis)), TMC5072_EVENT_POS_REACHED_MASK);
  if (RampStat & TMC5072_EVENT_STOP_SG_MASK)
    WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualAxis), TMC5072_RAMPSTAT(MOTOR_TO_IC_MOTOR(ActualAxis)), TMC5072_EVENT_STOP_SG_MASK);

  //next motor (stepRocker only has one)
  ActualAxis++;
  if (ActualAxis >= N_O_MOTORS)
  {
    ActualAxis = 0;
  }
}
