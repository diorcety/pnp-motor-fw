/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  SysTick.c
           Use of the system tick timer (1ms timer)

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
  \file SysTick.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief 1ms system tick timer functions

  This file provides all functions needed for easy
  access to the TMC262 stepper motor driver IC.
*/

#include "main.h"
#include "Types.h"
#include "PnpSpindle.h"
#include "UART.h"

extern volatile UINT UARTTransmitDelayTimer;
extern volatile UCHAR UARTTimeoutFlag;
extern volatile UINT UARTTimeoutTimer;
__IO uint32_t uwTick;

/***************************************************//**
  \fn SysTimerHandler(void)
  \brief System timer interrupt handler

  The system timer interrupt handler counts up the
  1ms counter.
*******************************************************/
void SysTimerHandler(void)
{
  uwTick++;

  //Count down RS485 transmit delay
  if (UARTTransmitDelayTimer > 0)
  {
    UARTTransmitDelayTimer--;
    UARTTransmitCheckTimeout();
  }

  //Count down RS485 receive timeout
  if (UARTTimeoutTimer > 0)
  {
    UARTTimeoutTimer--;
    if (UARTTimeoutTimer == 0)
    {
      UARTTimeoutFlag = TRUE;
    }
  }
}

/***************************************************//**
  \fn GetSysTimer(void)
  \brief Read the system timer
  \return System timer (1ms)

  This function returns the actual value of the 1ms
  system timer.
*******************************************************/
UINT GetSysTimer(void)
{
  return uwTick;
}
