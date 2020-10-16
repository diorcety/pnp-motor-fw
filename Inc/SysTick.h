/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SysTick.h
           Definitions of tick timer functions

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
  \file SysTick.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief System tick timer

  This file contains the definitions of the tick timer functions.
*/

#ifndef __SYSTICK_H
#define __SYSTICK_H

void SysTimerHandler(void);
UINT GetSysTimer(void);

#endif ///__SYSTICK_H
