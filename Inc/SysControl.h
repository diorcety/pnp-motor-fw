/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  SysControl.h

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
  \file SysControl.h
  \author Trinamic Motion Control GmbH & Co KG

  \brief Motor monitoring

  This file contains the definitions of the functions from the SysControl.c
  module.
*/

#ifndef SYSCONTROL_H
#define SYSCONTROL_H

#include "Types.h"

#define MOTOR_FLAG_STALL 1
#define MOTOR_FLAG_DEVIATION 2
#define MOTOR_FLAG_LSWITH 4
#define MOTOR_FLAG_RSWITH 8

int GetMeasuredSpeed(UCHAR Axis);
void SystemControl(void);

#endif //SYSCONTROL_H