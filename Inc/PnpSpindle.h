/*******************************************************************************
  Project: MVPnP Motor Board

  Modul:   PnpSpindle.h
           Definitions of globally used data types and macros

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
  \file PnpSpindle.h
  \author Trinamic Motion Control GmbH & Co KG

  \brief Basic type and macro definitions

  This file contains basic type and macro definitions needed by all
  modules of this project.
*/

#ifndef __PNPSPINDLE_H
#define __PNPSPINDLE_H

#include "Types.h"

#define SW_TYPE_HIGH 0x73                      //!< module number high byte
#define SW_TYPE_LOW  0x38                      //!< module number low byte

#define SW_VERSION_HIGH 0x00                   //!< software version high byte
#define SW_VERSION_LOW  0x01                   //!< software version low byte
#define SW_VERSION_STR  "7338V001"             //!< software version string version

#define N_O_MOTORS 2                           //!< number of motors supported by this module

#define DISABLE_DRIVERS() LL_GPIO_SetOutputPin(DRV_ENN_GPIO_Port, DRV_ENN_Pin);      //!< turn off all motor drivers
#define ENABLE_DRIVERS() LL_GPIO_ResetOutputPin(DRV_ENN_GPIO_Port, DRV_ENN_Pin);     //!< turn on all motor drivers

#endif //__PNPSPINDLE_H