/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  SPI.c
           Access to SPI devices (TMC429, TMC262, EEPROM)

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
  \file SPI.c
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief SPI functions

  This file provides all functions needed for SPI
  access to the other ICs (TMC429, TMC262, EEPROM).
*/

#include "Types.h"
#include "PnpSpindle.h"
#include "main.h"

/********************************************************//**
  \fn InitSPI()
  \brief Initalize SPI interface

  This function initializes the SPI interface.
************************************************************/
void InitSPI()
{
}

/***************************************************************//**
  \fn WriteSPIData(UCHAR DeviceNumber, UCHAR* Data, UCHAR Size)
  \brief SPI communication

  \param DeviceNumber  Index of the SPI device
  \param Data          Data byte buffer for sending bytes
  \param Size          Data byte buffer size
********************************************************************/
void WriteSPIData(USHORT DeviceNumber, UCHAR* Data, UCHAR Size)
{
  LL_SPI_Enable(SPI1);

  for (int i = 0; i < Size; ++i)
  {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, Data[i]);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    LL_SPI_ReceiveData8(SPI1);
  }
  while (LL_SPI_IsActiveFlag_BSY(SPI1));

  LL_SPI_Disable(SPI1);
}


/***************************************************************//**
  \fn ReadSPIData(UCHAR DeviceNumber, UCHAR* Data, UCHAR Size)
  \brief SPI communication

  \param DeviceNumber  Index of the SPI device
  \param Data          Data byte buffer for receiving bytes
  \param Size          Data byte buffer size
********************************************************************/
void ReadSPIData(USHORT DeviceNumber, UCHAR* Data, UCHAR Size)
{
  LL_SPI_Enable(SPI1);

  for (int i = 0; i < Size; ++i)
  {
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, 0);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    Data[i] = LL_SPI_ReceiveData8(SPI1);
  }
  while (LL_SPI_IsActiveFlag_BSY(SPI1));

  LL_SPI_Disable(SPI1);
}
