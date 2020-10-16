/*******************************************************************************
  Project: stepRocker Mini-TMCL (for stepRocker V2.2)

  Module:  UART.h
           Definitons of UART functions

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
  \file UART.h
  \author Trinamic Motion Control GmbH & Co KG
  \version 2.20

  \brief Motor monitoring

  This file contains the definitions of the UART functions.
*/

#ifndef __UART_H
#define __UART_H

void InitUART(UCHAR Baudrate);
void InterruptUART();
void WriteUART(UCHAR Byte);
UCHAR ReadUART(UCHAR* Byte);
void SetUARTTransmitDelay(UINT Delay);
void UARTTransmitCheckTimeout(void);
UINT CheckUARTTimeout(void);

#endif //__UART_H