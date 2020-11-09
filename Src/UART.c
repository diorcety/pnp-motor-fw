/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  UART.c
           Access to the UART interface

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
  \file UART.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief UART functions

  This file provides all functions needed for using
  the UART interface.
*/

#include "main.h"
#include "Types.h"
#include "main.h"
#include "PnpSpindle.h"
#include <circbuf.h>

#define UART_TIMEOUT_VALUE 3       //!< Timeout value (ms)

#define BAUDRATE_COUNT 12
/*
   0    1    2    3    4    5    6    7     8     9   10   11
  9.6  14.4 19.2 28.8 38.4 57.6 76.8 115.2 230.4 250 500 1000 kBaud
*/
static const UINT baudrateTable[BAUDRATE_COUNT] = { 9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400, 250000, 500000, 1000000 };

CIRC_GBUF_DEF(uint8_t, RxBuffer, 32);
CIRC_GBUF_DEF(uint8_t, TxBuffer, 32);

static volatile UINT UARTTransmitDelay;                 //!< Delay between receiving and sending
volatile UCHAR UARTTimeoutFlag;                         //!< Timeout flag (gets set in the system timer interrupt)
volatile UINT UARTTimeoutTimer;                         //!< Timeout timer (gets deceremented in the system timer interrupt)
volatile UINT UARTTransmitDelayTimer = 0;               //!< Timer for delay between receiving and sending (gets decremented in the system timer interrupt)

#if 1
#define USART2_HALF 1
#define UART_CONFIG_RX 0
#define UART_CONFIG_TX 1
#define UART_CONFIG_NONE 2
#else
#define USART2_HALF 0
#endif

#ifdef USART2

#if USART2_HALF
static uint8_t USART2_Config = UART_CONFIG_NONE;

void USART2_LoadRxConfig()
{
  if (USART2_Config != UART_CONFIG_RX)
  {
    USART2_Config = UART_CONFIG_RX;
    LL_USART_DisableIT_TXE(USART2);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_RX);
    LL_USART_EnableIT_RXNE(USART2);
  }
}

void USART2_LoadTxConfig()
{
  if (USART2_Config != UART_CONFIG_TX)
  {
    USART2_Config = UART_CONFIG_TX;
    LL_USART_DisableIT_RXNE(USART2);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX);
    LL_USART_EnableIT_TXE(USART2);
  }
}
#endif


/****************************************************//**
  \fn InterruptUART()
  \brief UART interrupt handler
********************************************************/
void InterruptUART(void)
{
  uint8_t byte;
  if (LL_USART_IsEnabledIT_RXNE(USART2) && LL_USART_IsActiveFlag_RXNE(USART2))
  {
    byte = LL_USART_ReceiveData8(USART2);
    CIRC_GBUF_PUSH(RxBuffer, &byte);

    //Set receive timeout to start value
    UARTTimeoutTimer = UART_TIMEOUT_VALUE;

    //Set transmit delay to start value
    UARTTransmitDelayTimer = UARTTransmitDelay;
  }
  if (LL_USART_IsEnabledIT_TXE(USART2) && LL_USART_IsActiveFlag_TXE(USART2))
  {
    if (CIRC_GBUF_POP(TxBuffer, &byte) == 0)
    {
      LL_USART_TransmitData8(USART2, byte);
    }
    else
    {
      LL_USART_DisableIT_TXE(USART2);
    }
  }

  if (LL_USART_IsEnabledIT_ERROR(USART2))
  {
    if (LL_USART_IsActiveFlag_FE(USART2))
    {
      LL_USART_ClearFlag_FE(USART2);
    }
    if (LL_USART_IsActiveFlag_ORE(USART2))
    {
      LL_USART_ClearFlag_ORE(USART2);
    }
    if (LL_USART_IsActiveFlag_NE(USART2))
    {
      LL_USART_ClearFlag_NE(USART2);
    }
  }
}
#endif //USART2

/********************************************************//**
  \fn InitUART(UCHAR baudrateIndex)
  \brief Initalize UART interface
  \param baudrateIndex    baud rate code (0..11)

  This function initializes the UART interface. The baud
  rate codes are the same as with TMCL.
************************************************************/
void InitUART(UCHAR baudrateIndex)
{
#ifdef USART2
  LL_USART_Disable(USART2);

  LL_USART_SetBaudRate(USART2,
    LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE),
    LL_USART_GetOverSampling(USART2),
    baudrateTable[baudrateIndex]);
  LL_USART_EnableIT_ERROR(USART2);
#if USART2_HALF
  USART2_LoadRxConfig();
#else
  LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
  LL_USART_EnableIT_RXNE(USART2);
#endif

  LL_USART_Enable(USART2);
#endif //USART2
}


/****************************************************//**
  \fn UARTTransmitCheckTimeout()
  \brief Check UART transmit timeout
********************************************************/
void UARTTransmitCheckTimeout(void)
{
#ifdef USART2

  if (UARTTransmitDelayTimer == 0)
  {
    if (CIRC_GBUF_US(TxBuffer) != 0)
    {
#if USART2_HALF
      USART2_LoadTxConfig();
#else
      LL_USART_EnableIT_TXE(USART2);
#endif //USART2_HALF
    }
  }
#endif //USART2
}


/****************************************************//**
  \fn WriteUART(char Byte)
  \brief Write to the UART interface
  \param Byte  Byte to be written

  This function puts a byte into the UART transmit
  buffer and starts sending if not already done.
********************************************************/
void WriteUART(UCHAR Byte)
{
#ifdef USART2
  CIRC_GBUF_PUSH(TxBuffer, &Byte);
  UARTTransmitCheckTimeout();
#endif //USART2
}


/****************************************************//**
  \fn ReadUART(char *Byte)
  \brief Read from the UART interface
  \param Byte  Pointer to variable for result
  \return TRUE if a byte could be read\n
          FALSE if the receive buffer was empty

  This function tries to read a byte from the UART receive
  buffer.
********************************************************/
UCHAR ReadUART(UCHAR* Byte)
{
#ifdef USART2
  if (LL_USART_IsActiveFlag_TC(USART2))
  {
    LL_USART_ClearFlag_TC(USART2);
#if USART2_HALF
    USART2_LoadRxConfig();
#endif //USART2_HALF
  }

  return CIRC_GBUF_POP(RxBuffer, Byte) == 0;
#else
  return FALSE;
#endif //USART2
}


/****************************************************//**
  \fn SetUARTTransmitDelay(UINT Delay)
  \brief Set UART transmit delay
  \param Delay  Delay in ms

  This function sets the delay between receiving the last
  byte and sending the first byte. This can be necessary
  with some UART transceiver (like UART half duplex).
********************************************************/
void SetUARTTransmitDelay(UINT Delay)
{
  UARTTransmitDelay = Delay;
}


/*******************************************************************************//**
  \fn CheckUARTTimeout(void)
  \brief Check and reset UART timeout flag
  \return TRUE if there has been a timeout (>5ms after last received byte)\n
          FALSE if there has not been a timeout since the last call of this function

  This function checks the timeout flag and then resets it.
***********************************************************************************/
UINT CheckUARTTimeout(void)
{
  if (UARTTimeoutFlag)
  {
    UARTTimeoutFlag = FALSE;
    return TRUE;
  }
  else return FALSE;
}
