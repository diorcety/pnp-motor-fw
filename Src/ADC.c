
/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  ADC.h
           Definitions for ADC

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
  \file ADC.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief Mini TMCL interpreter

  This file all functions necessary to implement a ADC.
*/

#include "ADC.h"
#include "Globals.h"
#include "main.h"

#define ADC_CLOCK_DIV 4

static float pValue;

/********************************************************//**
  \fn InitADC()
  \brief Initalize ADC

  This function initializes the ADC.
************************************************************/
void InitADC()
{
  // Calibrate
  LL_ADC_StartCalibration(ADC1);
  while (LL_ADC_IsCalibrationOnGoing(ADC1));

  // Wait calibration delay
  uint32_t wait_loop_index;
  wait_loop_index = (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * ADC_CLOCK_DIV / 2);
  while (wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  // Enable
  LL_ADC_EnableIT_EOC(ADC1);
  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversion(ADC1);
}

/****************************************************//**
  \fn GetGaugePressure()
  \brief Return gauge pressure

  Return the converted gauge pressure voltage to
  kPa float value
********************************************************/
float GetGaugePressure()
{
  return pValue;
}

static void ComputeGaugePressure(uint16_t adc)
{
  // * ADC *
  // VOUT = ADC_NORM x VS
  // ADC = VOUT / VS x 4096
  // ADC = ADC_NORM x 4096

  // * MP3V5050V *
  // VOUT = VS x (0.018 x P + 0.94) ± (Pressure Error x Temp Multi x 0.018 x VS)
  // P = ((VOUT ± (Pressure Error x Temp Multi x 0.018 x VS)) / VS - 0.94) / 0.018

  // * Simplification *
  // P = ((VSxADC_NORM ± (Pressure Error x Temp Multi x 0.018 x VS)) / VS - 0.94) / 0.018
  // P = (ADC_NORM ± (Pressure Error x Temp Multi x 0.018) - 0.94) / 0.018
  // P = (ADC_NORM - 0.94) / 0.018 ± (Pressure Error x Temp Multi)

  float adc_norm = (float)(adc) / (float)(0xFFF);
  pValue = (adc_norm - 0.94f) / 0.018f + ModuleConfig.GaugePressureOffset;
}

/****************************************************//**
  \fn InterruptADC()
  \brief ADC interrupt handler
********************************************************/
void InterruptADC(void)
{
  if (LL_ADC_IsActiveFlag_EOC(ADC1) && LL_ADC_IsEnabledIT_EOC(ADC1))
  {
    ComputeGaugePressure(LL_ADC_REG_ReadConversionData12(ADC1));

    // Clear IT flag
    LL_ADC_ClearFlag_EOC(ADC1);
  }
}