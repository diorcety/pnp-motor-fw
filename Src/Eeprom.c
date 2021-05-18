/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  Eeprom.c
           Access to the onboard EEPROM

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
  \file Eeprom.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief EEPROM access functions

  This file contains EEPROM access functions.
*/

#include <string.h>
#include "Eeprom.h"
#include "CRC.h"
#include "SysTick.h"
#include "pt.h"

#define EEPROM_LOCAL_DEVICE N_O_MOTORS
#define EEPROM_DEVICE_N (N_O_MOTORS+1)
#define EEPROM_TIMEOUT 1000

typedef struct
{
  USHORT offset;
  USHORT size;
} EepromWriteParams;

typedef struct 
{
  UCHAR* eeprom;
  UCHAR* data;
  USHORT offset;
  USHORT count;
} EepromWriteInternal;

typedef struct
{
  EepromWriteParams params;
  EepromWriteInternal internal;
  USHORT offset;
} EepromWriteState;

typedef struct pt ProtoThread;

typedef struct
{
  UINT tick;

  ProtoThread pt;
  EepromWriteState state;

  USHORT offset;
  UCHAR valid;
} EepromState;


static EepromModuleContent eeprom_module_data;
static EepromModuleContent eeprom_module_cache;
static EepromModuleContent eeprom_module_copy;
static EepromMotorContent eeprom_motor_data[N_O_MOTORS];
static EepromMotorContent eeprom_motor_cache[N_O_MOTORS];
static EepromMotorContent eeprom_motor_copy[N_O_MOTORS];
static EepromState eeprom_states[EEPROM_DEVICE_N];

#define GET_EEPROM_DATA(device) (device != EEPROM_LOCAL_DEVICE?(UCHAR *)&eeprom_motor_data[device]:(UCHAR *)&eeprom_module_data)
#define GET_EEPROM_CACHE(device) (device != EEPROM_LOCAL_DEVICE?(UCHAR *)&eeprom_motor_cache[device]:(UCHAR *)&eeprom_module_cache)
#define GET_EEPROM_COPY(device) (device != EEPROM_LOCAL_DEVICE?(UCHAR *)&eeprom_motor_copy[device]:(UCHAR *)&eeprom_module_copy)
#define GET_EEPROM_SIZE(device) (device != EEPROM_LOCAL_DEVICE?sizeof(EepromMotorContent):sizeof(EepromModuleContent))
#define IS_EEPROM_BUSY(device) __HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)

typedef int EEPROM_Status;
#define EEPROM_ERROR 1
#define EEPROM_OK 0

/*
 * Local Eeprom functions
 */

static EEPROM_Status EEPROM_ReadBytes(USHORT Address, UCHAR* Data, UCHAR Size)
{
  if ((Address + Size) >= DATA_EEPROM_SIZE)
  {
    return EEPROM_ERROR;
  }

  /* Disable all IRQs */
  __disable_irq();

  USHORT i = 0;
  // 4 bytes
  {
    volatile uint32_t* currentAddress = (volatile uint32_t*)(DATA_EEPROM_BASE + Address + i);
    uint32_t* currentBuffer = (uint32_t*)&(Data[i]);
    while ((i + 4) <= Size)
    {
      *(currentBuffer++) = *(currentAddress++);
      i += 4;
    }
  }
  // 1 byte
  {
    volatile uint8_t* currentAddress = (volatile uint8_t*)(DATA_EEPROM_BASE + Address + i);
    uint8_t* currentBuffer = (uint8_t*)&(Data[i]);
    while ((i + 1) <= Size)
    {
      *(currentBuffer++) = *(currentAddress++);
      i += 1;
    }
  }

  /* Enable IRQs */
  __enable_irq();

  return EEPROM_OK;
}

static void pt_EEPROM_WriteBytes(ProtoThread* pt, EepromWriteState* state)
{
  EEPROM_Status ret = EEPROM_OK;

  pt_begin(pt);

  if (HAL_OK != HAL_FLASH_Unlock())
  {
    pt_exit(pt, EEPROM_ERROR);
  }

  while (state->internal.offset < state->params.size)
  {
    /* Disable all IRQs */
    __disable_irq();

    UCHAR* eeprom_chunk = &(state->internal.eeprom[state->internal.offset]);
    UCHAR* data_chunk = &(state->internal.data[state->internal.offset]);

    // 4 bytes
    if (state->params.size >= 4 && (((uint32_t)eeprom_chunk) & (4 - 1)) == 0)
    {
      state->internal.count = 4;
      *((volatile uint32_t*)eeprom_chunk) = *((uint32_t*)data_chunk);
    }
    // 2 bytes
    else if (state->params.size >= 2 && (((uint32_t)eeprom_chunk) & (2 - 1)) == 0)
    {
      state->internal.count = 2;
      *((volatile uint16_t*)eeprom_chunk) = *((uint16_t*)data_chunk);
    }
    // 1 byte
    else
    {
      state->internal.count = 1;
      *((volatile uint8_t*)eeprom_chunk) = *((uint8_t*)data_chunk);
    }

    /* Enable IRQs */
    __enable_irq();

    /* Wait for last operation to be completed */
    pt_wait(pt, !__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));

    /* Check FLASH End of Operation flag  */
    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP))
    {
      /* Clear FLASH End of Operation pending bit */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    }

    if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_PGAERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_SIZERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_RDERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_FWWERR) ||
      __HAL_FLASH_GET_FLAG(FLASH_FLAG_NOTZEROERR))
    {
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_SIZERR |
        FLASH_FLAG_OPTVERR |
        FLASH_FLAG_RDERR |
        FLASH_FLAG_FWWERR |
        FLASH_FLAG_NOTZEROERR);
      ret = EEPROM_ERROR;
    }

    if (ret != EEPROM_OK)
    {
      break;
    }

    /* Update pointers */
    state->internal.offset += state->internal.count;
  }

  HAL_FLASH_Lock();

  pt_exit(pt, ret);

  pt_end(pt);
}

/*
 * Eeprom CRC functions
 */

static UCHAR ComputeEepromModule(EepromHeader* header, size_t size)
{
  header->version_low = SW_VERSION_LOW;
  header->version_high = SW_VERSION_HIGH;
  UCHAR crc = crc8((void*)header + 1, size - 1);
  UCHAR diff = header->crc != crc;
  header->crc = crc;
  return diff;
}

static UCHAR CheckEepromModule(EepromHeader* header, size_t size)
{
  return crc8((void*)header + 1, size - 1) == header->crc;
}

/*
 * Eeprom common functions
 */

static UCHAR* GetEeprom(UCHAR device)
{
  if (device > N_O_MOTORS) return NULL;
  EepromState* state = &eeprom_states[device];
  UCHAR* copy = GET_EEPROM_COPY(device);
  UCHAR* data = GET_EEPROM_DATA(device);
  USHORT size = GET_EEPROM_SIZE(device);
  if (!state->valid)
  {
    EEPROM_Status ret = EEPROM_ReadBytes(sizeof(EepromMotorContent) * device, copy, size);
    if (ret == EEPROM_OK)
      memcpy(data, copy, size);
    state->valid = TRUE;
  }
  return data;
}

static void pt_WriteEeprom(UCHAR device)
{
  EepromState* state = &eeprom_states[device];
  if (pt_status(&state->pt) == PT_STATUS_IDLE)
  {
    state->state.internal.eeprom = (UCHAR *)(DATA_EEPROM_BASE + (sizeof(EepromMotorContent) * device) + state->state.params.offset);
    state->state.internal.data = GET_EEPROM_CACHE(device) + state->state.params.offset;
    state->state.internal.offset = 0;
  }
  pt_EEPROM_WriteBytes(&state->pt, &state->state);
}

/*********************************************//**
  \fn InitEeprom(void)
  \brief Initialize EEPROM
*************************************************/
void InitEeprom()
{
  for (UCHAR i = 0; i < EEPROM_DEVICE_N; ++i)
  {
    EepromState* state = &eeprom_states[i];
    state->valid = FALSE;
    state->offset = 0;
    state->tick = 0;

    pt_reset(&state->pt);
  }
}


/*********************************************//**
  \fn UpdateEeprom
  \brief EEPROM monitoring
*************************************************/
void UpdateEeprom()
{
  for (UCHAR i = 0; i < EEPROM_DEVICE_N; ++i)
  {
    EepromState* state = &eeprom_states[i];
    UCHAR* copy = GET_EEPROM_COPY(i);
    UCHAR* cache = GET_EEPROM_CACHE(i);
    UCHAR* data = GET_EEPROM_DATA(i);
    UCHAR size = GET_EEPROM_SIZE(i);

    if (!state->valid) continue;
    if (pt_status(&state->pt) == PT_STATUS_FINISHED)
    {
      if (pt_exitcode(&state->pt) == EEPROM_OK)  //Update the copy buffer if the EEPROM is correctly updated
      {
        memcpy(&copy[state->state.params.offset], &cache[state->state.params.offset], state->state.params.size);
      }
      pt_reset(&state->pt);
    }
    else if (pt_status(&state->pt) != PT_STATUS_IDLE)  //Execute the coroutine if is currently running
    {
      pt_WriteEeprom(i);
    }
    else if (IS_EEPROM_BUSY(i))
    {
      // Don't start a new eeprom write if one is currently running (shared eeprom)
      continue;
    }
    else if (state->offset != 0 || abs(GetSysTimer() - state->tick) >= EEPROM_TIMEOUT)  //Start or continue to update the changes
    {
      if (state->offset == 0)
      {
        //Update CRC
        if (!ComputeEepromModule((EepromHeader*)data, size))
        {
          //Data buffer not modified: abort the update
          state->tick = GetSysTimer();
          continue;
        }

        //Copy data to cache
        memcpy(cache, data, size);
      }

      //Find the first difference between cache and copy
      while (state->offset < size && cache[state->offset] == copy[state->offset]) state->offset++;
      if (state->offset != size)
      {
        //Wait the first equality between cache and copy
        UCHAR length = 0;
        UCHAR* cache_buffer = &cache[state->offset];
        UCHAR* copy_buffer = &copy[state->offset];
        while ((state->offset + length) < size && cache_buffer[length] != copy_buffer[length]) length++;

        state->state.params.offset = state->offset;
        state->state.params.size = length;

        // The next check will lock after this chunk
        state->offset += length;

        //Start the Write
        pt_WriteEeprom(i);
      }
      else
      {
        //EEPROM update done
        state->offset = 0;
        state->tick = GetSysTimer();
      }
    }
  }
}


/*********************************************//**
  \fn GetModuleEeprom(UCHAR *valid)
  \brief Get module EEPROM structure
*************************************************/
TModuleEeprom* GetModuleEeprom(UCHAR* valid)
{
  EepromModuleContent* eeprom = (EepromModuleContent*)GetEeprom(EEPROM_LOCAL_DEVICE);
  if (valid != NULL)
    *valid = (eeprom != NULL && CheckEepromModule(&(eeprom->header), sizeof(EepromModuleContent))) && eeprom->content.config.EEPROMMagic == EEPROM_MAGIC;
  return eeprom != NULL ? &eeprom->content : NULL;
}


/*********************************************//**
  \fn GetMotorEeprom(UCHAR WhichMotor, UCHAR* valid)
  \brief Get motor EEPROM structure
*************************************************/
TMotorEeprom* GetMotorEeprom(UCHAR WhichMotor, UCHAR* valid)
{
  EepromMotorContent* eeprom = (EepromMotorContent*)GetEeprom(WhichMotor);
  if (valid != NULL)
    *valid = (eeprom != NULL && CheckEepromModule(&(eeprom->header), sizeof(EepromMotorContent))) && eeprom->content.config.EEPROMMagic == EEPROM_MAGIC;
  return eeprom != NULL ? &eeprom->content : NULL;
}