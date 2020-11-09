/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  Commands.c
           Processing of TMCL commands

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
  \file Commands.c
  \author Trinamic Motion Control GmbH & Co KG

  \brief Mini TMCL interpreter

  This file all functions necessary to implement a small TMCL interpreter.
*/

#include <stdlib.h>
#include <math.h>
#include "Types.h"
#include "PnpSpindle.h"
#include "main.h"
#include "Commands.h"
#include "Globals.h"
#include "ADC.h"
#include "UART.h"
#include "SysControl.h"
#include "TMC5072.h"
#include "Eeprom.h"
#include "Encoder.h"

#define VALID_MOTOR() (ActualCommand.Motor<N_O_MOTORS)

#define REGISTER_TO_VALUE(VALUE, MASK, SHIFT) (((VALUE) & (MASK)) >> (SHIFT))
#define VALUE_TO_REGISTER(VALUE, MASK, SHIFT) (((VALUE) << (SHIFT)) & (MASK))

#define CHECK_PARAMETER(VALUE, MASK, SHIFT)                                               \
   if((VALUE) != ((VALUE) & ((MASK) >> (SHIFT))))                                         \
   {                                                                                      \
     ActualReply.Status=REPLY_INVALID_VALUE;                                              \
     return;                                                                              \
   }

#define WRITE_PARAMETER(MOTOR, REGISTER, MASK, SHIFT, ORIGINAL_VALUE, NEW_VALUE)          \
   CHECK_PARAMETER(NEW_VALUE, MASK, SHIFT)                                                \
   WriteTMC5072Int(MOTOR_TO_IC_SPI(MOTOR), REGISTER(MOTOR_TO_IC_MOTOR(MOTOR)),            \
     ((ORIGINAL_VALUE) & ~(MASK)) | VALUE_TO_REGISTER(NEW_VALUE, MASK, SHIFT)             \
   )

#define READ_WRITE_PARAMETER(MOTOR, REGISTER, MASK, SHIFT, VALUE)                         \
   WRITE_PARAMETER(MOTOR, REGISTER, MASK, SHIFT, ReadTMC5072Int(MOTOR_TO_IC_SPI(MOTOR), REGISTER(MOTOR_TO_IC_MOTOR(MOTOR))), VALUE)


#define READ_PARAMETER(MOTOR, REGISTER, MASK, SHIFT) REGISTER_TO_VALUE(ReadTMC5072Int(MOTOR_TO_IC_SPI(MOTOR), REGISTER(MOTOR_TO_IC_MOTOR(MOTOR))), MASK, SHIFT)

//Variables
static UCHAR UARTCmd[9];                    //!< UART command buffer
static UCHAR UARTCount;                     //!< UART commnd byte counter
static UCHAR TMCLCommandState;              //!< State of the interpreter
static TTMCLCommand ActualCommand;          //!< TMCL command to be executed (with all parameters)
static TTMCLReply ActualReply;              //!< Reply of last executed TMCL command
static UCHAR TMCLReplyFormat;               //!< format of next reply (RF_NORMAL or RF_SPECIAL)
static UCHAR SpecialReply[9];               //!< buffer for special replies
static UCHAR ResetRequested;                //!< TRUE after executing the software reset command

static UCHAR RelativePositioningOptionCode[N_O_MOTORS];  //!< Option code for MVP REL command

typedef int (*PConvertFunction)(int);                    //!< Function pointer type definition for velocity and acceleration conversion functions
static PConvertFunction VelocityToInternal[N_O_MOTORS];      //!< Pointer to velocity conversion function from PPS to TMC4361
static PConvertFunction VelocityToUser[N_O_MOTORS];          //!< Pointer to velocity conversion function from TMC4361 to PPS
static PConvertFunction AccelerationToInternal[N_O_MOTORS];  //!< Pointer to acceleration conversion function from PPS to TMC4361
static PConvertFunction AccelerationToUser[N_O_MOTORS];      //!< Pointer to acceleration conversion function from TMC4361 to PPS

//Prototypes
static void RotateRight(void);
static void RotateLeft(void);
static void MotorStop(void);
static void MoveToPosition(void);
static void SetAxisParameter(void);
static void GetAxisParameter(void);
static void StoreAxisParameter(void);
static void RestoreAxisParameter(void);
static void SetGlobalParameter(void);
static void GetGlobalParameter(void);
static void StoreGlobalParameter(void);
static void RestoreGlobalParameter(void);
static void SetInputOutput(void);
static void GetInputOutput(void);
static void GetVersion(void);
static void SoftwareReset(void);


//Imported variables
extern char VersionString[];   //!< Imported version string


//Functions

/***************************************************************//**
   \fn ExecuteActualCommand()
   \brief Execute actual TMCL command

   Execute the TMCL command that must have been written
   to the global variable "ActualCommand" before.
********************************************************************/
static void ExecuteActualCommand(void)
{
  //Prepare answer
  ActualReply.Opcode = ActualCommand.Opcode;
  ActualReply.Status = REPLY_OK;
  ActualReply.Value.Int32 = ActualCommand.Value.Int32;

  //Execute command
  switch (ActualCommand.Opcode)
  {
  case TMCL_ROR:
    RotateRight();
    break;

  case TMCL_ROL:
    RotateLeft();
    break;

  case TMCL_MST:
    MotorStop();
    break;

  case TMCL_MVP:
    MoveToPosition();
    break;

  case TMCL_SAP:
    SetAxisParameter();
    break;

  case TMCL_GAP:
    GetAxisParameter();
    break;

  case TMCL_STAP:
    StoreAxisParameter();
    break;

  case TMCL_RSAP:
    RestoreAxisParameter();
    break;

  case TMCL_SGP:
    SetGlobalParameter();
    break;

  case TMCL_GGP:
    GetGlobalParameter();
    break;

  case TMCL_STGP:
    StoreGlobalParameter();
    break;

  case TMCL_RSGP:
    RestoreGlobalParameter();
    break;

  case TMCL_SIO:
    SetInputOutput();
    break;

  case TMCL_GIO:
    GetInputOutput();
    break;

  case TMCL_GetVersion:
    GetVersion();
    break;

  case TMCL_SoftwareReset:
    SoftwareReset();
    break;

  default:
    ActualReply.Status = REPLY_INVALID_CMD;
    break;
  }
}


/***************************************************************//**
   \fn InitTMCL(void)
   \brief Initialize TMCL interpreter

   Intialise the TMCL interpreter. Must be called once at startup.
********************************************************************/
void InitTMCL(void)
{
  UINT i;

  TMCLCommandState = TCS_IDLE;
  for (i = 0; i < N_O_MOTORS; i++)
  {
    if (MotorConfig[i].UnitMode == UNIT_MODE_INTERNAL)
    {
      VelocityToInternal[i] = ConvertInternalToInternal;
      VelocityToUser[i] = ConvertInternalToInternal;
      AccelerationToInternal[i] = ConvertInternalToInternal;
      AccelerationToUser[i] = ConvertInternalToInternal;
    }
    else
    {
      VelocityToInternal[i] = ConvertVelocityUserToInternal;
      VelocityToUser[i] = ConvertVelocityInternalToUser;
      AccelerationToInternal[i] = ConvertAccelerationUserToInternal;
      AccelerationToUser[i] = ConvertAccelerationInternalToUser;
    }
    RelativePositioningOptionCode[i] = RMO_TARGET;
    VMax[i] = ReadTMC5072Int(MOTOR_TO_IC_SPI(i), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(i)));
    VMaxModified[i] = FALSE;
    AMax[i] = ReadTMC5072Int(MOTOR_TO_IC_SPI(i), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(i)));
    AMaxModified[i] = FALSE;
    CalculateEncoderParameters(i);
  }
}


/***************************************************************//**
   \fn ProcessCommand(void)
   \brief Fetch and execute TMCL commands

   This is the main function for fetching and executing TMCL commands
   and has to be called periodically from the main loop.
********************************************************************/
void ProcessCommand(void)
{
  UCHAR Byte;
  UCHAR Checksum;
  UCHAR i;

  //**Send answer for the last command**
  if (TMCLCommandState == TCS_UART)  //via UART
  {
    if (TMCLReplyFormat == RF_STANDARD)
    {
      Checksum = ModuleConfig.SerialHostAddress + ModuleConfig.SerialModuleAddress +
        ActualReply.Status + ActualReply.Opcode +
        ActualReply.Value.Byte[3] +
        ActualReply.Value.Byte[2] +
        ActualReply.Value.Byte[1] +
        ActualReply.Value.Byte[0];

      WriteUART(ModuleConfig.SerialHostAddress);
      WriteUART(ModuleConfig.SerialModuleAddress);
      WriteUART(ActualReply.Status);
      WriteUART(ActualReply.Opcode);
      WriteUART(ActualReply.Value.Byte[3]);
      WriteUART(ActualReply.Value.Byte[2]);
      WriteUART(ActualReply.Value.Byte[1]);
      WriteUART(ActualReply.Value.Byte[0]);
      WriteUART(Checksum);
    }
    else if (TMCLReplyFormat == RF_SPECIAL)
    {
      for (i = 0; i < 9; i++)
      {
        WriteUART(SpecialReply[i]);
      }
    }
  }
  else if (TMCLCommandState == TCS_UART_ERROR)  //check sum of the last command has been wrong
  {
    ActualReply.Opcode = 0;
    ActualReply.Status = REPLY_CHKERR;
    ActualReply.Value.Int32 = 0;

    Checksum = ModuleConfig.SerialHostAddress + ModuleConfig.SerialModuleAddress +
      ActualReply.Status + ActualReply.Opcode +
      ActualReply.Value.Byte[3] +
      ActualReply.Value.Byte[2] +
      ActualReply.Value.Byte[1] +
      ActualReply.Value.Byte[0];

    WriteUART(ModuleConfig.SerialHostAddress);
    WriteUART(ModuleConfig.SerialModuleAddress);
    WriteUART(ActualReply.Status);
    WriteUART(ActualReply.Opcode);
    WriteUART(ActualReply.Value.Byte[3]);
    WriteUART(ActualReply.Value.Byte[2]);
    WriteUART(ActualReply.Value.Byte[1]);
    WriteUART(ActualReply.Value.Byte[0]);
    WriteUART(Checksum);
  }


  //Reset state (answer has been sent now)
  TMCLCommandState = TCS_IDLE;
  TMCLReplyFormat = RF_STANDARD;

  //Generate a system reset if requested by the host
  if (ResetRequested)
  {
    ResetCPU(TRUE);
  }

  //**Try to get a new command**
  if (ReadUART(&Byte))  //Get data from UART
  {
    if (CheckUARTTimeout())
    {
      UARTCount = 0;  //discard everything when there has been a command timeout
    }
    UARTCmd[UARTCount++] = Byte;

    if (UARTCount == 9)  //Nine bytes have been received without timeout
    {
      UARTCount = 0;

      if (UARTCmd[0] == ModuleConfig.SerialModuleAddress)  //Is this our addresss?
      {
        Checksum = 0;
        for (i = 0; i < 8; i++) Checksum += UARTCmd[i];

        if (Checksum == UARTCmd[8])  //Is the checksum correct?
        {
          ActualCommand.Opcode = UARTCmd[1];
          ActualCommand.Type = UARTCmd[2];
          ActualCommand.Motor = UARTCmd[3];
          ActualCommand.Value.Byte[3] = UARTCmd[4];
          ActualCommand.Value.Byte[2] = UARTCmd[5];
          ActualCommand.Value.Byte[1] = UARTCmd[6];
          ActualCommand.Value.Byte[0] = UARTCmd[7];

          TMCLCommandState = TCS_UART;

          UARTCount = 0;
        }
        else TMCLCommandState = TCS_UART_ERROR;  //Checksum wrong
      }
    }
  }

  //**Execute the command**
  //Check if a command could be fetched and execute it.
  if (TMCLCommandState != TCS_IDLE && TMCLCommandState != TCS_UART_ERROR)
  {
    ExecuteActualCommand();
  }
}


//** TMCL Commands **

/***************************************************************//**
  \fn RotateRight(void)
  \brief Command ROR (see TMCL manual)

  ROR (ROtate Right) command (see TMCL manual).
********************************************************************/
static void RotateRight(void)
{
  if (VALID_MOTOR())
  {
    MotorFlags[ActualCommand.Motor] = 0;

    if (AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor] = FALSE;
    }
    VMaxModified[ActualCommand.Motor] = TRUE;
    WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32)));
    if (ActualCommand.Value.Int32 > 0)
    {
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_VELPOS);
    }
    else
    {
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_VELNEG);
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn RotateLeft(void)
  \brief Command ROL

  ROL (ROtate Left) command (see TMCL manual).
********************************************************************/
static void RotateLeft(void)
{
  if (VALID_MOTOR())
  {
    MotorFlags[ActualCommand.Motor] = 0;

    if (AMaxModified[ActualCommand.Motor])
    {
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), AMax[ActualCommand.Motor]);
      AMaxModified[ActualCommand.Motor] = FALSE;
    }
    VMaxModified[ActualCommand.Motor] = TRUE;
    WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32)));
    if (ActualCommand.Value.Int32 > 0)
    {
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_VELNEG);
    }
    else
    {
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_VELPOS);
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MotorStop(void)
  \brief Command MST

  MST (Motor StoP) command (see TMCL manual).
********************************************************************/
static void MotorStop(void)
{
  if (VALID_MOTOR())
  {
    SoftStop(ActualCommand.Motor);
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn MoveToPosition(void)
  \brief Command MVP

  MVP (Move To Position) command (see TMCL manual).
********************************************************************/
static void MoveToPosition(void)
{
  int NewPosition;

  if (VALID_MOTOR())
  {
    switch (ActualCommand.Type)
    {
    case MVP_ABS:
      MotorFlags[ActualCommand.Motor] = 0;

      if (VMaxModified[ActualCommand.Motor])
      {
        WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), VMax[ActualCommand.Motor]);
        VMaxModified[ActualCommand.Motor] = FALSE;
      }
      if (AMaxModified[ActualCommand.Motor])
      {
        WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), AMax[ActualCommand.Motor]);
        AMaxModified[ActualCommand.Motor] = FALSE;
      }
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_XTARGET(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), ActualCommand.Value.Int32);
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_POSITION);
      break;

    case MVP_REL:
      MotorFlags[ActualCommand.Motor] = 0;

      switch (RelativePositioningOptionCode[ActualCommand.Motor])
      {
      case RMO_TARGET:
        NewPosition = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_XTARGET(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        break;

      case RMO_ACTINT:
        NewPosition = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        break;

      case RMO_ACTENC:
        NewPosition = GetEncoderPosition(ActualCommand.Motor);
        break;

      default:
        NewPosition = 0;
        break;
      }

      if (VMaxModified[ActualCommand.Motor])
      {
        WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), VMax[ActualCommand.Motor]);
        VMaxModified[ActualCommand.Motor] = FALSE;
      }
      if (AMaxModified[ActualCommand.Motor])
      {
        WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), AMax[ActualCommand.Motor]);
        AMaxModified[ActualCommand.Motor] = FALSE;
      }
      NewPosition += ActualCommand.Value.Int32;
      WriteTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_XTARGET(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), NewPosition);
      WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, 0, TMC5072_MODE_POSITION);
      ActualReply.Value.Int32 = NewPosition;
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
      break;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}

/***************************************************************//**
   \fn SetAxisParameter(void)
   \brief Command SAP

  SAP (Set Axis Parameter) command (see TMCL manual).
********************************************************************/
static void SetAxisParameter(void)
{
  int Value;

  if (VALID_MOTOR())
  {
    switch (ActualCommand.Type)
    {
    case TMCL_AP_TargetPosition:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_XTARGET, TMC5072_XTARGET_MASK, TMC5072_XTARGET_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ActualPosition:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_XACTUAL, TMC5072_XACTUAL_MASK, TMC5072_XACTUAL_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_TargetSpeed:
      if (ActualCommand.Value.Int32 > 0)
      {
        READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, TMC5072_MODE_VELPOS);
      }
      else
      {
        READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT, TMC5072_MODE_VELNEG);
      }
      VMaxModified[ActualCommand.Motor] = TRUE;
      ActualCommand.Value.Int32 = VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32));
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VMAX, TMC5072_VMAX_MASK, TMC5072_VMAX_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_MaximumPositioningSpeed:
      VMax[ActualCommand.Motor] = VelocityToInternal[ActualCommand.Motor](abs(ActualCommand.Value.Int32));
      if (READ_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT) == TMC5072_MODE_POSITION)
      {
        READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VMAX, TMC5072_VMAX_MASK, TMC5072_VMAX_SHIFT, VMax[ActualCommand.Motor]);
      }
      break;

    case TMCL_AP_MaximumAcceleration:
      AMaxModified[ActualCommand.Motor] = FALSE;
      AMax[ActualCommand.Motor] = AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_AMAX, TMC5072_AMAX_MASK, TMC5072_AMAX_SHIFT, AMax[ActualCommand.Motor]);
      break;

    case TMCL_AP_MaximumCurrent:
      ActualCommand.Value.Int32 /= 8;
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IRUN_MASK, TMC5072_IRUN_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_StandbyCurrent:
      ActualCommand.Value.Int32 /= 8;
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IHOLD_MASK, TMC5072_IHOLD_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_RightLimitSwitchDisable:
      ActualCommand.Value.Int32 = 1 - ActualCommand.Value.Int32;
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_STOP_R_ENABLE_MASK, TMC5072_STOP_R_ENABLE_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_LeftLimitSwitchDisable:
      ActualCommand.Value.Int32 = 1 - ActualCommand.Value.Int32;
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_STOP_L_ENABLE_MASK, TMC5072_STOP_L_ENABLE_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SwapLimitSwitches:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_SWAP_LR_MASK, TMC5072_SWAP_LR_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_Acceleration:
      ActualCommand.Value.Int32 = AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_A1, TMC5072_A1_MASK, TMC5072_A1_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_Velocity:
      ActualCommand.Value.Int32 = VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_V1, TMC5072_V1_MASK, TMC5072_V1_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_MaximumDeceleration:
      ActualCommand.Value.Int32 = AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_DMAX, TMC5072_DMAX_MASK, TMC5072_DMAX_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_Deceleration:
      ActualCommand.Value.Int32 = AccelerationToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_D1, TMC5072_D1_MASK, TMC5072_D1_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_VStart:
      ActualCommand.Value.Int32 = VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VSTART, TMC5072_VSTART_MASK, TMC5072_VSTART_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_VStop:
      ActualCommand.Value.Int32 = VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VSTOP, TMC5072_VSTOP_MASK, TMC5072_VSTOP_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_RampWaitTime:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_TZEROWAIT, TMC5072_TZEROWAIT_MASK, TMC5072_TZEROWAIT_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SpeedThresholdForCoolStep:
      if (ActualCommand.Value.Int32 < 0)
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
        break;
      }
      if (MotorConfig[ActualCommand.Motor].UnitMode == UNIT_MODE_INTERNAL)
      {
        ActualCommand.Value.Int32 = ConvertVelocityInternalToUser(ActualCommand.Value.Int32);
      }
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VHIGH, TMC5072_VHIGH_MASK, TMC5072_VHIGH_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_MinimumSpeedForDcStep:
      ActualCommand.Value.Int32 = VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VDCMIN, TMC5072_VDCMIN_MASK, TMC5072_VDCMIN_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_RightLimitSwitchPolarity:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_POL_STOP_R_MASK, TMC5072_POL_STOP_R_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_LeftLimitSwitchPolarity:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_POL_STOP_L_MASK, TMC5072_POL_STOP_L_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SoftStopEnable:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_EN_SOFTSTOP_MASK, TMC5072_EN_SOFTSTOP_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_HighSpeedChopperMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VHIGHCHM_MASK, TMC5072_VHIGHCHM_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_HighSpeedFullstepMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VHIGHFS_MASK, TMC5072_VHIGHFS_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_PowerDownRamp:
      ActualCommand.Value.Int32 /= 8;
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IHOLDDELAY_MASK, TMC5072_IHOLDDELAY_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_DcStepTime:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_DCCTRL, TMC5072_DC_TIME_MASK, TMC5072_DC_TIME_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_DcStepStallGuard:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_DCCTRL, TMC5072_DC_SG_MASK, TMC5072_DC_SG_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_EEPROMMagic:
      MotorConfig[ActualCommand.Motor].EEPROMMagic = ActualReply.Value.Int32 & 0xff;
      break;

    case TMCL_AP_RelativePositioningOption:
      if (ActualCommand.Value.Int32 != RMO_TARGET && ActualCommand.Value.Int32 != RMO_ACTINT && ActualCommand.Value.Int32 != RMO_ACTENC)
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
        break;
      }
      RelativePositioningOptionCode[ActualCommand.Motor] = ActualCommand.Value.Int32;
      break;

    case TMCL_AP_MicrostepResolution:
      ActualCommand.Value.Int32 = (8 - ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_MRES_MASK, TMC5072_MRES_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ChopperBlankTime:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_TBL_MASK, TMC5072_TBL_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ConstantTOffMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_CHM_MASK, TMC5072_CHM_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_DisableFastDecayComparator:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_DISFDCC_MASK, TMC5072_DISFDCC_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ChopperHysteresisEnd:
      Value = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
      if ((Value & TMC5072_CHM_MASK) == 0)
      {
        WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_HEND_MASK, TMC5072_HEND_SHIFT, Value, ActualCommand.Value.Int32);
      }
      else
      {
        WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_TFD_3_MASK | TMC5072_HEND_MASK, TMC5072_HEND_SHIFT, Value, (ActualCommand.Value.Int32 & ~BIT3) | ((ActualCommand.Value.Int32 & BIT3) << (TMC5072_TFD_3_SHIFT - TMC5072_HSTRT_SHIFT - 3)));
      }
      break;

    case TMCL_AP_ChopperHysteresisStart:
      Value = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
      if ((Value & TMC5072_CHM_MASK) == 0)
      {
        WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_HSTRT_MASK, TMC5072_HSTRT_SHIFT, Value, ActualCommand.Value.Int32);
      }
      else
      {
        WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_HEND_MASK, TMC5072_HEND_SHIFT, Value, ActualCommand.Value.Int32);
      }
      break;

    case TMCL_AP_ChopperOffTime:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_TOFF_MASK, TMC5072_TOFF_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyCurrentMinimum:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEIMIN_MASK, TMC5072_SEIMIN_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyCurrentDownStep:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEDN_MASK, TMC5072_SEDN_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyHysteresis:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEMAX_MASK, TMC5072_SEMAX_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyCurrentUpStep:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEUP_MASK, TMC5072_SEUP_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyHysteresisStart:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEMIN_MASK, TMC5072_SEMIN_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_StallGuard2FilterEnable:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SFILT_MASK, TMC5072_SFILT_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_StallGuard2Threshold:
      if (ActualReply.Value.Int32 & BIT31)
      {
        ActualReply.Value.Int32 &= ~(TMC5072_SGT_MASK >> TMC5072_SGT_SHIFT); // Convert 32 bits signed number to 7 bits signed number
      }
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SGT_MASK, TMC5072_SGT_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ShortToGndDisable:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_DISS2G_MASK, TMC5072_DISS2G_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_VSense:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VSENSE_MASK, TMC5072_VSENSE_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyStallVelocity:
      if (ActualCommand.Value.Int32 < 0)
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
        break;
      }
      MotorConfig[ActualCommand.Motor].StallVMin = VelocityToInternal[ActualCommand.Motor](ActualCommand.Value.Int32);
      break;

    case TMCL_AP_SmartEnergyThresholdSpeed:
      if (ActualCommand.Value.Int32 < 0)
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
        break;
      }
      if (MotorConfig[ActualCommand.Motor].UnitMode == UNIT_MODE_INTERNAL) ActualCommand.Value.Int32 = ConvertVelocityInternalToUser(ActualCommand.Value.Int32);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_VCOOLTHRS, TMC5072_VCOOLTHRS_MASK, TMC5072_VCOOLTHRS_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_RandomTOffMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_RNDTF_MASK, TMC5072_RNDTF_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_PWMGradient:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_GRAD_MASK, TMC5072_PWM_GRAD_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_PWMAmplitude:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_AMPL_MASK, TMC5072_PWM_AMPL_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_PWMFrequency:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_FREQ_MASK, TMC5072_PWM_FREQ_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_PWMAutomaticScale:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_AUTOSCALE_MASK, TMC5072_PWM_AUTOSCALE_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_EncoderMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_ENCMODE, 0xFFF, 0, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_MotorFullStepResolution:
      MotorConfig[ActualCommand.Motor].MotorFullStepResolution = ActualCommand.Value.Int32;
      CalculateEncoderParameters(ActualCommand.Motor);
      break;

    case TMCL_AP_FreewheelingMode:
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_FREEWHEEL_MASK, TMC5072_FREEWHEEL_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_EncoderPosition:
      SetEncoderPosition(ActualCommand.Motor, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_EncoderResolution:
      MotorConfig[ActualCommand.Motor].EncoderResolution = ActualCommand.Value.Int32;
      CalculateEncoderParameters(ActualCommand.Motor);
      break;

    case TMCL_AP_MaximumEncoderDeviation:
      MotorConfig[ActualCommand.Motor].MaxDeviation = ActualCommand.Value.Int32;
      break;

    case TMCL_AP_PowerDownDelay:
      ActualCommand.Value.Int32 = (int)floor((double)ActualCommand.Value.Int32 / TMC5072_TPOWERDOWN_FACTOR);
      READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_TZEROWAIT, TMC5072_TZEROWAIT_MASK, TMC5072_TZEROWAIT_SHIFT, ActualCommand.Value.Int32);
      break;

    case TMCL_AP_ReverseShaft:
      if (MOTOR_TO_IC_MOTOR(ActualCommand.Motor) == 0)
      {
        READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_GCONF, TMC5072_SHAFT1_MASK, TMC5072_SHAFT1_SHIFT, ActualCommand.Value.Int32);
      }
      else
      {
        READ_WRITE_PARAMETER(ActualCommand.Motor, TMC5072_GCONF, TMC5072_SHAFT2_MASK, TMC5072_SHAFT2_SHIFT, ActualCommand.Value.Int32);
      }
      break;

    case TMCL_AP_UnitMode:
      switch (ActualCommand.Value.Int32)
      {
      case UNIT_MODE_INTERNAL:
        MotorConfig[ActualCommand.Motor].UnitMode = 0;
        VelocityToInternal[ActualCommand.Motor] = ConvertInternalToInternal;
        VelocityToUser[ActualCommand.Motor] = ConvertInternalToInternal;
        AccelerationToInternal[ActualCommand.Motor] = ConvertInternalToInternal;
        AccelerationToUser[ActualCommand.Motor] = ConvertInternalToInternal;
        break;

      case UNIT_MODE_PPS:
        MotorConfig[ActualCommand.Motor].UnitMode = 1;
        VelocityToInternal[ActualCommand.Motor] = ConvertVelocityUserToInternal;
        VelocityToUser[ActualCommand.Motor] = ConvertVelocityInternalToUser;
        AccelerationToInternal[ActualCommand.Motor] = ConvertAccelerationUserToInternal;
        AccelerationToUser[ActualCommand.Motor] = ConvertAccelerationInternalToUser;
        break;

      default:
        ActualReply.Status = REPLY_INVALID_VALUE;
        break;
      }
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
      break;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}

/***************************************************************//**
  \fn GetAxisParameter(void)
  \brief Command GAP

  GAP (Get Axis Parameter) command (see TMCL manual).
********************************************************************/
static void GetAxisParameter(void)
{
  if (VALID_MOTOR())
  {
    ActualReply.Value.Int32 = 0;

    switch (ActualCommand.Type)
    {
    case TMCL_AP_TargetPosition:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_XTARGET, TMC5072_XTARGET_MASK, TMC5072_XTARGET_SHIFT);
      break;

    case TMCL_AP_ActualPosition:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_XACTUAL, TMC5072_XACTUAL_MASK, TMC5072_XACTUAL_SHIFT);
      break;

    case TMCL_AP_TargetSpeed:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VMAX, TMC5072_VMAX_MASK, TMC5072_VMAX_SHIFT);
      if (READ_PARAMETER(ActualCommand.Motor, TMC5072_RAMPMODE, TMC5072_RAMPMODE_MASK, TMC5072_RAMPMODE_SHIFT) == TMC5072_MODE_VELNEG)
      {
        ActualReply.Value.Int32 = -ActualReply.Value.Int32;
      }
      break;

    case TMCL_AP_ActualSpeed:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VACTUAL, TMC5072_VACTUAL_MASK, TMC5072_VACTUAL_SHIFT);
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_MaximumPositioningSpeed:
      ActualReply.Value.Int32 = VMax[ActualCommand.Motor];
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_MaximumAcceleration:
      ActualReply.Value.Int32 = AMax[ActualCommand.Motor];
      ActualReply.Value.Int32 = AccelerationToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_MaximumCurrent:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IRUN_MASK, TMC5072_IRUN_SHIFT);
      ActualReply.Value.Int32 *= 8;
      break;

    case TMCL_AP_StandbyCurrent:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IHOLD_MASK, TMC5072_IHOLD_SHIFT);
      ActualReply.Value.Int32 *= 8;
      break;

    case TMCL_AP_PositionReachedFlag:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_RAMPSTAT, TMC5072_POSITION_REACHED_MASK, TMC5072_POSITION_REACHED_SHIFT);
      break;

    case TMCL_AP_RightLimitSwitchState:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_RAMPSTAT, TMC5072_STATUS_STOP_R_MASK, TMC5072_STATUS_STOP_R_SHIFT);
      break;

    case TMCL_AP_LeftLimitSwitchState:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_RAMPSTAT, TMC5072_STATUS_STOP_L_MASK, TMC5072_STATUS_STOP_L_SHIFT);
      break;

    case TMCL_AP_RightLimitSwitchDisable:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_STOP_R_ENABLE_MASK, TMC5072_STOP_R_ENABLE_SHIFT);
      ActualReply.Value.Int32 = 1 - ActualReply.Value.Int32;
      break;

    case TMCL_AP_LeftLimitSwitchDisable:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_STOP_L_ENABLE_MASK, TMC5072_STOP_L_ENABLE_SHIFT);
      ActualReply.Value.Int32 = 1 - ActualReply.Value.Int32;
      break;

    case TMCL_AP_SwapLimitSwitches:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_SWAP_LR_MASK, TMC5072_SWAP_LR_SHIFT);
      break;

    case TMCL_AP_Acceleration:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_A1, TMC5072_A1_MASK, TMC5072_A1_SHIFT);
      ActualReply.Value.Int32 = AccelerationToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_Velocity:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_V1, TMC5072_V1_MASK, TMC5072_V1_SHIFT);
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_MaximumDeceleration:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DMAX, TMC5072_DMAX_MASK, TMC5072_DMAX_SHIFT);
      ActualReply.Value.Int32 = AccelerationToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_Deceleration:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_D1, TMC5072_D1_MASK, TMC5072_D1_SHIFT);
      ActualReply.Value.Int32 = AccelerationToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_VStart:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VSTART, TMC5072_VSTART_MASK, TMC5072_VSTART_SHIFT);
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_VStop:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VSTOP, TMC5072_VSTOP_MASK, TMC5072_VSTOP_SHIFT);
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_RampWaitTime:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_TZEROWAIT, TMC5072_TZEROWAIT_MASK, TMC5072_TZEROWAIT_SHIFT);
      break;

    case TMCL_AP_SpeedThresholdForCoolStep:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VHIGH, TMC5072_VHIGH_MASK, TMC5072_VHIGH_SHIFT);
      if (MotorConfig[ActualCommand.Motor].UnitMode == UNIT_MODE_INTERNAL)
      {
        ActualReply.Value.Int32 = ConvertVelocityUserToInternal(ActualReply.Value.Int32);
      }
      break;

    case TMCL_AP_MinimumSpeedForDcStep:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VDCMIN, TMC5072_VDCMIN_MASK, TMC5072_VDCMIN_SHIFT);
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](ActualReply.Value.Int32);
      break;

    case TMCL_AP_RightLimitSwitchPolarity:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_POL_STOP_R_MASK, TMC5072_POL_STOP_R_SHIFT);
      break;

    case TMCL_AP_LeftLimitSwitchPolarity:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_POL_STOP_L_MASK, TMC5072_POL_STOP_L_SHIFT);
      break;

    case TMCL_AP_SoftStopEnable:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_SWMODE, TMC5072_EN_SOFTSTOP_MASK, TMC5072_EN_SOFTSTOP_SHIFT);
      break;

    case TMCL_AP_HighSpeedChopperMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VHIGHCHM_MASK, TMC5072_VHIGHCHM_SHIFT);
      break;

    case TMCL_AP_HighSpeedFullstepMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VHIGHFS_MASK, TMC5072_VHIGHFS_SHIFT);
      break;

    case TMCL_AP_MeasuredSpeed:
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](GetMeasuredSpeed(ActualCommand.Motor));
      break;

    case TMCL_AP_PowerDownRamp:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_IHOLD_IRUN, TMC5072_IHOLDDELAY_MASK, TMC5072_IHOLDDELAY_SHIFT);
      ActualReply.Value.Int32 *= 8;
      break;

    case TMCL_AP_DcStepTime:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DCCTRL, TMC5072_DC_TIME_MASK, TMC5072_DC_TIME_SHIFT);
      break;

    case TMCL_AP_DcStepStallGuard:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DCCTRL, TMC5072_DC_SG_MASK, TMC5072_DC_SG_SHIFT);
      break;

    case TMCL_AP_EEPROMMagic:
      ActualReply.Value.Int32 = MotorConfig[ActualCommand.Motor].EEPROMMagic & 0xff;
      break;

    case TMCL_AP_RelativePositioningOption:
      ActualReply.Value.Int32 = RelativePositioningOptionCode[ActualCommand.Motor];
      break;

    case TMCL_AP_MicrostepResolution:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_MRES_MASK, TMC5072_MRES_SHIFT);
      ActualReply.Value.Int32 = 8 - ActualReply.Value.Int32;
      break;

      //TMC5072 specific parameters
    case TMCL_AP_ChopperBlankTime:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_TBL_MASK, TMC5072_TBL_SHIFT);
      break;

    case TMCL_AP_ConstantTOffMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_CHM_MASK, TMC5072_CHM_SHIFT);
      break;

    case TMCL_AP_DisableFastDecayComparator:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_DISFDCC_MASK, TMC5072_DISFDCC_SHIFT);
      break;

    case TMCL_AP_ChopperHysteresisEnd:
      ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
      if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
      {
        ActualReply.Value.Int32 = REGISTER_TO_VALUE(ActualReply.Value.Int32, TMC5072_HEND_MASK, TMC5072_HEND_SHIFT);
      }
      else
      {
        ActualReply.Value.Int32 = ((ActualReply.Value.Int32 & TMC5072_HSTRT_MASK) >> TMC5072_HSTRT_SHIFT) | ((ActualReply.Value.Int32 & TMC5072_TFD_3_MASK) >> (TMC5072_TFD_3_SHIFT - 3));
      }
      break;

    case TMCL_AP_ChopperHysteresisStart:
      ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
      if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
      {
        ActualReply.Value.Int32 = REGISTER_TO_VALUE(ActualReply.Value.Int32, TMC5072_HSTRT_MASK, TMC5072_HSTRT_SHIFT);
      }
      else
      {
        ActualReply.Value.Int32 = REGISTER_TO_VALUE(ActualReply.Value.Int32, TMC5072_HEND_MASK, TMC5072_HEND_SHIFT);

      }
      break;

    case TMCL_AP_ChopperOffTime:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_TOFF_MASK, TMC5072_TOFF_SHIFT);
      break;

    case TMCL_AP_SmartEnergyCurrentMinimum:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEIMIN_MASK, TMC5072_SEIMIN_SHIFT);
      break;

    case TMCL_AP_SmartEnergyCurrentDownStep:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEDN_MASK, TMC5072_SEDN_SHIFT);
      break;

    case TMCL_AP_SmartEnergyHysteresis:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEMAX_MASK, TMC5072_SEMAX_SHIFT);
      break;

    case TMCL_AP_SmartEnergyCurrentUpStep:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEUP_MASK, TMC5072_SEUP_SHIFT);
      break;

    case TMCL_AP_SmartEnergyHysteresisStart:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SEMIN_MASK, TMC5072_SEMIN_SHIFT);
      break;

    case TMCL_AP_StallGuard2FilterEnable:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SFILT_MASK, TMC5072_SFILT_SHIFT);
      break;

    case TMCL_AP_StallGuard2Threshold:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_COOLCONF, TMC5072_SGT_MASK, TMC5072_SGT_SHIFT);
      if (ActualReply.Value.Int32 & BIT6)
      {
        ActualReply.Value.Int32 |= ~(TMC5072_SGT_MASK >> TMC5072_SGT_SHIFT); // Convert 7 bits signed number to 32 bits signed number
      }
      break;

    case TMCL_AP_ShortToGndDisable:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_DISS2G_MASK, TMC5072_DISS2G_SHIFT);
      break;

    case TMCL_AP_VSense:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_VSENSE_MASK, TMC5072_VSENSE_SHIFT);
      break;

    case TMCL_AP_SmartEnergyActualCurrent:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DRVSTATUS, TMC5072_CS_ACTUAL_MASK, TMC5072_CS_ACTUAL_SHIFT);
      break;

    case TMCL_AP_SmartEnergyStallVelocity:
      ActualReply.Value.Int32 = VelocityToUser[ActualCommand.Motor](MotorConfig[ActualCommand.Motor].StallVMin);
      break;

    case TMCL_AP_SmartEnergyThresholdSpeed:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_VCOOLTHRS, TMC5072_VCOOLTHRS_MASK, TMC5072_VCOOLTHRS_SHIFT);
      if (MotorConfig[ActualCommand.Motor].UnitMode == UNIT_MODE_INTERNAL)
      {
        ActualReply.Value.Int32 = ConvertVelocityUserToInternal(ActualReply.Value.Int32);
      }
      break;

    case TMCL_AP_RandomTOffMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_CHOPCONF, TMC5072_RNDTF_MASK, TMC5072_RNDTF_SHIFT);
      break;

    case TMCL_AP_PWMGradient:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_GRAD_MASK, TMC5072_PWM_GRAD_SHIFT);
      break;

    case TMCL_AP_PWMAmplitude:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_AMPL_MASK, TMC5072_PWM_AMPL_SHIFT);
      break;

    case TMCL_AP_PWMFrequency:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_FREQ_MASK, TMC5072_PWM_FREQ_SHIFT);
      break;

    case TMCL_AP_PWMAutomaticScale:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_PWM_AUTOSCALE_MASK, TMC5072_PWM_AUTOSCALE_SHIFT);
      break;

    case TMCL_AP_EncoderMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_ENCMODE, 0xFFF, 0);
      break;

    case TMCL_AP_MotorFullStepResolution:
      ActualReply.Value.Int32 = MotorConfig[ActualCommand.Motor].MotorFullStepResolution;
      break;

    case TMCL_AP_FreewheelingMode:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_PWMCONF, TMC5072_FREEWHEEL_MASK, TMC5072_FREEWHEEL_SHIFT);
      break;

    case TMCL_AP_ActualLoadValue:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DRVSTATUS, TMC5072_SG_RESULT_MASK, TMC5072_SG_RESULT_SHIFT);
      break;

    case TMCL_AP_ExtendedErrorFlags:
      ActualReply.Value.Int32 = MotorFlags[ActualCommand.Motor];
      MotorFlags[ActualCommand.Motor] = 0;
      break;

    case TMCL_AP_MotorDriverErrorFlags:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_DRVSTATUS, TMC5072_STALLGUARD_MASK | TMC5072_OT_MASK | TMC5072_OTPW_MASK | TMC5072_S2GA_MASK | TMC5072_S2GB_MASK | TMC5072_OLA_MASK | TMC5072_OLB_MASK | TMC5072_STST_MASK, TMC5072_STALLGUARD_SHIFT);
      break;

    case TMCL_AP_EncoderPosition:
      ActualReply.Value.Int32 = GetEncoderPosition(ActualCommand.Motor);
      break;

    case TMCL_AP_EncoderResolution:
      ActualReply.Value.Int32 = MotorConfig[ActualCommand.Motor].EncoderResolution;
      break;

    case TMCL_AP_MaximumEncoderDeviation:
      ActualReply.Value.Int32 = MotorConfig[ActualCommand.Motor].MaxDeviation;
      break;

    case TMCL_AP_PowerDownDelay:
      ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_TZEROWAIT, TMC5072_TZEROWAIT_MASK, TMC5072_TZEROWAIT_SHIFT);
      ActualReply.Value.Int32 = (int)ceil((double)ActualReply.Value.Int32 * TMC5072_TPOWERDOWN_FACTOR);
      break;

    case TMCL_AP_ReverseShaft:
      if (MOTOR_TO_IC_MOTOR(ActualCommand.Motor) == 0)
      {
        ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_GCONF, TMC5072_SHAFT1_MASK, TMC5072_SHAFT1_SHIFT);
      }
      else
      {
        ActualReply.Value.Int32 = READ_PARAMETER(ActualCommand.Motor, TMC5072_GCONF, TMC5072_SHAFT2_MASK, TMC5072_SHAFT2_SHIFT);
      }
      break;

    case TMCL_AP_UnitMode:
      ActualReply.Value.Int32 = MotorConfig[ActualCommand.Motor].UnitMode;
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
      break;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}

static UCHAR SaveMotorRegister(TMotorEeprom* Eeprom, UCHAR Motor, UCHAR Register, UINT Mask)
{
  UCHAR index = TMC5072ReverseMotorRegisterMap[MOTOR_TO_IC_MOTOR(ActualCommand.Motor)][Register];
  if (index == TMC5072_INVALID_REGISTER)
  {
    return REPLY_WRONG_TYPE;
  }
  int current = ReadTMC5072Int(MOTOR_TO_IC_SPI(Motor), Register);
  Eeprom->registers[index] = (Eeprom->registers[index] & ~Mask) | (current & Mask);
  return REPLY_OK;
}


/***************************************************************//**
  \fn StoreAxisParameter(void)
  \brief Command STAP

  STAP (Store Axis Parameter) command (see TMCL manual).
********************************************************************/
static void StoreAxisParameter(void)
{
  if (VALID_MOTOR())
  {
    TMotorEeprom* eeprom = GetMotorEeprom(ActualCommand.Motor, NULL);
    if (eeprom != NULL)
    {
      ActualReply.Value.Int32 = 0;

      switch (ActualCommand.Type)
      {
      case TMCL_AP_ActualPosition:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_XACTUAL_MASK);
        break;

      case TMCL_AP_MaximumPositioningSpeed:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VMAX_MASK);
        break;

      case TMCL_AP_MaximumAcceleration:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_AMAX_MASK);
        break;

      case TMCL_AP_MaximumCurrent:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IRUN_MASK);
        break;

      case TMCL_AP_StandbyCurrent:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IHOLD_MASK);
        break;

      case TMCL_AP_RightLimitSwitchDisable:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_STOP_R_ENABLE_MASK);
        break;

      case TMCL_AP_LeftLimitSwitchDisable:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_STOP_L_ENABLE_MASK);
        break;

      case TMCL_AP_SwapLimitSwitches:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SWAP_LR_MASK);
        break;

      case TMCL_AP_Acceleration:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_A1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_A1_MASK);
        break;

      case TMCL_AP_Velocity:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_V1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_V1_MASK);
        break;

      case TMCL_AP_MaximumDeceleration:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DMAX_MASK);
        break;

      case TMCL_AP_Deceleration:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_D1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_D1_MASK);
        break;

      case TMCL_AP_VStart:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VSTART(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSTART_MASK);
        break;

      case TMCL_AP_VStop:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VSTOP(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSTOP_MASK);
        break;

      case TMCL_AP_RampWaitTime:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TZEROWAIT_MASK);
        break;

      case TMCL_AP_SpeedThresholdForCoolStep:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VHIGH(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGH_MASK);
        break;

      case TMCL_AP_MinimumSpeedForDcStep:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VDCMIN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VDCMIN_MASK);
        break;

      case TMCL_AP_RightLimitSwitchPolarity:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_POL_STOP_R_MASK);
        break;

      case TMCL_AP_LeftLimitSwitchPolarity:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_POL_STOP_L_MASK);
        break;

      case TMCL_AP_SoftStopEnable:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_EN_SOFTSTOP_MASK);
        break;

      case TMCL_AP_HighSpeedChopperMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGHCHM_MASK);
        break;

      case TMCL_AP_HighSpeedFullstepMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGHFS_MASK);
        break;

      case TMCL_AP_PowerDownRamp:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IHOLDDELAY_MASK);
        break;

      case TMCL_AP_DcStepTime:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DC_TIME_MASK);
        break;

      case TMCL_AP_DcStepStallGuard:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DC_SG_MASK);
        break;

      case TMCL_AP_EEPROMMagic:
        eeprom->config.EEPROMMagic = MotorConfig[ActualCommand.Motor].EEPROMMagic;
        break;

      case TMCL_AP_MicrostepResolution:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_MRES_MASK);
        break;

        //TMC5072 specific parameters
      case TMCL_AP_ChopperBlankTime:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TBL_MASK);
        break;

      case TMCL_AP_ConstantTOffMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_CHM_MASK);
        break;

      case TMCL_AP_DisableFastDecayComparator:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DISFDCC_MASK);
        break;

      case TMCL_AP_ChopperHysteresisEnd:
        ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HEND_MASK);
        }
        else
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TFD_3_MASK | TMC5072_HSTRT_MASK);
        }
        break;

      case TMCL_AP_ChopperHysteresisStart:
        ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HSTRT_MASK);
        }
        else
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HEND_MASK);
        }
        break;

      case TMCL_AP_ChopperOffTime:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TOFF_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentMinimum:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEIMIN_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentDownStep:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEDN_MASK);
        break;

      case TMCL_AP_SmartEnergyHysteresis:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEMAX_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentUpStep:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEUP_MASK);
        break;

      case TMCL_AP_SmartEnergyHysteresisStart:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEMIN_MASK);
        break;

      case TMCL_AP_StallGuard2FilterEnable:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SFILT_MASK);
        break;

      case TMCL_AP_StallGuard2Threshold:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SGT_MASK);
        break;

      case TMCL_AP_ShortToGndDisable:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DISS2G_MASK);
        break;

      case TMCL_AP_VSense:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSENSE_MASK);
        break;

      case TMCL_AP_SmartEnergyStallVelocity:
        eeprom->config.StallVMin = MotorConfig[ActualCommand.Motor].StallVMin;
        break;

      case TMCL_AP_SmartEnergyThresholdSpeed:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VCOOLTHRS(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VCOOLTHRS_MASK);
        break;

      case TMCL_AP_RandomTOffMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_RNDTF_MASK);
        break;

      case TMCL_AP_PWMGradient:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_GRAD_MASK);
        break;

      case TMCL_AP_PWMAmplitude:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_AMPL_MASK);
        break;

      case TMCL_AP_PWMFrequency:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_FREQ_MASK);
        break;

      case TMCL_AP_PWMAutomaticScale:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_AUTOSCALE_MASK);
        break;

      case TMCL_AP_EncoderMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), 0xFFF);
        break;

      case TMCL_AP_MotorFullStepResolution:
        eeprom->config.MotorFullStepResolution = MotorConfig[ActualCommand.Motor].MotorFullStepResolution;
        break;

      case TMCL_AP_FreewheelingMode:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_FREEWHEEL_MASK);
        break;

      case TMCL_AP_EncoderResolution:
        eeprom->config.EncoderResolution = MotorConfig[ActualCommand.Motor].EncoderResolution;
        break;

      case TMCL_AP_MaximumEncoderDeviation:
        eeprom->config.MaxDeviation = MotorConfig[ActualCommand.Motor].MaxDeviation;
        break;

      case TMCL_AP_PowerDownDelay:
        ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TZEROWAIT_MASK);
        break;

      case TMCL_AP_ReverseShaft:
        if (MOTOR_TO_IC_MOTOR(ActualCommand.Motor) == 0)
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_GCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SHAFT1_MASK);
        }
        else
        {
          ActualReply.Status = SaveMotorRegister(eeprom, ActualCommand.Motor, TMC5072_GCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SHAFT2_MASK);
        }
        break;

      case TMCL_AP_UnitMode:
        eeprom->config.UnitMode = MotorConfig[ActualCommand.Motor].UnitMode;
        break;

      default:
        ActualReply.Status = REPLY_WRONG_TYPE;
        break;
      }
    }
    else ActualReply.Status = REPLY_EEPROM_LOCKED;
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


static UCHAR LoadMotorRegister(TMotorEeprom* Eeprom, UCHAR Motor, UCHAR Register, UINT Mask)
{
  UCHAR index = TMC5072ReverseMotorRegisterMap[MOTOR_TO_IC_MOTOR(ActualCommand.Motor)][Register];
  if (index == TMC5072_INVALID_REGISTER)
  {
    return REPLY_WRONG_TYPE;
  }
  WriteTMC5072Int(MOTOR_TO_IC_SPI(Motor), Register, (ReadTMC5072Int(MOTOR_TO_IC_SPI(Motor), Register) & ~Mask) | (Eeprom->registers[index] & Mask));
  return REPLY_OK;
}


/***************************************************************//**
  \fn RestoreAxisParameter(void)
  \brief Command RSAP

  RSAP (Restore Axis Parameter) command (see TMCL manual).
********************************************************************/
static void RestoreAxisParameter(void)
{
  if (VALID_MOTOR())
  {
    TMotorEeprom* eeprom = GetMotorEeprom(ActualCommand.Motor, NULL);
    if (eeprom != NULL)
    {
      ActualReply.Value.Int32 = 0;

      switch (ActualCommand.Type)
      {
      case TMCL_AP_ActualPosition:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_XACTUAL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_XACTUAL_MASK);
        break;

      case TMCL_AP_MaximumPositioningSpeed:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VMAX_MASK);
        break;

      case TMCL_AP_MaximumAcceleration:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_AMAX_MASK);
        break;

      case TMCL_AP_MaximumCurrent:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IRUN_MASK);
        break;

      case TMCL_AP_StandbyCurrent:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_AMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IHOLD_MASK);
        break;

      case TMCL_AP_RightLimitSwitchDisable:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_STOP_R_ENABLE_MASK);
        break;

      case TMCL_AP_LeftLimitSwitchDisable:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_STOP_L_ENABLE_MASK);
        break;

      case TMCL_AP_SwapLimitSwitches:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SWAP_LR_MASK);
        break;

      case TMCL_AP_Acceleration:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_A1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_A1_MASK);
        break;

      case TMCL_AP_Velocity:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_V1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_V1_MASK);
        break;

      case TMCL_AP_MaximumDeceleration:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DMAX(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DMAX_MASK);
        break;

      case TMCL_AP_Deceleration:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_D1(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_D1_MASK);
        break;

      case TMCL_AP_VStart:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VSTART(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSTART_MASK);
        break;

      case TMCL_AP_VStop:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VSTOP(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSTOP_MASK);
        break;

      case TMCL_AP_RampWaitTime:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TZEROWAIT_MASK);
        break;

      case TMCL_AP_SpeedThresholdForCoolStep:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VHIGH(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGH_MASK);
        break;

      case TMCL_AP_MinimumSpeedForDcStep:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VDCMIN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VDCMIN_MASK);
        break;

      case TMCL_AP_RightLimitSwitchPolarity:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_POL_STOP_R_MASK);
        break;

      case TMCL_AP_LeftLimitSwitchPolarity:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_POL_STOP_L_MASK);
        break;

      case TMCL_AP_SoftStopEnable:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_SWMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_EN_SOFTSTOP_MASK);
        break;

      case TMCL_AP_HighSpeedChopperMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGHCHM_MASK);
        break;

      case TMCL_AP_HighSpeedFullstepMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VHIGHFS_MASK);
        break;

      case TMCL_AP_PowerDownRamp:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_IHOLD_IRUN(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_IHOLDDELAY_MASK);
        break;

      case TMCL_AP_DcStepTime:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DC_TIME_MASK);
        break;

      case TMCL_AP_DcStepStallGuard:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_DCCTRL(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DC_SG_MASK);
        break;

      case TMCL_AP_EEPROMMagic:
        MotorConfig[ActualCommand.Motor].EEPROMMagic = eeprom->config.EEPROMMagic;
        break;

      case TMCL_AP_MicrostepResolution:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_MRES_MASK);
        break;

        //TMC5072 specific parameters
      case TMCL_AP_ChopperBlankTime:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TBL_MASK);
        break;

      case TMCL_AP_ConstantTOffMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_CHM_MASK);
        break;

      case TMCL_AP_DisableFastDecayComparator:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DISFDCC_MASK);
        break;

      case TMCL_AP_ChopperHysteresisEnd:
        ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HEND_MASK);
        }
        else
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TFD_3_MASK | TMC5072_HSTRT_MASK);
        }
        break;

      case TMCL_AP_ChopperHysteresisStart:
        ActualReply.Value.Int32 = ReadTMC5072Int(MOTOR_TO_IC_SPI(ActualCommand.Motor), TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)));
        if ((ActualReply.Value.Int32 & TMC5072_CHM_MASK) == 0)
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HSTRT_MASK);
        }
        else
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_HEND_MASK);
        }
        break;

      case TMCL_AP_ChopperOffTime:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TOFF_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentMinimum:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEIMIN_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentDownStep:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEDN_MASK);
        break;

      case TMCL_AP_SmartEnergyHysteresis:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEMAX_MASK);
        break;

      case TMCL_AP_SmartEnergyCurrentUpStep:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEUP_MASK);
        break;

      case TMCL_AP_SmartEnergyHysteresisStart:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SEMIN_MASK);
        break;

      case TMCL_AP_StallGuard2FilterEnable:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SFILT_MASK);
        break;

      case TMCL_AP_StallGuard2Threshold:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_COOLCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SGT_MASK);
        break;

      case TMCL_AP_ShortToGndDisable:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_DISS2G_MASK);
        break;

      case TMCL_AP_VSense:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VSENSE_MASK);
        break;

      case TMCL_AP_SmartEnergyStallVelocity:
        MotorConfig[ActualCommand.Motor].StallVMin = eeprom->config.StallVMin;
        break;

      case TMCL_AP_SmartEnergyThresholdSpeed:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_VCOOLTHRS(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_VCOOLTHRS_MASK);
        break;

      case TMCL_AP_RandomTOffMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_CHOPCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_RNDTF_MASK);
        break;

      case TMCL_AP_PWMGradient:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_GRAD_MASK);
        break;

      case TMCL_AP_PWMAmplitude:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_AMPL_MASK);
        break;

      case TMCL_AP_PWMFrequency:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_FREQ_MASK);
        break;

      case TMCL_AP_PWMAutomaticScale:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_PWM_AUTOSCALE_MASK);
        break;

      case TMCL_AP_EncoderMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_ENCMODE(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), 0xFFF);
        break;

      case TMCL_AP_MotorFullStepResolution:
        MotorConfig[ActualCommand.Motor].MotorFullStepResolution = eeprom->config.MotorFullStepResolution;
        break;

      case TMCL_AP_FreewheelingMode:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_PWMCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_FREEWHEEL_MASK);
        break;

      case TMCL_AP_EncoderResolution:
        MotorConfig[ActualCommand.Motor].EncoderResolution = eeprom->config.EncoderResolution;
        break;

      case TMCL_AP_MaximumEncoderDeviation:
        MotorConfig[ActualCommand.Motor].MaxDeviation = eeprom->config.MaxDeviation;
        break;

      case TMCL_AP_PowerDownDelay:
        ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_TZEROWAIT(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_TZEROWAIT_MASK);
        break;

      case TMCL_AP_ReverseShaft:
        if (MOTOR_TO_IC_MOTOR(ActualCommand.Motor) == 0)
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_GCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SHAFT1_MASK);
        }
        else
        {
          ActualReply.Status = LoadMotorRegister(eeprom, ActualCommand.Motor, TMC5072_GCONF(MOTOR_TO_IC_MOTOR(ActualCommand.Motor)), TMC5072_SHAFT2_MASK);
        }
        break;

      case TMCL_AP_UnitMode:
        MotorConfig[ActualCommand.Motor].UnitMode = eeprom->config.UnitMode;
        break;

      default:
        ActualReply.Status = REPLY_WRONG_TYPE;
        break;
      }
    }
    else ActualReply.Status = REPLY_EEPROM_LOCKED;
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetGlobalParameter(void)
   \brief Command SGP

  SGP (Set Global Parameter) command (see TMCL manual).
********************************************************************/
static void SetGlobalParameter(void)
{
  if (ActualCommand.Motor == 0) //Bank 0
  {
    switch (ActualCommand.Type)
    {
    case TMCL_GP_EEPROMMagic:
      ModuleConfig.EEPROMMagic = ActualCommand.Value.Int32;
      break;
    case TMCL_GP_SerialBaudRate:
      ModuleConfig.SerialBitrate = ActualCommand.Value.Int32;
      break;
    case TMCL_GP_SerialAddress:
      ModuleConfig.SerialModuleAddress = ActualCommand.Value.Int32;
      break;
    case TMCL_GP_SerialHostAddress:
      ModuleConfig.SerialHostAddress = ActualCommand.Value.Int32;
      break;
    case TMCL_GP_GaugePressureOffset:
      ModuleConfig.GaugePressureOffset = ActualCommand.Value.Float32;
      break;
    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn GetGlobalParameter(void)
   \brief Command GGP

  GGP (Get Global Parameter) command (see TMCL manual).
********************************************************************/
static void GetGlobalParameter(void)
{
  if (ActualCommand.Motor == 0) //Bank 0
  {
    switch (ActualCommand.Type)
    {
    case TMCL_GP_EEPROMMagic:
      ActualReply.Value.Int32 = ModuleConfig.EEPROMMagic;
      break;
    case TMCL_GP_SerialBaudRate:
      ActualReply.Value.Int32 = ModuleConfig.SerialBitrate;
      break;
    case TMCL_GP_SerialAddress:
      ActualReply.Value.Int32 = ModuleConfig.SerialModuleAddress;
      break;
    case TMCL_GP_SerialHostAddress:
      ActualReply.Value.Int32 = ModuleConfig.SerialHostAddress;
      break;
    case TMCL_GP_GaugePressureOffset:
      ActualReply.Value.Float32 = ModuleConfig.GaugePressureOffset;
      break;
    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn StoreGlobalParameter(void)
  \brief Command STGP

  STGP (Store Global Parameter) command (see TMCL manual).
********************************************************************/
static void StoreGlobalParameter(void)
{
  if (ActualCommand.Motor == 0) //Bank 0
  {
    TModuleEeprom* eeprom = GetModuleEeprom(NULL);
    if (eeprom != NULL)
    {
      switch (ActualCommand.Type)
      {
      case TMCL_GP_EEPROMMagic:
        eeprom->config.EEPROMMagic = ModuleConfig.EEPROMMagic;
        break;
      case TMCL_GP_SerialBaudRate:
        eeprom->config.SerialBitrate = ModuleConfig.SerialBitrate;
        break;
      case TMCL_GP_SerialAddress:
        eeprom->config.SerialModuleAddress = ModuleConfig.SerialModuleAddress;
        break;
      case TMCL_GP_SerialHostAddress:
        eeprom->config.SerialModuleAddress = ModuleConfig.SerialModuleAddress;
        break;
      case TMCL_GP_GaugePressureOffset:
        eeprom->config.GaugePressureOffset = ModuleConfig.GaugePressureOffset;
        break;
      default:
        ActualReply.Status = REPLY_WRONG_TYPE;
      }
    }
    else ActualReply.Status = REPLY_EEPROM_LOCKED;
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn RestoreGlobalParameter(void)
  \brief Command RSGP

  RSGP (Restore Global Parameter) command (see TMCL manual).
********************************************************************/
static void RestoreGlobalParameter(void)
{
  if (ActualCommand.Motor == 0) //Bank 0
  {
    TModuleEeprom* eeprom = GetModuleEeprom(NULL);
    if (eeprom != NULL)
    {
      switch (ActualCommand.Type)
      {
      case TMCL_GP_EEPROMMagic:
        ModuleConfig.EEPROMMagic = eeprom->config.EEPROMMagic;
        break;
      case TMCL_GP_SerialBaudRate:
        ModuleConfig.SerialBitrate = eeprom->config.SerialBitrate;
        break;
      case TMCL_GP_SerialAddress:
        ModuleConfig.SerialModuleAddress = eeprom->config.SerialModuleAddress;
        break;
      case TMCL_GP_SerialHostAddress:
        ModuleConfig.SerialHostAddress = eeprom->config.SerialHostAddress;
        break;
      case TMCL_GP_GaugePressureOffset:
        ModuleConfig.GaugePressureOffset = eeprom->config.GaugePressureOffset;
        break;
      default:
        ActualReply.Status = REPLY_WRONG_TYPE;
      }
    }
    else ActualReply.Status = REPLY_EEPROM_LOCKED;
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn SetInputOutput(void)
   \brief Command SIO

  SIO (Set Input Outout) command (see TMCL manual).
********************************************************************/
static void SetInputOutput(void)
{
  if (ActualCommand.Motor == 0) //Bank 0
  {
    switch (ActualCommand.Type)
    {
    case 0:
      if (ActualCommand.Value.Int32 == 0)
      {
        LL_GPIO_ResetOutputPin(OUT1_GPIO_Port, OUT1_Pin);
      }
      else if (ActualCommand.Value.Int32 == 1)
      {
        LL_GPIO_SetOutputPin(OUT1_GPIO_Port, OUT1_Pin);
      }
      else
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
      }
      break;

    case 1:
      if (ActualCommand.Value.Int32 == 0)
      {
        LL_GPIO_ResetOutputPin(OUT2_GPIO_Port, OUT2_Pin);
      }
      else if (ActualCommand.Value.Int32 == 1)
      {

        LL_GPIO_SetOutputPin(OUT2_GPIO_Port, OUT2_Pin);
      }
      else
      {
        ActualReply.Status = REPLY_INVALID_VALUE;
      }
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
   \fn GetInputOutput(void)
   \brief Command GIO

  GIO (Get Input Outout) command (see TMCL manual).
********************************************************************/
static void GetInputOutput(void)
{
  if (ActualCommand.Motor == 1) //Bank 1
  {
    switch (ActualCommand.Type)
    {
    case 0:
      ActualReply.Value.Float32 = GetGaugePressure();
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
    }
  }
  else if (ActualCommand.Motor == 2) //Bank 2
  {
    switch (ActualCommand.Type)
    {
    case 0:
      ActualReply.Value.Int32 = LL_GPIO_IsOutputPinSet(OUT1_GPIO_Port, OUT1_Pin);
      break;

    case 1:
      ActualReply.Value.Int32 = LL_GPIO_IsOutputPinSet(OUT2_GPIO_Port, OUT2_Pin);
      break;

    default:
      ActualReply.Status = REPLY_WRONG_TYPE;
    }
  }
  else ActualReply.Status = REPLY_INVALID_VALUE;
}


/***************************************************************//**
  \fn GetVersion(void)
  \brief Command 136 (get version)

  Get the version number (when type==0) or
  the version string (when type==1).
********************************************************************/
static void GetVersion(void)
{
  UCHAR i;

  if (ActualCommand.Type == 0)
  {
    TMCLReplyFormat = RF_SPECIAL;
    SpecialReply[0] = ModuleConfig.SerialHostAddress;
    for (i = 0; i < 8; i++)
      SpecialReply[i + 1] = VersionString[i];
  }
  else if (ActualCommand.Type == 1)
  {
    ActualReply.Value.Byte[3] = SW_TYPE_HIGH;
    ActualReply.Value.Byte[2] = SW_TYPE_LOW;
    ActualReply.Value.Byte[1] = SW_VERSION_HIGH;
    ActualReply.Value.Byte[0] = SW_VERSION_LOW;
  }
}

/**************************//**
   \fn SoftwareReset(void)
   \brief TMCL software reset command

   Issue a TMCL software reset.
 *******************************/
static void SoftwareReset(void)
{
  if (ActualCommand.Value.Int32 == 1234)
  {
    ResetRequested = TRUE;
  }
}
