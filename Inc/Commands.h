/*******************************************************************************
  Project: MVPnP Motor Board

  Module:  Commands.h
           Definitions needed for the TMCL interpreter

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
  \file Commands.h
  \author Trinamic Motion Control GmbH & Co KG

  \brief TMCL command definitions

  This file contains all necessary definitions for the
  TMCL command interpreter.
*/
#ifndef __COMMANDS_H
#define __COMMANDS_H

//States of the command interpreter
#define TCS_IDLE  0            //!< TMCL interpreter is in idle mode (no command to process)
#define TCS_UART  1            //!< processing a command from RS485
#define TCS_UART_ERROR 2       //!< last command from RS485 had bad check sum

//Supported TMCL commands
#define TMCL_ROR           1      //!< ROR command opcode
#define TMCL_ROL           2      //!< ROL command opcode
#define TMCL_MST           3      //!< MST command opcode
#define TMCL_MVP           4      //!< MVP command opcode
#define TMCL_SAP           5      //!< SAP command opcode
#define TMCL_GAP           6      //!< GAP command opcode
#define TMCL_STAP          7      //!< STAP command opcode
#define TMCL_RSAP          8      //!< RSAP command opcode
#define TMCL_SGP           9      //!< SGP command opcode
#define TMCL_GGP           10     //!< GGP command opcode
#define TMCL_STGP          11     //!< STGP command opcode
#define TMCL_RSGP          12     //!< RSGP command opcode
#define TMCL_SIO           14     //!< SIO command opcode
#define TMCL_GIO           15     //!< GIO command opcode
#define TMCL_GetVersion    136    //!< GetVersion command opcode
#define TMCL_SoftwareReset 255    //!< software reset command opcode

//Supported TMCL Axis parameters
#define TMCL_AP_TargetPosition                   0
#define TMCL_AP_ActualPosition                   1
#define TMCL_AP_TargetSpeed                      2
#define TMCL_AP_ActualSpeed                      3
#define TMCL_AP_MaximumPositioningSpeed          4
#define TMCL_AP_MaximumAcceleration              5
#define TMCL_AP_MaximumCurrent                   6
#define TMCL_AP_StandbyCurrent                   7
#define TMCL_AP_PositionReachedFlag              8
#define TMCL_AP_HomeSwitchState                  9
#define TMCL_AP_RightLimitSwitchState            10
#define TMCL_AP_LeftLimitSwitchState             11
#define TMCL_AP_RightLimitSwitchDisable          12
#define TMCL_AP_LeftLimitSwitchDisable           13
#define TMCL_AP_SwapLimitSwitches                14
#define TMCL_AP_Acceleration                     15
#define TMCL_AP_Velocity                         16
#define TMCL_AP_MaximumDeceleration              17
#define TMCL_AP_Deceleration                     18
#define TMCL_AP_VStart                           19
#define TMCL_AP_VStop                            20
#define TMCL_AP_RampWaitTime                     21
#define TMCL_AP_SpeedThresholdForCoolStep        22
#define TMCL_AP_MinimumSpeedForDcStep            23
#define TMCL_AP_RightLimitSwitchPolarity         24
#define TMCL_AP_LeftLimitSwitchPolarity          25
#define TMCL_AP_SoftStopEnable                   26
#define TMCL_AP_HighSpeedChopperMode             27
#define TMCL_AP_HighSpeedFullstepMode            28
#define TMCL_AP_MeasuredSpeed                    29
#define TMCL_AP_PowerDownRamp                    31
#define TMCL_AP_DcStepTime                       32
#define TMCL_AP_DcStepStallGuard                 33
#define TMCL_AP_EEPROMMagic                      64
#define TMCL_AP_RelativePositioningOption        127
#define TMCL_AP_MicrostepResolution              140
#define TMCL_AP_ChopperBlankTime                 162
#define TMCL_AP_ConstantTOffMode                 163
#define TMCL_AP_DisableFastDecayComparator       164
#define TMCL_AP_ChopperHysteresisEnd             165
#define TMCL_AP_ChopperHysteresisStart           166
#define TMCL_AP_ChopperOffTime                   167
#define TMCL_AP_SmartEnergyCurrentMinimum        168
#define TMCL_AP_SmartEnergyCurrentDownStep       169
#define TMCL_AP_SmartEnergyHysteresis            170
#define TMCL_AP_SmartEnergyCurrentUpStep         171
#define TMCL_AP_SmartEnergyHysteresisStart       172
#define TMCL_AP_StallGuard2FilterEnable          173
#define TMCL_AP_StallGuard2Threshold             174
#define TMCL_AP_ShortToGndDisable                177
#define TMCL_AP_VSense                           179
#define TMCL_AP_SmartEnergyActualCurrent         180
#define TMCL_AP_SmartEnergyStallVelocity         181
#define TMCL_AP_SmartEnergyThresholdSpeed        182
#define TMCL_AP_RandomTOffMode                   184
#define TMCL_AP_ChopperSynchronization           185
//#define TMCL_AP_PWMThresholdSpeed                186
#define TMCL_AP_PWMGradient                      187
#define TMCL_AP_PWMAmplitude                     188
//#define TMCL_AP_PWMScale                         189
//#define TMCL_AP_PWMMode                          190
#define TMCL_AP_PWMFrequency                     191
#define TMCL_AP_PWMAutomaticScale                192
#define TMCL_AP_ReferenceSearchMode              193
#define TMCL_AP_ReferenceSearchSpeed             194
#define TMCL_AP_ReferenceSwitchSpeed             195
#define TMCL_AP_EndSwitchDistance                196
#define TMCL_AP_LastReferencePosition            197
#define TMCL_AP_LatchedActualPosition            198
#define TMCL_AP_LatchedEncoderPosition           199
#define TMCL_AP_EncoderMode                      201
#define TMCL_AP_MotorFullStepResolution          202
#define TMCL_AP_FreewheelingMode                 204
#define TMCL_AP_ActualLoadValue                  206
#define TMCL_AP_ExtendedErrorFlags               207
#define TMCL_AP_MotorDriverErrorFlags            208
#define TMCL_AP_EncoderPosition                  209
#define TMCL_AP_EncoderResolution                210
#define TMCL_AP_MaximumEncoderDeviation          212
#define TMCL_AP_GroupIndex                       213
#define TMCL_AP_PowerDownDelay                   214
#define TMCL_AP_ReverseShaft                     251
#define TMCL_AP_UnitMode                         255

//Supported TMCL Global parameters
#define TMCL_GP_EEPROMMagic                      64
#define TMCL_GP_SerialBaudRate                   65
#define TMCL_GP_SerialAddress                    66
#define TMCL_GP_SerialHostAddress                76
#define TMCL_GP_SerialSecondaryAddress           87
#define TMCL_GP_GaugePressureOffset              111

//Type codes of the MVP command
#define MVP_ABS   0            //!< absolute movement (with MVP command)
#define MVP_REL   1            //!< relative movement (with MVP command)
#define MVP_COORD 2            //!< coordinate movement (with MVO command)

//Relative positioning options
#define RMO_TARGET 0    //!< last target position
#define RMO_ACTINT 1    //!< actual ramp generator position
#define RMO_ACTENC 2    //!< actual encoder position

//Unit conversion mode
#define UNIT_MODE_INTERNAL 0   //!< use internal units of the TMC5072
#define UNIT_MODE_PPS      1   //!< use PPS units

//TMCL status codes
#define REPLY_OK 100                //!< command successfully executed
#define REPLY_CHKERR 1              //!< checksum error
#define REPLY_INVALID_CMD 2         //!< command not supported
#define REPLY_WRONG_TYPE 3          //!< wrong type code
#define REPLY_INVALID_VALUE 4       //!< wrong value
#define REPLY_EEPROM_LOCKED 5       //!< EEPROM is locked
#define REPLY_CMD_NOT_AVAILABLE 6   //!< command not available due to current state
#define REPLY_CMD_LOAD_ERROR 7      //!< error when storing command to EEPROM
#define REPLY_WRITE_PROTECTED 8     //!< EEPROM is write protected
#define REPLY_MAX_EXCEEDED 9        //!< maximum number of commands in EEPROM exceeded

//Reply format
#define RF_STANDARD 0               //!< use standard TMCL reply
#define RF_SPECIAL 1                //!< use special reply

//Data structures needed by the TMCL interpreter
//! TMCL command
typedef struct
{
  UCHAR Opcode;      //!< command opcode
  UCHAR Type;        //!< type parameter
  UCHAR Motor;       //!< motor/bank parameter
  union
  {
    long Int32;      //!< value parameter as 32 bit integer
    float Float32;   //!< reply value as 32 bit float
    UCHAR Byte[4];   //!< value parameter as 4 bytes
  } Value;           //!< value parameter
} TTMCLCommand;

//! TMCL reply
typedef struct
{
  UCHAR Status;      //!< status code
  UCHAR Opcode;      //!< opcode of executed command
  union
  {
    long Int32;      //!< reply value as 32 bit integer
    float Float32;   //!< reply value as 32 bit float
    UCHAR Byte[4];   //!< reply value as 4 bytes
  } Value;           //!< value parameter
} TTMCLReply;


//Prototypes of exported functions
void InitTMCL(void);
void ProcessCommand(void);

#endif //__COMMANDS_H