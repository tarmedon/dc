/*******************************************************************************
  Data Type definition of Timer/Counter(TCC) PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_tcc4.h

  Summary:
    Data Type definition of the TCC Peripheral Interface Plib.

  Description:
    This file defines the Data Types for the TCC Plib.

  Remarks:
    None.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_TCC4_H
#define PLIB_TCC4_H

#include "device.h"
#include "plib_tcc_common.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
/*  The following data type definitions are used by the functions in this
    interface and should be considered part it.
*/

/* Total number of TCC channels in a module */
#define TCC4_NUM_CHANNELS    (2U)

/* TCC Channel numbers

   Summary:
    Identifies channel number within TCC module

   Description:
    This enumeration identifies TCC channel number.

   Remarks:
    None.
*/
typedef enum
{
    TCC4_CHANNEL0,
    TCC4_CHANNEL1,
}TCC4_CHANNEL_NUM;

// *****************************************************************************

/* TCC Channel interrupt status

   Summary:
    Identifies TCC PWM interrupt status flags

   Description:
    This enumeration identifies TCC PWM interrupt status falgs

   Remarks:
    None.
*/
typedef enum
{
    TCC4_PWM_STATUS_OVF = TCC_INTFLAG_OVF_Msk,
    TCC4_PWM_STATUS_FAULT_0 = TCC_INTFLAG_FAULT0_Msk,
    TCC4_PWM_STATUS_FAULT_1 = TCC_INTFLAG_FAULT1_Msk,
    TCC4_PWM_STATUS_MC_0 = TCC_INTFLAG_MC0_Msk,
    TCC4_PWM_STATUS_MC_1 = TCC_INTFLAG_MC1_Msk,
}TCC4_PWM_STATUS;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions make up the methods (set of possible operations) of
   this interface.
*/

// *****************************************************************************
void TCC4_PWMInitialize(void);

void TCC4_PWMStart(void);

void TCC4_PWMStop(void);


void TCC4_PWMForceUpdate(void);


void TCC4_PWMPeriodInterruptEnable(void);

void TCC4_PWMPeriodInterruptDisable(void);

uint32_t TCC4_PWMInterruptStatusGet(void);

bool TCC4_PWM16bitPeriodSet(uint16_t period);

uint16_t TCC4_PWM16bitPeriodGet(void);

void TCC4_PWM16bitCounterSet(uint16_t countVal);

uint16_t TCC4_PWM16bitCounterGet(void);

__STATIC_INLINE bool TCC4_PWM16bitDutySet(TCC4_CHANNEL_NUM channel, uint16_t duty)
{
    bool status = false;
    if ((TCC4_REGS->TCC_STATUS & (1UL << (TCC_STATUS_CCBUFV0_Pos + (uint32_t)channel))) == 0U)
    {    
        TCC4_REGS->TCC_CCBUF[channel] = duty;
        status = true;
    }
    return status;
}

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }
#endif
// DOM-IGNORE-END

#endif /* PLIB_TCC4_H */
