/* 
 * File:   RGB.c
 * Author: Amit
 *
 * Created on August 24, 2023, 11:11 AM
 */



#include "RGB.h"
//#include "GUN1_Controller.h"

void TC1_PWM_User_Set_DutyCycle( uint16_t Compare_0_Value ,uint16_t Compare_1_Value)
{
    TC1_CompareStop();

    /* Reset TC */
    TC1_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC1_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC1_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure waveform generation mode */
    TC1_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_NPWM;


    TC1_REGS->COUNT16.TC_CC[0] = Compare_0_Value;
    TC1_REGS->COUNT16.TC_CC[1] = Compare_1_Value;

    /* Clear all interrupt flags */
    TC1_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;


    while((TC1_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
    TC1_CompareStart();
}


void TC0_PWM_User_Set_DutyCycle( uint16_t Compare_0_Value)
{
     TC0_CompareStop();
   
       /* Reset TC */
    TC0_REGS->COUNT16.TC_CTRLA = TC_CTRLA_SWRST_Msk;

    while((TC0_REGS->COUNT16.TC_SYNCBUSY & TC_SYNCBUSY_SWRST_Msk) == TC_SYNCBUSY_SWRST_Msk)
    {
        /* Wait for Write Synchronization */
    }

    /* Configure counter mode & prescaler */
    TC0_REGS->COUNT16.TC_CTRLA = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_PRESCSYNC_PRESC ;

    /* Configure waveform generation mode */
    TC0_REGS->COUNT16.TC_WAVE = (uint8_t)TC_WAVE_WAVEGEN_NPWM;


    TC0_REGS->COUNT16.TC_CC[0] = 65533U;
    TC0_REGS->COUNT16.TC_CC[1] = 24U;

    /* Clear all interrupt flags */
    TC0_REGS->COUNT16.TC_INTFLAG = (uint8_t)TC_INTFLAG_Msk;


    while((TC0_REGS->COUNT16.TC_SYNCBUSY) != 0U)
    {
        /* Wait for Write Synchronization */
    }
     
     
     TC0_CompareStart();
}
void RED(void)
{
    TC0_PWM_User_Set_DutyCycle(65535);
    TC1_PWM_User_Set_DutyCycle(0,32767);
}
void GREEN(void);
void BLUE(void);
void YELLOW(void);
void CYAN(void);
void MAGENTA(void);
void ORANGE(void);
void YELLOW_GREEN(void);
void CYAN_GREEN(void);
void CYAN_BLUE(void);
void BLUE_MAGENTA(void);
void RED_MAGENTA(void);





