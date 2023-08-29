/* 
 * File:   RGB.h
 * Author: Amit
 *
 * Created on August 24, 2023, 11:11 AM
 */
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


#ifndef RGB_H
#define	RGB_H

#ifdef	__cplusplus
extern "C" {
#endif

void TC1_PWM_User_Set_DutyCycle( uint16_t Compare_0_Value ,uint16_t Compare_1_Value);
void TC0_PWM_User_Set_DutyCycle( uint16_t Compare_0_Value);




void RED(void);
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


#ifdef	__cplusplus
}
#endif

#endif	/* RGB_H */

