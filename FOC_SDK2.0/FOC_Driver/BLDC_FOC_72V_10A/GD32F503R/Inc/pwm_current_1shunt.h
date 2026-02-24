/*!
    \file    pwm_current_1shunt.h
    \brief   the header file of pwm_current_1shunt

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef PWM_CURRENT_1SHUNT_H
#define PWM_CURRENT_1SHUNT_H

#include "motor.h"

/* macro definition */
/* 1-shunt sample parameters */
#define TADC_STABLE_POINT            (uint16_t)(TADC_RISE_NS*2*PWM_COUNT/TS_NS)
#define TMIN                         (uint16_t)(TWINDOW_NS*2*PWM_COUNT/TS_NS)

/* DMA transmit address */
#define TIMER_DMATB_ADDRESS          ((uint32_t)0x40012C4C)                                                                           /* the TIMERx_DMATB address */
#define TIMER_CH3CV_ADDRESS          ((uint32_t)0x40012CE4)                                                                           /* the TIMERx_CH3CV address */

/* enum definition */
/* the enum of 1shunt sampling */
typedef enum{
    NORMAL = 0,                                                                 /* normal region */
    REGION1,                                                                    /* region 1 */
    REGION2,                                                                    /* region 2 */
    LOW_MODULATION                                                              /* low modulation region */
}current_sample_region_type_enum;

/* the sampling current flag */
typedef enum{
    NONE = 0,                                                                   /* none sampling current flag */
    IA,                                                                         /* sampling IA flag */
    IB,                                                                         /* sampling IB flag */
    IC,                                                                         /* sampling IC flag */
}current_sample_type_enum;

/* srtuct definition */
/* the structure of 1-shunt sampling */
typedef struct{
    uint16_t ta;                                                                /* the calculated time a */
    uint16_t tb;                                                                /* the calculated time b */
    uint16_t tc;                                                                /* the calculated time c */
    uint16_t delta_t1;                                                          /* the error of two timing */
    uint16_t delta_t2;                                                          /* the error of two timing */
    current_sample_region_type_enum region;                                     /* the region of applying voltage */
    int16_t ta_mod1;                                                            /* the modified time a */
    int16_t ta_mod2;                                                            /* the modified time a */
    int16_t tb_mod1;                                                            /* the modified time b */
    int16_t tb_mod2;                                                            /* the modified time b */
    int16_t tc_mod1;                                                            /* the modified time c */
    int16_t tc_mod2;                                                            /* the modified time c */
    uint16_t samp1;                                                             /* the first sampling time */
    uint16_t samp2;                                                             /* the second sampling time */
    current_sample_type_enum current1;                                          /* the flag of first sampling current */
    current_sample_type_enum current2;                                          /* the flag of second sampling current */
}current_sample_struct;

/* function declaration */
/* initialize the 1shunt method */
void pwm_current_1shunt_peripheral_init(motor_struct* motor);
/* read the sampling current in the 1shunt method */
void pwm_current_1shunt_get_current(motor_struct* motor);
/* modify the sampling point in the 1shunt method */
void pwm_current_1shunt_sampling_point_config(modulation_struct* pwm, motor_struct* motor);

#endif /* PWM_CURRENT_1SHUNT_H */
