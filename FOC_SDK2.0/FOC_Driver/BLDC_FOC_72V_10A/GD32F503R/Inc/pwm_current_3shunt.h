/*!
    \file    pwm_current_3shunt.h
    \brief   the header file of pwm_current_3shunt

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

#ifndef PWM_CURRENT_3SHUNT_H
#define PWM_CURRENT_3SHUNT_H

#include "motor.h"

/* function declaration */
/* initialize the 3shunt method */
void pwm_current_3shunt_peripheral_init(motor_struct* motor);
/* read the sampling current in the 3shunt method */
void pwm_current_3shunt_get_current(motor_struct* motor);
/* modify the sampling point in the 3shunt method */
void pwm_current_3shunt_sampling_point_config(modulation_struct* pwm, motor_struct* motor);
/* pwm update in the 3shunt method */
void pwm_3shunt_update_duty(modulation_struct pwm);

#endif /* PWM_CURRENT_3SHUNT_H */
