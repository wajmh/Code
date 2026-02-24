/*!
    \file    pwm_current.h
    \brief   the header file of PWM and current sampling

    \version 2025-06-30, V1.0.0, GDFOC2.0
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

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

#ifndef PWM_CURRENT_H
#define PWM_CURRENT_H

#include "motor.h"

/* the sample mode of the current */
typedef enum{
    CURRENT_3SHUNT = 0,                                                  
    CURRENT_2SHUNT,                                                       
    CURRENT_1SHUNT,                                                    
}current_mode;

/* the mode of the modulation */
typedef enum{
    DSVPWMMIN = 1,                                                       
    CSVPWM,                                                    
}modul_mode;

/* pwm update duty cycle */
typedef void ( *pwm_config_duty )(modulation_struct pwm);
/* init adc and timer peripheral */
typedef void ( *pwm_current_init )(motor_struct* motor);
/* get three-phase current */
typedef void ( *get_current_shunt )(motor_struct* motor);
/* config and calculation current sample point */
typedef void ( *sample_config )(modulation_struct* pwm, motor_struct* motor);
/* pwm and current sample srtuct */
typedef struct{
    /* phase current sampling mode */
    current_mode samp_mode;      
    /* init ADC peripheral for current sample */
    pwm_current_init shunt3_init;                                       /* function pointer */
    pwm_current_init shunt2_init;                                       /* function pointer */
    pwm_current_init shunt1_init;                                       /* function pointer */
    /* get current iab */
    get_current_shunt shunt3_get_iab;                                   /* function pointer */
    get_current_shunt shunt2_get_iab;                                   /* function pointer */
    get_current_shunt shunt1_get_iab;                                   /* function pointer */
    /* config and calculation current sample point */
    sample_config config_sample_3s;                                     /* function pointer */
    sample_config config_sample_2s;                                     /* function pointer */
    sample_config config_sample_1s;                                     /* function pointer */
    
    /* phase current sampling mode: CSVPWM, DSVPWMMIN, CBSVPWM */
    modul_mode pwm_mode;                                                /* the mode of the modulation */
    modulation_struct modulation;
    pwm_config_duty  update_duty_2s;
    pwm_config_duty  update_duty_3s;
}pwm_current_struct;

/* initialize the pwm and adc sample peripheral */
void pwm_current_peripheral_init(pwm_current_struct* pwm_current, motor_struct* motor);
/* pwm wave control */
void pwm_generation(pwm_current_struct* pwm_current, motor_struct* motor);
/* get three-phase current */
void get_iab(pwm_current_struct* pwm_current, motor_struct* motor);


#endif /* PWM_CURRENT_H */
