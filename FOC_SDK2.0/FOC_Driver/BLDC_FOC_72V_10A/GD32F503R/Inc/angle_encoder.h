/*!
    \file    angle_encoder.h
    \brief   the header file of angle_encoder

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

#ifndef ANGLE_ENCODER_H
#define ANGLE_ENCODER_H

#include "motor.h"

#define ENCODER_DETECT_UQ_SET           (0.0f)
#define ENCODER_DETECT_UD_SET           (2.0f)
#define ENCODER_DETECT_ANGLE_INC        (0.0005f)
#define ENCODER_COUNT_NUM               (4000U)
#define ENCODER_ELEC_STEP               (MOTOR_PI_2 * NUMBER_OF_PAIRS / ENCODER_COUNT_NUM)
#define ENCODER_SPEED_FACTOR            (60.0f/(float)NUMBER_OF_PAIRS/6.28f)

/* the running mode of the encoder */
typedef enum{
    ENCODER_RUN_NORMAL = 0,                                                                 /* the encoder normal running mode */
    ENCODER_RUN_DETECT,                                                                     /* the encoder detect running mode */
}encoder_running_mode_enum;

typedef struct encoder_struct{
    float mech_speed;                                                                       /* mechanical speed, r/min */
    float elec_speed;                                                                       /* electric speed , rad/s */
    float phase;                                                                            /* the estimated spatial angle */
    float phase_pre;                                                                        /* the estimated spatial angle at previous instant */
    float phase_shift;                                                                      /* the phase shift of encoder */
    uint8_t flag;                                                                           /* the flag of encoder */
    encoder_running_mode_enum encoder_running_mode;                                         /* the running mode of the encoder */
    uint16_t encoder_count;                                                                 /* counter for control ENCODER_RUN_DETECT */
    
    /* virtual function definition */
    /* initialize peripherals */
    void (*encoder_peripheral_init)(void);                                                  /* function pointer */
    /* calculate rotor speed */
    void (*encoder_speed_calc)(struct encoder_struct* encoder);                             /* function pointer */
    /* calculate rotor angle */
    void (*encoder_angle_calc)(struct encoder_struct* encoder, motor_struct* motor);        /* function pointer */
    /* initialize encoder detect mode variables */
    void (*encoder_detect_var_init)(struct encoder_struct* encoder, motor_struct* motor);   /* function pointer */
    /* initialize encoder mode variables */
    void (*encoder_var_init)(struct encoder_struct* encoder, motor_struct* motor);          /* function pointer */
    /* hall installation angle detect */
    void (*encoder_angle_detect)(struct encoder_struct* encoder, motor_struct* motor);      /* function pointer */
    /* hall sensor interrupt handler */
    void (*encoder_interrupt_handler)(struct encoder_struct* encoder, motor_struct* motor); /* function pointer */
}encoder_struct;

/* encoder timer configuration */
void encoder_sensor_peripheral_init(void);
/* encoder control mode, update motor speed, which is expressed in revolutions per second */
void encoder_sensor_speed_calc(encoder_struct* encoder);
/* encoder control mode, update the rotor location information with encoder state changing */
void encoder_sensor_angle_calc(encoder_struct* encoder, motor_struct* motor);
/* initialize encoder detect mode variables */
void encoder_sensor_detect_var_init(encoder_struct* encoder, motor_struct* motor);
/* initialize encoder mode variables */
void encoder_sensor_var_init(encoder_struct* encoder, motor_struct* motor);
/* the encoder shift phase detecting routine */
void encoder_sensor_angle_detect(encoder_struct* encoder, motor_struct* motor);
/* encoder interrupt handler */
void encoder_sensor_interrupt_handler(encoder_struct* encoder, motor_struct* motor);
#endif /* ANGLE_ENCODER_H */

