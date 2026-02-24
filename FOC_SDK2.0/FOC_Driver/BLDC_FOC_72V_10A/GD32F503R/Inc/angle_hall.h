/*!
    \file    angle_hall.h
    \brief   the header file of angle_hall

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

#ifndef ANGLE_HALL_H
#define ANGLE_HALL_H

#include "motor.h"

/* macro definition */
/* HALL_POSITION_DETECT mode parameters */
#define HALL_DETECT_PHASE_SHIFT         (0.0f  / 360 * MOTOR_PI_2)              /* phase shift in HALL_POSITION_DETECT mode */
#define HALL_DETECT_TORQUE_SET          (1.8f)                                  /* hall detect step=1, openloop uq set */
#define HALL_DETECT_THETA_ADD1          (0.3f  / 360 * MOTOR_PI_2)              /* hall detect step=1, openloop phase add set */
#define HALL_DETECT_FLUX_SET            (3.0f)                                 /* hall detect step=2,3, openloop ud set */
#define HALL_DETECT_DELAY1              (0.2f * PWM_FREQUENCE)                  /* hall detect step=2, openloop phase add delay set */
#define HALL_DETECT_THETA_ADD2          (1.0f  / 360 * MOTOR_PI_2)              /* hall detect step=3, openloop phase add set */
#define HALL_DETECT_DELAY2              (0.1f * PWM_FREQUENCE)                  /* hall detect step=3, openloop phase add delay set */
#define HALL_DETECT_THETA_ADD3          (10.0f  / 360 * MOTOR_PI_2)             /* hall detect step=3, openloop phase add set when direction changes */

/* HALL_POSITION_FIX mode parameters */
/* the phase of each hall state when the direction is CCW */
#define HALL_FIX_PHASE_SHIFT            (0.0f)                                  /* phase shift in HALL_POSITION_FIX mode */
#define HALL_PHA_CCW_STATE_1            (4.753f)                                /* the phase of the hall state 1 */
#define HALL_PHA_CCW_STATE_2            (0.565f)                                /* the phase of the hall state 2 */
#define HALL_PHA_CCW_STATE_3            (5.801f)                                /* the phase of the hall state 3 */
#define HALL_PHA_CCW_STATE_4            (2.659f)                                /* the phase of the hall state 4 */
#define HALL_PHA_CCW_STATE_5            (3.706f)                                /* the phase of the hall state 5 */
#define HALL_PHA_CCW_STATE_6            (1.615f)                                /* the phase of the hall state 6 */
/* the sequence of each hall state when the direction is CCW */
#define HALL_SEQ_CCW_STATE_1            (5U)                                    /* the ccw sequence of the hall state 1 */
#define HALL_SEQ_CCW_STATE_2            (1U)                                    /* the ccw sequence of the hall state 2 */
#define HALL_SEQ_CCW_STATE_3            (6U)                                    /* the ccw sequence of the hall state 3 */
#define HALL_SEQ_CCW_STATE_4            (3U)                                    /* the ccwsequence of the hall state 4 */
#define HALL_SEQ_CCW_STATE_5            (4U)                                    /* the ccw sequence of the hall state 5 */
#define HALL_SEQ_CCW_STATE_6            (2U)                                    /* the ccw sequence of the hall state 6 */

/* hall speed calculate parameters */
#define HALL_AVE_SPEED_LEN              (6U)                                    /* hall moving average filter length */
#define HALL_SPEED_FACTOR               (1000000.0f * 60 /(NUMBER_OF_PAIRS*6))  /* the factor of the hall speed calculate */
#define HALL_HIGH_SPEED_FACTOR          (1000000.0f * 60 /NUMBER_OF_PAIRS)      /* the factor of the hall high speed calculate */
#define HALL_SPEED_THRESHOLD            (6000.0f)                               /* hall speed threshold */

/* hall angle calculate parameters */
#define HALL_SPEED_LIMIT                (80U)                                   /* the speed limit for angle estimation using HALL_PHASE_DIFF_INIT value */
#define HALL_PHASE_DIFF_INIT            (0.0008f)                               /* the init accumulated value of rotor angle at each PWM cycle */

/* enum definition */
/* the installation position location mode of the hall sensor */
typedef enum{
    HALL_POSITION_DETECT = 0,                                                   /* get hall installation position from hall detect results */
    HALL_POSITION_FIX,                                                          /* get hall installation position from macro definition */
}hall_position_mode_enum;

/* the running mode of the motor */
typedef enum{
    HALL_RUN_NORMAL = 0,                                                       /* the hall normal running mode */
    HALL_RUN_DETECT,                                                           /* the hall detect running mode */
}hall_running_mode_enum;

/* srtuct definition */
/* hall mode control srtuct */
typedef struct hall_struct{
    /* hall mode enum variable definition */
    hall_position_mode_enum   hall_position_mode;                               /* the hall position mode */
    hall_running_mode_enum    hall_running_mode;                                /* the hall running mode */

    /* position calculate variable definition*/
    uint8_t                   state;                                            /* the hall state mode */
    uint8_t                   sequence[2][8];                                   /* the hall ccw/cw sequence */
    float                     phase_table[2][8];                                /* the hall phase table */
    float                     phase_shift;                                      /* the hall phase shift */
    float                     phase;                                            /* the hall estimated rotor angle */
    float                     phase_error;                                      /* the hall estimated error of rotor angle */
    uint8_t                   direct_correct_flag;                              /* the hall first direct correction flag */
    uint8_t                   phase_correct_cnt;                                /* the hall multiperiod average correction count */
    uint8_t          seq_temp;                                                  /* sequence temp */
    
    /* virtual function definition */
    /* initialize peripherals */
    void (*hall_peripheral_init)(void);                                                             /* function pointer */
    /* initialize hall normal mode variables */
    void (*hall_normal_var_init)(struct hall_struct* hall, motor_struct* motor);                    /* function pointer */
    /* calculate rotor speed */
    int16_t (*hall_speed_calc)(void);                                                               /* function pointer */
    /* calculate rotor angle */
    void (*hall_angle_calc)(struct hall_struct* hall, motor_struct* motor);                         /* function pointer */
    /* initialize hall detect mode variables */
    void (*hall_detect_var_init)(struct hall_struct* hall, motor_struct* motor);                    /* function pointer */
    /* hall installation angle detect */
    void (*hall_angle_detect)(struct hall_struct* hall, motor_struct* motor);                       /* function pointer */
    /* hall sensor interrupt handler */
    void (*hall_sensor_interrupt_handler)(struct hall_struct* hall, motor_struct* motor);           /* function pointer */
}hall_struct;

/* hall detect mode srtuct */
typedef struct hall_detect_struct{
    uint8_t    step;                                                 /* the hall detect step */

    /* uq_detect variable definition */
    uint8_t    uq_data_record;                                       /* the hall uq detect data record flag */
    uint8_t    uq_circle;                                            /* the hall uq detect circle count */
    uint8_t    uq_sequence[8];                                       /* the hall uq detect result: sequence */
    float      uq_phase[8];                                          /* the hall uq detect result: relative phase */
    uint8_t    uq_count[8];                                          /* the hall uq detect result: relative phase record count */

    /* ud_detect variable definition */
    uint8_t    ud_direction;                                         /* the hall ud detect direction */
    uint8_t    ud_state[2];                                          /* the hall ud detect result: hall state */
    float      ud_phase[2];                                          /* the hall ud detect result: absolute phase  */

    /* result calculate variable definition */
    float      phase_shift;                                          /* the hall detect phase shift between relative phase and absolute phase*/
}hall_detect_struct;

/* hall speed calculate srtuct */
typedef struct hall_speed_struct{
    /* speed */
    float      speed_sum;                                          /* the sum of hall speed array data */
    float      aver_speed;                                         /* the result of hall speed moving average filter */
    float      low_speed;                                          /* speed calculate for low speed */
    float      high_speed;                                         /* speed calculate for high speed */
    float      speed_array[HALL_AVE_SPEED_LEN];                    /* the hall speed array */
    uint8_t    speed_index;                                        /* the hall speed array data pointer */
    uint8_t    timer_overflow;                                     /* the hall timer overflow count */
    uint32_t   timer_count_sum;                                    /* timer count for high speed calculate */
}hall_speed_struct;

/* function declaration */
/* initialize hall sensor peripherals */
void hall_peripheral_init(void);
/* initialize hall normal mode variables */
void hall_normal_var_init(hall_struct* hall,motor_struct* motor);
/* return rotor speed */
int16_t hall_speed_calc(void);
/* calculate rotor angle */
void hall_angle_calc(hall_struct* hall, motor_struct* motor);
/* initialize hall detect mode variables */
void hall_detect_var_init(hall_struct* hall, motor_struct* motor);
/* hall installation angle detect */
void hall_angle_detect(hall_struct* hall, motor_struct* motor);
/* hall sensor interrupt handler */
void hall_sensor_interrupt_handler(hall_struct* hall, motor_struct* motor);

#endif /* ANGLE_HALL_H */

