/*!
    \file    vf_if_startup.h
    \brief   the header file of vf or if startup algorithm

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
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

#ifndef VF_IF_STARTUP_H
#define VF_IF_STARTUP_H

#include "motor.h"

/* vf_startup parameters */
#define VF_PERIOD                                 (10000)                       /* the period for vf start up */
#define VF_START_UQ                               (0.5f)                        /* the start voltage of q-axis for vf start up */
#define VF_END_UQ                                 (5.0f)                        /* the end voltage of q-axis for vf start up */
#define VF_START_ANGULAR_SPEED                    (0.0015f)                     /* the start angular speed for vf start up */
#define VF_END_ANGULAR_SPEED                      (0.01f)                       /* the end angular speed for vf start up */

/* if_startup parameters */
#define IF_STAGE_ONE_PERIOD                       (10000)                       /* the period of stage one torque for if start up */
#define IF_STAGE_TWO_PERIOD                       (20000)                       /* the period of stage two torque for if start up */
#define IF_START_TORQUE                           (1.0f)                        /* the start torque for if start up */
#define IF_END_TORQUE                             (4.0f)                        /* the end torque for if start up */
#define IF_END_ANGULAR_SPEED                      (0.01f)                       /* the end angular speed for if start up */
#define IF_POWER_PHASE                            (MOTOR_PI/4)                  /* the power phase for if start up */
#define IF_PHASE_FACTOR                           (0.001f)                      /* phase factor for if start up */

/* if_startup pid parameters */
#define IF_STARTUP_LOOP_KP                        (0.002f)                      /* the proportional factor */
#define IF_STARTUP_LOOP_KI                        (0.00025f)                    /* the integral factor */
#define IF_STARTUP_LOOP_SUM_MAX                   (2.5f)                        /* the upper limit of integral term */
#define IF_STARTUP_LOOP_SUM_MIN                   (-2.5f)                       /* the lower limit of integral term */
#define IF_STARTUP_LOOP_OUTPUT_MAX                (3.5f)                        /* the upper limit of ouput */
#define IF_STARTUP_LOOP_OUTPUT_MIN                (-3.5f)                       /* the lower limit of ouput */

/* IF start up enum definition */
/* IF start up stage */
typedef enum{
    IF_STARTUP_OPEN_LOOP_STAGE = 1,                                             /* IF start up open loop stage */
    IF_STARTUP_PHASE_SELF_EQULIBRIUM_STAGE,                                     /* IF start up phase self-equlibrium stage */ 
    IF_STARTUP_PHASE_SMOOTH_TRANSITION_STAGE,                                   /* IF start up phase smooth transition stage */ 
    IF_STARTUP_CLOSE_LOOP_STAGE,                                                /* IF start up close loop stage */
}if_startup_stage_enum;
/* IF start up stage */

/* constants definitions */
/* vf start up struct */
typedef struct vf_startup_struct{
    uint16_t count;                                                             /* count value */

    uint16_t period;                                                            /* period value */
    uint16_t period_def;                                                        /* period value definition */

    float start_uq;                                                             /* q-axis voltage at start */
    float start_uq_def;                                                         /* q-axis voltage definition at start */
    float end_uq;                                                               /* q-axis voltage at end */
    float end_uq_def;                                                           /* q-axis voltage definition at end */

    float start_angular_speed;                                                  /* the angular speed at start time */
    float start_angular_speed_def;                                              /* the angular speed definition at start time */
    float end_angular_speed;                                                    /* the angular speed at end time */
    float end_angular_speed_def;                                                /* the angular speed definition at end time */

    float uq_slope;                                                             /* the slope for voltage of q-axis */
    float angular_speed_slope;                                                  /* the slope for angular speed */

    float angular_speed_reference;                                              /* the angular speed reference */
    float phase_reference;                                                      /* the phase reference */
    float uq_reference;                                                         /* q-axis voltage reference */

    /* vf start up init, initialization of parameters */
    void (*vf_start_up_init)(motor_struct* motor,struct vf_startup_struct* vf_startup);   /* function pointer */
    /* vf start up algorithm, which is called in each sampling period */
    void (*vf_start_up)(motor_struct* motor,struct vf_startup_struct* vf_startup);        /* function pointer */
}vf_startup_struct;

/* if start up struct */
typedef struct if_startup_struct{
    uint8_t stage;                                                              /* the stage of IF start up */
    uint16_t count;                                                             /* count value */

    uint16_t stage_one_period;                                                  /* period value of stage one, uint: PWM cycle */
    uint16_t stage_one_period_def;                                              /* period value definition of stage one, uint: PWM cycle */

    uint16_t stage_two_period;                                                  /* period value of stage two, uint: PWM cycle */
    uint16_t stage_two_period_def;                                              /* period value definition of stage two, uint: PWM cycle */

    float start_torque;                                                         /* torque at start time */
    float start_torque_def;                                                     /* torque definition at start time */
    float end_torque;                                                           /* torque at end time */
    float end_torque_def;                                                       /* torque definition at end time */

    float end_angular_speed;                                                    /* the angular speed at end time */
    float end_angular_speed_def;                                                /* the angular speed definition at end time */

    float torque_slope;                                                         /* the slope for torque */
    float angular_speed_slope;                                                  /* the slope for angular speed */

    float angular_speed_reference;                                              /* the angular speed reference */
    float phase_reference;                                                      /* the phase reference */
    float torque_reference;                                                     /* the torque reference */

    float phase_error;                                                          /* the phase error between the observer phase and the open-loop phase */
    float power_phase;                                                          /* the power phase between the observer phase and the open-loop phase */
    float power_phase_def;                                                      /* the power phase definition between the observer phase and the open-loop phase */
    float phase_factor;                                                         /* the factor between the observer phase and the open-loop phase */
    float phase_factor_def;                                                     /* the factor between the observer phase and the open-loop phase */
    float phase_factor_value;                                                   /* the factor value */

    float open_loop_phase;                                                      /* open loop phase */
    float observer_phase;                                                       /* observer phase */
    pid_struct* if_startup_pid;                                                 /* pointer to a pid struct for if start up control */

    /* if start up algorithm, which is called in each sampling period */
    void (*if_start_up_init)(motor_struct* motor,struct if_startup_struct *if_startup);     /* function pointer */
    /* if start up init, initialization of parameters */
    void (*if_start_up)(motor_struct* motor,struct if_startup_struct *if_startup);          /* function pointer */
}if_startup_struct;

/* vf start up init, initialization of parameters */
void vf_start_up_init(motor_struct* motor,vf_startup_struct *vf_startup);
/* vf start up algorithm, which is called in each sampling period */
void vf_start_up(motor_struct* motor,vf_startup_struct *vf_startup);

/* if start up init, initialization of parameters */
void if_start_up_init(motor_struct* motor,if_startup_struct *if_startup);
/* if start up algorithm, which is called in each sampling period */
void if_start_up(motor_struct* motor,if_startup_struct *if_startup);
#endif /* VF_IF_STARTUP_H */
