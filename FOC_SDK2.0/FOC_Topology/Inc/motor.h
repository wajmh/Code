/*!
    \file    motor.h
    \brief   the header file of motor

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

#ifndef MOTOR_H
#define MOTOR_H

#include "basic_head.h"

/* motor enum definition */
/* motor running mode control */
typedef enum{
    OPENLOOP_VF = 0,                                                            /* open loop position ,VF start up */
    OPENLOOP_IF,                                                                /* open loop position ,IF start up */
    SMO_VF,                                                                     /* sliding mode observer ,VF start up */
    SMO_IF,                                                                     /* sliding mode observer ,IF start up */
    AFLUX_VF,                                                                   /* active flux observer ,VF start up */
    AFLUX_IF,                                                                   /* active flux observer ,IF start up */
    HALL_SENSOR_DETECT,                                                         /* HALL sensor installation angle indentification */
    HALL_SENSOR,                                                                /* HALL sensor mode */
    ENCODER_DETECT,                                                             /* sliding mode observer ,HFI pulsating start up */
    ENCODER,                                                                    /* sliding mode observer ,HFI pluse start up */
}mc_running_mode_enum;

/* motor sector */
typedef enum{
    ONE = 1,                                                                    /* sector one */
    TWO,                                                                        /* sector two */
    THREE,                                                                      /* sector three */
    FOUR,                                                                       /* sector four */
    FIVE,                                                                       /* sector five */
    SIX                                                                         /* sector six */
}sector_enum;

/* the command of motor */
typedef enum{
    MC_NONE = 0,                                                                /* motor start command */
    MC_START,                                                                   /* motor start command */
    MC_STOP,                                                                    /* motor stop command */
    MC_DIR_CHANGE,                                                              /* motor direction change command */
    MC_BREAK                                                                    /* motor break command */
}mc_command_enum;

/* the state of motor */
typedef enum{
    MC_STATE_INIT = 0,                                                          /* init state */
    MC_STATE_IDLE,                                                              /* idle state */
    MC_STATE_RUNNING,                                                           /* running state */
    MC_STATE_BRAKE,                                                             /* brake state */
    MC_STATE_FAULT                                                              /* fault state */
}mc_state_enum;

/* the fault of the motor */
typedef enum{
    FAULT_NONE = 0,                                                             /* no faults */
    FAULT_VOLTAGE_ERROR,                                                        /* under voltage or over voltage fault */
    FAULT_OVER_CURRENT,                                                         /* over current fault */
    FAULT_MOTOR_LOCKED,                                                         /* blocking fault */
    FAULT_SPEED_ERROR,                                                          /* blocking fault */
    FAULT_PHASE_LOSS,                                                           /* phase loss fault */
    FAULT_ANGLE
}mc_fault_enum;

/* motor running direction */
typedef enum{
    DIRECTION_CCW = 0,                                                          /* the direction is counter clockwise */
    DIRECTION_CW,                                                               /* the direction is clockwise */
}mc_direction_enum;

/* motor start up mode */
typedef enum{
    VF_STARTUP = 0,                                                             /* vf start up */
    IF_STARTUP,                                                                 /* if start up */
    HFI_PULSATING_STARTUP,                                                      /* hfi start up */
    HFI_PULSE_STARTUP,                                                          /* hfi start up */
}mc_vf_if_startup_mode_enum;

/* motor ud and uq control */
typedef enum{
    UDQ_CUSTOM = 0,                                                             /* ud and uq in open loop mode */
    UDQ_MTPA_IF,                                                                /* ud and uq in mtpa mode */
    UDQ_FW,                                                                     /* ud and uq in flux weakening mode */
    UDQ_OBSERVER,                                                               /* ud and uq in observer mode */
}mc_udq_ctrl_enum;

/* motor id and iq control */
typedef enum{
    IDQ_CUSTOM = 0,                                                             /* id and iq in open loop mode */
    IDQ_SPEED_MTPA,                                                             /* id and iq mtpa control in speed mode */
    IDQ_SPEED_FW,                                                               /* id and iq in flux weakening control in speed mode */
    IDQ_CURRENT_MTPA,                                                           /* id and iq mtpa control in speed mode */
    IDQ_CURRENT_FW,                                                             /* id and iq in flux weakening control in speed mode */
    IDQ_SPEED_OBSERVER,                                                         /* id and iq in smo control in speed mode */
}mc_idqref_ctrl_enum;

/* motor openloop control struct */
typedef struct{
    float d_var;                                                                /* variate(voltage or current) of d winding, uint: V or A */
    float q_var;                                                                /* variate(voltage or current) of q winding, uint: V or A */
    float angle_var;                                                            /* angle variate, uint:rad(radian) */
}mc_openloop_struct;

/* motor vf start up struct */
typedef struct{
    uint8_t close_loop_flag;                                                    /* close loop flag, 1 is close loop, 0 is open loop */
    float ud_reference;                                                         /* q-axis voltage reference */
    float uq_reference;                                                         /* d-axis voltage reference */
}mc_vf_startup_struct;

/* motor vf start up struct */
typedef struct{
    uint8_t stage;
    uint32_t speed_ref;
    float ud;                                                                   /* q-axis voltage reference */
    float uq;                                                                   /* d-axis voltage reference */
    float id_lpf;
    float iq_lpf;
}mc_hfi_startup_struct;

/* motor if start up struct */
typedef struct{
    uint8_t close_loop_flag;                                                    /* close loop flag, 1 is close loop, 0 is open loop */
    float torque_reference;                                                     /* torque reference */
    float flux_reference;                                                       /* flux reference */
}mc_if_startup_struct;

typedef void ( *motor_control )(void);

/* motor struct definition */
typedef struct{
    /* motor body parameter */
    float rs;                                                                   /* winding phase resistance */
    float ld;                                                                   /* winding inductance of axis d, uint:H(Ampere) */
    float lq;                                                                   /* winding inductance of axis q, uint:H(Ampere) */
    float flux;                                                                 /* permanent flux, uint:Wb */
    float pwm_frequency;                                                        /* pwm frequency */
    float ts;                                                                   /* sampling time in seconds */
    uint8_t number_of_pairs;                                                    /* PMSM number of pair poles */
    
    /* physical quantities on A-B-C coordinate system */
    float ia;                                                                   /* phase current of A winding, uint:A(Ampere) */
    float ib;                                                                   /* phase current of B winding, uint:A(Ampere) */
    float ic;                                                                   /* phase current of C winding, uint:A(Ampere) */
    float ua;                                                                   /* phase voltage of A winding, uint:V(Volt) */
    float ub;                                                                   /* phase voltage of B winding, uint:V(Volt) */
    float uc;                                                                   /* phase voltage of C winding, uint:V(Volt) */
    uint16_t i_a_offset;                                                        /* the offset of current a */
    uint16_t i_b_offset;                                                        /* the offset of current b */
    uint16_t i_c_offset;                                                        /* the offset of current c */  
    uint16_t i_bus_offset;                                                      /* the offset of current bus */
    
    /* physical quantities on alpha-beta coordinate system */
    float ialpha;                                                               /* phase current of alpha winding, uint:A(Ampere) */
    float ibeta;                                                                /* phase current of beta winding, uint:A(Ampere) */
    float ualpha;                                                               /* phase voltage of alpha winding, uint:V(Volt) */
    float ubeta;                                                                /* phase voltage of beta winding, uint:V(Volt) */
    
    /* physical quantities on d-q coordinate system */
    float id;                                                                   /* phase current of d winding, uint:A(Ampere) */
    float iq;                                                                   /* phase current of q winding, uint:A(Ampere) */
    float ud;                                                                   /* phase voltage of d winding, uint:V(Volt) */
    float uq;                                                                   /* phase voltage of q winding, uint:V(Volt) */
    float id_ref;                                                               /* reference value of phase current of d winding, uint:A(Ampere) */
    float iq_ref;                                                               /* reference value of phase current of q winding, uint:A(Ampere) */
    
    float mech_speed;                                                           /* mechanical speed, r/min */
    
    /* control variate of motor */
    float us_base;                                                              /* voltage base value, 2/3*Udc, which Udc */
    float us_out_max;                                                           /* maximum reference voltage,uint:V */
    int32_t speed_ref;                                                          /* reference value of speed */
    uint32_t speed_ref_set;                                                     /* reference value of speed */
    sector_enum sector;                                                         /* the sector of reference voltage */
    mc_command_enum command;                                                    /* the state of motor */
    mc_state_enum state;                                                        /* the state of motor */
    mc_direction_enum direction_set;                                            /* the direction of motor */
    mc_direction_enum direction;                                                /* the direction of motor */
    mc_fault_enum fault;                                                        /* the fault of motor */
    mc_running_mode_enum running_mode;                                          /* motor running mode */
    mc_udq_ctrl_enum udq_mode;                                                  /* ud and uq control mode */
    mc_idqref_ctrl_enum idqref_mode;                                            /* id and iq control mode */
    mc_openloop_struct motor_openloop;                                          /* openloop contorl variate */
    mc_vf_if_startup_mode_enum startup_mode;                                    /* motor start up mode */
    pid_struct* torque_pid;                                                     /* pointer to a pid struct for torque control */
    pid_struct* flux_pid;                                                       /* pointer to a pid struct for flux control */
    pid_struct* speed_pid;                                                      /* pointer to a pid struct for speed control */
    
    mc_vf_startup_struct vf_startup;                                            /* motor vf start up struct */
    mc_if_startup_struct if_startup;                                            /* motor if start up struct */
    mc_hfi_startup_struct hfi_startup;                                          /* motor hfi start up struct */
    
    /* virtual function definition */
    /* configure peripherals to start motor */
    void ( *motor_start )(void);                                                /* function pointer */
    /* configure peripherals to stop motor */                                   
    void ( *motor_stop )(void);                                                 /* function pointer */
    /* configure peripherals to break motor */
    void ( *motor_break )(void);                                                /* function pointer */
}motor_struct;

/* PWM wave generation srtuct */
typedef struct{
    uint32_t pwm_top;                                                           /* the TIMER period value corresponding to PWM frequence */
    uint32_t tA;                                                                /* TIMER PWM timings of phase A*/
    uint32_t tB;                                                                /* TIMER PWM timings of phase B*/
    uint32_t tC;                                                                /* TIMER PWM timings of phase C*/
    uint16_t samp1;                                                             /* the first sampling time */
    uint16_t samp2;                                                             /* the second sampling time */
}modulation_struct;

#endif /* MOTOR_H */
