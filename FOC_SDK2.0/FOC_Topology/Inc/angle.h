/*!
    \file    angle.h
    \brief   the header file of angle

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

#ifndef ANGLE_H
#define ANGLE_H

#include "angle_hall.h"
#include "vf_if_startup.h"
#include "smo_observer.h"
#include "angle_encoder.h"
#include "aflux_observer.h"
/* angle enum definition */
/* angle mode enum */
typedef enum{
    ANGLE_CUSTOM_MODE = 0,                                              /* open loop mode */  
    MAGNETIC_ENCODER_MODE,                                              /* magnetic encoder mode */
    HALL_MODE,                                                          /* hall mode */
    HALL_DETECT_MODE,                                                   /* hall detect mode */
    SMO_MODE,                                                           /* smo mode */
    AFLUX_MODE,                                                         /* active flux observer mode */
    ENCODER_MODE,
    ENCODER_DETECT_MODE,
}angle_mode_enum;

/* angle srtuct definition */
/* angle struct */
typedef struct{
    /* angle and speed of rotor */
    angle_mode_enum pos_mode;                                           /* angle mode enum */
    float elec_angle;                                                   /* rotor electrical angle */
    float elec_speed;                                                   /* rotor electrical speed */
    float mech_angle;                                                   /* rotor mechanical angle */
    float mech_speed;                                                   /* rotor mechanical speed */

    /* angle sensor */
    hall_struct* hall;                                                  /* pointer to hall srtuct */
    encoder_struct* encoder;                                            /* pointer to encoder srtuct */
    vf_startup_struct* vf_startup;                                      /* pointer to vf start up struct */
    if_startup_struct* if_startup;                                      /* pointer to if start up struct */
    smo_struct *smo;                                                    /* pointer to smo struct */
    flux_struct *aflux;                                                 /* pointer to active flux observer struct */
}angle_struct;

/* function declaration */
/* initialize angle sensor */
void angle_init(angle_struct* angle,motor_struct* motor);
/** 主循环调用：把 UART4 读到的电角度(rad)写入，供 ANGLE_CUSTOM_MODE 下 FOC 使用 */
void angle_set_elec_angle_from_uart4(float elec_angle_rad);
/* get angle of motor */
void angle_get(angle_struct* angle, motor_struct* motor);
/* get speed of motor */
void speed_get(angle_struct* angle, motor_struct* motor);
/* angle sensor interrupt handler */
void angle_sensor_interrupt_handler(angle_struct* angle, motor_struct* motor);


#endif /* ANGLE_H */
