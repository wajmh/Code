/*!
    \file    smo_observer.h
    \brief   the header file of smo observer algorithm

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

#ifndef SMO_OBSERVER_H
#define SMO_OBSERVER_H

#include "motor.h"

/* smo observer parameters */
#define SMO_THRESHOLD                       (20.0f)               /* the threshold of smo */
#define SMO_GAIN                            (100.0f)              /* the gain of smo */
#define SMO_SPEED_BUFFER_SIZE               (100)                 /* smo speed buffer size */

#define MOTOR_NOMINAL_SPEED_RPM             (4000)                /* cut off speed for calculte cutoff frequence */
#define CUTOFF_FREQUENCE                    (float)(((MOTOR_NOMINAL_SPEED_RPM * NUMBER_OF_PAIRS)/60.0f*MOTOR_PI_2))
#define LPF_FACTOR                          (float)((CUTOFF_FREQUENCE/(CUTOFF_FREQUENCE + PWM_FREQUENCE)))

#define SMO_PLL_KP                          (0.01f)               /* the proportional factor */
#define SMO_PLL_KI                          (0.01f)               /* the integral factor */
#define SMO_PLL_SUM_MAX                     (15.0f)               /* the upper limit of integral term */
#define SMO_PLL_SUM_MIN                     (-15.0f)              /* the lower limit of integral term */
#define SMO_PLL_OUTPUT_MAX                  (15.0f)               /* the upper limit of ouput */
#define SMO_PLL_OUTPUT_MIN                  (-15.0f)              /* the lower limit of ouput */

/* enum definition */
/* the running mode of SMO */
typedef enum{
    SMO_ATAN = 1,                                                 /* using atan to estimate the speed and angle */
    SMO_PLL,                                                      /* using PLL to estimate the speed and angle */
}smo_mode_enum;

/* constants definitions */
/* SMO struct definitions*/
typedef struct smo_struct{
    uint8_t mode;                                                 /* SMO mode, include atan and PLL */
    float gain_limit;                                             /* limit the gain of SMO between -gain_limit and gain_limit */
    float error_threshold;                                        /* error threshold between real current and observed current by SMO */

    float k1;                                                     /* the gain of SMO in alpha axis */
    float k2;                                                     /* the gain of SMO in beta axis */

    float ialpha;                                                 /* observer current alpha */
    float ibeta;                                                  /* observer current beta */

    float ialpha_error;                                           /* error between real current and observed current by SMO in alpha axis */
    float ibeta_error;                                            /* error between real current and observed current by SMO in beta axis */

    float ialpha_next;                                            /* observer current alpha of the next period */
    float ibeta_next;                                             /* observer current beta of the next period */

    float bemf_alpha;                                             /* back electromotive force alpha */
    float bemf_beta;                                              /* back electromotive force beta */

    float bemf_alpha_next;                                        /* back electromotive force alpha of the next period */
    float bemf_beta_next;                                         /* back electromotive force beta of the next period */
    
    float mech_speed;                                             /* mechanical speed, r/min */
    float elec_speed;                                             /* electric speed , rad/s */
    float speed_factor;                                           /* factor for converting electrical speed to mechanical speed */
    
    float elec_speed_buffer[SMO_SPEED_BUFFER_SIZE];               /* electric speed filter array */
    float elec_speed_sum;                                         /* accumulated value of electric speed filter array */
    uint8_t elec_speed_index;                                     /* electric speed index */
    uint8_t elec_speed_buffer_length;                             /* electric speed buffer length */

    float lpf_cutoff_mech_speed;                                  /* section RC low-pass filter cutoff frequency corresponding to the mechanical speed */
    float lpf_factor;                                             /* section RC low-pass filter filter coefficient */

    float phase;                                                  /* compensated motor rotor phase */
    float phase_temp;                                             /* motor phase value, in SMO_PLL mode */
    float phase_compesation;                                      /* motor phase compensation value */
    float phase_previous;                                         /* previous motor phase, in SMO_ATAN mode */

    pid_struct* smo_pll_pid;                                      /* pointer to a pid struct for speed control */

    /* run the sliding-mode observer, which is called in each sampling period */
    void (*smo_observer_update)(motor_struct* motor,struct smo_struct *smo);     /* function pointer */
}smo_struct;

/* function declaration */
/* run the sliding-mode observer, which is called in each sampling period */
void smo_observer_update(motor_struct *motor, smo_struct *smo);
#endif /* SMO_OBSERVER_H */
