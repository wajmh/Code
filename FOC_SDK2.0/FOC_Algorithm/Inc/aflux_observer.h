/*!
    \file    aflux_observer.h
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

#ifndef AFLUX_OBSERVER_H
#define AFLUX_OBSERVER_H

#include "motor.h"

/* active flux observer parameters */
#define FLUX_PLL_KP                          (50.01f)               /* the proportional factor */
#define FLUX_PLL_KI                          (0.01f)                /* the integral factor */
#define FLUX_PLL_SUM_MAX                     (100.0f)               /* the upper limit of integral term */
#define FLUX_PLL_SUM_MIN                     (-100.0f)              /* the lower limit of integral term */
#define FLUX_PLL_OUTPUT_MAX                  (100.0f)               /* the upper limit of ouput */
#define FLUX_PLL_OUTPUT_MIN                  (-100.0f)              /* the lower limit of ouput */

/* constants definitions */
/* active flux struct definitions*/
typedef struct flux_struct{
    float i_d_observer;
    float i_q_observer;
    float phase_tmp;
    float phase;
    float phase_pre;
    float flux_d_observer;
    float flux_q_observer;
    float v_alpha;
    float v_beta;
    float speed_angular;
    float mech_speed;
    float flux_alpha_in;
    float flux_beta_in;
    
    float com_alpha;
    float com_beta;
    
    float flux_alpha_err;
    float flux_beta_err;
    
    float flux_alpha_observer;
    float flux_beta_observer;
    
    float flux_alpha_active;
    float flux_beta_active;
    
    float u_alpha_observer;
    float u_beta_observer;
    
    triangle_struct triangle;

    pid_struct* alpha_pll_pid;                                      /* pointer to pid_struct for alpha-axis flux control */
    pid_struct* beta_pll_pid;                                       /* pointer to pid_struct for beta-axis flux control */
    
    /* run the active flux observer, which is called in each sampling period */
    void (*flux_observer_update)(motor_struct *motor, struct flux_struct *active_flux);   /* function pointer */
}flux_struct;

/* function declaration */
void flux_observer_init(motor_struct *motor, flux_struct *active_flux);
/* run the active flux observer, which is called in each sampling period */
void flux_observer_update(motor_struct *motor, flux_struct *active_flux);
#endif /* AFLUX_OBSERVER_H */
