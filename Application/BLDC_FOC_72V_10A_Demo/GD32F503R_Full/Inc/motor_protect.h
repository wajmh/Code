/*!
    \file    motor_protect.h
    \brief   configure software protection parameters

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

#ifndef MOTOR_PROTECT_H
#define MOTOR_PROTECT_H

#include "motor.h"

/* macro definition */
/* bus voltage protect parameters */
#define BUS_VOLTAGE_CALC_FACTOR               (3.3f / (4096 * BUS_VOLTAGE_AMP_GAIN))                                /* the scale factor of calculate the real value of the motor bus voltage */
#define BUS_VOLTAGE_THRESHOLD_LOW             (16.0f)                                                               /* the low threshold value of bus voltage, uint: volt */
#define BUS_VOLTAGE_THRESHOLD_HIGH            (50.0f)                                                               /* the high threshold value of bus voltage, uint: volt */
#define VOLTAGE_ERROR_COUNT_MAX               (250U)                                                                /* the maximum value of voltage error count, uint: ms */
#define VOLTAGE_ERROR_RECOV_COUNT_MAX         (1000U)                                                               /* the maximum value of voltage error recovery count, uint: ms */

/* phase current protect parameters */
#define PHASE_CURRENT_THRESHOLD_HIGH          (20.0f)                                                                /* the high threshold value of phase current, uint: ampere */
#define OVER_CURRENT_COUNT_MAX                (250U)                                                                /* the maximum value of over current count, uint: ms */
#define OVER_CURRENT_RECOV_COUNT_MAX          (1000U)                                                               /* the maximum value of over current recovery count, uint: ms */

/* motor locked protect parameters */
#define START_TIME_SET_FACTOR                 (0.75f)                                                               /* the scale factor of start time and reference speed ,uint: ms/rpm */
#define MOTOR_LOCKED_VOLT_RATIO               (0.75f)                                                               /* the scale factor of voltage threshold and phase voltage */
#define MOTOR_LOCKED_COUNT_MAX                (150U)                                                                /* the maximum value of motor locked count, uint: ms */
#define MOTOR_LOCKED_RECOV_COUNT_MAX          (1000U)                                                               /* the maximum value of motor locked recovery count, uint: ms */

/* speed error protect parameters */
#define SPEED_THRESHOLD_LOW                   (0U)                                                                  /* the low threshold value of mechanical speed, uint: rpm */
#define SPEED_THRESHOLD_HIGH                  (6000U)                                                               /* the high threshold value of mechanical speed, uint: rpm */
#define SPEED_ERROR_COUNT_MAX                 (250U)                                                                /* the maximum value of speed error count, uint: ms */
#define SPEED_ERROR_RECOV_COUNT_MAX           (1000U)                                                               /* the maximum value of speed error recovery count, uint: ms */

/* phase loss protect parameters */
#define LOW_CURRENT_THRESHOLD                 (0.08f)                                                               /* the low threshold value of phase current, about 40% of the minimum phase current amplitude, uint: ampere */
#define PHASE_LOSS_COUNT_MAX                  (PWM_FREQUENCE * 0.5f)                                                /* the maximum value of pwm cycle */
#define PHASE_LOSS_COUNT_THRESHOLD            (PHASE_LOSS_COUNT_MAX * 0.90f)                                        /* When low_current_count is higher than PHASE_LOSS_COUNT_THRESHOLD, motor phase loss fault */

/* srtuct definition */
/* motor protect struct */
typedef struct {
    float v_bus;                                     /* the bus voltage value, uint: V */
    float i_phase_amplit;                            /* the amplitude of phase current, uint: A */
    float v_phase_amplit;                            /* the amplitude of phase voltage, uint: V */
    uint16_t fault_iteration[5];                     /* motor fault counts */
    uint16_t fault_recovery[5];                      /* motor fault recovery counts */
    int16_t start_time;                              /* motor locked fault: motor start time, uint: ms */
    float pre_speed_ref;                             /* motor locked fault: last speed reference value, uint: ms */
    uint16_t pwm_count;                              /* motor phase loss fault: the count value of pwm cycle */
    uint16_t low_current_count[3];                   /* motor phase loss fault: the count value of the phase current less than LOW_CURRENT_THRESHOLD */
} motor_protect_struct;
extern motor_protect_struct motor_protect;
/* function declaration */
/* initialize the motor protect peripherals */
void motor_protect_peripheral_init(void);
/* initialize the motor protect variables */
void motor_protect_var_init(void);
/* motor protect process */
void motor_protect_check(void);
/* check whether the motor phase loss */
void motor_protect_phase_loss_check(void);

#endif /* MOTOR_PROTECT_H */
