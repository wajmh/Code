/*!
    \file    control_para.h
    \brief   configure FOC control parameters

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

#ifndef CONTROL_PARA_H
#define CONTROL_PARA_H

/* system clock parameters */
#define SYSTEM_FREQUENCE                    (252000000U)                        /* system frequence, uint: Hz */
#define SYSTEM_FREQUENCE_MHz                (252U)                              /* system frequence, uint: MHz */
#define SYSTEM_ADC_MHz                      (42U)                               /* ADC frequence */
#define TS_NS                               (1000000000.0f/PWM_FREQUENCE)       /* sampling time in nanoseconds */

/* PWM parameters */
#define PWM_FREQUENCE                       (16000U)                                                    /* PWM frequence, uint:Hz */
#define PWM_COUNT                           (SYSTEM_FREQUENCE/PWM_FREQUENCE/2)                          /* the TIMER period value corresponding to PWM frequence */
#define US_BASE                             (32.0f)                                                     /* the base voltage of PWM modulation algorithm */

/* input reference parameters */
/* for speed mode */
#define SPEED_REFERENCE_MAX                 (3800U)                                                     /* the maximum value of the mechanical speed */
#define SPEED_REFERENCE_MIN                 (800U)                                                      /* the minimum value of the mechanical speed */
#define SPEED_REFERENCE_STEPPING            (20U)                                                       /* the speed stepping */
/* for current mode */
#define CURRENT_REFERENCE_MAX               (31000U)                                                    /* the maximum value of the current */
#define CURRENT_REFERENCE_MIN               (1500U)                                                     /* the minimum value of the current */
#define CURRENT_REFERENCE_STEPPING          (20U)                                                       /* the current stepping */

/* configure the MCU dead time */
#define DEAD_TIME                           0                                                           /* the dead time in TIMER*/ 

/* hardware circuit parameters */
#define TCHARGE_NS                          (400U)                                                      /* charge time */
#define TDEAD_NS                            (300U)                                                      /* the dead time in the hardware */
#define TADC_RISE_NS                        (1500U)                                                     /* the time between upper pwm out and adc channel input stable */

/* software sampling parameters */
#define ADC_SAMPLE_TIME                     (ADC_SAMPLETIME_1POINT5)                                    /* ADC sampling time configuration */
#define ADC_SAMPLE_TIME_ALL                 (1.5f)                                                      /* actual ADC sampling time */
#define TSAMPLE_NS                          (uint16_t)((ADC_SAMPLE_TIME_ALL+3)*1000/SYSTEM_ADC_MHz)     /* add trigger delay in ADC */
#define TWINDOW_NS                          (uint16_t)(TADC_RISE_NS+TSAMPLE_NS)                         /* PWM sampling window time */
#define TSAMPLE_POINT                       (float)((((TS_NS - TWINDOW_NS)/2) + TADC_RISE_NS)*2*PWM_COUNT/TS_NS) //3817
#define TSAMPLE_POINT_TIMER                 ((TSAMPLE_POINT) < (PWM_COUNT) ? (TSAMPLE_POINT) : (2*PWM_COUNT-TSAMPLE_POINT-1))
#define TSAMPLE_MODE_TIMER                  ((TSAMPLE_POINT) < (PWM_COUNT) ? (TIMER_OC_MODE_PWM1) : (TIMER_OC_MODE_PWM0))

/* 3 shunt csvpwm */
#define MI_SAMPLE_MAX_3SHUNT_CSVPWM         (float)(0.866f - 1.732f*TWINDOW_NS/TS_NS)
/* 3 shunt dsvpwm */
#define MI_SAMPLE_MAX_3SHUNT_DSVPWMMIN      (float)(0.866f - 0.866f*TWINDOW_NS/TS_NS)
/* 2 shunt csvpwm */
#define MI_SAMPLE_MAX_2SHUNT_CSVPWM         (float)(0.866f - 1.732f*TWINDOW_NS/TS_NS)
/* 2 shunt dsvpwm */
#define MI_SAMPLE_MAX_2SHUNT_DSVPWMMIN      (float)(0.866f - 0.866f*TWINDOW_NS/TS_NS)
/* 1 shunt csvpwm */
#define TIME_MARGIN                         (30U)                                                     /* the physical limitation */
#define MI_SAMPLE_MAX_1SHUNT_CSVPWM         (float)(1.0f - 4*(TWINDOW_NS + TIME_MARGIN*1000/SYSTEM_FREQUENCE_MHz)/TS_NS)
/* csvpwm */
#define MI_CHARGE_MAX_CSVPWM                (float)(0.866f - 1.732f*(TCHARGE_NS+TDEAD_NS)/TS_NS)
/* dsvpwmmin */
#define MI_CHARGE_MAX_DSVPWMMIN             (float)(0.866f - 0.866f*(TCHARGE_NS+TDEAD_NS)/TS_NS)
/* maximum voltage in linear region */
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#define US_OUT_MAX_RATIO                    (MIN(MI_SAMPLE_MAX_3SHUNT_CSVPWM, MI_CHARGE_MAX_CSVPWM))

/* pid parameter configuration */
#define SPEED_LOOP_KP                       (0.001f)                            /* the proportional paramter of speed regulator */
#define SPEED_LOOP_KI                       (0.000008f)                         /* the integral paramter of speed regulator */
#define SPEED_LOOP_SUM_MAX                  (2.0f)                              /* the integral paramter of speed regulator */
#define SPEED_LOOP_SUM_MIN                  (-2.0f)                             /* the integral paramter of speed regulator */
#define SPEED_LOOP_UOTPUT_MAX               (2.5f)                              /* the integral paramter of speed regulator */
#define SPEED_LOOP_UOTPUT_MIN               (-2.5f)                             /* the integral paramter of speed regulator */

//#define CURRENT_LOOP_KP                     (0.33f)
//#define CURRENT_LOOP_KI                     (0.0065f)                              /* the proportional paramter of speed regulator */
#define CURRENT_LOOP_KP                     (0.45f)
#define CURRENT_LOOP_KI                     (0.0065f)
//#define CURRENT_LOOP_SUM_MAX                (15.0f)                             /* the integral paramter of speed regulator */
//#define CURRENT_LOOP_SUM_MIN                (-15.0f)                            /* the integral paramter of speed regulator */
//#define CURRENT_LOOP_UOTPUT_MAX             (15.0f)                             /* the integral paramter of speed regulator */
//#define CURRENT_LOOP_UOTPUT_MIN             (-15.0f)                            /* the integral paramter of speed regulator */
#define CURRENT_LOOP_SUM_MAX                (24.0f)                             /* the integral paramter of speed regulator */
#define CURRENT_LOOP_SUM_MIN                (-24.0f)                            /* the integral paramter of speed regulator */
#define CURRENT_LOOP_UOTPUT_MAX             (24.0f)                             /* the integral paramter of speed regulator */
#define CURRENT_LOOP_UOTPUT_MIN             (-24.0f)                            /* the integral paramter of speed regulator */
#endif /* CONTROL_PARA_H */
