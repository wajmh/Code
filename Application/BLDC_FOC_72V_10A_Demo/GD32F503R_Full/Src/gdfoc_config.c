/*!
    \file    gdfoc_config.c
    \brief   configure FOC control algorithm

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

#include "gdfoc_config.h"
#include "motor_ctrl_driver.h"
#include "pwm_current_1shunt.h"
#include "pwm_current_2shunt.h"
#include "pwm_current_3shunt.h"
#include "vf_if_startup.h"
#include "angle_encoder.h"

/* PID configuration */
/* PID for speed control */
pid_struct speed_pid = {
    .kp_def = SPEED_LOOP_KP,                                                                /* the proportional factor */
    .ki_def = SPEED_LOOP_KI,                                                                /* the integral factor */
    .kd_def = 0,                                                                            /* the differential factor */
    .integral_sum_upper_limit_def = SPEED_LOOP_SUM_MAX,                                     /* the upper limit of integral term */
    .integral_sum_lower_limit_def = SPEED_LOOP_SUM_MIN,                                     /* the lower limit of integral term */
    .output_upper_limit_def = SPEED_LOOP_UOTPUT_MAX,                                        /* the upper limit of ouput */
    .output_lower_limit_def = SPEED_LOOP_UOTPUT_MIN,                                        /* the lower limit of ouput */
};
/* PID for torque control */
pid_struct torque_pid = {
    .kp_def = CURRENT_LOOP_KP,                                                              /* the proportional factor */
    .ki_def = CURRENT_LOOP_KI,                                                              /* the integral factor */
    .kd_def = 0,                                                                            /* the differential factor */
    .integral_sum_upper_limit_def = CURRENT_LOOP_SUM_MAX,                                   /* the upper limit of integral term */
    .integral_sum_lower_limit_def = CURRENT_LOOP_SUM_MIN,                                   /* the lower limit of integral term */
    .output_upper_limit_def = CURRENT_LOOP_UOTPUT_MAX,                                      /* the upper limit of ouput */
    .output_lower_limit_def = CURRENT_LOOP_UOTPUT_MIN,                                      /* the lower limit of ouput */
};
/* PID for flux control */
pid_struct flux_pid = {
    .kp_def = CURRENT_LOOP_KP,                                                              /* the proportional factor */
    .ki_def = CURRENT_LOOP_KI,                                                              /* the integral factor */
    .kd_def = 0,                                                                            /* the differential factor */
    .integral_sum_upper_limit_def = CURRENT_LOOP_SUM_MAX,                                   /* the upper limit of integral term */
    .integral_sum_lower_limit_def = CURRENT_LOOP_SUM_MIN,                                   /* the lower limit of integral term */
    .output_upper_limit_def = CURRENT_LOOP_UOTPUT_MAX,                                      /* the upper limit of ouput */
    .output_lower_limit_def = CURRENT_LOOP_UOTPUT_MIN,                                      /* the lower limit of ouput */
};

/* PID for smo pll control */
pid_struct smo_pll_pid = {
    .kp_def = SMO_PLL_KP,                                                                   /* the proportional factor */
    .ki_def = SMO_PLL_KI,                                                                   /* the integral factor */
    .kd_def = 0,                                                                            /* the differential factor */
    .integral_sum_upper_limit_def = SMO_PLL_SUM_MAX,                                        /* the upper limit of integral term */
    .integral_sum_lower_limit_def = SMO_PLL_SUM_MIN,                                        /* the lower limit of integral term */
    .output_upper_limit_def = SMO_PLL_OUTPUT_MAX,                                           /* the upper limit of ouput */
    .output_lower_limit_def = SMO_PLL_OUTPUT_MIN,                                           /* the lower limit of ouput */
};

/* PID for active flux observer control */
pid_struct alpha_pll_pid = {
    .kp_def = FLUX_PLL_KP,
    .ki_def = FLUX_PLL_KI,
    .kd_def = 0,
    
    .integral_sum_upper_limit = FLUX_PLL_SUM_MAX,
    .integral_sum_lower_limit = FLUX_PLL_SUM_MIN,
    .output_upper_limit_def = FLUX_PLL_OUTPUT_MAX,
    .output_lower_limit_def = FLUX_PLL_OUTPUT_MIN,
};

pid_struct beta_pll_pid = {
    .kp_def = FLUX_PLL_KP,
    .ki_def = FLUX_PLL_KI,
    .kd_def = 0,
    
    .integral_sum_upper_limit = FLUX_PLL_SUM_MAX,
    .integral_sum_lower_limit = FLUX_PLL_SUM_MIN,
    .output_upper_limit_def = FLUX_PLL_OUTPUT_MAX,
    .output_lower_limit_def = FLUX_PLL_OUTPUT_MIN,
};

vf_startup_struct vf_startup ={
    .period_def = VF_PERIOD,                                                                /* period value definition */
    .start_uq_def = VF_START_UQ,                                                            /* q-axis voltage definition at start */
    .end_uq_def = VF_END_UQ,                                                                /* q-axis voltage definition at end */
    .start_angular_speed_def = VF_START_ANGULAR_SPEED,                                      /* the angular speed definition at start time */
    .end_angular_speed_def = VF_END_ANGULAR_SPEED,                                          /* the angular speed definition at end time */
    .vf_start_up = vf_start_up,                                                             /* function pointer */
    .vf_start_up_init = vf_start_up_init,                                                   /* function pointer */
};

/* IF open-loop PI parameter configuration */
pid_struct if_startup_pid = {
    .kp_def = IF_STARTUP_LOOP_KP,                                                           /* the proportional factor */
    .ki_def = IF_STARTUP_LOOP_KI,                                                           /* the integral factor */
    .kd_def = 0,                                                                            /* the differential factor */
    .integral_sum_upper_limit_def = IF_STARTUP_LOOP_SUM_MAX,                                /* the upper limit of integral term */
    .integral_sum_lower_limit_def = IF_STARTUP_LOOP_SUM_MIN,                                /* the lower limit of integral term */
    .output_upper_limit_def = IF_STARTUP_LOOP_OUTPUT_MAX,                                   /* the upper limit of ouput */
    .output_lower_limit_def = IF_STARTUP_LOOP_OUTPUT_MIN,                                   /* the lower limit of ouput */
};

/* IF open-loop startup parameter configuration */
if_startup_struct if_startup ={
    .stage_one_period_def = IF_STAGE_ONE_PERIOD,                                            /* period value definition of stage one, uint: PWM cycle */
    .stage_two_period_def = IF_STAGE_TWO_PERIOD,                                            /* period value definition of stage two, uint: PWM cycle */
    .start_torque_def = IF_START_TORQUE,                                                    /* torque definition at start time */
    .end_torque_def = IF_END_TORQUE,                                                        /* torque definition at end time */
    .end_angular_speed_def = IF_END_ANGULAR_SPEED,                                          /* the angular speed definition at end time */
    .power_phase_def = IF_POWER_PHASE,                                                      /* the power phase definition between the observer phase and the open-loop phase */
    .phase_factor_def = IF_PHASE_FACTOR,                                                    /* the factor between the observer phase and the open-loop phase */
    .if_startup_pid = &if_startup_pid,                                                      /* pointer to a pid struct for if start up control */
    .if_start_up = if_start_up,                                                             /* function pointer */
    .if_start_up_init = if_start_up_init,                                                   /* function pointer */

};

/* motor control */
motor_struct motor = {
    .rs = RS,                                                                               /* equivalent stator resistance in alpha-beta axis, ohm*/
    .ld = LD,                                                                               /* equivalent stator inductance in d axis, H */
    .lq = LQ,                                                                               /* equivalent stator inductance in q axis, H */
    .flux = FLUX,                                                                           /* PMSM permanent flux, Wb */
    .pwm_frequency = PWM_FREQUENCE,                                                         /* PWM frequence, uint:Hz */
    .ts = 1.0f/PWM_FREQUENCE,                                                               /* PWM period, uint:s */
    .number_of_pairs = NUMBER_OF_PAIRS,                                                     /* PMSM number of pair poles */
    .running_mode = OPENLOOP_IF,                                                            /* motor running mode */
    .direction = DIRECTION_CCW,                                                             /* the direction of motor */
    .us_base = US_BASE,                                                                     /* voltage base value, 2/3*Udc, which Udc */
    .us_out_max = US_BASE * US_OUT_MAX_RATIO,                                               /* maximum reference voltage,uint:V */
    .speed_ref = SPEED_REFERENCE_MIN,                                                       /* reference value of speed */
    .torque_pid = &torque_pid,                                                              /* pointer to a pid struct for torque control */
    .flux_pid = &flux_pid,                                                                  /* pointer to a pid struct for flux control */
    .speed_pid = &speed_pid,                                                                /* pointer to a pid struct for speed control */
    .motor_start = start_ctrl,                                                              /* function pointer */
    .motor_stop = stop_ctrl,                                                                /* function pointer */
    .motor_break = break_ctrl,
    .motor_id = 1U,                                                             /* function pointer */
};

/* smo observer parameter configuration */
smo_struct smo = {
    .mode = SMO_PLL,                                                                       /* SMO mode, include atan and PLL */
    .gain_limit = SMO_GAIN,                                                                 /* limit the gain of SMO between -gain_limit and gain_limit */
    .error_threshold = SMO_THRESHOLD,                                                       /* error threshold between real current and observed current by SMO */
    .smo_pll_pid = &smo_pll_pid,                                                            /* pointer to a pid struct for speed control */
    .k1 = 0.0f,
    .k2 = 0.0f,
    .elec_speed_sum = 0,
    .elec_speed_index = 0,
    .elec_speed_buffer_length = SMO_SPEED_BUFFER_SIZE,                                      /* smo speed buffer size */
    .speed_factor = 60.0f/NUMBER_OF_PAIRS/MOTOR_PI_2,                                       /* factor for converting electrical speed to mechanical speed */
    .lpf_cutoff_mech_speed = MOTOR_NOMINAL_SPEED_RPM,                                       /* cut off speed for calculte cutoff frequence */
    .lpf_factor = LPF_FACTOR,
    .smo_observer_update = smo_observer_update,                                             /* function pointer */
};

/* active flux observer parameter configuration */
flux_struct aflux = {
    .alpha_pll_pid = &alpha_pll_pid,                                                        /* pointer to pid_struct for alpha-axis flux control */
    .beta_pll_pid = &beta_pll_pid,                                                          /* pointer to pid_struct for beta-axis flux control */
    .flux_observer_update = flux_observer_update,                                           /* function pointer */
};

/* angle feedback configuration */
/* HALL sensor */
hall_struct hall = {
    .hall_position_mode = HALL_POSITION_DETECT,                                             /* the hall position mode */
    .hall_peripheral_init = hall_peripheral_init,                                           /* function pointer */
    .hall_normal_var_init = hall_normal_var_init,                                           /* function pointer */
    .hall_angle_calc = hall_angle_calc,                                                     /* function pointer */
    .hall_speed_calc = hall_speed_calc,                                                     /* function pointer */
    .hall_detect_var_init = hall_detect_var_init,                                           /* function pointer */
    .hall_angle_detect = hall_angle_detect,                                                 /* function pointer */
    .hall_sensor_interrupt_handler = hall_sensor_interrupt_handler,                         /* function pointer */
};

/* Encoder */
encoder_struct encoder = {
    .mech_speed = 0,                                                                        /* mechanical speed, r/min */
    .elec_speed = 0,                                                                        /* electric speed , rad/s */
    .phase = 0,                                                                             /* the estimated spatial angle */
    .phase_shift = 0.00f,                                                                   /* the phase shift of encoder */
    .flag = 0,                                                                              /* the flag of encoder */
    .encoder_running_mode = ENCODER_RUN_NORMAL,                                             /* the running mode of the encoder */
    .encoder_count = 0,                                                                     /* counter for control ENCODER_RUN_DETECT */
    
    .encoder_peripheral_init = encoder_sensor_peripheral_init,                              /* function pointer */
    .encoder_angle_calc = encoder_sensor_angle_calc,                                        /* function pointer */
    .encoder_speed_calc = encoder_sensor_speed_calc,                                        /* function pointer */
    .encoder_detect_var_init = encoder_sensor_detect_var_init,                              /* function pointer */
    .encoder_var_init = encoder_sensor_var_init,                                            /* function pointer */
    .encoder_angle_detect = encoder_sensor_angle_detect,                                    /* function pointer */
    .encoder_interrupt_handler = encoder_sensor_interrupt_handler,                          /* function pointer */
};

/* angle sensor configuration */
angle_struct rotor_angle = {
    .pos_mode = ANGLE_CUSTOM_MODE,                                                          /* angle mode */
    .hall = &hall,                                                                          /* pointer to hall srtuct */
    .encoder = &encoder,                                                                    /* pointer to encoder srtuct */
    .vf_startup = &vf_startup,                                                              /* pointer to vf start up struct */
    .if_startup = &if_startup,                                                              /* pointer to if start up struct */
    .smo = &smo,                                                                            /* pointer to smo struct */
    .aflux = &aflux,                                                                        /* pointer to active flux observer struct */
    
};

/* current sampling and pwm configuration */
pwm_current_struct pwm_current = {
    .samp_mode = CURRENT_3SHUNT,                                                            /* phase current sampling mode */
    /* init ADC peripheral for current sample */
    .shunt3_init = pwm_current_3shunt_peripheral_init,
    .shunt2_init = pwm_current_2shunt_peripheral_init,
    .shunt1_init = pwm_current_1shunt_peripheral_init,
    /* get current iab */
    .shunt3_get_iab = pwm_current_3shunt_get_current,
    .shunt2_get_iab = pwm_current_2shunt_get_current,
    .shunt1_get_iab = pwm_current_1shunt_get_current,
    /* config current sample point */
    .config_sample_3s = pwm_current_3shunt_sampling_point_config,
    .config_sample_2s = pwm_current_2shunt_sampling_point_config,
    .config_sample_1s = pwm_current_1shunt_sampling_point_config,
    
    /* PWM modulation */
    .pwm_mode = CSVPWM,                                                                     /* the mode of the modulation: CSVPWM, DSVPWMMIN */
    .modulation.pwm_top = PWM_COUNT,                                                        /* the TIMER period value corresponding to PWM frequence */
    .update_duty_3s = pwm_3shunt_update_duty,
    .update_duty_2s = pwm_2shunt_update_duty,
};
