/*!
    \file    angle.c
    \brief   invoke motor rotor angle related functions

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

#include "angle.h"

/* macro definition */
#define POS_ANGLE_OFFSET_NUM                (uint16_t)(10000)  /* max count in offset mode */

/* 主循环里 uart4_get_elec_angle_rad 读到的电角度写到这里，FOC 中断里 ANGLE_CUSTOM_MODE 只读此变量 */
static volatile float angle_elec_from_uart4_rad = 0.0f;

void angle_set_elec_angle_from_uart4(float elec_angle_rad)
{
    angle_elec_from_uart4_rad = elec_angle_rad;
}

/*!
    \brief      initialize angle sensor
    \param[in]  angle: pointer to angle srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void angle_init(angle_struct* angle,motor_struct* motor)
{
    if(HALL_MODE == angle->pos_mode){
        angle->hall->hall_running_mode = HALL_RUN_NORMAL;
        /* initialize hall normal mode peripherals and variables  */
        angle->hall->hall_peripheral_init();
        angle->hall->hall_normal_var_init((angle->hall), motor);
    }else if(HALL_DETECT_MODE == angle->pos_mode){
        angle->hall->hall_running_mode = HALL_RUN_DETECT;
        /* initialize hall detect mode peripherals and variables */
        angle->hall->hall_peripheral_init();
        angle->hall->hall_detect_var_init((angle->hall), motor);
    }else if(SMO_MODE == angle->pos_mode){
        /* initialize smo pll pid */
        pid_init(angle->smo->smo_pll_pid);
        /* initialize start up */
        if(VF_STARTUP == motor->startup_mode){
            angle->vf_startup->vf_start_up_init(motor,angle->vf_startup);
        }else if(IF_STARTUP == motor->startup_mode){
            angle->if_startup->if_start_up_init(motor,angle->if_startup);
        }else{
        }
    }else if(AFLUX_MODE == angle->pos_mode){
        /* initialize active flux observer */
        pid_init(angle->aflux->alpha_pll_pid);
        pid_init(angle->aflux->beta_pll_pid);
        flux_observer_init(motor, angle->aflux);
        /* initialize start up */
        if(VF_STARTUP == motor->startup_mode){
            angle->vf_startup->vf_start_up_init(motor,angle->vf_startup);
        }else if(IF_STARTUP == motor->startup_mode){
            angle->if_startup->if_start_up_init(motor,angle->if_startup);
        }else{
        }
    }else if(ENCODER_DETECT_MODE == angle->pos_mode){
        angle->encoder->encoder_running_mode = ENCODER_RUN_DETECT;
        /* initialize encoder detect mode peripherals and variables  */
        angle->encoder->encoder_peripheral_init();
        angle->encoder->encoder_detect_var_init((angle->encoder), motor);
    }else if(ENCODER_MODE == angle->pos_mode){
        angle->encoder->encoder_running_mode = ENCODER_RUN_NORMAL;
        /* initialize encoder mode peripherals and variables  */
        angle->encoder->encoder_peripheral_init();
        angle->encoder->encoder_var_init((angle->encoder), motor);
        angle->if_startup->if_start_up_init(motor,angle->if_startup);
    }
}

/*!
    \brief      get angle of motor
    \param[in]  angle: pointer to angle srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void angle_get(angle_struct* angle, motor_struct* motor)
{
    if(ANGLE_CUSTOM_MODE == angle->pos_mode){
        /* 电角度由主循环读取 UART4 后通过 angle_set_elec_angle_from_uart4() 写入，此处只读 */
        angle->elec_angle = angle_elec_from_uart4_rad;
    }else if(HALL_MODE ==angle->pos_mode){
        angle->hall->hall_angle_calc((angle->hall),motor);
        angle->elec_angle = angle->hall->phase;
    }else if(HALL_DETECT_MODE ==angle->pos_mode){
        /* calculate offset angle  */
        angle->hall-> hall_angle_detect((angle->hall),motor);
        angle->elec_angle = angle->hall->phase;
    }else if(SMO_MODE ==angle->pos_mode){
        /* run SMO, which is called in each sampling period */
        angle->smo->smo_observer_update(motor,angle->smo);
        if(VF_STARTUP == motor->startup_mode){
            if(motor->vf_startup.close_loop_flag == 0){
                /* vf start up algorithm, which is called in each sampling period */
                angle->vf_startup->vf_start_up(motor,angle->vf_startup);
                /* the electrical angle is given by vf start up */
                angle->elec_angle = angle->vf_startup->phase_reference;
            }
            else{
                /* the electrical angle is given by smo */
                angle->elec_angle = angle->smo->phase;
            }
        }else if(IF_STARTUP ==motor->startup_mode){
            if(motor->if_startup.close_loop_flag == 0){
                /* if start up algorithm, which is called in each sampling period */
                angle->if_startup->if_start_up(motor,angle->if_startup);
                angle->if_startup->observer_phase = angle->smo->phase;
                angle->elec_angle = angle->if_startup->phase_reference;
            }else{
                angle->elec_angle = angle->smo->phase;
            }
        }
    }else if(AFLUX_MODE ==angle->pos_mode){
        /* run active flux observer, which is called in each sampling period */
        angle->aflux->flux_observer_update(motor,angle->aflux);
        if(VF_STARTUP == motor->startup_mode){
            if(motor->vf_startup.close_loop_flag == 0){
                /* vf start up algorithm, which is called in each sampling period */
                angle->vf_startup->vf_start_up(motor,angle->vf_startup);
                /* the electrical angle is given by vf start up */
                angle->elec_angle = angle->vf_startup->phase_reference;
            }
            else{
                /* the electrical angle is given by smo */
                angle->elec_angle = angle->aflux->phase;
            }
        }else if(IF_STARTUP ==motor->startup_mode){
            if(motor->if_startup.close_loop_flag == 0){
                /* if start up algorithm, which is called in each sampling period */
                angle->if_startup->if_start_up(motor,angle->if_startup);
                angle->if_startup->observer_phase = angle->aflux->phase;
                angle->elec_angle = angle->if_startup->phase_reference;
            }else{
                angle->elec_angle = angle->aflux->phase;
            }
        }else{
        }
    }else if(ENCODER_MODE == angle->pos_mode){
        if(IF_STARTUP == motor->startup_mode){
            if(motor->if_startup.close_loop_flag == 0){
                /* vf start up algorithm, which is called in each sampling period */
                angle->if_startup->if_start_up(motor,angle->if_startup);
                /* the electrical angle is given by vf start up */
                angle->elec_angle = angle->if_startup->phase_reference;
                /* the electrical angle is given by encoder */
                angle->encoder->encoder_angle_calc((angle->encoder),motor);
                angle->if_startup->observer_phase = angle->encoder->phase;
            }
            else{
                /* the electrical angle is given by encoder */
                angle->encoder->encoder_angle_calc((angle->encoder),motor);
                angle->elec_angle = angle->encoder->phase;
            }
        }
    }else if(ENCODER_DETECT_MODE == angle->pos_mode){
        /* angle in open loop */
        int16_t num;
        if(angle->encoder->encoder_count < 500){
            angle->encoder->encoder_count++;
            motor->motor_openloop.angle_var = 0.0f;
        }else if(angle->encoder->encoder_count >= 500){
            motor->motor_openloop.angle_var += ENCODER_DETECT_ANGLE_INC;
        }
        num = (int16_t)((motor->motor_openloop.angle_var)/MOTOR_PI_2);
        if(motor->motor_openloop.angle_var >= 0){
            angle->elec_angle = (motor->motor_openloop.angle_var - num*MOTOR_PI_2);
        }else{
            
            angle->elec_angle = motor->motor_openloop.angle_var - (num-1)*MOTOR_PI_2;
        }
    }else{
            /* none */
    }

}

/*!
    \brief      get speed of motor
    \param[in]  angle: pointer to angle srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void speed_get(angle_struct* angle, motor_struct* motor)
{
    if(HALL_MODE ==angle->pos_mode){
        /* update motor speed */
        angle->mech_speed = angle->hall->hall_speed_calc();
        motor->mech_speed = angle->mech_speed;
    }else if(SMO_MODE ==angle->pos_mode){
        if(VF_STARTUP == motor->startup_mode){
            /* update motor speed,motor speed is smo speed */
            motor->mech_speed = angle->smo->mech_speed;
        }else if(IF_STARTUP == motor->startup_mode){
            /* update motor speed,motor speed is smo speed */
            motor->mech_speed = angle->smo->mech_speed;
        }else{
        }
    }else if(AFLUX_MODE ==angle->pos_mode){
        if(VF_STARTUP == motor->startup_mode){
            /* update motor speed,motor speed is active flux observer speed */
            motor->mech_speed = angle->aflux->mech_speed;
        }else if(IF_STARTUP == motor->startup_mode){
            /* update motor speed,motor speed is active flux observer speed */
            motor->mech_speed = angle->aflux->mech_speed;
        }else{
        }
    }else if(ENCODER_MODE == angle->pos_mode){
        /* update motor speed */
        angle->mech_speed = angle->encoder->mech_speed;
        motor->mech_speed = angle->mech_speed;
    }
}

/*!
    \brief      angle sensor interrupt handler
    \param[in]  angle: pointer to angle srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void angle_sensor_interrupt_handler(angle_struct* angle, motor_struct* motor)
{
    if((HALL_MODE == angle->pos_mode)||(HALL_DETECT_MODE == angle->pos_mode)){
        angle->hall->hall_sensor_interrupt_handler((angle->hall),motor);
    }else if((ENCODER_MODE == angle->pos_mode)||(ENCODER_DETECT_MODE == angle->pos_mode)){
        if(ENCODER_RUN_DETECT == angle->encoder->encoder_running_mode){
            static uint8_t encoder_detect_cnt = 0;
            encoder_detect_cnt++;
            if(encoder_detect_cnt > 1){
                angle->encoder->phase_shift = angle->elec_angle;
                angle->encoder->encoder_interrupt_handler((angle->encoder),motor);
                motor->motor_stop();
                motor->command = MC_STOP;
                encoder_detect_cnt = 0;
            }else{
                timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH2);
            }
        }
        else{
            angle->encoder->encoder_interrupt_handler((angle->encoder),motor);
        }
    }
}
