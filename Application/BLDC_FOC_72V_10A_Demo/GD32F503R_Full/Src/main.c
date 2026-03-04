/*!
    \file    main.c
    \brief   initialize the board, motor running state control

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

#include "main.h"
#include <stdio.h>
#include "systick.h"
#include "gd32_bldc_foc_eval.h"
#include "gdfoc_config.h"
#include "communication.h"
#include "motor_ctrl.h"
#include "smo_observer.h"
#include "motor_protect.h"
#include "Uart4_drv.h"
#include "angle.h"
/* motor state scan */
static void motor_state_machine(void);
/* motor running mode process */
static void motor_running_mode_process(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)

{
    /* enable TIMER behavior when the mcu is in debug mode */
    dbg_periph_enable(DBG_TIMER0_HOLD);
    dbg_periph_enable(DBG_TIMER2_HOLD);
    /* configure systick */
    systick_config();
    /* initialize the hardware of communication function */
    comm_init();
    /* initialize board */
    eval_board_init();
    uart4_init(2500000);
    
    while(1){
        /* motor running mode process */
        motor_running_mode_process();

        /* oled flashing routine */
//        oled_display();
        /* key process */
        key_process();
        /* motor state scan */
        motor_state_machine();
        /* parse the received data by the usart */
//        comm_usart_receive_data_parse();
        /* the LED indicates the current state */
        led_sacn_state();
        /* speed limit */
//        if(motor.speed_ref > SPEED_REFERENCE_MAX){
//            motor.speed_ref = SPEED_REFERENCE_MAX;
//        }else if(motor.speed_ref <= SPEED_REFERENCE_MIN){
//            motor.speed_ref = SPEED_REFERENCE_MIN;
//        }else{
//        }
			  printf("%f,%f,%f,%f,%f,%f\n",motor.iq,motor.iq_ref,motor.ia,motor.ib,motor.ic,motor_protect.v_bus);
        
    }
}

/*!
    \brief      motor state scan
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_state_machine(void)
{
    /* check whether a fault occurs */
    if(motor.fault != FAULT_NONE){
        motor.state = MC_STATE_FAULT;
    }

    switch(motor.state){
        case MC_STATE_INIT:
            motor_init(&motor);
            pwm_current_peripheral_init(&pwm_current, &motor);
            motor_protect_peripheral_init();
            motor_protect_var_init();
            motor.state = MC_STATE_IDLE;
            motor.command = MC_NONE;
            break;

        case MC_STATE_IDLE:{
            if(motor.command == MC_DIR_CHANGE){
                motor.direction = motor.direction_set;
                motor.command = MC_NONE;
            }
            /* FUN key is pressed */
            if(motor.command == MC_START){
                pid_init(&speed_pid);
                pid_init(&torque_pid);
                pid_init(&flux_pid);
                motor_protect_var_init();
                angle_init(&rotor_angle, &motor);
                motor.state = MC_STATE_RUNNING;
                motor_start(&motor);
                motor.command = MC_NONE;
            }
            break;
        }
        
        case MC_STATE_RUNNING: {
            /* FUN key is pressed */
            if(motor.command == MC_STOP){
                motor.state = MC_STATE_INIT;
                motor_stop(&motor);
                motor.command = MC_NONE;
                delay_1ms(100U);
            }
            /* FUN key is pressed */
            if(motor.command == MC_BREAK){
                motor.state = MC_STATE_BRAKE;
                motor_break(&motor);
                motor.command = MC_NONE;
            }
            break;
        }

        case MC_STATE_BRAKE: {
            /* FUN key is pressed */
            if(motor.command == MC_BREAK){
                motor_stop(&motor);
                motor.state = MC_STATE_INIT;
                motor.command = MC_NONE;
            }
            break;
        }

        case MC_STATE_FAULT: {
            motor_stop(&motor);
            /* check whether a fault occurs */
            if(motor.fault == FAULT_NONE){
                motor.state = MC_STATE_INIT;
            }
            break;
        }

        default:
            break;
    }
}

/*!
    \brief      motor running mode process
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_running_mode_process(void)
{
    static mc_running_mode_enum running_mode_pre = OPENLOOP_IF;
    static uint8_t first_flag = 1;
    
    if((first_flag == 1) || (running_mode_pre != motor.running_mode)){
        first_flag = 0;
        if(OPENLOOP_VF == motor.running_mode ){
            rotor_angle.pos_mode = ANGLE_CUSTOM_MODE ;
            motor.udq_mode = UDQ_CUSTOM;
            motor.motor_openloop.d_var = 5.0f;
            motor.motor_openloop.q_var = 0.0f;
        }else if(OPENLOOP_IF == motor.running_mode){
            rotor_angle.pos_mode = ANGLE_CUSTOM_MODE ;
            motor.udq_mode = UDQ_MTPA_IF;
            motor.idqref_mode = IDQ_CUSTOM;
            motor.id_ref = 0.0f;
            motor.iq_ref = 0.0f;
        }else if(SMO_VF == motor.running_mode ){
            rotor_angle.pos_mode = SMO_MODE;
            motor.startup_mode = VF_STARTUP;
            motor.udq_mode = UDQ_OBSERVER;
            motor.idqref_mode = IDQ_SPEED_OBSERVER;
        }else if(SMO_IF == motor.running_mode ){
            rotor_angle.pos_mode = SMO_MODE ;
            motor.startup_mode = IF_STARTUP;
            motor.udq_mode = UDQ_OBSERVER;
            motor.idqref_mode = IDQ_SPEED_OBSERVER;
        }else if(AFLUX_VF == motor.running_mode ){
            rotor_angle.pos_mode = AFLUX_MODE;
            motor.startup_mode = VF_STARTUP;
            motor.udq_mode = UDQ_OBSERVER;
            motor.idqref_mode = IDQ_SPEED_OBSERVER;
        }else if(AFLUX_IF == motor.running_mode ){
            rotor_angle.pos_mode = AFLUX_MODE ;
            motor.startup_mode = IF_STARTUP;
            motor.udq_mode = UDQ_OBSERVER;
            motor.idqref_mode = IDQ_SPEED_OBSERVER;
        }else if(ENCODER_DETECT == motor.running_mode ){
            motor.udq_mode = UDQ_CUSTOM;
            rotor_angle.pos_mode = ENCODER_DETECT_MODE;
        }else if(ENCODER == motor.running_mode ){
            rotor_angle.pos_mode = ENCODER_MODE ;
            motor.udq_mode = UDQ_MTPA_IF;
            motor.idqref_mode = IDQ_SPEED_MTPA;
            motor.startup_mode = IF_STARTUP;
        }else if(HALL_SENSOR_DETECT == motor.running_mode){
            rotor_angle.pos_mode = HALL_DETECT_MODE ;
            motor.udq_mode = UDQ_CUSTOM;
        }else if(HALL_SENSOR == motor.running_mode){
            rotor_angle.pos_mode = HALL_MODE ;
            motor.udq_mode = UDQ_MTPA_IF;
            motor.idqref_mode = IDQ_SPEED_MTPA;
        }
    }
    running_mode_pre = motor.running_mode;
}
int fputc(int ch, FILE *f)
{
    usart_data_transmit(COMM_USART, (uint8_t)ch);
    while(RESET == usart_flag_get(COMM_USART, USART_FLAG_TBE)) {
    }
    return ch;
}
