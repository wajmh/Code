/*!
    \file    gd32_foc_it.c
    \brief   FOC interrupt service routines

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

#include "gd32_foc_it.h"
#include "systick.h"
#include "main.h"
#include "gdfoc_config.h"
#include "gd32_bldc_foc_eval.h"
#include "communication.h"
#include "motor_ctrl.h"
#include "modulation.h"
#include "motor_protect.h"
#include "angle.h"
#include "Uart4_drv.h"
#include "Can_protocol.h"
#define SRAM_ECC_ERROR_HANDLE(s)    do{}while(1)

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    if(SET == syscfg_sram_ecc_flag_get(SYSCFG_SRAMECCSTAT_SRAMECCMEIF)) {
        SRAM_ECC_ERROR_HANDLE("SRAM non-correction event detected\r\n");
    } else if(SET == syscfg_sram_ecc_flag_get(SYSCFG_SRAMECCSTAT_SRAMECCSEIF)) {
        SRAM_ECC_ERROR_HANDLE("SRAM single bit correction event detected\r\n");
    } else { 
        /* if NMI exception occurs, go to infinite loop */
        /* HXTAL clock monitor NMI error or NMI pin error */
        while(1) {
        }
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
    /* if DebugMon exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)//每一毫秒调用一次
{
    /* delay decrement */
    delay_decrement();
    /* get speed of motor */
    speed_get(&rotor_angle,&motor);
    /* calculate the reference current of d-q */
    motor_idqref_calc(&motor);
    /* motor protect process */
//    motor_protect_check();
    /* stop motor if CAN heartbeat is lost */
    can_heartbeat_timeout_check();//can的心跳检测，丢失则停止电机，500ms丢失则报警
    /* transmit the motor state data using the usart */
//    comm_usart_motor_state_data_transmit();
}

/*!
    \brief      this function handles HALL_ENCODER TIMER exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HALL_ENCODER_TIMER_IRQHandler(void)
{
    angle_sensor_interrupt_handler(&rotor_angle,&motor);
}

/*!
    \brief      this function handles FOC control
    \param[in]  none
    \param[out] none
    \retval     none
*/
void FOC_CONTORL_IRQHandler(void)
{
    /* clear the interrupt flag */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    triangle_struct phase_sincos;
    
    get_iab(&pwm_current, &motor);
	  
    /* phase-loss detection disabled by project setting */
    // motor_protect_phase_loss_check();

    if (rotor_angle.pos_mode == ANGLE_CUSTOM_MODE) {///* ANGLE_CUSTOM_MODE：FOC 里只发一次 0x6A；编码器应答在 UART RX 里解析并更新电角度，此处直接用上次结果，无延时 */
        uart4_request_encoder_data();
        float elec_rad;
        uart4_get_last_elec_angle_rad(&elec_rad);
        angle_set_elec_angle_from_uart4(elec_rad);
    }

    angle_get(&rotor_angle,&motor);

    if (g_can_rx_new_flag) {
    g_can_rx_new_flag = 0;
    od_0x07FF_callback((can_frame_t *)&g_can_rx_message);
   }

    sin_cos_float(rotor_angle.elec_angle,&phase_sincos.sin,&phase_sincos.cos);
    clarke_amplitude(&motor.ialpha, &motor.ibeta, motor.ia, motor.ib);
    park(&motor.id, &motor.iq, motor.ialpha, motor.ibeta, phase_sincos);
    motor_protect_fast_check();//过流过压检测
    if(motor.fault != FAULT_NONE) {
        motor.state = MC_STATE_FAULT;
        motor.command = MC_NONE;
        motor_stop(&motor);
        return;
    }

    if((motor.state == MC_STATE_RUNNING) && (motor.running_mode == OPENLOOP_IF)) {
        motor_mit_iq_ref_update(&motor);
    }

    motor_udq_calc(&motor);
    
    over_modulation(&motor);
    
    rev_park(&motor.ualpha, &motor.ubeta, motor.ud, motor.uq, phase_sincos);
    
    pwm_generation(&pwm_current,&motor);
}

/*!
    \brief      this function handles EC11 A port and EC11 B port
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EC11_IRQ_HANDLE(void)
{
    /* Port A: DATA */
    if(exti_interrupt_flag_get(EC11_A_EXTI_LINE) != RESET){
        exti_interrupt_flag_clear(EC11_A_EXTI_LINE);
        ec_delay(10000);
        if(RESET == gpio_input_bit_get(EC11_A_GPIO_PORT, EC11_A_GPIO_PIN)){
            //DATA: falling edge
            if(A_FALLING == end_edge){
                end_edge = AB_NONE;
                return;
            }
            if(B_FALLING == pre_edge ){
                pre_edge = AB_NONE;
                /* orientation:CCW */
                ec_orientation = DIRECTION_DECREASE;
                ec11_handle();
                end_edge = A_FALLING;
            }else{
                pre_edge = A_FALLING;
            }
        }else{
            //DATA: rising
            if(A_RISING == end_edge){
                end_edge = AB_NONE;
                return;
            }
            if(B_RISING == pre_edge ){
                /* orientation:CCW */
                pre_edge = AB_NONE;
                ec_orientation = DIRECTION_DECREASE;
                ec11_handle();
                end_edge = A_RISING;
            }
            else{
                pre_edge = A_RISING;
            }  
        }
    }
    
    /* Port B: CLK */
    if(exti_interrupt_flag_get(EC11_B_EXTI_LINE) != RESET){
        exti_interrupt_flag_clear(EC11_B_EXTI_LINE);
        ec_delay(10000);
        if(RESET == gpio_input_bit_get(EC11_B_GPIO_PORT, EC11_B_GPIO_PIN)){
            //CLK: falling edge
            if(B_FALLING == end_edge){
                end_edge = AB_NONE;
                return;
            }
            if(A_FALLING == pre_edge ){
                pre_edge = AB_NONE;
                /* orientation:CW */
                ec_orientation = DIRECTION_INCREASE;
                ec11_handle();
                end_edge = B_FALLING;
            }else{
                pre_edge = B_FALLING;
            }
        }else{
            //CLK: rising
            if(B_RISING == end_edge){
                end_edge = AB_NONE;
                return;
            }
            if(A_RISING == pre_edge ){
                pre_edge = AB_NONE;
                /* orientation:CW */
                ec_orientation = DIRECTION_INCREASE;
                ec11_handle();
                end_edge = B_RISING;
            }
            else{
                pre_edge = B_RISING;
            }  
        }
    }
}

/*!
    \brief      this function handles UART DMA interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void COMM_RECEIVE_IRQHandler(void)
{
    comm_usart_interrupt_handler();
}

