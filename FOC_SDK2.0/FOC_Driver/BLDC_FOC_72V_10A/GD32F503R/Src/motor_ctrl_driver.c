/*!
    \file    motor_ctrl_driver.c
    \brief   simple motor control drive

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

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

#include "motor_ctrl_driver.h"
#include "basic_head.h"
#include "foc_soft_delay.h"

/* the low-side mosfets output the fixed duty pwm */
static void motor_patial_brake(uint8_t duty);

/*!
    \brief      start TIMER and ADC to run motor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void start_ctrl(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
    timer_event_software_generate(TIMER0, TIMER_EVENT_SRC_UPG);
    timer_enable(TIMER0);
    
    adc_interrupt_flag_clear(ADC0, ADC_INT_EOIC);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_RISING);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
    
}

/*!
    \brief      stop TIMER and ADC to stop motor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void stop_ctrl(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);
    /* disable TIMER0 */
    timer_disable(TIMER0);
    
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);

    timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_CH2);
    timer_interrupt_disable(HALL_TIMER, TIMER_INT_CH2);
    timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_CH3);
    timer_interrupt_disable(HALL_TIMER, TIMER_INT_CH3);
    timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_TRG);
    timer_interrupt_disable(HALL_TIMER, TIMER_INT_TRG);
}

/*!
    \brief      stop TIMER and ADC to stop motor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void break_ctrl(void)
{
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);
    
    motor_patial_brake(20);
    
    soft_delay_ms(30);
    
    motor_patial_brake(0);
}

/*!
    \brief      the low-side mosfets output the fixed duty pwm
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_patial_brake(uint8_t duty)
{
    uint16_t pwm_set = 0;
    uint16_t pwm_top = 0;
    pwm_top = TIMER_CAR(TIMER0);

    pwm_set = pwm_top - pwm_top * duty / 100;

    timer_update_event_disable(TIMER0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pwm_set);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pwm_set);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, pwm_set);
    timer_update_event_enable(TIMER0);
    
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}
