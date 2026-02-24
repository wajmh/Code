/*!
    \file    pwm_current_2shunt.c
    \brief   2 shunts current sampling method

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

#include "pwm_current_2shunt.h"

/* initialize the RCU module */
static void pwm_current_2shunt_rcu_config(void);
/* initialize the GPIO peripheral */
static void pwm_current_2shunt_gpio_config(void);
/* initialize the TIMER peripheral for pwm */
static void pwm_2shunt_timer_config(void);
/* initialize the ADC peripheral for 2shunt current sample */
static void current_2shunt_adc_config(motor_struct* motor);
/* current offset calculate for 2shunt current sample */
static void current_calibration(motor_struct* motor);

/*!
    \brief      initialize the 2shunt method
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_current_2shunt_peripheral_init(motor_struct* motor)
{
    /* 2shunt current sample clocks configuration */
    pwm_current_2shunt_rcu_config();
    /* 2shunt current sample gpio configuration */
    pwm_current_2shunt_gpio_config();
    /* 2shunt pwm timer configuration */
    pwm_2shunt_timer_config();
    /* 2shunt current sample adc configuration */
    current_2shunt_adc_config(motor);
}

/*!
    \brief      initialize the RCU module
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void pwm_current_2shunt_rcu_config(void)
{
    /* enable the timer0 clock */
    rcu_periph_clock_enable(PWM_UPPER_A_RCU);
    rcu_periph_clock_enable(PWM_UPPER_B_RCU);
    rcu_periph_clock_enable(PWM_UPPER_C_RCU);
    rcu_periph_clock_enable(PWM_DOWN_A_RCU);
    rcu_periph_clock_enable(PWM_DOWN_B_RCU);
    rcu_periph_clock_enable(PWM_DOWN_C_RCU);
    rcu_periph_clock_enable(PWM_BRK_RCU);
    rcu_periph_clock_enable(RCU_AF);
    
    /* enable the adc clock */
    rcu_periph_clock_enable(CURRENT_A_RCU);
    rcu_periph_clock_enable(CURRENT_B_RCU);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
    
    /* enable the trigsel clock */
    rcu_periph_clock_enable(RCU_TRIGSEL);
}

/*!
    \brief      initialize the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void pwm_current_2shunt_gpio_config(void)
{
    /* configure timer0 pin */
    gpio_mode_set(PWM_DOWN_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_A_PIN);
    gpio_output_options_set(PWM_DOWN_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_A_PIN);
    gpio_af_set(PWM_DOWN_A_PORT, GPIO_AF_2, PWM_DOWN_A_PIN);
    
    gpio_mode_set(PWM_DOWN_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_B_PIN);
    gpio_output_options_set(PWM_DOWN_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_B_PIN);
    gpio_af_set(PWM_DOWN_B_PORT, GPIO_AF_2, PWM_DOWN_B_PIN);
    
    gpio_mode_set(PWM_DOWN_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_C_PIN);
    gpio_output_options_set(PWM_DOWN_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_C_PIN);
    gpio_af_set(PWM_DOWN_C_PORT, GPIO_AF_1, PWM_DOWN_C_PIN);
    
    gpio_mode_set(PWM_UPPER_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_A_PIN);
    gpio_output_options_set(PWM_UPPER_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_A_PIN);
    gpio_af_set(PWM_UPPER_A_PORT, GPIO_AF_1, PWM_UPPER_A_PIN);
    
    gpio_mode_set(PWM_UPPER_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_B_PIN);
    gpio_output_options_set(PWM_UPPER_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_B_PIN);
    gpio_af_set(PWM_UPPER_B_PORT, GPIO_AF_1, PWM_UPPER_B_PIN);
    
    gpio_mode_set(PWM_UPPER_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_C_PIN);
    gpio_output_options_set(PWM_UPPER_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_C_PIN);
    gpio_af_set(PWM_UPPER_C_PORT, GPIO_AF_1, PWM_UPPER_C_PIN);
    
    gpio_mode_set(PWM_BRK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PWM_BRK_PIN);
    gpio_af_set(PWM_BRK_PORT, GPIO_AF_4, PWM_BRK_PIN);
    
    /* configure ADC pin */
    gpio_mode_set(CURRENT_A_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_A_PIN);
    gpio_mode_set(CURRENT_B_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_B_PIN);
}

/*!
    \brief      initialize the TIMER peripheral for pwm
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void pwm_2shunt_timer_config(void)
{
    /* TIMER0 configuration */
    rcu_periph_clock_enable(RCU_TIMER0);
    
    timer_parameter_struct       timer_initpara;
    timer_oc_parameter_struct    timer_ocintpara;
    timer_break_parameter_struct timer_breakpara;
    timer_deinit(TIMER0);

    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_CENTER_UP;
    timer_initpara.counterdirection  = TIMER_COUNTER_DOWN;
    timer_initpara.period            = PWM_COUNT;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 1; 
    timer_init(TIMER0, &timer_initpara);
    
    timer_auto_reload_shadow_enable(TIMER0);
    
    /* TIMER0 output disable -- CH0/CH1/CH2 and CH0N/CH1N/CH2N */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_DISABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_0, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_1, &timer_ocintpara);
    timer_channel_output_config(TIMER0, TIMER_CH_2, &timer_ocintpara);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, PWM_COUNT/2);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, PWM_COUNT/2);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, PWM_COUNT/2);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_2, TIMER_OC_MODE_PWM0);

    /* channel 3 configuration in OC */ 
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH; 
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_LOW;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER0, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, (uint32_t)TSAMPLE_POINT_TIMER);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TSAMPLE_MODE_TIMER);
    /* enable the TIMER0 preload on CH3 register */
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_ENABLE);

    timer_channel_output_shadow_config(TIMER0, TIMER_CH_0, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_1, TIMER_OC_SHADOW_ENABLE);
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_2, TIMER_OC_SHADOW_ENABLE);
        
    /* TIMER break time */
    timer_break_struct_para_init(&timer_breakpara);
    timer_breakpara.runoffstate         = TIMER_ROS_STATE_ENABLE;
    timer_breakpara.ideloffstate        = TIMER_IOS_STATE_DISABLE;
    timer_breakpara.deadtime            = DEAD_TIME;
    timer_breakpara.outputautostate     = TIMER_OUTAUTO_DISABLE;
    timer_breakpara.protectmode         = TIMER_CCHP0_PROT_OFF;
    timer_breakpara.breakstate          = TIMER_BREAK_ENABLE;
    timer_breakpara.breakfilter         = 0U;
    timer_breakpara.breakpolarity       = TIMER_BREAK_POLARITY_HIGH;
    timer_break_config(TIMER0, &timer_breakpara);

    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_UPDATE);
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_ENABLE);

    timer_channel_primary_output_config(TIMER0, TIMER_CH_0, ENABLE);
    timer_channel_primary_output_config(TIMER0, TIMER_CH_1, ENABLE);
    timer_channel_primary_output_config(TIMER0, TIMER_CH_2, ENABLE);
    timer_primary_output_config(TIMER0, ENABLE);
}

/*!
    \brief      pwm update in the 2shunt method
    \param[in]  pwm: pointer to pwm_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_2shunt_update_duty(modulation_struct pwm)
{
    timer_update_event_disable(TIMER0);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_0, pwm.tA);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_1, pwm.tB);
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_2, pwm.tC);
    timer_update_event_enable(TIMER0);
}

/*!
    \brief      initialize the ADC peripheral for 2shunt current sample
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_2shunt_adc_config(motor_struct* motor)
{
    adc_deinit(ADC0);
    adc_deinit(ADC1);
    
    /* configure ADC mode */
    adc_sync_mode_config(ADC_DAUL_INSERTED_PARALLEL);
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC1, ADC_SCAN_MODE, ENABLE);

    /* configure ADC data alignment */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);

    /* configure ADC regular channel trigger */
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    
    /* configure ADC regular channel length */
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1);
    adc_routine_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_239POINT5);

    /* configure ADC inserted channel trigger */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_RISING);
    trigsel_init(TRIGSEL_OUTPUT_ADC0_INSTRG, TRIGSEL_INPUT_TIMER0_CH3);
    /* configure ADC inserted channel length */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 1);
    /* configure ADC inserted channel trigger */
    adc_external_trigger_config(ADC1, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    /* configure ADC inserted channel length */
    adc_channel_length_config(ADC1, ADC_INSERTED_CHANNEL, 1);
    /* configure ADC inserted channel */
    adc_inserted_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLE_TIME);
    adc_inserted_channel_config(ADC1, 0, CURRENT_B_CHANNEL, ADC_SAMPLE_TIME);

    /* enable ADC interface */
    adc_enable(ADC0);
    adc_enable(ADC1);

    current_calibration(motor);
    
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
}

/*!
    \brief      current offset calculate for 2shunt current sample
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_calibration(motor_struct* motor)
{
    uint8_t i;
    uint16_t current_sum_a = 0,current_sum_b = 0;
    
    /* down legs: closed, upper legs: open */
    gpio_mode_set(PWM_DOWN_A_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_DOWN_A_PIN);
    gpio_output_options_set(PWM_DOWN_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_A_PIN);
    
    gpio_mode_set(PWM_DOWN_B_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_DOWN_B_PIN);
    gpio_output_options_set(PWM_DOWN_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_B_PIN);
    
    gpio_mode_set(PWM_DOWN_C_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_DOWN_C_PIN);
    gpio_output_options_set(PWM_DOWN_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_C_PIN);
    
    gpio_mode_set(PWM_UPPER_A_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_UPPER_A_PIN);
    gpio_output_options_set(PWM_UPPER_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_A_PIN);
    
    gpio_mode_set(PWM_UPPER_B_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_UPPER_B_PIN);
    gpio_output_options_set(PWM_UPPER_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_B_PIN);
    
    gpio_mode_set(PWM_UPPER_C_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWM_UPPER_C_PIN);
    gpio_output_options_set(PWM_UPPER_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_C_PIN);
    
    gpio_bit_reset(PWM_DOWN_A_PORT, PWM_DOWN_A_PIN);
    gpio_bit_reset(PWM_DOWN_B_PORT, PWM_DOWN_B_PIN);
    gpio_bit_reset(PWM_DOWN_C_PORT, PWM_DOWN_C_PIN);
    gpio_bit_reset(PWM_UPPER_A_PORT, PWM_UPPER_A_PIN);
    gpio_bit_reset(PWM_UPPER_B_PORT, PWM_UPPER_B_PIN);
    gpio_bit_reset(PWM_UPPER_C_PORT, PWM_UPPER_C_PIN);

    /* use regular group to sample the current offset */
    /* the current offset of phase A */
    adc_routine_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLETIME_239POINT5);
    for(i=0; i<26; i++){
        adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
        while(RESET == adc_flag_get(ADC0, ADC_FLAG_EORC));
        adc_flag_clear(ADC0,ADC_FLAG_EORC);
        if(i >= 10){
            current_sum_a += adc_routine_data_read(ADC0);
        }
    }
    motor->i_a_offset = current_sum_a/16;

    /* the current offset of phase B */
    adc_routine_channel_config(ADC0, 0, CURRENT_B_CHANNEL, ADC_SAMPLETIME_239POINT5);
    for(i=0; i<26; i++){
        adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
        while(RESET == adc_flag_get(ADC0, ADC_FLAG_EORC));
        adc_flag_clear(ADC0,ADC_FLAG_EORC);
        if(i >= 10){
            current_sum_b += adc_routine_data_read(ADC0);
        }
    }
    motor->i_b_offset = current_sum_b/16;
    
    /* restore to default configuration */
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    adc_inserted_channel_config(ADC0, 0, CURRENT_A_CHANNEL, ADC_SAMPLE_TIME);
    adc_inserted_channel_config(ADC1, 0, CURRENT_B_CHANNEL, ADC_SAMPLE_TIME);

    /* release driver */
    gpio_mode_set(PWM_DOWN_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_A_PIN);
    gpio_output_options_set(PWM_DOWN_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_A_PIN);
    gpio_af_set(PWM_DOWN_A_PORT, GPIO_AF_2, PWM_DOWN_A_PIN);
    
    gpio_mode_set(PWM_DOWN_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_B_PIN);
    gpio_output_options_set(PWM_DOWN_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_B_PIN);
    gpio_af_set(PWM_DOWN_B_PORT, GPIO_AF_2, PWM_DOWN_B_PIN);
    
    gpio_mode_set(PWM_DOWN_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_C_PIN);
    gpio_output_options_set(PWM_DOWN_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_DOWN_C_PIN);
    gpio_af_set(PWM_DOWN_C_PORT, GPIO_AF_1, PWM_DOWN_C_PIN);
    
    gpio_mode_set(PWM_UPPER_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_A_PIN);
    gpio_output_options_set(PWM_UPPER_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_A_PIN);
    gpio_af_set(PWM_UPPER_A_PORT, GPIO_AF_1, PWM_UPPER_A_PIN);
    
    gpio_mode_set(PWM_UPPER_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_B_PIN);
    gpio_output_options_set(PWM_UPPER_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_B_PIN);
    gpio_af_set(PWM_UPPER_B_PORT, GPIO_AF_1, PWM_UPPER_B_PIN);
    
    gpio_mode_set(PWM_UPPER_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_C_PIN);
    gpio_output_options_set(PWM_UPPER_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, PWM_UPPER_C_PIN);
    gpio_af_set(PWM_UPPER_C_PORT, GPIO_AF_1, PWM_UPPER_C_PIN);
}

/*!
    \brief      read the sampling current in the 2shunt method
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_current_2shunt_get_current(motor_struct* motor)
{
    int16_t current_a = 0, current_b = 0 ,current_c = 0;
    current_a = motor->i_a_offset - adc_latch_data_read(ADC0, ADC_LATCH_DATA_0);
    current_b = motor->i_b_offset - adc_latch_data_read(ADC1, ADC_LATCH_DATA_0);
    current_c = -(current_a + current_b);
    
    motor->ia = ((float)current_a * 3.3f / 4096)/(CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
    motor->ib = ((float)current_b * 3.3f / 4096)/(CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
    motor->ic = ((float)current_c * 3.3f / 4096)/(CURRENT_SHUNT_RES * CURRENT_AMP_GAIN);
}


/*!
    \brief      modify the sampling point in the 2shunt method
    \param[in]  modulation: pointer to modulation_struct srtuct
                motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_current_2shunt_sampling_point_config(modulation_struct* modulation, motor_struct* motor)
{
    /* none */
}
