/*!
    \file    pwm_current_1shunt.c
    \brief   1 shunt current sampling method

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

#include "pwm_current_1shunt.h"

/* local function declaration */
/* initialize the RCU module */
static void pwm_current_1shunt_rcu_config(void);
/* initialize the GPIO peripheral */
static void pwm_current_1shunt_gpio_config(void);
/* initialize the TIMER peripheral for pwm */
static void pwm_current_1shunt_timer_config(void);
/* initialize the ADC peripheral for 1shunt current sample */
static void pwm_current_1shunt_adc_config(motor_struct* motor);
/* current offset calculate for 1shunt current sample */
static void current_calibration(motor_struct* motor);
/* timer0_chxcv and current sampling point configuration */
static void timer0_dma_config(void);
/* current sampling region recognize for 1shunt current sample */
static void current_sample_region_divide(motor_struct* motor);
/* 1shunt current sampling handle in region1 */
static void current_sample_region1_handle(motor_struct* motor);
/* 1shunt current sampling handle in region2 */
static void current_sample_region2_handle(motor_struct* motor);
/* make sure the distorted PWM timings are in the physical limitation */
static void oneshunt_parameter_check(int16_t *t1, int16_t *t2);

/* global variables definition */
/* one shunt sample srtuct */
static current_sample_struct oneshunt_current;
/* DMA transmission parameters */
static uint16_t timer_update_buffer[8] = {0};
/* DMA transmission second sample point */
static uint16_t sample_point = 500; 

/*!
    \brief      initialize the 1shunt method
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pwm_current_1shunt_peripheral_init(motor_struct* motor)
{
    /* 1shunt current sample clocks configuration */
    pwm_current_1shunt_rcu_config();
    /* 1shunt current sample gpio configuration */
    pwm_current_1shunt_gpio_config();
    /* 1shunt pwm timer configuration */
    pwm_current_1shunt_timer_config();
    /* 1shunt current sample adc configuration */
    pwm_current_1shunt_adc_config(motor);
    /* timer0_chxcv and current sampling point configuration */
    timer0_dma_config();
}

/*!
    \brief      initialize the RCU module
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void pwm_current_1shunt_rcu_config(void)
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
    rcu_periph_clock_enable(CURRENT_IBUS_RCU);
    rcu_periph_clock_enable(RCU_ADC0);
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
static void pwm_current_1shunt_gpio_config(void)
{
    /* configure timer0 pin */
    gpio_mode_set(PWM_DOWN_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_A_PIN);
    gpio_output_options_set(PWM_DOWN_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_DOWN_A_PIN);
    gpio_af_set(PWM_DOWN_A_PORT, GPIO_AF_2, PWM_DOWN_A_PIN);
    
    gpio_mode_set(PWM_DOWN_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_B_PIN);
    gpio_output_options_set(PWM_DOWN_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_DOWN_B_PIN);
    gpio_af_set(PWM_DOWN_B_PORT, GPIO_AF_2, PWM_DOWN_B_PIN);
    
    gpio_mode_set(PWM_DOWN_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_DOWN_C_PIN);
    gpio_output_options_set(PWM_DOWN_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_DOWN_C_PIN);
    gpio_af_set(PWM_DOWN_C_PORT, GPIO_AF_1, PWM_DOWN_C_PIN);
    
    gpio_mode_set(PWM_UPPER_A_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_A_PIN);
    gpio_output_options_set(PWM_UPPER_A_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_UPPER_A_PIN);
    gpio_af_set(PWM_UPPER_A_PORT, GPIO_AF_1, PWM_UPPER_A_PIN);
    
    gpio_mode_set(PWM_UPPER_B_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_B_PIN);
    gpio_output_options_set(PWM_UPPER_B_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_UPPER_B_PIN);
    gpio_af_set(PWM_UPPER_B_PORT, GPIO_AF_1, PWM_UPPER_B_PIN);
    
    gpio_mode_set(PWM_UPPER_C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWM_UPPER_C_PIN);
    gpio_output_options_set(PWM_UPPER_C_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, PWM_UPPER_C_PIN);
    gpio_af_set(PWM_UPPER_C_PORT, GPIO_AF_1, PWM_UPPER_C_PIN);
    
    gpio_mode_set(PWM_BRK_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, PWM_BRK_PIN);
    gpio_af_set(PWM_BRK_PORT, GPIO_AF_4, PWM_BRK_PIN);
    
    /* configure ADC pin */
    gpio_mode_set(CURRENT_IBUS_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_IBUS_PIN);
}

/*!
    \brief      initialize the TIMER peripheral for pwm
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void pwm_current_1shunt_timer_config(void)
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
    timer_initpara.repetitioncounter = 0;
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
    timer_channel_output_pulse_value_config(TIMER0, TIMER_CH_3, 300);
    timer_channel_output_mode_config(TIMER0, TIMER_CH_3, TIMER_OC_MODE_PWM1);
    /* enable the TIMER0 preload on CH3 register */
    timer_channel_output_shadow_config(TIMER0, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
        
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

    timer_dma_transfer_config(TIMER0, TIMER_DMACFG_DMATA_CH0CV, TIMER_DMACFG_DMATC_4TRANSFER);
    timer_dma_enable(TIMER0, TIMER_DMA_UPD);
    timer_dma_enable(TIMER0, TIMER_DMA_CH3D);
    timer_master_output_trigger_source_select(TIMER0, TIMER_TRI_OUT_SRC_UPDATE);
    timer_master_slave_mode_config(TIMER0, TIMER_MASTER_SLAVE_MODE_ENABLE);

    timer_channel_primary_output_config(TIMER0, TIMER_CH_0, ENABLE);
    timer_channel_primary_output_config(TIMER0, TIMER_CH_1, ENABLE);
    timer_channel_primary_output_config(TIMER0, TIMER_CH_2, ENABLE);
    timer_primary_output_config(TIMER0, ENABLE);
}

/*!
    \brief      initialize the ADC peripheral for 1shunt current sample
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void pwm_current_1shunt_adc_config(motor_struct* motor)
{
    adc_deinit(ADC0);

    /* configure ADC mode */
    /* ADC scan mode function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* configure ADC data alignment */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* configure ADC regular channel trigger */
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    
    /* configure ADC regular channel length */
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1);
    adc_routine_channel_config(ADC0, 0, CURRENT_IBUS_CHANNEL, ADC_SAMPLETIME_239POINT5);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    /* ADC channel offset */
    current_calibration(motor);

    /* ADC inserted channel mode configure */
    adc_discontinuous_mode_config(ADC0, ADC_INSERTED_CHANNEL, 2);
    /* ADC inserted channel trigger */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, EXTERNAL_TRIGGER_RISING);
    trigsel_init(TRIGSEL_OUTPUT_ADC0_INSTRG, TRIGSEL_INPUT_TIMER0_CH3);
    /* configure ADC inserted channel length */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);
    adc_inserted_channel_config(ADC0, 0, CURRENT_IBUS_CHANNEL, ADC_SAMPLE_TIME);
    adc_inserted_channel_config(ADC0, 1, CURRENT_IBUS_CHANNEL, ADC_SAMPLE_TIME);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    
    adc_dma_mode_enable(ADC0,ADC_ROUTINE_CHANNEL);
    nvic_irq_enable(ADC0_1_IRQn, 0, 0);
}

/*!
    \brief      current offset calculate for 1shunt current sample
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_calibration(motor_struct* motor)
{
    uint8_t i;
    uint16_t current_sum_bus = 0;
    
    /* ensure the calibration accuracy, close all driver */
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
    /* the current offset of I-BUS */
    adc_routine_channel_config(ADC0, 0, CURRENT_IBUS_CHANNEL, ADC_SAMPLETIME_239POINT5);
    for(i=0; i<26; i++){
        adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
        while(RESET == adc_flag_get(ADC0, ADC_FLAG_EORC));
        adc_flag_clear(ADC0,ADC_FLAG_EORC);
        if(i >= 10){
            current_sum_bus += adc_routine_data_read(ADC0);
        }
    }
    motor->i_bus_offset = current_sum_bus/16;
    
    /* restore to default configuration */
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

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
    \brief      timer0_chxcv and current sampling point configuration
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void timer0_dma_config(void)
{
    dma_parameter_struct dma_init_struct;
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMAMUX);

    /* DMA configuration */
    dma_deinit(DMA0, DMA_CH3);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.request      = DMA_REQUEST_TIMER0_CH3;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr  = (uint32_t)&sample_point;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number       = 1;
    dma_init_struct.periph_addr  = (uint32_t)(&TIMER_CH3CV(TIMER0));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH3, &dma_init_struct);
    dma_circulation_enable(DMA0, DMA_CH3);
    dma_memory_to_memory_disable(DMA0, DMA_CH3);
    dma_channel_enable(DMA0, DMA_CH3);

    /* DMA configuration */
    dma_deinit(DMA0, DMA_CH4);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.request      = DMA_REQUEST_TIMER0_UP;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr  = (uint32_t)timer_update_buffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number       = 8;
    dma_init_struct.periph_addr  = (uint32_t)(&TIMER_DMATB(TIMER0));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_init_struct);
    dma_circulation_enable(DMA0, DMA_CH4);
    dma_memory_to_memory_disable(DMA0, DMA_CH4);
    dma_channel_enable(DMA0, DMA_CH4);
    
    /* initialize timer_update_buffer array */
    timer_update_buffer[0] = PWM_COUNT/2;
    timer_update_buffer[1] = PWM_COUNT/2;
    timer_update_buffer[2] = PWM_COUNT/2;
    timer_update_buffer[3] = 300;
    timer_update_buffer[4] = PWM_COUNT/2;
    timer_update_buffer[5] = PWM_COUNT/2;
    timer_update_buffer[6] = PWM_COUNT/2;
    timer_update_buffer[7] = 0;
}

/*!
    \brief      read the sampling current in the 1shunt method
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_current_1shunt_get_current(motor_struct* motor)
{
    int16_t current_a = 0, current_b = 0 ,current_c = 0;
    /* according to the sector, read the second current */
    switch(motor->sector){
        case ONE: {
            current_c = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_a = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_b = -(current_a + current_c);
            break;
        }
        case TWO: {
            current_c = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_b = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_a = -(current_b + current_c);
            break;
        }
        case THREE: {
            current_a = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_b = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_c = -(current_a + current_b);
            break;
        }
        case FOUR: {
            current_a = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_c = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_b = -(current_a + current_c);
            break;
        }
        case FIVE: {
            current_b = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_c = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_a = -(current_b + current_c);
            break;
        }

        case SIX: {
            current_b = -(adc_latch_data_read(ADC0, ADC_LATCH_DATA_0) - motor->i_bus_offset);
            current_a = (adc_latch_data_read(ADC0, ADC_LATCH_DATA_1) - motor->i_bus_offset);
            current_c = -(current_a + current_b);
            break;
        }
    }
    motor->ia = (float)current_a * CURRENT_VALUE_CONVERSION;
    motor->ib = (float)current_b * CURRENT_VALUE_CONVERSION;
    motor->ic = (float)current_c * CURRENT_VALUE_CONVERSION;
}

/*!
    \brief      modify the sampling point in the 1shunt method
    \param[in]  modulation: pointer to modulation_struct srtuct
                motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_current_1shunt_sampling_point_config(modulation_struct* modulation, motor_struct* motor)
{
    /* calculate the sampling time */
    switch(motor->sector){
        case ONE:{
            oneshunt_current.samp1 = modulation->tB - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tB + TADC_STABLE_POINT;
            break;
        }
        case TWO:{
            oneshunt_current.samp1 = modulation->tA - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tA + TADC_STABLE_POINT;
            break;
        }
        case THREE:{
            oneshunt_current.samp1 = modulation->tC  - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tC + TADC_STABLE_POINT;
            break;
        }
        case FOUR:{
            oneshunt_current.samp1 = modulation->tB - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tB + TADC_STABLE_POINT;
            break;
        }
        case FIVE:{
            oneshunt_current.samp1 = modulation->tA - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tA + TADC_STABLE_POINT;
            break;
        }
        case SIX:{
            oneshunt_current.samp1 = modulation->tC - (TMIN - TADC_STABLE_POINT);
            oneshunt_current.samp2 = modulation->tC + TADC_STABLE_POINT;
            break;
        }
        default:
            break;
    }

    oneshunt_current.ta = modulation->tA;
    oneshunt_current.tb = modulation->tB;
    oneshunt_current.tc = modulation->tC;
    
    /* sampling region recognize */
    current_sample_region_divide(motor);
    
    /* wait until the timer counts down */
    if((MC_STATE_RUNNING == motor->state) && (MC_STOP != motor->command))
    while(1){
        uint16_t dir;
        dir = TIMER_CTL0(TIMER0);
        if(dir & TIMER_CTL0_DIR) break;
    }
    
    /* update the dma-timer buffer */
    timer_update_buffer[0] = modulation->tA;
    timer_update_buffer[1] = modulation->tB;
    timer_update_buffer[2] = modulation->tC;
    timer_update_buffer[3] = oneshunt_current.samp1;
    timer_update_buffer[4] = modulation->tA;
    timer_update_buffer[5] = modulation->tB;
    timer_update_buffer[6] = modulation->tC;
    timer_update_buffer[7] = 0;
    
    sample_point = oneshunt_current.samp2;
    
    /* phase shift process */
    if((oneshunt_current.current1 == IA)||(oneshunt_current.current2 == IA)){
        timer_update_buffer[0] = oneshunt_current.ta_mod1;
        timer_update_buffer[4] = oneshunt_current.ta_mod2;
    }
    if((oneshunt_current.current1 == IB)||(oneshunt_current.current2 == IB)){
        timer_update_buffer[1] = oneshunt_current.tb_mod1;
        timer_update_buffer[5] = oneshunt_current.tb_mod2;
    }
    if((oneshunt_current.current1 == IC)||(oneshunt_current.current2 == IC)){
        timer_update_buffer[2] = oneshunt_current.tc_mod1;
        timer_update_buffer[6] = oneshunt_current.tc_mod2;
    }
}

/*!
    \brief      1-shunt current sampling region recognize
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_sample_region_divide(motor_struct* motor)
{
    /* calculate the error of the time point */
    switch(motor->sector){
        case ONE:{
            oneshunt_current.delta_t1 = oneshunt_current.tb - oneshunt_current.tc;
            oneshunt_current.delta_t2 = oneshunt_current.ta - oneshunt_current.tb;
            break;
        }
        case TWO:{
            oneshunt_current.delta_t1 = oneshunt_current.ta - oneshunt_current.tc;
            oneshunt_current.delta_t2 = oneshunt_current.tb - oneshunt_current.ta;
            break;
        }
        case THREE:{
            oneshunt_current.delta_t1 = oneshunt_current.tc - oneshunt_current.ta;
            oneshunt_current.delta_t2 = oneshunt_current.tb - oneshunt_current.tc;
            break;
        }
        case FOUR:{
            oneshunt_current.delta_t1 = oneshunt_current.tb - oneshunt_current.ta;
            oneshunt_current.delta_t2 = oneshunt_current.tc - oneshunt_current.tb;
            break;
        }
        case FIVE:{
            oneshunt_current.delta_t1 = oneshunt_current.ta - oneshunt_current.tb;
            oneshunt_current.delta_t2 = oneshunt_current.tc - oneshunt_current.ta;
            break;
        }
        case SIX:{
            oneshunt_current.delta_t1 = oneshunt_current.tc - oneshunt_current.tb;
            oneshunt_current.delta_t2 = oneshunt_current.ta - oneshunt_current.tc;
            break;
        }
        default:
            break;
    }

    /* sampling region divide */
    if((oneshunt_current.delta_t1 < TMIN) && ((oneshunt_current.delta_t2 < TMIN))){
        oneshunt_current.region = LOW_MODULATION;
    }else if(oneshunt_current.delta_t1 < TMIN){
        oneshunt_current.region = REGION1;
    }else if(oneshunt_current.delta_t2 < TMIN){
        oneshunt_current.region = REGION2;
    }else{
        oneshunt_current.region = NORMAL;
    }

    /* phase shift operation and sampling time set */
    if(oneshunt_current.region == REGION1){
        current_sample_region1_handle(motor);
        oneshunt_current.current2 = NONE;
    }else if(oneshunt_current.region == REGION2){
        oneshunt_current.current1 = NONE;
        current_sample_region2_handle(motor);
    }else if(oneshunt_current.region == LOW_MODULATION){
        current_sample_region1_handle(motor);
        current_sample_region2_handle(motor);
    }else if(oneshunt_current.region == NORMAL){
        oneshunt_current.current1 = NONE;
        oneshunt_current.current2 = NONE;
    }
}

/*!
    \brief      1-shunt current sampling handle in region1
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_sample_region1_handle(motor_struct* motor)
{
    /* shift the phase output and modify the sampling time */
    switch(motor->sector){
        case THREE:
        case FOUR:{
            oneshunt_current.ta_mod1 = oneshunt_current.ta - (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.ta_mod2 = oneshunt_current.ta + (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.samp1 = oneshunt_current.ta_mod1 + TADC_STABLE_POINT;
            oneshunt_current.current1 = IA;
            oneshunt_parameter_check(&oneshunt_current.ta_mod1, &oneshunt_current.ta_mod2);
            break;
        }
        case FIVE:
        case SIX:{
            oneshunt_current.tb_mod1 = oneshunt_current.tb - (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.tb_mod2 = oneshunt_current.tb + (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.samp1 = oneshunt_current.tb_mod1 + TADC_STABLE_POINT;
            oneshunt_current.current1 = IB;
            oneshunt_parameter_check(&oneshunt_current.tb_mod1, &oneshunt_current.tb_mod2);
            break;
        }
        case ONE:
        case TWO:{
            oneshunt_current.tc_mod1 = oneshunt_current.tc - (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.tc_mod2 = oneshunt_current.tc + (TMIN - oneshunt_current.delta_t1);
            oneshunt_current.samp1 = oneshunt_current.tc_mod1 + TADC_STABLE_POINT;
            oneshunt_current.current1 = IC;
            oneshunt_parameter_check(&oneshunt_current.tc_mod1, &oneshunt_current.tc_mod2);
            break;
        }
        default:
            break;
    }
}

/*!
    \brief      1-shunt current sampling handle in region2
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
static void current_sample_region2_handle(motor_struct* motor)
{
    /* shift the phase output */
    switch(motor->sector){
        case FOUR:
        case FIVE:{
            oneshunt_current.tc_mod1 = oneshunt_current.tc + (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.tc_mod2 = oneshunt_current.tc - (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.current2 = IC;
            oneshunt_current.samp2 = oneshunt_current.tc_mod1 - (TMIN - TADC_STABLE_POINT);
            oneshunt_parameter_check(&oneshunt_current.tc_mod2, &oneshunt_current.tc_mod1);
            break;
        }
        case ONE:
        case SIX:{
            oneshunt_current.ta_mod1 = oneshunt_current.ta + (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.ta_mod2 = oneshunt_current.ta - (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.current2 = IA;
            oneshunt_current.samp2 = oneshunt_current.ta_mod1 - (TMIN - TADC_STABLE_POINT);
            oneshunt_parameter_check(&oneshunt_current.ta_mod2, &oneshunt_current.ta_mod1);
            break;
        }
        case TWO:
        case THREE:{
            oneshunt_current.tb_mod1 = oneshunt_current.tb + (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.tb_mod2 = oneshunt_current.tb - (TMIN - oneshunt_current.delta_t2);
            oneshunt_current.current2 = IB;
            oneshunt_current.samp2 = oneshunt_current.tb_mod1 - (TMIN - TADC_STABLE_POINT);
            oneshunt_parameter_check(&oneshunt_current.tb_mod2, &oneshunt_current.tb_mod1);
            break;
        }
        default:
            break;
    }
}

/*!
    \brief      make sure the distorted PWM timings are in the physical limitation
    \param[in]  t1: the first distorted PWM timing point
    \param[in]  t2: the second distorted PWM timing point
    \param[out] none
    \retval     none
*/
static void oneshunt_parameter_check(int16_t *t1, int16_t *t2)
{
    if(*t1 < TIME_MARGIN){
        *t1 = TIME_MARGIN;
    }

    if(*t2 > (PWM_COUNT-TIME_MARGIN)){
        *t2 = PWM_COUNT-TIME_MARGIN;
    }
}
