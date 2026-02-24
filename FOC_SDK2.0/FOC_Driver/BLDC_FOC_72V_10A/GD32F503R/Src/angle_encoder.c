/*!
    \file    angle_encoder.c
    \brief   calculate angle and speed using encoder position sensor

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

#include "angle_encoder.h"
#include "angle.h"

/* local function declaration */
/* erase fmc pages from FMC_WRITE_START_ADDR */
static void encoder_sensor_detect_fmc_erase_pages(void);
/* program hall detect results to flash*/
static void encoder_sensor_detect_fmc_data_program(encoder_struct* encoder);
/* read hall detect results from flash */
static void encoder_sensor_detect_fmc_data_read(encoder_struct* encoder);

/*!
    \brief      encoder timer configuration
    \param[in]  none
    \param[out] none
    \retval     none
*/
void encoder_sensor_peripheral_init(void)
{
    /* enable the clock */
    rcu_periph_clock_enable(HALL_A_GPIO_CLK);
    rcu_periph_clock_enable(HALL_B_GPIO_CLK);
    rcu_periph_clock_enable(HALL_C_GPIO_CLK);
    rcu_periph_clock_enable(HALL_TIMER_CLK);
    rcu_periph_clock_enable(RCU_AF);
    
    /* hall TIMER input--CH0/CH1/CH2 */ 
    gpio_mode_set(HALL_A_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_A_PIN);
    gpio_mode_set(HALL_B_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_B_PIN);
    gpio_mode_set(HALL_C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_C_PIN);
    
    gpio_af_set(HALL_A_GPIO_PORT, GPIO_AF_1, HALL_A_PIN);
    gpio_af_set(HALL_B_GPIO_PORT, GPIO_AF_1, HALL_B_PIN);
    gpio_af_set(HALL_C_GPIO_PORT, GPIO_AF_1, HALL_C_PIN);
    
    /* configure TIMER2 interrupt priority */
    nvic_irq_enable(TIMER2_IRQn, 1, 0);

    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;
    
    /* enable the clock */
    rcu_periph_clock_enable(RCU_TIMER4);

    timer_deinit(TIMER2);
    timer_initpara.prescaler         = 0;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = ENCODER_COUNT_NUM - 1;
    timer_initpara.clockdivision     = 0;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2, &timer_initpara);
    
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0;
    timer_input_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_1, &timer_icinitpara);
    timer_input_capture_config(TIMER2, TIMER_CH_2, &timer_icinitpara);

    timer_quadrature_decoder_mode_config(TIMER2,TIMER_QUAD_DECODER_MODE2,TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_RISING);

    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH2);
    timer_interrupt_enable(TIMER2, TIMER_INT_CH2);

    timer_enable(TIMER2);
}

/*!
    \brief      erase fmc pages from FMC_ENCODER_WRITE_START_ADDR to FMC_ENCODER_WRITE_END_ADDR
    \param[in]  page: the page will be erased
    \param[out] none
    \retval     none
*/
static void encoder_sensor_detect_fmc_erase_pages(void)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    fmc_page_erase(FMC_ENCODER_WRITE_START_ADDR );
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    /* lock the main FMC after the erase operation */
    fmc_lock();
}

/*!
    \brief      program hall detect results to flash
    \param[in]  hall: pointer to hall srtuct
    \param[out] none
    \retval     none
*/
static void encoder_sensor_detect_fmc_data_program(encoder_struct* encoder)
{
    uint32_t address = 0x00;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program hall shift data */
    uint16_t fmc_phase_shift = (uint16_t)(encoder->phase_shift / MOTOR_PI_2 * 36000.0f);
    address = FMC_ENCODER_SHIFT_ADDR;
    
    fmc_halfword_program(address, fmc_phase_shift);
    address += 2;
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      read hall detect results from flash
    \param[in]  hall: pointer to hall srtuct
    \param[out] none
    \retval     none
*/
static void encoder_sensor_detect_fmc_data_read(encoder_struct* encoder)
{
    uint16_t *ptrd1;

    /* read flash */
    ptrd1 = (uint16_t *)FMC_ENCODER_SHIFT_ADDR;
    encoder->phase_shift = ((float)(*ptrd1)) * MOTOR_PI_2 / 36000.0f;
    ptrd1 += 2U;
}

/*!
    \brief      encoder control mode, update motor speed, which is expressed in revolutions per second
    \param[in]  none
    \param[out] none
    \retval     the speed of motor
*/
void encoder_sensor_speed_calc(encoder_struct* encoder)
{
    encoder->mech_speed = encoder->elec_speed * ENCODER_SPEED_FACTOR;
}

/*!
    \brief      encoder control mode, update the rotor location information with encoder state changing
    \param[in]  none
    \param[out] none
    \retval     none
*/
void encoder_sensor_angle_calc(encoder_struct* encoder, motor_struct* motor)
{
    int16_t num;
    float speed_temp;
    
    encoder->phase = (TIMER_CNT(TIMER2) * ENCODER_ELEC_STEP + encoder->phase_shift);
    num = (int16_t)((encoder->phase)/MOTOR_PI_2);
    if(encoder->phase > 0){
        encoder->phase = (encoder->phase - num*MOTOR_PI_2);
    }else{
        encoder->phase = encoder->phase - (num-1)*MOTOR_PI_2;
    }
    
    if ((encoder->phase - encoder->phase_pre) < -MOTOR_PI)
    {
        speed_temp = encoder->phase - encoder->phase_pre + MOTOR_PI_2;
    } 
    else if ((encoder->phase - encoder->phase_pre) > MOTOR_PI)
    {
        speed_temp = encoder->phase - encoder->phase_pre - MOTOR_PI_2;
    } 
    else 
    {
        speed_temp = encoder->phase - encoder->phase_pre;
    }
        
    UTILS_LP_FAST(encoder->elec_speed, speed_temp*(float)PWM_FREQUENCE, 0.03f);
    encoder->phase_pre = encoder->phase;
}

/*!
    \brief      initialize encoder detect mode variables
    \param[in]  hall: pointer to encoder srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void encoder_sensor_detect_var_init(encoder_struct* encoder, motor_struct* motor)
{
    /* encoder phase shift initialization */
    encoder->phase_shift = 0;
    encoder->encoder_count = 0;

    /* encoder detect mode variables init */
    motor->motor_openloop.d_var = ENCODER_DETECT_UD_SET;
    motor->motor_openloop.q_var = ENCODER_DETECT_UQ_SET; 
}

/*!
    \brief      initialize encoder mode variables
    \param[in]  hall: pointer to encoder srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void encoder_sensor_var_init(encoder_struct* encoder, motor_struct* motor)
{
    encoder_sensor_detect_fmc_data_read(encoder);
}

/*!
    \brief      the encoder shift phase detecting routine
    \param[in]  none
    \param[out] none
    \retval     none
*/
void encoder_sensor_angle_detect(encoder_struct* encoder, motor_struct* motor)
{
}

/*!
    \brief      encoder interrupt handler
    \param[in]  encoder: pointer to encoder srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void encoder_sensor_interrupt_handler(encoder_struct* encoder, motor_struct* motor)
{
    if(timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH2)){
        /* clear interrupt flag */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH2);
        timer_event_software_generate(TIMER2,TIMER_EVENT_SRC_UPG);
        encoder->flag = 1;
        encoder_sensor_speed_calc(encoder);
        if(DIRECTION_CW == motor->direction){
            encoder->mech_speed = -encoder->mech_speed;
        }
        if(ENCODER_RUN_DETECT == encoder->encoder_running_mode){
            /* program the data to the flash */
            encoder_sensor_detect_fmc_erase_pages();
            encoder_sensor_detect_fmc_data_program(encoder);
        }
    }
}
