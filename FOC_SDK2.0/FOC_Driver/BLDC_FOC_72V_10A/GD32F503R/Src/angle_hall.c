/*!
    \file    angle_hall.c
    \brief   calculate angle and speed using hall position sensor

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

#include "angle_hall.h"

/* local function declaration */
/* configure peripheral clocks related to the hall sensor */
static void hall_rcu_config(void);
/* configure the GPIO peripheral related to the hall sensor */
static void hall_gpio_config(void);
/* configure the TIMER peripheral related to the hall sensor */
static void hall_timer_config(void);
/* erase fmc pages from FMC_WRITE_START_ADDR */
static void hall_detect_fmc_erase_pages(void);
/* program hall detect results to flash*/
static void hall_detect_fmc_data_program(hall_struct* hall);
/* read hall detect results from flash */
static void hall_detect_fmc_data_read(hall_struct* hall);
/* read hall installation position from macro */
static void hall_fix_macro_data_read(hall_struct* hall);
/* read hall state */
static uint8_t hall_read_state(void);
/* update rotor speed */
static void hall_speed_update(hall_struct* hall, motor_struct* motor);
/* update the rotor location information with hall state changing */
static void hall_rotor_location_update(hall_struct* hall, motor_struct* motor);

/* global variables definition */
/* hall speed calculate srtuct */
hall_speed_struct hall_speed;
/* hall detect mode srtuct */
hall_detect_struct hall_detect;
/* detect interrupt counter */
uint32_t adc_interrupt_cnt = 0;

/*!
    \brief      initialize hall sensor peripherals
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hall_peripheral_init(void)
{
    /* hall clocks configuration */
    hall_rcu_config();
    /* hall GPIO configuration */
    hall_gpio_config();
    /* hall timer configuration */
    hall_timer_config();
}

/*!
    \brief      initialize hall normal mode variables
    \param[in]  hall: pointer to hall srtuct
    \param[out] none
    \retval     none
*/
void hall_normal_var_init(hall_struct* hall,motor_struct* motor)
{
    /* hall phase table initialization */
    for(int i = 0; i <= 7; i++){
        hall->phase_table[DIRECTION_CCW][i] = 0.0f;
        hall->phase_table[DIRECTION_CW][i] = 0.0f;
        hall->sequence[DIRECTION_CCW][i] = 0;
        hall->sequence[DIRECTION_CW][i] = 0;
    }

    /* read hall phase data from flash or macro */
    if(hall->hall_position_mode == HALL_POSITION_DETECT){
        hall_detect_fmc_data_read(hall);
    }else{
        hall_fix_macro_data_read(hall);
    }

    /* hall struct variable initialization */
    hall->state = hall_read_state();
    hall->phase = hall->phase_table[0][hall->state] + MOTOR_PI / 6.0f;
    hall->phase = angle_nomalize_float(hall->phase);
    hall->phase_error = 0.0f;
    hall->direct_correct_flag = 0;
    hall->seq_temp = 1;
    
    /* hall speed struct variable initialization */
    hall_speed.timer_overflow = 0;
    hall_speed.aver_speed = 0.0f;
    hall_speed.high_speed = 0.0f;
    hall_speed.low_speed = 0.0f;
    hall_speed.timer_count_sum = 0;
    hall_speed.speed_index = 0;
    hall_speed.speed_sum = 0.0f;

    for(int i = 0; i < HALL_AVE_SPEED_LEN; i ++){
        hall_speed.speed_array[i] = 0.0f;
    }
}

/*!
    \brief      return rotor speed
    \param[in]  none
    \param[out] none
    \retval     motor mechanical speed, 0-4000rpm
*/
int16_t hall_speed_calc(void)
{
    return (int16_t)(hall_speed.aver_speed);
}

/*!
    \brief      calculate rotor angle
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void hall_angle_calc(hall_struct* hall, motor_struct* motor)
{
    float phase_diff = 0.0f;
    float phase_temp = hall->phase;
    uint16_t max_cnt = 0;

    /* calculate the angle interpolation in per pwm period */
    phase_diff = (float)NUMBER_OF_PAIRS * (hall_speed.low_speed) * MOTOR_PI_2 / (60 * PWM_FREQUENCE);

    if(hall_speed.low_speed < HALL_SPEED_LIMIT){
        phase_diff = HALL_PHASE_DIFF_INIT;
    }

    /* add the number of corrections in per pwm period */
    hall->phase_correct_cnt ++;
    max_cnt = (uint16_t)(MOTOR_PI / (6.0f * phase_diff));
    /* angle estimation error correction */
    if(hall->phase_correct_cnt < max_cnt){
        phase_diff = phase_diff + hall->phase_error / max_cnt;
    }

    /* calculate current rotor angle */
    if(DIRECTION_CW == motor->direction){
        phase_temp -= (phase_diff);
    }else{
        phase_temp += (phase_diff);
    }

    /* rotor angle process */
    phase_temp = angle_nomalize_float(phase_temp);
    hall->phase = phase_temp;
}

/*!
    \brief      initialize hall detect mode variables
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void hall_detect_var_init(hall_struct* hall, motor_struct* motor)
{
    /* hall phase table initialization */
    for(int i = 0; i <= 7; i ++){
        hall_detect.uq_phase[i] = 0.0f;
        hall_detect.uq_count[i] = 0;
        hall->phase_table[DIRECTION_CCW][i] = 0.0f;
        hall->phase_table[DIRECTION_CW][i] = 0.0f;
        hall->sequence[DIRECTION_CCW][i] = 0;
        hall->sequence[DIRECTION_CW][i] = 0;
    }
    adc_interrupt_cnt = 0;
    /* hall detect mode variables init */
    motor->motor_openloop.d_var = 0.0f;
    motor->motor_openloop.q_var = HALL_DETECT_TORQUE_SET;

    hall_detect.step = 1;
    hall_detect.uq_circle = 0;
}

/*!
    \brief      hall installation angle detect
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void hall_angle_detect(hall_struct* hall, motor_struct* motor)
{
    float seq_ccw_temp[8] = {0.0f};
    float seq_cw_temp[8] = {0.0f};

    switch(hall_detect.step){
    /* hall_detect step1 :uq detect start(uq = HALL_DETECT_TORQUE_SET, ud = 0) */
    case 1: {
        hall->phase = hall->phase + HALL_DETECT_THETA_ADD1;
        if(hall->phase > MOTOR_PI_2){
            hall->phase = hall->phase - MOTOR_PI_2;
            hall_detect.uq_circle ++;
        }
        if(hall_detect.uq_circle > 30){
            /* hall_detect step1 :start record data of uq detect */
            hall_detect.uq_data_record = 1;
            if(hall_detect.uq_circle > 40){
                hall_detect.uq_data_record = 0;
                adc_interrupt_cnt = 0;
                hall_detect.step = 2;
            }
        }
        break;
    }
    /* hall_detect step2 :switch to ud detect (ud = HALL_DETECT_FLUX_SET, uq = 0) */
    case 2: {
        adc_interrupt_cnt ++;
        if(adc_interrupt_cnt == 1){
            motor->motor_openloop.d_var = HALL_DETECT_FLUX_SET;
            motor->motor_openloop.q_var = 0.0f;
        /* hall_detect delay 1s */
        }else if(adc_interrupt_cnt >= HALL_DETECT_DELAY1){
            hall_detect.step = 3 ;
            hall_detect.ud_direction = DIRECTION_CCW;
            adc_interrupt_cnt = 0;
        }
        break;
    }
    case 3: {
        adc_interrupt_cnt ++;
        /* hall_detect delay 0.1s */
        if(adc_interrupt_cnt >= HALL_DETECT_DELAY2){
            if(hall_detect.ud_direction == DIRECTION_CCW)
                hall->phase = hall->phase + HALL_DETECT_THETA_ADD2;
            if(hall_detect.ud_direction == DIRECTION_CW)
                hall->phase = hall->phase - HALL_DETECT_THETA_ADD2;
            adc_interrupt_cnt = 0;
        }
        break;
    }
    case 4: {
        /* hall_detect stop */
        motor->motor_stop();
        motor->command = MC_STOP;
        
        for(int i = 1; i <= 6; i ++){
            hall_detect.uq_phase[i] = (hall_detect.uq_phase[i]) / (hall_detect.uq_count[i]);
        }
        /* calculate the result angle */
        hall_detect.ud_phase[DIRECTION_CCW] = (hall_detect.ud_phase[DIRECTION_CCW] + hall_detect.ud_phase[DIRECTION_CW]) * 0.5f;
        hall_detect.phase_shift = hall_detect.uq_phase[hall_detect.ud_state[DIRECTION_CCW]] - hall_detect.ud_phase[DIRECTION_CCW];

        for(int i = 1; i <= 6; i ++){
            /* calculate the hall sequence for counterclockwise and clockwise */
            hall->sequence[DIRECTION_CCW][hall_detect.uq_sequence[i]] = i;
            hall->sequence[DIRECTION_CW][hall_detect.uq_sequence[7-i]] = i;
            /* calculate the electrical angle for each hall state : ccw direction */
            hall->phase_table[DIRECTION_CCW][i] = angle_nomalize_float(hall_detect.uq_phase[i] - hall_detect.phase_shift);
        }
        /* calculate the electrical angle for each hall state : cw direction */
        for(int i = 1; i <= 6; i ++){
            seq_ccw_temp[hall->sequence[DIRECTION_CCW][i]] = hall->phase_table[DIRECTION_CCW][i];
        }
        for(int i = 2; i <= 6; i ++){
            seq_cw_temp[i-1] = seq_ccw_temp[i];
        } 
        seq_cw_temp[6] = seq_ccw_temp[1];

        for(int i = 1; i <= 6; i ++){
            hall->phase_table[DIRECTION_CW][i] = seq_cw_temp[hall->sequence[DIRECTION_CCW][i]];
        }

        /* program the data to the flash */
        hall_detect_fmc_erase_pages();
        hall_detect_fmc_data_program(hall);
        break;
    }
    default:
        break;
    }
}

/*!
    \brief      hall sensor interrupt handler
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void hall_sensor_interrupt_handler(hall_struct* hall, motor_struct* motor)
{

    if(timer_interrupt_flag_get(HALL_TIMER, TIMER_INT_FLAG_CH3)){
        /* clear interrupt flag */
        timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_CH3);
        hall_speed.timer_overflow ++;
        hall_speed.aver_speed = 0.0f;
        hall_speed.high_speed = 0.0f;
        hall_speed.low_speed = 0.0f;
    }

    if(timer_interrupt_flag_get(HALL_TIMER, TIMER_INT_FLAG_TRG)){
        /* clear interrupt flag */
        timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_TRG);
        /* calculate the speed of the rotor */
        hall_speed_update(hall, motor);
        hall_speed.timer_overflow = 0;

        if(HALL_RUN_NORMAL == hall->hall_running_mode){
            /* update the rotor location information */
            hall_rotor_location_update(hall, motor);

        }else if(HALL_RUN_DETECT == hall->hall_running_mode){
            /* hall_detect step1 : record data of uq detect */
            if(hall_detect.uq_data_record == 1){
                hall->state = hall_read_state();
                /* get the sequence of hall */
                if(hall->seq_temp <= 6)hall_detect.uq_sequence[hall->seq_temp++] = hall->state;
                hall_detect.uq_phase[hall->state] += hall->phase;
                hall_detect.uq_count[hall->state] ++;
            /* hall_detect step3 : record data of ud detect */
            }else{
                hall->seq_temp = 1;
                if(hall_detect.step == 3){
                    /*  ccw direction */
                    if(hall_detect.ud_direction == DIRECTION_CCW){
                        hall_detect.ud_state[DIRECTION_CCW] = hall_read_state();
                        hall_detect.ud_phase[DIRECTION_CCW] = hall->phase;
                        hall->phase = hall->phase + HALL_DETECT_THETA_ADD3;
                        hall_detect.ud_direction = DIRECTION_CW;
                     /* cw direction */
                    }else if(hall_detect.ud_direction == DIRECTION_CW){
                        hall_detect.ud_state[DIRECTION_CW] = hall_read_state();
                        hall_detect.ud_phase[DIRECTION_CW] = hall->phase;
                        hall_detect.step = 4;
                    }
                }
            }
        }
    }
}

/*!
    \brief      configure peripheral clocks related to the hall sensor
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hall_rcu_config(void)
{
    /* enable the clock */
    rcu_periph_clock_enable(HALL_A_GPIO_CLK);
    rcu_periph_clock_enable(HALL_B_GPIO_CLK);
    rcu_periph_clock_enable(HALL_C_GPIO_CLK);
    rcu_periph_clock_enable(HALL_TIMER_CLK);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SYSCFG);
}

/*!
    \brief      configure the GPIO peripheral related to the hall sensor
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hall_gpio_config(void)
{
    /* hall TIMER input--CH0/CH1/CH2 */ 
    gpio_mode_set(HALL_A_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_A_PIN);
    gpio_mode_set(HALL_B_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_B_PIN);
    gpio_mode_set(HALL_C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, HALL_C_PIN);
    
    gpio_af_set(HALL_A_GPIO_PORT, GPIO_AF_1, HALL_A_PIN);
    gpio_af_set(HALL_B_GPIO_PORT, GPIO_AF_1, HALL_B_PIN);
    gpio_af_set(HALL_C_GPIO_PORT, GPIO_AF_1, HALL_C_PIN);
}

/*!
    \brief      configure the TIMER peripheral related to the hall sensor
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hall_timer_config(void)
{
    /* configure hall TIMER interrupt priority */
    nvic_irq_enable(HALL_TIMER_IRQ, 1, 0);

    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icinitpara;
    timer_oc_parameter_struct timer_ocintpara;

    /* TIMER configuration */
    timer_deinit(HALL_TIMER);
    timer_initpara.prescaler         = SYSTEM_FREQUENCE/1000000-1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 49999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV4;
    timer_initpara.repetitioncounter = 0;
    timer_init(HALL_TIMER, &timer_initpara);

    /* CH0 configuration in input capture mode */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_ITS;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 15;
    timer_input_capture_config(HALL_TIMER, TIMER_CH_0, &timer_icinitpara);

    /* CH3 configuration in output timing mode */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_ocintpara.outputstate  = TIMER_CCX_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_channel_output_config(HALL_TIMER, TIMER_CH_3, &timer_ocintpara);
    timer_channel_output_pulse_value_config(HALL_TIMER, TIMER_CH_3, 49999);
    timer_channel_output_mode_config(HALL_TIMER, TIMER_CH_3, TIMER_OC_MODE_TIMING);

    /* TIMER hall sensor mode enable */
    timer_hall_mode_config(HALL_TIMER, TIMER_HALLINTERFACE_ENABLE);
    /* input trigger choose, TIMER internal trigger: TI0 Edge Detector */
    timer_input_trigger_source_select(HALL_TIMER, TIMER_SMCFG_TRGSEL_CI0F_ED);
    /* slave mode: reset mode */
    timer_slave_mode_select(HALL_TIMER, TIMER_SLAVE_MODE_RESTART);

    /* TIMER interrupt enable */
    timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_TRG);
    timer_interrupt_enable(HALL_TIMER, TIMER_INT_TRG);
    timer_interrupt_flag_clear(HALL_TIMER, TIMER_INT_FLAG_CH3);
    timer_interrupt_enable(HALL_TIMER, TIMER_INT_CH3);

    /* timer enable*/
    timer_enable(HALL_TIMER);
}

/*!
    \brief      erase fmc pages from FMC_WRITE_START_ADDR to FMC_WRITE_END_ADDR
    \param[in]  page: the page will be erased
    \param[out] none
    \retval     none
*/
static void hall_detect_fmc_erase_pages(void)
{
    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* clear all pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    fmc_page_erase(FMC_WRITE_START_ADDR );
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
static void hall_detect_fmc_data_program(hall_struct* hall)
{
    uint8_t i;
    uint32_t address = 0x00;
    uint32_t data = 0x00;

    /* unlock the flash program/erase controller */
    fmc_unlock();

    /* program hall shift data */
    address = FMC_HALL_SHIFT_ADDR;
    for(i = 0; i < 8; i ++){
        fmc_halfword_program(address, hall->sequence[DIRECTION_CCW][i]);
        address += 2;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    for(i = 0; i < 8; i ++){
        fmc_halfword_program(address, hall->sequence[DIRECTION_CW][i]);
        address += 2;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    for(i = 0; i < 8; i ++){
        data = (uint32_t)(hall->phase_table[DIRECTION_CCW][i] * 1000.0f);
        fmc_word_program(address, data);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    for(i = 0; i < 8; i ++){
        data = (uint32_t)(hall->phase_table[DIRECTION_CW][i] * 1000.0f);
        fmc_word_program(address, data);
        address += 4;
        fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);
    }
    /* lock the main FMC after the program operation */
    fmc_lock();
}

/*!
    \brief      read hall detect results from flash
    \param[in]  hall: pointer to hall srtuct
    \param[out] none
    \retval     none
*/
static void hall_detect_fmc_data_read(hall_struct* hall)
{
    uint32_t i;
    uint16_t *ptrd1;

    ptrd1 = (uint16_t *)FMC_HALL_SHIFT_ADDR;
    hall->phase_shift = HALL_DETECT_PHASE_SHIFT;

    /* read flash */
    for(i = 0; i < 8; i ++){
        hall->sequence[DIRECTION_CCW][i] = *ptrd1 ++;
    }
    for(i = 0; i < 8; i ++){
        hall->sequence[DIRECTION_CW][i] = *ptrd1 ++;
    }
    for(i = 0; i < 8; i ++){
        hall->phase_table[DIRECTION_CCW][i] = (float)(*ptrd1) * 0.001f;
        ptrd1 += 2U;
        /* add phase_shift */
        hall->phase_table[DIRECTION_CCW][i] = hall->phase_table[DIRECTION_CCW][i] + hall->phase_shift;
        hall->phase_table[DIRECTION_CCW][i] = angle_nomalize_float(hall->phase_table[DIRECTION_CCW][i]);
    }
    for(i = 0; i < 8; i ++){
        hall->phase_table[DIRECTION_CW][i] = (float)(*ptrd1) * 0.001f;
        ptrd1 += 2U;
        /* add phase_shift */
        hall->phase_table[DIRECTION_CW][i] = hall->phase_table[DIRECTION_CW][i] + hall->phase_shift;
        hall->phase_table[DIRECTION_CW][i] = angle_nomalize_float(hall->phase_table[DIRECTION_CW][i]);
    }
}

/*!
    \brief      read hall installation position from macro
    \param[in]  hall: pointer to hall srtuct
    \param[out] none
    \retval     none
*/
static void hall_fix_macro_data_read(hall_struct* hall)
{
    uint8_t sequence_temp[8] = {0};

    /* hall state 1 */
    sequence_temp[HALL_SEQ_CCW_STATE_1] = 1;
    /* hall state 2 */
    sequence_temp[HALL_SEQ_CCW_STATE_2] = 2;
    /* hall state 3 */
    sequence_temp[HALL_SEQ_CCW_STATE_3] = 3;
    /* hall state 4 */
    sequence_temp[HALL_SEQ_CCW_STATE_4] = 4;
    /* hall state 5 */
    sequence_temp[HALL_SEQ_CCW_STATE_5] = 5;
    /* hall state 6 */
    sequence_temp[HALL_SEQ_CCW_STATE_6] = 6;

    for(int i = 1; i < 7; i ++){
        hall->sequence[DIRECTION_CCW][sequence_temp[i]] = i;
        hall->sequence[DIRECTION_CW][sequence_temp[7-i]] = i;
    }

    hall->phase_shift = HALL_FIX_PHASE_SHIFT;
    /* CCW: according to the position of the rotor, determine the electrical angle */
    /* hall state 1 */
    hall->phase_table[DIRECTION_CCW][1] = HALL_PHA_CCW_STATE_1 + hall->phase_shift ;
    /* hall state 2 */
    hall->phase_table[DIRECTION_CCW][2] = HALL_PHA_CCW_STATE_2 + hall->phase_shift ;
    /* hall state 3 */
    hall->phase_table[DIRECTION_CCW][3] = HALL_PHA_CCW_STATE_3 + hall->phase_shift ;
    /* hall state 4 */
    hall->phase_table[DIRECTION_CCW][4] = HALL_PHA_CCW_STATE_4 + hall->phase_shift ;
    /* hall state 5 */
    hall->phase_table[DIRECTION_CCW][5] = HALL_PHA_CCW_STATE_5 + hall->phase_shift ;
    /* hall state 6 */
    hall->phase_table[DIRECTION_CCW][6] = HALL_PHA_CCW_STATE_6 + hall->phase_shift ;

    /* CW: according to the position of the rotor, determine the electrical angle */
    /* hall state 1 */
    hall->phase_table[DIRECTION_CW][1] = HALL_PHA_CCW_STATE_1 + hall->phase_shift + MOTOR_PI / 3.0f;
    /* hall state 2 */
    hall->phase_table[DIRECTION_CW][2] = HALL_PHA_CCW_STATE_2 + hall->phase_shift + MOTOR_PI / 3.0f;
    /* hall state 3 */
    hall->phase_table[DIRECTION_CW][3] = HALL_PHA_CCW_STATE_3 + hall->phase_shift + MOTOR_PI / 3.0f;
    /* hall state 4 */
    hall->phase_table[DIRECTION_CW][4] = HALL_PHA_CCW_STATE_4 + hall->phase_shift + MOTOR_PI / 3.0f;
    /* hall state 5 */
    hall->phase_table[DIRECTION_CW][5] = HALL_PHA_CCW_STATE_5 + hall->phase_shift + MOTOR_PI / 3.0f;
    /* hall state 6 */
    hall->phase_table[DIRECTION_CW][6] = HALL_PHA_CCW_STATE_6 + hall->phase_shift + MOTOR_PI / 3.0f;

    for(int i = 1; i < 8; i ++){
        hall->phase_table[DIRECTION_CCW][i] = angle_nomalize_float(hall->phase_table[DIRECTION_CCW][i]);
        hall->phase_table[DIRECTION_CW][i] = angle_nomalize_float(hall->phase_table[DIRECTION_CW][i]);
    }
}

/*!
    \brief      read hall state
    \param[in]  none
    \param[out] none
    \retval     hall state, 1-6
*/
static uint8_t hall_read_state(void)
{
    uint8_t hall_a_state, hall_b_state, hall_c_state;

    hall_a_state = (uint8_t)((GPIO_ISTAT(HALL_A_GPIO_PORT) & HALL_A_PIN) >> HALL_A_SHIFT);
    hall_b_state = (uint8_t)((GPIO_ISTAT(HALL_B_GPIO_PORT) & HALL_B_PIN) >> HALL_B_SHIFT);
    hall_c_state = (uint8_t)((GPIO_ISTAT(HALL_C_GPIO_PORT) & HALL_C_PIN) >> HALL_C_SHIFT);
    /* in order of CBA */
    hall_a_state = (uint8_t)((hall_c_state << 2) | (hall_b_state << 1) | hall_a_state);

    return (hall_a_state);
}

/*!
    \brief      update rotor speed
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
static void hall_speed_update(hall_struct* hall, motor_struct* motor)
{
    uint32_t hall_timer_count;
    uint8_t hall_pre_state;
    float speed_temp;

    /* update hall state */
    hall_pre_state = hall->state;
    hall->state = hall_read_state();

    /* calculate the speed of the rotor through the last hall state */
    hall_timer_count = (timer_channel_capture_value_register_read(HALL_TIMER, TIMER_CH_0) + 1 + (hall_speed.timer_overflow) * 50000);
    speed_temp = HALL_SPEED_FACTOR / hall_timer_count;
    /* avoid the effects of Hall signal burrs */
    if(speed_temp < HALL_SPEED_THRESHOLD){
        /* speed filter */
        hall_speed.low_speed = moving_average_float(hall_speed.speed_array, HALL_AVE_SPEED_LEN, &hall_speed.speed_sum, speed_temp, &hall_speed.speed_index);
        /* judge whether the change of hall state is consistent with the direction */
        if(hall->sequence[motor->direction][hall->state] - 1 != (hall->sequence[motor->direction][hall_pre_state]) % 6)
        hall_speed.low_speed = 0.0f;
        /* speed calculate for high speed */
        if(hall_speed.low_speed > 500.0f){
            hall_speed.timer_count_sum += hall_timer_count;
            if(hall->sequence[motor->direction][hall->state] == 1){
                hall_speed.high_speed = HALL_HIGH_SPEED_FACTOR / hall_speed.timer_count_sum;
                hall_speed.timer_count_sum = 0;
            }
        }
    }
    
    if(hall_speed.low_speed > 1500.0f){
        hall_speed.aver_speed = hall_speed.high_speed;
    }else{
        hall_speed.aver_speed = hall_speed.low_speed;
    }
}

/*!
    \brief      update the rotor location information with hall state changing
    \param[in]  hall: pointer to hall srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
static void hall_rotor_location_update(hall_struct* hall, motor_struct* motor)
{
    /* correct the angle directly to the value recorded in the phase_table, 
       when the hall state changes for the first time after motor start */
    if(hall->direct_correct_flag == 0){
        hall->phase = hall->phase_table[motor->direction][hall->state];
        hall->phase_error = 0.0f;
        hall->direct_correct_flag = 1;
    }else{
        /* calculate the error of the angle estimate, */
        if(motor->direction == DIRECTION_CW){
            hall->phase_error = hall->phase - hall->phase_table[motor->direction][hall->state];
        }else{
            hall->phase_error = hall->phase_table[motor->direction][hall->state] - hall->phase;
        }
        /* angle process */
        if(hall->phase_error > MOTOR_PI){
            hall->phase_error = -(MOTOR_PI_2 - hall->phase_error);
        }else if(hall->phase_error < -MOTOR_PI){
            hall->phase_error = MOTOR_PI_2 + hall->phase_error;
        }
    }
    /* clear the number of corrections */
    hall->phase_correct_cnt = 0;
}
