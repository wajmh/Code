/*!
    \file    motor_protect.c
    \brief   motor software protection program

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

#include "motor_protect.h"
#include "gdfoc_config.h"

/* local function declaration */
/* configure peripheral clock related to the motor protect */
static void motor_protect_rcu_config(void);
/* configure the GPIO peripheral related to the motor protect */
static void motor_protect_gpio_config(void);
/* configure the ADC peripheral related to the motor protect */
static void motor_protect_adc_config(void);
/* configure the DMA peripheral related to the motor protect */
static void motor_protect_dma_config(void);
/* check whether the motor is undervoltage or overvoltage */
static void motor_protect_voltage_check(void);
/* check whether the motor overcurrent */
static void motor_protect_over_current_check(void);
/* check whether the motor is locked */
static void motor_protect_motor_locked_check(void);
/* check whether the bus voltage exceeds threshold in FOC interrupt */
static void motor_protect_fast_voltage_check(void);
/* check whether the phase current exceeds threshold in FOC interrupt */
static void motor_protect_fast_over_current_check(void);
/* static variables definition */
/* ADC sampled bus voltage data */
static uint16_t adc_regular_buffer = 0;
/* the amplitude table of the ualpha and ubeta */
static float motor_lock_voltage[40] = {
    /* 0~900rpm */
    1.50f,  1.50f,  1.50f,  1.50f,  1.50f,  1.85f,  2.15f,  2.55f,  2.90f,  3.33f,
    /* 1000~1900rpm */
    3.69f,  4.05f,  4.35f,  4.65f,  5.02f,  5.35f,  5.70f,  6.07f,  6.42f,  6.75f,
    /* 2000~2900rpm */
    7.12f,  7.47f,  7.82f,  8.17f,  8.52f,  8.86f,  9.22f,  9.57f,  9.92f, 10.27f,
    /* 3000~3900rpm */
    10.62f, 10.96f, 11.31f, 11.66f, 12.01f, 12.35f, 12.70f, 13.02f, 13.02f, 13.02f
};

/* global variables definition */
/* motor protect struct */
motor_protect_struct motor_protect;

/*!
    \brief      initialize the motor protect peripherals
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_peripheral_init(void)
{
    /* configure the RCU peripheral */
    motor_protect_rcu_config();
    /* configure the GPIO peripheral */
    motor_protect_gpio_config();
    /* configure the DMA peripheral */
    motor_protect_dma_config();
    /* configure the ADC peripheral */
    motor_protect_adc_config();
}

/*!
    \brief      initialize the motor protect variables
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_var_init(void)
{
    /* initialize motor protect struct variables */
    motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] = 0;
    motor_protect.fault_iteration[FAULT_SPEED_ERROR] = 0;
    motor_protect.fault_iteration[FAULT_OVER_CURRENT] = 0;
    motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] = 0;
    for(int i = 0; i < 3; i ++) {
        motor_protect.low_current_count[i] = 0;
    }
    motor_protect.pwm_count = 0;

    motor_protect.start_time = (int16_t)(motor.speed_ref * START_TIME_SET_FACTOR);
    motor_protect.pre_speed_ref = motor.speed_ref;
}

/*!
    \brief      motor protect process
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_check(void)
{
    if(motor.state != MC_STATE_INIT) {
        /* check whether the motor is undervoltage or overvoltage */
        motor_protect_voltage_check();
        adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);

        if(motor.state == MC_STATE_RUNNING) {
            if((rotor_angle.pos_mode != HALL_DETECT_MODE)&&(rotor_angle.pos_mode != ANGLE_CUSTOM_MODE)) {
                /* check whether the motor overcurrent */
                motor_protect_over_current_check();
                /* check whether the motor is locked */
                motor_protect_motor_locked_check();
            }
        }
    }
}

/*!
    \brief      fast motor protect process in FOC interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_fast_check(void)
{

    if(motor.fault != FAULT_NONE) {
        return;
    }

    motor_protect_fast_voltage_check();
    motor_protect_fast_over_current_check();
}

/*!
    \brief      check whether the motor phase loss
    \param[in]  none
    \param[out] none
    \retval     none
*/
void motor_protect_phase_loss_check(void)
{
    if(rotor_angle.pos_mode != HALL_DETECT_MODE) {
        if(motor_protect.pwm_count < PHASE_LOSS_COUNT_MAX) {
            motor_protect.pwm_count ++;
            /* compare the phase current with the set threshold */
            if((motor.ia < LOW_CURRENT_THRESHOLD) && (motor.ia > -LOW_CURRENT_THRESHOLD)) {
                motor_protect.low_current_count[0] ++;
            }
            if((motor.ib < LOW_CURRENT_THRESHOLD) && (motor.ib > -LOW_CURRENT_THRESHOLD)) {
                motor_protect.low_current_count[1] ++;
            }
            if((motor.ic < LOW_CURRENT_THRESHOLD) && (motor.ic > -LOW_CURRENT_THRESHOLD)) {
                motor_protect.low_current_count[2] ++;
            }
        } else {
            motor_protect.pwm_count = 0;
            /* the phase loss fault count reaches the set value */
            if((motor_protect.low_current_count[0] > PHASE_LOSS_COUNT_THRESHOLD) ||
                    (motor_protect.low_current_count[1] > PHASE_LOSS_COUNT_THRESHOLD) ||
                    (motor_protect.low_current_count[2] > PHASE_LOSS_COUNT_THRESHOLD)) {
                motor.fault = FAULT_PHASE_LOSS;
            }
            motor_protect.low_current_count[0] = 0;
            motor_protect.low_current_count[1] = 0;
            motor_protect.low_current_count[2] = 0;
        }
    }
}

/*!
    \brief      configure peripheral clocks related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_rcu_config(void)
{
    /* configure the RCU peripheral */
    rcu_periph_clock_enable(BUS_VOLTAGE_RCU);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMAMUX);
}

/*!
    \brief      configure the GPIO peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_gpio_config(void)
{
    /* configure the GPIO peripheral */
    gpio_mode_set(BUS_VOLTAGE_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BUS_VOLTAGE_PIN);
}

/*!
    \brief      configure the ADC peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_adc_config(void)
{
    /* configure ADC data alignment */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* configure ADC regular channel trigger */
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
    
    /* configure ADC regular channel length */
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1);
    adc_routine_channel_config(ADC0, 0, BUS_VOLTAGE_CHANNEL, ADC_SAMPLETIME_7POINT5);

    /* enable DMA request */
    adc_dma_mode_enable(ADC0,ADC_ROUTINE_CHANNEL);
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
}

/*!
    \brief      configure the DMA peripheral related to the motor protect
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* DMA configuration */
    dma_deinit(DMA0, DMA_CH0);
    dma_init_struct.request      = DMA_REQUEST_ADC0_ROUTINE;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr  = (uint32_t)(&adc_regular_buffer);
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.number       = 1;
    dma_init_struct.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.priority     = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH0, &dma_init_struct);
    dma_circulation_enable(DMA0, DMA_CH0);
    dma_memory_to_memory_disable(DMA0, DMA_CH0);

    /* enable the DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}

/*!
    \brief      check whether the motor is undervoltage or overvoltage
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_voltage_check(void)
{
    /* calculate the value of the bus voltage */
    motor_protect.v_bus = adc_regular_buffer * BUS_VOLTAGE_CALC_FACTOR;

    /* check whether the bus voltage exceeds the threshold */
    if((motor_protect.v_bus < BUS_VOLTAGE_THRESHOLD_LOW) ||
            (motor_protect.v_bus > BUS_VOLTAGE_THRESHOLD_HIGH)) {
        motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR] = 0;
        if(motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] < VOLTAGE_ERROR_COUNT_MAX) {
            /* the fault count accumulation */
            motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR]++;
        } else {
            /* the fault count reaches the set value */
            motor.fault = FAULT_VOLTAGE_ERROR;
        }
        /* check whether the bus voltage returns to normal */
    } else if(motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] > 0) {
        motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR]++;
        /* the recovery count reaches the set value */
        if(motor_protect.fault_recovery[FAULT_VOLTAGE_ERROR] >= VOLTAGE_ERROR_RECOV_COUNT_MAX) {
            motor_protect.fault_iteration[FAULT_VOLTAGE_ERROR] = 0;
            /* clear the fault */
            if(motor.fault == FAULT_VOLTAGE_ERROR) {
                motor.fault = FAULT_NONE;
            }
        }
    }
}

/*!
    \brief      check whether the motor overcurrent
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_over_current_check(void)
{
    /* calculate the amplitude of the phase current */
    motor_protect.i_phase_amplit = sqrt_float(motor.ialpha * motor.ialpha + motor.ibeta * motor.ibeta);

    /* check whether the current exceeds the threshold */
    if(motor_protect.i_phase_amplit > PHASE_CURRENT_THRESHOLD_HIGH) {
        motor_protect.fault_recovery[FAULT_OVER_CURRENT] = 0;
        motor_protect.fault_iteration[FAULT_OVER_CURRENT]++;
        /* the fault count reaches the set value */
        if(motor_protect.fault_iteration[FAULT_OVER_CURRENT] >= OVER_CURRENT_COUNT_MAX) {
            motor.fault = FAULT_OVER_CURRENT;
        }
        /* check whether the current returns to normal */
    } else if(motor_protect.fault_iteration[FAULT_OVER_CURRENT] > 0) {
        motor_protect.fault_recovery[FAULT_OVER_CURRENT]++;
        /* the recovery count reaches the set value */
        if(motor_protect.fault_recovery[FAULT_OVER_CURRENT] >= OVER_CURRENT_RECOV_COUNT_MAX) {
            motor_protect.fault_iteration[FAULT_OVER_CURRENT] = 0;
        }
    }
}

/*!
    \brief      check whether the motor is undervoltage or overvoltage in FOC interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_fast_voltage_check(void)
{
    motor_protect.v_bus = adc_regular_buffer * BUS_VOLTAGE_CALC_FACTOR;

    if((motor_protect.v_bus < BUS_VOLTAGE_THRESHOLD_LOW) ||
            (motor_protect.v_bus > BUS_VOLTAGE_THRESHOLD_HIGH)) {
            motor.fault = FAULT_VOLTAGE_ERROR;
    }

    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
}

/*!
    \brief      check whether the motor overcurrent in FOC interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_fast_over_current_check(void)
{
    const float iq_abs = (motor.iq >= 0.0f) ? motor.iq : -motor.iq;

    if(iq_abs > PHASE_CURRENT_THRESHOLD_HIGH) {
            motor_protect.i_phase_amplit = iq_abs;//记录过流值
            motor.fault = FAULT_OVER_CURRENT;
        }
}

/*!
    \brief      check whether the motor is locked
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void motor_protect_motor_locked_check(void)
{
    float voltage_threshold;
    /* calculate the amplitude of the ualpha and ubeta */
    motor_protect.v_phase_amplit = sqrt_float(motor.ualpha * motor.ualpha + motor.ubeta * motor.ubeta);

    if(motor_protect.start_time > 0) {
        motor_protect.start_time --;
    } else {
        /* check whether the mechanical speed exceeds the threshold */
        if((motor.mech_speed > SPEED_THRESHOLD_HIGH) || (motor.mech_speed < SPEED_THRESHOLD_LOW)) {
            motor_protect.fault_recovery[FAULT_SPEED_ERROR] = 0;
            motor_protect.fault_iteration[FAULT_SPEED_ERROR] ++;
            /* the fault count reaches the set value */
            if(motor_protect.fault_iteration[FAULT_SPEED_ERROR] >= SPEED_ERROR_COUNT_MAX) {
                motor.fault = FAULT_SPEED_ERROR;
            }
            /* check whether the speed returns to normal */
        } else if(motor_protect.fault_iteration[FAULT_SPEED_ERROR] > 0) {
            motor_protect.fault_recovery[FAULT_SPEED_ERROR]++;
            /* the recovery count reaches the set value */
            if(motor_protect.fault_recovery[FAULT_SPEED_ERROR] >= SPEED_ERROR_RECOV_COUNT_MAX) {
                motor_protect.fault_iteration[FAULT_SPEED_ERROR] = 0;
            }
        }

        /* calculate the voltage threshold */
        voltage_threshold = MOTOR_LOCKED_VOLT_RATIO * motor_lock_voltage[(uint8_t)(motor.mech_speed * 0.01f)];
        /* check whether the phase voltage amplitude exceeds the threshold */
        if(motor_protect.v_phase_amplit < voltage_threshold) {
            motor_protect.fault_recovery[FAULT_MOTOR_LOCKED] = 0;
            motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] ++;
            /* the fault count reaches the set value */
            if(motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] >= MOTOR_LOCKED_COUNT_MAX) {
                motor.fault = FAULT_MOTOR_LOCKED;
            }
            /* check whether the phase voltage amplitude returns to normal */
        } else if(motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] > 0) {
            motor_protect.fault_recovery[FAULT_MOTOR_LOCKED]++;
            /* the recovery count reaches the set value */
            if(motor_protect.fault_recovery[FAULT_MOTOR_LOCKED] >= MOTOR_LOCKED_RECOV_COUNT_MAX) {
                motor_protect.fault_iteration[FAULT_MOTOR_LOCKED] = 0;
            }
        }
    }

    /* check whether the speed reference changes */
    if(motor.speed_ref != motor_protect.pre_speed_ref) {
        /* setting the motor startup time */
        if(motor.speed_ref > motor.mech_speed) {
            motor_protect.start_time = (uint32_t)((motor.speed_ref - motor.mech_speed) * START_TIME_SET_FACTOR);
        } else {
            motor_protect.start_time = (uint32_t)((motor.mech_speed - motor.speed_ref) * START_TIME_SET_FACTOR);
        }
    }
    motor_protect.pre_speed_ref = motor.speed_ref;
}
