/*!
    \file    communication.c
    \brief   communicate with the upper computer through the USART peripheral

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

#include "communication.h"
#include "gdfoc_config.h"
#include "string.h"

/* macro definition */
/* usart communication protocol parameters */
/* frame head */
#define USART_FRAME_HEAD1                   (0xAA)                              /* the frame head 1 of the usart communications protocol */
#define USART_FRAME_HEAD2                   (0x55)                              /* the frame head 2 of the usart communications protocol */
/* function code of the received command */
#define USART_MOTORCTL_FUNCODE              (0x01)                              /* the usart function code of the motor control command */
#define USART_MOTORSET_FUNCODE              (0x02)                              /* the usart function code of the motor setting command */
#define USART_READPID_FUNCODE               (0x03)                              /* the usart function code of the read pid parameter command */
#define USART_DEFAULTPID_FUNCODE            (0x04)                              /* the usart function code of the restore pid to default parameter and read command */
#define USART_WRITEPID_FUNCODE              (0x05)                              /* the usart function code of the write pid parameter command */
/* function code of the transmit data */
#define USART_WAVEFORM_FUNCODE              (0x21)                              /* the function code of the waveform data frame */
#define USART_MOTORSTATE_FUNCODE            (0x11)                              /* the function code of the motor state data frame */

/* DMA transmit data frame length */
#define USART_RESPONSE_FARME_LENGTH         (6U)                                /* the length of the usart transmit return check frame */
#define USART_RESPONSEPID_FARME_LENGTH      (USART_RECEIVE_FARME_LENGTH)        /* the length of the usart transmit PID data frame */
#define USART_WAVEFORM_DATABYTE             (USART_WAVEFORM_DATA_NUM * 2U)      /* the number of the transmit waveform data byte */
#define USART_WAVEFORM_FARME_LENGTH         (USART_WAVEFORM_DATABYTE + 5U)      /* the length of the usart transmit waveform data frame */
#define USART_MOTORSTATE_DATABYTE           (9U)                                /* the number of the motor state data byte */
#define USART_MOTORSTATE_FARME_LENGTH       (14U)                               /* the length of the usart transmit motor state data frame */

/* PID data number */
#define USART_PID_DATA_NUM                  (18U)                               /* the number of pid data: 3*6*/

/* local function declaration */
/* configure peripheral clocks related to the usart communication */
static void usart_rcu_config(void);
/* configure the GPIO peripheral related to the usart communication */
static void usart_gpio_config(void);
/* configure the USART peripheral related to the usart communication */
static void usart_config(void);
/* configure the DMA peripheral related to the usart communication */
static void usart_dma_config(void);
/* parse the received command by the usart */
static void usart_receive_command_parse(void);
/* transmit the response data for the received commands using the usart */
static void usart_response_data_transmit(void);

/* static variables definition */
/* DMA busy flag */
static uint8_t transmit_dma_busy = 0;
/* DMA transmit data array */
static uint8_t transmit_data[USART_RESPONSEPID_FARME_LENGTH] = {0};
/* DMA receive data array */
static uint8_t receive_data[USART_RECEIVE_FARME_LENGTH] = {0};
/* PID data in uint32_t format */
static uint32_t pid_buffer[USART_PID_DATA_NUM] = {0};

/* global variables definition */
/* the usart transmit data srtuct */
comm_usart_transmit_struct usart_transmit;
/* the usart receive data srtuct */
comm_usart_receive_struct usart_receive;
/* the pid data srtuct received and transmitted by the usart */
comm_usart_pid_struct usart_pid;

/*!
    \brief      initialize the usart communication peripherals
    \param[in]  none
    \param[out] none
    \retval     none
*/
void comm_init(void)
{
    /* configure the RCU peripheral */
    usart_rcu_config();
    /* configure the GPIO peripheral */
    usart_gpio_config();
    /* configure the USART peripheral */
    usart_config();
    /* configure the DMA peripheral */
//    usart_dma_config();
    /* configure the NVIC */
//    nvic_irq_enable(COMM_RECEIVE_IRQ, 1, 0);
}

/*!
    \brief      transmit the motor state data using the usart
    \param[in]  none
    \param[out] none
    \retval     none
*/
void comm_usart_motor_state_data_transmit(void)
{
    uint8_t index = 0, i = 0;
    uint8_t transmit_sum = 0;
    static uint16_t time_count = 0;
    
    if(time_count <= 999){
        time_count++;
    }else{
        time_count = 0;
        if(usart_transmit.transmit_state == STATE_TRANSMITE_WAVEFORM)
            usart_transmit.transmit_state = STATE_TRANSMITE_MOTORSTATE;
    }

    /* check whether the response data needs to be transmit */
    if(usart_transmit.transmit_state == STATE_TRANSMITE_MOTORSTATE) {
        if(dma_flag_get(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF) == SET) {
            /* clear the transmit finish flag */
            dma_flag_clear(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF);
            transmit_dma_busy = 0;
        }
        if(transmit_dma_busy == 0) {
            transmit_dma_busy = 1;
            /* disable DMA channel */
            dma_channel_disable(USART_DMA, USART_TX_DMA_CH);
            DMA_CHCNT_TX = USART_MOTORSTATE_FARME_LENGTH;

            /* write the sending array */
            transmit_data[index++] = USART_FRAME_HEAD2;
            transmit_data[index++] = USART_FRAME_HEAD1;
            transmit_data[index++] = USART_MOTORSTATE_FUNCODE;
            transmit_data[index++] = USART_MOTORSTATE_DATABYTE;

            transmit_data[index++] = motor.running_mode;
            transmit_data[index++] = pwm_current.samp_mode;
            transmit_data[index++] = pwm_current.pwm_mode;
            transmit_data[index++] = motor.direction;
            transmit_data[index++] = (motor.speed_ref >> 24);
            transmit_data[index++] = (motor.speed_ref >> 16);
            transmit_data[index++] = (motor.speed_ref >> 8);
            transmit_data[index++] = (motor.speed_ref);
            transmit_data[index++] = motor.state;
            
            /* sum check */
            for(i = 0; i < (USART_MOTORSTATE_FARME_LENGTH - 1); i ++) {
                transmit_sum += transmit_data[i];
            }
            transmit_data[index] = transmit_sum;

            /* enable DMA channel */
            dma_channel_enable(USART_DMA, USART_TX_DMA_CH);
            usart_transmit.transmit_state = STATE_TRANSMITE_WAVEFORM;
        }
    }
}

/*!
    \brief      transmit the waveform data using the usart
    \param[in]  none
    \param[out] none
    \retval     none
*/
void comm_usart_waveform_data_transmit(void)
{
    uint8_t index = 0, i = 0;
    int8_t transmit_sum = 0;

    if(dma_flag_get(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF) == SET) {
        /* clear the transmit finish flag */
        dma_flag_clear(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF);
        transmit_dma_busy = 0;
    }

    /* check whether the response data needs to be transmit */
    if(usart_transmit.transmit_state == STATE_TRANSMITE_WAVEFORM) {
        if(transmit_dma_busy == 0) {
            transmit_dma_busy = 1;
            /* disable DMA channel */
            dma_channel_disable(USART_DMA, USART_TX_DMA_CH);
            DMA_CHCNT_TX = USART_WAVEFORM_FARME_LENGTH;

            /* write the sending array */
            transmit_data[index++] = USART_FRAME_HEAD2;
            transmit_data[index++] = USART_FRAME_HEAD1;
            transmit_data[index++] = USART_WAVEFORM_FUNCODE;
            transmit_data[index++] = USART_WAVEFORM_DATABYTE;

            for(i = 0; i < USART_WAVEFORM_DATA_NUM; i ++) {
                transmit_data[index++] = usart_transmit.data[i] >> 8;
                transmit_data[index++] = usart_transmit.data[i];
            }

            /* sum check */
            for(i = 0; i < (USART_WAVEFORM_FARME_LENGTH - 1); i ++) {
                transmit_sum += transmit_data[i];
            }
            transmit_data[index] = transmit_sum;

            /* enable DMA channel */
            dma_channel_enable(USART_DMA, USART_TX_DMA_CH);
        }
    }
}

/*!
    \brief      usart receive communication interrupt handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void comm_usart_interrupt_handler(void)
{
    if((dma_interrupt_flag_get(USART_DMA, USART_RX_DMA_CH, DMA_FLAG_FTF)) != RESET) {
        /* clear the interrupt flag */
        dma_interrupt_flag_clear(USART_DMA, USART_RX_DMA_CH, DMA_FLAG_FTF);
        if(usart_receive.receive_state == STATE_RECEIVE_IDLE) {
            usart_receive.receive_state = STATE_RECEIVE_PARSE;
            memcpy(usart_receive.buffer, receive_data, USART_RECEIVE_FARME_LENGTH);
        }
    }
}

/*!
    \brief      parse the received data by the USART
    \param[in]  none
    \param[out] none
    \retval     none
*/
void comm_usart_receive_data_parse(void)
{
    uint8_t i = 0;
    uint8_t receive_length = 0;
    uint8_t receive_sum = 0;

    if(usart_receive.receive_state == STATE_RECEIVE_PARSE) {
        /* check the frame head */
        if((usart_receive.buffer[0] == USART_FRAME_HEAD1) && (usart_receive.buffer[1] == USART_FRAME_HEAD2)) {
            receive_length = usart_receive.buffer[3] + 4;
            /* sum check */
            for(i = 0; i < receive_length; i++) {
                receive_sum += usart_receive.buffer[i];
            }
            /* parse the received data */
            if(receive_sum == usart_receive.buffer[i]) {
                usart_receive.function_code = usart_receive.buffer[2];
                usart_receive.return_check_flag = usart_receive.buffer[i + 1];
                usart_receive.data_sum = receive_sum;
                /* parse the received command by the usart */
                usart_receive_command_parse();
                /* transmit the response data */
                if(usart_receive.return_check_flag == 1) {
                    /* change usart communication state */
                    usart_transmit.transmit_state = STATE_TRANSMIT_RESPONSE;
                } else {
                    /* change usart communication state */
                    usart_receive.receive_state = STATE_RECEIVE_IDLE;
                }
            } else {
                /* change usart communication state */
                usart_receive.receive_state = STATE_RECEIVE_IDLE;
            }
        }else{
            /* change usart communication state */
            usart_receive.receive_state = STATE_RECEIVE_IDLE;
        }
    }
    if(usart_transmit.transmit_state == STATE_TRANSMIT_RESPONSE) {
        /* transmit the response data */
        usart_response_data_transmit();
    }
}

/*!
    \brief      configure peripheral clocks related to the usart communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_rcu_config(void)
{
    /* enable the clock */
    rcu_periph_clock_enable(USART_RX_CLK);
    rcu_periph_clock_enable(USART_TX_CLK);
    rcu_periph_clock_enable(COMM_USART_CLK);
    rcu_periph_clock_enable(USART_DMA_RCU);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_DMAMUX);
}

/*!
    \brief      configure the GPIO peripheral related to the usart communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_gpio_config(void)
{
    /* USART_RX */
    gpio_mode_set(USART_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_RX_PIN);
    gpio_af_set(USART_RX_PORT, GPIO_AF_1, USART_RX_PIN);
    /* USART_TX */
    gpio_mode_set(USART_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN);
    gpio_output_options_set(USART_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, USART_TX_PIN);
    gpio_af_set(USART_TX_PORT, GPIO_AF_1, USART_TX_PIN);
}

/*!
    \brief      configure the usart peripheral related to the usart communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_config(void)
{
    /* configure the usart peripheral */
    usart_deinit(COMM_USART);
    usart_baudrate_set(COMM_USART, USART_BPS);
    usart_stop_bit_set(COMM_USART, USART_STB_1BIT);
    usart_word_length_set(COMM_USART, USART_WL_8BIT);
    usart_parity_config(COMM_USART, USART_PM_NONE);
    usart_hardware_flow_rts_config(COMM_USART, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(COMM_USART, USART_CTS_DISABLE);
    usart_receive_config(COMM_USART, USART_RECEIVE_ENABLE);
    usart_transmit_config(COMM_USART, USART_TRANSMIT_ENABLE);

    /* clear USART_FLAG_TC flag */
    usart_flag_clear(COMM_USART, USART_FLAG_TC);
    usart_dma_transmit_config(COMM_USART, USART_DENT_ENABLE);

    /* clear USART_FLAG_RBNE flag */
    usart_flag_clear(COMM_USART, USART_FLAG_RBNE);
    usart_dma_receive_config(COMM_USART, USART_DENR_ENABLE);

    /* enable the usart */
    usart_enable(COMM_USART);
}

/*!
    \brief      configure the DMA peripheral related to the usart communication
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_dma_config(void)
{
    dma_parameter_struct dma_init_struct;

    /* configure the DMA to transmit the data to be sent */
    dma_deinit(USART_DMA, USART_TX_DMA_CH);
    dma_init_struct.request      = DMA_REQUEST_USART2_TX;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr  = (uint32_t)transmit_data;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = 0;
    dma_init_struct.periph_addr  = USART_TX_ADDRESS;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(USART_DMA, USART_TX_DMA_CH, &dma_init_struct);

    /* disable DMA circulation mode */
    dma_circulation_disable(USART_DMA, USART_TX_DMA_CH);
    dma_memory_to_memory_disable(USART_DMA, USART_TX_DMA_CH);

    /* configure the DMA to transmit the data to be sent */
    dma_deinit(USART_DMA, USART_RX_DMA_CH);
    dma_init_struct.request      = DMA_REQUEST_USART2_RX;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr  = (uint32_t)receive_data;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number       = USART_RECEIVE_FARME_LENGTH;
    dma_init_struct.periph_addr  = USART_RX_ADDRESS;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(USART_DMA, USART_RX_DMA_CH, &dma_init_struct);

    /* enable DMA circulation mode */
    dma_circulation_enable(USART_DMA, USART_RX_DMA_CH);
    dma_memory_to_memory_disable(USART_DMA, USART_RX_DMA_CH);
    /* enable the USART_RX_DMA_CH full transfer finish interrupt */
    dma_interrupt_flag_clear(USART_DMA, USART_RX_DMA_CH, DMA_FLAG_FTF);
    dma_interrupt_enable(USART_DMA, USART_RX_DMA_CH, DMA_INT_FTF);
    /* dma channel enable */
    dma_channel_enable(USART_DMA, USART_RX_DMA_CH);
}

/*!
    \brief      parse the received data by the USART
    \param[in]  none
    \param[out] none
    \retval     none
*/

static void usart_receive_command_parse(void)
{
    uint8_t index = 4;
    uint32_t speed_ref = 0;
    float *pid_struct_ptr = (float *)(&usart_pid);

    /* determine the received command */
    if(usart_receive.function_code == USART_MOTORCTL_FUNCODE) {
        /* change motor command enum */
        switch(usart_receive.buffer[index]) {
            case 0x01:
                motor.command = MC_START;
                break;
            case 0x02:
                motor.command = MC_STOP;
                break;
            default:
                break;
        }
    } else if(usart_receive.function_code == USART_MOTORSET_FUNCODE) {
        /* read the position mode */
        motor.running_mode = (mc_running_mode_enum)(usart_receive.buffer[index]);
        /* read the current sample mode */
        pwm_current.samp_mode = (current_mode)(usart_receive.buffer[index + 1]);
        /* read the pwm mode */
        pwm_current.pwm_mode = (modul_mode)(usart_receive.buffer[index + 2]);
        /* read the direction_set */
        if(MC_STATE_RUNNING != motor.state)motor.direction = (mc_direction_enum)(usart_receive.buffer[index + 3]);
        /* read the speed_ref_set */
        speed_ref = (uint32_t)((usart_receive.buffer[index + 4] << 24) +
                              (usart_receive.buffer[index + 5] << 16) +
                              (usart_receive.buffer[index + 6] << 8) + usart_receive.buffer[index + 7]);

        if(speed_ref > SPEED_REFERENCE_MAX){
            motor.speed_ref = SPEED_REFERENCE_MAX;
        } else if(speed_ref <= SPEED_REFERENCE_MIN){
            motor.speed_ref = SPEED_REFERENCE_MIN;
        }else{
            motor.speed_ref = speed_ref;
        }

    } else if(usart_receive.function_code == USART_READPID_FUNCODE) {


    } else if(usart_receive.function_code == USART_DEFAULTPID_FUNCODE) {
        /* init pid of speed pid */
        motor.speed_pid->kp = SPEED_LOOP_KP;
        motor.speed_pid->ki = SPEED_LOOP_KI;
        motor.speed_pid->kd = 0.0f;
        /* init pid of torque pid */
        motor.torque_pid->kp = CURRENT_LOOP_KP;
        motor.torque_pid->ki = CURRENT_LOOP_KI;
        motor.torque_pid->kd = 0.0f;
        /* init pid of flux pid */
        motor.flux_pid->kp = CURRENT_LOOP_KP;
        motor.flux_pid->ki = CURRENT_LOOP_KI;
        motor.flux_pid->kd = 0.0f;
        /* init pid struct */
        for(int i = 0; i < USART_PID_DATA_NUM-6; i++) {
            /* PID parameters writing */
            *pid_struct_ptr = 0.0f;
            pid_struct_ptr++;
        }

    } else if(usart_receive.function_code == USART_WRITEPID_FUNCODE) {
        for(int i = 0; i < USART_PID_DATA_NUM; i++) {
            pid_buffer[i] = ((usart_receive.buffer[index] << 24) + (usart_receive.buffer[index + 1] << 16) +
                             (usart_receive.buffer[index + 2] << 8) + usart_receive.buffer[index + 3]);
            index += 4;
            if(i >= 6) {
                /* PID parameters writing */
                *pid_struct_ptr = *((float *)&pid_buffer[i]);
                pid_struct_ptr++;
            }
        }
        /* init pid of speed pid */
        motor.speed_pid->kp = *((float *)&pid_buffer[0]);
        motor.speed_pid->ki = *((float *)&pid_buffer[1]);
        motor.speed_pid->kd = *((float *)&pid_buffer[2]);
        /* init pid of torque pid */
        motor.torque_pid->kp = *((float *)&pid_buffer[3]);
        motor.torque_pid->ki = *((float *)&pid_buffer[4]);
        motor.torque_pid->kd = *((float *)&pid_buffer[5]);
        /* init pid of flux pid */
        motor.flux_pid->kp = *((float *)&pid_buffer[3]);
        motor.flux_pid->ki = *((float *)&pid_buffer[4]);
        motor.flux_pid->kd = *((float *)&pid_buffer[5]);
    }
}

/*!
    \brief      transmit the response data for the received commands using the usart
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void usart_response_data_transmit(void)
{
    uint8_t index = 0, i = 0;
    int8_t sum = 0;

    float *pid_struct_ptr = (float *)(&usart_pid);

    if(dma_flag_get(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF) == SET) {
        /* clear the transmit finish flag */
        dma_flag_clear(USART_DMA, USART_TX_DMA_CH, DMA_FLAG_FTF);
        transmit_dma_busy = 0;
    }
    if(transmit_dma_busy == 0) {
        transmit_dma_busy = 1;
        /*  initialize transmit_data array */
        memset(transmit_data, 0, USART_RESPONSEPID_FARME_LENGTH);
        /* disable DMA channel */
        dma_channel_disable(USART_DMA, USART_TX_DMA_CH);

        /* write the sending array */
        transmit_data[index++] = USART_FRAME_HEAD2;
        transmit_data[index++] = USART_FRAME_HEAD1;
        transmit_data[index++] = usart_receive.function_code;

        if((usart_receive.function_code == USART_READPID_FUNCODE) || (usart_receive.function_code == USART_DEFAULTPID_FUNCODE)) {
            DMA_CHCNT_TX = USART_RESPONSEPID_FARME_LENGTH;
            /* data length write */
            transmit_data[index++] = (USART_RESPONSEPID_FARME_LENGTH - 5);
            /* PID parameters sending */
            /* data type conversion of speed pid */
            pid_buffer[i++] = *((uint32_t *)&motor.speed_pid->kp);
            pid_buffer[i++] = *((uint32_t *)&motor.speed_pid->ki);
            pid_buffer[i++] = *((uint32_t *)&motor.speed_pid->kd);
            /* data type conversion of torque pid */
            pid_buffer[i++] = *((uint32_t *)&motor.torque_pid->kp);
            pid_buffer[i++] = *((uint32_t *)&motor.torque_pid->ki);
            pid_buffer[i++] = *((uint32_t *)&motor.torque_pid->kd);
            /* data type conversion of pid struct */
            for(; i < USART_PID_DATA_NUM; i++) {
                pid_buffer[i] = *((uint32_t *)pid_struct_ptr);
                pid_struct_ptr++;
            }
            /* divid PID data into bytes */
            for(i = 0; i < USART_PID_DATA_NUM; i++) {
                transmit_data[index++] = (uint8_t)(pid_buffer[i] >> 24);
                transmit_data[index++] = (uint8_t)(pid_buffer[i] >> 16);
                transmit_data[index++] = (uint8_t)(pid_buffer[i] >> 8);
                transmit_data[index++] = (uint8_t)pid_buffer[i];
            }
            /* sum check */
            for(i = 0; i < (USART_RESPONSEPID_FARME_LENGTH - 1); i++) {
                sum += transmit_data[i];
            }
            transmit_data[i] = sum;
        } else {
            DMA_CHCNT_TX = USART_RESPONSE_FARME_LENGTH;
            transmit_data[index++] = 1;
            transmit_data[index++] = (uint8_t)(usart_receive.data_sum);
            /* sum check */
            for(i = 0; i < (USART_RESPONSE_FARME_LENGTH - 1); i++) {
                sum += transmit_data[i];
            }
            transmit_data[i] = sum;
        }

        /* enable DMA channel */
        dma_channel_enable(USART_DMA, USART_TX_DMA_CH);
        /* change usart communication state */
        usart_receive.receive_state = STATE_RECEIVE_IDLE;
        usart_transmit.transmit_state = STATE_TRANSMITE_WAVEFORM;
    }
}
