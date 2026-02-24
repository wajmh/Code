/*!
    \file    communication.h
    \brief   the header file of communication module

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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "motor.h"

/* macro definition */
/* usart configuration parameters */
#define USART_BPS                           (460800U)                           /* the baudrate of the usart */
/* DMA transmit data length */
#define USART_RECEIVE_FARME_LENGTH          (128U)                              /* the length of the usart receive data frame and transmit pid data frame */
/* usart waveform data parameters */
#define USART_WAVEFORM_DATA_NUM             (0x05)                              /* the number of the transmit waveform data */

/* enum definition */
/* the state of usart receive communication */
typedef enum {
    STATE_RECEIVE_IDLE = 0,                                                     /* receive idle state */
    STATE_RECEIVE_PARSE                                                         /* received data parsing state */
} comm_usart_receive_state_enum;

/* the state of usart transmit communication */
typedef enum {
    STATE_TRANSMITE_WAVEFORM = 0,                                               /* transmit waveform data state */
    STATE_TRANSMITE_MOTORSTATE,                                                 /* transmit motor state data state */
    STATE_TRANSMIT_RESPONSE                                                     /* transmit response data state */
} comm_usart_transmit_state_enum;

/* srtuct definition */
/* the pid data srtuct received and transmitted by the usart */
typedef struct {
    /* pid variables definition*/
    float pid1_p;                                                               /* the kp value of pid1 */
    float pid1_i;                                                               /* the ki value of pid1 */
    float pid1_d;                                                               /* the kd value of pid1 */
    float pid2_p;                                                               /* the kp value of pid2 */
    float pid2_i;                                                               /* the ki value of pid2 */
    float pid2_d;                                                               /* the kd value of pid2 */
    float pid3_p;                                                               /* the kp value of pid3 */
    float pid3_i;                                                               /* the ki value of pid3 */
    float pid3_d;                                                               /* the kd value of pid3 */
    float pid4_p;                                                               /* the kp value of pid4 */
    float pid4_i;                                                               /* the ki value of pid4 */
    float pid4_d;                                                               /* the kd value of pid4 */
} comm_usart_pid_struct;

/* the usart transmit waveform data srtuct */
typedef struct {
    /* the enum variable definition */
    comm_usart_transmit_state_enum transmit_state;                              /* the state of usart transmit communication */
    /* usart transmit processing variable definition*/
    int16_t data[USART_WAVEFORM_DATA_NUM];                                      /* the waveform data transmitted by usart */
} comm_usart_transmit_struct;

/* the usart receive data srtuct */
typedef struct {
    /* the enum variable definition */
    comm_usart_receive_state_enum receive_state;                                /* the state of usart receive communication */
    /* usart receive processing variable definition*/
    uint8_t function_code;                                                      /* the function code in the usart receive frames */
    uint8_t buffer[USART_RECEIVE_FARME_LENGTH];                                 /* the data array in the usart receive frames */
    uint8_t data_sum;                                                           /* the sum check value in the usart receive frames */
    uint8_t return_check_flag;                                                  /* the flag that the usart received data frame needs return check */
} comm_usart_receive_struct;

/* the usart transmit data srtuct */
extern comm_usart_transmit_struct usart_transmit;

/* function declaration */
/* initialize the usart communication peripherals */
void comm_init(void);
/* transmit the motor state data using the usart */
void comm_usart_motor_state_data_transmit(void);
/* transmit the waveform data using the usart */
void comm_usart_waveform_data_transmit(void);
/* usart receive interrupt handler */
void comm_usart_interrupt_handler(void);
/* parse the received data by the usart */
void comm_usart_receive_data_parse(void);

#endif /* COMMUNICATION_H */
