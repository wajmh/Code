/*!
    \file    topology_para.h
    \brief   configure topology parameters in board 

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

#ifndef TOPOLOGY_PARA_H
#define TOPOLOGY_PARA_H

/* current sampling parameter */
/* GPIO for sampling A-phase current */
#define CURRENT_A_RCU                       RCU_GPIOA                       /* GPIO clock */
#define CURRENT_A_PIN                       GPIO_PIN_0                      /* GPIO pin */
#define CURRENT_A_PORT                      GPIOA                           /* GPIO port */
#define CURRENT_A_CHANNEL                   ADC_CHANNEL_0                   /* ADC channel corresponding to GPIO pin */
/* GPIO for sampling B-phase current */
#define CURRENT_B_RCU                       RCU_GPIOA                       /* GPIO clock */
#define CURRENT_B_PIN                       GPIO_PIN_1                      /* GPIO pin */
#define CURRENT_B_PORT                      GPIOA                           /* GPIO port */
#define CURRENT_B_CHANNEL                   ADC_CHANNEL_1                   /* ADC channel corresponding to GPIO pin */
/* GPIO for sampling C-phase current */
#define CURRENT_C_RCU                       RCU_GPIOA                       /* GPIO clock */
#define CURRENT_C_PIN                       GPIO_PIN_2                      /* GPIO pin */
#define CURRENT_C_PORT                      GPIOA                           /* GPIO port */
#define CURRENT_C_CHANNEL                   ADC_CHANNEL_2                   /* ADC channel corresponding to GPIO pin */
/* GPIO for sampling I-BUS current */
#define CURRENT_IBUS_RCU                     RCU_GPIOA                      /* GPIO clock */
#define CURRENT_IBUS_PIN                     GPIO_PIN_3                     /* GPIO pin */
#define CURRENT_IBUS_PORT                    GPIOA                          /* GPIO port */
#define CURRENT_IBUS_CHANNEL                 ADC_CHANNEL_3                  /* ADC channel corresponding to GPIO pin */
/* operational amplifier network parameter */
#define CURRENT_AMP_GAIN                    (10.0f)                         /* the gain of amplifier in the current conditioning circuit */
#define CURRENT_AMP_SHIFT                   (1.65f)                         /* the bias voltage in the current sampling circuit */
#define CURRENT_SHUNT_RES                   (0.010f)                        /* the sampling resistance in the current sampling circuit */
#define CURRENT_VALUE_CONVERSION            (3.3f/4096)/(CURRENT_SHUNT_RES*CURRENT_AMP_GAIN)     /* the sampling resistance in the current sampling circuit */
#define FOC_CONTORL_IRQHandler              ADC0_1_IRQHandler               /* FOC contorl handler */

/* PWM wave generation */
/* GPIO for A-phase PWM */
#define PWM_UPPER_A_RCU                     RCU_GPIOA                       /* GPIO clock */
#define PWM_UPPER_A_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_A_PIN                     GPIO_PIN_8                      /* GPIO pin */
#define PWM_DOWN_A_RCU                      RCU_GPIOB                       /* GPIO clock */
#define PWM_DOWN_A_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_A_PIN                      GPIO_PIN_13                     /* GPIO pin */
/* GPIO for B-phase PWM */
#define PWM_UPPER_B_RCU                     RCU_GPIOA                       /* GPIO clock */
#define PWM_UPPER_B_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_B_PIN                     GPIO_PIN_9                      /* GPIO pin */
#define PWM_DOWN_B_RCU                      RCU_GPIOB                       /* GPIO clock */
#define PWM_DOWN_B_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_B_PIN                      GPIO_PIN_14
/* GPIO for C-phase PWM */
#define PWM_UPPER_C_RCU                     RCU_GPIOA                       /* GPIO clock */
#define PWM_UPPER_C_PORT                    GPIOA                           /* GPIO port */
#define PWM_UPPER_C_PIN                     GPIO_PIN_10                     /* GPIO pin */
#define PWM_DOWN_C_RCU                      RCU_GPIOB                       /* GPIO clock */
#define PWM_DOWN_C_PORT                     GPIOB                           /* GPIO port */
#define PWM_DOWN_C_PIN                      GPIO_PIN_15                     /* GPIO pin */

/* break motor  */
/* GPIO for break motor */
#define PWM_BRK_RCU                         RCU_GPIOB                       /* GPIO clock */
#define PWM_BRK_PORT                        GPIOB                           /* GPIO port */
#define PWM_BRK_PIN                         GPIO_PIN_12                     /* GPIO pin */

/* hall signal */
#define HALL_A_GPIO_CLK                     RCU_GPIOA                       /* GPIO clock */
#define HALL_A_PIN                          GPIO_PIN_6                      /* GPIO pin */
#define HALL_A_SHIFT                        6                               /* GPIO pin shift */
#define HALL_A_GPIO_PORT                    GPIOA                           /* GPIO port */

#define HALL_B_GPIO_CLK                     RCU_GPIOA                       /* GPIO clock */
#define HALL_B_PIN                          GPIO_PIN_7                      /* GPIO pin */
#define HALL_B_SHIFT                        7                               /* GPIO pin shift */
#define HALL_B_GPIO_PORT                    GPIOA                           /* GPIO port */

#define HALL_C_GPIO_CLK                     RCU_GPIOB                       /* GPIO clock */
#define HALL_C_PIN                          GPIO_PIN_0                      /* GPIO pin */
#define HALL_C_SHIFT                        0                               /* GPIO pin shift */
#define HALL_C_GPIO_PORT                    GPIOB                           /* GPIO port */

#define HALL_TIMER_CLK                      RCU_TIMER2                      /* TIMER clock */
#define HALL_TIMER                          TIMER2                          /* TIMER */
#define HALL_TIMER_IRQ                      TIMER2_IRQn                     /* TIMER interrupt line */
#define HALL_ENCODER_TIMER_IRQHandler       TIMER2_IRQHandler               /* TIMER interrupt handler */

#define FMC_PAGE_SIZE                      ((uint16_t)0x0800U)              /* flash page size */
#define FMC_WRITE_START_ADDR               ((uint32_t)0x0803E000U)          /* flash write start address */
#define FMC_HALL_SHIFT_ADDR                ((uint32_t)0x0803E000U)          /* flash hall data write start address */

#define FMC_ENCODER_WRITE_START_ADDR       ((uint32_t)0x0803F000U)          /* flash encoder write start address */
#define FMC_ENCODER_SHIFT_ADDR             ((uint32_t)0x0803F000U)          /* flash encoder data write start address */
///
/* topology_para.h: 新增独立页(别和hall/encoder共页) */
#define FMC_MOTOR_ID_PAGE_ADDR   ((uint32_t)0x0803D000U)
#define FMC_MOTOR_ID_ADDR        ((uint32_t)0x0803D000U)
#define FMC_MOTOR_ID_MAGIC_ADDR  ((uint32_t)0x0803D004U)
#define MOTOR_ID_FLASH_MAGIC     ((uint32_t)0x5AA50F0FU)
#define MOTOR_ID_DEFAULT         ((uint8_t)1U)
/* usart communication */
/* usart peripheral */
#define COMM_USART_CLK                      RCU_USART2                      /* USART clock */
#define COMM_USART                          USART2                          /* USART */
#define USART_TX_ADDRESS                    ((uint32_t)(0x40004804U))       /* USART data address */
#define USART_RX_ADDRESS                    ((uint32_t)(0x40004804U))       /* USART data address */
#define USART_DMA_RCU                       RCU_DMA0                        /* DMA clock */
#define USART_DMA                           DMA0                            /* DMA */
#define USART_TX_DMA_CH                     DMA_CH1                         /* USART TX DMA channel */
#define USART_RX_DMA_CH                     DMA_CH2                         /* USART RX DMA channel */
#define DMA_CHCNT_TX                        DMA_CH1CNT(USART_DMA)           /* DMA transmit count */
#define DMA_CHCNT_RX                        DMA_CH2CNT(USART_DMA)           /* DMA transmit count */
/* communication interrupt */
#define COMM_RECEIVE_IRQ                    DMA0_Channel2_IRQn              /* USART RX DMA interrupt line */
#define COMM_RECEIVE_IRQHandler             DMA0_Channel2_IRQHandler        /* USART RX DMA interrupt handler */
/* usart gpio pins */
#define USART_RX_CLK                        RCU_GPIOB                       /* GPIO clock */
#define USART_RX_PIN                        GPIO_PIN_11                     /* GPIO pin */
#define USART_RX_PORT                       GPIOB                           /* GPIO port */
#define USART_TX_CLK                        RCU_GPIOB                       /* GPIO clock */
#define USART_TX_PIN                        GPIO_PIN_10                     /* GPIO pin */
#define USART_TX_PORT                       GPIOB                           /* GPIO port */
/* voltage sampling parameter */
/* GPIO for sampling bus voltage */
#define BUS_VOLTAGE_RCU                     RCU_GPIOA                       /* GPIO clock */
#define BUS_VOLTAGE_PIN                     GPIO_PIN_4                      /* GPIO pin */
#define BUS_VOLTAGE_PORT                    GPIOA                           /* GPIO port */
#define BUS_VOLTAGE_CHANNEL                 ADC_CHANNEL_4                   /* ADC channel corresponding to GPIO pin */
#define BUS_VOLTAGE_AMP_GAIN                (4.7f / 104.7f)                 /* the gain of amplifier in the current conditioning circuit */

#endif /* TOPOLOGY_PARA_H */
