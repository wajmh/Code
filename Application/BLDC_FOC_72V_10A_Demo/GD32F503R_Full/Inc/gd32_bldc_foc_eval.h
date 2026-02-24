/*!
    \file    gd32_bldc_foc_eval.h
    \brief   definitions for leds, keys, oled and COM ports hardware resources

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

#ifndef GD32_BLDC_FOC_EVAL_H
#define GD32_BLDC_FOC_EVAL_H

#include "basic_head.h"

/* macro definition */
/* the GPIO configuration of LED */
#define LED_PIN                             GPIO_PIN_5                          /* GPIO pin */
#define LED_GPIO_CLK                        RCU_GPIOB                           /* GPIO clock */
#define LED_GPIO_PORT                       GPIOB                               /* GPIO port */

/* the GPIO configuration of key */
#define KEY_START_STOP_PIN                  GPIO_PIN_4                          /* GPIO pin */
#define KEY_START_STOP_GPIO_CLK             RCU_GPIOB                           /* GPIO clock */
#define KEY_START_STOP_GPIO_PORT            GPIOB                               /* GPIO port */

#define KEY_ENTER_PIN                       GPIO_PIN_15                         /* GPIO pin */
#define KEY_ENTER_GPIO_CLK                  RCU_GPIOC                           /* GPIO clock */
#define KEY_ENTER_GPIO_PORT                 GPIOC                               /* GPIO port */

/* the GPIO configuration of ec11 */
#define EC11_A_GPIO_PORT                    GPIOC                               /* GPIO port */
#define EC11_A_GPIO_PIN                     GPIO_PIN_13                         /* GPIO pin */
#define EC11_A_GPIO_CLK                     RCU_GPIOC                           /* GPIO clock */
#define EC11_A_EXTI_LINE                    EXTI_13                             /* EXTI line number */
#define EC11_A_EXTI_LINE_EDGE               EXTI_TRIG_BOTH                      /* EXTI interrupt trigger mode */
#define EC11_A_EXTI_SRC_PORT                GPIO_PORT_SOURCE_GPIOC              /* GPIO output port source */
#define EC11_A_EXTI_SRC_PIN                 GPIO_PIN_SOURCE_13                  /* GPIO output pin source */
#define EC11_B_GPIO_PORT                    GPIOC                               /* GPIO port */
#define EC11_B_GPIO_PIN                     GPIO_PIN_14                         /* GPIO pin */
#define EC11_B_GPIO_CLK                     RCU_GPIOC                           /* GPIO clock */
#define EC11_B_EXTI_LINE                    EXTI_14                             /* EXTI line number */
#define EC11_B_EXTI_LINE_EDGE               EXTI_TRIG_BOTH                      /* EXTI interrupt trigger mode */
#define EC11_B_EXTI_SRC_PORT                GPIO_PORT_SOURCE_GPIOC              /* GPIO output port source */
#define EC11_B_EXTI_SRC_PIN                 GPIO_PIN_SOURCE_14                  /* GPIO output pin source */
#define EC11_EXTI_IRQ                       EXTI10_15_IRQn                      /* EXTI interrupt number */
#define EC11_IRQ_HANDLE                     EXTI10_15_IRQHandler                /* EXTI interrupts handler */

/* the configuration of SH1106 */
#define SH1106_I2C_PERIPHERAL               I2C0                                /* I2C peripheral */
#define SH1106_I2C_GPIO_REMAP               GPIO_I2C0_REMAP                     /* I2C peripheral GPIO repmap*/
#define SH1106_I2C_REMAP_CTRL               ENABLE                              /* I2C peripheral GPIO repmap enable */
#define SH1106_I2C_SDA_CLK                  RCU_GPIOB                           /* GPIO clock */
#define SH1106_I2C_SDA_PORT                 GPIOB                               /* GPIO port */
#define SH1106_I2C_SDA_PIN                  GPIO_PIN_9                          /* GPIO pin */
#define SH1106_I2C_SCL_CLK                  RCU_GPIOB                           /* GPIO clock */
#define SH1106_I2C_SCL_PORT                 GPIOB                               /* GPIO port */
#define SH1106_I2C_SCL_PIN                  GPIO_PIN_8                          /* GPIO pin */
#define SH1106_I2C_CLK                      RCU_I2C0                            /* I2C clock */
#define SH1106_I2C_SPEED                    400000                              /* I2C clock speed */

/* enum definition */
/* the key enumeration */
typedef enum {
    KEY_RUN = 0,                                                                /* button type, for start/stop motor */
    KEY_ENTER,                                                                  /* button type, change the motor running direction */
}key_switch_enum;

/* ec11 signal edge enumeration  */
typedef enum{
    A_FALLING = 0,                                                              /* detect falling edge on port A */
    A_RISING,                                                                   /* detect rising edge on port A */
    B_FALLING,                                                                  /* detect falling edge on port B */
    B_RISING,                                                                   /* detect rising edge on port B */
    AB_NONE
}ec11_dege_enum;

/* ec11 input direction enumeration */
typedef enum{
    DIRECTION_INCREASE = 1,                                                     /* the direction of ec11 input is increase */
    DIRECTION_DECREASE,                                                         /* the direction of ec11 input is decrease */
}ec11_direction_enum;

/* the SH1106 command enumeration */
typedef enum {
    SH1106_CMD = 0,                                                             /* command types sent to SH1106 */
    SH1106_DATA                                                                 /* data types sent to SH1106 */
}sh1106_cmd_enum;

/* the font enumeration */
typedef enum {
    FONT_6_8 = 8,                                                               /* font types: 8*8 */
    FONT_8_16 = 16,                                                             /* font types: 8*16 */
    FONT_12_24 = 24                                                             /* font types: 12*24 */
}oled_font_enum;

/* key index struct */
typedef struct
{
    uint8_t current_index;                                                      /* menu index: current menu */
    uint8_t up_index;                                                           /* menu index: up menu */
    uint8_t down_index;                                                         /* menu index: down menu */
    uint8_t enter_index;                                                        /* menu index: confirm menu */
    void (*fun_current_show)();
}key_index_struct;

typedef enum {
    I2C_START = 0,
    I2C_SEND_ADDRESS,
    I2C_RESTART,
    I2C_TRANSMIT_DATA,
    I2C_RELOAD,
    I2C_STOP,
    I2C_END
} i2c_process_enum;

/* variable definition */
/* ec11 previous edge */
extern ec11_dege_enum pre_edge;
/* ec11 end edge */
extern ec11_dege_enum end_edge;
/* ec11 direction */
extern ec11_direction_enum ec_orientation;

/* function declarations */
/* initialize board */
void eval_board_init(void);
/* delay stabilization */
void ec_delay(uint16_t num);
/* handle at one effective edge */
void ec11_handle(void);
/* key process */
void key_process(void);
/* oled flashing routine */
void oled_display(void);
/* the LED indicates the current state, which is called in the systick interrupt */
void led_sacn_state(void);

#endif /* GD32_BLDC_FOC_EVAL_H */
