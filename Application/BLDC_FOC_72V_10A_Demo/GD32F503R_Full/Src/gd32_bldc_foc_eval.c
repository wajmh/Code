/*!
    \file    gd32_bldc_foc_eval.c
    \brief   manage leds, keys, oled and COM ports in board

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

#include "gd32_bldc_foc_eval.h"
#include "foc_soft_delay.h"
#include "gd32_bldc_foc_oledfont.h"
#include "gd32_bldc_foc_oledbmp.h"
#include "gdfoc_config.h"
#include "systick.h"

/* SH1106 config */
#define SH1106_COLUMN_LENGTH            (128U)          /* column length */
#define SH1106_ROW_LENGTH               (64U)           /* row length */
#define SH1106_PAGE_LENGTH              (8U)            /* page length */
#define SH1106_I2C_SLAVE_ADDRESS7       (0x78U)         /* i2c slave address */
#define SH1106_BRIGHTNESS               (0xffU)         /* SH1106 brightess */
#define SH1106_I2C_TIMEOUT              (65530U)        /* time out for i2c */

/* multilevel menu length */
#define MENU_LENGTH                     30              /* multilevel menu length */
#define MENU_START_PAGE                 0               /* information for start page*/
#define MENU_STATE_HOME                 1               /* information for state */
#define MENU_SETTING_HOME               2               /* information for setting */
#define MENU_MODE_SELECT                3               /* mode select */
#define MENU_DIRECTION                  4               /* direction config */
#define MENU_CURRENT                    5               /* current sample */
#define MENU_MODULATION                 6               /* modulation set */
#define MENU_OPENLOOP_VF                7               /* mode select: open loop mode ,VF start up */
#define MENU_OPENLOOP_IF                8               /* mode select: open loop mode ,IF start up*/
#define MENU_SMO_VF                     9               /* mode select: sliding mode observer ,VF start up */
#define MENU_SMO_IF                     10              /* mode select: sliding mode observer ,IF start up */
#define MENU_AFLUX_VF                   11              /* mode select: active flux observer ,VF start up */
#define MENU_AFLUX_IF                   12              /* mode select: active flux observer ,IF start up */
#define MENU_HALL_SENSOR_DETECT         13              /* mode select: HALL sensor installation angle indentification */
#define MENU_HALL_SENSOR                14              /* mode select: HALL sensor mode */
#define MENU_ENCODER_DETECT             15              /* mode select: encoder mode installation angle indentification */
#define MENU_ENCODER                    16              /* mode select: encoder mode */
#define MENU_CCW                        17              /* direction select: CCW */
#define MENU_CW                         18              /* direction select: CW */
#define MENU_3_SHUNT                    19              /* current sample: 3_shunt */
#define MENU_2_SHUNT                    20              /* current sample: 2_shunt */
#define MENU_1_SHUNT                    21              /* current sample: 1_shunt */
#define MENU_CBSVPWM                    22              /* pwm mode: CBSVPWM */
#define MENU_DSVPWM                     23              /* pwm mode: DSVPWM */
#define MENU_CSVPWM                     24              /* pwm mode: CSVPWM */
#define MENU_HINT                       25              /* hint page */

/* variable definition */
/* ec11 previous edge */
ec11_dege_enum pre_edge = AB_NONE;
/* ec11 end edge */
ec11_dege_enum end_edge = AB_NONE;
/* ec11 direction */
ec11_direction_enum ec_orientation = DIRECTION_INCREASE;

/* local variable definition */
static uint16_t delay_ec = 0;

/* static function */
/* initialize key */
static void key_init(void);
/* key read */
static FlagStatus key_read(key_switch_enum key_sw);
/* key read debouncing */
static FlagStatus key_read_debouncing(key_switch_enum key);
/* initialize the EC11 port */
static void ec_init(void);
/* initialize the led */
static void led_init(void);
/* turn on the selection led */
static void led_on(void);
/* turn off the selection led */
static void led_off(void);
/* initialize the SH1106 */
static void oled_init(void);
/* initialize the oled parameter */
static void oled_parameter_init(void);
/* I2C configuration for SH1106 */
static void i2c_config(void);
/* write data or command to SH1106 module */
static void oled_write_byte(uint8_t data, sh1106_cmd_enum type);
/* power calculation */
static uint32_t oled_pow(uint8_t m,uint8_t n);
/* clear all oled */
static void oled_clear_all(void);
/* clear page oled */
static void oled_clear(uint8_t start_page);
/* set oled display reverse */
static void oled_display_reverse(ControlStatus reverse);
/* oled display flip horizontally */
static void oled_flip_horizontal(ControlStatus flip);
/* refresh oled screen */
static void oled_page_refresh(void);
/* display a point */
static void oled_draw_point(uint8_t x, uint8_t y, uint8_t value);
/* display a picture */
static void oled_show_picture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t picture[]);
/* display a characters */
static void oled_show_char(uint8_t x,uint8_t y,uint8_t chr,oled_font_enum size);
/* display a string */
static void oled_show_string(uint8_t x,uint8_t y,uint8_t *chr,oled_font_enum size);
/* display a number at the specified location */
static void oled_show_num(uint8_t x, uint8_t y, uint16_t num, uint8_t len, oled_font_enum size);
/* display a line at the specified location */
static void oled_show_line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
/* show index start page */
static void fun_index_start(void);
/* show index 0 page */
static void fun_index1_0(void);
/* show index 1 page */
static void fun_index1_1(void);
/* show index 2 page */
static void fun_index2_2(void);
/* show index 3 page */
static void fun_index2_3(void);
/* show index 4 page */
static void fun_index2_4(void);
/* show index 5 page */
static void fun_index2_5(void);
/* show index 7 page */
static void fun_index3_7(void);
/* show index 8 page */
static void fun_index3_8(void);
/* show index 9 page */
static void fun_index3_9(void);
/* show index 10 page */
static void fun_index3_10(void);
/* show index 11 page */
static void fun_index3_11(void);
/* show index 12 page */
static void fun_index3_12(void);
/* show index 13 page */
static void fun_index3_13(void);
/* show index 14 page */
static void fun_index3_14(void);
/* show index 15 page */
static void fun_index3_15(void);
/* show index 16 page */
static void fun_index3_16(void);
/* show index 17 page */
static void fun_index3_17(void);
/* show index 18 page */
static void fun_index3_18(void);
/* show index 19 page */
static void fun_index3_19(void);
/* show index 20 page */
static void fun_index3_20(void);
/* show index 21 page */
static void fun_index3_21(void);
/* show index 22 page */
static void fun_index3_22(void);
/* show index 23 page */
static void fun_index3_23(void);
/* show index 24 page */
static void fun_index3_24(void);
/* show tip  page */
static void fun_index_hint(void);
/* show QR code */
static void oled_show_qr_code(uint8_t x,uint8_t y);

/* multilevel menu */
static key_index_struct menu_table[MENU_LENGTH]=
{
    /* level 0  */
    {MENU_START_PAGE,MENU_START_PAGE,MENU_START_PAGE,MENU_START_PAGE,fun_index_start},                      /* information for start page */

    /* level 1  */
    {MENU_STATE_HOME,MENU_STATE_HOME,MENU_STATE_HOME,MENU_SETTING_HOME,fun_index1_0},                       /* information for state */
    {MENU_SETTING_HOME,MENU_MODE_SELECT,MENU_SETTING_HOME,MENU_STATE_HOME,fun_index1_1},                    /* information for setting */
    
    /* level 2 */
    {MENU_MODE_SELECT,MENU_DIRECTION,MENU_SETTING_HOME,MENU_OPENLOOP_VF,fun_index2_2},                      /* mode select */
    {MENU_DIRECTION,MENU_CURRENT,MENU_MODE_SELECT,MENU_CCW,fun_index2_3},                                   /* direction config */
    {MENU_CURRENT,MENU_MODULATION,MENU_DIRECTION,MENU_3_SHUNT,fun_index2_4},                                /* current sample */
    {MENU_MODULATION,MENU_MODULATION ,MENU_CURRENT,MENU_DSVPWM,fun_index2_5},                               /* modulation set */
    
    /* level 3: mode select */
    {MENU_OPENLOOP_VF,MENU_OPENLOOP_VF,MENU_OPENLOOP_IF,MENU_MODE_SELECT,fun_index3_7},                     /* mode select: open loop mode ,VF start up */
    {MENU_OPENLOOP_IF,MENU_OPENLOOP_VF,MENU_SMO_VF,MENU_MODE_SELECT,fun_index3_8},                          /* mode select: open loop mode ,IF start up*/
    {MENU_SMO_VF,MENU_OPENLOOP_IF,MENU_SMO_IF,MENU_MODE_SELECT,fun_index3_9},                               /* mode select: sliding mode observer ,VF start up */
    {MENU_SMO_IF,MENU_SMO_VF,MENU_AFLUX_VF,MENU_MODE_SELECT,fun_index3_10},                                 /* mode select: sliding mode observer ,IF start up */
    {MENU_AFLUX_VF,MENU_SMO_IF,MENU_AFLUX_IF,MENU_MODE_SELECT,fun_index3_11},                               /* mode select: active flux observer ,VF start up */
    {MENU_AFLUX_IF,MENU_AFLUX_VF,MENU_HALL_SENSOR_DETECT,MENU_MODE_SELECT,fun_index3_12},                   /* mode select: active flux observer ,IF start up */
    {MENU_HALL_SENSOR_DETECT,MENU_AFLUX_IF,MENU_HALL_SENSOR,MENU_MODE_SELECT,fun_index3_13},                /* mode select: HALL sensor installation angle indentification */
    {MENU_HALL_SENSOR,MENU_HALL_SENSOR_DETECT,MENU_ENCODER_DETECT,MENU_MODE_SELECT,fun_index3_14},          /* mode select: HALL sensor mode */    
    {MENU_ENCODER_DETECT,MENU_HALL_SENSOR,MENU_ENCODER,MENU_MODE_SELECT,fun_index3_15},                     /* mode select: encoder mode installation angle indentification */
    {MENU_ENCODER,MENU_ENCODER_DETECT,MENU_ENCODER,MENU_MODE_SELECT,fun_index3_16},                         /* mode select: encoder mode */

    /* level 3: direction config */
    {MENU_CCW,MENU_CCW,MENU_CW,MENU_DIRECTION,fun_index3_17},                                               /* direction select: CCW */
    {MENU_CW,MENU_CCW,MENU_CW,MENU_DIRECTION,fun_index3_18},                                                /* direction select: CW */
    
    /* level 3: current sample */
    {MENU_3_SHUNT,MENU_3_SHUNT,MENU_2_SHUNT,MENU_CURRENT,fun_index3_19},                                    /* current sample: 1_shunt */
    {MENU_2_SHUNT,MENU_3_SHUNT,MENU_1_SHUNT,MENU_CURRENT,fun_index3_20},                                    /* current sample: 2_shunt */
    {MENU_1_SHUNT,MENU_2_SHUNT,MENU_1_SHUNT,MENU_CURRENT,fun_index3_21},                                    /* current sample: 3_shunt */
    
    /* level 3: pwm mode */
    {MENU_CBSVPWM,MENU_CBSVPWM,MENU_DSVPWM,MENU_MODULATION,fun_index3_22},                                  /* pwm mode: CBSVPWM */
    {MENU_DSVPWM,MENU_DSVPWM,MENU_CSVPWM,MENU_MODULATION,fun_index3_23},                                    /* pwm mode: DSVPWM */
    {MENU_CSVPWM,MENU_DSVPWM,MENU_CSVPWM,MENU_MODULATION,fun_index3_24},                                    /* pwm mode: CSVPWM */
    
    /* tip page  */
    {MENU_HINT,MENU_HINT,MENU_HINT,MENU_MODE_SELECT,fun_index_hint},                                        /* hint page */
};
/* current index */
static uint8_t func_index = MENU_START_PAGE;
/* previous index */
static uint8_t func_index_pre = MENU_START_PAGE;
/* SH1106 RAM data */
static uint8_t oled_buffer[SH1106_COLUMN_LENGTH][SH1106_PAGE_LENGTH] = {0xff};
/* current display interface */
void (*display_menu)(void);

/*!
    \brief      initialize board
    \param[in]  none
    \param[out] none
    \retval     none
*/
void eval_board_init(void)
{
    /* initialize keys */
    key_init();
    /* initialize the EC11 port */
    ec_init();
    /* initialize the GPIO port of LED */
//    led_init();
    /* initialize the SH1106 */
    oled_init();
}

/*!
    \brief      delay stabilization
    \param[in]  num: number of software delays
    \param[out] none
    \retval     none
*/
void ec_delay(uint16_t num)
{
    for(delay_ec=0; delay_ec<num; delay_ec++);
}

/*!
    \brief      handle at one effective edge
    \param[in]  num
    \param[out] none
    \retval     none
*/
void ec11_handle(void)
{
    switch(motor.state){
        case MC_STATE_INIT:
            break;
        
        case MC_STATE_IDLE:
            if(DIRECTION_INCREASE == ec_orientation){
                func_index = menu_table[func_index].up_index;
                if(MENU_STATE_HOME == func_index){
                    /* adjust reference speed */
                    motor.speed_ref += SPEED_REFERENCE_STEPPING;
                    if(motor.speed_ref > SPEED_REFERENCE_MAX){
                        motor.speed_ref = SPEED_REFERENCE_MAX;
                    }
                }
            }else if(DIRECTION_DECREASE == ec_orientation){
                func_index = menu_table[func_index].down_index;
                if(MENU_STATE_HOME == func_index){
                    /* adjust reference speed */
                    if(motor.speed_ref <= SPEED_REFERENCE_MIN){
                        motor.speed_ref = SPEED_REFERENCE_MIN;
                    }
                    else{
                        motor.speed_ref -= SPEED_REFERENCE_STEPPING;
                    }
                }
            }
            break;
        
        case MC_STATE_RUNNING:
            if(DIRECTION_INCREASE == ec_orientation){
                /* adjust reference speed */
                motor.speed_ref += SPEED_REFERENCE_STEPPING;
                if(motor.speed_ref > SPEED_REFERENCE_MAX){
                    motor.speed_ref = SPEED_REFERENCE_MAX;
                }
            }else if(DIRECTION_DECREASE == ec_orientation){
                /* adjust reference speed */
                if(motor.speed_ref <= SPEED_REFERENCE_MIN){
                    motor.speed_ref = SPEED_REFERENCE_MIN;
                }
                else{
                    motor.speed_ref -= SPEED_REFERENCE_STEPPING;
                }
            }
            break;

        case MC_STATE_BRAKE: {
            break;
        }

        case MC_STATE_FAULT: {
            break;
        }
        default:
            break;
    }
    
}

/*!
    \brief      key process
    \param[in]  none
    \param[out] none
    \retval     none
*/
void key_process(void)
{
    switch(motor.state){
        case MC_STATE_INIT:
            /* start page */
            func_index_pre = func_index;
            func_index = menu_table[MENU_STATE_HOME].current_index;
            break;
        
        case MC_STATE_IDLE:
            if(key_read_debouncing(KEY_RUN)){
                /* start motor */
                motor.command = MC_START;
                /* run homepage */
                func_index_pre = func_index;
                func_index = menu_table[MENU_STATE_HOME].current_index;
            }
            
            /* confirm key */
            if(key_read_debouncing(KEY_ENTER)){

                if(MENU_OPENLOOP_VF == menu_table[func_index].current_index ){
                    /* mode select: open loop mode ,VF start up */
                    motor.running_mode = OPENLOOP_VF;
                }else if(MENU_OPENLOOP_IF == menu_table[func_index].current_index){
                    /* mode select: open loop mode ,IF start up */
                    motor.running_mode = OPENLOOP_IF;
                }else if(MENU_SMO_VF == menu_table[func_index].current_index ){
                     /* mode select: sliding mode observer ,VF start up */
                    motor.running_mode = SMO_VF;
                }else if(MENU_SMO_IF == menu_table[func_index].current_index ){
                    /* mode select: sliding mode observer ,IF start up */
                    motor.running_mode = SMO_IF;
                }else if(MENU_AFLUX_VF == menu_table[func_index].current_index ){
                     /* mode select: active flux observer ,VF start up */
                    motor.running_mode = AFLUX_VF;
                }else if(MENU_AFLUX_IF == menu_table[func_index].current_index ){
                    /* mode select: active flux observer ,IF start up */
                    motor.running_mode = AFLUX_IF;
                }else if(MENU_HALL_SENSOR_DETECT == menu_table[func_index].current_index){
                    /* mode select: HALL sensor installation angle indentification */
                    motor.running_mode = HALL_SENSOR_DETECT;
                }else if(MENU_HALL_SENSOR == menu_table[func_index].current_index){
                    /* mode select: HALL sensor mode */
                    motor.running_mode = HALL_SENSOR;
                }else if(MENU_ENCODER_DETECT == menu_table[func_index].current_index){
                    /* mode select: encoder detect mode */
                    motor.running_mode = ENCODER_DETECT;
                }else if(MENU_ENCODER == menu_table[func_index].current_index){
                    /* mode select: encoder detect mode */
                    motor.running_mode = ENCODER;
                }else if(MENU_CCW == menu_table[func_index].current_index){
                    /* direction select: CCW */
                    motor.direction = DIRECTION_CCW;
                }else if(MENU_CW == menu_table[func_index].current_index){
                     /* direction select: CW */
                    motor.direction = DIRECTION_CW;
                }else if(MENU_1_SHUNT == menu_table[func_index].current_index){
                    /* current sample: 1_shunt */
                    pwm_current.samp_mode = CURRENT_1SHUNT;
                }else if(MENU_2_SHUNT == menu_table[func_index].current_index){
                    /* current sample: 2_shunt */
                    pwm_current.samp_mode = CURRENT_2SHUNT;
                }else if(MENU_3_SHUNT == menu_table[func_index].current_index){
                    /* current sample: 3_shunt */
                    pwm_current.samp_mode = CURRENT_3SHUNT;
                }else if(MENU_DSVPWM == menu_table[func_index].current_index){
                    /* pwm mode: DSVPWM */
                    pwm_current.pwm_mode = DSVPWMMIN;
                }else if(MENU_CSVPWM == menu_table[func_index].current_index){
                    /* pwm mode: CSVPWM */
                    pwm_current.pwm_mode = CSVPWM;
                }

                /* change max modulation degree */
                /* current sample: 1_shunt */
                if(pwm_current.samp_mode == CURRENT_1SHUNT){
                    motor.us_out_max = US_BASE * (MIN(MI_SAMPLE_MAX_1SHUNT_CSVPWM, MI_CHARGE_MAX_CSVPWM));
                /* current sample: 2_shunt */
                }else if(pwm_current.samp_mode == CURRENT_2SHUNT){
                    if(pwm_current.pwm_mode == CSVPWM){
                        motor.us_out_max = US_BASE * (MIN(MI_SAMPLE_MAX_2SHUNT_CSVPWM, MI_CHARGE_MAX_CSVPWM));
                    }else{
                        motor.us_out_max = US_BASE * (MIN(MI_SAMPLE_MAX_2SHUNT_DSVPWMMIN, MI_CHARGE_MAX_DSVPWMMIN));
                   }
                /* current sample: 3_shunt */
                }else{
                    if(pwm_current.pwm_mode == CSVPWM){
                        motor.us_out_max = US_BASE * (MIN(MI_SAMPLE_MAX_3SHUNT_CSVPWM, MI_CHARGE_MAX_CSVPWM));
                    }else{
                        motor.us_out_max = US_BASE * (MIN(MI_SAMPLE_MAX_3SHUNT_DSVPWMMIN, MI_CHARGE_MAX_DSVPWMMIN));
                   }
                }

                /* change menu and state */
                motor.state = MC_STATE_INIT;
                func_index = menu_table[func_index].enter_index;
                
                if((pwm_current.samp_mode == CURRENT_1SHUNT)&&(pwm_current.pwm_mode == DSVPWMMIN)){
                    func_index = menu_table[MENU_HINT].current_index;
                }

            }
            break;
        
        case MC_STATE_RUNNING:
            if(key_read_debouncing(KEY_RUN)){
                motor.command = MC_STOP;
                func_index = menu_table[MENU_STATE_HOME].current_index;
                func_index_pre = menu_table[MENU_STATE_HOME].current_index;
                display_menu = menu_table[MENU_STATE_HOME].fun_current_show;
            }
            if(key_read_debouncing(KEY_ENTER)){
                motor.command = MC_BREAK;
                func_index = menu_table[MENU_STATE_HOME].current_index;
                func_index_pre = menu_table[MENU_STATE_HOME].current_index;
                display_menu = menu_table[MENU_STATE_HOME].fun_current_show;
            }
            break;

        case MC_STATE_BRAKE:
            if(key_read_debouncing(KEY_ENTER)){
                motor.command = MC_BREAK;
            }
            break;

        case MC_STATE_FAULT:
            break;
        
        default:
            break;
    }
}

/*!
    \brief      oled flashing routine
    \param[in]  none
    \param[out] none
    \retval     none
*/
void oled_display(void)
{
    if (func_index != func_index_pre)
    {
        display_menu = menu_table[func_index].fun_current_show;
        func_index_pre = func_index;
    }
    oled_clear_all();
    (*display_menu)();
    oled_page_refresh();
}

/*!
    \brief      the LED indicates the current state, which is called in the systick interrupt
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_sacn_state(void)
{
    static uint16_t k = 0U;
    static uint16_t i = 0,dir=0;
    static  int j=1;
    uint8_t count;

    switch(motor.state){
        case MC_STATE_INIT:
            if(++i == 15U){
                i = 0U;
                if(dir){
                    j++;
                }else{
                    j--;
                }
            }
            if(j == 40U){
                dir = 0U;
            }else if(j == -20){
                dir = 1U;
            }

            if(i < j){
                led_on();
            }else{
                led_off();
            }
        break;

        case MC_STATE_IDLE:
            if(2000U <= ++k){
                k = 0U;
            }

            if(k < 1000U){
                led_on();
            }else{
                led_off();
            }
        break;

        case MC_STATE_RUNNING:
            led_on();
        break;

        case MC_STATE_BRAKE:
            led_on();
        break;

        case MC_STATE_FAULT:
            for(count=0;count<(uint8_t)motor.fault;count++){
                led_on();
                delay_1ms(100);
                led_off();
                delay_1ms(100);
            }
        break;

        default:
        break;
    }
}

/*!
    \brief      initialize keys
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void key_init(void)
{
    /* enable the clock of GPIO and alternate function */
    rcu_periph_clock_enable(KEY_START_STOP_GPIO_CLK);
    rcu_periph_clock_enable(KEY_ENTER_GPIO_CLK);
    rcu_periph_clock_enable(RCU_AF);

    /* RUN, BRAKE, FUNCTION key initialize */
    gpio_mode_set(KEY_START_STOP_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, KEY_START_STOP_PIN);
    gpio_mode_set(KEY_ENTER_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, KEY_ENTER_PIN);
}

/*!
    \brief      read the state of the key
    \param[in]  none
    \param[out] none
    \retval     the state of the key
*/
static FlagStatus key_read(key_switch_enum key)
{
    uint8_t data=0;

    switch(key){
        case KEY_RUN:
            data = gpio_input_bit_get(KEY_START_STOP_GPIO_PORT, KEY_START_STOP_PIN);
        break;
        case KEY_ENTER:
            data = gpio_input_bit_get(KEY_ENTER_GPIO_PORT, KEY_ENTER_PIN);
        break;
        default: data = 0;
    }

    if(data){
        return RESET;
    }

    return SET;
}

/*!
    \brief      key read debouncing
    \param[in]  none
    \param[out] none
    \retval     none
*/
static FlagStatus key_read_debouncing(key_switch_enum key)
{
    if(key_read(key)){
        soft_delay_ms(20);
        if(key_read(key)){
            /* intermittent examination, press release to start */
            while(SET == key_read(key));
            return SET;
        }
    }
    return RESET;
}

/*!
    \brief      initialize the EC11 port
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void ec_init(void)
{
    /* enable the clock of GPIO */
    rcu_periph_clock_enable(EC11_A_GPIO_CLK);
    rcu_periph_clock_enable(EC11_B_GPIO_CLK);
    rcu_periph_clock_enable(RCU_AF);
    
    /* port A config */
    /* configure button pin A as input */
    /* configure button pin as input */
    gpio_mode_set(EC11_A_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, EC11_A_GPIO_PIN);
    gpio_output_options_set(EC11_A_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, EC11_A_GPIO_PIN);
    /* connect key EXTI line to key GPIO pin */
    gpio_exti_source_select(EC11_A_EXTI_SRC_PORT, EC11_A_EXTI_SRC_PIN);
    /* configure key EXTI line */
    exti_init(EC11_A_EXTI_LINE, EXTI_INTERRUPT, EC11_A_EXTI_LINE_EDGE);
    exti_interrupt_flag_clear(EC11_A_EXTI_LINE);

    /* port B config */
    /* configure button pin B as input */
    gpio_mode_set(EC11_B_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, EC11_B_GPIO_PIN);
    gpio_output_options_set(EC11_B_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, EC11_B_GPIO_PIN);
    /* connect key EXTI line to key GPIO pin */
    gpio_exti_source_select(EC11_B_EXTI_SRC_PORT, EC11_B_EXTI_SRC_PIN);
    /* configure key EXTI line */
    exti_init(EC11_B_EXTI_LINE, EXTI_INTERRUPT, EC11_B_EXTI_LINE_EDGE);
    exti_interrupt_flag_clear(EC11_B_EXTI_LINE);

    /* enable and set key EXTI interrupt to the lowest priority */
    nvic_irq_enable(EC11_EXTI_IRQ, 3U, 0U);
}

/*!
    \brief      initialize the GPIO port of LED
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void led_init(void)
{
    /* enable the clock of GPIO */
    rcu_periph_clock_enable(LED_GPIO_CLK);
    
    /* led initialize */
    gpio_mode_set(LED_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_output_options_set(LED_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, LED_PIN);
    
    /* turn off all the led */
    led_off();
}

/*!
    \brief      turn on the selection LED 
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void led_on(void)
{
    gpio_bit_set(LED_GPIO_PORT, LED_PIN);
}

/*!
    \brief      turn off the selection LED 
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void led_off(void)
{
    gpio_bit_reset(LED_GPIO_PORT, LED_PIN);
}

/*!
    \brief      reset I2C gpio configure
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_gpio_reset(void)
{
    /* reset I2C_SCL_PIN and I2C_SDA_PIN */
    gpio_mode_set(SH1106_I2C_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SH1106_I2C_SCL_PIN);
    gpio_output_options_set(SH1106_I2C_SCL_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, SH1106_I2C_SCL_PIN);
    gpio_mode_set(SH1106_I2C_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SH1106_I2C_SDA_PIN);
    gpio_output_options_set(SH1106_I2C_SDA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, SH1106_I2C_SDA_PIN);
}

/*!
    \brief      reset i2c bus
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_bus_reset()
{
    /* configure SDA/SCL for GPIO */
    GPIO_BC(SH1106_I2C_SCL_PORT) |= SH1106_I2C_SCL_PIN;
    GPIO_BC(SH1106_I2C_SDA_PORT) |= SH1106_I2C_SDA_PIN;
    /* reset I2C_SCL_PIN and I2C_SDA_PIN */
    i2c_gpio_reset();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    /* stop signal */
    GPIO_BOP(SH1106_I2C_SCL_PORT) |= SH1106_I2C_SCL_PIN;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    GPIO_BOP(SH1106_I2C_SDA_PORT) |= SH1106_I2C_SDA_PIN;
    /* connect I2C_SCL_PIN to I2C_SCL */
    /* connect I2C_SDA_PIN to I2C_SDA */
    gpio_mode_set(SH1106_I2C_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SH1106_I2C_SCL_PIN);
    gpio_output_options_set(SH1106_I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL2, SH1106_I2C_SCL_PIN);
    gpio_mode_set(SH1106_I2C_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SH1106_I2C_SDA_PIN);
    gpio_output_options_set(SH1106_I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL2, SH1106_I2C_SDA_PIN);
}

/*!
    \brief      initialize the SH1106
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void oled_init(void)
{
    i2c_config();                                       /* I2C interface initialization */

    oled_write_byte(0xAE,SH1106_CMD);                   /* sleep mode, turn off oled panel */
    
    oled_write_byte(0x02,SH1106_CMD);                   /* set lower column address */
    
    oled_write_byte(0x10,SH1106_CMD);                   /* set higher column address */
    
    oled_write_byte(0x40,SH1106_CMD);                   /* set display start line */
    
    oled_write_byte(0xB0,SH1106_CMD);                   /* set display start line */

    oled_write_byte(0x81,SH1106_CMD);                   /* double-byte command£ºset contrast control register */
    oled_write_byte(SH1106_BRIGHTNESS,SH1106_CMD);      /* double-byte command£ºset seg output brightness, 0x00-0xff */

    oled_write_byte(0xA0,SH1106_CMD);                   /* set seg/column mapping, column address 0 is mapped to seg0 */
    
    oled_write_byte(0xA6,SH1106_CMD);                   /* set normal display, 0 is OFF display and 1 is ON display */
    
    oled_write_byte(0xA8,SH1106_CMD);                   /* double-byte command£ºset multiplex ratio, 0x0f-0x3f */
    oled_write_byte(0x3F,SH1106_CMD);                   /* double-byte command£º1/64 duty */

    oled_write_byte(0xAD,SH1106_CMD);                   /* set charge pump enable */
    oled_write_byte(0x8B,SH1106_CMD);                   /* 0x8B ÄÚ¹© VCC */
    
    oled_write_byte(0x33,SH1106_CMD);                   /* 0x33 set VPP 9V */ 
    
    oled_write_byte(0xC0,SH1106_CMD);                   /* set com/row scan direction, scan from com[0] to com[n-1] */
    
    oled_write_byte(0xD3,SH1106_CMD);                   /* double-byte command£ºset vertical shift by com from 0d~63d */
    oled_write_byte(0x00,SH1106_CMD);                   /* double-byte command£ºnot offset */

    oled_write_byte(0xD5,SH1106_CMD);                   /* double-byte command£ºset display clock divide ratio/oscillator frequency */
    oled_write_byte(0x80,SH1106_CMD);                   /* double-byte command£ºdivide ratio is 1, use the default internal clock */
    
    oled_write_byte(0xD9,SH1106_CMD);                   /* double-byte command£ºset pre-charge period */
    oled_write_byte(0xF1,SH1106_CMD);                   /* double-byte command£ºset pre-charge as 15 clocks & discharge as 1 clock */

    oled_write_byte(0xDA,SH1106_CMD);                   /* double-byte command£ºset com pins hardware configuration */
    oled_write_byte(0x12,SH1106_CMD);                   /* double-byte command£ºsequential com pin configuration and disable com left/right remap */

    oled_write_byte(0xDB,SH1106_CMD);                   /* double-byte command£ºadjusts the Vcomh regulator output */
    oled_write_byte(0x30,SH1106_CMD);                   /* double-byte command£ºSet Vcom to 0.83Vcc */

    oled_clear_all();
    oled_page_refresh();
    oled_write_byte(0xAF,SH1106_CMD);                   /* normal mode, turn on oled panel */
    
    oled_display_reverse(DISABLE);                      /* disable oled display reverse */
    oled_flip_horizontal(DISABLE);                      /* disable oled display flip horizontally */
    oled_clear(0);
    
    oled_parameter_init();
}

/*!
    \brief      initialize the oled parameter
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void oled_parameter_init(void)
{
    func_index = menu_table[MENU_START_PAGE].current_index;
    func_index_pre = menu_table[MENU_START_PAGE].current_index;
    display_menu = menu_table[MENU_START_PAGE].fun_current_show;
}

/*!
    \brief      I2C configuration for SH1106
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void i2c_config(void)
{
    /* configure GPIO clock */
    rcu_periph_clock_enable(SH1106_I2C_SDA_CLK);
    rcu_periph_clock_enable(SH1106_I2C_SCL_CLK);
    /* configure GPIO remap */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(SH1106_I2C_CLK);
    /* configure GPIO */
    gpio_mode_set(SH1106_I2C_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SH1106_I2C_SDA_PIN);
    gpio_output_options_set(SH1106_I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL2, SH1106_I2C_SDA_PIN);
    gpio_af_set(SH1106_I2C_SDA_PORT, GPIO_AF_0, SH1106_I2C_SDA_PIN);
    
    gpio_mode_set(SH1106_I2C_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, SH1106_I2C_SCL_PIN);
    gpio_output_options_set(SH1106_I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_LEVEL2, SH1106_I2C_SCL_PIN);
    gpio_af_set(SH1106_I2C_SCL_PORT, GPIO_AF_0, SH1106_I2C_SCL_PIN);
    
    
    /* I2C clock configure */
    i2c_timing_config(SH1106_I2C_PERIPHERAL, 0x0, 0xC, 0);
    i2c_master_clock_config(SH1106_I2C_PERIPHERAL, 0x4B, 0xE9);

    /* configure I2C address */
    i2c_address_config(SH1106_I2C_PERIPHERAL, SH1106_I2C_SLAVE_ADDRESS7, I2C_ADDFORMAT_7BITS);
    /* send slave address to I2C bus */
    i2c_master_addressing(SH1106_I2C_PERIPHERAL, SH1106_I2C_SLAVE_ADDRESS7, I2C_MASTER_TRANSMIT);
    i2c_transfer_byte_number_config(SH1106_I2C_PERIPHERAL, 2);
    /* enable I2C0 */
    i2c_enable(SH1106_I2C_PERIPHERAL);
}

/*!
    \brief      write data or command to oled module
    \param[in]  data: transport data 
    \param[in]  type: transport type
    \param[out] none
    \retval     none
*/
static void oled_write_byte(uint8_t data, sh1106_cmd_enum type)
{
    uint8_t first_byte;
    i2c_process_enum state = I2C_START;
    uint16_t timeout = 0;
    uint8_t end_flag = 0;
    uint16_t number_of_byte = 1;
    
    if(type){
        first_byte = 0x40;
    }else{
        first_byte = 0x00;
    }
    while(!end_flag) {
        switch(state) {
            case I2C_START:
                /* configure slave address */
                i2c_master_addressing(I2C0, SH1106_I2C_SLAVE_ADDRESS7, I2C_MASTER_TRANSMIT);
                /* configure number of bytes to be transferred */
                i2c_transfer_byte_number_config(I2C0, number_of_byte + 1);
                /* enable I2C automatic end mode in master mode */
                i2c_automatic_end_enable(I2C0);
                /* i2c master sends start signal only when the bus is idle */
                while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY) && (timeout < SH1106_I2C_TIMEOUT)) {
                    timeout++;
                }
                if(timeout < SH1106_I2C_TIMEOUT) {
                    i2c_start_on_bus(I2C0);
                    timeout = 0;
                    state = I2C_SEND_ADDRESS;
                    /* clear I2C_TDATA register */
                    I2C_STAT(I2C0) |= I2C_STAT_TBE;
                } else {
                    /* timeout, bus reset */
                    i2c_bus_reset();
                    timeout = 0;
                    state = I2C_START;
                }
                break;
            case I2C_SEND_ADDRESS:
                /* wait until the transmit data buffer is empty */
                while((!i2c_flag_get(I2C0, I2C_FLAG_TBE)) && (timeout < SH1106_I2C_TIMEOUT)) {
                    timeout++;
                }
                if(timeout < SH1106_I2C_TIMEOUT) {
                    /* send the EEPROM's internal address to write to : only one byte address */
                    i2c_data_transmit(I2C0, first_byte);
                    timeout = 0;
                    state = I2C_TRANSMIT_DATA;
                } else {
                    timeout = 0;
                    state = I2C_START;
                }
                break;
            case I2C_TRANSMIT_DATA:
                while(number_of_byte--) {
                    /* wait until TI bit is set */
                    while((!i2c_flag_get(I2C0, I2C_FLAG_TBE)) && (timeout < SH1106_I2C_TIMEOUT)) {
                        timeout++;
                    }
                    if(timeout < SH1106_I2C_TIMEOUT) {
                        /* while there is data to be written */
                        i2c_data_transmit(I2C0, data);
                        /* point to the next byte to be written */
                        timeout = 0;
                        state = I2C_STOP;
                    } else {
                        /* wait TI timeout */
                        timeout = 0;
                        state = I2C_START;
                        return ;
                    }
                }
                break;
            case I2C_STOP:
                /* wait until the stop condition is finished */
                while((!i2c_flag_get(I2C0, I2C_FLAG_STPDET)) && (timeout < SH1106_I2C_TIMEOUT)) {
                    timeout++;
                }
                if(timeout < SH1106_I2C_TIMEOUT) {
                    /* clear STPDET flag */
                    i2c_flag_clear(I2C0, I2C_FLAG_STPDET);
                    timeout = 0;
                    state = I2C_END;
                    end_flag = 1;
                } else {
                    /* stop detect timeout */
                    timeout = 0;
                    state = I2C_START;
                }
                break;
            default:
                /* default status */
                state = I2C_START;
                end_flag = 1;
                timeout = 0;
                break;
        }
    }
}

/*!
    \brief      power calculation
    \param[in]  m: the base number
    \param[in]  n: the index number
    \param[out] none
    \retval     none
*/
static uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result = 1;

    while(n--){
        result*=m;
    }

    return result;
}

/*!
    \brief      clear all oled display
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void oled_clear_all(void)
{
    uint8_t i,n;

    for(i=0;i<SH1106_PAGE_LENGTH;i++){
        for(n=0;n<SH1106_COLUMN_LENGTH;n++){
            oled_buffer[n][i] = 0;
        }
    }
}

/*!
    \brief      clear page oled
    \param[in]  page:0~(SH1106_PAGE_LENGTH -1)
    \param[out] none
    \retval     none
*/
static void oled_clear(uint8_t page)
{
    uint8_t n;
    /* set page start address for page addressing mode(B0h~B7h) */
    oled_write_byte (0xB0+page,SH1106_CMD);
    /* set lower column start address for page addressing mode(00h~0Fh) */
    oled_write_byte (0x02,SH1106_CMD);
    /* set higher column start address for page addressing mode(10h~1Fh) */
    oled_write_byte (0x10,SH1106_CMD);

    for(n=0;n<SH1106_COLUMN_LENGTH;n++){
        oled_write_byte(0,SH1106_DATA);
    }
}

/*!
    \brief      set oled display reverse
    \param[in]  reverse: ENABLE(reverse display),DISABLE(normal display)
    \param[out] none
    \retval     none
*/
static void oled_display_reverse(ControlStatus reverse)
{
    if(ENABLE == reverse){
        oled_write_byte(0xA7,SH1106_CMD);
    }else {
        oled_write_byte(0xA6,SH1106_CMD);
    }
}

/*!
    \brief      oled display flip horizontally
    \param[in]  flip: ENABLE(flip horizontal),DISABLE(normal display)
    \param[out] none
    \retval     none
*/
static void oled_flip_horizontal(ControlStatus flip)
{
    if(ENABLE == flip){
        oled_write_byte(0xC8,SH1106_CMD);
        oled_write_byte(0xA1,SH1106_CMD);
    }else {
        oled_write_byte(0xC0,SH1106_CMD);
        oled_write_byte(0xA0,SH1106_CMD);
    }
}

/*!
    \brief      refresh oled screen
    \param[in]  none 
    \param[out] none
    \retval     none
*/
static void oled_page_refresh(void)
{
    uint8_t i,n;
    /* display update */
    for(i=0;i<SH1106_PAGE_LENGTH;i++){
        /* set page start address for page addressing mode(B0h~B7h) */
        oled_write_byte (0xB0+i,SH1106_CMD);
        /* set lower column start address for page addressing mode(00h~0Fh) */
        oled_write_byte (0x02,SH1106_CMD);
        /* set higher column start address for page addressing mode(10h~1Fh) */
        oled_write_byte (0x10,SH1106_CMD);

        for(n=0;n<SH1106_COLUMN_LENGTH;n++){
            oled_write_byte(oled_buffer[n][i],SH1106_DATA);
        }
    }
}

/*!
    \brief      display a point
    \param[in]  x: position of point, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  y: position of point, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  value: 0 or 1
    \retval     none
*/
static void oled_draw_point(uint8_t x, uint8_t y, uint8_t value)
{
    uint8_t i,bit,temp;
    
    if((x<SH1106_COLUMN_LENGTH)&&(y<SH1106_ROW_LENGTH)){
        /* locate the page */
        i = y>>3;
        bit = y%8;
        temp = 0x01<<bit;
        
        if(value == 1){
            oled_buffer[x][i] |= temp;
        }else{
            oled_buffer[x][i] &= (~temp);
        }
    }
}

/*!
    \brief      display a picture
    \param[in]  x: starting point, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  y: starting point, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  sizex: size of column, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  sizey: size of row, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  picture: input data
    \retval     none
*/
static void oled_show_picture(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t picture[])
{
    uint16_t j=0;
    uint8_t i,m, n,temp;
    uint8_t x0=x,y0=y;
    
    /* page size of picture */
    sizey=sizey/8+((sizey%8)?1:0);
    
    
    for(n=0;n<sizey;n++){
        /* scan page */
        for(i=0;i<sizex;i++){
            /* scan column */
            temp=picture[j++];
            for(m=0;m<8;m++){
                /* scan point */
                if(temp&0x01)
                    oled_draw_point(x,y,1);
                else 
                    oled_draw_point(x,y,0);
                temp>>=1;
                y++;
            }
            /* column increase */
            x++;
            if((x-x0)==sizex){
                x=x0;
                /* next page */
                y0=y0+8;
            }
            y=y0;
        }
    }
}

/*!
    \brief      display a characters
    \param[in]  x: starting point, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  y: starting point, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  chr: characters
    \param[in]  size1: font size
    \retval     none
*/
static void oled_show_char(uint8_t x,uint8_t y,uint8_t chr,oled_font_enum size)
{
    uint8_t i,m,temp,len,chr1;
    uint8_t x0=x,y0=y;
    
    /* get num of bytes occupied by a font character corresponding to a lattice set */
    if(size==FONT_6_8)
        len=6;
    else 
        len=(size/8+((size%8)?1:0))*(size/2);
    
    /* get offset value */
    chr1=chr-' '; 

    for(i=0;i<len;i++){
        /* scan a character */
        if(size == FONT_6_8)
            temp=asc2_0806[chr1][i]; 
        else if(size == FONT_8_16)
            temp=asc2_1608[chr1][i];
        else if(size == FONT_12_24)
            temp=asc2_2412[chr1][i];
        else 
            return;
        /* scan point */
        for(m=0;m<8;m++){
            if(temp&0x01)
                oled_draw_point(x,y,1);
            else 
                oled_draw_point(x,y,0);
            temp>>=1;
            y++;
        }
        /* column increase */
        x++;
        if((size!=FONT_6_8)&&((x-x0)==size/2)){
            x=x0;
            /* next page */
            y0=y0+8;
        }
        y=y0;
  }
}

/*!
    \brief      show a string
    \param[in]  x: starting point, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  y: starting point, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  chr: string
    \param[in]  size1: font size
    \retval     none
*/
static void oled_show_string(uint8_t x,uint8_t y,uint8_t *chr,oled_font_enum size)
{
    while((*chr >= ' ')&&(*chr <= '~')){
        /* show a character */
        oled_show_char(x,y,*chr,size);
        /* next character */
        if(size == FONT_6_8)
            x+=6;
        else 
            x+=size/2;
        chr++;
    }
}

/*!
    \brief      display a number at the specified location
    \param[in]  x: the horizontal direction, 0-127
    \param[in]  y: the vertical direction, 0-7
    \param[in]  num: the displaying number
    \param[in]  len: the number length
    \param[in]  size1: the font type
    \param[out] none
    \retval     none
*/
static void oled_show_num(uint8_t x, uint8_t y, uint16_t num, uint8_t len, oled_font_enum size)
{
    uint8_t t,temp,m=0;
    
    /*  */
    if(size==FONT_6_8)
        m=2;
    
    /* sacn each number from high to low */
    for(t=0;t<len;t++){
        /* get the digit */
        temp=(num/oled_pow(10,len-t-1))%10;
        /* show num */
        if(temp==0){
            oled_show_char(x+(size/2+m)*t,y,'0',size);
        }else{
            oled_show_char(x+(size/2+m)*t,y,temp+'0',size);
        }
  }
}

/*!
    \brief      display a line at the specified location
    \param[in]  x: the horizontal direction, 0-127
    \param[in]  y: the vertical direction, 0-7
    \param[in]  num: the displaying number
    \param[in]  len: the number length
    \param[in]  size1: the font type
    \param[out] none
    \retval     none
*/
static void oled_show_line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2)
{
    uint16_t t; 
    int16_t xerr=0,yerr=0,delta_x,delta_y,distance;
    int16_t incx,incy,row,col;
    
    /* calculate the delta x and delta y */
    delta_x=x2-x1; 
    delta_y=y2-y1;
    
    /* get the starting point */
    row=x1;
    col=y1;
    
    /* x direction */
    if(delta_x>0){
        incx=1; 
    }else if(delta_x==0){
        incx=0;
    }else{
        incx=-1;
        delta_x=-delta_x;
    }
    
    /* y direction */
    if(delta_y>0){
        incy=1;
    }else if (delta_y==0){
        incy=0;
    }else{
        incy=-1;
        delta_y=-delta_y;
    }
    
    /* make sure which basis is used to self-increment */
    if(delta_x>delta_y){
        distance=delta_x;
    }else{
        distance=delta_y;
    }
    
    /* draw a line */
    for(t=0;t<distance+1;t++){
        oled_draw_point(row,col,1);
        xerr+=delta_x;
        yerr+=delta_y;
        if(xerr>distance){
            xerr-=distance;
            row+=incx;
        }
        if(yerr>distance){
            yerr-=distance;
            col+=incy;
        }
    }
}

/*!
    \brief      show index start page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index_start(void)
{
    uint8_t i,n;

    /* display the logo */
    for(i=0;i<8;i++){
        /* set page start address for page addressing mode(B0h~B7h) */
        oled_write_byte (0xB0+i,SH1106_CMD);
        /* set lower column start address for page addressing mode(00h~0Fh) */
        oled_write_byte (0x00,SH1106_CMD);
        /* set higher column start address for page addressing mode(10h~1Fh) */
        oled_write_byte (0x10,SH1106_CMD);

        /* white space */
        for(n=0;n<18;n++){
            oled_write_byte(0,SH1106_DATA);
        }
        /* loga data */
        for(n=0;n<86;n++){
            oled_write_byte(gImage_logo[i*86+n],SH1106_DATA);
        }
        delay_1ms(100);
    }
    delay_1ms(1000);
}

/*!
    \brief      show index 0 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index1_0(void)
{
    uint16_t speed_temp, ref_speed_temp;
    uint8_t num_length, ref_num_length;
    
    /* display speed */
    if(motor.mech_speed>0){
        speed_temp = (uint16_t)motor.mech_speed;
    }else{
        speed_temp = -(uint16_t)motor.mech_speed;
    }
    if(speed_temp<10){
        num_length = 1;
    }else if(speed_temp<100){
        num_length = 2;
    }else if(speed_temp<1000){
        num_length = 3;
    }else if(speed_temp<10000){
        num_length = 4;
    }else{
        num_length = 5;
    }
    
    if(motor.speed_ref>0){
        ref_speed_temp = motor.speed_ref;
    }else{
        ref_speed_temp = -motor.speed_ref;
    }
    if(ref_speed_temp<10){
        ref_num_length = 1;
    }else if(ref_speed_temp<100){
        ref_num_length = 2;
    }else if(ref_speed_temp<1000){
        ref_num_length = 3;
    }else if(ref_speed_temp<10000){
        ref_num_length = 4;
    }else{
        ref_num_length = 5;
    }

    /* PAGE1*/
    if(motor.state != MC_STATE_RUNNING){
        oled_show_string(3,19,(uint8_t*)"STATE:",FONT_6_8);
        oled_show_string(43,5,state_database[motor.state],FONT_12_24);
    }else{
        oled_show_string(3,19,(uint8_t*)"SPEED:",FONT_6_8);
        oled_show_num(43,5,speed_temp,num_length,FONT_12_24);
        oled_show_string(104,19,(uint8_t*)"rpm",FONT_6_8);
    
    }
    oled_show_string(3,38,(uint8_t*)"SPEED_REF:",FONT_6_8);
    if((rotor_angle.pos_mode != HALL_DETECT_MODE)&&(rotor_angle.pos_mode != ANGLE_CUSTOM_MODE)) {
        oled_show_num(68,38,ref_speed_temp,ref_num_length,FONT_6_8);
        oled_show_string(104,38,(uint8_t*)"rpm",FONT_6_8);
    }else{
        oled_show_string(68,38,(uint8_t*)"INVALID",FONT_6_8);
    }

    
    oled_show_line(2,2,125,2);
    oled_show_line(2,2,2,48);
    oled_show_line(2,48,125,48);
    oled_show_line(125,2,125,48);

    oled_show_string(3,53,(uint8_t*)"FAULT:",FONT_6_8);
    oled_show_string(43,53,fault_database[motor.fault],FONT_6_8);

    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 1 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index1_1(void)
{
    oled_show_string(20,2,(uint8_t*)"--SETTING--",FONT_8_16);
    oled_show_string(3,17,(uint8_t*)"MODE: ",FONT_6_8);
    oled_show_string(35,17,mode_database[motor.running_mode],FONT_6_8);
    oled_show_string(3,29,(uint8_t*)"DIR : ",FONT_6_8);
    oled_show_string(35,29,direction_database[motor.direction],FONT_6_8);
    oled_show_string(3,41,(uint8_t*)"SAMP: ",FONT_6_8);
    oled_show_string(35,41,sample_database[pwm_current.samp_mode],FONT_6_8);    
    oled_show_string(3,53,(uint8_t*)"PWM : ",FONT_6_8);
    oled_show_string(35,53,modulation_database[pwm_current.pwm_mode],FONT_6_8);
    oled_show_picture(112,50,11,11,arrow_11_11_right);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
    /*  */
    oled_show_qr_code(91,25);
}

/*!
    \brief      show index 2 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index2_2(void)
{
    oled_show_picture(43,5,43,42,mode_select_43_42);
    oled_show_picture(5,50,11,11,arrow_11_11_left);
    oled_show_picture(112,50,11,11,arrow_11_11_right);
    oled_show_string(20,48,(uint8_t*)"MODE SELECT",FONT_8_16);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 3 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index2_3(void)
{
    oled_show_picture(43,5,43,42,direction_config_43_42);
    oled_show_picture(5,50,11,11,arrow_11_11_left);
    oled_show_picture(112,50,11,11,arrow_11_11_right);
    oled_show_string(20,48,(uint8_t*)" DIRECTION",FONT_8_16);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 4 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index2_4(void)
{
    oled_show_picture(43,5,43,42,current_sampling_43_42);
    oled_show_picture(5,50,11,11,arrow_11_11_left);
    oled_show_picture(112,50,11,11,arrow_11_11_right);
    oled_show_string(20,48,(uint8_t*)"SAMPLE MODE",FONT_8_16);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 5 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index2_5(void)
{
    oled_show_picture(43,5,43,42,modulation_set_43_42);
    oled_show_picture(5,50,11,11,arrow_11_11_left);
    oled_show_string(20,48,(uint8_t*)"  PWM MODE",FONT_8_16);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 7 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_7(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[0],FONT_6_8);
    oled_show_string(18,17,mode_database[1],FONT_6_8);
    oled_show_string(18,29,mode_database[2],FONT_6_8);
    oled_show_string(18,41,mode_database[3],FONT_6_8);
    oled_show_string(18,53,mode_database[4],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 8 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_8(void)
{
    oled_show_picture(3,15,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[0],FONT_6_8);
    oled_show_string(18,17,mode_database[1],FONT_6_8);
    oled_show_string(18,29,mode_database[2],FONT_6_8);
    oled_show_string(18,41,mode_database[3],FONT_6_8);
    oled_show_string(18,53,mode_database[4],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 9 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_9(void)
{
    oled_show_picture(3,27,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[0],FONT_6_8);
    oled_show_string(18,17,mode_database[1],FONT_6_8);
    oled_show_string(18,29,mode_database[2],FONT_6_8);
    oled_show_string(18,41,mode_database[3],FONT_6_8);
    oled_show_string(18,53,mode_database[4],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 10 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_10(void)
{
    oled_show_picture(3,39,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[0],FONT_6_8);
    oled_show_string(18,17,mode_database[1],FONT_6_8);
    oled_show_string(18,29,mode_database[2],FONT_6_8);
    oled_show_string(18,41,mode_database[3],FONT_6_8);
    oled_show_string(18,53,mode_database[4],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 11 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_11(void)
{
    oled_show_picture(3,51,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[0],FONT_6_8);
    oled_show_string(18,17,mode_database[1],FONT_6_8);
    oled_show_string(18,29,mode_database[2],FONT_6_8);
    oled_show_string(18,41,mode_database[3],FONT_6_8);
    oled_show_string(18,53,mode_database[4],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 12 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_12(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[5],FONT_6_8);
    oled_show_string(18,17,mode_database[6],FONT_6_8);
    oled_show_string(18,29,mode_database[7],FONT_6_8);
    oled_show_string(18,41,mode_database[8],FONT_6_8);
    oled_show_string(18,53,mode_database[9],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 13 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_13(void)
{
    oled_show_picture(3,15,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[5],FONT_6_8);
    oled_show_string(18,17,mode_database[6],FONT_6_8);
    oled_show_string(18,29,mode_database[7],FONT_6_8);
    oled_show_string(18,41,mode_database[8],FONT_6_8);
    oled_show_string(18,53,mode_database[9],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 14 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_14(void)
{
    oled_show_picture(3,27,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[5],FONT_6_8);
    oled_show_string(18,17,mode_database[6],FONT_6_8);
    oled_show_string(18,29,mode_database[7],FONT_6_8);
    oled_show_string(18,41,mode_database[8],FONT_6_8);
    oled_show_string(18,53,mode_database[9],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

 /*!
    \brief      show index 15 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_15(void)
{
    oled_show_picture(3,39,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[5],FONT_6_8);
    oled_show_string(18,17,mode_database[6],FONT_6_8);
    oled_show_string(18,29,mode_database[7],FONT_6_8);
    oled_show_string(18,41,mode_database[8],FONT_6_8);
    oled_show_string(18,53,mode_database[9],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

 /*!
    \brief      show index 16 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_16(void)
{
    oled_show_picture(3,51,11,11,arrow_11_11_right);
    oled_show_string(18,5,mode_database[5],FONT_6_8);
    oled_show_string(18,17,mode_database[6],FONT_6_8);
    oled_show_string(18,29,mode_database[7],FONT_6_8);
    oled_show_string(18,41,mode_database[8],FONT_6_8);
    oled_show_string(18,53,mode_database[9],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 17 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_17(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,direction_database[0],FONT_6_8);
    oled_show_string(18,17,direction_database[1],FONT_6_8);

    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 18 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_18(void)
{
    oled_show_picture(3,15,11,11,arrow_11_11_right);
    oled_show_string(18,5,direction_database[0],FONT_6_8);
    oled_show_string(18,17,direction_database[1],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 19 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_19(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,sample_database[0],FONT_6_8);
    oled_show_string(18,17,sample_database[1],FONT_6_8);
    oled_show_string(18,29,sample_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 20 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_20(void)
{
    oled_show_picture(3,15,11,11,arrow_11_11_right);
    oled_show_string(18,5,sample_database[0],FONT_6_8);
    oled_show_string(18,17,sample_database[1],FONT_6_8);
    oled_show_string(18,29,sample_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 21 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_21(void)
{
    oled_show_picture(3,27,11,11,arrow_11_11_right);
    oled_show_string(18,5,sample_database[0],FONT_6_8);
    oled_show_string(18,17,sample_database[1],FONT_6_8);
    oled_show_string(18,29,sample_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 22 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_22(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,modulation_database[0],FONT_6_8);
    oled_show_string(18,17,modulation_database[1],FONT_6_8);
    oled_show_string(18,29,modulation_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 23 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_23(void)
{
    oled_show_picture(3,3,11,11,arrow_11_11_right);
    oled_show_string(18,5,modulation_database[1],FONT_6_8);
    oled_show_string(18,17,modulation_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index 24 page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index3_24(void)
{
    oled_show_picture(3,15,11,11,arrow_11_11_right);
    oled_show_string(18,5,modulation_database[1],FONT_6_8);
    oled_show_string(18,17,modulation_database[2],FONT_6_8);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show index hint page
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void fun_index_hint(void)
{
    oled_show_string(20,2,(uint8_t*)"--WARNING--",FONT_8_16);
    oled_show_string(8,25,(uint8_t*)"You can't use",FONT_6_8);
    oled_show_string(8,35,(uint8_t*)"1_Shunt and DSVPWM",FONT_6_8);
    oled_show_string(8,45,(uint8_t*)"at the same time!!!",FONT_6_8);
     pwm_current.pwm_mode = CSVPWM;

    /* display a rectangular box */
    oled_show_line(3,3,125,3);
    oled_show_line(3,3,3,61);
    oled_show_line(3,61,125,61);
    oled_show_line(125,3,125,61);
    /* display a rectangular box */
    oled_show_line(0,0,127,0);
    oled_show_line(0,0,0,63);
    oled_show_line(0,63,127,63);
    oled_show_line(127,0,127,63);
}

/*!
    \brief      show QR code
    \param[in]  x: starting point, 0 ~ (SH1106_COLUMN_LENGTH - 1)
    \param[in]  y: starting point, 0 ~ (SH1106_ROW_LENGTH - 1)
    \param[in]  chr: string
    \param[in]  size1: font size
    \retval     none
*/
static void oled_show_qr_code(uint8_t x,uint8_t y)
{
    uint8_t col,row;
    for(row=0;row<25;row++){
        for(col=0;col<25;col++){
        /* scan point */
        if(GDQR_logo[row][col]&0x01)
            oled_draw_point(x+col,y+row,1);
        else 
            oled_draw_point(x+col,y+row,0);
        }
    }
}
