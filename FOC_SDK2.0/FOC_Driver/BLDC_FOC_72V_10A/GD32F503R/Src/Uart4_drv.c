#include "Uart4_drv.h"
#include "gd32f50x_usart.h"
#include "gd32f50x_gpio.h"
#include "gd32f50x_rcu.h"
#include "gd32f50x_misc.h"

/* 输入端编码器：17 位，零位与电角度对齐；电机极对数 15（与 motor_para.h 一致） */
#define ENCODER_COUNTS_PER_REV   (131072.0f)   /* 2^17 */
#define ENCODER_POLE_PAIRS       (15U)
#define MOTOR_PI_2_F             (6.2831853f)  /* 2*pi */

#define UART4_RX_PORT   GPIOD
#define UART4_RX_PIN    GPIO_PIN_2
#define UART4_TX_PORT   GPIOC
#define UART4_TX_PIN    GPIO_PIN_12
#define UART4_AF        GPIO_AF_1

#define UART4_RX_BUF_SIZE 256

static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static uint8_t rx_buf[UART4_RX_BUF_SIZE];
static volatile float uart4_last_elec_angle_rad = 0.0f;

static void rb_push(uint8_t b)
{
    uint16_t next = (uint16_t)((rx_head + 1) % UART4_RX_BUF_SIZE);
    if (next != rx_tail) {
        rx_buf[rx_head] = b;
        rx_head = next;
    }
}

int uart4_read(uint8_t *out, uint16_t max)
{
    uint16_t cnt = 0;
    while ((rx_tail != rx_head) && (cnt < max)) {
        out[cnt++] = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1) % UART4_RX_BUF_SIZE);
    }
    return (int)cnt;
}

void uart4_init(uint32_t baud)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_UART4);

    gpio_mode_set(UART4_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART4_RX_PIN);
    gpio_af_set(UART4_RX_PORT, UART4_AF, UART4_RX_PIN);

    gpio_mode_set(UART4_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, UART4_TX_PIN);
    gpio_output_options_set(UART4_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL2, UART4_TX_PIN);
    gpio_af_set(UART4_TX_PORT, UART4_AF, UART4_TX_PIN);

    usart_deinit(UART4);
    usart_baudrate_set(UART4, baud);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);

    usart_interrupt_enable(UART4, USART_INT_RBNE);
    nvic_irq_enable(UART4_IRQn, 1, 0);

    usart_enable(UART4);
}

void uart4_send(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        usart_data_transmit(UART4, buf[i]);
        while (RESET == usart_flag_get(UART4, USART_FLAG_TBE)) {
        }
    }

    while (RESET == usart_flag_get(UART4, USART_FLAG_TC)) {
    }

    /* RS485 半双工：最小延时，应答由 RX 中断解析，不在此等待 */
//    for (volatile uint32_t i = 0; i < 2; i++) { (void)i; }
}

static uint8_t crc8_calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* 仅发 0x6A 请求，不等待应答；应答由 UART RX 中断里 uart4_encoder_rx_process() 解析并更新电角度 */
void uart4_request_encoder_data(void)
{
    uint8_t cmd = 0x6A;
    uart4_send(&cmd, 1);
}

/* 从环形缓冲区尝试解析一帧编码器应答（0x6A + 10 字节），解析成功则更新 uart4_last_elec_angle_rad；在 UART RX 中断里调用，快速处理 */
void uart4_encoder_rx_process(void)
{
    uint16_t i, cnt = (uint16_t)((rx_head - rx_tail + UART4_RX_BUF_SIZE) % UART4_RX_BUF_SIZE);
    if (cnt < 11) {
        return;
    }
    if (rx_buf[rx_tail] != 0x6A) {
        rx_tail = (uint16_t)((rx_tail + 1) % UART4_RX_BUF_SIZE);
        return;
    }
    {
        uint8_t frame[11];
        for (i = 0; i < 11; i++) {
            frame[i] = rx_buf[(uint16_t)((rx_tail + i) % UART4_RX_BUF_SIZE)];
        }
        rx_tail = (uint16_t)((rx_tail + 11) % UART4_RX_BUF_SIZE);

    uint32_t input_single_pos = ((uint32_t)frame[3] << 16) | ((uint32_t)frame[2] << 8) | frame[1];
    input_single_pos &= 0x1FFFF;
    float mech_rad = ((float)input_single_pos / ENCODER_COUNTS_PER_REV) * MOTOR_PI_2_F;
    float elec = mech_rad * (float)ENCODER_POLE_PAIRS;
    while (elec >= MOTOR_PI_2_F) {
        elec -= MOTOR_PI_2_F;
    }
    while (elec < 0.0f) {
        elec += MOTOR_PI_2_F;
    }
    uart4_last_elec_angle_rad = elec;
    }
}


int uart4_parse_encoder_data(encoder_data_t *data)
{
    uint8_t rx_buf[12]; 
    int rx_len = uart4_read(rx_buf, sizeof(rx_buf));

    /* 必须收到至少 11 字节：帧头 0x6A + 10 字节数据 */
    if (rx_len < 11) {
        return -1;  
    }

    /* 帧头必须是 0x6A（与请求命令一致） */
    if (rx_buf[0] != 0x6A) {
        return -1;  
    }

    data->input_single_pos = ((uint32_t)rx_buf[3] << 16) | ((uint32_t)rx_buf[2] << 8) | rx_buf[1];
    data->input_single_pos &= 0x1FFFF;  

    data->output_single_pos = ((uint32_t)rx_buf[6] << 16) | ((uint32_t)rx_buf[5] << 8) | rx_buf[4];
    data->output_single_pos &= 0x1FFFF;  

    data->output_multi_pos = ((uint16_t)rx_buf[8] << 8) | rx_buf[7];

    data->fault_code = rx_buf[9];

    data->crc = rx_buf[10];
    uint8_t calculated_crc = crc8_calc(rx_buf, 10);


    return 0;
}

/* 等待编码器应答的延时（RS485 半双工切换 + 编码器处理时间）；减小可提高角度更新率、减轻大 q 时抖动 */



/**
 * 非阻塞：返回最近一次 uart4_get_elec_angle_rad 成功得到的电角度（FOC 中断中调用）
 * 主循环需周期性调用 uart4_get_elec_angle_rad() 以刷新该值
 */
void uart4_get_last_elec_angle_rad(float *elec_angle_rad)
{
    if (elec_angle_rad != ((void*)0)) {
        *elec_angle_rad = uart4_last_elec_angle_rad;
    }
}

void UART4_IRQHandler(void)
{
    if (RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE)) {
        uint8_t b = (uint8_t)usart_data_receive(UART4);
        rb_push(b);
        uart4_encoder_rx_process();  /* 收到字节即尝试解析一帧，解析到则更新电角度供 angle_get 用 */
    }
}
