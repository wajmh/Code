#ifndef UART4_DRV_H
#define UART4_DRV_H
// uart4_drv.h
#pragma once
#include "gd32f50x.h"
#include <stdint.h>

#define UART4_DE_PORT   GPIOC
#define UART4_DE_PIN    GPIO_PIN_13

// ????????
typedef struct {
    uint32_t input_single_pos;     // ??????? (17?)
    uint32_t output_single_pos;    // ??????? (17?)
    uint16_t output_multi_pos;     // ??????? (16?)
    uint8_t fault_code;           // ???
    uint8_t crc;                  // CRC??
} encoder_data_t;

void uart4_init(uint32_t baud);
void uart4_send(const uint8_t *buf, uint16_t len);
int  uart4_read(uint8_t *out, uint16_t max);  // ???????

// 485 编码器：仅发 0x6A 请求，不等待；应答在 UART RX 里由 uart4_encoder_rx_process() 解析并更新电角度
void uart4_request_encoder_data(void);
/** 在 UART RX 中断里调用：从接收缓冲解析一帧编码器应答，解析到则更新电角度供 angle_get 用 */
void uart4_encoder_rx_process(void);
int uart4_parse_encoder_data(encoder_data_t *data);

/**
 * 发送读取命令并解析得到输入端电角度（仅用输入端编码器，零位与电角度对齐）
 * \param[out] elec_angle_rad  电角度，单位 rad，范围 [0, 2π)
 * \return 0 成功，-1 接收/解析失败
 * \note 内部会发 0x6A 并等待一段时间再读；极对数、17 位分辨率与电机配置一致
 */
int uart4_get_elec_angle_rad(float *elec_angle_rad);

/** 非阻塞读最近一次电角度（FOC 中断中调用）；主循环需周期性调用 uart4_get_elec_angle_rad 刷新 */
void uart4_get_last_elec_angle_rad(float *elec_angle_rad);

#endif 