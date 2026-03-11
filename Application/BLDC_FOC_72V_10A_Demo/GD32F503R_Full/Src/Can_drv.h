#ifndef CAN_DRV_H
#define CAN_DRV_H
#define CAN_ENABLE_LOOPBACK   1   /* 1=MCU内部环回自测, 0=正常总线模式 */
#include "gd32f50x.h"

typedef struct {
    uint32_t id;      /* 11-bit or 29-bit */
    uint8_t ide;      /* 0: standard, 1: extended */
    uint8_t rtr;      /* 0: data, 1: remote */
    uint8_t len;      /* 0..8 */
    uint8_t data[8];
} can_frame_t;

extern volatile can_frame_t g_can_rx_message;
extern volatile uint8_t g_can_rx_new_flag;

/* init CAN1 on PB6(TX)/PB5(RX), fixed 1Mbps */
uint8_t can_drv_init(void);

/* send standard frame */
uint8_t can_drv_send_std(uint16_t std_id, const uint8_t *data, uint8_t len);

/* pop latest rx frame, return 1 when got frame */
uint8_t can_drv_pop(can_frame_t *frame);

#endif /* CAN_DRV_H */
