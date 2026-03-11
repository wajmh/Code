#include "Can_drv.h"
#include "gdfoc_config.h"
#include "gd32f50x_can.h"
#include "gd32f50x_gpio.h"
#include "gd32f50x_rcu.h"
#include "gd32f50x_misc.h"

#define CAN_TX_PORT         GPIOB
#define CAN_TX_PIN          GPIO_PIN_6
#define CAN_RX_PORT         GPIOB
#define CAN_RX_PIN          GPIO_PIN_5

/* 按你板子原理图: PB6=CAN1_TX, PB5=CAN1_RX
 * AF号请按芯片手册确认；GD32F503常用为 AF3
 */
#define CAN_GPIO_AF         GPIO_AF_3

/* fixed 1Mbps @ APB2=252MHz
 * bitrate = 252MHz / (14 * (1 + 13 + 4)) = 1MHz
 */
#define CAN_BRP_1M          14U
#define CAN_BS1_1M          CAN_BT_BS1_13TQ
#define CAN_BS2_1M          CAN_BT_BS2_4TQ
#define CAN_SJW_1M          CAN_BT_SJW_1TQ
#define CAN_TX_QUEUE_SIZE   8U

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} can_tx_frame_t;

static volatile uint8_t s_rx_ready = 0U;
static can_frame_t s_rx_frame;
static can_tx_frame_t s_tx_queue[CAN_TX_QUEUE_SIZE];
static volatile uint8_t s_tx_head = 0U;
static volatile uint8_t s_tx_tail = 0U;
volatile can_frame_t g_can_rx_message = {0};
volatile uint8_t g_can_rx_new_flag = 0U;
void od_0x07FF_callback(can_frame_t *can_rx_message);

static void can_tx_kick(void)
{
    can_transmit_message_struct tx;
    uint8_t i;

    while (s_tx_tail != s_tx_head) {
        can_struct_para_init(CAN_TX_MESSAGE_STRUCT, &tx);
        tx.tx_sfid = (uint32_t)(s_tx_queue[s_tx_tail].id & 0x7FFU);
        tx.tx_ff = (uint8_t)CAN_FF_STANDARD;
        tx.tx_ft = (uint8_t)CAN_FT_DATA;
        tx.tx_dlen = s_tx_queue[s_tx_tail].len;

        for (i = 0U; i < tx.tx_dlen; i++) {
            tx.tx_data[i] = s_tx_queue[s_tx_tail].data[i];
        }

        if (can_message_transmit(CAN1, &tx) == CAN_NOMAILBOX) {
            break;
        }

        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % CAN_TX_QUEUE_SIZE);
    }
}

static void can_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(CAN_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CAN_TX_PIN);
    gpio_output_options_set(CAN_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, CAN_TX_PIN);
    gpio_af_set(CAN_TX_PORT, CAN_GPIO_AF, CAN_TX_PIN);

    gpio_mode_set(CAN_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, CAN_RX_PIN);
    gpio_output_options_set(CAN_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_LEVEL3, CAN_RX_PIN);
    gpio_af_set(CAN_RX_PORT, CAN_GPIO_AF, CAN_RX_PIN);
}

uint8_t can_drv_init(void)
{
    can_parameter_struct can_para;
	  can_filter_parameter_struct can_filter;

    can_gpio_init();
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);

    can_deinit(CAN1);
    can_struct_para_init(CAN_INIT_STRUCT, &can_para);
    can_para.working_mode = CAN_NORMAL_MODE;
    can_para.resync_jump_width = CAN_SJW_1M;
    can_para.time_segment_1 = CAN_BS1_1M;
    can_para.time_segment_2 = CAN_BS2_1M;
    can_para.prescaler = CAN_BRP_1M;

    can_para.time_triggered = DISABLE;
    can_para.auto_bus_off_recovery = ENABLE;
    can_para.auto_wake_up = DISABLE;
    can_para.auto_retrans = ENABLE;
    can_para.rec_fifo_overwrite = DISABLE;
    can_para.trans_fifo_order = DISABLE;

    if (SUCCESS != can_init(CAN1, &can_para)) {
        return 0U;
    }

    /* CAN1 使用后半过滤器 bank，仅收 0x7FF 与 motor_id（标准帧） */
    can1_filter_start_bank(14U);
    /* 精确匹配标准 ID 0x7FF */
    can_filter_mask_mode_init(0x7FFU, 0x7FFU, CAN_STANDARD_FIFO0, 14U);
    /* 精确匹配 motor_id（低 11 位），放 FIFO0 */
    can_filter_mask_mode_init((uint32_t)(motor.motor_id & 0x7FFU), 0x7FFU, CAN_STANDARD_FIFO0, 15U);

    can_interrupt_enable(CAN1, CAN_INT_RFNE0);
    can_interrupt_enable(CAN1, CAN_INT_TME);
    nvic_irq_enable(CAN1_RX0_IRQn, 2U, 0U);
    nvic_irq_enable(CAN1_TX_IRQn, 3U, 0U);

    return 1U;
}

uint8_t can_drv_send_std(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    uint32_t primask;
    uint8_t i;
    uint8_t next;

    if (len > 8U) {
        len = 8U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    next = (uint8_t)((s_tx_head + 1U) % CAN_TX_QUEUE_SIZE);
    if (next == s_tx_tail) {
        s_tx_tail = (uint8_t)((s_tx_tail + 1U) % CAN_TX_QUEUE_SIZE);
    }

    s_tx_queue[s_tx_head].id = std_id;
    s_tx_queue[s_tx_head].len = len;

    for (i = 0U; i < len; i++) {
        s_tx_queue[s_tx_head].data[i] = data[i];
    }

    s_tx_head = next;

    __set_PRIMASK(primask);
    NVIC_SetPendingIRQ(CAN1_TX_IRQn);

    return 1U;
}

uint8_t can_drv_pop(can_frame_t *frame)
{
    uint32_t primask;

    if ((frame == 0) || (s_rx_ready == 0U)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *frame = s_rx_frame;
    s_rx_ready = 0U;
    __set_PRIMASK(primask);

    return 1U;
}

/* startup_gd32f50x.s 里是 weak，直接在这里实现即可 */
void CAN1_RX0_IRQHandler(void)
{
    can_receive_message_struct rx;
    uint8_t i, dl;

    while (can_receive_message_length_get(CAN1, CAN_FIFO0) > 0U) {
        can_struct_para_init(CAN_RX_MESSAGE_STRUCT, &rx);
        can_message_receive(CAN1, CAN_FIFO0, &rx);

        g_can_rx_message.ide = (rx.rx_ff == CAN_FF_EXTENDED) ? 1U : 0U;
        g_can_rx_message.id = g_can_rx_message.ide ? rx.rx_efid : rx.rx_sfid;
        g_can_rx_message.rtr = (rx.rx_ft == CAN_FT_REMOTE) ? 1U : 0U;

        dl = rx.rx_dlen;
        if (dl > 8U) dl = 8U;
        g_can_rx_message.len = dl;

        for (i = 0; i < dl; i++) {
            g_can_rx_message.data[i] = rx.rx_data[i];
        }

        s_rx_frame = g_can_rx_message;
        s_rx_ready = 1U;
        g_can_rx_new_flag = 1U;
    }
}

void CAN1_TX_IRQHandler(void)
{
    can_interrupt_flag_clear(CAN1, CAN_INT_FLAG_MTF0);
    can_interrupt_flag_clear(CAN1, CAN_INT_FLAG_MTF1);
    can_interrupt_flag_clear(CAN1, CAN_INT_FLAG_MTF2);
    can_tx_kick();
}
