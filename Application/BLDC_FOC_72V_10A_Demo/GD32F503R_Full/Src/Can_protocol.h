#ifndef __CAN_PROTOCOL_H__
#define __CAN_PROTOCOL_H__

#include "Can_drv.h" 

void od_0x07FF_callback(can_frame_t *can_rx_message);
void can_heartbeat_feed(void);
void can_heartbeat_timeout_check(void);

#endif
