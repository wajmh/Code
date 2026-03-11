#include "Can_protocol.h"
#include "Can_drv.h"
#include "gdfoc_config.h"
#include "Uart4_drv.h"
uint8_t return_4data[4] = {0x00,0x00,0x00,0x00};
uint8_t return_5data[5] = {0x00,0x00,0x00,0x00,0x00};
uint8_t return_8data[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static void type1_response(uint16_t motor_id);
void od_0x07FF_callback(can_frame_t *can_rx_message)
{
    uint8_t length = can_rx_message->len;
    uint32_t rw_buf = 0;
    uint8_t boot_state = 0xFF;
    uint8_t boot_partition_select = 0xFF;
    uint8_t write_cnt = 0;
    switch (length)
    {
        case 4:
            if ((can_rx_message->data[3]== 0x03))
            {
                uint16_t motor_id = can_rx_message->data[0] << 8 | can_rx_message->data[1];
                if (motor.motor_id != motor_id) return;
                
                // 电机零点设置（设置当前位置为零点）
//                if ((motor.error_flag == 0)&&(pri_float_abs(board_param.velocity_measure)<ZERO_VELOCITY_THD))
            if (motor.fault == FAULT_NONE)
                {
                    return_4data[0] = can_rx_message->data[0];
                    return_4data[1] = can_rx_message->data[1];
                    return_4data[2] = 0x01;
                    return_4data[3] = 0x03;
                    can_drv_send_std(0x7FF,return_4data,4);
                }
                else
                {
                    return_4data[0] = can_rx_message->data[0];
                    return_4data[1] = can_rx_message->data[1];
                    return_4data[2] = 0x00;
                    return_4data[3] = 0x00;
                    can_drv_send_std(0x7FF,return_4data,4);
                }
            }
            else if ((can_rx_message->data[3]== 0x82))
            {
                return_5data[0] = 0xFF;
                return_5data[1] = 0xFF;
                return_5data[2] = 0x01;
                return_5data[3] = (motor.motor_id >> 8) & (0xFF);
                return_5data[4] = (motor.motor_id & 0xFF);
                can_drv_send_std(0x7FF,return_5data,5);
            }
            break;
        case 6:
            if ((can_rx_message->data[3]== 0x04))
            {
                uint16_t motor_id = can_rx_message->data[0] << 8 | can_rx_message->data[1];
                if (motor.motor_id != motor_id) return;
                uint16_t motor_new_id = can_rx_message->data[4] << 8 | can_rx_message->data[5];
                //axis.set_motor_num(motor_new_id);
                //需要添加修改id的操作
                return_4data[0] = can_rx_message->data[4];
                return_4data[1] = can_rx_message->data[5];
                return_4data[2] = 0x01;
                return_4data[3] = 0x04;
                can_drv_send_std(0x7FF,return_4data,4);
            }
            else if ((can_rx_message->data[3]== 0x05))
            {
                //axis.set_motor_num(0x01);//重置id
                // rw_buf = read_settings_boot();
                // boot_state = rw_buf & 0xFF;
                // boot_partition_select = (rw_buf >> 8) & 0xFF;
                // while (write_settings_boot(boot_state,boot_partition_select,0x01)<=0)
                // {
                //     write_cnt++;
                //     if (write_cnt > 10)
                //     {
                //         break;
                //     }
                // }
                return_4data[0] = 0x7F;
                return_4data[1] = 0x7F;
                return_4data[2] = 0x01;
                return_4data[3] = 0x05;
                can_drv_send_std(0x7FF,return_4data,4);
            }
        case 8: {
            uint16_t motor_id = can_rx_message->id;
            uint16_t temp_cnt = 0;

            temp_cnt = (can_rx_message->data[0] & 0x1F) << 7;
            temp_cnt |= (can_rx_message->data[1] & 0xFE) >> 1;
            motor.force_position_kp = temp_cnt * (0.1221001221f); // 500.0f / 4095

            temp_cnt = 0;
            temp_cnt = (can_rx_message->data[1] & 0x01) << 8;
            temp_cnt |= (can_rx_message->data[2] & 0xFF);
            motor.force_position_kd = temp_cnt * (0.097847358121330724f); // 5.0f / 511.0f

            temp_cnt = 0;
            temp_cnt = (can_rx_message->data[3] & 0xFF) << 8;
            temp_cnt |= (can_rx_message->data[4] & 0xFF);
            motor.force_position_target_pos = (temp_cnt * (0.0003814697265625f) - 12.5f); // 25.0f / 65536.0f

            temp_cnt = 0;
            temp_cnt = (can_rx_message->data[5] & 0xFF) << 4;
            temp_cnt |= (can_rx_message->data[6] & 0xF0) >> 4;
            motor.force_position_target_vel = (temp_cnt * (0.0087890625f) - 18.0f); // 18.0f / 2048.0f

            temp_cnt = 0;
            temp_cnt = (can_rx_message->data[6] & 0x0F) << 8;
            temp_cnt |= (can_rx_message->data[7] & 0xFF);
            motor.force_position_target_torque = (temp_cnt * (0.087890625f) - 180.0f); // 180.0f / 2048.0f
            type1_response(motor_id);
            (void)motor_id;
            break;
        }
    }
}
static void type1_response(uint16_t motor_id){
    float pos_rad = 0.0f;
    float vel_rad_s = 0.0f;
    float load_iq = 0.0f;
    uint8_t jianmi_warning_flag = 0;
    uint16_t temp_cnt = 0;
    uart4_get_last_output_state(&pos_rad, &vel_rad_s);
    if (motor.fault == FAULT_VOLTAGE_ERROR) {
        jianmi_warning_flag |= 0x04;
    }
    if (motor.fault == FAULT_OVER_CURRENT) {
        jianmi_warning_flag |= 0x02;
    }
    if (motor.fault == FAULT_ANGLE) {
        jianmi_warning_flag  |= 0x08;
    }
    return_8data[0] = (jianmi_warning_flag & 0x1F) | 0x20;
    temp_cnt = (uint16_t)((pos_rad + 12.5f) * (65536.0f / 25.0f));
    return_8data[1] = (temp_cnt >> 8) | 0x80;
    return_8data[2] = temp_cnt & 0xFF;
    temp_cnt = (uint16_t)(vel_rad_s * (2048.0f / 18.0f) + 2048.0f);
    return_8data[3] = (temp_cnt & 0x0FF0) >> 4;
    return_8data[4] = (temp_cnt & 0x000F) << 4;
    load_iq = motor.iq;
    temp_cnt = (uint16_t)(load_iq * (2048.0f / 140.0f) + 2048.0f);
    return_8data[4] |= (temp_cnt & 0x0F00) >> 8;
    return_8data[5] = temp_cnt & 0xFF;
    return_8data[6] = 100;
    return_8data[7] = 100;
    can_drv_send_std(motor_id, return_8data, 8);

}
