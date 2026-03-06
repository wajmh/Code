#ifndef FLASH_DRV_H
#define FLASH_DRV_H

#include <stdint.h>

uint8_t motor_id_flash_load(uint8_t *id);
uint8_t motor_id_flash_save(uint8_t id);

#endif /* FLASH_DRV_H */
