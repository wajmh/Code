/* ÀýÈçÐÂ½¨ motor_id_flash.c */
#include "gd32f50x_fmc.h"
#include "topology_para.h"

uint8_t motor_id_flash_load(uint8_t *id)
{
    uint32_t magic;
    uint16_t raw;

    if(id == 0) return 0U;

    magic = *(volatile uint32_t *)FMC_MOTOR_ID_MAGIC_ADDR;
    raw   = *(volatile uint16_t *)FMC_MOTOR_ID_ADDR;

    if((magic != MOTOR_ID_FLASH_MAGIC) || (raw > 255U)) {
        *id = MOTOR_ID_DEFAULT;
        return 0U;
    }

    *id = (uint8_t)raw;
    return 1U;
}

uint8_t motor_id_flash_save(uint8_t id)
{
    if(id > 255U) return 0U;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    if(FMC_READY != fmc_page_erase(FMC_MOTOR_ID_PAGE_ADDR)) { fmc_lock(); return 0U; }
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    if(FMC_READY != fmc_halfword_program(FMC_MOTOR_ID_ADDR, (uint16_t)id)) { fmc_lock(); return 0U; }
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    if(FMC_READY != fmc_word_program(FMC_MOTOR_ID_MAGIC_ADDR, MOTOR_ID_FLASH_MAGIC)) { fmc_lock(); return 0U; }
    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    fmc_lock();
    return 1U;
}
