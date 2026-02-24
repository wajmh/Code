/*!
    \file    pwm_current.c
    \brief   invoke PWM and current sampling related functions

    \version 2025-06-30, V1.0.0, GDFOC2.0
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

#include "pwm_current.h"
#include "modulation.h"

/*!
    \brief      initialize the pwm and adc sample peripheral
    \param[in]  pwm_current: pointer to pwm_current_struct srtuct
                motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_current_peripheral_init(pwm_current_struct* pwm_current, motor_struct* motor)
{
    if(CURRENT_3SHUNT == pwm_current->samp_mode){
        pwm_current->shunt3_init(motor);
    }else if(CURRENT_2SHUNT == pwm_current->samp_mode){
        pwm_current->shunt2_init(motor);
    }else if(CURRENT_1SHUNT == pwm_current->samp_mode){
        pwm_current->shunt1_init(motor);
    }else{
        
    }
}

/*!
    \brief      pwm wave control
    \param[in]  pwm_current: pointer to pwm_current_struct srtuct
                motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void pwm_generation(pwm_current_struct* pwm_current, motor_struct* motor)
{
    if(CSVPWM == pwm_current->pwm_mode){
        csvpwm(&(pwm_current->modulation), motor);
    }else if (DSVPWMMIN == pwm_current->pwm_mode){
        dsvpwm(&(pwm_current->modulation), motor);
    }else{
    }

    if(CURRENT_3SHUNT == pwm_current->samp_mode){
        pwm_current->config_sample_3s(&(pwm_current->modulation),motor);
        pwm_current->update_duty_3s(pwm_current->modulation);
    } else if(CURRENT_2SHUNT == pwm_current->samp_mode){
        pwm_current->config_sample_2s(&(pwm_current->modulation),motor);
        pwm_current->update_duty_2s(pwm_current->modulation);
    }else if (CURRENT_1SHUNT == pwm_current->samp_mode){
        pwm_current->config_sample_1s(&(pwm_current->modulation),motor);
    }else{
        
    }
}

/*!
    \brief      get three-phase current
    \param[in]  pwm_current: pointer to pwm_current_struct srtuct
                motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     none
*/
void get_iab(pwm_current_struct* pwm_current, motor_struct* motor)
{
    if(CURRENT_3SHUNT == pwm_current->samp_mode){
        pwm_current->shunt3_get_iab(motor);
    }else if(CURRENT_2SHUNT == pwm_current->samp_mode){
        pwm_current->shunt2_get_iab(motor);
    }else if(CURRENT_1SHUNT == pwm_current->samp_mode){
        pwm_current->shunt1_get_iab(motor);
    }else{
        
    }
}
