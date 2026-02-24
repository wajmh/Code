/*!
    \file    motor_ctrl.c
    \brief   invoke motor control related functions

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

#include "motor_ctrl.h"

/*!
    \brief      initialize motor status
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_init(motor_struct* motor)
{
    motor->state = MC_STATE_INIT;
}

/*!
    \brief      start motor
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_start(motor_struct* motor)
{
    motor->motor_start();
}

/*!
    \brief      stop motor
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_stop(motor_struct* motor)
{
    motor->motor_stop();
}

/*!
    \brief      break motor
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_break(motor_struct* motor)
{
    motor->motor_break();
}

/*!
    \brief      calculate the motor voltage of d-q
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_udq_calc(motor_struct* motor)
{
    if(UDQ_CUSTOM == motor->udq_mode){
        /* ud and uq in custom mode */
        motor->ud = motor->motor_openloop.d_var;
        motor->uq = motor->motor_openloop.q_var;
    }else if (UDQ_MTPA_IF == motor->udq_mode){
        /* ud and uq in pid mode */
        motor->ud = pid_incremental_regulation(motor->id_ref,motor->id,motor->flux_pid);
        motor->uq = pid_incremental_regulation(motor->iq_ref,motor->iq,motor->torque_pid);
    }else{
        /* none */
    }
}

/*!
    \brief      calculate the reference current of d-q
    \param[in]  angle: pointer to angle srtuct
    \param[in]  motor: pointer to motor struct
    \param[out] none
    \retval     none
*/
void motor_idqref_calc(motor_struct* motor)
{
    if(MC_STATE_RUNNING == motor->state){
        if(IDQ_SPEED_MTPA == motor->idqref_mode){
            if(DIRECTION_CCW == motor->direction){
                motor->iq_ref = pid_incremental_regulation(motor->speed_ref, motor->mech_speed, (motor->speed_pid));
                motor->id_ref = 0;
            }else{
                motor->iq_ref = - pid_incremental_regulation(motor->speed_ref, motor->mech_speed, (motor->speed_pid));
                motor->id_ref = 0;
            }
        }else{
        }
    }
}
