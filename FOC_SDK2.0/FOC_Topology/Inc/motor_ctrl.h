/*!
    \file    motor_ctrl.h
    \brief   the header file of motor control

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

#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "motor.h"

/* function declaration */
/* initialize motor status */
void motor_init(motor_struct* motor);
/* start motor */
void motor_start(motor_struct* motor);
/* stop motor */
void motor_stop(motor_struct* motor);
/* break motor */
void motor_break(motor_struct* motor);
/* calculate the motor voltage of d-q */
void motor_udq_calc(motor_struct* motor);
/* calculate the reference current of d-q */
void motor_idqref_calc(motor_struct* motor);

#endif /* MOTOR_CTRL_H */
