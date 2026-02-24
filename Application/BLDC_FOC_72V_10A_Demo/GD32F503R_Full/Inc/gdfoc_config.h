/*!
    \file    gdfoc_config.h
    \brief   definitions for foc config

    \version 2025-06-30, V1.0.0, GDFOC2.0 for GD32F30x
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

#ifndef GDFOC_CONFIG_H
#define GDFOC_CONFIG_H

#include "angle.h"
#include "pwm_current.h"

/* extern variable definition */
/* PID for speed control */
extern pid_struct speed_pid;
/* PID for torque control */
extern pid_struct torque_pid;
/* PID for flux control */
extern pid_struct flux_pid;

extern pid_struct smo_pll_pid;
/* motor control */
extern motor_struct motor;
/* angle sensor configuration */
extern angle_struct rotor_angle;
/* current sampling and pwm configuration */
extern pwm_current_struct pwm_current;

#endif /* GDFOC_CONFIG_H */
