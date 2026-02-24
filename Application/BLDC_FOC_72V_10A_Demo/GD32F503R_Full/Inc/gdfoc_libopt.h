/*!
    \file    gdfoc_libopt.h
    \brief   library optional for foc control

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

#ifndef GDFOC_LIBOPT_H
#define GDFOC_LIBOPT_H

/* application layer */
#include "motor_para.h"
#include "control_para.h"
#include "motor_para.h"
#include "gd32_bldc_foc_eval.h"
#include "communication.h"
#include "systick.h"

/* FOC algorithm layer */
#include "foc_math.h"
#include "modulation.h"

/* FOC topology layer */
#include "topology_para.h"
#include "pwm_current.h"
#include "angle.h"
#include "motor_ctrl.h"

/* FOC driver layer */
#include "pwm_current_2shunt.h"
#include "pwm_current_3shunt.h"
#include "foc_soft_delay.h"
#include "motor_ctrl_driver.h"
#include "angle_mencoder_as5047p.h"




#endif /* GDFOC_LIBOPT_H */
