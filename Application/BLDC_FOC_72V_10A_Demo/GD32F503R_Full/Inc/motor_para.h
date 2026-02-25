/*!
    \file    motor_para.h
    \brief   configure parameters of motor

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

#ifndef MOTOR_PARA_H
#define MOTOR_PARA_H

/* motor parameters */
/* the pole-pair number of pmsm */
#define NUMBER_OF_PAIRS                     (15U)                                /* PMSM number of pair poles */
/* resistance and inductance in alpha-beta axis */
#define RS                                  (0.55f)                              /* equivalent stator resistance in alpha-beta axis, ohm*/
#define LD                                  (0.00044f)                           /* equivalent stator inductance in d axis, H */
#define LQ                                  (0.00066f)                           /* equivalent stator inductance in q, H */
#define FLUX                                (0.0178f)                            /* PMSM permanent flux, Wb */

#endif /* MOTOR_PARA_H */
