/*!
    \file    modulation.c
    \brief   modulation algorithm source file

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

#include "modulation.h"

/*!
    \brief      anti-windup processing of the applying voltage
    \param[in]  motor: pointer to motor_struct srtuct
    \param[out] none
    \retval     anti-windup flag
*/
uint8_t over_modulation(motor_struct* motor)
{
    uint8_t retval = 0;
    float mag;
    float temp,square_ud,square_uq;
    /* Calculate the square of ud and uq */
    square_ud = motor->ud * motor->ud;
    square_uq = motor->uq * motor->uq;
    /* determine whether overmodulation processing is required */
    if(motor->us_out_max * motor->us_out_max - square_ud - square_uq < 0){
        temp = motor->us_out_max * motor->us_out_max - square_ud;
        if(temp <= 0){
            /* The square of ud is greater than or equal to the square of us_out_max*/
            temp=0;
            if(motor->ud>0){
                /* ud is greater than the us_out_max */
                motor->ud = motor->us_out_max;
            }else{
                /* ud is less than or equal to the us_out_max */
                motor->ud = -motor->us_out_max;
            }
            motor->uq = 0;
        }else{
            /* The square of ud is less than the square of us_out_max */
            mag = sqrt_float(temp);
            if(motor->uq > 0){
                /* uq is greater than 0, so A takes the positive square root */
                motor->uq = mag;
            }else{
                /* uq is less than or equal to 0, so A takes the positive square root */
                motor->uq = -mag;
            }
        }
        /* set over modulation flag */
        retval = 1;
    }
    return retval;

}

/*!
    \brief      continuous space vector pulse width modulation
    \param[out] modul: pointer to modulation_struct srtuct
    \param[out] motor: pointer to motor_struct srtuct
    \param[in]  none
    \retval     none
*/
void csvpwm(modulation_struct* modul, motor_struct* motor)
{
    float alpha, beta;

    /* voltage normalization */
    alpha = (float)motor->ualpha/motor->us_base;
    beta = (float)motor->ubeta/motor->us_base;

    /* sector recognization */
    if(beta >= 0.0f){
        if(alpha >= 0.0f){
            /* quadrant I */
            if(ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=TWO;
            else
                motor->sector=ONE;
        }else{
            /* quadrant II */
            if(-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=THREE;
            else
                motor->sector=TWO;
        }
    }else{
        if(alpha >= 0.0f){
            /* quadrant IV */
            if(-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=FIVE;
            else
                motor->sector=SIX;
        }else{
            /* quadrant III */
            if(ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=FOUR;
            else
                motor->sector=FIVE;
        }
    }

    switch(motor->sector){
        /* sector 1: vector 100 and vector 110 */
        case ONE: {
            /* the time of vectors */
            uint32_t t4 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t6 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tC = (modul->pwm_top - t4 - t6) >>1;
            modul->tB = modul->tC + t6;
            modul->tA = modul->tB + t4;
            break;
        }

        /* sector 2: vector 110 and vector 010 */
        case TWO: {
            /* the time of vectors */
            uint32_t t6 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t2 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tC = (modul->pwm_top - t6 - t2) >>1;
            modul->tA = modul->tC + t6;
            modul->tB = modul->tA + t2;
            break;
        }

        /* sector 3: vector 010 and vector 011 */
        case THREE: {
            /* the time of vectors */
            uint32_t t2 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t3 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tA = (modul->pwm_top - t2 - t3) >>1;
            modul->tC = modul->tA + t3;
            modul->tB = modul->tC + t2;
            break;
        }

        /* sector 4: vector 011 and vector 001 */
        case FOUR: {
            /* the time of vectors */
            uint32_t t3 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t1 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tA = (modul->pwm_top - t3 - t1) >>1;
            modul->tB = modul->tA + t3;
            modul->tC = modul->tB + t1;
            break;
        }

        /* sector 5: vector 001 and vector 101 */
        case FIVE: {
            /* the time of vectors */
            uint32_t t1 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t5 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tB = (modul->pwm_top - t5 - t1) >>1;
            modul->tA = modul->tB + t5;
            modul->tC = modul->tA + t1;
            break;
        }

        /* sector 6: vector 101 and vector 100 */
        case SIX: {
            /* the time of vectors */
            uint32_t t5 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t4 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tB = (modul->pwm_top - t5 - t4) >>1;
            modul->tC = modul->tB + t5;
            modul->tA = modul->tC + t4;
            break;
        }

        default:
            break;
    }
}

/*!
    \brief      discontinuous space vector pulse width modulation
    \param[out] modul: pointer to modulation_struct srtuct
    \param[out] motor: pointer to motor_struct srtuct
    \param[in]  none
    \retval     none
*/
void dsvpwm(modulation_struct* modul, motor_struct* motor)
{
    float alpha, beta;

    /* voltage normalization */
    alpha = (float)motor->ualpha/motor->us_base;
    beta = (float)motor->ubeta/motor->us_base;

    /* sector recognization */
    if(beta >= 0.0f){
        if(alpha >= 0.0f){
            /* quadrant I */
            if(ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=TWO;
            else
                motor->sector=ONE;
        }else{
            /* quadrant II */
            if(-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=THREE;
            else
                motor->sector=TWO;
        }
    }else{
        if(alpha >= 0.0f){
            /* quadrant IV */
            if(-ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=FIVE;
            else
                motor->sector=SIX;
        }else{
            /* quadrant III */
            if(ONE_DIVIDE_SQRT3 * beta > alpha)
                motor->sector=FOUR;
            else
                motor->sector=FIVE;
        }
    }

    switch(motor->sector){
        /* sector 1: vector 100 and vector 110 */
        case ONE: {
            /* the time of vectors */
            uint32_t t4 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t6 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tC = 0;
            modul->tB = modul->tC + t6;
            modul->tA = modul->tB + t4;
            break;
        }

        /* sector 2: vector 110 and vector 010 */
        case TWO: {
            /* the time of vectors */
            uint32_t t6 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t2 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tC = 0;
            modul->tA = modul->tC + t6;
            modul->tB = modul->tA + t2;
            break;
        }

        /* sector 3: vector 010 and vector 011 */
        case THREE: {
            /* the time of vectors */
            uint32_t t2 = (uint32_t)((TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t3 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tA = 0;
            modul->tC = modul->tA + t3;
            modul->tB = modul->tC + t2;
            break;
        }

        /* sector 4: vector 011 and vector 001 */
        case FOUR: {
            /* the time of vectors */
            uint32_t t3 = (uint32_t)((-alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t1 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tA = 0;
            modul->tB = modul->tA + t3;
            modul->tC = modul->tB + t1;
            break;
        }

        /* sector 5: vector 001 and vector 101 */
        case FIVE: {
            /* the time of vectors */
            uint32_t t1 = (uint32_t)((-alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t5 = (uint32_t)((alpha - ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tB = 0;
            modul->tA = modul->tB + t5;
            modul->tC = modul->tA + t1;
            break;
        }

        /* sector 6: vector 101 and vector 100 */
        case SIX: {
            /* the time of vectors */
            uint32_t t5 = (uint32_t)((-TWO_DIVIDE_SQRT3 * beta) * modul->pwm_top);
            uint32_t t4 = (uint32_t)((alpha + ONE_DIVIDE_SQRT3 * beta) * modul->pwm_top);

            /* TIMER PWM timings */
            modul->tB = 0;
            modul->tC = modul->tB + t5;
            modul->tA = modul->tC + t4;
            break;
        }

        default:
            break;
    }
}
