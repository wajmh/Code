/*!
    \file    foc_math.h
    \brief   the header file of foc

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

#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>

/* constant definition */
#define ONE_DIVIDE_SQRT3                        (0.5773502f)                          /* the value of 1/sqrt(3) */
#define TWO_DIVIDE_SQRT3                        (1.1547005f)                          /* the value of 2/sqrt(3) */
#define SQRT6_DIVIDE_2                          (1.2247448f)                          /* the value of sqrt(6)/2 */
#define SQRT2_DIVIDE_2                          (0.7071067f)                          /* the value of sqrt(6)/2 */
#define SQRT2                                   (1.4142135f)                          /* the value of sqrt(6)/2 */
#define SQRT3_DIVIDE_2                          (0.8660254f)                          /* the value of sqrt(3)/2 */
#define SQRT6_DIVIDE_3                          (0.8164965f)                          /* the value of sqrt(3)/2 */
#define SQRT6_DIVIDE_6                          (0.4082482f)                          /* the value of sqrt(3)/2 */

#define MOTOR_PI                                (3.1415926f)                          /* the value of pi */
#define MOTOR_PI_BY_2                           (1.5707963f)                          /* the value of 2*pi */
#define MOTOR_PI_3_BY_2                         (4.7123889f)                          /* the value of 2*pi */
#define MOTOR_PI_2                              (6.2831853f)                          /* the value of 2*pi */

#define SIN_TABLE_SIZE                      131
#define ATAN_TABLE_SIZE                     102

/* trigonometric function srtuct definition */
/* trigonometric function srtuct */
typedef struct{
    float sin;      /*sin of phase*/
    float cos;      /*cos of phase*/
}triangle_struct;

/* pid srtuct definition */
/* the structure of pid parameter */
typedef struct{
    float kp_def;                           /* the proportional factor */
    float ki_def;                           /* the integral factor */
    float kd_def;                           /* the differential factor */
    float integral_sum_upper_limit_def;     /* the upper limit of integral term */
    float integral_sum_lower_limit_def;     /* the lower limit of integral term */
    float output_upper_limit_def;           /* the upper limit of ouput */
    float output_lower_limit_def;           /* the lower limit of ouput */
    float kp;                               /* the proportional factor */
    float ki;                               /* the integral factor */
    float kd;                               /* the differential factor */
    float integral_sum_upper_limit;         /* the upper limit of integral term */
    float integral_sum_lower_limit;         /* the lower limit of integral term */
    float output_upper_limit;               /* the upper limit of ouput */
    float output_lower_limit;               /* the lower limit of ouput */
    float integral_sum;                     /* the sum of the integral term */
    float error;                            /* the error of the input */
    float output;                           /* the output varialbe */
}pid_struct;

/* the structure of s_curve parameter */
typedef struct{
    float jerk;
    float fs;
    float start_reference;
    float end_reference;
    float output;
    int16_t reference_previous;
    int16_t reference_now;
    uint16_t cnt;
    uint8_t cnt_flag;
    uint8_t start_plan_flag;
    uint8_t plan_flag;
    uint8_t plan_add_flag;
    uint8_t plan_decrease_flag;
    
    float temp1; /*temp1 = jerk/2*/
    float temp2; /*temp2 = jerk/2*t1*/
    float temp3; /*temp3 = temp2*t1 = jerk/2*t1*t1*/
    float temp4; /*temp4 = temp2*t1 = jerk*t1*t2*/
    float temp5; /*temp4 = temp2*t1 = jerk*t1*t2*/
    
    float t1;
    float t2;
    float t;
}s_curve_struct;

/* function declaration */
/* initialize pid structure */
void pid_init(pid_struct* pid);
/* an incremental pid control algorithm */
float pid_incremental_regulation(float ref, float back, pid_struct *pid_para);

/* store sample value to the specified array */
void utils_add_sample(int16_t *buffer, int16_t sample, uint8_t length_bit, uint8_t *index);
/* get the average value of all the data in the array */
int16_t utils_average_sample(int16_t *buffer, uint8_t length_bit);

/* equal amplitude clarke transformation */
void clarke_amplitude(float *alpha, float *beta, float a, float b);
/* constant power clarke transformation */
void clarke_power(float *alpha, float *beta, float a, float b);
/* reverse equal amplitude clarke transformation */
void rev_clarke_amplitude(float *a, float *b, float alpha, float beta);
/* reverse constant power clarke transformation */
void rev_clarke_power(float *a, float *b, float alpha, float beta);
/* park transformation */
void park(float *q, float *d, float alpha, float beta, triangle_struct phase);
/* reverse park transformation */
void rev_park(float *alpha, float *beta, float q, float d, triangle_struct phase);

/* get the linear mapping fixed value from one space to another */
int16_t map_fixed(int16_t input_data, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
/* get the linear mapping float value from one space to another */
float map_float(float input_data, float in_min, float in_max, float out_min, float out_max);
/* get the average fixed value of all the data in the array */
int16_t moving_average_fixed(int16_t *buffer,uint8_t length,int32_t *sum, int16_t input_data, uint8_t *index);
/* get the average float value of all the data in the array */
float moving_average_float(float *buffer,uint8_t length,float *sum, float input_data, uint8_t *index);
/* get the mean square error (MSE) of fixed average value */
uint16_t mean_square_error_fixed(int16_t *buffer,uint8_t length, int16_t average_value);
/* get the mean square error (MSE) of float average value */
float mean_square_error_float(float *buffer,uint8_t length, float average_value);
/* give the source fixed data a step operation which makes it towards the destination fixed data */
int16_t step_towards_fixed(int16_t value, int16_t goal, int16_t step);
/* give the source float data a step operation which makes it towards the destination float data */
float step_towards_float(float value, float goal, float step);
/* nomalize the input angle */
float angle_nomalize_float(float angle);

/* filter the input_data */
float iir_filter_float(float *b, float *a,float *input_buffer,float *output_buffer,float input_data,uint8_t filter_length);
/* get the sqrt value of the float data */
float sqrt_float( float input_data);
/* floating-point sin function */
float sin_float(float input_data);
/* floating-point cosine function */
float cos_float(float input_data);
/* floating-point sin and cosine function */
void sin_cos_float(float input_data,float *sin_value,float *cos_value);
/* return radian value (0~2PI) of atan2 */
float atan2_float(float input_data_y,float input_data_x);
/* s curve plan */
float s_curve_plan_float(s_curve_struct *sigmoid_para, float reference);
#define UTILS_LP_FAST(value, sample, filter_constant)   (value -= (filter_constant) * (value - (sample)))
#endif /* FOC_MATH_H */
