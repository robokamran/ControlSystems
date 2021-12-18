/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FGS_H
#define __FGS_H

/* Includes ------------------------------------------------------------------*/
#include "math.h"

/* Private define ------------------------------------------------------------*/
#define B micro_big
#define S micro_small

/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
    float Kp_max;    /* Maximum normalizing range for Kp */
    float Kp_min;    /* Minimum normalizing range for Kp */
    float Kp_diff;   /* Differencial value between min and max ranges of Kp */
    float Kd_max;    /* Maximum normalizing range for Kd */
    float Kd_min;    /* Minimum normalizing range for Kd */
    float Kd_diff;   /* Differencial value between min and max ranges of Kd */
    float Ku;        /* the gain of oscillation at the stability
                        limit under P-control */
    float Tu;        /* the period of oscillation at the stability
                        limit under P-control */
    float err_max;   /* The maximum value of error */
    float d_err_max; /* The maximum value of error rate */
    float state[2];  /* The state array of length 2 */
} FGS;

/* Membership functions type */
typedef float (*FunctionPointer)(float);

/* Private functions ---------------------------------------------------------*/
static inline float membership(float x, char channel, float boundary)
{
    float y = -fabsf((x / boundary * 3.0f - channel)) + 1.0f;
    if (y < 0.0f)
        y = 0;
    if (fabsf(x) > boundary &&
        fabsf(channel) == 3 &&
        x * channel > 0)
        y = 1;
    return y;
}

static inline float micro_small(float micro)
{
    return expf(-4 * micro);
    // return (1 - micro);
}

static inline float micro_big(float micro)
{
    return (1 - expf(-4 * micro));
    // return micro;
}

/* Private variables ---------------------------------------------------------*/
char alpha_Rules[7][7] =
    {
        {2, 2, 2, 2, 2, 2, 2},
        {3, 3, 2, 2, 2, 3, 3},
        {4, 3, 3, 2, 3, 3, 4},
        {5, 4, 3, 3, 3, 4, 5},
        {4, 3, 3, 2, 3, 3, 4},
        {3, 3, 2, 2, 2, 3, 3},
        {2, 2, 2, 2, 2, 2, 2}};

FunctionPointer Kpp_Rules[7][7] =
    {
        {B, B, B, B, B, B, B},
        {S, B, B, B, B, B, S},
        {S, S, B, B, B, S, S},
        {S, S, S, B, S, S, S},
        {S, S, B, B, B, S, S},
        {S, B, B, B, B, B, S},
        {B, B, B, B, B, B, B}};

FunctionPointer Kpd_Rules[7][7] =
    {
        {S, S, S, S, S, S, S},
        {B, B, S, S, S, B, B},
        {B, B, B, S, B, B, B},
        {B, B, B, B, B, B, B},
        {B, B, B, S, B, B, B},
        {B, B, S, S, S, B, B},
        {S, S, S, S, S, S, S}};

/* Exported functions --------------------------------------------------------*/
void fgs_reset(FGS *s)
{
    /* Clear the state buffer. The size will be always 2 samples */
    memset(s->state, 0, 2u * sizeof(float));
}

void fgs_initialize(FGS *s, bool reset)
{
    /* Gain clculations */
    s->Kp_min = 0.32f * s->Ku;
    s->Kp_max = 0.60f * s->Ku;
    s->Kd_min = 0.08f * s->Ku * s->Tu;
    s->Kd_max = 0.15f * s->Ku * s->Tu;
    s->Kp_diff = s->Kp_max - s->Kp_min;
    s->Kd_diff = s->Kd_max - s->Kd_min;

    if (reset)
        fgs_reset(s);
}

static inline float fgs_execute(FGS *s, float err)
{
    float kpp = 0, kpd = 0, alpha = 0;
    float kp, ki, kd;
    float emf[7] = {0}, dmf[7] = {0};
    float d_err = err - s->state[0];
    float micro, output;

    /* Fuzzification and membership functions calculation */
    for (int channel = -3; channel <= 3; channel++)
    {
        emf[channel + 3] = membership(err, channel, s->err_max);
        dmf[channel + 3] = membership(d_err, channel, s->d_err_max);
    }

    /* Applying fuzzy operator, implication, aggregation and defuzzification */
    for (int i = 0; i < 7; i++)
    {
        for (int j = 0; j < 7; j++)
        {
            micro = emf[i] * dmf[j];
            kpp += micro * Kpp_Rules[i][j](micro);
            kpd += micro * Kpd_Rules[i][j](micro);
            alpha += micro * alpha_Rules[i][j];
        }
    }

    /* PID calculation */
    kp = (s->Kp_diff) * kpp + s->Kp_min;
    kd = (s->Kd_diff) * kpd + s->Kd_min;
    ki = (kp * kp) / (alpha * kd);
    s->state[0] = err;  // Last error value
    s->state[1] += err; // Integral of errors
    output = kp * err + kd * d_err + ki * s->state[1];

    return output;
}

#endif /* __FGS_H */
