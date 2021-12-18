/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADRC_H
#define __ADRC_H

/* Includes ------------------------------------------------------------------*/
#include "math.h"

/* Exported macro ------------------------------------------------------------*/
#define sat(v, b) (fabsf(v) > b ? copysignf(b, v) : v)
#define sign(x) (float)((x > 0) - (x < 0))

/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
	float ts;
	float b0, w0;
	float z1, z2, z3;
    float r1, r2, r3;
	float limit;
	float u;
} ADRC;

/* Exported functions --------------------------------------------------------*/
static inline float fal(float x, float a, float d)
{
    if (fabsf(x) < d)
        return x / powf(d, 1.0f - a);
    else
        return powf(fabsf(x), a) * sign(x);
}

static inline void adrc_reset(ADRC *s)
{
    s->z1 = s->z2 = s->z3 = 0;
    s->r1 = s->r2 = s->r3 = 0;
    s->u = 0;
}

static inline void adrc_eso_1(ADRC *s, float y)
{
    /* 1st order observer implementation */
	float e = y - s->z1;
	float l1 = 2.0f * s->w0;
	float l2 = s->w0 * s->w0;
    s->z1 += s->ts * (l1 * e + s->z2 + s->b0 * s->u);
    s->z2 += s->ts * (l2 * e);
}

static inline void adrc_ctrl_1(ADRC *s)
{
    /* 1st order controller implementation */
    float k1 = s->w0;
    float e1 = s->r1 - s->z1;
    float u0 = k1 * e1;
    s->u = (u0 - s->z2) / s->b0 + s->r2;
	s->u = sat(s->u, s->limit);
}

static inline float adrc_execute_1(ADRC *s, float y)
{
    /* Execute a 1st order ADRC */
	adrc_eso_1(s, y);
    adrc_ctrl_1(s);
    return s->u;
}

static inline void adrc_eso_2(ADRC *s, float y)
{
    /* 2nd order observer implementation */
    float e = y - s->z1;
    float l1 = 3.0f * s->w0;
    float l2 = 3.0f * s->w0 * s->w0;
    float l3 = s->w0 * s->w0 * s->w0;
    s->z1 += s->ts * (l1 * e + s->z2);
    s->z2 += s->ts * (l2 * e + s->z3 + s->b0 * s->u);
    s->z3 += s->ts * (l3 * e);
}

static inline void adrc_ctrl_2(ADRC *s)
{
    /* 2nd order controller implementation */
    float k1 = s->w0 * s->w0;
    float k2 = 2.0f * s->w0;
    float e1 = s->r1 - s->z1;
    float e2 = s->r2 - s->z2;
    float u0 = k1 * e1 + k2 * e2;
    s->u = (u0 - s->z3) / s->b0 + s->r3;
    s->u = sat(s->u, s->limit);
}

static inline float adrc_execute_2(ADRC *s, float y)
{
    /* Execute a 2nd order ADRC */
    adrc_eso_2(s, y);
    adrc_ctrl_2(s);
    return s->u;
}

#endif // __ADRC_H
