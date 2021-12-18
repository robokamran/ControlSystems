/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTERS_H
#define __FILTERS_H

/* Exported typedef ----------------------------------------------------------*/
/* Second Order transfer function filter */
typedef struct
{
	float a;		  /* Derived coefficient, a */
	float b;		  /* Derived coefficient, b */
	float c;		  /* Derived coefficient, c */
	float d;		  /* Derived coefficient, d */
	float e;		  /* Derived coefficient, e */
	float f;		  /* Derived coefficient, f */
	float ts;		  /* Sampling frequency */
	float zeta;		  /* Damping ratio */
	float wn;		  /* Undamped natural frequency */
	float u_state[2]; /* Input state array of length 2 */
	float y_state[2]; /* Response state array of length 2 */
} FILTER;

/* First Order Lead/Lag filter */
typedef struct
{
	float alpha;	  /* Signal ratio */
	float a, b, c, d; /* Derived coefficients */
	float ts;		  /* Sampling frequency */
	float w;		  /* Undamped natural frequency */
	float state[2];	  /* State array */
} LeadLag;

/* Exported functions --------------------------------------------------------*/
static inline void filter_reset(FILTER *s)
{
	/* Clear the state buffers. The size will be always 2 samples */
	memset(s->u_state, 0, 2u * sizeof(float));
	memset(s->y_state, 0, 2u * sizeof(float));
}

/* Low-ass derivative */
static inline void filter_lpd_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = 4.0f + t2w2 + 4.0f * ztw;
	s->a = (2.0f * s->ts * s->wn * s->wn) / s->d;
	s->b = 0;
	s->c = (-2.0f * s->ts * s->wn * s->wn) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

/* High-pass filter */
static inline void filter_hpf_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = 4.0f + t2w2 + 4.0f * ztw;
	s->a = (4) / s->d;
	s->b = (-8) / s->d;
	s->c = (4) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

/* High-band-pass filter => 1 - hpf - bpf = lpf */
static inline void filter_hpf_bpf_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = 4.0f + t2w2 + 4.0f * ztw;
	s->a = (4.0f * ztw + 4.0f) / s->d;
	s->b = (-8.0f) / s->d;
	s->c = (4.0f - 4.0f * ztw) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

/* Low-pass filter */
static inline void filter_lpf_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = 4.0f + t2w2 + 4.0f * ztw;
	s->a = (t2w2) / s->d;
	s->b = (2.0f * t2w2) / s->d;
	s->c = (t2w2) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

/* Band-pass filter */
static inline void filter_bpf_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = 4.0f + t2w2 + 4.0f * ztw;
	s->a = (4.0f * ztw) / s->d;
	s->b = 0;
	s->c = (-4.0f * ztw) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

/* Band-stop filter */
static inline void filter_bsf_init(FILTER *s, bool reset)
{
	/* Derive the coefficients */
	float t2w2 = s->ts * s->ts * s->wn * s->wn;
	float ztw = s->zeta * s->ts * s->wn;
	s->d = t2w2 + 4.0f * ztw + 4.0f;
	s->a = (t2w2 + 4.0f) / s->d;
	s->b = (2.0f * t2w2 - 8.0f) / s->d;
	s->c = (t2w2 + 4.0f) / s->d;
	s->e = (2.0f * t2w2 - 8.0f) / s->d;
	s->f = (t2w2 - 4.0f * ztw + 4.0f) / s->d;

	/* Check whether state needs reset or not */
	if (reset)
		filter_reset(s);
}

static inline float filter_execute(FILTER *s, float u)
{
	float y;

	/* y[k] = a * u[k] + b * u[k-1] + c * u[k-2]
	 *      - e * y[k-1] - f * y[k-2] */
	y = (s->a * u) + (s->b * s->u_state[0]) + (s->c * s->u_state[1]) - (s->e * s->y_state[0]) - (s->f * s->y_state[1]);

	/* Update state */
	s->y_state[1] = s->y_state[0];
	s->y_state[0] = y;
	s->u_state[1] = s->u_state[0];
	s->u_state[0] = u;

	return y;
}

static inline void leadlag_reset(LeadLag *s)
{
	/* Clear the state buffers. The size will be always 1 samples */
	memset(s->state, 0, 2u * sizeof(float));
}

static inline void leadlag_init(LeadLag *s, bool reset)
{
	/* Derive the coefficients */
	float tsw = s->ts * s->w;
	s->c = (tsw + 2.0f);
	s->d = (tsw - 2.0f) / s->c;
	s->a = (tsw + 2.0f * s->alpha) / s->c;
	s->b = (tsw - 2.0f * s->alpha) / s->c;

	/* Check whether state needs reset or not */
	if (reset)
		leadlag_reset(s);
}

static inline float leadlag_execute(LeadLag *s, float u)
{
	float y;

	/* y[k] = a * u[k] + b * u[k-1] - c * y[k-1] */
	y = (s->a * u) + (s->b * s->state[0]) - (s->d * s->state[1]);

	/* Update state */
	s->state[0] = u;
	s->state[1] = y;

	return y;
}

#endif // __FILTERS_H
