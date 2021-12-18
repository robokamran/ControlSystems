/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
    double A0;       /* Derived gain, A0 = Kp + Ki + Kd */
    double A1;       /* Derived gain, A1 = -Kp - 2Kd */
    double A2;       /* Derived gain, A2 = Kd */
    double state[3]; /* State array of length 3 */
    double Kp;       /* Proportional gain */
    double Ki;       /* Integral gain */
    double Kd;       /* Derivative gain */
} PID;

/* Exported functions --------------------------------------------------------*/
static inline void pid_reset(PID *s)
{
    /* Clear the state buffer. The size will be always 3 samples */
    memset(s->state, 0, 3u * sizeof(double));
}

static inline void pid_initialize(PID *s, bool reset)
{
    /* Derived coefficient A0 */
    s->A0 = s->Kp + s->Ki + s->Kd;

    /* Derived coefficient A1 */
    s->A1 = (-s->Kp) - (2.0f * s->Kd);

    /* Derived coefficient A2 */
    s->A2 = s->Kd;

    /* Check whether state needs reset or not */
    if (reset)
        pid_reset(s);
}

static inline double pid_execute(PID *s, double e)
{
    double c;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2] */
    c = (s->A0 * e) + (s->A1 * s->state[0]) + (s->A2 * s->state[1]) + (s->state[2]);

    /* Update state */
    s->state[1] = s->state[0];
    s->state[0] = e;
    s->state[2] = c;

    /* return to application */
    return c;
}
