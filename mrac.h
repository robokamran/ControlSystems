/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
    float gamma;  /* Learning rate */
    float theta;  /* Adaptable parameter  */
    PID pid;      /* PID controller instance */
    FILTER model; /* Reference model */
} MRAC;

/* Exported functions --------------------------------------------------------*/
static inline void mrac_reset(MRAC *s)
{
    pid_reset(&s->pid);
    filter_reset(&s->model);

    /* Reset adaptive mechanism integrator with initial condition '1' */
    s->theta = 1;
}

static inline void mrac_initialize(MRAC *s, bool reset)
{
    pid_initialize(&s->pid, reset);
    filter_lpf_init(&s->model, reset);

    /* Check whether state needs reset or not */
    if (reset)
        mrac_reset(s);
}

static inline float mrac_execute(MRAC *s, float ref, float yp)
{
    float u, uc, ym;

    /* Ym = TF_model(ref) */
    ym = filter_execute(&s->model, ref);

    /* uc = TF_pid(e = ref - yp) */
    uc = pid_execute(&s->pid, ref - yp);

    /* D_theta = -gamma * Ym * (e = Yp - Ym) */
    s->theta += -s->gamma * ym * (yp - ym);

    /* Feedforward gain adaptaion */
    u = s->theta * uc;

    /* return to application */
    return u;
}
