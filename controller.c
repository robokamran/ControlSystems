/* Private define ------------------------------------------------------------*/
#define S_FUNCTION_NAME controller
#define REPORT_LENGTH 1

/* Includes ------------------------------------------------------------------*/
#include "prototype.h"
#include "stdbool.h"
#include "filters.h"
#include "pid.h"
#include "mrac.h"
#include "fgs.h"

/* Private variables ---------------------------------------------------------*/
PID pid = {0};
MRAC mrac = {.theta = 1};
FGS fgs = {0};

/* Private functions ---------------------------------------------------------*/
void pid_setup(PID *s, float ts)
{
    float Ku = 1, Tu = 1.0f / ts;
    /* Tuning based on Ziegler�Nichols method */
    s->Kp = 0.6f * Ku;
    s->Ki = 1.2f * Ku / Tu;
    s->Kd = 0.075f * Ku * Tu;
    pid_initialize(s, false);
}

void fgs_setup(FGS *s, float ts)
{
    float Ku = 1.05f, Tu = 1.32f / ts;
    s->err_max = 2.0f;
    s->d_err_max = 2.0f;
    s->Ku = Ku;
    s->Tu = Tu;
    fgs_initialize(&fgs, false);
}

void mrac_setup(MRAC *s, float ts)
{
    s->pid.Kp = 1;
    s->pid.Ki = 1 * ts;
    s->pid.Kd = 0;
    s->model.ts = ts;
    s->model.zeta = 1;
    s->model.wn = 3;
    s->gamma = 0.1f;
    mrac_initialize(s, false);
}

real_T loop(real_T *ref, real_T *feed, time_T ts, time_T t, real_T *rep)
{
    pid_setup(&pid, ts);
    mrac_setup(&mrac, ts);
    fgs_setup(&fgs, ts);

    int selector = 2;
    switch (selector)
    {
    case 1:
        return pid_execute(&pid, ref[0] - feed[0]);
    case 2:
        return mrac_execute(&mrac, ref[0], feed[0]);
    case 3:
        return fgs_execute(&fgs, ref[0] - feed[0]);
    case 4:
        return filter_execute(&mrac.model, ref[0]);
    default:
        return 0;
    }
}
