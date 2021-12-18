/* Private define ------------------------------------------------------------*/
#define S_FUNCTION_NAME controller
#define REPORT_LENGTH 3

/* Includes ------------------------------------------------------------------*/
#include "prototype.h"
#include "adrc.h"

/* Private variables ---------------------------------------------------------*/
ADRC adrc = { 0 };

/* Private functions ---------------------------------------------------------*/
void adrc_setup(ADRC *s, float ts)
{
    /* Note that even though control signal saturation exists,
     * the controller shows great performance in handling the system */
	s->b0 = 1.0f / 2.0f; /* 1.0f / Inertia */
    s->w0 = 40.0f;
	s->limit = 3.0f;
	s->ts = ts;
}

real_T loop(real_T *ref, real_T *feed, time_T ts, time_T t, real_T *rep)
{
    /* Initialize controller parameters */
    adrc_setup(&adrc, ts);
    
    /* Report back ESO states */
    rep[1] = adrc.z1;
    rep[2] = adrc.z2;
    rep[3] = adrc.z3;
    
    /* Select either 1st or 2nd order controller */
    if (false)
    {
        adrc.r1 = ref[1];
        adrc.r2 = ref[2];
        return adrc_execute_1(&adrc, feed[1]);
    }
    else
    {
        adrc.r1 = ref[0];
        adrc.r2 = ref[1];
        adrc.r3 = ref[2];
        return adrc_execute_2(&adrc, feed[0]);
    }
}
