#include <math.h>

typedef struct
{
    float ACC, DCC, 
            J1,J2,J3,J4,
            AccFlag,
            Fout,Jout,Aout;
} vector;
vector VectorP;
float LoopTime;
    

void SCurveCalcFout(float Fref)
{
	/* (100Hz - 0Hz) / (AccTime * 0.1f - 0s) */
	VectorP.ACC = 10;
	VectorP.DCC = 10;

	VectorP.J1 = 0.5f;
	VectorP.J2 = -0.5f;
	VectorP.J3 = -0.5f;
	VectorP.J4 = 0.5f;

	if (VectorP.Fout < Fref)
		VectorP.Jout = VectorP.J1;
	else
		VectorP.Jout = VectorP.J3;

	if (VectorP.Fout < Fref) {
		VectorP.AccFlag = 1;
		if (VectorP.Aout >= 0) {
			float dv = fabsf(VectorP.Aout * VectorP.Aout / (2.0f * VectorP.J2));
			if (VectorP.Fout < (Fref - dv)) {
				if (VectorP.Aout >= VectorP.ACC) {
					VectorP.Jout = 0;
					VectorP.Aout = VectorP.ACC;
				}
			}
			else {
				if (fabsf(Fref - VectorP.Fout) < 0.0001f) {
					VectorP.Jout = VectorP.J2;
				}
				else {
					VectorP.Jout = -fabsf(0.5f * VectorP.Aout * VectorP.Aout / (Fref - VectorP.Fout));
				}
			}
		}
		if (VectorP.Jout < VectorP.J2 * 1.1f) VectorP.Jout = VectorP.J2 * 1.1f;
		if (VectorP.Jout > VectorP.J1 * 1.1f) VectorP.Jout = VectorP.J1 * 1.1f;
	}

	if (VectorP.Fout > Fref) {
		VectorP.AccFlag = 2;
		if (VectorP.Aout <= 0) {
			float dv = (VectorP.Aout * VectorP.Aout / (2.0f * VectorP.J4));
			if (VectorP.Fout >= (Fref + dv)) {
				if (VectorP.Aout <= -VectorP.DCC) {
					VectorP.Jout = 0;
					VectorP.Aout = -VectorP.DCC;
				}
			}
			else {
				if (fabsf(Fref - VectorP.Fout) < 0.0001f) {
					VectorP.Jout = VectorP.J4;
				}
				else {
					VectorP.Jout = fabsf(0.5f * VectorP.Aout * VectorP.Aout / (Fref - VectorP.Fout));
				}
			}
		}
		if (VectorP.Jout < VectorP.J3 * 1.1f) VectorP.Jout = VectorP.J3 * 1.1f;
		if (VectorP.Jout > VectorP.J4 * 1.1f) VectorP.Jout = VectorP.J4 * 1.1f;
	}

	if ((fabsf(Fref - VectorP.Fout) < 0.01f) && fabsf((VectorP.Aout)) < 0.01f)
	{
		VectorP.AccFlag = 0;
		VectorP.Jout = 0;
		VectorP.Aout = 0;
		VectorP.Fout = Fref;
	}

	/* v = a * t, a = j * t */
	VectorP.Aout += VectorP.Jout * LoopTime;
	VectorP.Fout += VectorP.Aout * LoopTime;
}



float loop(float position, float speed, float ts, float t)
{
    LoopTime = ts;
    if(t<10)
        return 0;
    if(t<20)
        SCurveCalcFout(1.0f);
    else
        SCurveCalcFout(0.0f);
    return VectorP.Fout;
}
