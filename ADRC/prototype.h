#define S_FUNCTION_LEVEL 2

#define BASE_SAMPLE_TIME(S) ssGetSFcnParam(S, 0)

#include "simstruc.h"
#include "stdbool.h"

static void mdlInitializeSizes(SimStruct *S)
{
    /* Set and check the numper of input s-function parameters */
    ssSetNumSFcnParams(S, 1);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
        return;

    /* Set the number of s-function inputs */
    ssSetNumInputPorts(S, 2);    
    for (int i = 0; i < ssGetNumInputPorts(S); i++)
    {    
        ssSetInputPortWidth(S, i, DYNAMICALLY_SIZED);
        ssSetInputPortDirectFeedThrough(S, i, true);
        ssSetInputPortRequiredContiguous(S, i, true);
    }

    /* Configure s-function outputs */
    ssSetNumOutputPorts(S, 2);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, REPORT_LENGTH);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    /* Set the s-function sample time from the first input parameter */
    time_T Ts = mxGetPr(BASE_SAMPLE_TIME(S))[0];
    ssSetSampleTime(S, 0, Ts);
}

/* Main controller loop prototype */
real_T loop(real_T *, real_T *, time_T, time_T, real_T *);

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    time_T t = ssGetT(S);
    time_T Ts = mxGetPr(BASE_SAMPLE_TIME(S))[0];
    real_T *reference = (real_T *)ssGetInputPortRealSignal(S, 0);
    real_T *feedback = (real_T *)ssGetInputPortRealSignal(S, 1);
    real_T *report = ssGetOutputPortRealSignal(S, 0);
    real_T output = loop(reference, feedback, Ts, t, report);
    ssGetOutputPortRealSignal(S, 0)[0] = output;
}

static void mdlTerminate(SimStruct *S) {}

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port,
                                         const DimsInfo_T *dimsInfo)
{
    ssSetInputPortDimensionInfo(S, port, dimsInfo);
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port,
                                          const DimsInfo_T *dimsInfo)
{
    ssSetOutputPortDimensionInfo(S, port, dimsInfo);
}

#include "simulink.c"
