/*
 * S-Function para concatenar un array N veces
 * 
 * repmat_sfun.c
 * 
 * Entradas:
 *   Puerto 0: Array de entrada x (vector)
 *   Parámetro: N (número de repeticiones)
 * 
 * Salida:
 *   Puerto 0: Array concatenado N veces
 * 
 * Ejemplo:
 *   x = [7 5 6], N = 3
 *   y = [7 5 6 7 5 6 7 5 6]
 */

#define S_FUNCTION_NAME  repmat_sfun
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <string.h>

/* Parámetro: N (número de repeticiones) */
#define N_PARAM ssGetSFcnParam(S, 0)

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Configurar tamaños de puertos y parámetros
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  /* Número de parámetros esperados */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Error en número de parámetros */
    }

    /* No se permiten parámetros ajustables */
    ssSetSFcnParamTunable(S, 0, SS_PRM_NOT_TUNABLE);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* Puerto de entrada */
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
    ssSetInputPortRequiredContiguous(S, 0, 1); /* Direct input signal access */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    /* Puerto de salida */
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Especificar tiempo de muestreo heredado
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_OUTPUT_PORT_WIDTH
/* Function: mdlSetOutputPortWidth ============================================
 * Abstract:
 *    Configurar ancho del puerto de salida basado en entrada y parámetro N
 */
static void mdlSetOutputPortWidth(SimStruct *S, int_T port, int_T width)
{
    int_T N = (int_T)mxGetScalar(N_PARAM);
    int_T inWidth = ssGetInputPortWidth(S, 0);
    
    if (port == 0) {
        ssSetOutputPortWidth(S, 0, inWidth * N);
    }
}

#define MDL_SET_INPUT_PORT_WIDTH
/* Function: mdlSetInputPortWidth =============================================
 * Abstract:
 *    Propagar ancho del puerto de entrada
 */
static void mdlSetInputPortWidth(SimStruct *S, int_T port, int_T width)
{
    int_T N = (int_T)mxGetScalar(N_PARAM);
    
    if (port == 0) {
        ssSetInputPortWidth(S, 0, width);
        ssSetOutputPortWidth(S, 0, width * N);
    }
}

#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
/* Function: mdlSetDefaultPortDimensionInfo ===================================
 * Abstract:
 *    Configurar dimensiones por defecto
 */
static void mdlSetDefaultPortDimensionInfo(SimStruct *S)
{
    if (ssGetInputPortWidth(S, 0) == DYNAMICALLY_SIZED) {
        if (!ssSetInputPortMatrixDimensions(S, 0, 1, 1)) return;
    }
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Calcular la salida: concatenar el array de entrada N veces
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *u = (const real_T*)ssGetInputPortSignal(S, 0);
    real_T       *y = ssGetOutputPortSignal(S, 0);
    
    int_T inWidth = ssGetInputPortWidth(S, 0);
    int_T N = (int_T)mxGetScalar(N_PARAM);
    int_T i;
    
    /* Verificar que N sea positivo */
    if (N <= 0) {
        ssSetErrorStatus(S, "El parámetro N debe ser un entero positivo.");
        return;
    }
    
    /* Copiar el array de entrada N veces */
    for (i = 0; i < N; i++) {
        memcpy(y + i * inWidth, u, inWidth * sizeof(real_T));
    }
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    Limpieza al finalizar
 */
static void mdlTerminate(SimStruct *S)
{
    /* No hay memoria dinámica que liberar */
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif