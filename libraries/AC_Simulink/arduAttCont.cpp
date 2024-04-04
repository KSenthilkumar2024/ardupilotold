//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: arduAttCont.cpp
//
// Code generated for Simulink model 'arduAttCont'.
//
// Model version                  : 7.29
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Thu Apr  4 15:11:45 2024
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Emulation hardware selection:
//    Differs from embedded hardware (MATLAB Host)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "arduAttCont.h"
#include "rtwtypes.h"

// Model step function
void arduAttContModelClass::step()
{
  real32_T rtb_FilterCoefficient;
  real32_T rtb_FilterCoefficient_b;
  real32_T rtb_FilterCoefficient_ja;
  real32_T rtb_FilterCoefficient_n;
  real32_T rtb_FilterCoefficient_o;
  real32_T rtb_FilterCoefficient_p;
  real32_T rtb_Sum2;
  real32_T rtb_Sum4;
  real32_T rtb_Sum_n;

  // Gain: 'Filter Coefficient' (':184:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':184:737')
  //   Gain: 'Derivative Gain' (':184:688')
  //   Inport: 'attitude_error' (':151')
  //   Sum: 'SumD' (':184:738')

  rtb_FilterCoefficient = (rtP.ANG_RLL_D * rtU.attitude_error[0] -
    rtDW.Filter_DSTATE) * rtP.ANGLE_rll_pid_N;

  // Sum: 'Sum' (':167') incorporates:
  //   DiscreteIntegrator: 'Integrator' (':184:843')
  //   Gain: 'Proportional Gain' (':184:913')
  //   Inport: 'attitude_error' (':151')
  //   Inport: 'rate_ff' (':178')
  //   Inport: 'rate_meas' (':179')
  //   Sum: 'Sum1' (':168')
  //   Sum: 'Sum' (':184:3526')

  rtb_Sum_n = (((rtP.ANG_RLL_P * rtU.attitude_error[0] + rtDW.Integrator_DSTATE)
                + rtb_FilterCoefficient) + rtU.rate_ff[0]) - rtU.rate_meas[0];

  // Gain: 'Filter Coefficient' (':166:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':166:737')
  //   Gain: 'Derivative Gain' (':166:688')
  //   Sum: 'SumD' (':166:738')

  rtb_FilterCoefficient_o = (rtP.RAT_RLL_D * rtb_Sum_n - rtDW.Filter_DSTATE_c) *
    rtP.RollPID_N;

  // Gain: 'Filter Coefficient' (':185:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':185:737')
  //   Gain: 'Derivative Gain' (':185:688')
  //   Inport: 'attitude_error' (':151')
  //   Sum: 'SumD' (':185:738')

  rtb_FilterCoefficient_n = (rtP.ANG_PIT_D * rtU.attitude_error[1] -
    rtDW.Filter_DSTATE_l) * rtP.angle_pit_PID_N;

  // Sum: 'Sum2' (':169') incorporates:
  //   DiscreteIntegrator: 'Integrator' (':185:843')
  //   Gain: 'Proportional Gain' (':185:913')
  //   Inport: 'attitude_error' (':151')
  //   Inport: 'rate_ff' (':178')
  //   Inport: 'rate_meas' (':179')
  //   Sum: 'Sum3' (':170')
  //   Sum: 'Sum' (':185:3526')

  rtb_Sum2 = (((rtP.ANG_PIT_P * rtU.attitude_error[1] + rtDW.Integrator_DSTATE_h)
               + rtb_FilterCoefficient_n) + rtU.rate_ff[1]) - rtU.rate_meas[1];

  // Gain: 'Filter Coefficient' (':165:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':165:737')
  //   Gain: 'Derivative Gain' (':165:688')
  //   Sum: 'SumD' (':165:738')

  rtb_FilterCoefficient_b = (rtP.RAT_PIT_D * rtb_Sum2 - rtDW.Filter_DSTATE_k) *
    rtP.PitchPID_N;

  // Gain: 'Filter Coefficient' (':186:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':186:737')
  //   Gain: 'Derivative Gain' (':186:688')
  //   Inport: 'attitude_error' (':151')
  //   Sum: 'SumD' (':186:738')

  rtb_FilterCoefficient_ja = (rtP.ANG_YAW_D * rtU.attitude_error[2] -
    rtDW.Filter_DSTATE_m) * rtP.Angle_YAW_PID_N;

  // Sum: 'Sum4' (':171') incorporates:
  //   DiscreteIntegrator: 'Integrator' (':186:843')
  //   Gain: 'Proportional Gain' (':186:913')
  //   Inport: 'attitude_error' (':151')
  //   Inport: 'rate_ff' (':178')
  //   Inport: 'rate_meas' (':179')
  //   Sum: 'Sum5' (':172')
  //   Sum: 'Sum' (':186:3526')

  rtb_Sum4 = (((rtP.ANG_YAW_P * rtU.attitude_error[2] + rtDW.Integrator_DSTATE_g)
               + rtb_FilterCoefficient_ja) + rtU.rate_ff[2]) - rtU.rate_meas[2];

  // Gain: 'Filter Coefficient' (':173:882') incorporates:
  //   DiscreteIntegrator: 'Filter' (':173:737')
  //   Gain: 'Derivative Gain' (':173:688')
  //   Sum: 'SumD' (':173:738')

  rtb_FilterCoefficient_p = (rtP.RAT_YAW_D * rtb_Sum4 - rtDW.Filter_DSTATE_g) *
    rtP.YawPID_N;

  // Outport: 'Out1' (':150') incorporates:
  //   DiscreteIntegrator: 'Integrator' (':165:843')
  //   DiscreteIntegrator: 'Integrator' (':166:843')
  //   DiscreteIntegrator: 'Integrator' (':173:843')
  //   Gain: 'Proportional Gain' (':165:913')
  //   Gain: 'Proportional Gain' (':166:913')
  //   Gain: 'Proportional Gain' (':173:913')
  //   Sum: 'Sum' (':165:3526')
  //   Sum: 'Sum' (':166:3526')
  //   Sum: 'Sum' (':173:3526')

  rtY.Out1[0] = (rtP.RAT_RLL_P * rtb_Sum_n + rtDW.Integrator_DSTATE_e) +
    rtb_FilterCoefficient_o;
  rtY.Out1[1] = (rtP.RAT_PIT_P * rtb_Sum2 + rtDW.Integrator_DSTATE_b) +
    rtb_FilterCoefficient_b;
  rtY.Out1[2] = (rtP.RAT_YAW_P * rtb_Sum4 + rtDW.Integrator_DSTATE_i) +
    rtb_FilterCoefficient_p;

  // Update for DiscreteIntegrator: 'Integrator' (':184:843') incorporates:
  //   Gain: 'Integral Gain' (':184:791')
  //   Inport: 'attitude_error' (':151')

  rtDW.Integrator_DSTATE += rtP.ANG_RLL_I * rtU.attitude_error[0] *
    rtP.Integrator_gainval;
  if (rtDW.Integrator_DSTATE > rtP.ANG_RLL_IMAX) {
    rtDW.Integrator_DSTATE = rtP.ANG_RLL_IMAX;
  } else if (rtDW.Integrator_DSTATE < rtP.ANGLE_rll_pid_LowerIntegratorSa) {
    rtDW.Integrator_DSTATE = rtP.ANGLE_rll_pid_LowerIntegratorSa;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':184:843')

  // Update for DiscreteIntegrator: 'Filter' (':184:737')
  rtDW.Filter_DSTATE += rtP.Filter_gainval * rtb_FilterCoefficient;

  // Update for DiscreteIntegrator: 'Integrator' (':166:843') incorporates:
  //   Gain: 'Integral Gain' (':166:791')

  rtDW.Integrator_DSTATE_e += rtP.RAT_RLL_I * rtb_Sum_n *
    rtP.Integrator_gainval_o;
  if (rtDW.Integrator_DSTATE_e > rtP.RAT_RLL_IMAX) {
    rtDW.Integrator_DSTATE_e = rtP.RAT_RLL_IMAX;
  } else if (rtDW.Integrator_DSTATE_e < rtP.RollPID_LowerIntegratorSaturati) {
    rtDW.Integrator_DSTATE_e = rtP.RollPID_LowerIntegratorSaturati;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':166:843')

  // Update for DiscreteIntegrator: 'Filter' (':166:737')
  rtDW.Filter_DSTATE_c += rtP.Filter_gainval_d * rtb_FilterCoefficient_o;

  // Update for DiscreteIntegrator: 'Integrator' (':185:843') incorporates:
  //   Gain: 'Integral Gain' (':185:791')
  //   Inport: 'attitude_error' (':151')

  rtDW.Integrator_DSTATE_h += rtP.ANG_PIT_I * rtU.attitude_error[1] *
    rtP.Integrator_gainval_d;
  if (rtDW.Integrator_DSTATE_h > rtP.ANG_PIT_IMAX) {
    rtDW.Integrator_DSTATE_h = rtP.ANG_PIT_IMAX;
  } else if (rtDW.Integrator_DSTATE_h < rtP.angle_pit_PID_LowerIntegratorSa) {
    rtDW.Integrator_DSTATE_h = rtP.angle_pit_PID_LowerIntegratorSa;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':185:843')

  // Update for DiscreteIntegrator: 'Filter' (':185:737')
  rtDW.Filter_DSTATE_l += rtP.Filter_gainval_k * rtb_FilterCoefficient_n;

  // Update for DiscreteIntegrator: 'Integrator' (':165:843') incorporates:
  //   Gain: 'Integral Gain' (':165:791')

  rtDW.Integrator_DSTATE_b += rtP.RAT_PIT_I * rtb_Sum2 *
    rtP.Integrator_gainval_m;
  if (rtDW.Integrator_DSTATE_b > rtP.RAT_PIT_IMAX) {
    rtDW.Integrator_DSTATE_b = rtP.RAT_PIT_IMAX;
  } else if (rtDW.Integrator_DSTATE_b < rtP.PitchPID_LowerIntegratorSaturat) {
    rtDW.Integrator_DSTATE_b = rtP.PitchPID_LowerIntegratorSaturat;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':165:843')

  // Update for DiscreteIntegrator: 'Filter' (':165:737')
  rtDW.Filter_DSTATE_k += rtP.Filter_gainval_f * rtb_FilterCoefficient_b;

  // Update for DiscreteIntegrator: 'Integrator' (':186:843') incorporates:
  //   Gain: 'Integral Gain' (':186:791')
  //   Inport: 'attitude_error' (':151')

  rtDW.Integrator_DSTATE_g += rtP.ANG_YAW_I * rtU.attitude_error[2] *
    rtP.Integrator_gainval_i;
  if (rtDW.Integrator_DSTATE_g > rtP.ANG_YAW_IMAX) {
    rtDW.Integrator_DSTATE_g = rtP.ANG_YAW_IMAX;
  } else if (rtDW.Integrator_DSTATE_g < rtP.Angle_YAW_PID_LowerIntegratorSa) {
    rtDW.Integrator_DSTATE_g = rtP.Angle_YAW_PID_LowerIntegratorSa;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':186:843')

  // Update for DiscreteIntegrator: 'Filter' (':186:737')
  rtDW.Filter_DSTATE_m += rtP.Filter_gainval_h * rtb_FilterCoefficient_ja;

  // Update for DiscreteIntegrator: 'Integrator' (':173:843') incorporates:
  //   Gain: 'Integral Gain' (':173:791')

  rtDW.Integrator_DSTATE_i += rtP.RAT_YAW_I * rtb_Sum4 *
    rtP.Integrator_gainval_h;
  if (rtDW.Integrator_DSTATE_i > rtP.RAT_YAW_IMAX) {
    rtDW.Integrator_DSTATE_i = rtP.RAT_YAW_IMAX;
  } else if (rtDW.Integrator_DSTATE_i < rtP.YawPID_LowerIntegratorSaturatio) {
    rtDW.Integrator_DSTATE_i = rtP.YawPID_LowerIntegratorSaturatio;
  }

  // End of Update for DiscreteIntegrator: 'Integrator' (':173:843')

  // Update for DiscreteIntegrator: 'Filter' (':173:737')
  rtDW.Filter_DSTATE_g += rtP.Filter_gainval_di * rtb_FilterCoefficient_p;
}

// Model initialize function
void arduAttContModelClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':184:843')
  rtDW.Integrator_DSTATE = rtP.ANGLE_rll_pid_InitialConditio_d;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':184:737')
  rtDW.Filter_DSTATE = rtP.ANGLE_rll_pid_InitialConditionF;

  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':166:843')
  rtDW.Integrator_DSTATE_e = rtP.RollPID_InitialConditionForInte;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':166:737')
  rtDW.Filter_DSTATE_c = rtP.RollPID_InitialConditionForFilt;

  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':185:843')
  rtDW.Integrator_DSTATE_h = rtP.angle_pit_PID_InitialConditio_g;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':185:737')
  rtDW.Filter_DSTATE_l = rtP.angle_pit_PID_InitialConditionF;

  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':165:843')
  rtDW.Integrator_DSTATE_b = rtP.PitchPID_InitialConditionForInt;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':165:737')
  rtDW.Filter_DSTATE_k = rtP.PitchPID_InitialConditionForFil;

  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':186:843')
  rtDW.Integrator_DSTATE_g = rtP.Angle_YAW_PID_InitialConditio_i;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':186:737')
  rtDW.Filter_DSTATE_m = rtP.Angle_YAW_PID_InitialConditionF;

  // InitializeConditions for DiscreteIntegrator: 'Integrator' (':173:843')
  rtDW.Integrator_DSTATE_i = rtP.YawPID_InitialConditionForInteg;

  // InitializeConditions for DiscreteIntegrator: 'Filter' (':173:737')
  rtDW.Filter_DSTATE_g = rtP.YawPID_InitialConditionForFilte;
}

// Constructor
arduAttContModelClass::arduAttContModelClass():
  rtU(),
  rtY(),
  rtDW()
{
  // Currently there is no constructor body generated.
}

// Destructor
arduAttContModelClass::~arduAttContModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
