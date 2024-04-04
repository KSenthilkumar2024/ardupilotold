//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: arduAttCont_data.cpp
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

// Block parameters (default storage)
arduAttContModelClass::P arduAttContModelClass::rtP = {
  // Variable: ANG_PIT_D
  //  Referenced by: 'Derivative Gain' (':185:688')

  0.01F,

  // Variable: ANG_PIT_I
  //  Referenced by: 'Integral Gain' (':185:791')

  0.025F,

  // Variable: ANG_PIT_IMAX
  //  Referenced by: 'Integrator' (':185:843')

  0.5F,

  // Variable: ANG_PIT_P
  //  Referenced by: 'Proportional Gain' (':185:913')

  3.5F,

  // Variable: ANG_RLL_D
  //  Referenced by: 'Derivative Gain' (':184:688')

  0.001F,

  // Variable: ANG_RLL_I
  //  Referenced by: 'Integral Gain' (':184:791')

  0.025F,

  // Variable: ANG_RLL_IMAX
  //  Referenced by: 'Integrator' (':184:843')

  0.5F,

  // Variable: ANG_RLL_P
  //  Referenced by: 'Proportional Gain' (':184:913')

  3.5F,

  // Variable: ANG_YAW_D
  //  Referenced by: 'Derivative Gain' (':186:688')

  0.0F,

  // Variable: ANG_YAW_I
  //  Referenced by: 'Integral Gain' (':186:791')

  0.0F,

  // Variable: ANG_YAW_IMAX
  //  Referenced by: 'Integrator' (':186:843')

  0.5F,

  // Variable: ANG_YAW_P
  //  Referenced by: 'Proportional Gain' (':186:913')

  4.5F,

  // Variable: RAT_PIT_D
  //  Referenced by: 'Derivative Gain' (':165:688')

  0.001F,

  // Variable: RAT_PIT_I
  //  Referenced by: 'Integral Gain' (':165:791')

  0.625F,

  // Variable: RAT_PIT_IMAX
  //  Referenced by: 'Integrator' (':165:843')

  0.5F,

  // Variable: RAT_PIT_P
  //  Referenced by: 'Proportional Gain' (':165:913')

  0.625F,

  // Variable: RAT_RLL_D
  //  Referenced by: 'Derivative Gain' (':166:688')

  0.001F,

  // Variable: RAT_RLL_I
  //  Referenced by: 'Integral Gain' (':166:791')

  0.325F,

  // Variable: RAT_RLL_IMAX
  //  Referenced by: 'Integrator' (':166:843')

  0.5F,

  // Variable: RAT_RLL_P
  //  Referenced by: 'Proportional Gain' (':166:913')

  0.325F,

  // Variable: RAT_YAW_D
  //  Referenced by: 'Derivative Gain' (':173:688')

  0.001F,

  // Variable: RAT_YAW_I
  //  Referenced by: 'Integral Gain' (':173:791')

  0.02F,

  // Variable: RAT_YAW_IMAX
  //  Referenced by: 'Integrator' (':173:843')

  0.5F,

  // Variable: RAT_YAW_P
  //  Referenced by: 'Proportional Gain' (':173:913')

  0.25F,

  // Mask Parameter: ANGLE_rll_pid_InitialConditionF
  //  Referenced by: 'Filter' (':184:737')

  0.0F,

  // Mask Parameter: RollPID_InitialConditionForFilt
  //  Referenced by: 'Filter' (':166:737')

  0.0F,

  // Mask Parameter: angle_pit_PID_InitialConditionF
  //  Referenced by: 'Filter' (':185:737')

  0.0F,

  // Mask Parameter: PitchPID_InitialConditionForFil
  //  Referenced by: 'Filter' (':165:737')

  0.0F,

  // Mask Parameter: Angle_YAW_PID_InitialConditionF
  //  Referenced by: 'Filter' (':186:737')

  0.0F,

  // Mask Parameter: YawPID_InitialConditionForFilte
  //  Referenced by: 'Filter' (':173:737')

  0.0F,

  // Mask Parameter: ANGLE_rll_pid_InitialConditio_d
  //  Referenced by: 'Integrator' (':184:843')

  0.0F,

  // Mask Parameter: RollPID_InitialConditionForInte
  //  Referenced by: 'Integrator' (':166:843')

  0.0F,

  // Mask Parameter: angle_pit_PID_InitialConditio_g
  //  Referenced by: 'Integrator' (':185:843')

  0.0F,

  // Mask Parameter: PitchPID_InitialConditionForInt
  //  Referenced by: 'Integrator' (':165:843')

  0.0F,

  // Mask Parameter: Angle_YAW_PID_InitialConditio_i
  //  Referenced by: 'Integrator' (':186:843')

  0.0F,

  // Mask Parameter: YawPID_InitialConditionForInteg
  //  Referenced by: 'Integrator' (':173:843')

  0.0F,

  // Mask Parameter: ANGLE_rll_pid_LowerIntegratorSa
  //  Referenced by: 'Integrator' (':184:843')

  -0.5F,

  // Mask Parameter: RollPID_LowerIntegratorSaturati
  //  Referenced by: 'Integrator' (':166:843')

  -0.5F,

  // Mask Parameter: angle_pit_PID_LowerIntegratorSa
  //  Referenced by: 'Integrator' (':185:843')

  -0.5F,

  // Mask Parameter: PitchPID_LowerIntegratorSaturat
  //  Referenced by: 'Integrator' (':165:843')

  -0.5F,

  // Mask Parameter: Angle_YAW_PID_LowerIntegratorSa
  //  Referenced by: 'Integrator' (':186:843')

  -0.5F,

  // Mask Parameter: YawPID_LowerIntegratorSaturatio
  //  Referenced by: 'Integrator' (':173:843')

  -0.5F,

  // Mask Parameter: ANGLE_rll_pid_N
  //  Referenced by: 'Filter Coefficient' (':184:882')

  100.0F,

  // Mask Parameter: RollPID_N
  //  Referenced by: 'Filter Coefficient' (':166:882')

  100.0F,

  // Mask Parameter: angle_pit_PID_N
  //  Referenced by: 'Filter Coefficient' (':185:882')

  100.0F,

  // Mask Parameter: PitchPID_N
  //  Referenced by: 'Filter Coefficient' (':165:882')

  100.0F,

  // Mask Parameter: Angle_YAW_PID_N
  //  Referenced by: 'Filter Coefficient' (':186:882')

  100.0F,

  // Mask Parameter: YawPID_N
  //  Referenced by: 'Filter Coefficient' (':173:882')

  100.0F,

  // Computed Parameter: Integrator_gainval
  //  Referenced by: 'Integrator' (':184:843')

  0.0025F,

  // Computed Parameter: Filter_gainval
  //  Referenced by: 'Filter' (':184:737')

  0.0025F,

  // Computed Parameter: Integrator_gainval_o
  //  Referenced by: 'Integrator' (':166:843')

  0.0025F,

  // Computed Parameter: Filter_gainval_d
  //  Referenced by: 'Filter' (':166:737')

  0.0025F,

  // Computed Parameter: Integrator_gainval_d
  //  Referenced by: 'Integrator' (':185:843')

  0.0025F,

  // Computed Parameter: Filter_gainval_k
  //  Referenced by: 'Filter' (':185:737')

  0.0025F,

  // Computed Parameter: Integrator_gainval_m
  //  Referenced by: 'Integrator' (':165:843')

  0.0025F,

  // Computed Parameter: Filter_gainval_f
  //  Referenced by: 'Filter' (':165:737')

  0.0025F,

  // Computed Parameter: Integrator_gainval_i
  //  Referenced by: 'Integrator' (':186:843')

  0.0025F,

  // Computed Parameter: Filter_gainval_h
  //  Referenced by: 'Filter' (':186:737')

  0.0025F,

  // Computed Parameter: Integrator_gainval_h
  //  Referenced by: 'Integrator' (':173:843')

  0.0025F,

  // Computed Parameter: Filter_gainval_di
  //  Referenced by: 'Filter' (':173:737')

  0.0025F
};

//
// File trailer for generated code.
//
// [EOF]
//
