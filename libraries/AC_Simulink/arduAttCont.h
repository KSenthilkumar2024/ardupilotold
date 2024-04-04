//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: arduAttCont.h
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
#ifndef RTW_HEADER_arduAttCont_h_
#define RTW_HEADER_arduAttCont_h_
#include "rtwtypes.h"
#include <stddef.h>

// Class declaration for model arduAttCont
class arduAttContModelClass
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system 'arduAttCont'
  struct DW {
    real32_T Integrator_DSTATE;        // 'Integrator' (':184:843')
    real32_T Filter_DSTATE;            // 'Filter' (':184:737')
    real32_T Integrator_DSTATE_e;      // 'Integrator' (':166:843')
    real32_T Filter_DSTATE_c;          // 'Filter' (':166:737')
    real32_T Integrator_DSTATE_h;      // 'Integrator' (':185:843')
    real32_T Filter_DSTATE_l;          // 'Filter' (':185:737')
    real32_T Integrator_DSTATE_b;      // 'Integrator' (':165:843')
    real32_T Filter_DSTATE_k;          // 'Filter' (':165:737')
    real32_T Integrator_DSTATE_g;      // 'Integrator' (':186:843')
    real32_T Filter_DSTATE_m;          // 'Filter' (':186:737')
    real32_T Integrator_DSTATE_i;      // 'Integrator' (':173:843')
    real32_T Filter_DSTATE_g;          // 'Filter' (':173:737')
  };

  // External inputs (root inport signals with default storage)
  struct ExtU {
    real32_T attitude_error[3];        // 'attitude_error' (':151')
    real32_T rate_ff[3];               // 'rate_ff' (':178')
    real32_T rate_meas[3];             // 'rate_meas' (':179')
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY {
    real32_T Out1[3];                  // 'Out1' (':150')
  };

  // Parameters (default storage)
  struct P {
    real32_T ANG_PIT_D;                // Variable: ANG_PIT_D
                                          //  Referenced by: 'Derivative Gain' (':185:688')

    real32_T ANG_PIT_I;                // Variable: ANG_PIT_I
                                          //  Referenced by: 'Integral Gain' (':185:791')

    real32_T ANG_PIT_IMAX;             // Variable: ANG_PIT_IMAX
                                          //  Referenced by: 'Integrator' (':185:843')

    real32_T ANG_PIT_P;                // Variable: ANG_PIT_P
                                          //  Referenced by: 'Proportional Gain' (':185:913')

    real32_T ANG_RLL_D;                // Variable: ANG_RLL_D
                                          //  Referenced by: 'Derivative Gain' (':184:688')

    real32_T ANG_RLL_I;                // Variable: ANG_RLL_I
                                          //  Referenced by: 'Integral Gain' (':184:791')

    real32_T ANG_RLL_IMAX;             // Variable: ANG_RLL_IMAX
                                          //  Referenced by: 'Integrator' (':184:843')

    real32_T ANG_RLL_P;                // Variable: ANG_RLL_P
                                          //  Referenced by: 'Proportional Gain' (':184:913')

    real32_T ANG_YAW_D;                // Variable: ANG_YAW_D
                                          //  Referenced by: 'Derivative Gain' (':186:688')

    real32_T ANG_YAW_I;                // Variable: ANG_YAW_I
                                          //  Referenced by: 'Integral Gain' (':186:791')

    real32_T ANG_YAW_IMAX;             // Variable: ANG_YAW_IMAX
                                          //  Referenced by: 'Integrator' (':186:843')

    real32_T ANG_YAW_P;                // Variable: ANG_YAW_P
                                          //  Referenced by: 'Proportional Gain' (':186:913')

    real32_T RAT_PIT_D;                // Variable: RAT_PIT_D
                                          //  Referenced by: 'Derivative Gain' (':165:688')

    real32_T RAT_PIT_I;                // Variable: RAT_PIT_I
                                          //  Referenced by: 'Integral Gain' (':165:791')

    real32_T RAT_PIT_IMAX;             // Variable: RAT_PIT_IMAX
                                          //  Referenced by: 'Integrator' (':165:843')

    real32_T RAT_PIT_P;                // Variable: RAT_PIT_P
                                          //  Referenced by: 'Proportional Gain' (':165:913')

    real32_T RAT_RLL_D;                // Variable: RAT_RLL_D
                                          //  Referenced by: 'Derivative Gain' (':166:688')

    real32_T RAT_RLL_I;                // Variable: RAT_RLL_I
                                          //  Referenced by: 'Integral Gain' (':166:791')

    real32_T RAT_RLL_IMAX;             // Variable: RAT_RLL_IMAX
                                          //  Referenced by: 'Integrator' (':166:843')

    real32_T RAT_RLL_P;                // Variable: RAT_RLL_P
                                          //  Referenced by: 'Proportional Gain' (':166:913')

    real32_T RAT_YAW_D;                // Variable: RAT_YAW_D
                                          //  Referenced by: 'Derivative Gain' (':173:688')

    real32_T RAT_YAW_I;                // Variable: RAT_YAW_I
                                          //  Referenced by: 'Integral Gain' (':173:791')

    real32_T RAT_YAW_IMAX;             // Variable: RAT_YAW_IMAX
                                          //  Referenced by: 'Integrator' (':173:843')

    real32_T RAT_YAW_P;                // Variable: RAT_YAW_P
                                          //  Referenced by: 'Proportional Gain' (':173:913')

    real32_T ANGLE_rll_pid_InitialConditionF;
                              // Mask Parameter: ANGLE_rll_pid_InitialConditionF
                                 //  Referenced by: 'Filter' (':184:737')

    real32_T RollPID_InitialConditionForFilt;
                              // Mask Parameter: RollPID_InitialConditionForFilt
                                 //  Referenced by: 'Filter' (':166:737')

    real32_T angle_pit_PID_InitialConditionF;
                              // Mask Parameter: angle_pit_PID_InitialConditionF
                                 //  Referenced by: 'Filter' (':185:737')

    real32_T PitchPID_InitialConditionForFil;
                              // Mask Parameter: PitchPID_InitialConditionForFil
                                 //  Referenced by: 'Filter' (':165:737')

    real32_T Angle_YAW_PID_InitialConditionF;
                              // Mask Parameter: Angle_YAW_PID_InitialConditionF
                                 //  Referenced by: 'Filter' (':186:737')

    real32_T YawPID_InitialConditionForFilte;
                              // Mask Parameter: YawPID_InitialConditionForFilte
                                 //  Referenced by: 'Filter' (':173:737')

    real32_T ANGLE_rll_pid_InitialConditio_d;
                              // Mask Parameter: ANGLE_rll_pid_InitialConditio_d
                                 //  Referenced by: 'Integrator' (':184:843')

    real32_T RollPID_InitialConditionForInte;
                              // Mask Parameter: RollPID_InitialConditionForInte
                                 //  Referenced by: 'Integrator' (':166:843')

    real32_T angle_pit_PID_InitialConditio_g;
                              // Mask Parameter: angle_pit_PID_InitialConditio_g
                                 //  Referenced by: 'Integrator' (':185:843')

    real32_T PitchPID_InitialConditionForInt;
                              // Mask Parameter: PitchPID_InitialConditionForInt
                                 //  Referenced by: 'Integrator' (':165:843')

    real32_T Angle_YAW_PID_InitialConditio_i;
                              // Mask Parameter: Angle_YAW_PID_InitialConditio_i
                                 //  Referenced by: 'Integrator' (':186:843')

    real32_T YawPID_InitialConditionForInteg;
                              // Mask Parameter: YawPID_InitialConditionForInteg
                                 //  Referenced by: 'Integrator' (':173:843')

    real32_T ANGLE_rll_pid_LowerIntegratorSa;
                              // Mask Parameter: ANGLE_rll_pid_LowerIntegratorSa
                                 //  Referenced by: 'Integrator' (':184:843')

    real32_T RollPID_LowerIntegratorSaturati;
                              // Mask Parameter: RollPID_LowerIntegratorSaturati
                                 //  Referenced by: 'Integrator' (':166:843')

    real32_T angle_pit_PID_LowerIntegratorSa;
                              // Mask Parameter: angle_pit_PID_LowerIntegratorSa
                                 //  Referenced by: 'Integrator' (':185:843')

    real32_T PitchPID_LowerIntegratorSaturat;
                              // Mask Parameter: PitchPID_LowerIntegratorSaturat
                                 //  Referenced by: 'Integrator' (':165:843')

    real32_T Angle_YAW_PID_LowerIntegratorSa;
                              // Mask Parameter: Angle_YAW_PID_LowerIntegratorSa
                                 //  Referenced by: 'Integrator' (':186:843')

    real32_T YawPID_LowerIntegratorSaturatio;
                              // Mask Parameter: YawPID_LowerIntegratorSaturatio
                                 //  Referenced by: 'Integrator' (':173:843')

    real32_T ANGLE_rll_pid_N;          // Mask Parameter: ANGLE_rll_pid_N
                                          //  Referenced by: 'Filter Coefficient' (':184:882')

    real32_T RollPID_N;                // Mask Parameter: RollPID_N
                                          //  Referenced by: 'Filter Coefficient' (':166:882')

    real32_T angle_pit_PID_N;          // Mask Parameter: angle_pit_PID_N
                                          //  Referenced by: 'Filter Coefficient' (':185:882')

    real32_T PitchPID_N;               // Mask Parameter: PitchPID_N
                                          //  Referenced by: 'Filter Coefficient' (':165:882')

    real32_T Angle_YAW_PID_N;          // Mask Parameter: Angle_YAW_PID_N
                                          //  Referenced by: 'Filter Coefficient' (':186:882')

    real32_T YawPID_N;                 // Mask Parameter: YawPID_N
                                          //  Referenced by: 'Filter Coefficient' (':173:882')

    real32_T Integrator_gainval;       // Computed Parameter: Integrator_gainval
                                          //  Referenced by: 'Integrator' (':184:843')

    real32_T Filter_gainval;           // Computed Parameter: Filter_gainval
                                          //  Referenced by: 'Filter' (':184:737')

    real32_T Integrator_gainval_o;   // Computed Parameter: Integrator_gainval_o
                                        //  Referenced by: 'Integrator' (':166:843')

    real32_T Filter_gainval_d;         // Computed Parameter: Filter_gainval_d
                                          //  Referenced by: 'Filter' (':166:737')

    real32_T Integrator_gainval_d;   // Computed Parameter: Integrator_gainval_d
                                        //  Referenced by: 'Integrator' (':185:843')

    real32_T Filter_gainval_k;         // Computed Parameter: Filter_gainval_k
                                          //  Referenced by: 'Filter' (':185:737')

    real32_T Integrator_gainval_m;   // Computed Parameter: Integrator_gainval_m
                                        //  Referenced by: 'Integrator' (':165:843')

    real32_T Filter_gainval_f;         // Computed Parameter: Filter_gainval_f
                                          //  Referenced by: 'Filter' (':165:737')

    real32_T Integrator_gainval_i;   // Computed Parameter: Integrator_gainval_i
                                        //  Referenced by: 'Integrator' (':186:843')

    real32_T Filter_gainval_h;         // Computed Parameter: Filter_gainval_h
                                          //  Referenced by: 'Filter' (':186:737')

    real32_T Integrator_gainval_h;   // Computed Parameter: Integrator_gainval_h
                                        //  Referenced by: 'Integrator' (':173:843')

    real32_T Filter_gainval_di;        // Computed Parameter: Filter_gainval_di
                                          //  Referenced by: 'Filter' (':173:737')

  };

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // Block states
  DW rtDW;

  // Tunable parameters
  static P rtP;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  arduAttContModelClass();

  // Destructor
  ~arduAttContModelClass();
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is 'block_name' ('SID'), where block_name is the name of the block
//  and SID is the Simulink identifier of the block without the model name.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system(':3')    - opens block with Simulink identifier 'model:3'

#endif                                 // RTW_HEADER_arduAttCont_h_

//
// File trailer for generated code.
//
// [EOF]
//
