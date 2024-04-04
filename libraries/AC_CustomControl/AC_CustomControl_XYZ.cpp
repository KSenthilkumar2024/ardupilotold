#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_XYZ_ENABLED

#include "AC_CustomControl_XYZ.h"
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_XYZ::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: Empty param1
    // @Description: Dummy parameter for xyz custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_XYZ, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dummy parameter for XYZ custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_XYZ, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dummy parameter for XYZ custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_XYZ, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_XYZ::AC_CustomControl_XYZ(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _p_angle_roll2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f),
    _p_angle_pitch2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f),
    _p_angle_yaw2(AC_ATTITUDE_CONTROL_ANGLE_P * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_I * 0.90f, AC_ATTITUDE_CONTROL_ANGLE_D * 0.90f, 0.0f, AC_ATTITUDE_CONTROL_ANGLE_IMAX * 0.90f),
    _pid_atti_rate_roll(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f),
    _pid_atti_rate_pitch(AC_ATC_MULTI_RATE_RP_P * 0.90f, AC_ATC_MULTI_RATE_RP_I * 0.90f, AC_ATC_MULTI_RATE_RP_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, 0.0f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f),
    _pid_atti_rate_yaw(AC_ATC_MULTI_RATE_YAW_P * 0.90f, AC_ATC_MULTI_RATE_YAW_I * 0.90f, AC_ATC_MULTI_RATE_YAW_D * 0.90f, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX * 0.90f, AC_ATC_MULTI_RATE_RPY_FILT_HZ * 0.90f, AC_ATC_MULTI_RATE_YAW_FILT_HZ * 0.90f, 0.0f)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
}
{
    AP_Param::setup_object_defaults(this, var_info);
    
    simulink_controller.initialize();  // new line
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_XYZ::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }
    //run custom controller after here
    Quaternion attitue_body, attitude_target;
    _ahrs->get_quat_boy_to_need(arttitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents theangular eror to rotate thethrust vectorusing x,y and z axis heading

    // arducopter main attitude controller already ran
    // we don't need to do anything else
    vector3f atitude_error;
    float_thrust_angle,_thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle, thrust_error_ange);
   
    //recalculate ang vel feedforward from attitude target model
    //rotation from ythe target frame to the body frame

    Quaternion rotation_target_to_body*_att_control->get_attitude_target_ang_vel();
     // target angle velocity in the body frame
    vector3f ang_vel_body_feedforwad = rotation_target_to_body*_att_control-get_attitude_target_amy_vel();
    vector3f gyro_latest - _ahrs->get_gyro_latest();
       

    // Attitude errors 
    float arg_attitude_error[3]{attitude_error.x, attitude_error.y, attitude_error.z};
    // Feed forward roots
    float arg_rate_ff[3]{ang_vel_body_feedforward.x,ang_vel_body_feedforward.y, ang_vel_body_feedforward.z};
    // Root/ rate Measurements
    float arg_rate_meas[3]{gyro_latest.x, gyro_latest.y, gyro_latest.z};
    float arg_ou1[3];
    
    
    simulink_controller.step(arg_attitude_error, arg_rate_ff,arg_rate_meas, arg_out1);
    
   //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "XYZ custom controller working");

    // return what arducopter main controller outputted
    
   return Vector3f(arg_out[0], arg_out[1], arg_out[2]);
    // return Vector3f(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_XYZ::reset(void)
{
}

#endif  // AP_CUSTOMCONTROL_XYZ_ENABLED
