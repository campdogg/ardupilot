#include "Sub.h"

#if RANGEFINDER_ENABLED == ENABLED

/*
 * RNG_HOLD (rangefinder hold) -- a variation on ALT_HOLD (depth hold)
 *
 * The real work for ALT_HOLD and RNG_HOLD is handled by AC_PosControl, which provides 2 inputs for depth:
 * -- target depth (sub.pos_control._pos_target.z). This is the desired depth, plus an offset.
 * -- target offset (sub.pos_control._pos_offset_target_z). This is the desired offset.
 *
 * ALT_HOLD and RNG_HOLD set the target depth in these situations:
 * -- During initialization, we call pos_control.init_z_controller(). This sets target depth to the current depth.
 * -- If the sub hits the surface or bottom we call pos_control.set_pos_target_z_cm().
 * -- If the pilot takes control we call pos_control.set_pos_target_z_from_climb_rate_cm().
 *
 * At the end of the control loop ALT_HOLD and RNG_HOLD call pos_control.update_z_controller() to pass the buck.
 *
 * ALT_HOLD does not use the target offset.
 *
 * RNG_HOLD sets the target offset to implement surface tracking. This is handled by Sub::SurfaceTracking. We call
 * SurfaceTracking in these situations:
 * -- During initialization, we call surface_tracking.enable().
 * -- During normal operation, we call surface_tracking.update_surface_offset().
 * -- If the sub hits the surface or bottom, or the pilot takes control, we call surface_tracking.reset().
 */

// rnghold_init - initialise rnghold controller
bool Sub::rnghold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.init_z_controller();

    last_pilot_heading = ahrs.yaw_sensor;

    // enable surface tracking
    surface_tracking.enable(true);

    return true;
}

// rnghold_run - runs the rnghold controller
// should be called at 100hz or more
void Sub::rnghold_run()
{
    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.5,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_z_controller(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    control_range();

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

void Sub::control_range() {
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, -get_pilot_speed_dn(), g.pilot_speed_up);

    // desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cm_s) < 0.05f)  {
        if (ap.at_surface) {
            // set target depth to 5 cm below SURFACE_DEPTH
            pos_control.set_pos_target_z_cm(MIN(pos_control.get_pos_target_z_cm(), g.surface_depth - 5.0f));
            surface_tracking.reset();
        } else if (ap.at_bottom) {
            // set target depth to 10 cm above bottom
            pos_control.set_pos_target_z_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, pos_control.get_pos_target_z_cm()));
            surface_tracking.reset();
        } else {
            // normal operation
            surface_tracking.update_surface_offset();
        }
    } else {
        // pilot is in control
        surface_tracking.reset();
    }

    // set the target z from the climb rate and the z offset, and adjust the z vel and accel targets
    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    // run the z vel and accel PID controllers
    pos_control.update_z_controller();
}

#endif