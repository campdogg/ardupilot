#include "Sub.h"

void Sub::SurfaceTracking::start_tracking()
{
    if (sub.rangefinder_alt_ok()) {
        target_rangefinder_cm = sub.rangefinder_state.alt_cm_filt.get();
        tracking = true;
        printf("Surface tracking on, rangefinder: %g\n", target_rangefinder_cm * 0.01);
    } else {
        tracking = false;
        printf("Surface tracking off, rangefinder not ok\n");
    }
}

// update_surface_offset - vertical offset of the position controller tracks the rangefinder
void Sub::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    // check for timeout
    const uint32_t now_ms = AP_HAL::millis();
    const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

    // check tracking state and that range finder is healthy
    if (tracking && sub.rangefinder_alt_ok() /* && (sub.rangefinder_state.glitch_count == 0) */) {

        // update position controller target offset
        const float pos_offset_target_z_cm = sub.pos_control.get_pos_offset_z_cm() + target_rangefinder_cm - sub.rangefinder_state.alt_cm_filt.get();
        sub.pos_control.set_pos_offset_target_z_cm(pos_offset_target_z_cm);
        last_update_ms = now_ms;
        valid_for_logging = true;

        // reset target altitude if this controller has just been engaged
        // target has been changed between upwards vs downwards TODO
        // or glitch has cleared
        if (timeout ||
            reset_target ||
            (last_glitch_cleared_ms != sub.rangefinder_state.glitch_cleared_ms)) {

            printf("Clyde u_s_o reset!\n");

            sub.pos_control.set_pos_offset_z_cm(pos_offset_target_z_cm);
            reset_target = false;
            last_glitch_cleared_ms = sub.rangefinder_state.glitch_cleared_ms;
        }

    } else {
        // reset position controller offsets if surface tracking is inactive
        // flag target should be reset when/if it next becomes active
        if (timeout) {
            if (tracking) {
                printf("Clyde u_s_o unhealthy, timeout!\n");
            }

            sub.pos_control.set_pos_offset_z_cm(0);
            sub.pos_control.set_pos_offset_target_z_cm(0);
            reset_target = true;
        }
    }
#else
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
#endif
}


// get target altitude (in cm) above ground
// returns true if there is a valid target
bool Sub::SurfaceTracking::get_target_alt_cm(float &target_alt_cm) const
{
    // fail if we are not tracking downwards
    if (!tracking) {
        return false;
    }
    // check target has been updated recently
    if (AP_HAL::millis() - last_update_ms > SURFACE_TRACKING_TIMEOUT_MS) {
        return false;
    }
    target_alt_cm = (sub.pos_control.get_pos_target_z_cm() - sub.pos_control.get_pos_offset_z_cm());
    return true;
}

// set target altitude (in cm) above ground
void Sub::SurfaceTracking::set_target_alt_cm(float _target_alt_cm)
{
    // fail if we are not tracking downwards
    if (!tracking) {
        return;
    }
    sub.pos_control.set_pos_offset_z_cm(sub.inertial_nav.get_position_z_up_cm() - _target_alt_cm);
    last_update_ms = AP_HAL::millis();
}
