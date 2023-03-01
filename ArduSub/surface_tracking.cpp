#include "Sub.h"

void Sub::SurfaceTracking::enable(bool _enabled)
{
    enabled = _enabled;
    reset_target = true;
}

void Sub::SurfaceTracking::reset()
{
    reset_target = true;
}

void Sub::SurfaceTracking::update_surface_offset()
{
#if RANGEFINDER_ENABLED == ENABLED
    if (enabled) {
        // check for first reading or timeout
        const uint32_t now_ms = AP_HAL::millis();
        const bool timeout = (now_ms - last_update_ms) > SURFACE_TRACKING_TIMEOUT_MS;

        if (sub.rangefinder_alt_ok()) {
            // handle first reading, recover from a timeout, or reengage tracking after a controller reset
            if (timeout || reset_target) {
                target_rangefinder_cm = sub.rangefinder_state.alt_cm_filt.get();
                sub.pos_control.set_pos_offset_z_cm(0);
                sub.pos_control.set_pos_offset_target_z_cm(0);
                reset_target = false;
                printf("target_rangefinder [re]set to %g\n", target_rangefinder_cm);
            }

            // update position controller target offset
            const float pos_offset_target_z_cm = sub.pos_control.get_pos_offset_z_cm() + target_rangefinder_cm -
                                                 sub.rangefinder_state.alt_cm_filt.get();
            sub.pos_control.set_pos_offset_target_z_cm(pos_offset_target_z_cm);
            last_update_ms = now_ms;
        }
    } else {
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
    }
#else
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
#endif
}
