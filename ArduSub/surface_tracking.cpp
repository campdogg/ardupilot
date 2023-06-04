#include "Sub.h"

#if RANGEFINDER_ENABLED == ENABLED

void Sub::SurfaceTracking::enable(bool _enabled)
{
    // don't enable surface tracking if rangefinder is missing or unhealthy
    if (_enabled && !sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder is not OK, holding depth");
        return;
    }

    enabled = _enabled;
    reset_target = true;
}

void Sub::SurfaceTracking::reset()
{
    reset_target = true;
}

void Sub::SurfaceTracking::update_surface_offset()
{
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
                sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g m", target_rangefinder_cm * 0.01f);
            }

            // update position controller target offset
            const float pos_offset_target_z_cm = sub.pos_control.get_pos_offset_z_cm() +
                    sub.g.rangefinder_gain * (target_rangefinder_cm - sub.rangefinder_state.alt_cm_filt.get());
            sub.pos_control.set_pos_offset_target_z_cm(pos_offset_target_z_cm);
            last_update_ms = now_ms;
        } else {
            // rangefinder is unhealthy
            if (!reset_target) {
                sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder is not OK, holding depth");
                reset_target = true;
            }
        }
    } else {
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
    }
}

#endif
