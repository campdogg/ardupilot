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
    if (!reset_target) {
        printf("reset target (pilot override? surface? bottom?)\n");
    }

    reset_target = true;
}

void Sub::SurfaceTracking::update_surface_offset()
{
    if (enabled) {
        // if the rangefinder is unhealthy, do nothing
        if (sub.rangefinder_alt_ok()) {
            // handle first reading or controller reset
            if (reset_target) {
                target_rangefinder_cm = sub.rangefinder_state.alt_cm_filt.get();
                sub.pos_control.set_pos_offset_z_cm(0);
                sub.pos_control.set_pos_offset_target_z_cm(0);
                reset_target = false;
                sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g m", target_rangefinder_cm * 0.01f);
            }

            // update position controller target offset
            const float pos_offset_target_z_cm = sub.pos_control.get_pos_offset_z_cm() +
                    sub.surface_tracking.pid_rangefinder.update_error(
                            target_rangefinder_cm - sub.rangefinder_state.alt_cm_filt.get(), 0.01);

            sub.pos_control.set_pos_offset_target_z_cm(pos_offset_target_z_cm);
        }
    } else {
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
    }
}

#endif
