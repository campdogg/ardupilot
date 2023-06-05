#pragma once

#include "defines.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduSub
#endif

// run at 400Hz on all systems
# define MAIN_LOOP_RATE    400

#ifndef SURFACE_DEPTH_DEFAULT
# define SURFACE_DEPTH_DEFAULT -10.0f // pressure sensor reading 10cm depth means craft is considered surfaced
#endif

//////////////////////////////////////////////////////////////////////////////
// PWM control
// default RC speed in Hz
#ifndef RC_SPEED_DEFAULT
#   define RC_SPEED_DEFAULT 200
#endif

//////////////////////////////////////////////////////////////////////////////
// Circle Nav parameters
//

#ifndef CIRCLE_NAV_ENABLED
# define CIRCLE_NAV_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// RCMAP
//

#ifndef RCMAP_ENABLED
# define RCMAP_ENABLED DISABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Rangefinder
//

#ifndef RANGEFINDER_ENABLED
# define RANGEFINDER_ENABLED ENABLED
#endif

#ifndef RANGEFINDER_HEALTH_MAX
# define RANGEFINDER_HEALTH_MAX 3          // number of good reads that indicates a healthy rangefinder
#endif

#ifndef RANGEFINDER_GAIN_DEFAULT
# define RANGEFINDER_GAIN_DEFAULT 0.8f     // gain for controlling how quickly rangefinder range adjusts target altitude (lower means slower reaction)
#endif

#ifndef RNGFND_P_DEFAULT
# define RNGFND_P_DEFAULT 0.4f     // default P term for rangefinder_pid
#endif

#ifndef RNGFND_I_DEFAULT
# define RNGFND_I_DEFAULT 0.0f     // default I term for rangefinder_pid
#endif

#ifndef RNGFND_D_DEFAULT
# define RNGFND_D_DEFAULT 0.0f     // default D term for rangefinder_pid
#endif

#ifndef THR_SURFACE_TRACKING_VELZ_MAX
# define THR_SURFACE_TRACKING_VELZ_MAX 150 // max vertical speed change while surface tracking with rangefinder
#endif

#ifndef RANGEFINDER_TIMEOUT_MS
# define RANGEFINDER_TIMEOUT_MS  1000      // desired rangefinder alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef RANGEFINDER_WPNAV_FILT_HZ
# define RANGEFINDER_WPNAV_FILT_HZ   0.25f // filter frequency for rangefinder altitude provided to waypoint navigation class
#endif

#ifndef SURFACE_TRACKING_TIMEOUT_MS
# define SURFACE_TRACKING_TIMEOUT_MS  1000 // surface tracking target alt will reset to current rangefinder alt after this many milliseconds without a good rangefinder alt
#endif

#ifndef RANGEFINDER_TILT_CORRECTION         // by disable tilt correction for use of range finder data by EKF
# define RANGEFINDER_TILT_CORRECTION ENABLED
#endif

// Avoidance (relies on Proximity and Fence)
#ifndef AVOIDANCE_ENABLED
# define AVOIDANCE_ENABLED DISABLED
#endif

#if AVOIDANCE_ENABLED == ENABLED // Avoidance Library relies on Fence
# define FENCE_ENABLED ENABLED
#endif

#ifndef MAV_SYSTEM_ID
# define MAV_SYSTEM_ID          1
#endif

#ifndef EKF_ORIGIN_MAX_DIST_M
# define EKF_ORIGIN_MAX_DIST_M         50000   // EKF origin and waypoints (including home) must be within 50km
#endif

//////////////////////////////////////////////////////////////////////////////
// Nav-Guided - allows external nav computer to control vehicle
#ifndef NAV_GUIDED
# define NAV_GUIDED    ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Flight mode definitions
//

// Acro Mode
#ifndef ACRO_RP_P
# define ACRO_RP_P                 4.5f
#endif

#ifndef ACRO_YAW_P
# define ACRO_YAW_P                3.375f
#endif

#ifndef ACRO_LEVEL_MAX_ANGLE
# define ACRO_LEVEL_MAX_ANGLE      3000
#endif

#ifndef ACRO_BALANCE_ROLL
#define ACRO_BALANCE_ROLL          1.0f
#endif

#ifndef ACRO_BALANCE_PITCH
#define ACRO_BALANCE_PITCH         1.0f
#endif

#ifndef ACRO_EXPO_DEFAULT
#define ACRO_EXPO_DEFAULT          0.3f
#endif

// AUTO Mode
#ifndef WP_YAW_BEHAVIOR_DEFAULT
# define WP_YAW_BEHAVIOR_DEFAULT   WP_YAW_BEHAVIOR_CORRECT_XTRACK
#endif

#ifndef AUTO_YAW_SLEW_RATE
# define AUTO_YAW_SLEW_RATE    60              // degrees/sec
#endif

#ifndef YAW_LOOK_AHEAD_MIN_SPEED
# define YAW_LOOK_AHEAD_MIN_SPEED  100             // minimum ground speed in cm/s required before vehicle is aimed at ground course
#endif

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_INPUT_MAX
# define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
#endif
#ifndef DEFAULT_ANGLE_MAX
# define DEFAULT_ANGLE_MAX         4500            // ANGLE_MAX parameters default value
#endif

//////////////////////////////////////////////////////////////////////////////
// Loiter position control gains
//
#ifndef POS_XY_P
# define POS_XY_P               1.0f
#endif

//////////////////////////////////////////////////////////////////////////////
// PosHold parameter defaults
//
#ifndef POSHOLD_ENABLED
# define POSHOLD_ENABLED               ENABLED // PosHold flight mode enabled by default
#endif

//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//

#ifndef THR_DZ_DEFAULT
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif

// default maximum vertical velocity and acceleration the pilot may request
#ifndef PILOT_VELZ_MAX
# define PILOT_VELZ_MAX    500     // maximum vertical velocity in cm/s
#endif
#ifndef PILOT_ACCEL_Z_DEFAULT
# define PILOT_ACCEL_Z_DEFAULT 100 // vertical acceleration in cm/s/s while altitude is under pilot control
#endif

#ifndef AUTO_DISARMING_DELAY
# define AUTO_DISARMING_DELAY  0
#endif

//////////////////////////////////////////////////////////////////////////////
// Logging control
//
#ifndef LOGGING_ENABLED
# define LOGGING_ENABLED                ENABLED
#endif

// Default logging bitmask
#ifndef DEFAULT_LOG_BITMASK
# define DEFAULT_LOG_BITMASK \
    MASK_LOG_ATTITUDE_MED | \
    MASK_LOG_GPS | \
    MASK_LOG_PM | \
    MASK_LOG_CTUN | \
    MASK_LOG_NTUN | \
    MASK_LOG_RCIN | \
    MASK_LOG_IMU | \
    MASK_LOG_CMD | \
    MASK_LOG_CURRENT | \
    MASK_LOG_RCOUT | \
    MASK_LOG_OPTFLOW | \
    MASK_LOG_COMPASS | \
    MASK_LOG_CAMERA | \
    MASK_LOG_MOTBATT
#endif
