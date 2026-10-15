#include "Copter.h"

#if FOLLOWER_LAUNCH_ENABLED

/*
 * follower_fsm.cpp - modular follower launch & engagement state machine
 *
 * See follower_fsm.h for the state description, GCS command interface and
 * copy/port notes.  This file contains:
 *   - FollowerFsm: state, FLWR_ parameters and engagement transitions
 *   - ModeFollow::follower_update(): glue that lets the state machine hold
 *     the vehicle on the ground (WAITING) or drive the interceptor launch
 *     (LAUNCHING) from within Follow mode
 */

// engagement state names, indexed by FollowerPhase
static const char *const follower_phase_names[] = {
    "WAITING",
    "LAUNCHING",
    "FOLLOW",
    "APPROACH",
    "COLLIDE",
    "DISMISSED",
};

// parameter definitions for the follower state machine (prefix FLWR_)
const AP_Param::GroupInfo FollowerFsm::var_info[] = {

    // @Param: TRIG_DIST
    // @DisplayName: Follower launch trigger distance
    // @Description: Horizontal distance from the target at which an armed, landed vehicle in FOLLOW mode automatically launches toward the target using the launch-pitch sequence (LAUNCH_PITCH_EN / LAUNCH_PITCH). The vehicle climbs to the target's altitude at trigger time and then follows it. 0 disables the automatic launch. Requires FOLL_ENABLE, FOLL_SYSID and a valid target position.
    // @Units: m
    // @Range: 0 10000
    // @User: Advanced
    AP_GROUPINFO("TRIG_DIST", 0, FollowerFsm, _trig_dist_m, 0),

    AP_GROUPEND
};

// mode-entry hook: capture the configured gap and pick WAITING or FOLLOW
// depending on whether the vehicle is on the ground
void FollowerFsm::enter()
{
    // capture the configured gap; FOLL_OFS_X is negative behind the target
    // when FOLL_OFS_TYPE=1 (lead-relative offsets)
    const float configured_gap_m = -copter.g2.follow.get_offset_m().x;
    if (configured_gap_m > 0.0f) {
        _safe_gap_m = configured_gap_m;
        _safe_gap_valid = true;
    }
    // the FSM owns the offsets whenever a gap is known; a remembered gap
    // survives engagement states that zeroed the offsets (e.g. a COLLIDE
    // followed by DISMISS and a later re-engagement)
    copter.g2.follow.set_offsets_managed(_safe_gap_valid);

    if (copter.motors->armed() && !copter.ap.land_complete) {
        // airborne (e.g. re-entering Follow after a dismiss): engage directly
        enter_follow();
        return;
    }

    // on the ground: wait for the launch trigger
    _active = false;
    set_phase(FollowerPhase::WAITING);
}

// start the interceptor launch toward the target's altitude
void FollowerFsm::enter_launching()
{
    set_phase(FollowerPhase::LAUNCHING);
}

// hold the lead vehicle at the configured safe gap
void FollowerFsm::enter_follow()
{
    if (!_safe_gap_valid) {
        // adopt whatever gap the follow offsets currently encode; with the
        // stock auto-initialisation this is the vehicle's position relative
        // to the target at the time Follow mode was entered
        _safe_gap_m = MAX(-copter.g2.follow.get_offset_m().x, 0.0f);
        _safe_gap_valid = true;
        if (_safe_gap_m > 0.0f) {
            copter.g2.follow.set_offsets_managed(true);
        }
    }

    _active = true;
    set_phase(FollowerPhase::FOLLOW);
    set_gap(_safe_gap_m);

    // restore the default horizontal speed in case an approach sped up
    copter.flightmode->set_speed_NE_ms(copter.wp_nav->get_default_speed_NE_ms());
}

// left Follow mode; re-entering Follow restarts the engagement
void FollowerFsm::enter_dismissed()
{
    _active = false;
    set_phase(FollowerPhase::DISMISSED);
}

// GCS engagement request (MAV_CMD_USER_1); returns true when accepted
bool FollowerFsm::handle_user_command(uint8_t request, float x_m, float speed_ms)
{
    switch (request) {
    case 1:     // FOLLOW: (re-)enter Follow mode / resume the safe gap
        if (copter.flightmode->mode_number() != Mode::Number::FOLLOW) {
            return copter.set_mode(Mode::Number::FOLLOW, ModeReason::GCS_COMMAND);
        }
        enter_follow();
        return true;

    case 2:     // APPROACH: close the gap by X
    case 3:     // COLLIDE: fly through the lead vehicle's position
        if (copter.flightmode->mode_number() != Mode::Number::FOLLOW || !_active) {
            // only meaningful once launched and following
            return false;
        }
        if (request == 2) {
            set_gap(MAX(_safe_gap_m - MAX(x_m, 0.0f), 0.0f));
            set_phase(FollowerPhase::APPROACH);
        } else {
            set_gap(0.0f);
            set_phase(FollowerPhase::COLLIDE);
        }
        if (is_positive(speed_ms)) {
            // speed up the closure; 0 keeps the current speed
            copter.flightmode->set_speed_NE_ms(speed_ms);
        }
        return true;

    case 4:     // DISMISS: leave Follow mode and hover
        if (copter.flightmode->mode_number() != Mode::Number::FOLLOW) {
            return false;
        }
        enter_dismissed();
        copter.set_mode(Mode::Number::LOITER, ModeReason::GCS_COMMAND);
        return true;

    case 5:     // SET_GAP: set a new safe gap (target of a later FOLLOW)
        _safe_gap_m = MAX(x_m, 0.0f);
        _safe_gap_valid = true;
        copter.g2.follow.set_offsets_managed(true);
        if (copter.flightmode->mode_number() == Mode::Number::FOLLOW &&
            _phase == FollowerPhase::FOLLOW) {
            set_gap(_safe_gap_m);
        }
        return true;
    }

    return false;
}

// set the engagement gap (metres behind the lead vehicle) in AP_Follow;
// with FOLL_OFS_TYPE=1 the X axis is along the lead vehicle's heading, so a
// negative X trails behind it
void FollowerFsm::set_gap(float gap_m)
{
    copter.g2.follow.set_offset_m(Vector3f(-MAX(gap_m, 0.0f), 0.0f, 0.0f));
}

void FollowerFsm::set_phase(FollowerPhase phase)
{
    _phase = phase;
    announce();
}

// announce the current phase to the GCS; the server/UI parse "FLWR_STATE "
void FollowerFsm::announce()
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FLWR_STATE %s",
                  follower_phase_names[(uint8_t)_phase]);
}

// drive the follower launch/engagement state machine; returns true when the
// state machine has handled this update cycle (i.e. the stock follow
// controller must not run)
bool ModeFollow::follower_update()
{
    FollowerFsm &fsm = g2.follower_fsm;

    switch (fsm.phase()) {
    case FollowerPhase::WAITING: {
        // hold safely on the ground while watching for the lead vehicle
        make_safe_ground_handling();

        if (!fsm.trigger_enabled() || !motors->armed() || !copter.ap.land_complete) {
            return true;
        }
        if (!g2.follow.have_target()) {
            return true;
        }
        Location target_loc;
        Vector3f target_vel_ned_ms;
        if (!g2.follow.get_target_location_and_velocity(target_loc, target_vel_ned_ms)) {
            return true;
        }
        if (copter.current_loc.get_distance(target_loc) > fsm.trigger_dist_m()) {
            return true;
        }

        // the lead vehicle is inside the trigger radius: launch toward its altitude
        float launch_alt_m;
        if (!target_loc.get_alt_m(Location::AltFrame::ABOVE_ORIGIN, launch_alt_m)) {
            return true;
        }
        launch_alt_m = MAX(launch_alt_m, copter.current_loc.alt * 0.01f + 2.0f);

        // when no gap was configured, let the stock offset auto-initialisation
        // place the vehicle at its current position relative to the target
        g2.follow.init_offsets_if_required();

        // point the launch-pitch thrust vector at the lead vehicle
        auto_yaw.set_fixed_yaw_rad(radians(copter.current_loc.get_bearing_to(target_loc)), 0.0f, 1, false);

        if (copter.g2.launch_pitch_enable) {
            copter.launch_sm.start();
        } else {
            copter.launch_sm.reset();
        }
        copter.set_auto_armed(true);
        auto_takeoff.start_m(launch_alt_m, false);
        fsm.enter_launching();
        return true;
    }

    case FollowerPhase::LAUNCHING: {
        // run the interceptor launch; the state machine flies the vehicle
        // until the launch altitude is reached
        copter.launch_sm.update();
        auto_takeoff.run();
        if (auto_takeoff.complete) {
            if (copter.g2.launch_pitch_enable) {
                copter.launch_sm.set_normal_nav();
            } else {
                copter.launch_sm.reset();
            }
            fsm.enter_follow();
        }
        return true;
    }

    default:
        // engagement states (FOLLOW/APPROACH/COLLIDE/DISMISSED) are flown by
        // the stock follow controller with the FSM-selected gap
        return false;
    }
}

#endif // FOLLOWER_LAUNCH_ENABLED
