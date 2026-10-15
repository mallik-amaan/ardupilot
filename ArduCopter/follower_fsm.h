#pragma once

#include <AP_Param/AP_Param.h>

/*
 * follower_fsm.h - modular follower launch & engagement state machine for Follow mode
 *
 * Self-contained implementation of an autonomous "follower" built on the stock
 * Follow mode and AP_Follow library.  The vehicle arms and waits on the ground
 * in FOLLOW; when the lead vehicle (FOLL_SYSID) comes within FLWR_TRIG_DIST it
 * launches with the interceptor launch-pitch sequence (LAUNCH_PITCH_EN /
 * LAUNCH_PITCH) toward the lead vehicle's altitude, then tracks the lead
 * vehicle while holding a configurable gap.  A GCS (MAV_CMD_USER_1) selects
 * between the FOLLOW / APPROACH / COLLIDE engagement states at runtime.
 *
 * States:
 *   WAITING    armed and on the ground, watching for the lead vehicle
 *   LAUNCHING  interceptor takeoff (launch pitch) toward the lead vehicle alt
 *   FOLLOW     tracking the lead vehicle at the configured safe gap
 *   APPROACH   tracking the lead vehicle at (safe gap - X)
 *   COLLIDE    tracking the lead vehicle's position (gap = 0)
 *   DISMISSED  left Follow mode (LOITER); re-entering Follow restarts
 *
 * ---------------------------------------------------------------------
 * Copy/port notes - this file (together with ArduCopter/follower_fsm.cpp)
 * is designed to be lifted into another codebase.  It depends on:
 *
 *  - AP_Follow (stock) target-data accessors plus three small extensions
 *    (libraries/AP_Follow/AP_Follow.h):
 *        set_offsets_managed(), set_offset_m(), get_offset_m()
 *  - the interceptor launch-pitch feature (LAUNCH_PITCH_EN / LAUNCH_PITCH
 *    parameters, Copter::LaunchStateMachine and _AutoTakeoff in
 *    ArduCopter/takeoff.cpp)
 *  - hooks in ModeFollow (ArduCopter/mode_follow.cpp):
 *        init():  g2.follower_fsm.enter();       (before "return true")
 *        run():   if (follower_update()) { return; }   (at the top)
 *    The ModeFollow::follower_update() glue is implemented in
 *    follower_fsm.cpp; it needs access to Mode's protected auto_takeoff
 *    object and make_safe_ground_handling(), which is why it lives on
 *    ModeFollow rather than on FollowerFsm.
 *  - a MAV_CMD_USER_1 case in
 *    GCS_MAVLink_Copter::handle_command_int_packet()
 *  - a ParametersG2 member plus AP_SUBGROUPINFO registration
 *    (ArduCopter/Parameters.h / Parameters.cpp)
 *  - the FOLLOWER_LAUNCH_ENABLED define (ArduCopter/config.h,
 *    Tools/scripts/build_options.py)
 *
 * ---------------------------------------------------------------------
 * GCS command interface (MAV_CMD_USER_1):
 *   param1: 1 = FOLLOW (resume the safe gap)
 *           2 = APPROACH
 *           3 = COLLIDE
 *           4 = DISMISS (switch to LOITER)
 *           5 = SET_GAP (param2 is the new safe gap; resumption target)
 *   param2: approach distance X in metres (APPROACH; the new gap is
 *           max(safe gap - X, 0)), or the new gap for SET_GAP
 *   param3: horizontal speed for APPROACH / COLLIDE in m/s
 *           (0 = unchanged); FOLLOW restores the default WP speed
 */

// engagement state machine phases
enum class FollowerPhase : uint8_t {
    WAITING = 0,
    LAUNCHING,
    FOLLOW,
    APPROACH,
    COLLIDE,
    DISMISSED,
};

class FollowerFsm {
public:

    // parameter table (prefix FLWR_)
    static const struct AP_Param::GroupInfo var_info[];

    // mode-entry hook: capture the configured gap and pick WAITING or
    // FOLLOW depending on whether the vehicle is on the ground
    void enter();

    // engagement transitions (announce themselves to the GCS)
    void enter_launching();
    void enter_follow();
    void enter_dismissed();

    // true when an automatic launch trigger is configured
    bool trigger_enabled() const { return _trig_dist_m > 0.0f; }
    float trigger_dist_m() const { return _trig_dist_m; }

    FollowerPhase phase() const { return _phase; }
    bool launched() const { return _active; }

    // GCS engagement request (MAV_CMD_USER_1); returns true when accepted
    bool handle_user_command(uint8_t request, float x_m, float speed_ms);

    // announce the current phase to the GCS ("FLWR_STATE <name>")
    void announce();

private:

    // set the engagement gap (metres behind the lead vehicle) in AP_Follow
    void set_gap(float gap_m);

    // select a phase and announce it
    void set_phase(FollowerPhase phase);

    AP_Float _trig_dist_m;      // FLWR_TRIG_DIST: launch trigger radius (m), 0 = disabled

    FollowerPhase _phase;       // current state
    float _safe_gap_m;          // gap to hold in FOLLOW; target of RESUME
    bool _safe_gap_valid;       // true once _safe_gap_m has been captured
    bool _active;               // true once launched / engaged (allows APPROACH & COLLIDE)
};
