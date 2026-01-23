#include "mech.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/feeder.hpp"
#include "subsystems/color.hpp"
#include "subsystems/loader.hpp"
#include "pros/apix.h"

// ================= CONTROLLER MAP =================
// Left side:
//   L1  -> Intake ON/OFF toggle
//   L2  -> Intake direction flip (rumble: "." = in, ".." = out)
// 
// Right side:
//   R1 -> Deploy loader
//   R2 -> Retract loader
//
// D-Pad (Feeder modes):
//   UP    -> Score Upper  (toggle; X to force off)
//   LEFT  -> Score Middle (toggle; X to force off)
//   DOWN  -> Score Lower  (toggle; X to force off)
//   RIGHT -> Intake / Store (toggle; X to force off)
//
// Buttons:
//   X -> Feeder OFF (force shutoff)
//   Y -> Toggle Field/Robot drive
//   B -> Zero heading
//   A -> (none)
// ================================================

using namespace pros;

namespace mech {

  // ---------------- Shared state (teleop + auton) ----------------
  static feeder::Mode g_feeder_mode = feeder::Mode::Off;
  static TeamColor g_team =
  #if defined(ALLIANCE_BLUE)
    TeamColor::Blue;
  #elif defined(ALLIANCE_RED)
    TeamColor::Red;
  #else
    TeamColor::Red;  // default
  #endif

  // ---------------- Auto-sort state machine internals ----------------
  enum class SortState {
    Idle,
    ReverseFeeder2,
    WaitSense,
    WaitEnd
  };

  inline TeamColor opponent(TeamColor c) {
    return (c == TeamColor::Red) ? TeamColor::Blue : TeamColor::Red;
  }

  void intake_stop() {
    intake::stop();
  }

  void intake_set(int pct) {
    intake::set(pct);
  }

  static SortState s_state = SortState::Idle;
  static uint32_t  s_t0    = 0;
  static int enemy_streak = 0;
  constexpr int ENEMY_CONFIRM_TICKS = 3;  // ~30ms if your loop is 10ms

  // =============== TUNABLES ================
  // =========================================
  constexpr int SORT_FEED2_PWR   = +127;  // reverse feeder2 to eject enemy
  constexpr uint32_t T_REV_FEED2 = 350;  // ms reverse time
  constexpr uint32_t T_WAIT_SENSE= 60;   // ms settle before re-sensing
  constexpr uint32_t T_WAIT_END  = 60;   // ms settle before re-arming detection
  // =========================================
  // ============= TUNABLES ==================

  inline bool have_object() {
    return color::proximity() >= PROX_THRESHOLD;
  }

  inline bool enemy_present() {
    if (color::proximity() < PROX_THRESHOLD) return false;

    const bool ours = color::is_team_color(g_team);
    if (ours) return false;

    // Only call it enemy if it matches opponent range.
    const bool opp = color::is_team_color(opponent(g_team));
    return opp;
  }

  // Auto-sort tick: ONLY active when feeder is storing.
  static void auto_sort_tick(feeder::Mode current_mode) {
    // If we're not storing, sorting is inactive and overrides must be cleared.
    if (current_mode != feeder::Mode::IntakeStore) {
      s_state = SortState::Idle;
      enemy_streak = 0;
      feeder::clear_overrides();
      return;
    }

    const uint32_t now = pros::millis();

    switch (s_state) {

      case SortState::Idle: {
        feeder::clear_overrides();

        // Step 1: detect color
        if (enemy_present()) enemy_streak++;
        else enemy_streak = 0;

        if (enemy_streak < ENEMY_CONFIRM_TICKS) break;

        // Enemy: step a) reverse feeder2
        feeder::override_motor2(SORT_FEED2_PWR);
        s_t0 = now;
        s_state = SortState::ReverseFeeder2;
      } break;

      case SortState::ReverseFeeder2: {
        // keep overriding while timed
        enemy_streak = 0;
        feeder::override_motor2(SORT_FEED2_PWR);

        if (now - s_t0 >= T_REV_FEED2) {
          feeder::override_motor2(0);   // stop the override power
          s_t0 = now;
          s_state = SortState::WaitSense;
        }
      } break;

      case SortState::WaitSense: {
        // step b) delay, then re-check
        if (now - s_t0 < T_WAIT_SENSE) break;

        if (enemy_present()) {
          // step i) still enemy -> loop back to reverse again
          feeder::override_motor2(SORT_FEED2_PWR);
          s_t0 = now;
          s_state = SortState::ReverseFeeder2;
        } else {
          // step ii) friendly OR gone -> clear overrides and resume normal mode behavior
          feeder::clear_overrides();

          // optional settle delay before re-arming
          s_t0 = now;
          s_state = SortState::WaitEnd;

          // (If you want zero settle delay, do: s_state = SortState::Idle;)
        }
      } break;

      case SortState::WaitEnd: {
        if (now - s_t0 >= T_WAIT_END) {
          s_state = SortState::Idle;
        }
      } break;
    }
  }

// ---------------- Public mech API ----------------

void initialize() {
  intake::initialize();
  feeder::initialize();
  color::initialize();
  loader::initialize();
}

void set_team_color(TeamColor c) { g_team = c; }
TeamColor team_color() { return g_team; }

feeder::Mode feeder_mode() { return g_feeder_mode; }

void set_feeder_mode(feeder::Mode m) {
  g_feeder_mode = m;
  feeder::set_mode(m);
}

void feeder_stop() {
  feeder::stop();
  g_feeder_mode = feeder::Mode::Off;
}

// loader pneumatics
bool loader_is_deployed() { return loader::is_deployed(); }
void loader_deploy()      { loader::deploy(); }
void loader_retract()     { loader::retract(); }

void update() {
  // Call this frequently in BOTH teleop and auton.
  auto_sort_tick(g_feeder_mode);
}

void teleop(pros::Controller& master) {

  // ----- LOADER (PNEUMATIC) -----
  // R1 -> Deploy
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
    loader_deploy();
  }  // R2 -> Retract
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
    loader_retract();
  }

  // ----- Intake ON/OFF + Direction Flip -----
  static bool intake_running = false;   // false = off, true = on
  static int intake_dir = 1;            // +1 = forward, -1 = reverse
  // Auto-intake override (driven by feeder modes)
  static bool auto_intake_active = false;
  static int  auto_intake_dir    = 1;   // +1=in, -1=out

  // L1 = toggle intake ON/OFF
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
      intake_running = !intake_running;   // flip state
  }

  // L2 = reverse direction
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
      intake_dir *= -1;                   // flip direction

    // RUMBLE FEEDBACK:
    if (intake_dir == 1) {
        master.rumble(".");    // 1 pulse = forward/intaking
    } else {
        master.rumble("..");   // 2 pulses = reverse/outtaking
    }
  }

  // Apply intake motor power
  if (intake_running) {
      intake::set(127 * intake_dir);
  } else {
      intake::stop();
  }

  feeder::Mode new_mode = g_feeder_mode;

  // ================== FEEDER (D-pad toggle + X force off) ==================
  // D-pad buttons as toggles (persistent across loops)
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
    new_mode = (g_feeder_mode == feeder::Mode::IntakeStore)
                ? feeder::Mode::Off
                : feeder::Mode::IntakeStore;
  } else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
    new_mode = (g_feeder_mode == feeder::Mode::ScoreLower)
                ? feeder::Mode::Off
                : feeder::Mode::ScoreLower;
  } else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
    new_mode = (g_feeder_mode == feeder::Mode::ScoreMiddle)
                ? feeder::Mode::Off
                : feeder::Mode::ScoreMiddle;
  } else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
    new_mode = (g_feeder_mode == feeder::Mode::ScoreUpper)
                ? feeder::Mode::Off
                : feeder::Mode::ScoreUpper;
  }

  // X overrides EVERYTHING (feeder + intake)
  if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
    // Feeder
    g_feeder_mode = feeder::Mode::Off;
    feeder::clear_overrides();

    // Intake
    intake_running = false;
    auto_intake_active = false;
    intake::stop();
  }

    // If mode changed, update g_feeder_mode and auto-intake behavior
    if (new_mode != g_feeder_mode) {
      g_feeder_mode = new_mode;


      // Auto intake rules
    if (g_feeder_mode == feeder::Mode::IntakeStore) {
      auto_intake_active = true;
      auto_intake_dir = +1;          // IN
      intake_running = true;         // ensure it actually runs
      intake_dir = +1;               // keep manual state aligned
    } else if (g_feeder_mode == feeder::Mode::ScoreLower) {
      auto_intake_active = true;
      auto_intake_dir = -1;          // OUT
      intake_running = true;
      intake_dir = -1;
    } else {
      // release auto override for other modes/off
      auto_intake_active = false;
    }
  }

  // Apply intake motor power
  if (intake_running) {
    const int dir = auto_intake_active ? auto_intake_dir : intake_dir;
    intake::set(127 * dir);
  } else {
    intake::stop();
  }

  // Apply mode to motors
  set_feeder_mode(g_feeder_mode);

  // Run autosort tick (will only do anything during IntakeStore)
  update();
} // end teleop

} // namespace mech
