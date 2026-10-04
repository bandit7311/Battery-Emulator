#ifndef RENAULT_KANGOO_VEHICLESTATE_H
#define RENAULT_KANGOO_VEHICLESTATE_H

#include <stdint.h>

// Live display of the vehicle state for the Renault Kangoo driver (RX only, in the car).
//
// The vehicle sends the frame 0x350 every 100 ms. Byte 0 is the vehicle state, bytes 1 to 3 are a minute counter.
// During the shutdown after "ignition off" the state runs through C4, C3, C2, C0 and 00, then the bus goes silent
// (lossless log of 04.10.2026: C3 63.7 s, C2 60.0 s, C0 10.1 s, 00 0.9 s, last frame 134.8 s after ignition off).
// The page shows the state, the time since ignition off and the reference points, so that the time button can be
// pressed at the right moments without a stopwatch and a serial log.
//
// Only logic, no Arduino / CAN driver dependencies. The driver calls on_frame() for every received frame and
// update() regularly (from transmit_can()). All times are uint32_t milliseconds like millis(), wrap safe.
class KangooVehicleState {
 public:
  static constexpr uint32_t BUS_SILENT_MS = 3000;  // no 0x350 for this long: the bus has ended

  // Reference points of one logged shutdown, in seconds after "ignition off" (C7 to C4). Measured values, no guarantee.
  static constexpr uint32_t REF_C2_S = 64;
  static constexpr uint32_t REF_C0_S = 124;
  static constexpr uint32_t REF_00_S = 134;
  static constexpr uint32_t REF_END_S = 135;

  void on_frame(uint32_t id, bool ext, const uint8_t* data, uint8_t dlc, uint32_t now) {
    if (id != 0x350 || ext || dlc < 4) {
      return;
    }
    const uint8_t s = data[0];
    if (finished) {
      // a frame after the bus had ended: a new cycle starts, forget the old shutdown
      finished = false;
      active = false;
      ready_seen = false;
    }
    if (!have_frame || s != state_value) {
      if (have_frame && state_value == 0xC7 && (s == 0xC4 || s == 0xC3) && !active) {
        active = true;  // ready to drive -> C4/C3: ignition off
        shutdown_start = now;
        seq++;
      } else if (active && s == 0xC7) {
        active = false;  // ready again: the shutdown was cancelled
      }
      state_value = s;
      state_since = now;
    }
    if (s == 0xC7) {
      ready_seen = true;
    }
    counter_value = ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
    last_frame = now;
    have_frame = true;
  }

  // Detects the end of the bus (no 0x350 for BUS_SILENT_MS) while a shutdown is running.
  void update(uint32_t now) {
    if (active && (uint32_t)(now - last_frame) >= BUS_SILENT_MS) {
      active = false;
      finished = true;
      shutdown_length = (uint32_t)(last_frame - shutdown_start);
    }
  }

  bool seen() const { return have_frame; }
  uint8_t state() const { return state_value; }
  uint32_t counter() const { return counter_value; }
  uint32_t state_age_ms(uint32_t now) const { return have_frame ? (uint32_t)(now - state_since) : 0; }
  bool shutdown_active() const { return active; }
  bool shutdown_finished() const { return finished; }
  // Counts the detected ignition-off events (changes at the start of every shutdown).
  uint8_t shutdown_seq() const { return seq; }
  // Time since ignition off, valid while a shutdown runs and after its end, until the next wake-up.
  bool since_off_valid() const { return active || finished; }
  uint32_t since_off_ms(uint32_t now) const { return (active || finished) ? (uint32_t)(now - shutdown_start) : 0; }
  // Time since ignition off while the shutdown runs; after the end the length from ignition off to the last frame.
  uint32_t shutdown_age_ms(uint32_t now) const {
    if (finished) {
      return shutdown_length;
    }
    return active ? (uint32_t)(now - shutdown_start) : 0;
  }
  // Time since the last 0x350 frame (0 when none was seen yet).
  uint32_t bus_silent_ms(uint32_t now) const { return have_frame ? (uint32_t)(now - last_frame) : 0; }

  // Names of the shutdown stages (from the Twingo driver). Only meaningful during a shutdown, C3 also occurs at start.
  static const char* stage_name(uint8_t s) {
    switch (s) {
      case 0xC3:
        return "BAT TEMPO LEVEL";
      case 0xC2:
        return "CUT OFF PENDING";
      case 0xC0:
        return "SLEEPING";
      default:
        return "";
    }
  }

  // The next reference point after age_s seconds of shutdown. Returns false after the last one.
  static bool next_reference(uint32_t age_s, const char*& label, uint32_t& in_s) {
    static const struct {
      const char* name;
      uint32_t at;
    } refs[] = {{"C2", REF_C2_S}, {"C0", REF_C0_S}, {"00", REF_00_S}, {"bus end", REF_END_S}};
    for (unsigned i = 0; i < sizeof(refs) / sizeof(refs[0]); i++) {
      if (age_s < refs[i].at) {
        label = refs[i].name;
        in_s = refs[i].at - age_s;
        return true;
      }
    }
    return false;
  }

 private:
  bool have_frame = false;
  bool active = false;
  bool finished = false;
  bool ready_seen = false;
  uint8_t state_value = 0;
  uint32_t counter_value = 0;
  uint32_t state_since = 0;
  uint32_t last_frame = 0;
  uint32_t shutdown_start = 0;
  uint32_t shutdown_length = 0;
  uint8_t seq = 0;
};

#endif
