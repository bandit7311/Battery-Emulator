#ifndef RENAULT_KANGOO_LIVE_H
#define RENAULT_KANGOO_LIVE_H

#include <stdint.h>
#include <stdio.h>
#include "RENAULT-KANGOO-VEHICLESTATE.h"

// Data of the live page /kangooLive of the Renault Kangoo driver (RX only, in the car):
//  - the list of the last results of the time query (22 92 61), with the time since ignition off, so that the value
//    that jumps at the shutdown can be read on the page even when the serial log has gaps,
//  - the optional automatic schedule that sends the time query at the right moments of a shutdown,
//  - the short text that the page fetches every second.
//
// The automatic schedule is OFF after every start. When it is switched on, it sends ONLY the read request
// 22 92 61, at most MAX_AUTO times per shutdown, one at a time, and never while the measurement list runs.
//
// Only logic, no Arduino / CAN driver dependencies. All times are uint32_t milliseconds like millis(), wrap safe.
class KangooLive {
 public:
  static constexpr uint8_t MAX_RESULTS = 30;
  static constexpr uint8_t MAX_AUTO = 40;  // automatic queries per shutdown

  // Automatic schedule (from the logs of 04.10.2026, see the vehicle state header for the stage times):
  //  - coarse: once at 60, 90 and 110 s after ignition off,
  //  - dense: every 500 ms from 7 s after the start of C0 until 3 s after the start of 00 (the value jumps there),
  //    ONCE per shutdown: it is never started again when the vehicle runs through C0 and 00 once more,
  //  - after the bus end (no 0x350 for 3 s): every 5 s from 40 s to 65 s of silence (the LBC falls asleep there).
  // No automatic query is sent while the last 0x350 frame is older than BUS_LIVE_MS: in the silence after the end of
  // the vehicle bus nothing is sent (in the log of 11:38 the vehicle started again several times, 1.8 s after the end
  // of the bus; the dense window had sent queries into those pauses; whether they caused it is not proven).
  static constexpr uint8_t COARSE_COUNT = 3;
  static constexpr uint32_t DENSE_FROM_C0_MS = 7000;
  static constexpr uint32_t DENSE_UNTIL_00_MS = 3000;
  static constexpr uint32_t DENSE_STEP_MS = 500;
  static constexpr uint32_t BUS_LIVE_MS = 500;  // the last 0x350 frame must not be older than this
  static constexpr uint8_t TAIL_COUNT = 6;
  static constexpr uint32_t TAIL_FIRST_S = 40;
  static constexpr uint32_t TAIL_STEP_S = 5;

  enum Kind : uint8_t { MANUAL = 0, AUTOMATIC = 1 };
  enum Result : uint8_t { RES_OK = 1, RES_NEGATIVE = 2, RES_NO_ANSWER = 3 };

  struct Entry {
    bool since_valid = false;
    uint32_t since_off_ms = 0;  // time since ignition off when the query was sent
    uint8_t state = 0;          // 0x350 state at that moment
    bool state_valid = false;
    uint32_t counter = 0;       // 0x350 minute counter at that moment
    uint8_t kind = MANUAL;
    uint8_t result = 0;
    uint32_t value = 0;         // 0x9261 (minutes) for RES_OK, NRC for RES_NEGATIVE
    uint8_t tries = 0;
  };

  // ---- results ----
  // Called when a query is started (manual or automatic): remembers the context of the moment.
  void begin_query(uint32_t now, const KangooVehicleState& v, bool automatic) {
    pending = Entry();
    pending.since_valid = v.since_off_valid();
    pending.since_off_ms = v.since_off_ms(now);
    pending.state_valid = v.seen();
    pending.state = v.state();
    pending.counter = v.counter();
    pending.kind = automatic ? AUTOMATIC : MANUAL;
    pending_active = true;
  }
  bool query_pending() const { return pending_active; }
  // Called when the query has finished: puts the entry into the list.
  void end_query(uint8_t result, uint32_t value, uint8_t tries) {
    if (!pending_active) {
      return;
    }
    pending.result = result;
    pending.value = value;
    pending.tries = tries;
    pending_active = false;
    entries[(first + count) % MAX_RESULTS] = pending;
    if (count < MAX_RESULTS) {
      count++;
    } else {
      first = (uint8_t)((first + 1) % MAX_RESULTS);
    }
  }
  uint8_t result_count() const { return count; }
  // i = 0 is the newest entry
  const Entry& result_newest_first(uint8_t i) const { return entries[(first + count - 1 - i + 2 * MAX_RESULTS) % MAX_RESULTS]; }
  void clear_results() { count = 0; first = 0; }

  // ---- automatic schedule ----
  void set_auto(bool on) {
    auto_on = on;
    if (!on) {
      reset_schedule();
    }
  }
  bool auto_enabled() const { return auto_on; }
  uint8_t auto_count() const { return auto_n; }

  // True when an automatic query has to be started now. bus_free = no other request is running.
  bool auto_poll(uint32_t now, const KangooVehicleState& v, bool bus_free) {
    if (!auto_on) {
      return false;
    }
    if (v.shutdown_seq() != seq_seen) {  // a new shutdown: start the schedule again
      seq_seen = v.shutdown_seq();
      reset_schedule();
    }
    if (!(v.shutdown_active() || v.shutdown_finished()) || !bus_free || auto_n >= MAX_AUTO) {
      return false;
    }
    if (v.shutdown_active()) {
      if (v.bus_silent_ms(now) > BUS_LIVE_MS) {  // the bus has ended (or is about to): send nothing into the silence
        if (dense_started) {
          dense_done = true;  // the dense window ends with the last frame and is not started again
        }
        return false;
      }
      const uint32_t since = v.since_off_ms(now);
      static const uint32_t coarse_s[COARSE_COUNT] = {60, 90, 110};
      for (uint8_t i = 0; i < COARSE_COUNT; i++) {
        if (!coarse_done[i] && since >= coarse_s[i] * 1000UL) {
          coarse_done[i] = true;
          return fire();
        }
      }
      const uint8_t st = v.state();
      const uint32_t age = v.state_age_ms(now);
      const bool dense = (st == 0xC0 && age >= DENSE_FROM_C0_MS) || (st == 0x00 && age < DENSE_UNTIL_00_MS);
      if (!dense_done) {
        if (dense) {
          if (!dense_started || (uint32_t)(now - last_dense) >= DENSE_STEP_MS) {
            dense_started = true;
            last_dense = now;
            return fire();
          }
        } else if (dense_started) {
          dense_done = true;  // the window was left: once per shutdown
        }
      }
    } else {  // finished: the bus has ended
      const uint32_t silent = v.bus_silent_ms(now);
      for (uint8_t k = 0; k < TAIL_COUNT; k++) {
        if (!tail_done[k] && silent >= (TAIL_FIRST_S + TAIL_STEP_S * k) * 1000UL) {
          tail_done[k] = true;
          return fire();
        }
      }
    }
    return false;
  }

  // ---- text for the page ----
  // One "key=value" per line. Times in seconds. Returns the length (the text is cut off cleanly if it does not fit).
  size_t format(char* out, size_t n, uint32_t now, const KangooVehicleState& v, bool query_running) const {
    if (n == 0) {
      return 0;
    }
    size_t len = 0;
    out[0] = 0;
    char line[96];
    auto put = [&](const char* s) {
      const size_t l = strlen_local(s);
      if (len + l + 1 < n) {
        for (size_t i = 0; i < l; i++) {
          out[len + i] = s[i];
        }
        len += l;
        out[len] = 0;
      }
    };
    if (!v.seen()) {
      put("state=--\nstate_age=0\ncounter=-\nbus=none\n");
    } else {
      snprintf(line, sizeof(line), "state=%02X\n", (unsigned)v.state());
      put(line);
      const bool in_shutdown = v.shutdown_active() || v.shutdown_finished();
      snprintf(line, sizeof(line), "name=%s\n", in_shutdown ? KangooVehicleState::stage_name(v.state()) : "");
      put(line);
      snprintf(line, sizeof(line), "state_age=%lu\n", (unsigned long)(v.state_age_ms(now) / 1000));
      put(line);
      snprintf(line, sizeof(line), "counter=%lu\n", (unsigned long)v.counter());
      put(line);
      const uint32_t silent = v.bus_silent_ms(now);
      if (silent >= KangooVehicleState::BUS_SILENT_MS) {
        snprintf(line, sizeof(line), "bus=silent:%lu\n", (unsigned long)(silent / 1000));
      } else {
        snprintf(line, sizeof(line), "bus=active\n");
      }
      put(line);
    }
    if (v.shutdown_active()) {
      put("shutdown=running\n");
    } else if (v.shutdown_finished()) {
      put("shutdown=finished\n");
    } else {
      put("shutdown=none\n");
    }
    if (v.since_off_valid()) {
      const uint32_t ms = v.since_off_ms(now);
      snprintf(line, sizeof(line), "since=%lu.%lu\n", (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 100));
      put(line);
      if (v.shutdown_active()) {
        const char* label = nullptr;
        uint32_t in_s = 0;
        if (KangooVehicleState::next_reference((ms + 500) / 1000, label, in_s)) {
          snprintf(line, sizeof(line), "next=%s:%lu\n", label, (unsigned long)in_s);
          put(line);
        }
      }
    }
    snprintf(line, sizeof(line), "query=%s\nauto=%u\nauto_n=%u\nres_n=%u\n", query_running ? "running" : "idle",
             auto_on ? 1u : 0u, (unsigned)auto_n, (unsigned)count);
    put(line);
    for (uint8_t i = 0; i < count; i++) {
      const Entry& e = result_newest_first(i);
      char since[16];
      if (e.since_valid) {
        snprintf(since, sizeof(since), "%lu.%lu", (unsigned long)(e.since_off_ms / 1000),
                 (unsigned long)((e.since_off_ms % 1000) / 100));
      } else {
        snprintf(since, sizeof(since), "-");
      }
      snprintf(line, sizeof(line), "r=%s|%s|%u|%u|%lu|%lu|%u\n", since, hex_state(e), (unsigned)e.kind, (unsigned)e.result,
               (unsigned long)e.value, (unsigned long)e.counter, (unsigned)e.tries);
      put(line);
    }
    return len;
  }

 private:
  static size_t strlen_local(const char* s) {
    size_t l = 0;
    while (s[l] != 0) {
      l++;
    }
    return l;
  }
  static const char* hex_state(const Entry& e) {
    static char buf[4][4];
    static uint8_t idx = 0;
    char* b = buf[idx = (uint8_t)((idx + 1) & 3)];
    if (e.state_valid) {
      snprintf(b, 4, "%02X", (unsigned)e.state);
    } else {
      snprintf(b, 4, "--");
    }
    return b;
  }
  bool fire() {
    auto_n++;
    return true;
  }
  void reset_schedule() {
    for (uint8_t i = 0; i < COARSE_COUNT; i++) {
      coarse_done[i] = false;
    }
    for (uint8_t k = 0; k < TAIL_COUNT; k++) {
      tail_done[k] = false;
    }
    dense_started = false;
    dense_done = false;
    last_dense = 0;
    auto_n = 0;
  }

  Entry entries[MAX_RESULTS];
  uint8_t first = 0;
  uint8_t count = 0;
  Entry pending;
  bool pending_active = false;

  bool auto_on = false;
  uint8_t auto_n = 0;
  uint8_t seq_seen = 0;
  bool coarse_done[COARSE_COUNT] = {false};
  bool tail_done[TAIL_COUNT] = {false};
  bool dense_started = false;
  bool dense_done = false;
  uint32_t last_dense = 0;
};

#endif
