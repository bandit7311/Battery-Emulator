#include <gtest/gtest.h>

#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "../../Software/src/battery/RENAULT-TWINGO-GEN1-BATTERY.h"
#include "../../Software/src/datalayer/datalayer.h"

#include "Arduino.h"

// TX frame capture injected by the emulated CAN layer (see emul/can.cpp).
void clear_transmitted_frames();
const std::vector<CAN_frame>& get_transmitted_frames();

namespace {

// Test double: replaces the three hooks that touch WiFi / NTP / the system clock.
class TestTwingo : public RenaultTwingoGen1Battery {
 public:
  bool network_up = false;
  int ntp_starts = 0;
  bool clock_set = false;
  uint32_t clock_secs = 0;

  bool network_ready() override { return network_up; }
  void start_ntp() override { ntp_starts++; }
  bool get_wall_clock_seconds_of_day(uint32_t& secs) override {
    if (!clock_set) {
      return false;
    }
    secs = clock_secs;
    return true;
  }
};

struct Tx {
  uint64_t t;
  CAN_frame f;
};

// Runs one transmit_can() call at time t and returns what went on the wire during it.
std::vector<CAN_frame> tick(RenaultTwingoGen1Battery& b, uint64_t t) {
  set_millis64(t);
  clear_transmitted_frames();
  b.transmit_can((unsigned long)t);
  return get_transmitted_frames();
}

// Same, but appends to a timeline (with the time of each tick) so the whole sleep run can be inspected.
void tick_log(RenaultTwingoGen1Battery& b, uint64_t t, std::vector<Tx>& log) {
  for (const CAN_frame& f : tick(b, t)) {
    log.push_back({t, f});
  }
}

std::vector<CAN_frame> with_id(const std::vector<CAN_frame>& v, uint32_t id) {
  std::vector<CAN_frame> out;
  for (const CAN_frame& f : v) {
    if (f.ID == id && !f.ext_ID) {
      out.push_back(f);
    }
  }
  return out;
}

std::vector<Tx> with_id(const std::vector<Tx>& v, uint32_t id) {
  std::vector<Tx> out;
  for (const Tx& x : v) {
    if (x.f.ID == id && !x.f.ext_ID) {
      out.push_back(x);
    }
  }
  return out;
}

bool bytes_are(const CAN_frame& f, std::initializer_list<uint8_t> expected) {
  if (f.DLC != expected.size()) {
    return false;
  }
  uint8_t i = 0;
  for (uint8_t b : expected) {
    if (f.data.u8[i++] != b) {
      return false;
    }
  }
  return true;
}

uint32_t secs(uint32_t h, uint32_t m, uint32_t s) {
  return h * 3600 + m * 60 + s;
}

CAN_frame frame_424(int t_min_c, int t_max_c) {
  CAN_frame f = {};
  f.DLC = 8;
  f.ID = 0x424;
  f.data.u8[4] = (uint8_t)(t_min_c + 40);
  f.data.u8[5] = 96;  // SOH
  f.data.u8[6] = 0x55;  // heartbeat
  f.data.u8[7] = (uint8_t)(t_max_c + 40);
  return f;
}

CAN_frame frame_658(uint8_t byte4) {
  CAN_frame f = {};
  f.DLC = 8;
  f.ID = 0x658;
  f.data.u8[0] = 0x67;
  f.data.u8[4] = byte4;
  return f;
}

void reset_datalayer_temperatures() {
  datalayer.battery.status.temperature_sensors_valid_mask = 0;  // no pack sensors -> the 0x424 values are used
  datalayer.battery.status.temperature_min_dC = 0;
  datalayer.battery.status.temperature_max_dC = 0;
}

bool contains(const String& s, const char* needle) {
  return std::string(s.c_str()).find(needle) != std::string::npos;
}

}  // namespace

// ---------------------------------------------------------------------------
// 0x53B / 0x350 in normal operation
// ---------------------------------------------------------------------------

TEST(TwingoTimeFramesTests, ClockFrameCarriesRealTimeAndFixedDate) {
  TestTwingo b;
  b.setup();
  b.clock_set = true;
  b.clock_secs = secs(10, 43, 11);
  auto frames = tick(b, 1000);
  auto f53b = with_id(frames, 0x53B);
  ASSERT_EQ(f53b.size(), 1u);
  // 10:43:11 on the fixed date 15.03.2025 (Saturday): same layout as the vehicle's `50 AC 06 4B ..` frame.
  EXPECT_TRUE(bytes_are(f53b[0], {0x50, 0xAC, 0x06, 0x4B, 0x30, 0x7D}));
  auto f350 = with_id(frames, 0x350);
  ASSERT_EQ(f350.size(), 1u);
  EXPECT_TRUE(bytes_are(f350[0], {0xC3, 0x10, 0x15, 0x7F, 0x14, 0x14, 0x96, 0x45}));  // state C3, fixed age
}

TEST(TwingoTimeFramesTests, ClockFrameOncePerSecondAndRunFrameEvery100ms) {
  TestTwingo b;
  b.setup();
  b.clock_set = true;
  b.clock_secs = secs(0, 0, 0);
  std::vector<Tx> log;
  for (uint64_t t = 1000; t < 11000; t += 100) {
    tick_log(b, t, log);
  }
  EXPECT_EQ(with_id(log, 0x53B).size(), 10u);   // 1 Hz
  EXPECT_EQ(with_id(log, 0x350).size(), 100u);  // every 100 ms, like the vehicle
}

TEST(TwingoTimeFramesTests, ClockFieldsCoverMidnightAndEndOfDay) {
  TestTwingo b;
  b.setup();
  b.clock_set = true;
  b.clock_secs = secs(23, 59, 59);
  auto f = with_id(tick(b, 1000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_TRUE(bytes_are(f[0], {0xB8, 0xEC, 0x06, 0x7B, 0x30, 0x7D}));  // 23:59:59, matches the earlier calculation
  b.clock_secs = secs(0, 0, 0);
  f = with_id(tick(b, 2000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_TRUE(bytes_are(f[0], {0x00, 0x00, 0x06, 0x40, 0x30, 0x7D}));  // date stays 15.03.2025, no day roll-over
}

TEST(TwingoTimeFramesTests, FallbackClockStartsAtNoonAndRunsOn) {
  TestTwingo b;
  b.setup();
  b.clock_set = false;  // NTP has not delivered a time
  auto f = with_id(tick(b, 1000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_TRUE(bytes_are(f[0], {0x60, 0x00, 0x06, 0x40, 0x30, 0x7D}));  // 12:00:00
  f = with_id(tick(b, 2000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_TRUE(bytes_are(f[0], {0x60, 0x00, 0x06, 0x41, 0x30, 0x7D}));  // 12:00:01, the clock runs on
  // 65 s after the start
  std::vector<CAN_frame> last;
  for (uint64_t t = 2100; t <= 66000; t += 100) {
    auto fr = with_id(tick(b, t), 0x53B);
    if (!fr.empty()) {
      last = fr;
    }
  }
  ASSERT_EQ(last.size(), 1u);
  EXPECT_TRUE(bytes_are(last[0], {0x60, 0x04, 0x06, 0x45, 0x30, 0x7D}));  // 12:01:05
}

TEST(TwingoTimeFramesTests, RealTimeReplacesTheFallbackAsSoonAsItIsAvailable) {
  TestTwingo b;
  b.setup();
  auto f = with_id(tick(b, 1000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_EQ(f[0].data.u8[0], 0x60);  // fallback 12:00
  b.clock_set = true;
  b.clock_secs = secs(7, 5, 9);
  f = with_id(tick(b, 2000), 0x53B);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_TRUE(bytes_are(f[0], {0x38, 0x14, 0x06, 0x49, 0x30, 0x7D}));  // 07:05:09
}

TEST(TwingoTimeFramesTests, NtpIsStartedOnceAsSoonAsTheNetworkIsUp) {
  TestTwingo b;
  b.setup();
  b.network_up = false;
  for (uint64_t t = 1000; t <= 5000; t += 100) {
    tick(b, t);
  }
  EXPECT_EQ(b.ntp_starts, 0);  // no WiFi yet
  b.network_up = true;         // WiFi comes up late, after the boot
  for (uint64_t t = 5100; t <= 12000; t += 100) {
    tick(b, t);
  }
  EXPECT_EQ(b.ntp_starts, 1);  // started once, not on every second
}

// ---------------------------------------------------------------------------
// Sleep / wake sequence: fixed 0x350 age, 0x53B follows the other own frames
// ---------------------------------------------------------------------------

TEST(TwingoTimeFramesTests, SleepAndWakeSequenceUseTheFixedVehicleAge) {
  reset_datalayer_temperatures();
  TestTwingo b;
  b.setup();
  b.clock_set = true;
  b.clock_secs = secs(12, 0, 0);
  std::vector<Tx> log;
  uint64_t t = 1000;
  for (; t < 5000; t += 100) {
    tick_log(b, t, log);
  }
  // Normal operation so far: the "C3" run frame every 100 ms.
  ASSERT_FALSE(with_id(log, 0x350).empty());
  for (const Tx& x : with_id(log, 0x350)) {
    EXPECT_TRUE(bytes_are(x.f, {0xC3, 0x10, 0x15, 0x7F, 0x14, 0x14, 0x96, 0x45}));
  }

  // A plausible temperature frame ends the boot phase of the temperature filter before the sleep run.
  b.handle_incoming_can_frame(frame_424(20, 22));
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.temperature_max_dC, 220);

  // Start "Sleep" and run through the whole shutdown sequence (66 + 60 + 10 + 1 s) into the silence.
  std::vector<Tx> sleep_log;
  b.request_sleep();
  uint64_t sleep_start = t;
  for (; t < sleep_start + 160000; t += 100) {
    tick_log(b, t, sleep_log);
  }
  auto f350 = with_id(sleep_log, 0x350);
  ASSERT_FALSE(f350.empty());
  bool seen_c3 = false, seen_c2 = false, seen_c0 = false, seen_00 = false;
  uint64_t t_first_00 = 0, t_last_350 = 0;
  for (const Tx& x : f350) {
    // Every 0x350 frame of the sequence carries the same fixed age in bytes 1-3, no counter, no "26 64".
    EXPECT_EQ(x.f.data.u8[1], 0x10);
    EXPECT_EQ(x.f.data.u8[2], 0x15);
    EXPECT_EQ(x.f.data.u8[3], 0x7F);
    uint8_t s = x.f.data.u8[0];
    seen_c3 |= (s == 0xC3);
    seen_c2 |= (s == 0xC2);
    seen_c0 |= (s == 0xC0);
    if (s == 0x00 && !seen_00) {
      seen_00 = true;
      t_first_00 = x.t;
    }
    t_last_350 = x.t;
  }
  EXPECT_TRUE(seen_c3 && seen_c2 && seen_c0 && seen_00);
  // The C3 stage of the sequence has bytes 5/7 = 14/45, exactly like the run frame; no second stream next to
  // it: about 10 frames per second during the C3 stage (the run frame is suppressed while the sequence runs).
  uint32_t c3_frames = 0;
  for (const Tx& x : f350) {
    if (x.f.data.u8[0] == 0xC3) {
      c3_frames++;
    }
  }
  EXPECT_LE(c3_frames, 66u * 10u + 12u);
  EXPECT_GE(c3_frames, 66u * 10u - 12u);

  // 0x53B keeps running through C3/C2/C0 like the other own frames, stops with them at the "00" stage.
  auto f53b = with_id(sleep_log, 0x53B);
  ASSERT_FALSE(f53b.empty());
  for (const Tx& x : f53b) {
    EXPECT_LT(x.t, t_first_00) << "0x53B must stop together with the other own frames at stage 00";
  }
  EXPECT_GE(f53b.size(), 120u);  // ~1 Hz for the ~136 s before stage 00
  // Silence afterwards: nothing at all on the bus.
  for (const Tx& x : sleep_log) {
    EXPECT_LE(x.t, t_last_350 + 0) << "frame after the last 0x350 of the sequence: ID 0x" << std::hex << x.f.ID;
  }

  // Wake up: first the burst (C0 once, then C3 x10) with the same fixed age, while 0x53B is back at once.
  std::vector<Tx> wake_log;
  b.request_wake_up();
  uint64_t wake_start = t;
  for (; t < wake_start + 3000; t += 100) {
    tick_log(b, t, wake_log);
  }
  // The burst is the first C0 frame plus the ten C3 frames after it; the run frame (every 100 ms) only
  // comes back once the burst has finished, so take exactly those 11 frames.
  std::vector<Tx> burst;
  {
    bool started = false;
    for (const Tx& x : with_id(wake_log, 0x350)) {
      if (!started && x.f.data.u8[0] == 0xC0) {
        started = true;
      }
      if (started && burst.size() < 11) {
        burst.push_back(x);
      }
    }
  }
  ASSERT_EQ(burst.size(), 11u);
  EXPECT_TRUE(bytes_are(burst[0].f, {0xC0, 0x10, 0x15, 0x7F, 0x14, 0x70, 0x96, 0x85}));
  for (size_t i = 1; i < burst.size(); i++) {
    EXPECT_EQ(burst[i].f.data.u8[0], 0xC3);
    EXPECT_EQ(burst[i].f.data.u8[1], 0x10);
    EXPECT_EQ(burst[i].f.data.u8[2], 0x15);
    EXPECT_EQ(burst[i].f.data.u8[3], 0x7F);
  }
  auto wake53b = with_id(wake_log, 0x53B);
  ASSERT_FALSE(wake53b.empty());
  EXPECT_LE(wake53b.front().t, wake_start + 1200);

  // After the burst the normal "C3" run frame is back, every 100 ms.
  std::vector<Tx> after_log;
  for (uint64_t end = t + 4000; t < end; t += 100) {
    tick_log(b, t, after_log);
  }
  auto run350 = with_id(after_log, 0x350);
  ASSERT_GE(run350.size(), 38u);
  ASSERT_LE(run350.size(), 41u);
  for (const Tx& x : run350) {
    EXPECT_TRUE(bytes_are(x.f, {0xC3, 0x10, 0x15, 0x7F, 0x14, 0x14, 0x96, 0x45}));
  }

  // After the wake-up the BMS behaves like after a boot: the temperature filter is armed again, an implausible
  // frame is dropped (the last accepted values stay), the first plausible one ends the boot phase again.
  b.handle_incoming_can_frame(frame_424(-40, 215));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 220);
  b.handle_incoming_can_frame(frame_424(21, 23));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 230);
  b.handle_incoming_can_frame(frame_424(21, 75));  // genuine over-temperature after the boot phase
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 750);
}

// ---------------------------------------------------------------------------
// Temperature boot filter (0x424)
// ---------------------------------------------------------------------------

TEST(TwingoTimeFramesTests, ImplausibleTemperaturesAreDroppedInTheBootPhase) {
  reset_datalayer_temperatures();
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  b.handle_incoming_can_frame(frame_424(-40, 215));  // raw 0 / 255, as right after a power cycle
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 0);
  EXPECT_EQ(datalayer.battery.status.temperature_min_dC, 0);
  b.handle_incoming_can_frame(frame_424(-40, 215));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 0);
}

TEST(TwingoTimeFramesTests, FirstPlausibleFrameEndsTheBootPhaseAndValuesPassAfterwards) {
  reset_datalayer_temperatures();
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  b.handle_incoming_can_frame(frame_424(20, 22));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_min_dC, 200);
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 220);
  // Boot phase over: a genuine over-temperature is no longer filtered.
  b.handle_incoming_can_frame(frame_424(20, 75));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 750);
  // The limits themselves are inclusive.
  reset_datalayer_temperatures();
  RenaultTwingoGen1Battery c;
  c.setup();
  c.handle_incoming_can_frame(frame_424(-20, 60));
  c.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_min_dC, -200);
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 600);
}

TEST(TwingoTimeFramesTests, BootPhaseEndsAfter60SecondsEvenWithoutAPlausibleFrame) {
  reset_datalayer_temperatures();
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  b.handle_incoming_can_frame(frame_424(-40, 215));  // first frame at 1000 ms starts the 60 s
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 0);
  set_millis64(1000 + 59999);
  b.handle_incoming_can_frame(frame_424(-40, 215));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 0);  // still inside the 60 s
  set_millis64(1000 + 60000);
  b.handle_incoming_can_frame(frame_424(-40, 215));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.temperature_max_dC, 2150);  // now taken over unfiltered
}

// ---------------------------------------------------------------------------
// New values on "More Battery Info"
// ---------------------------------------------------------------------------

TEST(TwingoTimeFramesTests, Soh658IsShownAsCandidate) {
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  EXPECT_TRUE(contains(b.get_uds_info_html(), "SOH candidate (0x658 byte 4): not received"));
  b.handle_incoming_can_frame(frame_658(0x5F));
  EXPECT_TRUE(contains(b.get_uds_info_html(), "SOH candidate (0x658 byte 4): 95 %"));
  b.handle_incoming_can_frame(frame_658(0x60));
  EXPECT_TRUE(contains(b.get_uds_info_html(), "SOH candidate (0x658 byte 4): 96 %"));
  b.handle_incoming_can_frame(frame_658(0x7F));
  EXPECT_TRUE(contains(b.get_uds_info_html(), "SOH candidate (0x658 byte 4): invalid (127)"));
  // The top bit is masked away like in OVMS (0xE0 & 0x7F = 96).
  b.handle_incoming_can_frame(frame_658(0xE0));
  EXPECT_TRUE(contains(b.get_uds_info_html(), "SOH candidate (0x658 byte 4): 96 %"));
}

TEST(TwingoTimeFramesTests, Soh658IsNotUsedForTheDatalayerSoh) {
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  b.handle_incoming_can_frame(frame_424(20, 22));  // SOH from 0x424 = 96
  b.update_values();
  uint16_t soh_before = datalayer.battery.status.soh_pptt;
  b.handle_incoming_can_frame(frame_658(0x5F));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.soh_pptt, soh_before);
  EXPECT_EQ(soh_before, 9600);
}

TEST(TwingoTimeFramesTests, TimePidsAreShownRaw) {
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  String html = b.get_uds_info_html();
  EXPECT_TRUE(contains(html, "Time (0x9261): not yet read"));
  EXPECT_TRUE(contains(html, "Pack time (0x91C1): not yet read"));
  // Real reply of the pack: 06 62 91 C1 0D EE 55 (three data bytes).
  CAN_frame f = {};
  f.ext_ID = true;
  f.DLC = 8;
  f.ID = 0x18DAF1DB;
  const uint8_t reply[8] = {0x06, 0x62, 0x91, 0xC1, 0x0D, 0xEE, 0x55, 0xAA};
  memcpy(f.data.u8, reply, 8);
  b.handle_incoming_can_frame(f);
  html = b.get_uds_info_html();
  EXPECT_TRUE(contains(html, "Pack time (0x91C1): 0DEE55 (912981)"));
  EXPECT_TRUE(contains(html, "Time (0x9261): not yet read"));
  // A two-byte reply for 0x9261.
  const uint8_t reply2[8] = {0x05, 0x62, 0x92, 0x61, 0x01, 0x2C, 0xAA, 0xAA};
  memcpy(f.data.u8, reply2, 8);
  b.handle_incoming_can_frame(f);
  html = b.get_uds_info_html();
  EXPECT_TRUE(contains(html, "Time (0x9261): 012C (300)"));
}

TEST(TwingoTimeFramesTests, TimePidsAreInThePollList) {
  set_millis64(1000);
  RenaultTwingoGen1Battery b;
  b.setup();
  std::set<uint16_t> pids;
  uint32_t requests = 0;
  for (uint64_t t = 1000; t < 1000 + 30000; t += 100) {
    for (const CAN_frame& f : tick(b, t)) {
      if (f.ext_ID && f.ID == 0x18DADBF1 && f.data.u8[0] == 0x03 && f.data.u8[1] == 0x22) {
        pids.insert((uint16_t)((f.data.u8[2] << 8) | f.data.u8[3]));
        requests++;
      }
    }
  }
  EXPECT_TRUE(pids.count(0x9261) == 1);
  EXPECT_TRUE(pids.count(0x91C1) == 1);
  EXPECT_EQ(pids.size(), 119u);  // 96 cells + 23 others, one request every 200 ms
  EXPECT_GE(requests, 119u);
}


// ---------------------------------------------------------------------------
// 0x090 (10 ms) and 0x242 (20 ms) with counter and CRC
// ---------------------------------------------------------------------------

namespace {

// Independent bitwise CRC-8 (poly 0x1D, start 0), not the table of the driver.
uint8_t crc8_ref(const std::vector<uint8_t>& d, uint8_t xor_out) {
  uint8_t c = 0;
  for (uint8_t b : d) {
    c ^= b;
    for (int i = 0; i < 8; i++) {
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x1D) : (uint8_t)(c << 1);
    }
  }
  return (uint8_t)(c ^ xor_out);
}

std::vector<uint8_t> bytes_of(const CAN_frame& f) {
  return std::vector<uint8_t>(f.data.u8, f.data.u8 + f.DLC);
}

// Runs 1 ms steps like the core loop does and collects every frame.
void run_ms(RenaultTwingoGen1Battery& b, uint64_t& t, uint64_t ms, std::vector<Tx>& log) {
  for (uint64_t end = t + ms; t < end; t++) {
    tick_log(b, t, log);
  }
}

}  // namespace

TEST(TwingoFastFramesTests, CrcAlgorithmMatchesRealVehicleFrames) {
  // Real frames from Log_Twingo_Ladung.log / canmitlog.log: 0x090 has the CRC in byte 3 (over the other six
  // bytes, XOR 0xF6), 0x242 in byte 7 (over the first seven, XOR 0x0A).
  const std::vector<std::vector<uint8_t>> real090 = {{0x00, 0xFF, 0xE7, 0xB4, 0xF0, 0x7F, 0xE0},
                                                     {0x00, 0xFF, 0xE8, 0xDC, 0xF0, 0x7F, 0xE0},
                                                     {0x01, 0xFF, 0xEE, 0x3E, 0xF0, 0x7F, 0xF0},
                                                     {0x00, 0xFF, 0xE9, 0x71, 0xD0, 0x7F, 0xE0}};
  for (const auto& f : real090) {
    std::vector<uint8_t> in = {f[0], f[1], f[2], f[4], f[5], f[6]};
    EXPECT_EQ(crc8_ref(in, 0xF6), f[3]);
  }
  const std::vector<std::vector<uint8_t>> real242 = {{0x00, 0x30, 0xFF, 0xEF, 0xFE, 0x00, 0x0D, 0x5A},
                                                     {0x00, 0x78, 0xFF, 0xEF, 0xFE, 0x00, 0x0D, 0x00},
                                                     {0x00, 0x00, 0xFF, 0xEF, 0xFE, 0x00, 0x0D, 0x66},
                                                     {0x00, 0x68, 0xFF, 0xEF, 0xFE, 0x00, 0x0D, 0x14}};
  for (const auto& f : real242) {
    std::vector<uint8_t> in(f.begin(), f.begin() + 7);
    EXPECT_EQ(crc8_ref(in, 0x0A), f[7]);
  }
}

TEST(TwingoFastFramesTests, Frame090Every10msWithCounterAndCrc) {
  reset_datalayer_temperatures();
  TestTwingo b;
  b.setup();
  std::vector<Tx> log;
  uint64_t t = 1000;
  run_ms(b, t, 1000, log);
  auto f = with_id(log, 0x090);
  ASSERT_GE(f.size(), 99u);
  ASSERT_LE(f.size(), 101u);
  for (size_t i = 0; i < f.size(); i++) {
    const CAN_frame& fr = f[i].f;
    ASSERT_EQ(fr.DLC, 7);
    EXPECT_EQ(fr.data.u8[0], 0x00);
    EXPECT_EQ(fr.data.u8[1], 0xFF);
    EXPECT_EQ(fr.data.u8[2] & 0xF0, 0xE0);
    EXPECT_EQ(fr.data.u8[4], 0xF0);
    EXPECT_EQ(fr.data.u8[5], 0x7F);
    EXPECT_EQ(fr.data.u8[6], 0xF0);
    std::vector<uint8_t> in = {fr.data.u8[0], fr.data.u8[1], fr.data.u8[2], fr.data.u8[4], fr.data.u8[5], fr.data.u8[6]};
    EXPECT_EQ(crc8_ref(in, 0xF6), fr.data.u8[3]) << "frame " << i;
    if (i > 0) {
      EXPECT_EQ((f[i].f.data.u8[2] & 0x0F), ((f[i - 1].f.data.u8[2] & 0x0F) + 1) & 0x0F);  // counter +1, wraps 15 -> 0
      EXPECT_EQ(f[i].t - f[i - 1].t, 10u);
    }
  }
}

TEST(TwingoFastFramesTests, Frame242Every20msWithCounterAndCrc) {
  reset_datalayer_temperatures();
  TestTwingo b;
  b.setup();
  std::vector<Tx> log;
  uint64_t t = 1000;
  run_ms(b, t, 1000, log);
  auto f = with_id(log, 0x242);
  ASSERT_GE(f.size(), 49u);
  ASSERT_LE(f.size(), 51u);
  for (size_t i = 0; i < f.size(); i++) {
    const CAN_frame& fr = f[i].f;
    ASSERT_EQ(fr.DLC, 8);
    EXPECT_EQ(fr.data.u8[0], 0x00);
    EXPECT_EQ(fr.data.u8[1] & 0x07, 0x00);  // counter sits in bits 6:3
    EXPECT_EQ(fr.data.u8[2], 0xFF);
    EXPECT_EQ(fr.data.u8[3], 0xEF);
    EXPECT_EQ(fr.data.u8[4], 0xFE);
    EXPECT_EQ(fr.data.u8[5], 0x00);
    EXPECT_EQ(fr.data.u8[6], 0x0D);
    EXPECT_EQ(crc8_ref(std::vector<uint8_t>(fr.data.u8, fr.data.u8 + 7), 0x0A), fr.data.u8[7]) << "frame " << i;
    if (i > 0) {
      EXPECT_EQ((f[i].f.data.u8[1] >> 3), ((f[i - 1].f.data.u8[1] >> 3) + 1) & 0x0F);  // counter +1, 0x78 -> 0x00
      EXPECT_EQ(f[i].t - f[i - 1].t, 20u);
    }
  }
}

TEST(TwingoFastFramesTests, StopInC0StageAndInSilenceAndStartWithFirstC3OfTheWakeBurst) {
  reset_datalayer_temperatures();
  TestTwingo b;
  b.setup();
  b.clock_set = true;
  b.clock_secs = secs(12, 0, 0);
  std::vector<Tx> log;
  uint64_t t = 1000;
  run_ms(b, t, 3000, log);
  ASSERT_FALSE(with_id(log, 0x090).empty());
  ASSERT_FALSE(with_id(log, 0x242).empty());

  // Sleep: the fast frames run in the C3 and C2 stage and stop with the C0 stage.
  std::vector<Tx> sleep_log;
  b.request_sleep();
  uint64_t sleep_start = t;
  run_ms(b, t, 140000, sleep_log);
  uint64_t t_c0 = 0, t_c2 = 0;
  for (const Tx& x : with_id(sleep_log, 0x350)) {
    if (x.f.data.u8[0] == 0xC2 && t_c2 == 0) {
      t_c2 = x.t;
    }
    if (x.f.data.u8[0] == 0xC0 && t_c0 == 0) {
      t_c0 = x.t;
    }
  }
  ASSERT_GT(t_c2, sleep_start);
  ASSERT_GT(t_c0, t_c2);
  uint64_t last090 = 0, last242 = 0, in_c2_090 = 0, in_c3_090 = 0;
  for (const Tx& x : with_id(sleep_log, 0x090)) {
    last090 = x.t;
    (x.t < t_c2 ? in_c3_090 : in_c2_090)++;
  }
  for (const Tx& x : with_id(sleep_log, 0x242)) {
    last242 = x.t;
  }
  EXPECT_GT(in_c3_090, 6000u);  // 66 s at 10 ms
  EXPECT_GT(in_c2_090, 5500u);  // 60 s at 10 ms
  EXPECT_LE(last090, t_c0 + 2u) << "0x090 must stop with the C0 stage";
  EXPECT_LE(last242, t_c0 + 2u) << "0x242 must stop with the C0 stage";

  // Wake up: nothing before the initial C0 frame of the burst; the fast frames start after it.
  std::vector<Tx> wake_log;
  b.request_wake_up();
  uint64_t wake_start = t;
  run_ms(b, t, 2600, wake_log);
  uint64_t t_first_c0 = 0, t_first_c3 = 0;
  for (const Tx& x : with_id(wake_log, 0x350)) {
    if (x.f.data.u8[0] == 0xC0 && t_first_c0 == 0) {
      t_first_c0 = x.t;
    }
    if (x.f.data.u8[0] == 0xC3 && t_first_c3 == 0) {
      t_first_c3 = x.t;
    }
  }
  ASSERT_GT(t_first_c0, 0u);
  auto w090 = with_id(wake_log, 0x090);
  auto w242 = with_id(wake_log, 0x242);
  ASSERT_FALSE(w090.empty());
  ASSERT_FALSE(w242.empty());
  EXPECT_GE(w090.front().t, t_first_c0) << "0x090 must not start before the initial C0 frame";
  EXPECT_GE(w242.front().t, t_first_c0) << "0x242 must not start before the initial C0 frame";
  EXPECT_LE(w090.front().t, t_first_c3);
  (void)wake_start;
}

TEST(TwingoFastFramesTests, NothingInTrueSilence) {
  reset_datalayer_temperatures();
  TestTwingo b;
  b.setup();
  std::vector<Tx> log;
  uint64_t t = 1000;
  run_ms(b, t, 2000, log);
  std::vector<Tx> sleep_log;
  b.request_sleep();
  run_ms(b, t, 141000, sleep_log);  // through the whole sequence (137 s) into the silence
  uint64_t t_last_350 = 0;
  for (const Tx& x : with_id(sleep_log, 0x350)) {
    t_last_350 = x.t;
  }
  ASSERT_GT(t_last_350, 0u);
  for (const Tx& x : sleep_log) {
    if (x.t > t_last_350 + 5) {
      ADD_FAILURE() << "frame 0x" << std::hex << x.f.ID << " in true silence at t=" << std::dec << x.t;
      break;
    }
  }
}
