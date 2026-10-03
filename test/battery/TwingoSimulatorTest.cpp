#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "../../Software/src/battery/RENAULT-TWINGO-GEN1-BATTERY.h"
#include "../../Software/src/datalayer/datalayer.h"
#include "../../Software/src/datalayer/datalayer_extended.h"
#include "../../Software/src/devboard/webserver/simulator_html.h"

#include "Arduino.h"

// TX frame capture injected by the emulated CAN layer (see emul/can.cpp).
void clear_transmitted_frames();
const std::vector<CAN_frame>& get_transmitted_frames();

// Tests of the 03.10. changes: vehicle age from the clock, 0x350 C7, switchable simulator rows, 0x42E / 0x18A /
// 0x1F8 content, free read request, fault counters (19 14), DTC details (19 06).

namespace {

class TestTwingo : public RenaultTwingoGen1Battery {
 public:
  bool unix_set = false;
  time_t unix_now = 0;
  bool clock_set = true;
  uint32_t clock_secs = 12 * 3600;
  bool network_ready() override { return false; }
  void start_ntp() override {}
  bool get_unix_time(time_t& now_utc) override {
    if (!unix_set) {
      return false;
    }
    now_utc = unix_now;
    return true;
  }
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

const time_t AGE_EPOCH = 1613387588;  // 15.02.2021 11:13:08 UTC = zero of the 0x350 vehicle age counter

std::vector<CAN_frame> tick(RenaultTwingoGen1Battery& b, uint64_t t) {
  set_millis64(t);
  clear_transmitted_frames();
  b.transmit_can((unsigned long)t);
  return get_transmitted_frames();
}

void run(RenaultTwingoGen1Battery& b, uint64_t& t, uint64_t ms, uint64_t step, std::vector<Tx>& log) {
  for (uint64_t end = t + ms; t < end; t += step) {
    for (const CAN_frame& f : tick(b, t)) {
      log.push_back({t, f});
    }
  }
}

std::vector<Tx> with_id(const std::vector<Tx>& v, uint32_t id, bool ext = false) {
  std::vector<Tx> out;
  for (const Tx& x : v) {
    if (x.f.ID == id && x.f.ext_ID == ext) {
      out.push_back(x);
    }
  }
  return out;
}

int row_of(uint32_t id) {
  for (int i = 0; i < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; i++) {
    if (RenaultTwingoGen1Battery::sim_signals[i].id == id) {
      return i;
    }
  }
  return -1;
}

void set_mask(uint32_t mask) {
  datalayer_extended.twingoGen1.simulator_enabled_mask = mask;
}

// CRC-8 SAE J1850 (poly 0x1D, start 0xFF, final XOR 0xFF), bit by bit - independent of the driver's table.
uint8_t crc_j1850(const std::vector<uint8_t>& d) {
  uint8_t crc = 0xFF;
  for (uint8_t b : d) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
    }
  }
  return (uint8_t)(crc ^ 0xFF);
}

CAN_frame reply_frame(std::initializer_list<uint8_t> bytes) {
  CAN_frame f = {};
  f.FD = false;
  f.ext_ID = true;
  f.ID = 0x18DAF1DB;
  f.DLC = 8;
  int i = 0;
  for (uint8_t v : bytes) {
    f.data.u8[i++] = v;
  }
  return f;
}


// Single-frame positive reply 62 <DID> <data...> as the LBC sends it.
CAN_frame did_reply(uint16_t did, std::initializer_list<uint8_t> data_bytes) {
  CAN_frame f = {};
  f.FD = false;
  f.ext_ID = true;
  f.ID = 0x18DAF1DB;
  f.DLC = 8;
  f.data.u8[0] = (uint8_t)(3 + data_bytes.size());
  f.data.u8[1] = 0x62;
  f.data.u8[2] = (uint8_t)(did >> 8);
  f.data.u8[3] = (uint8_t)(did & 0xFF);
  int i = 4;
  for (uint8_t v : data_bytes) {
    f.data.u8[i++] = v;
  }
  return f;
}

bool contains(const String& s, const char* needle) {
  return std::string(s.c_str()).find(needle) != std::string::npos;
}

// The two "DTC details" lines of the page (text after the button). The host emulation of String does not print
// String(x, HEX) as hex (the page label of the code looks odd there, on the ESP32 it is fine), so the tests
// look at the answer texts only.
std::string detail_lines(const String& html) {
  std::string h(html.c_str());
  size_t p = h.find("subfunction 0x06)</button><br>");
  if (p == std::string::npos) {
    return "";
  }
  return h.substr(p, 260);
}

// Request frames on 0x18DADBF1 (the driver's own TX of the diagnostic channel).
std::vector<CAN_frame> diag_tx() {
  std::vector<CAN_frame> out;
  for (const CAN_frame& f : get_transmitted_frames()) {
    if (f.ID == 0x18DADBF1 && f.ext_ID) {
      out.push_back(f);
    }
  }
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Simulator table
// ---------------------------------------------------------------------------

TEST(TwingoSimulatorTable, TenIRowsInTheDocumentedOrder) {
  const uint32_t expected_i[10] = {0x090, 0x242, 0x350, 0x19F, 0x426, 0x436, 0x423, 0x69F, 0x53B, 0x214};
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(RenaultTwingoGen1Battery::sim_signals[i].id, expected_i[i]) << "row " << i;
    EXPECT_EQ(RenaultTwingoGen1Battery::sim_signals[i].tag, 'I') << "row " << i;
  }
  for (int i = 10; i < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; i++) {
    EXPECT_NE(RenaultTwingoGen1Battery::sim_signals[i].tag, 'I') << "row " << i;
  }
}

TEST(TwingoSimulatorTable, OnlyTheFourZoeFramesAreMarkedX) {
  int x_count = 0;
  for (int i = 0; i < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; i++) {
    const auto& s = RenaultTwingoGen1Battery::sim_signals[i];
    bool expect_x = (s.id == 0x19F || s.id == 0x426 || s.id == 0x436 || s.id == 0x423);
    EXPECT_EQ(s.not_in_vehicle_log, expect_x) << "0x" << std::hex << s.id;
    x_count += s.not_in_vehicle_log ? 1 : 0;
  }
  EXPECT_EQ(x_count, 4);
}

TEST(TwingoSimulatorTable, EveryRowHasSenderAndMeaningText) {
  for (int i = 0; i < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; i++) {
    const auto& s = RenaultTwingoGen1Battery::sim_signals[i];
    ASSERT_NE(s.sender, nullptr);
    ASSERT_NE(s.info, nullptr);
    EXPECT_GT(strlen(s.sender), 0u) << "0x" << std::hex << s.id;
    EXPECT_GT(strlen(s.info), 10u) << "0x" << std::hex << s.id;
  }
}

TEST(TwingoSimulatorTable, SenderIsOnlyNamedWhereASourceExists) {
  // EVC for 0x1F8 / 0x18A / 0x186 etc. comes from the CanZE table; ids without an entry there say "unknown".
  EXPECT_STREQ(RenaultTwingoGen1Battery::sim_signals[row_of(0x186)].sender, "EVC (CanZE)");
  EXPECT_STREQ(RenaultTwingoGen1Battery::sim_signals[row_of(0x211)].sender, "unknown");
  EXPECT_STREQ(RenaultTwingoGen1Battery::sim_signals[row_of(0x350)].sender, "unknown");
}

// ---------------------------------------------------------------------------
// Vehicle age (0x350 bytes 1-3) and the C7 run frame
// ---------------------------------------------------------------------------

TEST(TwingoVehicleAge, EpochMatchesBothVehicleLogs) {
  // Counter values and the UTC time of their first frame in the two real logs.
  // 02.10.2026 22:35:35 UTC -> 2959882, 02.10.2026 14:48:24 UTC -> 2959415.
  RenaultTwingoGen1Battery::SimSignal dummy = RenaultTwingoGen1Battery::sim_signals[0];
  (void)dummy;
  TestTwingo b;
  b.setup();
  b.unix_set = true;
  uint64_t t = 1000;
  std::vector<Tx> log;
  b.unix_now = 1790980535;  // 2026-10-02 22:35:35 UTC
  run(b, t, 200, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[1], 0x2D);
  EXPECT_EQ(f[0].f.data.u8[2], 0x2A);
  EXPECT_EQ(f[0].f.data.u8[3], 0x0A);  // 2959882
  b.unix_now = 1790952504;  // 2026-10-02 14:48:24 UTC
  log.clear();
  run(b, t, 200, 100, log);
  f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  uint32_t age = ((uint32_t)f[0].f.data.u8[1] << 16) | ((uint32_t)f[0].f.data.u8[2] << 8) | f[0].f.data.u8[3];
  EXPECT_EQ(age, 2959415u);
}

TEST(TwingoVehicleAge, TicksOncePerMinute) {
  TestTwingo b;
  b.setup();
  b.unix_set = true;
  b.unix_now = AGE_EPOCH + 2959882LL * 60 + 59;
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 100, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f.back().f.data.u8[3], 0x0A);
  b.unix_now += 1;  // the next minute starts
  log.clear();
  run(b, t, 100, 100, log);
  f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f.back().f.data.u8[3], 0x0B);
}

TEST(TwingoVehicleAge, FallbackStartsAtTheBuildDateValueAndCountsMinutes) {
  TestTwingo b;
  b.setup();
  b.unix_set = false;  // no NTP time yet
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 200, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  // 2960506 = 0x2D2C7A
  EXPECT_EQ(f[0].f.data.u8[1], 0x2D);
  EXPECT_EQ(f[0].f.data.u8[2], 0x2C);
  EXPECT_EQ(f[0].f.data.u8[3], 0x7A);
  t = 1000 + 61000;  // one minute and one second after the first frame
  log.clear();
  run(b, t, 100, 100, log);
  f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f.back().f.data.u8[3], 0x7B);
}

TEST(TwingoVehicleAge, RealTimeReplacesTheFallbackAsSoonAsItIsAvailable) {
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 200, 100, log);
  EXPECT_EQ(with_id(log, 0x350).back().f.data.u8[3], 0x7A);
  b.unix_set = true;
  b.unix_now = AGE_EPOCH + 2959882LL * 60 + 3;
  log.clear();
  run(b, t, 200, 100, log);
  EXPECT_EQ(with_id(log, 0x350).back().f.data.u8[3], 0x0A);
}

TEST(TwingoVehicleAge, TimeBeforeTheEpochIsNotUsed) {
  TestTwingo b;
  b.setup();
  b.unix_set = true;
  b.unix_now = AGE_EPOCH - 1000;  // implausible clock
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 200, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[3], 0x7A);  // fallback value
}

// ---------------------------------------------------------------------------
// I rows are real switches
// ---------------------------------------------------------------------------

namespace {
size_t count_id(const std::vector<Tx>& log, uint32_t id) {
  return with_id(log, id).size();
}
}  // namespace

TEST(TwingoSimulatorSwitches, AllNineSteadyIRowsRunByDefault) {
  set_mask(0x3FF);
  TestTwingo b;
  b.setup();
  b.unix_set = true;
  b.unix_now = AGE_EPOCH + 2959882LL * 60;
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 3000, 10, log);
  for (uint32_t id : {0x090u, 0x242u, 0x350u, 0x19Fu, 0x426u, 0x436u, 0x423u, 0x69Fu, 0x53Bu}) {
    EXPECT_GT(count_id(log, id), 0u) << "0x" << std::hex << id;
  }
  EXPECT_EQ(count_id(log, 0x214), 0u);  // only during the shutdown sequence
}

TEST(TwingoSimulatorSwitches, EachRowSwitchesExactlyItsOwnFrame) {
  const uint32_t ids[9] = {0x090, 0x242, 0x350, 0x19F, 0x426, 0x436, 0x423, 0x69F, 0x53B};
  for (int k = 0; k < 9; k++) {
    int row = row_of(ids[k]);
    ASSERT_GE(row, 0);
    set_mask(0x3FF & ~(1u << row));
    TestTwingo b;
    b.setup();
    b.unix_set = true;
    b.unix_now = AGE_EPOCH + 2959882LL * 60;
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 3000, 10, log);
    EXPECT_EQ(count_id(log, ids[k]), 0u) << "0x" << std::hex << ids[k] << " must be off";
    for (int j = 0; j < 9; j++) {
      if (j != k) {
        EXPECT_GT(count_id(log, ids[j]), 0u) << "0x" << std::hex << ids[j] << " must still run while 0x" << ids[k]
                                             << " is off";
      }
    }
  }
  set_mask(0x3FF);
}

TEST(TwingoSimulatorSwitches, AllIRowsOffMeansNoVehicleFramesAtAll) {
  set_mask(0);
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 3000, 10, log);
  for (uint32_t id : {0x090u, 0x242u, 0x350u, 0x19Fu, 0x426u, 0x436u, 0x423u, 0x69Fu, 0x53Bu, 0x214u}) {
    EXPECT_EQ(count_id(log, id), 0u) << "0x" << std::hex << id;
  }
  set_mask(0x3FF);
}

TEST(TwingoSimulatorSwitches, SleepSequenceStillSendsItsOwn350EvenIfTheRunFrameIsOff) {
  set_mask(0x3FF & ~(1u << row_of(0x350)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1000, 100, log);
  EXPECT_EQ(count_id(log, 0x350), 0u);
  b.request_sleep();
  log.clear();
  run(b, t, 3000, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[0], 0xC3);  // C3 stage of the shutdown sequence
  set_mask(0x3FF);
}

TEST(TwingoSimulatorSwitches, Row214OnlyInShutdownAndItsSwitchWorks) {
  for (int on = 0; on < 2; on++) {
    set_mask(on ? 0x3FF : (0x3FF & ~(1u << row_of(0x214))));
    TestTwingo b;
    b.setup();
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 500, 100, log);
    b.request_sleep();
    run(b, t, 3000, 100, log);
    EXPECT_EQ(count_id(log, 0x214) > 0, on == 1) << "switch state " << on;
  }
  set_mask(0x3FF);
}

TEST(TwingoSimulatorSwitches, PlannedRowsStayOffByDefaultAndSwitchOnAndOff) {
  set_mask(0x3FF);
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1000, 10, log);
  EXPECT_EQ(count_id(log, 0x186), 0u);
  set_mask(0x3FF | (1u << row_of(0x186)));
  run(b, t, 1000, 10, log);
  EXPECT_GT(count_id(log, 0x186), 0u);
  size_t n = count_id(log, 0x186);
  set_mask(0x3FF);
  run(b, t, 1000, 10, log);
  EXPECT_EQ(count_id(log, 0x186), n);
}

// Every one of the 18 P/A rows (index 10-27) is its own on/off switch: ticked -> frames of exactly that ID with the
// interval of the table row, unticked -> nothing.
TEST(TwingoSimulatorSwitches, EveryPlannedAndAssumedRowSwitchesOnAndOff) {
  datalayer.battery.status.voltage_dV = 3840;  // 0x42E only sends while the pack voltage is known
  for (int row = 10; row < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; row++) {
    const auto& sig = RenaultTwingoGen1Battery::sim_signals[row];
    for (int on = 0; on < 2; on++) {
      set_mask(on ? (0x3FFu | (1u << row)) : 0x3FFu);
      TestTwingo b;
      b.setup();
      uint64_t t = 1000;
      std::vector<Tx> log;
      run(b, t, 2000, 1, log);
      size_t n = count_id(log, sig.id);
      if (on) {
        // about 2000 ms / interval frames (the table interval is the minimum spacing of the loop)
        EXPECT_GE(n, 2000u / sig.interval_ms - 2u) << "0x" << std::hex << sig.id << " row " << std::dec << row;
        EXPECT_LE(n, 2000u / sig.interval_ms + 2u) << "0x" << std::hex << sig.id << " row " << std::dec << row;
        for (const Tx& x : with_id(log, sig.id)) {
          EXPECT_EQ(x.f.DLC, sig.dlc) << "0x" << std::hex << sig.id;
        }
      } else {
        EXPECT_EQ(n, 0u) << "0x" << std::hex << sig.id << " row " << std::dec << row << " must be off";
      }
    }
  }
  set_mask(0x3FF);
  datalayer.battery.status.voltage_dV = 0;
}

// ---------------------------------------------------------------------------
// 0x42E: measured pack voltage
// ---------------------------------------------------------------------------

TEST(Twingo42E, CarriesTheMeasuredPackVoltageWithHalfVoltResolution) {
  set_mask(0x3FF | (1u << row_of(0x42E)));
  struct Case {
    uint16_t dV;
    uint8_t b3, b4;
  } cases[] = {
      {3390, 0x54, 0xC4},  // 339.0 V = real frame ff ff d0 54 c4 ..
      {3840, 0x60, 0x04},  // 384.0 V
      {3845, 0x60, 0x24},  // 384.5 V
      {3841, 0x60, 0x04},  // rounds to 384.0 V
      {3843, 0x60, 0x24},  // rounds to 384.5 V
  };
  for (const Case& c : cases) {
    TestTwingo b;
    b.setup();
    datalayer.battery.status.voltage_dV = c.dV;
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 300, 10, log);
    auto f = with_id(log, 0x42E);
    ASSERT_FALSE(f.empty()) << c.dV;
    EXPECT_EQ(f[0].f.data.u8[3], c.b3) << c.dV;
    EXPECT_EQ(f[0].f.data.u8[4], c.b4) << c.dV;
    // The OVMS RT32 decode of the real vehicle gives back the voltage.
    float v = (float)((((f[0].f.data.u8[3] << 8) | f[0].f.data.u8[4]) >> 5) & 0x3FF) / 2.0f;
    EXPECT_NEAR(v, c.dV / 10.0f, 0.3f) << c.dV;
    // The other bytes stay as in the table row.
    const auto& row = RenaultTwingoGen1Battery::sim_signals[row_of(0x42E)];
    for (int i : {0, 1, 2, 5, 6, 7}) {
      EXPECT_EQ(f[0].f.data.u8[i], row.data[i]) << "byte " << i;
    }
  }
  set_mask(0x3FF);
  datalayer.battery.status.voltage_dV = 0;
}

TEST(Twingo42E, NothingIsSentWhileThePackVoltageIsUnknown) {
  set_mask(0x3FF | (1u << row_of(0x42E)));
  datalayer.battery.status.voltage_dV = 0;
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1000, 10, log);
  EXPECT_EQ(count_id(log, 0x42E), 0u);
  datalayer.battery.status.voltage_dV = 3840;
  run(b, t, 500, 10, log);
  EXPECT_GT(count_id(log, 0x42E), 0u);
  set_mask(0x3FF);
  datalayer.battery.status.voltage_dV = 0;
}

TEST(Twingo42E, NeverUsesTheInvalidMarker) {
  set_mask(0x3FF | (1u << row_of(0x42E)));
  datalayer.battery.status.voltage_dV = 6000;  // 600 V, far above anything real: clamped, not 0x3FF
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 10, log);
  auto f = with_id(log, 0x42E);
  ASSERT_FALSE(f.empty());
  uint16_t raw = (((f[0].f.data.u8[3] << 8) | f[0].f.data.u8[4]) >> 5) & 0x3FF;
  EXPECT_EQ(raw, 0x3FEu);
  set_mask(0x3FF);
  datalayer.battery.status.voltage_dV = 0;
}

TEST(Twingo42E, StopsInTheSilenceOfTheSleepSequenceLikeTheOtherRows) {
  set_mask(0x3FF | (1u << row_of(0x42E)));
  datalayer.battery.status.voltage_dV = 3840;
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 500, 10, log);
  b.request_sleep();
  uint64_t sleep_start = t;
  std::vector<Tx> sleep_log;
  run(b, t, 150000, 100, sleep_log);
  uint64_t last = 0;
  for (const Tx& x : with_id(sleep_log, 0x42E)) {
    last = x.t;
  }
  EXPECT_GT(last, sleep_start);
  EXPECT_LT(last, sleep_start + 66000 + 60000 + 10000 + 1500);  // not after the C0 stage
  set_mask(0x3FF);
  datalayer.battery.status.voltage_dV = 0;
}

// ---------------------------------------------------------------------------
// 0x18A: counter and CRC
// ---------------------------------------------------------------------------

TEST(Twingo18A, ByteSixIsTheJ1850CrcOverTheOtherSevenBytes) {
  set_mask(0x3FF | (1u << row_of(0x18A)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1000, 10, log);
  auto f = with_id(log, 0x18A);
  ASSERT_GE(f.size(), 90u);
  for (const Tx& x : f) {
    std::vector<uint8_t> in = {x.f.data.u8[0], x.f.data.u8[1], x.f.data.u8[2], x.f.data.u8[3],
                               x.f.data.u8[4], x.f.data.u8[5], x.f.data.u8[7]};
    EXPECT_EQ(x.f.data.u8[6], crc_j1850(in));
  }
  // The counter in the high nibble of byte 7 steps by one per frame, the low nibble stays 0.
  for (size_t i = 1; i < f.size(); i++) {
    uint8_t prev = f[i - 1].f.data.u8[7] >> 4, cur = f[i].f.data.u8[7] >> 4;
    EXPECT_EQ((uint8_t)((prev + 1) & 0x0F), cur);
    EXPECT_EQ(f[i].f.data.u8[7] & 0x0F, 0);
  }
  set_mask(0x3FF);
}

TEST(Twingo18A, CrcMatchesRealVehicleFrames) {
  // Frames from canmitlog.log (02.10.): ff f0 00 06 40 3c d5 70, ff f0 00 06 40 3c aa 80, ff f0 00 06 40 3c 67 90
  const uint8_t real[3][8] = {{0xFF, 0xF0, 0x00, 0x06, 0x40, 0x3C, 0xD5, 0x70},
                              {0xFF, 0xF0, 0x00, 0x06, 0x40, 0x3C, 0xAA, 0x80},
                              {0xFF, 0xF0, 0x00, 0x06, 0x40, 0x3C, 0x67, 0x90}};
  for (const auto& r : real) {
    std::vector<uint8_t> in = {r[0], r[1], r[2], r[3], r[4], r[5], r[7]};
    EXPECT_EQ(crc_j1850(in), r[6]);
  }
}

TEST(Twingo18A, SameCrcFunctionServes090And242) {
  // The CRC of 0x090 / 0x242 in the vehicle is the same J1850 CRC (checked for all counters).
  for (int cnt = 0; cnt < 16; cnt++) {
    std::vector<uint8_t> d090 = {0x00, 0xFF, (uint8_t)(0xE0 | cnt), 0xF0, 0x7F, 0xF0};
    std::vector<uint8_t> d242 = {0x00, (uint8_t)(cnt << 3), 0xFF, 0xEF, 0xFE, 0x00, 0x0D};
    set_mask(0x3FF);
    TestTwingo b;
    b.setup();
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 400, 10, log);
    for (const Tx& x : with_id(log, 0x090)) {
      std::vector<uint8_t> in = {x.f.data.u8[0], x.f.data.u8[1], x.f.data.u8[2],
                                 x.f.data.u8[4], x.f.data.u8[5], x.f.data.u8[6]};
      EXPECT_EQ(x.f.data.u8[3], crc_j1850(in));
    }
    for (const Tx& x : with_id(log, 0x242)) {
      std::vector<uint8_t> in(x.f.data.u8, x.f.data.u8 + 7);
      EXPECT_EQ(x.f.data.u8[7], crc_j1850(in));
    }
    (void)d090;
    (void)d242;
    break;  // one run is enough, the loop above only documents the 16 counter values
  }
}

// ---------------------------------------------------------------------------
// 0x1F8: FA first, then 00 and a fade, then 0 - and no FA in the sleep stages
// ---------------------------------------------------------------------------

TEST(Twingo1F8, StartsWithFaThenFadesToZero) {
  set_mask(0x3FF | (1u << row_of(0x1F8)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1500, 10, log);
  auto f = with_id(log, 0x1F8);
  ASSERT_GE(f.size(), 140u);
  size_t i = 0;
  // FA phase: about 500 ms = 50 frames of 10 ms
  while (i < f.size() && f[i].f.data.u8[5] == 0xFA) {
    EXPECT_EQ(f[i].f.data.u8[6], 0x00);
    i++;
  }
  EXPECT_GE(i, 49u);
  EXPECT_LE(i, 52u);
  // Then the fade of the real vehicle: 00, 20, 0E(A0), 06(A0), 03, 01(60), 00(A0), 00(40), 00(20), 00
  const uint8_t fade[10][2] = {{0x00, 0x00}, {0x20, 0x00}, {0x0E, 0xA0}, {0x06, 0xA0}, {0x03, 0x00},
                               {0x01, 0x60}, {0x00, 0xA0}, {0x00, 0x40}, {0x00, 0x20}, {0x00, 0x00}};
  for (int k = 0; k < 10; k++) {
    ASSERT_LT(i + k, f.size());
    EXPECT_EQ(f[i + k].f.data.u8[5], fade[k][0]) << "fade step " << k;
    EXPECT_EQ(f[i + k].f.data.u8[6], fade[k][1]) << "fade step " << k;
  }
  // Afterwards the car stands: 0 for ever, never FA again
  for (size_t j = i + 10; j < f.size(); j++) {
    EXPECT_EQ(f[j].f.data.u8[5], 0x00);
    EXPECT_EQ(f[j].f.data.u8[6], 0x00);
  }
  // The other bytes are the constant ones of the vehicle
  for (const Tx& x : f) {
    EXPECT_EQ(x.f.data.u8[0], 0x00);
    EXPECT_EQ(x.f.data.u8[1], 0x84);
    EXPECT_EQ(x.f.data.u8[4], 0xFE);
    EXPECT_EQ(x.f.data.u8[7], 0x0F);
  }
  set_mask(0x3FF);
}

TEST(Twingo1F8, NoFaInTheShutdownStagesAnyMore) {
  set_mask(0x3FF | (1u << row_of(0x1F8)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 2000, 10, log);  // FA phase and fade are over
  b.request_sleep();
  std::vector<Tx> sleep_log;
  run(b, t, 130000, 10, sleep_log);  // C3 (66 s) and C2 (60 s) stages
  auto f = with_id(sleep_log, 0x1F8);
  ASSERT_FALSE(f.empty());
  for (const Tx& x : f) {
    EXPECT_EQ(x.f.data.u8[5], 0x00) << "t=" << x.t;
    EXPECT_EQ(x.f.data.u8[6], 0x00);
  }
  set_mask(0x3FF);
}

TEST(Twingo1F8, FaSequenceStartsAgainAfterWakeUp) {
  set_mask(0x3FF | (1u << row_of(0x1F8)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 2000, 10, log);
  b.request_sleep();
  std::vector<Tx> sleep_log;
  run(b, t, 160000, 100, sleep_log);  // through the whole shutdown into the silence
  b.request_wake_up();
  std::vector<Tx> wake_log;
  run(b, t, 8000, 10, wake_log);
  auto f = with_id(wake_log, 0x1F8);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[5], 0xFA);  // starts with the invalid marker again
  bool seen_zero_after_fa = false;
  for (const Tx& x : f) {
    if (x.f.data.u8[5] == 0x00) {
      seen_zero_after_fa = true;
    }
  }
  EXPECT_TRUE(seen_zero_after_fa);
  set_mask(0x3FF);
}

TEST(Twingo1F8, SwitchingTheRowOffAndOnStartsTheSequenceAgain) {
  set_mask(0x3FF | (1u << row_of(0x1F8)));
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 2000, 10, log);
  set_mask(0x3FF);
  run(b, t, 500, 10, log);
  set_mask(0x3FF | (1u << row_of(0x1F8)));
  std::vector<Tx> again;
  run(b, t, 100, 10, again);
  auto f = with_id(again, 0x1F8);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[5], 0xFA);
  set_mask(0x3FF);
}

// ---------------------------------------------------------------------------
// Free read request
// ---------------------------------------------------------------------------

TEST(TwingoFreeQuery, RejectsInvalidInput) {
  TestTwingo b;
  b.setup();
  EXPECT_STREQ(b.start_user_query(""), "empty request");
  EXPECT_STREQ(b.start_user_query("   "), "empty request");
  EXPECT_STREQ(b.start_user_query("22925G"), "invalid character (hex digits only)");
  EXPECT_STREQ(b.start_user_query("22925"), "odd number of hex digits");
  EXPECT_STREQ(b.start_user_query("2E928100"), "only the read services 0x22 and 0x19 are allowed");
  EXPECT_STREQ(b.start_user_query("14FFFFFF"), "only the read services 0x22 and 0x19 are allowed");
  EXPECT_STREQ(b.start_user_query("10 03"), "only the read services 0x22 and 0x19 are allowed");
  EXPECT_STREQ(b.start_user_query("31 01 B0 09"), "only the read services 0x22 and 0x19 are allowed");
  EXPECT_STREQ(b.start_user_query("22"), "0x22 needs one or more 2-byte identifiers");
  EXPECT_STREQ(b.start_user_query("2292"), "0x22 needs one or more 2-byte identifiers");
  EXPECT_STREQ(b.start_user_query("22 92 5E 92"), "0x22 needs one or more 2-byte identifiers");
  EXPECT_STREQ(b.start_user_query("19"), "0x19 needs a sub-function");
  EXPECT_STREQ(b.start_user_query("22 92 5E 92 61 92 5F 90"), "too long (at most 7 bytes)");
  EXPECT_STREQ(b.start_user_query(nullptr), "empty request");
  // None of the rejected inputs sent anything.
  clear_transmitted_frames();
  b.start_user_query("2E928100");
  EXPECT_TRUE(get_transmitted_frames().empty());
}

TEST(TwingoFreeQuery, ReadServicesWithOtherCaseAndSeparatorsAreAccepted) {
  TestTwingo b;
  b.setup();
  clear_transmitted_frames();
  EXPECT_STREQ(b.start_user_query("22:92:5e"), "OK");
  auto tx = diag_tx();
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data.u8[0], 0x03);
  EXPECT_EQ(tx[0].data.u8[1], 0x22);
  EXPECT_EQ(tx[0].data.u8[2], 0x92);
  EXPECT_EQ(tx[0].data.u8[3], 0x5E);
  EXPECT_EQ(tx[0].data.u8[4], 0xAA);  // padding like the other sequences of this driver
}

TEST(TwingoFreeQuery, SingleFrameAnswerIsShownTogetherWithTheRequest) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x06, 0x62, 0x92, 0x5E, 0x13, 0x88, 0x6F, 0x00}));
  EXPECT_STREQ(b.user_query_result(), "22 92 5E: OK 62 92 5E 13 88 6F");
  // The shared poll frame is back to its normal read template.
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 600, 100, log);
  auto polls = with_id(log, 0x18DADBF1, true);
  ASSERT_FALSE(polls.empty());
  EXPECT_EQ(polls[0].f.data.u8[0], 0x03);
  EXPECT_EQ(polls[0].f.data.u8[1], 0x22);
}

TEST(TwingoFreeQuery, TwoIdentifiersInOneRequestAreSentAsOneFrame) {
  TestTwingo b;
  b.setup();
  clear_transmitted_frames();
  ASSERT_STREQ(b.start_user_query("22 9005 9006"), "OK");
  auto tx = diag_tx();
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data.u8[0], 0x05);  // 22 + two identifiers = 5 bytes
  EXPECT_EQ(tx[0].data.u8[2], 0x90);
  EXPECT_EQ(tx[0].data.u8[3], 0x05);
  EXPECT_EQ(tx[0].data.u8[4], 0x90);
  EXPECT_EQ(tx[0].data.u8[5], 0x06);
}

TEST(TwingoFreeQuery, NegativeAnswerShowsSidAndNrc) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22FFFF"), "OK");
  b.handle_incoming_can_frame(reply_frame({0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0}));
  EXPECT_STREQ(b.user_query_result(), "22 FF FF: NEGATIVE 7F 22 31 (SID 0x22, NRC 0x31)");
}

TEST(TwingoFreeQuery, NoAnswerGivesTimeoutTextAndFreesTheChannel) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);  // the driver reads millis() when the request starts
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1500, 100, log);
  EXPECT_STREQ(b.user_query_result(), "requested");  // still waiting
  run(b, t, 1000, 100, log);                         // after the 2 s timeout
  EXPECT_STREQ(b.user_query_result(), "no response");
  EXPECT_STREQ(b.start_user_query("22925E"), "OK");  // channel is free again
}

TEST(TwingoFreeQuery, SecondRequestWhileOneIsRunningIsRefused) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  EXPECT_STREQ(b.start_user_query("229261"),
               "busy (another diagnostic exchange or a Sleep/NVROL run is active)");
  b.handle_incoming_can_frame(reply_frame({0x06, 0x62, 0x92, 0x5E, 0x13, 0x88, 0x6F, 0x00}));
  EXPECT_STREQ(b.start_user_query("229261"), "OK");
}

TEST(TwingoFreeQuery, RefusedWhileASleepRunIsActive) {
  TestTwingo b;
  b.setup();
  b.request_sleep();
  EXPECT_STREQ(b.start_user_query("22925E"),
               "busy (another diagnostic exchange or a Sleep/NVROL run is active)");
}

TEST(TwingoFreeQuery, MultiFrameAnswerSendsFlowControlAndIsReassembled) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22912B"), "OK");
  clear_transmitted_frames();
  // First frame: 10 bytes in total (62 91 2B + 7 data bytes), 6 of them here
  b.handle_incoming_can_frame(reply_frame({0x10, 0x0A, 0x62, 0x91, 0x2B, 0x01, 0x02, 0x03}));
  auto tx = diag_tx();
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data.u8[0], 0x30);  // flow control: continue to send
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x21, 0x04, 0x05, 0x06, 0x07, 0xAA, 0xAA, 0xAA}));
  EXPECT_STREQ(b.user_query_result(), "22 91 2B: OK 62 91 2B 01 02 03 04 05 06 07");
}

TEST(TwingoFreeQuery, ResponsePendingKeepsWaiting) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);  // the driver reads millis() when the request starts
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 1500, 100, log);
  b.handle_incoming_can_frame(reply_frame({0x03, 0x7F, 0x22, 0x78, 0, 0, 0, 0}));  // pending, restarts the timeout
  run(b, t, 1500, 100, log);                                                      // 3 s after the request in total
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x06, 0x62, 0x92, 0x5E, 0x13, 0x88, 0x6F, 0x00}));
  EXPECT_STREQ(b.user_query_result(), "22 92 5E: OK 62 92 5E 13 88 6F");
}

TEST(TwingoFreeQuery, VeryLongAnswerIsCutAndNeverOverflowsTheTextBuffer) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22926C"), "OK");
  // 300 bytes announced: 0x12C. First frame 6 bytes, then 42 consecutive frames of 7 bytes = 300.
  CAN_frame ff = reply_frame({0x11, 0x2C, 0x62, 0x92, 0x6C, 0x80, 0x00, 0x00});
  b.handle_incoming_can_frame(ff);
  uint8_t sn = 1;
  int received = 6;
  while (received < 300) {
    CAN_frame cf = reply_frame({(uint8_t)(0x20 | (sn & 0x0F)), 1, 2, 3, 4, 5, 6, 7});
    b.handle_incoming_can_frame(cf);
    received += 7;
    sn++;
  }
  const char* r = b.user_query_result();
  EXPECT_LT(strlen(r), 336u);
  EXPECT_NE(std::string(r).find("OK 62 92 6C"), std::string::npos);
  EXPECT_NE(std::string(r).find("..."), std::string::npos);  // text cut
  EXPECT_STREQ(b.start_user_query("22925E"), "OK");           // exchange finished, channel free
}

TEST(TwingoFreeQuery, StrayConsecutiveFrameIsIgnored) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  b.handle_incoming_can_frame(reply_frame({0x21, 1, 2, 3, 4, 5, 6, 7}));
  EXPECT_STREQ(b.user_query_result(), "requested");
}

TEST(TwingoFreeQuery, ServiceNineteenOpensTheSessionFirstAndThenSendsTheRequest) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  clear_transmitted_frames();
  ASSERT_STREQ(b.start_user_query("19 02 FF"), "OK");
  auto tx = diag_tx();
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data.u8[0], 0x02);  // session control 10 03
  EXPECT_EQ(tx[0].data.u8[1], 0x10);
  EXPECT_EQ(tx[0].data.u8[2], 0x03);
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);  // after the 100 ms gap the request follows
  bool found = false;
  for (const Tx& x : with_id(log, 0x18DADBF1, true)) {
    if (x.f.data.u8[1] == 0x19) {
      found = true;
      EXPECT_EQ(x.f.data.u8[0], 0x03);
      EXPECT_EQ(x.f.data.u8[2], 0x02);
      EXPECT_EQ(x.f.data.u8[3], 0xFF);
    }
  }
  EXPECT_TRUE(found);
  b.handle_incoming_can_frame(reply_frame({0x03, 0x59, 0x02, 0xFF, 0, 0, 0, 0}));
  EXPECT_STREQ(b.user_query_result(), "19 02 FF: OK 59 02 FF");
}

TEST(TwingoFreeQuery, ReadServiceTwentyTwoNeedsNoSession) {
  TestTwingo b;
  b.setup();
  clear_transmitted_frames();
  ASSERT_STREQ(b.start_user_query("22925F"), "OK");
  auto tx = diag_tx();
  ASSERT_EQ(tx.size(), 1u);
  EXPECT_EQ(tx[0].data.u8[1], 0x22);  // straight to the read request
}

TEST(TwingoFreeQuery, CellPollingIsPausedWhileTheRequestRuns) {
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 700, 100, log);
  EXPECT_FALSE(with_id(log, 0x18DADBF1, true).empty());  // normal polling runs
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  log.clear();
  run(b, t, 1500, 100, log);  // within the 2 s timeout, no answer: nothing else may be sent on this ID
  EXPECT_TRUE(with_id(log, 0x18DADBF1, true).empty());
}

// ---------------------------------------------------------------------------
// 19 14: fault detection counters
// ---------------------------------------------------------------------------

TEST(TwingoFaultCounters, RequestIsSentAfterTheSessionGapAndAnswerIsDecoded) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  clear_transmitted_frames();
  b.read_DTC_fdc();
  EXPECT_STREQ(b.fdc_query_result(), "requested");
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);
  bool found = false;
  for (const Tx& x : with_id(log, 0x18DADBF1, true)) {
    if (x.f.data.u8[1] == 0x19 && x.f.data.u8[2] == 0x14) {
      found = true;
      EXPECT_EQ(x.f.data.u8[0], 0x02);
    }
  }
  EXPECT_TRUE(found);
  // 59 14 E14381 7F 1B0715 FF = 10 bytes (multi-frame): first frame 6 bytes, then 4
  clear_transmitted_frames();
  b.handle_incoming_can_frame(reply_frame({0x10, 0x0A, 0x59, 0x14, 0xE1, 0x43, 0x81, 0x7F}));
  EXPECT_EQ(diag_tx().size(), 1u);  // flow control
  b.handle_incoming_can_frame(reply_frame({0x21, 0x1B, 0x07, 0x15, 0xFF, 0xAA, 0xAA, 0xAA}));
  EXPECT_STREQ(b.fdc_query_result(), "OK, 2 entries: E14381=+127 1B0715=-1");
  // The free query result is a separate field
  EXPECT_STREQ(b.user_query_result(), "not run yet");
}

TEST(TwingoFaultCounters, EmptyAnswerAndNegativeAnswerAndOddLength) {
  {
    TestTwingo b;
    b.setup();
    set_millis64(1000);
    b.read_DTC_fdc();
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 300, 50, log);
    b.handle_incoming_can_frame(reply_frame({0x02, 0x59, 0x14, 0, 0, 0, 0, 0}));
    EXPECT_STREQ(b.fdc_query_result(), "OK, no DTC with a fault detection counter");
  }
  {
    TestTwingo b;
    b.setup();
    set_millis64(1000);
    b.read_DTC_fdc();
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 300, 50, log);
    b.handle_incoming_can_frame(reply_frame({0x03, 0x7F, 0x19, 0x12, 0, 0, 0, 0}));
    EXPECT_STREQ(b.fdc_query_result(), "NEGATIVE 7F 19 12 (SID 0x19, NRC 0x12)");
  }
  {
    TestTwingo b;
    b.setup();
    set_millis64(1000);
    b.read_DTC_fdc();
    uint64_t t = 1000;
    std::vector<Tx> log;
    run(b, t, 300, 50, log);
    // 59 14 E1 43 81 = 5 bytes: not a multiple of 4 after the sub-function, so shown raw
    b.handle_incoming_can_frame(reply_frame({0x05, 0x59, 0x14, 0xE1, 0x43, 0x81, 0, 0}));
    EXPECT_STREQ(b.fdc_query_result(), "OK 59 14 E1 43 81");
  }
}

TEST(TwingoFaultCounters, NoAnswerGivesTimeout) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.read_DTC_fdc();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 3000, 100, log);
  EXPECT_STREQ(b.fdc_query_result(), "no response");
}

// ---------------------------------------------------------------------------
// 19 06: DTC details now with the multi-frame collector
// ---------------------------------------------------------------------------

TEST(TwingoDtcDetails, ShowsTheCompleteAnswerOfBothCodes) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.read_DTC_details();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);
  bool found = false;
  for (const Tx& x : with_id(log, 0x18DADBF1, true)) {
    if (x.f.data.u8[1] == 0x19 && x.f.data.u8[2] == 0x06) {
      found = true;
      EXPECT_EQ(x.f.data.u8[0], 0x06);
      EXPECT_EQ(x.f.data.u8[3], 0xE1);
      EXPECT_EQ(x.f.data.u8[4], 0x43);
      EXPECT_EQ(x.f.data.u8[5], 0x81);
      EXPECT_EQ(x.f.data.u8[6], 0xFF);
    }
  }
  EXPECT_TRUE(found);
  // Single frame answer of the real battery: 59 06 E1 43 81 50
  b.handle_incoming_can_frame(reply_frame({0x06, 0x59, 0x06, 0xE1, 0x43, 0x81, 0x50, 0}));
  run(b, t, 300, 50, log);  // next code after the session gap
  bool second = false;
  for (const Tx& x : with_id(log, 0x18DADBF1, true)) {
    if (x.f.data.u8[1] == 0x19 && x.f.data.u8[2] == 0x06 && x.f.data.u8[3] == 0x1B) {
      second = true;
    }
  }
  EXPECT_TRUE(second);
  // The second answer is longer (multi-frame): 59 06 1B 07 15 50 AA BB CC DD = 10 bytes
  b.handle_incoming_can_frame(reply_frame({0x10, 0x0A, 0x59, 0x06, 0x1B, 0x07, 0x15, 0x50}));
  b.handle_incoming_can_frame(reply_frame({0x21, 0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0}));
  std::string lines = detail_lines(b.get_uds_info_html());
  EXPECT_NE(lines.find(": OK 59 06 E1 43 81 50<br>"), std::string::npos) << lines;
  EXPECT_NE(lines.find(": OK 59 06 1B 07 15 50 AA BB CC DD<br>"), std::string::npos) << lines;
}

TEST(TwingoDtcDetails, NoAnswerGivesNoResponseForBothCodes) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.read_DTC_details();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 6000, 100, log);
  std::string lines = detail_lines(b.get_uds_info_html());
  size_t first = lines.find(": no response<br>");
  ASSERT_NE(first, std::string::npos) << lines;
  EXPECT_NE(lines.find(": no response<br>", first + 1), std::string::npos) << lines;
}

// ---------------------------------------------------------------------------
// More Battery Info page
// ---------------------------------------------------------------------------

TEST(TwingoPage, ShowsInputButtonAndAnswerFieldForTheFreeRequest) {
  TestTwingo b;
  b.setup();
  String html = b.get_uds_info_html();
  EXPECT_TRUE(contains(html, "id='twingoQueryHex'"));
  EXPECT_TRUE(contains(html, "onclick='twingoQuery()'"));
  EXPECT_TRUE(contains(html, "id='twingoQueryResult'"));
  EXPECT_TRUE(contains(html, "twingoFdc()"));
  EXPECT_TRUE(contains(html, "/twingoQuery?hex="));
  EXPECT_TRUE(contains(html, "/twingoQueryResult?which="));
  EXPECT_TRUE(contains(html, "/triggerTwingoDtcFdc"));
}

TEST(TwingoPage, AnswerFieldShowsTheLastResult) {
  TestTwingo b;
  b.setup();
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  b.handle_incoming_can_frame(reply_frame({0x06, 0x62, 0x92, 0x5E, 0x13, 0x88, 0x6F, 0x00}));
  String html = b.get_uds_info_html();
  EXPECT_TRUE(contains(html, "22 92 5E: OK 62 92 5E 13 88 6F"));
}

// ---------------------------------------------------------------------------
// 03.10. corrections: only the reply to the request is taken
// ---------------------------------------------------------------------------

TEST(TwingoReplyMatching, LateCellPollingReplyIsNotTakenAsTheAnswer) {
  // Screenshot case: request 22 90 05, the first reply was 62 90 72 0F 53 (cell 80 of the polling).
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("229005"), "OK");
  b.handle_incoming_can_frame(did_reply(0x9072, {0x0F, 0x53}));  // not ours
  EXPECT_STREQ(b.user_query_result(), "requested");              // still waiting
  b.handle_incoming_can_frame(did_reply(0x9005, {0x0E, 0x55}));  // ours
  EXPECT_STREQ(b.user_query_result(), "22 90 05: OK 62 90 05 0E 55");
}

TEST(TwingoReplyMatching, LateReplyStillReachesTheNormalHandlerAndIsNotLost) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("229005"), "OK");
  b.handle_incoming_can_frame(did_reply(0x9072, {0x0F, 0x53}));  // cell 80 = 3831 mV, goes to the polling
  b.handle_incoming_can_frame(did_reply(0x9005, {0x0E, 0x55}));
  EXPECT_TRUE(contains(b.get_uds_info_html(), "22 90 05: OK 62 90 05 0E 55"));
  EXPECT_EQ(datalayer.battery.status.cell_voltages_mV[79], 3831);
}

TEST(TwingoReplyMatching, SessionConfirmationIsNotTakenAsTheFaultCounterAnswer) {
  // Screenshot case: the button "Read DTC fault counters (19 14)" showed OK 50 03 00 32 01 F4.
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.read_DTC_fdc();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);  // session gap over, 19 14 sent, now waiting for the reply
  b.handle_incoming_can_frame(reply_frame({0x06, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4, 0}));
  EXPECT_STREQ(b.fdc_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x02, 0x59, 0x14, 0, 0, 0, 0, 0}));
  EXPECT_STREQ(b.fdc_query_result(), "OK, no DTC with a fault detection counter");
}

TEST(TwingoReplyMatching, NegativeReplyOfAnotherServiceIsIgnored) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  b.handle_incoming_can_frame(reply_frame({0x03, 0x7F, 0x19, 0x12, 0, 0, 0, 0}));  // SID 0x19, not ours
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x03, 0x7F, 0x22, 0x31, 0, 0, 0, 0}));  // ours
  EXPECT_STREQ(b.user_query_result(), "22 92 5E: NEGATIVE 7F 22 31 (SID 0x22, NRC 0x31)");
}

TEST(TwingoReplyMatching, FirstFrameOfAnotherReplyGoesToTheNormalPathAndFlowControlIsSentOnce) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  clear_transmitted_frames();
  // balancing reply of the polling (DID 0x912B) starts: not ours
  b.handle_incoming_can_frame(reply_frame({0x10, 0x0A, 0x62, 0x91, 0x2B, 0x01, 0x02, 0x03}));
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x21, 0x04, 0x05, 0x06, 0x07, 0xAA, 0xAA, 0xAA}));
  EXPECT_STREQ(b.user_query_result(), "requested");
  // our own reply is still accepted afterwards
  b.handle_incoming_can_frame(reply_frame({0x06, 0x62, 0x92, 0x5E, 0x13, 0x88, 0x6F, 0x00}));
  EXPECT_STREQ(b.user_query_result(), "22 92 5E: OK 62 92 5E 13 88 6F");
}

TEST(TwingoReplyMatching, SecondDidOfTheRequestDoesNotMatterOnlyTheFirstIsChecked) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("22 9005 9006"), "OK");
  b.handle_incoming_can_frame(reply_frame({0x07, 0x62, 0x90, 0x05, 0x0E, 0x55, 0x0E, 0x56}));
  EXPECT_STREQ(b.user_query_result(), "22 90 05 90 06: OK 62 90 05 0E 55 0E 56");
}

TEST(TwingoReplyMatching, NoMatchingReplyStillEndsWithTheTimeout) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("22925E"), "OK");
  b.handle_incoming_can_frame(did_reply(0x9072, {0x0F, 0x53}));
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 2500, 100, log);
  EXPECT_STREQ(b.user_query_result(), "no response");
  EXPECT_STREQ(b.start_user_query("22925E"), "OK");  // channel free again
}

TEST(TwingoReplyMatching, DtcDetailsIgnoreTheSessionConfirmationToo) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.read_DTC_details();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);
  b.handle_incoming_can_frame(reply_frame({0x06, 0x50, 0x03, 0x00, 0x32, 0x01, 0xF4, 0}));  // not the answer
  b.handle_incoming_can_frame(reply_frame({0x06, 0x59, 0x06, 0xE1, 0x43, 0x81, 0x2F, 0}));
  std::string lines = detail_lines(b.get_uds_info_html());
  EXPECT_NE(lines.find(": OK 59 06 E1 43 81 2F<br>"), std::string::npos) << lines;
  EXPECT_EQ(lines.find("50 03 00 32"), std::string::npos) << lines;
}

TEST(TwingoReplyMatching, ReadServiceNineteenSubFunctionMustBeEchoed) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  ASSERT_STREQ(b.start_user_query("190209"), "OK");
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 50, log);
  b.handle_incoming_can_frame(reply_frame({0x03, 0x59, 0x14, 0xFF, 0, 0, 0, 0}));  // other sub-function
  EXPECT_STREQ(b.user_query_result(), "requested");
  b.handle_incoming_can_frame(reply_frame({0x10, 0x0B, 0x59, 0x02, 0xFF, 0xE1, 0x43, 0x81}));
  b.handle_incoming_can_frame(reply_frame({0x21, 0x2F, 0x1B, 0x07, 0x15, 0x2F, 0xAA, 0xAA}));
  EXPECT_STREQ(b.user_query_result(), "19 02 09: OK 59 02 FF E1 43 81 2F 1B 07 15 2F");
}

// ---------------------------------------------------------------------------
// 03.10. corrections: no raw '%' in the pages (the web server's template engine eats text between two of them)
// ---------------------------------------------------------------------------

TEST(TwingoPagePercent, NoRawPercentSignAnywhereOnceAllPercentValuesAreRead) {
  TestTwingo b;
  b.setup();
  set_millis64(1000);
  b.handle_incoming_can_frame(did_reply(0x9003, {0x27, 0x10}));  // SOH 100 %
  b.handle_incoming_can_frame(did_reply(0x9001, {0x1A, 0x7E}));  // SOC
  b.handle_incoming_can_frame(did_reply(0x9002, {0x17, 0xDA}));  // USOC
  b.handle_incoming_can_frame(did_reply(0x91B9, {0x1B, 0x74}));  // SOC min
  b.handle_incoming_can_frame(did_reply(0x91BA, {0x1B, 0xC5}));  // SOC max
  CAN_frame f658 = {};
  f658.ID = 0x658;
  f658.DLC = 8;
  f658.data.u8[4] = 0x5F;  // SOH candidate
  b.handle_incoming_can_frame(f658);
  std::string page(b.get_uds_info_html().c_str());
  EXPECT_EQ(page.find('%'), std::string::npos) << "a raw percent sign is still in the More Battery Info page";
  EXPECT_NE(page.find("Battery SOH avg (0x9003): 100.000 &#37;"), std::string::npos);
  EXPECT_NE(page.find("SOH candidate (0x658 byte 4): 95 &#37;"), std::string::npos);
  // The block that was swallowed on 03.10. is present: checkbox, DTC read and erase lines
  EXPECT_NE(page.find("Read ALL DTC statuses (mask 0xFF)"), std::string::npos);
  EXPECT_NE(page.find("DTC Erase (ext. protocol, Erase DTC button):"), std::string::npos);
  EXPECT_NE(page.find("DTC Read (ext. protocol, Read DTC button):"), std::string::npos);
}

TEST(TwingoPagePercent, SimulatorPageHasNoRawPercentSignEither) {
  std::string page(simulator_processor(String("X")).c_str());
  ASSERT_GT(page.size(), 1000u);
  EXPECT_EQ(page.find('%'), std::string::npos);
}

// ---------------------------------------------------------------------------
// 03.10.: steady 0x350 frame C7 / C3
// ---------------------------------------------------------------------------

TEST(TwingoSteady350, DefaultIsC7AndTheSwitchChangesOnlyTheSteadyFrame) {
  RenaultTwingoGen1Battery::steady_350_use_c3 = false;
  TestTwingo b;
  b.setup();
  b.unix_set = true;
  b.unix_now = AGE_EPOCH + 2959882LL * 60;
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 300, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f.back().f.data.u8[0], 0xC7);
  EXPECT_EQ(f.back().f.data.u8[5], 0x98);
  EXPECT_EQ(f.back().f.data.u8[6], 0x94);
  RenaultTwingoGen1Battery::steady_350_use_c3 = true;
  log.clear();
  run(b, t, 300, 100, log);
  f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  // C3 as the emulator sent it before 03.10.: 14 14 96 45, age bytes stay the real minutes
  EXPECT_TRUE(f.back().f.data.u8[0] == 0xC3 && f.back().f.data.u8[1] == 0x2D && f.back().f.data.u8[2] == 0x2A &&
              f.back().f.data.u8[3] == 0x0A && f.back().f.data.u8[4] == 0x14 && f.back().f.data.u8[5] == 0x14 &&
              f.back().f.data.u8[6] == 0x96 && f.back().f.data.u8[7] == 0x45);
  RenaultTwingoGen1Battery::steady_350_use_c3 = false;
  log.clear();
  run(b, t, 300, 100, log);
  f = with_id(log, 0x350);
  EXPECT_EQ(f.back().f.data.u8[0], 0xC7);  // back to C7 at once
}

TEST(TwingoSteady350, SleepAndWakeSequencesStayTheSameWhateverIsSelected) {
  RenaultTwingoGen1Battery::steady_350_use_c3 = true;
  TestTwingo b;
  b.setup();
  uint64_t t = 1000;
  std::vector<Tx> log;
  run(b, t, 500, 100, log);
  b.request_sleep();
  log.clear();
  run(b, t, 3000, 100, log);
  auto f = with_id(log, 0x350);
  ASSERT_FALSE(f.empty());
  EXPECT_EQ(f[0].f.data.u8[0], 0xC3);  // C3 stage of the shutdown sequence, as before
  RenaultTwingoGen1Battery::steady_350_use_c3 = false;
}

TEST(TwingoSteady350, SimulatorPageShowsTheSelectionAndItsCurrentState) {
  RenaultTwingoGen1Battery::steady_350_use_c3 = false;
  std::string page(simulator_processor(String("X")).c_str());
  EXPECT_NE(page.find("id='steady350c7' checked "), std::string::npos);
  EXPECT_EQ(page.find("id='steady350c3' checked "), std::string::npos);
  EXPECT_NE(page.find("/editTwingoSteady350?value=1"), std::string::npos);
  RenaultTwingoGen1Battery::steady_350_use_c3 = true;
  page = std::string(simulator_processor(String("X")).c_str());
  EXPECT_EQ(page.find("id='steady350c7' checked "), std::string::npos);
  EXPECT_NE(page.find("id='steady350c3' checked "), std::string::npos);
  RenaultTwingoGen1Battery::steady_350_use_c3 = false;
}
