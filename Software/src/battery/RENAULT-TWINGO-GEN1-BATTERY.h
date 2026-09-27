#ifndef RENAULT_TWINGO_GEN1_BATTERY_H
#define RENAULT_TWINGO_GEN1_BATTERY_H

#include "../datalayer/datalayer.h"
#include "UdsCanBattery.h"

// ---------------------------------------------------------------------------
// Extended-address polling (0x18DADBF1 request / 0x18DAF1DB response) for
// individual cell voltages, balancing status, and a few lifetime metrics.
//
// This is a *separate* protocol from the standard KWP2000 UDS used above
// (0x79B/0x7BB) - the two do not interact. The 0x79B/0x7BB service has never
// answered a single request on this battery (confirmed across many CAN
// captures); this extended address is the one that actually works, confirmed
// three independent ways:
//   - the user's own OVMS RT32 vehicle module for this exact battery
//   - a captured real-vehicle OBD log during an AC charge session
//   - Battery-Emulator's own RENAULT-ZOE-GEN2-BATTERY.cpp driver (same LBC
//     protocol family, same PIDs)
//
// Comment out the line below to disable this extended polling entirely. With
// it disabled, cell_voltages_mV[]/cell_balancing_status[] stay unpopulated
// (as before) and the pack voltage estimate reverts to the original
// (min+max)/2*96 approximation further down in this file.
// ---------------------------------------------------------------------------
#define TWINGO_EXTENDED_CELL_POLLING

// Uncomment for verbose logging of every extended-channel (0x18DADBF1/
// 0x18DAF1DB) request, response and reassembly event - including negative
// responses and oversized multi-frame replies that get discarded because
// they don't fit ext_isotp_buffer. Off by default (no runtime cost when
// disabled); enable temporarily while investigating, same idea as the
// existing (also off-by-default) UDS_DEBUG in UdsCanBattery.cpp.
//#define EXTENDED_UDS_DEBUG

class RenaultTwingoGen1Battery : public UdsCanBattery {
 public:
  // Use this constructor for the second battery.
  RenaultTwingoGen1Battery(DATALAYER_BATTERY_TYPE* datalayer_ptr, CAN_Interface targetCan) : UdsCanBattery(targetCan) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
    dtc = &datalayer_battery->dtc;
    calculated_total_pack_voltage_mV = 0;
  }

  // Use the default constructor to create the first or single battery.
  RenaultTwingoGen1Battery() {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
    dtc = &datalayer_battery->dtc;
  }

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "Renault Twingo Gen1 22kWh";

  String get_uds_info_html() override;
  const char* get_dtc_json_filename() override { return "renault_zoe_gen1_dtc.json"; }
  bool get_dtc_standard_code_string() override { return false; }
  void read_DTC() override;
#ifdef TWINGO_EXTENDED_CELL_POLLING
  bool supports_reset_NVROL() override { return true; }
  void reset_NVROL() override { start_nvrol_run(0); }
  // "Sleep" / "Sleep 0x9281=1" / "Wake up" buttons: this driver stays completely silent (nothing at all is
  // transmitted) until "Wake up" is pressed, safety limit 30 minutes. "Sleep 0x9281=1" first opens a
  // diagnostic session and writes temporisation=1 (no reset routine B009), "Sleep" sends nothing at all.
  bool supports_sleep_control() override { return true; }
  void request_sleep() override { start_nvrol_run(1); }
  void request_sleep_temporisation() override { start_nvrol_run(2); }
  void request_wake_up() override { nvrol_wake_request = true; }
#endif

 protected:
  // Called by the UDS superclass for every successful PID response. `data`
  // points at the raw value bytes, starting right after the echoed local
  // identifier. Return 0 to continue the scan list in order.
  uint16_t handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) override;
  void on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) override;

 private:
  DATALAYER_BATTERY_TYPE* datalayer_battery;

  // If not null, this battery decides when the contactor can be closed and writes the value here.
  bool* allows_contactor_closing;

  static const int MAX_PACK_VOLTAGE_DV = 4040;  //5000 = 500.0V
  static const int MIN_PACK_VOLTAGE_DV = 3000;
  static const int MAX_CELL_DEVIATION_MV = 150;
  static const int MAX_CELL_VOLTAGE_MV = 4220;  //Battery is put into emergency stop if one cell goes over this value
  static const int MIN_CELL_VOLTAGE_MV = 2700;  //Battery is put into emergency stop if one cell goes below this value

  // One-byte KWP2000 local identifiers polled from the BMS (0x21 service).
  static const int GROUP1_CELLVOLTAGES_1_POLL = 0x41;  // Cells 1-62
  static const int GROUP2_CELLVOLTAGES_2_POLL = 0x42;  // Cells 63-96
  static const int GROUP3_METRICS = 0x61;              // Mileage + alltime energy
  static const int GROUP6_BALANCING = 0x07;            // Balancing status bits

  unsigned long previousMillis100 = 0;  // will store last time a 100ms CAN Message was sent
  unsigned long previousMillis1000_69f = 0;
  unsigned long previousMillis60000_436 = 0;
  uint8_t counter_423 = 0;
  uint8_t zoe_19F_counter = 0;
  uint16_t zoe_436_counter = 1;

  CAN_frame ZOE_423 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x423,
                       .data = {0x07, 0x1d, 0x00, 0x02, 0x5d, 0x80, 0x5d, 0xc8}};

  // Cyclic Broadcast Frames for Zoe Gen1 Powertrain (Eliminates U1000 / D000 CAN Loss Faults)
  CAN_frame ZOE_19F_INVERTER = {.FD = false,
                                .ext_ID = false,
                                .DLC = 8,
                                .ID = 0x19F,
                                .data = {0x00, 0x00, 0x7D, 0x04, 0x00, 0x6A, 0x90, 0xFE}};

  CAN_frame ZOE_426_POWER_MUX = {.FD = false,
                                 .ext_ID = false,
                                 .DLC = 8,
                                 .ID = 0x426,
                                 .data = {0x00, 0x60, 0x01, 0x00, 0x4B, 0xC8, 0x00, 0x40}};

  CAN_frame ZOE_436_VEHICLE_STATUS = {.FD = false,
                                      .ext_ID = false,
                                      .DLC = 6,
                                      .ID = 0x436,
                                      .data = {0x86, 0x14, 0x00, 0x01, 0xFF, 0xDC}};

  CAN_frame ZOE_69F_BCM_GATEWAY = {.FD = false,
                                   .ext_ID = false,
                                   .DLC = 4,
                                   .ID = 0x69F,
                                   .data = {0x71, 0x30, 0x28, 0x2F}};

  // Experiment (only sent during the shutdown sequence's C3/C2 stages, see transmit_can()): 0x214 was
  // found in Log_Twingo_Ladung.log changing in lockstep with 0x350's own C3/C2 ("08 00") vs. C0/00/awake
  // ("F8 3E") states, only 0.03-0.19s after each transition - possibly the EVC's own wake/sleep request
  // that JB2 exposes as diagnostic DID $5015 ("Vsx_evc_wkp_req", GoToSleep/WakeUp), though that is not
  // confirmed from any spec, only this timing correlation. Not sent outside the shutdown sequence.
  CAN_frame TWINGO_214_EVC_SLEEP_REQ = {.FD = false, .ext_ID = false, .DLC = 2, .ID = 0x214, .data = {0xF8, 0x3E}};

  uint16_t LB_SOC = 50;
  uint16_t LB_Display_SOC = 50;
  uint16_t LB_SOH = 99;
  int16_t LB_Average_Temperature = 0;
  uint32_t LB_Charging_Power_W = 0;
  uint32_t LB_Regen_allowed_W = 0;
  uint32_t LB_Discharge_allowed_W = 0;
  int16_t LB_Current_raw = 2000;  // 0x155 raw; 2000 => 0 A
  int16_t LB_Cell_minimum_temperature = 0;
  int16_t LB_Cell_maximum_temperature = 0;
  uint16_t LB_Cell_minimum_voltage = 3700;
  uint16_t LB_Cell_maximum_voltage = 3700;
  uint16_t LB_kWh_Remaining = 0;
  uint16_t LB_Battery_Voltage = 3700;
  uint8_t LB_Heartbeat = 0;
  uint8_t LB_CUV = 0;
  uint8_t LB_HVBIR = 0;
  uint8_t LB_HVBUV = 0;
  uint8_t LB_EOCR = 0;
  uint8_t LB_HVBOC = 0;
  uint8_t LB_HVBOT = 0;
  uint8_t LB_HVBOV = 0;
  uint8_t LB_COV = 0;
  uint32_t calculated_total_pack_voltage_mV = 370000;
  uint16_t battery_mileage_in_km = 0;
  uint16_t kWh_from_beginning_of_battery_life = 0;

#ifdef TWINGO_EXTENDED_CELL_POLLING
  // -------------------------------------------------------------------------
  // Extended-address (0x18DADBF1/0x18DAF1DB) polling: individual cell
  // voltages, balancing status, and a few lifetime metrics. Fully additive -
  // none of the fields/logic above are touched by any of this.
  // -------------------------------------------------------------------------

  // Local identifiers (2-byte PIDs), UDS SID 0x22 ReadDataByIdentifier.
  static const uint16_t EXT_POLL_BALANCE_SWITCHES = 0x912B;    // Multi-frame, 96 bit
  static const uint16_t EXT_POLL_CYCLES = 0x9210;              // Single-frame, 16-bit count
  static const uint16_t EXT_POLL_ENERGY_CHARGED = 0x9243;      // Single-frame, 32-bit, x0.001 kWh
  static const uint16_t EXT_POLL_ENERGY_DISCHARGED = 0x9245;   // Single-frame, 32-bit, x0.001 kWh
  static const uint16_t EXT_POLL_ENERGY_REGENERATED = 0x9247;  // Single-frame, 32-bit, x0.001 kWh
  static const uint16_t EXT_POLL_TEMPORISATION = 0x9281;       // Single-frame, top bit of byte 0
                                                                 // ("temporisation before sleep" status -
                                                                 // see NVROL comment block below)

  // 8 single pack temperature sensors, PIDs 0x9131-0x9138 ("Pack temperature 1-8" in the
  // OVMS RT32 module, same LBC). Single-frame, 16-bit raw value, degC = raw * 0.0625 - 40.
  static const uint16_t EXT_POLL_TEMP_FIRST = 0x9131;
  static const uint8_t EXT_TEMP_SENSOR_COUNT = 8;

  // BMS state list (32 x 1 byte, 35-byte multi-frame reply like 0x912B) and the balancing counters
  // (32-bit, single-frame). Counter value per CanZE: (raw XOR 0x80000000) / 1024, in Ah resp. h.
  static const uint16_t EXT_POLL_BMS_STATE = 0x9270;
  static const uint16_t EXT_POLL_BAL_CAP_TOTAL = 0x924F;   // Ah, total balanced capacity
  static const uint16_t EXT_POLL_BAL_TIME_TOTAL = 0x9250;  // h, total balancing time
  static const uint16_t EXT_POLL_BAL_CAP_SLEEP = 0x9251;   // Ah, balanced capacity in sleep mode
  static const uint16_t EXT_POLL_BAL_TIME_SLEEP = 0x9252;  // h, balancing time in sleep mode
  static const uint16_t EXT_POLL_BAL_CAP_WAKE = 0x9262;    // Ah, balanced capacity while awake
  static const uint16_t EXT_POLL_BAL_TIME_WAKE = 0x9263;   // h, balancing time while awake

  // 96 cell-voltage PIDs (0x9021-0x9083, skipping 0x9040/0x9060/0x9080) + the
  // 6 PIDs above + the 8 pack temperature PIDs + BMS state + 4 balancing counters
  // = 115 poll targets, cycled continuously, one every 200ms (same cadence as
  // Battery-Emulator's own Zoe Ph2 driver) -> ~23.0s/cycle.
  static const uint8_t EXT_POLL_LIST_LENGTH = 117;
  const uint16_t ext_poll_list[EXT_POLL_LIST_LENGTH] = {
      0x9021, 0x9022, 0x9023, 0x9024, 0x9025, 0x9026, 0x9027, 0x9028,
      0x9029, 0x902A, 0x902B, 0x902C, 0x902D, 0x902E, 0x902F, 0x9030,
      0x9031, 0x9032, 0x9033, 0x9034, 0x9035, 0x9036, 0x9037, 0x9038,
      0x9039, 0x903A, 0x903B, 0x903C, 0x903D, 0x903E, 0x903F, 0x9041,
      0x9042, 0x9043, 0x9044, 0x9045, 0x9046, 0x9047, 0x9048, 0x9049,
      0x904A, 0x904B, 0x904C, 0x904D, 0x904E, 0x904F, 0x9050, 0x9051,
      0x9052, 0x9053, 0x9054, 0x9055, 0x9056, 0x9057, 0x9058, 0x9059,
      0x905A, 0x905B, 0x905C, 0x905D, 0x905E, 0x905F, 0x9061, 0x9062,
      0x9063, 0x9064, 0x9065, 0x9066, 0x9067, 0x9068, 0x9069, 0x906A,
      0x906B, 0x906C, 0x906D, 0x906E, 0x906F, 0x9070, 0x9071, 0x9072,
      0x9073, 0x9074, 0x9075, 0x9076, 0x9077, 0x9078, 0x9079, 0x907A,
      0x907B, 0x907C, 0x907D, 0x907E, 0x907F, 0x9081, 0x9082, 0x9083,
      // Cell voltages end here (96 entries). Balancing + metrics follow:
      EXT_POLL_BALANCE_SWITCHES, EXT_POLL_CYCLES, EXT_POLL_ENERGY_CHARGED, EXT_POLL_ENERGY_DISCHARGED,
      EXT_POLL_ENERGY_REGENERATED, EXT_POLL_TEMPORISATION,
      // Pack temperature sensors 1-8
      0x9131, 0x9132, 0x9133, 0x9134, 0x9135, 0x9136, 0x9137, 0x9138,
      // BMS state + balancing counters (total, then in sleep mode)
      EXT_POLL_BMS_STATE, EXT_POLL_BAL_CAP_TOTAL, EXT_POLL_BAL_TIME_TOTAL, EXT_POLL_BAL_CAP_SLEEP,
      EXT_POLL_BAL_TIME_SLEEP, EXT_POLL_BAL_CAP_WAKE, EXT_POLL_BAL_TIME_WAKE};

  uint8_t ext_poll_index = 0;
  unsigned long previousMillisExtPoll = 0;
  static const unsigned long EXT_POLL_INTERVAL_MS = 200;  // Matches Zoe Ph2 driver's proven cadence

  // Generic ISO-TP multi-frame reassembly buffer for the extended channel.
  // Only ever needed for EXT_POLL_BALANCE_SWITCHES (96 bit doesn't fit a
  // single frame); everything else here is single-frame.
  uint8_t ext_isotp_buffer[40] = {0};  // Real 0x912B response is 35 bytes total (confirmed via
                                        // CAN log: First Frame "10 23 62 91 2b ...") - the old 24-byte
                                        // buffer was too small, causing every reassembly attempt to be
                                        // abandoned before Flow Control was even sent back.
  uint16_t ext_isotp_expected_len = 0;
  uint16_t ext_isotp_received_len = 0;
  bool ext_isotp_in_progress = false;
  unsigned long ext_isotp_started_ms = 0;
  static const unsigned long EXT_ISOTP_TIMEOUT_MS = 500;  // Abandon a stalled multi-frame reassembly

  // Boot plausibility filter for cell voltages: while the system is starting up, only cell
  // values inside 3.0-4.3V are accepted, everything else is dropped. The boot phase ends as
  // soon as all 96 cells have delivered one plausible value, or 60s after the first cell
  // reply at the latest. From then on every value is accepted unfiltered, so a genuine
  // under/overvoltage stays visible.
  static const uint16_t EXT_BOOT_CELL_MV_MIN = 3000;
  static const uint16_t EXT_BOOT_CELL_MV_MAX = 4300;
  static const unsigned long EXT_BOOT_FILTER_TIMEOUT_MS = 60000;
  unsigned long ext_first_cell_reply_ms = 0;  // 0 = no cell reply seen yet

  // Pack temperature sensors: time of the last plausible reply per sensor. A sensor that
  // has not answered for 60s (~2.7 poll cycles) is treated as invalid again.
  static const unsigned long EXT_TEMP_STALE_MS = 60000;
  unsigned long ext_temp_last_ms[EXT_TEMP_SENSOR_COUNT] = {0};

  // After the NVROL quiet phase the BMS wakes up like after a boot: the cell plausibility filter is
  // armed again for 60s (same limits as above), values that arrive implausible are dropped.
  bool ext_filter_rearm_active = false;
  unsigned long ext_filter_rearm_start_ms = 0;

  // Live values of the newly polled PIDs (raw, as received)
  uint8_t bms_state_raw[32] = {0};  // 0x9270, all 32 list bytes - which one is the current state is not
                                    // documented, both the first and the last byte are shown decoded
  bool bms_state_valid = false;
  uint32_t bal_raw[6] = {0};  // 0 = capacity total, 1 = time total, 2 = capacity sleep, 3 = time sleep,
                              // 4 = capacity wake, 5 = time wake
  bool bal_valid[6] = {false, false, false, false, false, false};

  // After the quiet phase these PIDs are asked first (round-robin, every 200ms) until each has
  // answered, for at most 30s: 0x9270, 0x9281, 0x9251, 0x9252 (bit n of the mask = list entry n).
  static const uint8_t EXT_PRIORITY_COUNT = 4;
  const uint16_t ext_priority_list[EXT_PRIORITY_COUNT] = {EXT_POLL_BMS_STATE, EXT_POLL_TEMPORISATION,
                                                          EXT_POLL_BAL_CAP_SLEEP, EXT_POLL_BAL_TIME_SLEEP};
  uint8_t ext_priority_pending_mask = 0;
  uint8_t ext_priority_next = 0;
  unsigned long ext_priority_start_ms = 0;
  static const unsigned long EXT_PRIORITY_TIMEOUT_MS = 30000;

  // How many of the 96 cell-voltage PIDs have replied at least once since boot.
  // Gates the sum-based pack voltage calculation so it never runs on a
  // partially-populated array.
  uint8_t ext_cells_seen = 0;

  uint16_t battery_charge_cycles = 0;
  float battery_energy_charged_kWh = 0.0f;
  float battery_energy_discharged_kWh = 0.0f;
  float battery_energy_regenerated_kWh = 0.0f;
  uint16_t battery_temporisation = 0x100;  // 0x100 = not yet read, otherwise the raw byte of 0x9281

  // -------------------------------------------------------------------------
  // NVROL reset ("Add NVROL and temporisation to Zoe Ph2", upstream issue #587,
  // fixed in #1416 - same LBC protocol family, same two write sequences).
  // Documented purpose: the LBC's own non-volatile time tracking can get
  // "confused", which can make SOC reset to a wrong value on every power
  // cycle. Reading EXT_POLL_TEMPORISATION above tells us whether this even
  // applies to this exact battery before considering the reset below.
  //
  // The reset only takes effect once the BMS actually goes to sleep afterwards (all frames to the
  // battery have to stop, per the author of the ZE50 reference). Instead of guessing a sleep flag
  // in ZOE_423, the two write sequences below are followed by a "quiet phase" (until Wake up is pressed, 30 minutes at most) in which this
  // driver transmits absolutely nothing (no wake/vehicle frames, no polls, no flow control) and only
  // listens: received frames are counted per second, replies on 0x18DAF1DB are recorded with a
  // time stamp. The 12V supply of the BMS must stay on during this time. Afterwards everything
  // restarts, the cell plausibility filter is armed again and a few PIDs are read with priority.
  // -------------------------------------------------------------------------
  bool UserRequestNVROLReset = false;
  uint8_t NVROLstateMachine = 0;
  unsigned long startTimeNVROL = 0;
  // What the run is: 0 = NVROL reset (full sequence, then silent until Wake up), 1 = "Sleep" (nothing sent at all,
  // silent until Wake up), 2 = "Sleep 0x9281=1" (session + write temporisation, then silent until Wake up).
  uint8_t nvrol_mode = 0;
  uint8_t nvrol_last_mode = 0;  // mode of the last finished run, for the display
  bool nvrol_wake_request = false;
  static const unsigned long SLEEP_MANUAL_FAILSAFE_MS = 30UL * 60UL * 1000UL;  // wakes up on its own after 30 min
  void start_nvrol_run(uint8_t mode) {
    if (!UserRequestNVROLReset) {
      nvrol_mode = mode;
      nvrol_wake_request = false;
      UserRequestNVROLReset = true;
    }
  }
  void transmit_reset_nvrol_frames(void);

  // Vehicle-state broadcast (0x350), captured from a real AC-charge session (Log_Twingo_Ladung.log):
  // byte 0 is the BCM vehicle state (0xC3 BAT_TEMPO_LEVEL while active, 0xC2 CUT_OFF_PENDING, 0xC0
  // SLEEPING, then 0x00), byte 3 a free-running minute counter, bytes 1/2/4/6 constant, bytes 5/7 depend
  // on the state. Sent only during the shutdown sequence and the wake burst below - never as a permanent
  // background frame.
  uint8_t vehicle_minute_counter = 0x7D;  // byte 3 of 0x350; +1 every 60s, keeps running even in true silence
  unsigned long vehicle_minute_counter_last_ms = 0;
  void tick_minute_counter(void);
  void send_vehicle_state_350(uint8_t byte0);
  void send_wake_burst_frame(uint8_t index);  // 0 = the initial C0 frame, 1..10 = the ten C3 frames

  // Shutdown sequence (NVROLstateMachine == 7): 0x350 walks C3 -> C2 -> C0 -> 00 like the real vehicle
  // (stage durations from the captured log); our own broadcast frames (423/19F/426/436/69F) keep running
  // through C3/C2/C0 and stop once the 00 stage begins, then true silence (state 5) follows.
  static const unsigned long POWERDOWN_C3_MS = 66000;
  static const unsigned long POWERDOWN_C2_MS = 60000;
  static const unsigned long POWERDOWN_C0_MS = 10000;
  static const unsigned long POWERDOWN_00_MS = 1000;
  uint8_t powerdown_stage = 0;  // 0=C3, 1=C2, 2=C0, 3=00
  unsigned long powerdown_stage_start_ms = 0;
  unsigned long powerdown_start_ms = 0;
  unsigned long previousMillis_350 = 0;
  static const unsigned long INTERVAL_350_MS = 100;
  void start_powerdown(void);

  // Wake-up burst (NVROLstateMachine == 8), same idea as the user's own OVMS RT32 module's
  // CommandWakeup2: 0x350 = C0 once, then C3 ten times, 200ms apart, before normal polling resumes.
  static const unsigned long WAKE_BURST_INTERVAL_MS = 200;
  static const uint8_t WAKE_BURST_COUNT = 10;  // number of C3 frames (plus 1 initial C0 frame)
  uint8_t wake_burst_index = 0;
  unsigned long wake_burst_last_ms = 0;
  void start_wake_burst(void);

  // Per-step result of the last NVROL sequence (index 0=open session,
  // 1=RoutineControl B009, 2=open session again, 3=WriteDataByIdentifier
  // 9281), shown on "More Battery Info" - lets us see whether each step was
  // accepted, actively rejected (with the BMS's own SID/NRC code), or never
  // answered at all, instead of sending blind like the first attempt.
  static const uint8_t NVROL_LOG_STEPS = 5;  // index 4 = 0x9281 read back right after the write
  char nvrol_log[NVROL_LOG_STEPS][48] = {"not run yet", "not run yet", "not run yet", "not run yet",
                                         "not run yet"};
  uint16_t temporisation_readback = 0x100;  // raw byte of 0x9281 read right after the write, 0x100 = none
  uint8_t nvrol_awaiting_step = 0;  // Which nvrol_log[] slot the next 0x18DAF1DB reply belongs to
  void handle_nvrol_reply(CAN_frame rx_frame);

  // Quiet phase (NVROLstateMachine == 5). UserRequestNVROLReset stays true during the whole time.
  static const unsigned long NVROL_SILENCE_MS = 45000;
  static const uint8_t NVROL_SILENCE_S = 45;
  unsigned long nvrol_silence_start_ms = 0;
  unsigned long nvrol_silence_last_rx_ms = 0;
  static const uint8_t SILENCE_SEC_BUCKETS = 61;   // frames per second for the first 60 s
  static const uint8_t SILENCE_10S_BUCKETS = 180;  // frames per 10 s for up to 30 min
  uint16_t nvrol_silence_rx_per_s[SILENCE_SEC_BUCKETS] = {0};
  uint16_t nvrol_silence_rx_per_10s[SILENCE_10S_BUCKETS] = {0};
  unsigned long nvrol_silence_end_ms = 0;
  // Every CAN ID heard during the quiet phase: how often, how often the data changed, when it was last
  // seen and its last data. Shows which frames stop first, and status bytes that change before the end.
  struct SilenceChange {  // one data change of an ID: when, new data, which bytes differed from before
    uint32_t t_ms;        // since the start of the quiet phase
    uint8_t mask;         // bit n set = byte n changed
    uint8_t d[8];
  };
  struct SilenceId {
    uint32_t id;
    uint32_t count;
    uint16_t changes;
    uint8_t dlc;
    uint8_t chg_count;  // stored changes (max 3, the last ones)
    uint8_t chg_next;   // ring index for the next change
    uint8_t first[8];
    uint8_t last[8];
    unsigned long last_ms;
    SilenceChange chg[3];
  };
  static const uint8_t SILENCE_ID_MAX = 40;
  SilenceId nvrol_silence_ids[SILENCE_ID_MAX] = {};
  uint8_t nvrol_silence_id_count = 0;
  uint32_t nvrol_silence_id_other = 0;  // frames of IDs beyond the table size
  void record_silence_frame(const CAN_frame& f);

  // How fast the BMS comes back after Wake up (offsets in ms since the end of the quiet phase, -1 = not yet)
  bool wake_tracking = false;
  unsigned long wake_start_ms = 0;
  int32_t wake_first_rx = -1;        // first frame from the BMS
  int32_t wake_first_uds = -1;       // first reply on 0x18DAF1DB
  int32_t wake_priority_done = -1;   // all four priority PIDs answered
  bool wake_priority_timeout = false;  // gave up after 30s
  void priority_answered(uint8_t bit);
  uint32_t nvrol_silence_rx_total = 0;
  bool nvrol_silence_done = false;  // a run has completed since the last button press
  struct NvrolSilenceReply {
    uint16_t t_ms;  // time since the start of the quiet phase
    uint8_t d[8];
  };
  static const uint8_t NVROL_SILENCE_REPLY_MAX = 8;
  NvrolSilenceReply nvrol_silence_reply[NVROL_SILENCE_REPLY_MAX] = {};
  uint8_t nvrol_silence_reply_count = 0;
  uint16_t nvrol_silence_reply_total = 0;
  struct NvrolSnapshot {  // values right before the reset, shown next to the live values afterwards
    uint8_t state[32];
    bool state_valid;
    uint32_t bal_raw[6];
    bool bal_valid[6];
    uint16_t temporisation;  // raw byte of 0x9281, 0x100 = not read
  };
  NvrolSnapshot nvrol_before = {};
  void finish_nvrol_silence(void);
  void append_live_html(String& s);
  void append_quiet_html(String& s);
  void append_refresh_script_html(String& s, bool busy);

  CAN_frame ZOE_POLL_18DADBF1 = {.FD = false,
                                 .ext_ID = true,
                                 .DLC = 8,
                                 .ID = 0x18DADBF1,
                                 .data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  CAN_frame ZOE_POLL_FLOW_CONTROL = {.FD = false,
                                     .ext_ID = true,
                                     .DLC = 8,
                                     .ID = 0x18DADBF1,
                                     .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

  void handle_extended_reply(CAN_frame rx_frame);
  void handle_extended_single_frame(uint16_t pid, const uint8_t* data, uint16_t length);
  void handle_extended_multiframe_complete();
#endif
};

#endif
