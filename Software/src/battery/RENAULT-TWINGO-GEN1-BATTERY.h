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
  void reset_NVROL() override { UserRequestNVROLReset = true; }
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

  // 96 cell-voltage PIDs (0x9021-0x9083, skipping 0x9040/0x9060/0x9080) + the
  // 6 PIDs above = 102 poll targets, cycled continuously, one every 200ms
  // (same cadence as Battery-Emulator's own Zoe Ph2 driver) -> ~20.4s/cycle.
  static const uint8_t EXT_POLL_LIST_LENGTH = 102;
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
      EXT_POLL_ENERGY_REGENERATED, EXT_POLL_TEMPORISATION};

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

  // How many of the 96 cell-voltage PIDs have replied at least once since boot.
  // Gates the sum-based pack voltage calculation so it never runs on a
  // partially-populated array.
  uint8_t ext_cells_seen = 0;

  uint16_t battery_charge_cycles = 0;
  float battery_energy_charged_kWh = 0.0f;
  float battery_energy_discharged_kWh = 0.0f;
  float battery_energy_regenerated_kWh = 0.0f;
  uint8_t battery_temporisation = 255;  // 255 = not yet read. 0/1 = actual BMS value once known.

  // -------------------------------------------------------------------------
  // NVROL reset ("Add NVROL and temporisation to Zoe Ph2", upstream issue #587,
  // fixed in #1416 - same LBC protocol family, same two write sequences).
  // Documented purpose: the LBC's own non-volatile time tracking can get
  // "confused", which can make SOC reset to a wrong value on every power
  // cycle. Reading EXT_POLL_TEMPORISATION above tells us whether this even
  // applies to this exact battery before considering the reset below.
  //
  // Deliberately NOT included: the 30s software "let it sleep" step Zoe Ph2
  // uses (toggling a sleep-flag byte in its own periodic wake frame). We have
  // no confirmed equivalent byte in ZOE_423 for this battery family, so
  // guessing one here would be exactly the kind of unverified change we've
  // avoided everywhere else in this driver. Instead: after the two write
  // sequences below complete, normal polling simply resumes - "letting it
  // sleep" (12V/contactor removal) is done manually, outside the firmware.
  // -------------------------------------------------------------------------
  bool UserRequestNVROLReset = false;
  uint8_t NVROLstateMachine = 0;
  unsigned long startTimeNVROL = 0;
  void transmit_reset_nvrol_frames(void);

  // Per-step result of the last NVROL sequence (index 0=open session,
  // 1=RoutineControl B009, 2=open session again, 3=WriteDataByIdentifier
  // 9281), shown on "More Battery Info" - lets us see whether each step was
  // accepted, actively rejected (with the BMS's own SID/NRC code), or never
  // answered at all, instead of sending blind like the first attempt.
  static const uint8_t NVROL_LOG_STEPS = 4;
  char nvrol_log[NVROL_LOG_STEPS][48] = {"not run yet", "not run yet", "not run yet", "not run yet"};
  uint8_t nvrol_awaiting_step = 0;  // Which nvrol_log[] slot the next 0x18DAF1DB reply belongs to
  void handle_nvrol_reply(CAN_frame rx_frame);

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
