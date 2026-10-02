#ifndef RENAULT_KANGOO_BATTERY_H
#define RENAULT_KANGOO_BATTERY_H

#include "../datalayer/datalayer.h"
#include "CanBattery.h"
#include "RENAULT-KANGOO-HTML.h"

extern bool user_selected_use_estimated_SOC;

class RenaultKangooBattery : public CanBattery {
 public:
  // Use this constructor for the second battery.
  RenaultKangooBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr, CAN_Interface targetCan, bool rxOnly = false)
      : CanBattery(targetCan) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
    rx_only = rxOnly;
    extras.rx_only = rxOnly;
  }

  // Use the default constructor to create the first or single battery.
  RenaultKangooBattery() {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
  }

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "Renault Kangoo";

  bool supports_reset_DTC() { return true; }
  void reset_DTC() { UserRequestDTCreset = true; }

  BatteryHtmlRenderer& get_status_renderer() { return renderer; }

 private:
  // Decodes a complete, reassembled ISO-TP response (SID 0x61 + PID + data) from the LBC
  void process_iso_tp_response();

  DATALAYER_BATTERY_TYPE* datalayer_battery;
  bool* allows_contactor_closing;
  bool rx_only = false;
  bool UserRequestDTCreset = false;

  // Values without a datalayer field, and the renderer that shows them on the advanced page.
  // extras must be declared before renderer.
  KangooExtraData extras;
  RenaultKangooHtmlRenderer renderer{&extras};

  // ISO-TP reassembly: buffer holds the response starting with the SID (0x61), then the PID
  uint8_t iso_tp_buffer[140] = {0};
  uint16_t iso_tp_bytes_received = 0;
  uint16_t iso_tp_expected = 0;

  // Poll cycle, one request per second: PID 0x01, 0x41, 0x42, 0x03, 0x04
  static const uint8_t POLL_PID_COUNT = 5;
  uint8_t poll_index = 0;

  static const int MAX_PACK_VOLTAGE_DV = 4150;  //5000 = 500.0V
  static const int MIN_PACK_VOLTAGE_DV = 2500;
  static const int MAX_CELL_DEVIATION_MV = 150;
  static const int MAX_CELL_VOLTAGE_MV = 4250;  //Battery is put into emergency stop if one cell goes over this value
  static const int MIN_CELL_VOLTAGE_MV = 2700;  //Battery is put into emergency stop if one cell goes below this value

  uint32_t LB_MaxChargeAllowed_W = 0;
  int32_t LB_Current = 0;
  int16_t LB_MAX_TEMPERATURE = 0;
  int16_t LB_MIN_TEMPERATURE = 0;
  uint16_t LB_SOC = 0;
  uint16_t LB_SOH = 0;
  uint16_t LB_SOC_polled_pptt = 0;  // SOC from PID 0x01 in 0.01% units
  uint32_t LB_Charge_Power_Limit_W = 0;
  uint32_t LB_Discharge_Power_Limit_W = 0;
  uint16_t LB_kWh_Remaining = 0;
  uint16_t LB_Cell_Max_Voltage = 3700;
  uint16_t LB_Cell_Min_Voltage = 3700;
  bool cell_broadcast_seen = false;  // 0x445 has not been seen on the 22kWh pack
  uint8_t GVI_Pollcounter = 0;
  uint8_t LB_EOCR = 0;
  uint8_t LB_HVBUV = 0;
  uint8_t LB_HVBIR = 0;
  uint8_t LB_CUV = 0;
  uint8_t LB_COV = 0;
  uint8_t LB_HVBOV = 0;
  uint8_t LB_HVBOT = 0;
  uint8_t LB_HVBOC = 0;
  uint8_t LB_MaxInput_kW = 0;
  uint8_t LB_MaxOutput_kW = 0;

  CAN_frame KANGOO_423 = {.FD = false,
                          .ext_ID = false,
                          .DLC = 8,
                          .ID = 0x423,
                          .data = {0x0B, 0x1D, 0x00, 0x02, 0xB2, 0x20, 0xB2, 0xD9}};  // Charging
  // Driving: 0x07  0x1D  0x00  0x02  0x5D  0x80  0x5D  0xD8
  // Charging: 0x0B   0x1D  0x00  0x02  0xB2  0x20  0xB2  0xD9
  // Fastcharging: 0x07   0x1E  0x00  0x01  0x5D  0x20  0xB2  0xC7
  // Old hardcoded message: .data = {0x33, 0x00, 0xFF, 0xFF, 0x00, 0xE0, 0x00, 0x00}};
  CAN_frame KANGOO_79B_Poll = {.FD = false,
                               .ext_ID = false,
                               .DLC = 8,
                               .ID = 0x79B,
                               .data = {0x02, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame KANGOO_79B_Continue = {.FD = false,
                                   .ext_ID = false,
                                   .DLC = 8,
                                   .ID = 0x79B,
                                   .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame KANGOO_CLEAR_DTC = {.FD = false,
                                .ext_ID = false,
                                .DLC = 8,
                                .ID = 0x79B,
                                .data = {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00}};

  unsigned long previousMillis10 = 0;    // will store last time a 10ms CAN Message was sent
  unsigned long previousMillis100 = 0;   // will store last time a 100ms CAN Message was sent
  unsigned long previousMillis1000 = 0;  // will store last time a 1000ms CAN Message was sent
  unsigned long GVL_pause = 0;
};

#endif
