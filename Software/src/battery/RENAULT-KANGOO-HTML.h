#ifndef _RENAULT_KANGOO_HTML_H
#define _RENAULT_KANGOO_HTML_H

#include <Arduino.h>
#include "../devboard/webserver/BatteryHtmlRenderer.h"

// Values decoded by the Renault Kangoo driver that have no field in the common datalayer.
// One instance lives inside each RenaultKangooBattery, so battery 2 shows its own data.
struct KangooExtraData {
  bool rx_only = false;

  // Broadcast (no request needed)
  bool soc_broadcast_seen = false;
  uint8_t soc_broadcast_pct = 0;  // 0x654 byte 3
  bool soh_broadcast_seen = false;
  uint8_t soh_broadcast_pct = 0;  // 0x658 byte 4 & 0x7F
  bool temp_60d_seen = false;     // 0x60D byte 4/5 minus 40 (candidate, not verified)
  int16_t temp_60d_a = 0;
  int16_t temp_60d_b = 0;

  // Polled: PID 0x01 ("generic data")
  bool pid01_seen = false;
  float soc_polled_pct = 0;  // "SOC CAN Output"
  int32_t current_mA = 0;    // sign convention not verified
  float charge_power_kW = 0;
  float discharge_power_kW = 0;
  float pack_voltage_pid01_V = 0;
  uint16_t aux_12v_mV = 0;
  uint16_t internal_resistance_mOhm = 0;
  float degradation_coefficient = 0;
  float full_capacity_Ah = 0;

  // Polled: PID 0x42 tail (two total pack voltage values after the cells)
  bool pack_voltage_pid42_seen = false;
  float pack_voltage_pid42_a_V = 0;
  float pack_voltage_pid42_b_V = 0;

  // Polled: PID 0x03
  bool pid03_seen = false;
  uint16_t cell_max_pid03_mV = 0;
  uint16_t cell_min_pid03_mV = 0;

  // Polled: PID 0x04
  bool pid04_seen = false;
  int8_t temp_pid04[4] = {0, 0, 0, 0};
  int8_t temp_min_pid04 = 0;

  // Computed by the driver from the 96 polled cell voltages
  bool cells_valid = false;
  float cell_sum_V = 0;
  uint16_t cell_min_calc_mV = 0;
  uint16_t cell_max_calc_mV = 0;
};

class RenaultKangooHtmlRenderer : public BatteryHtmlRenderer {
 public:
  explicit RenaultKangooHtmlRenderer(const KangooExtraData* d) : data(d) {}

  bool renders_own_battery_data() { return true; }

  String get_status_html() {
    String content;
    const KangooExtraData& d = *data;

    content += "<h4>Mode: ";
    content += d.rx_only ? "RX only (nothing is transmitted, no polled values)" : "polling";
    content += "</h4>";

    content += "<h4>SOC broadcast 0x654: ";
    content += d.soc_broadcast_seen ? String(d.soc_broadcast_pct) + " %" : String("n/a");
    content += "</h4>";
    content += "<h4>SOC polled PID 0x01: ";
    content += d.pid01_seen ? String(d.soc_polled_pct, 2) + " %" : String("n/a");
    content += "</h4>";
    content += "<h4>SOH broadcast 0x658: ";
    content += d.soh_broadcast_seen ? String(d.soh_broadcast_pct) + " %" : String("n/a");
    content += "</h4>";

    content += "<h4>Pack voltage, sum of cells: ";
    content += d.cells_valid ? String(d.cell_sum_V, 2) + " V" : String("n/a");
    content += "</h4>";
    content += "<h4>Pack voltage PID 0x01: ";
    content += d.pid01_seen ? String(d.pack_voltage_pid01_V, 2) + " V" : String("n/a");
    content += "</h4>";
    content += "<h4>Pack voltage PID 0x42 tail: ";
    content += d.pack_voltage_pid42_seen
                   ? String(d.pack_voltage_pid42_a_V, 2) + " V / " + String(d.pack_voltage_pid42_b_V, 2) + " V"
                   : String("n/a");
    content += "</h4>";

    content += "<h4>Cell min/max computed: ";
    content += d.cells_valid ? String(d.cell_min_calc_mV) + " mV / " + String(d.cell_max_calc_mV) + " mV"
                             : String("n/a");
    content += "</h4>";
    content += "<h4>Cell min/max PID 0x03: ";
    content += d.pid03_seen ? String(d.cell_min_pid03_mV) + " mV / " + String(d.cell_max_pid03_mV) + " mV"
                            : String("n/a");
    content += "</h4>";

    content += "<h4>Battery temperatures PID 0x04: ";
    if (d.pid04_seen) {
      content += String((int)d.temp_pid04[0]) + " / " + String((int)d.temp_pid04[1]) + " / " +
                 String((int)d.temp_pid04[2]) + " / " + String((int)d.temp_pid04[3]) + " &deg;C, min " +
                 String((int)d.temp_min_pid04) + " &deg;C";
    } else {
      content += "n/a";
    }
    content += "</h4>";
    content += "<h4>Battery temperatures 0x60D (candidate): ";
    content += d.temp_60d_seen ? String(d.temp_60d_a) + " / " + String(d.temp_60d_b) + " &deg;C" : String("n/a");
    content += "</h4>";

    content += "<h4>Charge power limit PID 0x01: ";
    content += d.pid01_seen ? String(d.charge_power_kW, 2) + " kW" : String("n/a");
    content += "</h4>";
    content += "<h4>Discharge power limit PID 0x01: ";
    content += d.pid01_seen ? String(d.discharge_power_kW, 2) + " kW" : String("n/a");
    content += "</h4>";
    content += "<h4>Current PID 0x01 (sign not verified): ";
    content += d.pid01_seen ? String(d.current_mA / 1000.0f, 3) + " A" : String("n/a");
    content += "</h4>";

    content += "<h4>12V battery: ";
    content += d.pid01_seen ? String(d.aux_12v_mV) + " mV" : String("n/a");
    content += "</h4>";
    content += "<h4>Internal resistance: ";
    content += d.pid01_seen ? String(d.internal_resistance_mOhm) + " mOhm" : String("n/a");
    content += "</h4>";
    content += "<h4>Degradation coefficient: ";
    content += d.pid01_seen ? String(d.degradation_coefficient, 4) : String("n/a");
    content += "</h4>";
    content += "<h4>Present full capacity: ";
    content += d.pid01_seen ? String(d.full_capacity_Ah, 2) + " Ah" : String("n/a");
    content += "</h4>";

    return content;
  }

 private:
  const KangooExtraData* data;
};

#endif
