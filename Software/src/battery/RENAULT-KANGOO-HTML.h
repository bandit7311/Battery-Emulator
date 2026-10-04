#ifndef _RENAULT_KANGOO_HTML_H
#define _RENAULT_KANGOO_HTML_H

#include <Arduino.h>
#include "../devboard/webserver/BatteryHtmlRenderer.h"
#include "RENAULT-KANGOO-MEASURE.h"
#include "RENAULT-KANGOO-QUICKQUERY.h"
#include "RENAULT-KANGOO-VEHICLESTATE.h"

// Values decoded by the Renault Kangoo driver that have no field in the common datalayer.
// One instance lives inside each RenaultKangooBattery, so battery 2 shows its own data.
struct KangooExtraData {
  bool rx_only = false;

  // Vehicle state from frame 0x350 (state, minute counter, shutdown after ignition off), ages in milliseconds
  bool vs_seen = false;
  uint8_t vs_state = 0;
  uint32_t vs_counter = 0;
  uint32_t vs_state_age_ms = 0;
  bool vs_shutdown_active = false;
  bool vs_shutdown_finished = false;
  uint32_t vs_shutdown_age_ms = 0;
  uint32_t vs_bus_silent_ms = 0;

  // Measurement list (button on the advanced page, RX only mode): 0 = not run, 1 = running, 2 = finished
  uint8_t measure_state = 0;
  uint8_t measure_ok = 0;
  uint8_t measure_total = 0;

  // Answers of the last run of the measurement list (status: 0 not asked, 1 answered, 2 negative, 3 no answer)
  uint8_t measure_status[KangooMeasureList::COUNT] = {0};
  uint8_t measure_len[KangooMeasureList::COUNT] = {0};
  uint8_t measure_nrc[KangooMeasureList::COUNT] = {0};
  uint32_t measure_raw[KangooMeasureList::COUNT] = {0};

  // Time button (22 92 61): state 0 idle, 1 waiting, 2 done; result 0 none, 1 ok, 2 negative, 3 no answer
  uint8_t quick_state = 0;
  uint8_t quick_result = 0;
  uint8_t quick_attempts = 0;
  uint8_t quick_len = 0;
  uint8_t quick_nrc = 0;
  uint32_t quick_raw = 0;

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

    // Vehicle state of frame 0x350, with the shutdown after ignition off. Helps to time the button presses.
    {
      content += "<h4>Vehicle state 0x350: ";
      if (!d.vs_seen) {
        content += "not seen";
      } else {
        char code[8];
        snprintf(code, sizeof(code), "%02X", (unsigned)d.vs_state);
        content += String(code);
        if (d.vs_shutdown_active || d.vs_shutdown_finished) {
          const char* name = KangooVehicleState::stage_name(d.vs_state);
          if (name[0] != 0) {
            content += " (" + String(name) + ")";
          }
        }
        content += ", " + String((unsigned long)(d.vs_state_age_ms / 1000)) + " s in this state";
      }
      content += "</h4>";
      content += "<h4>Minute counter 0x350: ";
      content += d.vs_seen ? String((unsigned long)d.vs_counter) : String("n/a");
      content += "</h4>";
      content += "<h4>Bus (frame 0x350): ";
      if (!d.vs_seen) {
        content += "no frame seen yet";
      } else if (d.vs_bus_silent_ms >= KangooVehicleState::BUS_SILENT_MS) {
        content += "silent for " + String((unsigned long)(d.vs_bus_silent_ms / 1000)) + " s";
      } else {
        content += "active";
      }
      content += "</h4>";
      content += "<h4>Shutdown after ignition off: ";
      if (d.vs_shutdown_active) {
        // rounded to whole seconds like the reference points (C2 starts at 63.8 s and counts as 64 s)
        const uint32_t age_s = (d.vs_shutdown_age_ms + 500) / 1000;
        content += "running for " + String((unsigned long)age_s) + " s. Reference points of one logged shutdown: C2 at " +
                   String((unsigned long)KangooVehicleState::REF_C2_S) + " s, C0 at " +
                   String((unsigned long)KangooVehicleState::REF_C0_S) + " s, 00 at " +
                   String((unsigned long)KangooVehicleState::REF_00_S) + " s, bus end at " +
                   String((unsigned long)KangooVehicleState::REF_END_S) + " s.";
        const char* label = nullptr;
        uint32_t in_s = 0;
        if (KangooVehicleState::next_reference(age_s, label, in_s)) {
          content += " Next: " + String(label) + " in about " + String((unsigned long)in_s) + " s.";
        } else {
          content += " Past the last reference point.";
        }
      } else if (d.vs_shutdown_finished) {
        content += "finished, the last 0x350 frame came " + String((unsigned long)((d.vs_shutdown_age_ms + 500) / 1000)) +
                   " s after ignition off";
      } else {
        content += "not running";
      }
      content += "</h4>";
    }

    if (d.rx_only) {
      content += "<h4><a href='/kangooLive' style='color:#9fd8ff;'>Open the live page</a> (state, shutdown, time button, results)</h4>";
      content += "<h4>Measurement list: ";
      if (d.measure_state == 1) {
        content += "running...";
      } else if (d.measure_state == 2) {
        content += (d.measure_ok == d.measure_total) ? "OK " : "NOK ";
        content += String(d.measure_ok) + "/" + String(d.measure_total);
      } else {
        content += "not run";
      }
      content += "</h4>";
      // Refresh only while a run is active (about one second). The live page /kangooLive follows the shutdown without
      // rebuilding this whole page (the reloads every 1.5 s during the shutdown made gaps in the serial log).
      if (d.measure_state == 1 || d.quick_state == 1) {
        content += "<script>setTimeout(function(){location.reload();},1500);</script>";
      }

      // Time button
      content += "<h4>Time query 22 92 61: ";
      if (d.quick_state == 1) {
        content += "running...";
      } else if (d.quick_result == 1) {
        content += "OK, " + String((unsigned long)d.quick_raw) + " min, raw 0x" + String((unsigned long)d.quick_raw, HEX);
        content += d.quick_attempts > 1 ? ", answered at the second try" : "";
      } else if (d.quick_result == 2) {
        content += "negative answer, NRC 0x" + String((unsigned int)d.quick_nrc, HEX);
      } else if (d.quick_result == 3) {
        content += "no answer (" + String(d.quick_attempts) + " tries)";
      } else {
        content += "not run";
      }
      content += "</h4>";

      // Values of the last run of the measurement list
      if (d.measure_state == 2) {
        content += "<h4>Values of the last measurement list:</h4>";
        for (uint8_t i = 0; i < KangooMeasureList::COUNT; i++) {
          content += "<h4>" + String(KangooMeasureList::label(i)) + ": ";
          if (d.measure_status[i] == KangooMeasureList::ANSWERED) {
            const uint32_t raw = d.measure_raw[i];
            if (KangooMeasureList::is_minutes(i)) {
              content += String((unsigned long)raw) + " min";
            } else if (KangooMeasureList::is_plain(i)) {
              content += String((unsigned long)raw);
            } else {
              content += String(KangooMeasureList::decode(i, raw), 3) + " " + String(KangooMeasureList::unit(i));
            }
            content += ", raw 0x" + String((unsigned long)raw, HEX);
          } else if (d.measure_status[i] == KangooMeasureList::NEGATIVE) {
            content += "negative answer, NRC 0x" + String((unsigned int)d.measure_nrc[i], HEX);
          } else if (d.measure_status[i] == KangooMeasureList::NO_ANSWER) {
            content += "no answer";
          } else {
            content += "not asked";
          }
          content += "</h4>";
        }
      }
    }

    content += "<h4>SOC broadcast 0x654: ";
    content += d.soc_broadcast_seen ? String(d.soc_broadcast_pct) + " &#37;" : String("n/a");
    content += "</h4>";
    content += "<h4>SOC polled PID 0x01: ";
    content += d.pid01_seen ? String(d.soc_polled_pct, 2) + " &#37;" : String("n/a");
    content += "</h4>";
    content += "<h4>SOH broadcast 0x658: ";
    content += d.soh_broadcast_seen ? String(d.soh_broadcast_pct) + " &#37;" : String("n/a");
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
