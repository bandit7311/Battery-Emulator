#include <string.h>
#include "RENAULT-KANGOO-BATTERY.h"
#include <Arduino.h>
#include "../battery/BATTERIES.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"

/* TODO:
There seems to be some values on the Kangoo that differ between the 22/33 kWh version
- Find some way to autodetect which Kangoo size we are working with
- Fix the mappings of values accordingly
- Values still need fixing
  - SOC% is not valid on all packs (added estimation)
  - Max charge power is 0W on some packs (Added estimation)
  - SOH% is too high on some packs
  - Add all cellvoltages from https://github.com/jamiejones85/Kangoo36_canDecode/tree/main

This page has info on the larger 33kWh pack: https://openinverter.org/wiki/Renault_Kangoo_36

Notes for the 22kWh pack (verified against a real CAN log and a diagnostic tool dump of that vehicle):
- The broadcasts 0x155, 0x424, 0x425 and 0x445 are NOT present on its bus.
- SOC is available as broadcast 0x654 (byte 3) and polled in PID 0x01. SOH as broadcast 0x658.
- The LBC answers service 0x21 requests on 0x79B/0x7BB: PID 0x01 (generic data), 0x41/0x42 (cell voltages),
  0x03 (min/max cell voltage) and 0x04 (battery temperatures).
*/

static inline uint16_t be16(const uint8_t* p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

static inline uint32_t be32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

uint16_t estimate_SOC_from_voltage(uint16_t voltage) {
  uint16_t result = 0;
  //Voltage ranges between 4000dV when full, and 3000dV when empty
  result = (voltage - 3000);  //Make the range
  result = result * 10;       //Add decimal
  return result;
}

void RenaultKangooBattery::
    update_values() {  //This function maps all the values fetched via CAN to the correct parameters

  // SOC: the 0x654 broadcast works with and without polling, the polled PID 0x01 value is the fallback
  if (user_selected_use_estimated_SOC) {
    datalayer_battery->status.real_soc = estimate_SOC_from_voltage(datalayer_battery->status.voltage_dV);
  } else if (extras.soc_broadcast_seen) {
    datalayer_battery->status.real_soc = (LB_SOC * 100);  //increase LB_SOC range from 0-100 -> 100.00
  } else if (extras.pid01_seen) {
    datalayer_battery->status.real_soc = LB_SOC_polled_pptt;
  } else {
    datalayer_battery->status.real_soc = 0;
  }

  datalayer_battery->status.soh_pptt = (LB_SOH * 100);  //Increase range from 99% -> 99.00%
  if (datalayer_battery->status.soh_pptt > 10000) {     // Cap value if glitched out
    datalayer_battery->status.soh_pptt = 10000;
  }

  // Cell values from the 96 polled cell voltages (PID 0x41/0x42)
  uint32_t sum_cell_mV = 0;
  uint16_t min_mV = datalayer_battery->status.cell_voltages_mV[0];
  uint16_t max_mV = datalayer_battery->status.cell_voltages_mV[0];
  uint8_t populated_cells = 0;
  for (uint8_t i = 0; i < 96; i++) {
    uint16_t v = datalayer_battery->status.cell_voltages_mV[i];
    sum_cell_mV += v;
    if (v < min_mV) {
      min_mV = v;
    }
    if (v > max_mV) {
      max_mV = v;
    }
    if (v > 0) {
      populated_cells++;
    }
  }
  extras.cells_valid = (populated_cells == 96);
  extras.cell_sum_V = sum_cell_mV / 1000.0f;
  extras.cell_min_calc_mV = min_mV;
  extras.cell_max_calc_mV = max_mV;

  if (rx_only && !extras.cells_valid) {
    //No individual cell data (nobody polled them). The 0x445 broadcast does not exist on the 22kWh pack,
    //so without it there is no source and the values stay 0 instead of showing a made up default.
    if (cell_broadcast_seen) {
      uint16_t avg_cellvoltage = (LB_Cell_Min_Voltage + LB_Cell_Max_Voltage) / 2;
      datalayer_battery->status.voltage_dV = (avg_cellvoltage * 96) / 100;
      datalayer_battery->status.cell_min_voltage_mV = LB_Cell_Min_Voltage;
      datalayer_battery->status.cell_max_voltage_mV = LB_Cell_Max_Voltage;
    } else {
      datalayer_battery->status.voltage_dV = 0;
      datalayer_battery->status.cell_min_voltage_mV = 0;
      datalayer_battery->status.cell_max_voltage_mV = 0;
    }
  } else {
    //Sum of all 96 real cell voltages for the pack voltage, min/max directly from the same cells
    datalayer_battery->status.voltage_dV = sum_cell_mV / 100;
    datalayer_battery->status.cell_min_voltage_mV = min_mV;
    datalayer_battery->status.cell_max_voltage_mV = max_mV;
  }

  datalayer_battery->status.current_dA = LB_Current * 10;

  datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);

  if (user_selected_use_estimated_charge_limits) {  //Some packs are locked? and do not report allowed charge/discharge power
    datalayer_battery->status.max_charge_power_W = datalayer_battery->status.override_charge_power_W;

    datalayer_battery->status.max_discharge_power_W = datalayer_battery->status.override_discharge_power_W;
  } else {
    //Limits from the polled PID 0x01 (input/output possible power, 0.01 kW per bit). Without polling they stay 0.
    datalayer_battery->status.max_discharge_power_W = LB_Discharge_Power_Limit_W;
    datalayer_battery->status.max_charge_power_W = LB_Charge_Power_Limit_W;
  }

  // Temperature: polled PID 0x04 (4 sensors + minimum) when available, otherwise the 0x60D broadcast candidate
  int16_t temp_min_C;
  int16_t temp_max_C;
  if (!rx_only && extras.pid04_seen) {
    temp_min_C = extras.temp_min_pid04;
    temp_max_C = extras.temp_pid04[0];
    for (uint8_t i = 1; i < 4; i++) {
      if (extras.temp_pid04[i] > temp_max_C) {
        temp_max_C = extras.temp_pid04[i];
      }
    }
  } else if (extras.temp_60d_seen) {
    temp_min_C = (extras.temp_60d_a < extras.temp_60d_b) ? extras.temp_60d_a : extras.temp_60d_b;
    temp_max_C = (extras.temp_60d_a < extras.temp_60d_b) ? extras.temp_60d_b : extras.temp_60d_a;
  } else {
    temp_min_C = LB_MIN_TEMPERATURE;
    temp_max_C = LB_MAX_TEMPERATURE;
  }
  datalayer_battery->status.temperature_min_dC = (temp_min_C * 10);
  datalayer_battery->status.temperature_max_dC = (temp_max_C * 10);
}

void RenaultKangooBattery::process_iso_tp_response() {
  const uint8_t* b = iso_tp_buffer;
  const uint16_t n = iso_tp_bytes_received;

  if (n < 2 || b[0] != 0x61) {  // Only positive responses to service 0x21
    return;
  }

  switch (b[1]) {  // The PID the LBC answered
    case 0x01:     // Generic data, 53 bytes
      if (n >= 53) {
        extras.current_mA = (int32_t)be32(&b[8]);
        extras.charge_power_kW = be16(&b[24]) * 0.01f;
        extras.discharge_power_kW = be16(&b[26]) * 0.01f;
        LB_Charge_Power_Limit_W = (uint32_t)be16(&b[24]) * 10;  // 0.01 kW = 10 W per bit
        LB_Discharge_Power_Limit_W = (uint32_t)be16(&b[26]) * 10;
        extras.pack_voltage_pid01_V = be16(&b[28]) * 0.01f;
        extras.aux_12v_mV = be16(&b[30]);
        extras.internal_resistance_mOhm = be16(&b[34]);
        extras.degradation_coefficient = be16(&b[36]) * 0.0001f;
        uint32_t soc_raw = be32(&b[38]);  // 0.0001 % per bit
        extras.soc_polled_pct = soc_raw * 0.0001f;
        LB_SOC_polled_pptt = (uint16_t)(soc_raw / 100);  // 0.01 % units
        extras.full_capacity_Ah = be32(&b[42]) * 0.0001f;
        extras.pid01_seen = true;
      }
      break;
    case 0x03:  // Max/min cell voltage
      if (n >= 16) {
        extras.cell_max_pid03_mV = be16(&b[12]);
        extras.cell_min_pid03_mV = be16(&b[14]);
        extras.pid03_seen = true;
      }
      break;
    case 0x04:  // Battery temperatures, raw byte is degrees C (no offset)
      if (n >= 15) {
        extras.temp_pid04[0] = (int8_t)b[4];
        extras.temp_pid04[1] = (int8_t)b[7];
        extras.temp_pid04[2] = (int8_t)b[10];
        extras.temp_pid04[3] = (int8_t)b[13];
        extras.temp_min_pid04 = (int8_t)b[14];
        extras.pid04_seen = true;
      }
      break;
    case 0x41:  // Cell voltages 1-62, 2 bytes each, mV
      if (n >= 126) {
        for (uint8_t i = 0; i < 62; i++) {
          datalayer_battery->status.cell_voltages_mV[i] = be16(&b[2 + 2 * i]);
        }
      }
      break;
    case 0x42:  // Cell voltages 63-96, 2 bytes each, mV, followed by two total pack voltage values
      if (n >= 70) {
        for (uint8_t i = 0; i < 34; i++) {
          datalayer_battery->status.cell_voltages_mV[62 + i] = be16(&b[2 + 2 * i]);
        }
      }
      if (n >= 74) {
        extras.pack_voltage_pid42_a_V = be16(&b[70]) * 0.01f;
        extras.pack_voltage_pid42_b_V = be16(&b[72]) * 0.01f;
        extras.pack_voltage_pid42_seen = true;
      }
      break;
    default:
      break;
  }
}

void RenaultKangooBattery::handle_incoming_can_frame(CAN_frame rx_frame) {

  // Answers to the measurement list (LBC 0x18DAF1DB, EVC 0x7EC, PEB 0x77E). Only evaluated while a run is active.
  measure.on_frame(rx_frame.ID, rx_frame.ext_ID, rx_frame.data.u8, rx_frame.DLC);
  // Answer to the time button (LBC 0x18DAF1DB). Only evaluated while a time query is active.
  quick.on_frame(rx_frame.ID, rx_frame.ext_ID, rx_frame.data.u8, rx_frame.DLC);
  // Vehicle state of frame 0x350 (state, minute counter, ignition off and bus end), shown on the advanced page.
  vehicle.on_frame(rx_frame.ID, rx_frame.ext_ID, rx_frame.data.u8, rx_frame.DLC, (uint32_t)millis());

  switch (rx_frame.ID) {
    case 0x155:  //BMS1 (not present on the 22kWh pack)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_MaxChargeAllowed_W = (rx_frame.data.u8[0] * 300);
      LB_Current = word((rx_frame.data.u8[1] & 0xF), rx_frame.data.u8[2]) * 0.25 - 500;  //OK!
      LB_SOC = ((rx_frame.data.u8[4] << 8) | (rx_frame.data.u8[5])) * 0.0025;            //OK!
      extras.soc_broadcast_seen = true;
      extras.soc_broadcast_pct = (uint8_t)LB_SOC;
      break;
    case 0x424:  //BMS2 (not present on the 22kWh pack)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_EOCR = (rx_frame.data.u8[0] & 0x03);
      LB_HVBUV = (rx_frame.data.u8[0] & 0x0C) >> 2;
      LB_HVBIR = (rx_frame.data.u8[0] & 0x30) >> 4;
      LB_CUV = (rx_frame.data.u8[0] & 0xC0) >> 6;
      LB_COV = (rx_frame.data.u8[1] & 0x03);
      LB_HVBOV = (rx_frame.data.u8[1] & 0x0C) >> 2;
      LB_HVBOT = (rx_frame.data.u8[1] & 0x30) >> 4;
      LB_HVBOC = (rx_frame.data.u8[1] & 0xC0) >> 6;
      LB_MaxInput_kW = rx_frame.data.u8[2] / 2;
      LB_MaxOutput_kW = rx_frame.data.u8[3] / 2;
      LB_SOH = (rx_frame.data.u8[5]);                     // Only seems valid on Kangoo33?
      LB_MIN_TEMPERATURE = ((rx_frame.data.u8[4]) - 40);  //OK!
      LB_MAX_TEMPERATURE = ((rx_frame.data.u8[7]) - 40);  //OK!
      break;
    case 0x654:  //SOC in percent, byte 3 (present on the 22kWh pack, source: OVMS vehicle_renaultzoe)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_SOC = rx_frame.data.u8[3];
      extras.soc_broadcast_seen = true;
      extras.soc_broadcast_pct = rx_frame.data.u8[3];
      break;
    case 0x658:  //SOH in percent, byte 4 with the top bit masked (present on the 22kWh pack, source: OVMS)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_SOH = (rx_frame.data.u8[4] & 0x7F);
      extras.soh_broadcast_seen = true;
      extras.soh_broadcast_pct = (uint8_t)LB_SOH;
      break;
    case 0x60D:  //Battery temperatures as byte 4/5 minus 40 (candidate found by log analysis, not verified)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      extras.temp_60d_a = (int16_t)rx_frame.data.u8[4] - 40;
      extras.temp_60d_b = (int16_t)rx_frame.data.u8[5] - 40;
      extras.temp_60d_seen = true;
      break;
    case 0x425:  //(not present on the 22kWh pack)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_kWh_Remaining = word((rx_frame.data.u8[0] & 0x1), rx_frame.data.u8[1]) / 10;  //OK!
      break;
    case 0x445:  //(not present on the 22kWh pack)
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS
      LB_Cell_Max_Voltage = 1000 + word((rx_frame.data.u8[3] & 0x1), rx_frame.data.u8[4]) * 10;  //OK!
      LB_Cell_Min_Voltage = 1000 + (word(rx_frame.data.u8[5], rx_frame.data.u8[6]) >> 7) * 10;   //OK!
      cell_broadcast_seen = true;

      if ((LB_Cell_Max_Voltage == 6110) or (LB_Cell_Min_Voltage == 6110)) {  //Read Error
        LB_Cell_Max_Voltage = 3880;
        LB_Cell_Min_Voltage = 3880;
        break;
      }
      break;
    case 0x7BB: {
      datalayer_battery->status.CAN_battery_still_alive =
          CAN_STILL_ALIVE;  //Indicate that we are still getting CAN messages from the BMS

      // ISO-TP reassembly. The PID is taken from the response itself, so it does not depend on our poll state.
      const uint8_t pci = rx_frame.data.u8[0] & 0xF0;
      if (pci == 0x10) {  //First frame: 12 bit length, then SID, PID and 4 data bytes
        if (!rx_only) {
          transmit_can_frame(&KANGOO_79B_Continue);
        }
        iso_tp_expected = ((rx_frame.data.u8[0] & 0x0F) << 8) | rx_frame.data.u8[1];
        if (iso_tp_expected < 8 || iso_tp_expected > sizeof(iso_tp_buffer)) {
          iso_tp_expected = 0;
          iso_tp_bytes_received = 0;
          break;
        }
        for (uint8_t i = 0; i < 6; i++) {
          iso_tp_buffer[i] = rx_frame.data.u8[2 + i];
        }
        iso_tp_bytes_received = 6;
      } else if (pci == 0x20 && iso_tp_expected > 0) {  //Continuation frame, sequence number 0x20-0x2F wraps
        for (uint8_t i = 1; i <= 7 && iso_tp_bytes_received < iso_tp_expected; i++) {
          iso_tp_buffer[iso_tp_bytes_received++] = rx_frame.data.u8[i];
        }
        if (iso_tp_bytes_received >= iso_tp_expected) {
          process_iso_tp_response();
          iso_tp_expected = 0;
          iso_tp_bytes_received = 0;
        }
      }
      break;
    }
    default:
      break;
  }
}

void RenaultKangooBattery::sync_measure_extras() {
  extras.measure_state = (uint8_t)measure.get_state();
  extras.measure_ok = measure.get_ok();
  extras.measure_total = KangooMeasureList::COUNT;
  for (uint8_t i = 0; i < KangooMeasureList::COUNT; i++) {
    const KangooMeasureList::Answer& a = measure.answer(i);
    extras.measure_status[i] = a.status;
    extras.measure_len[i] = a.len;
    extras.measure_nrc[i] = a.nrc;
    extras.measure_raw[i] = a.raw;
  }
  extras.quick_state = (uint8_t)quick.get_state();
  extras.quick_result = (uint8_t)quick.get_result();
  extras.quick_attempts = quick.get_attempts();
  extras.quick_len = quick.get_len();
  extras.quick_nrc = quick.get_nrc();
  extras.quick_raw = quick.get_raw();
}

size_t RenaultKangooBattery::live_text(char* out, size_t n) {
  return live.format(out, n, (uint32_t)millis(), vehicle, quick.busy());
}

void RenaultKangooBattery::live_action(const char* cmd, int value) {
  if (cmd == nullptr) {
    return;
  }
  if (strcmp(cmd, "time") == 0) {
    run_quick_time_query();
  } else if (strcmp(cmd, "auto") == 0) {
    live.set_auto(value != 0);
  } else if (strcmp(cmd, "clear") == 0) {
    live.clear_results();
  }
}

void RenaultKangooBattery::sync_vehicle_state_extras(unsigned long currentMillis) {
  const uint32_t now = (uint32_t)currentMillis;
  vehicle.update(now);
  extras.vs_seen = vehicle.seen();
  extras.vs_state = vehicle.state();
  extras.vs_counter = vehicle.counter();
  extras.vs_state_age_ms = vehicle.state_age_ms(now);
  extras.vs_shutdown_active = vehicle.shutdown_active();
  extras.vs_shutdown_finished = vehicle.shutdown_finished();
  extras.vs_shutdown_age_ms = vehicle.shutdown_age_ms(now);
  extras.vs_bus_silent_ms = vehicle.bus_silent_ms(now);
}

void RenaultKangooBattery::transmit_can(unsigned long currentMillis) {
  // Update the displayed vehicle state first: this runs in every mode and sends nothing.
  sync_vehicle_state_extras(currentMillis);

  // Measurement list (button on the advanced page): read requests are sent ONLY after a button press. This sits
  // in front of the rx_only return on purpose - RX only silences the periodic traffic below, not this button.
  {
    KangooMeasureList::Request req;
    if (measure.poll(currentMillis, req)) {
      CAN_frame f = {.FD = false, .ext_ID = req.ext, .DLC = 8, .ID = req.id};
      memcpy(f.data.u8, req.data, 8);
      transmit_can_frame(&f);
    }
    // Time button: one request 22 92 61, repeated once after 300 ms without an answer. Not at the same time as the list.
    KangooQuickQuery::Request qreq;
    if (quick.poll(currentMillis, qreq)) {
      CAN_frame f = {.FD = false, .ext_ID = qreq.ext, .DLC = 8, .ID = qreq.id};
      memcpy(f.data.u8, qreq.data, 8);
      transmit_can_frame(&f);
    }
    // Result list and automatic schedule of the live page. The automatic schedule is OFF after every start and sends
    // only the same request 22 92 61, one at a time, never while another request is running.
    const uint32_t now32 = (uint32_t)currentMillis;
    if (live.query_pending() && !quick.busy() && quick.get_state() == KangooQuickQuery::DONE) {
      live.end_query((uint8_t)quick.get_result(),
                     quick.get_result() == KangooQuickQuery::NEGATIVE ? quick.get_nrc() : quick.get_raw(),
                     quick.get_attempts());
    }
    if (live.auto_poll(now32, vehicle, !measure.busy() && !quick.busy())) {
      live.begin_query(now32, vehicle, true);
      quick.request_start();
    }
    sync_measure_extras();
  }

  if (rx_only) {  // Never send anything on the bus in this mode - pure listener
    return;
  }
  // Send 100ms CAN Message (for 2.4s, then pause 10s)
  if ((currentMillis - previousMillis100) >= (INTERVAL_100_MS + GVL_pause)) {
    previousMillis100 = currentMillis;
    transmit_can_frame(&KANGOO_423);
    GVI_Pollcounter++;
    GVL_pause = 0;
    if (GVI_Pollcounter >= 24) {
      GVI_Pollcounter = 0;
      GVL_pause = 10000;
    }
  }
  // 1000ms CAN handling
  if (currentMillis - previousMillis1000 >= INTERVAL_1_S) {
    previousMillis1000 = currentMillis;

    // One request per second, cycling through the PIDs we decode
    static const uint8_t poll_pids[POLL_PID_COUNT] = {0x01, 0x41, 0x42, 0x03, 0x04};
    KANGOO_79B_Poll.data.u8[2] = poll_pids[poll_index];
    poll_index = (poll_index + 1) % POLL_PID_COUNT;

    if (UserRequestDTCreset) {
      UserRequestDTCreset = false;
      transmit_can_frame(&KANGOO_CLEAR_DTC);
    } else {  //Normal polling if no DTC reset request
      transmit_can_frame(&KANGOO_79B_Poll);
    }
  }
}

void RenaultKangooBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  if (allows_contactor_closing) {
    *allows_contactor_closing = true;
  }
  datalayer_battery->info.number_of_cells = 96;
  datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
}
