#include "RENAULT-TWINGO-GEN1-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/utils/common_functions.h"  // crc8_table_SAE_J1850_ZER0
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"
#ifndef UNIT_TEST
#include <WiFi.h>  // WiFi.status() for the NTP start
#endif
#include <time.h>

/* Information in this file is based of the OVMS V3 vehicle_renaultzoe.cpp component 
https://github.com/openvehicles/Open-Vehicle-Monitoring-System-3/blob/master/vehicle/OVMS.V3/components/vehicle_renaultzoe/src/vehicle_renaultzoe.cpp
The Zoe BMS apparently does not send total pack voltage, so we use the polled 96x cellvoltages summed up as total voltage
Still TODO:
- Automatically detect what vehicle and battery size we are on (Zoe 22/41 , Kangoo 33, Fluence ZE 22/36)

 Do not change code below unless you are sure what you are doing */
void RenaultTwingoGen1Battery::
    update_values() {  //This function maps all the values fetched via CAN to the correct parameters used for modbus
  datalayer_battery->status.soh_pptt = (LB_SOH * 100);  // Increase range from 99% -> 99.00%

  datalayer_battery->status.real_soc = (uint16_t)(LB_Display_SOC * 0.25f);  // 0.0025% per bit -> pptt (0.01% units)
  // Alternative: datalayer_battery->status.real_soc = (LB_SOC * 100); // Use raw BMS Chemical SOC% (0x654)

  datalayer_battery->status.current_dA = (((int32_t)LB_Current_raw * 10) / 4) - 5000;

  //Calculate the remaining Wh amount from SOC% and max Wh value.
  datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);

  datalayer_battery->status.max_discharge_power_W = LB_Discharge_allowed_W;

  datalayer_battery->status.max_charge_power_W = LB_Regen_allowed_W;

  datalayer_battery->status.temperature_min_dC = LB_Cell_minimum_temperature * 10;
  datalayer_battery->status.temperature_max_dC = LB_Cell_maximum_temperature * 10;

  if (LB_Cell_minimum_voltage < 4400) {  //Value is initialized large for some reason
    datalayer_battery->status.cell_min_voltage_mV = LB_Cell_minimum_voltage;
  }

  if (LB_Cell_maximum_voltage < 4400) {  //Value is initialized large for some reason
    datalayer_battery->status.cell_max_voltage_mV = LB_Cell_maximum_voltage;
  }

#ifdef TWINGO_EXTENDED_CELL_POLLING
  // Single pack temperature sensors (0x9131-0x9138): first drop every sensor that has stopped
  // answering, then - only if all 8 are plausible and fresh - derive pack min/max from them
  // (0.1 degC resolution) instead of the 0x424 broadcast values assigned further up (1 degC
  // resolution). Otherwise the broadcast values simply stay in place.
  {
    unsigned long now = millis();
    uint8_t mask = datalayer_battery->status.temperature_sensors_valid_mask;
    for (uint8_t i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
      if ((mask & (1u << i)) && ((now - ext_temp_last_ms[i]) > EXT_TEMP_STALE_MS)) {
        mask &= (uint8_t)~(1u << i);
      }
    }
    datalayer_battery->status.temperature_sensors_valid_mask = mask;
    if (mask == 0xFF) {
      int16_t t_min = datalayer_battery->status.temperature_sensors_dC[0];
      int16_t t_max = t_min;
      for (uint8_t i = 1; i < EXT_TEMP_SENSOR_COUNT; i++) {
        int16_t t = datalayer_battery->status.temperature_sensors_dC[i];
        if (t < t_min) {
          t_min = t;
        }
        if (t > t_max) {
          t_max = t;
        }
      }
      datalayer_battery->status.temperature_min_dC = t_min;
      datalayer_battery->status.temperature_max_dC = t_max;
    }
  }

  // Once every one of the 96 cells has replied at least once via the extended
  // channel, use the real per-cell values instead of the coarse 0x425-broadcast
  // approximation (10mV resolution, always a multiple of 10) for pack voltage,
  // cell min and cell max. Until then (e.g. the ~20s after boot before the
  // first full poll cycle completes), keep using the broadcast values set
  // above (already assigned to cell_min/max_voltage_mV further up).
  if (ext_cells_seen >= 96) {
    uint32_t summed_mV = 0;
    uint16_t min_mV = datalayer_battery->status.cell_voltages_mV[0];
    uint16_t max_mV = datalayer_battery->status.cell_voltages_mV[0];
    for (uint8_t i = 0; i < datalayer_battery->info.number_of_cells; ++i) {
      uint16_t v = datalayer_battery->status.cell_voltages_mV[i];
      summed_mV += v;
      if (v < min_mV) {
        min_mV = v;
      }
      if (v > max_mV) {
        max_mV = v;
      }
    }
    calculated_total_pack_voltage_mV = summed_mV;
    datalayer_battery->status.cell_min_voltage_mV = min_mV;
    datalayer_battery->status.cell_max_voltage_mV = max_mV;
  } else {
    calculated_total_pack_voltage_mV = ((LB_Cell_minimum_voltage + LB_Cell_maximum_voltage) / 2) * 96;
  }
#else
  calculated_total_pack_voltage_mV = ((LB_Cell_minimum_voltage + LB_Cell_maximum_voltage) / 2) * 96;
#endif
  datalayer_battery->status.voltage_dV = ((calculated_total_pack_voltage_mV / 100));  // mV to dV

#ifdef TWINGO_EXTENDED_CELL_POLLING
  // Sleep run in progress (Sleep / Sleep 0x9281=1 / NVROL reset, from the button press through the whole
  // shutdown sequence, true silence and wake burst - UserRequestNVROLReset is only cleared again by
  // finish_nvrol_silence() once the wake burst has fully completed). While the battery itself is not being
  // polled, current/voltage/SOC above stay frozen at their last polled values (LB_* simply stop updating) -
  // that's correct for voltage and SOC, but current must not keep reporting whatever it happened to be at
  // the moment of the last poll. Force current and both power limits to 0 so the inverter is actively told
  // "neither charge nor discharge" instead of computing from a stale, possibly non-zero current. The CAN
  // frames towards the inverter keep going at their normal rate throughout (see PylonInverter::transmit_can(),
  // which reads these fields unconditionally) - only their content changes here.
  if (UserRequestNVROLReset) {
    datalayer_battery->status.current_dA = 0;
    datalayer_battery->status.max_discharge_power_W = 0;
    datalayer_battery->status.max_charge_power_W = 0;
  }
#endif
}

uint16_t RenaultTwingoGen1Battery::handle_pid(uint16_t pid, uint32_t value, const uint8_t* data, uint16_t length) {
  // Called by the UDS superclass for every successful PID response. `data`
  // points at the raw value bytes, starting right after the echoed local
  // identifier (the response is `61 <local ID> <value...>`).
  switch (pid) {
    case GROUP1_CELLVOLTAGES_1_POLL:  // 0x41, cells 1-62
      if (length >= 124) {
        for (uint8_t cell = 0; cell < 62; cell++) {
          datalayer_battery->status.cell_voltages_mV[cell] = (data[cell * 2] << 8) | data[cell * 2 + 1];
        }
        // Cell 47 measurement is inbetween pack halves. If low, fuse blown
        if (datalayer_battery->status.cell_voltages_mV[47] < 100) {
          set_event(EVENT_BATTERY_FUSE, datalayer_battery->status.cell_voltages_mV[47]);
        } else {
          clear_event(EVENT_BATTERY_FUSE);
        }
      }
      break;
    case GROUP2_CELLVOLTAGES_2_POLL:  // 0x42, cells 63-96
      if (length >= 68) {
        for (uint8_t cell = 0; cell < 34; cell++) {
          datalayer_battery->status.cell_voltages_mV[62 + cell] = (data[cell * 2] << 8) | data[cell * 2 + 1];
        }
      }
      break;
    case GROUP3_METRICS:  // 0x61, mileage + alltime energy
      if (length >= 17) {
        battery_mileage_in_km = (data[11] << 8) | data[12];
        kWh_from_beginning_of_battery_life = (data[15] << 8) | data[16];
      }
      break;
    case GROUP6_BALANCING: {  // 0x07, one bit per cell, LSB first within each byte
      bool any_balancing = false;
      for (uint8_t cell = 0; cell < 96; cell++) {
        if ((cell >> 3) >= length) {
          break;
        }
        bool is_balancing = (data[cell >> 3] >> (cell & 7)) & 0x01;
        datalayer_battery->status.cell_balancing_status[cell] = is_balancing;
        if (is_balancing) {
          any_balancing = true;
        }
      }
      datalayer_battery->status.balancing_status = any_balancing ? BALANCING_STATUS_ACTIVE : BALANCING_STATUS_READY;
      break;
    }
    default:  //Unknown PID, ignore
      break;
  }
  return 0;  //Continue scanning the PID list in order
}

#ifdef TWINGO_EXTENDED_CELL_POLLING
// ---------------------------------------------------------------------------
// Extended-address (0x18DADBF1/0x18DAF1DB) reply handling. See the comment
// block at the top of RENAULT-TWINGO-GEN1-BATTERY.h for what this is and why
// it exists alongside (not instead of) the KWP2000 UDS code above.
// ---------------------------------------------------------------------------

// Single-frame reply (fits in one CAN frame): `data`/`length` point at the
// payload bytes after the echoed SID+PID.
void RenaultTwingoGen1Battery::handle_extended_single_frame(uint16_t pid, const uint8_t* data, uint16_t length) {
  // A DTC Read is in flight and this single-frame reply isn't a normal PID response at all: for a
  // positive ReadDTCInformation reply, byte[1]=SID(0x59), byte[2]=subfunction(0x02), byte[3]=mask(0x09)
  // - which the generic caller already reinterprets as "pid", high byte 0x02, coincidentally never a
  // real PID. 01.10. real captured reply: byte[3] (the DTCStatusAvailabilityMask the ECU reports it
  // supports) came back 0xFF, NOT an echo of the 0x09 we requested - per the UDS 0x19/0x02 spec this
  // byte is the ECU's own supported-status mask, not a request echo, so only the subfunction byte
  // (pid's high byte, 0x02) is checked now, not the full 0x0209 - the exact-match version silently
  // dropped a fully, correctly reassembled reply because of this.
  // For a negative response, byte[1]=0x7F, byte[2]=<requested SID>=0x19, byte[3]=NRC - so pid's high
  // byte is 0x19. Both are handled here instead of falling through to the PID switch below.
  if (dtc_ext_state == DTC_EXT_DETAILS_CMD_SENT) {
    // Raw bytes only, single-frame reply assumed (no multi-frame reassembly hooked up for this specific,
    // UNTESTED flow) - logged as hex exactly as received, nothing about the format assumed.
    char* out = dtc_ext_log_details[dtc_ext_details_index];
    size_t outsz = sizeof(dtc_ext_log_details[dtc_ext_details_index]);
    size_t pos = 0;
    for (uint16_t i = 0; i < length && pos + 3 < outsz; i++) {
      pos += snprintf(out + pos, outsz - pos, "%02X ", data[i]);
    }
    if (pos == 0) {
      strncpy(out, "(empty reply)", outsz - 1);
    }
    dtc_ext_details_index++;
    if (dtc_ext_details_index < DTC_DETAILS_COUNT) {
      dtc_ext_step_start_ms = millis();
      dtc_ext_state = DTC_EXT_DETAILS_SESSION_SENT;  // next DTC
    } else {
      ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
      dtc_ext_state = DTC_EXT_IDLE;
    }
    return;
  }
  if (dtc_ext_state == DTC_EXT_READ_CMD_SENT) {
    if ((pid >> 8) == 0x19) {
      snprintf(dtc_ext_log_read, sizeof(dtc_ext_log_read), "NEGATIVE SID=0x19 NRC=0x%02X", pid & 0xFF);
      ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
      dtc_ext_state = DTC_EXT_IDLE;
      return;
    }
    if ((pid >> 8) == 0x02) {
      handle_dtc_read_response(data, length);
      return;
    }
  }
  if (pid >= 0x9021 && pid <= 0x9083) {
    // Individual cell voltage. Three offsets (0x9040/0x9060/0x9080) are
    // skipped in the BMS's own PID numbering; account for that. Verified
    // against Battery-Emulator's own RENAULT-ZOE-GEN2-BATTERY.cpp, which
    // polls the same 96 PIDs on the same LBC protocol.
    if (length < 2) {
      return;
    }
    int16_t cell_index = (int16_t)pid - 0x9021;
    if (pid > 0x903F) {
      cell_index -= 1;  // Account for missing 0x9040
    }
    if (pid > 0x905F) {
      cell_index -= 1;  // Account for missing 0x9060
    }
    if (pid > 0x907F) {
      cell_index -= 1;  // Account for missing 0x9080
    }
    if (cell_index < 0 || cell_index >= 96) {
      return;  // Shouldn't happen given the range check above, but be safe.
    }
    uint16_t cell_mV = (uint16_t)(((data[0] << 8) | data[1]) * 0.976563f);
    unsigned long now = millis();
    if (ext_first_cell_reply_ms == 0) {
      ext_first_cell_reply_ms = (now != 0) ? now : 1;  // 0 is reserved for "no reply yet"
    }
    // Boot plausibility filter (see header): only 3.0-4.3V count until all 96 cells have been
    // read once or 60s have passed since the first cell reply. Dropped values are neither
    // stored nor counted; an already stored plausible value stays untouched.
    // The filter is also armed again for 60s after the NVROL quiet phase (BMS wake-up, see finish_nvrol_silence).
    bool boot_phase = ((ext_cells_seen < 96) && ((now - ext_first_cell_reply_ms) < EXT_BOOT_FILTER_TIMEOUT_MS)) ||
                      (ext_filter_rearm_active && ((now - ext_filter_rearm_start_ms) < EXT_BOOT_FILTER_TIMEOUT_MS));
    if (boot_phase && (cell_mV < EXT_BOOT_CELL_MV_MIN || cell_mV > EXT_BOOT_CELL_MV_MAX)) {
#ifdef EXTENDED_UDS_DEBUG
      logging.printf("EXT UDS: boot filter dropped cell %d = %u mV\n", cell_index + 1, (unsigned)cell_mV);
#endif
      return;
    }
    if (datalayer_battery->status.cell_voltages_mV[cell_index] == 0 && ext_cells_seen < 96) {
      ext_cells_seen++;
    }
    datalayer_battery->status.cell_voltages_mV[cell_index] = cell_mV;
    if (datalayer_extended.twingoGen1.cellwatch_enabled &&
        (cell_index + 1) == datalayer_extended.twingoGen1.cellwatch_cell) {
      datalayer_extended.twingoGen1.cellwatch_last_mV = cell_mV;
      datalayer_extended.twingoGen1.cellwatch_sample_count++;
      datalayer_extended.twingoGen1.cellwatch_last_sample_ms = (uint32_t)now;
    }
    return;
  }

  if (pid >= EXT_POLL_TEMP_FIRST && pid < (uint16_t)(EXT_POLL_TEMP_FIRST + EXT_TEMP_SENSOR_COUNT)) {
    // Pack temperature sensor 1-8: 16-bit raw, degC = raw * 0.0625 - 40 (same as OVMS RT32), stored
    // in d°C = raw * 0.625 - 400. Raw 0 (-40.0 degC) and everything above +100.0 degC (e.g. 0xFFFF)
    // is treated as "no valid value" - the sensor then shows "--" and blocks the min/max feed.
    if (length < 2) {
      return;
    }
    uint8_t sensor = (uint8_t)(pid - EXT_POLL_TEMP_FIRST);
    uint16_t raw = (uint16_t)((data[0] << 8) | data[1]);
    int32_t temp_dC = (int32_t)(raw * 0.625f + 0.5f) - 400;
    if (temp_dC > -400 && temp_dC <= 1000) {
      datalayer_battery->status.temperature_sensors_dC[sensor] = (int16_t)temp_dC;
      datalayer_battery->status.temperature_sensors_valid_mask |= (uint8_t)(1u << sensor);
      ext_temp_last_ms[sensor] = millis();
    } else {
      datalayer_battery->status.temperature_sensors_valid_mask &= (uint8_t)~(1u << sensor);
    }
    return;
  }

  switch (pid) {
    case EXT_POLL_TIME:  // 0x9261 and 0x91C1: shown raw, up to 4 data bytes of a single-frame reply
    case EXT_POLL_PACK_TIME:
      if (length >= 1) {
        uint8_t idx = (pid == EXT_POLL_PACK_TIME) ? 1 : 0;
        uint8_t n = (length > 4) ? 4 : (uint8_t)length;
        uint32_t v = 0;
        for (uint8_t i = 0; i < n; i++) {
          v = (v << 8) | data[i];
        }
        time_pid_raw[idx] = v;
        time_pid_len[idx] = n;
      }
      break;
    // Display-only PIDs from the real "RBMS_MCPU_RL" dumps (28.09.): every one of them is stored as the raw
    // big-endian value of however many data bytes the reply carries (1, 2 or 4 here), the formula from the
    // header comment is applied only when rendering the HTML page.
    case EXT_POLL_MILEAGE_PACK:
    case EXT_POLL_MILEAGE_VEHICLE:
    case EXT_POLL_LV_SUPPLY:
    case EXT_POLL_PACK_VOLTAGE:
    case EXT_POLL_CELL_V_A:
    case EXT_POLL_CELL_V_B:
    case EXT_POLL_CELL_V_A_NR:
    case EXT_POLL_CELL_V_B_NR:
    case EXT_POLL_SOH_AVG:
    case EXT_POLL_MAX_CHARGE_POWER:
    case EXT_POLL_MAX_GEN_POWER:
    case EXT_POLL_MAX_AVAIL_POWER:
    case EXT_POLL_SOC_AVG:
    case EXT_POLL_USOC_AVG:
    case EXT_POLL_SOC_MIN:
    case EXT_POLL_SOC_MAX:
    case EXT_POLL_BATTERY_CURRENT:
      if (length >= 1) {
        uint32_t v = 0;
        for (uint8_t i = 0; i < length; i++) {
          v = (v << 8) | data[i];
        }
        ExtValue* target;
        switch (pid) {
          case EXT_POLL_MILEAGE_PACK:
            target = &ext_mileage_pack;
            break;
          case EXT_POLL_MILEAGE_VEHICLE:
            target = &ext_mileage_vehicle;
            break;
          case EXT_POLL_LV_SUPPLY:
            target = &ext_lv_supply;
            break;
          case EXT_POLL_PACK_VOLTAGE:
            target = &ext_pack_voltage;
            break;
          case EXT_POLL_CELL_V_A:
            target = &ext_cell_v_a;
            break;
          case EXT_POLL_CELL_V_B:
            target = &ext_cell_v_b;
            break;
          case EXT_POLL_CELL_V_A_NR:
            target = &ext_cell_v_a_nr;
            break;
          case EXT_POLL_CELL_V_B_NR:
            target = &ext_cell_v_b_nr;
            break;
          case EXT_POLL_SOH_AVG:
            target = &ext_soh_avg;
            break;
          case EXT_POLL_MAX_CHARGE_POWER:
            target = &ext_max_charge_power;
            break;
          case EXT_POLL_MAX_GEN_POWER:
            target = &ext_max_gen_power;
            break;
          case EXT_POLL_MAX_AVAIL_POWER:
            target = &ext_max_avail_power;
            break;
          case EXT_POLL_SOC_AVG:
            target = &ext_soc_avg;
            break;
          case EXT_POLL_USOC_AVG:
            target = &ext_usoc_avg;
            break;
          case EXT_POLL_SOC_MIN:
            target = &ext_soc_min;
            break;
          case EXT_POLL_SOC_MAX:
            target = &ext_soc_max;
            break;
          default:  // EXT_POLL_BATTERY_CURRENT
            target = &ext_battery_current;
            break;
        }
        target->raw = v;
        target->valid = true;
      }
      break;
    case EXT_POLL_CYCLES:  // 0x9210, 16-bit count, no scaling
      if (length >= 2) {
        battery_charge_cycles = (data[0] << 8) | data[1];
      }
      break;
    case EXT_POLL_ENERGY_CHARGED:  // 0x9243, 32-bit, x0.001 kWh
      if (length >= 4) {
        battery_energy_charged_kWh =
            (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3]) * 0.001f;
      }
      break;
    case EXT_POLL_ENERGY_DISCHARGED:  // 0x9245, 32-bit, x0.001 kWh
      if (length >= 4) {
        battery_energy_discharged_kWh =
            (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3]) * 0.001f;
      }
      break;
    case EXT_POLL_ENERGY_REGENERATED:  // 0x9247, 32-bit, x0.001 kWh
      if (length >= 4) {
        battery_energy_regenerated_kWh =
            (((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3]) * 0.001f;
      }
      break;
    case EXT_POLL_TEMPORISATION:  // 0x9281, top bit of byte 0 (mirrors Zoe Gen2's own reading of this PID)
      if (length >= 1) {
        // Raw byte is kept: the flag is bit 0 per the CanZE field list (bit 31 of the frame, 0 = temporisation
        // active, 1 = deactivated), not the top bit that the Zoe Ph2 driver evaluates.
        battery_temporisation = data[0];
        priority_answered(1);
      }
      break;
    case EXT_POLL_BAL_CAP_TOTAL:  // 0x924F-0x9252 and 0x9262/0x9263: balancing counters, raw 32-bit
    case EXT_POLL_BAL_TIME_TOTAL:  // (see bal_counter_value). 0x9262/0x9263 aren't contiguous with the
    case EXT_POLL_BAL_CAP_SLEEP:   // rest, so their array index is set explicitly instead of by offset.
    case EXT_POLL_BAL_TIME_SLEEP:
    case EXT_POLL_BAL_CAP_WAKE:
    case EXT_POLL_BAL_TIME_WAKE:
      if (length >= 4) {
        uint8_t idx;
        if (pid == EXT_POLL_BAL_CAP_WAKE) {
          idx = 4;
        } else if (pid == EXT_POLL_BAL_TIME_WAKE) {
          idx = 5;
        } else {
          idx = (uint8_t)(pid - EXT_POLL_BAL_CAP_TOTAL);
        }
        bal_raw[idx] =
            ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | (uint32_t)data[3];
        bal_valid[idx] = true;
        if (pid == EXT_POLL_BAL_CAP_SLEEP) {
          priority_answered(2);
        } else if (pid == EXT_POLL_BAL_TIME_SLEEP) {
          priority_answered(3);
        }
      }
      break;
    default:  // Unknown/unrequested PID, ignore
      break;
  }
}

// Called once a full multi-frame ISO-TP reassembly has completed.
//
// Byte layout confirmed two ways: (1) it matches Battery-Emulator's own
// RENAULT-ZOE-GEN2-BATTERY.cpp POLL_BALANCE_SWITCHES handling exactly -
// cells 0-31 from the LAST 4 bytes of the 3rd continuation frame, cells
// 32-87 from ALL 7 bytes of the 4th continuation frame, cells 88-95 from
// the FIRST byte of the 5th continuation frame, MSB-first within each byte.
// (2) applying it to a real captured 0x912B response from this exact
// battery produced a stable, structured, repeatable (non-zero, non-random)
// 96-bit pattern across 5 independent polls 2 minutes apart - our original
// guess (first 12 bytes from the start of the payload, LSB-first) always
// read all-zero on this battery because that part of the payload genuinely
// is unused padding; the real per-cell bits sit further in.
//
// Index order: Zoe Gen2's own driver documents (and corrects for) the LBC
// delivering these 96 bits in REVERSE cell order (scan position 0 = cell
// 96, scan position 95 = cell 1), while cell_voltages_mV[]/the rest of the
// datalayer is ordered cell 1->96. We apply the same (95 - scan_index)
// flip here. No bit-VALUE inversion - raw bit 1 means "balancing", matching
// Zoe Gen2's own unmodified convention. (An earlier version of this
// function instead inverted the bit value without reversing the index;
// that was the wrong fix for what looked like backwards data - reverting
// it here now that the actual cause is confirmed against Zoe Gen2's code.)
void RenaultTwingoGen1Battery::handle_extended_multiframe_complete() {
#ifdef EXTENDED_UDS_DEBUG
  logging.printf("EXT UDS RX reassembled (%u bytes): ", ext_isotp_received_len);
  for (uint16_t i = 0; i < ext_isotp_received_len; i++) {
    logging.printf("%02X ", ext_isotp_buffer[i]);
  }
  logging.println();
#endif
  if (ext_isotp_received_len < 3) {
    return;  // Not even enough for an echoed SID+PID
  }
  uint16_t pid = (ext_isotp_buffer[1] << 8) | ext_isotp_buffer[2];
  if ((pid >> 8) == 0x02 && dtc_ext_state == DTC_EXT_READ_CMD_SENT) {
    // Positive ReadDTCInformation reply that needed multiple frames (more than ~1 DTC worth of data) -
    // buffer[0]=SID(0x59), [1]=subfunction(0x02), [2]=DTCStatusAvailabilityMask, [3..]=DTC entries
    // (3-byte code + 1-byte status each). This is the case actually confirmed on this battery (01.10.
    // capture): First Frame + our Flow Control + 3 Consecutive Frames reassembled cleanly to the full
    // 27 bytes the First Frame announced - but buffer[2] (the ECU's reported supported-status mask) came
    // back 0xFF, not an echo of the 0x09 we requested, so only the subfunction byte is checked here now,
    // not the full previous pid==0x0209 (that exact-match version silently dropped this exact reply).
    handle_dtc_read_response(&ext_isotp_buffer[3], (uint16_t)(ext_isotp_received_len - 3));
    return;
  }
  if (pid == EXT_POLL_BMS_STATE) {
    // 0x9270: 32 bytes after SID+PID (35 bytes in total)
    if (ext_isotp_received_len >= 3 + 32) {
      memcpy(bms_state_raw, &ext_isotp_buffer[3], sizeof(bms_state_raw));
      bms_state_valid = true;
      priority_answered(0);
    }
    return;
  }
  if (pid != EXT_POLL_BALANCE_SWITCHES) {
    return;  // Nothing else expected to arrive as multi-frame
  }

  uint16_t payload_len = ext_isotp_received_len - 3;  // Bytes available after SID+PID
  if (payload_len < 32) {
    // The bits we actually need go up to payload byte 32 (buffer[34]) - see
    // the byte-layout comment above. Don't trust a partial reassembly,
    // flag it instead of silently showing stale/wrong data.
    datalayer_battery->status.balancing_status = BALANCING_STATUS_ERROR;
    return;
  }

  bool any_balancing = false;
  // Scan positions 0-31: last 4 bytes of the 3rd continuation frame -> buffer[23..26].
  // Scan position i holds the bit for physical cell (96 - i), i.e. array index (95 - i).
  for (uint8_t i = 0; i < 32; i++) {
    bool balancing = (ext_isotp_buffer[23 + (i >> 3)] >> (7 - (i & 7))) & 0x01;
    datalayer_battery->status.cell_balancing_status[95 - i] = balancing;
    if (balancing) {
      any_balancing = true;
    }
  }
  // Scan positions 32-87: all 7 bytes of the 4th continuation frame -> buffer[27..33]
  for (uint8_t i = 32; i < 88; i++) {
    uint8_t j = i - 32;
    bool balancing = (ext_isotp_buffer[27 + (j >> 3)] >> (7 - (j & 7))) & 0x01;
    datalayer_battery->status.cell_balancing_status[95 - i] = balancing;
    if (balancing) {
      any_balancing = true;
    }
  }
  // Scan positions 88-95: first byte of the 5th continuation frame -> buffer[34]
  for (uint8_t i = 88; i < 96; i++) {
    bool balancing = (ext_isotp_buffer[34] >> (7 - (i - 88))) & 0x01;
    datalayer_battery->status.cell_balancing_status[95 - i] = balancing;
    if (balancing) {
      any_balancing = true;
    }
  }
  datalayer_battery->status.balancing_status = any_balancing ? BALANCING_STATUS_ACTIVE : BALANCING_STATUS_READY;
}

// Dispatches an incoming 0x18DAF1DB frame: single-frame, ISO-TP First Frame
// (sends the Flow Control reply and starts reassembly), or ISO-TP
// Consecutive Frame (continues reassembly, dispatches once complete).
void RenaultTwingoGen1Battery::handle_extended_reply(CAN_frame rx_frame) {
  datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;

  uint8_t pci = rx_frame.data.u8[0];

  if (pci < 0x10) {
    // Single frame: pci is the payload length (echoed SID + PID + data).
    if (pci < 3) {
      return;  // Too short to even contain SID+PID
    }
#ifdef EXTENDED_UDS_DEBUG
    if (rx_frame.data.u8[1] == 0x7F) {
      // Negative response: 03 7F <requested SID> <NRC>
      logging.printf("EXT UDS: negative response, requested SID=0x%02X NRC=0x%02X\n", rx_frame.data.u8[2],
                      rx_frame.data.u8[3]);
    } else {
      logging.printf("EXT UDS RX single-frame: PID=0x%02X%02X data=", rx_frame.data.u8[2], rx_frame.data.u8[3]);
      for (uint8_t i = 4; i < (uint8_t)(pci + 1); i++) {
        logging.printf("%02X ", rx_frame.data.u8[i]);
      }
      logging.println();
    }
#endif
    uint16_t pid = (rx_frame.data.u8[2] << 8) | rx_frame.data.u8[3];
    handle_extended_single_frame(pid, &rx_frame.data.u8[4], (uint16_t)(pci - 3));
    ext_isotp_in_progress = false;  // A single-frame reply closes out any pending wait
    return;
  }

  if ((pci & 0xF0) == 0x10) {
    // First frame of a multi-frame response.
    ext_isotp_expected_len = (uint16_t)((pci & 0x0F) << 8) | rx_frame.data.u8[1];
#ifdef EXTENDED_UDS_DEBUG
    logging.printf("EXT UDS RX first-frame: expected_len=%u (buffer=%u)\n", ext_isotp_expected_len,
                    (unsigned)sizeof(ext_isotp_buffer));
#endif
    if (ext_isotp_expected_len == 0 || ext_isotp_expected_len > sizeof(ext_isotp_buffer)) {
#ifdef EXTENDED_UDS_DEBUG
      logging.printf("EXT UDS: reassembly abandoned, %u bytes wouldn't fit %u-byte buffer\n", ext_isotp_expected_len,
                      (unsigned)sizeof(ext_isotp_buffer));
#endif
      ext_isotp_in_progress = false;  // Wouldn't fit our buffer; give up cleanly rather than overflow it
      return;
    }
    for (uint8_t i = 0; i < 6; i++) {
      ext_isotp_buffer[i] = rx_frame.data.u8[2 + i];
    }
    ext_isotp_received_len = 6;
    ext_isotp_in_progress = true;
    ext_isotp_started_ms = millis();
    transmit_can_frame(&ZOE_POLL_FLOW_CONTROL);
    return;
  }

  if ((pci & 0xF0) == 0x20) {
    // Consecutive frame.
    if (!ext_isotp_in_progress) {
      return;  // Stray CF with no matching First Frame; ignore it
    }
    uint16_t remaining = ext_isotp_expected_len - ext_isotp_received_len;
    uint8_t copy_len = (remaining < 7) ? (uint8_t)remaining : 7;
    for (uint8_t i = 0; i < copy_len; i++) {
      ext_isotp_buffer[ext_isotp_received_len + i] = rx_frame.data.u8[1 + i];
    }
    ext_isotp_received_len += copy_len;
    if (ext_isotp_received_len >= ext_isotp_expected_len) {
      handle_extended_multiframe_complete();
      ext_isotp_in_progress = false;
    }
    return;
  }
}

// Minimal, dedicated response parser used only while UserRequestNVROLReset is
// active. Records what actually came back for nvrol_awaiting_step as a short
// readable string, later shown in get_uds_info_html(). Distinguishes a
// positive response (raw bytes) from a negative one (SID 0x7F + NRC) -
// that distinction is the whole point: it tells us whether e.g. the routine
// or the write is actively rejected, versus never answered at all.
void RenaultTwingoGen1Battery::handle_nvrol_reply(CAN_frame rx_frame) {
  if (NVROLstateMachine == 5) {
    // Quiet phase: only record what the BMS sends on 0x18DAF1DB (e.g. a late answer to routine B009),
    // never answer it - no flow control, nothing is transmitted.
    if (nvrol_silence_reply_total < 0xFFFF) {
      nvrol_silence_reply_total++;
    }
    if (nvrol_silence_reply_count < NVROL_SILENCE_REPLY_MAX) {
      NvrolSilenceReply& r = nvrol_silence_reply[nvrol_silence_reply_count++];
      r.t_ms = (uint16_t)(millis() - nvrol_silence_start_ms);
      for (uint8_t i = 0; i < 8; i++) {
        r.d[i] = rx_frame.data.u8[i];
      }
    }
    return;
  }
  uint8_t step = nvrol_awaiting_step;
  if (step >= NVROL_LOG_STEPS) {
    return;
  }
  uint8_t pci = rx_frame.data.u8[0];
  if (pci >= 0x10) {
    snprintf(nvrol_log[step], sizeof(nvrol_log[step]), "unexpected multi-frame (PCI=0x%02X)", pci);
  } else if (pci < 3) {
    snprintf(nvrol_log[step], sizeof(nvrol_log[step]), "short reply (%u bytes)", pci);
  } else if (rx_frame.data.u8[1] == 0x7F) {
    // Negative response: 03 7F <requested SID> <NRC>
    snprintf(nvrol_log[step], sizeof(nvrol_log[step]), "NEGATIVE SID=0x%02X NRC=0x%02X", rx_frame.data.u8[2],
             rx_frame.data.u8[3]);
  } else {
    snprintf(nvrol_log[step], sizeof(nvrol_log[step]), "OK raw=%02X %02X %02X %02X %02X %02X %02X",
             rx_frame.data.u8[1], rx_frame.data.u8[2], rx_frame.data.u8[3], rx_frame.data.u8[4], rx_frame.data.u8[5],
             rx_frame.data.u8[6], rx_frame.data.u8[7]);
    if (step == 4 && pci >= 4 && rx_frame.data.u8[1] == 0x62 && rx_frame.data.u8[2] == 0x92 &&
        rx_frame.data.u8[3] == 0x81) {
      temporisation_readback = rx_frame.data.u8[4];  // 0x9281 as the BMS holds it right after the write
    }
  }
}

// Sends the two documented write sequences (NVROL reset, then enable
// "temporisation before sleep") over the same extended channel used for
// polling - see the comment block above UserRequestNVROLReset in the header
// for what this is, what it's based on, and what's deliberately left out.
// Timing (100ms/1s/100ms gaps between steps) matches Battery-Emulator's own
// proven Zoe Ph2 sequence exactly; only the final 30s software-sleep step is
// omitted on purpose.
// Enter the shutdown sequence (state 7): 0x350 will start walking C3 -> C2 -> C0 -> 00.
// Configured sleep failsafe window (minutes, "More Battery Info" web UI, persisted to NVM), clamped to
// 1-1440 min (24h). Falls back to SLEEP_MANUAL_FAILSAFE_DEFAULT_MIN if the stored value is 0 or out of
// that range (e.g. before the setting has ever been written).
unsigned long RenaultTwingoGen1Battery::sleep_manual_failsafe_ms(void) {
  uint16_t configured_min = datalayer_extended.twingoGen1.sleep_failsafe_minutes;
  if (configured_min < 1 || configured_min > 1440) {
    configured_min = SLEEP_MANUAL_FAILSAFE_DEFAULT_MIN;
  }
  return (unsigned long)configured_min * 60UL * 1000UL;
}

void RenaultTwingoGen1Battery::start_powerdown(void) {
  powerdown_stage = 0;
  powerdown_start_ms = millis();
  powerdown_stage_start_ms = powerdown_start_ms;
  previousMillis_350 = 0;
  NVROLstateMachine = 7;
}

// Enter the wake-up burst (state 8): 0x350 = C0 once, then C3 x10. Also starts the "how fast does the
// BMS come back" timers, so they cover the whole wake-up, not just what happens after it.
void RenaultTwingoGen1Battery::start_wake_burst(void) {
  // True silence ends here (transmission resumes with the first burst frame), so this is when the
  // displayed "silent for" duration should stop counting - not once the burst itself has also finished.
  nvrol_silence_end_ms = millis();
  wake_burst_index = 0;
  wake_burst_last_ms = 0;
  wake_tracking = true;
  wake_start_ms = millis();
  wake_first_rx = -1;
  wake_first_uds = -1;
  wake_priority_done = -1;
  wake_priority_timeout = false;
  NVROLstateMachine = 8;
}

void RenaultTwingoGen1Battery::transmit_reset_nvrol_frames(void) {
  switch (NVROLstateMachine) {
    case 0:
      startTimeNVROL = millis();
      // Values right before the reset, for the before/after comparison on the web page
      nvrol_before.state_valid = bms_state_valid;
      memcpy(nvrol_before.state, bms_state_raw, sizeof(nvrol_before.state));
      for (uint8_t i = 0; i < 4; i++) {
        nvrol_before.bal_raw[i] = bal_raw[i];
        nvrol_before.bal_valid[i] = bal_valid[i];
      }
      nvrol_before.temporisation = battery_temporisation;
      memset(nvrol_silence_rx_per_s, 0, sizeof(nvrol_silence_rx_per_s));
      memset(nvrol_silence_rx_per_10s, 0, sizeof(nvrol_silence_rx_per_10s));
      memset(nvrol_silence_ids, 0, sizeof(nvrol_silence_ids));
      nvrol_silence_id_count = 0;
      nvrol_silence_id_other = 0;
      nvrol_silence_end_ms = 0;
      wake_tracking = false;
      temporisation_readback = 0x100;
      nvrol_silence_rx_total = 0;
      nvrol_silence_last_rx_ms = 0;
      nvrol_silence_reply_count = 0;
      nvrol_silence_reply_total = 0;
      nvrol_silence_done = false;
      for (uint8_t i = 0; i < NVROL_LOG_STEPS; i++) {
        strncpy(nvrol_log[i], "no response", sizeof(nvrol_log[i]) - 1);
        nvrol_log[i][sizeof(nvrol_log[i]) - 1] = '\0';
      }
      nvrol_awaiting_step = 0;
      if (nvrol_mode == 1) {
        // "Sleep": nothing else is sent, go straight into the shutdown sequence (0x350 walking
        // C3 -> C2 -> C0 -> 00), true silence follows once it reaches 00.
        for (uint8_t i = 0; i < NVROL_LOG_STEPS; i++) {
          strncpy(nvrol_log[i], "not sent (Sleep)", sizeof(nvrol_log[i]) - 1);
        }
        ext_isotp_in_progress = false;
        start_powerdown();
        break;
      }
      // NVROL reset, part 1: open diagnostic session (SID 0x10) before RoutineControl B009. Default
      // subfunction 0x03 (Extended) - the session type that was tried so far (B009 answered NEGATIVE,
      // NRC 0x7F serviceNotSupportedInActiveSession, unconfirmed whether Programming (0x02) fixes that).
      // Configurable via "More Battery Info" (datalayer_extended.twingoGen1.nvrol_b009_use_programming_session).
      {
        uint8_t session_sub = datalayer_extended.twingoGen1.nvrol_b009_use_programming_session ? 0x02 : 0x03;
        ZOE_POLL_18DADBF1.data = {0x02, 0x10, session_sub, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
      }
#ifdef EXTENDED_UDS_DEBUG
      logging.println("NVROL: step 0 - open session");
#endif
      NVROLstateMachine = 1;
      break;
    case 1:  // wait 100ms for step 0's response
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS && nvrol_mode == 2) {
        // "Sleep 0x9281=1": no reset routine, straight to the temporisation write (session is already open).
        // The write value is 0x00, not 0x01: a real "RBMS_MCPU_RL" ECU dump (28.09.) labels the live value
        // 0x00 of this PID as "temporisation is activated" - the button name is kept as-is (it predates
        // this finding), only the byte sent has changed.
        strncpy(nvrol_log[1], "skipped (Sleep 0x9281=1)", sizeof(nvrol_log[1]) - 1);
        strncpy(nvrol_log[2], "skipped (Sleep 0x9281=1)", sizeof(nvrol_log[2]) - 1);
        ZOE_POLL_18DADBF1.data = {0x04, 0x2E, 0x92, 0x81, datalayer_extended.twingoGen1.nvrol_temporisation_write_value,
                                   0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 3;
        startTimeNVROL = millis();
        NVROLstateMachine = 4;
        break;
      }
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS) {
        // NVROL reset, part 2: RoutineControl (SID 0x31) start routine (0x01) 0xB009
        ZOE_POLL_18DADBF1.data = {0x04, 0x31, 0x01, 0xB0, 0x09, 0x00, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 1;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 1 - start routine B009 (NVROL reset)");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 9;
      }
      break;
    case 9:  // wait 500ms, then ask the routine itself for its result (RequestRoutineResults, subfunction 0x03)
      if ((millis() - startTimeNVROL) > 500) {
        ZOE_POLL_18DADBF1.data = {0x04, 0x31, 0x03, 0xB0, 0x09, 0x00, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 5;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 1b - request routine B009 results");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 2;
      }
      break;
    case 2:  // wait 1s for step 1's response
      if ((millis() - startTimeNVROL) > INTERVAL_1_S) {
        // Enable temporisation before sleep, part 1: open extended diagnostic session again
        ZOE_POLL_18DADBF1.data = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 2;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 2 - open session again");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 3;
      }
      break;
    case 3:  // wait 100ms for step 2's response
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS) {
        // Enable temporisation before sleep, part 2: WriteDataByIdentifier (SID 0x2E) PID 0x9281.
        // Default 0x00 ("temporisation is activated" per a real ECU dump, see the comment above);
        // configurable to 0x01 via "More Battery Info" (datalayer_extended.twingoGen1.nvrol_temporisation_write_value).
        ZOE_POLL_18DADBF1.data = {0x04, 0x2E, 0x92, 0x81, datalayer_extended.twingoGen1.nvrol_temporisation_write_value,
                                   0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 3;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 3 - write temporisation (configured value)");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 4;
      }
      break;
    case 4:  // wait 100ms for step 3's response, then read 0x9281 back
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS) {
        // Read the temporisation flag back right away: does the BMS keep what was just written?
        ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x92, 0x81, 0x00, 0x00, 0x00, 0x00};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 4;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 4 - read 0x9281 back");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 6;
      }
      break;
    case 6:  // wait 150ms for the read-back, then start the shutdown sequence
      if ((millis() - startTimeNVROL) > 150) {
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: sequence complete, shutdown sequence starts");
#endif
        // Restore the poll frame to its normal read template - we're done with the special sequence.
        ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        ext_isotp_in_progress = false;
        start_powerdown();
      }
      break;
    case 7: {  // Shutdown sequence: 0x350 walks C3 -> C2 -> C0 -> 00, own frames keep running (see
               // transmit_can()) until the 00 stage begins, then true silence (state 5) follows.
      unsigned long now = millis();
      static const uint8_t stage_byte0[4] = {0xC3, 0xC2, 0xC0, 0x00};
      static const unsigned long stage_duration_ms[4] = {POWERDOWN_C3_MS, POWERDOWN_C2_MS, POWERDOWN_C0_MS,
                                                          POWERDOWN_00_MS};
      if (now - previousMillis_350 >= INTERVAL_350_MS) {
        previousMillis_350 = now;
        send_vehicle_state_350(stage_byte0[powerdown_stage]);
      }
      if (nvrol_wake_request) {
        start_wake_burst();
        break;
      }
      if (now - powerdown_stage_start_ms >= stage_duration_ms[powerdown_stage]) {
        if (powerdown_stage + 1 >= 4) {
          // The 00 stage's own duration has elapsed too: stop everything, true silence begins.
          nvrol_silence_start_ms = now;
          nvrol_silence_last_rx_ms = 0;
          NVROLstateMachine = 5;
        } else {
          powerdown_stage++;
          powerdown_stage_start_ms = now;
        }
      }
      break;
    }
    case 5:  // true silence: nothing is transmitted at all (see transmit_can()), only the timer runs
      if (nvrol_wake_request || (millis() - nvrol_silence_start_ms) >= sleep_manual_failsafe_ms()) {
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: true silence over, waking up");
#endif
        start_wake_burst();
      }
      break;
    case 8: {  // Wake-up burst: 0x350 = C0 once, then C3 x10, 200ms apart, before normal polling resumes
      unsigned long now = millis();
      if (now - wake_burst_last_ms >= WAKE_BURST_INTERVAL_MS) {
        wake_burst_last_ms = now;
        send_wake_burst_frame(wake_burst_index);
        wake_burst_index++;
      }
      if (wake_burst_index > WAKE_BURST_COUNT) {
        finish_nvrol_silence();
      }
      break;
    }
    default:  // Something went wrong; reset state machine
      NVROLstateMachine = 0;
      nvrol_mode = 0;
      UserRequestNVROLReset = false;
      break;
  }
}

// End of the quiet phase: from the next transmit_can() call on all frames are sent again, which wakes
// the BMS. It then behaves like after a boot, so the values are marked as unread, the cell plausibility
// filter is armed again and a few PIDs are asked with priority until they have answered.
// Called once the wake-up burst (state 8) has finished: the BMS/vehicle has had its wake signal, normal
// polling resumes from here. wake_tracking/wake_start_ms/wake_first_* were already set by
// start_wake_burst() when the burst began, so the "how fast did it come back" timers cover the burst too.
void RenaultTwingoGen1Battery::finish_nvrol_silence(void) {
  // Upstream's CAN_error_counter (safety.cpp, MAX_CAN_FAILURES=50) is never reset anywhere in
  // production code - once it crosses the threshold, EVENT_CAN_CORRUPTED_WARNING stays "active"
  // forever and its Count display climbs by 1 on every 1Hz update_machineryprotection() pass, even
  // with zero new corrupted frames (confirmed 01.10.: record_silence_frame showed 0 RX for 10+ min
  // while Count kept climbing). Reset it here, right after a successful wake-up resumes normal
  // communication - a fresh polling cycle is the natural "clean slate" point, same spirit as the
  // other per-wake resets below (bms_state_valid, bal_valid, temp filters).
  datalayer_battery->status.CAN_error_counter = 0;
  nvrol_silence_done = true;
  nvrol_last_mode = nvrol_mode;
  nvrol_mode = 0;
  nvrol_wake_request = false;
  bms_state_valid = false;
  for (uint8_t i = 0; i < 4; i++) {
    bal_valid[i] = false;
  }
  battery_temporisation = 0x100;
  ext_filter_rearm_active = true;
  ext_filter_rearm_start_ms = millis();
  temp_boot_filter_active = true;  // the temperature filter of 0x424 is armed again like the cell filter
  temp_boot_first_frame_ms = 0;
  ext425_boot_filter_active = true;  // same re-arm for the 0x425 pack-voltage boot filter
  ext425_boot_first_frame_ms = 0;
  time_pid_len[0] = 0;  // show the time PIDs as "not yet read" until they have been polled again
  time_pid_len[1] = 0;
  ext_isotp_in_progress = false;
  ext_poll_index = 0;
  ext_priority_pending_mask = 0x0F;
  ext_priority_next = 0;
  ext_priority_start_ms = millis();
  previousMillisExtPoll = millis();  // first poll after the restart follows 200ms later
  UserRequestNVROLReset = false;
  NVROLstateMachine = 0;
}

// One 0x350 frame for the shutdown sequence (case 7): bytes 5/6/7 depend only on whether the state is the
// "active" family (C3/C2, stable value from the log) or the "sleeping" family (C0/00, likewise stable).
void RenaultTwingoGen1Battery::send_vehicle_state_350(uint8_t byte0) {
  uint8_t b5, b7;
  if (byte0 == 0xC0 || byte0 == 0x00) {
    b5 = 0x70;
    b7 = 0x85;
  } else {  // C3 or C2
    b5 = 0x14;
    b7 = 0x45;
  }
  CAN_frame f = {.FD = false,
                 .ext_ID = false,
                 .DLC = 8,
                 .ID = 0x350,
                 .data = {byte0, VEHICLE_AGE_350_B1, VEHICLE_AGE_350_B2, VEHICLE_AGE_350_B3, 0x14, b5, 0x96, b7}};
  transmit_can_frame(&f);
}

// One 0x350 frame for the wake-up burst (case 8): the real vehicle briefly keeps the old C0-style bytes
// for the first C3 frame, then settles - captured in Log_Twingo_Ladung.log around 10:48:55.
void RenaultTwingoGen1Battery::send_wake_burst_frame(uint8_t index) {
  uint8_t byte0, b5, b7;
  if (index == 0) {
    byte0 = 0xC0;
    b5 = 0x70;
    b7 = 0x85;
  } else if (index == 1) {
    byte0 = 0xC3;
    b5 = 0x70;
    b7 = 0x85;
  } else if (index == 2) {
    byte0 = 0xC3;
    b5 = 0x10;
    b7 = 0x45;
  } else {
    byte0 = 0xC3;
    b5 = 0x14;
    b7 = 0x45;
  }
  CAN_frame f = {.FD = false,
                 .ext_ID = false,
                 .DLC = 8,
                 .ID = 0x350,
                 .data = {byte0, VEHICLE_AGE_350_B1, VEHICLE_AGE_350_B2, VEHICLE_AGE_350_B3, 0x14, b5, 0x96, b7}};
  transmit_can_frame(&f);
}

static const char* bms_state_name(uint8_t v) {
  switch (v) {
    case 1:
      return "Init";
    case 2:
      return "Wait";
    case 3:
      return "Isolated Charge";
    case 4:
      return "Non Isolated Charge";
    case 5:
      return "External Charge";
    case 6:
      return "Driving";
    case 7:
      return "SleepTransient";
    case 8:
      return "DC Charging without RISOL";
    case 9:
      return "DC Charging with RISOL";
    default:
      return "?";
  }
}

static void append_hex_bytes(String& s, const uint8_t* d, uint8_t n) {
  char b[4];
  for (uint8_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02X", d[i]);
    s += b;
    if (i + 1 < n) {
      s += ' ';
    }
  }
}

// Balancing counters (0x924F-0x9252) are signed 32-bit values with an offset of -2^31 and a factor of
// 1/1024 (CanZE field list): value = (raw XOR 0x80000000) / 1024. The raw value is always shown too.
static float bal_counter_value(uint32_t raw) {
  return (float)((double)(raw ^ 0x80000000u) / 1024.0);
}

// Live BMS state (all 32 bytes, first and last decoded) and the balancing counters.
void RenaultTwingoGen1Battery::append_live_html(String& s) {
  s += "BMS State (0x9270): ";
  if (!bms_state_valid) {
    s += "not yet read";
  } else {
    append_hex_bytes(s, bms_state_raw, 32);
    s += " (first: ";
    s += bms_state_name(bms_state_raw[0]);
    s += ", last: ";
    s += bms_state_name(bms_state_raw[31]);
    s += ")";
  }
  s += "<br>";
  for (uint8_t line = 0; line < 3; line++) {
    uint8_t cap = line * 2;
    uint8_t tim = line * 2 + 1;
    if (line == 0) {
      s += "Balancing total (0x924F/0x9250): ";
    } else if (line == 1) {
      s += "Balancing in sleep (0x9251/0x9252): ";
    } else {
      s += "Balancing while awake (0x9262/0x9263): ";
    }
    if (!bal_valid[cap] || !bal_valid[tim]) {
      s += "not yet read<br>";
      continue;
    }
    s += String(bal_counter_value(bal_raw[cap]), 3);
    s += " Ah / ";
    s += String(bal_counter_value(bal_raw[tim]), 3);
    s += " h (raw ";
    char b[24];
    snprintf(b, sizeof(b), "%08lX / %08lX", (unsigned long)bal_raw[cap], (unsigned long)bal_raw[tim]);
    s += b;
    s += ")<br>";
  }
  for (uint8_t idx = 0; idx < 2; idx++) {
    s += (idx == 0) ? "Time (0x9261): " : "Pack time (0x91C1): ";
    if (time_pid_len[idx] == 0) {
      s += "not yet read<br>";
      continue;
    }
    char b[24];
    snprintf(b, sizeof(b), "%0*lX", (int)(time_pid_len[idx] * 2), (unsigned long)time_pid_raw[idx]);
    s += b;
    s += " (";
    s += String((unsigned long)time_pid_raw[idx]);
    s += ")<br>";
  }
  s += "SOH candidate (0x658 byte 4): ";
  if (soh_658 == 0xFF) {
    s += "not received";
  } else if (soh_658 == 0x7F) {
    s += "invalid (127)";
  } else {
    s += String(soh_658);
    s += " %";
  }
  s += "<br>";
  append_ext_value(s, "Pack Mileage (0x91CF)", ext_mileage_pack, (ext_mileage_pack.raw ^ 0x80000000u) / 32.0, "km");
  append_ext_value(s, "Vehicle Distance Totalizer (0x925F)", ext_mileage_vehicle, ext_mileage_vehicle.raw * 0.01,
                    "km");
  append_ext_value(s, "Low Voltage Supply (0x9011)", ext_lv_supply, ext_lv_supply.raw / 1024.0, "V");
  append_ext_value(s, "Pack Voltage, cell sum (0x9006)", ext_pack_voltage, ext_pack_voltage.raw * 0.976563 / 1000.0,
                    "V");
  append_ext_value(s, "Cell Voltage A (0x9007, min/max not confirmed)", ext_cell_v_a, ext_cell_v_a.raw * 0.976563 / 1000.0,
                    "V");
  append_ext_value(s, "Cell Voltage B (0x9009, min/max not confirmed)", ext_cell_v_b, ext_cell_v_b.raw * 0.976563 / 1000.0,
                    "V");
  append_ext_value(s, "Cell Voltage A index (0x9008)", ext_cell_v_a_nr, (double)ext_cell_v_a_nr.raw, "");
  append_ext_value(s, "Cell Voltage B index (0x900A)", ext_cell_v_b_nr, (double)ext_cell_v_b_nr.raw, "");
  append_ext_value(s, "Battery SOH avg (0x9003)", ext_soh_avg, ext_soh_avg.raw / 100.0, "%");
  append_ext_value(s, "Max Charge Power (0x9018)", ext_max_charge_power, ext_max_charge_power.raw / 100.0, "kW");
  append_ext_value(s, "Max Generated Power (0x900E)", ext_max_gen_power, ext_max_gen_power.raw / 100.0, "kW");
  append_ext_value(s, "Max Available Power (0x900F)", ext_max_avail_power, ext_max_avail_power.raw / 100.0, "kW");
  append_ext_value(s, "Battery SOC, internal (0x9001)", ext_soc_avg, ext_soc_avg.raw * 0.01 - 3.0, "%");
  append_ext_value(s, "Battery USOC, dashboard (0x9002, display only)", ext_usoc_avg, ext_usoc_avg.raw * 0.01, "%");
  append_ext_value(s, "Battery SOC min (0x91B9)", ext_soc_min, ext_soc_min.raw * 0.01 - 3.0, "%");
  append_ext_value(s, "Battery SOC max (0x91BA)", ext_soc_max, ext_soc_max.raw * 0.01 - 3.0, "%");
  append_ext_value(s, "Battery Current (0x900D, display only, see current_dA ToDo)", ext_battery_current,
                    ((double)(int32_t)ext_battery_current.raw * 0.025 - 1200.0) * -1.0, "A");
}

void RenaultTwingoGen1Battery::append_ext_value(String& s, const char* label, const ExtValue& v, double value,
                                                 const char* unit) {
  s += label;
  s += ": ";
  if (!v.valid) {
    s += "not yet read<br>";
    return;
  }
  s += String(value, 3);
  if (unit[0] != '\0') {
    s += " ";
    s += unit;
  }
  s += "<br>";
}

void RenaultTwingoGen1Battery::priority_answered(uint8_t bit) {
  ext_priority_pending_mask &= (uint8_t)~(1u << bit);
  if (ext_priority_pending_mask == 0 && wake_tracking && wake_priority_done < 0) {
    wake_priority_done = (int32_t)(millis() - wake_start_ms);
  }
}

void RenaultTwingoGen1Battery::record_silence_frame(const CAN_frame& f) {
  unsigned long now = millis();
  unsigned long dt = now - powerdown_start_ms;  // one continuous timeline from the shutdown sequence's start
  unsigned long sec = dt / 1000;
  if (sec < SILENCE_SEC_BUCKETS && nvrol_silence_rx_per_s[sec] < 0xFFFF) {
    nvrol_silence_rx_per_s[sec]++;
  }
  unsigned long b10 = dt / 10000;
  if (b10 < SILENCE_10S_BUCKETS && nvrol_silence_rx_per_10s[b10] < 0xFFFF) {
    nvrol_silence_rx_per_10s[b10]++;
  }
  nvrol_silence_rx_total++;
  nvrol_silence_last_rx_ms = now;

  const uint8_t n = (f.DLC > 8) ? 8 : f.DLC;  // only the bytes the frame really carries
  SilenceId* rec = nullptr;
  for (uint8_t i = 0; i < nvrol_silence_id_count; i++) {
    if (nvrol_silence_ids[i].id == f.ID) {
      rec = &nvrol_silence_ids[i];
      break;
    }
  }
  if (!rec) {
    if (nvrol_silence_id_count >= SILENCE_ID_MAX) {
      nvrol_silence_id_other++;
      return;
    }
    rec = &nvrol_silence_ids[nvrol_silence_id_count++];
    rec->id = f.ID;
    rec->count = 0;
    rec->changes = 0;
    rec->chg_count = 0;
    rec->chg_next = 0;
    memset(rec->first, 0, 8);
    memset(rec->last, 0, 8);
    memcpy(rec->first, f.data.u8, n);
    memcpy(rec->last, f.data.u8, n);
  }
  // Data change: remember the last three (time, new data, which bytes differed)
  uint8_t mask = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (rec->last[i] != f.data.u8[i]) {
      mask |= (uint8_t)(1u << i);
    }
  }
  if (mask != 0) {
    if (rec->changes < 0xFFFF) {
      rec->changes++;
    }
    SilenceChange& ch = rec->chg[rec->chg_next];
    rec->chg_next = (uint8_t)((rec->chg_next + 1) % 3);
    if (rec->chg_count < 3) {
      rec->chg_count++;
    }
    ch.t_ms = (uint32_t)dt;
    ch.mask = mask;
    memset(ch.d, 0, 8);
    memcpy(ch.d, f.data.u8, n);
  }
  memcpy(rec->last, f.data.u8, n);
  rec->dlc = f.DLC;
  rec->last_ms = now;
  rec->count++;
}

// Temporisation (0x9281) as received: raw byte plus bit 0 and bit 7, no interpretation.
static String temporisation_text(uint16_t v) {
  if (v >= 0x100) {
    return String("not yet read");
  }
  char b[48];
  snprintf(b, sizeof(b), "raw 0x%02X (bit0=%u, bit7=%u)", (unsigned)v, (unsigned)(v & 1), (unsigned)((v >> 7) & 1));
  return String(b);
}

// Hex bytes; bytes whose bit is set in `mask` are put in [brackets] (the ones that changed).
static void append_hex_marked(String& s, const uint8_t* d, uint8_t n, uint8_t mask) {
  char b[8];
  for (uint8_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), (mask & (1u << i)) ? "[%02X]" : "%02X", d[i]);
    s += b;
    if (i + 1 < n) {
      s += ' ';
    }
  }
}

static void append_secs(String& s, long ms) {
  char b[24];
  snprintf(b, sizeof(b), "%ld.%ld s", ms / 1000, (ms % 1000) / 100);
  s += b;
}

static void append_mmss(String& s, unsigned long ms) {
  char b[16];
  unsigned long t = ms / 1000;
  snprintf(b, sizeof(b), "%02lu:%02lu", t / 60, t % 60);
  s += b;
}

// Sleep / quiet phase display: timer since nothing is sent, live BMS frame counter, per second and per
// 10 s history, when the BMS went silent, every CAN ID heard (which stops first, what changes), late
// replies and the values from before the run.
void RenaultTwingoGen1Battery::append_quiet_html(String& s) {
  const bool quiet = (NVROLstateMachine == 5);
  const bool active = UserRequestNVROLReset;
  const uint8_t mode = active ? nvrol_mode : nvrol_last_mode;
  const unsigned long now = millis();
  char b[120];

  if (mode == 1) {
    s += "Sleep (nothing sent at all, until Wake up): ";
  } else if (mode == 2) {
    s += "Sleep 0x9281=1 (temporisation written, then nothing sent until Wake up): ";
  } else {
    s += "NVROL reset (nothing sent after the sequence, until Wake up): ";
  }
  if (quiet) {
    unsigned long elapsed = now - nvrol_silence_start_ms;
    s += "<b>SILENT for ";
    append_mmss(s, elapsed);
    s += "</b> (safety limit ";
    append_mmss(s, sleep_manual_failsafe_ms());
    s += ") - nothing is sent, do not cut the 12V supply<br>";
  } else if (NVROLstateMachine == 7) {
    static const char* stage_names[4] = {"C3 (BAT TEMPO LEVEL)", "C2 (CUT OFF PENDING)", "C0 (SLEEPING)",
                                          "00"};
    s += "<b>vehicle state ";
    s += stage_names[powerdown_stage];
    s += "</b>, this stage for ";
    append_mmss(s, now - powerdown_stage_start_ms);
    s += ", shutdown running for ";
    append_mmss(s, now - powerdown_start_ms);
    s += " - our own frames still run, do not cut the 12V supply<br>";
  } else if (NVROLstateMachine == 8) {
    snprintf(b, sizeof(b), "<b>waking up</b> - sending burst frame %u/%u<br>", (unsigned)wake_burst_index,
             (unsigned)(WAKE_BURST_COUNT + 1));
    s += b;
  } else if (active) {
    s += "sequence running, silence follows<br>";
  } else if (!nvrol_silence_done) {
    s += "not run yet<br>";
  } else {
    s += "finished, silent for ";
    append_mmss(s, nvrol_silence_end_ms - nvrol_silence_start_ms);
    s += "<br>";
  }
  // During the wake-up burst (state 8) the info from the just-finished silence phase (frame counts,
  // ID table, ...) and the wake-timing line below are still worth showing, even though nvrol_silence_done
  // is only set true once the burst itself has also finished.
  // The recording (buckets/ID table below) now runs the whole way from the shutdown sequence's start
  // (state 7) through true silence (state 5), so it is shown live during state 7 too, not just once the
  // whole run is finished or genuinely silent.
  if (!quiet && !nvrol_silence_done && NVROLstateMachine != 8 && NVROLstateMachine != 7) {
    return;
  }

  const unsigned long duration_ms = (quiet || NVROLstateMachine == 7) ? (now - powerdown_start_ms)
                                                                       : (nvrol_silence_end_ms - powerdown_start_ms);

  snprintf(b, sizeof(b), "BMS frames received: <b>%lu</b>", (unsigned long)nvrol_silence_rx_total);
  s += b;
  if (quiet || NVROLstateMachine == 7) {
    if (nvrol_silence_rx_total == 0) {
      s += " - none so far";
    } else {
      unsigned long age = now - nvrol_silence_last_rx_ms;
      snprintf(b, sizeof(b), ", last one %lu.%lu s ago", age / 1000, (age % 1000) / 100);
      s += b;
      if (age > 3000) {
        s += " - <b>BMS SILENT since ";
        // Relative to true silence's own start while quiet (unchanged); during state 7, true silence has
        // not begun yet, so use the shutdown sequence's start instead - the same base the table below uses.
        append_mmss(s, quiet ? (nvrol_silence_last_rx_ms - nvrol_silence_start_ms)
                              : (nvrol_silence_last_rx_ms - powerdown_start_ms));
        s += "</b>";
      }
    }
  }
  s += "<br>Frames per second (first 60 s): ";
  unsigned long shown = duration_ms / 1000 + 1;
  if (shown > 60) {
    shown = 60;
  }
  for (unsigned long i = 0; i < shown; i++) {
    snprintf(b, sizeof(b), "%u", (unsigned)nvrol_silence_rx_per_s[i]);
    s += b;
    if (i + 1 < shown) {
      s += ',';
    }
  }
  s += "<br>";
  if (duration_ms >= 60000) {
    s += "Frames per 10 s (from 01:00): ";
    unsigned long last_bucket = duration_ms / 10000;
    if (last_bucket >= SILENCE_10S_BUCKETS) {
      last_bucket = SILENCE_10S_BUCKETS - 1;
    }
    for (unsigned long i = 6; i <= last_bucket; i++) {
      snprintf(b, sizeof(b), "%u", (unsigned)nvrol_silence_rx_per_10s[i]);
      s += b;
      if (i < last_bucket) {
        s += ',';
      }
    }
    s += "<br>";
  }

  // The "Result" verdict is only meaningful once the run has actually finished (or is genuinely silent):
  // during state 7 the BMS is expected to keep sending normally the whole time (our own frames still run),
  // so there is nothing to conclude yet.
  if (!quiet && nvrol_silence_done) {
    if (nvrol_silence_rx_total == 0) {
      s += "Result: no BMS frame at all - it was already silent when the shutdown sequence began.<br>";
    } else if (nvrol_silence_last_rx_ms <= nvrol_silence_start_ms) {
      // The BMS's last frame arrived before true silence even began - it stopped during the shutdown
      // announcement itself (C3/C2/C0/00), most likely right when our own frames stopped at 00.
      unsigned long before_ms = nvrol_silence_start_ms - nvrol_silence_last_rx_ms;
      snprintf(b, sizeof(b),
               "Result: BMS's last frame was %lu.%lu s BEFORE the quiet phase began - it was already silent "
               "during the shutdown announcement (that alone does not prove the pack slept).<br>",
               before_ms / 1000, (before_ms % 1000) / 100);
      s += b;
    } else {
      unsigned long last_ms = nvrol_silence_last_rx_ms - nvrol_silence_start_ms;
      unsigned long silence_duration_ms = nvrol_silence_end_ms - nvrol_silence_start_ms;
      if (last_ms + 3000 < silence_duration_ms) {
        snprintf(b, sizeof(b), "Result: BMS went silent after %lu.%lu s (measured from when the quiet phase began) - it stopped sending (that alone does not prove the pack slept).<br>",
                 last_ms / 1000, (last_ms % 1000) / 100);
      } else {
        snprintf(b, sizeof(b), "Result: BMS kept sending until the end - it did not go to sleep.<br>");
      }
      s += b;
    }
  }

  if (wake_tracking) {
    s += "Wake up: first BMS frame after ";
    if (wake_first_rx >= 0) {
      append_secs(s, wake_first_rx);
    } else {
      s += "- (none yet)";
    }
    s += ", first UDS reply after ";
    if (wake_first_uds >= 0) {
      append_secs(s, wake_first_uds);
    } else {
      s += "- (none yet)";
    }
    s += ", priority values ";
    if (wake_priority_done >= 0) {
      s += "complete after ";
      append_secs(s, wake_priority_done);
    } else if (wake_priority_timeout) {
      s += "NOT complete (gave up after 30 s)";
    } else {
      s += "pending";
    }
    s += "<br>";
  }

  if (nvrol_silence_id_count > 0) {
    s += "Frames by CAN ID, earliest silent first (count / data changes / last seen / last data):<br>";
    s += "Changed bytes of IDs with few changes are shown in [brackets], the last three changes per ID:<br>";
    uint8_t order[SILENCE_ID_MAX];
    for (uint8_t i = 0; i < nvrol_silence_id_count; i++) {
      order[i] = i;
    }
    for (uint8_t i = 1; i < nvrol_silence_id_count; i++) {  // insertion sort by last seen, ascending
      uint8_t v = order[i];
      int8_t k = i - 1;
      while (k >= 0 && nvrol_silence_ids[order[k]].last_ms > nvrol_silence_ids[v].last_ms) {
        order[k + 1] = order[k];
        k--;
      }
      order[k + 1] = v;
    }
    for (uint8_t n = 0; n < nvrol_silence_id_count; n++) {
      const SilenceId& r = nvrol_silence_ids[order[n]];
      unsigned long t = r.last_ms - powerdown_start_ms;
      snprintf(b, sizeof(b), "0x%03lX: %lu / %u / T+%lu.%lu s / ", (unsigned long)r.id, (unsigned long)r.count,
               (unsigned)r.changes, t / 1000, (t % 1000) / 100);
      s += b;
      append_hex_bytes(s, r.last, r.dlc > 8 ? 8 : r.dlc);
      s += "<br>";
      if (r.changes > 0 && r.changes <= 10) {  // status-like ID: show what changed and when
        const uint8_t len = r.dlc > 8 ? 8 : r.dlc;
        for (uint8_t k = 0; k < r.chg_count; k++) {
          const SilenceChange& ch = r.chg[(r.chg_count < 3) ? k : (uint8_t)((r.chg_next + k) % 3)];
          snprintf(b, sizeof(b), "&nbsp;&nbsp;change at T+%lu.%lu s: ", (unsigned long)(ch.t_ms / 1000),
                   (unsigned long)((ch.t_ms % 1000) / 100));
          s += b;
          append_hex_marked(s, ch.d, len, ch.mask);
          s += "<br>";
        }
      }
    }
    if (nvrol_silence_id_other > 0) {
      snprintf(b, sizeof(b), "(+ %lu frames of further IDs, table is full)<br>", (unsigned long)nvrol_silence_id_other);
      s += b;
    }
  }

  if (nvrol_silence_reply_total > 0) {
    snprintf(b, sizeof(b), "Replies on 0x18DAF1DB during the quiet phase: %u<br>", (unsigned)nvrol_silence_reply_total);
    s += b;
    for (uint8_t i = 0; i < nvrol_silence_reply_count; i++) {
      snprintf(b, sizeof(b), "t=%u.%u s: ", (unsigned)(nvrol_silence_reply[i].t_ms / 1000),
               (unsigned)((nvrol_silence_reply[i].t_ms % 1000) / 100));
      s += b;
      append_hex_bytes(s, nvrol_silence_reply[i].d, 8);
      s += "<br>";
    }
  }

  s += "Before the run - BMS state: ";
  if (nvrol_before.state_valid) {
    append_hex_bytes(s, nvrol_before.state, 32);
  } else {
    s += "not read";
  }
  s += "<br>Before the run - balancing in sleep: ";
  if (nvrol_before.bal_valid[2] && nvrol_before.bal_valid[3]) {
    s += String(bal_counter_value(nvrol_before.bal_raw[2]), 3);
    s += " Ah / ";
    s += String(bal_counter_value(nvrol_before.bal_raw[3]), 3);
    s += " h";
  } else {
    s += "not read";
  }
  s += "<br>Before the run - temporisation: ";
  s += temporisation_text(nvrol_before.temporisation);
  s += "<br>";
}

// While a reset is running (and until the priority PIDs have answered afterwards) the box refreshes
// itself once per second, so the countdown is visible without reloading the page.
void RenaultTwingoGen1Battery::append_refresh_script_html(String& s, bool busy) {
  if (!busy) {
    return;
  }
  s += "<script>(function(){function tick(){fetch(location.href,{cache:'no-store'})"
       ".then(function(r){return r.text();}).then(function(t){"
       "var d=new DOMParser().parseFromString(t,'text/html');"
       "var nb=d.getElementById('nvrolBox');var cur=document.getElementById('nvrolBox');"
       "if(!nb||!cur){return;}cur.innerHTML=nb.innerHTML;"
       "var a=nb.getAttribute('data-active');cur.setAttribute('data-active',a);"
       "if(a==='1'){setTimeout(tick,1000);}}).catch(function(){setTimeout(tick,2000);});}"
       "setTimeout(tick,1000);})();</script>";
}
#endif

void RenaultTwingoGen1Battery::handle_incoming_can_frame(CAN_frame rx_frame) {
#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (NVROLstateMachine == 5 || NVROLstateMachine == 7 || NVROLstateMachine == 8) {
    // Recording runs from the start of the shutdown announcement (state 7: C3/C2/C0/00) all the way
    // through true silence (state 5) AND the wake-up burst (state 8), on one continuous timeline - so
    // we can see what the BMS actually sends right after Wake up (e.g. the first 0x155 frames that
    // feed real_soc), not just whether/when it reacts. Kept as a separate condition from the
    // wake_first_rx tracking below (not else-if) so a frame seen during state 8 still counts for both.
    record_silence_frame(rx_frame);
  }
  if (wake_tracking && wake_first_rx < 0 && NVROLstateMachine != 5 && NVROLstateMachine != 7) {
    wake_first_rx = (int32_t)(millis() - wake_start_ms);  // first frame after Wake up
  }
#endif
  // UDS frames (0x7BB replies) are handled by the superclass.
  if (handle_incoming_uds_can_frame(rx_frame)) {
    return;
  }

  switch (rx_frame.ID) {
    case 0x155:  //10ms - Charging power, current and SOC - Confirmed sent by: Fluence ZE40, Zoe 22/41kWh, Kangoo 33kWh
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      LB_Charging_Power_W = rx_frame.data.u8[0] * 300;
      LB_Current_raw = ((rx_frame.data.u8[1] & 0x0F) << 8) | rx_frame.data.u8[2];
      LB_Display_SOC = ((rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5]);
      break;

    case 0x42E:  //NOTE: Not present on 41kWh battery!
      LB_Battery_Voltage = (((((rx_frame.data.u8[3] << 8) | (rx_frame.data.u8[4])) >> 5) & 0x3ff) * 0.5);  //0.5V/bit
      LB_Average_Temperature = (((((rx_frame.data.u8[5] << 8) | (rx_frame.data.u8[6])) >> 5) & 0x7F) - 40);
      break;
    case 0x424:  //100ms - Charge limits, Temperatures, SOH - Confirmed sent by: Fluence ZE40, Zoe 22/41kWh, Kangoo 33kWh
      LB_Heartbeat = rx_frame.data.u8[6];  // Alternates between 0x55 and 0xAA every 500ms (Same as on Nissan LEAF)
      if ((LB_Heartbeat != 0x55) && (LB_Heartbeat != 0xAA)) {
        datalayer_battery->status.CAN_error_counter++;
        break;
      }
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      LB_CUV = (rx_frame.data.u8[0] & 0x03);
      LB_HVBIR = (rx_frame.data.u8[0] & 0x0C) >> 2;
      LB_HVBUV = (rx_frame.data.u8[0] & 0x30) >> 4;
      LB_EOCR = (rx_frame.data.u8[0] & 0xC0) >> 6;
      LB_HVBOC = (rx_frame.data.u8[1] & 0x03);
      LB_HVBOT = (rx_frame.data.u8[1] & 0x0C) >> 2;
      LB_HVBOV = (rx_frame.data.u8[1] & 0x30) >> 4;
      LB_COV = (rx_frame.data.u8[1] & 0xC0) >> 6;
      LB_Regen_allowed_W = rx_frame.data.u8[2] * 500;
      LB_Discharge_allowed_W = rx_frame.data.u8[3] * 500;
      LB_SOH = rx_frame.data.u8[5];
      {
        // Boot plausibility filter for the temperatures (see the comment in the header): while it is active
        // only -20..+60 degC are accepted, it ends with the first plausible frame or 60 s after the first
        // 0x424 frame; from then on every value is taken over unfiltered.
        int16_t t_min = (int16_t)rx_frame.data.u8[4] - 40;
        int16_t t_max = (int16_t)rx_frame.data.u8[7] - 40;
        bool accept_temperatures = true;
        if (temp_boot_filter_active) {
          unsigned long now = millis();
          if (temp_boot_first_frame_ms == 0) {
            temp_boot_first_frame_ms = (now != 0) ? now : 1;  // 0 is reserved for "no frame yet"
          }
          bool plausible = (t_min >= TEMP_BOOT_MIN_C && t_min <= TEMP_BOOT_MAX_C && t_max >= TEMP_BOOT_MIN_C &&
                            t_max <= TEMP_BOOT_MAX_C);
          if (plausible || (now - temp_boot_first_frame_ms) >= TEMP_BOOT_FILTER_TIMEOUT_MS) {
            temp_boot_filter_active = false;
          } else {
            accept_temperatures = false;
          }
        }
        if (accept_temperatures) {
          LB_Cell_minimum_temperature = t_min;
          LB_Cell_maximum_temperature = t_max;
        }
      }
      break;
    case 0x425: {  //100ms Cellvoltages and kWh remaining - Confirmed sent by: Fluence ZE40 & Zoe Gen1
      // Boot plausibility filter (see the comment in the header): while active, only
      // EXT_425_BOOT_MIN_MV..EXT_425_BOOT_MAX_MV is accepted; it ends with the first plausible frame or
      // 60s after the first 0x425 frame, same mechanism as the 0x424 temperature filter above.
      uint16_t candidate_max = (((((rx_frame.data.u8[4] & 0x03) << 7) | (rx_frame.data.u8[5] >> 1)) * 10) + 1000);
      uint16_t candidate_min = (((((rx_frame.data.u8[6] & 0x01) << 8) | rx_frame.data.u8[7]) * 10) + 1000);
      bool accept_425 = true;
      if (ext425_boot_filter_active) {
        unsigned long now = millis();
        if (ext425_boot_first_frame_ms == 0) {
          ext425_boot_first_frame_ms = (now != 0) ? now : 1;  // 0 is reserved for "no frame yet"
        }
        bool plausible = (candidate_min >= EXT_425_BOOT_MIN_MV && candidate_min <= EXT_425_BOOT_MAX_MV &&
                           candidate_max >= EXT_425_BOOT_MIN_MV && candidate_max <= EXT_425_BOOT_MAX_MV);
        if (plausible || (now - ext425_boot_first_frame_ms) >= EXT_425_BOOT_FILTER_TIMEOUT_MS) {
          ext425_boot_filter_active = false;
        } else {
          accept_425 = false;
        }
      }
      if (accept_425) {
        LB_Cell_maximum_voltage = candidate_max;
        LB_Cell_minimum_voltage = candidate_min;
      }
      break;
    }
    case 0x427:  // NOTE: Not present on 41kWh battery!
      LB_kWh_Remaining = (((((rx_frame.data.u8[6] << 8) | (rx_frame.data.u8[7])) >> 6) & 0x3ff) * 0.1);
      break;
    case 0x445:                            //100ms - Confirmed sent by: Fluence ZE40 & Zoe Gen1
      LB_Heartbeat = rx_frame.data.u8[2];  // Alternates between 0x55 and 0xAA every 500ms (Same as on Nissan LEAF)
      if ((LB_Heartbeat != 0x55) && (LB_Heartbeat != 0xAA)) {
        datalayer_battery->status.CAN_error_counter++;
        break;
      }
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x654:  //SOC
      LB_SOC = rx_frame.data.u8[3];
      break;
    case 0x658:  // SOH candidate (OVMS: byte 4 & 0x7F, 127 = invalid) - display only, not used for anything else
      if (rx_frame.DLC >= 5) {
        soh_658 = rx_frame.data.u8[4] & 0x7F;
      }
      break;
#ifdef TWINGO_EXTENDED_CELL_POLLING
    case 0x18DAF1DB:  // Extended UDS LBC reply (cell voltages / balancing / lifetime metrics)
      if (wake_tracking && wake_first_uds < 0 && NVROLstateMachine != 5) {
        wake_first_uds = (int32_t)(millis() - wake_start_ms);  // first reply after Wake up
      }
      if (UserRequestNVROLReset) {
        // While the NVROL sequence is running, responses are Session
        // Control/RoutineControl/WriteDataByIdentifier replies, not
        // ReadDataByIdentifier ones - handle_extended_reply() would
        // misparse them (it assumes a PID sits at bytes 2-3).
        handle_nvrol_reply(rx_frame);
      } else if (dtc_ext_state == DTC_EXT_ERASE_CMD_SENT) {
        // Erase replies are always tiny (positive "54" / negative "7F 14 NRC"), never multi-frame -
        // the simple dedicated handler is enough, no need for the generic reassembly machinery.
        handle_dtc_ext_reply(rx_frame);
      } else {
        // Covers normal PID polls AND DTC Read replies (dtc_ext_state == DTC_EXT_READ_CMD_SENT): a
        // DTC Read reply can be multi-frame (confirmed 01.10. on this battery), so it needs the
        // generic reassembly path's flow-control handling - handle_extended_single_frame() and
        // handle_extended_multiframe_complete() both recognize and redirect it (see their DTC checks).
        handle_extended_reply(rx_frame);
      }
      break;
#endif
    default:
      break;
  }
}

#ifdef TWINGO_TIME_FRAMES
// ---------------------------------------------------------------------------
// Time frames (see the define at the top of the header). The three hooks below are the only places that
// touch WiFi / NTP / the system clock, the unit tests replace them.
// ---------------------------------------------------------------------------
bool RenaultTwingoGen1Battery::network_ready() {
#ifndef UNIT_TEST
  return WiFi.status() == WL_CONNECTED;
#else
  return false;
#endif
}

// Same call and servers as the Dala display firmware: German local time with daylight saving. It only sets the
// time zone and starts the SNTP client, which then keeps synchronising by itself.
void RenaultTwingoGen1Battery::start_ntp() {
#ifndef UNIT_TEST
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
#endif
}

bool RenaultTwingoGen1Battery::get_wall_clock_seconds_of_day(uint32_t& secs) {
  time_t now = time(nullptr);
  if (now < TIME_VALID_AFTER) {
    return false;  // not set yet (the ESP32 starts at 1970)
  }
  struct tm t;
  localtime_r(&now, &t);
  secs = (uint32_t)t.tm_hour * 3600UL + (uint32_t)t.tm_min * 60UL + (uint32_t)t.tm_sec;
  return true;
}

// Once per second: remember when the fallback clock started and, as soon as the WiFi station is connected, start
// NTP once (also when the WiFi only comes up after the boot).
void RenaultTwingoGen1Battery::time_service(unsigned long currentMillis) {
  if (!time_fallback_started) {
    time_fallback_started = true;
    time_fallback_start_ms = currentMillis;
  }
  if (currentMillis - previousMillis_time_service < INTERVAL_1_S) {
    return;
  }
  previousMillis_time_service = currentMillis;
  if (!ntp_started && network_ready()) {
    start_ntp();
    ntp_started = true;
  }
}

// 0x53B, called once per second from the own-broadcast block of transmit_can(). That block is skipped in
// true silence and from the "00" stage of the shutdown sequence on, so 0x53B stops with the other own frames.
void RenaultTwingoGen1Battery::send_time_frames(unsigned long currentMillis) {
  uint32_t secs = 0;
  if (!get_wall_clock_seconds_of_day(secs)) {
    secs = (TIME_FALLBACK_START_S + (uint32_t)((currentMillis - time_fallback_start_ms) / 1000UL)) % 86400UL;
  }
  TWINGO_53B_CLOCK.data.u8[0] = (uint8_t)((secs / 3600UL) << 3);
  TWINGO_53B_CLOCK.data.u8[1] = (uint8_t)(((secs / 60UL) % 60UL) << 2);
  TWINGO_53B_CLOCK.data.u8[2] = TIME_53B_BYTE2;
  TWINGO_53B_CLOCK.data.u8[3] = (uint8_t)((TIME_53B_YEAR_BITS << 6) | (secs % 60UL));
  TWINGO_53B_CLOCK.data.u8[4] = (uint8_t)(TIME_53B_MONTH << 4);
  TWINGO_53B_CLOCK.data.u8[5] = (uint8_t)((TIME_53B_DAY << 3) | TIME_53B_WEEKDAY);
  transmit_can_frame(&TWINGO_53B_CLOCK);
}

// The 0x350 run frame (state C3, every 100 ms like in the vehicle), called from the 100 ms block of
// transmit_can(). During the shutdown sequence and the wake burst those send their own 0x350.
void RenaultTwingoGen1Battery::send_run_350() {
#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (NVROLstateMachine == 7 || NVROLstateMachine == 8) {
    return;
  }
#endif
  transmit_can_frame(&TWINGO_350_RUN);
}
#endif  // TWINGO_TIME_FRAMES

#ifdef TWINGO_FAST_VEHICLE_FRAMES
// CRC-8 with poly 0x1D and start value 0 (same table as the Zoe Ph2 driver), final XOR per frame.
static uint8_t twingo_crc8(const uint8_t* d, uint8_t n, uint8_t xor_out) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < n; i++) {
    crc = crc8_table_SAE_J1850_ZER0[(crc ^ d[i]) & 0xFF];
  }
  return (uint8_t)(crc ^ xor_out);
}

// 0x090 (10 ms) and 0x242 (20 ms) like in the vehicle: they only run while it is awake. In the vehicle log they
// start with the first C3 frame after a wake-up and stop as soon as 0x350 goes to C0. Called every loop from the
// own-broadcast block, so they are also off in true silence and from the "00" stage on.
void RenaultTwingoGen1Battery::send_fast_frames(unsigned long currentMillis) {
#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (NVROLstateMachine == 7 && powerdown_stage >= 2) {
    return;  // C0 and 00 stage of the shutdown sequence
  }
  if (NVROLstateMachine == 8 && wake_burst_index == 0) {
    return;  // wake burst: the initial C0 frame has not been sent yet
  }
#endif
  if (currentMillis - previousMillis_090 >= INTERVAL_10_MS) {
    previousMillis_090 = currentMillis;
    fast_090_counter = (uint8_t)((fast_090_counter + 1) & 0x0F);
    TWINGO_090_FAST.data.u8[2] = (uint8_t)(0xE0 | fast_090_counter);
    const uint8_t crc_in[6] = {TWINGO_090_FAST.data.u8[0], TWINGO_090_FAST.data.u8[1], TWINGO_090_FAST.data.u8[2],
                               TWINGO_090_FAST.data.u8[4], TWINGO_090_FAST.data.u8[5], TWINGO_090_FAST.data.u8[6]};
    TWINGO_090_FAST.data.u8[3] = twingo_crc8(crc_in, 6, CRC_XOR_090);
    transmit_can_frame(&TWINGO_090_FAST);
  }
  if (currentMillis - previousMillis_242 >= INTERVAL_20_MS) {
    previousMillis_242 = currentMillis;
    fast_242_counter = (uint8_t)((fast_242_counter + 1) & 0x0F);
    TWINGO_242_FAST.data.u8[1] = (uint8_t)(fast_242_counter << 3);
    TWINGO_242_FAST.data.u8[7] = twingo_crc8(TWINGO_242_FAST.data.u8, 7, CRC_XOR_242);
    transmit_can_frame(&TWINGO_242_FAST);
  }
}

#endif  // TWINGO_FAST_VEHICLE_FRAMES

// 27 /simulator signals - content and interval_ms both taken from Log_Twingo_Ladung.log (02.10.), see the
// header comment. The 10 "I" entries (indices 0-9) are already sent for real by this driver's other
// functions (send_fast_frames()/send_time_frames()/etc.) - their rows here are kept for completeness per
// the planning session, toggling them sends an extra, redundant copy of the same content.
const RenaultTwingoGen1Battery::SimSignal RenaultTwingoGen1Battery::sim_signals[SIM_SIGNAL_COUNT] = {
    // --- 10 already-installed (I) ---
    {0x090, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 10, 'I', false, "0x090 Twingo-Fast (counter+CRC)"},
    {0x242, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 20, 'I', false, "0x242 Twingo-Fast (counter+CRC)"},
    {0x350, 8, {0xC0, 0x26, 0x64, 0x7D, 0x14, 0x70, 0x96, 0x85}, 100, 'I', false, "0x350 Vehicle state (C0)"},
    {0x19F, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 100, 'I', false, "0x19F (upstream PR #2907)"},
    {0x426, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 100, 'I', false, "0x426 (upstream PR #2907)"},
    {0x436, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 100, 'I', false, "0x436 (upstream PR #2907)"},
    {0x423, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 100, 'I', false, "0x423"},
    {0x69F, 8, {0, 0, 0, 0, 0, 0, 0, 0}, 1000, 'I', false, "0x69F (upstream PR #2907)"},
    {0x53B, 6, {0, 0, 0, 0, 0, 0, 0, 0}, 1000, 'I', false, "0x53B Time frame"},
    {0x214, 2, {0, 0, 0, 0, 0, 0, 0, 0}, 20, 'I', false, "0x214 (shutdown only)"},
    // --- 10ms group, new (P/A) ---
    {0x1F8, 8, {0x00, 0x84, 0xFF, 0xFF, 0xFE, 0x00, 0x00, 0x0F}, 10, 'P', false, "0x1F8 EVC heartbeat"},
    {0x18A, 8, {0xFF, 0xF0, 0x00, 0x06, 0x40, 0x3C, 0xD5, 0x70}, 10, 'P', true, "0x18A EVC/LBC response"},
    {0x17A, 8, {0xFF, 0xFF, 0xFF, 0xBB, 0x00, 0xF0, 0x31, 0xA3}, 10, 'A', true, "0x17A"},
    {0x17E, 8, {0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x40, 0x00, 0xFF}, 10, 'A', true, "0x17E (gear shift ID, byte6 varies)"},
    {0x186, 7, {0x00, 0x00, 0x32, 0x03, 0x20, 0x00, 0x20, 0}, 10, 'A', true, "0x186"},
    {0x1F6, 8, {0x1E, 0x00, 0xC0, 0x1D, 0x00, 0xFF, 0xFF, 0xFF}, 10, 'A', true, "0x1F6 (byte3 varies)"},
    // --- 20ms group, new (P/A) ---
    {0x211, 8, {0x80, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}, 20, 'P', false, "0x211 (Klemme15/Fahren)"},
    {0x1B0, 4, {0xFF, 0x2C, 0xFF, 0xC0, 0, 0, 0, 0}, 20, 'P', false, "0x1B0"},
    {0x217, 8, {0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xFF}, 20, 'A', true, "0x217"},
    // --- 100ms group, new (P/A) ---
    {0x5DE, 8, {0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x20, 0x42}, 100, 'A', false, "0x5DE"},
    {0x5DF, 3, {0xFC, 0x05, 0x00, 0, 0, 0, 0, 0}, 100, 'A', false, "0x5DF"},
    {0x634, 6, {0x80, 0x00, 0x00, 0x10, 0x00, 0x00, 0, 0}, 100, 'A', false, "0x634"},
    // --- was assumed 1000ms, confirmed 100ms in the real log (A) ---
    {0x427, 8, {0xDB, 0xFF, 0x00, 0x0F, 0xFF, 0x01, 0x1E, 0xC0}, 100, 'A', false, "0x427"},
    {0x42E, 8, {0x62, 0x1F, 0xD0, 0x5D, 0x44, 0x05, 0x80, 0xFF}, 100, 'A', false, "0x42E HV voltage/temp"},
    {0x432, 8, {0x50, 0x3F, 0xF6, 0x08, 0x00, 0x00, 0x00, 0x40}, 100, 'A', false, "0x432"},
    {0x650, 8, {0x00, 0x00, 0x00, 0x16, 0xC0, 0x53, 0xFE, 0x00}, 100, 'A', false, "0x650"},
    {0x1FD, 8, {0xFE, 0x40, 0x7F, 0xFF, 0x7F, 0x50, 0x50, 0x00}, 100, 'A', false, "0x1FD"},
    // 0x55D (02.10.): not in the original 27-signal plan, found while discussing a different topic.
    // Content/interval/direction as discussed; byte 4 is NOT a fast alive counter in the real log (only 2
    // transitions in the whole ~6min capture, 90/91/92 - see that discussion) - represented here as its
    // first observed steady value, 0x91, not re-derived as a counter.
    {0x55D, 8, {0x06, 0xFD, 0xF4, 0x0F, 0x91, 0x00, 0x00, 0x81}, 100, 'A', false, "0x55D"},
};

// EXPERIMENTAL override for the 0x55D row above, content from an unsourced text (no log/code evidence,
// see the 02.10. discussion) claiming a static "drive/discharge active" pattern. NOT from
// Log_Twingo_Ladung.log - that log never covers driving, only stationary charging. Kept deliberately
// separate from the main sim_signals table/checkboxes: toggling this does not touch the simulator_enabled_
// mask bit for 0x55D, it only swaps which bytes get sent IF that bit is already on.
static const uint8_t SIM_55D_DRIVE_MODE_DATA[8] = {0x05, 0xFD, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x81};

// EXPERIMENTAL, GUESSED (02.10.): "rest/idle" content for 0x55D - only byte 0 changed from the real,
// confirmed content (same reasoning as the drive-mode override: not from any log, no vehicle idle capture
// exists). 0x01 taken from Gemini's original, unconfirmed suggestion ("Sleep/Init").
static const uint8_t SIM_55D_REST_DATA[8] = {0x01, 0xFD, 0xF4, 0x0F, 0x91, 0x00, 0x00, 0x81};

void RenaultTwingoGen1Battery::send_simulator_signals(unsigned long currentMillis) {
  // Same true-silence rule as send_fast_frames()/the old send_evc_heartbeat() - applies to ALL 28 rows,
  // not just 0x1F8/0x18A (02.10. decision): C0/00 stage of shutdown, and the start of the wake burst.
#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (NVROLstateMachine == 7 && powerdown_stage >= 2) {
    return;
  }
  if (NVROLstateMachine == 8 && wake_burst_index == 0) {
    return;
  }
  const bool relays_virtually_open = (NVROLstateMachine == 5 || NVROLstateMachine == 7);
#else
  const bool relays_virtually_open = false;
#endif

  uint32_t mask = datalayer_extended.twingoGen1.simulator_enabled_mask;
  for (uint8_t i = 0; i < SIM_SIGNAL_COUNT; i++) {
    if (!(mask & (1UL << i))) {
      continue;
    }
    if (sim_signals[i].tag == 'I') {
      // Already sent for real, with real dynamic content (counters/CRCs this table does not have), by
      // this driver's other functions. Sending the table's static placeholder bytes instead would be a
      // WRONG duplicate, not a harmless one - skip. Row stays visible/checked on the page as a reference.
      continue;
    }
    if (currentMillis - sim_last_send_ms[i] < sim_signals[i].interval_ms) {
      continue;
    }
    sim_last_send_ms[i] = currentMillis;
    CAN_frame f = {.FD = false, .ext_ID = false, .DLC = sim_signals[i].dlc, .ID = sim_signals[i].id};
    memcpy(f.data.u8, sim_signals[i].data, 8);
    if (sim_signals[i].id == 0x1F8) {
      // Relay/contactor status byte, coupled to the sleep state machine - same logic the old, now-removed
      // send_evc_heartbeat() used (01.10. real-log analysis: 0x1F8 byte 5 went FA->00 as contactors closed).
      f.data.u8[5] = relays_virtually_open ? 0xFA : 0x00;
    } else if (sim_signals[i].id == 0x18A) {
      // Rolling counter, byte 7: observed real sequence 0x70,0x80,...,0xF0,0x00,0x10,... (steps of 0x10,
      // wraps at 0x100). Same counter variable/logic as the old send_evc_heartbeat().
      sim_18a_counter = (uint8_t)((sim_18a_counter + 1) & 0x0F);
      f.data.u8[7] = (uint8_t)(sim_18a_counter << 4);
    } else if (sim_signals[i].id == 0x55D) {
      bool restActive = datalayer_extended.twingoGen1.sim_55d_rest_active_enabled;
      bool restActivePrev = datalayer_extended.twingoGen1.sim_55d_rest_active_prev;
      if (restActive && !restActivePrev) {
        // Rising edge: start the staged precharge/main-relay sequence.
        datalayer_extended.twingoGen1.sim_55d_stage_start_ms = currentMillis;
      } else if (!restActive && restActivePrev) {
        // Falling edge: back to REST immediately, no staged animation back.
        datalayer_extended.twingoGen1.sim_55d_stage_start_ms = 0;
      }
      datalayer_extended.twingoGen1.sim_55d_rest_active_prev = restActive;

      if (!restActive) {
        memcpy(f.data.u8, SIM_55D_REST_DATA, 8);
      } else if (datalayer_extended.twingoGen1.sim_55d_drive_mode_enabled) {
        memcpy(f.data.u8, SIM_55D_DRIVE_MODE_DATA, 8);
      }
      // else: restActive and not drive mode -> leave the already-memcpy'd normal/real content as-is

      // Staged sequence overrides byte 0 only, on top of whichever target content was just selected above
      // - see the header comment, GUESSED intermediate values (precharge, then main relay closing).
      unsigned long stage_start = datalayer_extended.twingoGen1.sim_55d_stage_start_ms;
      if (stage_start != 0) {
        unsigned long elapsed = currentMillis - stage_start;
        if (elapsed < SIM_55D_STAGE_DURATION_MS) {
          f.data.u8[0] = 0x02;  // GUESSED: "Precharge"
        } else if (elapsed < 2 * SIM_55D_STAGE_DURATION_MS) {
          f.data.u8[0] = 0x04;  // GUESSED: "Main relay closing"
        } else {
          datalayer_extended.twingoGen1.sim_55d_stage_start_ms = 0;  // done, settle on the target content
        }
      }
    }
    transmit_can_frame(&f);
  }
}

void RenaultTwingoGen1Battery::transmit_can(unsigned long currentMillis) {
#ifdef TWINGO_TIME_FRAMES
  time_service(currentMillis);  // NTP start / fallback clock, transmits nothing itself
#endif
#ifdef TWINGO_EXTENDED_CELL_POLLING
  // NVROL/Sleep sequences: transmit nothing extra of our own (no legacy UDS) while a run is starting or
  // truly silent. Only the timer runs in true silence; frames from the BMS are still received and counted.
  if (UserRequestNVROLReset && nvrol_mode == 1 && NVROLstateMachine == 0) {
    // "Sleep": go straight into the shutdown sequence before anything else can be transmitted.
    transmit_reset_nvrol_frames();
  }
  if (NVROLstateMachine == 5) {
    // True silence lasts until Wake up is pressed: on purpose the "battery alive" watchdog must not trip.
    datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
    transmit_reset_nvrol_frames();
    return;
  }
#endif

#ifdef TWINGO_EXTENDED_CELL_POLLING
  // During the shutdown sequence's final "00" stage the vehicle has effectively gone silent too - stop our
  // own broadcast frames here already, not just once true silence (state 5) begins right after.
  const bool suppress_own_broadcast = (NVROLstateMachine == 7 && powerdown_stage >= 3);
#else
  const bool suppress_own_broadcast = false;
#endif
  if (suppress_own_broadcast) {
    datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
  } else {
  // Send 100ms CAN Message (the BMS only answers diagnostic requests while it
  // receives this wakeup frame)
  if (currentMillis - previousMillis100 >= INTERVAL_100_MS) {
    previousMillis100 = currentMillis;
    transmit_can_frame(&ZOE_423);

    if ((counter_423 / 5) % 2 == 0) {  // Alternate every 5 messages between these two
      ZOE_423.data.u8[4] = 0xB2;
      ZOE_423.data.u8[6] = 0xB2;
    } else {
      ZOE_423.data.u8[4] = 0x5D;
      ZOE_423.data.u8[6] = 0x5D;
    }
    counter_423 = (counter_423 + 1) % 10;

    // Broadcast 100ms vehicle frames (PEB Inverter 0x19F, EVC Power Mux 0x426, EVC Status 0x436)
    // Rolling 4-bit sequence counter (cycles 0-15)
    ZOE_19F_INVERTER.data.u8[3] = (zoe_19F_counter++ & 0x0F);
    transmit_can_frame(&ZOE_19F_INVERTER);

    transmit_can_frame(&ZOE_426_POWER_MUX);

    transmit_can_frame(&ZOE_436_VEHICLE_STATUS);

#ifdef TWINGO_TIME_FRAMES
    send_run_350();  // 0x350 vehicle state C3, every 100 ms
#endif

#ifdef TWINGO_EXTENDED_CELL_POLLING
    if (NVROLstateMachine == 7) {
      // Experiment: 0x214 only during the shutdown sequence's C3/C2 stages (see header comment). Stage 0/1
      // (C3/C2, "announcement active") -> 08 00; stage 2 (C0, "sleeping") -> F8 3E, matching the real log.
      // Stage 3 (00) is already covered by suppress_own_broadcast further up - not sent there either.
      TWINGO_214_EVC_SLEEP_REQ.data.u8[0] = (powerdown_stage <= 1) ? 0x08 : 0xF8;
      TWINGO_214_EVC_SLEEP_REQ.data.u8[1] = (powerdown_stage <= 1) ? 0x00 : 0x3E;
      transmit_can_frame(&TWINGO_214_EVC_SLEEP_REQ);
    }
#endif
  }

  // Update EVC 0x436 vehicle runtime clock every 60s
  if (currentMillis - previousMillis60000_436 >= INTERVAL_60_S) {
    previousMillis60000_436 = currentMillis;
    zoe_436_counter++;
    ZOE_436_VEHICLE_STATUS.data.u8[2] = (zoe_436_counter >> 8) & 0xFF;
    ZOE_436_VEHICLE_STATUS.data.u8[3] = zoe_436_counter & 0xFF;
  }

  // Broadcast 1000ms BCM Gateway alive token
  if (currentMillis - previousMillis1000_69f >= INTERVAL_1_S) {
    previousMillis1000_69f = currentMillis;
    transmit_can_frame(&ZOE_69F_BCM_GATEWAY);
#ifdef TWINGO_TIME_FRAMES
    send_time_frames(currentMillis);  // 0x53B clock, same 1 Hz cycle
#endif
  }

#ifdef TWINGO_FAST_VEHICLE_FRAMES
  send_fast_frames(currentMillis);     // 0x090 every 10 ms, 0x242 every 20 ms
  send_simulator_signals(currentMillis);  // /simulator page, per-signal checkboxes (incl. 0x1F8/0x18A)
#endif
  }  // !suppress_own_broadcast

#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (UserRequestNVROLReset) {
    // NVROL reset / Sleep / shutdown sequence / wake burst in progress: run its state machine instead of
    // normal extended polling, since both share the ZOE_POLL_18DADBF1 frame object below.
    transmit_reset_nvrol_frames();
  } else {
  handle_dtc_ext(currentMillis);  // DTC Read/Erase probe (see read_DTC()/reset_DTC()), if one is running
  // Extended-address polling: cycle through the 119 poll targets (96 cell
  // voltages + balancing + 4 lifetime metrics + temporisation + 8 pack
  // temperatures + BMS state + 6 balancing counters + 2 time PIDs), one every 200ms (same
  // cadence as Battery-Emulator's own Zoe Ph2 driver) -> ~23.8s per full cycle.
  // Right after the NVROL quiet phase a few PIDs are asked first, see ext_priority_list.
  //
  // Paused entirely (not just Cellwatch) while a DTC exchange is in flight (dtc_ext_state != IDLE):
  // 01.10. bug found - round-robin/Cellwatch kept sending their own unrelated requests on the same
  // CAN ID while a DTC multi-frame reply was still being reassembled. A normal single-frame poll
  // response in between Consecutive Frames would set ext_isotp_in_progress = false (see
  // handle_extended_reply), silently aborting the DTC reassembly already in progress - matches a
  // real-world "no response" result that followed a successful "unexpected multi-frame" one. Same
  // "pausieren, nicht verzahnen" principle as Cellwatch, just gated on dtc_ext_state too now.
  if (dtc_ext_state != DTC_EXT_IDLE) {
    // Nothing to do here - handle_dtc_ext() above drives the whole exchange by itself.
  } else if (datalayer_extended.twingoGen1.cellwatch_enabled) {
    // Cellwatch active: the normal round-robin is fully paused - previousMillisExtPoll/ext_poll_index
    // are simply never touched here, so it resumes from exactly where it left off once Cellwatch is
    // turned off again (Variante 1: pausieren, nicht verzahnen). Only the selected cell is requested,
    // back-to-back, limited only by CELLWATCH_MIN_GAP_MS.
    if (currentMillis - previousMillisCellwatch >= CELLWATCH_MIN_GAP_MS) {
      previousMillisCellwatch = currentMillis;
      uint8_t cell = datalayer_extended.twingoGen1.cellwatch_cell;
      if (cell < 1) {
        cell = 1;
      }
      if (cell > 96) {
        cell = 96;
      }
      uint16_t pid = ext_poll_list[cell - 1];  // cell 1..96 -> the first 96 entries, in order
      // Bytes 0/1 (PCI/SID) set explicitly every time, not assumed left over from the previous send -
      // ZOE_POLL_18DADBF1 is a shared frame object; DTC Read/Erase (or any future sequence) can leave
      // it on a different SID (see handle_dtc_ext's bug fix comment), which would otherwise silently
      // corrupt every poll after it.
      ZOE_POLL_18DADBF1.data.u8[0] = 0x03;
      ZOE_POLL_18DADBF1.data.u8[1] = 0x22;
      ZOE_POLL_18DADBF1.data.u8[2] = (uint8_t)((pid >> 8) & 0xFF);
      ZOE_POLL_18DADBF1.data.u8[3] = (uint8_t)(pid & 0xFF);
      transmit_can_frame(&ZOE_POLL_18DADBF1);
#ifdef EXTENDED_UDS_DEBUG
      logging.printf("EXT UDS TX (cellwatch): PID=0x%04X\n", pid);
#endif
    }
  } else if (currentMillis - previousMillisExtPoll >= EXT_POLL_INTERVAL_MS) {
    previousMillisExtPoll = currentMillis;
    uint16_t pid;
    if (ext_priority_pending_mask != 0 && (currentMillis - ext_priority_start_ms) < EXT_PRIORITY_TIMEOUT_MS) {
      // After the quiet phase: ask the priority PIDs that have not answered yet, round-robin.
      uint8_t tries = 0;
      while (!(ext_priority_pending_mask & (1u << ext_priority_next)) && tries < EXT_PRIORITY_COUNT) {
        ext_priority_next = (ext_priority_next + 1) % EXT_PRIORITY_COUNT;
        tries++;
      }
      pid = ext_priority_list[ext_priority_next];
      ext_priority_next = (ext_priority_next + 1) % EXT_PRIORITY_COUNT;
    } else {
      if (ext_priority_pending_mask != 0) {
        wake_priority_timeout = true;  // gave up after 30s
      }
      ext_priority_pending_mask = 0;
      pid = ext_poll_list[ext_poll_index];
      ext_poll_index = (ext_poll_index + 1) % EXT_POLL_LIST_LENGTH;
    }
    ZOE_POLL_18DADBF1.data.u8[0] = 0x03;  // see the matching comment in the Cellwatch branch above
    ZOE_POLL_18DADBF1.data.u8[1] = 0x22;
    ZOE_POLL_18DADBF1.data.u8[2] = (uint8_t)((pid >> 8) & 0xFF);
    ZOE_POLL_18DADBF1.data.u8[3] = (uint8_t)(pid & 0xFF);
    transmit_can_frame(&ZOE_POLL_18DADBF1);
#ifdef EXTENDED_UDS_DEBUG
    logging.printf("EXT UDS TX: PID=0x%04X\n", pid);
#endif
  }

  // Abandon a stalled multi-frame reassembly rather than let it block forever.
  if (ext_isotp_in_progress && (currentMillis - ext_isotp_started_ms >= EXT_ISOTP_TIMEOUT_MS)) {
    ext_isotp_in_progress = false;
    datalayer_battery->status.balancing_status = BALANCING_STATUS_ERROR;
#ifdef EXTENDED_UDS_DEBUG
    logging.printf("EXT UDS: reassembly timed out after %lums (%u/%u bytes received)\n", EXT_ISOTP_TIMEOUT_MS,
                    ext_isotp_received_len, ext_isotp_expected_len);
#endif
  }
  }
#endif

  // UDS PID polling and DTC handling
  transmit_uds_can(currentMillis);
}

template <typename T>
inline String& operator<<(String& str, const T& value) {
  str += value;
  return str;
}

String RenaultTwingoGen1Battery::get_uds_info_html() {
  String content;
  content.reserve(9000);

  // clang-format off
  content << "Cell Under Voltage: " << (LB_CUV >= 2 ? "FAULT" : "OK") << "<br>"
             "Cell Over Voltage: " << (LB_COV >= 2 ? "FAULT" : "OK") << "<br>"
             "Pack Under Voltage: " << (LB_HVBUV >= 2 ? "FAULT" : "OK") << "<br>"
             "Pack Over Voltage: " << (LB_HVBOV >= 2 ? "FAULT" : "OK") << "<br>"
             "Pack Over Current: " << (LB_HVBOC >= 2 ? "FAULT" : "OK") << "<br>"
             "Over Temp: " << (LB_HVBOT >= 2 ? "FAULT" : "OK") << "<br>"
             "Isolation: " << (LB_HVBIR >= 2 ? "FAULT" : "OK") << "<br>"
             "End Of Charge: " << (LB_EOCR >= 2 ? "YES" : "NO") << "<br>"
             "Battery Mileage: " << battery_mileage_in_km << " km<br>"
             "Lifetime Energy: " << kWh_from_beginning_of_battery_life << " kWh<br>";
  // clang-format on

#ifdef TWINGO_EXTENDED_CELL_POLLING
  // Everything below sits in one box that refreshes itself while an NVROL reset is running (countdown of
  // the quiet phase) and until the priority PIDs have answered afterwards.
  const bool nvrol_busy = UserRequestNVROLReset || (ext_priority_pending_mask != 0);
  // clang-format off
  content << "<div id='nvrolBox' data-active='" << (nvrol_busy ? "1" : "0") << "'>"
             "Charge Cycles: " << battery_charge_cycles << "<br>"
             "Energy Charged: " << battery_energy_charged_kWh << " kWh<br>"
             "Energy Discharged: " << battery_energy_discharged_kWh << " kWh<br>"
             "Energy Regenerated: " << battery_energy_regenerated_kWh << " kWh<br>"
             "Temporisation (0x9281): " << temporisation_text(battery_temporisation) << "<br>";
  // clang-format on

  // NVROL/Sleep settings: 0x9281 write value and B009 diagnostic session type (each an exclusive pair of
  // checkboxes - clicking one always forces the other off), plus the manual sleep failsafe window.
  // Persisted to NVM, same pattern as the BYD Atto3 auto-calibrate settings.
  {
    bool write0 = (datalayer_extended.twingoGen1.nvrol_temporisation_write_value == 0);
    bool prog = datalayer_extended.twingoGen1.nvrol_b009_use_programming_session;
    content += "<h4>0x9281 write value (NVROL reset + Sleep 0x9281=1): "
               "<input type='checkbox' id='twingoWrite00' onclick='twingoSetWriteValue(0)' ";
    content += write0 ? "checked>" : ">";
    content += " 0x00 (activated) "
               "<input type='checkbox' id='twingoWrite01' onclick='twingoSetWriteValue(1)' ";
    content += write0 ? ">" : "checked>";
    content += " 0x01</h4>";

    content += "<h4>B009 diagnostic session: "
               "<input type='checkbox' id='twingoB009Ext' onclick='twingoSetB009Session(0)' ";
    content += prog ? ">" : "checked>";
    content += " Extended (0x03) "
               "<input type='checkbox' id='twingoB009Prog' onclick='twingoSetB009Session(1)' ";
    content += prog ? "checked>" : ">";
    content += " Programming (0x02)</h4>";

    content += "<h4>Sleep failsafe (auto wake after): "
               "<input type='number' id='twingoSleepMinutes' min='1' max='1440' value='";
    content += String(datalayer_extended.twingoGen1.sleep_failsafe_minutes);
    content += "'> min <button onclick='twingoSetSleepMinutes()'>Set</button></h4>";

    content += "<h4>Cellwatch (fast single-cell poll): "
               "<input type='checkbox' id='twingoCellwatchEnable' onclick='twingoSetCellwatchEnable(this.checked)' ";
    content += datalayer_extended.twingoGen1.cellwatch_enabled ? "checked>" : ">";
    content += " enable, cell <input type='number' id='twingoCellwatchCell' min='1' max='96' value='";
    content += String(datalayer_extended.twingoGen1.cellwatch_cell);
    content +=
        "'> <button onclick='twingoSetCellwatchCell()'>Set</button> "
        "<button onclick=\"window.open('/cellwatch','_blank')\">Open Cellwatch page</button>"
        " - pauses the normal cell round-robin while enabled</h4>";

    content +=
        "<h4><button onclick=\"window.open('/simulator','_blank')\">Open CAN Signal Simulator page</button>"
        " - 28 individually toggleable cyclic signals</h4>";

    content += "<script>";
    content += "function twingoSetWriteValue(v){";
    content += "document.getElementById('twingoWrite00').checked=(v===0);";
    content += "document.getElementById('twingoWrite01').checked=(v===1);";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoNvrolWriteValue?value='+v,true);x.send();}";
    content += "function twingoSetB009Session(p){";
    content += "document.getElementById('twingoB009Ext').checked=(p===0);";
    content += "document.getElementById('twingoB009Prog').checked=(p===1);";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoB009Session?value='+p,true);x.send();}";
    content += "function twingoSetSleepMinutes(){";
    content += "var m=document.getElementById('twingoSleepMinutes').value;";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoSleepMinutes?value='+m,true);x.send();}";
    content += "function twingoSetCellwatchEnable(v){";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoCellwatchEnable?value='+(v?1:0),true);x.send();}";
    content += "function twingoSetDtcAllStatus(v){";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoDtcAllStatus?value='+(v?1:0),true);x.send();}";
    content += "function twingoSetCellwatchCell(){";
    content += "var c=document.getElementById('twingoCellwatchCell').value;";
    content += "var x=new XMLHttpRequest();x.open('GET','/editTwingoCellwatchCell?value='+c,true);x.send();}";
    content += "</script>";
  }

  append_live_html(content);
  // clang-format off
  content << "NVROL Log - Session1: " << nvrol_log[0] << "<br>"
             "NVROL Log - Routine B009: " << nvrol_log[1] << "<br>"
             "NVROL Log - Routine B009 results: " << nvrol_log[5] << "<br>"
             "NVROL Log - Session2: " << nvrol_log[2] << "<br>"
             "NVROL Log - Write 9281=0 (activated): " << nvrol_log[3] << "<br>"
             "NVROL Log - Read back 0x9281: " << nvrol_log[4] << "<br>"
             "Temporisation right after the write (read back): " << temporisation_text(temporisation_readback) << "<br>"
             "<input type='checkbox' id='twingoDtcAllStatus' onclick='twingoSetDtcAllStatus(this.checked)' "
             << (datalayer_extended.twingoGen1.dtc_ext_read_mask == 0xFF ? "checked>" : ">") <<
             " Read ALL DTC statuses (mask 0xFF) instead of only Active/Confirmed (0x09) on next Read DTC<br>"
             "DTC Read (ext. protocol, Read DTC button): " << dtc_ext_log_read << "<br>"
             "DTC Erase (ext. protocol, Erase DTC button): " << dtc_ext_log_erase << "<br>"
             "<button onclick=\"fetch('/triggerTwingoDtcDetails')\">Read DTC details (EXPERIMENTAL, UNTESTED, "
             "subfunction 0x06)</button><br>"
             "DTC details - " << String(DTC_DETAILS_CODES[0], HEX) << ": " << dtc_ext_log_details[0] << "<br>"
             "DTC details - " << String(DTC_DETAILS_CODES[1], HEX) << ": " << dtc_ext_log_details[1] << "<br>";
  // clang-format on
  append_quiet_html(content);
  content << "</div>";
  append_refresh_script_html(content, nvrol_busy);
#endif

  return content;
}

void RenaultTwingoGen1Battery::setup(void) {  // Performs one time setup at startup
  // Suppress CAN_NATIVE_BUS_ERROR/CAN_NATIVE_BUFFER_FULL for this interface during the boot window (same
  // duration as the 0x425/0x424 boot filters above) - covers the brief CAN-error burst right after
  // power-on/reset, same framework function MG-GEN1-BATTERY.cpp and MEB-BATTERY.cpp already use around
  // their own BMS-reset moments (02.10. phantom BATTERY_OVERVOLTAGE investigation).
  ignore_can_errors_for(can_interface, EXT_425_BOOT_FILTER_TIMEOUT_MS);

  // UDS: send requests/flow control to 0x79B, accept replies from the BMS on 0x7BB.
  setup_uds(0x79B, 0x7BB);

  // The Twngo Gen1 BMS only speaks KWP2000-style one-byte local identifiers.
  set_pid_scan_mode(PidScanMode::OneByteLocalId);

  static const uint16_t pid_scan_list[] = {
      GROUP1_CELLVOLTAGES_1_POLL,  // Cells 1-62
      GROUP2_CELLVOLTAGES_2_POLL,  // Cells 63-96
      GROUP6_BALANCING,            // Balancing status bits
      GROUP3_METRICS,              // Mileage + alltime energy
  };
  set_pid_scan_list(pid_scan_list, sizeof(pid_scan_list) / sizeof(pid_scan_list[0]));

  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.system.status.battery_allows_contactor_closing = true;
  datalayer_battery->info.number_of_cells = 96;
  datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
#ifdef TWINGO_EXTENDED_CELL_POLLING
  // This battery provides 8 single pack temperature sensors (shown on the Cellmonitor page).
  datalayer_battery->status.temperature_sensors_count = EXT_TEMP_SENSOR_COUNT;
#endif
}

// The generic UdsCanBattery sequence machinery (start_sequence/send_sequence_message/
// on_uds_sequence_step) is bound to setup_uds(0x79B, 0x7BB) - the standard KWP2000 path, which
// RenoLink confirmed (01.10.) never answers on this battery (same dead path as the generic
// supports_read_DTC()/supports_reset_DTC() buttons' default implementation would have used). read_DTC()
// and reset_DTC() below are therefore NOT routed through that machinery at all; they drive the separate
// dtc_ext_* state machine instead, which talks to the extended 29-bit protocol (0x18DADBF1/0x18DAF1DB)
// that this battery actually responds to for everything else (NVROL, 0x9281, cell polling).
void RenaultTwingoGen1Battery::on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  // Intentionally empty - see comment above.
}

void RenaultTwingoGen1Battery::read_DTC() {
  if (dtc_ext_state != DTC_EXT_IDLE || UserRequestNVROLReset) {
    return;  // already busy with a DTC exchange, or a Sleep/NVROL sequence is running
  }
  ZOE_POLL_18DADBF1.data = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};  // open extended session
  transmit_can_frame(&ZOE_POLL_18DADBF1);
  strncpy(dtc_ext_log_read, "requested", sizeof(dtc_ext_log_read) - 1);
  dtc_ext_log_read[sizeof(dtc_ext_log_read) - 1] = '\0';
  dtc_ext_state = DTC_EXT_READ_SESSION_SENT;
  dtc_ext_step_start_ms = millis();
}

const uint32_t RenaultTwingoGen1Battery::DTC_DETAILS_CODES[DTC_DETAILS_COUNT] = {0xE14381, 0x1B0715};

void RenaultTwingoGen1Battery::read_DTC_details() {
  if (dtc_ext_state != DTC_EXT_IDLE || UserRequestNVROLReset) {
    return;
  }
  dtc_ext_details_index = 0;
  ZOE_POLL_18DADBF1.data = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};  // open extended session
  transmit_can_frame(&ZOE_POLL_18DADBF1);
  for (uint8_t i = 0; i < DTC_DETAILS_COUNT; i++) {
    strncpy(dtc_ext_log_details[i], "requested", sizeof(dtc_ext_log_details[i]) - 1);
    dtc_ext_log_details[i][sizeof(dtc_ext_log_details[i]) - 1] = '\0';
  }
  dtc_ext_state = DTC_EXT_DETAILS_SESSION_SENT;
  dtc_ext_step_start_ms = millis();
}

void RenaultTwingoGen1Battery::reset_DTC() {
  if (dtc_ext_state != DTC_EXT_IDLE || UserRequestNVROLReset) {
    return;
  }
  ZOE_POLL_18DADBF1.data = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};  // open extended session
  transmit_can_frame(&ZOE_POLL_18DADBF1);
  strncpy(dtc_ext_log_erase, "requested", sizeof(dtc_ext_log_erase) - 1);
  dtc_ext_log_erase[sizeof(dtc_ext_log_erase) - 1] = '\0';
  dtc_ext_state = DTC_EXT_ERASE_SESSION_SENT;
  dtc_ext_step_start_ms = millis();
}

// Drives the DTC Read/Erase probe: open an extended diagnostic session (0x10 0x03, same subfunction
// used successfully for the 0x9281 sequence - NOT the 0xC0 session the old, broken read_DTC() used to
// send), wait DTC_EXT_SESSION_GAP_MS (mirrors the NVROL sequence's own 100ms pacing between steps, not
// a measured requirement of this BMS), then send the actual service. The raw reply (or its absence) is
// logged verbatim via handle_dtc_ext_reply() / the timeout branch below - nothing about the response is
// assumed or parsed into a DTC list, since none has ever been seen on this battery.
void RenaultTwingoGen1Battery::handle_dtc_ext(unsigned long currentMillis) {
  switch (dtc_ext_state) {
    case DTC_EXT_IDLE:
      break;

    case DTC_EXT_READ_SESSION_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_SESSION_GAP_MS) {
        // Gap elapsed: send ReadDTCInformation, status mask (0x09 default, or 0xFF if the "all statuses"
        // checkbox was ticked before pressing Read DTC).
        ZOE_POLL_18DADBF1.data = {0x03, 0x19, 0x02, datalayer_extended.twingoGen1.dtc_ext_read_mask, 0xAA,
                                   0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        dtc_ext_step_start_ms = currentMillis;
        dtc_ext_state = DTC_EXT_READ_CMD_SENT;
      }
      break;
    case DTC_EXT_READ_CMD_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_REPLY_TIMEOUT_MS) {
        strncpy(dtc_ext_log_read, "no response", sizeof(dtc_ext_log_read) - 1);
        dtc_ext_log_read[sizeof(dtc_ext_log_read) - 1] = '\0';
        ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
        dtc_ext_state = DTC_EXT_IDLE;
      }
      break;

    case DTC_EXT_ERASE_SESSION_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_SESSION_GAP_MS) {
        // ClearDiagnosticInformation, group 0xFFFFFF (all groups) - same group Zoe Gen2's existing,
        // upstream Erase DTC button uses on this same protocol.
        ZOE_POLL_18DADBF1.data = {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        dtc_ext_step_start_ms = currentMillis;
        dtc_ext_state = DTC_EXT_ERASE_CMD_SENT;
      }
      break;
    case DTC_EXT_ERASE_CMD_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_REPLY_TIMEOUT_MS) {
        strncpy(dtc_ext_log_erase, "no response", sizeof(dtc_ext_log_erase) - 1);
        dtc_ext_log_erase[sizeof(dtc_ext_log_erase) - 1] = '\0';
        ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
        dtc_ext_state = DTC_EXT_IDLE;
      }
      break;

    case DTC_EXT_DETAILS_SESSION_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_SESSION_GAP_MS) {
        // ReadDTCInformation, subfunction 0x06 (reportDTCExtDataRecordByDTCNumber), record 0xFF (all
        // records) for DTC_DETAILS_CODES[dtc_ext_details_index].
        uint32_t code = DTC_DETAILS_CODES[dtc_ext_details_index];
        ZOE_POLL_18DADBF1.data = {0x06, 0x19, 0x06, (uint8_t)(code >> 16), (uint8_t)(code >> 8),
                                   (uint8_t)code, 0xFF, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        dtc_ext_step_start_ms = currentMillis;
        dtc_ext_state = DTC_EXT_DETAILS_CMD_SENT;
      }
      break;
    case DTC_EXT_DETAILS_CMD_SENT:
      if (currentMillis - dtc_ext_step_start_ms >= DTC_EXT_REPLY_TIMEOUT_MS) {
        strncpy(dtc_ext_log_details[dtc_ext_details_index], "no response",
                sizeof(dtc_ext_log_details[dtc_ext_details_index]) - 1);
        dtc_ext_log_details[dtc_ext_details_index][sizeof(dtc_ext_log_details[dtc_ext_details_index]) - 1] = '\0';
        dtc_ext_details_index++;
        if (dtc_ext_details_index < DTC_DETAILS_COUNT) {
          dtc_ext_step_start_ms = currentMillis;
          dtc_ext_state = DTC_EXT_DETAILS_SESSION_SENT;  // next DTC
        } else {
          ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
          dtc_ext_state = DTC_EXT_IDLE;
        }
      }
      break;
  }
}

// Called once a positive ReadDTCInformation reply has been fully received (single- or multi-frame,
// see the two call sites above) - data/len already point past SID+subfunction+mask, i.e. straight at
// the repeated 4-byte DTC entries (3-byte code + 1-byte status), same layout UdsCanBattery's own
// (private, so not reusable from here) handle_dtc_response() expects for the non-KWP2000 case. Feeds
// the SAME `dtc` structure the existing "Diagnostic Trouble Codes" table on /advanced already renders
// (DTC/Status columns; Description stays "Unknown" - no renault_zoe_gen1_dtc.json exists, see 01.10.
// research) - this is the first time that table gets real data instead of "Not read yet".
void RenaultTwingoGen1Battery::handle_dtc_read_response(const uint8_t* data, uint16_t len) {
  if (dtc != nullptr) {
    int count = len / 4;
    if (count > dtc->MAX_DTC_COUNT) {
      count = dtc->MAX_DTC_COUNT;
    }
    if (count < 0) {
      count = 0;
    }
    for (int i = 0; i < count; i++) {
      uint16_t offset = (uint16_t)(i * 4);
      if ((uint16_t)(offset + 3) >= len) {
        break;
      }
      dtc->dtc_codes[i] = ((uint32_t)data[offset] << 16) | ((uint32_t)data[offset + 1] << 8) |
                          (uint32_t)data[offset + 2];
      dtc->dtc_status[i] = data[offset + 3];
    }
    dtc->dtc_count = (uint8_t)count;
    dtc->dtc_reported_count = (uint16_t)(len / 4);  // may exceed dtc_count if truncated at MAX_DTC_COUNT
    dtc->dtc_read_failed = false;
    dtc->dtc_last_read_millis = millis();
  }
  snprintf(dtc_ext_log_read, sizeof(dtc_ext_log_read), "OK, %u DTC(s), %u bytes raw", (unsigned)(len / 4),
           (unsigned)len);
  ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
  dtc_ext_state = DTC_EXT_IDLE;
}

// Logs whatever came back, verbatim - same format as handle_nvrol_reply() (OK raw=.. / NEGATIVE SID=../
// short reply / unexpected multi-frame), reused here since the two state machines never run at once.
// Only used for Erase replies now (always tiny: "54" positive / "7F 14 NRC" negative, both single-frame
// - Read replies are handled by handle_dtc_read_response() above via the generic reassembly path).
void RenaultTwingoGen1Battery::handle_dtc_ext_reply(CAN_frame rx_frame) {
  char* log = dtc_ext_log_erase;  // only Erase routes here now, see the dispatch comment above
  uint8_t pci = rx_frame.data.u8[0];
  if (pci >= 0x10) {
    snprintf(log, 48, "unexpected multi-frame (PCI=0x%02X)", pci);
  } else if (pci < 3) {
    snprintf(log, 48, "short reply (%u bytes)", pci);
  } else if (rx_frame.data.u8[1] == 0x7F) {
    snprintf(log, 48, "NEGATIVE SID=0x%02X NRC=0x%02X", rx_frame.data.u8[2], rx_frame.data.u8[3]);
  } else {
    snprintf(log, 48, "OK raw=%02X %02X %02X %02X %02X %02X %02X", rx_frame.data.u8[1], rx_frame.data.u8[2],
             rx_frame.data.u8[3], rx_frame.data.u8[4], rx_frame.data.u8[5], rx_frame.data.u8[6],
             rx_frame.data.u8[7]);
  }
  ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // restore poll template
  dtc_ext_state = DTC_EXT_IDLE;  // reply seen - no need to wait out the timeout
}
