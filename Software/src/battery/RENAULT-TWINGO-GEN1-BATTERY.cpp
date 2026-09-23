#include "RENAULT-TWINGO-GEN1-BATTERY.h"
#include "../datalayer/datalayer.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"

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
    if (datalayer_battery->status.cell_voltages_mV[cell_index] == 0 && ext_cells_seen < 96) {
      ext_cells_seen++;
    }
    datalayer_battery->status.cell_voltages_mV[cell_index] = (uint16_t)(((data[0] << 8) | data[1]) * 0.976563f);
    return;
  }

  switch (pid) {
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
        battery_temporisation = data[0] >> 7;
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
  }
}

// Sends the two documented write sequences (NVROL reset, then enable
// "temporisation before sleep") over the same extended channel used for
// polling - see the comment block above UserRequestNVROLReset in the header
// for what this is, what it's based on, and what's deliberately left out.
// Timing (100ms/1s/100ms gaps between steps) matches Battery-Emulator's own
// proven Zoe Ph2 sequence exactly; only the final 30s software-sleep step is
// omitted on purpose.
void RenaultTwingoGen1Battery::transmit_reset_nvrol_frames(void) {
  switch (NVROLstateMachine) {
    case 0:
      startTimeNVROL = millis();
      for (uint8_t i = 0; i < NVROL_LOG_STEPS; i++) {
        strncpy(nvrol_log[i], "no response", sizeof(nvrol_log[i]) - 1);
        nvrol_log[i][sizeof(nvrol_log[i]) - 1] = '\0';
      }
      nvrol_awaiting_step = 0;
      // NVROL reset, part 1: open extended diagnostic session (SID 0x10, subfunction 0x03)
      ZOE_POLL_18DADBF1.data = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
      transmit_can_frame(&ZOE_POLL_18DADBF1);
#ifdef EXTENDED_UDS_DEBUG
      logging.println("NVROL: step 0 - open session");
#endif
      NVROLstateMachine = 1;
      break;
    case 1:  // wait 100ms for step 0's response
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS) {
        // NVROL reset, part 2: RoutineControl (SID 0x31) start routine (0x01) 0xB009
        ZOE_POLL_18DADBF1.data = {0x04, 0x31, 0x01, 0xB0, 0x09, 0x00, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 1;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 1 - start routine B009 (NVROL reset)");
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
        // Enable temporisation before sleep, part 2: WriteDataByIdentifier (SID 0x2E) PID 0x9281 = 1
        ZOE_POLL_18DADBF1.data = {0x04, 0x2E, 0x92, 0x81, 0x01, 0xAA, 0xAA, 0xAA};
        transmit_can_frame(&ZOE_POLL_18DADBF1);
        nvrol_awaiting_step = 3;
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: step 3 - write temporisation=1");
#endif
        startTimeNVROL = millis();
        NVROLstateMachine = 4;
      }
      break;
    case 4:  // wait 100ms for step 3's response, then finish
      if ((millis() - startTimeNVROL) > INTERVAL_100_MS) {
#ifdef EXTENDED_UDS_DEBUG
        logging.println("NVROL: sequence complete");
#endif
        // Restore the poll frame to its normal read template - we're done with
        // the special sequence, normal extended polling resumes next cycle.
        ZOE_POLL_18DADBF1.data = {0x03, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        ext_poll_index = 0;
        UserRequestNVROLReset = false;
        NVROLstateMachine = 0;
        // No software sleep step here - see header comment. Letting the
        // battery actually sleep/save is done manually (12V/contactors) by
        // the user after this sequence finishes.
      }
      break;
    default:  // Something went wrong; reset state machine
      NVROLstateMachine = 0;
      UserRequestNVROLReset = false;
      break;
  }
}
#endif

void RenaultTwingoGen1Battery::handle_incoming_can_frame(CAN_frame rx_frame) {
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
      LB_Cell_minimum_temperature = (rx_frame.data.u8[4] - 40);
      LB_SOH = rx_frame.data.u8[5];
      LB_Cell_maximum_temperature = (rx_frame.data.u8[7] - 40);
      break;
    case 0x425:  //100ms Cellvoltages and kWh remaining - Confirmed sent by: Fluence ZE40 & Zoe Gen1
      LB_Cell_maximum_voltage = (((((rx_frame.data.u8[4] & 0x03) << 7) | (rx_frame.data.u8[5] >> 1)) * 10) + 1000);
      LB_Cell_minimum_voltage = (((((rx_frame.data.u8[6] & 0x01) << 8) | rx_frame.data.u8[7]) * 10) + 1000);
      break;
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
#ifdef TWINGO_EXTENDED_CELL_POLLING
    case 0x18DAF1DB:  // Extended UDS LBC reply (cell voltages / balancing / lifetime metrics)
      if (UserRequestNVROLReset) {
        // While the NVROL sequence is running, responses are Session
        // Control/RoutineControl/WriteDataByIdentifier replies, not
        // ReadDataByIdentifier ones - handle_extended_reply() would
        // misparse them (it assumes a PID sits at bytes 2-3).
        handle_nvrol_reply(rx_frame);
      } else {
        handle_extended_reply(rx_frame);
      }
      break;
#endif
    default:
      break;
  }
}

void RenaultTwingoGen1Battery::transmit_can(unsigned long currentMillis) {

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
  }

#ifdef TWINGO_EXTENDED_CELL_POLLING
  if (UserRequestNVROLReset) {
    // NVROL reset in progress: run its state machine instead of normal extended
    // polling, since both share the ZOE_POLL_18DADBF1 frame object below.
    transmit_reset_nvrol_frames();
  } else {
  // Extended-address polling: cycle through the 102 poll targets (96 cell
  // voltages + balancing + 5 lifetime metrics + temporisation), one every
  // 200ms (same cadence as Battery-Emulator's own Zoe Ph2 driver) -> ~20.4s
  // per full cycle.
  if (currentMillis - previousMillisExtPoll >= EXT_POLL_INTERVAL_MS) {
    previousMillisExtPoll = currentMillis;
    uint16_t pid = ext_poll_list[ext_poll_index];
    ZOE_POLL_18DADBF1.data.u8[2] = (uint8_t)((pid >> 8) & 0xFF);
    ZOE_POLL_18DADBF1.data.u8[3] = (uint8_t)(pid & 0xFF);
    transmit_can_frame(&ZOE_POLL_18DADBF1);
#ifdef EXTENDED_UDS_DEBUG
    logging.printf("EXT UDS TX: PID=0x%04X (index %u/%u)\n", pid, ext_poll_index, (unsigned)EXT_POLL_LIST_LENGTH);
#endif
    ext_poll_index = (ext_poll_index + 1) % EXT_POLL_LIST_LENGTH;
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
  content.reserve(400);

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
             "Lifetime Energy: " << kWh_from_beginning_of_battery_life << " kWh<br>"
#ifdef TWINGO_EXTENDED_CELL_POLLING
             "Charge Cycles: " << battery_charge_cycles << "<br>"
             "Energy Charged: " << battery_energy_charged_kWh << " kWh<br>"
             "Energy Discharged: " << battery_energy_discharged_kWh << " kWh<br>"
             "Energy Regenerated: " << battery_energy_regenerated_kWh << " kWh<br>"
             "Temporisation: " << (battery_temporisation == 255 ? "not yet read" : (battery_temporisation ? "1" : "0")) << "<br>"
             "NVROL Log - Session1: " << nvrol_log[0] << "<br>"
             "NVROL Log - Routine B009: " << nvrol_log[1] << "<br>"
             "NVROL Log - Session2: " << nvrol_log[2] << "<br>"
             "NVROL Log - Write 9281=1: " << nvrol_log[3] << "<br>"
#endif
             ;
  // clang-format on

  return content;
}

void RenaultTwingoGen1Battery::setup(void) {  // Performs one time setup at startup
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
}

static const uint16_t ZOE_STATE_OPEN_SESSION = 1;

void RenaultTwingoGen1Battery::read_DTC() {
  start_sequence(ZOE_STATE_OPEN_SESSION);
}

void RenaultTwingoGen1Battery::on_uds_sequence_step(uint16_t state, uint8_t sid, const uint8_t* data, uint16_t len) {
  if (state == ZOE_STATE_OPEN_SESSION) {
    send_sequence_message(ZOE_STATE_OPEN_SESSION + 10, SID::DiagnosticSessionControl, (const uint8_t*)"\xC0", 1, 20, 2);
  } else if (state == ZOE_STATE_OPEN_SESSION + 10 && sid == UDS_RESPONSE_SID_OF(SID::DiagnosticSessionControl)) {
    // Session 0xC0 granted! Transmit UDS ReadDTCInformation with status mask 0x09 (Active/Confirmed DTCs)
    send_sequence_message(UDS_STATE_READ_DTC, SID::ReadDTCInformation, (const uint8_t*)"\x02\x09", 2, 20, 2);
  }
}
