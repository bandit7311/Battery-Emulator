#include "simulator_html.h"
#include <Arduino.h>
#include "../../battery/RENAULT-TWINGO-GEN1-BATTERY.h"
#include "../../datalayer/datalayer_extended.h"

// Groups rows by interval_ms for display, in signal-table order within each group - no sorting needed
// since the table itself is already laid out 10/20/100/1000ms-grouped (see the header comment there).
static void append_signal_row(String& content, uint8_t i) {
  const RenaultTwingoGen1Battery::SimSignal& s = RenaultTwingoGen1Battery::sim_signals[i];
  bool checked = (datalayer_extended.twingoGen1.simulator_enabled_mask & (1UL << i)) != 0;

  content += "<tr><td><input type='checkbox' id='sim" + String(i) + "' " + (checked ? "checked " : "") +
             "onchange=\"fetch('/editTwingoSimSignal?index=" + String(i) +
             "&value='+(this.checked?1:0))\"></td>";

  content += "<td>0x" + String(s.id, HEX) + "</td>";

  const char* tagClass = (s.tag == 'I') ? "tag-i" : (s.tag == 'P') ? "tag-p" : "tag-a";
  content += "<td><span class='" + String(tagClass) + "'>" + String(s.tag) + "</span></td>";

  content += "<td>" + String(s.interval_ms) + " ms</td>";

  content += "<td>" + String(s.label);
  if (s.bms_origin) {
    content +=
        " <span class='note'>(normally sent by the BMS itself, not the EVC - no technical lock, just a "
        "note)</span>";
  }
  if (s.tag == 'I') {
    content += " <span class='note'>(already sent for real elsewhere in this driver)</span>";
  }
  content += "</td></tr>";
}

String simulator_processor(const String& var) {
  if (var == "X") {
    String content = "";
    content += "<style>";
    content += "body { background-color: black; color: white; font-family: sans-serif; }";
    content += "table { border-collapse: collapse; width: 100%; max-width: 820px; }";
    content += "td, th { border: 1px solid #444; padding: 4px 8px; text-align: left; }";
    content += "th { text-align: center; }";
    content += ".tag-i { color: #6fcf6f; font-weight: bold; }";
    content += ".tag-p { color: #ffd479; font-weight: bold; }";
    content += ".tag-a { color: #ff9b9b; font-weight: bold; }";
    content += ".note { color: #999; font-size: 0.85em; }";
    content += "h3 { margin-top: 24px; margin-bottom: 6px; }";
    content += "</style>";

    content += "<h2>CAN Signal Simulator</h2>";
    content +=
        "<p>28 cyclic signals a real vehicle sends on this bus. Each checkbox is independent. Content for "
        "every row below comes from a real capture (Log_Twingo_Ladung.log, 02.10.), not invented.</p>";
    content +=
        "<p><b>Legend:</b> <span class='tag-i'>I</span> = Installed, already sent for real by this "
        "driver &nbsp; <span class='tag-p'>P</span> = Planned, content/meaning from the real log, not yet "
        "sent &nbsp; <span class='tag-a'>A</span> = Assumed, content from the real log but meaning "
        "unconfirmed</p>";

    content += "<details style='margin-bottom:16px;'><summary style='cursor:pointer;color:#8fd3ff;'>Full list with startup state and sender</summary>";
    content += "<div style='padding:8px 0;'>";

    content += "<p><b>10 ms:</b><br>"
               "&#9745; 0x090 (Twingo-Fast, counter+CRC)<br>"
               "&#9744; 0x1F8 (EVC, heartbeat + relay byte)<br>"
               "&#9744; 0x18A (EVC, rolling counter byte 7)<br>"
               "&#9744; 0x17A, 0x17E, 0x186, 0x1F6 (BMS&rarr;EVC, burst packet, content unknown)</p>";

    content += "<p><b>20 ms:</b><br>"
               "&#9745; 0x242 (Twingo-Fast, counter+CRC)<br>"
               "&#9745; 0x214 (shutdown only)<br>"
               "&#9744; 0x211 (EVC, Klemme15/driving)<br>"
               "&#9744; 0x1B0 (EVC, inter-ECU sync, content known: FF 2C FF C0)<br>"
               "&#9744; 0x217 (BMS, charge/discharge limits, content known from log)</p>";

    content += "<p><b>100 ms:</b><br>"
               "&#9745; 0x350 (vehicle state, C0)<br>"
               "&#9745; 0x19F, 0x426, 0x436, 0x423 (upstream PR #2907)<br>"
               "&#9744; 0x5DE, 0x5DF, 0x634 (thermal management)<br>"
               "&#9744; 0x427, 0x42E, 0x432, 0x650, 0x1FD (SOC/SOH/odometer/energy - measured at 100ms "
               "in the real log, not 1000ms as originally assumed)<br>"
               "&#9744; 0x55D (EVC, assumed - see its own row for the experimental drive-mode variant)</p>";

    content += "<p><b>1000 ms:</b><br>"
               "&#9745; 0x69F (upstream PR #2907)<br>"
               "&#9745; 0x53B (time frame)</p>";

    content += "<p>&#9745; = on by default (already sent for real today) &nbsp; &#9744; = off by default "
               "(new, needs a deliberate click)</p>";
    content += "</div></details>";

    content += "<table><thead><tr><th>On</th><th>ID</th><th>Tag</th><th>Interval</th><th>Signal</th></tr></thead><tbody>";

    // Display grouped by interval (10/20/100/1000ms), even though the underlying table/NVM bit order is
    // not sorted that way - a simple stable selection sort over interval_ms, operating on original
    // indices only, so checkbox IDs/bit positions (fetch('/editTwingoSimSignal?index=N...')) stay the
    // fixed meaning they have everywhere else (sim_signals[N], bit N of simulator_enabled_mask).
    uint8_t order[RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT];
    for (uint8_t i = 0; i < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; i++) {
      order[i] = i;
    }
    for (uint8_t a = 0; a < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT - 1; a++) {
      for (uint8_t b = 0; b < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT - 1 - a; b++) {
        if (RenaultTwingoGen1Battery::sim_signals[order[b]].interval_ms >
            RenaultTwingoGen1Battery::sim_signals[order[b + 1]].interval_ms) {
          uint8_t t = order[b];
          order[b] = order[b + 1];
          order[b + 1] = t;
        }
      }
    }

    uint16_t lastInterval = 0;
    for (uint8_t k = 0; k < RenaultTwingoGen1Battery::SIM_SIGNAL_COUNT; k++) {
      uint8_t i = order[k];
      uint16_t iv = RenaultTwingoGen1Battery::sim_signals[i].interval_ms;
      if (iv != lastInterval) {
        content += "</tbody></table><h3>" + String(iv) + " ms</h3><table><tbody>";
        lastInterval = iv;
      }
      append_signal_row(content, i);
    }
    content += "</tbody></table>";

    bool driveMode = datalayer_extended.twingoGen1.sim_55d_drive_mode_enabled;
    content += "<h3 style='color:#ff9b9b;'>EXPERIMENTAL - unverified</h3>";
    content +=
        "<p>Only affects the 0x55D row above, and only while its own checkbox is also on. Content is NOT "
        "from a real log capture - driving was never recorded. Use at your own risk.</p>";
    content += "<label><input type='checkbox' id='sim55dDrive' " + String(driveMode ? "checked " : "") +
               "onchange=\"fetch('/editTwingoSim55dDriveMode?value='+(this.checked?1:0))\"> "
               "Send 0x55D as 'drive/discharge active' instead of the normal content</label>";
    content += "<table><tbody>";
    content += "<tr><td>Data</td><td colspan='2'>05 FD F0 01 00 00 00 81</td></tr>";
    content += "<tr><td>Byte 0</td><td>0x05</td><td>EVC_State = Drive Active (claimed)</td></tr>";
    content += "<tr><td>Byte 1</td><td>0xFD</td><td>HV_Enable (claimed)</td></tr>";
    content += "<tr><td>Byte 2</td><td>0xF0</td><td>Contactor_Cmd_Inverter, precharge done (claimed)</td></tr>";
    content += "<tr><td>Byte 3</td><td>0x01</td><td>Discharge_Enable (claimed)</td></tr>";
    content += "<tr><td>Byte 4-6</td><td>00 00 00</td><td>Reserve (claimed)</td></tr>";
    content += "<tr><td>Byte 7</td><td>0x81</td><td>Interlock_OK (claimed)</td></tr>";
    content += "</tbody></table>";

    bool restActive = datalayer_extended.twingoGen1.sim_55d_rest_active_enabled;
    content +=
        "<h3 style='color:#ff9b9b;'>EXPERIMENTAL - rest state &amp; staged relay closing</h3>"
        "<p>Off: 0x55D continuously sends a GUESSED \"rest/idle\" content (byte 0 = 0x01). Ticking this "
        "box starts a staged sequence - byte 0 steps 0x02 (\"Precharge\", GUESSED) &rarr; 0x04 (\"Main "
        "relay closing\", GUESSED) &rarr; settles on the normal/drive content above (REAL), 200ms per "
        "stage (GUESSED timing, no real log data). Unticking reverts to rest immediately. Only works "
        "while the 0x55D row checkbox is also on.</p>";
    content += "<label><input type='checkbox' id='sim55dRestActive' " + String(restActive ? "checked " : "") +
               "onchange=\"fetch('/editTwingoSim55dRestActive?value='+(this.checked?1:0))\"> "
               "Active (precharge &amp; close main relay)</label>";

    content += "<p><a href='/advanced' style='color:#8fd3ff;'>Back to More Battery Info</a></p>";
    return content;
  }
  return String();
}
