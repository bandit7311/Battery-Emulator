#ifndef CAN_LOG_FILTER_H
#define CAN_LOG_FILTER_H

#include <stddef.h>
#include <stdint.h>

// Filter for the CAN log that is printed on the USB serial port ("CAN message logging via USB serial").
//
// The USB log has no flow control: when the bus is faster than the serial line can print (about 1700 frames per
// second), frames are dropped and only a "[N CAN frames not printed]" marker remains. With this filter the user
// selects the IDs that are really needed, so that nothing is lost.
//
// Text syntax, set on the Settings page. Entries are separated by comma, space or semicolon. Hex, upper or lower
// case, an optional 0x in front:
//   350        one ID
//   700-7FF    a range of IDs (both ends included)
//   ext        every 29 bit frame
//   std        every 11 bit frame
//   !          as first character: inverted, print everything EXCEPT the listed IDs
// Empty text (or no valid entry): no filter, every frame is printed, as before.
//
// Header only and without Arduino dependencies, so it can be tested on the host.
class CanLogFilter {
 public:
  static constexpr uint8_t MAX_ENTRIES = 40;
  static constexpr size_t MAX_TEXT = 128;

  // Reads the text. Returns the number of valid entries. Unknown entries are ignored (see ignored()).
  // Call from one task only and while the log is not running (it is called at boot).
  uint8_t parse(const char* text) {
    count = 0;
    ignored_count = 0;
    ext_all = false;
    std_all = false;
    invert = false;
    if (text == nullptr) {
      return 0;
    }
    size_t pos = 0;
    while (text[pos] == ' ' || text[pos] == '\t') {
      pos++;
    }
    if (text[pos] == '!') {
      invert = true;
      pos++;
    }
    size_t end = pos;
    while (end < MAX_TEXT && text[end] != '\0') {
      end++;
    }
    while (pos < end) {
      while (pos < end && is_separator(text[pos])) {
        pos++;
      }
      size_t start = pos;
      while (pos < end && !is_separator(text[pos])) {
        pos++;
      }
      if (pos > start) {
        take_token(text + start, pos - start);
      }
    }
    if (count == 0 && !ext_all && !std_all) {
      invert = false;  // nothing valid: no filter at all
    }
    return (uint8_t)(count + (ext_all ? 1 : 0) + (std_all ? 1 : 0));
  }

  // True when a filter is set. False means: every frame is printed.
  bool active() const { return count > 0 || ext_all || std_all; }

  // Number of entries in the text that were not understood (for a log line at boot).
  uint8_t ignored() const { return ignored_count; }

  // Decides whether a frame is printed.
  bool matches(uint32_t id, bool ext) const {
    if (!active()) {
      return true;
    }
    bool hit = (ext_all && ext) || (std_all && !ext);
    for (uint8_t i = 0; !hit && i < count; i++) {
      hit = (id >= lo[i] && id <= hi[i]);
    }
    return invert ? !hit : hit;
  }

 private:
  static bool is_separator(char c) { return c == ',' || c == ' ' || c == ';' || c == '\t' || c == '\r' || c == '\n'; }

  static int hex_value(char c) {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
      return c - 'A' + 10;
    }
    return -1;
  }

  static bool equals_ignore_case(const char* s, size_t len, const char* word) {
    size_t i = 0;
    for (; i < len && word[i] != '\0'; i++) {
      char c = s[i];
      if (c >= 'A' && c <= 'Z') {
        c = (char)(c - 'A' + 'a');
      }
      if (c != word[i]) {
        return false;
      }
    }
    return i == len && word[i] == '\0';
  }

  // Reads a hex number from s[0..len). Returns false for an empty text, a character that is not hex, or more than
  // 8 digits (a CAN ID has at most 29 bits).
  static bool read_hex(const char* s, size_t len, uint32_t& out) {
    if (len >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
      s += 2;
      len -= 2;
    }
    if (len == 0 || len > 8) {
      return false;
    }
    uint32_t v = 0;
    for (size_t i = 0; i < len; i++) {
      int h = hex_value(s[i]);
      if (h < 0) {
        return false;
      }
      v = (v << 4) | (uint32_t)h;
    }
    out = v;
    return true;
  }

  void take_token(const char* s, size_t len) {
    if (equals_ignore_case(s, len, "ext")) {
      ext_all = true;
      return;
    }
    if (equals_ignore_case(s, len, "std")) {
      std_all = true;
      return;
    }
    uint32_t a = 0, b = 0;
    size_t dash = len;
    for (size_t i = 0; i < len; i++) {
      if (s[i] == '-') {
        dash = i;
        break;
      }
    }
    bool ok;
    if (dash == len) {
      ok = read_hex(s, len, a);
      b = a;
    } else {
      ok = read_hex(s, dash, a) && read_hex(s + dash + 1, len - dash - 1, b);
      if (ok && a > b) {
        uint32_t t = a;
        a = b;
        b = t;
      }
    }
    if (!ok || count >= MAX_ENTRIES) {
      ignored_count++;
      return;
    }
    lo[count] = a;
    hi[count] = b;
    count++;
  }

  uint32_t lo[MAX_ENTRIES] = {};
  uint32_t hi[MAX_ENTRIES] = {};
  uint8_t count = 0;
  uint8_t ignored_count = 0;
  bool ext_all = false;
  bool std_all = false;
  bool invert = false;
};

#endif
