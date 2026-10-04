#ifndef RENAULT_KANGOO_QUICKQUERY_H
#define RENAULT_KANGOO_QUICKQUERY_H

#include <stdint.h>

// "Time" button for the Renault Kangoo driver in RX only mode: ONE read request 22 92 61 (absolute time of
// vehicle, saved) to the LBC, one frame out and one frame back, no diagnostic session (the vehicle log shows that
// the LBC answers 22 requests in the default session, 12 of 12 for this identifier in every phase).
//
// It is independent of the measurement list and can be pressed at any time, also while the vehicle is shutting
// down. The driver makes sure that it never sends while the measurement list is running (and the other way round).
// If the answer does not come within 300 ms the request is repeated once (in the vehicle log the first request
// after the bus had gone silent got no answer, the next one did), then the result is "no answer".
//
// Only logic, no Arduino / CAN driver dependencies: the driver calls poll() from transmit_can() and on_frame() for
// every received frame.
class KangooQuickQuery {
 public:
  static constexpr unsigned long REPLY_TIMEOUT_MS = 300;  // per attempt
  static constexpr uint8_t MAX_ATTEMPTS = 2;               // first request plus one repeat

  enum State : uint8_t { IDLE = 0, WAITING = 1, DONE = 2 };
  enum Result : uint8_t { NONE = 0, OK = 1, NEGATIVE = 2, NO_ANSWER = 3 };

  struct Request {
    uint32_t id;
    bool ext;
    uint8_t data[8];
  };

  // Called by the button. A press while a query is active is ignored.
  void request_start() {
    if (!busy()) {
      start_requested = true;
    }
  }

  // True while a query is active or about to start.
  bool busy() const { return state == WAITING || start_requested; }

  // Returns true when a request has to be transmitted now (filled into out).
  bool poll(unsigned long now, Request& out) {
    if (start_requested) {
      start_requested = false;
      state = WAITING;
      result = NONE;
      attempts = 0;
      raw = 0;
      len = 0;
      nrc = 0;
      send_pending = true;
    } else if (state == WAITING && !send_pending && (unsigned long)(now - sent_ms) >= REPLY_TIMEOUT_MS) {
      if (attempts < MAX_ATTEMPTS) {
        send_pending = true;  // repeat once
      } else {
        state = DONE;
        result = NO_ANSWER;
      }
    }
    if (!send_pending) {
      return false;
    }
    send_pending = false;
    out.id = 0x18DADBF1;
    out.ext = true;
    out.data[0] = 0x03;  // single frame, 3 bytes: service and 2 byte identifier
    out.data[1] = 0x22;
    out.data[2] = 0x92;
    out.data[3] = 0x61;
    out.data[4] = 0xFF;  // padding like the diagnostic tester in the vehicle log
    out.data[5] = 0xFF;
    out.data[6] = 0xFF;
    out.data[7] = 0xFF;
    attempts++;
    sent_ms = now;
    return true;
  }

  // Called for every received frame. Only the answer to the outstanding request is evaluated.
  void on_frame(uint32_t id, bool ext, const uint8_t* data, uint8_t dlc) {
    if (state != WAITING || send_pending || id != 0x18DAF1DB || !ext || dlc < 4) {
      return;
    }
    const uint8_t l = data[0];
    if (l >= 4 && l <= 7 && data[1] == 0x62 && data[2] == 0x92 && data[3] == 0x61) {
      uint8_t n = (uint8_t)(l - 3);
      if (n > 4) {
        n = 4;
      }
      len = n;
      raw = 0;
      for (uint8_t k = 0; k < n && (4 + k) < dlc; k++) {
        raw = (raw << 8) | data[4 + k];
      }
      result = OK;
      state = DONE;
    } else if (l == 3 && data[1] == 0x7F && data[2] == 0x22) {
      nrc = (dlc > 3) ? data[3] : 0;
      result = NEGATIVE;
      state = DONE;
    }
    // any other frame from the LBC: keep waiting
  }

  State get_state() const { return state; }
  Result get_result() const { return result; }
  uint8_t get_attempts() const { return attempts; }
  uint8_t get_len() const { return len; }
  uint32_t get_raw() const { return raw; }
  uint8_t get_nrc() const { return nrc; }

 private:
  State state = IDLE;
  Result result = NONE;
  bool start_requested = false;
  bool send_pending = false;
  uint8_t attempts = 0;
  uint8_t len = 0;
  uint8_t nrc = 0;
  uint32_t raw = 0;
  unsigned long sent_ms = 0;
};

#endif
