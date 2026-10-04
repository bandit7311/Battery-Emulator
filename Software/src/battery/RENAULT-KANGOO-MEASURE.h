#ifndef RENAULT_KANGOO_MEASURE_H
#define RENAULT_KANGOO_MEASURE_H

#include <stddef.h>
#include <stdint.h>

// "Measurement list" for the Renault Kangoo driver running in RX only mode.
//
// One press of the button on the advanced page sends 15 UDS read requests (service 0x22), one after the
// other, and counts how many of them got a positive answer. Nothing is sent without a button press, so the
// "pure listener" behaviour of RX only is unchanged for everything else.
//
// All answers of this list fit into one CAN frame (at most 4 data bytes, 7 bytes with service and
// identifier), so no ISO-TP flow control is needed. A multi-frame answer would simply not match and count as "no answer".
//
// This class contains only logic (no Arduino / CAN driver dependencies). The driver calls poll() regularly
// from transmit_can() and on_frame() for every received frame.
class KangooMeasureList {
 public:
  static constexpr uint8_t COUNT = 15;
  static constexpr unsigned long REPLY_TIMEOUT_MS = 500;  // per request, then it counts as "no answer"
  static constexpr unsigned long GAP_MS = 50;             // pause between two requests

  enum State : uint8_t { IDLE = 0, RUNNING = 1, DONE = 2 };

  // Result of one request of the last run (shown on the advanced page).
  enum AnswerStatus : uint8_t { NOT_ASKED = 0, ANSWERED = 1, NEGATIVE = 2, NO_ANSWER = 3 };
  struct Answer {
    uint8_t status = NOT_ASKED;
    uint8_t len = 0;  // number of data bytes of a positive answer (1 to 4)
    uint8_t nrc = 0;  // negative response code
    uint32_t raw = 0;  // data bytes, big endian
  };
  static constexpr uint8_t MAX_DATA = 4;

  struct Request {
    uint32_t id;
    bool ext;
    uint8_t data[8];
  };

  // Called by the button. A second press while a run is active is ignored.
  void request_start() { start_requested = true; }

  // True while a run is active or about to start (the time button must not send in the middle of it).
  bool busy() const { return state == RUNNING || start_requested; }

  // Returns true when a request has to be transmitted now (filled into out).
  bool poll(unsigned long now, Request& out) {
    if (start_requested) {
      start_requested = false;
      if (state != RUNNING) {
        state = RUNNING;
        index = 0;
        ok_count = 0;
        waiting = false;
        gap_pending = true;
        for (uint8_t i = 0; i < COUNT; i++) {
          answers[i] = Answer();
        }
      }
    }
    if (state != RUNNING) {
      return false;
    }
    if (waiting && (unsigned long)(now - sent_ms) >= REPLY_TIMEOUT_MS) {
      waiting = false;  // no answer in time
      answers[index].status = NO_ANSWER;
      index++;
      gap_pending = true;
    }
    if (gap_pending) {  // same pause after an answer, after a timeout and before the first request
      gap_pending = false;
      next_ms = now + GAP_MS;
    }
    if (index >= COUNT) {
      state = DONE;
      return false;
    }
    if (waiting) {
      return false;
    }
    if ((long)(now - next_ms) < 0) {
      return false;
    }
    const Query& q = query(index);
    out.id = q.req_id;
    out.ext = q.ext;
    out.data[0] = 0x03;  // single frame, 3 bytes: service and 2 byte identifier
    out.data[1] = 0x22;
    out.data[2] = q.did_hi;
    out.data[3] = q.did_lo;
    out.data[4] = 0xFF;  // padding like the diagnostic tester in the vehicle log
    out.data[5] = 0xFF;
    out.data[6] = 0xFF;
    out.data[7] = 0xFF;
    waiting = true;
    sent_ms = now;
    return true;
  }

  // Called for every received frame. Only the answer to the outstanding request is evaluated.
  void on_frame(uint32_t id, bool ext, const uint8_t* data, uint8_t dlc) {
    if (state != RUNNING || !waiting || index >= COUNT) {
      return;
    }
    const Query& q = query(index);
    if (id != q.resp_id || ext != q.ext || dlc < 4) {
      return;
    }
    const uint8_t len = data[0];
    if (len >= 3 && len <= 7 && data[1] == 0x62 && data[2] == q.did_hi && data[3] == q.did_lo) {
      ok_count++;  // positive answer: 0x62 + the requested identifier
      Answer& a = answers[index];
      a.status = ANSWERED;
      uint8_t n_data = (uint8_t)(len - 3);
      if (n_data > 4) {
        n_data = 4;  // MAX_DATA
      }
      a.len = n_data;
      a.raw = 0;
      for (uint8_t k = 0; k < a.len && (4 + k) < dlc; k++) {
        a.raw = (a.raw << 8) | data[4 + k];
      }
    } else if (len == 3 && data[1] == 0x7F && data[2] == 0x22) {
      // negative answer: counts as "no valid answer"
      answers[index].status = NEGATIVE;
      answers[index].nrc = (dlc > 3) ? data[3] : 0;
    } else {
      return;  // some other frame from the same ECU, keep waiting
    }
    waiting = false;
    index++;
    gap_pending = true;
  }

  State get_state() const { return state; }
  uint8_t get_ok() const { return ok_count; }
  const Answer& answer(uint8_t i) const { return answers[i < COUNT ? i : 0]; }

  // Description of a request for the page: label, unit and conversion raw -> value.
  static const char* label(uint8_t i) { return query(i < COUNT ? i : 0).label; }
  static const char* unit(uint8_t i) { return query(i < COUNT ? i : 0).unit; }
  static bool is_minutes(uint8_t i) { return query(i < COUNT ? i : 0).kind == KIND_MINUTES; }
  static bool is_plain(uint8_t i) { return query(i < COUNT ? i : 0).kind == KIND_RAW; }
  static double decode(uint8_t i, uint32_t raw) {
    const Query& q = query(i < COUNT ? i : 0);
    return (double)raw * q.scale + q.offset;
  }
  static uint8_t did_hi(uint8_t i) { return query(i < COUNT ? i : 0).did_hi; }
  static uint8_t did_lo(uint8_t i) { return query(i < COUNT ? i : 0).did_lo; }

 private:
  enum Kind : uint8_t { KIND_VALUE = 0, KIND_RAW = 1, KIND_MINUTES = 2 };

  struct Query {
    uint32_t req_id;
    bool ext;
    uint32_t resp_id;
    uint8_t did_hi;
    uint8_t did_lo;
    const char* label;
    const char* unit;
    double scale;   // value = raw * scale + offset
    double offset;
    Kind kind;
  };

  // The 15 requests of the vehicle measurement list (03.10.: extended from 8 by 7), with the conversion that the
  // page uses. No percent sign in any text: the web server would take two of them for a template placeholder.
  static const Query& query(uint8_t i) {
    static const Query table[COUNT] = {
        {0x18DADBF1, true, 0x18DAF1DB, 0x90, 0x05, "LBC pack voltage (CAN value) $9005", "V", 0.1, 0, KIND_VALUE},
        {0x18DADBF1, true, 0x18DAF1DB, 0x90, 0x06, "LBC sum of all cell voltages $9006", "V", 0.0009765625, 0, KIND_VALUE},
        {0x18DADBF1, true, 0x18DAF1DB, 0x90, 0x12, "LBC average temperature $9012", "&deg;C", 0.0625, -40.0, KIND_VALUE},
        {0x18DADBF1, true, 0x18DAF1DB, 0x92, 0x61, "LBC absolute time of vehicle, saved $9261", "min", 1, 0, KIND_MINUTES},
        {0x18DADBF1, true, 0x18DAF1DB, 0x92, 0x64, "LBC total boost time from HEVC, saved $9264", "", 1, 0, KIND_RAW},
        {0x18DADBF1, true, 0x18DAF1DB, 0x92, 0x6B, "LBC abstime at transition start $926B", "", 1, 0, KIND_RAW},
        {0x18DADBF1, true, 0x18DAF1DB, 0x91, 0xC1, "LBC pack time life $91C1", "min", 1, 0, KIND_MINUTES},
        {0x18DADBF1, true, 0x18DAF1DB, 0x92, 0x5F, "LBC vehicle distance totalizer $925F", "km", 0.01, 0, KIND_VALUE},
        {0x7E4, false, 0x7EC, 0x32, 0x03, "EVC HV LBC voltage measure $3203", "V", 0.5, 0, KIND_VALUE},
        {0x7E4, false, 0x7EC, 0x35, 0x67, "EVC LBC frame status received $3567", "", 1, 0, KIND_RAW},
        {0x75A, false, 0x77E, 0x20, 0x04, "PEB battery voltage sensor $2004", "V", 0.03125, 0, KIND_VALUE},
        {0x75A, false, 0x77E, 0x70, 0xD7, "PEB HV network voltage from battery $70D7", "V", 0.5, 0, KIND_VALUE},
        {0x75A, false, 0x77E, 0x70, 0x16, "PEB result of the CAN check with the EVC $7016", "", 1, 0, KIND_RAW},
        {0x75A, false, 0x77E, 0x20, 0x03, "PEB torque setpoint from the EVC $2003", "Nm", 0.1, -250.0, KIND_VALUE},
        {0x75A, false, 0x77E, 0x20, 0x02, "PEB torque estimate from the current $2002", "Nm", 0.015625, -254.0, KIND_VALUE},
    };
    return table[i];
  }

  State state = IDLE;
  bool start_requested = false;
  bool waiting = false;
  bool gap_pending = false;
  uint8_t index = 0;
  uint8_t ok_count = 0;
  Answer answers[COUNT];
  unsigned long sent_ms = 0;
  unsigned long next_ms = 0;
};

#endif
