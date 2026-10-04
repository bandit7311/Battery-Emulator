#include "kangoo_live_html.h"

// Note: this text is sent as it is, but it must not contain a percent sign anyway (see the other pages).
const char kangoo_live_page[] = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Kangoo live</title>
<style>
body { background: #000; color: #fff; font-family: sans-serif; margin: 0; padding: 10px; }
.box { background: #303E47; border-radius: 20px; padding: 10px 16px; margin-bottom: 10px; }
.big { font-size: 1.8em; font-weight: bold; }
.small { color: #b8c4cc; font-size: 0.9em; }
button { background: #505E67; color: #fff; border: none; border-radius: 12px; padding: 14px 18px; margin: 4px; font-size: 1.1em; cursor: pointer; }
button.time { background: #2e7d5b; padding: 22px 30px; font-size: 1.5em; }
table { border-collapse: collapse; }
th, td { padding: 3px 9px; text-align: right; }
th { color: #b8c4cc; font-weight: normal; border-bottom: 1px solid #505E67; }
tr.auto td { color: #9fd8ff; }
</style></head><body>
<div class="box">
<button onclick="location.href='/'">Main page</button>
<button onclick="location.href='/advanced'">More Battery Info</button>
<span class="small" id="conn">connecting...</span>
</div>

<div class="box">
<div class="big" id="state">--</div>
<div id="detail" class="small"></div>
<div id="counter"></div>
<div id="bus"></div>
</div>

<div class="box">
<div id="shutdown">Shutdown: --</div>
<div id="next" class="big"></div>
<div class="small">Reference points of one logged shutdown, seconds after ignition off: C2 64, C0 124, 00 134, bus end 135. The time value of the LBC jumped at the start of 00.</div>
</div>

<div class="box">
<button class="time" onclick="act('time')">Time 22 92 61</button>
<div id="qstatus" class="small"></div>
<div><label><input type="checkbox" id="auto" onchange="act('auto', this.checked ? 1 : 0)"> Auto schedule</label>
<span class="small" id="auto_info"></span></div>
<div class="small">Auto schedule: sends only 22 92 61 after ignition off: once at 60, 90 and 110 s, every 0.5 s from 7 s after the start of C0 until 3 s after the start of 00 (once per shutdown, and only while 0x350 frames keep coming), and every 5 s from 40 to 65 s after the bus end. Switched off after every restart.</div>
</div>

<div class="box">
<div><b>Results</b> <button onclick="act('clear')">Clear list</button></div>
<table id="results"><thead><tr><th>since off (s)</th><th>state</th><th>kind</th><th>result</th><th>0x9261 (min)</th><th>0x350 counter</th><th>counter - value</th><th>tries</th></tr></thead><tbody id="rows"></tbody></table>
</div>

<script>
function parse(t) {
  var o = { r: [] };
  var L = t.split('\n');
  for (var i = 0; i < L.length; i++) {
    var s = L[i];
    var k = s.indexOf('=');
    if (k < 0) { continue; }
    var key = s.substring(0, k);
    var val = s.substring(k + 1);
    if (key === 'r') { o.r.push(val.split('|')); } else { o[key] = val; }
  }
  return o;
}
function set(id, text) { document.getElementById(id).textContent = text; }
function upd(d) {
  if (d.state === '--' || d.state === undefined) {
    set('state', 'no 0x350 frame yet');
    set('detail', '');
    set('counter', '');
  } else {
    set('state', 'Vehicle state ' + d.state + (d.name ? ' (' + d.name + ')' : '') + ', ' + d.state_age + ' s in this state');
    set('counter', 'Minute counter 0x350: ' + d.counter);
  }
  set('bus', d.bus === 'active' ? 'Bus: active' : (d.bus && d.bus.indexOf('silent:') === 0 ? 'Bus: no 0x350 for ' + d.bus.substring(7) + ' s' : 'Bus: --'));
  if (d.shutdown === 'running') {
    set('shutdown', 'Shutdown after ignition off: running for ' + d.since + ' s');
    var n = (d.next || '').split(':');
    set('next', n.length === 2 ? 'Next: ' + n[0] + ' in about ' + n[1] + ' s' : 'Past the last reference point');
  } else if (d.shutdown === 'finished') {
    set('shutdown', 'Shutdown finished, ' + d.since + ' s since ignition off');
    set('next', '');
  } else {
    set('shutdown', 'Shutdown after ignition off: not running');
    set('next', '');
  }
  set('qstatus', d.query === 'running' ? 'query running...' : '');
  var cb = document.getElementById('auto');
  var on = d.auto === '1';
  if (cb.checked !== on && document.activeElement !== cb) { cb.checked = on; }
  set('auto_info', on ? ' on, ' + d.auto_n + ' automatic queries in this shutdown' : ' off');
  var tb = document.getElementById('rows');
  var html = '';
  for (var i = 0; i < d.r.length; i++) {
    var f = d.r[i];
    var res = f[3] === '1' ? 'OK' : (f[3] === '2' ? 'negative, NRC 0x' + Number(f[4]).toString(16) : 'no answer');
    var val = f[3] === '1' ? f[4] : '-';
    var diff = (f[3] === '1' && f[5] !== undefined) ? String(Number(f[5]) - Number(f[4])) : '-';
    html += '<tr class="' + (f[2] === '1' ? 'auto' : '') + '"><td>' + f[0] + '</td><td>' + f[1] + '</td><td>' + (f[2] === '1' ? 'auto' : 'manual') +
            '</td><td>' + res + '</td><td>' + val + '</td><td>' + f[5] + '</td><td>' + diff + '</td><td>' + f[6] + '</td></tr>';
  }
  tb.innerHTML = html;
}
function poll() {
  fetch('/kangooLiveData', { cache: 'no-store' })
    .then(function (r) { return r.text(); })
    .then(function (t) { upd(parse(t)); set('conn', 'live'); setTimeout(poll, 1000); })
    .catch(function () { set('conn', 'connection lost'); setTimeout(poll, 2000); });
}
function act(c, v) {
  fetch('/kangooLiveAction?cmd=' + c + (v !== undefined ? '&value=' + v : ''), { cache: 'no-store' }).catch(function () {});
}
poll();
</script>
</body></html>
)rawliteral";
