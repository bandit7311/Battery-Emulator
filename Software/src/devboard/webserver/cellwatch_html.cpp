#include "cellwatch_html.h"
#include <Arduino.h>

// Cellwatch diagnostic page. Deliberately NOT using a fixed reload timer (unlike /cellmonitor,
// /events, the main page): this is meant to show individual samples arriving as fast as the BMS
// answers them, which a 1-2s full-page reload would either miss entirely or make flicker badly.
// Instead the page polls the lightweight /cellwatchStatus endpoint (plain text, pipe-separated)
// every 150ms and only touches the DOM when the sample counter has actually moved - no reload,
// no flicker, the page just grows a log of real samples as they come in.
String cellwatch_processor(const String& var) {
  if (var == "X") {
    String content = "";
    content += "<style>";
    content += "body { background-color: black; color: white; font-family: sans-serif; }";
    content +=
        "button { background-color: #505E67; color: white; border: none; padding: 10px 20px; "
        "cursor: pointer; border-radius: 10px; margin: 4px; }";
    content += "button:hover { background-color: #3A4A52; }";
    content +=
        "table { border-collapse: collapse; width: 100%; max-width: 480px; }";
    content += "td, th { border: 1px solid #444; padding: 4px 8px; text-align: right; }";
    content += "th { text-align: center; }";
    content += ".delta-up { color: #6fcf6f; }";
    content += ".delta-down { color: #ff7b7b; }";
    content += "#status { margin-bottom: 10px; }";
    content += "</style>";

    content += "<h2>Cellwatch</h2>";
    content +=
        "<p>Shows samples as they actually arrive for the cell currently selected on the "
        "<a href='/advanced' style='color:#8fd3ff;'>More Battery Info</a> page. Enable Cellwatch there "
        "first - this page only displays what comes in, it does not start polling by itself.</p>";
    content += "<div id='status'>Loading...</div>";
    content += "<table id='log'><thead><tr><th>Sample #</th><th>Time (s)</th><th>mV</th><th>&Delta;</th></tr></thead>";
    content += "<tbody id='rows'></tbody></table>";
    content += "<p><button onclick=\"document.getElementById('rows').innerHTML='';\">Clear log</button></p>";

    content += "<script>";
    content += "var lastCount=-1,lastMV=null,startMs=null,maxRows=200;";
    content += "function poll(){";
    content += "fetch('/cellwatchStatus').then(function(r){return r.text();}).then(function(t){";
    content += "var p=t.split('|');";
    content += "var enabled=p[0]==='1',cell=p[1],mv=parseInt(p[2],10),count=parseInt(p[3],10),sampleMs=parseInt(p[4],10);";
    content +=
        "document.getElementById('status').textContent="
        "(enabled?'Enabled':'Disabled (turn it on in More Battery Info)')+' - watching cell '+cell"
        "+' - samples so far: '+count;";
    content += "if(count!==lastCount && count>0){";
    content += "if(startMs===null){startMs=sampleMs;}";
    content += "var delta=(lastMV===null)?0:(mv-lastMV);";
    content +=
        "var cls=delta>0?'delta-up':(delta<0?'delta-down':'');";
    content += "var row=document.createElement('tr');";
    content +=
        "row.innerHTML='<td>'+count+'</td><td>'+((sampleMs-startMs)/1000).toFixed(1)+'</td>"
        "<td>'+mv+'</td><td class=\"'+cls+'\">'+(delta>0?'+':'')+delta+'</td>';";
    content += "var rows=document.getElementById('rows');";
    content += "rows.insertBefore(row,rows.firstChild);";
    content += "while(rows.children.length>maxRows){rows.removeChild(rows.lastChild);}";
    content += "lastMV=mv;lastCount=count;";
    content += "}";
    content += "if(count===0){lastCount=-1;lastMV=null;startMs=null;}";
    content += "}).catch(function(){});";
    content += "}";
    content += "setInterval(poll,150);";
    content += "poll();";
    content += "</script>";

    return content;
  }
  return String();
}
