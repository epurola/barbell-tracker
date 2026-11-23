#include "ui/HeadlessWebUi.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cstdio>

#include "common/DebugLog.h"
#include "runtime/RuntimeTasks.h"

namespace
{
    constexpr const char* AP_SSID = "VBT-Tracker";
    constexpr const char* AP_PASS = "12345678";
    constexpr const char* MDNS_HOST = "vbt";

    WebServer server(80);

    const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>VBT Tracker</title>
<style>
  *{box-sizing:border-box;font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif}
  body{margin:0;padding:16px;background:#111418;color:#f1f5f9;max-width:460px;margin:0 auto}
  h1{font-size:20px;margin:0 0 12px;color:#38bdf8;display:flex;justify-content:space-between;align-items:center}
  .badge{font-size:13px;padding:5px 10px;border-radius:999px;font-weight:700;background:#1e293b;color:#94a3b8}
  .badge.cal{background:#713f12;color:#fde047}
  .badge.rec{background:#7f1d1d;color:#fca5a5}
  .badge.res{background:#14532d;color:#86efac}
  .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px}
  .grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:14px}
  button{border:none;border-radius:12px;padding:16px 8px;font-size:15px;font-weight:700;color:#fff;background:#1e293b;cursor:pointer;-webkit-tap-highlight-color:transparent}
  button:active{transform:scale(0.97)}
  .btn-mode{border:1px solid #334155}
  .btn-stop{background:#dc2626;font-size:17px}
  .btn-next{background:#16a34a;font-size:17px}
  .card{background:#1a2029;border:1px solid #293342;border-radius:14px;padding:16px;margin-top:8px}
  .hero-label{font-size:12px;color:#94a3b8;text-transform:uppercase;letter-spacing:.06em;text-align:center}
  .hero-val{font-size:48px;font-weight:800;color:#4ade80;text-align:center;line-height:1.1;margin:6px 0}
  .sub{font-size:14px;color:#cbd5e1;text-align:center;margin-bottom:12px}
  .stats{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
  .stat{background:#111418;border-radius:10px;padding:10px;text-align:center}
  .stat .k{font-size:11px;color:#94a3b8}
  .stat .v{font-size:20px;font-weight:700;margin-top:2px}
  .warn{color:#facc15}
  .reps{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:12px;font-size:14px}
  .rep{background:#111418;padding:8px 10px;border-radius:8px;display:flex;justify-content:space-between}
</style>
</head>
<body>
  <h1>VBT Tracker <span id="badge" class="badge">IDLE</span></h1>
  <div class="grid">
    <button class="btn-mode" onclick="cmd('barbell')">Barbell</button>
    <button class="btn-mode" onclick="cmd('jump')">Jump</button>
    <button class="btn-mode" onclick="cmd('sprint')">Sprint</button>
  </div>
  <div class="grid2">
    <button class="btn-stop" onclick="cmd('stop')">STOP SET</button>
    <button class="btn-next" onclick="cmd('next')">NEXT SET</button>
  </div>
  <div id="out" class="card">
    <div class="sub" style="margin:0">Tap a mode, hold still 1s for calibration, then go!</div>
  </div>
<script>
async function cmd(a){
  try{ await fetch('/cmd?a='+a); }catch(e){}
  poll();
}
function render(d){
  const b=document.getElementById('badge');
  const states=['IDLE','HOLD STILL (1s)','RECORDING','RESULTS'];
  const cls=['','cal','rec','res'];
  b.textContent=states[d.state]||'IDLE';
  b.className='badge '+(cls[d.state]||'');

  const out=document.getElementById('out');
  if(d.state===1){
    out.innerHTML='<div class="hero-val" style="font-size:28px;color:#fde047">HOLD STILL</div><div class="sub">Calibrating IMU (1s)...</div>';
    return;
  }
  if(d.state===2){
    out.innerHTML='<div class="hero-val" style="font-size:28px;color:#fca5a5">RECORDING</div><div class="sub">500 Hz active — tap STOP SET when finished</div>';
    return;
  }
  if(d.state!==3){
    out.innerHTML='<div class="sub" style="margin:0">Tap Barbell, Jump, or Sprint to start.</div>';
    return;
  }
  if(d.mode===0){
    const r=d.rep;
    if(!r.n){ out.innerHTML='<div class="hero-val warn" style="font-size:32px">0 REPS</div><div class="sub">Min depth: 15 cm</div>'; return; }
    let repsHtml='<div class="reps">';
    for(let i=0;i<r.list.length;i++) repsHtml+='<div class="rep"><span>R'+(i+1)+'</span><b>'+r.list[i].toFixed(2)+' m/s</b></div>';
    repsHtml+='</div>';
    out.innerHTML='<div class="hero-label">Set '+d.set+' ('+r.n+' Reps) — Best Mean Vel</div>'+
      '<div class="hero-val">'+r.best.toFixed(2)+' <span style="font-size:18px">m/s</span></div>'+
      '<div class="sub">Avg: '+r.avg.toFixed(2)+' | Peak: '+r.pk.toFixed(2)+' | Loss: '+r.loss.toFixed(0)+'% | ROM: '+r.rom.toFixed(0)+'cm</div>'+repsHtml;
  } else if(d.mode===1){
    const j=d.jump;
    if(!j.n){ out.innerHTML='<div class="hero-val warn" style="font-size:32px">0 JUMPS</div><div class="sub">Stand still before jumping</div>'; return; }
    let listHtml='<div class="reps">';
    for(let i=0;i<j.list.length;i++) listHtml+='<div class="rep"><span>J'+(i+1)+'</span><b>'+j.list[i].toFixed(1)+' cm</b></div>';
    listHtml+='</div>';
    out.innerHTML='<div class="hero-label">Jump Set '+d.set+' ('+j.n+' Jumps) — Best Height</div>'+
      '<div class="hero-val">'+j.best.toFixed(1)+' <span style="font-size:18px">cm</span></div>'+
      '<div class="sub">Avg: '+j.avg.toFixed(1)+' cm | Flight: '+j.flt.toFixed(0)+' ms</div>'+listHtml;
  } else if(d.mode===3){
    const s=d.sprint;
    if(!s.valid||!s.n){ out.innerHTML='<div class="hero-val warn" style="font-size:32px">0 STEPS</div><div class="sub">Min 4 sprint steps</div>'; return; }
    out.innerHTML='<div class="hero-label">Sprint '+d.set+' ('+s.n+' Steps) — Mean GCT</div>'+
      '<div class="hero-val">'+s.mean.toFixed(0)+' <span style="font-size:18px">ms</span></div>'+
      '<div class="sub">Min: '+s.min.toFixed(0)+' ms | Flight: '+s.flt.toFixed(0)+' ms</div>'+
      '<div class="stats">'+
        '<div class="stat"><div class="k">FREQ</div><div class="v">'+s.freq.toFixed(2)+' Hz</div></div>'+
        '<div class="stat"><div class="k">ASYM</div><div class="v '+(s.asym>5?'warn':'')+'">'+s.asym.toFixed(1)+'%</div></div>'+
        '<div class="stat"><div class="k">DRIFT</div><div class="v '+(s.drift>5?'warn':'')+'">'+(s.drift>=0?'+':'')+s.drift.toFixed(0)+' ms</div></div>'+
        '<div class="stat"><div class="k">LAST STEP</div><div class="v">'+s.last.toFixed(0)+' ms</div></div>'+
      '</div>';
  }
}
async function poll(){
  try{
    const r=await fetch('/status');
    if(r.ok) render(await r.json());
  }catch(e){}
}
setInterval(poll, 700);
poll();
</script>
</body>
</html>)rawliteral";

    void handleRoot()
    {
        server.send_P(200, "text/html", INDEX_HTML);
    }

    void handleCmd()
    {
        const String a = server.arg("a");
        if (a == "barbell")      vbt::runtime::requestStartSession(vbt::ActivityMode::BarbellVbt, true);
        else if (a == "jump")    vbt::runtime::requestStartSession(vbt::ActivityMode::StandingJump, true);
        else if (a == "sprint")  vbt::runtime::requestStartSession(vbt::ActivityMode::SprintContact, true);
        else if (a == "stop")    vbt::runtime::requestStopSession();
        else if (a == "next")    vbt::runtime::requestNextSet();
        else if (a == "idle")    vbt::runtime::requestModeSelect();

        server.send(200, "text/plain", "OK");
    }

    void handleStatus()
    {
        const auto state = static_cast<unsigned>(vbt::runtime::activeSessionState());
        const auto mode = static_cast<unsigned>(vbt::runtime::activeActivityMode());
        const vbt::ActivityOutput res = vbt::runtime::latestActivityResult();

        String json;
        json.reserve(512);

        char buf[192];
        snprintf(buf, sizeof(buf), "{\"state\":%u,\"mode\":%u,\"set\":%lu,",
            state, mode, static_cast<unsigned long>(res.set_number));
        json += buf;

        // Barbell rep block
        snprintf(buf, sizeof(buf), "\"rep\":{\"n\":%lu,\"best\":%.2f,\"avg\":%.2f,\"pk\":%.2f,\"loss\":%.1f,\"rom\":%.1f,\"list\":[",
            static_cast<unsigned long>(res.rep.rep_number),
            res.rep.best_mean_velocity_mps,
            res.rep.set_mean_velocity_mps,
            res.rep.peak_concentric_velocity_mps,
            res.rep.velocity_loss_percent,
            res.rep.range_of_motion_m * 100.0f);
        json += buf;
        const uint32_t repCount = res.rep.rep_number < vbt::RepMetrics::MAX_SET_REPS ? res.rep.rep_number : vbt::RepMetrics::MAX_SET_REPS;
        for (uint32_t i = 0; i < repCount; ++i)
        {
            if (i > 0) json += ',';
            snprintf(buf, sizeof(buf), "%.2f", res.rep.rep_mean_velocity_mps[i]);
            json += buf;
        }
        json += "]},";

        // Jump block
        snprintf(buf, sizeof(buf), "\"jump\":{\"n\":%lu,\"best\":%.1f,\"avg\":%.1f,\"flt\":%.0f,\"list\":[",
            static_cast<unsigned long>(res.jump.jump_number),
            res.jump.best_jump_height_cm,
            res.jump.mean_jump_height_cm,
            res.jump.flight_time_ms);
        json += buf;
        const uint32_t jumpCount = res.jump.jump_number < vbt::JumpMetrics::MAX_SET_JUMPS ? res.jump.jump_number : vbt::JumpMetrics::MAX_SET_JUMPS;
        for (uint32_t i = 0; i < jumpCount; ++i)
        {
            if (i > 0) json += ',';
            snprintf(buf, sizeof(buf), "%.1f", res.jump.jump_heights_cm[i]);
            json += buf;
        }
        json += "]},";

        // Sprint block
        snprintf(buf, sizeof(buf), "\"sprint\":{\"valid\":%s,\"n\":%lu,\"mean\":%.1f,\"min\":%.1f,\"flt\":%.1f,\"freq\":%.2f,\"asym\":%.1f,\"drift\":%.1f,\"last\":%.1f}}",
            res.sprint.valid ? "true" : "false",
            static_cast<unsigned long>(res.sprint.step_count),
            res.sprint.mean_contact_time_ms,
            res.sprint.min_contact_time_ms,
            res.sprint.mean_flight_time_ms,
            res.sprint.step_frequency_hz,
            res.sprint.odd_even_asymmetry_percent,
            res.sprint.contact_drift_ms,
            res.sprint.last_contact_time_ms);
        json += buf;

        server.send(200, "application/json", json);
    }
}

namespace vbt::ui
{
    void HeadlessWebUi::begin()
    {
        if (running_) return;

        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS);
        MDNS.begin(MDNS_HOST);
        MDNS.addService("http", "tcp", 80);

        server.on("/", HTTP_GET, handleRoot);
        server.on("/cmd", HTTP_GET, handleCmd);
        server.on("/status", HTTP_GET, handleStatus);
        server.begin();
        running_ = true;

        VBT_LOGF("[WIFI] AP Ready! SSID: %s  Pass: %s  URL: http://%s.local (http://%s)\n", AP_SSID, AP_PASS, MDNS_HOST, WiFi.softAPIP().toString().c_str());
    }

    void HeadlessWebUi::end()
    {
        if (!running_) return;

        server.stop();
        MDNS.end();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        running_ = false;
    }

    void HeadlessWebUi::update()
    {
        if (!running_) return;
        server.handleClient();
    }
}
