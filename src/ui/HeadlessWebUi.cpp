#include "ui/HeadlessWebUi.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cstdio>

#include "common/DebugLog.h"
#include "runtime/GateReceiver.h"
#include "runtime/RuntimeTasks.h"
#include "runtime/TraceLogger.h"

extern const char index_html_start[] asm("_binary_web_index_html_start");
extern const char trace_html_start[] asm("_binary_web_trace_html_start");

namespace
{
    constexpr const char* AP_SSID = "VBT-Tracker";
    constexpr const char* AP_PASS = "12345678";
    constexpr const char* MDNS_HOST = "vbt";

    WebServer server(80);

    void handleRoot()
    {
        server.send(200, "text/html", index_html_start);
    }

    void handleTracePage()
    {
        server.send(200, "text/html", trace_html_start);
    }

    void handleTraceCsv()
    {
        const auto& logger = vbt::runtime::traceLogger();
        if (!logger.isFrozen())
        {
            server.send(409, "text/plain", "Recording in progress — stop set first");
            return;
        }

        const size_t total = logger.count();

        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "text/csv", "");

        String chunk;
        chunk.reserve(1024);

        char line[64];
        snprintf(line, sizeof(line), "#mode=%u\nt,az,vz,pz,ph,st,ev\n", static_cast<unsigned>(logger.mode()));
        chunk += line;

        vbt::runtime::TraceSample s{};
        for (size_t i = 0; i < total; ++i)
        {
            if (!logger.sampleAt(i, s)) continue;

            snprintf(line, sizeof(line), "%lu,%.2f,%.3f,%.1f,%u,%u,%u\n",
                static_cast<unsigned long>(s.dt_ms),
                static_cast<float>(s.az_cmps2) * 0.01f,
                static_cast<float>(s.vz_mmps) * 0.001f,
                static_cast<float>(s.pz_mm) * 0.1f,
                static_cast<unsigned>(s.phase),
                static_cast<unsigned>((s.flags & vbt::runtime::TraceLogger::FLAG_STATIONARY) ? 1u : 0u),
                static_cast<unsigned>((s.flags & vbt::runtime::TraceLogger::FLAG_COMPLETED) ? 1u : 0u));
            chunk += line;

            if (chunk.length() >= 900)
            {
                server.sendContent(chunk);
                chunk = "";
            }
        }

        if (chunk.length() > 0)
        {
            server.sendContent(chunk);
        }
        server.sendContent("");
    }

    void handleCmd()
    {
        const String a = server.arg("a");
        if (a == "barbell")      vbt::runtime::requestStartSession(vbt::ActivityMode::BarbellVbt, true);
        else if (a == "jump")    vbt::runtime::requestStartSession(vbt::ActivityMode::StandingJump, true);
        else if (a == "hurdle")  vbt::runtime::requestStartSession(vbt::ActivityMode::HurdleContact, true);
        else if (a == "sprint")  vbt::runtime::requestStartSession(vbt::ActivityMode::SprintContact, true);
        else if (a == "block")   vbt::runtime::requestStartSession(vbt::ActivityMode::BlockStart, true);
        else if (a == "stop")    vbt::runtime::requestStopSession();
        else if (a == "next")    vbt::runtime::requestNextSet();
        else if (a == "idle")    vbt::runtime::requestModeSelect();

        server.send(200, "text/plain", "OK");
    }

    void handleStatus()
    {
        const vbt::SessionState state = vbt::runtime::activeSessionState();
        const vbt::ActivityMode mode  = vbt::runtime::activeActivityMode();

        JsonDocument doc;
        doc["state"] = static_cast<unsigned>(state);
        doc["mode"]  = static_cast<unsigned>(mode);

        if (state == vbt::SessionState::ResultsReady)
        {
            const vbt::ActivityOutput res = vbt::runtime::latestActivityResult();
            doc["mode"] = static_cast<unsigned>(res.mode);
            doc["set"]  = res.set_number;

            switch (res.mode)
            {
                case vbt::ActivityMode::BarbellVbt:
                {
                    const float concMs = (res.rep.concentric_end_us > res.rep.concentric_start_us)
                        ? static_cast<float>(res.rep.concentric_end_us - res.rep.concentric_start_us) / 1000.0f
                        : 0.0f;
                    const float eccMs = (res.rep.concentric_start_us > res.rep.start_timestamp_us)
                        ? static_cast<float>(res.rep.concentric_start_us - res.rep.start_timestamp_us) / 1000.0f
                        : 0.0f;

                    JsonObject rep = doc["rep"].to<JsonObject>();
                    rep["n"]      = res.rep.rep_number;
                    rep["best"]   = res.rep.best_mean_velocity_mps;
                    rep["avg"]    = res.rep.set_mean_velocity_mps;
                    rep["pk"]     = res.rep.peak_concentric_velocity_mps;
                    rep["acc"]    = res.rep.peak_concentric_accel_mps2;
                    rep["loss"]   = res.rep.velocity_loss_percent;
                    rep["rom"]    = res.rep.range_of_motion_m * 100.0f;
                    rep["concMs"] = concMs;
                    rep["eccMs"]  = eccMs;

                    JsonArray repList = rep["list"].to<JsonArray>();
                    JsonArray pkList  = rep["pkList"].to<JsonArray>();
                    const uint32_t repCount = res.rep.rep_number < vbt::RepMetrics::MAX_SET_REPS ? res.rep.rep_number : vbt::RepMetrics::MAX_SET_REPS;
                    for (uint32_t i = 0; i < repCount; ++i)
                    {
                        repList.add(res.rep.rep_mean_velocity_mps[i]);
                        pkList.add(res.rep.rep_peak_velocity_mps[i]);
                    }
                    break;
                }

                case vbt::ActivityMode::StandingJump:
                {
                    JsonObject jump = doc["jump"].to<JsonObject>();
                    jump["n"]     = res.jump.jump_number;
                    jump["best"]  = res.jump.best_jump_height_cm;
                    jump["avg"]   = res.jump.mean_jump_height_cm;
                    jump["flt"]   = res.jump.flight_time_ms;
                    jump["toVel"] = res.jump.takeoff_velocity_mps;
                    jump["pkVel"] = res.jump.peak_velocity_mps;

                    JsonArray jumpList = jump["list"].to<JsonArray>();
                    const uint32_t jumpCount = res.jump.jump_number < vbt::JumpMetrics::MAX_SET_JUMPS ? res.jump.jump_number : vbt::JumpMetrics::MAX_SET_JUMPS;
                    for (uint32_t i = 0; i < jumpCount; ++i)
                    {
                        jumpList.add(res.jump.jump_heights_cm[i]);
                    }
                    break;
                }

                case vbt::ActivityMode::HurdleContact:
                {
                    JsonObject h = doc["hurdle"].to<JsonObject>();
                    h["valid"]   = res.hurdle.valid;
                    h["n"]       = res.hurdle.contact_number;
                    h["bestRsi"] = res.hurdle.best_rsi;
                    h["meanRsi"] = res.hurdle.mean_rsi;
                    h["minGct"]  = res.hurdle.min_contact_time_ms;
                    h["meanGct"] = res.hurdle.mean_contact_time_ms;
                    h["height"]  = res.hurdle.jump_height_cm;
                    h["flt"]     = res.hurdle.flight_time_ms;

                    JsonArray gctList = h["gctList"].to<JsonArray>();
                    JsonArray rsiList = h["rsiList"].to<JsonArray>();
                    const uint32_t hopCount = res.hurdle.contact_number < vbt::HurdleContactMetrics::MAX_SET_HURDLES
                        ? res.hurdle.contact_number
                        : vbt::HurdleContactMetrics::MAX_SET_HURDLES;
                    for (uint32_t i = 0; i < hopCount; ++i)
                    {
                        gctList.add(res.hurdle.hop_contact_time_ms[i]);
                        rsiList.add(res.hurdle.hop_rsi[i]);
                    }
                    break;
                }

                case vbt::ActivityMode::SprintContact:
                {
                    const float sprintDurS = (res.sprint.sprint_end_timestamp_us > res.sprint.sprint_start_timestamp_us)
                        ? static_cast<float>(res.sprint.sprint_end_timestamp_us - res.sprint.sprint_start_timestamp_us) / 1000000.0f
                        : 0.0f;

                    JsonObject sprint = doc["sprint"].to<JsonObject>();
                    sprint["valid"] = res.sprint.valid;
                    sprint["n"]     = res.sprint.step_count;
                    sprint["mean"]  = res.sprint.mean_contact_time_ms;
                    sprint["min"]   = res.sprint.min_contact_time_ms;
                    sprint["flt"]   = res.sprint.mean_flight_time_ms;
                    sprint["freq"]  = res.sprint.step_frequency_hz;
                    sprint["asym"]  = res.sprint.odd_even_asymmetry_percent;
                    sprint["drift"] = res.sprint.contact_drift_ms;
                    sprint["last"]  = res.sprint.last_contact_time_ms;
                    sprint["durS"]  = sprintDurS;
                    break;
                }

                case vbt::ActivityMode::BlockStart:
                {
                    JsonObject b = doc["block"].to<JsonObject>();
                    b["valid"]    = res.block_start.valid;
                    b["n"]        = res.block_start.step_count;
                    b["peak"]     = res.block_start.peak_drive_accel_mps2;
                    b["setPitch"] = res.block_start.set_pitch_deg;

                    JsonArray gct = b["gct"].to<JsonArray>();
                    JsonArray ang = b["ang"].to<JsonArray>();
                    const uint32_t stepCount = res.block_start.step_count < 4 ? res.block_start.step_count : 4;
                    for (uint32_t i = 0; i < stepCount; ++i)
                    {
                        gct.add(res.block_start.step_contact_time_ms[i]);
                        ang.add(res.block_start.step_pitch_deg[i]);
                    }
                    break;
                }

                default:
                    break;
            }

            if (res.gates.count > 0)
            {
                JsonObject gates = doc["gates"].to<JsonObject>();
                gates["n"]     = res.gates.count;
                gates["total"] = res.gates.total_seconds;
                JsonArray splits = gates["splits"].to<JsonArray>();
                for (uint8_t i = 1; i < res.gates.count; ++i)
                {
                    splits.add(res.gates.split_seconds[i - 1]);
                }
            }
        }

        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    }
}

namespace vbt::ui
{
    void HeadlessWebUi::begin()
    {
        if (running_) return;

        WiFi.mode(vbt::runtime::gateReceiver().isRunning() ? WIFI_AP_STA : WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASS, vbt::runtime::GateReceiver::WIFI_CHANNEL);
        MDNS.begin(MDNS_HOST);
        MDNS.addService("http", "tcp", 80);

        server.on("/", HTTP_GET, handleRoot);
        server.on("/trace", HTTP_GET, handleTracePage);
        server.on("/trace.csv", HTTP_GET, handleTraceCsv);
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
        WiFi.mode(vbt::runtime::gateReceiver().isRunning() ? WIFI_STA : WIFI_OFF);
        running_ = false;
    }

    void HeadlessWebUi::update()
    {
        if (!running_) return;
        server.handleClient();
    }
}
