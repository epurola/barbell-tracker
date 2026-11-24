#include "ui/HeadlessWebUi.h"

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <cstdio>

#include "common/DebugLog.h"
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
        json.reserve(768);

        char buf[224];
        snprintf(buf, sizeof(buf), "{\"state\":%u,\"mode\":%u,\"set\":%lu,",
            state, mode, static_cast<unsigned long>(res.set_number));
        json += buf;

        // Barbell rep block
        const float concMs = (res.rep.concentric_end_us > res.rep.concentric_start_us)
            ? static_cast<float>(res.rep.concentric_end_us - res.rep.concentric_start_us) / 1000.0f
            : 0.0f;
        const float eccMs = (res.rep.concentric_start_us > res.rep.start_timestamp_us)
            ? static_cast<float>(res.rep.concentric_start_us - res.rep.start_timestamp_us) / 1000.0f
            : 0.0f;

        snprintf(buf, sizeof(buf), "\"rep\":{\"n\":%lu,\"best\":%.2f,\"avg\":%.2f,\"pk\":%.2f,\"acc\":%.1f,\"loss\":%.1f,\"rom\":%.1f,\"concMs\":%.0f,\"eccMs\":%.0f,\"list\":[",
            static_cast<unsigned long>(res.rep.rep_number),
            res.rep.best_mean_velocity_mps,
            res.rep.set_mean_velocity_mps,
            res.rep.peak_concentric_velocity_mps,
            res.rep.peak_concentric_accel_mps2,
            res.rep.velocity_loss_percent,
            res.rep.range_of_motion_m * 100.0f,
            concMs,
            eccMs);
        json += buf;
        const uint32_t repCount = res.rep.rep_number < vbt::RepMetrics::MAX_SET_REPS ? res.rep.rep_number : vbt::RepMetrics::MAX_SET_REPS;
        for (uint32_t i = 0; i < repCount; ++i)
        {
            if (i > 0) json += ',';
            snprintf(buf, sizeof(buf), "%.2f", res.rep.rep_mean_velocity_mps[i]);
            json += buf;
        }
        json += "],\"pkList\":[";
        for (uint32_t i = 0; i < repCount; ++i)
        {
            if (i > 0) json += ',';
            snprintf(buf, sizeof(buf), "%.2f", res.rep.rep_peak_velocity_mps[i]);
            json += buf;
        }
        json += "]},";

        // Jump block
        snprintf(buf, sizeof(buf), "\"jump\":{\"n\":%lu,\"best\":%.1f,\"avg\":%.1f,\"flt\":%.0f,\"toVel\":%.2f,\"pkVel\":%.2f,\"list\":[",
            static_cast<unsigned long>(res.jump.jump_number),
            res.jump.best_jump_height_cm,
            res.jump.mean_jump_height_cm,
            res.jump.flight_time_ms,
            res.jump.takeoff_velocity_mps,
            res.jump.peak_velocity_mps);
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
        const float sprintDurS = (res.sprint.sprint_end_timestamp_us > res.sprint.sprint_start_timestamp_us)
            ? static_cast<float>(res.sprint.sprint_end_timestamp_us - res.sprint.sprint_start_timestamp_us) / 1000000.0f
            : 0.0f;

        snprintf(buf, sizeof(buf), "\"sprint\":{\"valid\":%s,\"n\":%lu,\"mean\":%.1f,\"min\":%.1f,\"flt\":%.1f,\"freq\":%.2f,\"asym\":%.1f,\"drift\":%.1f,\"last\":%.1f,\"durS\":%.2f}}",
            res.sprint.valid ? "true" : "false",
            static_cast<unsigned long>(res.sprint.step_count),
            res.sprint.mean_contact_time_ms,
            res.sprint.min_contact_time_ms,
            res.sprint.mean_flight_time_ms,
            res.sprint.step_frequency_hz,
            res.sprint.odd_even_asymmetry_percent,
            res.sprint.contact_drift_ms,
            res.sprint.last_contact_time_ms,
            sprintDurS);
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
        server.on("/trace", HTTP_GET, handleTracePage);
        server.on("/trace.csv", HTTP_GET, handleTraceCsv);
        server.on("/cmd", HTTP_GET, handleCmd);
        server.on("/status", HTTP_GET, handleStatus);
        server.begin();
        running_ = true;

        VBT_LOGF("[WIFI] AP Ready! SSID: %s  Pass: %s  URL: http://%s.local (http://%s)\n",
            AP_SSID, AP_PASS, MDNS_HOST, WiFi.softAPIP().toString().c_str());
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
