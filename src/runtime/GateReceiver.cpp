#include "runtime/GateReceiver.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_timer.h>
#include <esp_wifi.h>

#include "common/DebugLog.h"
#include "runtime/RuntimeTasks.h"

namespace
{
    portMUX_TYPE gateMux = portMUX_INITIALIZER_UNLOCKED;
    vbt::GateSplitMetrics liveGates{};
    uint64_t lastTriggerUs = 0;

    void handleGatePacket(const uint8_t* data, int len)
    {
        // Stamp immediately inside the Priority-23 Wi-Fi task (< 20 us latency).
        const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());

        if (len < 1 || data[0] != vbt::runtime::GateReceiver::MSG_GATE_TRIGGER) return;
        if (vbt::runtime::activeSessionState() != vbt::SessionState::Recording) return;

        portENTER_CRITICAL(&gateMux);
        const bool isDuplicateBurst = (lastTriggerUs != 0) && (nowUs - lastTriggerUs < vbt::runtime::GateReceiver::MIN_GATE_INTERVAL_US);
        if (!isDuplicateBurst && liveGates.count < vbt::GateSplitMetrics::MAX_GATES)
        {
            lastTriggerUs = nowUs;
            liveGates.timestamps_us[liveGates.count++] = nowUs;
        }
        portEXIT_CRITICAL(&gateMux);
    }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    void onEspNowRecv(const esp_now_recv_info_t*, const uint8_t* data, int len)
    {
        handleGatePacket(data, len);
    }
#else
    void onEspNowRecv(const uint8_t*, const uint8_t* data, int len)
    {
        handleGatePacket(data, len);
    }
#endif
}

namespace vbt::runtime
{
    void GateReceiver::begin()
    {
        if (running_) return;

        const wifi_mode_t currentMode = WiFi.getMode();
        if (currentMode == WIFI_AP || currentMode == WIFI_AP_STA)
        {
            WiFi.mode(WIFI_AP_STA);
        }
        else
        {
            WiFi.mode(WIFI_STA);
        }

        esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_ps(WIFI_PS_NONE);

        if (esp_now_init() != ESP_OK)
        {
            VBT_LOGLN("ERROR: ESP-NOW init failed");
            return;
        }

        esp_now_register_recv_cb(onEspNowRecv);
        running_ = true;
        VBT_LOGLN("[GATES] ESP-NOW receiver ready on Channel 1");
    }

    void GateReceiver::end()
    {
        if (!running_) return;

        esp_now_unregister_recv_cb();
        esp_now_deinit();
        running_ = false;

        // Only power down Wi-Fi if the Phone Web UI SoftAP is not active.
        if (WiFi.getMode() == WIFI_STA)
        {
            WiFi.mode(WIFI_OFF);
        }
    }

    void GateReceiver::reset()
    {
        portENTER_CRITICAL(&gateMux);
        liveGates = {};
        lastTriggerUs = 0;
        portEXIT_CRITICAL(&gateMux);
    }

    vbt::GateSplitMetrics GateReceiver::snapshot() const
    {
        vbt::GateSplitMetrics copy{};
        portENTER_CRITICAL(&gateMux);
        copy = liveGates;
        portEXIT_CRITICAL(&gateMux);

        for (uint8_t i = 1; i < copy.count; ++i)
        {
            const uint64_t dtUs = (copy.timestamps_us[i] > copy.timestamps_us[i - 1])
                ? (copy.timestamps_us[i] - copy.timestamps_us[i - 1])
                : 0;
            copy.split_seconds[i - 1] = static_cast<float>(dtUs) / 1000000.0f;
        }

        if (copy.count >= 2 && copy.timestamps_us[copy.count - 1] > copy.timestamps_us[0])
        {
            copy.total_seconds = static_cast<float>(copy.timestamps_us[copy.count - 1] - copy.timestamps_us[0]) / 1000000.0f;
        }

        return copy;
    }

    GateReceiver& gateReceiver()
    {
        static GateReceiver instance{};
        return instance;
    }
}
