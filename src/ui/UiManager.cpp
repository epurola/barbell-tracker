#include "ui/UiManager.h"

#include <Seeed_GFX.h>
#include "board/boards/XIAO_LCD_Board.h"
#include "driver/tft/Driver_JD9853A.h"
#include "panel/Panel_TFT.h"
#include "touch/Touch_AXS5106L.h"
#include <WiFi.h>
#include <cmath>
#include <cstdio>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include "common/DebugLog.h"
#include "runtime/GateReceiver.h"
#include "runtime/RuntimeTasks.h"

namespace
{
    Seeed_GFX display;
    // Shared reset is already handled by Board_XIAO_1inch47_Touch_Display; pass -1 for touch RST.
    Touch_AXS5106L touch(-1, vbt::ui::UiManager::PIN_TOUCH_IRQ, Wire, 172, 320);
    constexpr uint16_t COLOR_BG          = TFT_DARKGREY;
    constexpr uint16_t COLOR_CARD        = 0x18E3; // Dark slate
    constexpr uint16_t COLOR_ACCENT      = TFT_CYAN;
    constexpr uint16_t COLOR_START_BTN   = 0x03E0; // Dark green
    constexpr uint16_t COLOR_STOP_BG     = 0x3800; // Dark red
    constexpr uint16_t COLOR_TEXT        = TFT_WHITE;
    constexpr uint16_t COLOR_MUTED       = 0xBDF7; // Silver/gray
    constexpr uint16_t COLOR_GOOD        = TFT_GREEN;
    constexpr uint16_t COLOR_WARN        = TFT_YELLOW;
    constexpr int16_t SCREEN_W           = 172;
    constexpr int16_t SCREEN_H           = 320;
    constexpr uint32_t DEBOUNCE_MS       = 180;
    constexpr int32_t SWIPE_THRESHOLD_PX = 35;

    const char* modeTitle(vbt::ActivityMode mode)
    {
        switch (mode)
        {
            case vbt::ActivityMode::BarbellVbt:    return "BARBELL";
            case vbt::ActivityMode::StandingJump:  return "STANDING JUMP";
            case vbt::ActivityMode::HurdleContact: return "HURDLE HOPS";
            case vbt::ActivityMode::SprintContact: return "SPRINT";
            case vbt::ActivityMode::BlockStart:    return "BLOCK START";
            default:                               return "UNKNOWN";
        }
    }

    const char* modeSubtitle(vbt::ActivityMode mode)
    {
        switch (mode)
        {
            case vbt::ActivityMode::BarbellVbt:    return "VBT";
            case vbt::ActivityMode::StandingJump:  return "CNS Readiness CMJ";
            case vbt::ActivityMode::HurdleContact: return "Plyo GCT & RSI";
            case vbt::ActivityMode::SprintContact: return "GCT, Freq & Asymmetry";
            case vbt::ActivityMode::BlockStart:    return "First 4 Steps GCT";
            default:                               return "";
        }
    }

    const char* modeMountHint(vbt::ActivityMode mode)
    {
        switch (mode)
        {
            case vbt::ActivityMode::BarbellVbt: return "Mount: Barbell / Collar";
            default:                            return "Mount: Waist / Belt";
        }
    }
}

namespace vbt::ui
{
    void UiManager::startWebUI()
    {
        webUi_.begin();
        forceRedraw_ = true;
    }

    void UiManager::stopWebUI()
    {
        webUi_.end();
        forceRedraw_ = true;
    }

    void UiManager::enterDeepSleep()
    {
        VBT_LOGLN("[SLEEP] Entering deep sleep (press USR1 to wake)");

        // 1. Shut down Wi-Fi & ESP-NOW
        webUi_.end();
        vbt::runtime::gateReceiver().end();
        WiFi.mode(WIFI_OFF);

        // 2. Turn off LCD backlight and hold it LOW during deep sleep
        digitalWrite(PIN_LCD_BL, LOW);
        gpio_hold_en(static_cast<gpio_num_t>(PIN_LCD_BL));

        // 3. Stop 500 Hz acquisition timer and power down LSM6DS3 (~3 uA)
        vbt::runtime::sleepImu();

        // 4. Wake on USR1 (GPIO 11, active LOW)
        const auto wakePin = static_cast<gpio_num_t>(PIN_BTN_USR1);
        rtc_gpio_pullup_en(wakePin);
        rtc_gpio_pulldown_dis(wakePin);
        esp_sleep_enable_ext0_wakeup(wakePin, 0);

        delay(250); // Let touch/button release cleanly before sleeping
        esp_deep_sleep_start();
    }

    bool UiManager::begin()
    {
        gpio_hold_dis(static_cast<gpio_num_t>(PIN_LCD_BL));
        rtc_gpio_deinit(static_cast<gpio_num_t>(PIN_BTN_USR1));
        pinMode(PIN_BTN_USR1, INPUT_PULLUP);
        pinMode(PIN_BTN_USR2, INPUT_PULLUP);

        if (!display.begin<Board_XIAO_1inch47_Touch_Display<PIN_LCD_RST, PIN_LCD_BL>, Config_Seeed_1inch47_Touch_JD9853A>())
        {
            VBT_LOGLN("ERROR: JD9853A display initialization failed");
            return false;
        }
        
        if (!display.attachTouch(touch, display.panel().driver().bus()))
        {
            VBT_LOGLN("WARNING: AXS5106L touch attach failed (buttons still active)");
        }

        display.fillScreen(COLOR_BG);

        forceRedraw_ = true;
        return true;
    }

    void UiManager::cycleMode(int delta)
    {
        selectedCard_ = (selectedCard_ + delta + CARD_COUNT) % CARD_COUNT;
        if (selectedCard_ < MODE_COUNT)
        {
            selectedMode_ = static_cast<ActivityMode>(selectedCard_);
        }
        forceRedraw_ = true;
    }

    void UiManager::handlePrimaryAction(SessionState state)
    {
        switch (state)
        {
            case SessionState::IdleModeSelect:
                if (isSettingsCard())
                {
                    if (webUi_.isRunning()) stopWebUI();
                    else startWebUI();
                }
                else
                {
                    vbt::runtime::requestStartSession(selectedMode_, true);
                }
                break;
            case SessionState::Calibrating:
            case SessionState::Recording:    vbt::runtime::requestStopSession(); break;
            case SessionState::ResultsReady: vbt::runtime::requestNextSet(); break;
        }
        
        forceRedraw_ = true;
    }

    void UiManager::handleSecondaryAction(SessionState state)
    {
        switch (state)
        {
            case SessionState::IdleModeSelect: cycleMode(+1); break;
            case SessionState::Calibrating:
            case SessionState::Recording:
            case SessionState::ResultsReady:   vbt::runtime::requestModeSelect(); break;
        }

        forceRedraw_ = true;
    }

    void UiManager::pollButtons()
    {
        const uint32_t nowMs = millis();
        const bool usr1Pressed = (digitalRead(PIN_BTN_USR1) == LOW);
        const bool usr2Pressed = (digitalRead(PIN_BTN_USR2) == LOW);

        if (nowMs - lastButtonEventMs_ >= DEBOUNCE_MS)
        {
            const SessionState state = vbt::runtime::activeSessionState();
            if (usr1Pressed && !prevUsr1Pressed_)
            {
                lastButtonEventMs_ = nowMs;
                handlePrimaryAction(state);
            }
            else if (usr2Pressed && !prevUsr2Pressed_)
            {
                lastButtonEventMs_ = nowMs;
                handleSecondaryAction(state);
            }
        }

        prevUsr1Pressed_ = usr1Pressed;
        prevUsr2Pressed_ = usr2Pressed;
    }

    void UiManager::pollTouch(SessionState state)
    {
        // Do not access I2C touch while Calibrating or Recording (Core 1 owns Wire at 500 Hz).
        if (state == SessionState::Calibrating || state == SessionState::Recording)
        {
            touchActive_ = false;
            return;
        }

        int32_t tx = 0;
        int32_t ty = 0;

        if (display.getTouch(&tx, &ty))
        {
            if (!touchActive_)
            {
                touchActive_ = true;
                touchStartX_ = tx;
                touchStartY_ = ty;
            }
            touchLastX_ = tx;
            touchLastY_ = ty;
            return;
        }

        if (!touchActive_) return;

        touchActive_ = false;
        const int32_t dx = touchLastX_ - touchStartX_;
        const int32_t dy = touchLastY_ - touchStartY_;

        if (state == SessionState::IdleModeSelect)
        {
            if (dx <= -SWIPE_THRESHOLD_PX)
            {
                cycleMode(+1);
                return;
            }
            if (dx >= SWIPE_THRESHOLD_PX)
            {
                cycleMode(-1);
                return;
            }
            if (std::abs(dx) < 20 && std::abs(dy) < 20)
            {
                if (isSettingsCard())
                {
                    if (touchLastY_ >= 252) enterDeepSleep();
                    else if (touchLastY_ >= 192) handlePrimaryAction(state);
                }
                else if (touchLastY_ >= 210)
                {
                    handlePrimaryAction(state);
                }
            }
        }
        else if (state == SessionState::ResultsReady)
        {
            if (std::abs(dx) < 25 && std::abs(dy) < 25 && touchLastY_ >= 255)
            {
                if (touchLastX_ < SCREEN_W / 2) handlePrimaryAction(state);
                else handleSecondaryAction(state);
            }
        }
    }

    void UiManager::update()
    {
        webUi_.update();
        pollButtons();
        const SessionState state = vbt::runtime::activeSessionState();
        pollTouch(state);

        // Keep selectedMode_ synced if a session was started remotely from the Phone Web UI.
        if (state != SessionState::IdleModeSelect)
        {
            selectedMode_ = vbt::runtime::activeActivityMode();
        }

        if (!forceRedraw_ && state == lastRenderedState_ && selectedCard_ == lastRenderedCard_) return;
        
        lastRenderedState_ = state;
        lastRenderedCard_ = selectedCard_;
        forceRedraw_ = false;
        switch (state)
        {
            case SessionState::IdleModeSelect:
                if (isSettingsCard()) renderSettingsCard();
                else renderModeSelectCard();
                digitalWrite(PIN_LCD_BL, HIGH);
                break;
            case SessionState::Calibrating:
                renderCalibratingScreen();
                digitalWrite(PIN_LCD_BL, HIGH);
                break;
            case SessionState::Recording:
                digitalWrite(PIN_LCD_BL, LOW);
                break;
            case SessionState::ResultsReady:
                renderResultsScreen(vbt::runtime::latestActivityResult());
                digitalWrite(PIN_LCD_BL, HIGH);
                break;
        }
    }

    void UiManager::renderModeSelectCard()
    {
        display.fillScreen(COLOR_BG);

        for (int i = 0; i < CARD_COUNT; ++i)
        {
            const int16_t cx = 36 + i * 20;
            const uint16_t color = (i == selectedCard_) ? COLOR_ACCENT : COLOR_CARD;
            display.fillCircle(cx, 18, 4, color);
        }

        display.fillRoundRect(10, 34, SCREEN_W - 20, 170, 12, COLOR_CARD);
        display.drawRoundRect(10, 34, SCREEN_W - 20, 170, 12, COLOR_ACCENT);
        display.setTextColor(COLOR_ACCENT, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("< SWIPE OR USR2 >", SCREEN_W / 2, 48, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        display.drawCentreString(modeTitle(selectedMode_), SCREEN_W / 2, 84, 1);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString(modeSubtitle(selectedMode_), SCREEN_W / 2, 128, 1);
        display.drawCentreString(modeMountHint(selectedMode_), SCREEN_W / 2, 152, 1);

        if (webUi_.isRunning())
        {
            display.setTextColor(COLOR_GOOD, COLOR_CARD);
            display.drawCentreString("WIFI ON: vbt.local", SCREEN_W / 2, 180, 1);
        }

        display.fillRoundRect(14, 225, SCREEN_W - 28, 65, 10, COLOR_START_BTN);
        display.drawRoundRect(14, 225, SCREEN_W - 28, 65, 10, COLOR_GOOD);
        display.setTextColor(COLOR_TEXT, COLOR_START_BTN);
        display.setTextSize(2);
        display.drawCentreString("START", SCREEN_W / 2, 242, 1);
        display.setTextSize(1);
        display.drawCentreString("TAP OR PRESS USR1", SCREEN_W / 2, 268, 1);
    }

    void UiManager::renderSettingsCard()
    {
        display.fillScreen(COLOR_BG);

        for (int i = 0; i < CARD_COUNT; ++i)
        {
            const int16_t cx = 36 + i * 20;
            const uint16_t color = (i == selectedCard_) ? COLOR_ACCENT : COLOR_CARD;
            display.fillCircle(cx, 18, 4, color);
        }

        const bool wifiOn = webUi_.isRunning();

        display.fillRoundRect(10, 32, SCREEN_W - 20, 152, 12, COLOR_CARD);
        display.drawRoundRect(10, 32, SCREEN_W - 20, 152, 12, wifiOn ? COLOR_GOOD : COLOR_ACCENT);
        display.setTextColor(COLOR_ACCENT, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("< SWIPE OR USR2 >", SCREEN_W / 2, 44, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        display.drawCentreString("SETTINGS", SCREEN_W / 2, 66, 1);
        display.setTextSize(1);

        if (wifiOn)
        {
            display.setTextColor(COLOR_GOOD, COLOR_CARD);
            display.drawCentreString("PHONE WEB UI: ON", SCREEN_W / 2, 102, 1);
            display.setTextColor(COLOR_TEXT, COLOR_CARD);
            display.drawCentreString("WiFi: VBT-Tracker", SCREEN_W / 2, 124, 1);
            display.drawCentreString("Pass: 12345678", SCREEN_W / 2, 142, 1);
            display.setTextColor(COLOR_ACCENT, COLOR_CARD);
            display.drawCentreString("http://vbt.local", SCREEN_W / 2, 162, 1);
        }
        else
        {
            display.setTextColor(COLOR_MUTED, COLOR_CARD);
            display.drawCentreString("PHONE WEB UI: OFF", SCREEN_W / 2, 108, 1);
            display.drawCentreString("Wi-Fi radio powered off", SCREEN_W / 2, 134, 1);
            display.drawCentreString("Max battery life", SCREEN_W / 2, 154, 1);
        }

        // Button 1: Toggle Phone Wi-Fi
        const uint16_t btnBg = wifiOn ? COLOR_STOP_BG : COLOR_START_BTN;
        const uint16_t btnBorder = wifiOn ? COLOR_WARN : COLOR_GOOD;
        display.fillRoundRect(14, 194, SCREEN_W - 28, 50, 10, btnBg);
        display.drawRoundRect(14, 194, SCREEN_W - 28, 50, 10, btnBorder);
        display.setTextColor(COLOR_TEXT, btnBg);
        display.setTextSize(2);
        display.drawCentreString(wifiOn ? "WIFI OFF" : "WIFI ON", SCREEN_W / 2, 204, 1);
        display.setTextSize(1);
        display.drawCentreString("TAP OR USR1", SCREEN_W / 2, 228, 1);

        // Button 2: Deep Sleep (wake with USR1)
        display.fillRoundRect(14, 254, SCREEN_W - 28, 50, 10, COLOR_STOP_BG);
        display.drawRoundRect(14, 254, SCREEN_W - 28, 50, 10, COLOR_WARN);
        display.setTextColor(COLOR_TEXT, COLOR_STOP_BG);
        display.setTextSize(2);
        display.drawCentreString("SLEEP", SCREEN_W / 2, 264, 1);
        display.setTextSize(1);
        display.drawCentreString("WAKE: PRESS USR1", SCREEN_W / 2, 288, 1);
    }

    void UiManager::renderCalibratingScreen()
    {
        display.fillScreen(COLOR_BG);
        display.drawRoundRect(10, 40, SCREEN_W - 20, 240, 12, COLOR_WARN);
        display.setTextColor(COLOR_WARN, COLOR_BG);
        display.setTextSize(2);
        display.drawCentreString("HOLD STILL", SCREEN_W / 2, 110, 1);
        display.setTextColor(COLOR_TEXT, COLOR_BG);
        display.setTextSize(1);
        display.drawCentreString("Calibrating IMU (1s)...", SCREEN_W / 2, 155, 1);
        display.drawCentreString("Do not move sensor", SCREEN_W / 2, 180, 1);
    }

    void UiManager::renderRecordingScreen()
    {
        display.fillScreen(COLOR_STOP_BG);
        display.setTextColor(COLOR_GOOD, COLOR_STOP_BG);
        display.setTextSize(2);
        display.drawCentreString("RECORDING", SCREEN_W / 2, 40, 1);
        display.setTextColor(COLOR_TEXT, COLOR_STOP_BG);
        display.setTextSize(1);
        display.drawCentreString(modeTitle(selectedMode_), SCREEN_W / 2, 80, 1);
        display.fillRoundRect(14, 125, SCREEN_W - 28, 120, 10, COLOR_BG);
        display.drawRoundRect(14, 125, SCREEN_W - 28, 120, 10, COLOR_WARN);
        display.setTextColor(COLOR_WARN, COLOR_BG);
        display.drawCentreString("SCREEN FROZEN", SCREEN_W / 2, 148, 1);
        display.drawCentreString("500 Hz IMU ACTIVE", SCREEN_W / 2, 168, 1);
        display.setTextColor(COLOR_TEXT, COLOR_BG);
        display.drawCentreString("Press USR1 button", SCREEN_W / 2, 198, 1);
        display.drawCentreString("when set is done", SCREEN_W / 2, 216, 1);
    }

    void UiManager::renderResultsScreen(const ActivityOutput& result)
    {
        display.fillScreen(COLOR_BG);

        if (result.mode == ActivityMode::BarbellVbt)
        {
            renderBarbellResults(result);
        }
        else if (result.mode == ActivityMode::StandingJump)
        {
            renderJumpResults(result);
        }
        else if (result.mode == ActivityMode::HurdleContact)
        {
            renderHurdleResults(result);
        }
        else if (result.mode == ActivityMode::SprintContact)
        {
            renderSprintResults(result);
        }
        else
        {
            display.setTextColor(COLOR_ACCENT, COLOR_BG);
            display.setTextSize(2);
            display.drawCentreString(modeTitle(result.mode), SCREEN_W / 2, 30, 1);
            display.setTextColor(COLOR_TEXT, COLOR_BG);
            display.setTextSize(1);
            display.drawCentreString("Mode skeleton ready", SCREEN_W / 2, 130, 1);
        }

        display.fillRoundRect(6, 262, 78, 50, 8, COLOR_START_BTN);
        display.setTextColor(COLOR_TEXT, COLOR_START_BTN);
        display.setTextSize(1);
        display.drawCentreString("NEXT SET", 45, 275, 1);
        display.drawCentreString("(USR1)", 45, 291, 1);
        display.fillRoundRect(88, 262, 78, 50, 8, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.drawCentreString("MODES", 127, 275, 1);
        display.drawCentreString("(USR2)", 127, 291, 1);
    }

    void UiManager::renderBarbellResults(const ActivityOutput& result)
    {
        char buf[36];
        display.setTextColor(COLOR_ACCENT, COLOR_BG);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "SET %lu (%lu REPS) LOSS %.0f%%", static_cast<unsigned long>(result.set_number), static_cast<unsigned long>(result.rep.rep_number), result.rep.velocity_loss_percent);
        display.drawCentreString(buf, SCREEN_W / 2, 10, 1);

        if (result.rep.rep_number == 0)
        {
            display.setTextColor(COLOR_WARN, COLOR_BG);
            display.setTextSize(2);
            display.drawCentreString("0 REPS", SCREEN_W / 2, 100, 1);
            display.setTextSize(1);
            display.drawCentreString("Min depth: 15 cm", SCREEN_W / 2, 140, 1);
            return;
        }

        display.fillRoundRect(8, 26, SCREEN_W - 16, 84, 8, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("BEST MEAN VEL (m/s)", SCREEN_W / 2, 32, 1);
        display.setTextColor(COLOR_GOOD, COLOR_CARD);
        display.setTextSize(3);

        snprintf(buf, sizeof(buf), "%.2f", result.rep.best_mean_velocity_mps);
        display.drawCentreString(buf, SCREEN_W / 2, 50, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "AVG:%.2f PK:%.2f ROM:%.0fcm", result.rep.set_mean_velocity_mps, result.rep.peak_concentric_velocity_mps, result.rep.range_of_motion_m * 100.0f);
        display.drawCentreString(buf, SCREEN_W / 2, 92, 1);
        display.setTextColor(COLOR_TEXT, COLOR_BG);

        const uint32_t count = result.rep.rep_number < RepMetrics::MAX_SET_REPS ? result.rep.rep_number : RepMetrics::MAX_SET_REPS;

        for (uint32_t i = 0; i < count; ++i)
        {
            const int16_t col = (i % 2 == 0) ? 12 : 94;
            const int16_t row = 120 + static_cast<int16_t>(i / 2) * 26;
            snprintf(buf, sizeof(buf), "R%lu: %.2f", static_cast<unsigned long>(i + 1), result.rep.rep_mean_velocity_mps[i]);
            display.drawString(buf, col, row);
        }
    }

    void UiManager::renderJumpResults(const ActivityOutput& result)
    {
        char buf[36];
        display.setTextColor(COLOR_ACCENT, COLOR_BG);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "JUMP SET %lu (%lu JUMPS)", static_cast<unsigned long>(result.set_number), static_cast<unsigned long>(result.jump.jump_number));
        display.drawCentreString(buf, SCREEN_W / 2, 10, 1);

        if (result.jump.jump_number == 0)
        {
            display.setTextColor(COLOR_WARN, COLOR_BG);
            display.setTextSize(2);
            display.drawCentreString("0 JUMPS", SCREEN_W / 2, 100, 1);
            display.setTextSize(1);
            display.drawCentreString("Stand still before jump", SCREEN_W / 2, 140, 1);
            return;
        }

        display.fillRoundRect(8, 26, SCREEN_W - 16, 84, 8, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("BEST JUMP HEIGHT (cm)", SCREEN_W / 2, 32, 1);
        display.setTextColor(COLOR_GOOD, COLOR_CARD);
        display.setTextSize(3);

        snprintf(buf, sizeof(buf), "%.1f", result.jump.best_jump_height_cm);
        display.drawCentreString(buf, SCREEN_W / 2, 50, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "AVG:%.1fcm  FLIGHT:%.0fms", result.jump.mean_jump_height_cm, result.jump.flight_time_ms);

        display.drawCentreString(buf, SCREEN_W / 2, 92, 1);
        display.setTextColor(COLOR_TEXT, COLOR_BG);
        const uint32_t count = result.jump.jump_number < JumpMetrics::MAX_SET_JUMPS ? result.jump.jump_number : JumpMetrics::MAX_SET_JUMPS;

        for (uint32_t i = 0; i < count; ++i)
        {
            const int16_t col = (i % 2 == 0) ? 12 : 94;
            const int16_t row = 124 + static_cast<int16_t>(i / 2) * 28;
            snprintf(buf, sizeof(buf), "J%lu: %.1fcm", static_cast<unsigned long>(i + 1), result.jump.jump_heights_cm[i]);
            display.drawString(buf, col, row);
        }
    }

    void UiManager::renderHurdleResults(const ActivityOutput& result)
    {
        char buf[36];
        display.setTextColor(COLOR_ACCENT, COLOR_BG);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "HURDLES %lu (%lu HOPS)",
            static_cast<unsigned long>(result.set_number),
            static_cast<unsigned long>(result.hurdle.contact_number));
        display.drawCentreString(buf, SCREEN_W / 2, 10, 1);

        if (!result.hurdle.valid || result.hurdle.contact_number == 0)
        {
            display.setTextColor(COLOR_WARN, COLOR_BG);
            display.setTextSize(2);
            display.drawCentreString("0 HOPS", SCREEN_W / 2, 100, 1);
            display.setTextSize(1);
            display.drawCentreString("Hop over hurdles & land", SCREEN_W / 2, 140, 1);
            return;
        }

        // Hero card: Best RSI + Mean RSI & Min GCT
        display.fillRoundRect(8, 26, SCREEN_W - 16, 84, 8, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("BEST RSI (m/s)", SCREEN_W / 2, 32, 1);
        display.setTextColor(COLOR_GOOD, COLOR_CARD);
        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%.2f", result.hurdle.best_rsi);
        display.drawCentreString(buf, SCREEN_W / 2, 50, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "AVG:%.2f  MIN:%.0fms",
            result.hurdle.mean_rsi, result.hurdle.min_contact_time_ms);
        display.drawCentreString(buf, SCREEN_W / 2, 92, 1);

        // 2x2 mini-cards: Mean GCT, Min GCT, Last Height, Last Flight
        display.fillRoundRect(8, 120, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.drawCentreString("AVG GCT", 45, 128, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.0f", result.hurdle.mean_contact_time_ms);
        display.drawCentreString(buf, 45, 150, 1);

        display.fillRoundRect(89, 120, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("MIN GCT", 126, 128, 1);
        display.setTextColor(COLOR_GOOD, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.0f", result.hurdle.min_contact_time_ms);
        display.drawCentreString(buf, 126, 150, 1);

        display.fillRoundRect(8, 190, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("HEIGHT cm", 45, 198, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f", result.hurdle.jump_height_cm);
        display.drawCentreString(buf, 45, 220, 1);

        display.fillRoundRect(89, 190, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("FLIGHT ms", 126, 198, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.0f", result.hurdle.flight_time_ms);
        display.drawCentreString(buf, 126, 220, 1);
    }

    void UiManager::renderSprintResults(const ActivityOutput& result)
    {
        char buf[36];
        display.setTextColor(COLOR_ACCENT, COLOR_BG);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "SPRINT %lu (%lu STEPS)", static_cast<unsigned long>(result.set_number), static_cast<unsigned long>(result.sprint.step_count));
        display.drawCentreString(buf, SCREEN_W / 2, 10, 1);

        if (!result.sprint.valid || result.sprint.step_count == 0)
        {
            if (result.gates.count >= 2)
            {
                display.fillRoundRect(8, 56, SCREEN_W - 16, 94, 8, COLOR_CARD);
                display.setTextColor(COLOR_MUTED, COLOR_CARD);
                display.setTextSize(1);
                display.drawCentreString("GATE SPLIT (s)", SCREEN_W / 2, 66, 1);
                display.setTextColor(COLOR_GOOD, COLOR_CARD);
                display.setTextSize(3);
                snprintf(buf, sizeof(buf), "%.3f", result.gates.total_seconds);
                display.drawCentreString(buf, SCREEN_W / 2, 92, 1);
                return;
            }
            display.setTextColor(COLOR_WARN, COLOR_BG);
            display.setTextSize(2);
            display.drawCentreString("0 STEPS", SCREEN_W / 2, 100, 1);
            display.setTextSize(1);
            display.drawCentreString("Min 4 sprint steps", SCREEN_W / 2, 140, 1);
            return;
        }

        // Hero card: Mean GCT + Min GCT & Mean Flight
        display.fillRoundRect(8, 26, SCREEN_W - 16, 84, 8, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("MEAN GCT (ms)", SCREEN_W / 2, 32, 1);
        display.setTextColor(COLOR_GOOD, COLOR_CARD);
        display.setTextSize(3);
        snprintf(buf, sizeof(buf), "%.0f", result.sprint.mean_contact_time_ms);
        display.drawCentreString(buf, SCREEN_W / 2, 50, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(1);
        snprintf(buf, sizeof(buf), "MIN:%.0fms  FLT:%.0fms", result.sprint.min_contact_time_ms, result.sprint.mean_flight_time_ms);
        display.drawCentreString(buf, SCREEN_W / 2, 92, 1);

        // 2x2 mini-cards: Step Freq, Asymmetry, Drift, Last Step
        display.fillRoundRect(8, 120, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("FREQ (Hz)", 45, 128, 1);
        display.setTextColor(COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.2f", result.sprint.step_frequency_hz);
        display.drawCentreString(buf, 45, 150, 1);

        display.fillRoundRect(89, 120, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("ASYM (%)", 126, 128, 1);
        display.setTextColor(result.sprint.odd_even_asymmetry_percent > 5.0f ? COLOR_WARN : COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%.1f", result.sprint.odd_even_asymmetry_percent);
        display.drawCentreString(buf, 126, 150, 1);

        display.fillRoundRect(8, 190, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        display.drawCentreString("DRIFT ms", 45, 198, 1);
        display.setTextColor(result.sprint.contact_drift_ms > 5.0f ? COLOR_WARN : COLOR_TEXT, COLOR_CARD);
        display.setTextSize(2);
        snprintf(buf, sizeof(buf), "%+.0f", result.sprint.contact_drift_ms);
        display.drawCentreString(buf, 45, 220, 1);

        display.fillRoundRect(89, 190, 75, 62, 6, COLOR_CARD);
        display.setTextColor(COLOR_MUTED, COLOR_CARD);
        display.setTextSize(1);
        if (result.gates.count >= 2)
        {
            display.drawCentreString("GATE (s)", 126, 198, 1);
            display.setTextColor(COLOR_GOOD, COLOR_CARD);
            display.setTextSize(2);
            snprintf(buf, sizeof(buf), "%.2f", result.gates.total_seconds);
            display.drawCentreString(buf, 126, 220, 1);
        }
        else
        {
            display.drawCentreString("LAST ms", 126, 198, 1);
            display.setTextColor(COLOR_TEXT, COLOR_CARD);
            display.setTextSize(2);
            snprintf(buf, sizeof(buf), "%.0f", result.sprint.last_contact_time_ms);
            display.drawCentreString(buf, 126, 220, 1);
        }
    }
}

