#pragma once
#include <Arduino.h>
#include <cstdint>
#include "contracts/ModeTypes.h"
#include "ui/HeadlessWebUi.h"

#ifndef D15
#define D15 42
#endif

#ifndef D19
#define D19 11
#endif

namespace vbt::ui
{
    class UiManager
    {
        public:
            static constexpr int8_t PIN_LCD_RST  = 13;  // D17 on XIAO ESP32-S3 Plus
            static constexpr int8_t PIN_LCD_BL   = 12;  // D18 on XIAO ESP32-S3 Plus
            static constexpr int8_t PIN_TOUCH_IRQ = D7;
            static constexpr int PIN_BTN_USR1    = D19; // Primary: START / STOP / NEXT SET
            static constexpr int PIN_BTN_USR2    = D15; // Secondary: Next Mode / Back to Modes

            bool begin();

            // Call periodically (~30-50 Hz) on Core 0.
            void update();

        private:
            void pollButtons();
            void pollTouch(SessionState state);
            void handlePrimaryAction(SessionState state);
            void handleSecondaryAction(SessionState state);
            void cycleMode(int delta);
            void renderModeSelectCard();
            void renderSettingsCard();
            void renderCalibratingScreen();
            void renderRecordingScreen();
            void renderResultsScreen(const ActivityOutput& result);
            void renderBarbellResults(const ActivityOutput& result);
            void renderJumpResults(const ActivityOutput& result);
            void renderSprintResults(const ActivityOutput& result);

            void startWebUI();
            void stopWebUI();
            bool isSettingsCard() const { return selectedCard_ == SETTINGS_CARD_INDEX; }

            static constexpr int MODE_COUNT = 5;
            static constexpr int SETTINGS_CARD_INDEX = MODE_COUNT;
            static constexpr int CARD_COUNT = MODE_COUNT + 1;

            HeadlessWebUi webUi_{};
            int selectedCard_{0};
            ActivityMode selectedMode_{ActivityMode::BarbellVbt};
            SessionState lastRenderedState_{SessionState::ResultsReady};
            int lastRenderedCard_{-1};
            bool forceRedraw_{true};

            bool prevUsr1Pressed_{false};
            bool prevUsr2Pressed_{false};
            uint32_t lastButtonEventMs_{0};

            bool touchActive_{false};
            int32_t touchStartX_{0};
            int32_t touchStartY_{0};
            int32_t touchLastX_{0};
            int32_t touchLastY_{0};
    };
}