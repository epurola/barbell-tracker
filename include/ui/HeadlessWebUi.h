#pragma once

namespace vbt::ui
{
    class HeadlessWebUi
    {
    public:
        void begin();
        void end();
        void update();
        bool isRunning() const { return running_; }

    private:
        bool running_{false};
    };
}
