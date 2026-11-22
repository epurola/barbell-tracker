#include <Arduino.h>
#include "runtime/RuntimeTasks.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    if (!vbt::runtime::initializeImu())
    {
        Serial.println("ERROR: LSM6DS3 initialization failed. Check I2C wiring, address, and sensor configuration.");
        while (true) delay(1000);
    }

    if (!vbt::runtime::startTasks())
    {
        Serial.println("ERROR: Failed to create runtime tasks");
        while (true) delay(1000);
    }
}

void loop()
{
    // Suspend Arduino's Core 1 loopTask so Core 1 is 100% dedicated to acquisitionTask.
    vTaskDelay(portMAX_DELAY);
}