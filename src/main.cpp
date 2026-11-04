#include <Arduino.h>
#include "runtime/RuntimeTasks.h"

void setup()
{
    Serial.begin(115200);

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("VBT Runtime Architecture");
    Serial.println("================================");

     Serial.printf("setup running on Core %d\n", xPortGetCoreID());

     Serial.println("initializing IMU...");

     if(!vbt::runtime::initializeImu())
     {
        Serial.println("ERROR: LSM6DS3 initialization failed. Check I2C wiring, address, and sensor configuration.");

        while(true)
        {
            delay(1000);
        }
     }

     Serial.println("LSM6DS3 initialized successfully.");

     if (!vbt::runtime::startTasks())
     {
        Serial.println("ERROR: Failed to create runtime tasks");

        while (true)
        {
            delay(1000);
        }
     }

    Serial.println("Runtime tasks started");

}

void loop()
{
    delay(1000);
    Serial.println("Alive");
}