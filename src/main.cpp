#include <Arduino.h>
#include "runtime/RuntimeTasks.h"
#ifndef UNIT_TEST
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
#endif // UNIT_TEST