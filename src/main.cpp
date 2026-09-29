#include <Arduino.h>

void setup()
{
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("VBT Barbell Tracker");
    Serial.println("Firmware boot OK");
    Serial.println("C++17 runtime initialized");
}

void loop()
{
    delay(1000);
    Serial.println("Alive");
}
