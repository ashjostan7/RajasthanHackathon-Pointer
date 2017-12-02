
#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver;

void setup()
{
    Serial.begin(74880); // Debugging only
    if (!driver.init())
         Serial.println("init failed");
    pinMode(5, OUTPUT);
}
int state=0;
void loop()
{
    uint8_t buf[40];
    uint8_t buflen = sizeof(buf);
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      int i;
      Serial.println((char*)buf);    
    }
}

