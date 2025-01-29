#include <Wire.h>

const uint8_t smcDeviceNumber = 15;
// Required to allow motors to move.
// Must be called when controller restarts and after any error.

void exitSafeStart()
{
Wire.beginTransmission(smcDeviceNumber);
Wire.write(0x83); // Exit safe start
Wire.endTransmission();
}
void setMotorSpeed(int16_t speed)
{
uint8_t cmd = 0x85; // Motor forward
if (speed < 0)
{
cmd = 0x86; // Motor reverse
speed = -speed;
}
Wire.beginTransmission(smcDeviceNumber);
Wire.write(cmd);
Wire.write(speed & 0x1F);
Wire.write(speed >> 5 & 0x7F);
Wire.endTransmission();
}
uint16_t readUpTime()
{
Wire.beginTransmission(smcDeviceNumber);
Wire.write(0xA1); // Command: Get variable
Wire.write(28); // Variable ID: Up time (low)
Wire.endTransmission();
Wire.requestFrom(smcDeviceNumber, (uint8_t)2);
uint16_t upTime = Wire.read();
upTime |= Wire.read() << 8;
return upTime;
}
void setup()
{
Wire.begin();
exitSafeStart();
}
void loop()
{
// Read the up time from the controller and send it to
// the serial monitor.
uint16_t upTime = readUpTime();
Serial.print(F("Up time: "));
Serial.println(upTime);
setMotorSpeed(3200); // full-speed forward
delay(50);
}