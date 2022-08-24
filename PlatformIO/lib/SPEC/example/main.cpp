#include <Arduino.h>
#include <SPEC.h>
#include <SoftwareSerial.h>

//Create a SpecSensor object
// by specifying the serial port connection and the baudrate
SpecSensor o3(Serial2, 9600);
SpecSensor co(Serial1, 9600);

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(9600);

  o3.init();
  co.init();
}

void loop()
{
  // put your main code here, to run repeatedly:

  if (o3.request_data() & co.request_data())
  {
    Serial.println("O3 measurements");
    Serial.print("PPB: ");
    Serial.println(o3.ppb);

    Serial.print("Temp: ");
    Serial.println(o3.temp);

    Serial.print("Hum: ");
    Serial.println(o3.humi);

    Serial.print("On time: ");
    Serial.print(o3.days);
    Serial.print("days ");
    Serial.print(o3.hours);
    Serial.print("h ");
    Serial.print(o3.minutes);
    Serial.print("m ");
    Serial.print(o3.seconds);
    Serial.println("s ");

    Serial.println("");

    Serial.println("CO measurements");
    Serial.print("PPB: ");
    Serial.println(co.ppb);

    Serial.print("Temp: ");
    Serial.println(co.temp);

    Serial.print("Hum: ");
    Serial.println(co.humi);

    Serial.print("On time: ");
    Serial.print(co.days);
    Serial.print("days ");
    Serial.print(co.hours);
    Serial.print("h ");
    Serial.print(co.minutes);
    Serial.print("m ");
    Serial.print(co.seconds);
    Serial.println("s ");

    Serial.println("------------------------");
  }
  else
  {
    Serial.println("Could not request data");
  }

  delay(10000);
}