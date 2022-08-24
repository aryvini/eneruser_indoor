
/*
    SPEC.cpp - Library to interface arduino mega and SPEC sensors
    Created by Ary Vinicius Nervis Frigeri, April 2021
    Polytechnic Institute of Bragança (IPB) - Portugal
*/

#include "Arduino.h"
#include "SPEC.h"


SpecSensor::SpecSensor(Stream &port, int baud)
{
    //Initializate class variables
    ppb = 0;
    temp = 0;
    humi = 0;
    raw_sensor = 0;
    temp_digital = 0;
    humi_digital = 0;
    days = 0;
    hours = 0;
    minutes = 0;
    seconds = 0;

    _sensor_serial = &port;
    _baudrate = baud;
}

bool SpecSensor::init()
{

    Serial.println(F("Initializing SPEC sensor"));
    // _sensor_serial->begin(_baudrate); //deactivated because it was changed to Stream object
    Serial.println(F("Flushing the serial port of the sensor"));
    this->_flush_serial();
    Serial.println(F("Waiting sensor response"));
    if (!this->sensor_info())
    {
        Serial.println(F("Initialization failed"));
        return false;
    }

    Serial.println(F("SPEC sensor initialization finished"));
    Serial.println(F("---------------------"));

    return true;
}

bool SpecSensor::sensor_info()
{

    String data_string;
    String response_string;
    // First we trigger a measurement
    _sensor_serial->print(" ");
    // Within one second time we send the command "e"
    delay(1000);
    _sensor_serial->print("e");
    delay(100);
    //wait for the response from the sensor, if not responding, return
    for (int i = 0; i < 10; i++)
    {
        if (_sensor_serial->available() > 0)
        {
            delay(20);
            // data_string = _sensor_serial->readStringUntil('\n');
            // You can uncomment this line if you wish
            //Serial.println(data_string);
            while (_sensor_serial->available() > 0)
            {
                response_string = _sensor_serial->readStringUntil('\n');
                Serial.println(response_string);
                delay(20);
            }
            return true;
        }
        else
        {
            Serial.println(F("Waiting sensor response"));
        }

        delay(500);
    }

    Serial.println(F("Sensor not responding"));
    return false;
}

void SpecSensor::_flush_serial()
{

    char in_char;

    // Do we have data in the serial buffer?
    // If so, flush it
    if (_sensor_serial->available() > 0)
    {
        Serial.println(F("Flushing serial buffer..."));
        while (1)
        {
            in_char = (char)_sensor_serial->read();
            delay(10);
            Serial.print(in_char);
            if (in_char == '\n')
                break;
        }
        Serial.println(" ");
        Serial.println(F("Buffer flushed!"));
    }
}

bool SpecSensor::request_data()
{

    //Reset class variables
    ppb = 0;
    temp = 0;
    humi = 0;
    raw_sensor = 0;
    temp_digital = 0;
    humi_digital = 0;
    days = 0;
    hours = 0;
    minutes = 0;
    seconds = 0;

    String data_string;
    // First, we do some initialization
    // Clear the data string
    data_string = "";
    // Now we trigger a measurement
    _sensor_serial->print("\r");
    delay(200);
    // We wait for the sensor to respond
    data_string = _sensor_serial->readStringUntil('\n');
    //Serial.println(dataString);

    if (data_string.length() <= 1)
    {
        return false;
    }
    else
    {

        this->_parse_data(data_string);
        return true;
    }
}

void SpecSensor::_parse_data(String rawdata)
{
    // Parses the received dataString
    // Data string is comma separated
    // The format of the output is: SN[XXXXXXXXXXXX], PPB [0 : 999999], TEMP [-99:99],
    // RH[0:99], RawSensor1[ADCCount], TempDigital, RHDigital, Day[0:99], Hour [0:23],
    // Minute[0:59], Second[0 : 59]\r\n
    // Take a look also at
    // https://stackoverflow.com/questions/11068450/arduino-c-language-parsing-string-with-delimiter-input-through-serial-interfa
    // We look first for the SN

    String S_gas, S_temp, S_humi;
    String S_raw_gas, S_Tdigital, S_RHdigital;
    String S_Days, S_Hours, S_Minutes, S_Seconds;

    int idx1 = rawdata.indexOf(',');
    sensor_serialNo = rawdata.substring(0, idx1);

    int idx2 = rawdata.indexOf(',', idx1 + 1);
    // Hint: after comma there's a space - it should be ignored
    S_gas = rawdata.substring(idx1 + 2, idx2);
    ppb = S_gas.toInt();

    int idx3 = rawdata.indexOf(',', idx2 + 1);
    S_temp = rawdata.substring(idx2 + 2, idx3);
    temp = S_temp.toInt();

    int idx4 = rawdata.indexOf(',', idx3 + 1);
    S_humi = rawdata.substring(idx3 + 2, idx4);
    humi = S_humi.toInt();

    int idx5 = rawdata.indexOf(',', idx4 + 1);
    S_raw_gas = rawdata.substring(idx4 + 2, idx5);
    raw_sensor = S_raw_gas.toInt();

    int idx6 = rawdata.indexOf(',', idx5 + 1);
    S_Tdigital = rawdata.substring(idx5 + 2, idx6);
    temp_digital = S_Tdigital.toInt();

    int idx7 = rawdata.indexOf(',', idx6 + 1);
    S_RHdigital = rawdata.substring(idx6 + 2, idx7);
    humi_digital = S_RHdigital.toInt();

    int idx8 = rawdata.indexOf(',', idx7 + 1);
    S_Days = rawdata.substring(idx7 + 2, idx8);
    days = S_Days.toInt();

    int idx9 = rawdata.indexOf(',', idx8 + 1);
    S_Hours = rawdata.substring(idx8 + 2, idx9);
    hours = S_Hours.toInt();

    int idx10 = rawdata.indexOf(',', idx9 + 1);
    S_Minutes = rawdata.substring(idx9 + 2, idx10);
    minutes = S_Minutes.toInt();

    int idx11 = rawdata.indexOf('\r');
    S_Seconds = rawdata.substring(idx10 + 2, idx11);
    seconds = S_Seconds.toInt();
}
