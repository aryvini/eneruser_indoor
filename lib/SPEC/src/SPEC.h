#ifndef SPEC_H
#define SPEC_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "SoftwareSerial.h"

class SpecSensor
{

public:
    String sensor_serialNo;
    int ppb;
    int temp;
    int humi;
    int raw_sensor;
    int temp_digital;
    int humi_digital;
    int days;
    int hours;
    int minutes;
    int seconds;

    // Constructor of the class
    SpecSensor(Stream &port, int baud);

    // Function that initiate the sensor
    // flush EPROOM
    // Set the serial port of the object
    bool init();

    //Request data from the sensor
    bool request_data(void);

    //Request EPROM stored parameters
    //Print on the serial the informatio of the sensor
    bool sensor_info(void);

    //Print on time

private:
    int _baudrate;
    Stream *_sensor_serial; //Changed to Stream in oder to accept both Software and Hardware serial ports

    //Flush data left on sensor's serial
    void _flush_serial(void);

    //Parse data to the variables
    void _parse_data(String rawdata);
};

#endif