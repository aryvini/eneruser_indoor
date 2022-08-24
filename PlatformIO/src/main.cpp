#include <Arduino.h>

// ARDUINO LIBRARIES -----------------------------------------------------------------------------------------------

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <sps30.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <SPEC.h>
#include <WiFiEspAT.h>
#include <Statistic.h>
#include <NTPClient.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
//####################################
//############### OBS ################
//####################################
/*
WiFiEspAT --> changed WIFIESPAT_CLIENT_TX_BUFFER_SIZE to 128 bytes in WiFiEspAtConfig.h
WiFiEspAT --> changed WIFIESPAT_CLIENT_RX_BUFFER_SIZE to 128 bytes in WiFiEspAtConfig.h

PubSubClient --> changed MAX PACKET SIZE to 256
*/

// Set this to true to debug
#define PRINT_MILLIS false
#define SERIAL_PRINT false


// Include utils.h to instantiate functions
#include "utils.h"

//BLINK LED TYPES
#define LED_PIN           3 //PIN NUM or 0 to false

//Do not chage this numbers
#define LED_OFF           0
#define LED_DATASENT      1
#define LED_COHIGH        2
#define LED_ERROR         3


//####################################
//####### OLED CONFIGURATION #########
//####################################

// Insert oled config file
// Configures the ports and screen size
#include "OLEDconfig.h"

//Create the display object, based on the configuration file
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&Wire,OLED_RESET);

// IMAGES BITMAPS
// #include "bitmaps.h"

//####################################

//####################################
//####### Variables definitions ######
//####################################

float BMEtemp, BMEpress, BMEhum;                       //BME outputs after filter
float SPECco;                                          //SPECs outputs after filter
float DS18temp;                                        //DS18B20 output after filter
float SPSmassPM1, SPSmassPM2, SPSmassPM4, SPSmassPM10; //SPS outputs
float SPSnumPM1, SPSnumPM2, SPSnumPM4, SPSnumPM10;     //SPS
float SPSpartSize;                                     //SPS

Statistic SPSmassPM1avg, SPSmassPM2avg, SPSmassPM4avg, SPSmassPM10avg;  //outputs after avg
Statistic SPSnumPM1avg, SPSnumPM2avg, SPSnumPM4avg, SPSnumPM10avg;  //outputs after avg
Statistic SPSpartSizeavg; //outputs after avg
//Weights for the leaky filter
//Val = val * previous_weight + new measure * current_weight
float previous_weight = 0.7;
float current_weight = 0.3;

//####################################

//####################################
//####### Sensors Instances ##########
//####################################

//BME - I2C
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;


//SPEC sensors - Serial
SpecSensor co(Serial3, 9600);

//SPS
//define serial communication channel to use for SPS30
#define DEBUG 2
#define SP30_COMMS Serial1
SPS30 sps30;
struct sps_values SPSvalues; //struct that will contain the SPS reads

//DS18B20
//Define the OneWire pin
#define ONE_WIRE_BUS 38 //DS18B20 pin on the arduino
OneWire oneWire(ONE_WIRE_BUS); //Instantiate a OneWire object
DallasTemperature ds18(&oneWire); //Pass the OneWirte object as a reference to DallasTemperature
float ds18read; //variable to store the current reading of the DS18 object

//####################################

//####################################
//####### Auxiliary Modules ##########
//####################################

//RTC clock
RTC_DS3231 rtc; //Criação do objeto do tipo DS3231 I2C
DateTime now;   //Criação do objeto do tipo RTCDateTime
DateTime nextmeasuretime;
int loginterval = 1; //defines the interval in minutes in which the data will be stored in the SD card and reported to the cloud

//SDcard
File tabulardata; // Declaration the variable as a File
const int chipSelect = 53; // Declaration the output pin for the SD module (CS)
//####################################

//####################################
//####### WIFI RELATED CODE ##########
//####################################

//passwords and keys added to secrets.h
#include <secrets.h>







// Initialize the Wifi client library
WiFiClient client;
WiFiUdpSender udpSender;
WiFiUDP udpTime;

//PING TEST
IPAddress google(8,8,8,8);
DateTime nextpingtest;

const byte ESP_RESET_PIN = 7;

//NTP client
const char* NTPSERVER = "pt.pool.ntp.org";
NTPClient timeClient(udpTime,NTPSERVER,0,60000);
DateTime ntptime;
DateTime nextclockupdate;


String firstboot = "TRUE";

//####################################
// SETUP PROGRAM ---------------------------------------------------------------------------------------------------

void setup()
{
  //Serial Monitor start
  Serial.begin(9600);

  //Serial port for ESP module
  Serial2.begin(9600);

  //rtc.adjust(DateTime(F(__DATE__), F("15:29:30"))); 

  //Start of the serial ports of the sensors
  SP30_COMMS.begin(115200);
  //Since the SPEC library changed to accept hardware and software serial ports,
  //The initialization of the port must be done mannually. Before it was done by the library itself
  Serial3.begin(9600);  //SPEC CO

  delay(500);

  OLED_start(); // Starting the OLED display
  SD_start();   // Starting the micro-SD module

  delay(2000);

  //Begin ESP module
  ESP_start();
  wifi_connect();

  delay(500);
  
  //Adjust RTC on every start based on NTP time
  adjust_RTC(); 

  sensors_start(); //Start all sensors

  
  // SPS_set_autoclean(86400); //set autoclean interval, run once for each SPS sensor, this is remembered after power off
  // SPS_clean_now(); //manually clean the sensor
  //delay(5000);

  //lED

  #if LED_PIN != 0

    pinMode(LED_PIN,OUTPUT);

  #endif

  delay(500);

  zero_variables();                             //set all variable to zero before sensors readings
  now = rtc.now();                              // read the time right now
  nextmeasuretime = now + TimeSpan(0, 0, 2, (60-now.second())); // set the first measurement time
  nextclockupdate = now + TimeSpan(0,6,0,0); // set when the RTC clock will be updated
  nextpingtest = now + TimeSpan(0,0,5,0); // set when the ping test will be executed

//Activate watch dog timer

wdt_enable(WDTO_8S);
}

// MAIN PROGRAM ----------------------------------------------------------------------------------------------------

void loop()
{

  print_millis(1);
  #if SERIAL_PRINT
  Serial.println(F("Reading all sensors"));
  #endif
  read_sensors();

  print_millis(2);
  wdt_reset();

  delay(100);
  

  // Print to OLED
  OLED_readings_print();
  print_millis(3);

  // Print to serial
  #if SERIAL_PRINT
  Serial_readings_print();
  #endif

  print_millis(4);

  //Print wifi status
  #if SERIAL_PRINT
  Serial.print(F("Wifi status: "));
  Serial.println(status);
  #endif

  now = rtc.now();
  #if SERIAL_PRINT
  Serial.println(now.timestamp(DateTime::TIMESTAMP_FULL));
  #endif
  print_millis(5);
  wdt_reset();
  //Check if it's time to log
  if (nextmeasuretime <= now)
  {
    // WiFi.reset(ESP_RESET_PIN);
    // Serial.println(F("Waking ESP"));
    OLED_msg(F("Saving data to SD card"), 1000);
    //SD_save_eneruser_style();
    SD_save_tabular_style();

    
    OLED_msg(F("Sending data to cloud"), 1000);
    print_millis(6);
    cloud_send_data();
    print_millis(9);

    //clear averages values of the SPS measurements
    zero_averages();

    nextmeasuretime = now + TimeSpan(0, 0, 0, (60-now.second())); //update next measurement time by loginterval minutes span
  }

  //Check if it's time to adjust the clock
  wdt_reset();
  if(nextclockupdate<= now)
  {
    adjust_RTC();
    nextclockupdate = now + TimeSpan(0,6,0,0);
  }

  //Check if it's time pingtest
  if(nextpingtest <= now)
  {
    wdt_reset();
    ping_test();
    nextpingtest = now + TimeSpan(0,1,0,0);
  }

  delay(2000);
}

// UTILS FUNCTIONS

//Start all sensors and display information about initialization on the OLED screen
void sensors_start()
{
  display.clearDisplay();
  //Prepare OLED sensor's initialization screen
  //print all sensors' name on the screen
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("BME....");

  display.setCursor(0, 20);
  display.println("SPEC....");

  display.setCursor(62, 0);
  display.println("SPS30...");

  display.setCursor(62, 20);
  display.println(F("DS18..."));

  display.display();
  delay(1000);

  //INITIATE EACH SENSOR AND UPDATE OLED INFORMATION
  //IF SENSOR FAIL, PRINT "X"
  //ELSE, PRINT "OK"
  if (!BME_start())
  {
    //Funcao para mostar no OLED
    display.setCursor(0, 0);
    display.println("BME....X");
    display.display();
    delay(1000);
  }
  else
  {
    display.setCursor(0, 0);
    display.println("BME....OK");
    display.display();
    delay(1000);
  }

  if (!Specs_start())
  {
    //Funcao para mostar no OLED
    display.setCursor(0, 20);
    display.println("SPEC...X");
    display.display();
    delay(1000);
  }
  else
  {
    display.setCursor(0, 20);
    display.println("SPEC..OK");
    display.display();
    delay(1000);
  }

  if (!SPS_start())
  {
    //Funcao para mostar no OLED
    display.setCursor(62, 0);
    display.println("SPS30...X");
    display.display();
    delay(1000);
  }
  else
  {
    display.setCursor(62, 0);
    display.println("SPS30...OK");
    display.display();
    delay(1000);
  }

  if (!DS18_start())
  {
    //Funcao para mostar no OLED
    display.setCursor(62, 20);
    display.println("DS18...X");
    display.display();
    delay(1000);
  }
  else
  {
    display.setCursor(62, 20);
    display.println("DS18...OK");
    display.display();
    delay(1000);
  }

  delay(3000);
  display.clearDisplay();
}

bool BME_start()
{

  #if SERIAL_PRINT
  Serial.println(F("BME680 test"));
  #endif

  if (!bme.begin(0x76))
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    #endif
    return false;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  return true;
}


bool Specs_start()
{
  if (!co.init())
  {
    return false;
  }
  return true;
}

bool SPS_start()
{

  // Initialize SPS30 library
  if (!sps30.begin(&SP30_COMMS))
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not set serial communication channel"));
    #endif
    return false;
  }
  delay(500);
  // check for SPS30 connection
  if (!sps30.probe())
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not probe / connect with SPS30"));
    #endif

    return false;
  }
  delay(500);

  //reset SPS connection -- WITHOU THIS PART, SENSOR DO NOT READ, I DONT KNOW WHY
  if (!sps30.reset())
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not reset SPS30 connection"));
    #endif

    return false;
  }
  // start measurement
  if (!sps30.start())
  {
    #if SERIAL_PRINT
    Serial.println(F("SPS Measurement has not started"));
    #endif

    return false;
  }

  #if SERIAL_PRINT
  Serial.println(F("SPS Measurement started"));
  #endif
  
  return true;
}

bool DS18_start()
{
  //begin the sensor and request measurement to check the wiring
  ds18.begin();

  ds18.requestTemperatures();

  if(ds18.getTempCByIndex(0) != DEVICE_DISCONNECTED_C)
  {
    return true;
  }

  else
  {
    return false;
  }
  

}

void SD_start()
{

  pinMode(chipSelect, OUTPUT); //declaring CS pin as output pin
  if (SD.begin())
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("SD card is ready to use");
    display.display();
    delay(3000);
    
    #if SERIAL_PRINT
    Serial.println("SD card is initialized and it is ready to use");
    #endif

    return;
  }
  else
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("SD card is not initialized");
    display.display();
    delay(3000);

    #if SERIAL_PRINT
    Serial.println("SD card is not initialized");
    #endif
    
    return;
  }
}

// Request data from all sensors
// Initiate all variables to 0

void zero_variables()
{
  //Set zero to variables
  BMEtemp = 0;
  BMEpress = 0;
  BMEhum = 0;
  SPECco = 0;
  SPSmassPM1 = 0;
  SPSmassPM2 = 0;
  SPSmassPM4 = 0;
  SPSmassPM10 = 0;
  SPSnumPM1 = 0;
  SPSnumPM2 = 0;
  SPSnumPM4 = 0;
  SPSnumPM10 = 0;
  SPSpartSize = 0;
  SPSmassPM1avg.clear();
  SPSmassPM2avg.clear();
  SPSmassPM4avg.clear();
  SPSmassPM10avg.clear();
  SPSnumPM1avg.clear();
  SPSnumPM2avg.clear();
  SPSnumPM4avg.clear();
  SPSnumPM10avg.clear();
  SPSpartSizeavg.clear();
}

void zero_averages()
{
  SPSmassPM1avg.clear();
  SPSmassPM2avg.clear();
  SPSmassPM4avg.clear();
  SPSmassPM10avg.clear();
  SPSnumPM1avg.clear();
  SPSnumPM2avg.clear();
  SPSnumPM4avg.clear();
  SPSnumPM10avg.clear();
  SPSpartSizeavg.clear();
}

// Sensor's readings functions
// Update variables applying leaky integrator
void read_sensors()
{

  //BME READINDS ---------------------
  if (!bme.performReading())
  {
    #if SERIAL_PRINT
    Serial.println(F("Failed to perform BME reading"));
    #endif
  }
  else
  {
    BMEtemp = leaky_filter(BMEtemp, (0.8466*bme.temperature+0.0868));
    BMEpress = leaky_filter(BMEpress, bme.pressure / 100.0);
    BMEhum = leaky_filter(BMEhum, (1.2849*bme.humidity-4.307));
  }


  //SPEC readings ------------------------------------
  if (!co.request_data())
  {
    #if SERIAL_PRINT
    Serial.println(F("Failed to perform SPEC's readings"));
    #endif
  }
  else
  {
    SPECco = leaky_filter(SPECco, co.ppb / 1000.0); //division by 1000.0 to transfor int to float and to ppm

    // if CO TO HIGH, BLINK LED
    if(SPECco > 40)
    {
      blink(LED_COHIGH);
    }
    else
    {
      blink(LED_OFF);
    }

  }

  //SPS30 readings -----------------------------------
  if (!SPS_request_data())
  {
    #if SERIAL_PRINT
    Serial.println(F("Failed to perform SPS readings"));
    #endif
  }
  else
  {
    //Add the values to the mean function
    //Mass
    SPSmassPM1 = SPSvalues.MassPM1;
    SPSmassPM1avg.add(SPSmassPM1);

    SPSmassPM2 = SPSvalues.MassPM2;
    SPSmassPM2avg.add(SPSmassPM2);

    SPSmassPM4 = SPSvalues.MassPM4;
    SPSmassPM4avg.add(SPSmassPM4);

    SPSmassPM10 = SPSvalues.MassPM10;
    SPSmassPM10avg.add(SPSmassPM10);

    //Nums
    SPSnumPM1 = SPSvalues.NumPM1;
    SPSnumPM1avg.add(SPSnumPM1);

    SPSnumPM2 = SPSvalues.NumPM2;
    SPSnumPM2avg.add(SPSnumPM2);

    SPSnumPM4 = SPSvalues.NumPM4;
    SPSnumPM4avg.add(SPSnumPM4);

    SPSnumPM10 = SPSvalues.NumPM10;
    SPSnumPM10avg.add(SPSnumPM10);

    //Avg particle size
    SPSpartSize = SPSvalues.PartSize;
    SPSpartSizeavg.add(SPSpartSize);
  }

  //DS18B20 readings --------------------------------

  if(!DS18_request_data())
  {
    #if SERIAL_PRINT
    Serial.println(F("Failed to perform DS18 readings"));
    #endif
    DS18temp = -30;
  }
  else
  {
    //Apply leaky filter to the current measurement
    DS18temp = leaky_filter(DS18temp,ds18read-2.5);

  }

}

bool DS18_request_data()
{

  ds18.requestTemperatures();

  //Since there's only one ds18 sensor, get the temperature by index 0
  ds18read = ds18.getTempCByIndex(0);

  //Check if reading was sucessfull
  if(ds18read != DEVICE_DISCONNECTED_C)
  {
    return true;
  }
  else{
    return false;
  }


}

bool SPS_request_data()
{
  uint8_t ret, error_cnt = 0;

  // loop to get data
  do
  {

    ret = sps30.GetValues(&SPSvalues);

    // data might not have been ready
    if (ret == ERR_DATALENGTH)
    {

      if (error_cnt++ > 3)
      {
        #if SERIAL_PRINT
        // Serial.println("Passou por este erro 1");
        #endif
        return (false);
      }
      delay(1000);
    }

    // if other error
    else if (ret != ERR_OK)
    {
      #if SERIAL_PRINT
      // Serial.println("Passou por este erro 2");
      #endif
      return (false);
    }

  } while (ret != ERR_OK);

  return true;
}

float leaky_filter(float previous, float current)
{

  return ((previous)*previous_weight + (current)*current_weight);
}

// DISPLAY ------------------------------

void OLED_start()
{

  display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS); // Stating the display
  display.setRotation(2); //Rotate 180

  // Clear the buffer.
  display.clearDisplay();

  // Display text
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(1, 0);
  display.println(F("Initializing..."));
  display.display();

}

//-------------------------------------------------------------------------------------------------------------

void OLED_readings_print()
{

  display.clearDisplay();
  
  display.setCursor(0, 0);
  
  display.println(now.timestamp(DateTime::TIMESTAMP_FULL));

  //Column 1
  display.setCursor(0,8);
  display.print("T:");
  display.println(BMEtemp);

  display.print("P:");
  display.println(BMEpress);

  display.print("H:");
  display.println(BMEhum);

  // Column 2
  display.setCursor(62,8*1);
  display.print("Tg:");
  display.println(DS18temp);

  display.setCursor(62,8*2);
  display.print("P10:");
  display.println(SPSnumPM10avg.average(),4);

  display.setCursor(62,8*3);
  display.print("wifi:");
  display.println(status);

  display.display();
}

void Serial_readings_print()
{

  #if SERIAL_PRINT
  Serial.print("TemperatureBME= ");
  Serial.print(BMEtemp);
  Serial.println(" ºC");

  Serial.print("Pressure= ");
  Serial.print(BMEpress);
  Serial.println(" hPa");

  Serial.print("HumidityBME= ");
  Serial.print(BMEhum);
  Serial.println(" %");

  Serial.print("TemperaturaGLOBE= ");
  Serial.print(DS18temp);
  Serial.println(" ºC");

  Serial.print("CO= ");
  Serial.print(SPECco,4);
  Serial.println(" ppm");

  Serial.print("PartSize= ");
  Serial.print(SPSpartSizeavg.average(),4);
  Serial.println(" um");

  Serial.print("massPM1= ");
  Serial.print(SPSmassPM1avg.average(),4);
  Serial.println(" ug/m3");

  Serial.print("massPM2= ");
  Serial.print(SPSmassPM2avg.average(),4);
  Serial.println(" ug/m3");

  Serial.print("massPM4= ");
  Serial.print(SPSmassPM4avg.average(),4);
  Serial.println(" ug/m3");

  Serial.print("massPM10= ");
  Serial.print(SPSmassPM10avg.average(),4);
  Serial.println(" ug/m3");

  Serial.print("numPM1= ");
  Serial.print(SPSnumPM1avg.average(),4);
  Serial.println("#/cm3");

  Serial.print("numPM2= ");
  Serial.print(SPSnumPM2avg.average(),4);
  Serial.println("#/cm3");

  Serial.print("numPM2= ");
  Serial.print(SPSnumPM4avg.average(),4);
  Serial.println("#/cm3");

  Serial.print("numPM10= ");
  Serial.print(SPSnumPM10avg.average(),4);
  Serial.println("#/cm3");

  Serial.println();
  #endif
}

void SD_save_tabular_style()
{
  //If file do not exists, create and print the header
  //Filename must be 8+3 characters long
  //https://www.arduino.cc/en/Reference/SDCardNotes
  if (!SD.exists("indoor.txt"))
  {
    tabulardata = SD.open("indoor.txt", FILE_WRITE);
    tabulardata.print(F("datetime,temperature,pressure,humidity,globeTemperature,CO,partsize,massPM1,massPM2,massPM4,massPM10,numPM1,numPM2,numPM4,numPM10"));
    tabulardata.println();
    tabulardata.close();
  }

  //open file to print readings
  tabulardata = SD.open("indoor.txt", FILE_WRITE);
  if (tabulardata)
  {
    //Print UTC timestamp
    tabulardata.print(now.timestamp(DateTime::TIMESTAMP_FULL));
    tabulardata.print(",");
    // end of UTC timestamp

    //Print measurements
    tabulardata.print(BMEtemp);
    tabulardata.print(",");
    tabulardata.print(BMEpress);
    tabulardata.print(",");
    tabulardata.print(BMEhum);
    tabulardata.print(",");
    tabulardata.print(DS18temp);
    tabulardata.print(",");
    tabulardata.print(SPECco);
    tabulardata.print(",");
    tabulardata.print(SPSpartSizeavg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSmassPM1avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSmassPM2avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSmassPM4avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSmassPM10avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSnumPM1avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSnumPM2avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSnumPM4avg.average(),4);
    tabulardata.print(",");
    tabulardata.print(SPSnumPM10avg.average(),4);
    tabulardata.println();
    tabulardata.close();

    //Confirm on the serial port that the file is closed and measruments saved
    #if SERIAL_PRINT
    Serial.println("Data saved to tabular.txt");
    #endif
    OLED_msg(F("Data saved to tabular"), 2000);
  }
  else
  {
    #if SERIAL_PRINT
    Serial.println("Could not open file in the SD card");
    #endif
  }
}

void OLED_msg(const __FlashStringHelper *msg, int duration)
{
  //see https://forum.arduino.cc/t/passingpassing-f-string-as-a-parameter/108984
  // Pass F() as an function argument

  display.clearDisplay();
  display.setCursor(0, 8);
  display.setTextSize(1);
  display.println(msg);
  display.display();
  delay(duration);
}

void ESP_start()
{

  //WiFi.reset(ESP_RESET_PIN);
  // initialize ESP module
  //set reset pin to hardware reset
  WiFi.init(&Serial2,ESP_RESET_PIN);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD)
  {
    status = WL_NO_SHIELD;
    #if SERIAL_PRINT
    Serial.println("WiFi shield not present");
    #endif
    OLED_msg(F("WiFi shield not present"),2000);
    return;
  }

  WiFi.setAutoConnect(false);
  delay(100);
  WiFi.endAP(true);
  delay(100);
  WiFi.disconnect();
  WiFi.sleepMode(WIFI_NONE_SLEEP);
  delay(100);
}

void wifi_connect()
{

  
  int i = 0;
  // attempt to connect to WiFi network
  while (status != WL_CONNECTED)
  {
    
    i++;
    if (i >= 2)
    {
      return;
    }
    #if SERIAL_PRINT
    Serial.print("Attempting to connect to WPA SSID: ");
    #endif
    OLED_msg(F("Trying to connect"),100);
    #if SERIAL_PRINT
    Serial.println(ssid);
    #endif
    // Connect to WPA/WPA2 network
    wdt_reset();
    status = WiFi.begin(ssid, pwd);
    wdt_reset();
    if(!WiFi.ping(google))
    {
      status = WL_DISCONNECTED;
    }
    delay(1000);
  }
  
  wdt_reset();
  #if SERIAL_PRINT
  Serial.println("You're connected to the network");
  #endif
  OLED_msg(F("Connected to Network"),500);
}


void SPS_clean_now()
{

  if (sps30.clean() == true)
  {
    #if SERIAL_PRINT
    Serial.println(F("fan-cleaning manually started"));
    #endif
    }
  else
  { 
    #if SERIAL_PRINT
    Serial.println(F("Could NOT manually start fan-cleaning"));
    #endif

    }
}

void SPS_set_autoclean(uint32_t new_auto_clean_interval)
{
  uint32_t interval;
  uint8_t ret;

  // try to get interval
  ret = sps30.GetAutoCleanInt(&interval);
  if (ret == ERR_OK)
  {
    #if SERIAL_PRINT
    Serial.print(F("Current Auto Clean interval: "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
    #endif
  }
  else
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not get clean interval"));
    #endif
  }

  // try to set interval
  interval = new_auto_clean_interval;
  ret = sps30.SetAutoCleanInt(interval);
  if (ret == ERR_OK)
  {
    #if SERIAL_PRINT
    Serial.print(F("Auto Clean interval now set : "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
    #endif
  }
  else
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not set new clean interval"));
    #endif
  }

  // try to get interval
  ret = sps30.GetAutoCleanInt(&interval);
  if (ret == ERR_OK)
  {
    #if SERIAL_PRINT
    Serial.print(F("Current Auto Clean interval: "));
    Serial.print(interval);
    Serial.println(F(" seconds"));
    #endif
  }
  else
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not get clean interval"));
    #endif
  }
}

void cloud_send_data()
{

  delay(1000);
  wdt_reset();
  //Check wifi status
  //if not connected, try to connect
  status = WiFi.status();
  if ((status != WL_CONNECTED))
  {
    #if SERIAL_PRINT
    Serial.println(F("Not connected to wifi, trying to connect now!"));
    #endif
    OLED_msg(F("Wifi NOT connected..."), 1000);
    print_millis(7);
    wifi_connect();
    print_millis(8);
  }

  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    //return early if connection failed
    return;
  }

  //If the code doesnt return, send data to cloud
  //send data to spanish server via UDP
  print_millis(10);
  UDP_send_all();
  print_millis(11);
  blink(LED_DATASENT);
  

}


void UDP_send_data(String variable_name, float value, int decimals)
{
  String data = "";
  char packet[128] = "";
  // {variable_name},location={nome_local},host=node2 value={variable_value}
  // data = variable_name + ",location=" + String(location_name) + ",host=" + String(host_name) + ",value=" + String(value,decimals);
  data = variable_name + ",location=" + String(location_name) + ",host=" + String(host_name) + ",value=" + String(value,decimals)+","+ now.timestamp(DateTime::TIMESTAMP_FULL)+",";
  data.toCharArray(packet, data.length());

  wdt_reset();
  #if SERIAL_PRINT
  Serial.print("Sending data to UDP server: ");
  Serial.println(packet);
  #endif
  udpSender.beginPacket(spanish_server, spanish_port);
  udpSender.print(packet);
  udpSender.endPacket();
  #if SERIAL_PRINT
  Serial.println("UDP data sent!");
  #endif

  delay(100);
}

void UDP_all_variables_string(int decimals){

String data = "";
char packet[256] = "";

//Sequence: datetime,local,press,temp,tempglobe,hum,co2,co,o3,partsize,massPM1,massPM2,massPM4,massPM10,numPM1,numPM2,numPM4,numPM10
data = now.timestamp(DateTime::TIMESTAMP_FULL) + "," + String (location_name) + ",";
data += String(BMEpress)+","+String(BMEtemp)+","+String(DS18temp)+","+String(BMEhum);
data += ","; // empty co2 measurement
data += "," + String(SPECco)+",";
data += ","; //empty o3 measrument
data += String(SPSpartSizeavg.average(),4)+",";
data += String(SPSmassPM1avg.average(),4)+",";
data += String(SPSmassPM2avg.average(),4)+",";
data += String(SPSmassPM4avg.average(),4)+",";
data += String(SPSmassPM10avg.average(),4)+",";
data += String(SPSnumPM1avg.average(),4)+",";
data += String(SPSnumPM2avg.average(),4)+",";
data += String(SPSnumPM4avg.average(),4)+",";
data += String(SPSnumPM10avg.average(),4)+",";
data += firstboot+",";

data.toCharArray(packet,data.length());

wdt_reset();
#if SERIAL_PRINT
Serial.print("Sending WHOLE! data to UDP server: ");
Serial.println(packet);
#endif
udpSender.beginPacket(spanish_server, spanish_port);
udpSender.print(packet);
udpSender.endPacket();
#if SERIAL_PRINT
Serial.println("UDP data sent!");
#endif

delay(100);


};


void UDP_send_all()
{
  //udpSender.flush();

  UDP_all_variables_string();

  // UDP_send_data("temp", BMEtemp);
  // UDP_send_data("press", BMEpress);
  // UDP_send_data("hum", BMEhum);
  // UDP_send_data("tempglobe", DS18temp);
  // UDP_send_data("co", SPECco);
  // UDP_send_data("partsize", SPSpartSizeavg.average(),4);
  // UDP_send_data("massPM1", SPSmassPM1avg.average(),4);
  // UDP_send_data("massPM2", SPSmassPM2avg.average(),4);
  // UDP_send_data("massPM4", SPSmassPM4avg.average(),4);
  // UDP_send_data("massPM10", SPSmassPM10avg.average(),4);
  // UDP_send_data("numPM1", SPSnumPM1avg.average(),4);
  // UDP_send_data("numPM2", SPSnumPM2avg.average(),4);
  // UDP_send_data("numPM4", SPSnumPM4avg.average(),4);
  // UDP_send_data("numPM10", SPSnumPM10avg.average(),4);
  udpSender.stop();
  //delay(500);
  firstboot = "FALSE";
  
}


bool get_NTP_time()
{
  timeClient.begin();
  timeClient.update();
  delay(100);
  timeClient.update();
  delay(100);
  #if SERIAL_PRINT
  Serial.print("NTP time:");
  #endif
  ntptime = DateTime(timeClient.getEpochTime());
  if(ntptime.year() > 2030)
  {
    timeClient.end();
    return false;
  }
  #if SERIAL_PRINT
  Serial.println(ntptime.timestamp(DateTime::TIMESTAMP_FULL));
  #endif
  timeClient.end();
  return true;

}

void adjust_RTC()
{
  OLED_msg(F("Adjusting Time"),500);
  status = WiFi.status();
  if (status != WL_CONNECTED)
  {
    #if SERIAL_PRINT
    Serial.println(F("Could not adjust clock, wifi not connected"));
    #endif
    OLED_msg(F("Wifi not connected..."), 1500);
    return;
  }

  while (!get_NTP_time())
  {
    delay(1000);
  }

  rtc.adjust(ntptime);
  #if SERIAL_PRINT
  Serial.println(F("RTC time adjusted"));
  #endif
  OLED_msg(F("DONE"),500);
  
}

void print_millis(uint8_t checkpoint)
{
  #if PRINT_MILLIS == true
  Serial.println();
  Serial.print(F("Check point "));
  Serial.print(checkpoint);
  Serial.print(F(": "));
  Serial.println(millis());
  Serial.println();
  checkpoint = checkpoint+1;
  #endif

  return ;
}
  
void ping_test()
{
  if(!WiFi.ping(google))
  {
    status = WL_DISCONNECTED;
  }

}



void blink(uint8_t type){
  wdt_reset();

  #if LED_PIN != 0

    switch (type)
    {
    case LED_OFF:

      digitalWrite(LED_PIN,LOW);
      break;
    
    case LED_DATASENT:

      digitalWrite(LED_PIN,HIGH);
      delay(100);
      digitalWrite(LED_PIN,LOW);
      delay(100);

      digitalWrite(LED_PIN,HIGH);
      delay(100);
      digitalWrite(LED_PIN,LOW);
      delay(100);

      digitalWrite(LED_PIN,HIGH);
      delay(100);
      digitalWrite(LED_PIN,LOW);
      delay(100);

      digitalWrite(LED_PIN,HIGH);
      delay(100);
      digitalWrite(LED_PIN,LOW);
      
      break;
    
      case LED_COHIGH:

      digitalWrite(LED_PIN,HIGH);
      break;

      case LED_ERROR:

      digitalWrite(LED_PIN,HIGH);
      delay(1000);
      digitalWrite(LED_PIN,LOW);
      delay(1000);

      digitalWrite(LED_PIN,HIGH);
      delay(1000);
      digitalWrite(LED_PIN,LOW);
      delay(1000);

      digitalWrite(LED_PIN,HIGH);
      delay(1000);
      digitalWrite(LED_PIN,LOW);
      delay(1000);

      break;

    default:

      digitalWrite(LED_PIN,LOW);
      break;
    }

  #endif

};
