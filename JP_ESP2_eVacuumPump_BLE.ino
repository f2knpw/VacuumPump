


//Json
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson


//Preferences
#include <Preferences.h>
Preferences preferences;


enum { statusStop, statusStart, statusRun, statusIdle, statusAlarm };
int status = statusStart; //pump will automatically start its vacuum cycle after booting the ESP32
String displayStatus = "Initializing";

float pressure, lowPressure, highPressure, prevPressure;
long lastStart;
int alarmCount;

//OLED
#define OLED
#ifdef OLED
#define OLED_SCL_PIN 5
#define OLED_SDA_PIN 17
#include "SSD1306.h"
SSD1306  display(0x3c, OLED_SDA_PIN, OLED_SCL_PIN);// for 0.96" 128x64 LCD display ; i2c ADDR & SDA, SCL
#endif
String displayDebug;


//RTC_DATA_ATTR int bootCount = 0;

//touchpad
touch_pad_t touchPin;
int threshold = 35; //Threshold value for touchpads pins

void callback() {
  //placeholder callback function
}
boolean TouchWake = false;

#define RUN_PIN 26
#define START_PIN 27
#define LED_PIN 22
#define PRESSURE_PIN 36

#define ON_PIN    2  //touch pin to start pump
#define OFF_PIN  15  //touch pin to stop pump
#define PLUS_PIN  14  //touch pin to increase highPressure threshold
#define MINUS_PIN  12  //touch pin to decrease highPressure threshold
long lastTouch ;

#define RESTART_DELAY 15000

#define W_DEBUG     //debug Wifi and firebase
//#define G_DEBUG     //debug GCM serveur
#define DEBUG_OUT
//#define xDEBUG
//#define UDP_DEBUG
#define DEBUG
//#define TEST
#define PREFERENCES_DEBUG


//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

long LastBLEnotification;

//BLE declarations
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331915f"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26bb"

void BLEnotify(String theString )
{
  if (deviceConnected == true)
  {
    Serial.println (theString);
    char message[21];
    String small = "";          //BLE notification MTU is limited to 20 bytes
    while (theString.length() > 0)
    {
      small = theString.substring(0, 19); //cut into 20 chars slices
      theString = theString.substring(19);
      small.toCharArray(message, 20);
      pCharacteristic->setValue(message);
      pCharacteristic->notify();
      delay(3);             // bluetooth stack will go into congestion, if too many packets are sent
      LastBLEnotification = millis(); //will prevent to send new notification before this one is not totally sent
    }
  }
}

class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("client connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("client disconnected");
      // Start advertising
      pServer->getAdvertising()->stop();
      delay(100);
      pServer->getAdvertising()->start();
      Serial.println("Waiting a client connection to notify...");
      delay(100);
    }
};

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      std::string rxValue = pCharacteristic->getValue();
      String test = "";
      if (rxValue.length() > 0)
      {
        Serial.print("Received : ");
        for (int i = 0; i < rxValue.length(); i++)
        {
          Serial.print(rxValue[i]);
          test = test + rxValue[i];
        }
        Serial.println();
      }
      String Res;

      int i;
      if (test.startsWith("{")) //then it may contain JSON
      {
        StaticJsonDocument<2000> doc;
        DeserializationError error = deserializeJson(doc, test);
        // Test if parsing succeeds.
        if (error) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.c_str());
          Serial.println("deserializeJson() failed");                             //answer with error : {"answer" : "error","detail":"decoding failed"}
          BLEnotify("{\"answer\" : \"error\",\"detail\":\"decoding failed\"}");
        }
        else
        {
          // Fetch values --> {"Cmd":"Start"}
          String Cmd = doc["Cmd"];
          if (Cmd == "Start") //start pump
          {
            Serial.print("Start pump ");
            status = statusStart;
          }
          else if (Cmd == "Stop") //stop pump
          {
            Serial.print("Stop pump ");
            startPump(false);
            lastStart = millis();
            status = statusStop;
          }
          else if (Cmd == "SetH") //set high
          {
            String value = doc["value"];
            highPressure = value.toFloat();
            preferences.putFloat("highPressure", highPressure);
            Serial.print("Set high value ");
            Serial.println(highPressure);
          }
          else if (Cmd == "SetL") //set low
          {
            Serial.print("Set low value ");
            String value = doc["value"];
            lowPressure = value.toFloat();
            preferences.putFloat("lowPressure", lowPressure);
            Serial.print("Set low value ");
            Serial.println(lowPressure);
          }
        }
      }
    }
};



void setup()
{
  Serial.begin(115200);

  pinMode(RUN_PIN, OUTPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(START_PIN, HIGH); //start coil not powered
  digitalWrite(RUN_PIN, HIGH);   //run coil not powered
  delay(5000);


  //Preferences
  preferences.begin("VacuumPump", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  highPressure = preferences.getFloat("highPressure", 1000);
  lowPressure = preferences.getFloat("lowPressure", 100);

  //preferences.end();  // Close the Preferences

  Serial.println("*********************************************************");

#ifdef OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  displayLCD();
#endif




  //BLE
  // Create the BLE Device
  BLEDevice::init("JP VacuumPump");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  delay(100);

  //pressure sensor
  //analogSetClockDiv(255);
  //analogReadResolution(12);           // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);        // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db

  lastStart = millis() - RESTART_DELAY;
  lastTouch = millis();
}




void loop()
{
  //pressure sensor
  pressure =  0;
  for (int i = 0; i < 1000; i++)
  {
    pressure += analogRead(PRESSURE_PIN);
  }
  pressure = pressure / 1000;
  //pressure = mapfloat(pressure, fromLow, fromHigh, toLow, toHigh);
  pressure = mapfloat(pressure, 242, 3200, 100, 0); //242 is ADC for 0.2V ; 3200 is ADC for 2.7V (atmospheric pressure)
  // sensor datasheet here : https://www.micros.com.pl/mediaserver/CZ_XGZP6847a010kpg_0001.pdf
  // or here : https://cdn.hackaday.io/files/1966658414115360/negPressure%20Sensor_CZ_XGZP6847a010kpg_0001.pdf

#ifdef xxxDEBUG
  Serial.print("pressure : ");
  Serial.println(pressure);
#endif
#ifdef OLED
  displayLCD();
#endif

  switch (status)
  {
    case statusStart:
      if ((millis() - lastStart) > RESTART_DELAY)
      { 
        prevPressure = pressure;
        alarmCount = 0;
        startPump(true);
        status = statusRun;
      }
      break;
    case statusStop:
      startPump(false);
      status = statusStop;
      break;
    case statusRun:
      lastStart = millis();
      if (pressure > highPressure)
      {
        startPump(false);
        status = statusIdle;
      }
      if ((pressure - prevPressure) < 3) 
      {
        alarmCount ++;
        Serial.print("alarmCount ");
        Serial.println(alarmCount);
      }
      if (alarmCount > 20)
      {
        Serial.println("motor not started");
        status = statusAlarm;
        startPump(false);
      }
    case statusIdle:
      if (pressure < lowPressure)
      {
        if ((millis() - lastStart) > RESTART_DELAY)
        {
          status = statusStart;
        }
      }
      break;
    default:
      // statements
      break;
  }

  //send BLE notification
  if (((millis() - LastBLEnotification) > 2000) ) // send BLE message to Android App (no need to be connected, a simple notification, send and forget)
  {

    LastBLEnotification = millis();
    String res;
    res = "{\"P\":\"" + String(pressure) + "\",\"L\": " + String(lowPressure) + ",\"H\": " + String(highPressure) + ",\"S\": " + String(status) + "}";
    BLEnotify(res);

  }

  //manage touch pads
  if ((millis() - lastTouch) > 1000)
  {
    if (ftouchRead(ON_PIN) < threshold)
    {
      Serial.println("touch ON detected ");
      lastTouch = millis();
      status = statusStart; //will start the pump
      
    }
    if (ftouchRead(OFF_PIN)  < threshold)
    {
      Serial.println("touch OFF detected ");
      lastTouch = millis();
      status = statusStop;  // will stop the pump
    }
    if (ftouchRead(PLUS_PIN) < threshold)
    {
      Serial.println("touch + detected ");
      lastTouch = millis();
      if (status == statusStop)
      {
            highPressure += 5;
            preferences.putFloat("highPressure", highPressure);
            Serial.print("Set high value ");
            Serial.println(highPressure);
      }
      else
      {
            lowPressure += 5;
            preferences.putFloat("lowPressure", lowPressure);
            Serial.print("Set low value ");
            Serial.println(lowPressure);
      }
      
    }
    if (ftouchRead(MINUS_PIN) < threshold)
    {
      Serial.println("touch - detected ");
      lastTouch = millis();
          if (status == statusStop)
      {
            highPressure -= 5;
            preferences.putFloat("highPressure", highPressure);
            Serial.print("Set high value ");
            Serial.println(highPressure);
      }
      else
      {
            lowPressure -= 5;
            preferences.putFloat("lowPressure", lowPressure);
            Serial.print("Set low value ");
            Serial.println(lowPressure);
      }
    }
  }
}

void displayLCD(void) //refresh the LCD screen
{
  display.clear();
  display.drawString(0, 0, "Pressure : " + (String)pressure + " kPa");
  switch (status)
  {
    case statusStart:
      display.drawString(0, 10, "Pump Starting");
      break;
    case statusStop:
      display.drawString(0, 10, "Pump Stopped");
      break;
    case statusRun:
      display.drawString(0, 10, "Pump Running");
      break;
    case statusIdle:
      display.drawString(0, 10, "Pump Idle");
      break;
  }
  display.drawString(0, 30, "High : " + (String)highPressure + " kPa");
  display.drawString(0, 40, "Low  : " + (String)lowPressure + " kPa");

  //display.drawString(0, 50, "charging : " + (String)smoothedChargingCurrent + " A");

  display.display();
}

void startPump(boolean start)
{
  if (start == true)
  {  
    digitalWrite (RUN_PIN, LOW);
    digitalWrite(START_PIN, LOW);
    delay(500);
    digitalWrite(START_PIN, HIGH);
  }
  else
  {
    digitalWrite (RUN_PIN, HIGH);
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int ftouchRead(int gpio)  // this will filter false readings of touchRead() function...
{
  int val = 0;
  int readVal;
  for( int i = 0; i<10; i++)
  {
    readVal = touchRead(gpio);
    val = max (val, readVal);
  }
  return val;
}
