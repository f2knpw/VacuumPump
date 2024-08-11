#include <WiFi.h>
//#include <HTTPClient.h>
#include <WiFiClient.h>
#include <TelnetStream.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>

#include <esp_task_wdt.h>
//1 seconds WDT
#define WDT_TIMEOUT 1
int nbTimeout = 0;
int nbAlarm = 0;
long bootTime ;

//Json
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson


//Preferences
#include <Preferences.h>
Preferences preferences;


enum { statusStop, statusStart, statusRun, statusIdle, statusAlarm };
int status = statusStart; //pump will automatically start its vacuum cycle after booting the ESP32
String displayStatus = "Initializing";

float pressure, lowPressure, highPressure, prevPressure;
int pwmSpeed;
long lastStart, lastRun;
int alarmCount;

//OLED
//#define OLED
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



//WifiManager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager



//flag for saving data
bool shouldSaveConfig = false;
bool touch3detected = false;  //touch3 used to launch WifiManager (hold it while reseting)

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;


#define RUN_PIN 26
#define START_PIN 27
#define LED_PIN 22
#define PRESSURE_PIN 36

#define ON_PIN    2  //touch pin to start pump
#define OFF_PIN  15  //touch pin to stop pump
#define PLUS_PIN  14  //touch pin to increase highPressure threshold
#define MINUS_PIN  12  //touch pin to decrease highPressure threshold
long lastTouch ;

#define RESTART_DELAY 1000  // pump will not restart before this delay

#define DC_PWM_MOTOR              // the motor is a DC motor driven by PWM pin (else it's a 230VAC fridge motor with one or two relays or a DC motor at full power)

//#define W_DEBUG     //debug Wifi and firebase
//#define G_DEBUG     //debug GCM serveur
//#define DEBUG_OUT
//#define xDEBUG
#define UDP_DEBUG
//#define DEBUG
//#define TEST
//#define PREFERENCES_DEBUG
#define DEBUG_BLE
#define DEBUG_TELNET


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
  // if (deviceConnected == true)
  {
#ifdef DEBUG_BLE
    Serial.println (theString);
#endif
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
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
#ifdef DEBUG_BLE
      Serial.println("client connected");
#endif
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
#ifdef DEBUG_BLE
      Serial.println("client disconnected");
#endif
      // Start advertising
      pServer->getAdvertising()->stop();
      delay(100);
      pServer->getAdvertising()->start();
#ifdef DEBUG_BLE
      Serial.println("Waiting a client connection to notify...");
#endif
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
#ifdef DEBUG_OUT
        Serial.print("Received : ");
#endif
        for (int i = 0; i < rxValue.length(); i++)
        {
#ifdef DEBUG_OUT
          Serial.print(rxValue[i]);
#endif
          test = test + rxValue[i];
        }
#ifdef DEBUG_OUT
        Serial.println();
#endif
      }
      //      String Res;
      //
      //      int i;
      //then it may contain JSON
      readCmd( test);
    }
};

//UDP --------------
unsigned int localPort = 5000;      // local port to listen on
char packetBuffer[64]; //buffer to hold incoming packet
char AndroidConnected = 0;
long Timeout;
String device = "eVacuumPump";
String theMAC = "";

WiFiUDP Udp;
//end UDP-----------




void setup()
{
  Serial.begin(115200);
  delay(5000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

#ifdef DC_PWM_MOTOR
  ledcAttachPin(RUN_PIN, 0); // assign PWM pins to channels
  // Initialize channels : ledcSetup(uint8_t channel, uint32_t freq, uint8_t resolution_bits);
  ledcSetup(0, 24000, 11); // 24 kHz PWM, 11-bit resolution (range 0-2047)
  //start pump driver
  pwmSpeed = 2047;           //pump stopped at startup (range 0 to 2047)
  ledcWrite(0, pwmSpeed);
#else
  pinMode(RUN_PIN, OUTPUT);
  pinMode(START_PIN, OUTPUT);
  digitalWrite(START_PIN, HIGH); //start coil not powered
  digitalWrite(RUN_PIN, HIGH);   //run coil not powered
#endif

  //Preferences
  preferences.begin("VacuumPump", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  highPressure = preferences.getFloat("highPressure", 1000);
  lowPressure = preferences.getFloat("lowPressure", 100);
  ssid = preferences.getString("ssid", "");         // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");

  //preferences.end();  // Close the Preferences
#ifdef DEBUG_BLE
  Serial.println("*********************************************************");
#endif
#ifdef OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  displayLCD();
#endif

  if (ftouchRead(T3) < threshold) touch3detected = true; //detect touchpad for CONFIG_PIN


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
#ifdef DEBUG_BLE
  Serial.println("Waiting a client connection...");
#endif
  delay(100);

  //  //connect to WiFi

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);

  if ( touch3detected)  //then launch WifiManager
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal("JP VacuumPump"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  delay(2000);
  //  //save the custom WifiManager's parameters if needed
  if (shouldSaveConfig)
  {
    Serial.println("saving Wifi credentials ");
    //read updated parameters

    preferences.putString("password", WiFi.psk());
    preferences.putString("ssid", WiFi.SSID());
    delay(2000);
    ESP.restart();
    delay(5000);
  }




  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 20000))
  {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);



  if (hasWifiCredentials)
  {
    TelnetStream.begin(); //used to debug over telnet
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //Start UDP
    Udp.begin(localPort);
  }

  //pressure sensor
  //analogSetClockDiv(255);
  //analogReadResolution(12);           // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetAttenuation(ADC_11db);        // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db

  lastStart = millis() - RESTART_DELAY;
  lastTouch = millis();

  //Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  bootTime = millis();

#ifdef DEBUG_TELNET
  TelnetStream.print("start");
#endif

}




void loop()
{
  esp_task_wdt_reset(); //reset the watchdog

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
#ifdef DC_PWM_MOTOR
#else
        startPump(true);
#endif
        status = statusRun;
        lastRun = millis();
      }
      break;
    case statusStop:
      startPump(false);
      status = statusStop;
      break;
    case statusRun:
      lastStart = millis();

#ifdef DC_PWM_MOTOR
      pwmSpeed = mapfloat(pressure, 0, 100, 1500, 0);   //control pump speed with pressure value
      if (pressure < lowPressure * .5)  pwmSpeed = 0;
      ledcWrite(0, pwmSpeed);                           //PWM to the pump
#endif

      if (pressure > highPressure)
      {
        startPump(false);
        status = statusIdle;
        nbAlarm = 0;
      }
      if ((pressure - prevPressure) < 3)
      {
        alarmCount ++;
#ifdef DEBUG_OUT
        Serial.print("alarmCount ");
        Serial.println(alarmCount);
#endif
      }
      if (alarmCount > 20)
      {
        startPump(false);
        alarmCount = 0;
        nbAlarm ++;
        if (nbAlarm < 3)    // try to retry 3 times...
        {
          Serial.print("motor not started, retry ");
          Serial.println(nbAlarm);
          status = statusStart;
        }
        else
        {
          Serial.println("motor not started, alarm");
          status = statusAlarm;
        }
      }
      //      if ((millis() - lastRun) > 10000) //TEST : stop pump after 10s running
      //      {
      //        startPump(false);
      //        status = statusStart;
      //      }
      break;
    case statusIdle:
      if (pressure < lowPressure)
      {
        if ((millis() - lastStart) > RESTART_DELAY)
        {
          status = statusStart;
        }
      }
      else
      {
        startPump(false);
      }
      //if ((millis() - bootTime)> 600000) ESP.restart();
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
    res = "{\"P\":\"" + String(pressure) + "\",\"L\": " + String(lowPressure) + ",\"H\": " + String(highPressure) + ",\"S\": " + String(status) + ",\"Run\": " + String(LastBLEnotification / 1000 / 60) + "}";
    BLEnotify(res);
#ifdef DEBUG_TELNET
    TelnetStream.println(res);
#endif
    if (deviceConnected == false) sendUDP(res); //BLE not connected try to send via UDP
  }

  //manage touch pads
  if ((millis() - lastTouch) > 1000)
  {
    if (ftouchRead(ON_PIN) < threshold)
    {
      //Serial.println("touch ON detected ");
      lastTouch = millis();
      status = statusStart; //will start the pump

    }
    if (ftouchRead(OFF_PIN)  < threshold)
    {
      //Serial.println("touch OFF detected ");
      lastTouch = millis();
      status = statusStop;  // will stop the pump
    }
    if (ftouchRead(PLUS_PIN) < threshold)
    {
      //Serial.println("touch + detected ");
      lastTouch = millis();
      if (status == statusStop)
      {
        highPressure += 5;
        preferences.putFloat("highPressure", highPressure);
        //        Serial.print("Set high value ");
        //        Serial.println(highPressure);
      }
      else
      {
        lowPressure += 5;
        preferences.putFloat("lowPressure", lowPressure);
        //        Serial.print("Set low value ");
        //        Serial.println(lowPressure);
      }

    }
    if (ftouchRead(MINUS_PIN) < threshold)
    {
      //Serial.println("touch - detected ");
      lastTouch = millis();
      if (status == statusStop)
      {
        highPressure -= 5;
        preferences.putFloat("highPressure", highPressure);
        //        Serial.print("Set high value ");
        //        Serial.println(highPressure);
      }
      else
      {
        lowPressure -= 5;
        preferences.putFloat("lowPressure", lowPressure);
        //        Serial.print("Set low value ");
        //        Serial.println(lowPressure);
      }
    }
  }

  // UDP process : if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Timeout = millis();          //rearm software watchdog

#if defined xxxDEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
#endif
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
#if defined UDP_DEBUG
    Serial.print("UDP Contents: ");
    Serial.println(packetBuffer);
#endif
    String Res;
    String test;
    int i;
    test = packetBuffer;

    if (test == "ID")// send a reply, to the IP address and port that sent us the packet we received
    {
      AndroidConnected = 1;
      Res =  device;
      sendUDP(Res);

    }
    else if (test.startsWith("TOPIC"))// send a reply, to the IP address and port that sent us the packet we received
    {
      //{"sender":"18FE349E7690","device":"WaterLeak"}
      Res =  WiFi.macAddress();
      Res.replace(":", "");

      sendUDP(Res);


    }

    else if  (test.startsWith("{"))// decode json
    {
      // message = test.substring(6);
      readCmd( test);
    }

    else if  (test.startsWith("STOP"))// send a reply, to the IP address and port that sent us the packet we received
    {
      sendUDP("okSTOP");
    }
  }
}

#ifdef OLED
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
#endif

void startPump(boolean start)
{
  if (start == true)
  {
#ifdef DC_PWM_MOTOR
    //pwmSpeed = 0;
    ledcWrite(0, pwmSpeed);         //PWM full power
#else
    digitalWrite (RUN_PIN, LOW);  // no PWM regulation
    digitalWrite(START_PIN, LOW);
    delay(150);      //min tested 40
    digitalWrite(START_PIN, HIGH);
#endif
  }
  else
  {
#ifdef DC_PWM_MOTOR
    pwmSpeed = 2047;
    ledcWrite(0, pwmSpeed);             //PWM zero power
#else
    digitalWrite (RUN_PIN, HIGH);
#endif
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
  for ( int i = 0; i < 10; i++)
  {
    readVal = touchRead(gpio);
    val = max (val, readVal);
  }
  return val;
}


void sendUDP(String Res)
{
  char  ReplyBuffer[Res.length() + 1];     // a string to send back
  Res.toCharArray(ReplyBuffer, Res.length() + 1);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(ReplyBuffer); //was write...
  Udp.endPacket();
}

void readCmd(String test)
{
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
#ifdef DEBUG_OUT
        Serial.print("Start pump ");
#endif
        status = statusStart;
      }
      else if (Cmd == "Stop") //stop pump
      {
#ifdef DEBUG_OUT
        Serial.print("Stop pump ");
#endif
        startPump(false);
        lastStart = millis();
        status = statusStop;
      }
      else if (Cmd == "SetH") //set high
      {
        String value = doc["value"];
        highPressure = value.toFloat();
        preferences.putFloat("highPressure", highPressure);
#ifdef DEBUG_OUT
        Serial.print("Set high value ");
        Serial.println(highPressure);
#endif
      }
      else if (Cmd == "SetL") //set low
      {

        String value = doc["value"];
        lowPressure = value.toFloat();
        preferences.putFloat("lowPressure", lowPressure);
#ifdef DEBUG_OUT
        Serial.print("Set low value ");
        Serial.println(lowPressure);
#endif
      }
    }
  }
}
