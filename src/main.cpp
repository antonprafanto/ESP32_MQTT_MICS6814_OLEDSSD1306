#include <Arduino.h>

//Sensor OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include <math.h>
#include "EspMQTTClient.h"
#include <ArduinoJson.h>

//SettingMQTTClient
EspMQTTClient client(
  "HabibiGarden",
  "prodigy123",
  "151.106.125.63",  // MQTT Broker server ip
  "mqtt-smartpoultry",   // Can be omitted if not needed
  "@MySmartPoultry123",   // Can be omitted if not needed
  "Anton",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

//Sensor Gas MICS6814
// Enum for the sensor channels
enum channel {
  CH_NH3, CH_RED, CH_OX
};
typedef enum channel channel_t;

// Enum for proper gas declaration
enum gas {
  CO, NO2, NH3, C3H8, C4H10, CH4, H2, C2H5OH
};
typedef enum gas gas_t;

#define NH3PIN 35
#define COPIN 32
#define OXPIN 34

uint16_t NH3baseR;
uint16_t REDbaseR;
uint16_t OXbaseR;


/**
   Requests the current resistance for a given channel
   from the sensor. The value is an ADC value between
   0 and 1024.

   @param channel
          The channel to read the base resistance from.
   @return The unsigned 16-bit base resistance
           of the selected channel.
*/

uint16_t getResistance(channel_t channel) {
      unsigned long rs = 0;
      int counter = 0;

  switch (channel) {
    case CH_NH3:
      for(int i = 0; i < 100; i++) {
        rs += analogRead(NH3PIN);
        counter++;
        delay(2);
      }
      return rs/counter;
    case CH_RED:
      for(int i = 0; i < 100; i++) {
        rs += analogRead(COPIN);
        counter++;
        delay(2);
      }
      return rs/counter;
    case CH_OX:      
      for(int i = 0; i < 100; i++) {
        rs += analogRead(OXPIN);
        counter++;
        delay(2);
      }
      return rs/counter;

  }

  return 0;
}

void calibrateMICS() {
  // Continuously measure the resistance,
  // storing the last N measurements in a circular buffer.
  // Calculate the floating average of the last seconds.
  // If the current measurement is close to the average stop.

  // Seconds to keep stable for successful calibration
  // (Keeps smaller than 64 to prevent overflows)
  uint8_t seconds = 10;
  // Allowed delta for the average from the current value
  uint8_t delta = 2;

  // Circular buffer for the measurements
  uint16_t bufferNH3[seconds];
  uint16_t bufferRED[seconds];
  uint16_t bufferOX[seconds];
  // Pointers for the next element in the buffer
  uint8_t pntrNH3 = 0;
  uint8_t pntrRED = 0;
  uint8_t pntrOX = 0;
  // Current floating sum in the buffer
  uint16_t fltSumNH3 = 0;
  uint16_t fltSumRED = 0;
  uint16_t fltSumOX = 0;

  // Current measurements;
  uint16_t curNH3;
  uint16_t curRED;
  uint16_t curOX;

  // Flag to see if the channels are stable
  bool NH3stable = false;
  bool REDstable = false;
  bool OXstable = false;

  // Initialize buffer
  for (int i = 0; i < seconds; ++i) {
    bufferNH3[i] = 0;
    bufferRED[i] = 0;
    bufferOX[i] = 0;
  }

  do {
    // Wait a second
    delay(1000);
    Serial.print(".");
    // Read new resistances
    unsigned long rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(NH3PIN);
    }
    curNH3 = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(COPIN);
    }
    curRED = rs/3;
    rs = 0;
    delay(50);
    for (int i = 0; i < 3; i++) {
    delay(1);
    rs += analogRead(OXPIN);
    }
    curOX = rs/3;

    // Update floating sum by subtracting value
    // about to be overwritten and adding the new value.
    fltSumNH3 = fltSumNH3 + curNH3 - bufferNH3[pntrNH3];
    fltSumRED = fltSumRED + curRED - bufferRED[pntrRED];
    fltSumOX = fltSumOX + curOX - bufferOX[pntrOX];

    // Store new measurement in buffer
    bufferNH3[pntrNH3] = curNH3;
    bufferRED[pntrRED] = curRED;
    bufferOX[pntrOX] = curOX;

    // Determine new state of flags
    NH3stable = abs(fltSumNH3 / seconds - curNH3) < delta;
    REDstable = abs(fltSumRED / seconds - curRED) < delta;
    OXstable = abs(fltSumOX / seconds - curOX) < delta;

    // Advance buffer pointer
    pntrNH3 = (pntrNH3 + 1) % seconds ;
    pntrRED = (pntrRED + 1) % seconds;
    pntrOX = (pntrOX + 1) % seconds;

    //MikÃ¤ kestÃ¤Ã¤?
    if(!NH3stable) {
      Serial.print("(NH3:");
      Serial.print(abs(fltSumNH3 / seconds - curNH3));
      Serial.print(")");
    }
    if(!REDstable) {
      Serial.print("(RED:");
      Serial.print(abs(fltSumNH3 / seconds - curRED));
      Serial.print(")");
    }
    if(!OXstable) {
      Serial.print("(OX:");
      Serial.print(abs(fltSumNH3 / seconds - curOX));
      Serial.print(")");
    }

  } while (!NH3stable || !REDstable || !OXstable);

  NH3baseR = fltSumNH3 / seconds;
  REDbaseR = fltSumRED / seconds;
  OXbaseR = fltSumOX / seconds;

  // Store new base resistance values in EEPROM
}

uint16_t getBaseResistance(channel_t channel) {
  /* if (1 == __version) {
     // Version 1 can query every channel independently
     // Reply is 4 bytes long with relevant data in second and third byte
     switch (channel) {
       case CH_NH3:
         return getRuntimeData(CMD_V1_GET_R0_NH3, 4, 1);
       case CH_RED:
         return getRuntimeData(CMD_V1_GET_R0_RED, 4, 1);
       case CH_OX:
         return getRuntimeData(CMD_V1_GET_R0_OX, 4, 1);
     }
    }
    if (2 == __version) {
     // Version 2 uses the same command every time, but different offsets*/
     switch (channel) {
       case CH_NH3:
         return NH3baseR;
       case CH_RED:
         return REDbaseR;
       case CH_OX:
         return OXbaseR;
     }
  //  }
  
  return 0;
}


/**
   Calculates the current resistance ratio for the given channel.

   @param channel
          The channel to request resistance values from.
   @return The floating-point resistance ratio for the given channel.
*/
float getCurrentRatio(channel_t channel) {
  float baseResistance = (float) getBaseResistance(channel);
  float resistance = (float) getResistance(channel);

  return resistance / baseResistance * (1023.0 - baseResistance) / (1023.0 - resistance);
  

  return -1.0;
}

/**
   Measures the gas concentration in ppm for the specified gas.

   @param gas
          The gas to calculate the concentration for.
   @return The current concentration of the gas
           in parts per million (ppm).
*/
float measureMICS(gas_t gas) {
  float ratio;
  float c = 0;

  switch (gas) {
    case CO:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.179) * 4.385;
      break;
    case NO2:
      ratio = getCurrentRatio(CH_OX);
      c = pow(ratio, 1.007) / 6.855;
      break;
    case NH3:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -1.67) / 1.47;
      break;
    case C3H8:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -2.518) * 570.164;
      break;
    case C4H10:
      ratio = getCurrentRatio(CH_NH3);
      c = pow(ratio, -2.138) * 398.107;
      break;
    case CH4:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -4.363) * 630.957;
      break;
    case H2:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.8) * 0.73;
      break;
    case C2H5OH:
      ratio = getCurrentRatio(CH_RED);
      c = pow(ratio, -1.552) * 1.622;
      break;
  }

  return isnan(c) ? -1 : c;
}

void setup()
{
  Serial.begin(115200);

  //Sensor Gas MICS6814
  Serial.println("MICS-6814 Sensor Test v0.1");
  Serial.print("Calibrating Sensor");
  calibrateMICS();
  Serial.println("OK!");

  // Optional functionalities of EspMQTTClient
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overridded with enableHTTPWebUpdater("user", "password").
  client.enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true

  //Setting OLED
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
void onConnectionEstablished()
{
   
  Serial.print("NH3: ");
  Serial.print(getResistance(CH_NH3));
  Serial.print("/");
  Serial.print(getBaseResistance(CH_NH3));
  Serial.print(" = ");
  Serial.print(getCurrentRatio(CH_NH3));
  Serial.print(" => ");  
  Serial.print(measureMICS(NH3));
  Serial.println("ppm");
  delay(50);

  Serial.print("CO: ");
  Serial.print(getResistance(CH_RED));
  Serial.print("/");
  Serial.print(getBaseResistance(CH_RED));
  Serial.print(" = ");
  Serial.print(getCurrentRatio(CH_RED));
  Serial.print(" => ");  
  Serial.print(measureMICS(CO));
  Serial.println("ppm");
  delay(50);

  Serial.print("NO2: ");
  Serial.print(getResistance(CH_OX));
  Serial.print("/");
  Serial.print(getBaseResistance(CH_OX));
  Serial.print(" = ");
  Serial.print(getCurrentRatio(CH_OX));
  Serial.print(" => ");  
  Serial.print(measureMICS(NO2));
  Serial.println("ppm");
  delay(50);

  display.clearDisplay();
  display.setTextSize(2);       // Draw white text
  display.setTextColor(WHITE);
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Ammonia:")); 
  display.setTextSize(2);       // Draw white text
  display.setTextColor(WHITE);
  display.setCursor(0,40);             // Start at top-left corner
  display.println(measureMICS(NH3));
  display.setCursor(80,40);  
  display.println(F("ppm"));   
  display.display();
  delay(1000);

  client.setMaxPacketSize(512);
  StaticJsonDocument<256> doc;

  JsonObject info = doc.createNestedObject("info");
  info["device"] = "DEVICE_NAME";
  info["GMT"] = 7;
  info["date"] = "2022-10-21 11:09:17";
  info["farmcode"] = "SAMARINDA_1";
  info["zona"] = "0";

  JsonObject sensor = doc.createNestedObject("sensor");
  sensor["NH3"] = measureMICS(NH3);
  sensor["CO"] = measureMICS(CO);
  sensor["NO2"] = measureMICS(NO2);

  char payload[1024];
  serializeJson(doc, payload);
  // Publish a message to "mytopic/test"
  client.publish("main/status/main/SAMARINDA_1", payload); // You can activate the retain flag by setting the third parameter to true

  // Execute delayed instructions
 /* client.executeDelayed(5 * 1000, []() {
    client.publish("main/status/main/SAMARINDA_1", payload);
  });*/
}

void loop()
{
  client.loop();
  onConnectionEstablished();
}
