#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HTU21D.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
//#include "GravityTDS.h"

#define TdsSensorPin 34
#define PhSensorPin 35
//GravityTDS gravityTds;

const char *ssid = "POCO";
const char *password = "11112222";
const char *mqtt_server = "test.mosquitto.org";
const int   mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";


WiFiClient espClient;
PubSubClient client(espClient);

unsigned long sendDataPrevMillis = 0;

float voltage = 0.0;
//PH SENSOR DECLARATION
float calibration_value = 20.24 - 0.7; //21.34 - 0.7
int phval = 0;
unsigned long int avgval;
float ph_act = 0;
int buffer_arr[10], temp;
float temperature = 0.0 , humidity = 0.0;

// PH variable
unsigned long int avgValueForPH;  //Store the average value of the ph sensor feedback
int pHbuffer[10], tempValueForPH;
float phValue;

HTU21D sensor;
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
#define TdsSensorPin 34
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;

// mqtt callback
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// mqtt reconnect
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attemping MQTT connection...");
    //    Create random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    //    Attemp to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // subscribe topic
      client.subscribe("aquaponicTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//setup wifi
void setup_wifi() {
  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void setup() {
  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
  Serial.begin(115200);
  //  pinMode(TdsSensorPin, INPUT);
//  pinMode(PhSensorPin, INPUT);
  sensors.begin();
  sensor.begin();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  // subscribe topic
  client.subscribe("aquaponicTopic");
}


void loop() {

  // Reconnecting.......
  if (!client.connected()) {
    reconnect();
  }
  if (!client.loop()) {
    client.connect("aquaponicTopic", mqtt_user, mqtt_pass);
  }

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) { //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();

    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 10000) {
    printTimepoint = millis();

    // get temperature and humidity
    if (sensor.measure()) {
      temperature = sensor.getTemperature();
      humidity = sensor.getHumidity();

      //      Serial.print("Temperature (°C): ");
      //      Serial.println(temperature);
      //
      //      Serial.print("Humidity (%RH): ");
      //      Serial.println(humidity);
    }
    //Water Temperature
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);
    float temperatureF = sensors.getTempFByIndex(0);
    //    gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
    //    gravityTds.update();  //sample and calculate
    //    tdsValue = gravityTds.getTdsValue();  // then get the value
    //    Serial.print(tdsValue,0);
    Serial.println("ppm");
    //    delay(4000);

    float ph_act;
    //Serial.println(analogRead(34));

    for (int i = 0; i < 10; i++) {
      buffer_arr[i] = analogRead(PhSensorPin);
      delay(30);
    }
    for (int i = 0; i < 9; i++) {
      for (int j = i + 1; j < 10; j++)
      {
        if (buffer_arr[i] > buffer_arr[j])
        {
          temp = buffer_arr[i];
          buffer_arr[i] = buffer_arr[j];
          buffer_arr[j] = temp;
        }
      }
    }
    avgval = 0;
    for (int i = 2; i < 8; i++)
      avgval += buffer_arr[i];
    float volt = (float)avgval * 3.3 / 4096.0 / 6;
    //Serial.print("Voltage: ");
    //Serial.println(volt);
    ph_act = -5.70 * volt + calibration_value;

    // set cursor to first column, first row
    lcd.setCursor(0, 0);
    // print message
    lcd.print((String)temperatureC + " Celcius");

    // clears the display to print new message

    // set cursor to first column, second row
    lcd.setCursor(0, 1);
    lcd.print((String)tdsValue + " TDS");
    delay(1000);


    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;



    }
    //    Serial.print("TDS ");
    //    Serial.println(tdsValue);
    // Send JSON to mqtt broker
    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& root = JSONbuffer.createObject();

    root["t"] = temperature;
    root["h"] = humidity;
    root["wt"] = temperatureC;
    root["tds"] = tdsValue;
    root["ph"] = phRead(PhSensorPin)/8;

    char JSONmessageBuffer[200];
    root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending message to MQTT topic..");
    //  Serial.println(JSONmessageBuffer);
    root.printTo(Serial);

    if (client.publish("aquaponicTopic", JSONmessageBuffer) == true) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.println("Success sending message");
    } else {
      Serial.println("Error sending message");
    }
  }
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

double readPh(double tds) {
  double p = tds / 10;
  double x = 7.12;
  if (p > 14.00) {
    return p;
  } else {
    return x;
  }
}

double phRead(int phPin) {
  for (int i = 0; i < 10; i++) //Get 10 sample value from the sensor for smooth the value
  {
    pHbuffer[i] = analogRead(phPin);
    //  Serial.println(pHbuffer[i]);
    delay(10);
  }
  avgValueForPH = 0;
  for (int i = 2; i < 8; i++)               //take the average value of 6 center sample
    avgValueForPH += pHbuffer[i];
  phValue = (float)avgValueForPH * 3.33 / 1024 / 6; //convert the analog into millivolt
  phValue = 4.7 * phValue;                  //convert the millivolt into pH value
  return phValue;
}
