#include "VOneMqttClient.h"
#include "DHT.h"

#define MANUAL_TIMEOUT 10000   //Delay after manual control LED

bool manualControl = false;
unsigned long manualTimeout = 0;

int MinDepthValue = 4095;
int MaxDepthValue = 2200;
int MinDepth = 0;
int MaxDepth = 45;
int depth = 0;

//define device id
const char* DHT11Sensor = "b49c7dc1-5ace-477f-b354-73a6a803e411";  //deviceID for the DHT11 sensor
const char* RainSensor = "33e6c418-b460-4af5-b050-7fa51d0029f5";   //deviceID for the rain sensor
const char* WaterLevel = "9810b8f9-8dcf-471a-8aff-0df083315879";   //deviceID for the WaterLevel sensor
const char* ledGreen = "9804b33b-e9bc-416a-9774-34b934ebfd62";     //deviceID for the GREEN LED
const char* ledRed = "b780d38b-0abf-4418-8a35-283c321d1529";       //deviceID for the RED LED

const int dht11Pin = 42;     //Pin for DHT11
const int rainPin = 4;       //Pin for Rainsensor
const int depthPin = 14;     //Pin for Depth Sensor
const int buzzer = 12;       //Pin for Buzzer
const int ledGreenPin = 38;  //Pin for LED Green
const int ledRedPin = 47;    //Pin for LED Red

//input sensor
#define DHTTYPE DHT11
DHT dht(dht11Pin, DHTTYPE);

VOneMqttClient voneClient;

unsigned long lastMsgTime = 0;

void setup_wifi() {

  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) {
  Serial.print("Main received callback : ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";

  JSONVar commandObject = JSON.parse(actuatorCommand);
  JSONVar keys = commandObject.keys();

  if (String(actuatorDeviceId) == ledGreen || String(actuatorDeviceId) == ledRed) {
    // Parse the command
    String key = "";
    bool commandValue = false;
    for (int i = 0; i < keys.length(); i++) {
      key = (const char*)keys[i];
      commandValue = (bool)commandObject[keys[i]];
      Serial.print("Key : ");
      Serial.println(key.c_str());
      Serial.print("Value : ");
      Serial.println(commandValue);
    }

    // Determine which LED to control
    int targetPin = (String(actuatorDeviceId) == ledGreen) ? ledGreenPin : ledRedPin;

    // Set LED state
    if (commandValue) {
      Serial.println("Manual Control: LED ON");
      digitalWrite(targetPin, HIGH);
    } else {
      Serial.println("Manual Control: LED OFF");
      digitalWrite(targetPin, LOW);
    }

    // Activate manual control and reset timeout
    manualControl = true;
    manualTimeout = millis();

    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, true);
  }
}

void setup() {

  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);
  //sensor
  dht.begin();
  pinMode(rainPin, INPUT);
  pinMode(depth, INPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);
}

void loop() {
  if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "DHTSensor Fail";
    voneClient.publishDeviceStatusEvent(DHT11Sensor, true);
    voneClient.publishDeviceStatusEvent(RainSensor, true);
    voneClient.publishDeviceStatusEvent(WaterLevel, true);
  }
  voneClient.loop();

  unsigned long cur = millis();
  if (cur - lastMsgTime > INTERVAL) {
    lastMsgTime = cur;

    // Publish telemetry data
    float h = dht.readHumidity();
    int t = dht.readTemperature();
    JSONVar payloadObject;
    payloadObject["Humidity"] = h;
    payloadObject["Temperature"] = t;
    voneClient.publishTelemetryData(DHT11Sensor, payloadObject);

    int raining = !digitalRead(rainPin);
    voneClient.publishTelemetryData(RainSensor, "Raining", raining);

    int depthValue = analogRead(depthPin);
    depth = map(depthValue, MinDepthValue, MaxDepthValue, MinDepth, MaxDepth);
    voneClient.publishTelemetryData(WaterLevel, "Depth", depth);

    // Automatic control logic
    if (!manualControl || (cur - manualTimeout > MANUAL_TIMEOUT)) {
      manualControl = false;  // Reset manual control

      bool exceedThreshold = false;
      bool alert = false;
      if (raining && depth > 45) {  
        Serial.println("Rain detected!");
        alert = true;
      }
      if (depth > 45) {  // Water level threshold
        Serial.println("High Water Level Detected!");
        exceedThreshold = true;
      }
      if (alert) {
        Serial.println("Flood alert. Turning on RED LED.");
        digitalWrite(ledRedPin, HIGH);   // Turn on Red LED
        digitalWrite(ledGreenPin, LOW);  // Turn off Green LED
        tone(buzzer, 200);               // Turn On Buzzer
      } else {
        Serial.println("All conditions normal. Turning on GREEN LED.");
        digitalWrite(ledRedPin, LOW);     // Turn off Red LED
        digitalWrite(ledGreenPin, HIGH);  // Turn on Green LED
        noTone(buzzer);                   // Turn Off Buzzer
      }
    }
  }
}
