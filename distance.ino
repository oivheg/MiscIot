#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <NewPing.h>
#define MAX_DISTANCE 1000

const int TriggerPin = 19; //trigger pin
const int EchoPin = 21;   // echo pin
unsigned long Duration = 0;

NewPing sonar(TriggerPin, EchoPin, MAX_DISTANCE);

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the Wifi radio's status


int count = 0;
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.1.103";
int        port     = 1884;
const char topic[]  = "stairs/state";
unsigned long Distance_cm = 0;
const long interval = 1000;
unsigned long previousMillis = 0;


unsigned long time1;
unsigned long time2 = 0;
void setup ()
{
  //setup code

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }



  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  pinMode(TriggerPin, OUTPUT); // trigger is output pin
  pinMode(EchoPin, INPUT); // Echop is input pin
  Serial.begin(9600);  // serial baud rate
}

void loop () {
  mqttClient.poll();

  time1 = millis();
  if ((time1 - time2) > 60000) //every 60s
  {
    time2 = time1;
    TestWiFiConnection(); //test connection, and reconnect if necessary
    if (!mqttClient.connected()) {
      mqttClient.connect(broker, port);
    }
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    mqttClient.beginMessage(topic);
    //mqttClient.print(Distance_cm);
    mqttClient.print( sonar.ping_cm());


    mqttClient.endMessage();


  }
}

//unsigned long Distance (unsigned long time)
//{
//  unsigned long DistanceCalc;
//  DistanceCalc = (( time * 0.034) / 2);

//  return DistanceCalc;

//}


void TestWiFiConnection()
//test if always connected
{
  int StatusWiFi = WiFi.status();
  if (StatusWiFi == WL_CONNECTION_LOST || StatusWiFi == WL_DISCONNECTED || StatusWiFi == WL_SCAN_COMPLETED) //if no connection
  {

    WiFiConnect(); //if my SSID is present, connect

  }
}
void WiFiConnect()
//connect to my SSID
{
  status = WL_IDLE_STATUS;
  while (status != WL_CONNECTED)
  {
    status = WiFi.begin(ssid, pass);
    if (status == WL_CONNECTED) digitalWrite(9, HIGH); //LED ON to show connected
    else delay(500);
  }
}

void printData() {
  Serial.println("Board Information:");
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

}
