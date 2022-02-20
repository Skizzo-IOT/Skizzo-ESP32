#include <Arduino.h>

#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiMulti.h>

#include <WebSocketsClient.h>
#include <SocketIOclient.h>

// Définition des constantes globales
// Numéro de port auquel est branchée la LED servant de flash.
#define PORT_LED_FLASH 4
#define USER_SERIAL Serial

const char *hostname = "192.168.1.57";
const int port = 3000;
const char *ssid = "lolosaint";
const char *password = "88C9BCD885";
SocketIoClient socket;

WiFiClient WiFiMulti;

MotorDriver motor;

void event(const char *payload, size_t length)
{
  Serial.println((String) "Message reçu : " + payload);
}

void searchWifi()
{
  int numberOfNetwork = WiFi.scanNetworks();
  Serial.println("-----");
  for (int i = 0; i < numberOfNetwork; i++)
  {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));
    Serial.print("Signal Strength: ");
    Serial.println(WiFi.RSSI(i));
    Serial.println("-------------");
  }
}

void setupWifi()
{
  Serial.println((String) "Connexion au réseau wifi : " + ssid);
  // WiFi.mode(WIFI_STA_AP);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(500);
  }
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.println("");
  Serial.println("WiFi connecté");
  Serial.print("MAC : ");
  Serial.println(WiFi.macAddress());
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

void setupSocket()
{
  Serial.println((String) "Connexion du socket au serveur : " + hostname + ":" + port);
  // socket.on("event", event);
  // socket.begin(hostname, port);

  socket.on("message", event);
  socket.on("light_on", event);
  socket.emit("message", "hello from esp");
  socket.begin(hostname, port, "/");

  // use HTTP Basic Authorization this is optional remove if not needed
  // socket.setAuthorization("username", "password");
}

void setupMotor()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(14, 15);
  motor.init();
  pinMode(PORT_LED_FLASH, OUTPUT);
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Start");

  searchWifi();
  setupWifi();
  setupSocket();
  // setupMotor();

  // socket.on("message", handleMessage);
  // socket.on("light_on", handleMessage);
  // socket.emit("message", "hello from esp");

  // // Socket
  // socket.begin(hostname, port);
  // event handler
  // socketIO.onEvent(socketIOEvent);
}

void loop()
{
  socket.loop();
  // Serial.println("Test");
  // digitalWrite(PORT_LED_FLASH, HIGH);
  // delay(1000);
  // digitalWrite(PORT_LED_FLASH, LOW);
  // delay(1000);

  // // drive 2 dc motors at speed=255, clockwise
  // Serial.println("run at speed=255");
  // motor.dcMotorRun(MOTOR_CHA, 255);
  // motor.dcMotorRun(MOTOR_CHB, 255);
  // delay(1000);

  // // brake
  // Serial.println("brake");
  // motor.dcMotorBrake(MOTOR_CHA);
  // motor.dcMotorBrake(MOTOR_CHB);
  // delay(1000);

  // // drive 2 dc motors at speed=200, anticlockwise
  // Serial.println("run at speed=-200");
  // motor.dcMotorRun(MOTOR_CHA, -200);
  // motor.dcMotorRun(MOTOR_CHB, -200);
  // delay(1000);

  // // stop 2 motors
  // Serial.println("stop");
  // motor.dcMotorStop(MOTOR_CHA);
  // motor.dcMotorStop(MOTOR_CHB);
  // delay(1000);
}