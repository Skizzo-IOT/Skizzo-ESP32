#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "esp_camera.h"

#include "Grove_Motor_Driver_TB6612FNG.h"
#include <Wire.h>

//Camera
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

// Définition des constantes globales
// Numéro de port auquel est branchée la LED servant de flash.
#define PORT_LED_FLASH 4
MotorDriver motor;

// Socket
const char *hostname = "192.168.1.57";
const int port = 3000;

WebSocketsClient webSocket;

// const char *ssid = "miNetwork";
// const char *password = "mikamika";

const char *ssid = "lolosaint";
const char *password = "88C9BCD885";

unsigned long messageInterval = 5000;
bool connected = false;

#define DEBUG_SERIAL Serial

// UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
// UDP.write(reply);
// UDP.endPacket();

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16)
{
  const uint8_t *src = (const uint8_t *)mem;
  DEBUG_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
  for (uint32_t i = 0; i < len; i++)
  {
    if (i % cols == 0)
    {
      DEBUG_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
    }
    DEBUG_SERIAL.printf("%02X ", *src);
    src++;
  }
  DEBUG_SERIAL.printf("\n");
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{

  switch (type)
  {
  case WStype_DISCONNECTED:
    DEBUG_SERIAL.printf("[WSc] Disconnected!\n");
    connected = false;
    break;
  case WStype_CONNECTED:
  {
    DEBUG_SERIAL.printf("[WSc] Connected to url: %s\n", payload);
    connected = true;

    // send message to server when Connected
    DEBUG_SERIAL.println("[WSc] SENT: Connected");
    webSocket.sendTXT("Connected");
  }
  break;
  case WStype_TEXT:
    DEBUG_SERIAL.printf("[WSc] RESPONSE: %s\n", payload);
    break;
  case WStype_BIN:
    DEBUG_SERIAL.printf("[WSc] get binary length: %u\n", length);
    hexdump(payload, length);
    break;
  case WStype_PING:
    // pong will be send automatically
    DEBUG_SERIAL.printf("[WSc] get ping\n");
    break;
  case WStype_PONG:
    // answer to a ping we send
    DEBUG_SERIAL.printf("[WSc] get pong\n");
    break;
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
    break;
  }
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
  DEBUG_SERIAL.println((String) "Connexion du socket au serveur : " + hostname + ":" + port);

  webSocket.begin(hostname, port, "/esp32");
  // event handler
  webSocket.onEvent(webSocketEvent);

  // use HTTP Basic Authorization this is optional remove if not needed
  // socket.setAuthorization("username", "password");
}

void setupCamera() {
  DEBUG_SERIAL.println("Début de l'initialisation de la camera");

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    // config.fb_location = CAMERA_FB_IN_DRAM;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void setupMotor()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(14, 15);
  motor.init();
  pinMode(PORT_LED_FLASH, OUTPUT);
}

void startCameraServer();

void setup()
{
  DEBUG_SERIAL.begin(115200);

  //  DEBUG_SERIAL.setDebugOutput(true);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("Start");

  searchWifi();
  setupWifi();
  setupSocket();
  setupCamera();
  // setupMotor();
}

unsigned long lastUpdate = millis();
void loop()
{
  webSocket.loop();
  if (connected && lastUpdate + messageInterval < millis())
  {
    DEBUG_SERIAL.println("[WSc] SENT: Simple js client message!!");
    webSocket.sendTXT("Simple js client message!!");
    lastUpdate = millis();
  }

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