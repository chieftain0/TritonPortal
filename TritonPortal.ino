#include <Wire.h>
#include "BNO055_support.h"

#include <WiFi.h>
#include <ESP32Servo.h>
#include <WebServer.h>

// WiFi configuration
const char *ssid = "TritonPortal";
const char *password = "";

// IMU and hardware timer
struct bno055_t BNO;
struct bno055_euler hrpEulerData;
struct bno055_accel accelData;
struct bno055_mag magData;
struct bno055_gyro gyroData;
TimerHandle_t rtosTimer;

// Server configuration
WebServer server(80);

// ESC objects
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// GPIO pin definitions
const uint8_t ESCPins[4] = {6, 7, 15, 16};
const uint8_t ch1 = 13, ch2 = 12;
const uint8_t SDA_pin = 4, SCL_pin = 5;
const uint8_t ledPin = 21;

// Global variables
uint8_t ledBrightness = 128;
volatile unsigned long highTime1 = 1500, highTime2 = 1500;
volatile bool manualMode = false;

void onTimer(TimerHandle_t xTimer)
{
  bno055_read_euler_hrp(&hrpEulerData); // divide by 16 for °
  bno055_read_gyro_xyz(&gyroData);      // divide by 16 for °/s
  bno055_read_accel_xyz(&accelData);    // divide by 100 for m/s^2
  bno055_read_mag_xyz(&magData);        // divide by 16 for uT
}
void IRAM_ATTR pulseCh1()
{
  static unsigned long startTime = 0;
  if (REG_READ(GPIO_IN_REG) & (1 << ch1))
  {
    startTime = micros();
  }
  else
  {
    highTime1 = micros() - startTime;
  }
}
void IRAM_ATTR pulseCh2()
{
  static unsigned long startTime = 0;
  if (REG_READ(GPIO_IN_REG) & (1 << ch2))
  {
    startTime = micros();
  }
  else
  {
    highTime2 = micros() - startTime;
  }
}

void setup()
{
  Wire.begin(SDA_pin, SCL_pin);
  Serial.begin(115200);
  SafeDelay(100); // Small delay for i2c init

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  ESC1.attach(ESCPins[0]);
  ESC2.attach(ESCPins[1]);
  ESC3.attach(ESCPins[2]);
  ESC4.attach(ESCPins[3]);

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);

  ledcAttach(ledPin, 5000, 8);

  attachInterrupt(digitalPinToInterrupt(ch1), pulseCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2), pulseCh2, CHANGE);

  server.on("/", handleRoot);
  server.on("/ESC", handleESC);
  server.on("/data", handleData);
  server.on("/led", handleLed);
  server.on("/switch", handleSwitch);

  xTaskCreatePinnedToCore(
      webServerTask,   // Task function
      "WebServerTask", // Task name
      8192,            // Stack size
      NULL,            // Parameter
      1,               // Priority
      NULL,            // Task handle
      0                // Core 0
  );

  BNO_Init(&BNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  rtosTimer = xTimerCreate("RTOS_Timer", pdMS_TO_TICKS(100), pdTRUE, NULL, onTimer);
  xTimerStart(rtosTimer, 0);
}

void webServerTask(void *pvParameters)
{
  server.begin();
  while (true)
  {
    server.handleClient();
    vTaskDelay(1);
  }
}

void loop()
{
  if (!manualMode)
  {
    ESC1.writeMicroseconds(highTime1);
    ESC2.writeMicroseconds(highTime1);
    ESC3.writeMicroseconds(highTime2);
    ESC4.writeMicroseconds(highTime2);
  }

  // Print the values from IMU
  Serial.println("HRP");
  Serial.print("Pitch: " + String(hrpEulerData.p / 16.0) + "° ");
  Serial.print("Roll: " + String(hrpEulerData.r / 16.0) + "° ");
  Serial.println("Heading: " + String(hrpEulerData.h / 16.0) + "° ");

  Serial.println("Gyro");
  Serial.print("X: " + String(gyroData.x / 16.0) + " °/s ");
  Serial.print("Y: " + String(gyroData.y / 16.0) + " °/s ");
  Serial.println("Z: " + String(gyroData.z / 16.0) + " °/s ");

  Serial.println("Accel");
  Serial.print("X: " + String(accelData.x / 100.0) + " m/s^2 ");
  Serial.print("Y: " + String(accelData.y / 100.0) + " m/s^2 ");
  Serial.println("Z: " + String(accelData.z / 100.0) + " m/s^2 ");

  Serial.println("Mag");
  Serial.print("X: " + String(magData.x / 16.0) + " uT ");
  Serial.print("Y: " + String(magData.y / 16.0) + " uT ");
  Serial.println("Z: " + String(magData.z / 16.0) + " uT ");

  SafeDelay(5000);
}

void handleRoot()
{
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Triton Portal</title>";
  html += "<style>";
  html += "body { text-align: center; font-family: Arial, sans-serif; }";
  html += "#container { position: relative; width: 300px; height: 300px; border-radius: 50%; border: 2px solid black; margin: auto; }";
  html += "#ball { position: absolute; width: 20px; height: 20px; background: red; border-radius: 50%; }";
  html += "input:disabled { background: #ddd; }";
  html += "</style>";
  html += "<script>";
  html += "var xhr = new XMLHttpRequest();";
  html += "function updateESC(ESC, value){";
  html += "  xhr.open('GET', '/ESC?ESC=' + ESC + '&value=' + value, true);";
  html += "  xhr.send();";
  html += "}";
  html += "function updateLED(value){";
  html += "  xhr.open('GET', '/led?value=' + value, true);";
  html += "  xhr.send();";
  html += "}";
  html += "function updateSwitch(checked){";
  html += "  var state = checked ? 'on' : 'off';";
  html += "  xhr.open('GET', '/switch?state=' + state, true);";
  html += "  xhr.send();";
  html += "  var sliders = document.getElementsByClassName('escSlider');";
  html += "  for (var i = 0; i < sliders.length; i++) {";
  html += "    sliders[i].disabled = !checked;";
  html += "  }";
  html += "}";
  html += "function updateData(){";
  html += "  xhr.open('GET', '/data', true);";
  html += "  xhr.onreadystatechange = function(){";
  html += "    if(xhr.readyState == 4 && xhr.status == 200){";
  html += "      var data = JSON.parse(xhr.responseText);";
  html += "      document.getElementById('rc1').value = data.ch1;";
  html += "      document.getElementById('rc2').value = data.ch2;";
  html += "      document.getElementById('rc1Val').innerHTML = data.ch1;";
  html += "      document.getElementById('rc2Val').innerHTML = data.ch2;";
  html += "      var xInput = (data.ch1 - 1500) / 500;";
  html += "      var yInput = -1 * (data.ch2 - 1500) / 500;";
  html += "      var magnitude = Math.sqrt(xInput*xInput + yInput*yInput);";
  html += "      if (magnitude > 1) { xInput /= magnitude; yInput /= magnitude; }";
  html += "      var radius = 140;";
  html += "      var x = xInput * radius + 150 - 10;";
  html += "      var y = yInput * radius + 150 - 10;";
  html += "      document.getElementById('ball').style.left = x + 'px';";
  html += "      document.getElementById('ball').style.top = y + 'px';";
  html += "      if(!document.getElementById('manualControl').checked){";
  html += "         document.getElementById('esc1').value = data.ch1;";
  html += "         document.getElementById('esc2').value = data.ch1;";
  html += "         document.getElementById('esc3').value = data.ch2;";
  html += "         document.getElementById('esc4').value = data.ch2;";
  html += "      }";
  html += "    }";
  html += "  };";
  html += "  xhr.send();";
  html += "}";
  html += "setInterval(updateData, 150);";
  html += "</script></head><body>";
  html += "<h1>Triton Portal</h1>";
  html += "<h2>ESC Control</h2>";
  html += "<label for='manualControl'>Manual Control: </label>";
  html += "<input type='checkbox' id='manualControl' onchange='updateSwitch(this.checked)' />";
  html += "<br><br>";
  for (int i = 1; i <= 4; i++)
  {
    html += "ESC " + String(i) + ": ";
    html += "<input type='range' class='escSlider' id='esc" + String(i) + "' min='1000' max='2000' value='1500' disabled oninput='updateESC(" + String(i) + ", this.value)' />";
    html += "<br><br>";
  }
  html += "<h2>LED Control</h2>";
  html += "LED Brightness: <input type='range' min='0' max='255' value='" + String(ledBrightness) + "' oninput='updateLED(this.value)' />";
  html += "<br><br>";
  html += "<h2>RC Channel Outputs</h2>";
  html += "RC Channel 1: <input type='range' id='rc1' min='1000' max='2000' value='1500' disabled /> <span id='rc1Val'>1500</span><br><br>";
  html += "RC Channel 2: <input type='range' id='rc2' min='1000' max='2000' value='1500' disabled /> <span id='rc2Val'>1500</span><br><br>";
  html += "<div id='container'><div id='ball'></div></div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleData()
{
  if (highTime1 < 1000)
    highTime1 = 1000;
  else if (highTime1 > 2000)
    highTime1 = 2000;
  if (highTime2 < 1000)
    highTime2 = 1000;
  else if (highTime2 > 2000)
    highTime2 = 2000;
  String json = "{\"ch1\":" + String(highTime1) + ",\"ch2\":" + String(highTime2) + "}";
  server.send(200, "application/json", json);
}

void handleESC()
{
  if (!server.hasArg("ESC") || !server.hasArg("value"))
  {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  int ESCNum = server.arg("ESC").toInt();
  int ESCValue = server.arg("value").toInt();
  if (ESCValue < 1000 || ESCValue > 2000)
  {
    server.send(400, "text/plain", "Value out of range");
    return;
  }
  switch (ESCNum)
  {
  case 1:
    ESC1.writeMicroseconds(ESCValue);
    break;
  case 2:
    ESC2.writeMicroseconds(ESCValue);
    break;
  case 3:
    ESC3.writeMicroseconds(ESCValue);
    break;
  case 4:
    ESC4.writeMicroseconds(ESCValue);
    break;
  default:
    server.send(400, "text/plain", "Invalid ESC number");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleLed()
{
  if (!server.hasArg("value"))
  {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  ledBrightness = server.arg("value").toInt();
  ledcWrite(ledPin, ledBrightness);
  server.send(200, "text/plain", "OK");
}

void handleSwitch()
{
  if (!server.hasArg("state"))
  {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  String state = server.arg("state");
  if (state == "off")
  {
    manualMode = false;
  }
  else if (state == "on")
  {
    manualMode = true;
  }
  else
  {
    server.send(400, "text/plain", "Invalid state");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void SafeDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    yield();
  }
}
