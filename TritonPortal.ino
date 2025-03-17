#include <Wire.h>
#include "BNO055_support.h"

#include <WiFi.h>
#include <ESP32Servo.h>
#include <WebServer.h>

#include <ArduinoJson.h>

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

// Global variables
volatile unsigned long highTime1 = 1500, highTime2 = 1500;
volatile bool manualMode = false;
JsonDocument response;

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

  ESC1.attach(ESCPins[0]);
  ESC2.attach(ESCPins[1]);
  ESC3.attach(ESCPins[2]);
  ESC4.attach(ESCPins[3]);

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ch1), pulseCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2), pulseCh2, CHANGE);

  server.on("/", handleRoot);
  server.on("/ESC", handleESC);
  server.on("/imu", handleIMU);
  server.on("/data", handleData);
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
  WiFi.softAP(ssid, password);
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
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    if (command == "GET_IMU")
    {
      JsonObject IMU = response["IMU"].to<JsonObject>();
      IMU["pitch"] = hrpEulerData.p / 16.0;
      IMU["roll"] = hrpEulerData.r / 16.0;
      IMU["heading"] = hrpEulerData.h / 16.0;
      IMU["accel_x"] = accelData.x / 100.0;
      IMU["accel_y"] = accelData.y / 100.0;
      IMU["accel_z"] = accelData.z / 100.0;
      IMU["gyro_x"] = gyroData.x / 16.0;
      IMU["gyro_y"] = gyroData.y / 16.0;
      IMU["gyro_z"] = gyroData.z / 16.0;
      IMU["mag_x"] = magData.x / 16.0;
      IMU["mag_y"] = magData.y / 16.0;
      IMU["mag_z"] = magData.z / 16.0;

      response.shrinkToFit();

      serializeJson(response, Serial);
      Serial.print("\n");
    }
    else
    {
      Serial.print("UNKNOWN COMMAND\n");
    }
  }
}

void handleRoot()
{
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Triton Portal</title>";
  html += "<style>";
  html += "body { text-align: center; font-family: Arial, sans-serif; }";
  html += "#container { position: relative; width: 300px; height: 300px; border-radius: 50%; border: 2px solid black; margin: auto; }";
  html += "#ball { position: absolute; width: 20px; height: 20px; background: red; border-radius: 50%; }";
  html += "#boatContainer { width: 300px; height: 300px; margin: auto; position: relative; perspective: 800px; border: 1px solid #ccc; }";
  html += "#boat { position: absolute; top: 50%; left: 50%; width: 150px; height: 150px; transform: translate(-50%, -50%); transform-origin: center; }";
  html += "</style>";
  html += "<script>";
  html += "var xhr = new XMLHttpRequest();";
  html += "function updateESC(ESC, value){";
  html += "  xhr.open('GET', '/ESC?ESC=' + ESC + '&value=' + value, true);";
  html += "  xhr.send();";
  html += "}";
  html += "function updateSwitch(checked){";
  html += "  var state = checked ? 'on' : 'off';";
  html += "  xhr.open('GET', '/switch?state=' + state, true);";
  html += "  xhr.send();";
  html += "  var sliders = document.getElementsByClassName('escSlider');";
  html += "  for(var i = 0; i < sliders.length; i++) {";
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
  html += "      if(magnitude > 1){ xInput /= magnitude; yInput /= magnitude; }";
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
  html += "function updateIMU(){";
  html += "  var xhr_imu = new XMLHttpRequest();";
  html += "  xhr_imu.open('GET', '/imu', true);";
  html += "  xhr_imu.onreadystatechange = function(){";
  html += "    if(xhr_imu.readyState == 4 && xhr_imu.status == 200){";
  html += "      var imu = JSON.parse(xhr_imu.responseText);";
  html += "      document.getElementById('pitch').innerHTML = imu.pitch;";
  html += "      document.getElementById('roll').innerHTML = imu.roll;";
  html += "      document.getElementById('heading').innerHTML = imu.heading;";
  html += "      document.getElementById('gyroX').innerHTML = imu.gyroX;";
  html += "      document.getElementById('gyroY').innerHTML = imu.gyroY;";
  html += "      document.getElementById('gyroZ').innerHTML = imu.gyroZ;";
  html += "      document.getElementById('accelX').innerHTML = imu.accelX;";
  html += "      document.getElementById('accelY').innerHTML = imu.accelY;";
  html += "      document.getElementById('accelZ').innerHTML = imu.accelZ;";
  html += "      document.getElementById('magX').innerHTML = imu.magX;";
  html += "      document.getElementById('magY').innerHTML = imu.magY;";
  html += "      document.getElementById('magZ').innerHTML = imu.magZ;";
  html += "var Zrot = 0;";
  html += "if(imu.pitch < 0){";
  html += "   Zrot = 180;";
  html += "} else {";
  html += "   Zrot = 0;";
  html += "}";
  html += "var boat = document.getElementById('boat');";
  html += "if(boat){";
  html += "   boat.style.transform = 'translate(-50%, -50%) ' + ' rotateX(' + imu.pitch + 'deg) rotateY(' + (-imu.roll) + 'deg) rotateZ(' + Zrot + 'deg)';";
  html += "}";
  html += "    }";
  html += "  };";
  html += "  xhr_imu.send();";
  html += "}";
  html += "setInterval(updateData, 150);";
  html += "setInterval(updateIMU, 150);";
  html += "</script>";
  html += "</head><body>";
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
  html += "<h2>Remote Control</h2>";
  html += "RC Channel 1: <input type='range' id='rc1' min='1000' max='2000' value='1500' disabled /> <span id='rc1Val'>1500</span><br><br>";
  html += "RC Channel 2: <input type='range' id='rc2' min='1000' max='2000' value='1500' disabled /> <span id='rc2Val'>1500</span><br><br>";
  html += "<div id='container'><div id='ball'></div></div>";
  html += "<h2>IMU Data</h2>";
  html += "<p>Pitch: <span id='pitch'>0</span>&#176</p>";
  html += "<p>Roll: <span id='roll'>0</span>&#176</p>";
  html += "<p>Heading: <span id='heading'>0</span>&#176</p>";
  html += "<p>Gyro: X: <span id='gyroX'>0</span> &#176/s, Y: <span id='gyroY'>0</span> &#176/s, Z: <span id='gyroZ'>0</span> &#176/s</p>";
  html += "<p>Accel: X: <span id='accelX'>0</span> m/s^2, Y: <span id='accelY'>0</span> m/s^2, Z: <span id='accelZ'>0</span> m/s^2</p>";
  html += "<p>Mag: X: <span id='magX'>0</span> uT, Y: <span id='magY'>0</span> uT, Z: <span id='magZ'>0</span> uT</p>";
  html += "<div id='boatContainer'>";
  html += "  <svg id='boat' viewBox='0 0 100 100'>";
  html += "    <polygon points='50,0 90,80 50,70 10,80' fill='blue' stroke='black' stroke-width='2'/>";
  html += "  </svg>";
  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleData()
{
  if (highTime1 < 1000)
  {
    highTime1 = 1000;
  }
  else if (highTime1 > 2000)
  {
    highTime1 = 2000;
  }
  if (highTime2 < 1000)
  {
    highTime2 = 1000;
  }
  else if (highTime2 > 2000)
  {
    highTime2 = 2000;
  }
  String json = "{\"ch1\":" + String(highTime1) + ",\"ch2\":" + String(highTime2) + "}";
  server.send(200, "application/json", json);
}

void handleIMU()
{
  String json = "{";
  json += "\"pitch\":" + String(int(hrpEulerData.p / 16.0)) + ",";
  json += "\"roll\":" + String(int(hrpEulerData.r / 16.0)) + ",";
  json += "\"heading\":" + String(int(hrpEulerData.h / 16.0)) + ",";
  json += "\"gyroX\":" + String(gyroData.x / 16.0, 1) + ",";
  json += "\"gyroY\":" + String(gyroData.y / 16.0, 1) + ",";
  json += "\"gyroZ\":" + String(gyroData.z / 16.0, 1) + ",";
  json += "\"accelX\":" + String(accelData.x / 100.0, 1) + ",";
  json += "\"accelY\":" + String(accelData.y / 100.0, 1) + ",";
  json += "\"accelZ\":" + String(accelData.z / 100.0, 1) + ",";
  json += "\"magX\":" + String(magData.x / 16.0, 1) + ",";
  json += "\"magY\":" + String(magData.y / 16.0, 1) + ",";
  json += "\"magZ\":" + String(magData.z / 16.0, 1);
  json += "}";
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