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
volatile unsigned long thrust, right, left = 0;
volatile bool RCmode = false;
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

  BNO_Init(&BNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  rtosTimer = xTimerCreate("RTOS_Timer", pdMS_TO_TICKS(100), pdTRUE, NULL, onTimer);
  xTimerStart(rtosTimer, 0);

  server.on("/", handleRoot);
  server.on("/imu", handleIMU);
  server.on("/data", handleData);

  xTaskCreatePinnedToCore(
      webServerTask,   // Task function
      "WebServerTask", // Task name
      8192,            // Stack size
      NULL,            // Parameter
      1,               // Priority
      NULL,            // Task handle
      0                // Core 0
  );
}

void webServerTask(void *pvParameters)
{
  WiFi.softAP(ssid, password);
  server.begin();
  while (true)
  {
    server.handleClient();
    vTaskDelay(10);
  }
}

void loop()
{
  Serial.println(RCmode);
  if (!RCmode)
  {
    ESC1.writeMicroseconds(highTime1);
    ESC2.writeMicroseconds(highTime1);
    ESC3.writeMicroseconds(highTime2);
    ESC4.writeMicroseconds(highTime2);
  }
  if (Serial.available() > 0 && Serial.available() < 32)
  {
    char command[32] = {0};
    Serial.readBytesUntil('\n', command, 32);
    if (strcmp(command, "GET_IMU") == 0)
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
  const char *html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>Triton Portal</title>
    <style>
    body { text-align: center; font-family: Arial, sans-serif; }
    #container { position: relative; width: 300px; height: 300px; border-radius: 50%; border: 2px solid black; margin: auto; }
    #ball { position: absolute; width: 20px; height: 20px; background: red; border-radius: 50%; }
    #boatContainer { width: 300px; height: 300px; margin: auto; position: relative; perspective: 800px; border: 1px solid #ccc; }
    #boat { position: absolute; top: 50%; left: 50%; width: 150px; height: 150px; transform: translate(-50%, -50%); transform-origin: center; }
    </style>
    <script>
    var xhr = new XMLHttpRequest();
    function updateData(){
      xhr.open('GET', '/data', true);
      xhr.onreadystatechange = function(){
        if(xhr.readyState == 4 && xhr.status == 200){
          var data = JSON.parse(xhr.responseText);
          document.getElementById('rc1').value = data.ch1;
          document.getElementById('rc2').value = data.ch2;
          document.getElementById('rc1Val').innerHTML = data.ch1;
          document.getElementById('rc2Val').innerHTML = data.ch2;
          var xInput = (data.ch1 - 1500) / 500;
          var yInput = -1 * (data.ch2 - 1500) / 500;
          var magnitude = Math.sqrt(xInput*xInput + yInput*yInput);
          if(magnitude > 1){ xInput /= magnitude; yInput /= magnitude; }
          var radius = 140;
          var x = xInput * radius + 150 - 10;
          var y = yInput * radius + 150 - 10;
          document.getElementById('ball').style.left = x + 'px';
          document.getElementById('ball').style.top = y + 'px';
        }
      };
      xhr.send();
    }
    function updateIMU(){
      var xhr_imu = new XMLHttpRequest();
      xhr_imu.open('GET', '/imu', true);
      xhr_imu.onreadystatechange = function(){
        if(xhr_imu.readyState == 4 && xhr_imu.status == 200){
          var imu = JSON.parse(xhr_imu.responseText);
          document.getElementById('pitch').innerHTML = imu.pitch;
          document.getElementById('roll').innerHTML = imu.roll;
          document.getElementById('heading').innerHTML = imu.heading;
          document.getElementById('gyroX').innerHTML = imu.gyroX;
          document.getElementById('gyroY').innerHTML = imu.gyroY;
          document.getElementById('gyroZ').innerHTML = imu.gyroZ;
          document.getElementById('accelX').innerHTML = imu.accelX;
          document.getElementById('accelY').innerHTML = imu.accelY;
          document.getElementById('accelZ').innerHTML = imu.accelZ;
          document.getElementById('magX').innerHTML = imu.magX;
          document.getElementById('magY').innerHTML = imu.magY;
          document.getElementById('magZ').innerHTML = imu.magZ;
    var Zrot = 0;
    if(imu.pitch < 0){
       Zrot = 180;
    } else {
       Zrot = 0;
    }
    var boat = document.getElementById('boat');
    if(boat){
       boat.style.transform = 'translate(-50%, -50%) ' + ' rotateX(' + imu.pitch + 'deg) rotateY(' + (-imu.roll) + 'deg) rotateZ(' + Zrot + 'deg)';
    }
        }
      };
      xhr_imu.send();
    }
    setInterval(updateData, 150);
    setInterval(updateIMU, 150);
    </script>
    </head>
    <body>
    <h1>Triton Portal</h1>
    <h2>Remote Control</h2>
    RC Channel 1: <input type='range' id='rc1' min='1000' max='2000' value='1500' disabled /> <span id='rc1Val'>1500</span><br><br>
    RC Channel 2: <input type='range' id='rc2' min='1000' max='2000' value='1500' disabled /> <span id='rc2Val'>1500</span><br><br>
    <div id='container'><div id='ball'></div></div>
    <h2>IMU Data</h2>
    <p>Pitch: <span id='pitch'>0</span>&#176</p>
    <p>Roll: <span id='roll'>0</span>&#176</p>
    <p>Heading: <span id='heading'>0</span>&#176</p>
    <p>Gyro: X: <span id='gyroX'>0</span> &#176/s, Y: <span id='gyroY'>0</span> &#176/s, Z: <span id='gyroZ'>0</span> &#176/s</p>
    <p>Accel: X: <span id='accelX'>0</span> m/s^2, Y: <span id='accelY'>0</span> m/s^2, Z: <span id='accelZ'>0</span> m/s^2</p>
    <p>Mag: X: <span id='magX'>0</span> uT, Y: <span id='magY'>0</span> uT, Z: <span id='magZ'>0</span> uT</p>
    <div id='boatContainer'>
      <svg id='boat' viewBox='0 0 100 100'>
        <polygon points='50,0 90,80 50,70 10,80' fill='blue' stroke='black' stroke-width='2'/>
      </svg>
    </div>
    </body>
    </html>
    )rawliteral";

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
  char jsonResponse[64] = "";
  snprintf(jsonResponse, sizeof(jsonResponse) / sizeof(char), "{\"ch1\":%d,\"ch2\":%d}", highTime1, highTime2);
  server.send(200, "application/json", jsonResponse);
}

void handleIMU()
{
  char jsonResponse[512] = "";

  snprintf(jsonResponse,
           sizeof(jsonResponse) / sizeof(char),
           "{"
           "\"pitch\":%d,"
           "\"roll\":%d,"
           "\"heading\":%d,"
           "\"gyroX\":%.1f,"
           "\"gyroY\":%.1f,"
           "\"gyroZ\":%.1f,"
           "\"accelX\":%.1f,"
           "\"accelY\":%.1f,"
           "\"accelZ\":%.1f,"
           "\"magX\":%.1f,"
           "\"magY\":%.1f,"
           "\"magZ\":%.1f"
           "}",
           int(hrpEulerData.p / 16.0),
           int(hrpEulerData.r / 16.0),
           int(hrpEulerData.h / 16.0),
           gyroData.x / 16.0,
           gyroData.y / 16.0,
           gyroData.z / 16.0,
           accelData.x / 100.0,
           accelData.y / 100.0,
           accelData.z / 100.0,
           magData.x / 16.0,
           magData.y / 16.0,
           magData.z / 16.0);

  server.send(200, "application/json", jsonResponse);
}

void SafeDelay(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms)
  {
    yield();
  }
}