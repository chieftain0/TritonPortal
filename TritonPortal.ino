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
Servo ESC[4] = {Servo(), Servo(), Servo(), Servo()};

// GPIO pin definitions
const gpio_num_t ESCPins[4] = {GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_15, GPIO_NUM_16};
volatile int ESCvalues[4] = {1500, 1500, 1500, 1500};
const uint8_t ch1 = 13, ch2 = 12, ch3 = 11;
const uint8_t SDA_pin = 4, SCL_pin = 5;
const gpio_num_t relayPin = GPIO_NUM_10;
const gpio_num_t LEDpin = GPIO_NUM_21;

// Global variables
#define BUFFER_SIZE 1024
#define MIN_ESC_VALUE 1000
#define MAX_ESC_VALUE 2000
volatile unsigned long highTime1 = 1500, highTime2 = 1500, highTime3 = 1000;
volatile bool RCmode = 1;
volatile bool ConveyorRelayState = 0;
volatile double ConveyorDutyCycle = 0.5;

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

void IRAM_ATTR pulseCh3()
{
  static unsigned long startTime = 0;
  if (REG_READ(GPIO_IN_REG) & (1 << ch3))
  {
    startTime = micros();
  }
  else
  {
    highTime3 = micros() - startTime;
  }
}

void setup()
{
  // Start serial communication
  Serial.begin(115200);

  // Start I2C communication
  Wire.begin(SDA_pin, SCL_pin);

  // Set the drive strength for ESC pins
  gpio_set_drive_capability(ESCPins[0], GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(ESCPins[1], GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(ESCPins[2], GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(ESCPins[3], GPIO_DRIVE_CAP_3);

  // Setup ESCs
  ESC[0].attach(ESCPins[0]);
  ESC[0].writeMicroseconds(ESCvalues[0]);
  ESC[1].attach(ESCPins[1]);
  ESC[1].writeMicroseconds(ESCvalues[1]);
  ESC[2].attach(ESCPins[2]);
  ESC[2].writeMicroseconds(ESCvalues[2]);
  ESC[3].attach(ESCPins[3]);
  ESC[3].writeMicroseconds(ESCvalues[3]);

  // Setup RC inputs
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);

  // Setup relay and LED pins
  pinMode(LEDpin, OUTPUT);
  ledcAttach(relayPin, 250, 8);

  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(ch1), pulseCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2), pulseCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3), pulseCh3, CHANGE);

  // Setup IMU and routines
  BNO_Init(&BNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // Setup web server and RTOS
  server.on("/", handleRoot);
  server.on("/imu", handleIMU);
  server.on("/rc", handleRC);
  server.on("/esc", handleESC);
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
  WiFi.softAP(ssid, password); // Start WiFi
  server.begin();              // Start server
  while (true)
  {
    server.handleClient(); // Handle HTTP requests
    vTaskDelay(10);        // Allow other tasks (if any)
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    JsonDocument jsonCommand;
    char command[BUFFER_SIZE] = {0};
    Serial.readBytesUntil('\n', command, BUFFER_SIZE);

    DeserializationError jsonError = deserializeJson(jsonCommand, command, BUFFER_SIZE);

    if (jsonError)
    {
      Serial.print("UNKNOWN_COMMAND: ");
      Serial.println(jsonError.c_str());
      return;
    }

    if (jsonCommand.containsKey("SET_RC_MODE"))
    {
      RCmode = bool(jsonCommand["SET_RC_MODE"]);
    }

    if (jsonCommand.containsKey("SET_CONVEYOR_DUTY_CYCLE"))
    {
      if (double(jsonCommand["SET_CONVEYOR_DUTY_CYCLE"]) >= 0.0 && double(jsonCommand["SET_CONVEYOR_DUTY_CYCLE"]) <= 1.0)
      {
        ConveyorDutyCycle = double(jsonCommand["SET_CONVEYOR_DUTY_CYCLE"]);
      }
      else
      {
        Serial.print("CONVEYOR_DUTY_CYCLE_VALUE_ERROR\r\n");
      }
    }

    if (jsonCommand.containsKey("SET_CONVEYOR_MODE") && RCmode == false)
    {
      ConveyorRelayState = bool(jsonCommand["SET_CONVEYOR_MODE"]);
      ledcWrite(relayPin, uint8_t(ConveyorRelayState * ConveyorDutyCycle * 255));
      gpio_set_level(LEDpin, ConveyorRelayState);
    }

    if (jsonCommand.containsKey("ESC1") && RCmode == false)
    {
      if (jsonCommand["ESC1"] >= MIN_ESC_VALUE && jsonCommand["ESC1"] <= MAX_ESC_VALUE)
      {
        if (ESCvalues[0] != int(jsonCommand["ESC1"]))
        {
          ESCvalues[0] = int(jsonCommand["ESC1"]);
          ESC[0].writeMicroseconds(ESCvalues[0]);
        }
      }
      else
      {
        Serial.print("ESC1_VALUE_ERROR\r\n");
      }
    }

    if (jsonCommand.containsKey("ESC2") && RCmode == false)
    {
      if (jsonCommand["ESC2"] >= MIN_ESC_VALUE && jsonCommand["ESC2"] <= MAX_ESC_VALUE)
      {
        if (ESCvalues[1] != int(jsonCommand["ESC2"]))
        {
          ESCvalues[1] = int(jsonCommand["ESC2"]);
          ESC[1].writeMicroseconds(ESCvalues[1]);
        }
      }
      else
      {
        Serial.print("ESC2_VALUE_ERROR\r\n");
      }
    }

    if (jsonCommand.containsKey("ESC3") && RCmode == false)
    {
      if (jsonCommand["ESC3"] >= MIN_ESC_VALUE && jsonCommand["ESC3"] <= MAX_ESC_VALUE)
      {
        if (ESCvalues[2] != int(jsonCommand["ESC3"]))
        {
          ESCvalues[2] = int(jsonCommand["ESC3"]);
          ESC[2].writeMicroseconds(ESCvalues[2]);
        }
      }
      else
      {
        Serial.print("ESC3_VALUE_ERROR\r\n");
      }
    }

    if (jsonCommand.containsKey("ESC4") && RCmode == false)
    {
      if (jsonCommand["ESC4"] >= MIN_ESC_VALUE && jsonCommand["ESC4"] <= MAX_ESC_VALUE)
      {
        if (ESCvalues[3] != int(jsonCommand["ESC4"]))
        {
          ESCvalues[3] = int(jsonCommand["ESC4"]);
          ESC[3].writeMicroseconds(ESCvalues[3]);
        }
      }
      else
      {
        Serial.print("ESC4_VALUE_ERROR\r\n");
      }
    }

    if (RCmode)
    {
      if (ESCvalues[0] != constrain(((highTime2 + (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE))
      {
        ESCvalues[0] = constrain(((highTime2 + (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE);
        ESC[0].writeMicroseconds(ESCvalues[0]);
      }
      if (ESCvalues[1] != constrain(((highTime2 - (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE))
      {
        ESCvalues[1] = constrain(((highTime2 - (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE);
        ESC[1].writeMicroseconds(ESCvalues[1]);
      }
      if (ESCvalues[2] != constrain(((highTime2 + (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE))
      {
        ESCvalues[2] = constrain(((highTime2 + (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE);
        ESC[2].writeMicroseconds(ESCvalues[2]);
      }
      if (ESCvalues[3] != constrain(((highTime2 - (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE))
      {
        ESCvalues[3] = constrain(((highTime2 - (highTime1 - 1500) + 10) / 20) * 20, MIN_ESC_VALUE, MAX_ESC_VALUE);
        ESC[3].writeMicroseconds(ESCvalues[3]);
      }
      if (ConveyorRelayState != (highTime3 > 1500))
      {
        ConveyorRelayState = (highTime3 > 1500);
        ledcWrite(relayPin, uint8_t(ConveyorRelayState * ConveyorDutyCycle * 255));
        gpio_set_level(LEDpin, ConveyorRelayState);
      }
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
              body {
                  text-align: center;
                  font-family: Arial, sans-serif;
              }

              #container {
                  position: relative;
                  width: 300px;
                  height: 300px;
                  border-radius: 50%;
                  border: 2px solid black;
                  margin: auto;
              }

              #ball {
                  position: absolute;
                  width: 20px;
                  height: 20px;
                  background: red;
                  border-radius: 50%;
              }

              #boatContainer {
                  width: 300px;
                  height: 300px;
                  margin: auto;
                  position: relative;
                  perspective: 800px;
                  border: 1px solid #ccc;
              }

              #boat {
                  position: absolute;
                  top: 50%;
                  left: 50%;
                  width: 150px;
                  height: 150px;
                  transform: translate(-50%, -50%);
                  transform-origin: center;
              }
          </style>
          <script>
              var xhr = new XMLHttpRequest();
              function updateRC() {
                  xhr.open('GET', '/rc', true);
                  xhr.onreadystatechange = function () {
                      if (xhr.readyState == 4 && xhr.status == 200) {
                          var data = JSON.parse(xhr.responseText);
                          document.getElementById('rc1').value = data.ch1;
                          document.getElementById('rc2').value = data.ch2;
                          document.getElementById('rc1Val').innerHTML = data.ch1;
                          document.getElementById('rc2Val').innerHTML = data.ch2;
                          document.getElementById('rcMode').checked = data.rcMode;
                          document.getElementById('conveyorState').checked = data.conveyorState;
                          var xInput = (data.ch1 - 1500) / 500;
                          var yInput = -1 * (data.ch2 - 1500) / 500;
                          var magnitude = Math.sqrt(xInput * xInput + yInput * yInput);
                          if (magnitude > 1) { xInput /= magnitude; yInput /= magnitude; }
                          var radius = 140;
                          var x = xInput * radius + 150 - 10;
                          var y = yInput * radius + 150 - 10;
                          document.getElementById('ball').style.left = x + 'px';
                          document.getElementById('ball').style.top = y + 'px';
                      }
                  };
                  xhr.send();
              }
              function updateESC() {
                  var xhr_esc = new XMLHttpRequest();
                  xhr_esc.open('GET', '/esc', true);
                  xhr_esc.onreadystatechange = function () {
                      if (xhr_esc.readyState == 4 && xhr_esc.status == 200) {
                          var esc = JSON.parse(xhr_esc.responseText);
                          document.getElementById('esc1').value = esc.esc1;
                          document.getElementById('esc2').value = esc.esc2;
                          document.getElementById('esc3').value = esc.esc3;
                          document.getElementById('esc4').value = esc.esc4;

                          document.getElementById('esc1Val').innerHTML = esc.esc1;
                          document.getElementById('esc2Val').innerHTML = esc.esc2;
                          document.getElementById('esc3Val').innerHTML = esc.esc3;
                          document.getElementById('esc4Val').innerHTML = esc.esc4;
                      }
                  };
                  xhr_esc.send();
              }

              function updateIMU() {
                  var xhr_imu = new XMLHttpRequest();
                  xhr_imu.open('GET', '/imu', true);
                  xhr_imu.onreadystatechange = function () {
                      if (xhr_imu.readyState == 4 && xhr_imu.status == 200) {
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
                          if (imu.pitch < 0) {
                              Zrot = 180;
                          } else {
                              Zrot = 0;
                          }
                          var boat = document.getElementById('boat');
                          if (boat) {
                              boat.style.transform = 'translate(-50%, -50%) ' + ' rotateX(' + imu.pitch + 'deg) rotateY(' + (-imu.roll) + 'deg) rotateZ(' + Zrot + 'deg)';
                          }
                      }
                  };
                  xhr_imu.send();
              }
              setInterval(updateRC, 150);
              setInterval(updateIMU, 150);
              setInterval(updateESC, 150);
          </script>
      </head>

      <body>
          <h1>Triton Portal</h1>
          <h2>Remote Control</h2>
          <label>
              RC Mode:
              <input type="checkbox" id="rcMode" disabled />
          </label><br><br>
          <label>
              Conveyor:
              <input type="checkbox" id="conveyorState" disabled />
          </label><br><br>
          RC Channel 1: <input type='range' id='rc1' min='1000' max='2000' value='1500' disabled /> <span
              id='rc1Val'>1500</span><br><br>
          RC Channel 2: <input type='range' id='rc2' min='1000' max='2000' value='1500' disabled /> <span
              id='rc2Val'>1500</span><br><br>
          <div id='container'>
              <div id='ball'></div>
          </div>
          <h2>ESC Values</h2>
          ESC1: <input type='range' id='esc1' min='1000' max='2000' value='1500' disabled /> <span
              id='esc1Val'>1500</span><br><br>
          ESC2: <input type='range' id='esc2' min='1000' max='2000' value='1500' disabled /> <span
              id='esc2Val'>1500</span><br><br>
          ESC3: <input type='range' id='esc3' min='1000' max='2000' value='1500' disabled /> <span
              id='esc3Val'>1500</span><br><br>
          ESC4: <input type='range' id='esc4' min='1000' max='2000' value='1500' disabled /> <span
              id='esc4Val'>1500</span><br><br>
          <h2>IMU Data</h2>
          <p>Pitch: <span id='pitch'>0</span>&#176</p>
          <p>Roll: <span id='roll'>0</span>&#176</p>
          <p>Heading: <span id='heading'>0</span>&#176</p>
          <p>Gyro: X: <span id='gyroX'>0</span> &#176/s, Y: <span id='gyroY'>0</span> &#176/s, Z: <span id='gyroZ'>0</span>
              &#176/s</p>
          <p>Accel: X: <span id='accelX'>0</span> m/s^2, Y: <span id='accelY'>0</span> m/s^2, Z: <span id='accelZ'>0</span>
              m/s^2</p>
          <p>Mag: X: <span id='magX'>0</span> uT, Y: <span id='magY'>0</span> uT, Z: <span id='magZ'>0</span> uT</p>
          <div id='boatContainer'>
              <svg id='boat' viewBox='0 0 100 100'>
                  <polygon points='50,0 90,80 50,70 10,80' fill='blue' stroke='black' stroke-width='2' />
              </svg>
          </div>
      </body>

      </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleRC()
{
  char jsonResponse[128] = "";
  snprintf(jsonResponse, sizeof(jsonResponse),
           "{\"ch1\":%d,\"ch2\":%d,\"rcMode\":%s, \"conveyorState\":%s}",
           constrain(highTime1, 1000, 2000),
           constrain(highTime2, 1000, 2000),
           RCmode ? "true" : "false",
           ConveyorRelayState ? "true" : "false");

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

void handleESC()
{
  char jsonResponse[128];
  snprintf(jsonResponse, sizeof(jsonResponse),
           "{\"esc1\":%d,\"esc2\":%d,\"esc3\":%d,\"esc4\":%d}",
           ESCvalues[0], ESCvalues[1], ESCvalues[2], ESCvalues[3]);
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
void GetIMUinJSON(JsonDocument &doc)
{
  bno055_read_euler_hrp(&hrpEulerData); // divide by 16 for °
  bno055_read_gyro_xyz(&gyroData);      // divide by 16 for °/s
  bno055_read_accel_xyz(&accelData);    // divide by 100 for m/s^2
  bno055_read_mag_xyz(&magData);        // divide by 16 for uT

  JsonObject IMU = doc["IMU"].to<JsonObject>();
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

  doc.shrinkToFit();
}