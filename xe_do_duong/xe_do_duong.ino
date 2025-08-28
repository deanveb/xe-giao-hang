#include <esp_task_wdt.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <WebServer.h>
#include "index.h"
#include <I2Cdev.h>
#include <math.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h>  // not necessary if using MotionApps include file
#include "Wire.h"

void move(int action, bool disableEditSpeed = true, int modifiedSpeed = 255);

// debugging
const int buttonPin = 9;
bool isLeft = false;

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
#define PI 3.1416
#define MAX_PWM 255
#define MIN_PWM 0
#define WHEEL_RADIUS 6.5
#define GEAR_REVOLUTION 975.0
#define MAX_MPU_COUNT 5

long duration;
// bool end0 = false; // Rotate
// bool end = false; // forward
// bool end1 = false; // align

int currentSpeed = 255;
int count = 0;

float distanceCmL;
float distanceCmR;
float distanceInch;

float filteredValL = 0.0;
float filteredValR = 0.0;

const float alpha = 0.5678;
const float alpha1 = 0.5678;

int MPUOffset = 0;

double Setpoint, Input, Output;
double Kp = 50, Ki = 2, Kd = 30;
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO  storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

enum control {
  Left,
  Right,
  Up,
  Down,
  Stop,
};

// Số lượng pin cần sử dụng
const int n = 4;

// in1.3, in2.4, in1.3(2), in2.4(2)
// const int pins[n] = {19, 20, 21, 47, 48, 45, 37, 36, 35, 41, 39, 38};
// const int pins[n] = {19, 20, 21, 47, 48, 45, 41, 37, 36, 35, 39, 38};
const int pins[n] = { 21, 19, 33, 32 };


const int truthTable[][n] = {
  { LOW, HIGH, HIGH, LOW },  // Left
  { HIGH, LOW, LOW, HIGH },  // Right
  { HIGH, LOW, HIGH, LOW },  //Up
  { LOW, HIGH, LOW, HIGH },  // Down
  { LOW, LOW, LOW, LOW }     // Stop
};

class encoder {
public:
  int pinA;
  int pinB;
  volatile int counter = 0;
  int direction;
  int prev_counter;
  int error;
  encoder(int pinA, int pinB, int error) {
    this->pinA = pinA;
    this->pinB = pinB;
    this->error = error;
  }
};

encoder rightEncoder(25, 26, 1);

void IRAM_ATTR updateRightEncoder() {
  bool A = digitalRead(rightEncoder.pinA);
  bool B = digitalRead(rightEncoder.pinB);
  if (A == B) rightEncoder.counter++;
  else rightEncoder.counter--;
}

const char* ssid = "Dinv";
const char* password = "chiandeptrai";

WebServer server(80);

// THE DEFAULT TIMER IS SET TO 10 SECONDS FOR TESTING PURPOSES
// For a final application, check the API call limits per hour/minute to avoid getting blocked/banned
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 10 seconds (10000)
unsigned long timerDelay = 10000;

String jsonBuffer;
const char baseUrl[255] = "https://gps-car-api.vercel.app/api/navigation";
JSONVar deliveryInstruction;
JSONVar returnInstruction;
int repeatedTimes = 0;
int progress = 0;

bool isBeingDelivered = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(18, 17);
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // devStatus = 1;

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(26.00000);
  mpu.setYGyroOffset(-31.00000);
  mpu.setZGyroOffset(-2.00000);
  mpu.setZAccelOffset(1224.00000);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.println(F(")..."));
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pinMode(rightEncoder.pinA, INPUT_PULLUP);
  pinMode(rightEncoder.pinB, INPUT_PULLUP);
  for (int i = 0; i < n; i++) {
    pinMode(pins[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(rightEncoder.pinA), updateRightEncoder, CHANGE);
  waitMPU();

  // Serial.println(html);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", setupHTML);
  // server.on("/progress", []() {
  //   server.send(200, "text/plain", String(progress));
  // });
  // server.on("/inProgress", []() {
  //   server.send(200, "text/plain", String(isBeingDelivered));
  // });

  server.begin();

  // Serial.println("Timer set to 10 seconds (timerDelay variable), it will take 10 seconds before publishing the first reading.");
}

void loop() {
  server.handleClient();
  // move(Up);

  if (isBeingDelivered) {
    deliverPackage();
    if (repeatedTimes != 0) {
      repeatedTimes--;
    } else {
      isBeingDelivered = false;
    }
  }
}

void deliverPackage() {
  int distance = 0;
  for (int i = 0; i < deliveryInstruction.length(); i++) {
    control currentMode = Up;
    // const char* charPointer = (JSON.stringify(deliveryInstruction[i]["direction"])).c_str();
    String direction = JSON.stringify(deliveryInstruction[i]["direction"]); 
    // Serial.println(direction == thang);
    
    if (direction == String("\"thang\"")) {
      currentMode = Up;
    } else if (direction == String("\"trai\"")) {
      currentMode = Left;
    } else if (direction == String("\"phai\"")) {
      currentMode = Right;
    }
    distance = (int) deliveryInstruction[i]["distance"];
    Serial.print(currentMode);
    Serial.print(" ");
    Serial.println(JSON.stringify(deliveryInstruction[i]["direction"]));
    forward(distance / 25);
    resetEncoder();
    // move(Stop);
    delay(50);
    rotate(currentMode);
    resetEncoder();
    delay(50);
    // move(Stop);
    // progress = floor(i / deliveryInstruction.length() * 100);
  }
  forward(distance / 25);
  resetEncoder();
  delay(50);
  forward(distance / 25);
  delay(50);
  // To rotate Down
  rotate(Left);
  delay(50);
  rotate(Left);
  // progress = 0;
  // for (int i = 0; i < returnInstruction.length(); i++) {
  //   String direction = JSON.stringify(returnInstruction[i]["direction"]); 
  //   control currentMode;
  //   if (direction == String("\"thang\"")) {
  //     currentMode = Up;
  //   } else if (direction == String("\"trai\"")) {
  //     currentMode = Left;
  //   } else if (direction == String("\"phai\"")) {
  //     currentMode = Right;
  //   }

  //   const int distance = (int)returnInstruction[i]["distance"];
  //   resetEncoder();
  //   forward(distance / 25);
  //   delay(50);
  //   resetEncoder();
  //   rotate(currentMode);
  //   delay(50);
  //   progress = floor(i / returnInstruction.length() * 100);
  // }
  // forward(distance / 25);
  // resetEncoder();
  // delay(50);
  // forward(distance / 25);
  // delay(50);
  // // To rotate Down
  // rotate(Left);
  // delay(50);
  // rotate(Left);
}

String httpGETRequest(const char* serverName) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return "";
  }
  HTTPClient http;

  // Your Domain name with URL path or IP address with path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "{}";

  if (httpResponseCode > 0) {
    // Serial.print("HTTP Response code: ");
    // Serial.println(httpResponseCode);
    payload = http.getString();
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void setupHTML() {
  // esp_task_wdt_deinit();
  server.send(200, "text/html", html);
  // Get GET parameters
if (server.hasArg("to") && !isBeingDelivered) {
Serial.println(server.arg("to"));
    Serial.println(server.arg("from"));
    JSONVar toRequest = JSON.parse(server.arg("to"));
    JSONVar fromRequest = JSON.parse(server.arg("from"));
    // JSON.typeof(jsonVar) can be used to get the type of the var
    if (JSON.typeof(toRequest) == "undefined") {
      Serial.println("Parsing input failed!");
      return;
    }
    if (JSON.typeof(fromRequest) == "undefined") {
      Serial.println("Parsing input failed!");
      return;
    }
    // check if their the same
    char urlBufferDeliver[255] = "";

    String urlStringDelivery = String(baseUrl);
    urlStringDelivery += "/?start0=" + JSON.stringify(fromRequest["pos0"]);
    urlStringDelivery += "&start1=" + JSON.stringify(fromRequest["pos1"]);
    urlStringDelivery += "&end0=" + JSON.stringify(toRequest["pos0"]);
    urlStringDelivery += "&end1=" + JSON.stringify(toRequest["pos1"]);

    char urlBufferReturn[255] = "";

    String urlStringReturn = String(baseUrl);
    urlStringReturn += "/?start0=" + JSON.stringify(toRequest["pos0"]);
    urlStringReturn += "&start1=" + JSON.stringify(toRequest["pos1"]);
    urlStringReturn += "&end0=" + JSON.stringify(fromRequest["pos0"]);
    urlStringReturn += "&end1=" + JSON.stringify(fromRequest["pos1"]);

    Serial.println("Final URL:");
    Serial.println(urlStringDelivery);
    Serial.println(urlStringReturn);

    urlStringDelivery.toCharArray(urlBufferDeliver, sizeof(urlBufferDeliver));
    urlStringReturn.toCharArray(urlBufferReturn, sizeof(urlBufferReturn));

    deliveryInstruction = JSON.parse(httpGETRequest(urlBufferDeliver));
    returnInstruction = JSON.parse(httpGETRequest(urlBufferReturn));
    Serial.println(JSON.stringify(deliveryInstruction));
    Serial.println(JSON.stringify(returnInstruction));

    if (JSON.typeof(deliveryInstruction) != "array") {
      Serial.println("Failed to parse delivery instruction or invalid format.");
      isBeingDelivered = false;
      return;
    }
    if (JSON.typeof(returnInstruction) != "array") {
      Serial.println("Failed to parse return instruction or invalid format.");
      isBeingDelivered = false;
      return;
    }
    isBeingDelivered = true;
    // Serial.println(instruction[0]["distance"]);
  }
  // if (server.hasArg("time")) {
  //   try {
  //     repeatedTimes = server.arg("time").toInt();
  //   } catch (const char* msg) {
  //     Serial.println(msg);
  //     repeatedTimes = 0;
  //   }
  // }
}

void resetEncoder() {
  rightEncoder.counter = 0;
}

void forward(int step) {
  const int size = 25 / 2;  // (cm)
  const int error = 0;
  const int speed = 255;

  float traveledRight = abs((rightEncoder.counter / GEAR_REVOLUTION) * WHEEL_RADIUS * PI);

  int slowDown = 0;
    while (traveledRight <= size * step - error) {
      traveledRight = abs((rightEncoder.counter / GEAR_REVOLUTION) * WHEEL_RADIUS * PI);
      Serial.print("traveledRight:");
      Serial.println(traveledRight);
// currentSpeed = 255;
      // alignCar();
      move(Up, false, 255);
  }

  move(Stop);
  resetEncoder();
}


float angle = 0;

void rotate(control direction) {
  if (!dmpReady) {
    Serial.println("dmp just got disconnected");
    return;
  }

  // read a packet from FIFO

  const float lowerNum = 89.0;
  const float upperNum = 90.0;
  bool init = false;
  float originAngle = 0;
  int speed = 0;
  int speed1 = 0;
  
  if (direction == Up || direction == Down)
  {
    return;
  }

  while (true) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      angle = (ypr[0] * 180 / M_PI) - (MPUOffset);
      angle = angle < 0 ? angle + 360 : angle;
      if (!init) {
        originAngle = angle;
        init = true;
      }
      Serial.print("ypr\t");
      Serial.print(angle);
      Serial.print(" ");
      Serial.print(direction);
      speed = 0;
      // Ignore Up direction
      if (direction == Right) {
        const float lower = (originAngle + lowerNum) >= 360 ? originAngle + lowerNum - 360 : originAngle + lowerNum;
        const float upper = (originAngle + upperNum) >= 360 ? originAngle + upperNum - 360 : originAngle + upperNum;
        speed = map(angle, originAngle, upper, 0, 155);
        Serial.print(constrain(255 - speed, MIN_PWM, MAX_PWM));
        Serial.print(" ");
        Serial.println(lower);

        if (angle >= lower && angle < upper) {
          move(Stop);
          mpu.resetFIFO();
          delay(10);
          break;
        }

        if (angle <= lower) {
          move(Left, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
        } else if (angle >= upper) {
          move(Right, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
        }
      } else if (direction == Left) {
        const float upper = (originAngle - lowerNum) < 0 ? originAngle - lowerNum + 360 : originAngle - lowerNum;
        const float lower = (originAngle - upperNum) < 0 ? originAngle - upperNum + 360 : originAngle - upperNum;
        speed = map(angle, originAngle, upper, 0, 200);
        Serial.println(constrain(255 - speed, MIN_PWM, MAX_PWM));
        Serial.print(" ");
        Serial.println(lower);
        if (angle >= lower && angle < upper) {
          move(Stop);
          mpu.resetFIFO();
          delay(10);
          break;
        }
        if(lower-360<0){
          if (angle <= upper) {
            move(Left, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
          } else if (angle >= upper) {
            move(Right, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
          }
        }else{
          if (angle <= lower) {
            move(Right, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
          } else if (angle >= upper) {
            move(Left, false, constrain(255 - speed, MIN_PWM, MAX_PWM));
          }
        }
      }
    }
  }
}

void move(int action, bool disableEditSpeed, int modifiedSpeed) {
  const int defaultSpeed = 255;
int speedToUse = disableEditSpeed ? defaultSpeed : modifiedSpeed;

  // First turn all pins LOW to prevent any overlap
  for (int i = 0; i < n; i++) {
    analogWrite(pins[i], 0);
  }

  // Then activate only the needed pins
  for (int i = 0; i < n; i++) {
    if (truthTable[action][i] == HIGH) {
      analogWrite(pins[i], speedToUse);
    }
  }
}

float getDuration(int trigPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  const float duration = pulseIn(echoPin, HIGH, 30000);

  return duration;
}

void waitMPU() {
  float prevAngle = 0;
  float currentAngle = 0;
  int count = 0;

  do {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      prevAngle = currentAngle;
      currentAngle = ypr[0] * 180 / M_PI;
      if (prevAngle == currentAngle) {
        count++;
      }
}
  } while (count != MAX_MPU_COUNT);
  MPUOffset = currentAngle;
  Serial.println(currentAngle);
  Serial.println("Finished");
}