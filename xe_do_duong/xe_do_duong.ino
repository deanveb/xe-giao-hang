#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <WebServer.h>
#include "index.h"
#include <I2Cdev.h>
#include <math.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU6050.h> // not necessary if using MotionApps include file
#include "Wire.h"

void move(int action, bool disableEditSpeed = false);

// debugging
const int buttonPin = 9;
bool isLeft = false;

const int trigPinL = 9;
const int echoPinL = 10;
const int trigPinR = 11;
const int echoPinR = 12;
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
double Kp=50, Ki=2, Kd=30;
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

enum control {
    Left,
    Right,
    Down,
    Up,
    Stop,
};

// Số lượng pin cần sử dụng
const int n = 12;

// in1, in2, in3, in4, enA, enB, in11, in21, in31, in41, enA1, enB1
const int pins[n] = {19, 20, 21, 47, 48, 45, 37, 36, 35, 41, 39, 38};


const int truthTable[][n] = {
  {LOW, HIGH, LOW, HIGH, 255, 255, HIGH, LOW, HIGH, LOW, 255, 255}, // Left
  {HIGH, LOW, HIGH, LOW, 255, 255, LOW, HIGH, LOW, HIGH, 255, 255}, // Right
  {HIGH, LOW, HIGH, LOW, 255, 255, HIGH, LOW, HIGH, LOW, 255, 255}, //Up
  {LOW, HIGH, LOW, HIGH, 255, 255, LOW, HIGH, LOW, HIGH, 255, 255}, // Down
  {LOW, LOW, LOW, LOW, 0, 0, LOW, LOW, LOW, LOW, 0, 0} // Stop
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

encoder rightEncoder(17, 18, 1);
encoder leftEncoder(15, 16, 2);

void IRAM_ATTR updateRightEncoder() {
  bool A = digitalRead(rightEncoder.pinA);
  bool B = digitalRead(rightEncoder.pinB);
  if (A == B) rightEncoder.counter++;
  else        rightEncoder.counter--;
}

void IRAM_ATTR updateLeftEncoder() {
  bool A = digitalRead(leftEncoder.pinA);
  bool B = digitalRead(leftEncoder.pinB);
  if (A == B) leftEncoder.counter++;
  else        leftEncoder.counter--;
}

const char* ssid = "PIF_Client";
const char* password = "88888888";

WebServer server(80);

// THE DEFAULT TIMER IS SET TO 10 SECONDS FOR TESTING PURPOSES
// For a final application, check the API call limits per hour/minute to avoid getting blocked/banned
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 10 seconds (10000)
unsigned long timerDelay = 10000;
int start0, start1, end0, end1;

String jsonBuffer;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(13, 14);
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(26.00000);
  mpu.setYGyroOffset(-31.00000);
  mpu.setZGyroOffset(-2.00000);
  mpu.setZAccelOffset(1224.00000); // 1688 factory default for my test chip

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

  pinMode(trigPinL, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinR, INPUT);
  Setpoint = 0;
  // myPID.SetMode(AUTOMATIC);

  pinMode(rightEncoder.pinA, INPUT_PULLUP);
  pinMode(rightEncoder.pinB, INPUT_PULLUP);
  pinMode(leftEncoder.pinA, INPUT_PULLUP);
  pinMode(leftEncoder.pinB, INPUT_PULLUP);
  for (int i = 0; i < n; i++)
  {
    pinMode(pins[i], OUTPUT);
  }

  attachInterrupt(digitalPinToInterrupt(rightEncoder.pinA), updateRightEncoder, CHANGE);
  waitMPU();

  // Serial.println(html);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
 
  server.on("/", setupHTML);
  server.begin();
  
  Serial.println("Timer set to 10 seconds (timerDelay variable), it will take 10 seconds before publishing the first reading.");
}

void loop() {
  server.handleClient();
  // Send an HTTP GET request
  if ((millis() - lastTime) > timerDelay) {
    // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      String serverPath = "https://gps-car-api.vercel.app/api/navigation?start0=6&start1=1&end0=6&end1=6";
      
      jsonBuffer = httpGETRequest(serverPath.c_str());
      Serial.println(jsonBuffer);
      JSONVar myObject = JSON.parse(jsonBuffer);
      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
    
      // Serial.print("JSON object = ");
      // Serial.println(myObject["message"]);
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;
  
  // Your Domain name with URL path or IP address with path
  http.begin(serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}";
  
  if (httpResponseCode>0) {
    // Serial.print("HTTP Response code: ");
    // Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void setupHTML()
{
  server.send(200, "text/html", html);
  // Get GET parameters
  if (server.hasArg("to"))
  {
    Serial.println(server.arg("to"));
  }
}

void resetEncoder()
{
  rightEncoder.counter = 0;
}

void forward(int step)
{
  const int size = 25; // (cm)
  const int error = 0;
  const int speed = 255;

  float traveledRight = abs((rightEncoder.counter/GEAR_REVOLUTION)*WHEEL_RADIUS*PI);

  int slowDown = 0;
  while(traveledRight != size*step && slowDown <= 255)
  {
    while (traveledRight <= size * step - error)
    {
      traveledRight = abs((rightEncoder.counter/GEAR_REVOLUTION)*WHEEL_RADIUS*PI);
      // Serial.print("traveledRight:");
      // Serial.println(traveledRight);
      analogWrite(pins[4], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[5], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[10], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[11], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      // currentSpeed = 255;
      // alignCar();
      move(Up, true);
    }
    slowDown += 50;
    while (traveledRight <= size * step - error)
    {
      traveledRight = abs((rightEncoder.counter/GEAR_REVOLUTION)*WHEEL_RADIUS*PI);
      analogWrite(pins[4], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[5], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[10], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      analogWrite(pins[11], constrain(speed - slowDown, MIN_PWM, MAX_PWM));
      // currentSpeed = 255;
      // alignCar();
      move(Down, true);
    }
  }

  move(Stop);
  resetEncoder();
}

void rotate(control direction)
{
  if (!dmpReady) return;
    // read a packet from FIFO

  float angle = 0;
  int speed = 255;
  while(true)
  {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      float angle = (ypr[0] * 180/M_PI) - (MPUOffset);
      Serial.print("ypr\t");
      Serial.print(angle);
      Serial.print(" ");
      speed = map(angle, 0, 88, 0, 45);
      // speed = 0;
      Serial.println(255 - speed);
      if (direction == Right)
      {
        if (angle >=88.0 && angle < 89.0)
        {
          move(Stop);
          break;
        }

        if (angle <= 90.0) 
        {
          analogWrite(pins[4], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[5], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[10], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[11], constrain(255 - speed, MIN_PWM, MAX_PWM));
          move(Right, false);
        }
        else if (angle >= 90.0)
        {
          analogWrite(pins[4], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[5], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[10], constrain(255 - speed, MIN_PWM, MAX_PWM));
          analogWrite(pins[11], constrain(255 - speed, MIN_PWM, MAX_PWM));
          move(Left, false);
        }
      } else if (direction == Left)
      {
        if (angle <= -89.8) move(Left);
        else 
        {
          move(Stop);
        }
      }
      // if (angle == 90.0)
      // {
      //   move(Stop);
      //   end0 = true;
      // }
    }
  }
  // MPUOffset = angle;
}

void move(int action, bool disableEditSpeed) {
    for (int i = 0; i < n; i++) {
      if (i != 4 || i != 5 || i != 10 || i != 11) {
        digitalWrite(pins[i], truthTable[action][i]);
      }
      else {
        if (disableEditSpeed) continue;
        analogWrite(pins[i], truthTable[action][i]);
      }
    }
  }

float getDuration(int trigPin, int echoPin)
{
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

int alignCar()
{
  const float durationL = getDuration(trigPinL, echoPinL);
  const float durationR = getDuration(trigPinR, echoPinR);
  const float acceptableError = 1.0;

  if(fabs(durationL-0.3) < 0){
    distanceCmL=-1;
  }else{
     distanceCmL = durationL * SOUND_SPEED/2;
  }
  if(fabs(durationR-0.3) < 0){
    distanceCmR=-1;
  }else{
     distanceCmR = durationR * SOUND_SPEED/2;
  }
  
  // Convert to inches
  // distanceInch = distanceCm * CM_TO_INCH;
  // tuning parameter between 0 and 1
  filteredValL = (alpha * distanceCmL + (1 - alpha) * filteredValL);
  filteredValR = (alpha1 * distanceCmR + (1 - alpha1) * filteredValR);

  // Prints the distance in the Serial Monitor
  Serial.print(filteredValL);
  Serial.print(" ");
  Serial.print(filteredValR);
  Serial.print(" ");
  Serial.print(Output);
  Serial.print(" ");


  Input = filteredValL - filteredValR;
  const int speed = Output;
  Serial.println(constrain(currentSpeed - speed, MIN_PWM, MAX_PWM));
  
  // if (Input > 0)
  // {
  //   analogWrite(pins[4], constrain(currentSpeed - speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[5], constrain(currentSpeed - speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[10], constrain(currentSpeed + speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[11], constrain(currentSpeed + speed, MIN_PWM, MAX_PWM));
  // } else if (Input < 0)
  // {
  //   analogWrite(pins[4], constrain(currentSpeed + speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[5], constrain(currentSpeed + speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[10], constrain(currentSpeed - speed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[11], constrain(currentSpeed - speed, MIN_PWM, MAX_PWM));
  // } else
  // {
  //   analogWrite(pins[4], constrain(currentSpeed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[5], constrain(currentSpeed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[10], constrain(currentSpeed, MIN_PWM, MAX_PWM));
  //   analogWrite(pins[11], constrain(currentSpeed, MIN_PWM, MAX_PWM));
  // }


  // Determine direction based on error
  if (abs(Input) <= 1) {  // Deadband for small errors
    move(Stop, true);
  } 
  else if (Input > 0) {
    move(Right, true);
  } 
  else {
    move(Left, true);
  }

  resetEncoder();

  return abs(Input);
}

void waitMPU()
{
  float prevAngle = 0;
  float currentAngle = 0;
  int count = 0;

  do
  {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      prevAngle = currentAngle;
      currentAngle = ypr[0] * 180/M_PI;
      if (prevAngle == currentAngle)
      {
        count++;
      }
    }
  } while(count != MAX_MPU_COUNT);
  MPUOffset = currentAngle;
  Serial.println(currentAngle);
  Serial.println("Finished");
}