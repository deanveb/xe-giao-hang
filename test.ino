#include <PID_v1.h>

double Setpoint, Input_L, Input_R, Setpoint_r;
double Output_L, Output_R, Output_hR;
double Kp = 16, Ki = 0.005, Kd = 1;        //left motr pid
double Kp_r = 18, Ki_r = 0.005, Kd_r = 1;  //left motr pid

PID myPIDl(&Input_L, &Output_L, &Setpoint, Kp, Ki, Kd, DIRECT); 
PID myPIDr(&Input_R, &Output_R, &Setpoint_r, Kp_r, Ki_r, Kd_r, DIRECT);

const int trigRPin = 23, echoRPin = 22;
const int trigLPin = 19, echoLPin = 21;

double distanceL;
double distanceR;
double durationL;
double durationR;

#define ENA 13
#define ENB 33
#define A1 27
#define A2 14
#define B1 26
#define B2 25

#define ir_l 17
#define ir_r 18
#define ir_sl 4
#define ir_sr 16
#define ir_mid 35

#define sw 2  //left algo right algo

unsigned long startTime = 0;
bool conditionMet = false;

int turn = 400, turns = 400, p_turn = 0, spped = 110;  // turn for forward delay after doing a turn //pturn 0 means right, 1 means left // spped is normal wroking spped only for forward

void setup() {
  Setpoint = 7;  //left motor pid
  Setpoint_r = 6.5;  //right motor pid

  myPIDl.SetMode(AUTOMATIC);
  myPIDl.SetTunings(Kp, Ki, Kd);
  myPIDl.SetOutputLimits(-100, 100);  //pid speed left

  myPIDr.SetMode(AUTOMATIC);
  myPIDr.SetTunings(Kp, Ki, Kd);
  myPIDr.SetOutputLimits(-100, 100);  //pid speed right motor

  pinMode(trigLPin, OUTPUT);
  pinMode(echoLPin, INPUT);
  pinMode(trigRPin, OUTPUT);
  pinMode(echoRPin, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);

  pinMode(ir_r, INPUT);
  pinMode(ir_l, INPUT);
  pinMode(ir_sr, INPUT);
  pinMode(ir_sl, INPUT);
  pinMode(ir_mid, INPUT);

  pinMode(sw, INPUT);

  Serial.begin(115200);
}

void loop() {
  read();

  Input_L = distanceL;  //input in pid means distance ,, output means pid for motor
  Input_R = distanceR;
  myPIDl.Compute();
  myPIDr.Compute();
  // below two if , are used to remove the  minus sighn
  if (Output_L >= 0) {
    Output_L = Output_L;
  } else if (Output_L < 0) {
    Output_L = -Output_L;
  }

  if (Output_R >= 0) {
    Output_R = Output_R;
  } else if (Output_R < 0) {
    Output_R = -Output_R;
  }

  Serial.print("OUTPUT Left");
  Serial.print(Output_L);
  Serial.print("\t");
  Serial.print("OUTPUT right");
  Serial.println(Output_R);

  positioning();

  if (Output_L <= 16 && Output_R <= 16) {  //if acuracy is less than 16
    if (!conditionMet) {
      startTime = millis();  // here time start
      conditionMet = true;
    } else if (millis() - startTime >= 300) {  //if the acuray remains in 16 for 300millisec
      rotate();                                //rotate means right or left with respect to algorithm
    }
  } else {
    conditionMet = false;  //if the accuracy does not remains in 16, condition fails
  }

  short_cut();  //turn without pid
  m_move();     //midle ir sensor
}

void read() {
  digitalWrite(trigLPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigLPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigLPin, LOW);

  durationL = pulseIn(echoLPin, HIGH);
  distanceL = durationL * 0.034 / 2.0;

  digitalWrite(trigRPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigRPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigRPin, LOW);

  durationR = pulseIn(echoRPin, HIGH);
  distanceR = durationR * 0.034 / 2.0;

  Serial.print("Ultra L :");
  Serial.print(distanceL);
  Serial.print("\t");
  Serial.print("Ultra R :");
  Serial.println(distanceR);
  Serial.println("........................");

  delay(100);
}

// tells motor how to move, with respect to pid
void motor_move() {

  analogWrite(ENA, Output_L);  //here pid is used
  analogWrite(ENB, Output_R);
  //left motor
  if (Input_L >= Setpoint) {
    digitalWrite(A1, 1);
    digitalWrite(A2, 0);
  } else if (Input_L < Setpoint) {
    Output_L = -Output_L;
    digitalWrite(A1, 0);
    digitalWrite(A2, 1);
  }
  //right motor
  if (Input_R >= Setpoint_r) {
    digitalWrite(B1, 1);
    digitalWrite(B2, 0);
  } else if (Input_R < Setpoint_r) {
    Output_R = -Output_R;
    digitalWrite(B1, 0);
    digitalWrite(B2, 1);
  }
}

void forward() {
  analogWrite(ENA, spped);
  analogWrite(ENB, spped);

  Serial.print("Status:      ");
  Serial.println("forward");
  digitalWrite(A1, 1);
  digitalWrite(A2, 0);
  digitalWrite(B1, 1);
  digitalWrite(B2, 0);
}

void turn_right() {
  p_turn = 0;  //right
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);

  Serial.print("Status:      ");
  Serial.println("right");
  digitalWrite(A1, 1);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 1);

  delay(180);
}

void turn_left() {
  p_turn = 1;  //left
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);

  Serial.print("Status:      ");
  Serial.println("left");
  digitalWrite(A1, 0);
  digitalWrite(A2, 1);
  digitalWrite(B1, 1);
  digitalWrite(B2, 0);

  delay(170);
}

void stop() {
  Serial.print("Status:      ");
  Serial.println("stop");
  digitalWrite(A1, 0);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 0);

  delay(250);
}

void sright() {
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);

  Serial.print("Status:      ");
  Serial.println("sright");
  digitalWrite(A1, 1);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 1);
}

void sleft() {
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);

  Serial.print("Status:      ");
  Serial.println("sleft");
  digitalWrite(A1, 0);
  digitalWrite(A2, 1);
  digitalWrite(B1, 1);
  digitalWrite(B2, 0);
}

void uturn() {
  p_turn = 1;  //left
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  if (digitalRead(sw) == 0) {
    Serial.print("Status:      ");
    Serial.println("u");
    digitalWrite(A1, 1);
    digitalWrite(A2, 0);
    digitalWrite(B1, 0);
    digitalWrite(B2, 1);
    delay(415);
  }
  else {
    Serial.print("Status:      ");
    Serial.println("u");
    digitalWrite(A1, 0);
    digitalWrite(A2, 1);
    digitalWrite(B1, 1);
    digitalWrite(B2, 0);
    delay(400);
  }
}

void sstop() {
  Serial.print("Status:      ");
  Serial.println("stop");
  digitalWrite(A1, 0);
  digitalWrite(A2, 0);
  digitalWrite(B1, 0);
  digitalWrite(B2, 0);

  delay(250);
}

void rotate() {
  //ir read 1 means no wall, 0 means wall
  if (digitalRead(ir_r) == 0 && digitalRead(ir_l) == 1) {
    turn_left();
    sstop();
    forward();
    delay(turn);
    // stop();
    return;
  } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 0) {
    turn_right();
    sstop();
    forward();
    delay(turn);
    // stop();
    return;
  } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 1 && digitalRead(sw) == 1) {  //sw mean algo switching, 1 means right, 0 means left
    turn_right();
    sstop();
    forward();
    delay(turn);
    // stop();
    return;
  } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 1 && digitalRead(sw) == 0) {
    turn_left();
    sstop();
    forward();
    delay(turn);
    // stop();
    return;
  } else if (digitalRead(ir_r) == 0 && digitalRead(ir_l) == 0) {
    uturn();
    stop();
    delay(100);
    return;
  }
}

// pid + normal move + adj
void positioning() {
  if (distanceL <= 30 && distanceR <= 30) {  //if ultrasonic data less than 23 pid will work
    motor_move();
  } else {
    forward();  //normal

    //adjustment
    if (digitalRead(ir_sr) == 0 && digitalRead(ir_sl) == 1) {
      sleft();
      return;
    } else if (digitalRead(ir_sl) == 0 && digitalRead(ir_sr) == 1) {
      sright();
      return;
    }
  }
}

void short_cut() {
  // 0 meanms wall, 1 means no wall
  if (distanceL > 31 && distanceR > 31) {  //ultrasonic data greater than 24, no pid only normal
    forward();
    if (digitalRead(ir_sr) == 0 && digitalRead(ir_sl) == 1) {
      sleft();
      // return;
    } else if (digitalRead(ir_sl) == 0 && digitalRead(ir_sr) == 1) {
      sright();
      // return;
    } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 1 && digitalRead(sw) == 1) {
      delay(185);
      sstop();
      turn_right();
      sstop();
      forward();
      delay(turns);
    } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 1 && digitalRead(sw) == 0) {
      delay(185);
      sstop();
      turn_left();
      sstop();
      forward();
      delay(turns);
    } else if (digitalRead(ir_r) == 1 && digitalRead(ir_l) == 0 && digitalRead(sw) == 1) {
      delay(185);  ///change to 150 and try...........................................................................................
      sstop();
      turn_right();
      sstop();
      forward();
      delay(turns);
    } else if (digitalRead(ir_r) == 0 && digitalRead(ir_l) == 1 && digitalRead(sw) == 0) {
      delay(185
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      );  //.......................................................................................................................
      sstop();
      turn_left();
      sstop();
      forward();
      delay(turns);
    }
  }
}

void m_move() {
  //0 means wall
  if (digitalRead(ir_mid) == 0) {
    analogWrite(ENA, 80);
    analogWrite(ENB, 80);
    //pturn 1 = right, pturn 0 = left
    if (p_turn == 1) {
      digitalWrite(A1, 0);
      digitalWrite(A2, 1);
      digitalWrite(B1, 0);
      digitalWrite(B2, 1);
      delay(200);
      digitalWrite(A1, 0);
      digitalWrite(A2, 1);
      digitalWrite(B1, 0);
      digitalWrite(B2, 0);
      delay(200);
    } else if (p_turn == 0) {
      digitalWrite(A1, 0);
      digitalWrite(A2, 1);
      digitalWrite(B1, 0);
      digitalWrite(B2, 1);
      delay(200);
      digitalWrite(A1, 0);
      digitalWrite(A2, 0);
      digitalWrite(B1, 0);
      digitalWrite(B2, 1);
      delay(200);
    }
  }
}