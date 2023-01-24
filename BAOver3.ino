/* 배선표

  arduino - Serial Switch Relay
   (A3)17 - Signal
        
  arduino - blutooth
       10 - Tx
       11 - Rx

  arduino - encoder
        2 - A
        3 - B

  arduino - MPU6050
  (A4)SDA - SDA
  (A5)SCL - SCL

  arduino - motor
        6 - 1

  arduino - servo
        5 - Signal

  arduino - button
   (A0)14 - left
   (A1)15 - right

  arduino - LED
       12 - uphill LED
       13 - downhill LED
*/

#define uphill 12
#define downhill 13

#include <Servo.h>
Servo myServo;
#define left 14
#define right 15
#define servo 5
void setDirection();

// encoder 속도 측정 관련 변수 & 함수
#define encoderA 2
#define encoderB 3
double spd = 0;
volatile byte cnt = 0;
void encoderCount();
void encoderSpeed();
void printState();
// motor 구동
#define motor 6

#include <TaskScheduler.h>
Scheduler ts;
// 폴링(polling) 방식으로 encoder의 회전을 감지.
// 인터럽트(interrupt) 방식의 경우, 디바운싱을 방지하는 방법을 강구해야함.
// 카운터는 시작부터 활성화한다.
Task encoderC(0, TASK_FOREVER, encoderCount, &ts, true);
// encoder를 통해 회전 속도를 측정하고, 측정 결과를 출력한다.
// 속도는 카운트가 쌓인 약 500ms 이후부터 계산한다.
Task encoderS(100, TASK_FOREVER, encoderSpeed, &ts, false);
Task printS(100, TASK_FOREVER, printState, &ts, true);
// 버튼 입력에 따라 서보모터를 돌려, 방향을 정한다.
Task steering(10, TASK_FOREVER, setDirection, &ts, true);

#include <SoftwareSerial.h>
SoftwareSerial bluetooth(10, 11);

#define relay 17
void sendBluetooth(String s);

// 추후 개발 예정.
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

void setup() {
  // 디버깅용 시리얼 포트
  Serial.begin(9600);
  // 블루투스 시리얼 포트
  bluetooth.begin(9600);

  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);

  pinMode(motor, OUTPUT);
  digitalWrite(motor, HIGH);

  pinMode(left, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  myServo.attach(servo);
  myServo.write(90);

  pinMode(uphill, OUTPUT);
  pinMode(downhill, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(encoderA), encoderCount, FALLING);
  // 속도는 카운트가 쌓인 약 500ms 이후부터 계산한다.
  encoderS.enableDelayed(500);
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();
}

void loop() {
  ts.execute();
}

void setDirection() {
  static bool prevPush = false;
  int L = digitalRead(left);
  int R = digitalRead(right);
  if (L == R) {
    myServo.write(90);
  }
  else if (L) {
    myServo.write(165);
    if(prevPush){
      sendBluetooth("right");
      prevPush = false;
    }
  }
  else {
    myServo.write(15);
    if(!prevPush){
      sendBluetooth("left");
      prevPush = true;
    }
  }
}

void sendBluetooth(String s){
  digitalWrite(relay, HIGH);
  delayMicroseconds(10000);
  bluetooth.println("#" + s + "#");
  digitalWrite(relay, LOW);
}

void printState() {
  Serial.print(spd);
  Serial.print(" ");
  Serial.print(cnt);
  Serial.print(" ");
  Serial.print(mpu.getAngleY());
  Serial.println();
}
void encoderCount() {
  // Task Scheduler code
  static bool prevA = true;
  bool A = digitalRead( encoderA );
  bool B = digitalRead( encoderB );
  // A의 falling edge일 때, B가 HIGH이면 카운트한다.
  if ( prevA && !A && B ) {
    cnt++;
  }
  prevA = A;
  // interrupt code
  //  static unsigned long prevTime = 0;
  //  unsigned long Time = millis();
  //  // 디바운싱 억제용 코드였지만, 효용이 없었다.
  //  if ( digitalRead(encoderB) && prevTime + 3 <= Time ) {
  //    cnt++;
  //    prevTime = Time;
  //  }
}

void encoderSpeed() {
  static unsigned long prevTime = 0;
  unsigned long Time = millis();
  // 시간당 회전 각도를 계산.
  // 한 카운트 당 15도 이며,
  // 시간 단위가 millisecond이므로 1000을 곱함.
  spd = (double)cnt * 1000 * 15 / (Time - prevTime);

  mpu.update();
  float tilt = mpu.getAngleY();
  if (tilt > 20) {
    spd = spd * 1.5;
    digitalWrite(uphill, HIGH);
    digitalWrite(downhill, LOW);
  }
  else if (tilt > -20) {
    digitalWrite(uphill, LOW);
    digitalWrite(downhill, LOW);
  }
  else {
    spd = spd * 0.5;
    digitalWrite(uphill, LOW);
    digitalWrite(downhill, HIGH);
  }

  spd = map((long)spd, 0, 1500, 255, 0);
  if (spd > 255) {
    spd = 255;
  }
  else if (spd < 0) {
    spd = 0;
  }
  analogWrite(motor, spd);

  //printState();
  cnt = 0;
  prevTime = Time;
}
