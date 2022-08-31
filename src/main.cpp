#include <Arduino.h>
#include<Servo.h>
#include<PID_v1.h>
#include<SoftwareSerial.h>

int set=25,neg=-50,pos=50,base=130;
long cm1=set;
const double a=0.5;
const int servoPin = 9;


// PID tunning
float Kp = 2;
float Ki = 1;
float Kd = 4;
double Setpoint, Input, Output, ServoOutput;                                       


// PID and servo initalization
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);    
Servo myServo;

float readPosition()
{
  delay(40);                                                            
  const int TrigPin = 2;
  const int EchoPin = 3;
  const int BeamLength = 50;
  long duration, cm,cmn;
  unsigned long now = millis();
  pinMode(TrigPin, OUTPUT);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(TrigPin, LOW);


  pinMode(EchoPin, INPUT);
  duration = pulseIn(EchoPin, HIGH);

  cm = duration/(15*2);
  if(cm > 50)
  {
    cm=50;
  }
  else if (cm == 0)
  {
    cm+=2;
  }
  else
  {
    cm = cm;
  }
  cmn = a * cm + (1 - a) * cm1;
  Serial.print(cm); Serial.print("\t");
  Serial.println(cmn);
  delay(10);
  cm1 = cmn;
  return (cmn);
}
void setup()
{
  Serial.begin(9600);
  myServo.attach(servoPin);
  Input = readPosition();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(neg,pos);
}
void loop()
{
  if (readPosition() >= 20 && readPosition() <=26){
    myServo.write(127);
    digitalWrite(10, HIGH);
    delay(6000);
  }
  digitalWrite(10, LOW);
  if (readPosition() >=49){
    myServo.write(0);
  }
  Setpoint = set;
  Input = readPosition();
  if (readPosition() == 0){
    Input=5;
  }
  if (readPosition() == 7 || readPosition() == 8 || readPosition() == 9){
    myServo.write(180);
    delay(200);
  }
  myPID.Compute();
  ServoOutput=base+Output;
  myServo.write(ServoOutput);
  delay(320);
}
