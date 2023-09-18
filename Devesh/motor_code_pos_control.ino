
#include "CytronMotorDriver.h"
#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5 //this is enable
#define IN1 6 //this is direction
#define GR 50.9 // Gear Ratio
#define PPR 7.0 // Pulse Per Revolution
char receivedChars[12];
boolean newData = false;

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
CytronMD motor(PWM_DIR, 5, 6);
void setup() {
  Serial.begin(9600);
  Serial.println("<Arduino is ready>");
  
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
  // int target = -1200;
 int target = 250*sin(prevT/1e6);

// Now, frontLeft, frontRight, rearLeft, and rearRight contain the parsed values.



  // PID constants
  float kp = 100;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  deltaT=deltaT*PPR*GR;
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e =target-pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power

  // motor direction
  int dir {};
  if(u>=0){
    dir = 1;
  }
  if(u<0){
    dir = -1;
  }
  float pwr = u;
  if( pwr > 255 ){
    pwr = 255;
  }




  // signal the motor
  setMotor(dir,pwr,PWM,IN1);


  // store previous error
  eprev = e;


  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.print(pwr);
  Serial.print(" ");
  Serial.print(dir);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  motor.setSpeed(pwmVal);
  if(dir == 1){
    digitalWrite(in1,LOW);
   
  }
  else if (dir==-1) {
    digitalWrite(in1,HIGH);
  } 
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

