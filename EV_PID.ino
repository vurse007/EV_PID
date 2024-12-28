#include <math.h>

//pins for controlling direction of movement
#define MotorDirPin1 9
#define MotorDirPin2 10

//pin that controls the motor speed
#define MotorSpeedPin 11

//encoder stuff
volatile unsigned long currentPosENC = 0;
double pulsesPerRev = 102;
//do NOT change off of pins 2 and 3, attachInterrupt is only on these pins and this capability is needed to make the code work
//attachInterrupt is essentially being able to run things in the background
#define EncPinA 2
#define EncPinB 3

//laser stuff
#define LaserPin 7
#define LaserButtonPin 5
bool laser = false;

//start button stuff
#define StartButtonPin 12
bool start = false;

//misc variables
double wheelDiameterCM = 5.08;
double targetPos = 8;

//variables that are the distance values converted into encoder units
double targetPosENC;

//variables used to break out of the pid loop
int count = 0;

//calculating pid
double calculatePID(double error, double kP, double kI, double kD, double totalError=0, double prevError=0, double integralThreshold=0, double maxI=500){
  //calculate integral
  if (integralThreshold != 0 && fabs(error) < integralThreshold){
    totalError += error;
  }
  else if (integralThreshold == 0){
    totalError += error;
  }
  if (error > 0) {
    if (totalError > maxI){
      totalError = maxI;
    }
  }
  else{
    if (totalError < -maxI){
      totalError = -maxI;
    }
  }

  //calculate derivative
  double derivative = error - prevError;
  prevError = error;

  //calculate output
  double speed = (error*kP) + (totalError*kI) + (derivative*kD);
  if (speed > 255){
    speed = 255; //cap the speed at the max speed of arduino - 255
  }
  else if (speed < -255){
    speed = -255;
  }

  return speed;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //init motor pins
  pinMode(MotorDirPin1, OUTPUT);
  pinMode(MotorDirPin2, OUTPUT);
  pinMode(MotorSpeedPin, OUTPUT);

  //init encoder pins
  pinMode(EncPinA, INPUT);
  pinMode(EncPinB, INPUT);
  digitalWrite(EncPinA, HIGH); //turn on pullup resistors - prevents floating states, by making the value default to HIGH whenever the encoder doesnt show a value
  digitalWrite(EncPinB, HIGH); //turn on pullup resistors - basically the sensor is always in a defined state so there are no tweakouts

  //setting up interrupt
  //A rising pulse from encoder activated ai0(). attachinterrupt 0 is digitalpin2 on most arduinos
  attachInterrupt(0, ai0, RISING);
  //B rising pulse from encoder activated ai1(). attachinterrupt 1 is digitalpin3 on most arduinos
  attachInterrupt(1, ai1, RISING);

  //initialize button pins
  pinMode(StartButtonPin, INPUT);
  digitalWrite(StartButtonPin, HIGH); //enable pullups to make pin high

  pinMode(LaserButtonPin, INPUT);
  digitalWrite(LaserButtonPin, HIGH); //enable pullups to make pin high

  //init laser pins
  pinMode(LaserPin, OUTPUT);
  digitalWrite(LaserPin, LOW); //start off

  //reset bools
  start = false;
  laser = false;

  //get enc values
  targetPosENC = getEncoderValue(targetPos, wheelDiameterCM, pulsesPerRev);
}

void loop() {
  // put your main code here, to run repeatedly:

  //laser button toggle
  if (digitalRead(LaserButtonPin) == LOW){
    if (laser == 0){
      Serial.println("laser turned on");
      digitalWrite(LaserPin, HIGH);
      laser = true;
      delay(300); //to prevent unintentional double clicks
    }
    else{
      Serial.println("laser is off");
      digitalWrite(LaserPin, LOW);
      laser = false;
      delay(300);
    }
  }

  //start button toggle
  if (digitalRead(StartButtonPin) == LOW){
    Serial.println("pid started");
    Serial.print("target pos in meters: ");
    Serial.println(targetPos);
    Serial.print("target pos in encoder ticks: ");
    Serial.println(targetPosENC);
    currentPosENC = 0;
    start = true;
  }

  //pid loop below
  while (start==true){
    double error = targetPosENC - currentPosENC;
    double speed = calculatePID(error, 1, 1, 1);
     
    //change motor direction based on the value of speed
    if (speed > 0){
      //set motor direction to forward
      digitalWrite(MotorDirPin1, LOW);
      digitalWrite(MotorDirPin2, HIGH);
    }
    else if (speed < 0){
      //set motor direction to reverse
      digitalWrite(MotorDirPin1, HIGH);
      digitalWrite(MotorDirPin2, LOW);

      speed = fabs(speed);
    }

    //run the motor
    analogWrite(MotorSpeedPin, speed);

    //break out of the loop
    if (error < 50){
      count++;
    }

    if (count > 10){
      Serial.println("pid break");
      start = false;
    }
  }
}

void ai0() {
  //ai0 is activated if digitalpin nr 2 is going from low to high
  //check pin 3 to determine the direction
  if (digitalRead(3) == HIGH){
    currentPosENC++;
  }
  else{
    currentPosENC--;
  }
}

void ai1() {
  //ai1 is activated if digitalpin nr 3 is going from low to high
  //check with pin 2 to determine the direction
  if (digitalRead(2) == HIGH){
    currentPosENC--;
  }
  else{
    currentPosENC++;
  }
}

double getEncoderValue(double targetPos, double WD, double PPR){
  double wheelCircumference = M_PI * WD; //circumference is 2*pi*r or pi*d
  double tgtENCval = ((targetPos*100) / wheelCircumference) * PPR; //target pos is in meters times 100 -> target pos in cm divided by wheel circumference in cm -> rotations of wheel in target * times PPR -> pulses in target 
  return tgtENCval;
}