#include <Servo.h>

class UltrasonicSensor
{
  private:
    int ultrasonicTrigPin, ultrasonicEchoPin;
    long ultrasonicDuration, ultrasonicCm;

  public:
    void init(int ultrasonicTrigPin, int ultrasonicEchoPin){
      this->ultrasonicTrigPin = ultrasonicTrigPin;
      this->ultrasonicEchoPin = ultrasonicEchoPin;
      pinMode(this->ultrasonicTrigPin, OUTPUT);
      pinMode(this->ultrasonicEchoPin, INPUT);
    }

  public:
    long getDistanceInCentimeters(){
      // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
      // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
      digitalWrite(this->ultrasonicTrigPin, LOW);
      delayMicroseconds(5);
      digitalWrite(this->ultrasonicTrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(this->ultrasonicTrigPin, LOW);
      // Read the signal from the sensor: a HIGH pulse whose
      // duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.
      this->ultrasonicDuration = pulseIn(this->ultrasonicEchoPin, HIGH);
      // convert the time into a distance in centimeters
      this->ultrasonicCm = (this->ultrasonicDuration / 2) / 29.1;
      return this->ultrasonicCm;
    }
};

class RobotServo
{
  private:
    Servo servo;

  public:
    int servoPos = 75;
    int servoPin;

  public:
    void init(int servoPin){
      this->servoPin = servoPin;
      this->servo.attach(this->servoPin);
      delay(15);
      this->panCenter();
    }

    void panCenter(){
      this->servoPos = 75;
      this->servo.write(this->servoPos);
      delay(500);
    }

    void panLeft(){
      this->servoPos = 160;
      this->servo.write(this->servoPos);
      delay(500);
    }

    void panRight(){
      this->servoPos = 0;
      this->servo.write(this->servoPos);
      delay(500);
    }

};

class Wheel
{
  protected:
    int enablePin;
    int in1;
    int in2;
    int wheelSpeed = 150;

  public:
    void init(int enablePin, int in1, int in2){
      this->enablePin = enablePin;
      this->in1 = in1;
      this->in2 = in2;
      pinMode(this->enablePin, OUTPUT);
      pinMode(this->in1, OUTPUT);
      pinMode(this->in2, OUTPUT);
    }

  public:
    void off(){
      digitalWrite(this->enablePin, LOW);
      digitalWrite(this->in1, LOW);
      digitalWrite(this->in2, LOW);
    }

  protected:
    void forward(){
      digitalWrite(this->enablePin, HIGH);
    }

  protected:
    void backward(){
      digitalWrite(this->enablePin, HIGH);
    }

};

class LeftWheel: public Wheel
{
  public:
  void forward(){
    Wheel::forward();
    digitalWrite(this->in1, HIGH);
    digitalWrite(this->in2, LOW);
  }

  public:
  void backward(){
    Wheel::backward();
    digitalWrite(this->in1, LOW);
    digitalWrite(this->in2, HIGH);
  }

};

class RightWheel: public Wheel
{
  public:
  void forward(){
    Wheel::forward();
    digitalWrite(this->in1, LOW);
    digitalWrite(this->in2, HIGH);
  }

  public:
  void backward(){
    Wheel::backward();
    digitalWrite(this->in1, HIGH);
    digitalWrite(this->in2, LOW);
  }

};

class Robot
{
  private:
    // Ultrasonic sensor
    UltrasonicSensor ultrasonicSensor;

    // Servo
    RobotServo robotServo;

    // Wheels
    LeftWheel leftWheel;
    RightWheel rightWheel;

  public:
    void init(){
      // Initialize pins
      this->leftWheel.init(6, 4, 5);
      this->rightWheel.init(1, 2, 3);
      this->robotServo.init(7);
      this->ultrasonicSensor.init(8, 9);
    }

    void forward(){
      this->leftWheel.forward();
      this->rightWheel.forward();
    }

    void backward(){
      this->leftWheel.backward();
      this->rightWheel.backward();
    }

    void turnLeft(){
      this->leftWheel.forward();
      this->rightWheel.backward();
    }

    void turnRight(){
      this->leftWheel.backward();
      this->rightWheel.forward();
    }

    void brake(){
      this->leftWheel.off();
      this->rightWheel.off();
    }

    boolean isDetectingObstacle(){
      long currentValue = 0;
      long ultrasonicCm = 0;
      for (int i = 0; i < 10; i++) {
        currentValue += this->ultrasonicSensor.getDistanceInCentimeters();
        delay(5);
      }
      int distance = 40;
      return ( (currentValue / 5) < distance);
    }

    void lookCenter(){
      this->robotServo.panCenter();
    }

    void lookLeft(){
      this->robotServo.panLeft();
    }

    void lookRight(){
      this->robotServo.panRight();
    }

};

// Robot
Robot robot;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Initialize robot
  robot.init();
}

boolean hasDetectedObstacle = false;
boolean hasObstacleOnRight = false;
boolean hasObstacleOnLeft = false;

void loop()
{
  hasDetectedObstacle = robot.isDetectingObstacle();
  if(!hasDetectedObstacle) {
    robot.forward();
  } else {
    robot.brake();
    delay(1000);
    detectPath();
  }
}

void detectPath(){
  robot.lookRight();
  hasObstacleOnRight = robot.isDetectingObstacle();
  robot.lookCenter();
  robot.lookLeft();
  hasObstacleOnLeft = robot.isDetectingObstacle();
  robot.lookCenter();
  if(hasObstacleOnRight && hasObstacleOnLeft) {
    robot.backward();
    delay(500);
    detectPath();
  } else if (!hasObstacleOnRight) {
    robot.turnRight();
    delay(300);
  } else if (!hasObstacleOnLeft) {
    robot.turnLeft();
    delay(300);
  }
}